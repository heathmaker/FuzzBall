// Links fuzzylib to a real ArduPilot vehicle running in SITL (and, through
// it, to Mission Planner or any other MAVLink ground station): this
// process is the "brain" that flies the vehicle, but it never speaks
// MAVLink itself. A small companion process (tools/ardupilot_bridge/
// mavlink_shim.py, using pymavlink) does the real MAVLink handshake, mode
// changes, arming, and takeoff, then — once the vehicle is airborne in
// GUIDED mode — relays vehicle state to us and our velocity/yaw commands
// back to ArduPilot, over a trivial local JSON-over-UDP protocol. Splitting
// it this way keeps the actual MAVLink wire protocol (binary framing,
// per-message CRCs) on the well-tested reference implementation, while
// this file stays pure fuzzylib: a Sugeno guidance law that governs a
// vehicle's approach speed to a sequence of waypoints, tapering smoothly
// on arrival instead of the constant-speed-then-stop profile a simple
// waypoint follower would use.
//
// Wire protocol (newline-free, one JSON object per UDP datagram):
//   shim -> here  {"type":"state","t":.,"armed":.,"mode":".",
//                  "pos":{"n":.,"e":.,"d":.},"vel":{"n":.,"e":.,"d":.},"yaw":.}
//   here -> shim  {"type":"setpoint","vel":{"n":.,"e":.,"d":.},"yaw":.}
// Positions/velocities are local NED meters/m-per-second relative to the
// EKF origin (ArduPilot's usual "local position" frame); yaw is radians.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"
#include "nlohmann/json.hpp"

using namespace fuzzylib;
using json = nlohmann::json;

namespace {

struct Vec3 {
    double n = 0.0, e = 0.0, d = 0.0;
    Vec3 operator-(const Vec3& o) const { return {n - o.n, e - o.e, d - o.d}; }
    double length() const { return std::sqrt(n * n + e * e + d * d); }
    Vec3 normalized() const {
        const double len = length();
        return len > 1e-6 ? Vec3{n / len, e / len, d / len} : Vec3{0.0, 0.0, 0.0};
    }
};

// Distance-to-target -> approach speed, tapering to a creep speed near
// arrival so the vehicle settles onto a waypoint instead of overshooting
// (real ArduPilot loiter/waypoint speed profiles do something similar;
// here it's an explicit, inspectable 3-rule zero-order Sugeno FIS instead
// of a hardcoded deceleration curve).
SugenoEngine buildGuidanceFIS() {
    LinguisticVariable distance("distance", 0.0, 100.0);
    distance.addTerm("near", mf::make<mf::Trapezoidal>(0.0, 0.0, 2.0, 8.0));
    distance.addTerm("medium", mf::make<mf::Triangular>(4.0, 15.0, 30.0));
    distance.addTerm("far", mf::make<mf::Trapezoidal>(20.0, 50.0, 100.0, 100.0));

    SugenoEngine engine;
    engine.addInput(distance);

    SugenoConsequent creep, cruise, fast;
    creep.variable = cruise.variable = fast.variable = "speed";
    creep.constant = 0.4;
    cruise.constant = 2.5;
    fast.constant = 6.0;

    engine.rules().add(Rule(antecedent::is("distance", "near")).then(creep));
    engine.rules().add(Rule(antecedent::is("distance", "medium")).then(cruise));
    engine.rules().add(Rule(antecedent::is("distance", "far")).then(fast));
    return engine;
}

constexpr double kArrivalTolerance = 1.5;  // meters

class UdpEndpoint {
public:
    explicit UdpEndpoint(int port) {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("socket() failed");
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("bind() failed on port " + std::to_string(port));
        }
    }
    ~UdpEndpoint() {
        if (fd_ >= 0) close(fd_);
    }

    // Blocks until a datagram arrives; returns its JSON payload and
    // records the sender so a reply can be routed back to them.
    json receive() {
        char buf[4096];
        sockaddr_in sender{};
        socklen_t senderLen = sizeof(sender);
        const ssize_t n = recvfrom(fd_, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr*>(&sender), &senderLen);
        if (n < 0) throw std::runtime_error("recvfrom() failed");
        buf[n] = '\0';
        lastSender_ = sender;
        haveSender_ = true;
        return json::parse(buf, buf + n);
    }

    void reply(const json& payload) {
        if (!haveSender_) return;
        const std::string text = payload.dump();
        sendto(fd_, text.data(), text.size(), 0, reinterpret_cast<sockaddr*>(&lastSender_), sizeof(lastSender_));
    }

private:
    int fd_ = -1;
    sockaddr_in lastSender_{};
    bool haveSender_ = false;
};

Vec3 parseVec3(const json& v) { return {v.at("n").get<double>(), v.at("e").get<double>(), v.at("d").get<double>()}; }
json toJson(const Vec3& v) { return json{{"n", v.n}, {"e", v.e}, {"d", v.d}}; }

}  // namespace

int main(int argc, char** argv) {
    int port = 6001;
    if (argc > 1) port = std::atoi(argv[1]);

    // A simple square circuit at 10 m altitude (NED "down" is negative up),
    // relative to wherever ArduPilot's EKF origin ends up (typically the
    // SITL vehicle's start location).
    const std::vector<Vec3> waypoints = {
        {0.0, 0.0, -10.0}, {40.0, 0.0, -10.0}, {40.0, 40.0, -10.0}, {0.0, 40.0, -10.0}, {0.0, 0.0, -10.0},
    };

    const SugenoEngine guidance = buildGuidanceFIS();
    UdpEndpoint endpoint(port);

    std::cout << "fuzzylib ArduPilot bridge listening on UDP :" << port << "\n";
    std::cout << "Flying a " << waypoints.size() << "-waypoint circuit under Sugeno-FIS speed guidance.\n\n";

    std::size_t targetIndex = 0;
    double lastYaw = 0.0;
    int packetCount = 0;

    while (true) {
        json state;
        try {
            state = endpoint.receive();
        } catch (const std::exception& ex) {
            std::cerr << "recv error: " << ex.what() << "\n";
            continue;
        }
        if (state.value("type", "") != "state") continue;

        const Vec3 pos = parseVec3(state.at("pos"));
        Vec3 toTarget = waypoints[targetIndex] - pos;
        double distance = toTarget.length();

        if (distance < kArrivalTolerance && targetIndex + 1 < waypoints.size()) {
            // Recompute against the new target immediately: otherwise the
            // command sent out on this exact cycle would still aim at the
            // waypoint just reached (a stale, wrong-direction pulse for
            // one control cycle) instead of the next one.
            ++targetIndex;
            toTarget = waypoints[targetIndex] - pos;
            distance = toTarget.length();
        }

        const auto guidanceOut = guidance.evaluate({{"distance", distance}});
        const double speed = guidanceOut.at("speed");
        const Vec3 direction = toTarget.normalized();
        const Vec3 velocityCmd{direction.n * speed, direction.e * speed, direction.d * speed};

        if (distance > 0.5) lastYaw = std::atan2(direction.e, direction.n);

        json setpoint;
        setpoint["type"] = "setpoint";
        setpoint["vel"] = toJson(velocityCmd);
        setpoint["yaw"] = lastYaw;
        endpoint.reply(setpoint);

        if (++packetCount % 20 == 0) {
            std::cout << "  wp=" << targetIndex << "/" << (waypoints.size() - 1) << "  pos=(" << std::fixed
                       << std::setprecision(1) << pos.n << "," << pos.e << "," << pos.d << ")  dist=" << distance
                       << "  speed=" << speed << "  mode=" << state.value("mode", "?")
                       << "  armed=" << state.value("armed", false) << "\n";
        }
    }
}
