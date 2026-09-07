// Flies a real 3-vehicle ArduPilot swarm in SITL under the same
// boids + HDC-gain-scheduled flocking mechanism as examples/drone_swarm,
// instead of that example's own internal rigid-body simulation: one
// process here manages N independent UDP links (one per vehicle, each
// paired with its own tools/ardupilot_bridge/mavlink_shim.py instance and
// real ArduCopter SITL instance), caches every vehicle's latest reported
// position/velocity, and on each vehicle's state update recomputes the
// full flock's separation/cohesion/alignment/goal-seeking using whatever
// neighbor data is freshest -- then replies with just that vehicle's
// velocity setpoint. The shims and wire protocol are unchanged from the
// single-vehicle bridge; what's new is running the shared multi-agent
// behavior over several real, independent MAVLink links instead of one.
//
// Coordinate frames: every vehicle's local-NED position is relative to
// its OWN EKF origin, so raw positions from different vehicles are never
// directly comparable -- true in SITL and, more importantly, true of any
// two physically separate real vehicles, which obviously can't share a
// GPS position the way SITL instances safely could. This code (and the
// wire protocol it shares with the single-vehicle bridge) is agnostic to
// how that gets resolved: it just consumes whatever "pos" each shim
// reports. The actual fix lives in mavlink_shim.py's --common-origin
// option, which converts each vehicle's own absolute GPS fix into meters
// relative to one shared reference point given to every swarm member
// (see launch_swarm_shims.sh) -- the same mechanism a real multi-vehicle
// deployment would use, so vehicles can launch from realistic, distinct
// positions (see launch_swarm_sitl.sh) instead of a SITL-only trick.
//
// Wire protocol: identical to ardupilot_bridge.cpp, one instance per agent
// on its own UDP port (default 6001, 6002, 6003, ...).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"
#include "nlohmann/json.hpp"

using namespace fuzzylib;
using namespace fuzzylib::blocks;
using json = nlohmann::json;

namespace {

constexpr int kNumAgents = 3;
constexpr double kSenseRadius = 15.0;          // neighbor sensing range, meters
constexpr double kMinSafeDistance = 2.0;       // reporting threshold, meters
constexpr double kMaxSpeed = 4.0;              // m/s -- conservative for a real vehicle
constexpr double kMaxAccel = 2.5;              // m/s^2 applied to the velocity setpoint per cycle
constexpr double kControlDt = 0.1;             // assumed interval between a vehicle's state updates
constexpr double kMaxExpectedNeighbors = 2.0;  // "fully crowded" reference for a 3-agent swarm

struct Vec3 {
    double n = 0.0, e = 0.0, d = 0.0;
    Vec3 operator+(const Vec3& o) const { return {n + o.n, e + o.e, d + o.d}; }
    Vec3 operator-(const Vec3& o) const { return {n - o.n, e - o.e, d - o.d}; }
    Vec3 operator*(double s) const { return {n * s, e * s, d * s}; }
    double length() const { return std::sqrt(n * n + e * e + d * d); }
    Vec3 normalized() const {
        const double len = length();
        return len > 1e-6 ? (*this) * (1.0 / len) : Vec3{0.0, 0.0, 0.0};
    }
};

Vec3 limit(Vec3 v, double maxLen) {
    const double len = v.length();
    if (len > maxLen && len > 1e-9) return v * (maxLen / len);
    return v;
}

struct AgentState {
    Vec3 pos, vel;
    bool haveState = false;
    bool armed = false;
    std::string mode = "?";
};

// Same crowding classifier as examples/drone_swarm, just re-tuned for a
// 3-agent swarm via kMaxExpectedNeighbors above.
std::shared_ptr<HDCPrototypeBlock> buildCrowdingClassifier() {
    auto space = std::make_shared<hdc::HDCSpace>(4000, /*seed=*/2026);
    auto block = std::make_shared<HDCPrototypeBlock>(space);
    block->addInput("density", 0.0, 1.0, /*levels=*/50);
    for (double level : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        block->addPrototype("level_" + std::to_string(level), {{"density", level}}, level);
    }
    return block;
}

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
    UdpEndpoint(const UdpEndpoint&) = delete;
    UdpEndpoint& operator=(const UdpEndpoint&) = delete;

    int fd() const { return fd_; }

    // Only call once select()/poll() has confirmed this socket is readable.
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
    int basePort = 6001;
    if (argc > 1) basePort = std::atoi(argv[1]);

    // North-east-ish goal, modest scale for a real, non-sped-up SITL flight.
    const Vec3 goal{40.0, 30.0, -10.0};

    // Distinct per-agent formation slots on a small ring around the shared
    // goal, rather than every agent seeking that exact same point. Confirmed
    // live: with a single shared goal, once the flock converged, separation
    // (pushing agents apart) and goal-seeking (pulling all of them to one
    // point) fought each other indefinitely -- the flock overshot the goal
    // by 30+ m and kept orbiting outward instead of settling. Real vehicles
    // converging on a rendezvous need distinct slots for the same reason
    // (they can't occupy the same point either).
    constexpr double kFormationRadius = 5.0;  // meters
    std::vector<Vec3> formationSlot(kNumAgents);
    for (int k = 0; k < kNumAgents; ++k) {
        const double angle = 2.0 * std::acos(-1.0) * k / kNumAgents;
        formationSlot[k] = goal + Vec3{kFormationRadius * std::cos(angle), kFormationRadius * std::sin(angle), 0.0};
    }

    std::vector<std::unique_ptr<UdpEndpoint>> endpoints;
    for (int i = 0; i < kNumAgents; ++i) endpoints.push_back(std::make_unique<UdpEndpoint>(basePort + i));

    auto crowdingClassifier = buildCrowdingClassifier();
    std::vector<AgentState> agents(kNumAgents);
    std::vector<double> lastYaw(kNumAgents, 0.0);
    int packetCount = 0;

    std::cout << "fuzzylib ArduPilot swarm bridge: " << kNumAgents << " agents on UDP ports " << basePort << ".."
               << (basePort + kNumAgents - 1) << "\n";
    std::cout << "Flocking toward goal (" << goal.n << ", " << goal.e << ", " << goal.d << "), each agent to its own "
               << kFormationRadius << "m-ring formation slot, under HDC-scheduled separation/cohesion/alignment.\n\n";

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        int maxfd = 0;
        for (auto& ep : endpoints) {
            FD_SET(ep->fd(), &readfds);
            maxfd = std::max(maxfd, ep->fd());
        }
        if (select(maxfd + 1, &readfds, nullptr, nullptr, nullptr) < 0) {
            std::cerr << "select() failed\n";
            continue;
        }

        for (int i = 0; i < kNumAgents; ++i) {
            if (!FD_ISSET(endpoints[i]->fd(), &readfds)) continue;

            json state;
            try {
                state = endpoints[i]->receive();
            } catch (const std::exception& ex) {
                std::cerr << "recv error on agent " << i << ": " << ex.what() << "\n";
                continue;
            }
            if (state.value("type", "") != "state") continue;

            agents[i].pos = parseVec3(state.at("pos"));
            agents[i].vel = parseVec3(state.at("vel"));
            agents[i].haveState = true;
            agents[i].armed = state.value("armed", false);
            agents[i].mode = state.value("mode", "?");

            // Flocking, using whatever neighbor data is freshest -- other
            // agents' state updates arrive asynchronously on their own
            // sockets, so this is necessarily a little stale, same as real
            // distributed swarms operating over unreliable comms.
            Vec3 separation{}, cohesionCenter{}, avgVelocity{};
            int neighborCount = 0;
            for (int j = 0; j < kNumAgents; ++j) {
                if (j == i || !agents[j].haveState) continue;
                const Vec3 delta = agents[i].pos - agents[j].pos;
                const double dist = std::max(0.05, delta.length());
                if (dist < kSenseRadius) {
                    ++neighborCount;
                    separation = separation + delta * (1.0 / (dist * dist));
                    cohesionCenter = cohesionCenter + agents[j].pos;
                    avgVelocity = avgVelocity + agents[j].vel;
                }
            }

            const double density = std::min(1.0, static_cast<double>(neighborCount) / kMaxExpectedNeighbors);
            const double crowdingLevel = crowdingClassifier->evaluate({{"density", density}}).at("value");

            const double separationGain = 8.0 * (0.4 + 1.6 * crowdingLevel);
            const double cohesionGain = 0.6 * (1.2 - 0.9 * crowdingLevel);
            constexpr double kAlignmentGain = 0.4;
            constexpr double kGoalGain = 0.5;
            // Braking, proportional to current speed: unlike the pure
            // simulation this is adapted from, here the goal is close
            // enough to actually reach within a real flight -- and
            // velocityCmd = measured_velocity + accel*dt carries speed
            // forward with nothing to bleed it off once goalPull shrinks
            // near arrival (alignment only levels differences between
            // agents' velocities, it doesn't slow the flock down as a
            // whole). Confirmed live: without this, an agent flew straight
            // through the goal at cruise speed and kept going. Alone this
            // accel is negative feedback, so it can't itself cause
            // overshoot; kMaxAccel still bounds the total.
            //
            // Sized above critical damping for the goal spring alone
            // (accel ~= kGoalGain*(target-pos) near arrival, so this is a
            // damped 2nd-order system with natural frequency
            // sqrt(kGoalGain) =~ 0.7 rad/s and critical damping =~ 1.4);
            // confirmed live that 0.6 (underdamped) let the flock settle
            // into a sustained, only slowly-decaying orbit around its
            // formation ring rather than parking there, driven mostly by
            // separation gain staying near its max (crowding pins at 1.0
            // whenever all agents are mutually visible, regardless of how
            // far apart they actually are) even once everyone reached
            // their own slot.
            constexpr double kVelocityDamping = 2.0;  // 1/s

            Vec3 cohesion{}, alignment{};
            if (neighborCount > 0) {
                cohesionCenter = cohesionCenter * (1.0 / neighborCount);
                cohesion = (cohesionCenter - agents[i].pos) * cohesionGain;
                avgVelocity = avgVelocity * (1.0 / neighborCount);
                alignment = (avgVelocity - agents[i].vel) * kAlignmentGain;
            }

            const Vec3 toGoal = formationSlot[i] - agents[i].pos;
            const double goalDist = std::max(1e-6, toGoal.length());
            const Vec3 goalPull = toGoal.normalized() * (kGoalGain * std::min(goalDist, 20.0));

            const Vec3 damping = agents[i].vel * (-kVelocityDamping);
            const Vec3 accel =
                limit(separation * separationGain + cohesion + alignment + goalPull + damping, kMaxAccel);

            // Real ArduPilot owns position integration; we only command a
            // velocity setpoint, built from the *measured* current velocity
            // plus this cycle's computed acceleration (a rate-limited
            // velocity command), not from an internally-simulated one.
            const Vec3 velocityCmd = limit(agents[i].vel + accel * kControlDt, kMaxSpeed);
            if (velocityCmd.length() > 0.3) lastYaw[i] = std::atan2(velocityCmd.e, velocityCmd.n);

            json setpoint;
            setpoint["type"] = "setpoint";
            setpoint["vel"] = toJson(velocityCmd);
            setpoint["yaw"] = lastYaw[i];
            endpoints[i]->reply(setpoint);

            if (++packetCount % (10 * kNumAgents) == 0) {
                std::cout << "  agent" << i << " pos=(" << std::fixed << std::setprecision(1) << agents[i].pos.n
                           << "," << agents[i].pos.e << "," << agents[i].pos.d << ")  neighbors=" << neighborCount
                           << "  crowding=" << std::setprecision(2) << crowdingLevel << "  distToGoal="
                           << std::setprecision(1) << goalDist << "  mode=" << agents[i].mode
                           << "  armed=" << agents[i].armed << "\n";
            }
        }
    }
}
