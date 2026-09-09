// Fuzzy adaptive cruise control (ACC) hooked up to a real 10-mile SUMO
// highway, via libtraci (SUMO's native C++ TraCI client library) --
// unlike this repo's other examples, this one doesn't own any physics
// itself: SUMO simulates both vehicles' dynamics and the road, and this
// program only supplies the ACC (follower) vehicle's per-step speed
// command, the same guidance-layer role examples/ardupilot_bridge plays
// for a real ArduPilot vehicle.
//
// The controller is the same 9-rule gain-scheduled TSK/Sugeno ACC as
// examples/car_platooning (gap-error x relative-velocity -> acceleration).
// What's different here is the disturbance: instead of a scripted leader
// velocity profile, the leader is SUMO's own default "Krauss" model --
// the standard human-driver car-following model, complete with its usual
// driver imperfection (sigma) and reaction time (tau) -- and the
// disturbance is a real 2-mile, 45 mph speed-limit zone in the middle of
// an otherwise 65 mph corridor (see tools/sumo_acc/network.edg.xml). The
// human leader slows for and re-accelerates out of that zone entirely
// under SUMO's own logic; the ACC has to track it doing so.
//
// Requires SUMO's C++ libtraci headers/library on the system -- this
// target is skipped by CMake if they aren't found. See the "SUMO adaptive
// cruise control" section of the top-level README for setup, and
// tools/sumo_acc/build_network.sh, which must be run once before this.
//
// Usage: sumo_acc [path/to/sumo.sumocfg] [--gui]

#include <libsumo/libtraci.h>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;

namespace {

constexpr double kMilesToMeters = 1609.344;
constexpr double kMphToMps = kMilesToMeters / 3600.0;

constexpr double kStandstillGap = 5.0;          // meters, desired clear gap at v=0
constexpr double kTimeHeadway = 1.5;            // seconds, highway-appropriate following time
constexpr double kMaxAccel = 3.0;               // m/s^2, comfortable limit
constexpr double kMaxDecel = 4.0;               // m/s^2, comfortable braking limit
constexpr double kMaxSpeed = 40.0;              // m/s, ACC ceiling (well above the 65 mph target)
constexpr double kLeaderSearchRadius = 200.0;   // meters, how far ahead to look for a leader
constexpr double kDt = 0.1;                     // seconds -- must match sumo.sumocfg's step-length
constexpr double kMaxSimTime = 900.0;           // seconds, generous cap (10 mi at 45-65 mph is ~9-10 min)
constexpr const char* kEgoId = "ego";

// Identical 9-rule gain-scheduled TSK ACC controller to
// examples/car_platooning: the local linear law accel = Kp*gapError +
// Kd*relVel is the same everywhere, scaled up when both cues agree
// something is wrong and scaled down for fine tracking near the origin.
SugenoEngine buildACCController() {
    struct NamedTerm {
        std::string name;
        int index;
    };
    const std::array<NamedTerm, 3> terms{{{"Neg", -1}, {"Zero", 0}, {"Pos", 1}}};

    LinguisticVariable gapError("gapError", -15.0, 15.0);
    gapError.addTerm("Neg", mf::make<mf::Trapezoidal>(-15.0, -15.0, -6.0, -1.0));
    gapError.addTerm("Zero", mf::make<mf::Triangular>(-3.0, 0.0, 3.0));
    gapError.addTerm("Pos", mf::make<mf::Trapezoidal>(1.0, 6.0, 15.0, 15.0));

    LinguisticVariable relVel("relVel", -8.0, 8.0);
    relVel.addTerm("Neg", mf::make<mf::Trapezoidal>(-8.0, -8.0, -3.0, -0.5));
    relVel.addTerm("Zero", mf::make<mf::Triangular>(-1.5, 0.0, 1.5));
    relVel.addTerm("Pos", mf::make<mf::Trapezoidal>(0.5, 3.0, 8.0, 8.0));

    SugenoEngine engine;
    engine.addInput(gapError).addInput(relVel);

    constexpr double kKp = 0.35;  // accel per meter of gap error
    constexpr double kKd = 0.9;   // accel per (m/s) of relative velocity

    for (const auto& gapTerm : terms) {
        for (const auto& relTerm : terms) {
            const double scale = 1.0 + 0.5 * std::abs(gapTerm.index + relTerm.index);
            SugenoConsequent accel;
            accel.variable = "accel";
            accel.coefficients["gapError"] = kKp * scale;
            accel.coefficients["relVel"] = kKd * scale;
            engine.rules().add(
                Rule(antecedent::is("gapError", gapTerm.name) & antecedent::is("relVel", relTerm.name))
                    .then(accel));
        }
    }
    return engine;
}

double desiredGap(double ownVelocity) { return kStandstillGap + kTimeHeadway * ownVelocity; }

}  // namespace

int main(int argc, char** argv) {
    std::string cfgPath = argc > 1 ? argv[1] : "tools/sumo_acc/sumo.sumocfg";
    std::string binary = "sumo";
    if (argc > 2 && std::string(argv[2]) == "--gui") binary = "sumo-gui";

    const SugenoEngine acc = buildACCController();

    std::cout << "fuzzylib SUMO ACC bridge -- config: " << cfgPath << " (" << binary << ")\n";
    std::cout << "10-mile one-lane highway, 65 mph limit with a 2-mile 45 mph zone; leader is SUMO's "
                 "Krauss human-driver model.\n\n";

    libtraci::Simulation::start({binary, "-c", cfgPath, "--seed", "42", "--no-step-log", "--collision.action", "warn"});

    double worstGapError = 0.0;
    double minGap = 1e9;
    bool egoSpeedModeSet = false;
    int step = 0;
    const int maxSteps = static_cast<int>(kMaxSimTime / kDt);
    const int printEvery = static_cast<int>(5.0 / kDt);

    std::cout << "  time    leaderV(mph)  egoV(mph)   gap(m)   gapErr(m)\n";

    while (libtraci::Simulation::getMinExpectedNumber() > 0 && step <= maxSteps) {
        libtraci::Simulation::step();
        ++step;

        const auto activeIds = libtraci::Vehicle::getIDList();
        const bool egoActive = std::find(activeIds.begin(), activeIds.end(), kEgoId) != activeIds.end();
        if (!egoActive) continue;  // not yet departed, or already arrived

        if (!egoSpeedModeSet) {
            // Disable SUMO's own "regard safe speed" clamp (bit 0) on this
            // vehicle's setSpeed calls, so the fuzzy ACC's own gap-keeping
            // is what's actually under test here rather than being
            // silently backstopped by SUMO's internal collision-avoidance
            // layer. Physical accel/decel limits (bits 1 and 2) stay
            // enforced, same as a real vehicle's engine/brakes would.
            libtraci::Vehicle::setSpeedMode(kEgoId, 30);
            egoSpeedModeSet = true;
        }

        const double egoSpeed = libtraci::Vehicle::getSpeed(kEgoId);
        const auto [leaderId, gap] = libtraci::Vehicle::getLeader(kEgoId, kLeaderSearchRadius);

        double commandedSpeed;
        double gapErrForLog = 0.0;
        double leaderSpeedForLog = 0.0;
        if (!leaderId.empty()) {
            const double leaderSpeed = libtraci::Vehicle::getSpeed(leaderId);
            const double gapErr = gap - desiredGap(egoSpeed);
            const double relVel = leaderSpeed - egoSpeed;
            minGap = std::min(minGap, gap);
            worstGapError = std::max(worstGapError, std::fabs(gapErr));

            const auto result = acc.evaluate({{"gapError", std::clamp(gapErr, -15.0, 15.0)},
                                                {"relVel", std::clamp(relVel, -8.0, 8.0)}});
            const double accel = std::clamp(result.at("accel"), -kMaxDecel, kMaxAccel);
            commandedSpeed = std::clamp(egoSpeed + accel * kDt, 0.0, kMaxSpeed);
            gapErrForLog = gapErr;
            leaderSpeedForLog = leaderSpeed;
        } else {
            // No leader in range: free-cruise toward the ACC's own top
            // speed rather than stall, same as a real ACC on a clear road.
            commandedSpeed = std::clamp(egoSpeed + kMaxAccel * kDt, 0.0, kMaxSpeed);
        }

        libtraci::Vehicle::setSpeed(kEgoId, commandedSpeed);

        if (step % printEvery == 0) {
            std::cout << "  " << std::setw(5) << std::fixed << std::setprecision(1)
                       << libtraci::Simulation::getTime() << "   " << std::setw(10) << (leaderSpeedForLog / kMphToMps)
                       << "   " << std::setw(8) << (egoSpeed / kMphToMps) << "   " << std::setw(6) << gap << "   "
                       << std::setw(7) << gapErrForLog << "\n";
        }
    }

    libtraci::Simulation::close();

    std::cout << "\nWorst |gap error| observed: " << worstGapError << " m\n";
    std::cout << "Smallest gap observed:      " << minGap << " m\n";
    std::cout << (minGap > 1.0 ? "ACC maintained safe spacing across the full 10-mile highway.\n"
                                 : "ACC spacing became unsafe!\n");
    return 0;
}
