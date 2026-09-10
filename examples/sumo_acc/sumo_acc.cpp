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
// --record <dir> saves a periodic chase-cam PNG screenshot (via sumo-gui's
// screenshot API) plus a telemetry.csv alongside them, for assembling into
// a demo video with ffmpeg afterward (a run prints the exact command).
// Implies --gui, since screenshots need an actual GUI view to capture.
// Off by default: a full 10-mile run captures on the order of 800 PNGs,
// so only pass this when you actually want a recording, not on every run.
//
// Known caveat: --gui (and so --record) drives sumo-gui over a real X
// display. In at least one headless/Xvfb sandbox, a full run occasionally
// hung indefinitely partway through (reproducibly, around the leader's
// exit from the 45 mph zone, though a --gui run with no recording
// involved at all hit the identical hang, so it isn't specific to
// screenshot capture) -- apparently an interaction between sumo-gui and
// libtraci under that specific setup, not a logic bug in this file: the
// default headless mode (no --gui, no --record) uses a completely
// different SUMO code path and has never shown this. If a --gui/--record
// run seems stuck, that's the known issue; a real desktop X session may
// not hit it at all.
//
// Usage: sumo_acc [path/to/sumo.sumocfg] [--gui] [--record <dir>]

#include <libsumo/libtraci.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;

namespace {

constexpr double kMilesToMeters = 1609.344;
constexpr double kMphToMps = kMilesToMeters / 3600.0;

constexpr double kTimeHeadway = 1.0;            // seconds, target following gap at/above the floor speed below
constexpr double kLowSpeedFloorMph = 25.0;      // below this, hold the gap at its floor-speed value rather
                                                 // than letting it keep shrinking toward 0 near a stop
constexpr double kLowSpeedFloorMps = kLowSpeedFloorMph * kMphToMps;
constexpr double kMaxAccel = 3.0;               // m/s^2, comfortable limit
constexpr double kMaxDecel = 4.0;               // m/s^2, comfortable braking limit
constexpr double kMaxSpeed = 40.0;              // m/s, ACC ceiling (well above the 65 mph target)
constexpr double kLeaderSearchRadius = 200.0;   // meters, how far ahead to look for a leader
constexpr double kDt = 0.1;                     // seconds -- must match sumo.sumocfg's step-length
constexpr double kMaxSimTime = 900.0;           // seconds, generous cap (10 mi at 45-65 mph is ~9-10 min)
constexpr const char* kEgoId = "ego";

// --record settings: a chase-cam window centered between the two vehicles,
// captured every kCaptureEveryNSteps simulation steps. kVideoFps is only
// used for the ffmpeg command this program prints, not for anything here.
constexpr int kCaptureEveryNSteps = 8;   // 0.8s sim-time per frame
constexpr double kVideoFps = 10.0;       // -> 8x speedup once encoded at this rate
constexpr int kFrameW = 1280;
constexpr int kFrameH = 340;
constexpr double kChaseCamHalfWidth = 130.0;   // meters; comfortably covers the largest gap seen in practice
// SUMO's screenshot view appears to derive its actual vertical extent
// from the requested width and the frame's pixel aspect ratio, not from
// this value directly -- but setBoundary still requires *a* height, so
// this is a plausible one (matching the width) rather than a load-bearing
// setting. The result is a road that renders as a thin horizontal band
// with generous sky/ground padding; that's inherent to viewing a
// one-lane road wide enough to show a 100+ m gap, not a bug to tune away.
constexpr double kChaseCamHalfHeight = 130.0;  // meters

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

// A 1-second time gap above 25 mph; below that, the gap is held at its
// 25-mph value rather than shrinking further, so it doesn't collapse
// toward 0 while crawling or stopped (the standard shape for a real ACC's
// distance setting: a time headway at speed, a fixed minimum near a stop).
double desiredGap(double ownVelocity) { return kTimeHeadway * std::max(ownVelocity, kLowSpeedFloorMps); }

}  // namespace

int main(int argc, char** argv) {
    std::string cfgPath = "tools/sumo_acc/sumo.sumocfg";
    std::string binary = "sumo";
    std::string recordDir;  // empty => recording disabled (the default)
    bool cfgPathSet = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--gui") {
            binary = "sumo-gui";
        } else if (arg == "--record") {
            if (i + 1 >= argc) {
                std::cerr << "--record requires a directory argument\n";
                return 1;
            }
            recordDir = argv[++i];
        } else if (!cfgPathSet) {
            cfgPath = arg;
            cfgPathSet = true;
        }
    }
    const bool recording = !recordDir.empty();
    if (recording) binary = "sumo-gui";  // screenshot() needs an actual GUI view

    const SugenoEngine acc = buildACCController();

    std::cout << "fuzzylib SUMO ACC bridge -- config: " << cfgPath << " (" << binary << ")\n";
    std::cout << "10-mile one-lane highway, 65 mph limit with a 2-mile 45 mph zone; leader is SUMO's "
                 "Krauss human-driver model.\n\n";

    // --start matters specifically for sumo-gui: without it, the GUI opens
    // paused waiting for a human to click play, and never even opens its
    // TraCI port -- this program would then hang forever retrying a
    // connection to a server that's never going to start listening.
    // Harmless (a no-op) for headless sumo, which always runs immediately.
    libtraci::Simulation::start(
        {binary, "-c", cfgPath, "--seed", "42", "--start", "--no-step-log", "--collision.action", "warn"});

    std::FILE* telemetry = nullptr;
    int frameIndex = 0;
    if (recording) {
        std::filesystem::create_directories(recordDir);
        const std::string telemetryPath = recordDir + "/telemetry.csv";
        telemetry = std::fopen(telemetryPath.c_str(), "w");
        std::fprintf(telemetry, "frame,sim_time,leader_mph,ego_mph,gap_m,gap_err_m\n");
        std::cout << "Recording chase-cam frames to " << recordDir << "/\n\n";
    }

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

        if (recording && step % kCaptureEveryNSteps == 0) {
            const auto egoPos = libtraci::Vehicle::getPosition(kEgoId);
            double centerX = egoPos.x;
            if (!leaderId.empty()) {
                const auto leaderPos = libtraci::Vehicle::getPosition(leaderId);
                centerX = 0.5 * (egoPos.x + leaderPos.x);
            }
            libtraci::GUI::setBoundary("View #0", centerX - kChaseCamHalfWidth, -kChaseCamHalfHeight,
                                        centerX + kChaseCamHalfWidth, kChaseCamHalfHeight);
            char filename[256];
            std::snprintf(filename, sizeof(filename), "%s/frame_%06d.png", recordDir.c_str(), frameIndex);
            libtraci::GUI::screenshot("View #0", filename, kFrameW, kFrameH);
            std::fprintf(telemetry, "%d,%.1f,%.1f,%.1f,%.1f,%.1f\n", frameIndex, libtraci::Simulation::getTime(),
                         leaderSpeedForLog / kMphToMps, egoSpeed / kMphToMps, gap, gapErrForLog);
            ++frameIndex;
        }

        if (step % printEvery == 0) {
            std::cout << "  " << std::setw(5) << std::fixed << std::setprecision(1)
                       << libtraci::Simulation::getTime() << "   " << std::setw(10) << (leaderSpeedForLog / kMphToMps)
                       << "   " << std::setw(8) << (egoSpeed / kMphToMps) << "   " << std::setw(6) << gap << "   "
                       << std::setw(7) << gapErrForLog << "\n";
        }
    }

    libtraci::Simulation::close();
    if (recording) {
        std::fclose(telemetry);
        std::cout << "\n"
                   << frameIndex << " frames recorded to " << recordDir << "/. Encode with:\n"
                   << "  ffmpeg -framerate " << kVideoFps << " -i " << recordDir << "/frame_%06d.png "
                   << "-c:v libx264 -pix_fmt yuv420p " << recordDir << "/sumo_acc_demo.mp4\n";
    }

    std::cout << "\nWorst |gap error| observed: " << worstGapError << " m\n";
    std::cout << "Smallest gap observed:      " << minGap << " m\n";
    std::cout << (minGap > 1.0 ? "ACC maintained safe spacing across the full 10-mile highway.\n"
                                 : "ACC spacing became unsafe!\n");
    return 0;
}
