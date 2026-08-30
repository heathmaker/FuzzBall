// Car platooning: a chain of vehicles uses adaptive cruise control (ACC)
// to track a constant-time-headway gap behind the vehicle ahead. Each
// follower runs the *same* gain-scheduled TSK/Sugeno controller (9 rules
// over gap-error x relative-velocity), demonstrating that one FIS instance
// scales cleanly to a multi-agent system. The leader drives a scripted
// velocity profile (cruise, accelerate, hard brake, cruise) to show the
// platoon absorbing a disturbance without any gap collapsing.

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;

namespace {

constexpr double kStandstillGap = 5.0;   // meters, gap at v=0
constexpr double kTimeHeadway = 1.0;     // seconds
constexpr double kMaxAccel = 3.0;        // m/s^2, comfortable limit
constexpr double kMaxSpeed = 35.0;       // m/s
constexpr double kDt = 0.1;              // seconds
constexpr int kNumFollowers = 4;
constexpr double kSimTime = 40.0;        // seconds

// Piecewise-linear leader speed profile (m/s) over time (s): cruise at 15,
// ramp to a 25 m/s highway speed, hold, then a hard brake to 10 m/s, hold.
double leaderVelocity(double t) {
    struct Point {
        double t, v;
    };
    static const std::array<Point, 5> profile{{{0.0, 15.0}, {5.0, 15.0}, {15.0, 25.0}, {25.0, 25.0},
                                                  {30.0, 10.0}}};
    if (t <= profile.front().t) return profile.front().v;
    if (t >= profile.back().t) return profile.back().v;
    for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
        if (t >= profile[i].t && t <= profile[i + 1].t) {
            const double frac = (t - profile[i].t) / (profile[i + 1].t - profile[i].t);
            return profile[i].v + frac * (profile[i + 1].v - profile[i].v);
        }
    }
    return profile.back().v;
}

// A 3x3 = 9-rule gain-scheduled first-order TSK controller: the local
// linear law accel = Kp*gapError + Kd*relVel is the same everywhere, but
// its *aggressiveness* is scaled up in the corner rules (both cues agree
// something is wrong) and scaled down near the origin (fine tracking),
// which is the essence of Sugeno gain scheduling.
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

int main() {
    const SugenoEngine acc = buildACCController();

    const int numVehicles = kNumFollowers + 1;  // index 0 = leader
    std::vector<double> pos(numVehicles), vel(numVehicles, 15.0);
    for (int i = 0; i < numVehicles; ++i) {
        pos[i] = static_cast<double>(kNumFollowers - i) * desiredGap(15.0);
    }

    const int steps = static_cast<int>(kSimTime / kDt);
    const int printEvery = static_cast<int>(2.0 / kDt);

    std::cout << "Car platooning (leader + " << kNumFollowers << " followers, shared 9-rule TSK ACC)\n\n";
    std::cout << "  time   leaderV  " ;
    for (int i = 1; i <= kNumFollowers; ++i) std::cout << "gapErr" << i << "  ";
    std::cout << "\n";

    double worstGapError = 0.0;
    double minGap = 1e9;

    std::vector<double> nextPos(numVehicles), nextVel(numVehicles);
    for (int step = 0; step <= steps; ++step) {
        const double t = step * kDt;

        // Snapshot -> compute every vehicle's next state from the same
        // "start of step" values (Jacobi-style synchronous update).
        nextVel[0] = leaderVelocity(t);
        nextPos[0] = pos[0] + nextVel[0] * kDt;

        std::vector<double> gapErrors(kNumFollowers + 1, 0.0);
        for (int i = 1; i < numVehicles; ++i) {
            const double gap = pos[i - 1] - pos[i];
            const double gapErr = gap - desiredGap(vel[i]);
            const double relVel = vel[i - 1] - vel[i];
            gapErrors[i] = gapErr;
            minGap = std::min(minGap, gap);
            worstGapError = std::max(worstGapError, std::fabs(gapErr));

            const auto result = acc.evaluate({{"gapError", std::clamp(gapErr, -15.0, 15.0)},
                                                {"relVel", std::clamp(relVel, -8.0, 8.0)}});
            const double accel = std::clamp(result.at("accel"), -kMaxAccel, kMaxAccel);

            nextVel[i] = std::clamp(vel[i] + accel * kDt, 0.0, kMaxSpeed);
            nextPos[i] = pos[i] + nextVel[i] * kDt;
        }

        if (step % printEvery == 0) {
            std::cout << "  " << std::setw(4) << std::fixed << std::setprecision(1) << t << "   "
                       << std::setw(6) << vel[0] << "   ";
            for (int i = 1; i <= kNumFollowers; ++i) std::cout << std::setw(8) << gapErrors[i] << "  ";
            std::cout << "\n";
        }

        pos = nextPos;
        vel = nextVel;
    }

    std::cout << "\nWorst |gap error| observed: " << worstGapError << " m\n";
    std::cout << "Smallest gap observed:      " << minGap << " m\n";
    std::cout << (minGap > 1.0 ? "Platoon maintained safe spacing throughout.\n"
                                 : "Platoon spacing became unsafe!\n");
    return 0;
}
