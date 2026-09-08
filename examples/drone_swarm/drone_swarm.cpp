// Drone swarm planning and control in 3D, with static obstacles: a
// boids-style flock of drones flies toward a shared goal while avoiding
// collisions with each other *and* with obstacles in the environment,
// demonstrating two different techniques gain-scheduling the same kind of
// behavior side by side:
//   - Each drone classifies how crowded its local neighborhood is using
//     an HDC (hyperdimensional computing) prototype block, and uses that
//     to scale its separation/cohesion gains (crowded -> push apart
//     harder, sparse -> pull together harder).
//   - Each drone also runs a small Mamdani FIS on its clearance to the
//     nearest obstacle to decide how hard to push away from it (far ->
//     ignore it, near -> repel strongly), the same "near/far -> gain"
//     gain-scheduling pattern used by the quadcopter's fuzzy PID
//     scheduler, applied here to a geometric avoidance behavior instead
//     of a control-loop blend.
// This shows fuzzy logic and HDC sitting side by side behind the same
// IBlock interface, each doing the kind of local-context classification
// it's best suited for.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;
using namespace fuzzylib::blocks;

namespace {

constexpr int kNumDrones = 12;
constexpr double kSenseRadius = 18.0;        // neighbor sensing range, for flocking
constexpr double kObstacleSenseRadius = 25.0;  // obstacle sensing range
constexpr double kMinSafeDistance = 2.0;      // minimum tolerable drone-drone separation
constexpr double kMaxSpeed = 6.0;
constexpr double kMaxAccel = 4.0;
constexpr double kDt = 0.1;
constexpr double kSimTime = 90.0;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        const double len = length();
        return len > 1e-9 ? (*this) * (1.0 / len) : Vec3{0.0, 0.0, 0.0};
    }
};

struct Drone {
    Vec3 pos, vel;
};

struct Obstacle {
    Vec3 center;
    double radius;
};

// Classifies local crowding density (fraction of neighbors within sense
// radius, relative to a "fully crowded" reference count) into 5 discrete
// levels via nearest-prototype matching in hyperspace, and reports back
// that same level as a [0,1] "crowdingLevel" scalar for gain scheduling.
std::shared_ptr<HDCPrototypeBlock> buildCrowdingClassifier() {
    auto space = std::make_shared<hdc::HDCSpace>(4000, /*seed=*/2026);
    auto block = std::make_shared<HDCPrototypeBlock>(space);
    block->addInput("density", 0.0, 1.0, /*levels=*/50);
    for (double level : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        block->addPrototype("level_" + std::to_string(level), {{"density", level}}, level);
    }
    return block;
}

// Obstacle-avoidance urgency: near an obstacle's surface -> push hard;
// well clear of it -> essentially ignore it. Same gain-scheduling idea as
// the crowding classifier, but via a Mamdani FIS instead of HDC.
MamdaniEngine buildObstacleAvoidanceFIS() {
    LinguisticVariable clearance("clearance", 0.0, kObstacleSenseRadius);
    clearance.addTerm("near", mf::make<mf::Trapezoidal>(0.0, 0.0, 2.0, 10.0));
    clearance.addTerm("far", mf::make<mf::Trapezoidal>(6.0, 14.0, kObstacleSenseRadius, kObstacleSenseRadius));

    LinguisticVariable gain("avoidGain", 0.0, 1.0);
    gain.addTerm("low", mf::make<mf::Trapezoidal>(0.0, 0.0, 0.1, 0.3));
    gain.addTerm("high", mf::make<mf::Trapezoidal>(0.3, 0.7, 1.0, 1.0));

    MamdaniEngine engine;
    engine.addInput(clearance).addOutput(gain);
    engine.rules().add(Rule(antecedent::is("clearance", "near")).then(MamdaniConsequent{"avoidGain", "high"}));
    engine.rules().add(Rule(antecedent::is("clearance", "far")).then(MamdaniConsequent{"avoidGain", "low"}));
    return engine;
}

Vec3 limit(Vec3 v, double maxLen) {
    const double len = v.length();
    if (len > maxLen && len > 1e-9) return v * (maxLen / len);
    return v;
}

}  // namespace

int main() {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jitter(-1.0, 1.0);

    // Start the swarm on a loose grid (with a little random jitter) rather
    // than pure random scatter: a real swarm launches from staged
    // positions, and this guarantees a safe initial spacing so the
    // collision check below measures the controller's behavior in flight
    // rather than an accident of random placement.
    constexpr int kGridCols = 4;
    constexpr double kGridSpacing = 6.0;
    std::vector<Drone> drones(kNumDrones);
    for (int i = 0; i < kNumDrones; ++i) {
        const auto col = static_cast<double>(i % kGridCols);
        const auto row = static_cast<double>(i / kGridCols);  // NOLINT(bugprone-integer-division) -- intentional grid-row index
        drones[i].pos = {col * kGridSpacing + jitter(rng), row * kGridSpacing + jitter(rng), jitter(rng)};
        drones[i].vel = {0.0, 0.0, 0.0};
    }
    const Vec3 goal{200.0, 150.0, 60.0};

    // Two obstacles roughly along the direct path to the goal, sized
    // larger than the formation's typical spread so the swarm genuinely
    // has to maneuver around them rather than squeeze through.
    const std::vector<Obstacle> obstacles{
        {{70.0, 50.0, 20.0}, 18.0},
        {{140.0, 100.0, 40.0}, 16.0},
    };

    auto crowdingClassifier = buildCrowdingClassifier();
    const MamdaniEngine obstacleFIS = buildObstacleAvoidanceFIS();
    constexpr double kMaxExpectedNeighbors = 6.0;  // "fully crowded" reference
    constexpr double kMaxObstaclePush = 10.0;      // acceleration at full avoidance gain

    const int steps = static_cast<int>(kSimTime / kDt);
    const int printEvery = static_cast<int>(5.0 / kDt);

    std::cout << "Drone swarm planning (" << kNumDrones << " drones, 3D, " << obstacles.size()
               << " obstacles, HDC + fuzzy gain scheduling)\n\n";
    std::cout << "  time   avgDistToGoal   formationSpread   avgCrowding   minPairDist   minObstacleClearance\n";

    double globalMinPairDist = 1e9;
    double globalMinObstacleClearance = 1e9;

    for (int step = 0; step <= steps; ++step) {
        std::vector<Vec3> accelerations(kNumDrones);
        double crowdingSum = 0.0;
        double minPairDist = 1e9;
        double minObstacleClearance = 1e9;

        for (int i = 0; i < kNumDrones; ++i) {
            Vec3 separation{0.0, 0.0, 0.0};
            Vec3 cohesionCenter{0.0, 0.0, 0.0};
            Vec3 avgVelocity{0.0, 0.0, 0.0};
            int neighborCount = 0;

            for (int j = 0; j < kNumDrones; ++j) {
                if (i == j) continue;
                const Vec3 delta = drones[i].pos - drones[j].pos;
                const double dist = std::max(0.05, delta.length());
                minPairDist = std::min(minPairDist, dist);
                if (dist < kSenseRadius) {
                    ++neighborCount;
                    separation = separation + delta * (1.0 / (dist * dist));
                    cohesionCenter = cohesionCenter + drones[j].pos;
                    avgVelocity = avgVelocity + drones[j].vel;
                }
            }

            const double density =
                std::min(1.0, static_cast<double>(neighborCount) / kMaxExpectedNeighbors);
            const auto crowding = crowdingClassifier->evaluate({{"density", density}});
            const double crowdingLevel = crowding.at("value");
            crowdingSum += crowdingLevel;

            const double separationGain = 8.0 * (0.4 + 1.6 * crowdingLevel);
            const double cohesionGain = 0.6 * (1.2 - 0.9 * crowdingLevel);
            constexpr double kAlignmentGain = 0.4;
            constexpr double kGoalGain = 0.5;

            Vec3 cohesion{0.0, 0.0, 0.0};
            Vec3 alignment{0.0, 0.0, 0.0};
            if (neighborCount > 0) {
                cohesionCenter = cohesionCenter * (1.0 / neighborCount);
                cohesion = (cohesionCenter - drones[i].pos) * cohesionGain;
                avgVelocity = avgVelocity * (1.0 / neighborCount);
                alignment = (avgVelocity - drones[i].vel) * kAlignmentGain;
            }

            const Vec3 toGoal = goal - drones[i].pos;
            const double goalDist = std::max(1e-6, toGoal.length());
            const Vec3 goalPull = (toGoal * (1.0 / goalDist)) * kGoalGain * std::min(goalDist, 20.0);

            // Fuzzy obstacle avoidance: push away from every obstacle
            // within sensing range, harder the closer the drone gets. Track
            // the *unclamped* signed clearance (negative if actually
            // inside the obstacle) for the safety report below — clamping
            // it to zero here would silently hide a real penetration.
            Vec3 avoidance{0.0, 0.0, 0.0};
            double dangerLevel = 0.0;
            for (const auto& obstacle : obstacles) {
                const Vec3 toDrone = drones[i].pos - obstacle.center;
                const double distToCenter = toDrone.length();
                const double rawClearance = distToCenter - obstacle.radius;
                minObstacleClearance = std::min(minObstacleClearance, rawClearance);

                const double fisClearance = std::max(0.0, std::min(kObstacleSenseRadius, rawClearance));
                if (rawClearance < kObstacleSenseRadius) {
                    const auto avoid = obstacleFIS.evaluate({{"clearance", fisClearance}});
                    const double avoidGain = avoid.at("avoidGain");
                    dangerLevel = std::max(dangerLevel, avoidGain);
                    avoidance = avoidance + toDrone.normalized() * (avoidGain * kMaxObstaclePush);
                }
            }

            // A purely radial repulsion only decelerates a head-on
            // approach, it doesn't add sideways deflection — so goal-
            // seeking and avoidance can nearly cancel instead of steering
            // around. Suppressing the "keep going" behaviors (goal pull,
            // cohesion, alignment) in proportion to danger lets whatever
            // small lateral component avoidance/separation/neighbor drift
            // contributes actually turn the drone, rather than being
            // fought by a forward push of comparable size; a widened
            // accel budget under high danger gives it the authority to
            // actually execute that turn in time.
            const Vec3 accel = separation * separationGain +
                                 (cohesion + alignment + goalPull) * (1.0 - dangerLevel) + avoidance;
            const double effectiveMaxAccel = kMaxAccel * (1.0 + 1.5 * dangerLevel);
            accelerations[i] = limit(accel, effectiveMaxAccel);
        }

        for (int i = 0; i < kNumDrones; ++i) {
            drones[i].vel = limit(drones[i].vel + accelerations[i] * kDt, kMaxSpeed);
            drones[i].pos = drones[i].pos + drones[i].vel * kDt;
        }
        globalMinPairDist = std::min(globalMinPairDist, minPairDist);
        globalMinObstacleClearance = std::min(globalMinObstacleClearance, minObstacleClearance);

        if (step % printEvery == 0) {
            Vec3 centroid{0.0, 0.0, 0.0};
            double avgDistToGoal = 0.0;
            for (const auto& d : drones) {
                centroid = centroid + d.pos;
                avgDistToGoal += (goal - d.pos).length();
            }
            centroid = centroid * (1.0 / kNumDrones);
            avgDistToGoal /= kNumDrones;

            double spread = 0.0;
            for (const auto& d : drones) spread += (d.pos - centroid).length();
            spread /= kNumDrones;

            std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(5) << (step * kDt) << "   "
                       << std::setw(13) << avgDistToGoal << "   " << std::setw(15) << spread << "   "
                       << std::setprecision(2) << std::setw(11) << (crowdingSum / kNumDrones) << "   "
                       << std::setw(11) << minPairDist << "   " << std::setw(20) << minObstacleClearance << "\n";
        }
    }

    double finalAvgDist = 0.0;
    for (const auto& d : drones) finalAvgDist += (goal - d.pos).length();
    finalAvgDist /= kNumDrones;
    std::cout << "\nFinal average distance to goal: " << finalAvgDist << "\n";
    std::cout << "The swarm " << (finalAvgDist < 30.0 ? "successfully converged on the goal.\n"
                                                          : "did not fully converge in the simulated time.\n");
    std::cout << "Closest any two drones ever got: " << globalMinPairDist << " m -- "
               << (globalMinPairDist > kMinSafeDistance ? "no drone-drone collisions.\n"
                                                           : "UNSAFE, a drone-drone collision occurred!\n");
    std::cout << "Closest approach to an obstacle surface: " << globalMinObstacleClearance << " m -- "
               << (globalMinObstacleClearance > 0.0 ? "no obstacle collisions.\n"
                                                       : "UNSAFE, an obstacle collision occurred!\n");
    return 0;
}
