// Drone swarm planning and control: a boids-style flock of drones flies
// toward a shared goal while avoiding collisions and staying loosely
// cohesive. Each drone classifies how crowded its local neighborhood is
// using an HDC (hyperdimensional computing) prototype block — a
// lightweight, robust way to turn a continuous local measurement into a
// discretized "regime" — and uses that to scale its separation/cohesion
// gains: crowded -> push apart harder, sparse -> pull together harder.
// This is the same fuzzy-style gain-scheduling idea used in the quadcopter
// example, but driven by HDC pattern-matching instead of a FIS, to show
// the two techniques are interchangeable behind the same block interface.

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
constexpr double kSenseRadius = 18.0;
constexpr double kMinSafeDistance = 2.0;
constexpr double kMaxSpeed = 6.0;
constexpr double kMaxAccel = 4.0;
constexpr double kDt = 0.1;
constexpr double kSimTime = 60.0;

struct Vec2 {
    double x = 0.0, y = 0.0;
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    double length() const { return std::sqrt(x * x + y * y); }
};

struct Drone {
    Vec2 pos, vel;
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

Vec2 limit(Vec2 v, double maxLen) {
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
        const double col = static_cast<double>(i % kGridCols);
        const double row = static_cast<double>(i / kGridCols);
        drones[i].pos = {col * kGridSpacing + jitter(rng), row * kGridSpacing + jitter(rng)};
        drones[i].vel = {0.0, 0.0};
    }
    const Vec2 goal{200.0, 150.0};

    auto crowdingClassifier = buildCrowdingClassifier();
    constexpr double kMaxExpectedNeighbors = 6.0;  // "fully crowded" reference

    const int steps = static_cast<int>(kSimTime / kDt);
    const int printEvery = static_cast<int>(5.0 / kDt);

    std::cout << "Drone swarm planning (" << kNumDrones << " drones, HDC-scheduled boids)\n\n";
    std::cout << "  time   avgDistToGoal   formationSpread   avgCrowding   minPairDist\n";

    double globalMinPairDist = 1e9;
    for (int step = 0; step <= steps; ++step) {
        std::vector<Vec2> accelerations(kNumDrones);
        double crowdingSum = 0.0;
        double minPairDist = 1e9;

        for (int i = 0; i < kNumDrones; ++i) {
            Vec2 separation{0.0, 0.0};
            Vec2 cohesionCenter{0.0, 0.0};
            Vec2 avgVelocity{0.0, 0.0};
            int neighborCount = 0;

            for (int j = 0; j < kNumDrones; ++j) {
                if (i == j) continue;
                const Vec2 delta = drones[i].pos - drones[j].pos;
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

            Vec2 cohesion{0.0, 0.0};
            Vec2 alignment{0.0, 0.0};
            if (neighborCount > 0) {
                cohesionCenter = cohesionCenter * (1.0 / neighborCount);
                cohesion = (cohesionCenter - drones[i].pos) * cohesionGain;
                avgVelocity = avgVelocity * (1.0 / neighborCount);
                alignment = (avgVelocity - drones[i].vel) * kAlignmentGain;
            }

            const Vec2 toGoal = goal - drones[i].pos;
            const double goalDist = std::max(1e-6, toGoal.length());
            const Vec2 goalPull = (toGoal * (1.0 / goalDist)) * kGoalGain * std::min(goalDist, 20.0);

            Vec2 accel = separation * separationGain + cohesion + alignment + goalPull;
            accelerations[i] = limit(accel, kMaxAccel);
        }

        for (int i = 0; i < kNumDrones; ++i) {
            drones[i].vel = limit(drones[i].vel + accelerations[i] * kDt, kMaxSpeed);
            drones[i].pos = drones[i].pos + drones[i].vel * kDt;
        }
        globalMinPairDist = std::min(globalMinPairDist, minPairDist);

        if (step % printEvery == 0) {
            Vec2 centroid{0.0, 0.0};
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

            std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(5) << (step * kDt)
                       << "   " << std::setw(13) << avgDistToGoal << "   " << std::setw(15) << spread
                       << "   " << std::setprecision(2) << std::setw(11) << (crowdingSum / kNumDrones)
                       << "   " << std::setw(11) << minPairDist << "\n";
        }
    }

    double finalAvgDist = 0.0;
    for (const auto& d : drones) finalAvgDist += (goal - d.pos).length();
    finalAvgDist /= kNumDrones;
    std::cout << "\nFinal average distance to goal: " << finalAvgDist << "\n";
    std::cout << "The swarm " << (finalAvgDist < 30.0 ? "successfully converged on the goal.\n"
                                                          : "did not fully converge in the simulated time.\n");
    std::cout << "Closest any two drones ever got: " << globalMinPairDist << " m -- "
               << (globalMinPairDist > kMinSafeDistance ? "no collisions.\n" : "UNSAFE, a collision occurred!\n");
    return 0;
}
