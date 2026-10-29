#pragma once

// The thesis' SUMO experiment, driven through libtraci: five vehicles depart
// together on a 40-mile highway, a human-driver (Krauss) lead and four
// followers. After a 300 s Krauss warm-up the four followers switch to the
// driving model under test -- Krauss, SUMO's CACC, the hand-picked fuzzy
// system (FIS) or the genetically tuned one (GFS) -- and the run measures how
// well they hold a 1 s time gap to the vehicle ahead.
//
// This is a port of run() in the thesis' test_sumo_script.py. Where the
// Python has quirks that shape the published numbers (the TTC speed-difference
// indexing, list-index slicing of the fitness window, ...) they are kept and
// called out in comments, so the numbers stay comparable to the thesis.

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "thesis_fis.hpp"

namespace thesis {

enum class Model { Krauss, Cacc, Fis, Gfs };

const char* modelName(Model model);

// The three controllers' membership-function parameters.
struct FisSet {
    Triples lon1;  // gap error + rate -> acceleration
    Triples lat;   // time loss + rate -> change lanes
    Triples lon2;  // platoon leader after a lane change

    static FisSet handPicked() { return {fisLon1(), fisLat(), fisLon2()}; }
    static FisSet tuned() { return {gfsLon1(), gfsLat(), gfsLon2()}; }
};

struct ScenarioOptions {
    Model model = Model::Fis;
    bool singleLane = false;  // highway_2_mod (thesis scenarios 1-2) instead of highway_2 (3-4)
    bool slowDown = false;    // the lead slows to ~55 mph at about 9 minutes (scenarios 2 and 4)
    bool gui = false;
    int takeoverStep = 300;  // followers hand over to the model under test
    int endStep = 2000;      // end of the measured window
    bool stopAtEnd = false;  // GA training: no need to simulate past endStep
    unsigned seed = 0;       // 0 keeps SUMO's default seed, as the thesis did
    std::string scenarioDir;  // folder holding highway_2*.{sumocfg,rou.xml,net.xml}
};

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Per-vehicle samples recorded once per simulation step (what the thesis got
// from SUMO's fcd-output).
struct VehicleTrace {
    std::vector<double> time;
    std::vector<double> x;
    std::vector<double> speed;
    std::vector<double> accel;
};

// Everything the thesis' run() returned, plus the fcd-style traces. Per-vehicle
// series are indexed [0..3] for followers 1..4; each is appended to only on the
// steps the Python appended to it, so their lengths can differ.
struct RunResult {
    std::array<std::vector<double>, 4> gapError;      // time-gap error, s
    std::array<std::vector<double>, 4> gapErrorRate;  // change in gap error per step
    std::array<std::vector<double>, 4> gap;           // time gap, s
    std::vector<std::array<double, 4>> ttc;           // time to collision rows, s (NaN = n/a)
    std::vector<double> totalTimeLoss;                // summed SUMO time loss of the followers
    std::vector<int> laneChangeDecision;
    std::array<VehicleTrace, 5> trace;                // vehicles 0..4

    // Sum of |gap error| over the fitness window, as the thesis computed it
    // (see gapErrorFitness). NaN for a vehicle's window makes it count as 0.
    double fitness(int takeoverStep, int endStep) const;
    double totalTimeLost() const;
    double minTtc(int takeoverStep, int endStep) const;
};

// Runs one scenario to completion. Throws if SUMO cannot be started or a fuzzy
// controller fails (std::runtime_error when no rule fires).
RunResult runScenario(const ScenarioOptions& options, const FisSet& fis);

}  // namespace thesis
