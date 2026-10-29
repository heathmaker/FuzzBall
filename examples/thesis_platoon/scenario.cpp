#include "scenario.hpp"

#include <libsumo/libtraci.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace thesis {

const char* modelName(Model model) {
    switch (model) {
        case Model::Krauss: return "Krauss";
        case Model::Cacc: return "CACC";
        case Model::Fis: return "FIS";
        case Model::Gfs: return "GFS";
    }
    return "?";
}

namespace {

constexpr double kIdealGap = 1.0;      // s
constexpr double kVehicleLength = 5.0;  // m
constexpr double kDesiredSpeed = 31.292;  // m/s, 70 mph
constexpr int kFollowers = 4;

using Vec5 = std::array<double, 5>;

// A step's worth of state for vehicles 0..4, indexed by vehicle id.
struct Snapshot {
    Vec5 x{};
    Vec5 speed{};
    std::array<double, 4> timeLoss{};  // vehicles 1..4

    double totalTimeLoss() const { return std::accumulate(timeLoss.begin(), timeLoss.end(), 0.0); }
};

// Lane index from a lane id such as "gneE0_1" or an internal lane ":gneJ1_0_1".
// The thesis read it as int(lane_id[6]); parsing the suffix gives the same
// answer on edge lanes and also copes with junction-internal lanes.
int laneIndex(const std::string& laneId) {
    const auto underscore = laneId.rfind('_');
    if (underscore == std::string::npos) throw std::runtime_error("unexpected lane id '" + laneId + "'");
    return std::stoi(laneId.substr(underscore + 1));
}

int laneOf(int vehicle) { return laneIndex(libtraci::Vehicle::getLaneID(std::to_string(vehicle))); }

// Time gap in seconds from vehicle `ahead` to the follower, following the
// thesis' definition: bumper-to-bumper distance over the follower's speed.
double timeGap(double positionAhead, double position, double followerSpeed) {
    if (followerSpeed == 0.0) throw std::runtime_error("division by zero: stopped follower");
    return (positionAhead - kVehicleLength - position) / followerSpeed;
}

Snapshot snapshot() {
    Snapshot s;
    for (int id = 0; id <= kFollowers; ++id) {
        const std::string name = std::to_string(id);
        s.x[id] = libtraci::Vehicle::getPosition(name).x;
        s.speed[id] = libtraci::Vehicle::getSpeed(name);
        if (id > 0) s.timeLoss[id - 1] = libtraci::Vehicle::getTimeLoss(name);
    }
    return s;
}

// The thesis' calculateTimeToCollision(). Reproduced as written, including the
// speed difference being taken between vehicle i and the vehicle *ahead of it*
// (index i-1, wrapping to vehicle 4 for i = 0) rather than the follower: the
// published TTC numbers come from this.
std::array<double, 4> timeToCollisionAsPublished(const Snapshot& s) {
    std::array<double, 4> ttc{};
    for (int i = 0; i < 4; ++i) {
        const double speedDiff = s.speed[i] - s.speed[(i + 4) % 5];
        ttc[i] = speedDiff > 0 ? (s.x[i] - kVehicleLength - s.x[i + 1]) / speedDiff : kNaN;
    }
    return ttc;
}

struct FisInputs {
    double gap;
    double gapError;
    double gapErrorRate;
    double acceleration;
    long decision;
};

// FuzzyHWClass.calc_Inputs: the gap-error FIS plus the lane-change FIS for one follower.
FisInputs calcInputs(const LongitudinalFis& lon1, const LaneChangeFis& lat, double positionAhead, double previousGap,
                     double position, double speed, double avgTimeLoss, double timeLossRate) {
    FisInputs r{};
    r.gap = timeGap(positionAhead, position, speed);
    r.gapError = r.gap - kIdealGap;
    r.gapErrorRate = r.gapError - (previousGap - kIdealGap);
    r.acceleration = lon1.acceleration(r.gapError, r.gapErrorRate * 30);  // x30: the rate's universe is +-1
    r.decision = std::lround(std::nearbyint(lat.decision(avgTimeLoss, timeLossRate)));
    return r;
}

// Everything one scenario run needs: the options, the controllers and the
// accumulating result, plus the run-to-run state the Python kept in locals.
class Runner {
public:
    Runner(ScenarioOptions options, const FisSet& fis)
        : options_(std::move(options)), lon1_(fis.lon1), lat_(fis.lat), lon2_(fis.lon2) {}

    RunResult run();

private:
    void recordTrace();
    void slowDownLead(int step) const;

    void krauss(int step);
    void warmupMeasureBaseline(int step);
    void controlMeasureBaseline(int step);

    void fis(int step);
    void fisWarmup();
    void fisControl(int step);
    void fisPlatoonInLeftLane(const Snapshot& s, const std::array<double, 4>& previousTimeLoss);
    void fisPlatoonInRightLane(const Snapshot& s, double avgTimeLoss, double timeLossRate);

    void appendGapError(int follower, double value) { result_.gapError[follower - 1].push_back(value); }
    double adjacentGapError(const Snapshot& s, int ahead, int follower) const {
        return timeGap(s.x[ahead], s.x[follower], s.speed[follower]) - kIdealGap;
    }

    const ScenarioOptions options_;
    LongitudinalFis lon1_;
    LaneChangeFis lat_;
    PlatoonLeaderFis lon2_;
    RunResult result_;

    bool veh1InLeftLane_ = false;  // veh1_lane_state in the Python (baseline warm-up bookkeeping)
    std::array<double, 5> previousGap_{};  // time gap per follower at the previous step (index = vehicle id)
    std::array<double, 4> previousTimeLoss_{};
};

void Runner::recordTrace() {
    const double time = libtraci::Simulation::getTime();
    for (const std::string& id : libtraci::Vehicle::getIDList()) {
        VehicleTrace& t = result_.trace.at(static_cast<size_t>(std::stoi(id)));
        t.time.push_back(time);
        t.x.push_back(libtraci::Vehicle::getPosition(id).x);
        t.speed.push_back(libtraci::Vehicle::getSpeed(id));
        t.accel.push_back(libtraci::Vehicle::getAcceleration(id));
    }
}

// Scenarios 2 and 4: the human lead brakes to ~55 mph over 90 s starting around
// 9 minutes, then recovers to 70 mph over 60 s.
void Runner::slowDownLead(int step) const {
    if (!options_.slowDown) return;
    if (525 < step && step < 615) {
        libtraci::Vehicle::slowDown("0", 24.5872, 90);
    } else if (614 < step && step < 675) {
        libtraci::Vehicle::slowDown("0", kDesiredSpeed, 60);
    }
}

// ----------------------------------------------------------------------------
// Krauss / CACC baselines. SUMO drives the followers; this code only measures.
// ----------------------------------------------------------------------------

void Runner::warmupMeasureBaseline(int step) {
    (void)step;
    const Snapshot s = snapshot();
    const bool cacc = options_.model == Model::Cacc;
    const int lane1 = laneOf(1);
    const int lane2 = cacc ? laneOf(2) : 0;

    const auto followersFrom = [&](int first) {
        for (int k = first; k <= kFollowers; ++k) appendGapError(k, adjacentGapError(s, k - 1, k));
    };

    if (lane1 == 1 || (!(cacc && lane2 == 1) && veh1InLeftLane_)) {
        // vehicle 1 has moved to the left lane: it leads there, so its own gap error is 0
        if (lane1 == 1) veh1InLeftLane_ = true;
        appendGapError(1, 0);
        followersFrom(2);
    } else if (cacc && lane2 == 1) {
        // CACC only: vehicle 2 has moved to the left lane instead
        appendGapError(1, adjacentGapError(s, 0, 1));
        appendGapError(2, 0);
        followersFrom(3);
    } else {
        followersFrom(1);
    }
    result_.totalTimeLoss.push_back(s.totalTimeLoss());
    result_.ttc.push_back(timeToCollisionAsPublished(s));
}

void Runner::controlMeasureBaseline(int step) {
    libtraci::Vehicle::changeLane("0", 0, 3);
    if (options_.model == Model::Cacc) {
        for (int k = 1; k <= kFollowers; ++k) libtraci::Vehicle::setType(std::to_string(k), "Car04");
    }
    const Snapshot s = snapshot();

    // Vehicles per lane, front to back. getLastStepVehicleIDs lists a lane
    // rear-to-front, so the edges are concatenated in route order and the
    // result reversed, as the thesis did.
    const auto vehiclesInLane = [](int lane) {
        std::vector<int> ids;
        for (const char* edge : {"gneE0", "gneE1", "gneE3"}) {
            for (const std::string& id : libtraci::Lane::getLastStepVehicleIDs(std::string(edge) + "_" + std::to_string(lane))) {
                ids.push_back(std::stoi(id));
            }
        }
        std::reverse(ids.begin(), ids.end());
        return ids;
    };

    std::array<double, 4> ttc{kNaN, kNaN, kNaN, kNaN};
    const auto laneAheadMetrics = [&](const std::vector<int>& lane, size_t index, int vehicle) {
        if (index == 0) return;
        const int ahead = lane[index - 1];
        const double speedDiff = s.speed[vehicle] - s.speed[ahead];
        if (speedDiff > 0) ttc[vehicle - 1] = (s.x[ahead] - kVehicleLength - s.x[vehicle]) / speedDiff;
    };

    if (!options_.singleLane) {
        const std::vector<int> left = vehiclesInLane(1);
        for (size_t i = 0; i < left.size(); ++i) {
            const int v = left[i];
            if (v == 0) continue;
            laneAheadMetrics(left, i, v);
            // the front car of the left lane has nothing ahead to measure against
            if (v == 1) {
                appendGapError(1, 0);
            } else {
                appendGapError(v, i == 0 ? 0 : adjacentGapError(s, left[i - 1], v));
            }
        }
    }
    const std::vector<int> right = vehiclesInLane(0);
    for (size_t i = 0; i < right.size(); ++i) {
        const int v = right[i];
        if (v == 0) continue;
        laneAheadMetrics(right, i, v);
        if (v == 1) {
            appendGapError(1, i == 0 ? kNaN : adjacentGapError(s, 0, 1));  // vehicle 1 always trails vehicle 0
        } else {
            appendGapError(v, i == 0 ? kNaN : adjacentGapError(s, right[i - 1], v));
        }
    }

    result_.totalTimeLoss.push_back(s.totalTimeLoss());
    result_.ttc.push_back(ttc);
    slowDownLead(step);
}

void Runner::krauss(int step) {
    // The same measurement code serves Krauss and CACC; for CACC SUMO swaps the
    // followers' car-following model to "Car04" in controlMeasureBaseline.
    if (30 < step && step < options_.takeoverStep + 1) {
        warmupMeasureBaseline(step);
        slowDownLead(step);
    } else if (options_.takeoverStep < step && step < options_.endStep) {
        controlMeasureBaseline(step);
    } else {
        // past the measured window: only the time-loss total and NaN gap errors are kept
        double timeLoss = 0;
        for (const std::string& id : libtraci::Vehicle::getIDList()) {
            if (std::stoi(id) > 0) timeLoss += libtraci::Vehicle::getTimeLoss(id);
        }
        for (int k = 1; k <= kFollowers; ++k) appendGapError(k, kNaN);
        result_.totalTimeLoss.push_back(timeLoss);
    }
}

// ----------------------------------------------------------------------------
// FIS / GFS: the followers' speeds are commanded from the fuzzy controllers.
// ----------------------------------------------------------------------------

void Runner::fisWarmup() {
    const Snapshot s = snapshot();
    for (int k = 1; k <= kFollowers; ++k) {
        previousGap_[k] = timeGap(s.x[k - 1], s.x[k], s.speed[k]);
        appendGapError(k, previousGap_[k] - kIdealGap);
    }
    previousTimeLoss_ = s.timeLoss;
    result_.totalTimeLoss.push_back(s.totalTimeLoss());
    result_.ttc.push_back(timeToCollisionAsPublished(s));
    result_.laneChangeDecision.push_back(0);
}

// Vehicle 1 (the platoon leader) has changed into the left lane and no longer
// follows the human lead, so its own controller is the platoon-leader FIS.
void Runner::fisPlatoonInLeftLane(const Snapshot& s, const std::array<double, 4>& previousTimeLoss) {
    result_.laneChangeDecision.push_back(0);
    std::array<std::string, 4> laneIds;
    for (int k = 1; k <= kFollowers; ++k) laneIds[k - 1] = libtraci::Vehicle::getLaneID(std::to_string(k));
    const bool allInLeftLane = std::all_of(laneIds.begin(), laneIds.end(), [](const std::string& id) {
        return laneIndex(id) == 1;
    });

    double sumDiff = 0;
    for (int i = 0; i < 4; ++i) sumDiff += s.timeLoss[i] - previousTimeLoss[i];
    const double avgTimeLoss = s.totalTimeLoss() / 4;
    const double timeLossRate = sumDiff / 4;

    if (allInLeftLane) {
        for (int k = 1; k <= kFollowers; ++k) libtraci::Vehicle::changeLane(std::to_string(k), 1, 300);
    } else {
        // Some of the platoon has not made the lane change yet. Nudge speeds so the
        // stragglers can slot in: vehicles 2 and 4 slow by 0.5 m/s, vehicle 3 speeds up.
        //
        // The Python compared lane-id *strings* against the integer 1 (`x != 1`), which is
        // always true, so every vehicle took the "not in the left lane" path: vehicles 2,
        // 3, 4 were queued to slow down (4 twice) and vehicle 3 to speed up. All of them are
        // set from their speed at the start of the step, so the later call wins: the net
        // effect below is what the thesis' runs did.
        for (int k = 1; k <= kFollowers; ++k) libtraci::Vehicle::changeLane(std::to_string(k), 1, 300);
        libtraci::Vehicle::setSpeed("2", libtraci::Vehicle::getSpeed("2") - 0.5);
        libtraci::Vehicle::setSpeed("3", libtraci::Vehicle::getSpeed("3") + 0.5);
        libtraci::Vehicle::setSpeed("4", libtraci::Vehicle::getSpeed("4") - 0.5);
    }

    // vehicles 2-4 keep following the vehicle ahead with the gap-error FIS
    std::array<FisInputs, 5> r{};
    for (int k = 2; k <= kFollowers; ++k) {
        r[k] = calcInputs(lon1_, lat_, s.x[k - 1], previousGap_[k], s.x[k], s.speed[k], avgTimeLoss, timeLossRate);
        previousGap_[k] = r[k].gap;
        result_.gap[k - 1].push_back(r[k].gap);
        appendGapError(k, r[k].gapError);
        result_.gapErrorRate[k - 1].push_back(r[k].gapErrorRate);
        if (allInLeftLane) libtraci::Vehicle::setSpeed(std::to_string(k), s.speed[k] + r[k].acceleration);
    }

    auto ttc = timeToCollisionAsPublished(s);
    ttc[0] = kNaN;

    // vehicle 1: speed error against 70 mph and the mean gap error of the vehicles behind it
    const double veh1Speed = libtraci::Vehicle::getSpeed("1");
    const double platoonGapError = (result_.gapError[1].back() + result_.gapError[2].back() + result_.gapError[3].back()) / 3;
    const double velocityError = kDesiredSpeed - veh1Speed;
    const double veh1Command = veh1Speed + lon2_.acceleration(platoonGapError, velocityError);
    if (allInLeftLane) libtraci::Vehicle::setSpeed("1", veh1Command);

    result_.ttc.push_back(ttc);
    previousTimeLoss_ = s.timeLoss;
}

void Runner::fisPlatoonInRightLane(const Snapshot& s, double avgTimeLoss, double timeLossRate) {
    std::array<FisInputs, 5> r{};
    for (int k = 1; k <= kFollowers; ++k) {
        r[k] = calcInputs(lon1_, lat_, s.x[k - 1], previousGap_[k], s.x[k], s.speed[k], avgTimeLoss, timeLossRate);
        previousGap_[k] = r[k].gap;
        result_.gap[k - 1].push_back(r[k].gap);
        appendGapError(k, r[k].gapError);
        result_.gapErrorRate[k - 1].push_back(r[k].gapErrorRate);
        libtraci::Vehicle::setSpeed(std::to_string(k), s.speed[k] + r[k].acceleration);
    }
    // Keep the platoon in the right lane unless it decides to overtake below.
    for (int k = 1; k <= kFollowers; ++k) libtraci::Vehicle::changeLane(std::to_string(k), 0, 300);

    // Overtake when every follower wants to change lanes, the platoon is settled
    // (all gap errors < 0.5 s) and the lanes are clear. The Python's `a and b and c and d == 1`
    // is "a, b, c non-zero and d == 1".
    const bool wantsChange = r[1].decision != 0 && r[2].decision != 0 && r[3].decision != 0 && r[4].decision == 1;
    bool changed = false;
    if (wantsChange) {
        const double worstGapError = std::max({r[1].gapError, r[2].gapError, r[3].gapError, r[4].gapError});
        if (worstGapError < 0.5) {
            // "Is there room in the new lane?": the thesis checked that every follower is more than
            // 250 m from a reference that, with no fifth vehicle in the scenario, is the road origin.
            const bool roomToChange = std::all_of(s.x.begin() + 1, s.x.end(), [](double x) { return std::fabs(x) > 250; });
            if (roomToChange) {
                const int targetLane = options_.singleLane ? 0 : 1;
                for (int k = 1; k <= kFollowers; ++k) libtraci::Vehicle::changeLane(std::to_string(k), targetLane, 300);
                changed = true;
            }
        }
    }
    result_.laneChangeDecision.push_back(changed ? 1 : 0);
    result_.ttc.push_back(timeToCollisionAsPublished(s));
    previousTimeLoss_ = s.timeLoss;
}

void Runner::fisControl(int step) {
    const Snapshot s = snapshot();
    result_.totalTimeLoss.push_back(s.totalTimeLoss());

    if (laneOf(1) == 1) {
        fisPlatoonInLeftLane(s, previousTimeLoss_);
    } else {
        double sumDiff = 0;
        for (int i = 0; i < 4; ++i) sumDiff += s.timeLoss[i] - previousTimeLoss_[i];
        fisPlatoonInRightLane(s, s.totalTimeLoss() / 4, sumDiff / 4);
    }
    slowDownLead(step);
}

void Runner::fis(int step) {
    if (30 < step && step < options_.takeoverStep + 1) {
        fisWarmup();
    } else if (options_.takeoverStep < step && step < options_.endStep) {
        fisControl(step);
    }
}

RunResult Runner::run() {
    const bool baseline = options_.model == Model::Krauss || options_.model == Model::Cacc;
    const std::string config = options_.scenarioDir + "/" + (options_.singleLane ? "highway_2_mod" : "highway_2");

    std::string binary = options_.gui ? "sumo-gui" : "sumo";
    if (const char* home = std::getenv("SUMO_HOME")) {
        const std::string candidate = std::string(home) + "/bin/" + binary;
        if (FILE* f = std::fopen(candidate.c_str(), "r")) {
            std::fclose(f);
            binary = candidate;
        }
    }
    std::vector<std::string> cmd = {binary, "-c", config + ".sumocfg", "--route-files", config + ".rou.xml", "--start",
                                    "--quit-on-end", "--no-step-log", "--no-warnings"};
    if (options_.seed != 0) {
        cmd.emplace_back("--seed");
        cmd.push_back(std::to_string(options_.seed));
    }
    libtraci::Simulation::start(cmd);

    try {
        int step = 0;
        while (libtraci::Simulation::getMinExpectedNumber() > 0) {
            libtraci::Simulation::step();
            if (baseline) {
                krauss(step);
            } else {
                fis(step);
            }
            recordTrace();
            ++step;
            if (options_.stopAtEnd && step >= options_.endStep) break;
        }
    } catch (...) {
        libtraci::Simulation::close();
        throw;
    }
    libtraci::Simulation::close();
    return std::move(result_);
}

}  // namespace

RunResult runScenario(const ScenarioOptions& options, const FisSet& fis) { return Runner(options, fis).run(); }

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

double RunResult::fitness(int takeoverStep, int endStep) const {
    // Python: sum(abs(list)[takeover:end]) per vehicle, then np.nansum over vehicles.
    // The slice is by list index (not simulation step) and a single NaN in a
    // vehicle's window makes that vehicle's whole sum NaN, which nansum drops.
    double total = 0;
    for (const auto& series : gapError) {
        const auto first = static_cast<size_t>(std::max(takeoverStep, 0));
        const auto last = std::min(series.size(), static_cast<size_t>(std::max(endStep, 0)));
        double sum = 0;
        for (size_t i = first; i < last; ++i) sum += std::fabs(series[i]);
        if (!std::isnan(sum)) total += sum;
    }
    return total;
}

double RunResult::totalTimeLost() const { return std::accumulate(totalTimeLoss.begin(), totalTimeLoss.end(), 0.0); }

double RunResult::minTtc(int takeoverStep, int endStep) const {
    double best = kNaN;
    const auto first = static_cast<size_t>(std::max(takeoverStep, 0));
    const auto last = std::min(ttc.size(), static_cast<size_t>(std::max(endStep, 0)));
    for (size_t i = first; i < last; ++i) {
        for (double v : ttc[i]) {
            if (!std::isnan(v) && (std::isnan(best) || v < best)) best = v;
        }
    }
    return best;
}

}  // namespace thesis
