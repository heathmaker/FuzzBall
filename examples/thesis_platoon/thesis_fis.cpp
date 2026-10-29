#include "thesis_fis.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "fuzzylib/core/membership.hpp"

namespace thesis {

namespace {

using fuzzylib::LinguisticVariable;
using fuzzylib::MamdaniEngine;
using fuzzylib::MamdaniConsequent;
using fuzzylib::Rule;

// A discretized universe of discourse like numpy.arange(start, stop, step):
// the thesis' scikit-fuzzy variables are sampled on such a grid, and every
// membership function is a sampled array rather than a formula.
struct Grid {
    double start;
    double step;
    std::size_t count;

    // numpy.arange(start, stop, step)
    static Grid arange(double start, double stop, double step) {
        return Grid{start, step, static_cast<std::size_t>(std::ceil((stop - start) / step))};
    }
    // numpy fills arange as start + i * ((start + step) - start), not start + i * step;
    // the two differ in the last bits, which decides whether a sample that lies
    // exactly on a membership function's foot is 0 or a tiny positive number.
    double delta() const { return (start + step) - start; }
    double at(std::size_t i) const { return start + static_cast<double>(i) * delta(); }
    double last() const { return at(count - 1); }
};

// skfuzzy.membership.trimf sampled on a Grid, evaluated like
// skfuzzy.interp_membership: linear interpolation between samples and
// saturation (not zero) outside the universe. Reproducing this -- instead of
// the analytic triangle -- makes the engine's fuzzification and clipped
// output curve match the Python system sample for sample.
class SampledTriangle final : public fuzzylib::MembershipFunction {
public:
    SampledTriangle(const Grid& grid, const Triple& abc) : grid_(grid), samples_(grid.count, 0.0) {
        const double a = abc[0];
        const double b = abc[1];
        const double c = abc[2];
        if (!(a <= b && b <= c)) throw std::invalid_argument("trimf requires a <= b <= c");
        for (std::size_t i = 0; i < grid.count; ++i) {
            const double x = grid.at(i);
            double y = 0.0;
            if (a != b && a < x && x < b) y = (x - a) / (b - a);
            if (b != c && b < x && x < c) y = (c - x) / (c - b);
            if (x == b) y = 1.0;
            samples_[i] = y;
        }
    }

    double operator()(double x) const override {
        const double pos = (x - grid_.start) / grid_.delta();
        if (pos <= 0.0) return samples_.front();
        if (pos >= static_cast<double>(grid_.count - 1)) return samples_.back();
        const auto lo = static_cast<std::size_t>(pos);
        const double frac = pos - static_cast<double>(lo);
        return samples_[lo] + frac * (samples_[lo + 1] - samples_[lo]);
    }

    std::unique_ptr<fuzzylib::MembershipFunction> clone() const override {
        return std::make_unique<SampledTriangle>(*this);
    }

private:
    Grid grid_;
    std::vector<double> samples_;
};

LinguisticVariable makeVariable(const std::string& name, const Grid& grid,
                                const std::vector<std::string>& termNames, const Triples& mf,
                                std::size_t firstTriple) {
    LinguisticVariable variable(name, grid.start, grid.last());
    for (std::size_t i = 0; i < termNames.size(); ++i) {
        variable.addTerm(termNames[i], std::make_shared<SampledTriangle>(grid, mf.at(firstTriple + i)));
    }
    return variable;
}

using TermPair = std::pair<const char*, const char*>;

// IF (a1 AND b1) OR (a2 AND b2) OR ... THEN output IS term. All of the
// thesis' rules have exactly this "OR of AND pairs" shape.
void addRule(MamdaniEngine& engine, const std::string& varA, const std::string& varB,
             const std::vector<TermPair>& pairs, const std::string& outVar, const std::string& outTerm) {
    using namespace fuzzylib::antecedent;
    fuzzylib::AntecedentPtr condition;
    for (const auto& [termA, termB] : pairs) {
        auto pair = is(varA, termA) & is(varB, termB);
        condition = condition ? (std::move(condition) | std::move(pair)) : std::move(pair);
    }
    engine.rules().add(Rule(std::move(condition)).then(MamdaniConsequent{outVar, outTerm}));
}

// The thesis computes with scikit-fuzzy's min/max logic, clipped implication
// and max aggregation: the engine's defaults. The engine's output grid is the
// output variable's own sampled universe, refined at the clip points and
// defuzzified with the exact piecewise-linear centroid, as scikit-fuzzy does.
MamdaniEngine makeEngine(const Grid& outputGrid) {
    MamdaniEngine engine(fuzzylib::TNorm::zadeh(), fuzzylib::Aggregator::max(),
                         static_cast<int>(outputGrid.count));
    engine.setRefineAtCuts(true);
    return engine;
}

constexpr fuzzylib::DefuzzMethod kDefuzz = fuzzylib::DefuzzMethod::CentroidExact;

const std::vector<std::string> kGapTerms = {"ExtraExtraExtraSmall", "ExtraExtraSmall", "ExtraSmall", "Small",
                                            "Medium",               "Large",           "ExtraLarge"};
const std::vector<std::string> kRateTerms = {"ExtraExtraExtraSmall", "ExtraExtraSmall", "ExtraSmall", "Small",
                                             "Medium",               "Large",           "ExtraLarge",
                                             "ExtraExtraLarge"};
const std::vector<std::string> kAccelTerms = {"ExtraExtraSmall", "ExtraSmall", "Small",          "Medium",
                                              "Large",           "ExtraLarge", "ExtraExtraLarge"};
const std::vector<std::string> kThreeTerms = {"Small", "Medium", "Large"};

const char* const kGapError = "gap-error-value";
const char* const kGapErrorRate = "gap-error-change-rate-value";
const char* const kAcceleration = "acceleration-value";

}  // namespace

LongitudinalFis::LongitudinalFis(const Triples& mf)
    : engine_([&mf] {
          if (mf.size() != kLon1Triples) throw std::invalid_argument("longitudinal FIS needs 22 triples");
          const Grid gapGrid = Grid::arange(-2, 3, 0.01);
          const Grid rateGrid = Grid::arange(-1, 1, 0.001);
          const Grid accelGrid = Grid::arange(-5, 5.1, 0.001);

          MamdaniEngine engine = makeEngine(accelGrid);
          engine.addInput(makeVariable(kGapError, gapGrid, kGapTerms, mf, 0));
          engine.addInput(makeVariable(kGapErrorRate, rateGrid, kRateTerms, mf, 7));
          engine.addOutput(makeVariable(kAcceleration, accelGrid, kAccelTerms, mf, 15));

          const auto rule = [&engine](const std::vector<TermPair>& pairs, const char* out) {
              addRule(engine, kGapError, kGapErrorRate, pairs, kAcceleration, out);
          };
          // Terms below: EES = ExtraExtraSmall, ES = ExtraSmall, S = Small, ...
          rule({{"ExtraExtraSmall", "ExtraExtraSmall"}, {"ExtraExtraExtraSmall", "ExtraExtraExtraSmall"}},
               "ExtraExtraSmall");
          rule({{"ExtraSmall", "ExtraExtraSmall"}}, "ExtraSmall");
          rule({{"Small", "ExtraExtraSmall"},
                {"Medium", "ExtraExtraSmall"},
                {"Large", "ExtraExtraSmall"},
                {"ExtraLarge", "ExtraExtraSmall"},
                {"ExtraExtraSmall", "ExtraSmall"},
                {"ExtraSmall", "ExtraSmall"},
                {"Small", "ExtraSmall"},
                {"Medium", "ExtraSmall"},
                {"Large", "ExtraSmall"},
                {"ExtraLarge", "ExtraSmall"},
                {"ExtraExtraSmall", "Small"},
                {"ExtraSmall", "Small"},
                {"ExtraExtraSmall", "Medium"},
                {"ExtraSmall", "Medium"}},
               "Small");
          rule({{"Small", "Small"},
                {"Medium", "Small"},
                {"Large", "Small"},
                {"ExtraLarge", "Small"},
                {"ExtraExtraSmall", "Large"},
                {"ExtraSmall", "Large"},
                {"Medium", "Medium"}},
               "Medium");
          rule({{"Small", "Medium"},
                {"Large", "Medium"},
                {"ExtraLarge", "Medium"},
                {"ExtraExtraSmall", "ExtraLarge"},
                {"ExtraSmall", "ExtraLarge"},
                {"ExtraExtraSmall", "ExtraExtraLarge"},
                {"ExtraSmall", "ExtraExtraLarge"}},
               "Large");
          rule({{"Medium", "Large"}, {"Large", "Large"}, {"Small", "ExtraLarge"}, {"Medium", "ExtraLarge"}},
               "ExtraLarge");
          rule({{"ExtraLarge", "Large"},
                {"Large", "ExtraLarge"},
                {"ExtraLarge", "ExtraLarge"},
                {"Small", "ExtraExtraLarge"},
                {"Medium", "ExtraExtraLarge"},
                {"Large", "ExtraExtraLarge"},
                {"ExtraLarge", "ExtraExtraLarge"}},
               "ExtraExtraLarge");
          return engine;
      }()) {}

double LongitudinalFis::acceleration(double gapError, double gapErrorRate) const {
    const auto out = engine_.evaluateStrict({{kGapError, gapError},
                                             {kGapErrorRate, gapErrorRate}},
                                            kDefuzz);
    return out.at(kAcceleration);
}

namespace {
const char* const kTimeLoss = "avg-time-loss";
const char* const kTimeLossRate = "avg-time-loss-rate";
const char* const kChangeLane = "change-lane-decision";
}  // namespace

LaneChangeFis::LaneChangeFis(const Triples& mf)
    : engine_([&mf] {
          if (mf.size() != kLatTriples) throw std::invalid_argument("lane-change FIS needs 8 triples");
          const Grid lossGrid = Grid::arange(0, 100, 1);
          const Grid rateGrid = Grid::arange(-2, 1, 0.005);
          const Grid decisionGrid = Grid::arange(-1, 2, 0.01);

          MamdaniEngine engine = makeEngine(decisionGrid);
          engine.addInput(makeVariable(kTimeLoss, lossGrid, kThreeTerms, mf, 0));
          engine.addInput(makeVariable(kTimeLossRate, rateGrid, kThreeTerms, mf, 3));
          engine.addOutput(makeVariable(kChangeLane, decisionGrid, {"NoChange", "ChangeLane"}, mf, 6));

          const auto rule = [&engine](const std::vector<TermPair>& pairs, const char* out) {
              addRule(engine, kTimeLoss, kTimeLossRate, pairs, kChangeLane, out);
          };
          rule({{"Small", "Small"}, {"Small", "Medium"}, {"Medium", "Small"}}, "NoChange");
          rule({{"Small", "Large"},
                {"Medium", "Medium"},
                {"Medium", "Large"},
                {"Large", "Small"},
                {"Large", "Medium"},
                {"Large", "Large"}},
               "ChangeLane");
          return engine;
      }()) {}

double LaneChangeFis::decision(double avgTimeLoss, double timeLossRate) const {
    const auto out = engine_.evaluateStrict({{kTimeLoss, avgTimeLoss},
                                             {kTimeLossRate, timeLossRate}},
                                            kDefuzz);
    return out.at(kChangeLane);
}

namespace {
const char* const kPlatoonGapError = "platoon-gap-error-value";
const char* const kVelocityError = "vehicle-error-value";
}  // namespace

PlatoonLeaderFis::PlatoonLeaderFis(const Triples& mf)
    : engine_([&mf] {
          if (mf.size() != kLon2Triples) throw std::invalid_argument("platoon-leader FIS needs 9 triples");
          const Grid gapGrid = Grid::arange(-10, 10, 0.01);
          const Grid velGrid = Grid::arange(-31.292, 3, 0.01);
          const Grid accelGrid = Grid::arange(-5, 5.1, 0.01);

          MamdaniEngine engine = makeEngine(accelGrid);
          engine.addInput(makeVariable(kPlatoonGapError, gapGrid, kThreeTerms, mf, 0));
          engine.addInput(makeVariable(kVelocityError, velGrid, kThreeTerms, mf, 3));
          engine.addOutput(makeVariable(kAcceleration, accelGrid, kThreeTerms, mf, 6));

          const auto rule = [&engine](const std::vector<TermPair>& pairs, const char* out) {
              addRule(engine, kPlatoonGapError, kVelocityError, pairs, kAcceleration, out);
          };
          rule({{"Small", "Small"}, {"Small", "Medium"}, {"Medium", "Small"}, {"Large", "Small"},
                {"Large", "Medium"}},
               "Small");
          rule({{"Small", "Large"}, {"Medium", "Medium"}}, "Medium");
          rule({{"Medium", "Large"}, {"Large", "Large"}}, "Large");
          return engine;
      }()) {}

double PlatoonLeaderFis::acceleration(double platoonGapError, double velocityError) const {
    const auto out =
        engine_.evaluateStrict({{kPlatoonGapError, platoonGapError},
                                {kVelocityError, velocityError}},
                                kDefuzz);
    return out.at(kAcceleration);
}

}  // namespace thesis
