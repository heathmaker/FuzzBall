#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "fuzzylib/core/rule.hpp"
#include "fuzzylib/core/tnorm.hpp"
#include "fuzzylib/core/variable.hpp"

namespace fuzzylib {

// Centroid integrates x*mu(x) with the trapezoid rule over the sampled curve;
// CentroidExact instead takes the exact centroid of the piecewise-linear curve
// through the samples (the formula scikit-fuzzy uses), which matters when the
// samples are few or the curve is refined at its clipping points.
enum class DefuzzMethod { Centroid, CentroidExact, Bisector, MeanOfMax, SmallestOfMax, LargestOfMax };

// Classical Mamdani fuzzy inference: fuzzify -> fire rules -> clip/scale each
// consequent set by its rule's firing strength -> aggregate across rules ->
// defuzzify the resulting output membership curve into a crisp number.
class MamdaniEngine {
public:
    explicit MamdaniEngine(TNorm tnorm = TNorm::zadeh(), Aggregator aggregator = Aggregator::max(),
                            int resolution = 200);

    MamdaniEngine& addInput(LinguisticVariable variable);
    MamdaniEngine& addOutput(LinguisticVariable variable);

    // When enabled, the output grid is refined before aggregation: for every
    // output term a rule fired, the exact points where the term's curve (linearly
    // interpolated between grid samples) crosses its firing level are merged
    // into the grid. A clipped triangle's corners then sit exactly on the grid,
    // so a coarse `resolution` no longer shaves them off. scikit-fuzzy does
    // this too; it is off by default.
    MamdaniEngine& setRefineAtCuts(bool enabled) {
        refineAtCuts_ = enabled;
        return *this;
    }

    RuleBase& rules() { return ruleBase_; }
    const RuleBase& rules() const { return ruleBase_; }

    const LinguisticVariable& input(const std::string& name) const { return inputs_.at(name); }
    const LinguisticVariable& output(const std::string& name) const { return outputs_.at(name); }

    // Runs one full inference cycle and defuzzifies every output variable.
    std::unordered_map<std::string, double> evaluate(
        const std::unordered_map<std::string, double>& crispInputs,
        DefuzzMethod method = DefuzzMethod::Centroid) const;

    // Same as evaluate(), but throws std::runtime_error when no rule fires for
    // an output variable (its aggregated curve is empty) instead of silently
    // defuzzifying to the middle of the universe. This is the behavior of
    // scikit-fuzzy's ControlSystemSimulation ("system is too sparse"), which
    // callers such as a GA fitness function rely on to flag a bad rule base.
    std::unordered_map<std::string, double> evaluateStrict(
        const std::unordered_map<std::string, double>& crispInputs,
        DefuzzMethod method = DefuzzMethod::Centroid) const;

    // Exposes the aggregated (post-implication, pre-defuzzification) output
    // membership curve for a single output variable; useful for plotting/tests.
    std::vector<std::pair<double, double>> aggregatedCurve(
        const std::string& outputVariable, const std::unordered_map<std::string, double>& crispInputs) const;

private:
    std::unordered_map<std::string, double> evaluateImpl(
        const std::unordered_map<std::string, double>& crispInputs, DefuzzMethod method,
        bool strict) const;
    double defuzzify(const std::vector<double>& xs, const std::vector<double>& ys,
                      DefuzzMethod method) const;

    std::unordered_map<std::string, LinguisticVariable> inputs_;
    std::unordered_map<std::string, LinguisticVariable> outputs_;
    RuleBase ruleBase_;
    TNorm tnorm_;
    Aggregator aggregator_;
    int resolution_;
    bool refineAtCuts_ = false;
};

}  // namespace fuzzylib
