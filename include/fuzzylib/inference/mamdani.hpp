#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "fuzzylib/core/rule.hpp"
#include "fuzzylib/core/tnorm.hpp"
#include "fuzzylib/core/variable.hpp"

namespace fuzzylib {

enum class DefuzzMethod { Centroid, Bisector, MeanOfMax, SmallestOfMax, LargestOfMax };

// Classical Mamdani fuzzy inference: fuzzify -> fire rules -> clip/scale each
// consequent set by its rule's firing strength -> aggregate across rules ->
// defuzzify the resulting output membership curve into a crisp number.
class MamdaniEngine {
public:
    explicit MamdaniEngine(TNorm tnorm = TNorm::zadeh(), Aggregator aggregator = Aggregator::max(),
                            int resolution = 200);

    MamdaniEngine& addInput(LinguisticVariable variable);
    MamdaniEngine& addOutput(LinguisticVariable variable);

    RuleBase& rules() { return ruleBase_; }
    const RuleBase& rules() const { return ruleBase_; }

    const LinguisticVariable& input(const std::string& name) const { return inputs_.at(name); }
    const LinguisticVariable& output(const std::string& name) const { return outputs_.at(name); }

    // Runs one full inference cycle and defuzzifies every output variable.
    std::unordered_map<std::string, double> evaluate(
        const std::unordered_map<std::string, double>& crispInputs,
        DefuzzMethod method = DefuzzMethod::Centroid) const;

    // Exposes the aggregated (post-implication, pre-defuzzification) output
    // membership curve for a single output variable; useful for plotting/tests.
    std::vector<std::pair<double, double>> aggregatedCurve(
        const std::string& outputVariable, const std::unordered_map<std::string, double>& crispInputs) const;

private:
    double defuzzify(const std::vector<double>& xs, const std::vector<double>& ys,
                      DefuzzMethod method) const;

    std::unordered_map<std::string, LinguisticVariable> inputs_;
    std::unordered_map<std::string, LinguisticVariable> outputs_;
    RuleBase ruleBase_;
    TNorm tnorm_;
    Aggregator aggregator_;
    int resolution_;
};

}  // namespace fuzzylib
