#pragma once

#include <string>
#include <unordered_map>

#include "fuzzylib/core/rule.hpp"
#include "fuzzylib/core/tnorm.hpp"
#include "fuzzylib/core/variable.hpp"

namespace fuzzylib {

// Takagi-Sugeno-Kang fuzzy inference: fuzzify -> compute each rule's firing
// strength and crisp consequent value z_i = f_i(inputs) -> combine every
// output via the firing-strength-weighted average. Scales naturally to many
// rules/inputs since there is no discretized output universe to integrate.
class SugenoEngine {
public:
    explicit SugenoEngine(TNorm tnorm = TNorm::algebraic());

    SugenoEngine& addInput(LinguisticVariable variable);

    RuleBase& rules() { return ruleBase_; }
    const RuleBase& rules() const { return ruleBase_; }

    const LinguisticVariable& input(const std::string& name) const { return inputs_.at(name); }

    // Weighted-average crisp output per output variable name.
    std::unordered_map<std::string, double> evaluate(
        const std::unordered_map<std::string, double>& crispInputs) const;

    // Firing strength of every rule for the given crisp inputs, in rule order.
    std::vector<double> firingStrengths(const std::unordered_map<std::string, double>& crispInputs) const;

private:
    std::unordered_map<std::string, LinguisticVariable> inputs_;
    RuleBase ruleBase_;
    TNorm tnorm_;
};

}  // namespace fuzzylib
