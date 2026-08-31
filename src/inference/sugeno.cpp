#include "fuzzylib/inference/sugeno.hpp"

#include <limits>

#include "fuzzylib/core/fuzzification.hpp"

namespace fuzzylib {

SugenoEngine::SugenoEngine(TNorm tnorm) : tnorm_(tnorm) {}

SugenoEngine& SugenoEngine::addInput(LinguisticVariable variable) {
    inputs_.emplace(variable.name(), std::move(variable));
    return *this;
}

std::vector<double> SugenoEngine::firingStrengths(
    const std::unordered_map<std::string, double>& crispInputs) const {
    const FuzzificationTable table = fuzzifyAll(inputs_, crispInputs);
    std::vector<double> strengths;
    strengths.reserve(ruleBase_.rules().size());
    for (const Rule& rule : ruleBase_.rules()) {
        strengths.push_back(rule.firingStrength(table, tnorm_));
    }
    return strengths;
}

std::unordered_map<std::string, double> SugenoEngine::evaluate(
    const std::unordered_map<std::string, double>& crispInputs) const {
    const std::vector<double> firings = firingStrengths(crispInputs);

    std::unordered_map<std::string, double> numerators;
    std::unordered_map<std::string, double> denominators;

    const auto& rules = ruleBase_.rules();
    // Register every output variable named by any rule up front (at 0/0) so
    // a variable that no rule happens to fire for this cycle still reports a
    // defined (zero) crisp output instead of being silently absent.
    for (const Rule& rule : rules) {
        for (const SugenoConsequent& consequent : rule.sugenoConsequents()) {
            numerators.try_emplace(consequent.variable, 0.0);
            denominators.try_emplace(consequent.variable, 0.0);
        }
    }

    for (size_t i = 0; i < rules.size(); ++i) {
        const double firing = firings[i];
        if (firing <= 0.0) continue;
        for (const SugenoConsequent& consequent : rules[i].sugenoConsequents()) {
            const double z = consequent.evaluate(crispInputs);
            numerators[consequent.variable] += firing * z;
            denominators[consequent.variable] += firing;
        }
    }

    std::unordered_map<std::string, double> results;
    for (const auto& [name, denom] : denominators) {
        results[name] = denom > std::numeric_limits<double>::epsilon() ? numerators[name] / denom : 0.0;
    }
    return results;
}

}  // namespace fuzzylib
