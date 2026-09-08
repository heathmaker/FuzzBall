#include "fuzzylib/inference/mamdani.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "fuzzylib/core/fuzzification.hpp"

namespace fuzzylib {

MamdaniEngine::MamdaniEngine(TNorm tnorm, Aggregator aggregator, int resolution)
    : tnorm_(tnorm), aggregator_(aggregator), resolution_(resolution) {}

MamdaniEngine& MamdaniEngine::addInput(LinguisticVariable variable) {
    inputs_.emplace(variable.name(), std::move(variable));
    return *this;
}

MamdaniEngine& MamdaniEngine::addOutput(LinguisticVariable variable) {
    outputs_.emplace(variable.name(), std::move(variable));
    return *this;
}

std::vector<std::pair<double, double>> MamdaniEngine::aggregatedCurve(
    const std::string& outputVariable, const std::unordered_map<std::string, double>& crispInputs) const {
    const LinguisticVariable& outVar = outputs_.at(outputVariable);
    const FuzzificationTable table = fuzzifyAll(inputs_, crispInputs);

    std::vector<double> xs(resolution_);
    std::vector<double> ys(resolution_, 0.0);
    const double span = outVar.max() - outVar.min();
    for (int i = 0; i < resolution_; ++i) {
        xs[i] = outVar.min() + span * static_cast<double>(i) / static_cast<double>(resolution_ - 1);
    }

    for (const Rule& rule : ruleBase_.rules()) {
        const double firing = rule.firingStrength(table, tnorm_);
        if (firing <= 0.0) continue;
        for (const MamdaniConsequent& consequent : rule.mamdaniConsequents()) {
            if (consequent.variable != outputVariable) continue;
            const Term& term = outVar.term(consequent.term);
            const double weightedFiring = firing * consequent.weight;
            for (int i = 0; i < resolution_; ++i) {
                const double clipped = tnorm_.implication(weightedFiring, term.membership(xs[i]));
                ys[i] = aggregator_.combine(ys[i], clipped);
            }
        }
    }

    std::vector<std::pair<double, double>> curve;
    curve.reserve(resolution_);
    for (int i = 0; i < resolution_; ++i) curve.emplace_back(xs[i], ys[i]);
    return curve;
}

std::unordered_map<std::string, double> MamdaniEngine::evaluate(
    const std::unordered_map<std::string, double>& crispInputs, DefuzzMethod method) const {
    std::unordered_map<std::string, double> results;
    for (const auto& [name, outVar] : outputs_) {
        const auto curve = aggregatedCurve(name, crispInputs);
        std::vector<double> xs;
        std::vector<double> ys;
        xs.reserve(curve.size());
        ys.reserve(curve.size());
        for (const auto& [x, y] : curve) {
            xs.push_back(x);
            ys.push_back(y);
        }
        results[name] = defuzzify(xs, ys, method);
    }
    return results;
}

namespace {
double trapz(const std::vector<double>& xs, const std::vector<double>& ys) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        total += 0.5 * (ys[i] + ys[i + 1]) * (xs[i + 1] - xs[i]);
    }
    return total;
}
}  // namespace

double MamdaniEngine::defuzzify(const std::vector<double>& xs, const std::vector<double>& ys,
                                 DefuzzMethod method) const {
    if (xs.empty()) return 0.0;
    const double totalArea = trapz(xs, ys);
    const double fallback = (xs.front() + xs.back()) / 2.0;

    switch (method) {
        case DefuzzMethod::Centroid: {
            if (totalArea <= std::numeric_limits<double>::epsilon()) return fallback;
            std::vector<double> moment(xs.size());
            for (size_t i = 0; i < xs.size(); ++i) moment[i] = xs[i] * ys[i];
            return trapz(xs, moment) / totalArea;
        }
        case DefuzzMethod::Bisector: {
            if (totalArea <= std::numeric_limits<double>::epsilon()) return fallback;
            const double half = totalArea / 2.0;
            double cumulative = 0.0;
            for (size_t i = 0; i + 1 < xs.size(); ++i) {
                const double segment = 0.5 * (ys[i] + ys[i + 1]) * (xs[i + 1] - xs[i]);
                if (cumulative + segment >= half) {
                    const double remaining = half - cumulative;
                    const double t = segment > 0.0 ? remaining / segment : 0.0;
                    return xs[i] + t * (xs[i + 1] - xs[i]);
                }
                cumulative += segment;
            }
            return xs.back();
        }
        case DefuzzMethod::MeanOfMax:
        case DefuzzMethod::SmallestOfMax:
        case DefuzzMethod::LargestOfMax: {
            const double peak = *std::max_element(ys.begin(), ys.end());
            if (peak <= std::numeric_limits<double>::epsilon()) return fallback;
            const double tolerance = 1e-9;
            double sum = 0.0;
            int count = 0;
            double smallest = std::numeric_limits<double>::infinity();
            double largest = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < xs.size(); ++i) {
                if (ys[i] >= peak - tolerance) {
                    sum += xs[i];
                    ++count;
                    smallest = std::min(smallest, xs[i]);
                    largest = std::max(largest, xs[i]);
                }
            }
            if (method == DefuzzMethod::SmallestOfMax) return smallest;
            if (method == DefuzzMethod::LargestOfMax) return largest;
            return count > 0 ? sum / count : fallback;
        }
    }
    return fallback;
}

}  // namespace fuzzylib
