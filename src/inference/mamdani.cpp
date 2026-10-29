#include "fuzzylib/inference/mamdani.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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

namespace {
// Points where `term`, linearly interpolated between the samples at xs, crosses
// `level` -- the same construction as scikit-fuzzy's interp_universe: one point
// per grid interval whose end samples straddle the level.
void appendCutPoints(const Term& term, double level, const std::vector<double>& xs, std::vector<double>& out) {
    std::vector<double> mu(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) mu[i] = term.membership(xs[i]);
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        const bool lowSide = mu[i] >= level;
        const bool highSide = mu[i + 1] >= level;
        if (lowSide != highSide) {
            out.push_back(xs[i] + (level - mu[i]) * (xs[i + 1] - xs[i]) / (mu[i + 1] - mu[i]));
        }
    }
}
}  // namespace

std::vector<std::pair<double, double>> MamdaniEngine::aggregatedCurve(
    const std::string& outputVariable, const std::unordered_map<std::string, double>& crispInputs) const {
    const LinguisticVariable& outVar = outputs_.at(outputVariable);
    const FuzzificationTable table = fuzzifyAll(inputs_, crispInputs);

    std::vector<double> xs(resolution_);
    const double span = outVar.max() - outVar.min();
    for (int i = 0; i < resolution_; ++i) {
        xs[i] = outVar.min() + span * static_cast<double>(i) / static_cast<double>(resolution_ - 1);
    }

    // Rules that fired for this output, and the strongest firing of each term.
    struct Fired {
        const Term* term;
        double weightedFiring;
    };
    std::vector<Fired> fired;
    std::unordered_map<const Term*, double> strongest;
    for (const Rule& rule : ruleBase_.rules()) {
        const double firing = rule.firingStrength(table, tnorm_);
        if (firing <= 0.0) continue;
        for (const MamdaniConsequent& consequent : rule.mamdaniConsequents()) {
            if (consequent.variable != outputVariable) continue;
            const Term* term = &outVar.term(consequent.term);
            const double weightedFiring = firing * consequent.weight;
            fired.push_back({term, weightedFiring});
            double& best = strongest[term];
            best = std::max(best, weightedFiring);
        }
    }

    if (refineAtCuts_ && !fired.empty()) {
        std::vector<double> extra;
        for (const auto& [term, level] : strongest) appendCutPoints(*term, level, xs, extra);
        xs.insert(xs.end(), extra.begin(), extra.end());
        std::sort(xs.begin(), xs.end());
        xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    }

    std::vector<double> ys(xs.size(), 0.0);
    for (const Fired& f : fired) {
        for (size_t i = 0; i < xs.size(); ++i) {
            ys[i] = aggregator_.combine(ys[i], tnorm_.implication(f.weightedFiring, f.term->membership(xs[i])));
        }
    }

    std::vector<std::pair<double, double>> curve;
    curve.reserve(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) curve.emplace_back(xs[i], ys[i]);
    return curve;
}

std::unordered_map<std::string, double> MamdaniEngine::evaluate(
    const std::unordered_map<std::string, double>& crispInputs, DefuzzMethod method) const {
    return evaluateImpl(crispInputs, method, false);
}

std::unordered_map<std::string, double> MamdaniEngine::evaluateStrict(
    const std::unordered_map<std::string, double>& crispInputs, DefuzzMethod method) const {
    return evaluateImpl(crispInputs, method, true);
}

std::unordered_map<std::string, double> MamdaniEngine::evaluateImpl(
    const std::unordered_map<std::string, double>& crispInputs, DefuzzMethod method,
    bool strict) const {
    std::unordered_map<std::string, double> results;
    for (const auto& [name, outVar] : outputs_) {
        const auto curve = aggregatedCurve(name, crispInputs);
        if (strict &&
            std::none_of(curve.begin(), curve.end(), [](const auto& p) { return p.second > 0.0; })) {
            throw std::runtime_error("No rule fired for output '" + name +
                                     "'; the rule base is too sparse for these inputs");
        }
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
        case DefuzzMethod::CentroidExact: {
            // Exact centroid of the piecewise-linear curve through the samples:
            // each segment is a rectangle or a trapezoid with a closed-form
            // area and centroid.
            double sumMomentArea = 0.0;
            double sumArea = 0.0;
            for (size_t i = 1; i < xs.size(); ++i) {
                const double x1 = xs[i - 1];
                const double x2 = xs[i];
                const double y1 = ys[i - 1];
                const double y2 = ys[i];
                if ((y1 == 0.0 && y2 == 0.0) || x1 == x2) continue;
                double moment = 0.0;
                double area = 0.0;
                if (y1 == y2) {
                    moment = 0.5 * (x1 + x2);
                    area = (x2 - x1) * y1;
                } else if (y1 == 0.0) {
                    moment = 2.0 / 3.0 * (x2 - x1) + x1;
                    area = 0.5 * (x2 - x1) * y2;
                } else if (y2 == 0.0) {
                    moment = 1.0 / 3.0 * (x2 - x1) + x1;
                    area = 0.5 * (x2 - x1) * y1;
                } else {
                    moment = (2.0 / 3.0 * (x2 - x1) * (y2 + 0.5 * y1)) / (y1 + y2) + x1;
                    area = 0.5 * (x2 - x1) * (y1 + y2);
                }
                sumMomentArea += moment * area;
                sumArea += area;
            }
            if (sumArea <= std::numeric_limits<double>::epsilon()) return fallback;
            return sumMomentArea / sumArea;
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
