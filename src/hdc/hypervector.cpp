#include "fuzzylib/hdc/hypervector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace fuzzylib::hdc {

HDCSpace::HDCSpace(std::size_t dimension, unsigned seed) : dimension_(dimension), rng_(seed) {}

Hypervector HDCSpace::random() const {
    Hypervector v(dimension_);
    std::uniform_int_distribution<int> coin(0, 1);
    for (auto& bit : v) bit = coin(rng_) == 0 ? -1 : 1;
    return v;
}

Hypervector HDCSpace::bind(const Hypervector& a, const Hypervector& b) const {
    if (a.size() != b.size()) throw std::invalid_argument("bind: dimension mismatch");
    Hypervector out(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) out[i] = static_cast<std::int8_t>(a[i] * b[i]);
    return out;
}

Hypervector HDCSpace::bundle(const std::vector<Hypervector>& vectors) const {
    if (vectors.empty()) throw std::invalid_argument("bundle: no vectors given");
    Hypervector out(dimension_, 0);
    std::vector<int> sums(dimension_, 0);
    for (const auto& v : vectors) {
        if (v.size() != dimension_) throw std::invalid_argument("bundle: dimension mismatch");
        for (std::size_t i = 0; i < dimension_; ++i) sums[i] += v[i];
    }
    std::uniform_int_distribution<int> coin(0, 1);
    for (std::size_t i = 0; i < dimension_; ++i) {
        if (sums[i] > 0) {
            out[i] = 1;
        } else if (sums[i] < 0) {
            out[i] = -1;
        } else {
            out[i] = coin(rng_) == 0 ? -1 : 1;  // break ties randomly
        }
    }
    return out;
}

Hypervector HDCSpace::permute(const Hypervector& v, int shift) const {
    const std::size_t n = v.size();
    Hypervector out(n);
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t src = ((static_cast<long long>(i) - shift) % static_cast<long long>(n) +
                            static_cast<long long>(n)) %
                           static_cast<long long>(n);
        out[i] = v[src];
    }
    return out;
}

double HDCSpace::cosineSimilarity(const Hypervector& a, const Hypervector& b) {
    if (a.size() != b.size()) throw std::invalid_argument("cosineSimilarity: dimension mismatch");
    double dot = 0.0, na = 0.0, nb = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        dot += static_cast<double>(a[i]) * b[i];
        na += static_cast<double>(a[i]) * a[i];
        nb += static_cast<double>(b[i]) * b[i];
    }
    const double denom = std::sqrt(na) * std::sqrt(nb);
    return denom > std::numeric_limits<double>::epsilon() ? dot / denom : 0.0;
}

void ItemMemory::add(const std::string& name, Hypervector vector) { items_[name] = std::move(vector); }

const Hypervector& ItemMemory::get(const std::string& name) const { return items_.at(name); }

bool ItemMemory::contains(const std::string& name) const { return items_.find(name) != items_.end(); }

std::pair<std::string, double> ItemMemory::nearest(const Hypervector& query) const {
    std::string bestName;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (const auto& [name, vec] : items_) {
        const double score = HDCSpace::cosineSimilarity(vec, query);
        if (score > bestScore) {
            bestScore = score;
            bestName = name;
        }
    }
    return {bestName, bestScore};
}

ScalarEncoder::ScalarEncoder(HDCSpace& space, double min, double max, std::size_t levels, unsigned seed)
    : space_(&space), min_(min), max_(max), levels_(levels) {
    if (levels_ < 2) throw std::invalid_argument("ScalarEncoder requires at least 2 levels");

    // Build correlated level vectors: start from a random base vector, then
    // progressively flip a growing, fixed random subset of its bits so that
    // level i and level j share (1 - |i-j|/levels) fraction of their bits.
    std::mt19937 rng(seed);
    std::vector<std::size_t> flipOrder(space_->dimension());
    std::iota(flipOrder.begin(), flipOrder.end(), 0);
    std::shuffle(flipOrder.begin(), flipOrder.end(), rng);

    Hypervector base = space_->random();
    levelVectors_.reserve(levels_);
    levelVectors_.push_back(base);

    const std::size_t dim = space_->dimension();
    for (std::size_t level = 1; level < levels_; ++level) {
        Hypervector v = levelVectors_.back();
        const std::size_t flipStart = (level - 1) * dim / levels_;
        const std::size_t flipEnd = level * dim / levels_;
        for (std::size_t i = flipStart; i < flipEnd; ++i) {
            v[flipOrder[i]] = static_cast<std::int8_t>(-v[flipOrder[i]]);
        }
        levelVectors_.push_back(std::move(v));
    }
}

Hypervector ScalarEncoder::encode(double value) const {
    double clamped = std::min(max_, std::max(min_, value));
    const double t = (clamped - min_) / (max_ - min_);
    std::size_t idx = static_cast<std::size_t>(std::lround(t * static_cast<double>(levels_ - 1)));
    idx = std::min(idx, levels_ - 1);
    return levelVectors_[idx];
}

}  // namespace fuzzylib::hdc
