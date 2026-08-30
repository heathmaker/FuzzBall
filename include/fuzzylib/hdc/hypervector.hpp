#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuzzylib::hdc {

// Bipolar hypervector: each component is -1 or +1. Bipolar (rather than
// binary 0/1) makes bind/bundle symmetric and cosine similarity meaningful.
using Hypervector = std::vector<std::int8_t>;

// A hyperdimensional-computing workspace: fixes the dimensionality and RNG
// used to generate random/orthogonal hypervectors and implements the
// algebra (bind, bundle, permute, similarity) used to build symbolic
// representations that can sit alongside fuzzy/PID/NN blocks.
class HDCSpace {
public:
    explicit HDCSpace(std::size_t dimension, unsigned seed = 7);

    std::size_t dimension() const { return dimension_; }

    Hypervector random() const;

    // Elementwise product; binds two concepts into a dissimilar third vector.
    Hypervector bind(const Hypervector& a, const Hypervector& b) const;

    // Majority-vote sum; superimposes vectors into one similar to all inputs.
    Hypervector bundle(const std::vector<Hypervector>& vectors) const;

    // Cyclic shift; used to encode sequence/position information.
    Hypervector permute(const Hypervector& v, int shift) const;

    static double cosineSimilarity(const Hypervector& a, const Hypervector& b);

private:
    std::size_t dimension_;
    mutable std::mt19937 rng_;
};

// A named associative memory of hypervectors ("cleanup memory"): stores
// prototypes and finds the closest match to a noisy/composite query vector.
class ItemMemory {
public:
    void add(const std::string& name, Hypervector vector);
    const Hypervector& get(const std::string& name) const;
    bool contains(const std::string& name) const;

    // Best-matching item name and its cosine similarity to the query.
    std::pair<std::string, double> nearest(const Hypervector& query) const;

private:
    std::unordered_map<std::string, Hypervector> items_;
};

// Encodes a continuous scalar into a hypervector using correlated "level"
// vectors: nearby scalar values map to hypervectors with high similarity,
// while far-apart values map to nearly-orthogonal vectors. Built by flipping
// a growing fraction of a base vector's bits level by level.
class ScalarEncoder {
public:
    ScalarEncoder(HDCSpace& space, double min, double max, std::size_t levels, unsigned seed = 99);

    Hypervector encode(double value) const;

private:
    HDCSpace* space_;
    double min_, max_;
    std::size_t levels_;
    std::vector<Hypervector> levelVectors_;
};

}  // namespace fuzzylib::hdc
