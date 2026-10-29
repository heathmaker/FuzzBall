#pragma once

// The three fuzzy inference systems of the Master's thesis "Optimizing
// Platoon Time Gap Following on the Highway using Genetic Fuzzy Systems"
// (Palmer, Sathyan, Cohen), ported from scikit-fuzzy to fuzzylib.
//
//   LongitudinalFis   gap error + gap-error rate        -> acceleration  (7 rules)
//   LaneChangeFis     avg time loss + time-loss rate    -> change-lane   (2 rules)
//   PlatoonLeaderFis  platoon gap error + velocity err. -> acceleration  (3 rules)
//
// Each is a Mamdani system built on fuzzylib::MamdaniEngine whose membership
// functions are the thesis' triangles, one {a, b, c} triple per term. The
// hand-picked FIS is the default parameter set; the GFS is the same rule base
// with genetically tuned triples.

#include <array>
#include <vector>

#include "fuzzylib/inference/mamdani.hpp"

namespace thesis {

using Triple = std::array<double, 3>;
using Triples = std::vector<Triple>;

// Number of {a, b, c} triples (membership functions) each controller takes.
constexpr std::size_t kLon1Triples = 22;  // 7 gap error + 8 gap-error rate + 7 acceleration
constexpr std::size_t kLatTriples = 8;    // 3 time loss + 3 time-loss rate + 2 decision
constexpr std::size_t kLon2Triples = 9;   // 3 gap error + 3 velocity error + 3 acceleration

// Hand-picked FIS parameters, as used for the "FIS" driving model.
Triples fisLon1();
Triples fisLat();
Triples fisLon2();

// Genetically tuned parameters, as used for the "GFS" driving model.
Triples gfsLon1();
Triples gfsLat();
Triples gfsLon2();

// Groups a flat GA chromosome (what the genetic algorithm evolves) into
// triples (what the controllers take), as parseGAChrmosome did in Python.
Triples toTriples(const std::vector<double>& flat);
std::vector<double> toFlat(const Triples& triples);

// Inputs outside a variable's universe of discourse saturate at its edge
// (membership is read from the nearest sample), exactly like scikit-fuzzy's
// interpolated membership lookup, rather than dropping to zero membership.
class LongitudinalFis {
public:
    explicit LongitudinalFis(const Triples& mf);
    // Throws std::runtime_error if no rule fires (skfuzzy raises the same).
    double acceleration(double gapError, double gapErrorRate) const;

private:
    fuzzylib::MamdaniEngine engine_;
};

class LaneChangeFis {
public:
    explicit LaneChangeFis(const Triples& mf);
    // Raw defuzzified output; the thesis rounds it to 0 (stay) / 1 (change).
    double decision(double avgTimeLoss, double timeLossRate) const;

private:
    fuzzylib::MamdaniEngine engine_;
};

class PlatoonLeaderFis {
public:
    explicit PlatoonLeaderFis(const Triples& mf);
    double acceleration(double platoonGapError, double velocityError) const;

private:
    fuzzylib::MamdaniEngine engine_;
};

}  // namespace thesis
