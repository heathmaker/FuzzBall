#pragma once

#include <cstddef>
#include <functional>
#include <random>
#include <vector>

namespace fuzzylib::ga {

using Genome = std::vector<double>;

struct Bounds {
    double low;
    double high;
};

struct GAConfig {
    std::size_t populationSize = 60;
    std::size_t generations = 100;
    double crossoverRate = 0.8;
    double mutationRate = 0.15;          // per-gene probability of mutation
    double mutationSigmaFraction = 0.1;  // gaussian sigma as a fraction of (high-low)
    std::size_t tournamentSize = 3;
    std::size_t eliteCount = 2;
    unsigned seed = 42;
};

struct GAResult {
    Genome best;
    double bestFitness = 0.0;
    std::vector<double> bestFitnessHistory;  // best fitness seen per generation
};

// A general-purpose real-coded genetic algorithm that maximizes a
// user-supplied fitness function over a bounded search space. Because any
// fuzzylib block exposes/accepts plain parameter vectors (Sugeno rule
// coefficients, PID gains, NN weights, membership function shapes), this GA
// can tune any of them by wrapping "genome -> build block -> simulate -> score"
// as the fitness function — the mechanism used to blend GA with fuzzy/PID/NN.
class GeneticAlgorithm {
public:
    GeneticAlgorithm(GAConfig config, std::vector<Bounds> bounds,
                      std::function<double(const Genome&)> fitness);

    GAResult run();

private:
    Genome randomGenome();
    std::size_t tournamentSelect(const std::vector<double>& fitnesses);
    Genome crossover(const Genome& a, const Genome& b);
    void mutate(Genome& genome);
    void clamp(Genome& genome) const;

    GAConfig config_;
    std::vector<Bounds> bounds_;
    std::function<double(const Genome&)> fitness_;
    std::mt19937 rng_;
};

}  // namespace fuzzylib::ga
