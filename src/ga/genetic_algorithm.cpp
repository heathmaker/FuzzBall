#include "fuzzylib/ga/genetic_algorithm.hpp"

#include <algorithm>
#include <limits>

namespace fuzzylib::ga {

GeneticAlgorithm::GeneticAlgorithm(GAConfig config, std::vector<Bounds> bounds,
                                     std::function<double(const Genome&)> fitness)
    : config_(config), bounds_(std::move(bounds)), fitness_(std::move(fitness)), rng_(config.seed) {}

Genome GeneticAlgorithm::randomGenome() {
    Genome genome(bounds_.size());
    for (std::size_t i = 0; i < bounds_.size(); ++i) {
        std::uniform_real_distribution<double> dist(bounds_[i].low, bounds_[i].high);
        genome[i] = dist(rng_);
    }
    return genome;
}

void GeneticAlgorithm::clamp(Genome& genome) const {
    for (std::size_t i = 0; i < genome.size(); ++i) {
        genome[i] = std::min(bounds_[i].high, std::max(bounds_[i].low, genome[i]));
    }
}

std::size_t GeneticAlgorithm::tournamentSelect(const std::vector<double>& fitnesses) {
    std::uniform_int_distribution<std::size_t> pick(0, fitnesses.size() - 1);
    std::size_t best = pick(rng_);
    for (std::size_t i = 1; i < config_.tournamentSize; ++i) {
        std::size_t challenger = pick(rng_);
        if (fitnesses[challenger] > fitnesses[best]) best = challenger;
    }
    return best;
}

Genome GeneticAlgorithm::crossover(const Genome& a, const Genome& b) {
    // BLX-alpha crossover: sample each gene uniformly from an extended range
    // around the parents' interval, preserving diversity better than a
    // simple midpoint average.
    constexpr double alpha = 0.5;
    Genome child(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double lo = std::min(a[i], b[i]);
        const double hi = std::max(a[i], b[i]);
        const double span = hi - lo;
        std::uniform_real_distribution<double> dist(lo - alpha * span, hi + alpha * span);
        child[i] = dist(rng_);
    }
    return child;
}

void GeneticAlgorithm::mutate(Genome& genome) {
    std::uniform_real_distribution<double> chance(0.0, 1.0);
    for (std::size_t i = 0; i < genome.size(); ++i) {
        if (chance(rng_) > config_.mutationRate) continue;
        const double sigma = config_.mutationSigmaFraction * (bounds_[i].high - bounds_[i].low);
        std::normal_distribution<double> noise(0.0, sigma);
        genome[i] += noise(rng_);
    }
}

GAResult GeneticAlgorithm::run() {
    std::vector<Genome> population(config_.populationSize);
    for (auto& g : population) g = randomGenome();

    GAResult result;
    result.bestFitness = -std::numeric_limits<double>::infinity();

    for (std::size_t gen = 0; gen < config_.generations; ++gen) {
        std::vector<double> fitnesses(population.size());
        for (std::size_t i = 0; i < population.size(); ++i) fitnesses[i] = fitness_(population[i]);

        // Track global best.
        std::size_t bestIdx = 0;
        for (std::size_t i = 1; i < fitnesses.size(); ++i) {
            if (fitnesses[i] > fitnesses[bestIdx]) bestIdx = i;
        }
        if (fitnesses[bestIdx] > result.bestFitness) {
            result.bestFitness = fitnesses[bestIdx];
            result.best = population[bestIdx];
        }
        result.bestFitnessHistory.push_back(result.bestFitness);

        // Elitism: carry the top performers of this generation forward unchanged.
        std::vector<std::size_t> order(population.size());
        for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(),
                  [&](std::size_t a, std::size_t b) { return fitnesses[a] > fitnesses[b]; });

        std::vector<Genome> nextGen;
        nextGen.reserve(population.size());
        for (std::size_t i = 0; i < config_.eliteCount && i < order.size(); ++i) {
            nextGen.push_back(population[order[i]]);
        }

        std::uniform_real_distribution<double> chance(0.0, 1.0);
        while (nextGen.size() < population.size()) {
            const Genome& parentA = population[tournamentSelect(fitnesses)];
            const Genome& parentB = population[tournamentSelect(fitnesses)];
            Genome child = (chance(rng_) < config_.crossoverRate) ? crossover(parentA, parentB) : parentA;
            mutate(child);
            clamp(child);
            nextGen.push_back(std::move(child));
        }
        population = std::move(nextGen);
    }

    return result;
}

}  // namespace fuzzylib::ga
