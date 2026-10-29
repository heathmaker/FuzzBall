#include "easy_ga.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace thesis {

EasyGa::EasyGa(EasyGaConfig config, std::size_t chromosomeLength, std::function<Genes(Rng&)> randomChromosome,
               std::function<Genes()> seedChromosome, std::function<double(const Genes&)> fitness)
    : config_(config),
      chromosomeLength_(chromosomeLength),
      randomChromosome_(std::move(randomChromosome)),
      seedChromosome_(std::move(seedChromosome)),
      fitness_(std::move(fitness)),
      rng_(config.seed),
      selectionProbability_(config.selectionProbability),
      chromosomeMutationRate_(config.chromosomeMutationRate),
      geneMutationRate_(config.geneMutationRate) {}

void EasyGa::initializePopulation() {
    population_.clear();
    for (std::size_t i = 0; i < config_.populationSize; ++i) population_.push_back({randomChromosome_(rng_), std::nullopt});
    if (seedChromosome_ && !population_.empty()) {
        Genes seed = seedChromosome_();
        if (!seed.empty()) population_[0] = {std::move(seed), std::nullopt};
    }
    matingPool_.clear();
    nextPopulation_.clear();
}

void EasyGa::evaluateAll() {
    for (Chromosome& c : population_) {
        if (!c.fitness) c.fitness = fitness_(c.genes);
    }
}

void EasyGa::sortByFitness() {
    // minimizing; an unevaluated chromosome sorts last
    std::stable_sort(population_.begin(), population_.end(), [](const Chromosome& a, const Chromosome& b) {
        const double fa = a.fitness.value_or(std::numeric_limits<double>::infinity());
        const double fb = b.fitness.value_or(std::numeric_limits<double>::infinity());
        return fa < fb;
    });
}

// random.sample(range(from, to), count): `count` distinct indices.
std::vector<std::size_t> EasyGa::sample(std::size_t from, std::size_t to, std::size_t count) {
    std::vector<std::size_t> pool(to > from ? to - from : 0);
    std::iota(pool.begin(), pool.end(), from);
    count = std::min(count, pool.size());
    for (std::size_t i = 0; i < count; ++i) {
        std::uniform_int_distribution<std::size_t> pick(i, pool.size() - 1);
        std::swap(pool[i], pool[pick(rng_)]);
    }
    pool.resize(count);
    return pool;
}

// EasyGA's Parent.Rank.tournament: draw tournament groups of at least 5
// chromosomes; the better ranked a chromosome is in its group, the likelier it
// wins (p, p(1-p), p(1-p)^2, ...), falling back to a random group member.
void EasyGa::selectParents() {
    sortByFitness();
    const auto size = population_.size();
    const auto parentAmount = std::max<std::size_t>(2, static_cast<std::size_t>(std::nearbyint(static_cast<double>(size) * config_.parentRatio)));
    auto tournamentSize = static_cast<std::size_t>(static_cast<double>(size) * config_.tournamentSizeRatio);
    if (tournamentSize < 5) tournamentSize = std::min<std::size_t>(5, size);

    while (matingPool_.size() < parentAmount) {
        std::vector<std::size_t> group = sample(0, size, tournamentSize);
        std::sort(group.begin(), group.end());
        std::size_t winner = tournamentSize;
        for (std::size_t rank = 0; rank < tournamentSize; ++rank) {
            if (uniform01() < selectionProbability_ * std::pow(1 - selectionProbability_, static_cast<double>(rank))) {
                winner = rank;
                break;
            }
        }
        if (winner == tournamentSize) winner = std::uniform_int_distribution<std::size_t>(0, tournamentSize - 1)(rng_);
        matingPool_.push_back(population_[group[winner]]);
    }
}

// EasyGA's simple_linear: a random number in [0, 1) skewed toward `weight`'s side.
double EasyGa::weightedRandom(double weight) {
    const double r = uniform01();
    return r < weight ? r * (1 - weight) / weight : 1 - (1 - r) * weight / (1 - weight);
}

void EasyGa::crossoverIndividual(const Chromosome& a, const Chromosome& b, double weight, std::vector<Chromosome>& out) {
    // EasyGA's _check_weight rejects weights outside (0, 1); adapt() relies on that.
    if (!(0 < weight && weight < 1)) throw std::invalid_argument("weight must be between 0 and 1");
    const std::size_t minLength = std::min(a.genes.size(), b.genes.size());
    const auto swapIndex = static_cast<std::size_t>(weightedRandom(weight) * static_cast<double>(minLength));
    Genes first(a.genes.begin(), a.genes.begin() + static_cast<std::ptrdiff_t>(swapIndex));
    first.insert(first.end(), b.genes.begin() + static_cast<std::ptrdiff_t>(swapIndex), b.genes.end());
    Genes second(b.genes.begin(), b.genes.begin() + static_cast<std::ptrdiff_t>(swapIndex));
    second.insert(second.end(), a.genes.begin() + static_cast<std::ptrdiff_t>(swapIndex), a.genes.end());
    out.push_back({std::move(first), std::nullopt});
    out.push_back({std::move(second), std::nullopt});
}

// The thesis' survivor_selection_impl (identical in all three trainers),
// including its quirk: a failed chromosome is replaced by a fresh random one
// and then, because there is no `continue`, is also kept if it differs from the best.
void EasyGa::selectSurvivors() {
    auto needed = static_cast<std::ptrdiff_t>(population_.size()) - static_cast<std::ptrdiff_t>(nextPopulation_.size());
    const Chromosome best = population_.front();
    if (needed > 0) {
        nextPopulation_.push_back(best);
        if (--needed <= 0) return;
    }
    for (const Chromosome& chromosome : population_) {
        if (chromosome.fitness && *chromosome.fitness == kFailedFitness) {
            nextPopulation_.push_back({randomChromosome_(rng_), std::nullopt});
            if (--needed <= 0) break;
        }
        const std::size_t common = std::min(best.genes.size(), chromosome.genes.size());
        const bool differsFromBest = !std::equal(best.genes.begin(), best.genes.begin() + static_cast<std::ptrdiff_t>(common), chromosome.genes.begin());
        if (differsFromBest) {
            nextPopulation_.push_back(chromosome);
            if (--needed <= 0) break;
        }
    }
}

// EasyGA's Mutation.Individual.individual_genes: redraw ceil(length * geneRate)
// genes, each as a random integer 1..10.
void EasyGa::mutateChromosome(Chromosome& chromosome) {
    chromosome.fitness.reset();
    const auto count = static_cast<std::size_t>(std::ceil(static_cast<double>(chromosome.genes.size()) * geneMutationRate_));
    for (std::size_t index : sample(0, chromosome.genes.size(), count)) {
        if (index < chromosomeLength_) chromosome.genes[index] = static_cast<double>(std::uniform_int_distribution<int>(1, 10)(rng_));
    }
}

// Mutation.Population.random_avoid_best: mutate ceil(rate * size) chromosomes,
// never the best-ranked ones.
void EasyGa::mutatePopulation() {
    const auto size = population_.size();
    const auto firstEligible = static_cast<std::size_t>(std::ceil(config_.percentConverged * static_cast<double>(size) * 3.0 / 16.0));
    const auto count = static_cast<std::size_t>(std::ceil(chromosomeMutationRate_ * static_cast<double>(size)));
    for (std::size_t index : sample(firstEligible, size, count)) mutateChromosome(population_[index]);
}

// Every 1/adaptRate generations: nudge the selection / mutation rates toward
// "explore" or "exploit" depending on how converged the population is, then
// pull the worse chromosomes toward the best with a gentle crossover.
void EasyGa::adaptProbabilities() {
    const double weight = config_.adaptProbabilityRate;
    if (weight <= 0) return;
    const auto convergedCount = static_cast<std::size_t>(std::nearbyint(config_.percentConverged * static_cast<double>(population_.size())));
    const auto distance = [&](std::size_t i) {
        return std::sqrt(std::fabs(*population_.front().fitness - *population_.at(i).fitness));
    };
    double selection = kMinSelectionProbability;
    double chromosomeRate = kMaxChromosomeMutationRate;
    double geneRate = kMaxGeneMutationRate;
    if (distance(convergedCount / 2) > distance(convergedCount / 4) * 2) {  // too few converged
        selection = kMaxSelectionProbability;
        chromosomeRate = kMinChromosomeMutationRate;
        geneRate = kMinGeneMutationRate;
    }
    const auto average = [weight](double toward, double current) { return weight * toward + (1 - weight) * current; };
    selectionProbability_ = average(selection, selectionProbability_);
    chromosomeMutationRate_ = average(chromosomeRate, chromosomeMutationRate_);
    geneMutationRate_ = average(geneRate, geneMutationRate_);
}

void EasyGa::adaptPopulation() {
    selectParents();
    for (std::size_t n = 0; n < matingPool_.size(); ++n) {
        if (population_[n].genes != population_.front().genes) {
            // EasyGA first tries a negative weight (push away); _check_weight rejects it
            // with a ValueError and the fallback is the positive weight below.
            crossoverIndividual(population_[n], matingPool_[n], 0.25, nextPopulation_);
        }
        if (nextPopulation_.size() >= population_.size()) break;
    }
    const std::size_t replace = std::min(population_.size() - 1, nextPopulation_.size());
    for (std::size_t i = 0; i < replace; ++i) population_[population_.size() - replace + i] = nextPopulation_[i];
    nextPopulation_.clear();
    matingPool_.clear();
}

void EasyGa::adapt() {
    adaptProbabilities();
    adaptPopulation();
    evaluateAll();
    sortByFitness();
}

void EasyGa::evolveOnce() {
    if (population_.empty()) initializePopulation();

    if (generation_ != 0) {
        selectParents();
        // Population.sequential: every parent crosses with the one before it
        for (std::size_t i = 0; i < matingPool_.size(); ++i) {
            const Chromosome& partner = matingPool_[(i + matingPool_.size() - 1) % matingPool_.size()];
            crossoverIndividual(matingPool_[i], partner, 0.5, nextPopulation_);
        }
        selectSurvivors();
        population_ = std::move(nextPopulation_);
        nextPopulation_.clear();
        matingPool_.clear();
        sortByFitness();
        mutatePopulation();
    }
    evaluateAll();
    sortByFitness();

    const double adaptCounter = config_.adaptRate * static_cast<double>(generation_);
    if (static_cast<long>(adaptCounter) < static_cast<long>(adaptCounter + config_.adaptRate)) adapt();
    ++generation_;
}

}  // namespace thesis
