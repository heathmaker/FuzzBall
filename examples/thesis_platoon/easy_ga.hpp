#pragma once

// The genetic algorithm the thesis used, as configured through the Python
// EasyGA package (v1.6) and re-implemented here operation for operation:
// rank tournament parent selection, sequential single-point crossover,
// random-avoid-best mutation, the periodic "adapt" step, and the thesis' own
// survivor-selection override (elitism, replace failed chromosomes with fresh
// random ones, keep only chromosomes that differ from the best). It is kept
// separate from fuzzylib::ga::GeneticAlgorithm on purpose: that one is a
// general real-coded maximizer, this one reproduces a specific published
// training procedure -- including its idiosyncrasies -- so the GFS can be
// retrained the way the thesis trained it.
//
// Fitness is *minimized*. A fitness of kFailedFitness marks a chromosome whose
// controller could not run (invalid membership functions, or no rule fired);
// survivor selection swaps those for fresh random chromosomes.

#include <cstddef>
#include <functional>
#include <optional>
#include <random>
#include <vector>

namespace thesis {

constexpr double kFailedFitness = 100000.0;

struct Chromosome {
    std::vector<double> genes;
    std::optional<double> fitness;  // unset until evaluated, like EasyGA's None
};

struct EasyGaConfig {
    std::size_t populationSize = 10;
    std::size_t generationGoal = 500;
    double selectionProbability = 0.5;
    double parentRatio = 0.1;  // with 10 chromosomes: 2 parents
    double tournamentSizeRatio = 0.1;
    double chromosomeMutationRate = 0.1;  // fraction of the population mutated per generation
    double geneMutationRate = 0.05;       // fraction of a chromosome's genes replaced when mutated
    double adaptRate = 0.05;              // adapt every 1/adaptRate generations
    double adaptProbabilityRate = 0.1;
    double percentConverged = 0.5;
    unsigned seed = 1;
};

class EasyGa {
public:
    using Genes = std::vector<double>;
    using Rng = std::mt19937_64;

    // randomChromosome: builds one random chromosome (the thesis' create_pop()).
    // seedChromosome: optional (may be empty or return no genes); replaces chromosome 0 of every fresh population
    //   (the thesis seeded lon1 and lon2 with their hand-picked parameters).
    // fitness: lower is better.
    // chromosomeLength: genes per chromosome; mutated genes are redrawn as
    //   random integers 1..10, which is EasyGA's default gene_impl -- the
    //   thesis never overrode it, and the integer genes in its trained
    //   chromosomes (6, 2, 8, 10, ...) come from this.
    EasyGa(EasyGaConfig config, std::size_t chromosomeLength, std::function<Genes(Rng&)> randomChromosome,
           std::function<Genes()> seedChromosome, std::function<double(const Genes&)> fitness);

    // One EasyGA evolve(1): evaluates the initial population on the first call,
    // afterwards selection -> crossover -> survivors -> mutation -> evaluation.
    void evolveOnce();

    // Throws the population away and starts over (EasyGA's initialize_population).
    void initializePopulation();

    bool active() const { return generation_ < config_.generationGoal; }
    std::size_t generation() const { return generation_; }
    const std::vector<Chromosome>& population() const { return population_; }  // best first
    const Chromosome& best() const { return population_.front(); }

private:
    void evaluateAll();
    void sortByFitness();
    void selectParents();
    void crossoverIndividual(const Chromosome& a, const Chromosome& b, double weight, std::vector<Chromosome>& out);
    double weightedRandom(double weight);
    void selectSurvivors();
    void mutatePopulation();
    void mutateChromosome(Chromosome& chromosome);
    void adapt();
    void adaptProbabilities();
    void adaptPopulation();
    std::vector<std::size_t> sample(std::size_t from, std::size_t to, std::size_t count);
    double uniform01() { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_); }

    EasyGaConfig config_;
    std::size_t chromosomeLength_;
    std::function<Genes(Rng&)> randomChromosome_;
    std::function<Genes()> seedChromosome_;
    std::function<double(const Genes&)> fitness_;
    Rng rng_;

    std::vector<Chromosome> population_;
    std::vector<Chromosome> matingPool_;
    std::vector<Chromosome> nextPopulation_;
    std::size_t generation_ = 0;

    // adapt() moves these toward bounds; the bounds are those EasyGA derived at
    // construction from its default chromosome mutation rate of 0.15.
    double selectionProbability_;
    double chromosomeMutationRate_;
    double geneMutationRate_;
    static constexpr double kMaxSelectionProbability = 0.75;
    static constexpr double kMinSelectionProbability = 0.25;
    static constexpr double kMaxChromosomeMutationRate = 0.3;
    static constexpr double kMinChromosomeMutationRate = 0.075;
    static constexpr double kMaxGeneMutationRate = 0.15;
    static constexpr double kMinGeneMutationRate = 0.01;
};

}  // namespace thesis
