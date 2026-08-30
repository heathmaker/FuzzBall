#include "fuzzylib/ga/genetic_algorithm.hpp"

#include <cmath>

#include "minitest.hpp"

using namespace fuzzylib::ga;

TEST_CASE(ga_finds_maximum_of_simple_quadratic_bowl) {
    // Maximize -(x-3)^2 - (y+2)^2, whose optimum is (3, -2) with fitness 0.
    auto fitness = [](const Genome& g) {
        const double dx = g[0] - 3.0;
        const double dy = g[1] + 2.0;
        return -(dx * dx) - (dy * dy);
    };

    GAConfig config;
    config.populationSize = 80;
    config.generations = 150;
    config.seed = 7;

    GeneticAlgorithm ga(config, {Bounds{-10.0, 10.0}, Bounds{-10.0, 10.0}}, fitness);
    GAResult result = ga.run();

    CHECK_NEAR(result.best[0], 3.0, 0.2);
    CHECK_NEAR(result.best[1], -2.0, 0.2);
    CHECK(result.bestFitness > -0.05);
}

TEST_CASE(ga_fitness_history_is_monotonically_nondecreasing) {
    auto fitness = [](const Genome& g) { return -std::abs(g[0]); };
    GAConfig config;
    config.populationSize = 20;
    config.generations = 30;
    GeneticAlgorithm ga(config, {Bounds{-5.0, 5.0}}, fitness);
    GAResult result = ga.run();

    for (std::size_t i = 1; i < result.bestFitnessHistory.size(); ++i) {
        CHECK(result.bestFitnessHistory[i] >= result.bestFitnessHistory[i - 1] - 1e-12);
    }
}

int main() { return minitest::run_all_tests(); }
