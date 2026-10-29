// Re-trains one of the thesis' three genetic fuzzy systems with the same
// procedure as test_easy_ga_lon1.py / _lon2.py / _lat.py: a genetic algorithm
// tunes the membership-function triangles of one controller while the other two
// stay hand-picked, scoring each candidate by running the two-lane cruising
// scenario in SUMO and summing the platoon's absolute time-gap error.
//
//   thesis_train --stage lon1   66 genes: gap error / gap-error rate / acceleration
//   thesis_train --stage lat    24 genes: time loss / time-loss rate / change-lane
//   thesis_train --stage lon2   27 genes: platoon-leader controller after a lane change
//
// One fitness evaluation is a full 2000 s SUMO run, and the thesis' settings
// are 10 chromosomes x 500 generations, so a complete training run takes hours;
// use --generations to try it out (the trained chromosomes shipped in
// thesis_params.cpp are the thesis' own results, so training is optional).
//
// Usage: thesis_train --stage lon1|lat|lon2 [--generations N] [--population N]
//                     [--seed N] [--out DIR] [--scenario-dir DIR]
//                     [--objective gap-error|time-lost]

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>

#include "easy_ga.hpp"
#include "scenario.hpp"

namespace {

using thesis::EasyGa;
using thesis::Triples;

enum class Stage { Lon1, Lat, Lon2 };

// One block of triangle genes: `count` membership functions whose outer bounds
// are drawn uniformly from [lo1, hi1] and [lo2, hi2] (sorted), with the peak
// uniformly in between -- the thesis' create_pop() building block.
void appendTriangles(EasyGa::Genes& genes, EasyGa::Rng& rng, int count, double lo1, double hi1, double lo2, double hi2) {
    for (int i = 0; i < count; ++i) {
        double a = std::uniform_real_distribution<double>(lo1, hi1)(rng);
        double c = std::uniform_real_distribution<double>(lo2, hi2)(rng);
        if (a > c) std::swap(a, c);
        const double b = std::uniform_real_distribution<double>(a, c)(rng);
        genes.insert(genes.end(), {a, b, c});
    }
}

EasyGa::Genes randomChromosome(Stage stage, EasyGa::Rng& rng) {
    EasyGa::Genes genes;
    switch (stage) {
        case Stage::Lon1:
            appendTriangles(genes, rng, 7, -2, 3, -2, 3);
            appendTriangles(genes, rng, 8, -6, 10, -6, 10);
            appendTriangles(genes, rng, 7, -5, 5.1, -5, 5.1);
            break;
        case Stage::Lat:
            appendTriangles(genes, rng, 3, -1, 100, -1, 100);
            appendTriangles(genes, rng, 3, -0.1, 1, -0.1, 1);
            appendTriangles(genes, rng, 2, -0.1, 1.1, -0.1, 1.1);
            break;
        case Stage::Lon2:
            appendTriangles(genes, rng, 3, -10, 10, -10, 10);
            appendTriangles(genes, rng, 3, -31.292, 3, -31.292, 10);
            appendTriangles(genes, rng, 3, -5, 3, -5, 5.1);
            break;
    }
    return genes;
}

// Chromosome 0 of every fresh population: lon1 and lon2 start from a hand-picked
// system (lat starts fully random).
EasyGa::Genes seedChromosome(Stage stage) {
    switch (stage) {
        case Stage::Lon1:
            return thesis::toFlat({{-2, -1, -0.5},    {-0.6, -0.5, -0.25},     {-0.5, -0.25, 0},        {-0.25, 0, 0.25},
                                   {0, 0.5, 1},       {0.5, 1, 1.5},           {1, 1.5, 3},             {-3.33, -2.5, -1.866},
                                   {-2, -1.7866, -0.745}, {-1.7866, -0.745, -0.447}, {-3.33, -0.745, 0}, {-0.149, 0, 0.149},
                                   {0, 0.149, 0.745}, {0.149, 0.745, 1.7866},  {0.745, 1.7866, 3.33},   {-5, -4.572, -3},
                                   {-4.572, -3, -1.5}, {-2.235, -1.5, 0},      {-1.5, 0, 1.5},          {0, 1.5, 3},
                                   {1.5, 3, 4.572},   {3, 4.572, 5}});
        case Stage::Lon2:
            return thesis::toFlat(thesis::fisLon2());
        case Stage::Lat:
            return {};
    }
    return {};
}

// The controllers held fixed while one stage trains: copies of the parameter
// arrays in each trainer script (they differ slightly between the three).
thesis::FisSet trainingContext(Stage stage) {
    const Triples bigRateLon1 = {{-2, -1, -0.5},       {-0.6, -0.5, -0.25},   {-0.5, -0.25, 0},   {-0.25, 0, 0.25},
                                 {0, 0.5, 1},          {0.5, 1, 1.5},         {1, 1.5, 3},        {-10, -7.5, -5.6},
                                 {-6, -5.36, -2.235},  {-5.36, -2.235, -0.447}, {-10, -2.235, 0}, {-0.447, 0, 0.447},
                                 {0, 0.447, 2.235},    {0.447, 2.235, 5.36},  {2.235, 5.36, 10},  {-5, -4.572, -3},
                                 {-4.572, -3, -1.5},   {-2.235, -1.5, 0},     {-1.5, 0, 1.5},     {0, 1.5, 3},
                                 {1.5, 3, 4.572},      {3, 4.572, 5}};
    Triples narrowRateLon1 = bigRateLon1;
    narrowRateLon1[10] = {-0.5, -0.02, 0};
    narrowRateLon1[11] = {-0.01, 0, 0.01};
    narrowRateLon1[12] = {0, 0.02, 0.5};

    switch (stage) {
        case Stage::Lon1:
            return {{}, thesis::fisLat(),
                    {{-10, -1.5, 0}, {-1.5, 0, 1.5}, {0, 1.5, 10}, {-10, -2.235, 0}, {-0.447, 0, 0.447}, {0, 0.447, 2.235},
                     {-3, -1.5, 0}, {-1.5, 0, 1.5}, {0, 1.5, 3}}};
        case Stage::Lat:
            return {narrowRateLon1, {},
                    {{-10, -1.5, 0}, {-1.5, 0, 1.5}, {0, 1.5, 10}, {-10, -2.235, 0}, {-0.447, 0, 0.447}, {0, 0.447, 2.235},
                     {-2.235, -1.5, 0}, {-1.5, 0, 1.5}, {0, 1.5, 3}}};
        case Stage::Lon2:
            return {bigRateLon1, thesis::fisLat(), {}};
    }
    return {};
}

double bestFitness(const EasyGa& ga) { return ga.best().fitness.value_or(thesis::kFailedFitness); }

std::string joinGenes(const EasyGa::Genes& genes, const char* sep) {
    std::ostringstream out;
    out << std::setprecision(17);
    for (size_t i = 0; i < genes.size(); ++i) out << (i ? sep : "") << genes[i];
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    Stage stage = Stage::Lon1;
    thesis::EasyGaConfig config;
    std::string outDir;
    std::string objective;
    thesis::ScenarioOptions scenario;
    scenario.model = thesis::Model::Fis;
    scenario.scenarioDir = "tools/thesis_platoon";
    scenario.stopAtEnd = true;  // fitness only looks at steps before endStep

    std::string stageName = "lon1";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << arg << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--stage") stageName = next();
        else if (arg == "--generations") config.generationGoal = std::stoul(next());
        else if (arg == "--population") config.populationSize = std::stoul(next());
        else if (arg == "--seed") config.seed = static_cast<unsigned>(std::stoul(next()));
        else if (arg == "--out") outDir = next();
        else if (arg == "--scenario-dir") scenario.scenarioDir = next();
        else if (arg == "--objective") objective = next();
        else {
            std::cerr << "usage: thesis_train --stage lon1|lat|lon2 [--generations N] [--population N] [--seed N]\n"
                         "                    [--out DIR] [--scenario-dir DIR] [--objective gap-error|time-lost]\n";
            return arg == "--help" ? 0 : 2;
        }
    }
    std::size_t chromosomeLength = 0;
    if (stageName == "lon1") { stage = Stage::Lon1; chromosomeLength = 66; config.chromosomeMutationRate = 0.1; }
    else if (stageName == "lat") { stage = Stage::Lat; chromosomeLength = 24; config.chromosomeMutationRate = 0.3; }
    else if (stageName == "lon2") { stage = Stage::Lon2; chromosomeLength = 66; config.chromosomeMutationRate = 0.1; }
    else { std::cerr << "unknown stage " << stageName << "\n"; return 2; }
    // The thesis' lat trainer scores gap error like the other two (its time-lost
    // objective is commented out), though the thesis text says time lost.
    const bool timeLost = objective == "time-lost";
    if (outDir.empty()) outDir = "thesis_results/train_" + stageName;
    std::filesystem::create_directories(outDir);

    const thesis::FisSet context = trainingContext(stage);
    int evaluations = 0;
    const auto fitness = [&](const EasyGa::Genes& genes) -> double {
        thesis::FisSet fis = context;
        try {
            const Triples triples = thesis::toTriples(genes);
            switch (stage) {
                case Stage::Lon1: fis.lon1 = triples; break;
                case Stage::Lat: fis.lat = triples; break;
                case Stage::Lon2: fis.lon2 = triples; break;
            }
            const thesis::RunResult result = thesis::runScenario(scenario, fis);
            ++evaluations;
            return timeLost ? result.totalTimeLost() : result.fitness(scenario.takeoverStep, scenario.endStep);
        } catch (const std::exception& e) {
            // invalid membership functions, or no rule fired: the thesis scored these 100000
            std::cout << "  (candidate failed: " << e.what() << ")\n";
            return thesis::kFailedFitness;
        }
    };

    EasyGa ga(config, chromosomeLength, [stage](EasyGa::Rng& rng) { return randomChromosome(stage, rng); },
              [stage]() { return seedChromosome(stage); }, fitness);

    std::ofstream log(outDir + "/ga_information.csv");
    log << "generation,seconds,best_fitness,best_chromosome,population_fitness\n";
    const auto start = std::chrono::steady_clock::now();

    std::cout << "Training " << stageName << ": " << config.populationSize << " chromosomes x " << config.generationGoal
              << " generations, objective " << (timeLost ? "total time lost" : "total gap error") << "\n";
    while (ga.active()) {
        ga.evolveOnce();
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::string fitnesses;
        for (const auto& c : ga.population()) fitnesses += (fitnesses.empty() ? "" : " ") + std::to_string(c.fitness.value_or(-1));
        log << ga.generation() << "," << elapsed << "," << bestFitness(ga) << "," << joinGenes(ga.best().genes, " ") << ","
            << fitnesses << "\n";
        log.flush();
        std::cout << "generation " << std::setw(4) << ga.generation() << "  best fitness " << std::fixed << std::setprecision(4)
                  << bestFitness(ga) << std::defaultfloat << "  (" << static_cast<int>(elapsed) << " s)\n";
        // The thesis restarted the population when the best fitness was exactly 10000.
        if (bestFitness(ga) == 10000) ga.initializePopulation();
    }

    const EasyGa::Genes best = ga.best().genes;
    std::ofstream out(outDir + "/best_" + stageName + ".txt");
    out << "# best fitness " << bestFitness(ga) << "\n# thesis format (list of one-element lists)\n[";
    for (size_t i = 0; i < best.size(); ++i) out << (i ? "," : "") << "[" << best[i] << "]";
    out << "]\n# C++ triples for thesis_params.cpp\n";
    for (size_t i = 0; i < best.size(); i += 3) out << "{" << best[i] << ", " << best[i + 1] << ", " << best[i + 2] << "},\n";
    std::cout << "\nBest chromosome (fitness " << bestFitness(ga) << ") written to " << outDir << "/best_" << stageName << ".txt\n";
    return 0;
}
