// The thesis' three fuzzy systems, checked against values produced by the
// original scikit-fuzzy implementation (skfuzzy 0.5.0, running the thesis'
// own test_fuzzy_controller_live.py), plus the EasyGA-equivalent genetic
// algorithm's invariants.

#include <cmath>
#include <limits>

#include "easy_ga.hpp"
#include "minitest.hpp"
#include "thesis_fis.hpp"

using namespace thesis;

namespace {
struct Golden {
    const char* system;
    double a;
    double b;
    double expected;
};

void checkAgainstSkfuzzy(const Triples& lon1, const Triples& lat, const Triples& lon2, const std::vector<Golden>& golden) {
    const LongitudinalFis l1(lon1);
    const LaneChangeFis lc(lat);
    const PlatoonLeaderFis l2(lon2);
    for (const Golden& g : golden) {
        const std::string system = g.system;
        double got = 0.0;
        if (system == "lon1") {
            got = l1.acceleration(g.a, g.b);
        } else if (system == "lat") {
            got = lc.decision(g.a, g.b);
        } else {
            got = l2.acceleration(g.a, g.b);
        }
        CHECK_NEAR(got, g.expected, 1e-9);
    }
}

double bestFitness(const EasyGa& ga) { return ga.best().fitness.value_or(std::numeric_limits<double>::infinity()); }
}  // namespace

TEST_CASE(hand_picked_fis_matches_skfuzzy) {
    checkAgainstSkfuzzy(fisLon1(), fisLat(), fisLon2(),
                        {{"lon1", -0.55700341100102579, -2.7932066086039846, -2.2353817840599999},
                         {"lon1", -0.30586649852448655, -3.5360086018023456, -2.1212928847099999},
                         {"lon1", 1.2837554943904226, 0.13279504713665169, 3.1842321168400001},
                         {"lat", 41.265124431359055, 0.94842981489667433, 0.79595736887599999},
                         {"lat", 90.407929222240767, 0.65396542332752206, 0.809430961171},
                         {"lat", 49.634190675423575, 0.22132827023284812, 0.85775795572400004},
                         {"lon2", -4.8534752269251156, -5.6557973151387628, -1.5},
                         {"lon2", 11.431108233836497, -23.37374497927917, -1.5}});
}

TEST_CASE(tuned_gfs_matches_skfuzzy) {
    checkAgainstSkfuzzy(gfsLon1(), gfsLat(), gfsLon2(),
                        {{"lon1", -0.55700341100102579, -2.7932066086039846, 1.0732153494600001},
                         {"lon1", -1.2118109149860297, -0.66248442609732539, 1.37782820899},
                         {"lat", 90.407929222240767, 0.65396542332752206, 0.90066969383999995},
                         {"lat", 4.5475801038408328, 0.81133379727439126, 0.90027061060799995},
                         {"lon2", -4.8534752269251156, -5.6557973151387628, 3.1482476132400001},
                         {"lon2", 1.3757208589689682, -18.408392224978858, 3.0519692960000002}});
}

TEST_CASE(fis_throws_when_no_rule_fires_like_skfuzzy) {
    // a gap error of -2.2 saturates to the universe edge, where no gap-error term is active
    const LongitudinalFis lon1(fisLon1());
    CHECK_THROWS(lon1.acceleration(-2.210281458239126, 0.5140388671860934));
    // average time loss 10 with rate -0.1: every "small/medium/large" term is zero on the rate axis edge
    const LaneChangeFis lat(fisLat());
    CHECK_THROWS(lat.decision(123.61994544818799, -0.10109239067846088));
}

TEST_CASE(unordered_membership_triple_is_rejected) {
    Triples bad = fisLon1();
    bad[3] = {1.0, 0.0, -1.0};  // trimf requires a <= b <= c
    CHECK_THROWS(LongitudinalFis{bad});
}

TEST_CASE(parameter_sets_have_the_thesis_gene_counts) {
    CHECK(toFlat(gfsLon1()).size() == 66);
    CHECK(toFlat(gfsLat()).size() == 24);
    CHECK(toFlat(gfsLon2()).size() == 27);
    CHECK(toTriples(toFlat(fisLon1())) == fisLon1());
}

TEST_CASE(hand_picked_longitudinal_fis_reproduces_the_thesis_set_points) {
    const LongitudinalFis lon1(fisLon1());
    // values from scikit-fuzzy: on target and steady the rule base commands +1.5 m/s
    // (rule 5, "Small gap error & Medium rate -> Large"), a too-large gap does the same,
    // and closing in too much brakes
    CHECK_NEAR(lon1.acceleration(0.0, 0.0), 1.5, 1e-9);
    CHECK_NEAR(lon1.acceleration(1.0, 0.0), 1.5, 1e-9);
    CHECK_NEAR(lon1.acceleration(-0.5, 0.0), -1.245, 1e-9);
}

TEST_CASE(easy_ga_improves_a_toy_problem_and_keeps_its_invariants) {
    // minimize the distance of 6 genes from a target vector
    const EasyGa::Genes target = {3, 1, 4, 1, 5, 9};
    EasyGaConfig config;
    config.generationGoal = 60;
    config.seed = 7;
    int evaluations = 0;
    EasyGa ga(
        config, target.size(),
        [&](EasyGa::Rng& rng) {
            EasyGa::Genes genes(target.size());
            for (double& g : genes) g = std::uniform_real_distribution<double>(-20, 20)(rng);
            return genes;
        },
        nullptr,
        [&](const EasyGa::Genes& genes) {
            ++evaluations;
            double d = 0;
            for (size_t i = 0; i < genes.size(); ++i) d += std::fabs(genes[i] - target[i]);
            return d;
        });

    ga.evolveOnce();
    const double initial = bestFitness(ga);
    double previous = initial;
    while (ga.active()) {
        ga.evolveOnce();
        CHECK(ga.population().size() <= config.populationSize);
        // elitism: the best chromosome is always carried over
        CHECK(bestFitness(ga) <= previous + 1e-12);
        previous = bestFitness(ga);
        for (const auto& c : ga.population()) CHECK(c.fitness.has_value());
    }
    CHECK(ga.generation() == config.generationGoal);
    CHECK(bestFitness(ga) < initial);
    // chromosomes are only re-evaluated after they change: far fewer evaluations than 10 per generation
    CHECK(evaluations < static_cast<int>(config.populationSize * config.generationGoal));
}

TEST_CASE(easy_ga_replaces_failed_chromosomes_with_fresh_random_ones) {
    EasyGaConfig config;
    config.generationGoal = 5;
    config.seed = 3;
    // every chromosome with a negative first gene "fails"
    EasyGa ga(
        config, 4,
        [](EasyGa::Rng& rng) {
            EasyGa::Genes genes(4);
            for (double& g : genes) g = std::uniform_real_distribution<double>(-1, 1)(rng);
            return genes;
        },
        nullptr, [](const EasyGa::Genes& genes) { return genes[0] < 0 ? kFailedFitness : genes[0]; });
    while (ga.active()) ga.evolveOnce();
    CHECK(bestFitness(ga) < kFailedFitness);
}

int main() { return minitest::run_all_tests(); }
