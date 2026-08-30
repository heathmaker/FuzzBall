#include "fuzzylib/inference/mamdani.hpp"

#include "fuzzylib/core/membership.hpp"
#include "minitest.hpp"

using namespace fuzzylib;

namespace {
MamdaniEngine buildTipper() {
    LinguisticVariable service("service", 0.0, 10.0);
    service.addTerm("poor", mf::make<mf::Trapezoidal>(0.0, 0.0, 2.0, 4.0));
    service.addTerm("average", mf::make<mf::Triangular>(2.0, 5.0, 8.0));
    service.addTerm("good", mf::make<mf::Trapezoidal>(6.0, 8.0, 10.0, 10.0));

    LinguisticVariable food("food", 0.0, 10.0);
    food.addTerm("poor", mf::make<mf::Trapezoidal>(0.0, 0.0, 3.0, 7.0));
    food.addTerm("good", mf::make<mf::Trapezoidal>(3.0, 7.0, 10.0, 10.0));

    LinguisticVariable tip("tip", 0.0, 30.0);
    tip.addTerm("low", mf::make<mf::Trapezoidal>(0.0, 0.0, 5.0, 13.0));
    tip.addTerm("medium", mf::make<mf::Triangular>(7.0, 15.0, 23.0));
    tip.addTerm("high", mf::make<mf::Trapezoidal>(17.0, 25.0, 30.0, 30.0));

    MamdaniEngine engine;
    engine.addInput(service).addInput(food).addOutput(tip);

    using namespace antecedent;
    engine.rules().add(Rule(is("service", "poor") | is("food", "poor")).then(MamdaniConsequent{"tip", "low"}));
    engine.rules().add(Rule(is("service", "average")).then(MamdaniConsequent{"tip", "medium"}));
    engine.rules().add(Rule(is("service", "good") | is("food", "good")).then(MamdaniConsequent{"tip", "high"}));
    return engine;
}
}  // namespace

TEST_CASE(tipper_excellent_service_and_food_gives_high_tip) {
    MamdaniEngine engine = buildTipper();
    auto result = engine.evaluate({{"service", 9.8}, {"food", 9.8}});
    CHECK(result["tip"] > 20.0);
}

TEST_CASE(tipper_poor_service_and_food_gives_low_tip) {
    MamdaniEngine engine = buildTipper();
    auto result = engine.evaluate({{"service", 0.5}, {"food", 0.5}});
    CHECK(result["tip"] < 10.0);
}

TEST_CASE(tipper_output_is_monotonic_in_service) {
    MamdaniEngine engine = buildTipper();
    const double low = engine.evaluate({{"service", 2.0}, {"food", 5.0}})["tip"];
    const double high = engine.evaluate({{"service", 9.0}, {"food", 5.0}})["tip"];
    CHECK(high > low);
}

TEST_CASE(defuzz_methods_agree_on_a_single_symmetric_triangle) {
    LinguisticVariable dummy("in", 0.0, 1.0);
    dummy.addTerm("any", mf::make<mf::Constant>(1.0));

    LinguisticVariable out("out", 0.0, 10.0);
    out.addTerm("mid", mf::make<mf::Triangular>(2.0, 5.0, 8.0));

    MamdaniEngine engine(TNorm::zadeh(), Aggregator::max(), 500);
    engine.addInput(dummy).addOutput(out);
    engine.rules().add(Rule(antecedent::is("in", "any")).then(MamdaniConsequent{"out", "mid"}));

    const double centroid = engine.evaluate({{"in", 0.0}}, DefuzzMethod::Centroid)["out"];
    const double bisector = engine.evaluate({{"in", 0.0}}, DefuzzMethod::Bisector)["out"];
    const double mom = engine.evaluate({{"in", 0.0}}, DefuzzMethod::MeanOfMax)["out"];
    CHECK_NEAR(centroid, 5.0, 0.05);
    CHECK_NEAR(bisector, 5.0, 0.05);
    CHECK_NEAR(mom, 5.0, 0.05);
}

int main() { return minitest::run_all_tests(); }
