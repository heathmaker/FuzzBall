#include "fuzzylib/inference/sugeno.hpp"

#include "fuzzylib/core/membership.hpp"
#include "minitest.hpp"

using namespace fuzzylib;

TEST_CASE(zero_order_sugeno_weighted_average) {
    LinguisticVariable x("x", 0.0, 10.0);
    x.addTerm("low", mf::make<mf::Trapezoidal>(0.0, 0.0, 2.0, 6.0));
    x.addTerm("high", mf::make<mf::Trapezoidal>(4.0, 8.0, 10.0, 10.0));

    SugenoEngine engine;
    engine.addInput(x);

    SugenoConsequent lowOut;
    lowOut.variable = "y";
    lowOut.constant = 0.0;
    SugenoConsequent highOut;
    highOut.variable = "y";
    highOut.constant = 10.0;

    engine.rules().add(Rule(antecedent::is("x", "low")).then(lowOut));
    engine.rules().add(Rule(antecedent::is("x", "high")).then(highOut));

    // At x=5: low membership = (6-5)/(6-2) = 0.25, high membership = (5-4)/(8-4) = 0.25.
    const auto result = engine.evaluate({{"x", 5.0}});
    const double expected = (0.25 * 0.0 + 0.25 * 10.0) / (0.25 + 0.25);
    CHECK_NEAR(result.at("y"), expected, 1e-9);
}

TEST_CASE(first_order_sugeno_uses_linear_consequent) {
    LinguisticVariable angle("angle", -1.0, 1.0);
    angle.addTerm("any", mf::make<mf::Constant>(1.0));

    SugenoEngine engine;
    engine.addInput(angle);

    SugenoConsequent force;
    force.variable = "force";
    force.constant = 1.0;
    force.coefficients["angle"] = 3.0;
    engine.rules().add(Rule(antecedent::is("angle", "any")).then(force));

    const auto result = engine.evaluate({{"angle", 2.0}});
    CHECK_NEAR(result.at("force"), 1.0 + 3.0 * 2.0, 1e-9);
}

TEST_CASE(no_firing_rule_yields_zero_output) {
    LinguisticVariable x("x", 0.0, 10.0);
    x.addTerm("high", mf::make<mf::Trapezoidal>(8.0, 9.0, 10.0, 10.0));
    SugenoEngine engine;
    engine.addInput(x);
    SugenoConsequent out;
    out.variable = "y";
    out.constant = 5.0;
    engine.rules().add(Rule(antecedent::is("x", "high")).then(out));

    const auto result = engine.evaluate({{"x", 0.0}});
    CHECK_NEAR(result.at("y"), 0.0, 1e-9);
}

int main() { return minitest::run_all_tests(); }
