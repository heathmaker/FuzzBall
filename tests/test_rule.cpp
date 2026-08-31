#include "fuzzylib/core/rule.hpp"

#include "fuzzylib/core/membership.hpp"
#include "minitest.hpp"

using namespace fuzzylib;

TEST_CASE(and_uses_min_by_default) {
    FuzzificationTable table;
    table["service"]["good"] = 0.7;
    table["food"]["good"] = 0.3;

    auto rule = antecedent::is("service", "good") & antecedent::is("food", "good");
    CHECK_NEAR(rule->evaluate(table, TNorm::zadeh()), 0.3, 1e-9);
}

TEST_CASE(or_uses_max_by_default) {
    FuzzificationTable table;
    table["service"]["good"] = 0.7;
    table["food"]["good"] = 0.3;

    auto rule = antecedent::is("service", "good") | antecedent::is("food", "good");
    CHECK_NEAR(rule->evaluate(table, TNorm::zadeh()), 0.7, 1e-9);
}

TEST_CASE(not_complements_degree) {
    FuzzificationTable table;
    table["service"]["good"] = 0.7;
    auto rule = ~antecedent::is("service", "good");
    CHECK_NEAR(rule->evaluate(table, TNorm::zadeh()), 0.3, 1e-9);
}

TEST_CASE(hedge_very_squares_degree) {
    FuzzificationTable table;
    table["service"]["good"] = 0.5;
    auto rule = antecedent::is("service", "good", Hedge::Very);
    CHECK_NEAR(rule->evaluate(table, TNorm::zadeh()), 0.25, 1e-9);
}

TEST_CASE(unknown_variable_throws) {
    FuzzificationTable table;
    table["service"]["good"] = 0.5;
    auto rule = antecedent::is("nonexistent", "good");
    CHECK_THROWS(rule->evaluate(table, TNorm::zadeh()));
}

TEST_CASE(rule_weight_scales_firing_strength) {
    FuzzificationTable table;
    table["service"]["good"] = 0.8;
    Rule rule(antecedent::is("service", "good"), 0.5);
    CHECK_NEAR(rule.firingStrength(table, TNorm::zadeh()), 0.4, 1e-9);
}

TEST_CASE(sugeno_consequent_first_order) {
    SugenoConsequent c;
    c.variable = "force";
    c.constant = 1.0;
    c.coefficients["angle"] = 2.0;
    c.coefficients["rate"] = -0.5;

    std::unordered_map<std::string, double> inputs{{"angle", 3.0}, {"rate", 4.0}};
    // 1.0 + 2.0*3.0 - 0.5*4.0 = 1 + 6 - 2 = 5
    CHECK_NEAR(c.evaluate(inputs), 5.0, 1e-9);
}

int main() { return minitest::run_all_tests(); }
