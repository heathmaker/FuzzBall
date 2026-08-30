// The classic "tipping problem": given how good the service and the food
// were, decide how much to tip. This is the textbook introduction to
// Mamdani fuzzy inference (the same example scikit-fuzzy and MATLAB's
// Fuzzy Logic Toolbox use), included here as the simplest possible
// end-to-end demonstration of fuzzylib's Mamdani engine.

#include <iomanip>
#include <iostream>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;

namespace {

MamdaniEngine buildTipperFIS() {
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
    // IF service is poor OR food is poor THEN tip is low
    engine.rules().add(Rule(is("service", "poor") | is("food", "poor")).then(MamdaniConsequent{"tip", "low"}));
    // IF service is average THEN tip is medium
    engine.rules().add(Rule(is("service", "average")).then(MamdaniConsequent{"tip", "medium"}));
    // IF service is good OR food is good THEN tip is high
    engine.rules().add(Rule(is("service", "good") | is("food", "good")).then(MamdaniConsequent{"tip", "high"}));

    return engine;
}

void report(const MamdaniEngine& engine, double service, double food) {
    const auto result = engine.evaluate({{"service", service}, {"food", food}});
    std::cout << "  service=" << std::setw(4) << service << "  food=" << std::setw(4) << food
               << "  ->  tip = " << std::fixed << std::setprecision(2) << result.at("tip") << "%\n";
}

}  // namespace

int main() {
    const MamdaniEngine engine = buildTipperFIS();

    std::cout << "Tipper problem (Mamdani FIS)\n"
                  "  Inputs:  service, food in [0, 10]\n"
                  "  Output:  tip in [0, 30] (%)\n\n";

    report(engine, 9.8, 9.8);  // excellent everything -> generous tip
    report(engine, 1.0, 1.0);  // terrible everything -> minimal tip
    report(engine, 5.0, 5.0);  // middling everything -> moderate tip
    report(engine, 9.8, 2.0);  // great service, bad food -> service alone should carry it
    report(engine, 2.0, 9.8);  // bad service, great food -> food alone should carry it

    return 0;
}
