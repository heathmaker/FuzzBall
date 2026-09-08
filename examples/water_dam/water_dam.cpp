// Water dam gate control: a Mamdani FIS decides how far to open a
// spillway gate from the reservoir's water level and the current inflow
// rate, then drives a simple tank simulation to show the level being kept
// inside a safe band despite a varying inflow profile (drought, then a
// storm surge, then a lull).

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;

namespace {

MamdaniEngine buildGateController() {
    LinguisticVariable level("level", 0.0, 100.0);
    level.addTerm("low", mf::make<mf::Trapezoidal>(0.0, 0.0, 20.0, 45.0));
    level.addTerm("normal", mf::make<mf::Triangular>(30.0, 55.0, 80.0));
    level.addTerm("high", mf::make<mf::Trapezoidal>(60.0, 80.0, 100.0, 100.0));

    LinguisticVariable inflow("inflow", 0.0, 100.0);
    inflow.addTerm("low", mf::make<mf::Trapezoidal>(0.0, 0.0, 20.0, 45.0));
    inflow.addTerm("medium", mf::make<mf::Triangular>(25.0, 50.0, 75.0));
    inflow.addTerm("high", mf::make<mf::Trapezoidal>(55.0, 80.0, 100.0, 100.0));

    LinguisticVariable gate("gate", 0.0, 100.0);
    gate.addTerm("closed", mf::make<mf::Trapezoidal>(0.0, 0.0, 5.0, 25.0));
    gate.addTerm("half", mf::make<mf::Triangular>(15.0, 45.0, 75.0));
    gate.addTerm("open", mf::make<mf::Trapezoidal>(65.0, 90.0, 100.0, 100.0));

    MamdaniEngine engine;
    engine.addInput(level).addInput(inflow).addOutput(gate);

    using namespace antecedent;
    // Low reservoir: hold water back regardless of inflow.
    engine.rules().add(Rule(is("level", "low")).then(MamdaniConsequent{"gate", "closed"}));
    // Normal level: release rate should track inflow rate to stay in balance.
    engine.rules().add(
        Rule(is("level", "normal") & is("inflow", "low")).then(MamdaniConsequent{"gate", "closed"}));
    engine.rules().add(
        Rule(is("level", "normal") & is("inflow", "medium")).then(MamdaniConsequent{"gate", "half"}));
    engine.rules().add(
        Rule(is("level", "normal") & is("inflow", "high")).then(MamdaniConsequent{"gate", "open"}));
    // High reservoir: always spill, and spill hard if inflow is also high.
    engine.rules().add(Rule(is("level", "high") & is("inflow", "high")).then(MamdaniConsequent{"gate", "open"}));
    engine.rules().add(Rule(is("level", "high") & ~is("inflow", "high")).then(MamdaniConsequent{"gate", "open"}));

    return engine;
}

// A simplified single-reservoir tank: level rises with inflow and falls
// with an outflow proportional to how far the gate is open. Units are
// abstract percentage points rather than physical volumes, chosen so the
// dynamics are easy to read on a printed trace.
struct TankSimulator {
    double level = 50.0;
    // A well-sized spillway can, at full opening, shed water at least as
    // fast as the worst-case inflow (100%/step) — otherwise no gate policy
    // could ever prevent overflow during a severe storm.
    static constexpr double kMaxOutflowRate = 100.0;  // level %/step at gate fully open

    void step(double inflowRate, double gateOpeningPercent, double dt) {
        const double outflowRate = kMaxOutflowRate * (gateOpeningPercent / 100.0);
        level += dt * (inflowRate - outflowRate);
        level = std::min(100.0, std::max(0.0, level));
    }
};

}  // namespace

int main() {
    const MamdaniEngine controller = buildGateController();
    TankSimulator tank;

    // A scripted inflow profile: calm, then a storm surge, then a lull.
    std::vector<double> inflowProfile;
    inflowProfile.reserve(50);
    for (int t = 0; t < 15; ++t) inflowProfile.push_back(20.0);
    for (int t = 0; t < 20; ++t) inflowProfile.push_back(85.0);
    for (int t = 0; t < 15; ++t) inflowProfile.push_back(10.0);

    std::cout << "Water dam gate control (Mamdani FIS)\n"
                  "  step  inflow  level   gate_opening\n";

    double minLevel = tank.level;
    double maxLevel = tank.level;
    for (std::size_t t = 0; t < inflowProfile.size(); ++t) {
        const double inflow = inflowProfile[t];
        const auto result = controller.evaluate({{"level", tank.level}, {"inflow", inflow}});
        const double gateOpening = result.at("gate");

        std::cout << "  " << std::setw(4) << t << "  " << std::setw(6) << std::fixed << std::setprecision(1)
                   << inflow << "  " << std::setw(6) << tank.level << "  " << std::setw(6) << gateOpening
                   << "\n";

        tank.step(inflow, gateOpening, /*dt=*/1.0);
        minLevel = std::min(minLevel, tank.level);
        maxLevel = std::max(maxLevel, tank.level);
    }

    std::cout << "\nFinal level: " << tank.level << "%  (range visited: " << minLevel << "% - " << maxLevel
               << "%)\n";
    std::cout << (maxLevel < 100.0 && minLevel > 0.0 ? "Reservoir stayed within safe bounds.\n"
                                                       : "Reservoir hit a safety limit!\n");
    return 0;
}
