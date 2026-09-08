// Inverted pendulum (cart-pole) stabilization using a first-principles
// Takagi-Sugeno-Kang fuzzy controller, then a demonstration of blending
// that FIS with a genetic algorithm: the GA tunes a single force-gain
// parameter of the fuzzy controller against a closed-loop simulation cost,
// showing how fuzzylib's blocks can be optimized rather than hand-tuned.
//
// The 5x5 = 25 rule TSK rule base is generated programmatically from a
// simple "sum of linguistic indices" scheme (NB=-2 ... PB=+2), which is
// what "scaling up" a FIS to a complex, many-rule TSK system looks like in
// practice: rules follow a systematic gain-scheduling pattern rather than
// being hand-authored one by one.

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.8;
constexpr double kCartMass = 1.0;
constexpr double kPoleMass = 0.1;
constexpr double kHalfPoleLength = 0.5;
constexpr double kFailureAngle = 0.2094;  // ~12 degrees, the classic cart-pole failure threshold
constexpr double kTrackLimit = 2.4;       // meters
constexpr double kDt = 0.02;              // seconds per control/physics step
constexpr double kMaxForce = 15.0;        // newtons

struct CartPoleState {
    double x = 0.0, xDot = 0.0;
    double theta = 0.0, thetaDot = 0.0;
};

// Correct nonlinear cart-pole dynamics (Florian 2007's fix to the classic
// Barto/Sutton/Anderson 1983 equations), integrated with explicit Euler.
void step(CartPoleState& s, double force, double dt) {
    const double totalMass = kCartMass + kPoleMass;
    const double poleMassLength = kPoleMass * kHalfPoleLength;

    const double costheta = std::cos(s.theta);
    const double sintheta = std::sin(s.theta);

    const double temp = (force + poleMassLength * s.thetaDot * s.thetaDot * sintheta) / totalMass;
    const double thetaAcc = (kGravity * sintheta - costheta * temp) /
                             (kHalfPoleLength * (4.0 / 3.0 - kPoleMass * costheta * costheta / totalMass));
    const double xAcc = temp - poleMassLength * thetaAcc * costheta / totalMass;

    s.xDot += xAcc * dt;
    s.x += s.xDot * dt;
    s.thetaDot += thetaAcc * dt;
    s.theta += s.thetaDot * dt;
}

// Builds a 5x5 zero-order TSK controller: each of the 5 linguistic terms
// per input carries an integer index in {-2,-1,0,1,2} (NB..PB), and rule
// (i, j)'s crisp output is a force proportional to -(i + j), i.e. "push
// harder the further the pole has tipped and the faster it's still moving
// that way" — a fuzzy gain-scheduled PD controller expressed as 25 rules.
SugenoEngine buildPendulumFIS() {
    struct NamedTerm {
        std::string name;
        int index;
    };
    const std::array<NamedTerm, 5> terms{{{"NB", -2}, {"NS", -1}, {"Z", 0}, {"PS", 1}, {"PB", 2}}};

    LinguisticVariable angle("angle", -kFailureAngle, kFailureAngle);
    angle.addTerm("NB", mf::make<mf::Trapezoidal>(-kFailureAngle, -kFailureAngle, -0.14, -0.04));
    angle.addTerm("NS", mf::make<mf::Triangular>(-0.14, -0.06, 0.0));
    angle.addTerm("Z", mf::make<mf::Triangular>(-0.05, 0.0, 0.05));
    angle.addTerm("PS", mf::make<mf::Triangular>(0.0, 0.06, 0.14));
    angle.addTerm("PB", mf::make<mf::Trapezoidal>(0.04, 0.14, kFailureAngle, kFailureAngle));

    constexpr double kRateSpan = 2.5;
    LinguisticVariable rate("rate", -kRateSpan, kRateSpan);
    rate.addTerm("NB", mf::make<mf::Trapezoidal>(-kRateSpan, -kRateSpan, -1.4, -0.4));
    rate.addTerm("NS", mf::make<mf::Triangular>(-1.4, -0.7, 0.0));
    rate.addTerm("Z", mf::make<mf::Triangular>(-0.5, 0.0, 0.5));
    rate.addTerm("PS", mf::make<mf::Triangular>(0.0, 0.7, 1.4));
    rate.addTerm("PB", mf::make<mf::Trapezoidal>(0.4, 1.4, kRateSpan, kRateSpan));

    SugenoEngine engine;
    engine.addInput(angle).addInput(rate);

    for (const auto& angleTerm : terms) {
        for (const auto& rateTerm : terms) {
            const auto indexSum = static_cast<double>(angleTerm.index + rateTerm.index);
            SugenoConsequent force;
            force.variable = "force";
            // Positive theta means the pole is tipping in the positive
            // direction; per the cart-pole dynamics below, countering that
            // requires a *positive* cart force (it's the cos(theta)*F term
            // that must outrun gravity's g*sin(theta) term), so the
            // corrective force must carry the *same* sign as (angle+rate).
            force.constant = (kMaxForce / 4.0) * indexSum;
            engine.rules().add(
                Rule(antecedent::is("angle", angleTerm.name) & antecedent::is("rate", rateTerm.name))
                    .then(force));
        }
    }
    return engine;
}

struct SimulationResult {
    double cost;
    int stepsSurvived;
    bool fell;
};

// Runs one closed-loop trial. `forceGain` scales the FIS output before it's
// applied to the plant — this is the single parameter the GA below tunes.
SimulationResult simulate(const SugenoEngine& fis, double forceGain, double initialAngle, int maxSteps) {
    CartPoleState state;
    state.theta = initialAngle;

    double cost = 0.0;
    for (int t = 0; t < maxSteps; ++t) {
        const double clampedAngle = std::max(-kFailureAngle, std::min(kFailureAngle, state.theta));
        const double clampedRate = std::max(-2.5, std::min(2.5, state.thetaDot));
        const auto result = fis.evaluate({{"angle", clampedAngle}, {"rate", clampedRate}});
        double force = forceGain * result.at("force");
        force = std::max(-kMaxForce, std::min(kMaxForce, force));

        step(state, force, kDt);

        // ITAE-style cost: penalize sustained angle error, weighted by time
        // so late-trial wobble is punished more than an initial transient.
        cost += (t * kDt) * std::fabs(state.theta) * kDt;

        if (std::fabs(state.theta) > kFailureAngle || std::fabs(state.x) > kTrackLimit) {
            const double remaining = static_cast<double>(maxSteps - t) * kDt;
            cost += 50.0 * remaining;  // large penalty for falling early
            return {cost, t, true};
        }
    }
    return {cost, maxSteps, false};
}

void printTrace(const SugenoEngine& fis, double forceGain, double initialAngle, int maxSteps) {
    CartPoleState state;
    state.theta = initialAngle;
    const int printInterval = std::max(1, maxSteps / 10);

    std::cout << "  step   time    x(m)   theta(deg)\n";
    for (int t = 0; t <= maxSteps; ++t) {
        if (t % printInterval == 0) {
            std::cout << "  " << std::setw(4) << t << "  " << std::fixed << std::setprecision(2)
                       << std::setw(5) << (t * kDt) << "  " << std::setw(6) << state.x << "   "
                       << std::setw(8) << (state.theta * 180.0 / kPi) << "\n";
        }
        if (t == maxSteps) break;
        const auto result = fis.evaluate({{"angle", state.theta}, {"rate", state.thetaDot}});
        double force = std::max(-kMaxForce, std::min(kMaxForce, forceGain * result.at("force")));
        step(state, force, kDt);
        if (std::fabs(state.theta) > kFailureAngle) {
            std::cout << "  Pole fell at step " << t << "\n";
            break;
        }
    }
}

}  // namespace

int main() {
    const SugenoEngine fis = buildPendulumFIS();
    constexpr double kInitialAngle = 0.15;  // radians (~8.6 degrees)
    constexpr int kMaxSteps = 400;          // 8 seconds at dt=0.02

    std::cout << "Inverted pendulum stabilization (25-rule TSK/Sugeno FIS)\n\n";

    std::cout << "Baseline controller (force gain = 1.0):\n";
    printTrace(fis, 1.0, kInitialAngle, kMaxSteps);
    const SimulationResult baseline = simulate(fis, 1.0, kInitialAngle, kMaxSteps);
    std::cout << "  Baseline cost = " << baseline.cost << (baseline.fell ? "  (fell)\n" : "  (stable)\n");

    std::cout << "\nTuning force gain with a genetic algorithm (blending GA + TSK-FIS)...\n";
    auto fitness = [&](const ga::Genome& genome) {
        const double gain = genome[0];
        return -simulate(fis, gain, kInitialAngle, kMaxSteps).cost;
    };
    ga::GAConfig gaConfig;
    gaConfig.populationSize = 30;
    gaConfig.generations = 40;
    gaConfig.seed = 123;
    ga::GeneticAlgorithm optimizer(gaConfig, {ga::Bounds{0.1, 5.0}}, fitness);
    const ga::GAResult gaResult = optimizer.run();
    const double tunedGain = gaResult.best[0];

    std::cout << "  GA-tuned force gain = " << tunedGain << "\n\n";
    std::cout << "Tuned controller (force gain = " << tunedGain << "):\n";
    printTrace(fis, tunedGain, kInitialAngle, kMaxSteps);
    const SimulationResult tuned = simulate(fis, tunedGain, kInitialAngle, kMaxSteps);
    std::cout << "  Tuned cost = " << tuned.cost << (tuned.fell ? "  (fell)\n" : "  (stable)\n");

    std::cout << "\nCost improvement: "
               << (baseline.cost > 0 ? 100.0 * (baseline.cost - tuned.cost) / baseline.cost : 0.0) << "%\n";
    return 0;
}
