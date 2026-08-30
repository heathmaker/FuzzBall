// Quadcopter attitude control via a fuzzy-gain-scheduled PID hybrid: the
// textbook way to blend fuzzy logic with classical control. A small
// Mamdani FIS looks at the current attitude error and decides how much to
// trust an "aggressive" (fast, high-gain) PID versus a "gentle" (soft,
// low-overshoot) PID; a Blend block combines their torque commands using
// that fuzzy-computed weight. The same controller is instantiated three
// times to stabilize roll, pitch, and yaw independently.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;
using namespace fuzzylib::blocks;

namespace {

constexpr double kDt = 0.01;
constexpr double kInertia = 0.02;    // kg*m^2, single-axis moment of inertia
constexpr double kDamping = 0.08;    // N*m per (rad/s), aerodynamic damping
constexpr double kMaxTorque = 5.0;   // N*m, actuator limit

// A single rotational degree of freedom: torque -> angular acceleration
// -> angular rate -> angle, with linear aerodynamic damping.
struct AxisPlant {
    double angle = 0.0;
    double rate = 0.0;

    void step(double torque, double dt) {
        const double angularAccel = (torque - kDamping * rate) / kInertia;
        rate += angularAccel * dt;
        angle += rate * dt;
    }
};

// Fuzzy scheduler: |error| -> aggressiveness weight in [0, 1]. Small error
// -> prefer the gentle PID (avoid overshoot near setpoint); large error ->
// prefer the aggressive PID (react quickly to big disturbances/commands).
std::shared_ptr<MamdaniEngine> buildScheduler() {
    LinguisticVariable absError("absError", 0.0, 1.6);  // radians, ~0-90 degrees
    absError.addTerm("small", mf::make<mf::Trapezoidal>(0.0, 0.0, 0.05, 0.2));
    absError.addTerm("large", mf::make<mf::Trapezoidal>(0.1, 0.3, 1.6, 1.6));

    LinguisticVariable weight("aggressiveWeight", 0.0, 1.0);
    weight.addTerm("low", mf::make<mf::Trapezoidal>(0.0, 0.0, 0.15, 0.5));
    weight.addTerm("high", mf::make<mf::Trapezoidal>(0.5, 0.85, 1.0, 1.0));

    auto engine = std::make_shared<MamdaniEngine>();
    engine->addInput(absError).addOutput(weight);
    engine->rules().add(
        Rule(antecedent::is("absError", "small")).then(MamdaniConsequent{"aggressiveWeight", "low"}));
    engine->rules().add(
        Rule(antecedent::is("absError", "large")).then(MamdaniConsequent{"aggressiveWeight", "high"}));
    return engine;
}

// Tiny glue block: turns the scheduler's single output into a complementary
// pair of weights so Blend can read one weight key per PID member.
class ComplementBlock final : public IBlock {
public:
    ComplementBlock(std::string sourceKey, std::string complementKey)
        : sourceKey_(std::move(sourceKey)), complementKey_(std::move(complementKey)) {}

    Signals evaluate(const Signals& inputs) override {
        return {{complementKey_, 1.0 - inputs.at(sourceKey_)}};
    }
    std::string name() const override { return "complement"; }

private:
    std::string sourceKey_, complementKey_;
};

// Builds one axis's full hybrid controller: fuzzy scheduler -> complement
// -> weighted blend of a gentle and an aggressive PID.
std::shared_ptr<Pipeline> buildAxisController(std::shared_ptr<MamdaniEngine> scheduler) {
    control::PIDController::Config gentleCfg;
    gentleCfg.kp = 2.0;
    gentleCfg.ki = 0.5;
    gentleCfg.kd = 0.4;
    gentleCfg.outputMin = -kMaxTorque;
    gentleCfg.outputMax = kMaxTorque;
    gentleCfg.integralMin = -2.0;
    gentleCfg.integralMax = 2.0;

    control::PIDController::Config aggressiveCfg;
    aggressiveCfg.kp = 8.0;
    aggressiveCfg.ki = 1.0;
    aggressiveCfg.kd = 1.2;
    aggressiveCfg.outputMin = -kMaxTorque;
    aggressiveCfg.outputMax = kMaxTorque;
    aggressiveCfg.integralMin = -2.0;
    aggressiveCfg.integralMax = 2.0;

    auto gentlePID = std::make_shared<PIDBlock>(gentleCfg, kDt);
    auto aggressivePID = std::make_shared<PIDBlock>(aggressiveCfg, kDt);

    auto blend = std::make_shared<Blend>(
        std::vector<BlendMember>{
            BlendMember{gentlePID, "output", "gentleWeight", 1.0},
            BlendMember{aggressivePID, "output", "aggressiveWeight", 1.0},
        },
        "torque");

    auto pipeline = std::make_shared<Pipeline>();
    pipeline->add("scheduler", std::make_shared<MamdaniBlock>(scheduler));
    pipeline->add("complement", std::make_shared<ComplementBlock>("aggressiveWeight", "gentleWeight"));
    pipeline->add("blend", blend);
    return pipeline;
}

double toDegrees(double radians) { return radians * 180.0 / 3.14159265358979323846; }
double toRadians(double degrees) { return degrees * 3.14159265358979323846 / 180.0; }

}  // namespace

int main() {
    auto scheduler = buildScheduler();
    auto rollController = buildAxisController(scheduler);
    auto pitchController = buildAxisController(scheduler);
    auto yawController = buildAxisController(scheduler);

    AxisPlant roll, pitch, yaw;
    const double rollSetpoint = toRadians(15.0);
    const double pitchSetpoint = toRadians(-10.0);
    const double yawSetpointLate = toRadians(20.0);

    constexpr double kSimTime = 5.0;
    const int steps = static_cast<int>(kSimTime / kDt);
    const int printEvery = static_cast<int>(0.25 / kDt);

    std::cout << "Quadcopter attitude control (fuzzy gain-scheduled PID, 3 axes)\n\n";
    std::cout << "  time   roll(deg)  pitch(deg)  yaw(deg)\n";

    for (int step = 0; step <= steps; ++step) {
        const double t = step * kDt;
        const double yawSetpoint = (t >= 1.0) ? yawSetpointLate : 0.0;

        const double absRollErr = std::fabs(rollSetpoint - roll.angle);
        auto rollOut = rollController->evaluate(
            {{"setpoint", rollSetpoint}, {"measurement", roll.angle}, {"absError", absRollErr}});
        roll.step(rollOut.at("torque"), kDt);

        const double absPitchErr = std::fabs(pitchSetpoint - pitch.angle);
        auto pitchOut = pitchController->evaluate(
            {{"setpoint", pitchSetpoint}, {"measurement", pitch.angle}, {"absError", absPitchErr}});
        pitch.step(pitchOut.at("torque"), kDt);

        const double absYawErr = std::fabs(yawSetpoint - yaw.angle);
        auto yawOut = yawController->evaluate(
            {{"setpoint", yawSetpoint}, {"measurement", yaw.angle}, {"absError", absYawErr}});
        yaw.step(yawOut.at("torque"), kDt);

        if (step % printEvery == 0) {
            std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(5) << t << "   "
                       << std::setw(8) << toDegrees(roll.angle) << "   " << std::setw(9)
                       << toDegrees(pitch.angle) << "   " << std::setw(7) << toDegrees(yaw.angle) << "\n";
        }
    }

    std::cout << "\nFinal errors: roll=" << toDegrees(rollSetpoint - roll.angle)
               << " deg, pitch=" << toDegrees(pitchSetpoint - pitch.angle)
               << " deg, yaw=" << toDegrees(yawSetpointLate - yaw.angle) << " deg\n";
    return 0;
}
