#pragma once

#include <string>

#include "fuzzylib/blocks/block.hpp"
#include "fuzzylib/control/pid.hpp"

namespace fuzzylib::blocks {

// Wraps a PIDController as an IBlock. Reads `setpointKey`/`measurementKey`
// from the incoming signals (and an optional per-cycle "dt" override, else
// falls back to the configured fixed step), and writes a single output.
class PIDBlock final : public IBlock {
public:
    PIDBlock(control::PIDController::Config config, double dt, std::string setpointKey = "setpoint",
              std::string measurementKey = "measurement", std::string outputKey = "output")
        : controller_(config),
          dt_(dt),
          setpointKey_(std::move(setpointKey)),
          measurementKey_(std::move(measurementKey)),
          outputKey_(std::move(outputKey)) {}

    Signals evaluate(const Signals& inputs) override {
        const double setpoint = inputs.at(setpointKey_);
        const double measurement = inputs.at(measurementKey_);
        const double dt = inputs.count("dt") ? inputs.at("dt") : dt_;
        Signals out;
        out[outputKey_] = controller_.update(setpoint, measurement, dt);
        return out;
    }

    void reset() override { controller_.reset(); }

    std::string name() const override { return "pid"; }

    control::PIDController& controller() { return controller_; }

private:
    control::PIDController controller_;
    double dt_;
    std::string setpointKey_, measurementKey_, outputKey_;
};

}  // namespace fuzzylib::blocks
