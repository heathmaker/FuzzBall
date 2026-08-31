#pragma once

namespace fuzzylib::control {

// Textbook PID controller with anti-windup (clamped integral) and a
// low-pass filtered derivative computed on measurement (avoids derivative
// kick on setpoint changes). Meant to be used standalone or wrapped as a
// blocks::IBlock (see blocks/pid_block.hpp) so it can sit in a Pipeline or
// Blend alongside fuzzy or learned controllers.
class PIDController {
public:
    struct Config {
        double kp = 1.0;
        double ki = 0.0;
        double kd = 0.0;
        double outputMin = -1e300;
        double outputMax = 1e300;
        double integralMin = -1e300;
        double integralMax = 1e300;
        // Derivative low-pass filter coefficient in (0, 1]; 1 = unfiltered.
        double derivativeFilter = 1.0;
    };

    // Delegates rather than using `Config config = {}` directly: GCC has a
    // long-standing bug (aggregate-init of a nested class as a default
    // function argument, evaluated before the class is "complete") that
    // rejects that spelling even though it is valid C++17.
    PIDController() : PIDController(Config{}) {}
    explicit PIDController(Config config) : cfg_(config) {}

    void setGains(double kp, double ki, double kd) {
        cfg_.kp = kp;
        cfg_.ki = ki;
        cfg_.kd = kd;
    }

    const Config& config() const { return cfg_; }

    double update(double setpoint, double measurement, double dt) {
        const double error = setpoint - measurement;

        integral_ += error * dt;
        if (integral_ > cfg_.integralMax) integral_ = cfg_.integralMax;
        if (integral_ < cfg_.integralMin) integral_ = cfg_.integralMin;

        double rawDerivative = 0.0;
        if (!firstUpdate_ && dt > 0.0) {
            rawDerivative = -(measurement - prevMeasurement_) / dt;
        }
        filteredDerivative_ += cfg_.derivativeFilter * (rawDerivative - filteredDerivative_);

        double output = cfg_.kp * error + cfg_.ki * integral_ + cfg_.kd * filteredDerivative_;
        if (output > cfg_.outputMax) output = cfg_.outputMax;
        if (output < cfg_.outputMin) output = cfg_.outputMin;

        prevMeasurement_ = measurement;
        firstUpdate_ = false;
        return output;
    }

    void reset() {
        integral_ = 0.0;
        filteredDerivative_ = 0.0;
        prevMeasurement_ = 0.0;
        firstUpdate_ = true;
    }

private:
    Config cfg_;
    double integral_ = 0.0;
    double filteredDerivative_ = 0.0;
    double prevMeasurement_ = 0.0;
    bool firstUpdate_ = true;
};

}  // namespace fuzzylib::control
