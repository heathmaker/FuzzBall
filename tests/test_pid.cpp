#include "fuzzylib/control/pid.hpp"

#include "minitest.hpp"

using namespace fuzzylib::control;

TEST_CASE(proportional_only_responds_to_error) {
    PIDController pid(PIDController::Config{2.0, 0.0, 0.0});
    const double out = pid.update(/*setpoint=*/10.0, /*measurement=*/4.0, /*dt=*/0.1);
    CHECK_NEAR(out, 12.0, 1e-9);
}

TEST_CASE(integral_accumulates_over_time) {
    PIDController pid(PIDController::Config{0.0, 1.0, 0.0});
    pid.update(1.0, 0.0, 1.0);  // error 1, integral -> 1
    const double out = pid.update(1.0, 0.0, 1.0);  // error 1, integral -> 2
    CHECK_NEAR(out, 2.0, 1e-9);
}

TEST_CASE(output_clamped_to_configured_range) {
    PIDController::Config cfg;
    cfg.kp = 100.0;
    cfg.outputMin = -1.0;
    cfg.outputMax = 1.0;
    PIDController pid(cfg);
    CHECK_NEAR(pid.update(10.0, 0.0, 0.1), 1.0, 1e-9);
    CHECK_NEAR(pid.update(-10.0, 0.0, 0.1), -1.0, 1e-9);
}

TEST_CASE(integral_windup_is_clamped) {
    PIDController::Config cfg;
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.integralMin = -5.0;
    cfg.integralMax = 5.0;
    PIDController pid(cfg);
    for (int i = 0; i < 100; ++i) pid.update(100.0, 0.0, 1.0);
    const double out = pid.update(100.0, 0.0, 1.0);
    CHECK_NEAR(out, 5.0, 1e-9);
}

TEST_CASE(reset_clears_internal_state) {
    PIDController pid(PIDController::Config{0.0, 1.0, 0.0});
    pid.update(1.0, 0.0, 1.0);
    pid.reset();
    const double out = pid.update(1.0, 0.0, 1.0);
    CHECK_NEAR(out, 1.0, 1e-9);
}

int main() { return minitest::run_all_tests(); }
