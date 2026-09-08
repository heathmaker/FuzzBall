// Full 6-degree-of-freedom quadcopter flight simulation: 3 translational +
// 3 rotational degrees of freedom (position/velocity in x,y,z and
// attitude/angular-rate in roll,pitch,yaw), integrated as a rigid body
// with a quaternion orientation. (This is a dynamics model, not a sensor
// model — a 9DoF IMU, 3-axis accel + 3-axis gyro + 3-axis magnetometer,
// is a different concept used for *estimating* attitude from noisy
// sensors; here the controller reads exact simulated state.)
//
// Control is the same cascaded architecture real autopilots use:
//   position error -> desired acceleration -> desired thrust vector
//     -> desired roll/pitch/yaw -> attitude controller -> torques
//     -> motor mixer -> 4 individual rotor thrusts -> rigid-body dynamics
// The attitude loop reuses the fuzzy-gain-scheduled PID hybrid from the
// original single-axis quadcopter demo (a Mamdani scheduler blending a
// gentle and an aggressive PID by error magnitude), now driving all three
// axes against time-varying setpoints produced by the position loop
// instead of a single fixed step command.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>

#include "fuzzylib/fuzzylib.hpp"

using namespace fuzzylib;
using namespace fuzzylib::blocks;

namespace {

constexpr double kPi = 3.14159265358979323846;
double toDegrees(double radians) { return radians * 180.0 / kPi; }
double toRadians(double degrees) { return degrees * kPi / 180.0; }

// ---------------------------------------------------------------------
// Minimal 3D vector / quaternion math (kept local to this example: it's
// rigid-body support code, not fuzzy-logic-library scope).
// ---------------------------------------------------------------------

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        const double len = length();
        return len > 1e-9 ? (*this) * (1.0 / len) : Vec3{0.0, 0.0, 1.0};
    }
};
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Quaternion {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    Quaternion operator*(const Quaternion& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }
    Quaternion operator+(const Quaternion& o) const { return {w + o.w, x + o.x, y + o.y, z + o.z}; }
    Quaternion operator*(double s) const { return {w * s, x * s, y * s, z * s}; }

    void normalize() {
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n > 1e-12) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
    }
};

// Rotates a body-frame vector into the world frame: v_world = R(q) * v_body.
Vec3 rotateBodyToWorld(const Quaternion& q, const Vec3& v) {
    const Vec3 qv{q.x, q.y, q.z};
    const Vec3 t = cross(qv, v) * 2.0;
    return v + t * q.w + cross(qv, t);
}

// Standard aerospace ZYX (yaw-pitch-roll) Euler extraction from a
// world-from-body quaternion.
struct EulerAngles {
    double roll, pitch, yaw;
};
EulerAngles toEuler(const Quaternion& q) {
    const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    double pitchArg = 2.0 * (q.w * q.y - q.z * q.x);
    pitchArg = std::max(-1.0, std::min(1.0, pitchArg));
    const double pitch = std::asin(pitchArg);
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return {roll, pitch, yaw};
}

// ---------------------------------------------------------------------
// Rigid-body plant: 6DoF state and the equations of motion.
// ---------------------------------------------------------------------

constexpr double kMass = 1.2;                  // kg
constexpr double kGravity = 9.81;               // m/s^2
constexpr double kArmLength = 0.2;              // m, motor distance from center
constexpr double kYawDragCoeff = 0.02;          // N*m per N of thrust (reactive drag torque)
constexpr Vec3 kInertia{0.015, 0.015, 0.03};    // kg*m^2, diagonal (Ixx, Iyy, Izz)
constexpr double kMaxThrustPerMotor = 6.0;      // N (thrust/weight ratio ~2, a nimble quad)
constexpr double kLinearDrag = 0.25;            // N per (m/s), world-frame

struct RigidBodyState {
    Vec3 pos, vel;
    Quaternion q;
    Vec3 omega;  // body-frame angular velocity (p, q, r)
};

// + configuration motor mixing: front/back motors control pitch, left/right
// control roll, and all four (via alternating spin direction) control yaw
// through reactive drag torque. Solving the 4 mixing equations for the
// individual motor thrusts given a desired total thrust and 3 torques:
struct MotorThrusts {
    double front, back, left, right;
};
MotorThrusts mixMotors(double totalThrust, double tauRoll, double tauPitch, double tauYaw) {
    MotorThrusts m;
    m.front = totalThrust / 4.0 - tauPitch / (2.0 * kArmLength) + tauYaw / (4.0 * kYawDragCoeff);
    m.back = totalThrust / 4.0 + tauPitch / (2.0 * kArmLength) + tauYaw / (4.0 * kYawDragCoeff);
    m.left = totalThrust / 4.0 + tauRoll / (2.0 * kArmLength) - tauYaw / (4.0 * kYawDragCoeff);
    m.right = totalThrust / 4.0 - tauRoll / (2.0 * kArmLength) - tauYaw / (4.0 * kYawDragCoeff);
    m.front = std::max(0.0, std::min(kMaxThrustPerMotor, m.front));
    m.back = std::max(0.0, std::min(kMaxThrustPerMotor, m.back));
    m.left = std::max(0.0, std::min(kMaxThrustPerMotor, m.left));
    m.right = std::max(0.0, std::min(kMaxThrustPerMotor, m.right));
    return m;
}

void stepRigidBody(RigidBodyState& s, const MotorThrusts& m, double dt) {
    // Recompute achieved thrust/torques from the (possibly saturated)
    // motor outputs, so actuator limits are physically consistent.
    const double totalThrust = m.front + m.back + m.left + m.right;
    const double tauRoll = kArmLength * (m.left - m.right);
    const double tauPitch = kArmLength * (m.back - m.front);
    const double tauYaw = kYawDragCoeff * (m.front + m.back - m.left - m.right);

    const Vec3 thrustWorld = rotateBodyToWorld(s.q, Vec3{0.0, 0.0, totalThrust});
    const Vec3 accel = thrustWorld * (1.0 / kMass) + Vec3{0.0, 0.0, -kGravity} - s.vel * (kLinearDrag / kMass);
    s.vel = s.vel + accel * dt;
    s.pos = s.pos + s.vel * dt;

    const Vec3 iOmega{kInertia.x * s.omega.x, kInertia.y * s.omega.y, kInertia.z * s.omega.z};
    const Vec3 gyroscopic = cross(s.omega, iOmega);
    const Vec3 tau{tauRoll, tauPitch, tauYaw};
    const Vec3 angularAccel{(tau.x - gyroscopic.x) / kInertia.x, (tau.y - gyroscopic.y) / kInertia.y,
                              (tau.z - gyroscopic.z) / kInertia.z};
    s.omega = s.omega + angularAccel * dt;

    const Quaternion omegaQuat{0.0, s.omega.x, s.omega.y, s.omega.z};
    const Quaternion qDot = (s.q * omegaQuat) * 0.5;
    s.q = s.q + qDot * dt;
    s.q.normalize();
}

// ---------------------------------------------------------------------
// Attitude loop: the fuzzy-gain-scheduled PID hybrid from the original
// single-axis demo, generalized to take its own gentle/aggressive gains
// per axis (roll/pitch share dynamics; yaw has different inertia and much
// less torque authority from the mixer, so it needs its own tuning).
// ---------------------------------------------------------------------

std::shared_ptr<MamdaniEngine> buildScheduler() {
    LinguisticVariable absError("absError", 0.0, 1.6);
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

class ComplementBlock final : public IBlock {
public:
    ComplementBlock(std::string sourceKey, std::string complementKey)
        : sourceKey_(std::move(sourceKey)), complementKey_(std::move(complementKey)) {}
    Signals evaluate(const Signals& inputs) override { return {{complementKey_, 1.0 - inputs.at(sourceKey_)}}; }
    std::string name() const override { return "complement"; }

private:
    std::string sourceKey_, complementKey_;
};

std::shared_ptr<Pipeline> buildAxisController(const std::shared_ptr<MamdaniEngine>& scheduler,
                                                 control::PIDController::Config gentleCfg,
                                                 control::PIDController::Config aggressiveCfg) {
    auto gentlePID = std::make_shared<PIDBlock>(gentleCfg, /*dt placeholder, overridden per-call via "dt"*/ 0.005);
    auto aggressivePID = std::make_shared<PIDBlock>(aggressiveCfg, 0.005);

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

}  // namespace

int main() {
    constexpr double kDt = 0.005;  // 200 Hz control/physics loop
    constexpr double kSimTime = 18.0;

    // Position/altitude loop: plain PID per axis, producing a desired
    // world-frame acceleration.
    control::PIDController::Config xyCfg;
    xyCfg.kp = 0.8;
    xyCfg.ki = 0.05;
    xyCfg.kd = 0.6;
    xyCfg.outputMin = -4.0;
    xyCfg.outputMax = 4.0;
    xyCfg.integralMin = -3.0;
    xyCfg.integralMax = 3.0;
    control::PIDController xPID(xyCfg);
    control::PIDController yPID(xyCfg);

    control::PIDController::Config zCfg;
    zCfg.kp = 3.0;
    zCfg.ki = 0.5;
    zCfg.kd = 2.5;
    zCfg.outputMin = -5.0;
    zCfg.outputMax = 5.0;
    zCfg.integralMin = -3.0;
    zCfg.integralMax = 3.0;
    control::PIDController zPID(zCfg);

    // Attitude loop: fuzzy-scheduled gentle/aggressive PID blend per axis.
    // Roll/pitch torque authority is +-(armLength * maxThrustPerMotor) =
    // ~1.2 N*m; yaw's is much smaller (~0.48 N*m via reactive drag), so it
    // gets its own, gentler gains.
    auto scheduler = buildScheduler();

    control::PIDController::Config rpGentle;
    rpGentle.kp = 0.35;
    rpGentle.ki = 0.05;
    rpGentle.kd = 0.05;
    rpGentle.outputMin = -1.2;
    rpGentle.outputMax = 1.2;
    rpGentle.integralMin = -0.5;
    rpGentle.integralMax = 0.5;
    control::PIDController::Config rpAggressive = rpGentle;
    rpAggressive.kp = 1.2;
    rpAggressive.kd = 0.12;

    control::PIDController::Config yawGentle;
    yawGentle.kp = 0.2;
    yawGentle.ki = 0.02;
    yawGentle.kd = 0.03;
    yawGentle.outputMin = -0.45;
    yawGentle.outputMax = 0.45;
    yawGentle.integralMin = -0.2;
    yawGentle.integralMax = 0.2;
    control::PIDController::Config yawAggressive = yawGentle;
    yawAggressive.kp = 0.6;
    yawAggressive.kd = 0.06;

    auto rollController = buildAxisController(scheduler, rpGentle, rpAggressive);
    auto pitchController = buildAxisController(scheduler, rpGentle, rpAggressive);
    auto yawController = buildAxisController(scheduler, yawGentle, yawAggressive);

    RigidBodyState state;
    state.pos = Vec3{0.0, 0.0, 0.0};

    // Waypoint schedule: climb straight up, then fly a diagonal leg while
    // holding a commanded yaw, then hold position.
    const Vec3 climbTarget{0.0, 0.0, 2.0};
    const Vec3 cruiseTarget{5.0, 4.0, 2.5};
    const double yawTarget = toRadians(30.0);

    auto positionSetpoint = [&](double t) -> Vec3 {
        if (t < 3.0) return climbTarget;
        if (t < 13.0) {
            const double frac = (t - 3.0) / 10.0;
            return climbTarget + (cruiseTarget - climbTarget) * frac;
        }
        return cruiseTarget;
    };
    auto yawSetpoint = [&](double t) -> double { return t < 1.0 ? 0.0 : yawTarget; };

    const int steps = static_cast<int>(kSimTime / kDt);
    const int printEvery = static_cast<int>(1.0 / kDt);

    std::cout << "Quadcopter 6DoF flight (position PID -> fuzzy-PID attitude -> motor mixer)\n\n";
    std::cout << "  time    x      y      z    roll(deg) pitch(deg) yaw(deg)\n";

    double maxTiltDeg = 0.0;
    for (int step = 0; step <= steps; ++step) {
        const double t = step * kDt;
        const Vec3 posSp = positionSetpoint(t);
        const double yawSp = yawSetpoint(t);

        const double axDes = xPID.update(posSp.x, state.pos.x, kDt);
        const double ayDes = yPID.update(posSp.y, state.pos.y, kDt);
        const double azDes = zPID.update(posSp.z, state.pos.z, kDt);

        const EulerAngles current = toEuler(state.q);

        // Desired thrust direction = normalize(desired_accel - gravity);
        // required thrust magnitude is how much force that takes.
        const Vec3 required{axDes, ayDes, azDes + kGravity};
        const double totalThrust = kMass * required.length();
        const Vec3 dir = required.normalized();

        // Small-angle inversion of "thrust direction depends on tilt and
        // current heading" (derived from the ZYX body-z-axis formula),
        // clamped to a sane envelope so the linearization stays valid.
        double rollDes = std::sin(current.yaw) * dir.x - std::cos(current.yaw) * dir.y;
        double pitchDes = std::cos(current.yaw) * dir.x + std::sin(current.yaw) * dir.y;
        constexpr double kMaxTiltRad = 0.5;  // ~28.6 degrees
        rollDes = std::max(-kMaxTiltRad, std::min(kMaxTiltRad, rollDes));
        pitchDes = std::max(-kMaxTiltRad, std::min(kMaxTiltRad, pitchDes));

        const double rollErr = std::fabs(rollDes - current.roll);
        const auto rollOut = rollController->evaluate(
            {{"setpoint", rollDes}, {"measurement", current.roll}, {"absError", rollErr}, {"dt", kDt}});
        const double pitchErr = std::fabs(pitchDes - current.pitch);
        const auto pitchOut = pitchController->evaluate(
            {{"setpoint", pitchDes}, {"measurement", current.pitch}, {"absError", pitchErr}, {"dt", kDt}});
        const double yawErr = std::fabs(yawSp - current.yaw);
        const auto yawOut = yawController->evaluate(
            {{"setpoint", yawSp}, {"measurement", current.yaw}, {"absError", yawErr}, {"dt", kDt}});

        const MotorThrusts motors =
            mixMotors(totalThrust, rollOut.at("torque"), pitchOut.at("torque"), yawOut.at("torque"));
        stepRigidBody(state, motors, kDt);

        maxTiltDeg = std::max({maxTiltDeg, std::fabs(toDegrees(current.roll)), std::fabs(toDegrees(current.pitch))});

        if (step % printEvery == 0) {
            std::cout << "  " << std::fixed << std::setprecision(1) << std::setw(5) << t << "  " << std::setw(5)
                       << state.pos.x << "  " << std::setw(5) << state.pos.y << "  " << std::setw(5) << state.pos.z
                       << "    " << std::setw(6) << toDegrees(current.roll) << "    " << std::setw(6)
                       << toDegrees(current.pitch) << "   " << std::setw(6) << toDegrees(current.yaw) << "\n";
        }
    }

    const EulerAngles finalAngles = toEuler(state.q);
    std::cout << "\nFinal position: (" << state.pos.x << ", " << state.pos.y << ", " << state.pos.z << ")"
               << "  target: (" << cruiseTarget.x << ", " << cruiseTarget.y << ", " << cruiseTarget.z << ")\n";
    std::cout << "Position error: " << (cruiseTarget - state.pos).length() << " m\n";
    std::cout << "Final yaw: " << toDegrees(finalAngles.yaw) << " deg (target " << toDegrees(yawTarget) << " deg)\n";
    std::cout << "Max tilt observed: " << maxTiltDeg << " deg -- "
               << (maxTiltDeg < 45.0 ? "stayed within a safe attitude envelope.\n" : "came dangerously close to flipping!\n");
    return 0;
}
