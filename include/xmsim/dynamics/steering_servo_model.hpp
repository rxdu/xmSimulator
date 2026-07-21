/*
 * steering_servo_model.hpp — xmSim
 *
 * Single-axis, acceleration-limited, velocity-capped motion-profile follower —
 * the dynamic model of a servo that tracks a commanded angle under a trapezoidal
 * motion profile (arriving with no overshoot). Reusable for any steered actuator
 * (swerve module, steering column). Defaults are the swervebot bench numbers;
 * override per robot.
 *
 * Pure, header-only, no MuJoCo dependency.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_DYNAMICS_STEERING_SERVO_MODEL_HPP
#define XMSIM_DYNAMICS_STEERING_SERVO_MODEL_HPP

#include <algorithm>
#include <cmath>

namespace xmotion::sim {

class SteeringServoModel {
 public:
  struct Params {
    double max_accel = 6.5;   // rad/s²
    double max_rate = 3.316;  // rad/s
  };

  SteeringServoModel() = default;
  explicit SteeringServoModel(Params params, double initial_angle_rad = 0.0)
      : params_(params), angle_(initial_angle_rad) {}

  // Advance dt seconds toward target_rad. Caps the profile-commanded rate at the
  // speed from which we can still decelerate to a stop at the target
  // (v_stop = sqrt(2·a·|error|)); an overshoot guard snaps to the target under
  // finite dt. Direction reversal falls out of the same law.
  void Step(double target_rad, double dt) {
    if (dt <= 0.0) return;
    const double error = target_rad - angle_;
    const double dir = static_cast<double>((error > 0.0) - (error < 0.0));
    const double v_stop = std::sqrt(2.0 * params_.max_accel * std::abs(error));
    const double v_des = dir * std::min(params_.max_rate, v_stop);
    const double dv_max = params_.max_accel * dt;
    rate_ += std::clamp(v_des - rate_, -dv_max, dv_max);
    angle_ += rate_ * dt;
    if (dir != 0.0 && (target_rad - angle_) * dir < 0.0) {
      angle_ = target_rad;
      rate_ = 0.0;
    }
  }

  double angle() const { return angle_; }
  double rate() const { return rate_; }
  void Reset(double angle_rad, double rate_rad_s = 0.0) {
    angle_ = angle_rad;
    rate_ = rate_rad_s;
  }

 private:
  Params params_{};
  double angle_ = 0.0;
  double rate_ = 0.0;
};

}  // namespace xmotion::sim

#endif  // XMSIM_DYNAMICS_STEERING_SERVO_MODEL_HPP
