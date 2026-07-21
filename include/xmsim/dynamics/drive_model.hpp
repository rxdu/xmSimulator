/*
 * drive_model.hpp — xmSim
 *
 * First-order + acceleration-bounded transient for a wheel's ground speed.
 * Reusable for any driven wheel; shape it with a step-response capture per robot.
 *
 * Pure, header-only, no MuJoCo dependency.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_DYNAMICS_DRIVE_MODEL_HPP
#define XMSIM_DYNAMICS_DRIVE_MODEL_HPP

#include <algorithm>
#include <cmath>

namespace xmotion::sim {

class DriveModel {
 public:
  struct Params {
    double tau = 0.04;       // first-order time constant (s)
    double max_accel = 8.0;  // ground-speed acceleration bound (m/s²)
  };

  DriveModel() = default;
  explicit DriveModel(Params params, double initial_speed_mps = 0.0)
      : params_(params), speed_(initial_speed_mps) {}

  void Step(double target_mps, double dt) {
    if (dt <= 0.0) return;
    const double alpha =
        params_.tau > 0.0 ? (1.0 - std::exp(-dt / params_.tau)) : 1.0;
    double desired = speed_ + alpha * (target_mps - speed_);
    const double dv_max = params_.max_accel * dt;
    desired = std::clamp(desired, speed_ - dv_max, speed_ + dv_max);
    speed_ = desired;
  }

  double speed() const { return speed_; }
  void Reset(double speed_mps) { speed_ = speed_mps; }

 private:
  Params params_{};
  double speed_ = 0.0;
};

}  // namespace xmotion::sim

#endif  // XMSIM_DYNAMICS_DRIVE_MODEL_HPP
