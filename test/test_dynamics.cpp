/*
 * test_dynamics.cpp — xmSim
 *
 * Pins the actuator dynamics models: the accel-limited steering profile (no
 * overshoot, √Δθ peak-rate ~190°/s at 90°, ~0.9 s settle, reversal) and the
 * first-order + accel-bounded drive transient.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cmath>

#include "gtest/gtest.h"

#include "xmsim/dynamics/drive_model.hpp"
#include "xmsim/dynamics/steering_servo_model.hpp"

namespace {

using xmotion::sim::DriveModel;
using xmotion::sim::SteeringServoModel;

constexpr double kDeg = M_PI / 180.0;

struct StepResult {
  double peak_rate = 0.0, settle_s = -1.0, max_overshoot = 0.0, final_angle = 0.0;
};

StepResult RunStep(double target, double dt = 0.001, double t_max = 3.0) {
  SteeringServoModel s;
  StepResult r;
  bool settled = false;
  for (double t = 0.0; t < t_max; t += dt) {
    s.Step(target, dt);
    r.peak_rate = std::max(r.peak_rate, std::abs(s.rate()));
    r.max_overshoot = std::max(r.max_overshoot, s.angle() - target);
    if (!settled && std::abs(s.angle() - target) <= 2.0 * kDeg) {
      r.settle_s = t;
      settled = true;
    }
  }
  r.final_angle = s.angle();
  return r;
}

TEST(SteeringServoModel, NoOvershootReachesTarget) {
  const auto r = RunStep(90.0 * kDeg);
  EXPECT_NEAR(r.final_angle, 90.0 * kDeg, 1.0 * kDeg);
  EXPECT_LT(r.max_overshoot, 0.5 * kDeg);
}
TEST(SteeringServoModel, PeakRateAt90) {
  const auto r = RunStep(90.0 * kDeg);
  EXPECT_GT(r.peak_rate, 3.0);
  EXPECT_LT(r.peak_rate, 3.40);
}
TEST(SteeringServoModel, SqrtLaw) {
  EXPECT_NEAR(RunStep(90.0 * kDeg).peak_rate / RunStep(45.0 * kDeg).peak_rate,
              std::sqrt(2.0), 0.12);
}
TEST(SteeringServoModel, SettleBand) {
  const auto r = RunStep(90.0 * kDeg);
  ASSERT_GE(r.settle_s, 0.0);
  EXPECT_GT(r.settle_s, 0.6);
  EXPECT_LT(r.settle_s, 1.15);
}
TEST(SteeringServoModel, Reversal) {
  SteeringServoModel s;
  for (int i = 0; i < 1000; ++i) s.Step(60.0 * kDeg, 0.001);
  ASSERT_NEAR(s.angle(), 60.0 * kDeg, 1.0 * kDeg);
  for (int i = 0; i < 1500; ++i) s.Step(-60.0 * kDeg, 0.001);
  EXPECT_NEAR(s.angle(), -60.0 * kDeg, 1.5 * kDeg);
}

TEST(DriveModel, ApproachesTarget) {
  DriveModel d;
  for (int i = 0; i < 500; ++i) d.Step(0.5, 0.001);
  EXPECT_NEAR(d.speed(), 0.5, 1e-3);
}
TEST(DriveModel, RespectsAccelBound) {
  DriveModel d({0.0, 4.0});
  d.Step(1.0, 0.01);
  EXPECT_LE(d.speed(), 4.0 * 0.01 + 1e-9);
}
TEST(DriveModel, ZeroDtNoOp) {
  DriveModel d({}, 0.3);
  d.Step(1.0, 0.0);
  EXPECT_EQ(d.speed(), 0.3);
}

}  // namespace
