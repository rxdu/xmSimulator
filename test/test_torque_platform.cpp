/*
 * test_torque_platform.cpp — xmSimulator
 *
 * API-FREEZE check against a legged-style platform: a torque-controlled arm
 * (<motor> actuators, NO jointpos/jointvel sensors). It must be fully drivable
 * and readable through the same MujocoWorld — proving the world API covers
 * torque-joint robots (which read qpos/qvel/actuator_force directly) without
 * forcing sensor tags into their MJCF. This is the second real actuator scheme
 * (torque) alongside the cart's velocity servos and the swerve base's position
 * servos, which together freeze the world's command/read surface.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "xmsim/world.hpp"

namespace {

using xmotion::sim::MujocoWorld;

std::string ArmModel() { return std::string(XMSIM_TEST_MODELS) + "/arm.xml"; }

TEST(TorquePlatform, DrivesAndReadsWithoutJointSensors) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(ArmModel()));
  const int j1 = w.Joint("j1"), a1 = w.Actuator("j1");
  ASSERT_GE(j1, 0);
  ASSERT_GE(a1, 0);
  ASSERT_LT(w.Sensor("j1_pos"), 0) << "arm deliberately has NO jointpos sensor";

  const double p0 = w.JointPosition(j1);
  for (int i = 0; i < 400; ++i) {
    w.SetCtrl(a1, 1.0);  // constant torque
    w.StepOnce();
  }
  EXPECT_GT(std::abs(w.JointPosition(j1) - p0), 0.1) << "torque did not move the joint";
  EXPECT_NEAR(w.ActuatorForce(a1), 1.0, 0.2) << "motor force should equal ctrl (gear 1)";
  EXPECT_TRUE(std::isfinite(w.JointVelocity(j1)));
}

TEST(TorquePlatform, ImuAndGroundTruthGeneric) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(ArmModel()));
  for (int i = 0; i < 5; ++i) w.StepOnce();
  const auto gt = w.ground_truth("chassis");
  EXPECT_NEAR(gt.position[2], 0.6, 0.05) << "fixed-base pose read from kinematics";
  // The accessors work on this robot too; a rigidly-fixed base has no proper
  // acceleration in MuJoCo (gravity only shows through dynamics), so accel ~ 0
  // here — the ~1 g case is covered by the free-base cart (test_world_lidar).
  const auto imu = w.imu();
  for (int k = 0; k < 3; ++k) {
    EXPECT_TRUE(std::isfinite(imu.accel[k]));
    EXPECT_TRUE(std::isfinite(imu.gyro[k]));
    EXPECT_LT(std::abs(imu.gyro[k]), 0.1) << "static base should not rotate";
  }
}

}  // namespace
