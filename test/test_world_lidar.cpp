/*
 * test_world_lidar.cpp — xmSim
 *
 * SECOND-PLATFORM validation: a differential-drive cart (not a swerve robot)
 * driven entirely through the generic MujocoWorld verbs, with the same Lidar
 * class. Proves the world/sensor API is robot-agnostic — the whole point of
 * extracting xmSim. Checks: name lookup, SetCtrl/Step, sensor + IMU + ground
 * truth reads, and lidar returns off the walls with the robot excluded.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "xmsim/lidar/lidar.hpp"
#include "xmsim/world.hpp"

namespace {

using xmotion::sim::GroundTruth;
using xmotion::sim::ImuSample;
using xmotion::sim::MujocoWorld;
using xmotion::sim::lidar::Lidar;
using xmotion::sim::lidar::LidarFrame;

std::string CartModel() { return std::string(XMSIM_TEST_MODELS) + "/cart.xml"; }

TEST(MujocoWorld, LoadsAndResolvesNames) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(CartModel()));
  EXPECT_GT(w.timestep(), 0.0);
  EXPECT_GE(w.Actuator("left"), 0);
  EXPECT_GE(w.Actuator("right"), 0);
  EXPECT_GE(w.Sensor("left_vel"), 0);
  EXPECT_GE(w.Body("chassis"), 0);
  EXPECT_LT(w.Actuator("nonexistent"), 0);
}

TEST(MujocoWorld, DrivesCartForwardViaGenericVerbs) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(CartModel()));
  const int left = w.Actuator("left"), right = w.Actuator("right");
  const int lvel = w.Sensor("left_vel");
  const GroundTruth gt0 = w.ground_truth("chassis");
  // 8 rad/s * 0.05 m wheel ~ 0.4 m/s forward, for ~1.5 s.
  for (int i = 0; i < 750; ++i) {
    w.SetCtrl(left, 8.0);
    w.SetCtrl(right, 8.0);
    w.StepOnce();
  }
  EXPECT_NEAR(w.SensorScalar(lvel), 8.0, 2.0) << "wheel not spinning at target";
  const GroundTruth gt = w.ground_truth("chassis");
  EXPECT_GT(gt.position[0] - gt0.position[0], 0.1) << "cart did not move forward";

  const ImuSample imu = w.imu();
  const double acc_mag = std::sqrt(imu.accel[0] * imu.accel[0] +
                                   imu.accel[1] * imu.accel[1] +
                                   imu.accel[2] * imu.accel[2]);
  EXPECT_NEAR(acc_mag, 9.81, 3.0) << "accelerometer should sense ~1 g";
}

TEST(MujocoWorld, LidarSeesWallsNotRobot) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(CartModel()));
  Lidar lidar(w, {});
  ASSERT_TRUE(lidar.valid());
  const LidarFrame f = lidar.Capture(0.0);
  EXPECT_GT(f.points.size(), 500u);

  double max_x = -1e9, min_range = 1e9;
  bool saw_floor = false;
  for (const auto& p : f.points) {
    const double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_range = std::min(min_range, range);
    if (p.z < -0.1) saw_floor = true;
  }
  EXPECT_NEAR(max_x, 3.0, 0.4) << "front wall (x=3) range wrong";
  EXPECT_TRUE(saw_floor) << "no ground returns";
  EXPECT_GT(min_range, 0.2) << "hitting the robot's own body?";
}

}  // namespace
