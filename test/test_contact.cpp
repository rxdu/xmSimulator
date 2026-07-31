/*
 * test_contact.cpp — xmSimulator
 *
 * Freeze check for the contact / foot-contact force API. A legged robot needs to
 * know when a foot bears load; MujocoWorld::ContactNormalForce sums the true
 * contact normal force on a named body (via mj_contactForce, no declared sensor),
 * and a declared MJCF <touch> sensor reads the same quantity through the existing
 * SensorScalar path. A free body resting on a ground plane must report a normal
 * force ≈ its supported weight, and ≈ 0 once lifted clear of the plane.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "xmsim/world.hpp"

namespace {

using xmotion::sim::MujocoWorld;

std::string ContactModel() { return std::string(XMSIM_TEST_MODELS) + "/contact.xml"; }

constexpr double kWeight = 2.0 * 9.81;  // box mass * g (N)

TEST(Contact, NormalForceTracksGroundContact) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(ContactModel()));
  const int foot = w.Body("foot");
  const int touch = w.Sensor("foot_touch");
  ASSERT_GE(foot, 0);
  ASSERT_GE(touch, 0);

  // Airborne at start (the MJCF places the body well above the plane): both the
  // mj_contactForce path and the declared <touch> sensor read ~0.
  EXPECT_LT(w.ContactNormalForce(foot), 1e-6) << "airborne body has no contact";
  EXPECT_LT(w.SensorScalar(touch), 1e-6) << "touch sensor reads zero in the air";

  // Let it fall and settle; at rest the ground supports the body's weight.
  for (int i = 0; i < 800; ++i) w.StepOnce();
  const double f_rest = w.ContactNormalForce(foot);
  EXPECT_GT(f_rest, 1.0) << "resting body should press on the ground";
  EXPECT_NEAR(f_rest, kWeight, 3.0) << "normal force ≈ supported weight";
  // The declared <touch> sensor sees the same normal force via SensorScalar.
  EXPECT_NEAR(w.SensorScalar(touch), f_rest, 1.0)
      << "touch sensor and mj_contactForce agree at rest";
}

TEST(Contact, AbsentBodyReadsZero) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(ContactModel()));
  for (int i = 0; i < 50; ++i) w.StepOnce();
  // Body() returns -1 for an unknown name; the accessor must degrade to 0.
  EXPECT_EQ(w.ContactNormalForce(w.Body("does_not_exist")), 0.0);
}

}  // namespace
