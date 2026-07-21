/*
 * test_camera.cpp — xmSim
 *
 * Renders the cart's onboard camera and checks the center-pixel depth against
 * the known front wall (x=3). Skips if no headless GL context is available.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <string>

#include "gtest/gtest.h"

#include "xmsim/camera/camera.hpp"
#include "xmsim/world.hpp"

namespace {

using xmotion::sim::MujocoWorld;
using xmotion::sim::camera::Camera;
using xmotion::sim::camera::CameraImage;

std::string CartModel() { return std::string(XMSIM_TEST_MODELS) + "/cart.xml"; }

TEST(Camera, RendersDepthOffTheWall) {
  MujocoWorld w;
  ASSERT_TRUE(w.Load(CartModel()));
  Camera cam(w, {});
  if (!cam.valid()) GTEST_SKIP() << "no headless GL context";

  EXPECT_EQ(cam.width(), 320);
  EXPECT_EQ(cam.height(), 240);
  const CameraImage img = cam.Capture(0.0);
  ASSERT_TRUE(img.ok);
  ASSERT_EQ(static_cast<int>(img.rgb.size()), 320 * 240 * 3);

  bool varied = false;
  for (std::size_t i = 3; i < img.rgb.size(); i += 3) {
    if (img.rgb[i] != img.rgb[0]) { varied = true; break; }
  }
  EXPECT_TRUE(varied) << "flat image";

  // Center looks straight ahead at wall_px (x=3); camera at x≈0.15 -> ~2.85 m.
  const int cx = 320 / 2, cy = 240 / 2;
  const float d = img.depth[static_cast<std::size_t>(cy) * 320 + cx];
  EXPECT_GT(d, 2.4f);
  EXPECT_LT(d, 3.3f);
}

}  // namespace
