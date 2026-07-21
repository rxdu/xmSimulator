/*
 * test_scan_pattern.cpp — xmSim
 *
 * Pins the Livox Mid-360 pattern: rate, FOV, unit rays, and non-repetitive
 * coverage growth (more frames -> more angular cells).
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cmath>
#include <set>
#include <vector>

#include "gtest/gtest.h"

#include "xmsim/lidar/scan_pattern.hpp"

namespace {

using xmotion::sim::lidar::LivoxMid360Pattern;
using xmotion::sim::lidar::ScanRay;

constexpr double kDeg = M_PI / 180.0;

TEST(LivoxMid360Pattern, RateAndFrame) {
  LivoxMid360Pattern p;
  EXPECT_EQ(p.PointsPerFrame(), 20000);
  EXPECT_NEAR(p.FramePeriod(), 0.1, 1e-12);
}

TEST(LivoxMid360Pattern, RaysAreUnitAndInFov) {
  LivoxMid360Pattern p;
  std::vector<ScanRay> rays;
  p.Frame(0.0, rays);
  ASSERT_EQ(static_cast<int>(rays.size()), p.PointsPerFrame());
  double az_min = 1e9, az_max = -1e9;
  for (const auto& r : rays) {
    const double n = std::sqrt(r.dir[0] * r.dir[0] + r.dir[1] * r.dir[1] +
                               r.dir[2] * r.dir[2]);
    EXPECT_NEAR(n, 1.0, 1e-9);
    const double el = std::asin(r.dir[2]);
    EXPECT_GE(el, -7.0 * kDeg - 1e-6);
    EXPECT_LE(el, 52.0 * kDeg + 1e-6);
    const double az = std::atan2(r.dir[1], r.dir[0]);
    az_min = std::min(az_min, az);
    az_max = std::max(az_max, az);
  }
  EXPECT_GT(az_max - az_min, 6.0);
}

TEST(LivoxMid360Pattern, NonRepetitiveCoverageGrows) {
  auto cells = [](int frames) {
    LivoxMid360Pattern p;
    std::set<int> s;
    std::vector<ScanRay> rays;
    for (int f = 0; f < frames; ++f) {
      p.Frame(f * p.FramePeriod(), rays);
      for (const auto& r : rays) {
        const double az = std::atan2(r.dir[1], r.dir[0]);
        const double el = std::asin(r.dir[2]);
        const int ai = static_cast<int>((az + M_PI) / (2 * M_PI) * 360);
        const int ei = static_cast<int>((el + 10 * kDeg) / kDeg);
        s.insert(ai * 1000 + ei);
      }
    }
    return s.size();
  };
  EXPECT_GT(cells(10), cells(1) * 3 / 2);
}

}  // namespace
