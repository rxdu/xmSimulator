/*
 * lidar.hpp — xmSim
 *
 * A 3D lidar that ray-casts a MujocoWorld from a named mount site. Pure CPU
 * (mj_multiRay) — no OpenGL, runs headless. Robot-agnostic: it needs only a site
 * to mount at and a group convention (the robot's own geoms in visual groups >0
 * so they are excluded unless self_hits). Output is a per-frame cloud in the
 * sensor frame, Livox-CustomMsg-shaped, with per-point time offsets for de-skew.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_LIDAR_LIDAR_HPP
#define XMSIM_LIDAR_LIDAR_HPP

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "xmsim/lidar/scan_pattern.hpp"
#include "xmsim/world.hpp"

namespace xmotion::sim::lidar {

struct LidarPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float intensity = 0.0f;
  float time = 0.0f;
  std::uint16_t line = 0;
  std::uint8_t tag = 0;
};

struct LidarFrame {
  std::vector<LidarPoint> points;
  double stamp = 0.0;
};

class Lidar {
 public:
  struct Config {
    std::string site = "lidar";
    LivoxMid360Pattern::Params pattern;
    double range_min = 0.1;
    double range_max = 40.0;
    double range_noise_std = 0.02;
    bool self_hits = false;
    std::uint64_t seed = 1;
  };

  Lidar(MujocoWorld& world, Config config);

  // Capture one frame at the world's current state (locks world.mutex()).
  // Advances the non-repetitive pattern one frame period per call.
  LidarFrame Capture(double stamp);

  int points_per_frame() const { return pattern_.PointsPerFrame(); }
  bool valid() const { return site_id_ >= 0; }

 private:
  MujocoWorld& world_;
  Config config_;
  LivoxMid360Pattern pattern_;
  int site_id_ = -1;
  double pattern_time_ = 0.0;
  std::mt19937_64 rng_;
  std::normal_distribution<double> range_noise_;
  std::vector<ScanRay> rays_;
  std::vector<double> vec_, dist_, normal_;
  std::vector<int> geomid_;
};

}  // namespace xmotion::sim::lidar

#endif  // XMSIM_LIDAR_LIDAR_HPP
