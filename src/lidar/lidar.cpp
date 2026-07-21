/*
 * lidar.cpp — xmSim
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "xmsim/lidar/lidar.hpp"

#include <algorithm>
#include <cmath>

#include <mujoco/mujoco.h>

namespace xmotion::sim::lidar {

Lidar::Lidar(MujocoWorld& world, Config config)
    : world_(world),
      config_(std::move(config)),
      pattern_(config_.pattern),
      rng_(config_.seed),
      range_noise_(0.0, config_.range_noise_std) {
  if (world_.loaded()) {
    site_id_ = mj_name2id(world_.model(), mjOBJ_SITE, config_.site.c_str());
  }
  const int n = pattern_.PointsPerFrame();
  vec_.resize(static_cast<std::size_t>(n) * 3);
  dist_.resize(n);
  normal_.resize(static_cast<std::size_t>(n) * 3);
  geomid_.resize(n);
}

LidarFrame Lidar::Capture(double stamp) {
  LidarFrame frame;
  frame.stamp = stamp;
  if (site_id_ < 0) return frame;

  std::lock_guard<std::mutex> lock(world_.mutex());
  const mjModel* m = world_.model();
  mjData* d = world_.data();

  const double* pnt = &d->site_xpos[3 * site_id_];
  const double* R = &d->site_xmat[9 * site_id_];

  pattern_.Frame(pattern_time_, rays_);
  pattern_time_ += pattern_.FramePeriod();
  const int n = static_cast<int>(rays_.size());

  for (int k = 0; k < n; ++k) {
    const auto& dir = rays_[k].dir;
    vec_[3 * k + 0] = R[0] * dir[0] + R[1] * dir[1] + R[2] * dir[2];
    vec_[3 * k + 1] = R[3] * dir[0] + R[4] * dir[1] + R[5] * dir[2];
    vec_[3 * k + 2] = R[6] * dir[0] + R[7] * dir[1] + R[8] * dir[2];
  }

  mjtByte gg[mjNGROUP] = {1, 0, 0, 0, 0, 0};
  const mjtByte* geomgroup = config_.self_hits ? nullptr : gg;

  mj_multiRay(m, d, pnt, vec_.data(), geomgroup, /*flg_static=*/1,
              /*bodyexclude=*/-1, geomid_.data(), dist_.data(), normal_.data(), n,
              /*cutoff=*/config_.range_max);

  frame.points.reserve(n / 2);
  for (int k = 0; k < n; ++k) {
    double range = dist_[k];
    if (range < config_.range_min || range > config_.range_max) continue;
    if (config_.range_noise_std > 0.0) range += range_noise_(rng_);

    const auto& sd = rays_[k].dir;
    LidarPoint p;
    p.x = static_cast<float>(range * sd[0]);
    p.y = static_cast<float>(range * sd[1]);
    p.z = static_cast<float>(range * sd[2]);
    p.time = static_cast<float>(rays_[k].time_offset);
    p.line = rays_[k].line;
    p.tag = 0;

    double lum = 0.6;
    const int gid = geomid_[k];
    if (gid >= 0 && gid < m->ngeom) {
      const float* rgba = &m->geom_rgba[4 * gid];
      lum = 0.2126 * rgba[0] + 0.7152 * rgba[1] + 0.0722 * rgba[2];
    }
    const double incidence = std::abs(vec_[3 * k + 0] * normal_[3 * k + 0] +
                                      vec_[3 * k + 1] * normal_[3 * k + 1] +
                                      vec_[3 * k + 2] * normal_[3 * k + 2]);
    p.intensity = static_cast<float>(
        std::clamp(255.0 * lum * std::max(0.1, incidence), 0.0, 255.0));
    frame.points.push_back(p);
  }
  return frame;
}

}  // namespace xmotion::sim::lidar
