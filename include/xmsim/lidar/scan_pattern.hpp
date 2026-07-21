/*
 * scan_pattern.hpp — xmSim
 *
 * Time-parameterized 3D lidar scan patterns. A pattern turns a timestamp into a
 * unit ray direction (sensor frame: x forward, y left, z up); Lidar casts an
 * mj_ray along each. The Livox Mid-360 model reproduces NON-REPETITIVE scanning
 * (consecutive frames sample different directions, so coverage grows with
 * integration time) with mutually incommensurate azimuth/elevation oscillators.
 * Specs modeled: 360°×59° (-7°..+52°), 200 000 pts/s, 10 Hz. The weave is a
 * behavioral approximation; the same interface accepts a device-captured angle
 * table (a CSV pattern) for exact fidelity.
 *
 * Pure, header-only, no MuJoCo dependency.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_LIDAR_SCAN_PATTERN_HPP
#define XMSIM_LIDAR_SCAN_PATTERN_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace xmotion::sim::lidar {

struct ScanRay {
  std::array<double, 3> dir{1.0, 0.0, 0.0};
  double time_offset = 0.0;
  std::uint16_t line = 0;
};

class ScanPattern {
 public:
  virtual ~ScanPattern() = default;
  virtual int PointsPerFrame() const = 0;
  virtual double FramePeriod() const = 0;
  virtual void Frame(double t_start, std::vector<ScanRay>& out) const = 0;
};

class LivoxMid360Pattern final : public ScanPattern {
 public:
  struct Params {
    double point_rate = 200000.0;
    double frame_rate = 10.0;
    double el_min = -7.0 * M_PI / 180.0;
    double el_max = 52.0 * M_PI / 180.0;
    double spin_hz = 47.3;
    double el_hz1 = 73.0;
    double el_hz2 = 119.0;
    double az_wobble = 0.10;
    double wobble_hz = 57.0;
  };

  LivoxMid360Pattern() = default;
  explicit LivoxMid360Pattern(Params params) : p_(params) {}

  int PointsPerFrame() const override {
    return static_cast<int>(std::lround(p_.point_rate / p_.frame_rate));
  }
  double FramePeriod() const override { return 1.0 / p_.frame_rate; }

  ScanRay At(double t) const {
    const double two_pi = 2.0 * M_PI;
    const double az =
        two_pi * p_.spin_hz * t + p_.az_wobble * std::sin(two_pi * p_.wobble_hz * t);
    const double e =
        0.5 * (std::sin(two_pi * p_.el_hz1 * t) + std::sin(two_pi * p_.el_hz2 * t + 1.0));
    const double el_c = 0.5 * (p_.el_max + p_.el_min);
    const double el_a = 0.5 * (p_.el_max - p_.el_min);
    const double el = el_c + el_a * e;
    const double ce = std::cos(el);
    ScanRay r;
    r.dir = {ce * std::cos(az), ce * std::sin(az), std::sin(el)};
    double frac = (el - p_.el_min) / (p_.el_max - p_.el_min);
    frac = std::min(0.999, std::max(0.0, frac));
    r.line = static_cast<std::uint16_t>(frac * 4.0);
    return r;
  }

  void Frame(double t_start, std::vector<ScanRay>& out) const override {
    const int n = PointsPerFrame();
    const double dt = 1.0 / p_.point_rate;
    out.clear();
    out.reserve(n);
    for (int k = 0; k < n; ++k) {
      const double dt_k = k * dt;
      ScanRay r = At(t_start + dt_k);
      r.time_offset = dt_k;
      out.push_back(r);
    }
  }

  const Params& params() const { return p_; }

 private:
  Params p_{};
};

}  // namespace xmotion::sim::lidar

#endif  // XMSIM_LIDAR_SCAN_PATTERN_HPP
