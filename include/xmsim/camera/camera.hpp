/*
 * camera.hpp — xmSim
 *
 * Offscreen RGB + depth camera rendering a MujocoWorld from a named MJCF
 * <camera>, via a headless EGL OpenGL context. Depth is linearized to metres.
 * Robot-agnostic. Built only when EGL is found (XMSIM_HAS_CAMERA); valid() is
 * false if no GL context can be made at runtime.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_CAMERA_CAMERA_HPP
#define XMSIM_CAMERA_CAMERA_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "xmsim/world.hpp"

namespace xmotion::sim::camera {

struct CameraImage {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgb;  // width*height*3, row 0 = top
  std::vector<float> depth;       // width*height, metres
  double stamp = 0.0;
  bool ok = false;
};

class Camera {
 public:
  struct Config {
    std::string camera = "front";
    bool render_depth = true;
  };

  Camera(MujocoWorld& world, Config config);
  ~Camera();
  Camera(const Camera&) = delete;
  Camera& operator=(const Camera&) = delete;

  CameraImage Capture(double stamp);  // locks world.mutex()
  bool valid() const;
  int width() const;
  int height() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xmotion::sim::camera

#endif  // XMSIM_CAMERA_CAMERA_HPP
