/*
 * viewer.hpp — xmSim
 *
 * Interactive GLFW window rendering a MujocoWorld — a live view of the physics
 * for development and demos. Robot-agnostic: it draws whatever the model holds
 * and knows nothing about swerve/legged/tracked. The counterpart to Camera:
 * Camera renders offscreen via EGL for sensing, Viewer renders onscreen via
 * GLFW for humans.
 *
 * Built only when GLFW is found (XMSIM_HAS_VIEWER). valid() is false when no
 * window or GL context can be created at runtime (headless box, no DISPLAY),
 * so callers degrade to headless instead of crashing — same contract as Camera.
 *
 * THREADING: GLFW is main-thread-only — construct, PollEvents() and Render()
 * from the thread that owns the window. Render() holds world.mutex() only while
 * it copies mjData into the scene, so a separate stepping thread is safe; a
 * single-threaded step-then-render loop is simpler and equally correct.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_VIEWER_VIEWER_HPP
#define XMSIM_VIEWER_VIEWER_HPP

#include <memory>
#include <string>

#include "xmsim/world.hpp"

namespace xmotion::sim::viewer {

class Viewer {
 public:
  struct Config {
    std::string title = "xmSim viewer";
    int width = 1200;
    int height = 900;
    // When this names a body in the model the camera follows it
    // (mjCAMERA_TRACKING) — the right default for a mobile robot that drives
    // out of frame. Empty (or an unknown name) leaves a free camera.
    std::string track_body = "";
    double azimuth = 120.0;    // deg, initial free/tracking camera framing
    double elevation = -20.0;  // deg
    double distance = 2.5;     // m
  };

  Viewer(MujocoWorld& world, Config config);
  ~Viewer();
  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;

  // False if the window/GL context could not be created; every other method is
  // then a safe no-op (ShouldClose() returns true so loops exit immediately).
  bool valid() const;

  // True once the user closes the window or presses ESC.
  bool ShouldClose() const;

  // Pump the event queue (keyboard, mouse, close). Call once per frame BEFORE
  // reading KeyDown().
  void PollEvents();

  // Draw one frame. Locks world.mutex() while updating the scene from mjData.
  // Both overlay strings are optional and render top-left as a label/value pair
  // of columns ('\n'-separated lines, laid out row-wise by mjr_overlay).
  void Render(const std::string& overlay_title = "",
              const std::string& overlay_body = "");

  // Level-triggered key state, queried by character: KeyDown('W'), KeyDown(' ').
  // Letters, digits and space map straight through because GLFW's key codes for
  // them ARE their uppercase-ASCII values — which is what keeps GLFW out of the
  // consumer's include path. Lower-case input is upper-cased here.
  bool KeyDown(char key) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xmotion::sim::viewer

#endif  // XMSIM_VIEWER_VIEWER_HPP
