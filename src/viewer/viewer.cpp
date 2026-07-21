/*
 * viewer.cpp — xmSim
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "xmsim/viewer/viewer.hpp"

#include <cctype>
#include <cstdio>
#include <mutex>

#include <GLFW/glfw3.h>

#include <mujoco/mujoco.h>

namespace xmotion::sim::viewer {
namespace {

// GLFW is a process-global library: glfwInit/glfwTerminate are refcounted here
// so two Viewers (or a Viewer outliving another) cannot tear the library out
// from under each other.
int g_glfw_users = 0;

bool AcquireGlfw() {
  if (g_glfw_users == 0 && glfwInit() != GLFW_TRUE) return false;
  ++g_glfw_users;
  return true;
}

void ReleaseGlfw() {
  if (g_glfw_users > 0 && --g_glfw_users == 0) glfwTerminate();
}

}  // namespace

struct Viewer::Impl {
  MujocoWorld* world = nullptr;
  Config config;
  GLFWwindow* window = nullptr;
  bool glfw_owned = false;
  bool valid = false;

  mjrContext con{};
  mjvScene scn{};
  mjvOption opt{};
  mjvCamera cam{};

  // Mouse drag state (window coords of the last cursor position).
  bool btn_left = false, btn_right = false;
  double last_x = 0.0, last_y = 0.0;

  void OnMouseMove(double x, double y) {
    const double dx = x - last_x, dy = y - last_y;
    last_x = x;
    last_y = y;
    if (!btn_left && !btn_right) return;
    int width = 1, height = 1;
    glfwGetWindowSize(window, &width, &height);
    // mjv_moveCamera takes motion RELATIVE to the window size.
    const int action = btn_left ? mjMOUSE_ROTATE_V : mjMOUSE_MOVE_V;
    mjv_moveCamera(world->model(), action, dx / height, dy / height, &scn, &cam);
  }

  void OnScroll(double yoffset) {
    mjv_moveCamera(world->model(), mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn, &cam);
  }

  // GLFW callbacks. They live INSIDE Impl (as statics, which convert to plain
  // function pointers) because Impl is a private nested type — a free function
  // in an anonymous namespace cannot name it.
  static Impl* Of(GLFWwindow* w) {
    return static_cast<Impl*>(glfwGetWindowUserPointer(w));
  }

  static void MouseButtonCb(GLFWwindow* w, int button, int action, int /*mods*/) {
    Impl* impl = Of(w);
    if (impl == nullptr) return;
    const bool down = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_LEFT) impl->btn_left = down;
    if (button == GLFW_MOUSE_BUTTON_RIGHT) impl->btn_right = down;
    glfwGetCursorPos(w, &impl->last_x, &impl->last_y);
  }

  static void CursorPosCb(GLFWwindow* w, double x, double y) {
    Impl* impl = Of(w);
    if (impl != nullptr) impl->OnMouseMove(x, y);
  }

  static void ScrollCb(GLFWwindow* w, double /*xoffset*/, double yoffset) {
    Impl* impl = Of(w);
    if (impl != nullptr) impl->OnScroll(yoffset);
  }
};

Viewer::Viewer(MujocoWorld& world, Config config) : impl_(std::make_unique<Impl>()) {
  impl_->world = &world;
  impl_->config = std::move(config);
  if (!world.loaded()) {
    std::fprintf(stderr, "[xmsim] viewer: world not loaded\n");
    return;
  }
  if (!AcquireGlfw()) {
    std::fprintf(stderr, "[xmsim] viewer: glfwInit failed (no display?)\n");
    return;
  }
  impl_->glfw_owned = true;

  impl_->window = glfwCreateWindow(impl_->config.width, impl_->config.height,
                                   impl_->config.title.c_str(), nullptr, nullptr);
  if (impl_->window == nullptr) {
    std::fprintf(stderr, "[xmsim] viewer: could not create a window/GL context\n");
    return;
  }
  glfwMakeContextCurrent(impl_->window);
  glfwSwapInterval(1);  // vsync — the render loop must not spin the GPU
  glfwSetWindowUserPointer(impl_->window, impl_.get());
  glfwSetMouseButtonCallback(impl_->window, &Impl::MouseButtonCb);
  glfwSetCursorPosCallback(impl_->window, &Impl::CursorPosCb);
  glfwSetScrollCallback(impl_->window, &Impl::ScrollCb);

  const mjModel* m = world.model();
  mjv_defaultScene(&impl_->scn);
  mjv_makeScene(m, &impl_->scn, 10000);
  mjv_defaultOption(&impl_->opt);
  mjv_defaultCamera(&impl_->cam);
  impl_->cam.azimuth = impl_->config.azimuth;
  impl_->cam.elevation = impl_->config.elevation;
  impl_->cam.distance = impl_->config.distance;

  // Follow the named body if it exists, so a driving robot stays in frame.
  if (!impl_->config.track_body.empty()) {
    const int body_id = world.Body(impl_->config.track_body);
    if (body_id >= 0) {
      impl_->cam.type = mjCAMERA_TRACKING;
      impl_->cam.trackbodyid = body_id;
    } else {
      std::fprintf(stderr, "[xmsim] viewer: no body '%s' — using a free camera\n",
                   impl_->config.track_body.c_str());
    }
  }

  mjr_defaultContext(&impl_->con);
  mjr_makeContext(m, &impl_->con, mjFONTSCALE_150);
  impl_->valid = true;
}

Viewer::~Viewer() {
  if (!impl_) return;
  if (impl_->window != nullptr) {
    glfwMakeContextCurrent(impl_->window);
    mjr_freeContext(&impl_->con);
    mjv_freeScene(&impl_->scn);
    glfwDestroyWindow(impl_->window);
  }
  if (impl_->glfw_owned) ReleaseGlfw();
}

bool Viewer::valid() const { return impl_ && impl_->valid; }

bool Viewer::ShouldClose() const {
  // An invalid viewer reports "closed" so `while (!v.ShouldClose())` loops in
  // callers exit immediately instead of spinning on a window that never opened.
  if (!impl_ || !impl_->valid) return true;
  return glfwWindowShouldClose(impl_->window) ||
         glfwGetKey(impl_->window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
}

void Viewer::PollEvents() {
  if (!impl_ || !impl_->valid) return;
  glfwPollEvents();
}

void Viewer::Render(const std::string& overlay_title, const std::string& overlay_body) {
  if (!impl_ || !impl_->valid) return;
  glfwMakeContextCurrent(impl_->window);

  int width = 0, height = 0;
  glfwGetFramebufferSize(impl_->window, &width, &height);
  mjrRect viewport = {0, 0, width, height};

  // Only mjv_updateScene touches mjData — hold the world lock just for that, so
  // a concurrent stepping thread is blocked for the copy and not the draw.
  {
    std::lock_guard<std::mutex> lock(impl_->world->mutex());
    mjv_updateScene(impl_->world->model(), impl_->world->data(), &impl_->opt, nullptr,
                    &impl_->cam, mjCAT_ALL, &impl_->scn);
  }

  mjr_render(viewport, &impl_->scn, &impl_->con);
  if (!overlay_title.empty() || !overlay_body.empty()) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, overlay_title.c_str(),
                overlay_body.c_str(), &impl_->con);
  }
  glfwSwapBuffers(impl_->window);
}

bool Viewer::KeyDown(char key) const {
  if (!impl_ || !impl_->valid) return false;
  const int code = std::toupper(static_cast<unsigned char>(key));
  return glfwGetKey(impl_->window, code) == GLFW_PRESS;
}

}  // namespace xmotion::sim::viewer
