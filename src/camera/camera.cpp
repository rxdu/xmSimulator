/*
 * camera.cpp — xmSim
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "xmsim/camera/camera.hpp"

#include <cstdio>
#include <cstring>
#include <mutex>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <mujoco/mujoco.h>

namespace xmotion::sim::camera {
namespace {

bool CreateEglContext(int w, int h, EGLDisplay* dpy_out, EGLContext* ctx_out,
                      EGLSurface* surf_out) {
  *dpy_out = EGL_NO_DISPLAY;
  *ctx_out = EGL_NO_CONTEXT;
  *surf_out = EGL_NO_SURFACE;

  EGLDisplay dpy = EGL_NO_DISPLAY;
  auto query_devices =
      reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
  auto get_platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
      eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (query_devices && get_platform_display) {
    EGLDeviceEXT devices[16];
    EGLint num_devices = 0;
    if (query_devices(16, devices, &num_devices) == EGL_TRUE) {
      for (int i = 0; i < num_devices && dpy == EGL_NO_DISPLAY; ++i) {
        EGLDisplay d = get_platform_display(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
        EGLint major = 0, minor = 0;
        if (d != EGL_NO_DISPLAY && eglInitialize(d, &major, &minor) == EGL_TRUE) dpy = d;
      }
    }
  }
  if (dpy == EGL_NO_DISPLAY) {
    dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint major = 0, minor = 0;
    if (dpy == EGL_NO_DISPLAY || eglInitialize(dpy, &major, &minor) != EGL_TRUE) return false;
  }

  const EGLint cfg_attr[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RED_SIZE, 8,
                             EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_DEPTH_SIZE, 24,
                             EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
  EGLConfig config;
  EGLint num_config = 0;
  if (eglChooseConfig(dpy, cfg_attr, &config, 1, &num_config) != EGL_TRUE || num_config < 1)
    return false;
  const EGLint pbuf_attr[] = {EGL_WIDTH, w, EGL_HEIGHT, h, EGL_NONE};
  EGLSurface surf = eglCreatePbufferSurface(dpy, config, pbuf_attr);
  if (surf == EGL_NO_SURFACE) return false;
  if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) return false;
  EGLContext ctx = eglCreateContext(dpy, config, EGL_NO_CONTEXT, nullptr);
  if (ctx == EGL_NO_CONTEXT) return false;
  if (eglMakeCurrent(dpy, surf, surf, ctx) != EGL_TRUE) return false;

  *dpy_out = dpy;
  *ctx_out = ctx;
  *surf_out = surf;
  return true;
}

}  // namespace

struct Camera::Impl {
  MujocoWorld* world = nullptr;
  Config config;
  int w = 0, h = 0, cam_id = -1;
  double znear = 0.0, zfar = 0.0;
  bool valid = false;
  EGLDisplay dpy = EGL_NO_DISPLAY;
  EGLContext ctx = EGL_NO_CONTEXT;
  EGLSurface surf = EGL_NO_SURFACE;
  mjrContext con{};
  mjvScene scn{};
  mjvOption opt{};
  mjvCamera cam{};

  bool MakeCurrent() {
    return dpy != EGL_NO_DISPLAY && eglMakeCurrent(dpy, surf, surf, ctx) == EGL_TRUE;
  }
};

Camera::Camera(MujocoWorld& world, Config config) : impl_(std::make_unique<Impl>()) {
  impl_->world = &world;
  impl_->config = std::move(config);
  if (!world.loaded()) return;
  const mjModel* m = world.model();
  impl_->w = m->vis.global.offwidth;
  impl_->h = m->vis.global.offheight;
  impl_->cam_id = mj_name2id(m, mjOBJ_CAMERA, impl_->config.camera.c_str());
  if (impl_->cam_id < 0) {
    std::fprintf(stderr, "[xmsim] camera: no camera '%s' in model\n",
                 impl_->config.camera.c_str());
    return;
  }
  const double extent = m->stat.extent;
  impl_->znear = m->vis.map.znear * extent;
  impl_->zfar = m->vis.map.zfar * extent;

  if (!CreateEglContext(impl_->w, impl_->h, &impl_->dpy, &impl_->ctx, &impl_->surf)) {
    std::fprintf(stderr, "[xmsim] camera: no headless EGL/GL context (no GPU/Mesa?)\n");
    return;
  }
  mjv_defaultScene(&impl_->scn);
  mjv_makeScene(m, &impl_->scn, 10000);
  mjv_defaultOption(&impl_->opt);
  mjv_defaultCamera(&impl_->cam);
  impl_->cam.type = mjCAMERA_FIXED;
  impl_->cam.fixedcamid = impl_->cam_id;
  mjr_defaultContext(&impl_->con);
  mjr_makeContext(m, &impl_->con, mjFONTSCALE_150);
  mjr_setBuffer(mjFB_OFFSCREEN, &impl_->con);
  if (impl_->con.currentBuffer != mjFB_OFFSCREEN) {
    std::fprintf(stderr, "[xmsim] camera: offscreen framebuffer unavailable\n");
    return;
  }
  impl_->valid = true;
}

Camera::~Camera() {
  if (!impl_) return;
  if (impl_->dpy != EGL_NO_DISPLAY) {
    impl_->MakeCurrent();
    mjr_freeContext(&impl_->con);
    mjv_freeScene(&impl_->scn);
    eglMakeCurrent(impl_->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (impl_->ctx != EGL_NO_CONTEXT) eglDestroyContext(impl_->dpy, impl_->ctx);
    if (impl_->surf != EGL_NO_SURFACE) eglDestroySurface(impl_->dpy, impl_->surf);
    eglTerminate(impl_->dpy);
  }
}

CameraImage Camera::Capture(double stamp) {
  CameraImage img;
  img.stamp = stamp;
  if (!impl_->valid || !impl_->MakeCurrent()) return img;

  std::lock_guard<std::mutex> lock(impl_->world->mutex());
  const mjModel* m = impl_->world->model();
  mjData* d = impl_->world->data();
  const int w = impl_->w, h = impl_->h;
  mjrRect viewport = {0, 0, w, h};
  mjv_updateScene(m, d, &impl_->opt, nullptr, &impl_->cam, mjCAT_ALL, &impl_->scn);
  mjr_render(viewport, &impl_->scn, &impl_->con);

  std::vector<std::uint8_t> raw_rgb(static_cast<std::size_t>(w) * h * 3);
  std::vector<float> raw_depth;
  if (impl_->config.render_depth) raw_depth.resize(static_cast<std::size_t>(w) * h);
  mjr_readPixels(raw_rgb.data(), impl_->config.render_depth ? raw_depth.data() : nullptr,
                 viewport, &impl_->con);

  img.width = w;
  img.height = h;
  img.rgb.resize(raw_rgb.size());
  if (impl_->config.render_depth) img.depth.resize(raw_depth.size());
  const double znear = impl_->znear, zfar = impl_->zfar;
  for (int y = 0; y < h; ++y) {
    const int src = h - 1 - y;
    std::memcpy(&img.rgb[static_cast<std::size_t>(y) * w * 3],
                &raw_rgb[static_cast<std::size_t>(src) * w * 3],
                static_cast<std::size_t>(w) * 3);
    if (impl_->config.render_depth) {
      for (int x = 0; x < w; ++x) {
        const double dv = raw_depth[static_cast<std::size_t>(src) * w + x];
        const double metric = (dv >= 1.0) ? 0.0 : znear / (1.0 - dv * (1.0 - znear / zfar));
        img.depth[static_cast<std::size_t>(y) * w + x] = static_cast<float>(metric);
      }
    }
  }
  img.ok = true;
  return img;
}

bool Camera::valid() const { return impl_ && impl_->valid; }
int Camera::width() const { return impl_ ? impl_->w : 0; }
int Camera::height() const { return impl_ ? impl_->h : 0; }

}  // namespace xmotion::sim::camera
