cmake_minimum_required(VERSION 3.16)
project(xmSimulator VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# xmSimulator — the XMotion family's MuJoCo simulation library. Robot-agnostic:
# a MujocoWorld (step/actuate/sense by name) + reusable sensors (3D lidar with
# Livox-style non-repetitive patterns, RGB/depth camera) + actuator-dynamics
# models. Platform apps compose it into a robot-specific plant adapter; xmSimulator
# itself knows nothing about swerve/legged/tracked.
#
# Consumers link `xmotion::xmSimulator` and do NOT need mujoco.h (libmujoco is private).
# Host-x86 only (the sim never cross-compiles to a robot's embedded target).
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

option(XMSIM_BUILD_TESTS "Build xmSimulator tests" ON)

# --- MuJoCo (required) — prebuilt, fetched by scripts/setup/fetch_mujoco.sh ---
set(XMSIM_MUJOCO_DIR "${CMAKE_SOURCE_DIR}/third_party/mujoco"
    CACHE PATH "MuJoCo prebuilt SDK root (include/ + lib/)")
if (NOT EXISTS "${XMSIM_MUJOCO_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "xmSimulator: MuJoCo not found at ${XMSIM_MUJOCO_DIR} — "
      "run scripts/setup/fetch_mujoco.sh (or set XMSIM_MUJOCO_DIR)")
endif ()
list(APPEND CMAKE_BUILD_RPATH "${XMSIM_MUJOCO_DIR}/lib")

# --- EGL (optional) — headless GL for the camera -----------------------------
set(XMSIM_HAS_CAMERA OFF)
find_library(XMSIM_EGL_LIB NAMES EGL)
find_path(XMSIM_EGL_INC NAMES EGL/egl.h)
if (XMSIM_EGL_LIB AND XMSIM_EGL_INC)
  set(XMSIM_HAS_CAMERA ON)
  message(STATUS "xmSimulator: camera ENABLED (EGL ${XMSIM_EGL_LIB})")
else ()
  message(STATUS "xmSimulator: camera DISABLED — EGL not found (lidar still built)")
endif ()

# --- GLFW (optional) — onscreen GL for the interactive viewer ----------------
# Independent of EGL: a headless CI box gets the camera but no viewer, a desktop
# gets both. Prefer the installed CMake package; fall back to pkg-config.
set(XMSIM_HAS_VIEWER OFF)
find_package(glfw3 QUIET)
if (glfw3_FOUND)
  set(XMSIM_HAS_VIEWER ON)
  set(XMSIM_GLFW_TARGET glfw)
  message(STATUS "xmSimulator: viewer ENABLED (glfw3 ${glfw3_VERSION}, find_package)")
else ()
  find_package(PkgConfig QUIET)
  if (PkgConfig_FOUND)
    pkg_check_modules(XMSIM_GLFW QUIET glfw3)
  endif ()
  if (XMSIM_GLFW_FOUND)
    set(XMSIM_HAS_VIEWER ON)
    set(XMSIM_GLFW_TARGET PkgConfig::XMSIM_GLFW)
    message(STATUS "xmSimulator: viewer ENABLED (glfw3 ${XMSIM_GLFW_VERSION}, pkg-config)")
  else ()
    message(STATUS "xmSimulator: viewer DISABLED — GLFW not found (headless sim still built)")
  endif ()
endif ()

# ------------------------------- library ------------------------------------
add_library(xmSimulator STATIC src/world.cpp src/lidar/lidar.cpp)
add_library(xmotion::xmSimulator ALIAS xmSimulator)
target_include_directories(xmSimulator PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(xmSimulator SYSTEM PRIVATE "${XMSIM_MUJOCO_DIR}/include")
target_link_libraries(xmSimulator PRIVATE "${XMSIM_MUJOCO_DIR}/lib/libmujoco.so")
target_compile_options(xmSimulator PRIVATE -Wall -Wextra)
set_target_properties(xmSimulator PROPERTIES BUILD_RPATH "${XMSIM_MUJOCO_DIR}/lib")

if (XMSIM_HAS_CAMERA)
  target_sources(xmSimulator PRIVATE src/camera/camera.cpp)
  target_include_directories(xmSimulator SYSTEM PRIVATE "${XMSIM_EGL_INC}")
  target_link_libraries(xmSimulator PRIVATE "${XMSIM_EGL_LIB}")
  target_compile_definitions(xmSimulator PUBLIC XMSIM_HAS_CAMERA=1)
endif ()

if (XMSIM_HAS_VIEWER)
  target_sources(xmSimulator PRIVATE src/viewer/viewer.cpp)
  # PRIVATE: viewer.hpp exposes no GLFW type, so consumers never need its headers.
  target_link_libraries(xmSimulator PRIVATE ${XMSIM_GLFW_TARGET})
  target_compile_definitions(xmSimulator PUBLIC XMSIM_HAS_VIEWER=1)
endif ()

# Expose the optional-feature flags so a consumer added via add_subdirectory can
# gate its OWN targets (e.g. a GUI app that needs the viewer). The compile
# definitions above only reach C++; these reach the parent's CMake scope.
set(XMSIM_HAS_VIEWER "${XMSIM_HAS_VIEWER}" CACHE INTERNAL "xmSimulator viewer built")
set(XMSIM_HAS_CAMERA "${XMSIM_HAS_CAMERA}" CACHE INTERNAL "xmSimulator camera built")

# Expose the MuJoCo lib dir so consumers can add it to their own rpath.
set(XMSIM_MUJOCO_LIB_DIR "${XMSIM_MUJOCO_DIR}/lib"
    CACHE INTERNAL "MuJoCo lib dir (for consumer rpath)")

# -------------------------------- tests -------------------------------------
if (XMSIM_BUILD_TESTS)
  enable_testing()
  set(XMSIM_GTEST_DIR "${CMAKE_SOURCE_DIR}/third_party/googletest"
      CACHE PATH "googletest checkout")
  if (NOT TARGET gtest)
    if (EXISTS "${XMSIM_GTEST_DIR}/CMakeLists.txt")
      add_subdirectory("${XMSIM_GTEST_DIR}" ${CMAKE_BINARY_DIR}/_deps/googletest EXCLUDE_FROM_ALL)
    else ()
      find_package(GTest REQUIRED)
    endif ()
  endif ()
  # Bare targets from an add_subdirectory checkout, namespaced from find_package.
  if (TARGET gtest)
    set(XMSIM_GTEST_LIBS gtest gtest_main)
  else ()
    set(XMSIM_GTEST_LIBS GTest::gtest GTest::gtest_main)
  endif ()

  set(XMSIM_TEST_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/test/models")
  foreach (t test_dynamics test_scan_pattern test_world_lidar test_torque_platform test_contact)
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} PRIVATE xmSimulator ${XMSIM_GTEST_LIBS})
    target_compile_definitions(${t} PRIVATE XMSIM_TEST_MODELS="${XMSIM_TEST_MODELS}")
    target_compile_options(${t} PRIVATE -Wall -Wextra)
    add_test(NAME ${t} COMMAND ${t})
  endforeach ()
  if (XMSIM_HAS_CAMERA)
    add_executable(test_camera test/test_camera.cpp)
    target_link_libraries(test_camera PRIVATE xmSimulator ${XMSIM_GTEST_LIBS})
    target_compile_definitions(test_camera PRIVATE XMSIM_TEST_MODELS="${XMSIM_TEST_MODELS}")
    target_compile_options(test_camera PRIVATE -Wall -Wextra)
    add_test(NAME test_camera COMMAND test_camera)
  endif ()
endif ()
