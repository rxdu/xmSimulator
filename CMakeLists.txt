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

  set(XMSIM_TEST_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/test/models")
  foreach (t test_dynamics test_scan_pattern test_world_lidar)
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} PRIVATE xmSimulator gtest gtest_main)
    target_compile_definitions(${t} PRIVATE XMSIM_TEST_MODELS="${XMSIM_TEST_MODELS}")
    target_compile_options(${t} PRIVATE -Wall -Wextra)
    add_test(NAME ${t} COMMAND ${t})
  endforeach ()
  if (XMSIM_HAS_CAMERA)
    add_executable(test_camera test/test_camera.cpp)
    target_link_libraries(test_camera PRIVATE xmSimulator gtest gtest_main)
    target_compile_definitions(test_camera PRIVATE XMSIM_TEST_MODELS="${XMSIM_TEST_MODELS}")
    target_compile_options(test_camera PRIVATE -Wall -Wextra)
    add_test(NAME test_camera COMMAND test_camera)
  endif ()
endif ()
