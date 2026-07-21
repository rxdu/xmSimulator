# xmSimulator — the XMotion family's simulation library

A **robot-agnostic** MuJoCo simulation library: a world you step and sense by name, plus reusable sensors and actuator-dynamics models. Platform applications (swerve, legged, tracked) compose it into a robot-specific plant adapter; xmSimulator itself knows nothing about any particular robot.

It exists so the pieces that *are* platform-independent — the sensors especially — are written and maintained once instead of reinvented per robot.

## What's in it

| Piece | Header | Reuse |
|---|---|---|
| **MujocoWorld** | `xmsim/world.hpp` | load MJCF, `StepOnce`, `SetCtrl`, `SensorScalar`, `ground_truth`, `imu` — all by name | any robot |
| **3D lidar** | `xmsim/lidar/lidar.hpp` | ray-cast (`mj_multiRay`, CPU/headless) with pluggable `ScanPattern` | any robot |
| **Livox Mid-360 pattern** | `xmsim/lidar/scan_pattern.hpp` | non-repetitive scan (360°×59°, 200k pts/s); CSV-pluggable | device model |
| **RGB + depth camera** | `xmsim/camera/camera.hpp` | headless EGL offscreen render, metric depth | any robot |
| **Steering / drive models** | `xmsim/dynamics/*.hpp` | accel-limited steering profile, first-order drive | steered/driven wheels |

## The one design rule

**Consumers link `xmotion::xmSimulator` but never include `mujoco.h`.** `mjModel`/`mjData` are forward-declared and libmujoco is a *private* dependency, so a plant adapter compiles against a clean C++ surface. A platform maps its command seam onto the world's verbs:

```cpp
xmotion::sim::MujocoWorld world;
world.Load("robot.xml");
int left = world.Actuator("left"), right = world.Actuator("right");
world.SetCtrl(left, 8.0); world.SetCtrl(right, 8.0);
world.StepOnce();
auto gt = world.ground_truth("chassis");   // pose vs. estimator
xmotion::sim::lidar::Lidar lidar(world, {});       // same sensor, any robot
auto cloud = lidar.Capture(gt.time);
```

Two non-swerve platforms freeze the API: `test/models/cart.xml` (a differential-drive cart, **velocity** actuators, declared sensors) and `test/models/arm.xml` (a **torque** `<motor>` arm read via `qpos`/`qvel`/`actuator_force` with **no** joint sensors — the legged pattern). The *same* world and sensors that drive the swerve base drive both, so the command/read surface covers position, velocity, and torque control.

### Scene conventions

A robot MJCF that xmSimulator sensors expect: a body named `chassis`, an `imu` site with `<gyro>/<accelerometer>/<framequat>`, a `lidar` site, a `front` `<camera>`, the robot's own geoms in visual groups > 0 (so the lidar excludes them), and `<visual><global offwidth offheight>` for the camera.

## Build

```bash
scripts/setup/fetch_mujoco.sh            # prebuilt MuJoCo 3.9.0 -> third_party/mujoco
git submodule update --init third_party/googletest   # or set XMSIM_GTEST_DIR / system GTest
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
```

The camera builds only when EGL is present (`XMSIM_HAS_CAMERA`); the lidar and world are pure CPU and always build. Host-x86 only — the sim never cross-compiles to a robot's embedded target.

## Consuming it

```cmake
add_subdirectory(path/to/xmSimulator)      # or find_package(xmSimulator)
target_link_libraries(my_plant PUBLIC xmotion::xmSimulator)
```

See `swervebot_controller`'s `MujocoSwerveActuator` for a full adapter that maps a 4-module swerve seam onto the world, applies the steering/drive dynamics, and exposes `world()` for mounting sensors.
