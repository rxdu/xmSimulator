/*
 * world.hpp — xmSim
 *
 * MujocoWorld: a robot-agnostic wrapper around a MuJoCo model. It owns the
 * mjModel/mjData, loads an MJCF, steps physics, and exposes actuators / joints /
 * sensors / body pose / IMU by NAME — with no notion of any particular robot.
 * Platform-specific plants (a swerve actuator, a legged joint-torque adapter)
 * compose this: they map their command seam onto SetCtrl + StepOnce and read
 * back through the sensor accessors. Sensors (Lidar, Camera) take a MujocoWorld&.
 *
 * Consumers of xmSim link the library but do NOT need mujoco.h — mjModel/mjData
 * are forward-declared and libmujoco is a private dependency. model()/data() are
 * provided for advanced use (a viewer) only.
 *
 * THREADING: MujocoWorld is not internally locked. It exposes mutex(); a caller
 * that steps or reads from one thread while another reads (e.g. a sensor capture
 * thread) must hold mutex() around the access. The bundled plants/sensors do so.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMSIM_WORLD_HPP
#define XMSIM_WORLD_HPP

#include <array>
#include <mutex>
#include <string>

struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

namespace xmotion::sim {

// Noiseless world-frame chassis state (a "ground truth" reference).
struct GroundTruth {
  double time = 0.0;
  std::array<double, 3> position{};     // world x,y,z (m)
  std::array<double, 4> orientation{};  // quat w,x,y,z
  std::array<double, 3> linear_vel{};   // world m/s
  std::array<double, 3> angular_vel{};  // world rad/s
};

// Simulated IMU (from <gyro>/<accelerometer>/<framequat> sensors at a site).
// Accelerometer is proper acceleration (includes gravity), body frame.
struct ImuSample {
  double time = 0.0;
  std::array<double, 3> gyro{};         // body rad/s
  std::array<double, 3> accel{};        // body m/s²
  std::array<double, 4> orientation{};  // quat w,x,y,z (body->world)
};

class MujocoWorld {
 public:
  MujocoWorld() = default;
  ~MujocoWorld();
  MujocoWorld(const MujocoWorld&) = delete;
  MujocoWorld& operator=(const MujocoWorld&) = delete;

  // Load an MJCF (scene). Returns false and logs on failure.
  bool Load(const std::string& model_path);
  bool loaded() const { return model_ != nullptr && data_ != nullptr; }

  double timestep() const;  // model opt.timestep (s)
  double time() const;      // data->time (s)
  void Reset();             // mj_resetData + mj_forward

  // Name -> index (-1 if absent). Cache these once; the lookups are string hashes.
  int Actuator(const std::string& name) const;
  int Sensor(const std::string& name) const;
  int Body(const std::string& name) const;

  // Stepping / actuation (caller holds mutex()).
  void SetCtrl(int actuator_id, double value);
  void StepOnce();  // one mj_step

  // Sensor reads by id (caller holds mutex()).
  double SensorScalar(int sensor_id) const;               // first channel
  void SensorVec(int sensor_id, double* out, int n) const;

  // Convenience state reads (caller holds mutex()). Body pose is read directly
  // from the kinematics (xpos/xquat) — no dedicated sensors needed. IMU needs
  // the named site sensors (defaults match the sim MJCF convention).
  GroundTruth ground_truth(const std::string& body = "chassis") const;
  ImuSample imu(const std::string& gyro = "imu_gyro",
                const std::string& accel = "imu_acc",
                const std::string& quat = "imu_quat") const;

  // Advanced access (viewers, sensors). Prefer the named API above.
  const mjModel* model() const { return model_; }
  mjData* data() { return data_; }
  std::mutex& mutex() const { return mutex_; }

 private:
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  mutable std::mutex mutex_;
};

}  // namespace xmotion::sim

#endif  // XMSIM_WORLD_HPP
