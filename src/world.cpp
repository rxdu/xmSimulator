/*
 * world.cpp — xmSim
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "xmsim/world.hpp"

#include <cstdio>

#include <mujoco/mujoco.h>

namespace xmotion::sim {

MujocoWorld::~MujocoWorld() {
  if (data_ != nullptr) mj_deleteData(data_);
  if (model_ != nullptr) mj_deleteModel(model_);
}

bool MujocoWorld::Load(const std::string& model_path) {
  char error[1024] = {0};
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    std::fprintf(stderr, "[xmsim] failed to load model '%s': %s\n",
                 model_path.c_str(), error);
    return false;
  }
  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    std::fprintf(stderr, "[xmsim] mj_makeData failed for '%s'\n", model_path.c_str());
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  return true;
}

double MujocoWorld::timestep() const { return model_ ? model_->opt.timestep : 0.0; }
double MujocoWorld::time() const { return data_ ? data_->time : 0.0; }

void MujocoWorld::Reset() {
  if (!loaded()) return;
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
}

int MujocoWorld::Actuator(const std::string& name) const {
  return loaded() ? mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str()) : -1;
}
int MujocoWorld::Sensor(const std::string& name) const {
  return loaded() ? mj_name2id(model_, mjOBJ_SENSOR, name.c_str()) : -1;
}
int MujocoWorld::Body(const std::string& name) const {
  return loaded() ? mj_name2id(model_, mjOBJ_BODY, name.c_str()) : -1;
}
int MujocoWorld::Joint(const std::string& name) const {
  return loaded() ? mj_name2id(model_, mjOBJ_JOINT, name.c_str()) : -1;
}

double MujocoWorld::JointPosition(int joint_id) const {
  if (joint_id < 0) return 0.0;
  return data_->qpos[model_->jnt_qposadr[joint_id]];
}
double MujocoWorld::JointVelocity(int joint_id) const {
  if (joint_id < 0) return 0.0;
  return data_->qvel[model_->jnt_dofadr[joint_id]];
}
double MujocoWorld::ActuatorForce(int actuator_id) const {
  if (actuator_id < 0) return 0.0;
  return data_->actuator_force[actuator_id];
}

double MujocoWorld::ContactNormalForce(int body_id) const {
  if (!loaded() || body_id < 0) return 0.0;
  double total = 0.0;
  mjtNum wrench[6] = {0};  // contact-frame [force(3), torque(3)]
  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact& c = data_->contact[i];
    // geom[k] is -1 for a flex contact; skip those (no rigid-geom body).
    const int b0 = c.geom[0] >= 0 ? model_->geom_bodyid[c.geom[0]] : -1;
    const int b1 = c.geom[1] >= 0 ? model_->geom_bodyid[c.geom[1]] : -1;
    if (b0 != body_id && b1 != body_id) continue;
    mj_contactForce(model_, data_, i, wrench);
    total += wrench[0];  // normal component (contact-frame axis 0, >= 0)
  }
  return total;
}

void MujocoWorld::SetCtrl(int actuator_id, double value) {
  if (actuator_id >= 0 && actuator_id < model_->nu) data_->ctrl[actuator_id] = value;
}

void MujocoWorld::StepOnce() { mj_step(model_, data_); }

void MujocoWorld::SetJointPosition(int joint_id, double value) {
  if (!loaded() || joint_id < 0) return;
  data_->qpos[model_->jnt_qposadr[joint_id]] = value;
}

void MujocoWorld::SetFreeBasePose(const std::string& body,
                                  const std::array<double, 3>& position,
                                  const std::array<double, 4>& orientation) {
  if (!loaded()) return;
  const int bid = mj_name2id(model_, mjOBJ_BODY, body.c_str());
  if (bid < 0) return;
  // A floating base is a free joint attached to the body (its first joint).
  const int jid = model_->body_jntadr[bid];
  if (jid < 0 || model_->jnt_type[jid] != mjJNT_FREE) return;
  const int adr = model_->jnt_qposadr[jid];
  for (int k = 0; k < 3; ++k) data_->qpos[adr + k] = position[k];
  for (int k = 0; k < 4; ++k) data_->qpos[adr + 3 + k] = orientation[k];
}

void MujocoWorld::Forward() {
  if (loaded()) mj_forward(model_, data_);
}

double MujocoWorld::SensorScalar(int sensor_id) const {
  if (sensor_id < 0) return 0.0;
  return data_->sensordata[model_->sensor_adr[sensor_id]];
}

void MujocoWorld::SensorVec(int sensor_id, double* out, int n) const {
  if (sensor_id < 0) {
    for (int i = 0; i < n; ++i) out[i] = 0.0;
    return;
  }
  const int adr = model_->sensor_adr[sensor_id];
  for (int i = 0; i < n; ++i) out[i] = data_->sensordata[adr + i];
}

GroundTruth MujocoWorld::ground_truth(const std::string& body) const {
  GroundTruth gt;
  if (!loaded()) return gt;
  gt.time = data_->time;
  const int bid = mj_name2id(model_, mjOBJ_BODY, body.c_str());
  if (bid < 0) return gt;
  for (int k = 0; k < 3; ++k) gt.position[k] = data_->xpos[3 * bid + k];
  for (int k = 0; k < 4; ++k) gt.orientation[k] = data_->xquat[4 * bid + k];
  // Spatial velocity of the body in world frame: [angular(3), linear(3)].
  mjtNum vel[6] = {0};
  mj_objectVelocity(model_, data_, mjOBJ_BODY, bid, vel, /*flg_local=*/0);
  for (int k = 0; k < 3; ++k) {
    gt.angular_vel[k] = vel[k];
    gt.linear_vel[k] = vel[3 + k];
  }
  return gt;
}

ImuSample MujocoWorld::imu(const std::string& gyro, const std::string& accel,
                           const std::string& quat) const {
  ImuSample s;
  if (!loaded()) return s;
  s.time = data_->time;
  SensorVec(Sensor(gyro), s.gyro.data(), 3);
  SensorVec(Sensor(accel), s.accel.data(), 3);
  SensorVec(Sensor(quat), s.orientation.data(), 4);
  return s;
}

}  // namespace xmotion::sim
