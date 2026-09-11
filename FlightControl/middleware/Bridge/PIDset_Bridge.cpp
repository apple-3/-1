#include "PIDset_Bridge.h"
#include "PIDset.hpp"
#include <vector>
#define MAX_PID 12
static std::vector<PID<float> *> F_PID;
static bool initialized = false;
void PID_Init() {
  if (!initialized) {
    F_PID.reserve(MAX_PID);
    initialized = true;
  }
}

static PID<float> F_yaw_rate_pid(2, 0, 0.05);

extern "C" {

PID_Handle PID_Creat(float kp, float ki, float kd) {
  PID_Init();
  if (F_PID.size() > MAX_PID)
    return nullptr;
  PID<float> *pid = new PID<float>(kp, ki, kd);
  F_PID.push_back(pid);
  return static_cast<PID_Handle>(pid);
}

float F_PID_Up(PID_Handle angle_handle, PID_Handle gryo_handle, float setpoint,
               float measure, float rate, float dt) {
  if (angle_handle == nullptr || gryo_handle == nullptr)
    return 0;
  PID<float> *angle_pid = static_cast<PID<float> *>(angle_handle);
  PID<float> *gryo_pid = static_cast<PID<float> *>(gryo_handle);
  angle_pid->Set_Setpoint(setpoint);
  return angle_pid->Update_Chain(*gryo_pid, measure, rate, dt);
}
float PID_Yaw_Update_Float(float setpoint, float measure, float dt) {
  F_yaw_rate_pid.Set_Setpoint(setpoint);
  return F_yaw_rate_pid.Update(measure, dt);
}
}
