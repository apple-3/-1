#ifndef PIDSET_BRIDGE_H
#define PIDSET_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif
typedef void *PID_Handle;
PID_Handle PID_Creat(float kp, float ki, float kd);
float F_PID_Up(PID_Handle angle_handle, PID_Handle gryo_handle, float setpoint,
               float measure, float rate, float dt);
float PID_Yaw_Update_Float(float setpoint, float measure, float dt);

#ifdef __cplusplus
}
#endif

#endif // PIDSET_BRIDGE_H