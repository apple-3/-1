#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#ifdef __cplusplus

extern "C" {
#endif
typedef void *Motor_Handle;
Motor_Handle Motor_Create(TIM_HandleTypeDef *htim, uint32_t Channel);
void Motor_Start(Motor_Handle handle);
void Motor_Set_Speed(Motor_Handle handle, int16_t speed);
void Motor_Stop(Motor_Handle handle);
#ifdef __cplusplus
}
#endif

#endif // MOTOR_BRIDGE_H