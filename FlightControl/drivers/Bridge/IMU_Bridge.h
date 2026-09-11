#ifndef IMU_BRIDGE_H
#define IMU_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

void MPU_Init();
void MPU_Read_Accel_Float(float *ax, float *ay, float *az);
void MPU_Read_Gyro_Float(float *gx, float *gy, float *gz);
void MPU_Zero_Offset_Accel(float *ax, float *ay, float *az);
void MPU_Zero_Offset_Gyro(float *gx, float *gy, float *gz);
void MPU_Zero_Offset();
#ifdef __cplusplus
}
#endif

#endif // IMU_BRIDGE_H