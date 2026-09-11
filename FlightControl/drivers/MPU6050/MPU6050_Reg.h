#ifndef MPU6050_REG_H
#define MPU6050_REG_H

#ifndef __MY_MPU6050_REGS_H
#define __MY_MPU6050_REGS_H
// 设置位
#define mpu6050_wake_up 0x00
#define mpu6050_gyro_2kdps 0x18
#define mpu6050_accel_2g 0x00
#define mpu6050_dlpf_20hz 0x04
#define mpu6050_sample_rate_50hz 19
// 寄存器地址
#define mpu6050_accel_config 0x1C
#define mpu6050_gyro_config 0x1B
#define mpu6050_config 0x1A
#define mpu6050_pwm_wake_up 0x6B
#define mpu6050_hz 0x19
#define mpu6050_accel_out 0x3B
#define mpu6050_gyro_out 0x43
#define mpu6050_who_am_i 0x75


#endif

#endif // MPU6050_REG_H