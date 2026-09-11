#include "MPU6050.hpp"
#include "MPU6050_Reg.h"
#include "iimu.hpp"

#include <cstdint>

static uint8_t _ADDR = 0x68 << 1;

MPU6050::MPU6050(mpu6050_write_reg_t Write_Func, mpu6050_read_reg_t Read_Func)
    : Hal_Write(Write_Func), Hal_Read(Read_Func) {}

bool MPU6050::init() {
  uint8_t addr_candidates[2] = {0x68 << 1, 0x69 << 1}; // 0xD0, 0xD2
  uint8_t who_am_i = 0;
  int found = 0;
  for (int i = 0; i < 2; i++) {
    for (int retry = 0; retry < 3; retry++) {
      if (Hal_Read(addr_candidates[i], mpu6050_who_am_i, &who_am_i, 1) == 0) {
        if (who_am_i == 0x68) {
          _ADDR = addr_candidates[i];
          found = 1;
          break;
        }
      }
      for (volatile uint32_t j = 0; j < 10000; j++)
        ;
    }
    if (found)
      break;
  }

  if (!found) {
    return false;
  }

  uint8_t data;
  data = mpu6050_wake_up;
  Hal_Write(_ADDR, mpu6050_pwm_wake_up, &data, 1);

  // 等待 MPU6050 从睡眠中唤醒
  for (volatile uint32_t i = 0; i < 100000; i++)
    ;

  data = mpu6050_gyro_2kdps;
  Hal_Write(_ADDR, mpu6050_gyro_config, &data, 1);
  data = mpu6050_accel_2g;
  Hal_Write(_ADDR, mpu6050_accel_config, &data, 1);
  data = mpu6050_dlpf_20hz;
  Hal_Write(_ADDR, mpu6050_config, &data, 1);
  data = mpu6050_sample_rate_50hz;
  Hal_Write(_ADDR, mpu6050_hz, &data, 1);
  return true;
}
bool MPU6050::read(imusample &imu) {
  uint8_t Buf[6] = {0};
  int16_t raw_ax, raw_ay, raw_az;
  if (Hal_Read(_ADDR, mpu6050_accel_out, Buf, 6) != 0) {
    imu.ax = imu.ay =imu.az = 0.0f;
    return false;
  }
  raw_ax = (int16_t)((Buf[0] << 8) | Buf[1]);
  raw_ay = (int16_t)((Buf[2] << 8) | Buf[3]);
  raw_az = (int16_t)((Buf[4] << 8) | Buf[5]);
  imu.ax = (float)raw_ax / 16384.0f;
  imu.ay = (float)raw_ay / 16384.0f;
  imu.az = (float)raw_az / 16384.0f;
  return true;
}
void MPU6050::MPU6050_Read_Accel_Float(float &ax, float &ay, float &az) {
  uint8_t Buf[6] = {0};
  int16_t raw_ax, raw_ay, raw_az;
  if (Hal_Read(_ADDR, mpu6050_accel_out, Buf, 6) != 0) {
    ax = ay = az = 0.0f;
    return;
  }
  raw_ax = (int16_t)((Buf[0] << 8) | Buf[1]);
  raw_ay = (int16_t)((Buf[2] << 8) | Buf[3]);
  raw_az = (int16_t)((Buf[4] << 8) | Buf[5]);
  ax = (float)raw_ax / 16384.0f;
  ay = (float)raw_ay / 16384.0f;
  az = (float)raw_az / 16384.0f;
}

void MPU6050::MPU6050_Read_Gyro_Float(float &gx, float &gy, float &gz) {
  uint8_t Buf[6] = {0};
  int16_t raw_gx, raw_gy, raw_gz;
  if (Hal_Read(_ADDR, mpu6050_gyro_out, Buf, 6) != 0) {
    gx = gy = gz = 0.0f;
    return;
  }
  raw_gx = (int16_t)((Buf[0] << 8) | Buf[1]);
  raw_gy = (int16_t)((Buf[2] << 8) | Buf[3]);
  raw_gz = (int16_t)((Buf[4] << 8) | Buf[5]);
  gx = (float)raw_gx / 16.4f;
  gy = (float)raw_gy / 16.4f;
  gz = (float)raw_gz / 16.4f;
}
// ========== 零偏校准 ==========
// 等待无人机 accel 三轴连续 100 帧跳变 ≤ 400 视为平稳，
// 然后采集 accel + gyro 原始 ADC 平均值作为零偏。
void MPU6050::Zero_Offset() {
  uint8_t Buf[14] = {0}; // accel(6) + temp(2) + gyro(6) = 14
  int16_t prev_ax = 0, prev_ay = 0, prev_az = 0;
  int16_t cur_ax, cur_ay, cur_az;

  // ----- 阶段 1：判断平稳 -----
  // 先读取一帧作为上一帧基准
  if (Hal_Read(_ADDR, mpu6050_accel_out, Buf, 6) != 0)
    return;
  prev_ax = (int16_t)((Buf[0] << 8) | Buf[1]);
  prev_ay = (int16_t)((Buf[2] << 8) | Buf[3]);
  prev_az = (int16_t)((Buf[4] << 8) | Buf[5]);

  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int stable_count = 0;
  int sample_count = 0;
  const int required_stable = 100; // 连续 100 次平稳才算达标
  const int16_t threshold = 400;   // 单轴最大允许前后跳变

  while (stable_count < required_stable) {
    // 一次读出全部 14 字节：accel(6) + temp(2) + gyro(6)
    if (Hal_Read(_ADDR, mpu6050_accel_out, Buf, 14) != 0)
      return;

    cur_ax = (int16_t)((Buf[0] << 8) | Buf[1]);
    cur_ay = (int16_t)((Buf[2] << 8) | Buf[3]);
    cur_az = (int16_t)((Buf[4] << 8) | Buf[5]);

    int16_t cur_gx = (int16_t)((Buf[8] << 8) | Buf[9]);
    int16_t cur_gy = (int16_t)((Buf[10] << 8) | Buf[11]);
    int16_t cur_gz = (int16_t)((Buf[12] << 8) | Buf[13]);

    // 计算加速度差值（绝对值）
    int16_t diff_ax =
        (cur_ax > prev_ax) ? (cur_ax - prev_ax) : (prev_ax - cur_ax);
    int16_t diff_ay =
        (cur_ay > prev_ay) ? (cur_ay - prev_ay) : (prev_ay - cur_ay);
    int16_t diff_az =
        (cur_az > prev_az) ? (cur_az - prev_az) : (prev_az - cur_az);

    if (diff_ax <= threshold && diff_ay <= threshold && diff_az <= threshold) {
      // 平稳 —— 累加采样（accel + gyro 同时累积）
      stable_count++;
      sum_ax += cur_ax;
      sum_ay += cur_ay;
      sum_az += cur_az;
      sum_gx += cur_gx;
      sum_gy += cur_gy;
      sum_gz += cur_gz;
      sample_count++;
    } else {
      // 任意轴跳变超过阈值 → 不平稳，全部重置
      stable_count = 0;
      sample_count = 0;
      sum_ax = sum_ay = sum_az = 0;
      sum_gx = sum_gy = sum_gz = 0;
    }

    prev_ax = cur_ax;
    prev_ay = cur_ay;
    prev_az = cur_az;

    // 采样间隔
    for (volatile uint32_t j = 0; j < 10000; j++)
      ;
  }

  // ----- 阶段 2：计算零偏（原始 ADC 值）-----
  // 加速度计：静止时应当为 [0, 0, +1g]（1g = 16384）
  // 所以 X/Y 偏移就是平均值，Z 偏移 = 平均值 - 16384
  ax_offset_raw = (int16_t)(sum_ax / sample_count);
  ay_offset_raw = (int16_t)(sum_ay / sample_count);
  az_offset_raw = (int16_t)(sum_az / sample_count) - 16384;

  // 陀螺仪：静止时应当为 0，偏移就是平均值
  gx_offset_raw = (int16_t)(sum_gx / sample_count);
  gy_offset_raw = (int16_t)(sum_gy / sample_count);
  gz_offset_raw = (int16_t)(sum_gz / sample_count);
}

// ========== 加速度计校准读取（自动减去零偏）==========
void MPU6050::Read_Accel_Calibrated(float &ax, float &ay, float &az) {
  uint8_t Buf[6] = {0};
  if (Hal_Read(_ADDR, mpu6050_accel_out, Buf, 6) != 0) {
    ax = ay = az = 0.0f;
    return;
  }
  int16_t raw_ax = (int16_t)((Buf[0] << 8) | Buf[1]) - ax_offset_raw;
  int16_t raw_ay = (int16_t)((Buf[2] << 8) | Buf[3]) - ay_offset_raw;
  int16_t raw_az = (int16_t)((Buf[4] << 8) | Buf[5]) - az_offset_raw;

  ax = (float)raw_ax / 16384.0f; // ±2g → LSB/g
  ay = (float)raw_ay / 16384.0f;
  az = (float)raw_az / 16384.0f;
}

// ========== 陀螺仪校准读取（自动减去零偏）==========
void MPU6050::Read_Gyro_Calibrated(float &gx, float &gy, float &gz) {
  uint8_t Buf[6] = {0};
  if (Hal_Read(_ADDR, mpu6050_gyro_out, Buf, 6) != 0) {
    gx = gy = gz = 0.0f;
    return;
  }
  int16_t raw_gx = (int16_t)((Buf[0] << 8) | Buf[1]) - gx_offset_raw;
  int16_t raw_gy = (int16_t)((Buf[2] << 8) | Buf[3]) - gy_offset_raw;
  int16_t raw_gz = (int16_t)((Buf[4] << 8) | Buf[5]) - gz_offset_raw;

  gx = (float)raw_gx / 16.4f; // ±2000°/s → LSB/(°/s)
  gy = (float)raw_gy / 16.4f;
  gz = (float)raw_gz / 16.4f;
}