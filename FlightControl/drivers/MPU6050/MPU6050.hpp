#ifndef MPU6050_H
#define MPU6050_H

#include "MPU6050_Reg.h"
#include "iimu.hpp"
#include <stdint.h>


typedef int (*mpu6050_write_reg_t)(uint8_t addr, uint8_t reg, uint8_t *data,
                                   uint16_t len);
typedef int (*mpu6050_read_reg_t)(uint8_t addr, uint8_t reg, uint8_t *data,
                                  uint16_t len);
class MPU6050 : public IImu{
private:
  mpu6050_write_reg_t Hal_Write;
  mpu6050_read_reg_t Hal_Read;

  // 原始 ADC 零偏（减去后得到真实值）
  int16_t ax_offset_raw = 0;
  int16_t ay_offset_raw = 0;
  int16_t az_offset_raw = 0;
  int16_t gx_offset_raw = 0;
  int16_t gy_offset_raw = 0;
  int16_t gz_offset_raw = 0;

public:
  bool init() override;
  bool read(imusample &sample) override;
  MPU6050(mpu6050_write_reg_t Write_Func, mpu6050_read_reg_t Read_Func);
  void MPU6050_Init();

  // 原始读取（无校准）
  void MPU6050_Read_Accel_Float(float &ax, float &ay, float &az);
  void MPU6050_Read_Gyro_Float(float &gx, float &gy, float &gz);

  // 校准后的读取（自动减去零偏）
  void Read_Accel_Calibrated(float &ax, float &ay, float &az);
  void Read_Gyro_Calibrated(float &gx, float &gy, float &gz);

  // 零偏校准：等待无人机平稳后采集 accel + gyro 原始偏移
  void Zero_Offset();

  // 获取原始 ADC 零偏值
  int16_t Get_Ax_Offset_Raw() const { return ax_offset_raw; }
  int16_t Get_Ay_Offset_Raw() const { return ay_offset_raw; }
  int16_t Get_Az_Offset_Raw() const { return az_offset_raw; }
  int16_t Get_Gx_Offset_Raw() const { return gx_offset_raw; }
  int16_t Get_Gy_Offset_Raw() const { return gy_offset_raw; }
  int16_t Get_Gz_Offset_Raw() const { return gz_offset_raw; }
};

#endif // MPU6050_H