#include "IMU_Bridge.h"
#include "MPU6050.hpp"
#include "i2c.h"
#include "stm32f4xx_hal_def.h"

// I2C1 引脚：PB6 = SCL, PB7 = SDA（见 Core/Src/i2c.c）
#define IMU_I2C_PORT GPIOB
#define IMU_I2C_SCL_PIN GPIO_PIN_6
#define IMU_I2C_SDA_PIN GPIO_PIN_7

// 恢复限流：设备本来就不在线时，不要每次读写失败都去复位总线
#define IMU_I2C_RECOVER_MIN_INTERVAL_MS 100U

// 累计恢复次数，调试时可以直接看这个值
volatile uint32_t IMU_I2C_Recover_Count = 0;
static uint32_t imu_i2c_last_recover_tick = 0;

// I2C 总线卡死恢复。
// 现象：HAL_I2C_Mem_Read/Write 开头的"等 BUSY 标志复位"超时
//       （HAL 内部固定 I2C_TIMEOUT_BUSY_FLAG = 25ms），返回 HAL_BUSY，
//       且不会自己恢复，之后每次访问都同样失败。
// 原因通常是传输中途被打断（在 I2C 传输里下断点/复位），
// 或者从机(MPU6050)抓着 SDA 不放，MCU 误判为总线忙。
static void IMU_I2C_Bus_Recover(void) {
  uint32_t now = HAL_GetTick();
  if ((now - imu_i2c_last_recover_tick) < IMU_I2C_RECOVER_MIN_INTERVAL_MS) {
    return;
  }
  imu_i2c_last_recover_tick = now;

  GPIO_InitTypeDef gpio = {0};

  // 1) 复位 I2C 外设，清掉锁死的 BUSY 状态
  HAL_I2C_DeInit(&hi2c1);

  // 2) 在 SCL 上打 9 个时钟，把卡在字节中间的从机顶出去
  gpio.Pin = IMU_I2C_SCL_PIN | IMU_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_I2C_PORT, &gpio);
  HAL_GPIO_WritePin(IMU_I2C_PORT, IMU_I2C_SCL_PIN | IMU_I2C_SDA_PIN,
                    GPIO_PIN_SET);

  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(IMU_I2C_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 200; d++) {
      __NOP();
    }
    HAL_GPIO_WritePin(IMU_I2C_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
    for (volatile int d = 0; d < 200; d++) {
      __NOP();
    }
  }

  // 3) 补一个 STOP 条件：SCL 为高时 SDA 由低跳高
  HAL_GPIO_WritePin(IMU_I2C_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_RESET);
  for (volatile int d = 0; d < 200; d++) {
    __NOP();
  }
  HAL_GPIO_WritePin(IMU_I2C_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
  for (volatile int d = 0; d < 200; d++) {
    __NOP();
  }
  HAL_GPIO_WritePin(IMU_I2C_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);

  // 4) 重新初始化外设，并把 PB6/PB7 配回复用功能
  MX_I2C1_Init();
  IMU_I2C_Recover_Count++;
}

// 读写统一封装：失败就尝试恢复总线，避免 BUSY 卡死后永久失效
static int IMU_I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data,
                         uint16_t len) {
  int ret = HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data,
                              len, HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    IMU_I2C_Bus_Recover();
  }
  return ret;
}

static int IMU_I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data,
                        uint16_t len) {
  int ret = HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len,
                             HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    IMU_I2C_Bus_Recover();
  }
  return ret;
}

mpu6050_write_reg_t MPU6050_Write_Reg = IMU_I2C_Write;
mpu6050_read_reg_t MPU6050_Read_Reg = IMU_I2C_Read;
MPU6050 mpu(MPU6050_Write_Reg, MPU6050_Read_Reg);
void MPU_Init() { mpu.init(); }
void MPU_Zero_Offset() { mpu.Zero_Offset(); }
void MPU_Read_Accel_Float(float *ax, float *ay, float *az) {
  if (ax && ay && az) {
    mpu.MPU6050_Read_Accel_Float(*ax, *ay, *az);
  }
}
void MPU_Read_Gyro_Float(float *gx, float *gy, float *gz) {
  if (gx && gy && gz) {
    mpu.MPU6050_Read_Gyro_Float(*gx, *gy, *gz);
  }
}
void MPU_Zero_Offset_Accel(float *ax, float *ay, float *az) {
  mpu.Read_Accel_Calibrated(*ax, *ay, *az);
}
void MPU_Zero_Offset_Gyro(float *gx, float *gy, float *gz) {
  mpu.Read_Gyro_Calibrated(*gx, *gy, *gz);
}
