#ifndef COM_CONFIG_H
#define COM_CONFIG_H
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>

typedef struct Joy_data {
  uint16_t pit;
  uint16_t rol;
  uint16_t yaw;
  uint16_t thr;
  uint8_t shortdown;
  uint8_t fix_high;
  uint8_t calibrate;
} Remote_data;
typedef struct Measure_Data {
  float roll, pitch, yaw;
} MPU_Data;
void Com_Limit(float *inner, float Max, float Min);
void Com_NRF_Access(Remote_data *data, uint8_t *Buf);

#endif // COM_CONFIG_H