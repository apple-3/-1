#include "ultrasonic.h"
#include "FreeRTOS.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include <stdint.h>

static void Ultrasonic_DelayUs(uint32_t us)
{
  static uint8_t initialized = 0;
  if (!initialized) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    initialized = 1;
  }

  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks) {
    __NOP();
  }
}

float Ultrasonic_Getdistance(void)
{
  uint32_t time;
  uint32_t timeout;

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  Ultrasonic_DelayUs(2);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  Ultrasonic_DelayUs(10);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  timeout = 30000;
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET && timeout--) {
    __NOP();
  }
  if (timeout == 0) {
    return 0.0f;
  }

  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_ENABLE(&htim4);

  timeout = 30000;
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET && timeout--) {
    __NOP();
  }
  __HAL_TIM_DISABLE(&htim4);

  if (timeout == 0) {
    return 0.0f;
  }

  time = __HAL_TIM_GET_COUNTER(&htim4);
  return time * 0.017f;
}
