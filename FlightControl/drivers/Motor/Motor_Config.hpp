#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "imotor.hpp"
#include "stm32f4xx_hal.h"

class Motor : public imotor {
public:
  Motor(TIM_HandleTypeDef *_pwm_tim, uint32_t _pwm_channel);
  bool init() override;
  void setSpeed(int16_t speed);
  int16_t getSpeed() const;
  bool stop() override;

private:
  TIM_HandleTypeDef *_pwm_tim;
  uint32_t _pwm_channel;
  int16_t _current_speed;
};

#endif // MOTOR_CONFIG_H