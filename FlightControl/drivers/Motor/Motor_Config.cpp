#include "Motor_Config.hpp"
#include "stm32f4xx_hal_tim.h"
Motor::Motor(TIM_HandleTypeDef *_pwm_tim, uint32_t _pwm_channel)
    : _pwm_tim(_pwm_tim), _pwm_channel(_pwm_channel), _current_speed(0) {}
bool Motor::init() {
  HAL_TIM_PWM_Start(this->_pwm_tim, this->_pwm_channel);
  return true;
}
void Motor::setSpeed(int16_t speed) {
  _current_speed = speed;
  __HAL_TIM_SET_COMPARE(_pwm_tim, _pwm_channel, speed);
}
int16_t Motor::getSpeed() const { return _current_speed; }
bool Motor::stop() {
  HAL_TIM_PWM_Stop(_pwm_tim, _pwm_channel);
  return true;
}