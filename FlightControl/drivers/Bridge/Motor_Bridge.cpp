#include "Motor_Config.hpp"
#include "Motor_Bridge.h"
#include <cstddef>
#include <vector>
#define MAX_MOTORS 8
static std::vector<Motor *> motor;
static bool initialized = false;
void Init_Pool() {
  if (!initialized) {
    motor.reserve(MAX_MOTORS);
    initialized = true;
  }
}
extern "C" {
Motor_Handle Motor_Create(TIM_HandleTypeDef *htim, uint32_t Channel) {
  Init_Pool();
  if (motor.size() >= MAX_MOTORS)
    return nullptr;
  Motor *m = new Motor(htim, Channel);
  motor.push_back(m);
  return static_cast<Motor_Handle>(m);
}

void Motor_Start(Motor_Handle handle) {
  if (handle == nullptr)
    return;
  Motor *m = static_cast<Motor *>(handle);
  m->init();
}
void Motor_Set_Speed(Motor_Handle handle, int16_t speed) {
  if (handle == nullptr)
    return;
  Motor *m = static_cast<Motor *>(handle);
  m->setSpeed(speed);
}
void Motor_Stop(Motor_Handle handle) {
  if (handle == nullptr)
    return;
  Motor *m = static_cast<Motor *>(handle);
  m->stop();
}
}