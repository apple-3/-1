#include "freertos_start.h"
#include "Com_Config.h"
#include "Filter_Bridge.h"
#include "FreeRTOS.h"
#include "IMU_Bridge.h"
#include "Motor_Bridge.h"
#include "NRF24L01_Bridge.h"
#include "PIDset_Bridge.h"
#include "main.h"
#include "portmacro.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include "ultrasonic.h"
#include "usart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

uint8_t Buf[NRF24L01_Buf_Len] = {0};
uint8_t Telecontrol = 0;                                       // 遥控器接收状态
Remote_data Remote_Control_Data = {500, 500, 500, 0, 0, 0, 0}; // 遥控器接收数据

// 电机控制器句柄
Motor_Handle Left_Front;
Motor_Handle Right_Behind;
Motor_Handle Left_Behind;
Motor_Handle Right_Front;

// PID控制器句柄
PID_Handle Pitch_Angle_PID;
PID_Handle Roll_Angle_PID;
PID_Handle Yaw_Rate_PID;
PID_Handle Pitch_Gyro_PID;
PID_Handle Roll_Gyro_PID;
PID_Handle Yaw_Gyro_PID;

MPU_Data MPU_Data_Real = {0, 0, 0};
MPU_Data MPU_Data_Last = {0, 0, 0};

// 六向加速度
float ax, ay, az;
float gx, gy, gz;

// 串口调试
char msg[128];

// 状态机枚举
typedef enum {
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_ARMED, // 你们说我给无人机加制导系统会不会被抓进去
  MOTOR_STATE_ACTIVE,
  MOTOR_STATE_CALI,
  MOTOR_STATE_UNREMOTE,
  MOTOR_STATE_SLOW,
} MotorState;

static MotorState motor_state = MOTOR_STATE_IDLE;
static TickType_t last_control_tick = 0;
static float distance = 0.0f;
static const float pitch_target_scale = 0.1f;     // 0..1000 -> -10..10 deg
static const float roll_target_scale = 0.1f;      // 0..1000 -> -10..10 deg
static const float yaw_rate_target_scale = 0.12f; // 0..1200 -> -60..60 deg
static const int16_t max_motor_speed = 2000;

// 电机任务
#define Motor_Task_stack_size 512
#define Motor_Task_prioritize 2
TaskHandle_t Motor_Task_handle;
void Motor_Task(void *any);
// 配置任务
#define Status_Task_stack_size 128
#define Status_Task_prioritize 1
TaskHandle_t Status_Task_handle;
void Status_Task(void *any);
// 陀螺仪任务
#define MPU_Task_stack_size 512
#define MPU_Task_prioritize 2
TaskHandle_t MPU_Task_handle;
void MPU_Task(void *any);

void Start_Rtos(void) {
  NRF_Init();
  MPU_Init();
  MPU_Zero_Offset();

  Left_Front = Motor_Create(&htim3, TIM_CHANNEL_2);
  Right_Behind = Motor_Create(&htim3, TIM_CHANNEL_3);
  Left_Behind = Motor_Create(&htim2, TIM_CHANNEL_1);
  Right_Front = Motor_Create(&htim2, TIM_CHANNEL_2);
  Motor_Start(Left_Front);
  Motor_Start(Right_Behind);
  Motor_Start(Left_Behind);
  Motor_Start(Right_Front);
  // PID还没调
  Pitch_Angle_PID = PID_Creat(1.0f, 0.05f, 0.05f);
  Roll_Angle_PID = PID_Creat(1.0f, 0.05f, 0.05f);
  Yaw_Rate_PID = PID_Creat(1.0f, 0, 0.05f);
  Pitch_Gyro_PID = PID_Creat(1.0f, 0.05f, 0.01f);
  Roll_Gyro_PID = PID_Creat(1.0f, 0.05f, 0.01f);

  App_Rtos_Creat();
  vTaskStartScheduler();
}
void App_Rtos_Creat(void) {
  xTaskCreate(Motor_Task, "Motor_Task", Motor_Task_stack_size, NULL,
              Motor_Task_prioritize, &Motor_Task_handle);
  xTaskCreate(Status_Task, "Status_Task", Status_Task_stack_size, NULL,
              Status_Task_prioritize, &Status_Task_handle);
  xTaskCreate(MPU_Task, "MPU_Task", MPU_Task_stack_size, NULL,
              MPU_Task_prioritize, &MPU_Task_handle);
}

// PWM限幅
static int16_t clamp_speed(int16_t v) {
  if (v <= 1000)
    return 1000;
  if (v >= max_motor_speed)
    return max_motor_speed;
  return v;
}

void Motor_Task(void *any) {
  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    float throttle = (float)Remote_Control_Data.thr;
    float base_speed = 1000.0f + throttle;
    float pitch_target =
        ((float)Remote_Control_Data.pit - 500.0f) * pitch_target_scale;
    float roll_target =
        ((float)Remote_Control_Data.rol - 500.0f) * roll_target_scale;
    float yaw_target =
        ((float)Remote_Control_Data.yaw - 500.0f) * yaw_rate_target_scale;

    TickType_t now = xTaskGetTickCount();
    float dt = (now - last_control_tick) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    last_control_tick = now;

    // 神秘状态机，说实话我都不会写捏
    if (Remote_Control_Data.fix_high == 0 ||
        Remote_Control_Data.shortdown != 0) {
      motor_state = MOTOR_STATE_IDLE;
    } else if (motor_state == MOTOR_STATE_IDLE && throttle < 50.0f) {
      motor_state = MOTOR_STATE_ARMED;
    } else if (motor_state == MOTOR_STATE_ARMED && throttle > 50.0f) {
      motor_state = MOTOR_STATE_ACTIVE;
    } else if (Remote_Control_Data.shortdown == 0) {
      motor_state = MOTOR_STATE_UNREMOTE;
    }

    /*
    电机控制逻辑：
      NRF从机接收指令
      怠速：这里选百分之零是因为我害怕
      校准：2000为临时数据，后续准备在电机类加上校准逻辑，此处为临时校准
      激活：激活后通过计算PID串级角速度环按电机对应位置输出PWM信号
      控制器失联：通过内部的姿态计算选择对应的电机控制，比如：降落、大幅度矫正、或者如果有视觉就选没人的地方迫降吧（，当然还有可能还没飞就失联了
    */
    if (motor_state == MOTOR_STATE_IDLE) {
      Motor_Set_Speed(Left_Front, 1000);
      Motor_Set_Speed(Right_Front, 1000);
      Motor_Set_Speed(Left_Behind, 1000);
      Motor_Set_Speed(Right_Behind, 1000);
    } else if (motor_state == MOTOR_STATE_CALI) {
      Motor_Set_Speed(Left_Front, 2000);
      Motor_Set_Speed(Right_Front, 2000);
      Motor_Set_Speed(Left_Behind, 2000);
      Motor_Set_Speed(Right_Behind, 2000);
    } else if (motor_state == MOTOR_STATE_ACTIVE) {
      float pitch_output = F_PID_Up(Pitch_Angle_PID, Pitch_Gyro_PID,
                                    pitch_target, MPU_Data_Real.pitch, gy, dt);
      float roll_output = F_PID_Up(Roll_Angle_PID, Roll_Gyro_PID, roll_target,
                                   MPU_Data_Real.roll, gx, dt);
      float yaw_output = PID_Yaw_Update_Float(yaw_target, gz, dt);

      int16_t speed_LB =
          (int16_t)(base_speed - pitch_output + roll_output - yaw_output);
      int16_t speed_RB =
          (int16_t)(base_speed - pitch_output - roll_output + yaw_output);
      int16_t speed_LF =
          (int16_t)(base_speed + pitch_output + roll_output + yaw_output);
      int16_t speed_RF =
          (int16_t)(base_speed + pitch_output - roll_output - yaw_output);

      Motor_Set_Speed(Left_Front, clamp_speed(speed_LF));
      Motor_Set_Speed(Right_Front, clamp_speed(speed_RF));
      Motor_Set_Speed(Left_Behind, clamp_speed(speed_LB));
      Motor_Set_Speed(Right_Behind, clamp_speed(speed_RB));

      int n = snprintf(msg, sizeof(msg), ":%d,%d,%d,%d\n",
                       (int)clamp_speed(speed_LF), (int)clamp_speed(speed_RF),
                       (int)clamp_speed(speed_LB), (int)clamp_speed(speed_RB));
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, HAL_MAX_DELAY);

    } else if (motor_state == MOTOR_STATE_UNREMOTE) {
      //这里是蜂鸣器的逻辑，并且最好能计算当前姿态然后选择模式，大概（
      //比如说单纯平飞失联，慢慢降落或者返程（通过GPS）
      //再者就是大角度坠机（ =。=那就要看看怎么让电机骚操作救回来了
    } else if (motor_state == MOTOR_STATE_SLOW) {
      //这里就是降落吧
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
  }
}
void MPU_Task(void *any) {
  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    MPU_Zero_Offset_Accel(&ax, &ay, &az);
    MPU_Zero_Offset_Gyro(&gx, &gy, &gz);
    MPU_Data_Real.roll = atan2f(ay, az) * 180.0f / 3.141592f;
    MPU_Data_Real.pitch =
        atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.141592f;

    float dt = (xTaskGetTickCount() - last_wake) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;

    MPU_Data_Real.yaw += gz * dt;

    if (MPU_Data_Real.yaw > 180.0f)
      MPU_Data_Real.yaw -= 360.0f;
    if (MPU_Data_Real.yaw < -180.0f)
      MPU_Data_Real.yaw += 360.0f;

    MPU_Data_Real.pitch =
        First_order_Lowpass_Filter(MPU_Data_Real.pitch, MPU_Data_Last.pitch);

    MPU_Data_Real.roll =
        First_order_Lowpass_Filter(MPU_Data_Real.roll, MPU_Data_Last.roll);

    MPU_Data_Real.yaw =
        First_order_Lowpass_Filter(MPU_Data_Real.yaw, MPU_Data_Last.yaw);

    MPU_Data_Last.pitch = MPU_Data_Real.pitch;
    MPU_Data_Last.roll = MPU_Data_Real.roll;
    MPU_Data_Last.yaw = MPU_Data_Real.yaw;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(6));
  }
}
void Status_Task(void *any) {
  while (1) {
    Telecontrol = NRF_Receive(Buf);
    if (Telecontrol == NRF24L01_RX_OK) {
      Com_NRF_Access(&Remote_Control_Data, Buf);
      motor_state = MOTOR_STATE_ACTIVE;
    } else {
      motor_state = MOTOR_STATE_UNREMOTE;
    }
    distance = Ultrasonic_Getdistance();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
