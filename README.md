# 简易飞控
## 项目简介

四旋翼飞控，基于STM32F407，支持姿态解算与PID控制

## 功能
- [x] MPU6050姿态数据采集(iic)
- [x] 简单互补滤波
- [x] NRF24L01双向无线通信
- [x] 四路电机驱动
- [] 气压计定高
- [] gps巡航
- [] 视觉辅助定位
## 硬件平台

| 部件 | 型号 | 说明 |
| :--- | :--- | :--- |
| 主控 | STM32F407VGT6 | 168MHz, 1MB Flash, 192KB RAM |
| 惯性传感器 | MPU6050 | 六轴 IMU，I2C 接口 |
| 无线通信 | NRF24L01 | SPI 接口，2.4GHz |
| 电机 | 2212 无刷电机 × 4 | 配 30A 电调 |
| 电池 | 3S 锂电 | 11.1V |
## 软件环境

- **编译器**：arm-none-eabi-gcc（版本 ≥ 10）
- **构建工具**：CMake ≥ 3.15 + Ninja / Make
- **代码生成**：STM32CubeMX
- **HAL 库**：STM32F4xx HAL
- **RTOS**：FreeRTOS
- **开发系统**：Windows / Linux（CachyOS）

## cmake设置
```cmake
add_subdirectory(FlightControl/middleware)
add_subdirectory(FlightControl/interface)
add_subdirectory(FlightControl/drivers)
add_subdirectory(FlightControl/apps)
```
