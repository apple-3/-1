file(GLOB_RECURSE DRIVERS_SOURCES "*.cpp" "*.c")

add_library(drivers_lib STATIC ${DRIVERS_SOURCES})

target_include_directories(drivers_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/MPU6050
    ${CMAKE_CURRENT_SOURCE_DIR}/Bridge
    ${CMAKE_CURRENT_SOURCE_DIR}/Motor
    ${CMAKE_CURRENT_SOURCE_DIR}/NRF24L01
    ${CMAKE_CURRENT_SOURCE_DIR}/HC-SR04
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/FilightControl/interface
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

target_compile_definitions(drivers_lib PUBLIC
    USE_HAL_DRIVER
    STM32F407xx
)

target_link_libraries(drivers_lib PUBLIC interfaces_lib)
