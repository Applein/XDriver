/*
 * button_stm32.c
 *
 * 上游: Port/button_stm32.cpp
 *
 *   case 1 -> BUTTON1 (上游 PB12)
 *   case 2 -> BUTTON2 (上游 PB2)
 *
 * 依赖 CubeMX 生成的 main.h 引脚标签 (阶段 D 加入), 因此本文件
 * 暂时不加入 cmake/foc.cmake 的 FOC_SRC, 等 CubeMX 加上
 * BUTTON1_Pin / BUTTON1_GPIO_Port / BUTTON2_Pin / BUTTON2_GPIO_Port 后再启用。
 */

#include "button_stm32.h"

#include "main.h"

bool button_stm32_read_pin_io(uint8_t id)
{
    switch (id)
    {
    case 1:
        return HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) == GPIO_PIN_SET;
    case 2:
        return HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin) == GPIO_PIN_SET;
    default:
        return false;
    }
}
