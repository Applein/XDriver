/*
 * led_stm32.c
 *
 * 上游: Port/led_stm32.cpp
 *
 *   0 -> LED1 (上游 PC13)
 *   1 -> LED2 (上游 PC14)
 *
 * 上游是低电平点亮: _state == true 时输出 RESET。
 *
 * 依赖 CubeMX 生成的 main.h 引脚标签 (阶段 D 加入), 因此本文件
 * 暂时不加入 cmake/foc.cmake 的 FOC_SRC, 等 CubeMX 加上
 * LED1_Pin / LED1_GPIO_Port / LED2_Pin / LED2_GPIO_Port 后再启用。
 */

#include "led_stm32.h"

#include "main.h"

void led_stm32_set_state(uint8_t id, bool state)
{
    switch (id)
    {
    case 0:
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case 1:
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    default:
        break;
    }
}
