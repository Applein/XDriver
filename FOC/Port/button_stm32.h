/*
 * button_stm32.h
 *
 * 上游: Port/button_stm32.h  (class Button : public ButtonBase)
 *
 * C 化后不再需要"具体类", 这里只提供 ButtonBase 需要的引脚读取函数。
 * 上游 Button::IsPressed() 已提升为 button_base.c 里的 button_is_pressed()。
 */

#ifndef CTRL_STEP_FW_BUTTON_STM32_H
#define CTRL_STEP_FW_BUTTON_STM32_H

#include <stdbool.h>
#include <stdint.h>

/* 供 button_init() 使用: button_init(&button1, 1, 1000, button_stm32_read_pin_io);
 * 返回引脚真实电平 (高 = 未按下) */
bool button_stm32_read_pin_io(uint8_t id);

#endif /* CTRL_STEP_FW_BUTTON_STM32_H */
