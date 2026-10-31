/*
 * led_stm32.h
 *
 * 上游: Port/led_stm32.h  (class Led : public LedBase)
 *
 * C 化后不再需要"具体类", 这里只提供 LedBase 需要的写引脚函数。
 */

#ifndef CTRL_STEP_FW_LED_STM32_H
#define CTRL_STEP_FW_LED_STM32_H

#include <stdbool.h>
#include <stdint.h>

/* 供 led_init() 使用: led_init(&status_led, led_stm32_set_state);
 * _state == true 表示点亮 (上游为低电平点亮, 实现内部取反) */
void led_stm32_set_state(uint8_t id, bool state);

#endif /* CTRL_STEP_FW_LED_STM32_H */
