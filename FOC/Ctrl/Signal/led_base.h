/*
 * led_base.h
 *
 * 上游: Ctrl/Signal/led_base.h  (class LedBase)
 *
 * C 化要点:
 *   1. private 纯虚 SetLedState()        -> set_led_state 函数指针 (Port 层提供)
 *   2. Tick(ms, Motor::State_t)          -> led_tick(self, ms, MotorState)
 *   3. 成员默认初始化器 heartBeatPhase=1 / blinkPhase=1 -> led_init() 显式赋值
 *   4. Motor::State_t                    -> MotorState (定义在 Motor/motor_types.h)
 */

#ifndef CTRL_STEP_FW_LED_BASE_H
#define CTRL_STEP_FW_LED_BASE_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor/motor_types.h"

/* 上游: virtual void SetLedState(uint8_t _id, bool _state)
 * 注意上游是"低电平点亮": Port 层实现内部做了取反 */
typedef void (*LedSetStateFn)(uint8_t id, bool state);

typedef struct
{
    uint32_t timer;
    uint32_t timer_heart_beat;
    bool motor_enable;
    bool heart_beat_enable;
    uint8_t heart_beat_phase;   /* 上游默认 1 */
    uint8_t blink_num;
    uint8_t target_blink_num;
    uint32_t timer_blink;
    uint8_t blink_phase;        /* 上游默认 1 */

    LedSetStateFn set_led_state;
} LedBase;

void led_init(LedBase *self, LedSetStateFn set_led_state);

/* 上游: LedBase::Tick(uint32_t _timeElapseMillis, Motor::State_t _state) */
void led_tick(LedBase *self, uint32_t elapsed_ms, MotorState state);

#endif /* CTRL_STEP_FW_LED_BASE_H */
