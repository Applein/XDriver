/*
 * led_base.c
 *
 * 上游: Ctrl/Signal/led_base.cpp
 *
 * 逐段对照移植, 状态机时序(100ms/700ms/1000ms)与灯序保持完全一致。
 * 与上游的唯一结构差异: 用显式大括号消除上游"嵌套 if 无括号"的悬垂 else 写法,
 * 语义不变 (-Wdangling-else)。
 */

#include "Signal/led_base.h"

void led_init(LedBase *self, LedSetStateFn set_led_state)
{
    self->timer = 0;
    self->timer_heart_beat = 0;
    self->motor_enable = false;
    self->heart_beat_enable = false;
    self->heart_beat_phase = 1;     /* 上游成员默认初始化器 */
    self->blink_num = 0;
    self->target_blink_num = 0;
    self->timer_blink = 0;
    self->blink_phase = 1;          /* 上游成员默认初始化器 */
    self->set_led_state = set_led_state;
}

void led_tick(LedBase *self, uint32_t elapsed_ms, MotorState state)
{
    self->timer += elapsed_ms;

    /* ------------------------- 依据电机状态决定灯型 ------------------------- */
    switch (state)
    {
    case MOTOR_STATE_NO_CALIB:
        self->motor_enable = false;
        self->heart_beat_enable = false;
        self->target_blink_num = 1;
        break;
    case MOTOR_STATE_RUNNING:
        self->motor_enable = true;
        self->heart_beat_enable = true;
        self->target_blink_num = 0;
        break;
    case MOTOR_STATE_FINISH:
        self->motor_enable = true;
        self->heart_beat_enable = false;
        self->target_blink_num = 0;
        break;
    case MOTOR_STATE_STOP:
        self->motor_enable = false;
        self->heart_beat_enable = false;
        self->target_blink_num = 0;
        break;
    case MOTOR_STATE_OVERLOAD:
        self->motor_enable = true;
        self->heart_beat_enable = false;
        self->target_blink_num = 3;
        break;
    case MOTOR_STATE_STALL:
        self->motor_enable = false;
        self->heart_beat_enable = false;
        self->target_blink_num = 2;
        break;
    default:
        break;
    }

    /* ------------------------ LED0: 常亮 / 心跳 / 灭 ------------------------ */
    if (self->motor_enable)
    {
        if (self->heart_beat_enable)
        {
            switch (self->heart_beat_phase)
            {
            case 1:
                if (self->timer - self->timer_heart_beat > 100)
                {
                    self->set_led_state(0, false);
                    self->timer_heart_beat = self->timer;
                    self->heart_beat_phase = 2;
                }
                break;
            case 2:
                if (self->timer - self->timer_heart_beat > 100)
                {
                    self->set_led_state(0, true);
                    self->timer_heart_beat = self->timer;
                    self->heart_beat_phase = 3;
                }
                break;
            case 3:
                if (self->timer - self->timer_heart_beat > 100)
                {
                    self->set_led_state(0, false);
                    self->timer_heart_beat = self->timer;
                    self->heart_beat_phase = 4;
                }
                break;
            case 4:
                if (self->timer - self->timer_heart_beat > 700)
                {
                    self->set_led_state(0, true);
                    self->timer_heart_beat = self->timer;
                    self->heart_beat_phase = 1;
                }
                break;
            default:
                break;
            }
        }
        else
        {
            self->set_led_state(0, true);
            self->heart_beat_phase = 1;
        }
    }
    else
    {
        self->set_led_state(0, false);
    }

    /* ---------------------------- LED1: 错误闪码 ---------------------------- */
    switch (self->blink_phase)
    {
    case 1:
        if (self->timer - self->timer_blink > 100)
        {
            self->set_led_state(1, false);
            self->timer_blink = self->timer;
            self->blink_phase = 2;
        }
        break;
    case 2:
        if (self->timer - self->timer_blink > 100)
        {
            self->blink_num++;
            if (self->target_blink_num > self->blink_num)
            {
                self->set_led_state(1, true);
                self->blink_phase = 1;
                self->timer_blink = self->timer;
            }
            else
            {
                self->set_led_state(1, false);
                self->blink_phase = 3;
                self->timer_blink = self->timer;
            }
        }
        break;
    case 3:
        if (self->timer - self->timer_blink > 1000)
        {
            self->blink_num = 0;
            self->set_led_state(1, self->target_blink_num > 0);
            self->timer_blink = self->timer;
            self->blink_phase = 1;
        }
        break;
    default:
        break;
    }
}
