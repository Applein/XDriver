/*
 * button_base.h
 *
 * 上游: Ctrl/Signal/button_base.h  (class ButtonBase)
 *
 * C 化要点:
 *   1. protected 纯虚 ReadButtonPinIO()  -> read_pin_io 函数指针 (Port 层提供)
 *   2. enum Event                        -> ButtonEvent (BTN_EVT_*)
 *   3. 两个构造函数                      -> button_init(self, id, long_press_ms, read_pin_io)
 *   4. 成员默认初始化器 longPressTime=2000 -> button_init() 显式赋值
 *
 * 说明: 上游 Button 只有这一个实现, 因此不再为"具体类"单独造一层结构体;
 *       Port 层只需要提供一个 bool (*)(uint8_t) 形式的引脚读取函数。
 */

#ifndef CTRL_STEP_FW_BUTTON_BASE_H
#define CTRL_STEP_FW_BUTTON_BASE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 上游: ButtonBase::Event */
typedef enum
{
    BTN_EVT_UP = 0,
    BTN_EVT_DOWN,
    BTN_EVT_LONG_PRESS,
    BTN_EVT_CLICK,
} ButtonEvent;

/* 上游: virtual bool ReadButtonPinIO(uint8_t _id)
 * 返回引脚的真实电平 (高电平 = 未按下), 取反在 button_is_pressed() 里做 */
typedef bool (*ButtonReadPinIoFn)(uint8_t id);

/* 上游: void (*OnEventFunc)(Event) */
typedef void (*ButtonEventFn)(ButtonEvent event);

/* 上游 ButtonBase 的成员 (protected 部分在这里是公开字段, C 无访问控制) */
typedef struct
{
    uint8_t id;
    bool last_pin_io;
    uint32_t timer;
    uint32_t press_time;
    uint32_t long_press_time;

    ButtonEventFn on_event;
    ButtonReadPinIoFn read_pin_io;
} ButtonBase;

/* 上游: longPressTime 的默认值 2000ms */
enum
{
    BUTTON_DEFAULT_LONG_PRESS_MS = 2000,
};

void button_init(ButtonBase *self, uint8_t id, uint32_t long_press_ms,
                 ButtonReadPinIoFn read_pin_io);

/* 上游: ButtonBase::Tick() */
void button_tick(ButtonBase *self, uint32_t elapsed_ms);

/* 上游: ButtonBase::SetOnEventListener() */
void button_set_on_event_listener(ButtonBase *self, ButtonEventFn callback);

/* 上游: Button::IsPressed() (原在 Port/button_stm32) */
bool button_is_pressed(ButtonBase *self);

#endif /* CTRL_STEP_FW_BUTTON_BASE_H */
