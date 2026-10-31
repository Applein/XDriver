/*
 * button_base.c
 *
 * 上游: Ctrl/Signal/button_base.cpp
 *
 * 与上游唯一的行为差异: on_event 为 NULL 时直接跳过事件派发。
 *   上游在 SetOnEventListener() 之前调用 Tick() 会解引用空函数指针崩溃;
 *   这里加保护是纯增量安全措施, 不影响任何已注册回调的场景。
 */

#include "Signal/button_base.h"

void button_init(ButtonBase *self, uint8_t id, uint32_t long_press_ms,
                 ButtonReadPinIoFn read_pin_io)
{
    self->id = id;
    self->last_pin_io = false;      /* 上游: bool lastPinIO{} */
    self->timer = 0;
    self->press_time = 0;
    self->long_press_time = long_press_ms;
    self->on_event = NULL;
    self->read_pin_io = read_pin_io;
}

void button_tick(ButtonBase *self, uint32_t elapsed_ms)
{
    bool pin_io;

    self->timer += elapsed_ms;
    pin_io = self->read_pin_io(self->id);

    if (self->last_pin_io == pin_io)
    {
        return;
    }

    /* 尚未注册回调: 只同步状态, 不派发事件 (避免空指针调用) */
    if (self->on_event == NULL)
    {
        self->last_pin_io = pin_io;
        return;
    }

    if (pin_io)
    {
        self->on_event(BTN_EVT_UP);
        if (self->timer - self->press_time > self->long_press_time)
        {
            self->on_event(BTN_EVT_LONG_PRESS);
        }
        else
        {
            self->on_event(BTN_EVT_CLICK);
        }
    }
    else
    {
        self->on_event(BTN_EVT_DOWN);
        self->press_time = self->timer;
    }

    self->last_pin_io = pin_io;
}

void button_set_on_event_listener(ButtonBase *self, ButtonEventFn callback)
{
    /* 上游: 先同步一次引脚状态, 避免注册瞬间产生虚假事件 */
    self->last_pin_io = self->read_pin_io(self->id);
    self->on_event = callback;
}

bool button_is_pressed(ButtonBase *self)
{
    /* 上游 Button::IsPressed(): return !ReadButtonPinIO(id) */
    return !self->read_pin_io(self->id);
}
