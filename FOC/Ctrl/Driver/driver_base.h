/*
 * driver_base.h
 *
 * 上游: Ctrl/Driver/driver_base.h  (class DriverBase, 4 个纯虚函数)
 *
 * C 化设计 (模式 1: 虚表):
 *   DriverBase 结构体的成员就是原来的 4 个虚函数 + 2 个 protected 成员。
 *   Motor 只持有 DriverBase * 指针, 通过 driver_xxx(d) 包装调用,
 *   调用点写法与上游 motor->driver->Sleep() 一一对应。
 *
 *   phase_a / phase_b 是上游 protected 成员, C 无访问控制, 直接放在结构体里。
 *   TB67H450Base 必须把 DriverBase 作为【第一个成员】内嵌,
 *   这样才能用 (TB67H450Base *)driver 从基类指针取回派生对象。
 */

#ifndef CTRL_STEP_FW_DRIVER_BASE_H
#define CTRL_STEP_FW_DRIVER_BASE_H

#include <stdint.h>

/* 上游: DriverBase::FastSinToDac_t (protected 嵌套结构体) */
typedef struct
{
    uint16_t sin_map_ptr;
    int16_t sin_map_data;
    uint16_t dac_value_12bits;
} FastSinToDac;

typedef struct DriverBase DriverBase;

struct DriverBase
{
    /* ---- 上游 4 个纯虚函数 ---- */
    void (*init)(DriverBase *self);

    /*
     * FOC 电流矢量方向以 counts 表示: 把 360° 分成 N 份,
     * _direction_in_count 取 0 ~ N-1, 计算后电流范围 0 ~ 3300mA
     */
    void (*set_foc_current_vector)(DriverBase *self, uint32_t direction_in_count,
                                   int32_t current_ma);

    void (*sleep)(DriverBase *self);
    void (*brake)(DriverBase *self);

    /* ---- 上游 protected 成员 ---- */
    FastSinToDac phase_a;
    FastSinToDac phase_b;
};

/* 便捷包装: 与上游 driver->Sleep() / driver->Brake() 一一对应 */
static inline void driver_init(DriverBase *self)
{
    self->init(self);
}

static inline void driver_set_foc_current_vector(DriverBase *self,
                                                 uint32_t direction_in_count,
                                                 int32_t current_ma)
{
    self->set_foc_current_vector(self, direction_in_count, current_ma);
}

static inline void driver_sleep(DriverBase *self)
{
    self->sleep(self);
}

static inline void driver_brake(DriverBase *self)
{
    self->brake(self);
}

#endif /* CTRL_STEP_FW_DRIVER_BASE_H */
