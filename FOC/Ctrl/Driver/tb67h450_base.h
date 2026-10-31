/*
 * tb67h450_base.h
 *
 * 上游: Ctrl/Driver/tb67h450_base.h  (class TB67H450Base : public DriverBase)
 *
 * C 化设计 (模式 2: 基类内嵌 + PortOps):
 *   上游的继承链是 TB67H450Base (算法) -> TB67H450 (板级, 5 个 protected 虚函数)。
 *   C 里去掉继承链:
 *     - TB67H450Base 内嵌 DriverBase base  【必须是第一个成员】
 *     - 板级差异收敛成 const TB67H450PortOps *ops
 *
 *   SetTwoCoilsCurrent 不虚化: 上游只有一个实现, 直接调 ops->dac_output_voltage,
 *   少一次间接调用 (20 kHz 路径上每 tick 调用一次)。
 */

#ifndef CTRL_STEP_FW_TB67H450_BASE_H
#define CTRL_STEP_FW_TB67H450_BASE_H

#include <stdbool.h>
#include <stdint.h>

#include "Driver/driver_base.h"

/* 上游: 5 个 protected 纯虚函数 -> 板级实现表 (定义在 Port/tb67h450_stm32.c) */
typedef struct
{
    void (*init_gpio)(void);
    void (*init_pwm)(void);
    void (*dac_output_voltage)(uint16_t voltage_a_3300mv_in_12bits,
                               uint16_t voltage_b_3300mv_in_12bits);
    void (*set_input_a)(bool status_ap, bool status_am);
    void (*set_input_b)(bool status_bp, bool status_bm);
} TB67H450PortOps;

typedef struct
{
    DriverBase base;                    /* 【必须第一个】便于基类指针回取自身 */
    const TB67H450PortOps *ops;
} TB67H450Base;

/* 上游 TB67H450Base() 默认构造: 零初始化 phase_a/phase_b, ops 由调用方指定 */
void tb67h450_base_init(TB67H450Base *self, const TB67H450PortOps *ops);

/* 上游: DriverBase 的 4 个虚函数实现 (供 tb67h450_base_init 挂到 base 上) */
void tb67h450_base_driver_init(DriverBase *driver);
void tb67h450_base_set_foc_current_vector(DriverBase *driver, uint32_t direction_in_count,
                                          int32_t current_ma);
void tb67h450_base_sleep(DriverBase *driver);
void tb67h450_base_brake(DriverBase *driver);

#endif /* CTRL_STEP_FW_TB67H450_BASE_H */
