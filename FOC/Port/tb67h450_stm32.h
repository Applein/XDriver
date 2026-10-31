/*
 * tb67h450_stm32.h
 *
 * 上游: Port/tb67h450_stm32.h  (class TB67H450 : public TB67H450Base)
 *
 * C 化后不再有"具体类": 板级差异全部收敛到 tb67h450_stm32_port_ops,
 * 由 tb67h450_stm32_setup() 把 ops 与虚表一起挂到 TB67H450Base 上。
 *
 * 用法 (对应上游 `TB67H450 tb67H450;` + `motor.AttachDriver(&tb67H450)`):
 *      static TB67H450Base tb67h450;
 *      tb67h450_stm32_setup(&tb67h450);
 *      motor_attach_driver(&motor, &tb67h450.base);
 */

#ifndef CTRL_STEP_FW_TB67H450_STM32_H
#define CTRL_STEP_FW_TB67H450_STM32_H

#include "Driver/tb67h450_base.h"

/* 板级实现表:
 *   init_gpio          -> PA2~PA5 (HW_ELEC_BM/BP/AM/AP) 推挽输出
 *   init_pwm           -> 启动 TIM2 CH3(PB10)/CH4(PB11)
 *   dac_output_voltage -> 写 TIM2 比较值 (12bit -> 10bit)
 *   set_input_a/b      -> 方向脚 BSRR/BRR 直接操作
 */
extern const TB67H450PortOps tb67h450_stm32_port_ops;

/* 初始化: 挂 ops + 4 个虚函数指针, 清零相位状态。
 * 注意: 不启动 PWM、不初始化定时器 —— 调用 motor->driver->Init() 时才做。 */
void tb67h450_stm32_setup(TB67H450Base *self);

#endif /* CTRL_STEP_FW_TB67H450_STM32_H */
