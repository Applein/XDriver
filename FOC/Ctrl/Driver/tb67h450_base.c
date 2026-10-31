/*
 * tb67h450_base.c
 *
 * 上游: Ctrl/Driver/tb67h450_base.cpp
 *
 * 逐行移植。与上游的唯一差异是"取回派生对象"这一层:
 *   上游方法内部直接用 this 访问 protected 成员与虚函数;
 *   C 里 DriverBase* 就是 TB67H450Base* (base 是首成员),
 *   因此用 tb67h450(driver) 宏做一次显式转换, 语义等价且零开销。
 *
 * 注意: 本文件在 20 kHz 中断里每 tick 执行一次, 因此:
 *   - 不使用任何 RT-Thread API / printf / 动态内存
 *   - ops-> 调用是普通函数指针, 编译器可直接内联 Port 层实现
 */

#include "Driver/tb67h450_base.h"

#include <stdlib.h>     /* abs() */

#include "Driver/sin_map.h"

/* DriverBase* -> TB67H450Base* (base 是首成员, 地址相同) */
#define TB67H450(driver)    ((TB67H450Base *)(driver))

/* 上游: 相电流 -> 12bit DAC 值的比例常数
 * (3.3V / 3.3A 满量程, 12bit: 4095 -> 3300mA) */
enum
{
    TB67H450_DAC_GAIN = 5083,
    TB67H450_SIN_MASK = 0x03FF,     /* 1024 counts / 电周期 */
    TB67H450_PHASE_B_OFFSET = 256,  /* 90° 相位差 */
    TB67H450_DAC_MASK = 0x0FFF,
};

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450Base::SetTwoCoilsCurrent(...)
 *   上游只有一个实现, 因此不虚化, 直接调用板级 ops
 * -------------------------------------------------------------------------- */
static void tb67h450_set_two_coils_current(TB67H450Base *self,
                                           uint16_t current_a_3300ma_in_12bits,
                                           uint16_t current_b_3300ma_in_12bits)
{
    /*
     * After SetFocCurrentVector calculation a 12bits value was mapped to 0~3300mA.
     * And due to used 0.1Ohm shank resistor, 0~3300mV V-ref means 0~3300mA
     * CurrentSetPoint.
     */
    self->ops->dac_output_voltage(current_a_3300ma_in_12bits,
                                  current_b_3300ma_in_12bits);
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450Base::Init()  { InitGpio(); InitPwm(); }
 * -------------------------------------------------------------------------- */
void tb67h450_base_driver_init(DriverBase *driver)
{
    TB67H450Base *self = TB67H450(driver);

    self->ops->init_gpio();
    self->ops->init_pwm();
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450Base::SetFocCurrentVector(uint32_t, int32_t)
 * -------------------------------------------------------------------------- */
void tb67h450_base_set_foc_current_vector(DriverBase *driver, uint32_t direction_in_count,
                                          int32_t current_ma)
{
    TB67H450Base *self = TB67H450(driver);
    uint32_t dac_reg;

    /* 相位指针: B 相在前, A 相超前 90° */
    self->base.phase_b.sin_map_ptr =
        (uint16_t)(direction_in_count & TB67H450_SIN_MASK);
    self->base.phase_a.sin_map_ptr =
        (uint16_t)((self->base.phase_b.sin_map_ptr + TB67H450_PHASE_B_OFFSET) &
                   TB67H450_SIN_MASK);

    self->base.phase_a.sin_map_data = sin_pi_m2[self->base.phase_a.sin_map_ptr];
    self->base.phase_b.sin_map_data = sin_pi_m2[self->base.phase_b.sin_map_ptr];

    dac_reg = (uint32_t)abs(current_ma);
    dac_reg = (uint32_t)(dac_reg * TB67H450_DAC_GAIN) >> 12;
    dac_reg = dac_reg & TB67H450_DAC_MASK;

    self->base.phase_a.dac_value_12bits =
        (uint16_t)(((uint32_t)(dac_reg * (uint32_t)abs(self->base.phase_a.sin_map_data)))
                   >> sin_pi_m2_dpiybit);
    self->base.phase_b.dac_value_12bits =
        (uint16_t)(((uint32_t)(dac_reg * (uint32_t)abs(self->base.phase_b.sin_map_data)))
                   >> sin_pi_m2_dpiybit);

    tb67h450_set_two_coils_current(self,
                                   self->base.phase_a.dac_value_12bits,
                                   self->base.phase_b.dac_value_12bits);

    if (self->base.phase_a.sin_map_data > 0)
    {
        self->ops->set_input_a(true, false);
    }
    else if (self->base.phase_a.sin_map_data < 0)
    {
        self->ops->set_input_a(false, true);
    }
    else
    {
        self->ops->set_input_a(true, true);
    }

    if (self->base.phase_b.sin_map_data > 0)
    {
        self->ops->set_input_b(true, false);
    }
    else if (self->base.phase_b.sin_map_data < 0)
    {
        self->ops->set_input_b(false, true);
    }
    else
    {
        self->ops->set_input_b(true, true);
    }
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450Base::Sleep()
 *   DAC 归零 + 两个方向脚都拉低 (高阻/关断)
 * -------------------------------------------------------------------------- */
void tb67h450_base_sleep(DriverBase *driver)
{
    TB67H450Base *self = TB67H450(driver);

    self->base.phase_a.dac_value_12bits = 0;
    self->base.phase_b.dac_value_12bits = 0;

    tb67h450_set_two_coils_current(self, 0, 0);

    self->ops->set_input_a(false, false);
    self->ops->set_input_b(false, false);
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450Base::Brake()
 *   DAC 归零 + 两个方向脚都拉高 (双低侧/双高侧短路制动)
 * -------------------------------------------------------------------------- */
void tb67h450_base_brake(DriverBase *driver)
{
    TB67H450Base *self = TB67H450(driver);

    self->base.phase_a.dac_value_12bits = 0;
    self->base.phase_b.dac_value_12bits = 0;

    tb67h450_set_two_coils_current(self, 0, 0);

    self->ops->set_input_a(true, true);
    self->ops->set_input_b(true, true);
}

/* -------------------------------------------------------------------------- */
/* 上游: TB67H450Base() 默认构造
 *   上游是 = default, 成员为 POD 未初始化; 这里显式清零, 语义更安全。
 *   base.phase_a/phase_b 的指针在下面挂接。
 * -------------------------------------------------------------------------- */
void tb67h450_base_init(TB67H450Base *self, const TB67H450PortOps *ops)
{
    self->ops = ops;

    self->base.init = tb67h450_base_driver_init;
    self->base.set_foc_current_vector = tb67h450_base_set_foc_current_vector;
    self->base.sleep = tb67h450_base_sleep;
    self->base.brake = tb67h450_base_brake;

    self->base.phase_a.sin_map_ptr = 0;
    self->base.phase_a.sin_map_data = 0;
    self->base.phase_a.dac_value_12bits = 0;

    self->base.phase_b.sin_map_ptr = 0;
    self->base.phase_b.sin_map_data = 0;
    self->base.phase_b.dac_value_12bits = 0;
}
