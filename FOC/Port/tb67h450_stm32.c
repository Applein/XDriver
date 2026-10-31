/*
 * tb67h450_stm32.c
 *
 * 上游: Port/tb67h450_stm32.cpp
 *
 * 与上游的差异 (两处, 均为适配 RT-Thread 工程):
 *
 * 1. init_pwm: 上游是 MX_TIM2_Init()。
 *    本工程 MX_TIM2_Init() 已由 rt_hw_board_init() 调用, 再调一次会把
 *    TIM2 计数器/比较寄存器重置 (PWM 短暂归零)。因此这里只启动 PWM 输出。
 *
 * 2. init_gpio: 上游自行使能 GPIOA/GPIOB 时钟并配置 PA2~PA5。
 *    本工程 MX_GPIO_Init() 已做过同样的事, 这里保持相同配置重复设置一次,
 *    行为与上游一致 (幂等), 不依赖初始化顺序。
 */

#include "tb67h450_stm32.h"

#include "main.h"
#include "tim.h"

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450::InitGpio()                                            */
/* -------------------------------------------------------------------------- */
static void tb67h450_stm32_init_gpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 方向脚初始低电平 (功率级关断) */
    HAL_GPIO_WritePin(GPIOA,
                      HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin,
                      GPIO_PIN_RESET);

    gpio.Pin = HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450::InitPwm()  { MX_TIM2_Init(); }
 *   见文件头说明 1: 定时器已由 rt_board.c 初始化, 这里只启动输出
 * -------------------------------------------------------------------------- */
static void tb67h450_stm32_init_pwm(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   /* HW_ELEC_BPWM = PB10 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   /* HW_ELEC_APWM = PB11 */
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450::DacOutputVoltage(a, b)
 *   12bit (0~4095) -> TIM2 比较值 (ARR = 1023, 即 10bit)
 *   注意通道对应关系与上游一致: A 相 -> CH4, B 相 -> CH3
 * -------------------------------------------------------------------------- */
static void tb67h450_stm32_dac_output_voltage(uint16_t voltage_a_3300mv_in_12bits,
                                              uint16_t voltage_b_3300mv_in_12bits)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(voltage_a_3300mv_in_12bits >> 2));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(voltage_b_3300mv_in_12bits >> 2));
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450::SetInputA(bool _statusAp, bool _statusAm)
 *   PA5 = HW_ELEC_AP, PA4 = HW_ELEC_AM
 *   直接写 BSRR/BRR: 单条总线写, 无读-改-写, 适合 20 kHz 中断
 * -------------------------------------------------------------------------- */
static void tb67h450_stm32_set_input_a(bool status_ap, bool status_am)
{
    if (status_ap)
    {
        GPIOA->BSRR = (uint32_t)HW_ELEC_AP_Pin;
    }
    else
    {
        GPIOA->BRR = (uint32_t)HW_ELEC_AP_Pin;
    }

    if (status_am)
    {
        GPIOA->BSRR = (uint32_t)HW_ELEC_AM_Pin;
    }
    else
    {
        GPIOA->BRR = (uint32_t)HW_ELEC_AM_Pin;
    }
}

/* -------------------------------------------------------------------------- */
/* 上游: void TB67H450::SetInputB(bool _statusBp, bool _statusBm)
 *   PA3 = HW_ELEC_BP, PA2 = HW_ELEC_BM
 * -------------------------------------------------------------------------- */
static void tb67h450_stm32_set_input_b(bool status_bp, bool status_bm)
{
    if (status_bp)
    {
        GPIOA->BSRR = (uint32_t)HW_ELEC_BP_Pin;
    }
    else
    {
        GPIOA->BRR = (uint32_t)HW_ELEC_BP_Pin;
    }

    if (status_bm)
    {
        GPIOA->BSRR = (uint32_t)HW_ELEC_BM_Pin;
    }
    else
    {
        GPIOA->BRR = (uint32_t)HW_ELEC_BM_Pin;
    }
}

const TB67H450PortOps tb67h450_stm32_port_ops =
{
    .init_gpio = tb67h450_stm32_init_gpio,
    .init_pwm = tb67h450_stm32_init_pwm,
    .dac_output_voltage = tb67h450_stm32_dac_output_voltage,
    .set_input_a = tb67h450_stm32_set_input_a,
    .set_input_b = tb67h450_stm32_set_input_b,
};

void tb67h450_stm32_setup(TB67H450Base *self)
{
    tb67h450_base_init(self, &tb67h450_stm32_port_ops);
}
