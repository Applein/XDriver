/*
 * app_foc.h
 *
 * 上游: UserApp/main.cpp  (Main() + Tim1Callback100Hz + Tim4Callback20kHz + 按键回调)
 *
 * 本文件把上游的"裸机 main"改写成 RT-Thread 下的应用层:
 *   foc_app_init()      <- 上游 Main() 的初始化部分
 *   foc_tick_20khz()    <- 上游 Tim4Callback20kHz() 的内容, 由 TIM4 中断调用
 *   100 Hz 家务线程     <- 上游 Tim1Callback100Hz() 的内容 (按键/指示灯/校准主循环)
 *
 * 注意: 上游用 TIM1 产生 100 Hz 应用节拍; 本工程的 TIM1 已被 HAL 用作
 *       时基 (stm32f1xx_hal_timebase_tim.c), 因此 100 Hz 节拍改用 RT 线程。
 */

#ifndef CTRL_STEP_FW_APP_FOC_H
#define CTRL_STEP_FW_APP_FOC_H

/* 初始化: 载入配置 -> 构建 motor -> 挂驱动/编码器 -> 启动 20 kHz 与 100 Hz */
void foc_app_init(void);

/* 20 kHz 闭环入口, 必须在 TIM4 更新中断里调用 (不得调用任何 RT-Thread API) */
void foc_tick_20khz(void);

#endif /* CTRL_STEP_FW_APP_FOC_H */
