/*
 * rt_board.h
 *
 * XDriver 的 RT-Thread 板级移植接口 (STM32F103C8T6 + USART1 控制台)
 */

#ifndef __RT_BOARD_H__
#define __RT_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * CubeMX 生成的外设初始化函数。
 * 为了让 rt_board.c 能复用它们, main.c 中这几个函数已去掉 static。
 * 注意: CubeMX 重新生成代码后会把它们改回 static, 需要重新去掉。
 */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);

/* RT-Thread 板级初始化 (由 rtthread_startup 调用) */
void rt_hw_board_init(void);

/* USART1 中断服务: 把接收到的字节放入控制台环形缓冲 (在 USART1_IRQHandler 中调用) */
void rt_hw_console_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* __RT_BOARD_H__ */
