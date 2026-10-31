/*
 * RT-Thread configuration for XDriver (STM32F103C8T6)
 *
 * 说明:
 *  - 只启用内核 + FinSH(msh) 所需的最小配置, 节省 RAM/FLASH
 *  - 不使用 RT_USING_DEVICE, FinSH 直接通过 rt_hw_console_output /
 *    rt_hw_console_getchar 访问 USART1 (见 Core/Src/rt_board.c)
 *  - 使用小内存管理算法 (small mem) 作为堆
 */

#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/* ---------------- Basic Configuration ---------------- */
#define RT_THREAD_PRIORITY_MAX          32
#define RT_TICK_PER_SECOND              1000
#define RT_ALIGN_SIZE                   4
#define RT_NAME_MAX                     8

/* 使用组件自动初始化 (.rti_fn.* 段) */
#define RT_USING_COMPONENTS_INIT

/* 使用 RT-Thread 的 main 线程机制: startup 调用 entry() -> rtthread_startup() */
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE       1024
#define RT_MAIN_THREAD_PRIORITY         10

/* ---------------- Debug ---------------- */
/* #define RT_DEBUG */
#define RT_DEBUG_INIT                   0
/* #define RT_USING_OVERFLOW_CHECK */

/* ---------------- Hook ---------------- */
/* #define RT_USING_HOOK */
/* #define RT_USING_IDLE_HOOK */

/* ---------------- Software Timer ---------------- */
/* 不使用软件定时器线程 */
/* #define RT_USING_TIMER_SOFT */
#define RT_TIMER_THREAD_PRIO            4
#define RT_TIMER_THREAD_STACK_SIZE      512

/* ---------------- IPC ---------------- */
/* FinSH 内部使用信号量 */
#define RT_USING_SEMAPHORE
/* #define RT_USING_MUTEX */
/* #define RT_USING_EVENT */
/* #define RT_USING_MAILBOX */
/* #define RT_USING_MESSAGEQUEUE */

/* ---------------- Memory ---------------- */
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP
/* #define RT_USING_MEMPOOL */

/* ---------------- Console ---------------- */
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE              128

/* ---------------- FinSH / msh ---------------- */
#define RT_USING_FINSH
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES             5
#define FINSH_THREAD_PRIORITY           20
/* msh_exec / shell_auto_complete 会用到较多栈, 保持 RT-Thread 默认 2KB 更稳 */
#define FINSH_THREAD_STACK_SIZE         2048
#define FINSH_CMD_SIZE                  80
#define MSH_USING_BUILT_IN_COMMANDS

/* ---------------- Device framework ---------------- */
/* 不启用设备框架, 控制台直接走 BSP 钩子函数 */
/* #define RT_USING_DEVICE */

#endif /* __RTTHREAD_CFG_H__ */
