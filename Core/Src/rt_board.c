/*
 * rt_board.c
 *
 * XDriver 的 RT-Thread 板级移植 (STM32F103C8T6)
 *
 *  - 板级初始化: HAL / 时钟 / GPIO / DMA / USART1 / 堆 / SysTick(OS tick)
 *  - 控制台: USART1 (PB6=TX, PB7=RX, 115200 8N1, 默认重映射)
 *      * 发送: 直接寄存器轮询 (rt_hw_console_output / __io_putchar)
 *      * 接收: USART1 中断 -> 256 字节环形缓冲 -> FinSH 读取
 *
 * 注意:
 *  - 这里不使用 HAL 的 HAL_UART_Receive_IT, 因为单字节 IT 链路在大量
 *    输入时容易因为 HAL 状态机 (RxState) 卡死丢数据; 直接操作寄存器更可靠
 *  - 不使用 RT_USING_DEVICE, FinSH 直接调用本文件中的 console 钩子函数
 */

#include <rtthread.h>

#include "main.h"
#include "rt_board.h"

/*
 * CubeMX (CoupleFile = true) 把各外设初始化拆到了独立文件,
 * 这些 MX_*_Init() 由本文件的 rt_hw_board_init() 统一调用,
 * 因此必须在这里包含对应的头文件 (main.c 不再负责外设初始化)。
 */
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"

/*
 * 控制台引脚选择:
 *   1 = PB6(TX) / PB7(RX)  -- 默认, USART1 重映射
 *   0 = PA9(TX) / PA10(RX) -- 备用, PB6/PB7 电路有问题时可用
 *
 * 改这里后重新编译即可, 外部接线跟着改到 PA9/PA10。
 */
#define CONSOLE_USE_REMAP       1

/* 控制台引脚 (由 CONSOLE_USE_REMAP 决定) */
#if CONSOLE_USE_REMAP
#define CONSOLE_TX_PORT         GPIOB
#define CONSOLE_TX_PIN          GPIO_PIN_6
#define CONSOLE_RX_PORT         GPIOB
#define CONSOLE_RX_PIN          GPIO_PIN_7
#else
#define CONSOLE_TX_PORT         GPIOA
#define CONSOLE_TX_PIN          GPIO_PIN_9
#define CONSOLE_RX_PORT         GPIOA
#define CONSOLE_RX_PIN          GPIO_PIN_10
#endif

/* -------------------------------------------------------------------------- */
/* 控制台接收环形缓冲                                                          */
/* -------------------------------------------------------------------------- */
#define CONSOLE_RX_BUF_SIZE     256U
#define CONSOLE_RX_BUF_MASK     (CONSOLE_RX_BUF_SIZE - 1U)

static volatile rt_uint8_t  console_rx_buf[CONSOLE_RX_BUF_SIZE];
static volatile rt_uint16_t console_rx_head = 0U;   /* 中断写入位置 */
static volatile rt_uint16_t console_rx_tail = 0U;   /* FinSH 读取位置 */
static struct rt_semaphore  console_rx_sem;

extern UART_HandleTypeDef huart1;

/* -------------------------------------------------------------------------- */
/* USART1 中断: 读取 SR + DR 清除所有错误标志, 把有效字节放进环形缓冲        */
/* -------------------------------------------------------------------------- */
void rt_hw_console_isr(void)
{
    /* 必须先读 SR 再读 DR, 才能清掉 RXNE/ORE/NE/FE/PE */
    rt_uint32_t sr = USART1->SR;
    rt_uint8_t  dr = (rt_uint8_t)USART1->DR;

    if (sr & USART_SR_RXNE)
    {
        rt_uint16_t next = (rt_uint16_t)((console_rx_head + 1U) & CONSOLE_RX_BUF_MASK);

        if (next != console_rx_tail)
        {
            console_rx_buf[console_rx_head] = dr;
            console_rx_head = next;

            /* 唤醒阻塞在 rt_hw_console_getchar() 的 FinSH 线程 */
            rt_sem_release(&console_rx_sem);
        }
        /* 环形缓冲满时丢弃该字节 */
    }
}

/* -------------------------------------------------------------------------- */
/* 控制台输出                                                                  */
/* -------------------------------------------------------------------------- */
static void console_putc(char c)
{
    rt_uint32_t timeout = 1000000U;

    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
        if (--timeout == 0U)
        {
            return;             /* USART 未初始化/异常时避免死等 */
        }
    }
    USART1->DR = (rt_uint8_t)c;
}

/* newlib printf 重定向 (syscalls.c 的 _write 会调用它) */
int __io_putchar(int ch)
{
    console_putc((char)ch);
    return ch;
}

/* rt_kprintf / FinSH 输出钩子 */
void rt_hw_console_output(const char *str)
{
    if (str == RT_NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        if (*str == '\n')
        {
            console_putc('\r');
        }
        console_putc(*str);
        str++;
    }

    /* 等待最后一个字节真正移出 (TC), 方便调试时断电/复位不丢字符 */
    {
        rt_uint32_t timeout = 1000000U;
        while ((USART1->SR & USART_SR_TC) == 0U)
        {
            if (--timeout == 0U)
            {
                break;
            }
        }
    }
}

/* FinSH 输入钩子: 阻塞等待字节 (无数据时让出 CPU, 避免空转) */
char rt_hw_console_getchar(void)
{
    register char ch;

    while (console_rx_tail == console_rx_head)
    {
        /* 调度器未启动时不能阻塞 (例如早期 rt_kprintf) */
        if (rt_thread_self() == RT_NULL)
        {
            return (char)-1;    /* ARM 上 char 无符号 => 0xFF, FinSH 会忽略 */
        }

        if (rt_sem_take(&console_rx_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            return (char)-1;
        }
    }

    ch = (char)console_rx_buf[console_rx_tail];
    console_rx_tail = (rt_uint16_t)((console_rx_tail + 1U) & CONSOLE_RX_BUF_MASK);
    return ch;
}

/* -------------------------------------------------------------------------- */
/* 板级初始化 (rtthread_startup 的第一步)                                      */
/* -------------------------------------------------------------------------- */
void rt_hw_board_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1) HAL + 时钟 + 外设 (复用 CubeMX 生成的初始化函数)
     *    顺序与源工程 (Dummy-Robot Ctrl-Step-Driver) 的 main() 保持一致 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_CAN_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();

    /* 2) RX 引脚配置为上拉输入 (CubeMX 生成的是 NOPULL / 重映射由 CMSIS 宏控制)
     *    F1 的 GPIO: INPUT + PULLUP => CNF=10, MODE=00, ODR=1 */
    gpio.Pin   = CONSOLE_RX_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CONSOLE_RX_PORT, &gpio);

#if (!CONSOLE_USE_REMAP)
    /* 改用 PA9/PA10: 关闭 USART1 重映射, 并把 PB6/PB7 释放为浮空输入
     * (CubeMX 生成的 MSP 仍会把它们配成 USART1 的复用脚) */
    __HAL_AFIO_REMAP_USART1_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);

    gpio.Pin  = CONSOLE_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CONSOLE_TX_PORT, &gpio);

    gpio.Pin  = CONSOLE_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CONSOLE_RX_PORT, &gpio);
#endif
    /* 3) 控制台接收: 信号量 + RXNE 中断 */
    rt_sem_init(&console_rx_sem, "conrx", 0, RT_IPC_FLAG_FIFO);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 4) RT-Thread 堆: 从 .bss 结束处到主栈 (_sstack) 之间
     *    _sstack 由链接脚本定义 = RAM 末尾 - _Min_Stack_Size(1KB) */
#ifdef RT_USING_HEAP
    {
        extern char __bss_end;
        extern char _sstack;

        rt_system_heap_init((void *)(((rt_uint32_t)&__bss_end + 7U) & ~7UL),
                            (void *)&_sstack);
    }
#endif /* RT_USING_HEAP */

    /* 5) OS tick 使用 SysTick (HAL 的时基仍然使用 TIM1, 二者互不影响) */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* RT-Thread 要求 PendSV / SysTick 为最低优先级 */
    NVIC_SetPriority(PendSV_IRQn, 0xFFU);
    NVIC_SetPriority(SysTick_IRQn, 0xFFU);

    /* 6) 执行 INIT_BOARD_EXPORT 注册的板级初始化函数 */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
