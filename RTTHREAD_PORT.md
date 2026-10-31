# RT-Thread + FinSH 移植说明（XDriver / STM32F103C8T6）

本项目已集成 **RT-Thread 4.1.1 内核 + FinSH(msh) 控制台**，控制台复用 CubeMX 已配置的
**USART1**（PB6=TX，PB7=RX，115200 8N1，默认重映射）。

## 1. 目录与文件

| 文件 | 作用 |
| --- | --- |
| `rtconfig.h` | RT-Thread 裁剪配置（根目录，`rtthread.h` 会自动包含） |
| `Core/Inc/rt_board.h` | 板级接口声明 |
| `Core/Src/rt_board.c` | 板级初始化、USART1 控制台、堆、OS tick |
| `cmake/rtthread.cmake` | 把内核 / libcpu / FinSH / 板级文件加入构建 |
| `CMakeLists.txt` | `include(cmake/rtthread.cmake)` |
| `STM32F103xx_FLASH.ld` | 增加 `.rti_fn.*`、`FSymTab`、`VSymTab` 输出段 |
| `startup_stm32f103xb.s` | `Reset_Handler` 改为 `bl entry`（RT-Thread 入口） |
| `Core/Src/main.c` | `main()` 运行在 RT-Thread main 线程；示例 msh 命令 `hello` |
| `Core/Src/stm32f1xx_it.c` | SysTick 提供 OS tick；USART1 ISR 喂控制台环形缓冲 |

## 2. 启动流程

```
Reset_Handler
  -> SystemInit / 拷贝 .data / 清零 .bss / __libc_init_array
  -> entry()                        (RT-Thread/src/components.c)
     -> rtthread_startup()
        -> rt_hw_board_init()       (Core/Src/rt_board.c)
             HAL_Init / SystemClock_Config
             MX_GPIO_Init / MX_DMA_Init / MX_USART1_UART_Init
             PB7 上拉 + USART1 RXNE 中断 + 控制台信号量
             rt_system_heap_init(__bss_end .. _sstack)
             SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND)
        -> rt_show_version()
        -> rt_system_timer_init / rt_system_scheduler_init
        -> rt_application_init()    // 创建 main 线程
        -> rt_system_scheduler_start()
             main 线程: rt_components_init() -> finsh_system_init() -> main()
```

- **OS tick**：SysTick（`SysTick_Handler` 中 `rt_tick_increase()`）。
- **HAL 时基**：仍然是 TIM1（`TIM1_UP_IRQHandler` -> `HAL_IncTick()`），
  所以 `HAL_Delay()` / `HAL_GetTick()` 继续可用。
- `PendSV_Handler`、`HardFault_Handler` 由
  `RT-Thread/libcpu/arm/cortex-m3/context_gcc.S` 提供，**不能在 `stm32f1xx_it.c` 中重复定义**。

## 3. 控制台（USART1）

- 发送：`rt_hw_console_output()` 直接轮询 `USART1->SR/DR`（`\n` 自动补 `\r`）。
  `printf` 通过 `syscalls.c -> __io_putchar()` 同样输出到 USART1。
- 接收：`USART1_IRQHandler` -> `rt_hw_console_isr()`
  读 `SR` + 读 `DR`（一次清掉 RXNE/ORE/NE/FE/PE）-> 256 字节环形缓冲
  -> `rt_hw_console_getchar()` 用信号量阻塞等待（不空转 CPU）。
- **`USART1_IRQHandler` 必须是强定义**：启动文件里有
  `.weak USART1_IRQHandler` + `.thumb_set ..., Default_Handler`，
  如果项目里也写成 weak，链接器会选 `Default_Handler`，表现为能看到 `msh >` 但键盘无反应。
- 不使用 `HAL_UART_Receive_IT`：单字节 HAL 链路在连续输入时容易因 `RxState`
  停留在 `BUSY_RX` 而停摆，直接寄存器 ISR 更可靠且更省 FLASH。

串口助手设置：`115200, 8, N, 1`，勾选发送新行/回车（`CR` 或 `CRLF`）。

## 4. FinSH 使用

上电后串口应输出 RT-Thread 版本、`[main] ...` 启动信息以及 `msh >` 提示符。常用命令：

```
help            # 列出所有命令
ps              # 查看线程
free            # 查看堆使用
list_thread     # 线程列表
hello           # 本项目示例命令
hello Alice     # 带参数
```

添加自己的命令（任意 `.c` 文件）：

```c
#include <rtthread.h>

static void my_cmd(int argc, char **argv)
{
    rt_kprintf("my_cmd argc=%d\n", argc);
}
MSH_CMD_EXPORT(my_cmd, my test command);
```

命令表位于 `FSymTab` 段，链接脚本已 `KEEP`，不会因 `--gc-sections` 被丢弃。

## 5. 内存

- 堆：`__bss_end` 到 `_sstack`（RAM 末尾 − 1KB 主栈），约 17KB。
- `RT_USING_SMALL_MEM` 小内存算法；`FINSH_THREAD_STACK_SIZE=2048`，
  `RT_MAIN_THREAD_STACK_SIZE=1024`。
- 当前构建：**RAM ≈ 3.3KB 静态 + 堆**，**FLASH ≈ 29KB / 128KB**。
- 注意：`syscalls.c` 的 `_sbrk()` 起点也在 `__bss_end` 附近，**不要混用 newlib `malloc`
  和 `rt_malloc`**，统一使用 RT-Thread 的 `rt_malloc/rt_free`。

## 6. 重新用 CubeMX 生成代码后必须检查

CubeMX 会覆盖以下内容，需要重新应用：

1. `startup_stm32f103xb.s`：`bl main` 改回 `bl entry`。
2. `STM32F103xx_FLASH.ld`：确认 `.rti_fn` / `.fsymtab` / `.vsymtab` 三个输出段仍存在。
3. `Core/Src/stm32f1xx_it.c`：
   - 删除 `HardFault_Handler` 与 `PendSV_Handler`（RT-Thread 提供）；
   - `SysTick_Handler` 中加入 `rt_interrupt_enter(); rt_tick_increase(); rt_interrupt_leave();`；
   - `USART1_IRQHandler` 改为调用 `rt_hw_console_isr();`（不要调用 `HAL_UART_IRQHandler`）。
4. `Core/Src/main.c`：
   - `MX_GPIO_Init` / `MX_DMA_Init` / `MX_USART1_UART_Init` 去掉 `static`（`rt_board.c` 要调用）；
   - 保留 `USER CODE` 区内的 `hello` 命令，`main()` 中不要再做 HAL/时钟初始化。
5. `Core/Src/stm32f1xx_hal_msp.c`：PB7 的 `GPIO_NOPULL` 可保留，
   `rt_hw_board_init()` 会重新配置为 `GPIO_PULLUP`。

## 7. 构建与烧录

```powershell
cmake --preset Debug
cmake --build --preset Debug
arm-none-eabi-objcopy -O ihex build/Debug/XDriver.elf build/Debug/XDriver.hex
arm-none-eabi-objcopy -O binary build/Debug/XDriver.elf build/Debug/XDriver.bin
```

## 8. 故障排查：能输出但键盘没反应

固件已不含串口测试代码，排查按硬件顺序做：

1. **终端关闭硬件流控（RTS/CTS）**——最常见原因，PC 侧会完全不发送。
2. **接线**：适配器 **TX → PB7**（`CONSOLE_USE_REMAP=1` 时），RX → PB6，**GND 共地**。
   USART1 已重映射，MCU 的 RX 是 PB7，**不是 PA10**。
3. **万用表量 RX 引脚对 GND 电压**：空闲必须 ≈3.3V。
   - ≈0V → RX 被外部硬拉低（短路到 GND / 接到别的输出脚 / 未供电芯片）。
     STM32 的 USART 要靠"空闲高 → 低"的起始位启动接收，**RX 持续为低时收不到任何数据，
     也不会产生帧错误**（`SR` 只有 `TXE|TC`），很容易误判成软件没进中断。
   - 负压 → 接到 RS232 电平，**立即断开**，否则会损坏引脚。
4. 拔掉 RX 线复位：若 `msh >` 仍正常、接上线后无输入 → 问题在这根线/适配器一侧。
5. 确认适配器 VCC 已供电、杜邦线没有插到 GND。

软件侧可自查的点：`USART1_IRQHandler` 必须是强定义（启动文件里同名符号是 weak，
若项目里也写 weak，链接器会落到 `Default_Handler`，表现为有提示符但输入无反应）。

### 应急方案：改用 PA9/PA10

`Core/Src/rt_board.c` 顶部改 `CONSOLE_USE_REMAP` 为 `0`，重编译，把线换到
PA9(TX)/PA10(RX) 即可绕开 PB6/PB7 电路。


