# XDriver

基于 **STM32F103CBT6 + RT-Thread 4.1.1** 的步进电机 FOC 闭环驱动器固件。

FOC 控制算法移植自
[Dummy-Robot Ctrl-Step-Driver](https://github.com/peng-zhihui/Dummy-Robot)（原本是 C++ / 裸机），
已**完整改写为纯 C**，并运行在 RT-Thread 之上。

| 项 | 值 |
|---|---|
| MCU | STM32F103CBT6（LQFP48，128 KB Flash / 20 KB RAM） |
| 时钟 | HSE 12 MHz × PLL6 = **72 MHz** |
| RTOS | RT-Thread 4.1.1（内核 + FinSH/msh，未启用设备框架） |
| 编码器 | MT6816 磁编码器（14 bit，SPI1） |
| 驱动芯片 | TB67H450（电流矢量 → 双相 PWM） |
| 控制频率 | **20 kHz** 电流/位置/速度环（TIM4 中断） |
| 当前占用 | Flash 57220 B / 95 KB（58.8%），RAM 7728 B / 20 KB（37.7%） |

---

## 快速开始

### 编译

```bash
cmake --preset Debug          # 配置
cmake --build --preset Debug   # 编译
```

产物：`build/Debug/XDriver.elf`（HEX/BIN 用 `arm-none-eabi-objcopy`，
或直接用 VS Code 任务 **Build + HEX/BIN**）。

VS Code 任务（`Ctrl+Shift+P` → `Tasks: Run Task`）：

| 任务 | 作用 |
|---|---|
| `Configure` | `cmake --preset Debug` |
| `Build` | 编译 |
| `Clean` | 清理 |
| `Build + HEX/BIN` | 编译并生成 `.hex` / `.bin` |
| `Post-CubeMX Patch` | CubeMX 重新生成代码后，恢复 RT-Thread 集成补丁 |
| `Post-CubeMX Patch (check only)` | 只报告需要修改的地方（不写文件） |

### 串口控制台

**USART1：PB6 = TX、PB7 = RX，115200 8N1**（USART1 重映射）。

上电后应看到 RT-Thread 横幅和 `msh >` 提示符，输入 `help` 查看命令。

---

## 引脚分配

| 引脚 | 功能 | 说明 |
|---|---|---|
| PA0 / PA1 | ADC1_IN0 / IN1 | 母线电压 `POWER_U` / 驱动温度 `DRV_TEMP`（当前未启用） |
| PA2 / PA3 / PA4 / PA5 | GPIO 输出 | TB67H450 方向脚 `HW_ELEC_BM / BP / AM / AP` |
| PA6 | GPIO 输出 | `debug_1`，主循环 1 Hz 方波（系统存活指示） |
| PA7 | — | 未配置（源工程的 step-dir 计数脚，源工程里也是死代码） |
| PA13 / PA14 | SWD | 调试口（`SWJ_NOJTAG`，PA15/PB3/PB4 因此可用） |
| PA15 | GPIO 输出 | MT6816 片选 `SPI1_CS`（初始高） |
| PB0 | GPIO 输入 | `SIGNAL_COUNT_EN` |
| PB1 | GPIO 输出 | `SIGNAL_ALERT` |
| PB2 / PB12 | GPIO 输入（上拉） | `BUTTON2` / `BUTTON1`（低有效） |
| PB3 / PB4 / PB5 | SPI1_SCK / MISO / MOSI | **SPI1 重映射**，Mode 3，16 bit，9 MHz |
| PB6 / PB7 | USART1_TX / RX | **USART1 重映射**，FinSH 控制台 |
| PB8 / PB9 | CAN_RX / CAN_TX | **CAN1 重映射 2**，1 Mbit/s（协议未实现） |
| PB10 / PB11 | TIM2_CH3 / CH4 | 电流 PWM `HW_ELEC_BPWM / APWM`，70.3 kHz |
| PC13 / PC14 | GPIO 输出 | `LED1` / `LED2`（低电平点亮） |
| PD0 / PD1 | HSE | 12 MHz 晶振 |

### 定时器 / 中断

| 外设 | 用途 | 优先级 |
|---|---|---|
| **TIM4** | 20 kHz FOC 闭环（`foc_tick_20khz()`） | **0（最高）** |
| TIM2 | 双相电流 PWM（ARR = 1023） | — |
| TIM1_UP | HAL 时基（`HAL_GetTick`），**不是**应用节拍 | 15 |
| SysTick | RT-Thread OS tick（1000 Hz） | 15（最低） |
| PendSV | 线程上下文切换 | 15（最低） |
| USART1 | 控制台接收 | 5 |
| CAN1 ×4 | 未启用协议 | 3 |
| DMA1_Ch1 | ADC（循环，未启用） | 15 |

> RT-Thread 要求 SysTick / PendSV 为最低优先级，FOC 闭环为最高优先级 ——
> 这保证了电流环不会被任何线程或 tick 推迟。

---

## Flash 布局

```
0x08000000 ┌──────────────────────────┐
           │  代码区  95 KB           │  ← 链接脚本 FLASH LENGTH = 0x17C00
0x08017C00 ├──────────────────────────┤
           │  编码器校准表  32 KB     │  16384 × uint16_t 全分辨率查表
0x0801FC00 ├──────────────────────────┤
           │  板级配置页  1 KB        │  BoardConfig_t + 模拟 EEPROM
0x08020000 └──────────────────────────┘
```

`tools/post_cubemx_patch.ps1` 会在每次 CubeMX 重新生成代码后把 `LENGTH` 钉回
`0x17C00`。**若手动改回 128K，链接器会把代码铺到校准表上且不报错**，表现为
校准白做（重启后 `xdrv` 显示 `calib: NO`）。

---

## msh 命令

所有 FOC 操作收敛到单个顶层命令 **`xdrv`**，避免污染 `help` 输出。
不带参数直接打印状态。

| 命令 | 作用 |
|---|---|
| `xdrv` / `xdrv status` | 状态：state / mode / pos / vel / current / calib / cfg |
| `xdrv help` | 子命令列表 |
| `xdrv cali` | 编码器校准（要求电机已停止） |
| `xdrv mode <stop\|pos\|vel\|cur>` | 切换控制模式 |
| `xdrv pos <milli-rev>` | 目标位置，**1000 = 1.000 圈** |
| `xdrv vel <milli-rev/s>` | 目标速度，**1000 = 1 圈/秒** |
| `xdrv cur <mA>` | 目标电流 |
| `xdrv on` / `off` / `brake` | 使能 / 失能 / 制动 |
| `xdrv home` | 把当前位置设为零点（要求停机） |
| `xdrv save` | 保存配置到 Flash（要求停机） |
| `xdrv enc` | MT6816 SPI 通路探测 |
| `xdrv loop <on\|off>` | 手动启停 20 kHz 闭环 |

### 首次使用流程

```
msh > xdrv                 # 应显示 calib: NO
msh > xdrv cali            # 校准（电机空载、可自由转动！）
                           # 电机会正转一圈、反转一圈，然后自动重启
msh > xdrv                 # 现在应显示 calib: yes

msh > xdrv pos 1000        # 转到 1 圈
msh > xdrv vel 500         # 半圈/秒
msh > xdrv cur 300         # 300 mA 力矩
```

> **校准必须空载**：校准过程用电流矢量强制拖动电机转两圈，
> 带负载会导致采样不连续而失败。失败时重启后 `xdrv` 仍显示 `calib: NO`，
> 且 `stall` 字段为 `1`（上游用它标记校准失败），Flash 校准区会被清空。

---

## 目录结构

```
├── Core/                    CubeMX 生成的 HAL 初始化代码
│   ├── Inc/                 main.h（引脚标签）、rt_board.h、HAL 配置
│   └── Src/
│       ├── main.c           main() → foc_app_init()
│       ├── rt_board.c       RT-Thread 板级移植（控制台 / 堆 / OS tick）
│       └── stm32f1xx_it.c   中断向量（SysTick→RT tick，TIM4→FOC 闭环）
├── FOC/                     FOC 算法（移植自 Dummy-Robot，纯 C）
│   ├── Ctrl/
│   │   ├── Driver/          driver_base.h  sin_map  tb67h450_base
│   │   ├── Motor/           motor_types.h  motion_planner  motor
│   │   ├── Sensor/Encoder/  encoder_base  mt6816_base  encoder_calibrator_base
│   │   └── Signal/          button_base  led_base
│   ├── Port/                板级实现（TB67H450 / MT6816 / Flash / 按键 / LED）
│   ├── UserApp/             app_foc.c（全局对象 + 20kHz 入口 + 100Hz 线程 + msh）
│   └── docs/FOC_C_PORT.md   C 化约定、符号对照表、进度
├── RT-Thread/               内核源码（4.1.1，直接编入 elf）
├── cmake/
│   ├── foc.cmake            FOC 源文件清单（显式列表）
│   ├── rtthread.cmake       内核 + FinSH 构建
│   └── stm32cubemx/         CubeMX 生成的外设源文件
├── tools/post_cubemx_patch.ps1
└── 文档
    ├── RTTHREAD_PORT.md     RT-Thread 移植记录（控制台 / 中断 / 堆）
    ├── FOC/docs/FOC_C_PORT.md   C++ → C 移植约定
    └── 更新计划.md           完整移植计划
```

---

## 设计要点

### 1. 用 C 模拟面向对象

上游是 C++ 代码，移植时保留了三层抽象但不使用 C++：

| 模式 | 用途 | 形态 |
|---|---|---|
| **虚表** | 会被 `Motor` 以基类指针持有的对象 | `DriverBase` / `EncoderBase` 内含函数指针 + `static inline` 包装 |
| **基类内嵌 + PortOps** | 芯片相关实现 | `TB67H450Base` 内嵌 `DriverBase base`（首成员）+ `const TB67H450PortOps *ops` |
| **纯组合** | 无多态的业务对象 | `Motor` / `MotionPlanner` / `MotionController` + `self` 首参 |

好处：不需要 `enable_language(CXX)`、无全局构造顺序问题、
**0 字节 C++ 运行时开销**（镜像里没有 `__cxa_*` / `_ZTV*` / `__aeabi_unwind*`）。

细节见 [`FOC/docs/FOC_C_PORT.md`](FOC/docs/FOC_C_PORT.md)。

### 2. 实时性分工

```
TIM4 中断 (20 kHz, 优先级 0)      ← 中断上下文，禁止调用任何 RT-Thread API
  └ foc_tick_20khz()
      ├ 编码器读角度（SPI1）
      ├ 位置/速度估计
      └ 位置环 / 速度环 / 电流环 → TIM2 PWM

foc_hk 线程 (100 Hz, 优先级 12)   ← 线程上下文
  ├ 按键扫描 / 指示灯
  └ Flash 校准写入（阻塞数十 ms，仅在电机休眠时执行）

msh 线程 (FinSH)                  ← 线程上下文
  └ xdrv 命令（改设定点时用临界区保护）
```

### 3. Flash 写入保护

Flash 擦除会暂停 CPU 取指 20~40 ms，期间 PWM 保持最后占空比 → 电机继续出力。
因此 `xdrv cali / home / save` 都会检查电机是否已停止，不满足则拒绝执行：

```
msh > xdrv save
refuse save: motor is active (mode=vel)
        run 'xdrv mode stop' or 'xdrv off' first
```

---

## 重新生成 CubeMX 代码后

CubeMX 会覆盖 `USER CODE` 区之外的所有内容，因此必须重新打补丁：

```bash
powershell -ExecutionPolicy Bypass -File tools\post_cubemx_patch.ps1
```

脚本会（幂等）修复：

1. `startup_*.s`：`Reset_Handler` 的 `bl main` → `bl entry`
2. `*.ld`：补 `.rti_fn` / `.fsymtab` / `.vsymtab` 段；
   `FLASH LENGTH` → `0x17C00`
3. `stm32f1xx_it.c`：删除 `HardFault_Handler` / `PendSV_Handler`（RT-Thread 提供）；
   `SysTick` → `rt_tick_increase()`；`USART1` → `rt_hw_console_isr()`
4. `main.c`：`MX_*_Init()` 去 `static`；从 `main()` 移除 `HAL_Init` / 时钟 / 外设初始化
5. `CMakeLists.txt`：确保 `include(cmake/rtthread.cmake)`

> `TIM4_IRQHandler` 里调用 `foc_tick_20khz()` 与 `main()` 里的 `foc_app_init()`
> 都写在 `USER CODE` 区内，CubeMX 重新生成时**不会**被清除。

---

## 已知限制

| 项 | 说明 |
|---|---|
| CAN 协议 | CAN1 硬件与位时序已配置（1 Mbit/s），应用层协议未实现 |
| ADC | 母线电压 / 驱动温度采样未启用（源工程里也未被读取） |
| step-dir 模式 | 枚举值保留，未接入（PA6 现用于调试指示） |
| MT6816 超时 | 读角度用 `HAL_MAX_DELAY`，编码器掉线会导致 20 kHz 中断永久等待。首刷排障可先用 `xdrv enc` 探测 |
| 配置版本 | `BoardConfig_t` 无 magic/version 字段，结构变更后旧 Flash 内容会被当成有效配置 |

---

## 参考

- [Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot) —— FOC 算法与硬件设计来源
- [RT-Thread](https://github.com/RT-Thread/rt-thread) —— 实时操作系统
