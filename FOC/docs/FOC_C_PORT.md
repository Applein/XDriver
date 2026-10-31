# FOC 纯 C 移植约定

> 上游：`Dummy-Robot/2.Firmware/Ctrl-Step-Driver-STM32F1-fw`（C++/C 混合，裸机）
> 本工程：`FOC/`（纯 C + RT-Thread）
> 本次移植把上游的类/虚函数/模板改写成 `struct` + 函数指针，**不启用 C++**。

`FOC/` 下的 `*.cpp` 是**上游参照实现**，不参与构建（见 `cmake/foc.cmake` 的 `FOC_SRC`
显式列表）。转换完一个模块后，把对应 `.c` 加入列表；**等同名 `.c` 通过验证并提交后**
再删除 `.cpp`，在此之前保留以便逐行 `git diff` 对照。

---

## 1. 命名约定

| 对象 | 规则 | 示例 |
|---|---|---|
| 类型 | `大驼峰` | `Motor`, `MotionController`, `DriverBase` |
| 函数 | `模块前缀_小写动词` | `motor_tick_20khz()`, `motion_ctrl_set_position_set_point()` |
| 虚表方法 | 小写动词（通过 `obj->method(obj, ...)` 调用） | `driver->set_foc_current_vector(driver, dir, mA)` |
| 成员函数包装 | `模块前缀_小写动词`，`static inline` 放头文件 | `driver_sleep(d)` |
| 字段 | `snake_case` | `soft_position`, `real_lap_position` |
| 枚举值 | `大写下划线` + 模块前缀 | `MOTOR_MODE_STOP`, `CALI_STATE_FORWARD_MEASURE` |
| 编译期常量 | 文件/头文件级 `enum` | `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 51200` |
| 文件内辅助函数 | `static` | `cycle_mod()`, `compensate_advanced_angle()` |

`self` 统一作为第一个参数名。

---

## 2. 三种面向对象模拟模式

### 2.1 虚表（多态）

用于会被 `Motor` 以基类指针持有的对象：`DriverBase`、`EncoderBase`。

```c
typedef struct DriverBase DriverBase;
struct DriverBase {
    void (*init)(DriverBase *self);
    void (*set_foc_current_vector)(DriverBase *self, uint32_t dir_in_count, int32_t current_ma);
    void (*sleep)(DriverBase *self);
    void (*brake)(DriverBase *self);
    FastSinToDac phase_a, phase_b;      /* 原 protected 成员 */
};

static inline void driver_sleep(DriverBase *d) { d->sleep(d); }
```

调用点写成 `driver->sleep(driver)` / `driver_sleep(driver)`，与原 `motor->driver->Sleep()`
一一对应，便于 diff。

### 2.2 基类内嵌 + PortOps（替代继承链）

用于芯片实现：`TB67H450Base`、`MT6816Base`、`EncoderCalibrator`。

```c
struct TB67H450Base {
    DriverBase base;                        /* 必须是第一个成员 */
    const TB67H450PortOps *ops;             /* 板级实现 */
};
```

- `base` **必须是第一个成员**：这样才能用 `(TB67H450Base *)driver_ptr` 从基类指针取回自身。
- `PortOps` 是 `const` 结构体，放在 `Port/*.c` 里，替代原 `protected` 纯虚函数。

### 2.3 纯组合（业务层）

无多态的模块（`Motor`、`MotionPlanner`、`MotionController`）直接用 struct + 首参 `self`：

```c
void motor_tick_20khz(Motor *self);
void motion_planner_calc_soft_goal(MotionPlanner *self, int32_t goal);
```

---

## 3. 符号对照表

| C++ | C |
|---|---|
| `Motor::Controller` | `MotionController` + `motion_ctrl_*()` |
| `Motor::Controller::PID_t` | `ControllerPid` |
| `Motor::Controller::DCE_t` | `ControllerDce` |
| `Motor::Controller::Config_t` | `ControllerConfig` |
| `Motor::Config_t` | `MotorConfig` |
| `Motor::Mode_t` / `MODE_*` | `MotorMode` / `MOTOR_MODE_*` |
| `Motor::State_t` / `STATE_*` | `MotorState` / `MOTOR_STATE_*` |
| `MotionPlanner::CurrentTracker` | `CurrentTracker` + `current_tracker_*()` |
| `MotionPlanner::VelocityTracker` | `VelocityTracker` + `velocity_tracker_*()` |
| `MotionPlanner::PositionTracker` | `PositionTracker` + `position_tracker_*()` |
| `MotionPlanner::PositionInterpolator` | `PositionInterpolator` + `position_interpolator_*()` |
| `MotionPlanner::TrajectoryTracker` | `TrajectoryTracker` + `trajectory_tracker_*()` |
| `ButtonBase::Event` / `UP..CLICK` | `ButtonEvent` / `BTN_EVT_UP..BTN_EVT_CLICK` |
| `EncoderBase::AngleData_t` | `EncoderAngleData` |
| `EncoderCalibratorBase::State_t` / `CALI_*` | `CaliState` / `CALI_STATE_*` |
| `EncoderCalibratorBase::Error_t` / `CALI_ERROR_*` | `CaliError` / `CALI_ERR_*` |
| `EncoderBase::RESOLUTION` | `ENCODER_RESOLUTION`（`enum { ... = 1 << 14 }`） |
| `EEPROM::get<T>` / `put<T>` | `eeprom_get_blob()` / `eeprom_put_blob()` |

---

## 4. 转换自查清单

每转换一个模块，逐条确认：

1. **头文件不放变量定义** —— `sin_map.h` 的数组定义必须拆到 `sin_map.c`，否则多 TU `multiple definition`。
2. **构造函数 → `*_init()`** —— `angleData{0}`、`spiRawData{0}` 这类零初始化要显式写出来。
3. **`static` 局部变量 → 结构体字段** —— 如 `CloseLoopControlTick()` 的 `isFirstCalled` → `is_first_called`。
4. **`context` 反向指针逐个接线** —— `motion_planner_init()` 里 5 个 tracker 的 `context` 各一行，漏一行就是运行期空指针。
5. **成员常量去掉对象前缀** —— `motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` → `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS`。
6. **枚举统一加前缀** —— C 的枚举在全局命名空间，`UP/DOWN/CLICK` 这类名字必须加 `BTN_EVT_`。
7. **补 C 头文件** —— `<stdbool.h>` `<stdint.h>` `<stdlib.h>`(abs) `<math.h>`(sqrtf)。
8. **严格原型** —— 所有无参函数写 `(void)`；用 `-Wstrict-prototypes` 检查。
9. **`nullptr` → `NULL`**。
10. **`static const` 类成员 → `enum`**。
11. **构造函数重载 → 单函数 + 显式参数**（如 `button_init(self, id, long_press_ms, read_pin_io)`）。
12. **结构体不能有默认成员初始化器** —— 用 `*_init()` 或 `= {0}`。

编译校验（单文件）：

```
arm-none-eabi-gcc -c -std=gnu11 -Wall -Wextra -Wstrict-prototypes \
  -mcpu=cortex-m3 -mthumb -Os -I FOC -I FOC/Ctrl -I FOC/Port -I FOC/UserApp \
  -I Core/Inc -I Drivers/STM32F1xx_HAL_Driver/Inc \
  -I Drivers/CMSIS/Device/ST/STM32F1xx/Include -I Drivers/CMSIS/Include \
  -DUSE_HAL_DRIVER -DSTM32F103xB <file.c>
```

要求 **0 警告**。

### Port 层文件的提前验证

`Port/*.c` 依赖 CubeMX 生成的 `main.h` 引脚标签（`BUTTON1_Pin`、`LED1_GPIO_Port` …），
而阶段 D 会补上这些标签。在标签生成之前，可以用**临时 `-D` 宏**验证语法：

```
arm-none-eabi-gcc <同上 flags> \
  -DBUTTON1_Pin=GPIO_PIN_12 -DBUTTON1_GPIO_Port=GPIOB \
  -DBUTTON2_Pin=GPIO_PIN_2  -DBUTTON2_GPIO_Port=GPIOB \
  -c FOC/Port/button_stm32.c -o /tmp/btn.o
```

这样能把“C 语法/C 化是否正确”和“CubeMX 引脚是否就绪”两个问题分开定位。

---

## 6. 转换进度

| 模块 | 目标文件 | 状态 |
|---|---|---|
| 类型定义 | `Ctrl/Motor/motor_types.h` | ✅ 完成 |
| 信号层 base | `Ctrl/Signal/button_base.{h,c}` | ✅ 完成，已入 `FOC_SRC` |
| 信号层 base | `Ctrl/Signal/led_base.{h,c}` | ✅ 完成，已入 `FOC_SRC` |
| 信号层 Port | `Port/button_stm32.{h,c}` | ✅ 转换完成，**待阶段 D 引脚标签**后入 `FOC_SRC` |
| 信号层 Port | `Port/led_stm32.{h,c}` | ✅ 转换完成，**待阶段 D 引脚标签**后入 `FOC_SRC` |
| 驱动层 base | `Ctrl/Driver/…` | ⬜ 未开始 |
| 编码器层 | `Ctrl/Sensor/Encoder/…` | ⬜ 未开始 |
| 规划器 | `Ctrl/Motor/motion_planner.*` | ⬜ 未开始 |
| 电机 | `Ctrl/Motor/motor.*` | ⬜ 未开始 |
| 校准器 | `Ctrl/Sensor/Encoder/encoder_calibrator_base.*` | ⬜ 未开始 |
| 存储层 | `Port/Platform/Memory/…` | ⬜ 未开始 |
| 应用层 | `UserApp/app_foc.{h,c}` | ⬜ 未开始 |

---

## 5. 不参与编译的文件

| 文件 | 原因 |
|---|---|
| `Port/Platform/retarget.c` | `_write` / `__io_putchar` 与 `Core/Src/syscalls.c` + `rt_board.c` 重复定义 |
| `UserApp/protocols/interface_uart.cpp` | USART1 由 FinSH 独占，原 ASCII 协议不移植 |
| `UserApp/main.cpp` | 改名 `app_foc.c`，避免与 `Core/Src/main.c` 冲突 |
| `FOC/**/*.cpp` | 上游参照实现，仅用于对照 |

---

## 7. 目录结构

```
FOC/
  Ctrl/
    Motor/        motor_types.h  motion_planner.{h,c}  motor.{h,c}
    Driver/       driver_base.h  sin_map.{h,c}  tb67h450_base.{h,c}
    Sensor/
      Encoder/    encoder_base.h  mt6816_base.{h,c}  encoder_calibrator_base.{h,c}
    Signal/       button_base.{h,c}  led_base.{h,c}
  Port/           {tb67h450,mt6816,encoder_calibrator,button,led}_stm32.{h,c}
    Platform/
      Memory/     eeprom_interface.h  emulated_eeprom.{h,c}
                  stockpile_config.h  stockpile_f103cb.{h,c}
      Utils/      st_hardware.{h,c}
  UserApp/        app_foc.{h,c}  common_inc.h  configurations.h
    protocols/    interface_can.{h,c}
  docs/           FOC_C_PORT.md
```

> `motor_types.h` 是从上游 `Motor` 类内部提取的独立头文件（枚举 + 常量），
> 目的是让 `Signal` 层能引用 `MotorState` 而不包含整个 `motor.h`，
> 避免 `Signal` ↔ `Motor` 的头文件循环依赖。