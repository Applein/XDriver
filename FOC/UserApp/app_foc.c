/*
 * app_foc.c
 *
 * 上游: UserApp/main.cpp
 *
 * 与上游的结构对应关系:
 *   BoardConfig_t boardConfig;      -> 同名全局 (定义在此处, configurations.h 里 extern)
 *   Motor motor;                    -> 同名全局
 *   TB67H450 tb67H450;              -> TB67H450Base tb67H450 + tb67h450_stm32_setup()
 *   MT6816 mt6816;                  -> MT6816Base mt6816 + mt6816_stm32_setup()
 *   EncoderCalibrator encoderCalibrator(&motor);
 *                                   -> EncoderCalibrator + encoder_calibrator_stm32_setup()
 *   Button button1(1,1000), button2(2,3000);
 *                                   -> ButtonBase + button_init(..., 1000/3000, ...)
 *   Led statusLed;                  -> LedBase + led_init(..., led_stm32_set_state)
 *
 * 三处刻意的行为差异 (均为适配 RT-Thread, 已在注释中标明):
 *   1. HAL_Delay(100) -> rt_thread_mdelay(100)
 *   2. 100 Hz 节拍由 TIM1 中断 -> RT 线程 (TIM1 已是 HAL 时基)
 *   3. 上游 main 的 for(;;) 轮询 -> 线程里 rt_thread_mdelay(10)
 */

#include "app_foc.h"

#include <stdlib.h>     /* atoi */

#include <rtthread.h>
#include <rthw.h>       /* rt_hw_interrupt_disable / enable */

#include "main.h"
#include "rt_board.h"   /* console_err_ore / console_err_other (控制台诊断计数) */
#include "spi.h"
#include "tim.h"

#include "configurations.h"

#include "Motor/motor.h"
#include "Sensor/Encoder/encoder_calibrator_base.h"
#include "Signal/button_base.h"
#include "Signal/led_base.h"

#include "button_stm32.h"
#include "encoder_calibrator_stm32.h"
#include "led_stm32.h"
#include "mt6816_stm32.h"
#include "tb67h450_stm32.h"

#include "Platform/Memory/eeprom_interface.h"
#include "Platform/Utils/st_hardware.h"

/* ========================================================================== */
/* 全局对象 (对应上游 main.cpp 的 Component Definitions)                       */
/* ========================================================================== */

BoardConfig_t boardConfig;                  /* 上游: BoardConfig_t boardConfig; */

static Motor motor;                         /* 上游: Motor motor; */
static TB67H450Base tb67h450;               /* 上游: TB67H450 tb67H450; */
static MT6816Base mt6816;                   /* 上游: MT6816 mt6816; */
static EncoderCalibrator encoderCalibrator; /* 上游: EncoderCalibrator encoderCalibrator(&motor); */
static ButtonBase button1;                  /* 上游: Button button1(1, 1000); */
static ButtonBase button2;                  /* 上游: Button button2(2, 3000); */
static LedBase statusLed;                   /* 上游: Led statusLed; */

/* 100 Hz 家务线程 */
#define FOC_HOUSEKEEPING_PERIOD_MS      10
#define FOC_HOUSEKEEPING_STACK_SIZE     1024
#define FOC_HOUSEKEEPING_PRIORITY       12  /* 高于 main(10) 的数值 = 更低优先级 */

/*
 * bring-up 开关: 置 0 则上电不启动 20 kHz 闭环, 先用 msh 的 foc_enc 确认
 * MT6816 SPI 通路, 再执行 foc_ctrl on 手动启动。
 *
 * 之所以提供这个开关: mt6816 读角度用的是 HAL_MAX_DELAY, 一旦 SPI 不通,
 * 20 kHz 中断会永久卡在 TIM4 (优先级 0) 里, RT-Thread 的 tick 也停摆,
 * 整机表现为"串口无响应 + PA6 心跳停止", 无法在线挽救, 只能重新烧写。
 */
#ifndef FOC_AUTOSTART_20KHZ
#define FOC_AUTOSTART_20KHZ             1
#endif

static struct rt_thread foc_housekeeping_thread;
static rt_uint8_t foc_housekeeping_stack[FOC_HOUSEKEEPING_STACK_SIZE];

/* ========================================================================== */
/* 按键回调 (上游 OnButton1Event / OnButton2Event)                             */
/* ========================================================================== */

static void on_button1_event(ButtonEvent event)
{
    switch (event)
    {
    case BTN_EVT_UP:
    case BTN_EVT_DOWN:
        break;

    case BTN_EVT_LONG_PRESS:
        HAL_NVIC_SystemReset();
        break;

    case BTN_EVT_CLICK:
        if (motor.controller->mode_running != MOTOR_MODE_STOP)
        {
            boardConfig.default_mode = (uint32_t)motor.controller->mode_running;
            motor.controller->request_mode = MOTOR_MODE_STOP;
        }
        else
        {
            motor.controller->request_mode = (MotorMode)boardConfig.default_mode;
        }
        break;

    default:
        break;
    }
}

static void on_button2_event(ButtonEvent event)
{
    switch (event)
    {
    case BTN_EVT_UP:
    case BTN_EVT_DOWN:
        break;

    case BTN_EVT_LONG_PRESS:
        switch (motor.controller->mode_running)
        {
        case MOTOR_MODE_COMMAND_CURRENT:
        case MOTOR_MODE_PWM_CURRENT:
            motion_ctrl_set_current_set_point(motor.controller, 0);
            break;
        case MOTOR_MODE_COMMAND_VELOCITY:
        case MOTOR_MODE_PWM_VELOCITY:
            motion_ctrl_set_velocity_set_point(motor.controller, 0);
            break;
        case MOTOR_MODE_COMMAND_POSITION:
        case MOTOR_MODE_PWM_POSITION:
            motion_ctrl_set_position_set_point(motor.controller, 0);
            break;
        case MOTOR_MODE_COMMAND_TRAJECTORY:
        case MOTOR_MODE_STEP_DIR:
        case MOTOR_MODE_STOP:
            break;
        default:
            break;
        }
        break;

    case BTN_EVT_CLICK:
        motion_ctrl_clear_stall_flag(motor.controller);
        break;

    default:
        break;
    }
}

/* ========================================================================== */
/* 配置载入 / 保存                                                            */
/* ========================================================================== */

/* 出厂默认值 (恢复配置时复用, 避免两处写不同的数字) */
static void board_config_set_defaults(uint16_t node_id)
{
    boardConfig.config_status = CONFIG_OK;
    boardConfig.can_node_id = node_id;
    boardConfig.encoder_home_offset = 0;
    boardConfig.default_mode = (uint32_t)MOTOR_MODE_COMMAND_POSITION;
    boardConfig.current_limit = 1 * 1000;                               /* mA */
    boardConfig.velocity_limit = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS; /* r/s */
    boardConfig.velocity_acc = 100 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;  /* r/s^2 */
    boardConfig.calibration_current = 2000;

    /*
     * 位置环默认增益: 500/80/5/250
     *
     * 与上游 (200/80/300/250) 不同 —— ki=300 在 20 kHz 下会让积分
     * 0.2 秒内冲到限幅, 造成到位后持续振荡。详见 motor.c 里的注释。
     * 必须与 motor.c 的 MOTOR_CONFIG_DEFAULT 保持一致, 否则首次上电
     * (Flash 为空) 与后续启动 (读 Flash) 会得到不同手感。
     */
    boardConfig.dce_kp = 500;
    boardConfig.dce_kv = 80;
    boardConfig.dce_ki = 5;
    boardConfig.dce_kd = 250;
    boardConfig.enable_motor_on_boot = false;
    boardConfig.enable_stall_protect = false;
}

/*
 * 配置合法性检查
 *
 * 为什么必须有: 配置页位于 0x0801FC00, 掉电、Flash 写失败、或结构体布局
 * 变更 (字段增删) 都会让它读出一堆无意义的值。而 config_status 恰好可能
 * 仍是 CONFIG_OK, 于是坏配置被当成"有效"直接用下去。
 *
 * 最典型的破坏是 current_limit = 0, 后果有两重且互相掩盖:
 *   1. 输出被限幅到 0      -> 电机不出力, 完全不转
 *   2. current == limit    -> abs(foc_current)==0 与 limit==0 恒相等,
 *                            1 秒后永久报 OVERLOAD
 * 用户看到"不转 + 过载", 很容易误判成驱动或编码器坏了。
 */
static bool board_config_is_sane(void)
{
    if (boardConfig.config_status != CONFIG_OK)
    {
        return false;
    }
    if ((boardConfig.current_limit <= 0) || (boardConfig.current_limit > 5000))
    {
        return false;
    }
    if ((boardConfig.velocity_limit <= 0) ||
        (boardConfig.velocity_limit > 200 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS))
    {
        return false;
    }
    if ((boardConfig.velocity_acc <= 0) ||
        (boardConfig.velocity_acc > 2000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS))
    {
        return false;
    }
    if ((boardConfig.default_mode != (uint32_t)MOTOR_MODE_STOP) &&
        (boardConfig.default_mode != (uint32_t)MOTOR_MODE_COMMAND_POSITION) &&
        (boardConfig.default_mode != (uint32_t)MOTOR_MODE_COMMAND_VELOCITY) &&
        (boardConfig.default_mode != (uint32_t)MOTOR_MODE_COMMAND_CURRENT))
    {
        return false;
    }

    return true;
}

/* 上游 Main() 里"EEPROM 里没有有效配置时写入默认值"的那段 */
static void load_board_config(void)
{
    uint64_t serial_num;
    uint16_t default_node_id = 0;

    /* 上游: 按芯片唯一 ID 指定默认节点号 (Dummy-Robot 六个关节) */
    serial_num = GetSerialNumber();
    switch (serial_num)
    {
    case 120401936275074ULL:    /* J1 */
        default_node_id = 1;
        break;
    case 384624576568ULL:       /* J2 */
        default_node_id = 2;
        break;
    case 384290670648ULL:       /* J3 */
        default_node_id = 3;
        break;
    case 431531051064ULL:       /* J4 */
        default_node_id = 4;
        break;
    case 431466760248ULL:       /* J5 */
        default_node_id = 5;
        break;
    case 431484848184ULL:       /* J6 */
        default_node_id = 6;
        break;
    default:
        break;
    }

    /* 上游: EEPROM eeprom; eeprom.get(0, boardConfig); */
    eeprom_get_blob(0, &boardConfig, (uint16_t)sizeof(boardConfig));

    if (!board_config_is_sane())
    {
        /* 上游: boardConfig = BoardConfig_t{ ... }; eeprom.put(0, boardConfig); */
        rt_kprintf("config: invalid page content, restoring defaults\n"
                   "        (was status=%d limit=%d vel=%d acc=%d)\n",
                   (int)boardConfig.config_status, (int)boardConfig.current_limit,
                   (int)boardConfig.velocity_limit, (int)boardConfig.velocity_acc);

        board_config_set_defaults(default_node_id);

        eeprom_put_blob(0, &boardConfig, (uint16_t)sizeof(boardConfig));
    }
}

/* 上游 Main() 里把 boardConfig 抄到 motor.config 的那段 */
static void apply_board_config_to_motor(void)
{
    motor.config.motion_params.encoder_home_offset = boardConfig.encoder_home_offset;
    motor.config.motion_params.rated_current = boardConfig.current_limit;
    motor.config.motion_params.rated_velocity = boardConfig.velocity_limit;
    motor.config.motion_params.rated_velocity_acc = boardConfig.velocity_acc;

    velocity_tracker_set_velocity_acc(&motor.motion_planner.velocity_tracker,
                                      boardConfig.velocity_acc);
    position_tracker_set_velocity_acc(&motor.motion_planner.position_tracker,
                                      boardConfig.velocity_acc);

    motor.config.motion_params.cali_current = boardConfig.calibration_current;

    motor.config.ctrl_params.dce.kp = boardConfig.dce_kp;
    motor.config.ctrl_params.dce.kv = boardConfig.dce_kv;
    motor.config.ctrl_params.dce.ki = boardConfig.dce_ki;
    motor.config.ctrl_params.dce.kd = boardConfig.dce_kd;

    motor.config.ctrl_params.stall_protect_switch = boardConfig.enable_stall_protect;
}

/* ========================================================================== */
/* 100 Hz 家务线程                                                            */
/* ========================================================================== */

static void foc_housekeeping_entry(void *parameter)
{
    (void)parameter;

    for (;;)
    {
        /* 上游 Tim1Callback100Hz(): button1.Tick(10); button2.Tick(10);
         *                          statusLed.Tick(10, state); */
        button_tick(&button1, FOC_HOUSEKEEPING_PERIOD_MS);
        button_tick(&button2, FOC_HOUSEKEEPING_PERIOD_MS);
        led_tick(&statusLed, FOC_HOUSEKEEPING_PERIOD_MS, motor.controller->state);

        /* 上游 main() 的 for(;;) 里: encoderCalibrator.TickMainLoop();
         * 该函数只在 CALI_STATE_CALCULATING 时才真正干活 (会写 Flash) */
        encoder_calibrator_tick_main_loop(&encoderCalibrator);

        /* 上游 main() 的 for(;;) 里: CONFIG_COMMIT / CONFIG_RESTORE 处理 */
        if (boardConfig.config_status == CONFIG_COMMIT)
        {
            boardConfig.config_status = CONFIG_OK;
            eeprom_put_blob(0, &boardConfig, (uint16_t)sizeof(boardConfig));
        }
        else if (boardConfig.config_status == CONFIG_RESTORE)
        {
            eeprom_put_blob(0, &boardConfig, (uint16_t)sizeof(boardConfig));
            HAL_NVIC_SystemReset();
        }
        else
        {
            /* nothing */
        }

        rt_thread_mdelay(FOC_HOUSEKEEPING_PERIOD_MS);
    }
}

/* ========================================================================== */
/* 20 kHz 中断入口                                                            */
/* ========================================================================== */

/*
 * ISR 耗时统计 (DWT 周期计数器)
 *
 * 为什么需要: TIM4 是抢占优先级 0, 一旦某次 ISR 拖长, USART1 (优先级 5)
 * 就会被推迟, 115200 下超过 ~174us 就开始丢字节 —— 表现就是"串口输入没反应"。
 * 有了 max 值就能直接判断是不是这个原因, 而不是靠猜。
 *
 * DWT->CYCCNT 是 32 位递增计数器, 72MHz 下约 59.6 秒回绕一次,
 * 单次相减得到的增量在回绕处仍然正确。
 */
static volatile uint32_t foc_isr_cycles_last = 0;
static volatile uint32_t foc_isr_cycles_max = 0;
static volatile uint32_t foc_isr_count = 0;

/*
 * 超期报警阈值
 *
 * TIM4 周期 = 50us = 3600 周期。留 10% 余量取 3240 (45us):
 * 一旦某次 ISR 超过它, 说明已经侵占了本该留给主线程/串口的时间。
 *   warn_count : 超过阈值的次数 (持续增长 => 控制环太重或 SPI 在超时)
 *   warn_max   : 超期样本里的最大值
 * 这两个计数只增不减, 由 xdrv 打印; 想清零请重启 (或执行 xdrv loop on)。
 */
#define FOC_ISR_BUDGET_CYCLES       3240U       /* 45us @ 72MHz */

static volatile uint32_t foc_isr_warn_count = 0;
static volatile uint32_t foc_isr_warn_max = 0;

static void foc_isr_stats_init(void)
{
    /* 使能 DWT 周期计数器 (Cortex-M3 标准做法) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    foc_isr_cycles_last = 0;
    foc_isr_cycles_max = 0;
    foc_isr_count = 0;
    foc_isr_warn_count = 0;
    foc_isr_warn_max = 0;
}

void foc_tick_20khz(void)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t used;

    /* 上游 Tim4Callback20kHz() 第一句 */
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    if (encoderCalibrator.is_triggered)
    {
        encoder_calibrator_tick_20khz(&encoderCalibrator);
    }
    else
    {
        motor_tick_20khz(&motor);
    }

    used = DWT->CYCCNT - t0;
    foc_isr_cycles_last = used;
    if (used > foc_isr_cycles_max)
    {
        foc_isr_cycles_max = used;
    }

    /* 超期统计: 只在中断里累加计数, 不做任何打印/阻塞操作 */
    if (used > FOC_ISR_BUDGET_CYCLES)
    {
        foc_isr_warn_count++;
        if (used > foc_isr_warn_max)
        {
            foc_isr_warn_max = used;
        }
    }

    foc_isr_count++;
}

/* ========================================================================== */
/* 初始化                                                                     */
/* ========================================================================== */

void foc_app_init(void)
{
    /* ---------- 1. 配置: EEPROM > 默认值 ---------- */
    load_board_config();

    /* ---------- 2. 构建 motor (取代 Motor 构造函数) ---------- */
    motor_init(&motor);

    /* ---------- 3. 板级对象挂接 ---------- */
    tb67h450_stm32_setup(&tb67h450);
    mt6816_stm32_setup(&mt6816);
    encoder_calibrator_stm32_setup(&encoderCalibrator, &motor);

    motor_attach_driver(&motor, &tb67h450.base);
    motor_attach_encoder(&motor, &mt6816.base);

    /* ---------- 4. 把 boardConfig 应用到 motor ---------- */
    apply_board_config_to_motor();

    /* ---------- 5. 上游 Init 顺序: controller -> driver -> encoder ---------- */
    motion_ctrl_init(motor.controller);     /* 上游 motor.controller->Init();  */
    driver_init(motor.driver);              /* 上游 motor.driver->Init();      */
    encoder_init(motor.encoder);            /* 上游 motor.encoder->Init();     */

    /* ---------- 6. 按键 / 指示灯 ---------- */
    button_init(&button1, 1, 1000, button_stm32_read_pin_io);   /* 上游 button1(1,1000) */
    button_init(&button2, 2, 3000, button_stm32_read_pin_io);   /* 上游 button2(2,3000) */
    button_set_on_event_listener(&button1, on_button1_event);
    button_set_on_event_listener(&button2, on_button2_event);

    led_init(&statusLed, led_stm32_set_state);

    /* ---------- 7. 等待功率级稳定 (上游 HAL_Delay(100)) ---------- */
    rt_thread_mdelay(100);

    /* ---------- 8. 同时按下两个按键 -> 触发编码器校准 ---------- */
    if (button_is_pressed(&button1) && button_is_pressed(&button2))
    {
        encoderCalibrator.is_triggered = true;
    }

    /* ---------- 9. 启动 100 Hz 家务线程 ---------- */
    foc_isr_stats_init();

    rt_thread_init(&foc_housekeeping_thread, "foc_hk", foc_housekeeping_entry, RT_NULL,
                   foc_housekeeping_stack, sizeof(foc_housekeeping_stack),
                   FOC_HOUSEKEEPING_PRIORITY, 10);
    rt_thread_startup(&foc_housekeeping_thread);

    /* ---------- 10. 最后启动 20 kHz 闭环 (上游: HAL_TIM_Base_Start_IT(&htim4)) ---------- */
#if FOC_AUTOSTART_20KHZ
    HAL_TIM_Base_Start_IT(&htim4);
#else
    rt_kprintf("20kHz NOT started (FOC_AUTOSTART_20KHZ=0), use 'foc_enc' then 'foc_ctrl on'\n");
#endif
}

/* ========================================================================== */
/* msh 调试命令 (替代上游的 ASCII 串口协议)                                    */
/*                                                                           */
/* 单位约定 (全部整数, 避免 nano.specs 下的 %f 开销):                          */
/*   位置: 千分之一圈 (1000 = 1.000 rev)                                       */
/*   速度: 千分之一圈每秒                                                      */
/*   电流: mA                                                                 */
/* ========================================================================== */

/*
 * msh 只注册一个顶层命令, 子命令列表见文件末尾的 foc_cli_usage()。
 *
 * 想改名只需改下面这个宏 (usage 文本用字符串拼接, 会自动跟随);
 * 唯一还必须同步改的是文件末尾 MSH_CMD_EXPORT_ALIAS 里的字面量 ——
 * finsh 的宏会先对参数做字符串化, 宏不会被展开。
 */
#define FOC_CMD_NAME                "xdrv"

/* 改设定点时要与 20 kHz 中断互斥, 避免读到半个结构体 */
#define FOC_CMD_ENTER_CRITICAL()    rt_base_t level_ = rt_hw_interrupt_disable()
#define FOC_CMD_EXIT_CRITICAL()     rt_hw_interrupt_enable(level_)

/* 前置声明: foc_flash_write_allowed() 比定义处更早用到 */
static const char *foc_mode_name(MotorMode mode);

/*
 * Flash 写入前置检查
 *
 * 为什么需要: STM32F1 在擦除/编程 Flash 期间会暂停 CPU 的 Flash 取指,
 * 一次页擦除约 20~40ms。这段时间里:
 *   - 20 kHz 闭环中断被推迟, PWM 保持最后一次的比较值
 *   - 电机以旧力矩持续推 (带载时可能顶坏机构或过流)
 * 因此只在电机已停止且功率级已休眠时允许写 Flash。
 *
 * 判定条件:
 *   mode_running == MOTOR_MODE_STOP  或  soft_disable == true
 * 前者表示闭环已进入 STOP 分支 (每次 tick 调 driver_sleep),
 * 后者表示已执行 disable (同样每 tick 调 driver_sleep), 两种情况下功率级都是关断的。
 */
static bool foc_flash_write_allowed(const char *who)
{
    if ((motor.controller->mode_running == MOTOR_MODE_STOP) ||
        motor.controller->soft_disable)
    {
        return true;
    }

    rt_kprintf("refuse %s: motor is active (mode=%s)\n", who,
               foc_mode_name(motor.controller->mode_running));
    rt_kprintf("        run '" FOC_CMD_NAME " off' first\n");

    return false;
}

/*
 * 下设定点即视为"要电机出力"。
 *
 * 必须顺手清掉 disable / brake: 闭环 tick 的前两个分支分别是
 *     if (is_stalled || soft_disable || !calibrated) driver_sleep();
 *     else if (soft_brake)                           driver_brake();
 * 它们根本不看 mode_running。所以 'xdrv off' 之后再 'xdrv pos/vel/cur'
 * 会静默不动 —— 命令看起来生效了, 实际什么都没发生。
 */
static void foc_motion_prepare(void)
{
    motion_ctrl_set_disable(motor.controller, false);
    motion_ctrl_set_brake(motor.controller, false);
}

/*
 * 控制模式的显示名。
 *
 * MOTOR_MODE_STOP 必须叫 "idle" 而不是 "stop":
 *   state 里的 STOP 是"功率级已关断"(可能因为 off / brake / mode 无控制环),
 *   mode 里的 stop  是"没有控制环在跑"。两者不是一回事, 名字必須区分开,
 *   否则 status 会同时打印两行 stop, 看到的人无法判断该信哪个。
 */
static const char *foc_mode_name(MotorMode mode)
{
    switch (mode)
    {
    case MOTOR_MODE_STOP:               return "idle";
    case MOTOR_MODE_COMMAND_POSITION:   return "pos";
    case MOTOR_MODE_COMMAND_VELOCITY:   return "vel";
    case MOTOR_MODE_COMMAND_CURRENT:    return "cur";
    case MOTOR_MODE_COMMAND_TRAJECTORY: return "traj";
    case MOTOR_MODE_PWM_POSITION:       return "pwm-pos";
    case MOTOR_MODE_PWM_VELOCITY:       return "pwm-vel";
    case MOTOR_MODE_PWM_CURRENT:        return "pwm-cur";
    case MOTOR_MODE_STEP_DIR:           return "step-dir";
    default:                            return "?";
    }
}

/*
 * 功率级状态
 *
 * 判定顺序严格照抄 motor.c 里 CloseLoopControlTick() 的 Ctrl Loop 分支,
 * 这样 power 与 state 永远不会有矛盾的说法:
 *
 *     if (is_stalled || soft_disable || !calibrated)  driver_sleep();
 *     else if (soft_brake)                            driver_brake();
 *     else switch (mode_running) {
 *          case MOTOR_MODE_STOP:                      driver_sleep();
 *          ...
 *     }
 *
 * 之所以按这个顺序: 只要前面任一条件成立, 后面的分支根本不会执行,
 * 所以"真正决定功率级状态"的永远是第一个命中的条件。
 *
 * 特别是 mode_running == MOTOR_MODE_STOP 的情况 —— Button 1 单击停止就是
 * 走这条路 (只切 idle, 不置 soft_disable), 此时功率级确实每 tick 都在
 * driver_sleep(), 因此必须报 asleep; 早期版本在这里报 "active",
 * 导致 state 显示 STOP 而 power 显示 active, 看起来自相矛盾。
 */
static const char *foc_power_name(MotionController *c, bool calibrated)
{
    if (!calibrated)
    {
        return "asleep (no calib)";
    }
    if (c->is_stalled)
    {
        return "asleep (stall)";
    }
    if (c->soft_disable)
    {
        return "asleep (disabled)";
    }
    if (c->soft_brake)
    {
        return "brake (coils shorted)";
    }
    if (c->mode_running == MOTOR_MODE_STOP)
    {
        return "asleep (idle)";
    }

    return "active";
}

static const char *foc_state_name(MotorState state)
{
    switch (state)
    {
    case MOTOR_STATE_STOP:     return "STOP";
    case MOTOR_STATE_FINISH:   return "FINISH";
    case MOTOR_STATE_RUNNING:  return "RUNNING";
    case MOTOR_STATE_OVERLOAD: return "OVERLOAD";
    case MOTOR_STATE_STALL:    return "STALL";
    case MOTOR_STATE_NO_CALIB: return "NO_CALIB";
    default:                   return "?";
    }
}

static void foc_cmd_status(int argc, char **argv)
{
    MotionController *c = motor.controller;
    bool calibrated = encoder_is_calibrated(motor.encoder);
    int32_t pos_mrev;
    int32_t vel_mrps;

    (void)argc;
    (void)argv;

    /* real_position 单位是 1/51200 圈 -> 千分之一圈 */
    pos_mrev = (int32_t)(((int64_t)c->real_position * 1000) / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
    vel_mrps = (int32_t)(((int64_t)c->est_velocity * 1000) / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);

    rt_kprintf("state   : %s\n", foc_state_name(c->state));
    rt_kprintf("power   : %s\n", foc_power_name(c, calibrated));

    /* request 与 running 通常一致 (20kHz 一个周期内就会同步);
     * 只有切换瞬间才不同, 那种情况下附在后面, 避免稳态输出噪声。 */
    if (c->request_mode == c->mode_running)
    {
        rt_kprintf("mode    : %s\n", foc_mode_name(c->mode_running));
    }
    else
    {
        rt_kprintf("mode    : %s -> %s\n", foc_mode_name(c->mode_running),
                   foc_mode_name(c->request_mode));
    }

    rt_kprintf("pos     : %d (mrev)\n", pos_mrev);
    rt_kprintf("vel     : %d (mrps)\n", vel_mrps);
    rt_kprintf("current : %d (mA)\n", c->foc_current);

    /*
     * 控制器内部量 (单位: 1/51200 圈) —— 用于诊断"到位后抖动"。
     *
     * 判读: 稳定保持时 tgt==est 且 err 只有 ±1~2 的编码器量化噪声。
     *   - est 在 tgt 两侧来回跳, err 正负交替, out 跟着正负翻
     *       => 位置环形成极限环 (欠阻尼/增益不当), 需要调 gain
     *   - out 长期为 0 但电机动
     *       => 保持力矩太小 (kp 太小), 转子被齿槽转矩带着走
     *   - est 出现单次大跳变 (几十 counts) 后又回来
     *       => 编码器校准表在该 raw angle 上有断点
     * ki 是积分项 (放大 1024 倍), out 是最终电流 (mA, 已限幅)。
     */
    rt_kprintf("ctrl    : tgt=%d est=%d err=%d\n",
               (int)c->goal_position, (int)c->est_position,
               (int)c->config->dce.p_error);
    rt_kprintf("dce     : ki=%d out=%d (kp=%d kv=%d ki=%d kd=%d)\n",
               (int)(c->config->dce.output_ki >> 10), (int)c->config->dce.output,
               (int)c->config->dce.kp, (int)c->config->dce.kv,
               (int)c->config->dce.ki, (int)c->config->dce.kd);

    rt_kprintf("calib   : %s\n", calibrated ? "yes" : "NO");
    rt_kprintf("stall   : %d, overload %d\n", (int)c->is_stalled, (int)c->overload_flag);
    rt_kprintf("cfg     : node=%u limit=%dmA vel=%d acc=%d\n",
               (unsigned)boardConfig.can_node_id, (int)boardConfig.current_limit,
               (int)boardConfig.velocity_limit, (int)boardConfig.velocity_acc);

    /*
     * 实时性诊断 —— 判断"串口输入没反应"是不是 20 kHz 中断造成的:
     *   isr max 应 < 约 45us (3240 周期)。20kHz 周期才 50us,
     *   max 一旦逼近 50us, USART1 (优先级 5) 就会被长期推迟而丢字节。
     *   spi_err 非 0 且增长 => 编码器 SPI 通路有问题 (每 tick 都在超时重试)。
     */
    rt_kprintf("isr     : last=%lu max=%lu cyc (%lu.%02lu us), n=%lu\n",
               (unsigned long)foc_isr_cycles_last,
               (unsigned long)foc_isr_cycles_max,
               (unsigned long)(foc_isr_cycles_max / 72U),
               (unsigned long)((foc_isr_cycles_max % 72U) * 100U / 72U),
               (unsigned long)foc_isr_count);

    /* 超期报警: 只有真的超了才多打一行, 正常时保持输出简洁 */
    if (foc_isr_warn_count != 0U)
    {
        rt_kprintf("isr_warn: %lu over budget, peak %lu cyc (%lu us > 45us)\n",
                   (unsigned long)foc_isr_warn_count,
                   (unsigned long)foc_isr_warn_max,
                   (unsigned long)(foc_isr_warn_max / 72U));
    }
    rt_kprintf("spi_err : %lu (mt6816 timeouts)\n",
               (unsigned long)mt6816_spi_fail_count);
    if (mt6816_spi_fail_count != 0U)
    {
        /* 分类计数 + 出错瞬间的寄存器快照, 用来区分软件还是硬件 */
        rt_kprintf("spi_fail: txe=%lu rxne=%lu bsy=%lu  sr=0x%04x cr1=0x%04x\n",
                   (unsigned long)mt6816_spi_err_txe,
                   (unsigned long)mt6816_spi_err_rxne,
                   (unsigned long)mt6816_spi_err_bsy,
                   (unsigned)mt6816_spi_last_sr,
                   (unsigned)mt6816_spi_last_cr1);
    }
    rt_kprintf("uart    : ore=%lu err=%lu  (rx lost if growing)\n",
               (unsigned long)console_err_ore,
               (unsigned long)console_err_other);
}

/*
 * jit —— 量化"到位后抖动"
 *
 * 用法: xdrv jit [ms]      默认 500ms, 上限 5000ms
 *
 * 在指定时间窗内连续采样 real_position 与 est_velocity, 输出:
 *   span  : 位置峰峰值 (counts, 1 圈 = 51200)
 *   vpp   : 速度峰峰值 (steps/s)
 *   flip  : 位置变化方向翻转次数
 *
 * 判读:
 *   span <= 3 且 flip 很小        -> 稳定, 抖动只是编码器量化噪声
 *   span 几十~几百且 flip 很多     -> 极限环振荡, 调 xdrv gain
 *   span 极大且 vpp 极大           -> 有单次跳变, 怀疑校准表断点
 *
 * 注意: 本命令在采样循环里空转 (不 delay), 会把 shell 线程占满,
 *       仅用于诊断, 不要长时间运行。
 */
/*
 * jit —— 量化"到位后抖动"
 *
 * 用法: xdrv jit [ms]      默认 500ms, 上限 5000ms
 *
 * 输出三个峰峰值, 各自回答不同问题:
 *   span  : 位置峰峰值 (counts, 1 圈 = 51200)  —— 轴到底动没动
 *   vspan : 速度峰峰值 (steps/s)               —— 用窗口差分算, 不受量化噪声
 *   ispan : 电流峰峰值 (mA)                    —— 线圈是否在持续通断
 *
 * 速度算法说明 (重要):
 *   早期版本直接用 est_velocity 求峰峰值, 结果静止时也能报出 3000+ steps/s,
 *   完全是假象: 静止时 delta_position 只有 0/±1, 乘 20000 再经 1/32 低通,
 *   就是几百 steps/s 的噪声。est_velocity 在低速/静止区不可信。
 *   现在改为对 real_position 做 WINDOW_MS 窗口差分再换算成 steps/s,
 *   1 count 的量化误差被窗口长度摊薄, 静止时读数接近 0。
 *
 * 判读 (三者结合):
 *   span<=3, ispan 小           -> 真正静止
 *   span<=3, ispan 大 (几百mA)  -> 位置不动但电流来回翻 = 换相角在跳, 见
 *                                  motor.c 里 FOC_CURRENT_SIGN_HYST_MA 的说明
 *   span 大, vspan 大, flips 多 -> 极限环振荡, 调 xdrv gain
 *   span 极大, vspan 极大       -> 单次跳变, 怀疑校准表断点
 *
 * 注意: 采样循环里空转 (不 delay), 会占满 shell 线程, 仅用于诊断。
 */
/*
 * 速度窗口长度 (ms)
 *
 * 这是量化噪声与时间分辨率之间的折中:
 *   - 窗口越短, 越能反映快速变化, 但 1 count 的量化误差放大越严重
 *   - 窗口越长, 噪声越小, 但会抹掉真实的快速抖动
 *
 * 取 20ms 的理由: 1 count 跳变在 20ms 窗口下只是 50 steps/s,
 * 而在 2ms 窗口下会被放大成 500 steps/s(相差 10 倍)。实测 2ms 时
 * 静止状态会报出 3000 steps/s 的假峰峰值, 与实际位移完全不符。
 *
 * 判读参考 (静止保持时):
 *   vspan < 200 steps/s  正常 (编码器量化 + 微小真实抖动)
 *   vspan 数百以上       真实运动, 结合 span 判断是振荡还是漂移
 */
#define FOC_JIT_WINDOW_MS               20      /* 速度差分窗口 */

static void foc_cmd_jit(int argc, char **argv)
{
    int32_t duration_ms = 500;
    int32_t min_pos;
    int32_t max_pos;
    int32_t min_cur;
    int32_t max_cur;
    int32_t min_vel = 0;
    int32_t max_vel = 0;
    int32_t prev_pos;
    int32_t win_pos;                    /* 窗口起点位置 */
    rt_tick_t win_t0;                   /* 窗口起点时刻 */
    int32_t prev_dir = 0;
    uint32_t flips = 0;
    uint32_t samples = 0;
    uint32_t vel_samples = 0;
    rt_tick_t t0;

    if (argc >= 2)
    {
        duration_ms = (int32_t)atoi(argv[1]);
        if (duration_ms <= 0)
        {
            duration_ms = 500;
        }
        if (duration_ms > 5000)
        {
            duration_ms = 5000;
        }
    }

    if ((motor.controller->mode_running != MOTOR_MODE_COMMAND_POSITION) ||
        (motor.controller->state != MOTOR_STATE_FINISH))
    {
        rt_kprintf("note: motor is not holding a position (mode=%s state=%s)\n",
                   foc_mode_name(motor.controller->mode_running),
                   foc_state_name(motor.controller->state));
        rt_kprintf("      run 'xdrv pos <mrev>' and wait for FINISH first\n");
    }

    min_pos = motor.controller->real_position;
    max_pos = min_pos;
    prev_pos = min_pos;
    win_pos = min_pos;
    min_cur = motor.controller->foc_current;
    max_cur = min_cur;

    t0 = rt_tick_get();
    win_t0 = t0;
    while ((rt_tick_get() - t0) < (rt_tick_t)rt_tick_from_millisecond(duration_ms))
    {
        int32_t pos = motor.controller->real_position;
        int32_t cur = motor.controller->foc_current;
        int32_t step = pos - prev_pos;
        int32_t dir = (step > 0) ? 1 : ((step < 0) ? -1 : 0);

        if (pos < min_pos) { min_pos = pos; }
        if (pos > max_pos) { max_pos = pos; }
        if (cur < min_cur) { min_cur = cur; }
        if (cur > max_cur) { max_cur = cur; }

        /* 方向真正反转才计数 (静止的 0 不算, 避免噪声放大) */
        if ((dir != 0) && (prev_dir != 0) && (dir != prev_dir))
        {
            flips++;
        }
        if (dir != 0)
        {
            prev_dir = dir;
        }

        /*
         * 每 FOC_JIT_WINDOW_MS 采一个速度样本:
         *   v = delta_position / window_time
         * 窗口内的 1 count 量化误差被摊薄, 静止时结果接近 0。
         *
         * 注意: 这是空转循环, 迭代速度远超 20 kHz, 所以窗口必须按
         * 真实时间 (rt_tick) 判定, 不能数迭代次数。
         */
        if ((rt_tick_get() - win_t0) >= (rt_tick_t)rt_tick_from_millisecond(FOC_JIT_WINDOW_MS))
        {
            int32_t v = (int32_t)(((int64_t)(pos - win_pos) * 1000) /
                                  (int64_t)FOC_JIT_WINDOW_MS);

            if (v < min_vel) { min_vel = v; }
            if (v > max_vel) { max_vel = v; }

            win_pos = pos;
            win_t0 = rt_tick_get();
            vel_samples++;
        }

        prev_pos = pos;
        samples++;
    }

    rt_kprintf("jit     : span=%d cnt (%d.%03d mrev)\n",
               (int)(max_pos - min_pos),
               (int)((max_pos - min_pos) / 51),
               (int)(((max_pos - min_pos) % 51) * 1000 / 51));
    rt_kprintf("        : vspan=%d steps/s (over %d ms windows, n=%lu)\n",
               (int)(max_vel - min_vel), FOC_JIT_WINDOW_MS,
               (unsigned long)vel_samples);
    rt_kprintf("        : ispan=%d mA, flips=%lu, samples=%lu in %d ms\n",
               (int)(max_cur - min_cur), (unsigned long)flips,
               (unsigned long)samples, (int)duration_ms);

    /*
     * vspan 的合理性上限由 span 推导, 而不是用绝对值阈值。
     *
     * 推导: vspan 是峰峰值 (max-min)。位置若在 span 内来回, 一个窗口正向
     * 跳 span 得 +span/w, 下一个窗口反向跳 span 得 -span/w, 于是
     *     vspan ≈ 2 * span / window
     * 取 span=3, window=20ms => 2*3/0.02 = 300 steps/s。
     *
     * 所以 span=3 配 vspan=300 是【正常】的, 早期用固定阈值 200 会误报。
     * 这里把允许上限设为理论值的 1.6 倍 (容纳窗口边界错位), 只要不超过它,
     * 就说明 vspan 完全由这 span 个 count 的抖动解释, 没有额外的位移。
     */
    {
        int32_t span_cnt = max_pos - min_pos;
        int32_t vspan_pp = max_vel - min_vel;
        int32_t vlimit = ((span_cnt + 1) * 2000) / FOC_JIT_WINDOW_MS;

        if (span_cnt <= 3)
        {
            if ((max_cur - min_cur) > 60)
            {
                rt_kprintf("        : position stable BUT current churning ->\n");
                rt_kprintf("          commutation sign is flipping (raise hyst or check gain)\n");
            }
            else if (vspan_pp > vlimit)
            {
                /* 排除量化噪声后仍有大速度 => 真正的单次跳变 */
                rt_kprintf("        : position quiet but vspan %d > %d expected\n",
                           (int)vspan_pp, (int)vlimit);
                rt_kprintf("          single jumps beyond quantisation ->\n");
                rt_kprintf("          if this repeats, suspect calibration table defects\n");
            }
            else
            {
                /* 到这里说明 vspan 完全由 span 个 count 的抖动解释 */
                rt_kprintf("        : stable (span explains vspan; %d <= %d)\n",
                           (int)vspan_pp, (int)vlimit);
            }
        }
        else if (flips > (samples / 20U))
        {
            rt_kprintf("        : oscillating -> try 'xdrv gain' to retune\n");
        }
        else
        {
            rt_kprintf("        : jumperry / possible calibration table defect\n");
        }
    }
}

/*
 * gain —— 在线读写位置环 (DCE) 增益
 *
 * 用法:
 *   xdrv gain                     查看当前值
 *   xdrv gain <kp> <kv> <ki> <kd> 设置 (立即生效, 不写 Flash)
 *
 * 改完确认手感正常后, 用 xdrv save 持久化 (需先 off)。
 *
 * 各参数含义 (与上游一致, 均为定点整数):
 *   kp  位置误差 -> 电流, 输出贡献 = kp * p_error / 1024  mA
 *   kv  速度误差 -> 积分项 (抑制稳态误差)
 *   ki  位置误差 -> 积分项 (消除长期偏差, 太大会振荡)
 *   kd  速度误差 -> 阻尼项 (抑制振荡, 抖就加大它)
 */
static void foc_cmd_gain(int argc, char **argv)
{
    ControllerDce *dce = &motor.config.ctrl_params.dce;

    if (argc == 1)
    {
        rt_kprintf("gain    : kp=%d kv=%d ki=%d kd=%d\n",
                   (int)dce->kp, (int)dce->kv, (int)dce->ki, (int)dce->kd);
        rt_kprintf("          'xdrv gain <kp> <kv> <ki> <kd>' to change,\n");
        rt_kprintf("          then 'xdrv save' to persist (needs 'off' first)\n");
        return;
    }

    if (argc != 5)
    {
        rt_kprintf("usage: " FOC_CMD_NAME " gain [<kp> <kv> <ki> <kd>]\n");
        return;
    }

    FOC_CMD_ENTER_CRITICAL();
    dce->kp = (int32_t)atoi(argv[1]);
    dce->kv = (int32_t)atoi(argv[2]);
    dce->ki = (int32_t)atoi(argv[3]);
    dce->kd = (int32_t)atoi(argv[4]);

    /* 同步到配置结构, 这样 'xdrv save' 能把它写进 Flash */
    boardConfig.dce_kp = dce->kp;
    boardConfig.dce_kv = dce->kv;
    boardConfig.dce_ki = dce->ki;
    boardConfig.dce_kd = dce->kd;
    FOC_CMD_EXIT_CRITICAL();

    rt_kprintf("gain    : kp=%d kv=%d ki=%d kd=%d (run 'xdrv save' to keep)\n",
               (int)dce->kp, (int)dce->kv, (int)dce->ki, (int)dce->kd);
}

/*
 * restore —— 恢复出厂默认配置
 *
 * 为什么不能复用 CONFIG_RESTORE:
 *   那个状态的含义是"把当前 boardConfig 写回 Flash 然后复位",
 *   写回去的还是当前值, 起不到恢复默认的作用。
 *
 * 这里改为: 先把 can_node_id 留下来 (它是硬件身份, 不该丢),
 * 其余字段重写成出厂默认 → 写 Flash → 复位。
 *
 * 同样要求电机停机 (会擦写 Flash)。
 */
static void foc_cmd_restore(int argc, char **argv)
{
    uint16_t keep_node_id;

    (void)argc;
    (void)argv;

    if (!foc_flash_write_allowed("restore"))
    {
        return;
    }

    keep_node_id = (uint16_t)boardConfig.can_node_id;

    /* 启动阶段用过的同一个默认值函数, 避免两处写不同数字 */
    board_config_set_defaults(keep_node_id);

    eeprom_put_blob(0, &boardConfig, (uint16_t)sizeof(boardConfig));

    rt_kprintf("config restored to defaults (node=%u), rebooting...\n",
               (unsigned)keep_node_id);

    rt_thread_mdelay(50);           /* 让串口把上面这句发完 */
    HAL_NVIC_SystemReset();
}

static void foc_cmd_cali(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 校准要写 32KB Flash (CPU 停 20~40ms), 且会用电流矢量强行推着电机
     * 转两圈, 因此必须先用 'off' 让功率级休眠、轴松手。
     * 统一用 foc_flash_write_allowed(), 与 home/save 同一套判定,
     * 不再单独看 mode_running —— 否则 'off' 之后会被误拒。 */
    if (!foc_flash_write_allowed("cali"))
    {
        return;
    }

    encoderCalibrator.is_triggered = true;
    rt_kprintf("calibration triggered (takes a few seconds, motor will rotate)\n");
}

static void foc_cmd_pos(int argc, char **argv)
{
    int32_t mrev;

    if ((argc != 2) || (rt_strcmp(argv[1], "-h") == 0))
    {
        rt_kprintf("usage: " FOC_CMD_NAME " pos <milli-rev>   (1000 = 1.000 rev)\n");
        return;
    }
    mrev = (int32_t)atoi(argv[1]);

    FOC_CMD_ENTER_CRITICAL();
    foc_motion_prepare();
    motor.controller->request_mode = MOTOR_MODE_COMMAND_POSITION;
    motion_ctrl_set_position_set_point(
        motor.controller,
        (int32_t)(((int64_t)mrev * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) / 1000));
    FOC_CMD_EXIT_CRITICAL();
}

static void foc_cmd_vel(int argc, char **argv)
{
    int32_t mrps;

    if ((argc != 2) || (rt_strcmp(argv[1], "-h") == 0))
    {
        rt_kprintf("usage: " FOC_CMD_NAME " vel <milli-rev/s>   (1000 = 1 rev/s)\n");
        return;
    }
    mrps = (int32_t)atoi(argv[1]);

    FOC_CMD_ENTER_CRITICAL();
    foc_motion_prepare();
    /* 与上游一致: 只在"进入"速度模式时把 rated_velocity 恢复成配置值,
     * 避免每次下发设定点都覆盖掉已被收窄的限速。 */
    if (motor.controller->mode_running != MOTOR_MODE_COMMAND_VELOCITY)
    {
        motor.config.motion_params.rated_velocity = boardConfig.velocity_limit;
    }
    motor.controller->request_mode = MOTOR_MODE_COMMAND_VELOCITY;
    motion_ctrl_set_velocity_set_point(
        motor.controller,
        (int32_t)(((int64_t)mrps * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) / 1000));
    FOC_CMD_EXIT_CRITICAL();
}

static void foc_cmd_cur(int argc, char **argv)
{
    int32_t ma;

    if ((argc != 2) || (rt_strcmp(argv[1], "-h") == 0))
    {
        rt_kprintf("usage: " FOC_CMD_NAME " cur <mA>\n");
        return;
    }
    ma = (int32_t)atoi(argv[1]);

    FOC_CMD_ENTER_CRITICAL();
    foc_motion_prepare();
    motor.controller->request_mode = MOTOR_MODE_COMMAND_CURRENT;
    motion_ctrl_set_current_set_point(motor.controller, ma);
    FOC_CMD_EXIT_CRITICAL();
}

static void foc_cmd_enable(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 恢复出力。mode_running / request_mode 不被 'off' 修改, 所以这里会
     * 回到停机前的模式与设定点; 只有开机后一直是 STOP 的情况才用
     * boardConfig.default_mode (与上游按键逻辑一致)。 */
    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_set_disable(motor.controller, false);
    motion_ctrl_set_brake(motor.controller, false);
    if (motor.controller->request_mode == MOTOR_MODE_STOP)
    {
        motor.controller->request_mode = (MotorMode)boardConfig.default_mode;
    }
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("motor enabled (mode=%s)\n",
               foc_mode_name(motor.controller->request_mode));
}

/*
 * off —— 失能 / 释放轴。
 *
 * 回周行为的唯一区别是"叫法": 上游区分了 MODE_STOP (模式) 和 SetDisable()
 * (失能), 但两者对功率级的作用完全一样 —— 每 tick 都调 driver_sleep()。
 * 所以这里只保留一个 'off', 避免用户纠结该用哪个。
 *
 * 注意: mode_running 不在这里改。一是为了让 'on' 能恢复到停机前的模式,
 * 二是状态机已把 soft_disable 归为 MOTOR_STATE_STOP, 所以 'xdrv' 会正确
 * 显示 STOP 而不是 RUNNING。
 */
static void foc_cmd_disable(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_set_disable(motor.controller, true);
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("motor disabled, shaft released\n");
}

static void foc_cmd_brake(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_set_brake(motor.controller, true);
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("motor braked\n");
}

static void foc_cmd_home(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 会立即触发 Flash 写入 (CONFIG_COMMIT -> 100Hz 线程), 必须停机 */
    if (!foc_flash_write_allowed("home"))
    {
        return;
    }

    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_apply_pos_as_home_offset(motor.controller);
    boardConfig.encoder_home_offset =
        motor.config.motion_params.encoder_home_offset % MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    boardConfig.config_status = CONFIG_COMMIT;
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("home offset applied & queued for save\n");
}

static void foc_cmd_save(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 会立即触发 Flash 写入 (CONFIG_COMMIT -> 100Hz 线程), 必须停机 */
    if (!foc_flash_write_allowed("save"))
    {
        return;
    }

    boardConfig.config_status = CONFIG_COMMIT;
    rt_kprintf("config queued for save\n");
}

/* ========================================================================== */
/* 上板调试命令 (仅用于验证, 不属于上游功能)                                   */
/* ========================================================================== */

/*
 * foc_enc —— MT6816 通信自检
 *
 * 为什么需要它: mt6816_stm32_spi_transmit_and_read16() 用 HAL_MAX_DELAY 在
 * TIM4 (优先级 0) 中断里等 SPI。若编码器缺失/接线错误, HAL 的 tick 靠 TIM1
 * (优先级 15) 更新, 抢不到 CPU -> 超时永远不触发 -> 整机死锁 (PA6 心跳停摆)。
 * 本命令用短超时探测, 可在启动 20 kHz 之前先确认 SPI 通路。
 */
static void foc_cmd_enc(int argc, char **argv)
{
    uint16_t tx = (uint16_t)((0x80 | 0x03) << 8);
    uint16_t rx = 0;
    HAL_StatusTypeDef status;

    (void)argc;
    (void)argv;

    rt_kprintf("SPI1 CR1=0x%04x SR=0x%04x  CS(PA15)=%d\n",
               (unsigned)SPI1->CR1, (unsigned)SPI1->SR,
               (GPIOA->IDR & SPI1_CS_Pin) ? 1 : 0);

    GPIOA->BRR = (uint32_t)SPI1_CS_Pin;
    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, 10);
    GPIOA->BSRR = (uint32_t)SPI1_CS_Pin;

    rt_kprintf("probe  : %s, rx=0x%04x, SR=0x%04x\n",
               (status == HAL_OK) ? "OK" :
               (status == HAL_TIMEOUT) ? "TIMEOUT" : "ERROR",
               (unsigned)rx, (unsigned)SPI1->SR);

    if (status == HAL_OK)
    {
        rt_kprintf("hint   : SPI 通路正常, 可以 '" FOC_CMD_NAME " loop on' 启动 20kHz\n");
    }
    else
    {
        rt_kprintf("hint   : 检查 MT6816 供电 / PB3-PB5 接线 / PA15 片选, 不要启动 20kHz\n");
    }
}

/*
 * loop —— 手动启停 20 kHz 闭环 (子命令 "loop")
 *   xdrv loop off : 上电后先关掉闭环, 方便量波形/查寄存器
 *   xdrv loop on  : 确认 SPI 正常后再启动
 * 注意: 系统上电默认已启动, 本命令用于 bring-up 阶段临时关闭。
 */
static void foc_cmd_loop(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("usage: " FOC_CMD_NAME " loop on|off   (running=%d)\n",
                   (TIM4->CR1 & TIM_CR1_CEN) ? 1 : 0);
        return;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        HAL_TIM_Base_Start_IT(&htim4);
        rt_kprintf("20kHz control loop: ON\n");
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        HAL_TIM_Base_Stop_IT(&htim4);
        driver_sleep(motor.driver);     /* 立刻关断功率级 */
        rt_kprintf("20kHz control loop: OFF (motor is asleep)\n");
    }
    else
    {
        rt_kprintf("usage: " FOC_CMD_NAME " loop on|off\n");
    }
}

/* ========================================================================== */
/* 顶层命令分发                                                               */
/*                                                                           */
/* msh 里只注册一个命令名 (FOC_CMD_NAME), 以减少 help 输出的噪声。             */
/* 子命令一律按 "<cmd> <sub> [arg]" 的形式调用。                              */
/* ========================================================================== */

/*
 * 精简用法说明
 *
 * 只保留一份总览, 不为每个子命令单独存说明文本 —— 13 份详细帮助要吃掉
 * 约 5 KB Flash, 占 95 KB 代码区的 5%, 不划算。各子命令的具体语义写在
 * 对应 foc_cmd_*() 函数的注释里, 需要时看源码。
 *
 * 逐行 rt_kprintf 而不是一个长字符串: rt_kprintf 的格式化缓冲是
 * RT_CONSOLEBUF_SIZE (128 字节), 单次输出过长会被静默截断。
 */
static void foc_cli_usage(void)
{
    rt_kprintf("usage: " FOC_CMD_NAME " [cmd] [arg]      (no arg = status)\n");
    rt_kprintf("\n");
    rt_kprintf("  pos <milli-rev>          absolute position, 1000 = 1 rev\n");
    rt_kprintf("  vel <milli-rev/s>        speed, 1000 = 1 rev/s\n");
    rt_kprintf("  cur <mA>                 current (torque)\n");
    rt_kprintf("        ^ switch mode and energize the motor\n");
    rt_kprintf("  on                       energize, resume the previous mode\n");
    rt_kprintf("  off                      release the shaft (no torque)\n");
    rt_kprintf("  brake                    short the coils (stiff, no current)\n");
    rt_kprintf("  cali | home | save       calibrate / zero / flash [need 'off']\n");
    rt_kprintf("  restore                  factory defaults + reboot [need 'off']\n");
    rt_kprintf("  jit [ms]                 measure holding jitter (default 500ms)\n");
    rt_kprintf("  gain [kp kv ki kd]       show / retune position loop gains\n");
    rt_kprintf("  enc  | loop <on|off>     SPI probe / 20kHz control loop\n");
    rt_kprintf("\n");
    rt_kprintf("  no calibration -> no torque; run 'cali' with a free shaft\n");
}

static void foc_cmd_xdrv(int argc, char **argv)
{
    const char *sub;

    /* 只敲命令名 -> 直接打印状态 (最常用的操作) */
    if (argc < 2)
    {
        foc_cmd_status(1, argv);
        return;
    }

    sub = argv[1];

    /* 参数平移一位: 让子命令函数仍以 argv[0]=子命令名, argv[1]=参数 的约定工作,
     * 这样各子命令的实现与原先的平级命令完全一致。 */
    argc--;
    argv++;

    if (rt_strcmp(sub, "status") == 0)
    {
        foc_cmd_status(argc, argv);
    }
    else if (rt_strcmp(sub, "cali") == 0)
    {
        foc_cmd_cali(argc, argv);
    }
    else if (rt_strcmp(sub, "jit") == 0)
    {
        foc_cmd_jit(argc, argv);
    }
    else if (rt_strcmp(sub, "gain") == 0)
    {
        foc_cmd_gain(argc, argv);
    }
    else if (rt_strcmp(sub, "pos") == 0)
    {
        foc_cmd_pos(argc, argv);
    }
    else if (rt_strcmp(sub, "vel") == 0)
    {
        foc_cmd_vel(argc, argv);
    }
    else if (rt_strcmp(sub, "cur") == 0)
    {
        foc_cmd_cur(argc, argv);
    }
    else if (rt_strcmp(sub, "on") == 0)
    {
        foc_cmd_enable(argc, argv);
    }
    else if (rt_strcmp(sub, "off") == 0)
    {
        foc_cmd_disable(argc, argv);
    }
    else if (rt_strcmp(sub, "brake") == 0)
    {
        foc_cmd_brake(argc, argv);
    }
    else if (rt_strcmp(sub, "home") == 0)
    {
        foc_cmd_home(argc, argv);
    }
    else if (rt_strcmp(sub, "save") == 0)
    {
        foc_cmd_save(argc, argv);
    }
    else if (rt_strcmp(sub, "restore") == 0)
    {
        foc_cmd_restore(argc, argv);
    }
    else if (rt_strcmp(sub, "enc") == 0)
    {
        foc_cmd_enc(argc, argv);
    }
    else if (rt_strcmp(sub, "loop") == 0)
    {
        foc_cmd_loop(argc, argv);
    }
    else if ((rt_strcmp(sub, "help") == 0) || (rt_strcmp(sub, "?") == 0))
    {
        foc_cli_usage();
    }
    else
    {
        rt_kprintf("unknown cmd '%s'\n", sub);
        foc_cli_usage();
    }
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_xdrv, xdrv, XDriver motor control (type 'xdrv help'));
