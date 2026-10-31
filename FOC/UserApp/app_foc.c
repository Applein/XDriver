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

    if (boardConfig.config_status != CONFIG_OK)
    {
        /* 上游: boardConfig = BoardConfig_t{ ... }; eeprom.put(0, boardConfig); */
        boardConfig.config_status = CONFIG_OK;
        boardConfig.can_node_id = default_node_id;
        boardConfig.encoder_home_offset = 0;
        boardConfig.default_mode = (uint32_t)MOTOR_MODE_COMMAND_POSITION;
        boardConfig.current_limit = 1 * 1000;                                   /* mA */
        boardConfig.velocity_limit = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;     /* r/s */
        boardConfig.velocity_acc = 100 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;      /* r/s^2 */
        boardConfig.calibration_current = 2000;
        boardConfig.dce_kp = 200;
        boardConfig.dce_kv = 80;
        boardConfig.dce_ki = 300;
        boardConfig.dce_kd = 250;
        boardConfig.enable_motor_on_boot = false;
        boardConfig.enable_stall_protect = false;

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

void foc_tick_20khz(void)
{
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
    rt_kprintf("        run 'foc_mode stop' or 'foc_disable' first, then retry\n");

    return false;
}

static const char *foc_mode_name(MotorMode mode)
{
    switch (mode)
    {
    case MOTOR_MODE_STOP:               return "stop";
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
    int32_t pos_mrev;
    int32_t vel_mrps;

    (void)argc;
    (void)argv;

    /* real_position 单位是 1/51200 圈 -> 千分之一圈 */
    pos_mrev = (int32_t)(((int64_t)c->real_position * 1000) / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
    vel_mrps = (int32_t)(((int64_t)c->est_velocity * 1000) / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);

    rt_kprintf("state   : %s\n", foc_state_name(c->state));
    rt_kprintf("mode    : %s (request %s)\n", foc_mode_name(c->mode_running),
               foc_mode_name(c->request_mode));
    rt_kprintf("pos     : %d (mrev)\n", pos_mrev);
    rt_kprintf("vel     : %d (mrev/s)\n", vel_mrps);
    rt_kprintf("current : %d (mA)\n", c->foc_current);
    rt_kprintf("calib   : %s\n", encoder_is_calibrated(motor.encoder) ? "yes" : "NO");
    rt_kprintf("stall   : %d, overload %d\n", (int)c->is_stalled, (int)c->overload_flag);
    rt_kprintf("cfg     : node=%u limit=%dmA vel=%d acc=%d\n",
               (unsigned)boardConfig.can_node_id, (int)boardConfig.current_limit,
               (int)boardConfig.velocity_limit, (int)boardConfig.velocity_acc);
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_status, foc_status, show FOC status);

static void foc_cmd_cali(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 校准过程中电机会被强制推着转两圈, 必须确认当前不在闭环运行态,
     * 否则会在上一个模式的控制输出上叠加校准电流矢量。
     * 同时也避免 CALI_CALCULATING 阶段写 32KB Flash 时电机还在出力。 */
    if (motor.controller->mode_running != MOTOR_MODE_STOP)
    {
        rt_kprintf("refuse foc_cali: motor is active (mode=%s)\n",
                   foc_mode_name(motor.controller->mode_running));
        rt_kprintf("        run 'foc_mode stop' first, then retry\n");
        return;
    }

    encoderCalibrator.is_triggered = true;
    rt_kprintf("calibration triggered (takes a few seconds, motor will rotate)\n");
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_cali, foc_cali, start encoder calibration);

static void foc_cmd_mode(int argc, char **argv)
{
    MotorMode mode;

    if (argc != 2)
    {
        rt_kprintf("usage: foc_mode stop|pos|vel|cur\n");
        return;
    }

    if (rt_strcmp(argv[1], "stop") == 0)
    {
        mode = MOTOR_MODE_STOP;
    }
    else if (rt_strcmp(argv[1], "pos") == 0)
    {
        mode = MOTOR_MODE_COMMAND_POSITION;
    }
    else if (rt_strcmp(argv[1], "vel") == 0)
    {
        mode = MOTOR_MODE_COMMAND_VELOCITY;
    }
    else if (rt_strcmp(argv[1], "cur") == 0)
    {
        mode = MOTOR_MODE_COMMAND_CURRENT;
    }
    else
    {
        rt_kprintf("usage: foc_mode stop|pos|vel|cur\n");
        return;
    }

    FOC_CMD_ENTER_CRITICAL();
    motor.controller->request_mode = mode;
    FOC_CMD_EXIT_CRITICAL();
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_mode, foc_mode, set control mode);

static void foc_cmd_pos(int argc, char **argv)
{
    int32_t mrev;

    if ((argc != 2) || (rt_strcmp(argv[1], "-h") == 0))
    {
        rt_kprintf("usage: foc_pos <milli-rev>   (1000 = 1.000 rev)\n");
        return;
    }
    mrev = (int32_t)atoi(argv[1]);

    FOC_CMD_ENTER_CRITICAL();
    if (motor.controller->mode_running != MOTOR_MODE_COMMAND_POSITION)
    {
        motor.controller->request_mode = MOTOR_MODE_COMMAND_POSITION;
    }
    motion_ctrl_set_position_set_point(
        motor.controller,
        (int32_t)(((int64_t)mrev * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) / 1000));
    FOC_CMD_EXIT_CRITICAL();
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_pos, foc_pos, set position (milli-rev));

static void foc_cmd_vel(int argc, char **argv)
{
    int32_t mrps;

    if ((argc != 2) || (rt_strcmp(argv[1], "-h") == 0))
    {
        rt_kprintf("usage: foc_vel <milli-rev/s>   (1000 = 1 rev/s)\n");
        return;
    }
    mrps = (int32_t)atoi(argv[1]);

    FOC_CMD_ENTER_CRITICAL();
    if (motor.controller->mode_running != MOTOR_MODE_COMMAND_VELOCITY)
    {
        motor.config.motion_params.rated_velocity = boardConfig.velocity_limit;
        motor.controller->request_mode = MOTOR_MODE_COMMAND_VELOCITY;
    }
    motion_ctrl_set_velocity_set_point(
        motor.controller,
        (int32_t)(((int64_t)mrps * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) / 1000));
    FOC_CMD_EXIT_CRITICAL();
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_vel, foc_vel, set velocity (milli-rev/s));

static void foc_cmd_cur(int argc, char **argv)
{
    int32_t ma;

    if ((argc != 2) || (rt_strcmp(argv[1], "-h") == 0))
    {
        rt_kprintf("usage: foc_cur <mA>\n");
        return;
    }
    ma = (int32_t)atoi(argv[1]);

    FOC_CMD_ENTER_CRITICAL();
    if (motor.controller->mode_running != MOTOR_MODE_COMMAND_CURRENT)
    {
        motor.controller->request_mode = MOTOR_MODE_COMMAND_CURRENT;
    }
    motion_ctrl_set_current_set_point(motor.controller, ma);
    FOC_CMD_EXIT_CRITICAL();
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_cur, foc_cur, set current (mA));

static void foc_cmd_enable(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_set_disable(motor.controller, false);
    motion_ctrl_set_brake(motor.controller, false);
    if (motor.controller->request_mode == MOTOR_MODE_STOP)
    {
        motor.controller->request_mode = (MotorMode)boardConfig.default_mode;
    }
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("motor enabled\n");
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_enable, foc_enable, enable motor);

static void foc_cmd_disable(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_set_disable(motor.controller, true);
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("motor disabled\n");
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_disable, foc_disable, disable motor);

static void foc_cmd_brake(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    FOC_CMD_ENTER_CRITICAL();
    motion_ctrl_set_brake(motor.controller, true);
    FOC_CMD_EXIT_CRITICAL();
    rt_kprintf("motor braked\n");
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_brake, foc_brake, brake motor);

static void foc_cmd_home(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 会立即触发 Flash 写入 (CONFIG_COMMIT -> 100Hz 线程), 必须停机 */
    if (!foc_flash_write_allowed("foc_home"))
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
MSH_CMD_EXPORT_ALIAS(foc_cmd_home, foc_home, apply current position as home);

static void foc_cmd_save(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 会立即触发 Flash 写入 (CONFIG_COMMIT -> 100Hz 线程), 必须停机 */
    if (!foc_flash_write_allowed("foc_save"))
    {
        return;
    }

    boardConfig.config_status = CONFIG_COMMIT;
    rt_kprintf("config queued for save\n");
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_save, foc_save, save config to flash);

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
        rt_kprintf("hint   : SPI 通路正常, 可以 foc_ctrl on 启动 20kHz\n");
    }
    else
    {
        rt_kprintf("hint   : 检查 MT6816 供电 / PB3-PB5 接线 / PA15 片选, 不要启动 20kHz\n");
    }
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_enc, foc_enc, probe MT6816 SPI link);

/*
 * foc_ctrl —— 手动启停 20 kHz 闭环
 *   foc_ctrl off : 上电后先关掉闭环, 方便量波形/查寄存器
 *   foc_ctrl on  : 确认 SPI 正常后再启动
 * 注意: 系统上电默认已启动, 本命令用于 bring-up 阶段临时关闭。
 */
static void foc_cmd_ctrl(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("usage: foc_ctrl on|off   (running=%d)\n",
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
        rt_kprintf("usage: foc_ctrl on|off\n");
    }
}
MSH_CMD_EXPORT_ALIAS(foc_cmd_ctrl, foc_ctrl, start/stop 20kHz control loop);
