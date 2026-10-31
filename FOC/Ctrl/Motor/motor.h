/*
 * motor.h
 *
 * 上游: Ctrl/Motor/motor.h  (class Motor + 嵌套 class Controller)
 *
 * C 化设计 (模式 3: 纯组合 + self 参数):
 *   1. 嵌套类 Motor::Controller 提升为顶层 struct MotionController。
 *   2. Motor 与 MotionController 互相引用 (Controller 需要 Motor*,
 *      Motor 内嵌 Controller), 用前置声明 + 指针打破循环:
 *          typedef struct Motor Motor;
 *          struct MotionController { Motor *context; ... };
 *          struct Motor { ... MotionController controller_instance; ... };
 *   3. Motor 保留 controller 指针 (= &controller_instance), 使上游
 *      `motor.controller->state` 这类访问在 C 里写作 `motor->controller->state`,
 *      字段名完全不变, 便于与上游逐行对照。
 *   4. 上游 Controller::CloseLoopControlTick() 里的函数内 static bool isFirstCalled
 *      提升为 MotionController.is_first_called 字段。
 *   5. 上游 Motor 的成员常量与 Mode_t/State_t 已移到 Motor/motor_types.h
 *      (Signal 层的 led_base 需要 MotorState, 但不应包含整个 motor.h)。
 *
 *   注意: 上游 `motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` 这种"通过对象访问常量"
 *   的写法在 C 里改成直接使用 MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS。
 */

#ifndef CTRL_STEP_FW_MOTOR_H
#define CTRL_STEP_FW_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "Driver/driver_base.h"
#include "Motor/motion_planner.h"
#include "Motor/motor_types.h"
#include "Sensor/Encoder/encoder_base.h"

typedef struct Motor Motor;

/* -------------------------------------------------------------------------- */
/* 上游: Motor::Controller::PID_t                                             */
/* -------------------------------------------------------------------------- */
typedef struct
{
    bool kp_valid;
    bool ki_valid;
    bool kd_valid;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t v_error;
    int32_t v_error_last;
    int32_t output_kp;
    int32_t output_ki;
    int32_t output_kd;
    int32_t integral_round;
    int32_t integral_remainder;
    int32_t output;
} ControllerPid;

/* -------------------------------------------------------------------------- */
/* 上游: Motor::Controller::DCE_t                                             */
/* -------------------------------------------------------------------------- */
typedef struct
{
    int32_t kp;
    int32_t kv;
    int32_t ki;
    int32_t kd;
    int32_t p_error;
    int32_t v_error;
    int32_t output_kp;
    int32_t output_ki;
    int32_t output_kd;
    int32_t integral_round;
    int32_t integral_remainder;
    int32_t output;
} ControllerDce;

/* -------------------------------------------------------------------------- */
/* 上游: Motor::Controller::Config_t                                          */
/* -------------------------------------------------------------------------- */
typedef struct
{
    ControllerPid pid;
    ControllerDce dce;

    bool stall_protect_switch;
} ControllerConfig;

/* -------------------------------------------------------------------------- */
/* 上游: class Motor::Controller                                              */
/* -------------------------------------------------------------------------- */
typedef struct
{
    Motor *context;                 /* 上游: Motor* context (private) */

    ControllerConfig *config;       /* 上游: Config_t* config = nullptr */

    /* ---- 上游 public 成员 ---- */
    MotorMode request_mode;
    MotorMode mode_running;
    MotorState state;
    bool is_stalled;

    /* ---- 上游 private 成员 ---- */
    int32_t real_lap_position;
    int32_t real_lap_position_last;
    int32_t real_position;
    int32_t real_position_last;
    int32_t est_velocity;
    int32_t est_velocity_integral;
    int32_t est_lead_position;
    int32_t est_position;
    int32_t est_error;
    int32_t foc_current;
    int32_t goal_position;
    int32_t goal_velocity;
    int32_t goal_current;
    bool goal_disable;
    bool goal_brake;
    int32_t soft_position;
    int32_t soft_velocity;
    int32_t soft_current;
    bool soft_disable;
    bool soft_brake;
    bool soft_new_curve;
    int32_t foc_position;
    int32_t foc_sign;       /* 换相角符号的滞回状态: -1 / 0 / +1, 见 calc_current_to_output */
    uint32_t stalled_time;
    uint32_t overload_time;
    bool overload_flag;

    /* ---- 取代上游 CloseLoopControlTick() 内 static bool isFirstCalled ---- */
    bool is_first_called;
} MotionController;

/* 上游: Controller 的公开方法 (friend Motor 的私有方法放在 motor.c 内部声明) */
void motion_ctrl_init(MotionController *self);
void motion_ctrl_set_ctrl_mode(MotionController *self, MotorMode mode);
void motion_ctrl_set_current_set_point(MotionController *self, int32_t current);
void motion_ctrl_set_velocity_set_point(MotionController *self, int32_t velocity);
void motion_ctrl_set_position_set_point(MotionController *self, int32_t position);
bool motion_ctrl_set_position_set_point_with_time(MotionController *self, int32_t position,
                                                  float time_s);
float motion_ctrl_get_position(MotionController *self, bool is_lap);
float motion_ctrl_get_velocity(MotionController *self);
float motion_ctrl_get_foc_current(MotionController *self);
void motion_ctrl_add_trajectory_set_point(MotionController *self, int32_t position,
                                         int32_t velocity);
void motion_ctrl_set_disable(MotionController *self, bool disable);
void motion_ctrl_set_brake(MotionController *self, bool brake);
void motion_ctrl_apply_pos_as_home_offset(MotionController *self);
void motion_ctrl_clear_stall_flag(MotionController *self);

/* -------------------------------------------------------------------------- */
/* 上游: struct Motor::Config_t                                               */
/* -------------------------------------------------------------------------- */
typedef struct
{
    MotionPlannerConfig motion_params;
    ControllerConfig ctrl_params;
} MotorConfig;

/* -------------------------------------------------------------------------- */
/* 上游: class Motor                                                          */
/* -------------------------------------------------------------------------- */
struct Motor
{
    MotorConfig config;

    MotionPlanner motion_planner;

    MotionController controller_instance;   /* 上游 private: Controller controllerInstance */
    MotionController *controller;           /* = &controller_instance */

    EncoderBase *encoder;
    DriverBase *driver;
};

/*
 * 上游: Motor() 构造函数
 *   写默认 MotionPlannerConfig / ControllerConfig, 并挂接两个 config 指针
 *   (motionPlanner.AttachConfig / controller->AttachConfig)。
 *   只做内存初始化与指针挂接, 不访问外设、不依赖 boardConfig。
 */
void motor_init(Motor *self);

/* 上游: void Motor::AttachEncoder(EncoderBase*) */
void motor_attach_encoder(Motor *self, EncoderBase *encoder);

/* 上游: void Motor::AttachDriver(DriverBase*) */
void motor_attach_driver(Motor *self, DriverBase *driver);

/* 上游: void Motor::Tick20kHz()   —— 20 kHz 中断内调用 */
void motor_tick_20khz(Motor *self);

#endif /* CTRL_STEP_FW_MOTOR_H */
