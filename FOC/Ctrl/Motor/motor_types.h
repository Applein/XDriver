/*
 * motor_types.h
 *
 * 上游 Dummy-Robot Ctrl-Step-Driver 里定义在 Motor 类内部的
 *    - 成员常量 (MOTOR_ONE_CIRCLE_HARD_STEPS / SOFT_DIVIDE_NUM / ...)
 *    - Motor::Mode_t
 *    - Motor::State_t
 *
 * C 化后提取到独立头文件, 目的是让 Signal 层(led_base) 能引用 MotorState
 * 而不必包含整个 motor.h, 避免 Signal <-> Motor 的头文件循环依赖。
 *
 * 注意: 这是唯一的类型定义处, 其它任何文件都不要再重复定义。
 */

#ifndef CTRL_STEP_FW_MOTOR_TYPES_H
#define CTRL_STEP_FW_MOTOR_TYPES_H

/* -------------------------------------------------------------------------- */
/* 上游 Motor 的成员常量 -> 编译期 enum                                       */
/* -------------------------------------------------------------------------- */
enum
{
    MOTOR_ONE_CIRCLE_HARD_STEPS = 200,                                      /* 1.8° 步进电机 */
    SOFT_DIVIDE_NUM = 256,
    MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM,  /* 51200 */
};

/* -------------------------------------------------------------------------- */
/* 上游 Motor::Mode_t                                                          */
/*   原枚举值 MODE_COMMAND_Trajectory 大小写不规范 -> MOTOR_MODE_COMMAND_TRAJECTORY */
/* -------------------------------------------------------------------------- */
typedef enum
{
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_COMMAND_POSITION,
    MOTOR_MODE_COMMAND_VELOCITY,
    MOTOR_MODE_COMMAND_CURRENT,
    MOTOR_MODE_COMMAND_TRAJECTORY,
    MOTOR_MODE_PWM_POSITION,
    MOTOR_MODE_PWM_VELOCITY,
    MOTOR_MODE_PWM_CURRENT,
    MOTOR_MODE_STEP_DIR,
} MotorMode;

/* -------------------------------------------------------------------------- */
/* 上游 Motor::State_t                                                         */
/* -------------------------------------------------------------------------- */
typedef enum
{
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_FINISH,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_OVERLOAD,
    MOTOR_STATE_STALL,
    MOTOR_STATE_NO_CALIB,
} MotorState;

#endif /* CTRL_STEP_FW_MOTOR_TYPES_H */
