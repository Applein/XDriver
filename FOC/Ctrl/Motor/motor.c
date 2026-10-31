/*
 * motor.c
 *
 * 上游: Ctrl/Motor/motor.cpp
 *
 * 逐段移植。要点:
 *   1. Motor::Controller 的私有方法 (CalcCurrentToOutput / CalcPidToOutput /
 *      CalcDceToOutput / ClearIntegral / CompensateAdvancedAngle / AttachConfig)
 *      都收在本文件内; 前 4 个为 static, CompensateAdvancedAngle 内联进调用处
 *      但保持独立 static 函数以便逐行对照。
 *   2. CloseLoopControlTick() 的 static bool isFirstCalled -> is_first_called 字段。
 *   3. 所有定点运算 (>>5 / >>7 / >>10、integralRound 余数回填) 的运算顺序
 *      与上游完全一致, 不可重排。
 *   4. 本文件会在 20 kHz 中断里执行, 不使用 RT-Thread API / printf / 动态内存。
 */

#include "Motor/motor.h"

#include <math.h>       /* sqrtf */
#include <stddef.h>     /* NULL */
#include <stdlib.h>     /* abs */
#include <string.h>     /* memset */

#include "configurations.h"     /* boardConfig (motion_ctrl_init 读取) */

/*
 * 换相角符号的滞回阈值 (mA)
 *
 * 只有 |电流| 超过它才允许改变超前/滞后方向, 零附近沿用上一次方向。
 * 目的见 motion_ctrl_calc_current_to_output() 的注释。
 *
 * 取值考虑:
 *   - 太小 (如 2mA): 仍会被量化噪声穿透, 起不到作用
 *   - 太大 (如 200mA): 真实的微小指令 (cur 10) 会被误判为"不出力"
 * 15mA 约为额定 1000mA 的 1.5%, 远高于噪声, 又低于任何有意义的指令。
 */
enum
{
    FOC_CURRENT_SIGN_HYST_MA = 15,
};

/* ========================================================================== */
/* Controller: 私有方法                                                        */
/* ========================================================================== */

/* 上游: static int32_t Motor::Controller::CompensateAdvancedAngle(int32_t _vel)
 *
 * 说明: 这段补偿曲线是为 DPS 系列传感器实测的, TLE5012/MT6816 需要重新测量后
 *       再调整。数值保持与上游完全一致。 */
static int32_t compensate_advanced_angle(int32_t velocity)
{
    int32_t compensate;

    if (velocity < 0)
    {
        if (velocity > -100000)
        {
            compensate = 0;
        }
        else if (velocity > -1300000)
        {
            compensate = (((velocity + 100000) * 262) >> 20) - 0;
        }
        else if (velocity > -2200000)
        {
            compensate = (((velocity + 1300000) * 105) >> 20) - 300;
        }
        else
        {
            compensate = (((velocity + 2200000) * 52) >> 20) - 390;
        }

        if (compensate < -430)
        {
            compensate = -430;
        }
    }
    else
    {
        if (velocity < 100000)
        {
            compensate = 0;
        }
        else if (velocity < 1300000)
        {
            compensate = (((velocity - 100000) * 262) >> 20) + 0;
        }
        else if (velocity < 2200000)
        {
            compensate = (((velocity - 1300000) * 105) >> 20) + 300;
        }
        else
        {
            compensate = (((velocity - 2200000) * 52) >> 20) + 390;
        }

        if (compensate > 430)
        {
            compensate = 430;
        }
    }

    return compensate;
}

/* 上游: void Motor::Controller::ClearIntegral() const */
static void motion_ctrl_clear_integral(MotionController *self)
{
    self->config->pid.integral_round = 0;
    self->config->pid.integral_remainder = 0;
    self->config->pid.output_ki = 0;

    self->config->dce.integral_round = 0;
    self->config->dce.integral_remainder = 0;
    self->config->dce.output_ki = 0;
}

/* 上游: void Motor::Controller::AttachConfig(Config_t*)  (private, friend Motor) */
static void motion_ctrl_attach_config(MotionController *self, ControllerConfig *config)
{
    self->config = config;
}

/* 上游: void Motor::Controller::CalcCurrentToOutput(int32_t current) */
static void motion_ctrl_calc_current_to_output(MotionController *self, int32_t current)
{
    /*
     * 换相角符号的滞回 (本工程在上游基础上的唯一控制逻辑改动)
     *
     * 上游直接按 foc_current 的正负决定超前/滞后 90°:
     *     current > 0 -> est_position + SOFT_DIVIDE_NUM
     *     current < 0 -> est_position - SOFT_DIVIDE_NUM
     * 而 SOFT_DIVIDE_NUM=256 是一圈的 1/4, 即 90° 电角度, 两侧相差 180°。
     *
     * 位置环稳定时 p_error 只剩 ±1~2 counts 的量化噪声, 输出电流在 0 附近
     * 正负跳变, 于是换相角每个 tick 就跳 512 counts。结果是力矩矢量在高频
     * 左右翻 180°: 转子不产生净位移, 却持续受到交变力矩 —— 表现为
     * "电机轴不转, 但用手能摸到震动", 并且线圈会发热、可能发出啸叫。
     *
     * 加滞回后, 只有电流明确超过阈值才换向; 零附近的抖动一律沿用上一次
     * 的方向。真实指令 (cur 200 / pos 阶跃) 远大于阈值, 行为与上游一致。
     */
    if (current > FOC_CURRENT_SIGN_HYST_MA)
    {
        self->foc_sign = 1;
    }
    else if (current < -FOC_CURRENT_SIGN_HYST_MA)
    {
        self->foc_sign = -1;
    }
    else
    {
        /* 保持上一次方向; 若从未出力过则 foc_sign 仍为 0, 走下面的零分支 */
    }

    self->foc_current = current;

    if (self->foc_sign > 0)
    {
        self->foc_position = self->est_position + SOFT_DIVIDE_NUM;  /* 超前 90° */
    }
    else if (self->foc_sign < 0)
    {
        self->foc_position = self->est_position - SOFT_DIVIDE_NUM;  /* 滞后 90° */
    }
    else
    {
        self->foc_position = self->est_position;
    }

    driver_set_foc_current_vector(self->context->driver, (uint32_t)self->foc_position,
                                 self->foc_current);
}

/* 上游: void Motor::Controller::CalcPidToOutput(int32_t _speed) */
static void motion_ctrl_calc_pid_to_output(MotionController *self, int32_t speed)
{
    ControllerPid *pid = &self->config->pid;

    pid->v_error_last = pid->v_error;
    pid->v_error = speed - self->est_velocity;
    if (pid->v_error > (1024 * 1024))
    {
        pid->v_error = (1024 * 1024);
    }
    if (pid->v_error < (-1024 * 1024))
    {
        pid->v_error = (-1024 * 1024);
    }
    pid->output_kp = ((pid->kp) * (pid->v_error));

    pid->integral_round += (pid->ki * pid->v_error);
    pid->integral_remainder = pid->integral_round >> 10;
    pid->integral_round -= (pid->integral_remainder << 10);
    pid->output_ki += pid->integral_remainder;
    /* 积分限幅: ratedCurrent * 1024 */
    if (pid->output_ki > self->context->config.motion_params.rated_current << 10)
    {
        pid->output_ki = self->context->config.motion_params.rated_current << 10;
    }
    else if (pid->output_ki < -(self->context->config.motion_params.rated_current << 10))
    {
        pid->output_ki = -(self->context->config.motion_params.rated_current << 10);
    }

    pid->output_kd = pid->kd * (pid->v_error - pid->v_error_last);

    pid->output = (pid->output_kp + pid->output_ki + pid->output_kd) >> 10;
    if (pid->output > self->context->config.motion_params.rated_current)
    {
        pid->output = self->context->config.motion_params.rated_current;
    }
    else if (pid->output < -self->context->config.motion_params.rated_current)
    {
        pid->output = -self->context->config.motion_params.rated_current;
    }

    motion_ctrl_calc_current_to_output(self, pid->output);
}

/* 上游: void Motor::Controller::CalcDceToOutput(int32_t _location, int32_t _speed) */
static void motion_ctrl_calc_dce_to_output(MotionController *self, int32_t location,
                                           int32_t speed)
{
    ControllerDce *dce = &self->config->dce;

    dce->p_error = location - self->est_position;
    if (dce->p_error > (3200))
    {
        dce->p_error = (3200);      /* 限幅到 1/16 圈 (51200/16) */
    }
    if (dce->p_error < (-3200))
    {
        dce->p_error = (-3200);
    }
    dce->v_error = (speed - self->est_velocity) >> 7;
    if (dce->v_error > (4000))
    {
        dce->v_error = (4000);
    }
    if (dce->v_error < (-4000))
    {
        dce->v_error = (-4000);
    }

    dce->output_kp = dce->kp * dce->p_error;

    dce->integral_round += (dce->ki * dce->p_error + dce->kv * dce->v_error);
    dce->integral_remainder = dce->integral_round >> 7;
    dce->integral_round -= (dce->integral_remainder << 7);
    dce->output_ki += dce->integral_remainder;
    /* 限幅到 ratedCurrent * 1024, 换用不同负载时应重新整定 */
    if (dce->output_ki > self->context->config.motion_params.rated_current << 10)
    {
        dce->output_ki = self->context->config.motion_params.rated_current << 10;
    }
    else if (dce->output_ki < -(self->context->config.motion_params.rated_current << 10))
    {
        dce->output_ki = -(self->context->config.motion_params.rated_current << 10);
    }

    dce->output_kd = ((dce->kd) * (dce->v_error));

    dce->output = (dce->output_kp + dce->output_ki + dce->output_kd) >> 10;
    if (dce->output > self->context->config.motion_params.rated_current)
    {
        dce->output = self->context->config.motion_params.rated_current;
    }
    else if (dce->output < -self->context->config.motion_params.rated_current)
    {
        dce->output = -self->context->config.motion_params.rated_current;
    }

    motion_ctrl_calc_current_to_output(self, dce->output);
}

/* ========================================================================== */
/* Controller: 公开方法                                                        */
/* ========================================================================== */

/* 上游: void Motor::Controller::Init()
 *   注意: 读取全局 boardConfig, 因此必须在配置从 Flash 载入之后再调用。 */
void motion_ctrl_init(MotionController *self)
{
    self->request_mode = boardConfig.enable_motor_on_boot ?
                         (MotorMode)boardConfig.default_mode : MOTOR_MODE_STOP;

    self->mode_running = MOTOR_MODE_STOP;
    self->state = MOTOR_STATE_STOP;

    self->real_lap_position = 0;
    self->real_lap_position_last = 0;
    self->real_position = 0;
    self->real_position_last = 0;

    self->est_velocity_integral = 0;
    self->est_velocity = 0;
    self->est_lead_position = 0;
    self->est_position = 0;
    self->est_error = 0;

    self->goal_position = self->context->config.motion_params.encoder_home_offset;
    self->goal_velocity = 0;
    self->goal_current = 0;
    self->goal_disable = false;
    self->goal_brake = false;

    self->soft_position = 0;
    self->soft_velocity = 0;
    self->soft_current = 0;
    self->soft_disable = false;
    self->soft_brake = false;
    self->soft_new_curve = false;

    self->foc_position = 0;
    self->foc_sign = 0;
    self->foc_current = 0;

    self->stalled_time = 0;
    self->is_stalled = false;

    self->overload_time = 0;
    self->overload_flag = false;

    self->config->pid.v_error = 0;
    self->config->pid.v_error_last = 0;
    self->config->pid.output_kp = 0;
    self->config->pid.output_ki = 0;
    self->config->pid.output_kd = 0;
    self->config->pid.integral_round = 0;
    self->config->pid.integral_remainder = 0;
    self->config->pid.output = 0;

    self->config->dce.p_error = 0;
    self->config->dce.v_error = 0;
    self->config->dce.output_kp = 0;
    self->config->dce.output_ki = 0;
    self->config->dce.output_kd = 0;
    self->config->dce.integral_round = 0;
    self->config->dce.integral_remainder = 0;
    self->config->dce.output = 0;
}

/* 上游: void Motor::Controller::SetCtrlMode(Mode_t _mode) */
void motion_ctrl_set_ctrl_mode(MotionController *self, MotorMode mode)
{
    self->request_mode = mode;
}

/* 上游: void Motor::Controller::SetCurrentSetPoint(int32_t _cur) */
void motion_ctrl_set_current_set_point(MotionController *self, int32_t current)
{
    if (current > self->context->config.motion_params.rated_current)
    {
        self->goal_current = self->context->config.motion_params.rated_current;
    }
    else if (current < -self->context->config.motion_params.rated_current)
    {
        self->goal_current = -self->context->config.motion_params.rated_current;
    }
    else
    {
        self->goal_current = current;
    }
}

/* 上游: void Motor::Controller::SetVelocitySetPoint(int32_t _vel) */
void motion_ctrl_set_velocity_set_point(MotionController *self, int32_t velocity)
{
    if ((velocity >= -self->context->config.motion_params.rated_velocity) &&
        (velocity <= self->context->config.motion_params.rated_velocity))
    {
        self->goal_velocity = velocity;
    }
}

/* 上游: void Motor::Controller::SetPositionSetPoint(int32_t _pos) */
void motion_ctrl_set_position_set_point(MotionController *self, int32_t position)
{
    self->goal_position = position + self->context->config.motion_params.encoder_home_offset;
}

/* 上游: bool Motor::Controller::SetPositionSetPointWithTime(int32_t _pos, float _time) */
bool motion_ctrl_set_position_set_point_with_time(MotionController *self, int32_t position,
                                                  float time_s)
{
    int32_t delta_pos = abs(position - self->real_position +
                            self->context->config.motion_params.encoder_home_offset);
    float p_max = (float)self->context->config.motion_params.rated_velocity_acc *
                  time_s * time_s / 4;

    if ((float)delta_pos > p_max)
    {
        self->context->config.motion_params.rated_velocity = boardConfig.velocity_limit;
        motion_ctrl_set_position_set_point(self, position);

        return false;
    }
    else
    {
        float v_max = time_s * (float)self->context->config.motion_params.rated_velocity_acc;

        v_max -= (float)self->context->config.motion_params.rated_velocity_acc *
                 (sqrtf(time_s * time_s -
                        4 * (float)delta_pos /
                        (float)self->context->config.motion_params.rated_velocity_acc));
        v_max /= 2;

        self->context->config.motion_params.rated_velocity = (int32_t)v_max;
        motion_ctrl_set_position_set_point(self, position);

        return true;
    }
}

/* 上游: float Motor::Controller::GetPosition(bool _isLap = false) */
float motion_ctrl_get_position(MotionController *self, bool is_lap)
{
    if (is_lap)
    {
        return (float)(self->real_lap_position -
                       self->context->config.motion_params.encoder_home_offset) /
               (float)(MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
    }

    return (float)(self->real_position -
                   self->context->config.motion_params.encoder_home_offset) /
           (float)(MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}

/* 上游: float Motor::Controller::GetVelocity() */
float motion_ctrl_get_velocity(MotionController *self)
{
    return (float)self->est_velocity / (float)(MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}

/* 上游: float Motor::Controller::GetFocCurrent() */
float motion_ctrl_get_foc_current(MotionController *self)
{
    return (float)self->foc_current / 1000.0f;
}

/* 上游: void Motor::Controller::AddTrajectorySetPoint(int32_t _pos, int32_t _vel) */
void motion_ctrl_add_trajectory_set_point(MotionController *self, int32_t position,
                                          int32_t velocity)
{
    motion_ctrl_set_position_set_point(self, position);
    motion_ctrl_set_velocity_set_point(self, velocity);
}

/* 上游: void Motor::Controller::SetDisable(bool _disable) */
void motion_ctrl_set_disable(MotionController *self, bool disable)
{
    self->goal_disable = disable;
}

/* 上游: void Motor::Controller::SetBrake(bool _brake) */
void motion_ctrl_set_brake(MotionController *self, bool brake)
{
    self->goal_brake = brake;
}

/* 上游: void Motor::Controller::ApplyPosAsHomeOffset() */
void motion_ctrl_apply_pos_as_home_offset(MotionController *self)
{
    self->context->config.motion_params.encoder_home_offset =
        self->real_position % MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}

/* 上游: void Motor::Controller::ClearStallFlag() */
void motion_ctrl_clear_stall_flag(MotionController *self)
{
    self->stalled_time = 0;
    self->is_stalled = false;
}

/* ========================================================================== */
/* Motor 闭环控制                                                              */
/* ========================================================================== */

/* 上游: void Motor::CloseLoopControlTick() */
static void motor_close_loop_control_tick(Motor *self)
{
    MotionController *controller = self->controller;

    /* ------------------------------ First Called ------------------------------ */
    if (controller->is_first_called)
    {
        int32_t angle;

        if (self->config.motion_params.encoder_home_offset <
            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2)
        {
            angle = (self->encoder->angle_data.rectified_angle >
                     self->config.motion_params.encoder_home_offset +
                     MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2) ?
                    (self->encoder->angle_data.rectified_angle -
                     MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) :
                    self->encoder->angle_data.rectified_angle;
        }
        else
        {
            angle = (self->encoder->angle_data.rectified_angle <
                     self->config.motion_params.encoder_home_offset -
                     MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2) ?
                    (self->encoder->angle_data.rectified_angle +
                     MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) :
                    self->encoder->angle_data.rectified_angle;
        }

        controller->real_lap_position = angle;
        controller->real_lap_position_last = angle;
        controller->real_position = angle;
        controller->real_position_last = angle;

        controller->is_first_called = false;
        return;
    }

    /* ------------------------------- Update Data ------------------------------ */
    {
        int32_t delta_lap_position;

        /* 读取编码器数据 */
        controller->real_lap_position_last = controller->real_lap_position;
        controller->real_lap_position = self->encoder->angle_data.rectified_angle;

        /* 单圈位置差 (处理 51200 -> 0 的回绕) */
        delta_lap_position = controller->real_lap_position -
                             controller->real_lap_position_last;
        if (delta_lap_position > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        {
            delta_lap_position -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        }
        else if (delta_lap_position < -(MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1))
        {
            delta_lap_position += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        }

        /* 累加绝对位置 */
        controller->real_position_last = controller->real_position;
        controller->real_position += delta_lap_position;
    }

    /* ------------------------------ Estimate Data ----------------------------- */
    /* 速度估计 (一阶低通, 系数 1/32) */
    controller->est_velocity_integral +=
        ((controller->real_position - controller->real_position_last) *
         FOC_CONTROL_FREQUENCY +
         ((controller->est_velocity << 5) - controller->est_velocity));
    controller->est_velocity = controller->est_velocity_integral >> 5;
    controller->est_velocity_integral -= (controller->est_velocity << 5);

    /* 位置估计 (超前角补偿) */
    controller->est_lead_position = compensate_advanced_angle(controller->est_velocity);
    controller->est_position = controller->real_position + controller->est_lead_position;

    /* 估计误差 */
    controller->est_error = controller->soft_position - controller->est_position;

    /* --------------------------------- Ctrl Loop ------------------------------- */
    if (controller->is_stalled ||
        controller->soft_disable ||
        !encoder_is_calibrated(self->encoder))
    {
        motion_ctrl_clear_integral(controller);
        controller->foc_position = 0;
        controller->foc_sign = 0;
        controller->foc_current = 0;
        driver_sleep(self->driver);
    }
    else if (controller->soft_brake)
    {
        motion_ctrl_clear_integral(controller);
        controller->foc_position = 0;
        controller->foc_sign = 0;
        controller->foc_current = 0;
        driver_brake(self->driver);
    }
    else
    {
        switch (controller->mode_running)
        {
        case MOTOR_MODE_STEP_DIR:
            motion_ctrl_calc_dce_to_output(controller, controller->soft_position,
                                           controller->soft_velocity);
            break;
        case MOTOR_MODE_STOP:
            driver_sleep(self->driver);
            break;
        case MOTOR_MODE_COMMAND_TRAJECTORY:
            motion_ctrl_calc_dce_to_output(controller, controller->soft_position,
                                           controller->soft_velocity);
            break;
        case MOTOR_MODE_COMMAND_CURRENT:
            motion_ctrl_calc_current_to_output(controller, controller->soft_current);
            break;
        case MOTOR_MODE_COMMAND_VELOCITY:
            motion_ctrl_calc_pid_to_output(controller, controller->soft_velocity);
            break;
        case MOTOR_MODE_COMMAND_POSITION:
            motion_ctrl_calc_dce_to_output(controller, controller->soft_position,
                                           controller->soft_velocity);
            break;
        case MOTOR_MODE_PWM_CURRENT:
            motion_ctrl_calc_current_to_output(controller, controller->soft_current);
            break;
        case MOTOR_MODE_PWM_VELOCITY:
            motion_ctrl_calc_pid_to_output(controller, controller->soft_velocity);
            break;
        case MOTOR_MODE_PWM_POSITION:
            motion_ctrl_calc_dce_to_output(controller, controller->soft_position,
                                           controller->soft_velocity);
            break;
        default:
            break;
        }
    }

    /* ---------------------------- Mode Change Handle --------------------------- */
    if (controller->mode_running != controller->request_mode)
    {
        controller->mode_running = controller->request_mode;
        controller->soft_new_curve = true;
    }

    /* ---------------------------- Update Hard-Goal ----------------------------- */
    if (controller->goal_velocity > self->config.motion_params.rated_velocity)
    {
        controller->goal_velocity = self->config.motion_params.rated_velocity;
    }
    else if (controller->goal_velocity < -self->config.motion_params.rated_velocity)
    {
        controller->goal_velocity = -self->config.motion_params.rated_velocity;
    }

    if (controller->goal_current > self->config.motion_params.rated_current)
    {
        controller->goal_current = self->config.motion_params.rated_current;
    }
    else if (controller->goal_current < -self->config.motion_params.rated_current)
    {
        controller->goal_current = -self->config.motion_params.rated_current;
    }

    /* ------------------------------- Motion Plan ------------------------------- */
    if ((controller->soft_disable && !controller->goal_disable) ||
        (controller->soft_brake && !controller->goal_brake))
    {
        controller->soft_new_curve = true;
    }

    if (controller->soft_new_curve)
    {
        controller->soft_new_curve = false;
        motion_ctrl_clear_integral(controller);
        motion_ctrl_clear_stall_flag(controller);

        switch (controller->mode_running)
        {
        case MOTOR_MODE_STOP:
            break;
        case MOTOR_MODE_COMMAND_POSITION:
            position_tracker_new_task(&self->motion_planner.position_tracker,
                                      controller->est_position, controller->est_velocity);
            break;
        case MOTOR_MODE_COMMAND_VELOCITY:
            velocity_tracker_new_task(&self->motion_planner.velocity_tracker,
                                      controller->est_velocity);
            break;
        case MOTOR_MODE_COMMAND_CURRENT:
            current_tracker_new_task(&self->motion_planner.current_tracker,
                                     controller->foc_current);
            break;
        case MOTOR_MODE_COMMAND_TRAJECTORY:
            trajectory_tracker_new_task(&self->motion_planner.trajectory_tracker,
                                        controller->est_position, controller->est_velocity);
            break;
        case MOTOR_MODE_PWM_POSITION:
            position_tracker_new_task(&self->motion_planner.position_tracker,
                                      controller->est_position, controller->est_velocity);
            break;
        case MOTOR_MODE_PWM_VELOCITY:
            velocity_tracker_new_task(&self->motion_planner.velocity_tracker,
                                      controller->est_velocity);
            break;
        case MOTOR_MODE_PWM_CURRENT:
            current_tracker_new_task(&self->motion_planner.current_tracker,
                                     controller->foc_current);
            break;
        case MOTOR_MODE_STEP_DIR:
            position_interpolator_new_task(&self->motion_planner.position_interpolator,
                                           controller->est_position,
                                           controller->est_velocity);
            /* step/dir 模式用增量位置, 因此目标保持在当前位置 */
            controller->goal_position = controller->est_position;
            break;
        default:
            break;
        }
    }

    /* ---------------------------- Update Soft Goal ----------------------------- */
    switch (controller->mode_running)
    {
    case MOTOR_MODE_STOP:
        break;
    case MOTOR_MODE_COMMAND_POSITION:
        position_tracker_calc_soft_goal(&self->motion_planner.position_tracker,
                                        controller->goal_position);
        controller->soft_position = self->motion_planner.position_tracker.go_location;
        controller->soft_velocity = self->motion_planner.position_tracker.go_velocity;
        break;
    case MOTOR_MODE_COMMAND_VELOCITY:
        velocity_tracker_calc_soft_goal(&self->motion_planner.velocity_tracker,
                                        controller->goal_velocity);
        controller->soft_velocity = self->motion_planner.velocity_tracker.go_velocity;
        break;
    case MOTOR_MODE_COMMAND_CURRENT:
        current_tracker_calc_soft_goal(&self->motion_planner.current_tracker,
                                       controller->goal_current);
        controller->soft_current = self->motion_planner.current_tracker.go_current;
        break;
    case MOTOR_MODE_COMMAND_TRAJECTORY:
        trajectory_tracker_calc_soft_goal(&self->motion_planner.trajectory_tracker,
                                          controller->goal_position,
                                          controller->goal_velocity);
        controller->soft_position = self->motion_planner.trajectory_tracker.go_position;
        controller->soft_velocity = self->motion_planner.trajectory_tracker.go_velocity;
        break;
    case MOTOR_MODE_PWM_POSITION:
        position_tracker_calc_soft_goal(&self->motion_planner.position_tracker,
                                        controller->goal_position);
        controller->soft_position = self->motion_planner.position_tracker.go_location;
        controller->soft_velocity = self->motion_planner.position_tracker.go_velocity;
        break;
    case MOTOR_MODE_PWM_VELOCITY:
        velocity_tracker_calc_soft_goal(&self->motion_planner.velocity_tracker,
                                        controller->goal_velocity);
        controller->soft_velocity = self->motion_planner.velocity_tracker.go_velocity;
        break;
    case MOTOR_MODE_PWM_CURRENT:
        current_tracker_calc_soft_goal(&self->motion_planner.current_tracker,
                                       controller->goal_current);
        controller->soft_current = self->motion_planner.current_tracker.go_current;
        break;
    case MOTOR_MODE_STEP_DIR:
        position_interpolator_calc_soft_goal(&self->motion_planner.position_interpolator,
                                             controller->goal_position);
        controller->soft_position =
            self->motion_planner.position_interpolator.go_position;
        controller->soft_velocity =
            self->motion_planner.position_interpolator.go_velocity;
        break;
    default:
        break;
    }

    controller->soft_disable = controller->goal_disable;
    controller->soft_brake = controller->goal_brake;

    /* ------------------------------- State Check ------------------------------- */
    {
        int32_t current = abs(controller->foc_current);

        /* 堵转检测 */
        if (controller->config->stall_protect_switch)
        {
            if (/* 电流模式 */
                (((controller->mode_running == MOTOR_MODE_COMMAND_CURRENT) ||
                  (controller->mode_running == MOTOR_MODE_PWM_CURRENT)) &&
                 (current != 0))
                || /* 其它模式 */
                (current == self->config.motion_params.rated_current))
            {
                if (abs(controller->est_velocity) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5)
                {
                    if (controller->stalled_time >= 1000 * 1000)
                    {
                        controller->is_stalled = true;
                    }
                    else
                    {
                        controller->stalled_time += FOC_CONTROL_PERIOD_US;
                    }
                }
            }
            else    /* 只能手动清除堵转标志 */
            {
                controller->stalled_time = 0;
            }
        }

        /*
         * 过载检测
         *
         * 必须额外判断 rated_current > 0:
         *   判定用的是 current == rated_current, 若限流被误配成 0,
         *   则 abs(foc_current)==0 恒成立 -> 永远"过载";
         *   同时输出也被限幅到 0 -> 电机完全没有力矩、不转。
         *   两个现象会同时出现, 极易被误判成硬件故障。
         */
        if ((self->config.motion_params.rated_current > 0) &&
            (controller->mode_running != MOTOR_MODE_COMMAND_CURRENT) &&
            (controller->mode_running != MOTOR_MODE_PWM_CURRENT) &&
            (current == self->config.motion_params.rated_current))
        {
            if (controller->overload_time >= 1000 * 1000)
            {
                controller->overload_flag = true;
            }
            else
            {
                controller->overload_time += FOC_CONTROL_PERIOD_US;
            }
        }
        else    /* 释放时自动清除过载标志 */
        {
            controller->overload_time = 0;
            controller->overload_flag = false;
        }
    }

    /* ------------------------------- Update State ------------------------------ */
    if (!encoder_is_calibrated(self->encoder))
    {
        controller->state = MOTOR_STATE_NO_CALIB;
    }
    /*
     * 功率级关断的情况统一报告 STOP:
     *   mode_running == STOP   闭环进入 STOP 分支 (每 tick driver_sleep)
     *   soft_disable           'xdrv off'   置位, 第一个分支走 driver_sleep
     *   soft_brake             'xdrv brake' 置位, 第二个分支走 driver_brake
     * 否则 'off' 之后 mode_running 仍是 vel/pos, 状态机会误报 RUNNING —— 明明
     * 轴已经松开却显示"正在运行", 这是把 'off' 当作"停止"使用时的最大误导。
     */
    else if ((controller->mode_running == MOTOR_MODE_STOP) ||
             controller->soft_disable ||
             controller->soft_brake)
    {
        controller->state = MOTOR_STATE_STOP;
    }
    else if (controller->is_stalled)
    {
        controller->state = MOTOR_STATE_STALL;
    }
    else if (controller->overload_flag)
    {
        controller->state = MOTOR_STATE_OVERLOAD;
    }
    else
    {
        if (controller->mode_running == MOTOR_MODE_COMMAND_POSITION)
        {
            if ((controller->soft_position == controller->goal_position) &&
                (controller->soft_velocity == 0))
            {
                controller->state = MOTOR_STATE_FINISH;
            }
            else
            {
                controller->state = MOTOR_STATE_RUNNING;
            }
        }
        else if (controller->mode_running == MOTOR_MODE_COMMAND_VELOCITY)
        {
            if (controller->soft_velocity == controller->goal_velocity)
            {
                controller->state = MOTOR_STATE_FINISH;
            }
            else
            {
                controller->state = MOTOR_STATE_RUNNING;
            }
        }
        else if (controller->mode_running == MOTOR_MODE_COMMAND_CURRENT)
        {
            if (controller->soft_current == controller->goal_current)
            {
                controller->state = MOTOR_STATE_FINISH;
            }
            else
            {
                controller->state = MOTOR_STATE_RUNNING;
            }
        }
        else
        {
            controller->state = MOTOR_STATE_FINISH;
        }
    }
}

/* ========================================================================== */
/* Motor 对外接口                                                              */
/* ========================================================================== */

/* 上游: void Motor::Tick20kHz() */
void motor_tick_20khz(Motor *self)
{
    /* 1. 更新编码器数据 */
    encoder_update_angle(self->encoder);

    /* 2. 闭环控制更新 */
    motor_close_loop_control_tick(self);
}

/* 上游: void Motor::AttachEncoder(EncoderBase*) */
void motor_attach_encoder(Motor *self, EncoderBase *encoder)
{
    self->encoder = encoder;
}

/* 上游: void Motor::AttachDriver(DriverBase*) */
void motor_attach_driver(Motor *self, DriverBase *driver)
{
    self->driver = driver;
}

/* -------------------------------------------------------------------------- */
/* 上游: Motor() 构造函数
 *
 * 上游构造函数体:
 *     motionPlanner.AttachConfig(&config.motionParams);
 *     controller->AttachConfig(&config.ctrlParams);
 * 另外还有成员初始化器:
 *     Controller controllerInstance = Controller(this);   -> context/mode 初值
 *     Config_t* config = nullptr;  (在 AttachConfig 里赋值)
 *     Controller* controller = &controllerInstance;        (构造函数初始化列表)
 *     State_t state = STATE_STOP;
 * -------------------------------------------------------------------------- */
void motor_init(Motor *self)
{
    /* 上游 Motor() 构造函数里的默认配置 (designated initializer 其余字段为 0) */
    static const MotorConfig MOTOR_CONFIG_DEFAULT =
    {
        .motion_params =
        {
            .encoder_home_offset = 0,
            .cali_current = 2000,                                       /* (mA) */
            .rated_current = 1000,                                      /* (mA) */
            .rated_velocity = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,
            .rated_velocity_acc = 1000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,
            .rated_current_acc = 2 * 1000,                              /* (mA/s) */
        },
        .ctrl_params =
        {
            .pid =
            {
                .kp = 5,
                .ki = 30,
                .kd = 0,
            },
            .dce =
            {
                /*
                 * 与上游不同: 上游默认 200/80/300/250。
                 *
                 * ki=300 在 20 kHz 下积分过快: 每 tick 累加 300*p_error,
                 * 0.2 秒就能冲到 +/-rated_current 限幅, 于是积分在目标点
                 * 两侧来回充放电, 位置环持续振荡。kd=250 的阻尼又只有
                 * 0.0019 mA/(count/s), 几乎起不到抑制作用。
                 *
                 * 实测 500/80/5/250 在 XDriver 硬件上到位后无抖动,
                 * 所以把它作为默认值。若换电机/负载, 用 xdrv gain 重新整定,
                 * 确认后 xdrv save 写进 Flash (此时 Flash 里的值优先)。
                 */
                .kp = 500,
                .kv = 80,
                .ki = 5,
                .kd = 250,
            },
            .stall_protect_switch = false,
        },
    };

    /* 1) 配置: 整体赋默认值 */
    self->config = MOTOR_CONFIG_DEFAULT;

    /* 2) 运动规划器: 先接线再挂配置 (AttachConfig 内部会调用各 tracker 的 Init) */
    motion_planner_init(&self->motion_planner);
    motion_planner_attach_config(&self->motion_planner, &self->config.motion_params);

    /* 3) 控制器: 上游 Controller(this) 只设 context 与两个 mode */
    memset(&self->controller_instance, 0, sizeof(self->controller_instance));
    self->controller_instance.context = self;
    self->controller_instance.request_mode = MOTOR_MODE_STOP;
    self->controller_instance.mode_running = MOTOR_MODE_STOP;
    self->controller_instance.state = MOTOR_STATE_STOP;
    self->controller_instance.is_first_called = true;   /* 上游 static bool = true */

    /* 4) 控制器配置 (上游 controller->AttachConfig(&config.ctrlParams)) */
    motion_ctrl_attach_config(&self->controller_instance, &self->config.ctrl_params);

    /* 5) 指针成员 (上游构造函数初始化列表) */
    self->controller = &self->controller_instance;
    self->encoder = NULL;
    self->driver = NULL;
}
