/*
 * motion_planner.c
 *
 * 上游: Ctrl/Motor/motion_planner.cpp
 *
 * 逐段移植, 所有整数/浮点运算顺序保持与上游完全一致 (定点积分、余数回填
 * 对数值结果敏感, 不可重排)。
 *
 * 私有辅助函数 (上游各 tracker 的 private 成员):
 *   current_tracker_calc_current_integral()
 *   velocity_tracker_calc_velocity_integral()
 *   position_tracker_calc_velocity_integral() / calc_position_integral()
 *   trajectory_tracker_calc_velocity_integral() / calc_position_integral()
 * 都改成文件内 static。
 */

#include "Motor/motion_planner.h"

#include <stddef.h>     /* NULL */
#include <stdlib.h>     /* abs() */

/* ========================================================================== */
/* CurrentTracker                                                             */
/* ========================================================================== */

static void current_tracker_calc_current_integral(CurrentTracker *self, int32_t current)
{
    self->current_integral += current;
    self->track_current += self->current_integral / FOC_CONTROL_FREQUENCY;
    self->current_integral = self->current_integral % FOC_CONTROL_FREQUENCY;
}

void current_tracker_init(CurrentTracker *self)
{
    current_tracker_set_current_acc(self, self->context->config->rated_current_acc);
}

void current_tracker_set_current_acc(CurrentTracker *self, int32_t current_acc)
{
    self->current_acc = current_acc;
}

void current_tracker_new_task(CurrentTracker *self, int32_t real_current)
{
    self->current_integral = 0;
    self->track_current = real_current;
}

void current_tracker_calc_soft_goal(CurrentTracker *self, int32_t goal_current)
{
    int32_t delta_current = goal_current - self->track_current;

    if (delta_current == 0)
    {
        self->track_current = goal_current;
    }
    else if (delta_current > 0)
    {
        if (self->track_current >= 0)
        {
            current_tracker_calc_current_integral(self, self->current_acc);
            if (self->track_current >= goal_current)
            {
                self->current_integral = 0;
                self->track_current = goal_current;
            }
        }
        else
        {
            current_tracker_calc_current_integral(self, self->current_acc);
            if (self->track_current >= 0)
            {
                self->current_integral = 0;
                self->track_current = 0;
            }
        }
    }
    else    /* delta_current < 0 */
    {
        if (self->track_current <= 0)
        {
            current_tracker_calc_current_integral(self, -self->current_acc);
            if (self->track_current <= goal_current)
            {
                self->current_integral = 0;
                self->track_current = goal_current;
            }
        }
        else
        {
            current_tracker_calc_current_integral(self, -self->current_acc);
            if (self->track_current <= 0)
            {
                self->current_integral = 0;
                self->track_current = 0;
            }
        }
    }

    self->go_current = self->track_current;
}

/* ========================================================================== */
/* VelocityTracker                                                            */
/* ========================================================================== */

static void velocity_tracker_calc_velocity_integral(VelocityTracker *self, int32_t velocity)
{
    self->velocity_integral += velocity;
    self->track_velocity += self->velocity_integral / FOC_CONTROL_FREQUENCY;
    self->velocity_integral = self->velocity_integral % FOC_CONTROL_FREQUENCY;
}

void velocity_tracker_init(VelocityTracker *self)
{
    velocity_tracker_set_velocity_acc(self, self->context->config->rated_velocity_acc);
}

void velocity_tracker_set_velocity_acc(VelocityTracker *self, int32_t velocity_acc)
{
    self->velocity_acc = velocity_acc;
}

void velocity_tracker_new_task(VelocityTracker *self, int32_t real_velocity)
{
    self->velocity_integral = 0;
    self->track_velocity = real_velocity;
}

void velocity_tracker_calc_soft_goal(VelocityTracker *self, int32_t goal_velocity)
{
    int32_t delta_velocity = goal_velocity - self->track_velocity;

    if (delta_velocity == 0)
    {
        self->track_velocity = goal_velocity;
    }
    else if (delta_velocity > 0)
    {
        if (self->track_velocity >= 0)
        {
            velocity_tracker_calc_velocity_integral(self, self->velocity_acc);
            if (self->track_velocity >= goal_velocity)
            {
                self->velocity_integral = 0;
                self->track_velocity = goal_velocity;
            }
        }
        else
        {
            velocity_tracker_calc_velocity_integral(self, self->velocity_acc);
            if (self->track_velocity >= 0)
            {
                self->velocity_integral = 0;
                self->track_velocity = 0;
            }
        }
    }
    else    /* delta_velocity < 0 */
    {
        if (self->track_velocity <= 0)
        {
            velocity_tracker_calc_velocity_integral(self, -self->velocity_acc);
            if (self->track_velocity <= goal_velocity)
            {
                self->velocity_integral = 0;
                self->track_velocity = goal_velocity;
            }
        }
        else
        {
            velocity_tracker_calc_velocity_integral(self, -self->velocity_acc);
            if (self->track_velocity <= 0)
            {
                self->velocity_integral = 0;
                self->track_velocity = 0;
            }
        }
    }

    self->go_velocity = self->track_velocity;
}

/* ========================================================================== */
/* PositionTracker                                                            */
/* ========================================================================== */

static void position_tracker_calc_velocity_integral(PositionTracker *self, int32_t value)
{
    self->velocity_integral += value;
    self->track_velocity += self->velocity_integral / FOC_CONTROL_FREQUENCY;
    self->velocity_integral = self->velocity_integral % FOC_CONTROL_FREQUENCY;
}

static void position_tracker_calc_position_integral(PositionTracker *self, int32_t value)
{
    self->position_integral += value;
    self->track_position += self->position_integral / FOC_CONTROL_FREQUENCY;
    self->position_integral = self->position_integral % FOC_CONTROL_FREQUENCY;
}

void position_tracker_init(PositionTracker *self)
{
    position_tracker_set_velocity_acc(self, self->context->config->rated_velocity_acc);

    /*
     * 速度低于 speedLockingBrake 时允许锁定刹车。
     * 最佳取值应为 ratedMoveAcc / 1000。
     */
    self->speed_locking_brake = self->context->config->rated_velocity_acc / 1000;
}

void position_tracker_set_velocity_acc(PositionTracker *self, int32_t value)
{
    self->velocity_up_acc = value;
    self->velocity_down_acc = value;
    self->quick_velocity_down_acc = 0.5f / (float)self->velocity_down_acc;
}

void position_tracker_new_task(PositionTracker *self, int32_t real_location,
                               int32_t real_speed)
{
    self->velocity_integral = 0;
    self->track_velocity = real_speed;
    self->position_integral = 0;
    self->track_position = real_location;
}

void position_tracker_calc_soft_goal(PositionTracker *self, int32_t goal_position)
{
    int32_t delta_position = goal_position - self->track_position;

    if (delta_position == 0)
    {
        if ((self->track_velocity >= -self->speed_locking_brake) &&
            (self->track_velocity <= self->speed_locking_brake))
        {
            self->velocity_integral = 0;
            self->track_velocity = 0;
            self->position_integral = 0;
        }
        else if (self->track_velocity > 0)
        {
            position_tracker_calc_velocity_integral(self, -self->velocity_down_acc);
            if (self->track_velocity <= 0)
            {
                self->velocity_integral = 0;
                self->track_velocity = 0;
            }
        }
        else if (self->track_velocity < 0)
        {
            position_tracker_calc_velocity_integral(self, self->velocity_down_acc);
            if (self->track_velocity >= 0)
            {
                self->velocity_integral = 0;
                self->track_velocity = 0;
            }
        }
    }
    else
    {
        if (self->track_velocity == 0)
        {
            if (delta_position > 0)
            {
                position_tracker_calc_velocity_integral(self, self->velocity_up_acc);
            }
            else
            {
                position_tracker_calc_velocity_integral(self, -self->velocity_up_acc);
            }
        }
        else if ((delta_position > 0) && (self->track_velocity > 0))
        {
            if (self->track_velocity <= self->context->config->rated_velocity)
            {
                int32_t need_down_location =
                    (int32_t)((float)self->track_velocity * (float)self->track_velocity *
                              (float)self->quick_velocity_down_acc);

                if (abs(delta_position) > need_down_location)
                {
                    if (self->track_velocity < self->context->config->rated_velocity)
                    {
                        position_tracker_calc_velocity_integral(self, self->velocity_up_acc);
                        if (self->track_velocity >= self->context->config->rated_velocity)
                        {
                            self->velocity_integral = 0;
                            self->track_velocity = self->context->config->rated_velocity;
                        }
                    }
                    else if (self->track_velocity > self->context->config->rated_velocity)
                    {
                        position_tracker_calc_velocity_integral(self, -self->velocity_down_acc);
                    }
                }
                else
                {
                    position_tracker_calc_velocity_integral(self, -self->velocity_down_acc);
                    if (self->track_velocity <= 0)
                    {
                        self->velocity_integral = 0;
                        self->track_velocity = 0;
                    }
                }
            }
            else
            {
                position_tracker_calc_velocity_integral(self, -self->velocity_down_acc);
                if (self->track_velocity <= 0)
                {
                    self->velocity_integral = 0;
                    self->track_velocity = 0;
                }
            }
        }
        else if ((delta_position < 0) && (self->track_velocity < 0))
        {
            if (self->track_velocity >= -self->context->config->rated_velocity)
            {
                int32_t need_down_location =
                    (int32_t)((float)self->track_velocity * (float)self->track_velocity *
                              (float)self->quick_velocity_down_acc);

                if (abs(delta_position) > need_down_location)
                {
                    if (self->track_velocity > -self->context->config->rated_velocity)
                    {
                        position_tracker_calc_velocity_integral(self, -self->velocity_up_acc);
                        if (self->track_velocity <= -self->context->config->rated_velocity)
                        {
                            self->velocity_integral = 0;
                            self->track_velocity = -self->context->config->rated_velocity;
                        }
                    }
                    else if (self->track_velocity < -self->context->config->rated_velocity)
                    {
                        position_tracker_calc_velocity_integral(self, self->velocity_down_acc);
                    }
                }
                else
                {
                    position_tracker_calc_velocity_integral(self, self->velocity_down_acc);
                    if (self->track_velocity >= 0)
                    {
                        self->velocity_integral = 0;
                        self->track_velocity = 0;
                    }
                }
            }
            else
            {
                position_tracker_calc_velocity_integral(self, self->velocity_down_acc);
                if (self->track_velocity >= 0)
                {
                    self->velocity_integral = 0;
                    self->track_velocity = 0;
                }
            }
        }
        else if ((delta_position < 0) && (self->track_velocity > 0))
        {
            position_tracker_calc_velocity_integral(self, -self->velocity_down_acc);
            if (self->track_velocity <= 0)
            {
                self->velocity_integral = 0;
                self->track_velocity = 0;
            }
        }
        else if ((delta_position > 0) && (self->track_velocity < 0))
        {
            position_tracker_calc_velocity_integral(self, self->velocity_down_acc);
            if (self->track_velocity >= 0)
            {
                self->velocity_integral = 0;
                self->track_velocity = 0;
            }
        }
    }

    position_tracker_calc_position_integral(self, self->track_velocity);

    self->go_location = self->track_position;
    self->go_velocity = self->track_velocity;
}

/* ========================================================================== */
/* PositionInterpolator                                                       */
/* ========================================================================== */

void position_interpolator_init(PositionInterpolator *self)
{
    /* 上游: void Init() { } 空实现 */
    (void)self;
}

void position_interpolator_new_task(PositionInterpolator *self, int32_t real_position,
                                    int32_t real_velocity)
{
    self->record_position = real_position;
    self->record_position_last = real_position;
    self->est_position = real_position;
    self->est_velocity = real_velocity;
}

void position_interpolator_calc_soft_goal(PositionInterpolator *self, int32_t goal_position)
{
    self->record_position_last = self->record_position;
    self->record_position = goal_position;

    self->est_position_integral +=
        (((self->record_position - self->record_position_last) * FOC_CONTROL_FREQUENCY) +
         ((self->est_velocity << 6) - self->est_velocity));
    self->est_velocity = self->est_position_integral >> 6;
    self->est_position_integral -= (self->est_velocity << 6);

    self->est_position = self->record_position;

    self->go_position = self->est_position;
    self->go_velocity = self->est_velocity;
}

/* ========================================================================== */
/* TrajectoryTracker                                                          */
/* ========================================================================== */

static void trajectory_tracker_calc_velocity_integral(TrajectoryTracker *self, int32_t value)
{
    self->dynamic_velocity_acc_remainder += value;  /* 累加上次的余数 */
    self->velocity_now += self->dynamic_velocity_acc_remainder / FOC_CONTROL_FREQUENCY;
    self->dynamic_velocity_acc_remainder =
        self->dynamic_velocity_acc_remainder % FOC_CONTROL_FREQUENCY;
}

static void trajectory_tracker_calc_position_integral(TrajectoryTracker *self, int32_t value)
{
    self->velocity_now_remainder += value;
    self->position_now += self->velocity_now_remainder / FOC_CONTROL_FREQUENCY;
    self->velocity_now_remainder = self->velocity_now_remainder % FOC_CONTROL_FREQUENCY;
}

void trajectory_tracker_init(TrajectoryTracker *self, int32_t update_timeout)
{
    /* 上游注释掉了 config->ratedVelocityAcc / 10 的写法, 与上游保持一致 */
    trajectory_tracker_set_slow_down_velocity_acc(
        self, self->context->config->rated_velocity_acc);
    self->update_timeout = update_timeout;
}

void trajectory_tracker_set_slow_down_velocity_acc(TrajectoryTracker *self, int32_t value)
{
    self->velocity_down_acc = value;
}

void trajectory_tracker_new_task(TrajectoryTracker *self, int32_t real_location,
                                 int32_t real_speed)
{
    self->update_time = 0;
    self->overtime_flag = 0;
    self->dynamic_velocity_acc_remainder = 0;
    self->velocity_now = real_speed;
    self->velocity_now_remainder = 0;
    self->position_now = real_location;
}

void trajectory_tracker_calc_soft_goal(TrajectoryTracker *self, int32_t goal_position,
                                       int32_t goal_velocity)
{
    if ((goal_velocity != self->record_velocity) || (goal_position != self->record_position))
    {
        self->update_time = 0;
        self->record_velocity = goal_velocity;
        self->record_position = goal_position;

        self->dynamic_velocity_acc =
            (int32_t)((float)(goal_velocity + self->velocity_now) *
                      (float)(goal_velocity - self->velocity_now) /
                      (float)(2 * (goal_position - self->position_now)));

        self->overtime_flag = 0;
    }
    else
    {
        if (self->update_time >= (self->update_timeout * 1000))
        {
            self->overtime_flag = 1;
        }
        else
        {
            self->update_time += FOC_CONTROL_PERIOD_US;
        }
    }

    if (self->overtime_flag)
    {
        if (self->velocity_now == 0)
        {
            self->dynamic_velocity_acc_remainder = 0;
        }
        else if (self->velocity_now > 0)
        {
            trajectory_tracker_calc_velocity_integral(self, -self->velocity_down_acc);
            if (self->velocity_now <= 0)
            {
                self->dynamic_velocity_acc_remainder = 0;
                self->velocity_now = 0;
            }
        }
        else
        {
            trajectory_tracker_calc_velocity_integral(self, self->velocity_down_acc);
            if (self->velocity_now >= 0)
            {
                self->dynamic_velocity_acc_remainder = 0;
                self->velocity_now = 0;
            }
        }
    }
    else
    {
        trajectory_tracker_calc_velocity_integral(self, self->dynamic_velocity_acc);
    }

    trajectory_tracker_calc_position_integral(self, self->velocity_now);

    self->go_position = self->position_now;
    self->go_velocity = self->velocity_now;
}

/* ========================================================================== */
/* MotionPlanner                                                              */
/* ========================================================================== */

/*
 * 取代上游 5 个成员的初始化器 `XXXTracker xxx = XXXTracker(this);`
 * 以及 `MotionPlanner() = default` 的隐式零初始化。
 */
void motion_planner_init(MotionPlanner *self)
{
    self->config = NULL;

    /* ---- 5 条 context 接线: 漏一条就是运行期空指针 ---- */
    self->current_tracker.context = self;
    self->velocity_tracker.context = self;
    self->position_tracker.context = self;
    self->position_interpolator.context = self;
    self->trajectory_tracker.context = self;

    /* ---- CurrentTracker ---- */
    self->current_tracker.go_current = 0;
    self->current_tracker.current_acc = 0;
    self->current_tracker.current_integral = 0;
    self->current_tracker.track_current = 0;

    /* ---- VelocityTracker ---- */
    self->velocity_tracker.go_velocity = 0;
    self->velocity_tracker.velocity_acc = 0;
    self->velocity_tracker.velocity_integral = 0;
    self->velocity_tracker.track_velocity = 0;

    /* ---- PositionTracker ---- */
    self->position_tracker.go_location = 0;
    self->position_tracker.go_velocity = 0;
    self->position_tracker.velocity_up_acc = 0;
    self->position_tracker.velocity_down_acc = 0;
    self->position_tracker.quick_velocity_down_acc = 0.0f;
    self->position_tracker.speed_locking_brake = 0;
    self->position_tracker.velocity_integral = 0;
    self->position_tracker.track_velocity = 0;
    self->position_tracker.position_integral = 0;
    self->position_tracker.track_position = 0;

    /* ---- PositionInterpolator ---- */
    self->position_interpolator.go_position = 0;
    self->position_interpolator.go_velocity = 0;
    self->position_interpolator.record_position = 0;
    self->position_interpolator.record_position_last = 0;
    self->position_interpolator.est_position = 0;
    self->position_interpolator.est_position_integral = 0;
    self->position_interpolator.est_velocity = 0;

    /* ---- TrajectoryTracker (updateTimeout 默认 200ms, 见上游成员初始化器) ---- */
    self->trajectory_tracker.go_position = 0;
    self->trajectory_tracker.go_velocity = 0;
    self->trajectory_tracker.velocity_down_acc = 0;
    self->trajectory_tracker.dynamic_velocity_acc = 0;
    self->trajectory_tracker.update_time = 0;
    self->trajectory_tracker.update_timeout = 200;
    self->trajectory_tracker.overtime_flag = 0;
    self->trajectory_tracker.record_velocity = 0;
    self->trajectory_tracker.record_position = 0;
    self->trajectory_tracker.dynamic_velocity_acc_remainder = 0;
    self->trajectory_tracker.velocity_now = 0;
    self->trajectory_tracker.velocity_now_remainder = 0;
    self->trajectory_tracker.position_now = 0;
}

/*
 * 上游: void MotionPlanner::AttachConfig(MotionPlanner::Config_t* _config)
 *   config 挂上后立即调用 5 个 tracker 的 Init()。
 */
void motion_planner_attach_config(MotionPlanner *self, MotionPlannerConfig *config)
{
    self->config = config;

    current_tracker_init(&self->current_tracker);
    velocity_tracker_init(&self->velocity_tracker);
    position_tracker_init(&self->position_tracker);
    position_interpolator_init(&self->position_interpolator);
    trajectory_tracker_init(&self->trajectory_tracker, 200);
}
