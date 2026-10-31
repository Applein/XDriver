/*
 * motion_planner.h
 *
 * 上游: Ctrl/Motor/motion_planner.h  (class MotionPlanner + 5 个嵌套类)
 *
 * C 化设计 (模式 3: 纯组合 + self 参数):
 *   5 个嵌套类提升为 5 个顶层 struct, 每个结构体的【第一个成员】是
 *   MotionPlanner *context —— 对应上游构造函数里的 CurrentTracker(this)。
 *   因此 motion_planner_init() 必须逐个接线这 5 个指针, 漏一个就是
 *   运行期空指针 (这是本模块唯一的高危点)。
 *
 *   常量:
 *     ControlFrequency = 20000 Hz
 *     ControlPeriod    = 1000000 / 20000 = 50 us
 *   上游是可实例化成员; C 里改成枚举常量 (编译期, 不占 RAM)。
 */

#ifndef CTRL_STEP_FW_MOTION_PLANNER_H
#define CTRL_STEP_FW_MOTION_PLANNER_H

#include <stdint.h>

#include "Motor/motor_types.h"

enum
{
    FOC_CONTROL_FREQUENCY = 20000,                          /* Hz */
    FOC_CONTROL_PERIOD_US = 1000000 / FOC_CONTROL_FREQUENCY, /* 50 us */
};

typedef struct MotionPlanner MotionPlanner;

/* 上游: MotionPlanner::Config_t */
typedef struct
{
    int32_t encoder_home_offset;
    int32_t cali_current;
    int32_t rated_current;
    int32_t rated_velocity;
    int32_t rated_velocity_acc;
    int32_t rated_current_acc;
} MotionPlannerConfig;

/* -------------------------------------------------------------------------- */
/* 上游: MotionPlanner::CurrentTracker                                        */
/* -------------------------------------------------------------------------- */
typedef struct
{
    MotionPlanner *context;
    int32_t go_current;
    int32_t current_acc;
    int32_t current_integral;
    int32_t track_current;
} CurrentTracker;

void current_tracker_init(CurrentTracker *self);
void current_tracker_set_current_acc(CurrentTracker *self, int32_t current_acc);
void current_tracker_new_task(CurrentTracker *self, int32_t real_current);
void current_tracker_calc_soft_goal(CurrentTracker *self, int32_t goal_current);

/* -------------------------------------------------------------------------- */
/* 上游: MotionPlanner::VelocityTracker                                       */
/* -------------------------------------------------------------------------- */
typedef struct
{
    MotionPlanner *context;
    int32_t go_velocity;
    int32_t velocity_acc;
    int32_t velocity_integral;
    int32_t track_velocity;
} VelocityTracker;

void velocity_tracker_init(VelocityTracker *self);
void velocity_tracker_set_velocity_acc(VelocityTracker *self, int32_t velocity_acc);
void velocity_tracker_new_task(VelocityTracker *self, int32_t real_velocity);
void velocity_tracker_calc_soft_goal(VelocityTracker *self, int32_t goal_velocity);

/* -------------------------------------------------------------------------- */
/* 上游: MotionPlanner::PositionTracker                                       */
/* -------------------------------------------------------------------------- */
typedef struct
{
    MotionPlanner *context;
    int32_t go_location;
    int32_t go_velocity;
    int32_t velocity_up_acc;
    int32_t velocity_down_acc;
    float quick_velocity_down_acc;
    int32_t speed_locking_brake;
    int32_t velocity_integral;
    int32_t track_velocity;
    int32_t position_integral;
    int32_t track_position;
} PositionTracker;

void position_tracker_init(PositionTracker *self);
void position_tracker_set_velocity_acc(PositionTracker *self, int32_t value);
void position_tracker_new_task(PositionTracker *self, int32_t real_location,
                               int32_t real_speed);
void position_tracker_calc_soft_goal(PositionTracker *self, int32_t goal_position);

/* -------------------------------------------------------------------------- */
/* 上游: MotionPlanner::PositionInterpolator                                  */
/* -------------------------------------------------------------------------- */
typedef struct
{
    MotionPlanner *context;
    int32_t go_position;
    int32_t go_velocity;
    int32_t record_position;
    int32_t record_position_last;
    int32_t est_position;
    int32_t est_position_integral;
    int32_t est_velocity;
} PositionInterpolator;

void position_interpolator_init(PositionInterpolator *self);
void position_interpolator_new_task(PositionInterpolator *self, int32_t real_position,
                                    int32_t real_velocity);
void position_interpolator_calc_soft_goal(PositionInterpolator *self, int32_t goal_position);

/* -------------------------------------------------------------------------- */
/* 上游: MotionPlanner::TrajectoryTracker                                     */
/* -------------------------------------------------------------------------- */
typedef struct
{
    MotionPlanner *context;
    int32_t go_position;
    int32_t go_velocity;
    int32_t velocity_down_acc;
    int32_t dynamic_velocity_acc;
    int32_t update_time;
    int32_t update_timeout;             /* (ms) 运动设定点命令最大间隔, 默认 200 */
    int32_t overtime_flag;
    int32_t record_velocity;
    int32_t record_position;
    int32_t dynamic_velocity_acc_remainder;
    int32_t velocity_now;
    int32_t velocity_now_remainder;     /* 上游拼写 velovityNowRemainder */
    int32_t position_now;
} TrajectoryTracker;

void trajectory_tracker_init(TrajectoryTracker *self, int32_t update_timeout);
void trajectory_tracker_set_slow_down_velocity_acc(TrajectoryTracker *self, int32_t value);
void trajectory_tracker_new_task(TrajectoryTracker *self, int32_t real_location,
                                 int32_t real_speed);
void trajectory_tracker_calc_soft_goal(TrajectoryTracker *self, int32_t goal_position,
                                       int32_t goal_velocity);

/* -------------------------------------------------------------------------- */
/* 上游: class MotionPlanner                                                  */
/* -------------------------------------------------------------------------- */
struct MotionPlanner
{
    MotionPlannerConfig *config;

    CurrentTracker current_tracker;
    VelocityTracker velocity_tracker;
    PositionTracker position_tracker;
    PositionInterpolator position_interpolator;
    TrajectoryTracker trajectory_tracker;
};

/* 零初始化 + 接线 5 个 tracker 的 context 指针 (取代上游 5 个成员初始化器) */
void motion_planner_init(MotionPlanner *self);

/* 上游: MotionPlanner::AttachConfig()
 *   注意: 内部会调用 5 个 tracker 的 Init(), 其中 position_tracker_init() 与
 *   trajectory_tracker_init() 读取 config->rated_velocity_acc,
 *   因此必须先 memcpy 好 MotionPlannerConfig 再调用本函数。 */
void motion_planner_attach_config(MotionPlanner *self, MotionPlannerConfig *config);

#endif /* CTRL_STEP_FW_MOTION_PLANNER_H */
