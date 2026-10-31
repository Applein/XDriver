/*
 * encoder_calibrator_base.h
 *
 * 上游: Ctrl/Sensor/Encoder/encoder_calibrator_base.h
 *       (class EncoderCalibratorBase, 4 个 protected 纯虚函数)
 *
 * C 化设计 (模式 2: 基类内嵌的替代物 -> 独立结构体 + PortOps):
 *   上游的继承链是 EncoderCalibratorBase (算法) -> EncoderCalibrator (Flash 实现)。
 *   C 里改为:
 *     - EncoderCalibrator 结构体直接持有 Motor* 与 const CaliFlashOps *flash
 *     - 4 个纯虚函数 (BeginWriteFlash / EndWriteFlash / ClearFlash /
 *       WriteFlash16bitsAppend) 收敛成 CaliFlashOps 函数表
 *       (定义在 Port/encoder_calibrator_stm32.c)
 *
 *   上游 static const 成员 -> 文件级枚举 (CALI_*)
 *   上游 static 私有函数 (CycleMod/CycleSubtract/CycleAverage/CycleDataAverage)
 *        -> encoder_calibrator_base.c 内的 static 函数
 *
 * 注意: Tick20kHz() 在 20 kHz 中断里跑, 只做力控推位置与采样;
 *       TickMainLoop() 在主循环线程里跑, 负责校验 + 擦写 Flash (会阻塞几十 ms)。
 */

#ifndef CTRL_STEP_FW_ENCODER_CALIBRATOR_BASE_H
#define CTRL_STEP_FW_ENCODER_CALIBRATOR_BASE_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor/motor.h"

/* 上游: EncoderCalibratorBase 的 static const 成员 */
enum
{
    CALI_SAMPLE_COUNTS_PER_STEP = 16,
    CALI_AUTO_SPEED = 2,
    CALI_FINE_TUNE_CALIB_SPEED = 1,
};

/* 上游: EncoderCalibratorBase::Error_t */
typedef enum
{
    CALI_NO_ERROR = 0x00,
    CALI_ERR_AVERAGE_DIR,
    CALI_ERR_AVERAGE_CONTINUTY,
    CALI_ERR_PHASE_STEP,
    CALI_ERR_ANALYSIS_QUANTITY,
} CaliError;

/* 上游: EncoderCalibratorBase::State_t */
typedef enum
{
    CALI_STATE_DISABLE = 0x00,
    CALI_STATE_FORWARD_PREPARE,
    CALI_STATE_FORWARD_MEASURE,
    CALI_STATE_BACKWARD_RETURN,
    CALI_STATE_BACKWARD_GAP_DISMISS,
    CALI_STATE_BACKWARD_MEASURE,
    CALI_STATE_CALCULATING,
} CaliState;

/* 上游: 4 个 protected 纯虚函数 -> Flash 操作表
 * (定义在 Port/encoder_calibrator_stm32.c, 内部调用 stockpile_*) */
typedef struct
{
    void (*begin_write)(void);
    void (*end_write)(void);
    void (*clear)(void);
    void (*write16_append)(uint16_t data);
} CaliFlashOps;

typedef struct
{
    Motor *motor;

    bool is_triggered;

    CaliError error_code;
    CaliState state;

    uint32_t go_position;
    bool go_direction;

    uint16_t sample_count;
    uint16_t sample_data_raw[CALI_SAMPLE_COUNTS_PER_STEP];
    uint16_t sample_data_average_forward[MOTOR_ONE_CIRCLE_HARD_STEPS + 1];
    uint16_t sample_data_average_backward[MOTOR_ONE_CIRCLE_HARD_STEPS + 1];

    int32_t rcd_x;
    int32_t rcd_y;
    uint32_t result_num;

    const CaliFlashOps *flash;
} EncoderCalibrator;

/* 上游: explicit EncoderCalibratorBase(Motor* _motor) + 派生类构造函数 */
void encoder_calibrator_init(EncoderCalibrator *self, Motor *motor,
                             const CaliFlashOps *flash);

/* 上游: void Tick20kHz()   —— 20 kHz 中断内调用 */
void encoder_calibrator_tick_20khz(EncoderCalibrator *self);

/* 上游: void TickMainLoop()  —— 主循环调用, 会写 Flash */
void encoder_calibrator_tick_main_loop(EncoderCalibrator *self);

#endif /* CTRL_STEP_FW_ENCODER_CALIBRATOR_BASE_H */
