/*
 * encoder_calibrator_stm32.h
 *
 * 上游: Port/encoder_calibrator_stm32.h  (class EncoderCalibrator)
 *
 * C 化后不再有派生类: 4 个 Flash 操作做成函数表, 由
 * encoder_calibrator_stm32_setup() 一次挂到 EncoderCalibrator 上。
 *
 * 用法:
 *      static EncoderCalibrator encoderCalibrator;
 *      encoder_calibrator_stm32_setup(&encoderCalibrator, &motor);
 */

#ifndef CTRL_STEP_FW_ENCODER_CALIBRATOR_STM32_H
#define CTRL_STEP_FW_ENCODER_CALIBRATOR_STM32_H

#include "Sensor/Encoder/encoder_calibrator_base.h"

/* 校准表 Flash 操作表 (内部调用 stockpile_quick_cali) */
extern const CaliFlashOps encoder_calibrator_stm32_flash_ops;

/* 初始化: 挂 motor + Flash 操作表, 状态清零 */
void encoder_calibrator_stm32_setup(EncoderCalibrator *self, Motor *motor);

#endif /* CTRL_STEP_FW_ENCODER_CALIBRATOR_STM32_H */
