/*
 * encoder_base.h
 *
 * 上游: Ctrl/Sensor/Encoder/encoder_base.h  (class EncoderBase, 3 个纯虚函数)
 *
 * C 化设计 (模式 1: 虚表):
 *   与 DriverBase 相同的做法 —— Motor 只持有 EncoderBase *。
 *
 * 注意 ENCODER_RESOLUTION:
 *   上游是 const int32_t RESOLUTION = ((int32_t)(1U << 14)),
 *   这里改成枚举常量 (编译期计算, 不占 RAM)。
 *   校准表是"全分辨率查表": 16384 项 uint16_t = 32KB, 无需插值。
 */

#ifndef CTRL_STEP_FW_ENCODER_BASE_H
#define CTRL_STEP_FW_ENCODER_BASE_H

#include <stdbool.h>
#include <stdint.h>

enum
{
    ENCODER_RESOLUTION = 1 << 14,   /* 16384 */
};

/* 上游: EncoderBase::AngleData_t */
typedef struct
{
    uint16_t raw_angle;         /* SPI 原始角度 */
    uint16_t rectified_angle;   /* 查表校正后的角度 (0 ~ 51199 细分步) */
    bool rectify_valid;         /* 校准表是否有效 */
} EncoderAngleData;

typedef struct EncoderBase EncoderBase;

struct EncoderBase
{
    EncoderAngleData angle_data;

    uint16_t (*update_angle)(EncoderBase *self);
    bool (*is_calibrated)(EncoderBase *self);
    /* 上游 Init() 返回 bool; 纯 SPI 读取类编码器返回 true,
     * MT6816 返回校准表是否有效 */
    bool (*init)(EncoderBase *self);
};

/* 便捷包装, 与上游 motor->encoder->UpdateAngle() 一一对应 */
static inline uint16_t encoder_update_angle(EncoderBase *self)
{
    return self->update_angle(self);
}

static inline bool encoder_is_calibrated(EncoderBase *self)
{
    return self->is_calibrated(self);
}

static inline bool encoder_init(EncoderBase *self)
{
    return self->init(self);
}

#endif /* CTRL_STEP_FW_ENCODER_BASE_H */
