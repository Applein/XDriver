/*
 * mt6816_base.h
 *
 * 上游: Ctrl/Sensor/Encoder/mt6816_base.h  (class MT6816Base : public EncoderBase)
 *
 * C 化设计 (模式 2: 基类内嵌 + PortOps):
 *   - EncoderBase base 必须是【第一个成员】
 *   - 上游 2 个 protected 纯虚函数 (SpiInit / SpiTransmitAndRead16Bits)
 *     收敛成 const MT6816PortOps *ops
 *   - quickCaliDataPtr 由板级指定 (STOCKPILE_APP_CALI_ADDR)
 */

#ifndef CTRL_STEP_FW_MT6816_BASE_H
#define CTRL_STEP_FW_MT6816_BASE_H

#include <stdbool.h>
#include <stdint.h>

#include "Sensor/Encoder/encoder_base.h"

/* 上游: 2 个 protected 纯虚函数 -> 板级实现表 (定义在 Port/mt6816_stm32.c) */
typedef struct
{
    void (*spi_init)(void);

    /* 全双工 16bit 收发 (片选由实现内部处理) */
    uint16_t (*spi_transmit_and_read16)(uint16_t data_tx);
} MT6816PortOps;

/* 上游: MT6816Base::SpiRawData_t */
typedef struct
{
    uint16_t raw_data;          /* SPI 原始 16bit */
    uint16_t raw_angle;         /* raw_data 中的 14bit 角度 */
    bool no_mag_flag;
    bool checksum_flag;
} MT6816SpiRawData;

typedef struct
{
    EncoderBase base;                   /* 【必须第一个】 */

    const MT6816PortOps *ops;
    uint16_t *quick_cali_data;          /* 校准表基址 (16384 项) */

    MT6816SpiRawData spi_raw_data;
    uint16_t data_tx[2];
    uint16_t data_rx[2];
    uint8_t h_count;

    /*
     * 3 点中值滤波 (可选, 默认关闭)
     *
     * 用途: 抑制"线圈通电时传感器偶发跳变"造成的控制环抖动。
     * 只影响 rectified_angle (控制用), raw_angle 仍保留原始值供诊断。
     * 代价: 1 个 tick (50us) 的群延迟; 高速时引入的滞后可忽略,
     *       但会削掉真实的单 tick 大幅运动 (转速 >10 rev/s 时需谨慎)。
     */
    bool median_enable;
    uint16_t median_hist[2];            /* 最近两次 raw 采样 */
} MT6816Base;
/* 上游: explicit MT6816Base(uint16_t* _quickCaliDataPtr) */
void mt6816_base_init(MT6816Base *self, const MT6816PortOps *ops,
                      uint16_t *quick_cali_data);

/* 上游: EncoderBase 的 3 个虚函数实现 */
bool mt6816_base_init_device(EncoderBase *encoder);          /* Init() */
uint16_t mt6816_base_update_angle(EncoderBase *encoder);     /* UpdateAngle() */
bool mt6816_base_is_calibrated(EncoderBase *encoder);        /* IsCalibrated() */

/* 3 点中值滤波开关 (诊断/应急用, 默认关闭; 只过滤控制用的 rectified_angle) */
void mt6816_base_set_median_filter(MT6816Base *self, bool enable);

#endif /* CTRL_STEP_FW_MT6816_BASE_H */
