/*
 * mt6816_base.c
 *
 * 上游: Ctrl/Sensor/Encoder/mt6816_base.cpp
 *
 * 逐行移植。UpdateAngle() 在 20 kHz 中断里每个 tick 调用一次 (经
 * encoder_update_angle()), 因此:
 *   - 3 次重试 + 16bit 奇偶校验的逻辑原样保留
 *   - 校准表查找是 1 次 flash 读 (32KB 表在 0x08017C00), 无插值
 *
 * 注意 spi_raw_data.raw_angle: 上游只在 checksumFlag 为真时更新,
 *   校验失败时沿用上一次的角度 (保持电机控制连续性)。这里保持一致。
 */

#include "Sensor/Encoder/mt6816_base.h"

/* EncoderBase* -> MT6816Base* (base 是首成员, 地址相同) */
#define MT6816(encoder)     ((MT6816Base *)(encoder))

/* -------------------------------------------------------------------------- */
/* 3 点环形中值 (raw 角度域, 0 ~ 16383)
 *
 * 用于抑制"线圈通电时传感器偶发单点跳变"。若不处理, 一次 12 counts 的假
 * 读数会经校准表放大成 ~37 counts, 位置环随即反冲, 形成持续抖动。
 *
 * 环形处理: 先把差值折到 ±8192, 再比较, 避免跨越 16383 -> 0 时判错。
 * -------------------------------------------------------------------------- */
static uint16_t mt6816_median3(uint16_t prev2, uint16_t prev1, uint16_t cur)
{
    const int32_t half = ENCODER_RESOLUTION / 2;
    int32_t x = (int32_t)((uint16_t)((uint16_t)(prev2 - prev1) + half) % ENCODER_RESOLUTION) - half;
    int32_t y = (int32_t)((uint16_t)((uint16_t)(cur - prev1) + half) % ENCODER_RESOLUTION) - half;
    int32_t m;

    if ((x > 0) && (y > 0))
    {
        m = (x < y) ? x : y;            /* 两个都比中间值大 -> 取较小的那个 */
    }
    else if ((x < 0) && (y < 0))
    {
        m = (x > y) ? x : y;            /* 两个都比中间值小 -> 取较大的那个 */
    }
    else
    {
        m = 0;                          /* 中间值本身就是中值 */
    }

    return (uint16_t)(((int32_t)prev1 + m + ENCODER_RESOLUTION) % ENCODER_RESOLUTION);
}

void mt6816_base_set_median_filter(MT6816Base *self, bool enable)
{
    self->median_enable = enable;
}

/* -------------------------------------------------------------------------- */
/* 上游: bool MT6816Base::Init()
 * -------------------------------------------------------------------------- */
bool mt6816_base_init_device(EncoderBase *encoder)
{
    MT6816Base *self = MT6816(encoder);
    uint32_t i;

    self->ops->spi_init();
    mt6816_base_update_angle(encoder);

    /* 检查已存储的校准表是否有效:
     * 全分辨率表, 每个 raw angle count 对应一项 */
    self->base.angle_data.rectify_valid = true;
    for (i = 0; i < (uint32_t)ENCODER_RESOLUTION; i++)
    {
        if (self->quick_cali_data[i] == 0xFFFF)
        {
            self->base.angle_data.rectify_valid = false;
        }
    }

    return self->base.angle_data.rectify_valid;
}

/* -------------------------------------------------------------------------- */
/* 上游: uint16_t MT6816Base::UpdateAngle()
 * -------------------------------------------------------------------------- */
uint16_t mt6816_base_update_angle(EncoderBase *encoder)
{
    MT6816Base *self = MT6816(encoder);
    uint8_t i;
    uint8_t j;

    self->data_tx[0] = (uint16_t)((0x80 | 0x03) << 8);
    self->data_tx[1] = (uint16_t)((0x80 | 0x04) << 8);

    for (i = 0; i < 3; i++)
    {
        self->data_rx[0] = self->ops->spi_transmit_and_read16(self->data_tx[0]);
        self->data_rx[1] = self->ops->spi_transmit_and_read16(self->data_tx[1]);

        self->spi_raw_data.raw_data =
            (uint16_t)(((self->data_rx[0] & 0x00FF) << 8) |
                       (self->data_rx[1] & 0x00FF));

        /* 奇偶校验: 16bit 中 1 的个数必须为偶数 */
        self->h_count = 0;
        for (j = 0; j < 16; j++)
        {
            if (self->spi_raw_data.raw_data & (uint16_t)(0x0001U << j))
            {
                self->h_count++;
            }
        }

        if (self->h_count & 0x01)
        {
            self->spi_raw_data.checksum_flag = false;
        }
        else
        {
            self->spi_raw_data.checksum_flag = true;
            break;
        }
    }

    if (self->spi_raw_data.checksum_flag)
    {
        self->spi_raw_data.raw_angle = (uint16_t)(self->spi_raw_data.raw_data >> 2);
        self->spi_raw_data.no_mag_flag =
            (bool)(self->spi_raw_data.raw_data & (uint16_t)(0x0001U << 1));
    }

    /* angle_data.raw_angle 始终是【未滤波】的原始值, 供 xdrv encjit 诊断 */
    self->base.angle_data.raw_angle = self->spi_raw_data.raw_angle;

    /*
     * 查表用的角度: 可选 3 点中值滤波。
     * 只过滤控制路径 (rectified_angle), 原始值保持不动, 诊断时能分辨
     * "传感器本身在跳" 和 "滤波后仍有跳变"。
     */
    {
        uint16_t raw_for_table = self->spi_raw_data.raw_angle;

        if (self->spi_raw_data.checksum_flag)
        {
            if (self->median_enable)
            {
                raw_for_table = mt6816_median3(self->median_hist[0], self->median_hist[1],
                                               self->spi_raw_data.raw_angle);
            }

            self->median_hist[0] = self->median_hist[1];
            self->median_hist[1] = self->spi_raw_data.raw_angle;
        }

        /*
         * 全分辨率查表: 每个 raw angle count 对应一项精确位置, 不需要插值。
         * 表中位置以 51200 (一圈的细分数) 取模, 因此 table[16383] (~51197)
         * 与 table[0] (~0) 天然相邻; 51200 -> 0 的回绕由控制环自己的
         * 跨圈检测处理 (-51197 的跳变远大于半圈阈值)。
         */
        self->base.angle_data.rectified_angle = self->quick_cali_data[raw_for_table];
    }

    return self->base.angle_data.rectified_angle;
}

/* -------------------------------------------------------------------------- */
/* 上游: bool MT6816Base::IsCalibrated()
 * -------------------------------------------------------------------------- */
bool mt6816_base_is_calibrated(EncoderBase *encoder)
{
    MT6816Base *self = MT6816(encoder);

    return self->base.angle_data.rectify_valid;
}

/* -------------------------------------------------------------------------- */
/* 上游: explicit MT6816Base(uint16_t* _quickCaliDataPtr)
 *   上游构造函数只初始化 quickCaliDataPtr 与 spiRawData(SpiRawData_t{0});
 *   这里显式清零全部状态, 并挂接 EncoderBase 的 3 个虚函数指针。
 * -------------------------------------------------------------------------- */
void mt6816_base_init(MT6816Base *self, const MT6816PortOps *ops,
                      uint16_t *quick_cali_data)
{
    self->ops = ops;
    self->quick_cali_data = quick_cali_data;

    self->base.init = mt6816_base_init_device;
    self->base.update_angle = mt6816_base_update_angle;
    self->base.is_calibrated = mt6816_base_is_calibrated;

    self->base.angle_data.raw_angle = 0;
    self->base.angle_data.rectified_angle = 0;
    self->base.angle_data.rectify_valid = false;

    self->spi_raw_data.raw_data = 0;
    self->spi_raw_data.raw_angle = 0;
    self->spi_raw_data.no_mag_flag = false;
    self->spi_raw_data.checksum_flag = false;

    self->data_tx[0] = 0;
    self->data_tx[1] = 0;
    self->data_rx[0] = 0;
    self->data_rx[1] = 0;
    self->h_count = 0;

    self->median_enable = false;
    self->median_hist[0] = 0;
    self->median_hist[1] = 0;
}
