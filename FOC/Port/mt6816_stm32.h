/*
 * mt6816_stm32.h
 *
 * 上游: Port/mt6816_stm32.h  (class MT6816 : public MT6816Base)
 *
 * C 化后不再有"具体类": 校准表地址 (STOCKPILE_APP_CALI_ADDR) 与板级 SPI 实现
 * 都由 mt6816_stm32_setup() 一次挂好。
 *
 * 用法 (对应上游 `MT6816 mt6816;` + `motor.AttachEncoder(&mt6816)`):
 *      static MT6816Base mt6816;
 *      mt6816_stm32_setup(&mt6816);
 *      motor_attach_encoder(&motor, &mt6816.base);
 */

#ifndef CTRL_STEP_FW_MT6816_STM32_H
#define CTRL_STEP_FW_MT6816_STM32_H

#include "Sensor/Encoder/mt6816_base.h"

/* 板级实现表:
 *   spi_init                 -> MX_SPI1_Init()
 *   spi_transmit_and_read16  -> 拉低 PA15 片选 -> SPI1 全双工 16bit -> 拉高
 */
extern const MT6816PortOps mt6816_stm32_port_ops;

/*
 * 诊断: SPI 收发超出循环上界 (编码器掉线 / 总线卡住) 的累计次数。
 *   - 非 0 且持续增长  -> 编码器接线/供电有问题
 *   - 保持不变         -> SPI 通路健康
 * 正常运行时这个值应当始终为 0。
 */
extern volatile uint32_t mt6816_spi_fail_count;
extern volatile uint32_t mt6816_spi_err_txe;    /* 等 TXE 超时 */
extern volatile uint32_t mt6816_spi_err_rxne;   /* 等 RXNE 超时 (最常见: OVR/MODF) */
extern volatile uint32_t mt6816_spi_err_bsy;    /* 等 BSY 清零超时 */
extern volatile uint16_t mt6816_spi_last_sr;    /* 出错瞬间的 SR */
extern volatile uint16_t mt6816_spi_last_cr1;   /* 出错瞬间的 CR1 */

/* 初始化: 挂 ops、校准表基址 (STOCKPILE_APP_CALI_ADDR) 与 3 个虚函数指针 */
void mt6816_stm32_setup(MT6816Base *self);

#endif /* CTRL_STEP_FW_MT6816_STM32_H */
