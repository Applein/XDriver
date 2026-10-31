/*
 * mt6816_stm32.c
 *
 * 上游: Port/mt6816_stm32.cpp
 *
 * 与上游的唯一差异:
 *   spi_init 上游调用 MX_SPI1_Init()。本工程 MX_SPI1_Init() 已由
 *   rt_hw_board_init() 调用, 但重复调用它是幂等的 (只是重新写一遍寄存器,
 *   不涉及计数器状态), 因此这里保留上游写法以便与上游逐行对应。
 *
 * 片选 (PA15) 用 BSRR/BRR 直接操作: 单条指令, 适合 20 kHz 路径。
 */

#include "mt6816_stm32.h"

#include "main.h"
#include "spi.h"

#include "Platform/Memory/stockpile_config.h"

/* -------------------------------------------------------------------------- */
/* 上游: void MT6816::SpiInit()  { MX_SPI1_Init(); }                           */
/* -------------------------------------------------------------------------- */
static void mt6816_stm32_spi_init(void)
{
    MX_SPI1_Init();
}

/* -------------------------------------------------------------------------- */
/* 上游: uint16_t MT6816::SpiTransmitAndRead16Bits(uint16_t _dataTx)
 *   PA15 低 -> 收发 1 个 16bit -> PA15 高
 *   注意 hspi1.Init.DataSize = SPI_DATASIZE_16BIT, 因此 Size 参数填 1
 * -------------------------------------------------------------------------- */
static uint16_t mt6816_stm32_spi_transmit_and_read16(uint16_t data_tx)
{
    uint16_t data_rx = 0;

    GPIOA->BRR = (uint32_t)SPI1_CS_Pin;         /* 片选有效 (低) */

    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data_tx, (uint8_t *)&data_rx, 1,
                            HAL_MAX_DELAY);

    GPIOA->BSRR = (uint32_t)SPI1_CS_Pin;        /* 片选无效 (高) */

    return data_rx;
}

const MT6816PortOps mt6816_stm32_port_ops =
{
    .spi_init = mt6816_stm32_spi_init,
    .spi_transmit_and_read16 = mt6816_stm32_spi_transmit_and_read16,
};

void mt6816_stm32_setup(MT6816Base *self)
{
    /* 上游: explicit MT6816() : MT6816Base((uint16_t*)(STOCKPILE_APP_CALI_ADDR)) */
    mt6816_base_init(self, &mt6816_stm32_port_ops,
                     (uint16_t *)(STOCKPILE_APP_CALI_ADDR));
}
