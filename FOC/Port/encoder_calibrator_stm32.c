/*
 * encoder_calibrator_stm32.c
 *
 * 上游: Port/encoder_calibrator_stm32.cpp
 *
 * 4 个 Flash 操作与上游一一对应:
 *   BeginWriteFlash        -> Stockpile_Flash_Data_Begin(&stockpile_quick_cali)
 *   EndWriteFlash          -> Stockpile_Flash_Data_End(&stockpile_quick_cali)
 *   ClearFlash             -> Stockpile_Flash_Data_Empty(&stockpile_quick_cali)
 *   WriteFlash16bitsAppend -> Stockpile_Flash_Data_Write_Data16(...)
 *
 * 目标区: STOCKPILE_APP_CALI_ADDR = 0x08017C00, 32KB (16384 x uint16_t)
 *         与 STM32F103xx_FLASH.ld 的 0x17C00 代码上限是同一处配置,
 *         改其中一个必须同步另一个。
 */

#include "encoder_calibrator_stm32.h"

#include "Platform/Memory/stockpile_f103cb.h"

static void encoder_calibrator_stm32_begin_write(void)
{
    Stockpile_Flash_Data_Begin(&stockpile_quick_cali);
}

static void encoder_calibrator_stm32_end_write(void)
{
    Stockpile_Flash_Data_End(&stockpile_quick_cali);
}

static void encoder_calibrator_stm32_clear(void)
{
    Stockpile_Flash_Data_Empty(&stockpile_quick_cali);
}

static void encoder_calibrator_stm32_write16_append(uint16_t data)
{
    Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &data, 1);
}

const CaliFlashOps encoder_calibrator_stm32_flash_ops =
{
    .begin_write = encoder_calibrator_stm32_begin_write,
    .end_write = encoder_calibrator_stm32_end_write,
    .clear = encoder_calibrator_stm32_clear,
    .write16_append = encoder_calibrator_stm32_write16_append,
};

void encoder_calibrator_stm32_setup(EncoderCalibrator *self, Motor *motor)
{
    encoder_calibrator_init(self, motor, &encoder_calibrator_stm32_flash_ops);
}
