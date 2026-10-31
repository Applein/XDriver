#ifndef CTRL_STEP_FW_MT6816_STM32_H
#define CTRL_STEP_FW_MT6816_STM32_H

#include "Sensor/Encoder/mt6816_base.h"
#include "Platform/Memory/stockpile_config.h"

class MT6816 : public MT6816Base
{
public:
    /*
     * _quickCaliDataPtr is the start address where calibration data stored.
     * The address is defined in stockpile_config.h (STOCKPILE_APP_CALI_ADDR).
     * Full-resolution table: RESOLUTION entries (16384 uint16_t, 32KB total),
     * one position value per raw angle count, no interpolation needed.
     */
    explicit MT6816() : MT6816Base((uint16_t*) (STOCKPILE_APP_CALI_ADDR))
    {}


private:
    void SpiInit() override;

    uint16_t SpiTransmitAndRead16Bits(uint16_t _data) override;
};

#endif
