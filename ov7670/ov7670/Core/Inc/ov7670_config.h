#ifndef OV7670_CONFIG_H
#define OV7670_CONFIG_H

#include "stm32f4xx_hal.h"

const uint8_t OV7670_reg[][2] = {
    {0x12, 0x80},
    {0x12, 0x8}, // 0x8 = QCIF
    {0x11, 0b1000000},
};

#endif /* OV7670_CONFIG_H */
