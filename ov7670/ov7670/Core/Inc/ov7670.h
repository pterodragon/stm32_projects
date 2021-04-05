#ifndef OV7670_H
#define OV7670_H 

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef sccb_write(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef sccb_read(uint8_t reg_addr, uint8_t *pdata);
void ov7670_init();
void ov7670_config();
void ov7670_start_capture();
void ov7670_stop_capture();

#endif /* OV7670_H */
