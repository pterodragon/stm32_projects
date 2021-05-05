#ifndef OV7670_H
#define OV7670_H 

#include "stm32f4xx_hal.h"

int sccb_write(uint8_t reg_addr, uint8_t data);
uint8_t sccb_read(uint8_t reg_addr);
int ov7670_init();
int ov7670_config();
void ov7670_start_capture();
void ov7670_stop_capture();

#endif /* OV7670_H */
