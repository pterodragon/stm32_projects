#include "ov7670.h"
#include "ov7670_config.h"
#include <stdio.h>
#include <stm32f4xx_ll_cortex.h>

#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_12
#define SDA_GPIO_Port GPIOC

#define CAMERA_RESET_Pin GPIO_PIN_5
#define CAMERA_RESET_GPIO_Port GPIOC

#define SCCB_SLAVE_ADDR 0x42

extern I2C_HandleTypeDef hi2c2;
extern DCMI_HandleTypeDef hdcmi;

#define TIMEOUT_WAIT 100

HAL_StatusTypeDef sccb_write_hal(uint8_t reg_addr, uint8_t data) {
  return HAL_I2C_Mem_Write(&hi2c2, SCCB_SLAVE_ADDR, reg_addr,
                           I2C_MEMADD_SIZE_8BIT, &data, 1, TIMEOUT_WAIT);
}

HAL_StatusTypeDef sccb_write(uint8_t reg_addr, uint8_t data) {
#ifdef SCCB_BITBANG
#else
  return sccb_write_hal(reg_addr, data);
#endif
}

uint8_t sccb_read_hal(uint8_t reg_addr) {
  uint8_t data;
  HAL_I2C_Master_Transmit(&hi2c2, SCCB_SLAVE_ADDR, &reg_addr, 1, TIMEOUT_WAIT);
  HAL_I2C_Master_Receive(&hi2c2, SCCB_SLAVE_ADDR, &data, 1, TIMEOUT_WAIT);
  return data;
}
void sda_set(int x) {
  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, x ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void scl_set(int x) {
  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, x ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static uint8_t fac_us = 0;
static uint16_t fac_ms = 0;

void delay_init(uint8_t sysclk) {
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK_DIV8);
  fac_us = sysclk / 8;
  fac_ms = (uint16_t)fac_us * 1000;
  RCC;
}

void delay_us(uint32_t nus) {
  uint32_t temp;
  SysTick->LOAD = nus * fac_us;
  SysTick->VAL = 0x00;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
  do {
    temp = SysTick->CTRL;
  } while ((temp & 0x01) && !(temp & (1 << 16)));
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  SysTick->VAL = 0X00;
}

void sccb_delay() {
  delay_us(5);
}

void sccb_stop(void) {
  sda_set(0);
  sccb_delay();
  scl_set(1);
  sccb_delay();
  sda_set(1);
  sccb_delay();
}

void sccb_start() {
  sda_set(1);
  scl_set(1);
  sccb_delay();
  sda_set(0);
  sccb_delay();
  scl_set(0);
}

void sccb_no_ack() {
  sccb_delay();
  sda_set(1);
  scl_set(1);
  sccb_delay();
  scl_set(0);
  sccb_delay();
  sda_set(0);
  sccb_delay();
}

void sda_set_io_mode(int x) {
  // 1: output mode; 0: input mode
  if (x) { // output mode
    GPIOC->MODER |= GPIO_MODER_MODER12;
  } else { // input mode
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
  }
}

GPIO_PinState sda_read() { return HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin); }

uint8_t sccb_read_byte() {
  sda_set_io_mode(0);

  uint8_t res = 0;
  for (int j = 0; j < 8; ++j) {
    sccb_delay();
    scl_set(1);
    res <<= 1;
    if (sda_read())
      ++res;
    sccb_delay();
    scl_set(0);
  }

  sda_set_io_mode(1);
  return res;
}

int sccb_write_byte(uint8_t data) {
  for (int j = 0; j < 8; ++j) {
    sda_set(data & 0x80);
    data <<= 1;
    sccb_delay();
    scl_set(1);
    sccb_delay();
    scl_set(0);
  }
  sda_set_io_mode(0);
  sccb_delay();
  scl_set(1);
  sccb_delay();
  GPIO_PinState res = sda_read(); // can check if ack or not
  scl_set(0);
  sda_set_io_mode(1);
  return res;
}

uint8_t sccb_read_bitbang(uint8_t reg_addr) {
  sccb_start();
  sccb_write_byte(SCCB_SLAVE_ADDR);
  sccb_write_byte(reg_addr);
  sccb_stop();
  sccb_delay();
  sccb_start();
  sccb_write_byte(SCCB_SLAVE_ADDR | 0x01);
  uint8_t res = sccb_read_byte();
  sccb_no_ack();
  sccb_stop();
  return res;
}

HAL_StatusTypeDef sccb_write_bitbang(uint8_t reg_addr, uint8_t data) {
  uint8_t res = 0;
  sccb_start();
  sccb_write_byte(SCCB_SLAVE_ADDR);
  if (sccb_write_byte(SCCB_SLAVE_ADDR))
    res = 1;
  if (sccb_write_byte(reg_addr))
    res = 1;
  if (sccb_write_byte(data))
    res = 1;
  sccb_stop();
  return res;
}

HAL_StatusTypeDef sccb_read(uint8_t reg_addr) {
#ifdef SCCB_BITBANG
  return sccb_read_bitbang(reg_addr);
#else
  return sccb_read_hal(reg_addr);
#endif
}

int ov7670_init() {
  delay_init(170);

  // returns 1 if errored, 0 if ok
  // HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_RESET);
  // HAL_Delay(100);
  // HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
  // HAL_Delay(100);

  uint8_t data = sccb_read(0x0a);
  printf("[OV7670] pid = %02X\r\n", data);
  if (data != 0x76)
    return 1;

  data = sccb_read(0x0b);
  printf("[OV7670] ver = %02X\r\n", data);
  return 1;

  {
    HAL_StatusTypeDef status;
    do {
      status = sccb_write(0x12, 0x80); // RESET
      HAL_Delay(30);
      printf("reset status = %d\r\n", status);
    } while (status != HAL_OK && 0);
  }
  return 0;
}

#define CAP_LEN 320 * 240
static uint8_t pic_buffer[CAP_LEN];

void ov7670_start_capture() {
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)pic_buffer,
                     CAP_LEN / 4);
  HAL_Delay(2000);
  ov7670_stop_capture();
}

void ov7670_stop_capture() {
  // HAL_DCMI_Suspend(&hdcmi);
  HAL_DCMI_Stop(&hdcmi);
  HAL_Delay(30);
}

void ov7670_config(uint32_t mode) {
  ov7670_stop_capture();
  // sccb_write(0x12, 0x80);  // RESET
  // HAL_Delay(30);
  for (int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
    HAL_StatusTypeDef status;
    do {
      status = sccb_write(OV7670_reg[i][0], OV7670_reg[i][1]);
      HAL_Delay(1);
      printf("sccb_write %x => %x: %d\r\n", OV7670_reg[i][0], OV7670_reg[i][1],
             status);
    } while (status != HAL_OK && 0);
  }
}

// RET ov7670_startCap(uint32_t capMode, uint32_t destAddress)
// {
//   ov7670_stopCap();
//   if (capMode == OV7670_CAP_CONTINUOUS) {
//     /* note: continuous mode automatically invokes DCMI, but DMA needs to be
//     invoked manually */ s_destAddressForContiuousMode = destAddress;
//     HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, destAddress,
//     OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//   } else if (capMode == OV7670_CAP_SINGLE_FRAME) {
//     s_destAddressForContiuousMode = 0;
//     HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress,
//     OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//   }
//
//   return RET_OK;
// }
//
// RET ov7670_stopCap()
// {
//   HAL_DCMI_Stop(sp_hdcmi);
// //  HAL_Delay(30);
//   return RET_OK;
// }
//
