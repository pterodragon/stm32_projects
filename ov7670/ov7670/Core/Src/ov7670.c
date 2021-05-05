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

extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_dcmi;
extern UART_HandleTypeDef huart2;

#ifdef DEBUG_PRINTF
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...) 
#endif

#define TIMEOUT_WAIT 100

#ifdef SCCB_BITBANG
#else
extern I2C_HandleTypeDef hi2c2;
HAL_StatusTypeDef sccb_write_hal(uint8_t reg_addr, uint8_t data) {
  return HAL_I2C_Mem_Write(&hi2c2, SCCB_SLAVE_ADDR, reg_addr,
                           I2C_MEMADD_SIZE_8BIT, &data, 1, TIMEOUT_WAIT);
  k

      uint8_t
      sccb_read_hal(uint8_t reg_addr) {
    uint8_t data;
    HAL_I2C_Master_Transmit(&hi2c2, SCCB_SLAVE_ADDR, &reg_addr, 1,
                            TIMEOUT_WAIT);
    HAL_I2C_Master_Receive(&hi2c2, SCCB_SLAVE_ADDR, &data, 1, TIMEOUT_WAIT);
    return data;
  }
#endif

void sda_set(int x) {
  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, x ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void scl_set(int x) {
  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, x ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void delay_us(uint32_t us) {
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < us)
    ;
}

void sccb_delay() { delay_us(5); }

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
    GPIOC->MODER |= GPIO_MODER_MODER12_0;
  } else { // input mode
    GPIOC->MODER &= ~GPIO_MODER_MODER12_0;
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

uint8_t sccb_read(uint8_t reg_addr) {
#ifdef SCCB_BITBANG
  return sccb_read_bitbang(reg_addr);
#else
    return sccb_read_hal(reg_addr);
#endif
}

int sccb_write_bitbang(uint8_t reg_addr, uint8_t data) {
  int res = 0;
  sccb_start();
  res |= sccb_write_byte(SCCB_SLAVE_ADDR);
  res |= sccb_write_byte(reg_addr);
  res |= sccb_write_byte(data);
  sccb_stop();
  return res;
}

int sccb_write(uint8_t reg_addr, uint8_t data) {
#ifdef SCCB_BITBANG
  return sccb_write_bitbang(reg_addr, data);
#else
    return sccb_write_hal(reg_addr, data);
#endif
}

int ov7670_init() {
  {
    uint8_t data = sccb_read(0x0a);
    dprintf("[OV7670] pid = %02X\r\n", data);
    if (data != 0x76)
      return 1;

    data = sccb_read(0x0b);
    dprintf("[OV7670] ver = %02X\r\n", data);
  }

  {
    int err;
    do {
      err = sccb_write(0x12, 0x80); // RESET
      dprintf("reset status err = %d\r\n", err);
    } while (err);
  }
  dprintf("ov7670_init end\r\n");
  return 0;
}
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi) {}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi) {
  dprintf("HAL_DCMI_ErrorCallback\r\n");
  while (1) {
    HAL_Delay(1000);
  }
}
#define CAP_LEN 144 * 174
static uint8_t frame_buf[CAP_LEN * 2];
static uint8_t temp_buf[CAP_LEN];

void ov7670_start_capture() {
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&frame_buf, CAP_LEN / 2);
}

void ov7670_stop_capture() {
  HAL_DCMI_Stop(&hdcmi);
  HAL_Delay(30);
}

int ov7670_config(uint32_t mode) {
  dprintf("ov7670_config\r\n");
  int OV7670_REG_NUM = sizeof(OV7670_reg) / sizeof(OV7670_reg[0]);
  ov7670_stop_capture();
  for (int i = 0; i < OV7670_REG_NUM; ++i) {
    int err = sccb_write(OV7670_reg[i][0], OV7670_reg[i][1]);
    HAL_Delay(1);
    dprintf("sccb_write %x => %x: err = %d\r\n", OV7670_reg[i][0],
           OV7670_reg[i][1], err);
    if (err)
      return 1;
  }
  return 0;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi) {
  static uint32_t count = 0;
  dprintf("frame %ld\r\n", count++);
  for (int i = 0; i < 200; ++i) {
    frame_buf[i] = 0xff;
  }
  for (int i = 1; i < CAP_LEN * 2; i += 2) {
    temp_buf[i / 2] = frame_buf[i];
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)temp_buf, CAP_LEN, HAL_MAX_DELAY);
  HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)&frame_buf, CAP_LEN / 2);
}
