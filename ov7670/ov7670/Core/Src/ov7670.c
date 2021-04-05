#include "ov7670.h"
#include "ov7670_config.h"
#include <stdio.h>

#define SCCB_WRITE_SLAVE_ADDR 0x42
#define SCCB_READ_SLAVE_ADDR 0x43

extern I2C_HandleTypeDef hi2c2;
extern DCMI_HandleTypeDef hdcmi;

HAL_StatusTypeDef sccb_write(uint8_t reg_addr, uint8_t data) {
  return HAL_I2C_Mem_Write(&hi2c2, SCCB_WRITE_SLAVE_ADDR , reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

HAL_StatusTypeDef sccb_read(uint8_t reg_addr, uint8_t *data) {
  HAL_StatusTypeDef ret;
  ret = HAL_I2C_Master_Transmit(&hi2c2, SCCB_WRITE_SLAVE_ADDR , &reg_addr, 1, 100);
  ret |= HAL_I2C_Master_Receive(&hi2c2, SCCB_WRITE_SLAVE_ADDR, data, 1, 100);
  return ret;
}
#define CAMERA_RESET_Pin GPIO_PIN_5
#define CAMERA_RESET_GPIO_Port GPIOC

void ov7670_init() {
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  sccb_write(0x12, 0x80);  // RESET
  HAL_Delay(30);

  uint8_t buffer[4];
  sccb_read(0x0b, buffer);
  printf("[OV7670] dev id = %02X\n", buffer[0]);
}

#define CAP_LEN 320 * 100
static uint8_t pic_buffer[CAP_LEN];

void ov7670_start_capture() {
  ov7670_stop_capture();
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) pic_buffer, CAP_LEN/4);
}

void ov7670_stop_capture() {
  HAL_DCMI_Stop(&hdcmi);
}

void ov7670_config(uint32_t mode)
{
  ov7670_stop_capture();
  sccb_write(0x12, 0x80);  // RESET
  HAL_Delay(30);
  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
    HAL_StatusTypeDef status = sccb_write(OV7670_reg[i][0], OV7670_reg[i][1]);
    HAL_Delay(1);
    printf("sccb_write %x => %x: %d\n", OV7670_reg[i][0], OV7670_reg[i][1], status);
  }
}

// RET ov7670_startCap(uint32_t capMode, uint32_t destAddress)
// {
//   ov7670_stopCap();
//   if (capMode == OV7670_CAP_CONTINUOUS) {
//     /* note: continuous mode automatically invokes DCMI, but DMA needs to be invoked manually */
//     s_destAddressForContiuousMode = destAddress;
//     HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//   } else if (capMode == OV7670_CAP_SINGLE_FRAME) {
//     s_destAddressForContiuousMode = 0;
//     HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
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
