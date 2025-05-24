#include "bmi323.h"
#include <stdio.h>

/* SPI read/write bit */
#define BMI323_SPI_READ          0x80
#define BMI323_SPI_WRITE         0x00

/**
  * @brief Select the BMI323 chip
  */

static void BMI323_CS_Low(void)
{
  // Now assert CS
  HAL_GPIO_WritePin(BMI323_CS_GPIO_Port, BMI323_CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief Deselect the BMI323 chip
  */
static void BMI323_CS_High(void)
{
  HAL_GPIO_WritePin(BMI323_CS_GPIO_Port, BMI323_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief Read a register from BMI323
  * @param reg: Register address
  * @param data: Pointer to data buffer
  * @retval HAL status
  */
HAL_StatusTypeDef BMI323_ReadReg(uint8_t reg, uint8_t *data)
{
  HAL_StatusTypeDef status;
  uint16_t tx_word = 0;
  uint16_t rx_word = 0;
  
  // Set read bit in address and put it in the MSB (upper byte) of the 16-bit word
  tx_word = ((uint16_t)(reg | BMI323_SPI_READ) << 8);
  
  BMI323_CS_Low();
  
  // Transmit address and receive data using 16-bit mode
  status = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx_word, (uint8_t*)&rx_word, 1, HAL_MAX_DELAY);
  
  // Wait for transmission to complete
  while(hspi3.State == HAL_SPI_STATE_BUSY);
  
  BMI323_CS_High();
  
  if (status != HAL_OK) {
    return status;
  }
  
  // Data is in the lower byte (second byte) of the received word
  *data = (uint8_t)(rx_word & 0xFF);
  
  return status;
}

/**
  * @brief Write a register to BMI323
  * @param reg: Register address
  * @param data: Data to write
  * @retval HAL status
  */
HAL_StatusTypeDef BMI323_WriteReg(uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef status;
  uint16_t tx_word;
  
  // Command in upper byte, data in lower byte
  tx_word = ((uint16_t)(reg | BMI323_SPI_WRITE) << 8) | data;
  
  BMI323_CS_Low();
  status = HAL_SPI_Transmit(&hspi3, (uint8_t*)&tx_word, 1, HAL_MAX_DELAY);
  while(hspi3.State == HAL_SPI_STATE_BUSY);
  BMI323_CS_High();
  
  return status;
}

/**
  * @brief Initialize the BMI323 sensor
  * @retval HAL status
  */
HAL_StatusTypeDef BMI323_Init(void)
{
  HAL_StatusTypeDef status;
  uint8_t chip_id = 0;
  uint8_t status_reg = 0;
  
  // Configure CS pin
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = BMI323_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BMI323_CS_GPIO_Port, &GPIO_InitStruct);
  
  
  
  // Step 2: Read chip ID to verify communication
  status = BMI323_ReadReg(BMI323_REG_CHIP_ID, &chip_id);
  if (status != HAL_OK) {
    printf("BMI323: Failed to read chip ID\r\n");
    return status;
  }
  
  // Verify chip ID
  if (chip_id != BMI323_CHIP_ID) {
    printf("BMI323: Invalid chip ID (got 0x%02X, expected 0x%02X)\r\n", chip_id, BMI323_CHIP_ID);
    return HAL_ERROR;
  }
  
  printf("BMI323: Chip ID verified (0x%02X)\r\n", chip_id);
  
  // Step 3: Check device status
  status = BMI323_ReadReg(BMI323_REG_STATUS, &status_reg);
  if (status != HAL_OK) {
    printf("BMI323: Failed to read status register\r\n");
    return status;
  }
  
  printf("BMI323: Status register value: 0x%02X\r\n", status_reg);
  
  // Step 4: Configure the sensor (example configuration)
  
  printf("BMI323: Initialization complete\r\n");
  return HAL_OK;
}
