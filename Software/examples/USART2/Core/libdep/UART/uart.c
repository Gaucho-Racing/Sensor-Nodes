#include "uart.h"
#include "stm32f1xx_hal.h"
#include <string.h>

uint8_t init(uart_t *uart, UART_HandleTypeDef *huart, IRQn_Type DMA_type, uint32_t baudrate, USART_TypeDef *Instance) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA_type, 0, 0);
    HAL_NVIC_EnableIRQ(DMA_type);
    uart->huart = huart;
    uart->huart->Instance = Instance;
    uart->huart->Init.BaudRate = baudrate;
    uart->huart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->huart->Init.StopBits = UART_STOPBITS_1;
    uart->huart->Init.Parity = UART_PARITY_NONE;
    uart->huart->Init.Mode = UART_MODE_TX_RX;
    uart->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->huart->Init.OverSampling = UART_OVERSAMPLING_16;
    return (HAL_UART_Init(uart->huart) == HAL_OK);
}

uint8_t send(uart_t *uart, char* data) {
    return HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)data, strlen(data)) == HAL_OK;
}
