#ifndef __UART_H
#define __UART_H

#include "stm32f1xx_hal.h"

typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;
} uart_t;

uint8_t init(uart_t *uart, UART_HandleTypeDef *huart, IRQn_Type DMA_type, uint32_t baudrate, USART_TypeDef *Instance);

uint8_t send(uart_t *uart, char* data);


#endif



