#ifndef __CAN_H
#define __CAN_H

#include "stm32f1xx_hal.h"

typedef struct {
    CAN_HandleTypeDef *hcan;
} can_t;

uint8_t can_init(can_t *can, CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);

uint8_t can_send(can_t *can, uint8_t data[8], uint32_t id);

uint8_t can_start(can_t *can);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif