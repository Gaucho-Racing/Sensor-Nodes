#include "can.h"

uint8_t can_init(can_t *can, CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig) {
    can->hcan = hcan;
    can->hcan->Instance = CAN1;
    can->hcan->Init.Prescaler = 2;
    can->hcan->Init.Mode = CAN_MODE_NORMAL;
    can->hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
    can->hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
    can->hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
    can->hcan->Init.TimeTriggeredMode = DISABLE;
    can->hcan->Init.AutoBusOff = DISABLE;
    can->hcan->Init.AutoWakeUp = DISABLE;
    can->hcan->Init.AutoRetransmission = DISABLE;
    can->hcan->Init.ReceiveFifoLocked = DISABLE;
    can->hcan->Init.TransmitFifoPriority = DISABLE;
    if(HAL_CAN_Init(can->hcan) != HAL_OK) {
        return 0;
    }

    sFilterConfig->FilterIdHigh = 0x0000;
	sFilterConfig->FilterIdLow = 0x0000;
	sFilterConfig->FilterMaskIdHigh = 0x0000;
	sFilterConfig->FilterMaskIdLow = 0x0000;
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig->FilterBank = 0;
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig->FilterActivation = ENABLE;
	if(HAL_CAN_ConfigFilter(can->hcan, sFilterConfig) != HAL_OK){
        return 0;
    }
    return 1;
}

//callback when there is a message in the FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
  //HAL_UART_Transmit(&huart2, (uint8_t *)"CAN RX\r\n", 9, 1000);
}

//starts the can bus and activates the CB register in the CAN controller
uint8_t can_start(can_t *can) {
    HAL_CAN_ActivateNotification(can->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if(HAL_CAN_Start(can->hcan) != HAL_OK) {
        return 0;
    }
    return 1;
}

//basic CAN transmite function
uint8_t can_send(can_t *can, uint8_t data[8], uint32_t id) {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.ExtId = id;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    if(HAL_CAN_AddTxMessage(can->hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
        return 0;
    }
    return 1;
}



