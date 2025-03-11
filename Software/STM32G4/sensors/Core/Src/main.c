/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "vl53l0x_api.h"
//#include "circularBuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
PUTCHAR_PROTOTYPE
{
  ITM_SendChar(ch);
  return ch;
}
//CircularBuffer *cb;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// FDCAN_RxHeaderTypeDef RxHeader_FDCAN2;
VL53L0X_Dev_t dev = {.I2cHandle = &hi2c3, .I2cDevAddr = 0x52};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
I2C2 - is meant for thermal sensor MLX90640 and is ran using DMA to offload CPU for calculations
     - still need to fully test but ideally implmentation is done


 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //cb = circular_buffer_init(64, 68 * sizeof(uint8_t));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  
  // HAL_FDCAN_Start(&hfdcan1);
  // HAL_FDCAN_Start(&hfdcan2);
  // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  // HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  
  // static uint16_t eeMLX90640[832];
  // static paramsMLX90640 mlx90640;
  // #define MLX90640_ADDRESS 0x33<<1
  // MLX90640_DumpEE(MLX90640_ADDRESS, eeMLX90640);
  
  // MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  
  // MLX90640_SetRefreshRate(MLX90640_ADDRESS, 0x05);

  // MLX90640_SynchFrame(MLX90640_ADDRESS);
  //  MLX90640_SetRefreshRate(0x33, 0x05);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // unsigned char slaveAddress; 
  // slaveAddress = 0x33<<1;
  // static uint16_t eeMLX90640[832];  
  // static uint16_t mlx90640Frame[834];  
  // paramsMLX90640 mlx90640; 
  // static float mlx90640Image[768]; 
  // int status; 
  // status = MLX90640_DumpEE (slaveAddress, eeMLX90640);   
  // status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640); 
  // status = MLX90640_SetRefreshRate (slaveAddress, 0x05);
  // // status = MLX90640_TriggerMeasurement (slaveAddress);
  // status = MLX90640_GetFrameData (0x33, mlx90640Frame);   
  // // status = MLX90640_GetFrameData (0x33, mlx90640Frame);   
  // MLX90640_GetImage(mlx90640Frame, &mlx90640, mlx90640Image);
  //uint16_t pressure = 0;
  VL53L0X_WaitDeviceBooted(&dev);
  VL53L0X_DataInit(&dev);
  VL53L0X_DeviceParameters_t  params = {0};
  VL53L0X_GetDeviceParameters(&dev, &params);
  params.DeviceMode = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
  VL53L0X_SetDeviceParameters(&dev, &params);
  VL53L0X_GetDeviceParameters(&dev, &params);
  VL53L0X_GetMeasurementTimingBudgetMicroSeconds(&dev, &params.MeasurementTimingBudgetMicroSeconds);
  int status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);
  // VL53L0X_PerformSingleMeasurement(&dev);
  VL53L0X_StartMeasurement(&dev);
  VL53L0X_ClearInterruptMask(&dev, 0);
  VL53L0X_RangingMeasurementData_t measure = {0};


  uint8_t TxData[8] = {0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22};
  // FDCAN_TxHeaderTypeDef TxHeader;
  // TxHeader.Identifier = 0x3FF;
  // TxHeader.IdType = FDCAN_STANDARD_ID;
  // TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  // TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  // TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  // TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  // TxHeader.MessageMarker = 0;

  while (1)
  { 
    int status;
    VL53L0X_GetMeasurementDataReady(&dev, &status);
    VL53L0X_GetRangingMeasurementData(&dev, &measure);
    printf("Distance: %d\n", measure.RangeMilliMeter);
    VL53L0X_ClearInterruptMask(&dev, 0);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
    // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
//   uint8_t RxData[64];
//   HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_FDCAN2, RxData);
//   printf("got messgae\n");
//   //circularBufferPush(cb, RxData, sizeof(RxData));


// }
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
