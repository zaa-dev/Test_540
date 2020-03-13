/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
  
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef canFilterConfig;

uint8_t tx_buf[4] = {0x01, 0xDF, 0xC5, 0xD1};
uint8_t rx_buf[8] = {0};
uint8_t rx_buf_FMI[8][8] = {0};
uint8_t flag = 0;
uint32_t freq_a, freq_b;
uint32_t TxMailbox;
HAL_StatusTypeDef TxStatus;
uint16_t uSendLimit;
uint16_t can_rx_counter = 0;
uint32_t can_sleep = 0;
typedef struct 
{
	uint16_t can_rx;
	uint16_t can_rx_insec;
	uint8_t wakeup;
}counter_t;
counter_t counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFIFO0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_buf) != HAL_OK)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	for (uint8_t i = 0; i < RxHeader.DLC; i++)
	{
		rx_buf_FMI[RxHeader.FilterMatchIndex][i] = rx_buf[i];
	}
	counter.can_rx++;
}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
	counter.wakeup++;
	HAL_CAN_WakeUp(hcan);
	can_sleep = HAL_CAN_IsSleepActive(hcan);	//don*t wakeup without this status request
}
/*void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_CAN_GetError(hcan);
	HAL_CAN_ResetError(hcan);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	canFilterConfig.FilterBank = 0;	/*!< Specifies the filter which will be initialized. 
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 13. */
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;	//CAN_FILTERMODE_IDLIST or CAN_FILTERMODE_IDMASK
  canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;	//CAN_FILTERSCALE_16BIT or CAN_FILTERSCALE_32BIT 
  canFilterConfig.FilterIdHigh = 0x00C1 << 5;	// идентификатор №1
  canFilterConfig.FilterIdLow = 0x00C2 << 5;	// идентификатор №2
  canFilterConfig.FilterMaskIdHigh = 0x00C3 << 5;	// идентификатор №3
  canFilterConfig.FilterMaskIdLow = 0x00C4 << 5;	// идентификатор №4
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  /*canFilterConfig.BankNumber = 0;*/	/*!< Select the start slave bank filter
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 28. */ 
	HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
	
	canFilterConfig.FilterBank = 1;	/*!< Specifies the filter which will be initialized. 
                                       This parameter must be a number between Min_Data = 0 and Max_Data = 13. */
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;	//CAN_FILTERMODE_IDLIST or CAN_FILTERMODE_IDMASK
  canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;	//CAN_FILTERSCALE_16BIT or CAN_FILTERSCALE_32BIT 
  canFilterConfig.FilterIdHigh = 0x00D1 << 5;	// идентификатор №1
  canFilterConfig.FilterIdLow = 0x00D2 << 5;	// идентификатор №2
  canFilterConfig.FilterMaskIdHigh = 0x00D3 << 5;	// идентификатор №3
  canFilterConfig.FilterMaskIdLow = 0x00D4 << 5;	// идентификатор №4
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_BUSOFF|CAN_IT_LAST_ERROR_CODE|CAN_IT_ERROR);
	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, HAL_CAN_RxFIFO0MsgPendingCallback);
	
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_WAKEUP);
	HAL_CAN_RegisterCallback(&hcan, HAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID, HAL_CAN_WakeUpFromRxMsgCallback);
	HAL_CAN_Start(&hcan);
	freq_a = HAL_RCC_GetHCLKFreq();
	
	TxHeader.StdId = 0x0C1;             //
	TxHeader.ExtId = 0x00;                          //
	TxHeader.IDE = CAN_ID_STD ;                 // 
	TxHeader.RTR = CAN_RTR_DATA;                    // 
	TxHeader.DLC = 4;                               // 
     
 counter.can_rx_insec = 0;
 HAL_DBGMCU_EnableDBGSleepMode();	//DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;
 HAL_CAN_RequestSleep(&hcan);
  while (1)
  {
		can_sleep = HAL_CAN_IsSleepActive(&hcan);
		if (!can_sleep)
		{
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
			{
				uSendLimit = 0xFF;
				do 
				{
					TxStatus = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_buf, &TxMailbox);
				}
				while (TxStatus != HAL_OK && uSendLimit--);
				tx_buf[0]++;
			}
		}
		HAL_Delay(1000);
		counter.can_rx_insec = counter.can_rx;
		counter.can_rx = 0;
		
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA5 PA6 PA7 PA8 
                           PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
