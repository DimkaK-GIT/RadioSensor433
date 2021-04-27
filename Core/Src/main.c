/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "hdlc.h"
#include "intflash.h"
#include "hc12.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ID_Current_Sensor 0x90000003
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId ConnectTaskHandle;
osThreadId SensorTaskHandle;
osThreadId PowerTaskHandle;
osMessageQId AnswerQueueHandle;
osMessageQId RecQueueHandle;
/* USER CODE BEGIN PV */
char report[30];
uint8_t recUart = 0;
extern interfaceHDLC_TX masterTX;
extern interfaceHDLC_RX masterRX;

extern uint8_t answerAT;
extern char hcAnswer[20];
structParameterHeader Param;

uint8_t HC12Mode = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);
void ConnectTaskStart(void const * argument);
void SensorTaskStart(void const * argument);
void PowerTaskStart(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch((uint32_t)huart->Instance)
	{
		case (uint32_t)USART1: // 433MHz

					xQueueSendFromISR(RecQueueHandle, (void *)&recUart, pdFALSE);
					HAL_UART_Receive_IT(&huart1, (void *)&recUart, 1);													
					break;
	}		
	__NOP();
}

void SendPacket433(uint8_t ADR_DST, uint8_t ADR_SRC, uint32_t ID_Sensor,uint8_t Command)
{
	uint8_t trmdata[10];
	struct_HDLC_Header MasterHDLC;
	MasterHDLC.ADR_DST = ADR_DST;
	MasterHDLC.ADR_SRC = ADR_SRC;
	trmdata[0] = (uint8_t)(	ID_Sensor & 0x000000FF);
	trmdata[1] = (uint8_t)((ID_Sensor & 0x0000FF00)>>8);
	trmdata[2] = (uint8_t)((ID_Sensor & 0x00FF0000)>>16);
	trmdata[3] = (uint8_t)((ID_Sensor & 0xFF000000)>>24);
	send_Master_HDLC (MasterHDLC,Command, (char*)trmdata, 4);

}

structParameterHeader ReadParam()
{
	structParameterHeader result;
	
	flash_read(PARAM_START_ADDRESS,(uint32_t*)&result,sizeof(result) / 4);
	
	return result;
}

void WriteParam (structParameterHeader Parameter)
{
	
	flash_write(PARAM_START_ADDRESS, (uint32_t *)&Parameter, sizeof(Parameter) / 4);

}	

structSensorHeader isSensorChange(structSensorHeader sens)
{
	structSensorHeader result = sens;
	
	result.change = 0;
	uint8_t SensorState = HAL_GPIO_ReadPin( sens.SensorPort,sens.SensorPin);
	
	if(SensorState == GPIO_PIN_RESET)
	{
		if(sens.countSensor<25)
			result.countSensor = sens.countSensor + 1;
		else
			result.countSensor = sens.countSensor;
	}
	else 
	{
		if(sens.countSensor > 1)
			result.countSensor = sens.countSensor - 1;
		else
			result.countSensor = sens.countSensor;
	}
		
		if(result.countSensor > 20)
			result.sensor = GPIO_PIN_RESET;
		
		if(result.countSensor < 4)
			result.sensor = GPIO_PIN_SET;

		if(	result.sensor != sens.prevSensor )
		{
			result.change = 1;
			result.prevSensor = result.sensor;
		}
		
		return result;
}
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
//	sprintf(report,"Module start\r\n");
	
//	CDC_Transmit_FS((uint8_t*)report,strlen(report));
//  __enable_irq();
	HAL_UART_Receive_IT(&huart1, (void *)&recUart, 1);	


	Param = ReadParam();
	
	

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of AnswerQueue */
  osMessageQDef(AnswerQueue, 16, uint16_t);
  AnswerQueueHandle = osMessageCreate(osMessageQ(AnswerQueue), NULL);

  /* definition and creation of RecQueue */
  osMessageQDef(RecQueue, 16, uint16_t);
  RecQueueHandle = osMessageCreate(osMessageQ(RecQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ConnectTask */
  osThreadDef(ConnectTask, ConnectTaskStart, osPriorityNormal, 0, 512);
  ConnectTaskHandle = osThreadCreate(osThread(ConnectTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, SensorTaskStart, osPriorityAboveNormal, 0, 512);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of PowerTask */
  osThreadDef(PowerTask, PowerTaskStart, osPriorityNormal, 0, 128);
  PowerTaskHandle = osThreadCreate(osThread(PowerTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(set_GPIO_Port, set_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : key_Pin sensor_Pin */
  GPIO_InitStruct.Pin = key_Pin|sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : set_Pin */
  GPIO_InitStruct.Pin = set_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(set_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ConnectTaskStart */
/**
* @brief Function implementing the ConnectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ConnectTaskStart */
void ConnectTaskStart(void const * argument)
{
  /* USER CODE BEGIN ConnectTaskStart */
	uint8_t recData;

	uint8_t answerIndex;
  /* Infinite loop */
  for(;;)
  {
		if(xQueueReceive(RecQueueHandle, &recData, portMAX_DELAY) != errQUEUE_EMPTY)
		{
			if(HC12Mode == 0)
			{
				recHDLC(recData);
				if(masterRX.Command != 0)
				{
					uint32_t ID_Sensor = (uint32_t)(masterRX.arg[0] + (masterRX.arg[1]<<8) +  (masterRX.arg[2]<<16) + (masterRX.arg[3]<<24));
					if(ID_Sensor == Param.ID)
					{
						xQueueSend(AnswerQueueHandle, (void *)&masterRX.Command, 100);
					}
					masterRX.Command = 0;
				}
				answerIndex = 0;
			}
			else
			{
				if(recData == 0x0D)
				{
					answerIndex = 0;
					answerAT = 1;
				}

				if(answerIndex < 20)
				{
					if((recData != 0x0A) & (recData != 0x0D) & (answerIndex < 20))
						hcAnswer[answerIndex++] = recData;
				}
				
				
			}
			
		}
   }
  /* USER CODE END ConnectTaskStart */
}

/* USER CODE BEGIN Header_SensorTaskStart */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTaskStart */
void SensorTaskStart(void const * argument)
{
  /* USER CODE BEGIN SensorTaskStart */
	uint8_t LiveCount = 0;
	uint8_t Answer = 0;
	uint8_t CountSend = 0;
	uint8_t CountSendMax = 5;
	uint8_t RecAnswer = 0;
	structSensorHeader gerkon;
	
	// Setup HC-12
	//void initHC12(uint16_t speed, uint8_t mode, uint8_t cannel, uint8_t power);
	osDelay(3000);
	gerkon.countSensor = 0;
	gerkon.sensor = GPIO_PIN_SET;
	gerkon.prevSensor = GPIO_PIN_SET;
	gerkon.SensorPort = sensor_GPIO_Port;
	gerkon.SensorPin = sensor_Pin;
	gerkon.change = 0;
	
	uint8_t resIni = initHC12(SPEED9600,FU3, 6, P8);	
//	if (resIni == 1)
//		while(1);
	//initHC12(9600,3,4,8);
	modeModem();
	for(int index = 0; index < 10; index++)
	{
		HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
		SendPacket433(Param.ADR_DST, Param.ADR_SRC, Param.ID,0x33);
		osDelay(300);  
	}
//	SendPacket433(Param.ID,0x31);
  /* Infinite loop */
  for(;;)
  {
    gerkon = isSensorChange(gerkon);
		
		if(gerkon.change == 1)
		{
			//  send ALARM
			RecAnswer = 0;
			CountSend = 0;
			uint8_t SendCommand = 0x10;
			HAL_GPIO_WritePin(led_GPIO_Port,led_Pin,GPIO_PIN_RESET);
			if(gerkon.sensor == GPIO_PIN_SET)
			{
				SendCommand = 0x12;
				HAL_GPIO_WritePin(led_GPIO_Port,led_Pin,GPIO_PIN_SET);
			}
			
			
			while(RecAnswer == 0)
			{
				SendPacket433(Param.ADR_DST, Param.ADR_SRC, Param.ID,SendCommand);
				if(xQueueReceive(AnswerQueueHandle, &Answer, 200) != errQUEUE_EMPTY)
				{
					// answer return
					RecAnswer = 1;
				}
				if(++CountSend > CountSendMax)
				{
					// no answer
					break;
				}
			}
			
		}
//		
//		if(++LiveCount > 49)
//		{
//			LiveCount = 0;
//			// send Live Packet
//			SendPacket433(ID_Current_Sensor,0x10);
//			if(xQueueReceive(AnswerQueueHandle, &Answer, 300) != errQUEUE_EMPTY)
//			{
//				// answer return
//			}
//			else
//			{
//				// answer no return
//				SendPacket433(ID_Current_Sensor,0x10);
//			}
//			
//		}
//		
//		
		osDelay(10);
  }
  /* USER CODE END SensorTaskStart */
}

/* USER CODE BEGIN Header_PowerTaskStart */
/**
* @brief Function implementing the PowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowerTaskStart */
void PowerTaskStart(void const * argument)
{
  /* USER CODE BEGIN PowerTaskStart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(35);
  }
  /* USER CODE END PowerTaskStart */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
