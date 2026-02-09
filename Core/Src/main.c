/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "MPU6050.h"
#include <stdio.h>
#include <string.h>
#include "IMU.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//过滤器配置（全通）
HAL_StatusTypeDef Init_Filter(){
	CAN_FilterTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.FilterBank=0;//指定需要初始化的过滤器，过滤器总共有13个
	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	CAN_FilterInitStructure.FilterIdHigh=0x0000;
	CAN_FilterInitStructure.FilterIdLow=0x0000;
	CAN_FilterInitStructure.FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.FilterActivation=CAN_FILTER_ENABLE;
	return HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure);
}


//发送函数
HAL_StatusTypeDef MyCAN_Transmit(CAN_TxHeaderTypeDef *TxMessage, uint8_t *Data)
{
	uint32_t pTxMailbox;
	return HAL_CAN_AddTxMessage(&hcan, TxMessage, Data, &pTxMailbox);
}

//接收标志位
uint8_t MyCAN_ReceiveFlag(void)
{
	uint32_t aaa=HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0);
	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) > 0)
	{
		return 1;
	}
	return 0;
}

//接收函数
void MyCAN_Receive(CAN_RxHeaderTypeDef *RxMessage, uint8_t *Data) {
	HAL_CAN_GetRxMessage(&hcan, CAN_FILTER_FIFO0, RxMessage, Data); 
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*要发送的内容*/
uint8_t TxMsgDataArray[2] = {0x11,0x22}; 
CAN_TxHeaderTypeDef TxMsgHeaderArray[] = {
	{ 0x100, 0x00000000, CAN_ID_STD, CAN_RTR_DATA, 1 ,DISABLE}, 
	{ 0x222, 0x00000000, CAN_ID_STD, CAN_RTR_DATA, 1 ,DISABLE}, 
	
};


uint8_t StartFlag = 2;  // 夹爪闭合与打开的启动标志位
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t ID;
//int16_t AX,AY,AZ,GX,GY,GZ;
float roll;
float picth;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	OLED_Init();
//	OLED_Clear();
//	OLED_ShowString(1, 1, "RxFlag:");
//	OLED_ShowString(2, 1, "RxID:");
//	OLED_ShowString(3, 1, "KeyFlag:");
	
	Init_Filter();
	HAL_CAN_Start(&hcan);
	
	MPU6050_Init();
	ID = MPU6050_GetID();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*按下按键1后，夹爪闭合（通过CAN来进行通信）*/
//	  if(HAL_GPIO_ReadPin(GPIOB,KEY_1_Pin) == GPIO_PIN_RESET)
//	  {
//		  MyCAN_Transmit(&TxMsgHeaderArray[0],&TxMsgDataArray[0]);
//		  StartFlag = 1;  
//	  }
//	  /*按下按键2后，夹爪打开*/
//	  if(HAL_GPIO_ReadPin(GPIOB,KEY_2_Pin) == GPIO_PIN_RESET)
//	  {
//		  MyCAN_Transmit(&TxMsgHeaderArray[1],&TxMsgDataArray[1]);
//		  StartFlag = 0;
//	  }
	  
//	  MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
//	  OLED_ShowSignedNum(0,0,AX,5,OLED_8X16);
//	  OLED_ShowSignedNum(0,16,AY,5,OLED_8X16);  
//	  OLED_ShowSignedNum(0,32,AZ,5,OLED_8X16);
//	  OLED_ShowSignedNum(64,0,GX,5,OLED_8X16);
//	  OLED_ShowSignedNum(64,16,GY,5,OLED_8X16);
//	  OLED_ShowSignedNum(64,32,GZ,5,OLED_8X16);
//	  OLED_Update();
	  loop(&roll,&picth);
	  
	  OLED_ShowFloatNum(0,0,roll,2,3,OLED_8X16);
	  OLED_ShowFloatNum(0,16,picth,2,3,OLED_8X16);
	  OLED_Update();
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
