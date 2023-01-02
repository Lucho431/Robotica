/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "math.h"

#include "mpu_9265_lfs.h"
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

/* USER CODE BEGIN PV */
uint16_t set_pwm_L = 50;
uint16_t set_pwm_R = 50;

uint16_t cuenta_pulsosL = 0;
uint16_t cuenta_pulsosR = 0;

uint8_t desbordes;

float magX, magY;
int16_t MAX_magX = -10000, MAX_magY = -10000, MIN_magX = 10000, MIN_magY = 10000;
int16_t media_magX, media_magY;
float direccion_f32;
int16_t direccion_i16;
mpuData_t mpu9265;

int32_t deltaT;

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
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART7_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2); //encoder.
  HAL_TIM_Base_Start(&htim3); //encoder.

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //rueda izquierda.
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //rueda derecha.
//  TIM4->CCR1 = set_pwm_L;
//  TIM4->CCR2 = set_pwm_R;
  TIM4->CCR1 = 0;
  TIM4->CCR2 = 0;

  //avanza
  HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
  HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);

  HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
  HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);


  HAL_TIM_Base_Start_IT(&htim7); //desborda cada 1 s.

  mpu9265_Init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  TIM4->CCR1 = set_pwm_L;
//	  TIM4->CCR2 = set_pwm_R;

	  if (desbordes != 0){

		  deltaT = __HAL_TIM_GET_COUNTER(&htim7);

		  mpu9265_Read_Magnet(&mpu9265);
		  magX = (float) (mpu9265.Magnet_X_RAW + 388.0); //media empirica
		  magY = (float) (mpu9265.Magnet_Y_RAW - 234.0); //media empirica
/*
		  if (magX > MAX_magX) MAX_magX = magX;
		  if (magX < MIN_magX) MIN_magX = magX;
		  if (magY > MAX_magY) MAX_magY = magY;
		  if (magY < MIN_magY) MIN_magY = magY;

		  media_magX = (MAX_magX + MIN_magX) / 2;
		  media_magY = (MAX_magY + MIN_magY) / 2;
*/

		  direccion_f32 = atan2f(magY, magX);
		  direccion_f32 *= (180.0/M_PI);
		  //			direccion_i16 = direccion_f32/180;
//		  direccion_i16 = direccion_f32;
//		  direccion_i16 -= 129;
//		  direccion_i16 = -direccion_i16;
//		  if (magX > 0) direccion_i16 = -direccion_i16;
		  //			if (magX < magY) direccion_i16 = -direccion_i16;
		  //			direccion_f32 *= (180.0/M_PI);
		  //			direccion_i16 = direccion_f32/180;


		  deltaT = __HAL_TIM_GET_COUNTER(&htim7) - deltaT;

//		  txUart[0] = COORD_ANG;
//		  txUart[1] = (uint8_t)(direccion_i16 >> 8);
//		  txUart[2] = (uint8_t)(direccion_i16 & 0xFF);
//		  txUart[3] = '\0';
//		  HAL_UART_Transmit_IT(&huart7, txUart, 4);
		  desbordes = 0;
	  }




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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance==TIM7){
//		cuenta_pulsosL = __HAL_TIM_GET_COUNTER(&htim3);
//		cuenta_pulsosR = __HAL_TIM_GET_COUNTER(&htim2);
//		__HAL_TIM_SET_COUNTER(&htim3, 0);
//		__HAL_TIM_SET_COUNTER(&htim2, 0);

		desbordes++;

	}
}

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
