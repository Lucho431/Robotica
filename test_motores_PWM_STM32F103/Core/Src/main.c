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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	QUIETO,
	AVANZANDO,
	RETROCEDIENDO,
	ROTANDO_IZQ,
	ROTANDO_DER,
	PIVOTE_IZQ_AVAN,
	PIVOTE_DER_AVAN,
	PIVOTE_IZQ_RETR,
	PIVOTE_DER_RETR,
}T_MOV;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//variables de control
T_MOV status_movimiento = QUIETO;
T_MOV last_movimiento = QUIETO;
uint8_t sensores_dist = 0;

//variables del HC-SR04
uint16_t desbordeTIM1 = 0;
uint32_t ic1 = 0, ic2 = 0; //capturas de los flancos
uint16_t cuentasDesbordes = 0;
uint32_t cuentaPulsos = 0;
uint8_t flagEco;
uint8_t statusBurst = 0;
uint8_t delayTrig = 0;
uint16_t distancia = 400;

//variables

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
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
  HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);

  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (statusBurst){
	  	  case 0:
	  		  HAL_GPIO_WritePin(OUT_Trig_GPIO_Port, OUT_Trig_Pin, 1);

	  		  delayTrig = 10 + (uint8_t)__HAL_TIM_GET_COUNTER (&htim1);
	  		  if (delayTrig > 32){
	  			  delayTrig -= 32;
	  			  while (__HAL_TIM_GET_COUNTER (&htim1) > 32);
	  		  }
	  		  while (__HAL_TIM_GET_COUNTER (&htim1) < delayTrig);

	  		  HAL_GPIO_WritePin(OUT_Trig_GPIO_Port, OUT_Trig_Pin, 0);

	  		  statusBurst = 1;
		  break;
	  	  case 1:
	  		  if (flagEco != 0){
	  			  flagEco = 0;
	  			  statusBurst = 2;
	  		  }
		  break;
	  	  case 2:
	  		  if (cuentasDesbordes != 0){
				  cuentaPulsos = 42 * cuentasDesbordes + ic2 - ic1;
			  }

			  if (cuentaPulsos < 25000){
				  distancia = cuentaPulsos * 34 / 2000;
			  }

			  statusBurst = 0;
		  break;

	  } //fin switch (statusBurst)


	  //TIM2->CCR1;

	  sensores_dist = (HAL_GPIO_ReadPin(IN_sensorL_GPIO_Port, IN_sensorL_Pin)) << 0b01;
	  sensores_dist += HAL_GPIO_ReadPin(IN_sensorR_GPIO_Port, IN_sensorR_Pin);

	  switch (sensores_dist) {
		case 0b0:
			status_movimiento = ROTANDO_DER;
		break;
		case 0b10:
			status_movimiento = ROTANDO_DER;
		break;
		case 0b11:
			status_movimiento = AVANZANDO;
		break;
		case 0b01:
			status_movimiento = ROTANDO_IZQ;
		default:
		break;
	} //fin switch sensopres_dist


	  switch (status_movimiento) {
		case QUIETO:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in3_Pin, 0);
		break;
		case AVANZANDO:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in3_Pin, 0);
		break;
		case ROTANDO_IZQ:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in2_Pin, 1);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in3_Pin, 0);
		break;
		case ROTANDO_DER:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in3_Pin, 1);
		break;
		case RETROCEDIENDO:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in2_Pin, 1);
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in3_Pin, 1);
		break;
		case PIVOTE_IZQ_AVAN:

		break;
		case PIVOTE_DER_AVAN:

		default:
		break;
	} //fin switch status_movimiento



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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance==TIM1){
		desbordeTIM1++;
	}

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		//HAL_TIM_ReadCapturedValue(htim, HAL_TIM_ACTIVE_CHANNEL_3);
		ic1 = htim->Instance->CCR3;
		desbordeTIM1 = 0;
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		ic2 = htim->Instance->CCR4;
		cuentasDesbordes = desbordeTIM1;
		flagEco = 1;
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
