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
#include "comandosUart.h"
#include "mpu_9265_lfs.h"
#include "stdlib.h"
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
#define DIAMETRO_RUEDAS 	65  //en mm
#define RANURAS_ENCODER		20  //vuelta completa
#define AVANCE_X_PULSO		10  //en mm
#define DIAMETRO_ACRILICO	130 //en mm
#define ANCHO_TOTAL			156 //156,6 mm
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRIG_SR04 ( __HAL_TIM_SET_COUNTER(&htim5, 0xFFFFFFF5) )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//comunicacion//
uint8_t rxUart[4];
uint8_t flag_cmd = 0;
uint8_t txUart[4];
uint8_t esp01Presente = 0;

//modo de funcionamiento//
T_MODO modoFuncionamiento= MANUAL;

//prueba//
uint8_t read_button = 1;
uint8_t last_button = 1;

//movimiento//
T_MOV status_movimiento = QUIETO;
T_MOV last_movimiento = QUIETO;

uint16_t avance_cant = 0;
uint16_t retroceso_cant = 0;
uint16_t giroIzq_cant = 0;
uint16_t giroDer_cant = 0;


//sensores//
uint8_t SI, SF, SD;
uint8_t sensores_dist = 0;
mpuData_t mpu9265;

//ticks//
uint8_t desbordeTIM7 = 0; //desborda cada 10 ms.

//SR-04//
uint32_t ic1 = 0;
uint32_t ic2 = 0;
uint8_t flancoEco = 0; //cuando cuenta 2, detectó ambos flancos. Valores mayores, son errores.
int32_t cuentaPulsos = 0;
uint16_t distanciaSR04 = 0; //distancia en cm.

//encoders//
uint8_t flag_encoders = 0;
int16_t encoderL;
int16_t encoderR;
int16_t acum_encoderL = 0;
int16_t acum_encoderR = 0;

//velocidades//
uint8_t velL = 5; //en ranuras cada 210 ms
uint8_t velR = 5; //en ranuras cada 210 ms
uint8_t velLFinal = 5; //en ranuras cada 210 ms
uint8_t velRFinal = 5; //en ranuras cada 210 ms
uint8_t Kp = 1;
int8_t  Vr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SR_04 (void);
void sensores (void);
void movimientoLibre (void);
void modo_funcionamiento (void);
void encoders (void);
void PWM_motores (void);
void velocidades (int8_t, int8_t);
void aceleracion (void);
void check_rxUart (void);

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

  HAL_TIM_Base_Start_IT(&htim7); //desborda cada 10 ms.

  HAL_TIM_Base_Start(&htim2); //encoder R.
  HAL_TIM_Base_Start(&htim3); //encoder L.

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //rueda izquierda.
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //rueda derecha.

  HAL_TIM_Base_Start(&htim5); //control del SR-04.
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //para el pulso del trigger.
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3); //para capturar el eco (flanco ascendente).
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); //para capturar el eco (flanco descendente).

  mpu9265_Init(&hi2c1);


  HAL_UART_Receive(&huart7, rxUart, 4, 500);

  if (rxUart[0] == HOLA){

	  if (!rxUart[3]){
		  txUart[0] = CMD_ERROR;
		  txUart[3] = '\0';
		  HAL_UART_Transmit_IT(&huart7, txUart, 4);
	  } else {
		  esp01Presente = 1;
		  txUart[0] = HOLA;
		  txUart[3] = '\0';
		  HAL_UART_Transmit_IT(&huart7, txUart, 4);
	  }

  }

  HAL_UART_Receive_IT(&huart7, rxUart, 4);

  if (!esp01Presente) {
	  modoFuncionamiento = AUTOMATICO;
  }else{
	  modoFuncionamiento = MANUAL;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  read_button = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);

	  if (!read_button){
		  if (last_button != 0){
			  TRIG_SR04;
		  }
	  }
	  last_button = read_button;
	  */

	  if (flag_cmd != 0){
		  check_rxUart();
		  flag_cmd = 0;
	  }

	  SR_04();
	  sensores();
	  modo_funcionamiento();

	  if (desbordeTIM7 > 21){
		  flag_encoders = 1;
		  desbordeTIM7 = 0;

		  TRIG_SR04;
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
		desbordeTIM7++;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		ic1 = htim->Instance->CCR3;
		flancoEco++;
	}

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		ic2 = htim->Instance->CCR4;
		flancoEco++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	flag_cmd = 1;
}


void SR_04 (void){

	switch (flancoEco){
		case 0:
		case 1:
			return;
		case 2:
			cuentaPulsos = (int32_t)(ic2 - ic1);
			if (cuentaPulsos < 23310){
				distanciaSR04 = cuentaPulsos * 34 / 2000;
			}else{
				distanciaSR04 = 400;
			}
			flancoEco = 0;
		break;
		default:
			flancoEco = 0;
		break;
	} //end switch flancoEco

} //end SR_04()

void sensores (void){
	//sensores_dist = SI << 2 | SF << 1 | SD (logica negativa)
	SI = (HAL_GPIO_ReadPin(IN_sensorL_GPIO_Port, IN_sensorL_Pin)) ;
	SD = HAL_GPIO_ReadPin(IN_sensorR_GPIO_Port, IN_sensorR_Pin);
	if (distanciaSR04 < 25) SF = 0; else SF = 1;

	sensores_dist = SI << 2 | SF << 1 | SD;
} //end sensores()

void PWM_motores (void){

	TIM4->CCR1 += velL - encoderL;
	if (TIM4->CCR1 < 62) TIM4->CCR1 = 62;
	if (TIM4->CCR1 > 82) TIM4->CCR1 = 82;

	TIM4->CCR2 += velR - encoderR;
	if (TIM4->CCR2 < 62) TIM4->CCR2 = 62;
	if (TIM4->CCR2 > 82) TIM4->CCR2 = 82;

} //end PWM_motores()

void aceleracion (void){
	if (velL < velLFinal) velL++;

} //end aceleracion()

void velocidades (int8_t vl, int8_t vr){

	if (vl < 0){
		HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
		HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
	} else if (vl > 0){
		HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
		HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
	} else {
		HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
		HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
	}

	if (vr < 0){
		HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
		HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);
	} else if (vr > 0){
		HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);
		HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);
	} else {
		HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
		HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);
	}

	velLFinal = abs(vl);
	velRFinal = abs(vr);

} //end velocidades()

void movimientoLibre (void){

	if (avance_cant != 0) avance_cant = 0;
	if (retroceso_cant != 0) retroceso_cant = 0;
	if (giroIzq_cant != 0) giroIzq_cant = 0;
	if (giroDer_cant != 0) giroDer_cant = 0;

	switch (status_movimiento) {
		case QUIETO:

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 0;
			velR = 0;

			status_movimiento = AVANZANDO;
		break;
		case AVANZANDO:

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 5;
			velR = 5;

			switch (sensores_dist) {
				case 0b110:
					//agregado para prueba
					//status_movimiento = PIVOTE_IZQ_AVAN;
					status_movimiento = ROTANDO_IZQ;
				break;
				case 0b101:
				case 0b100:
				case 0b000:
					status_movimiento = ROTANDO_IZQ;
				break;
				case 0b011:
					//agregado para prueba
					//status_movimiento = PIVOTE_DER_AVAN;
					status_movimiento = ROTANDO_DER;
				break;
				case 0b001:
					status_movimiento = ROTANDO_DER;
				break;
				default:
				break;
			} //end switch sensores_dist

		break;
		case ROTANDO_IZQ:

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 5;
			velR = 5;

			switch (sensores_dist){
				case 0b111:
					status_movimiento = AVANZANDO;
					break;
				case 0b011:
					status_movimiento = ROTANDO_DER;
				default:
					break;
			} //end switch sensores_dist

		break;
		case ROTANDO_DER:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			velL = 5;
			velR = 5;

			switch (sensores_dist){
				case 0b111:
					status_movimiento = AVANZANDO;
					break;
				case 0b110:
					status_movimiento = ROTANDO_IZQ;
				default:
					break;
			} //end switch sensores_dist

		break;
		case RETROCEDIENDO:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			velL = 5;
			velR = 5;

			status_movimiento = AVANZANDO;
			break;
		case PIVOTE_IZQ_AVAN:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 0;
			velR = 5;

			switch (sensores_dist){
				case 0b111:
					status_movimiento = AVANZANDO;
				break;
				case 0b011:
					status_movimiento = ROTANDO_DER;
				break;
				case 0b100:
					status_movimiento = ROTANDO_IZQ;
				default:
				break;
			} //end switch sensores_dist

		break;
		case PIVOTE_DER_AVAN:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 5;
			velR = 0;

			switch (sensores_dist){
				case 0b111:
					status_movimiento = AVANZANDO;
				break;
				case 0b110:
					status_movimiento = ROTANDO_IZQ;
				break;
				case 0b001:
					status_movimiento = ROTANDO_DER;
				default:
				break;
			} //end switch sensores_dist

		default:
		break;

	} //fin switch status_movimiento

} //fin movimientoLibre()

void movimientoRC (void){

	if (!avance_cant && !retroceso_cant && !giroIzq_cant && !giroDer_cant ){
		status_movimiento = QUIETO;
	}

	switch (status_movimiento) {
		case QUIETO:

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			if (avance_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = AVANZANDO;
				break;
			}

			if (retroceso_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = RETROCEDIENDO;
				break;
			}

			if (giroIzq_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_IZQ;
				break;
			}

			if (giroDer_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_DER;
				break;
			}

		break;
		case AVANZANDO:

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			encoders();

			if (avance_cant > ((acum_encoderL + acum_encoderR) >> 1) ) break;

			avance_cant = 0;

			status_movimiento = QUIETO;

			if (retroceso_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = RETROCEDIENDO;
				break;
			}

			if (giroIzq_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_IZQ;
				break;
			}

			if (giroDer_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_DER;
				break;
			}

		break;
		case ROTANDO_IZQ:

			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			encoders();

			if (giroIzq_cant > ((acum_encoderL + acum_encoderR) >> 1) ) break;

			giroIzq_cant = 0;

			status_movimiento = QUIETO;

			if (avance_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = AVANZANDO;
				break;
			}

			if (retroceso_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = RETROCEDIENDO;
				break;
			}

			if (giroDer_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_DER;
				break;
			}

		break;
		case ROTANDO_DER:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			encoders();

			if (giroDer_cant > ((acum_encoderL + acum_encoderR) >> 1) ) break;

			giroDer_cant = 0;

			status_movimiento = QUIETO;

			if (avance_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = AVANZANDO;
				break;
			}

			if (retroceso_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = RETROCEDIENDO;
				break;
			}

			if (giroIzq_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_IZQ;
				break;
			}

		break;
		case RETROCEDIENDO:
			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);

			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			encoders();

			if (retroceso_cant > ((acum_encoderL + acum_encoderR) >> 1) ) break;

			retroceso_cant = 0;

			status_movimiento = QUIETO;

			if (avance_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = AVANZANDO;
				break;
			}

			if (giroIzq_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_IZQ;
				break;
			}

			if (giroDer_cant != 0){
				acum_encoderL = 0;
				acum_encoderR = 0;
				status_movimiento = ROTANDO_DER;
				break;
			}

		break;
		case PIVOTE_IZQ_AVAN:

		break;
		case PIVOTE_DER_AVAN:

		default:
		break;

	} //fin switch status_movimiento

} //fin movimientoRC()

void encoders (void){

	if (!flag_encoders) return;

	encoderL = __HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	encoderR = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

//	if (encoderL > velL){
//		if (TIM4->CCR1 > 45)
//			TIM4->CCR1--;
//	}else if (encoderL < velL){
//		if (TIM4->CCR1 < 85)
//			TIM4->CCR1++;
//	}

	TIM4->CCR1 += velL - encoderL;

//	if (encoderR > velR){
//		if (TIM4->CCR2 > 45)
//			TIM4->CCR2--;
//	}else if (encoderR < velR){
//		if (TIM4->CCR2 < 85)
//			TIM4->CCR2++;
//	}

	TIM4->CCR2 += velR - encoderR;

	acum_encoderL += encoderL;
	acum_encoderR += encoderR;

	flag_encoders = 0;

} //fin encoders()

void test_respuesta (void){
	switch(rxUart[0]){
		case AVANCE:
//			sprintf((char *)txUart, "avan");
		break;
		case RETROCEDE:
//			sprintf((char *)txUart, "RETR");
		break;
		case GIRO_IZQ:
//			sprintf((char *)txUart, "IZQU");
		break;
		case GIRO_DER:
//			sprintf((char *)txUart, "DERE");
		break;
	} //end switch rxUart

	HAL_UART_Transmit(&huart7, txUart, 4, 20);
	HAL_UART_Receive_IT(&huart7, rxUart, 4);
}

void check_rxUart (void){

	if (rxUart[3] != 0){
		txUart[0] = CMD_ERROR;
		txUart[3] = '\0';
		HAL_UART_Transmit_IT(&huart7, txUart, 4);
		HAL_UART_Receive_IT(&huart7, rxUart, 4);
		return;
	}

	switch (rxUart[0]) {
		case HOLA:
			esp01Presente = 1;
			txUart[0] = HOLA;
			txUart[3] = '\0';
			HAL_UART_Transmit_IT(&huart7, txUart, 4);

		break;
		case MODO:

			switch (rxUart[1]) {
				case AUTOMATICO:
					modoFuncionamiento = AUTOMATICO;
					flag_encoders = 0;
					txUart[0] = OK_;
					txUart[3] = '\0';
					HAL_UART_Transmit_IT(&huart7, txUart, 4);
				break;
				case MANUAL:
					status_movimiento = QUIETO;
					modoFuncionamiento = MANUAL;
					txUart[0] = OK_;
					txUart[3] = '\0';
					HAL_UART_Transmit_IT(&huart7, txUart, 4);
				break;
				default:
					txUart[0] = CMD_ERROR;
					txUart[3] = '\0';
					HAL_UART_Transmit_IT(&huart7, txUart, 4);
			} //end switch rxUart[1]

		break;
		case AVANCE:
			avance_cant += (uint16_t) (rxUart[2] + (rxUart[1] << 8));

			txUart[0] = OK_;
			txUart[3] = '\0';
			HAL_UART_Transmit_IT(&huart7, txUart, 4);
		break;
		case RETROCEDE:
			retroceso_cant += (uint16_t) (rxUart[2] + (rxUart[1] << 8));

			txUart[0] = OK_;
			txUart[3] = '\0';
			HAL_UART_Transmit_IT(&huart7, txUart, 4);
			//sprintf(txUart, "RETR");
		break;
		case GIRO_IZQ:
			giroIzq_cant += (uint16_t) (rxUart[2] + (rxUart[1] << 8));

			txUart[0] = OK_;
			txUart[3] = '\0';
			HAL_UART_Transmit_IT(&huart7, txUart, 4);
			//sprintf(txUart, "IZQU");
		break;
		case GIRO_DER:
			giroDer_cant += (uint16_t) (rxUart[2] + (rxUart[1] << 8));

			txUart[0] = OK_;
			txUart[3] = '\0';
			HAL_UART_Transmit_IT(&huart7, txUart, 4);
			//sprintf(txUart, "DERE");
		break;
		case VEL_AVANCE:
			mpu9265_Read_Accel(&mpu9265);

			txUart[0] = VEL_AVANCE;
			txUart[1] = (uint8_t)(mpu9265.Accel_X_RAW >> 8);
			txUart[2] = (uint8_t)(mpu9265.Accel_X_RAW & 0xFF);
			txUart[3] = '\0';
			HAL_UART_Transmit_IT(&huart7, txUart, 4);
		break;


	} //end switch rxUart[0]

	HAL_UART_Receive_IT(&huart7, rxUart, 4);

} //end check_rxUart ()

void modo_funcionamiento (void){

	switch (modoFuncionamiento) {
		case AUTOMATICO:
			movimientoLibre();
			encoders();
		break;
		case MANUAL:
			movimientoRC();
		break;
		default:
		break;
	} //end switch modoFuncionamiento

} //end modo_funcionamiento ()

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
