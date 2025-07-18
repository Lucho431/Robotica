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
#include "comunicacionUART.h"
#include "mpu_9265_lfs.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
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
#define AVANCE_X_PULSO_F	10.21 //float en mm
#define DIAMETRO_ACRILICO	130 //en mm
#define ANCHO_TOTAL			156 //156,6 mm
#define DISTANCIA_RUEDAS	125.0 //float en mm
#define RADIANES_X_PULSO	0.081681408993334 //float en radianes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRIG_SR04 ( __HAL_TIM_SET_COUNTER(&htim5, 0xFFFFFFF5) )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//variables de orientacion//
volatile uint8_t estatusOrientando = 0;
volatile uint16_t ticks_orientando = 0xFFFF;

//variables de mov_puntoAPunto//
volatile uint8_t estatusPuntoAPunto = 0;
volatile uint16_t ticks_PuntoAPunto = 0xFFFF;
uint8_t flag_dest = 0; //indica si hay un destino establecido.
float anguloDest_rad_f32; //angulo a la coordenanda de destino desde la posicion actual.
float anguloDest_grad_f32;
int16_t anguloDest_grad_i16;
int16_t deltaAng_dest;
int16_t deltaX_dest;
int16_t deltaY_dest;

//comunicacion//
uint8_t rxUart[8];
uint8_t flag_cmd = 0;
uint8_t txUart[8];
uint8_t esp01Presente = 0;

//modo de funcionamiento//
T_MODO modoFuncionamiento= MANUAL;

//prueba//
uint8_t read_button = 1;
uint8_t last_button = 1;
uint32_t last_cuentaTIM7=0;
uint32_t duracionWhile1 = 0; // en 10 * ns
uint8_t flag_quieto = 0;

//movimiento//
T_MOV status_movimiento = QUIETO;
T_MOV last_movimiento = QUIETO;
int16_t distL; //distancia relativa de la rueda izquierda en ranuras cada 210 ms.
int16_t distR; //distancia relativa de la rueda derecha en ranuras cada 210 ms.
int16_t distC = 0; //distancia relativa del centro en ranuras cada 210 ms.


//mpu9265//
float magX, magY;

float magX_min = 1000, magX_max = -1000;
float magY_min = 1000, magY_max = -1000;
float magX_media = 1, magY_media = 1;
float direccionMag_rad_f32;
float direccionMag_grad_f32;
int16_t direccionMag_grad_i16;

float gyroZ, modGyroZ;
float direccionGiro_rad_f32;
float direccionGiro_grad_f32;
int16_t direccionGiro_grad_i16;

float coef_gyro = 0.0; //0.98;
float coef_mag = 1.0; //0.02;

float direccion_f32;
float direccion_rad;
float direccion_grad;
int16_t direccion_i16;
int16_t direccion_home = 0;
int16_t direccion_dest;

//desplazamientos manuales//
uint16_t avance_cant = 0;
uint16_t retroceso_cant = 0;
uint16_t giroIzq_cant = 0;
uint16_t giroDer_cant = 0;

//coordenadas//
uint8_t pos_x;
uint8_t pos_y;
uint8_t pos_ang;
float posX_f32 = 0;
float posY_f32 = 0;
int16_t posX_i16;
int16_t posY_i16;
int16_t posX_home = 0;
int16_t posY_home = 0;
int16_t posX_dest;
int16_t posY_dest;

//sensores//
uint8_t SI, SF, SD;
uint8_t sensores_dist = 0;
mpuData_t mpu9265;

//ticks//
uint8_t desbordeTIM7 = 0; //desborda cada 10 ms.
uint8_t periodo_Encoder = 7;
uint8_t periodo_SR04 = 0;
uint8_t periodo_pos = 14; //offset para operar intercalado con el periodo encoder
uint8_t flag_checkUart = 0;

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
int16_t encoderL_delta;
int16_t encoderR_delta;
int16_t acum_encoderL = 0;
int16_t acum_encoderR = 0;
int16_t encoderL_memPositivo = 0;
int16_t encoderR_memPositivo = 0;
int16_t encoderL_memNegativo = 0;
int16_t encoderR_memNegativo = 0;

//velocidades//
int8_t velL = 5; //en ranuras cada 210 ms
int8_t velR = 5; //en ranuras cada 210 ms
uint8_t velLFinal = 5; //en ranuras cada 210 ms
uint8_t velRFinal = 5; //en ranuras cada 210 ms




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SR_04 (void);
void sensores (void);
void movimientoLibre (void);
void modo_funcionamiento (void);
void velocidades (int8_t, int8_t);
void encoders (void);
void posicionamiento (void);
void PWM_motores (void);
void aceleracion (void);

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

  //flag_quieto = 1; //comentar para que se mueva el robot

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

  init_controlRxTx (&huart7);
  HAL_Delay(5); //mas de 1ms para omitir la basura del arranque del ESP01
  HAL_UART_Receive_IT(&huart7, rxUart, 8);

  if (!esp01Presente) {
	  modoFuncionamiento = AUTOMATICO;
  }else{
	  modoFuncionamiento = MANUAL;
  }

  //para pruebas (comentar cuando no se requiera):
  modoFuncionamiento = CALIBRA_MAG;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (flag_cmd != 0){
		  controlRxTxUART(rxUart);
		  flag_cmd = 0;
	  }

	  SR_04();
	  sensores();
	  modo_funcionamiento();
	  encoders();
	  velocidades(velL, velR);


	  if (desbordeTIM7 != 0){ //cada 10 ms
		  periodo_Encoder += desbordeTIM7;
		  periodo_SR04 += desbordeTIM7;
		  periodo_pos += desbordeTIM7;
		  desbordeTIM7 = 0;

		  if (periodo_Encoder > 21){ // en 10 * ms
			  flag_encoders = 1;
			  periodo_Encoder = 0;
		  }
		  if (periodo_SR04 > 21){
			  TRIG_SR04;
			  periodo_SR04 = 0;
		  }
		  if (periodo_pos > 21){
			  posicionamiento();
			  periodo_pos = 0;
		  }

		  //para la funcion orientando (comentar cuando no se requiera):
		  if (ticks_orientando != 0) ticks_orientando--;
		  if (ticks_PuntoAPunto != 0) ticks_PuntoAPunto--;

		  //valida que la uart no tenga el buffer a medio llenar (trama desfasada)
		  if (huart7.RxXferCount != 8){
			  if (!flag_checkUart){
				  flag_checkUart = 1;
			  }else{
				  HAL_UART_AbortReceive_IT(&huart7);
				  flag_cmd = 0;
				  HAL_UART_Receive_IT(&huart7, rxUart, 8);
			  }
		  }else{
			  flag_checkUart = 0;
		  } //end if huart7.RxXferCount

	  } //fin if desbordeTIM7

	  //cuenta duracion del bucle
	  duracionWhile1 = TIM7->CNT - last_cuentaTIM7;
	  last_cuentaTIM7 = TIM7->CNT;


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

void movimientoLibre (void){

	if (avance_cant != 0) avance_cant = 0;
	if (retroceso_cant != 0) retroceso_cant = 0;
	if (giroIzq_cant != 0) giroIzq_cant = 0;
	if (giroDer_cant != 0) giroDer_cant = 0;

	switch (status_movimiento) {
		case QUIETO:

//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 0;
			velR = 0;
			//periodo_Encoder = 0;

			status_movimiento = AVANZANDO;
		break;
		case AVANZANDO:

//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 5;
			velR = 5;

			switch (sensores_dist) {
				case 0b110:
					//agregado para prueba
					//status_movimiento = PIVOTE_IZQ_AVAN;
					status_movimiento = ROTANDO_IZQ;

					encoderL_memPositivo += __HAL_TIM_GET_COUNTER(&htim3);
					__HAL_TIM_SET_COUNTER(&htim3, 0);

				break;
				case 0b101:
				case 0b100:
				case 0b000:
					status_movimiento = ROTANDO_IZQ;

					encoderL_memPositivo += __HAL_TIM_GET_COUNTER(&htim3);
					__HAL_TIM_SET_COUNTER(&htim3, 0);
				break;
				case 0b011:
					//agregado para prueba
					//status_movimiento = PIVOTE_DER_AVAN;
					status_movimiento = ROTANDO_DER;

					encoderR_memPositivo += __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2, 0);

				break;
				case 0b001:
					status_movimiento = ROTANDO_DER;

					encoderR_memPositivo += __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2, 0);

				break;
				default:
				break;
			} //end switch sensores_dist

		break;
		case ROTANDO_IZQ:

//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = -5;
			velR = 5;

			switch (sensores_dist){
				case 0b111:
					status_movimiento = AVANZANDO;

					encoderL_memNegativo += __HAL_TIM_GET_COUNTER(&htim3);
					__HAL_TIM_SET_COUNTER(&htim3, 0);

					break;
				case 0b011:
					status_movimiento = ROTANDO_DER;

					encoderL_memNegativo += __HAL_TIM_GET_COUNTER(&htim3);
					__HAL_TIM_SET_COUNTER(&htim3, 0);
					encoderR_memPositivo += __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2, 0);

				default:
					break;
			} //end switch sensores_dist

		break;
		case ROTANDO_DER:
//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			velL = 5;
			velR = -5;

			switch (sensores_dist){
				case 0b111:
					status_movimiento = AVANZANDO;

					encoderR_memNegativo += __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2, 0);

					break;
				case 0b110:
					status_movimiento = ROTANDO_IZQ;

					encoderL_memPositivo += __HAL_TIM_GET_COUNTER(&htim3);
					__HAL_TIM_SET_COUNTER(&htim3, 0);
					encoderR_memNegativo += __HAL_TIM_GET_COUNTER(&htim2);
					__HAL_TIM_SET_COUNTER(&htim2, 0);

				default:
					break;
			} //end switch sensores_dist

		break;
		case RETROCEDIENDO:
//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			velL = -5;
			velR = -5;

			status_movimiento = AVANZANDO;
			break;
		case PIVOTE_IZQ_AVAN:
//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

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
//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

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

//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 0;
			velR = 0;

			//periodo_Encoder = 0;

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

//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = 5;
			velR = 5;

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

//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 1);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);

			velL = -5;
			velR = 5;


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
//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			velL = 5;
			velR = -5;

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
//			HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
//			HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
//
//			HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 1);
//			HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 1);

			velL = -5;
			velR = -5;


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

void orientando (void){

	switch (estatusOrientando) {
		case 0:
			velL = -4;
			velR = +4;
			giroIzq_cant = 170;
			estatusOrientando = 1;
		break;
		case 1:
			if (giroIzq_cant < ((acum_encoderL + acum_encoderR) >> 1) ){
				velL = 0;
				velR = 0;
				giroIzq_cant = 0;
				estatusOrientando = 2;
				ticks_orientando = 200;
			}
		break;
		case 2:
			if (!ticks_orientando){
				estatusOrientando = 3;
			}
		break;
		case 3:
			if (direccion_i16 > 17){
				velL = +4;
				velR = -4;
			}else if (direccion_i16 < -17){
				velL = -4;
				velR = +4;
			}else{
				velL = 0;
				velR = 0;
				estatusOrientando = 4;
			}

		break;
		case 4:
			direccionGiro_rad_f32 = direccionMag_rad_f32;
			modoFuncionamiento = MANUAL;
			estatusOrientando = 0;
			sprintf((char*)txUart, "modMAN");
			send_info(txUart);
		break;
		default:
		break;
	}

} //fin orientando ()


void mov_puntoAPunto (void){

	switch (estatusPuntoAPunto) {
		case 0: //espera destino
			if(flag_dest != 0){
				estatusPuntoAPunto = 1;
				sprintf((char*)txUart, "girand");
				send_info(txUart);
			}
		break;
		case 1: //gira al destino
			deltaX_dest = posX_dest - posX_i16;
			deltaY_dest = posY_dest - posY_i16;
			anguloDest_rad_f32 = atan2f(deltaY_dest, deltaX_dest);
			anguloDest_grad_f32 = anguloDest_rad_f32 * 180.0 / M_PI; //grados en float
			anguloDest_grad_i16 = anguloDest_grad_f32; //grados en int16

			deltaAng_dest = anguloDest_grad_i16 - direccion_i16;
			if (deltaAng_dest > 180) deltaAng_dest -= 360;
			if (deltaAng_dest < -180) deltaAng_dest += 360;

			if (deltaAng_dest > 17){
				velL = -4;
				velR = +4;
				ticks_PuntoAPunto = 20;
			}else if (deltaAng_dest < -17){
				velL = +4;
				velR = -4;
				ticks_PuntoAPunto = 20;
			}else{
				velL = 0;
				velR = 0;
				if (!ticks_PuntoAPunto){
					estatusPuntoAPunto = 2;
					sprintf((char*)txUart, "avanza");
					send_info(txUart);
				}
			}

		break;
		case 2: //avanza al destino
			deltaX_dest = posX_dest - posX_i16;
			deltaY_dest = posY_dest - posY_i16;

			if (  abs(deltaX_dest) < 2 && abs(deltaY_dest) < 2){
				estatusPuntoAPunto = 3;
				velL = 0;
				velR = 0;
				sprintf((char*)txUart, "orient");
				send_info(txUart);
				break;
			}

			anguloDest_rad_f32 = atan2f(deltaY_dest, deltaX_dest);
			anguloDest_grad_f32 = anguloDest_rad_f32 * 180.0 / M_PI; //grados en float
			anguloDest_grad_i16 = anguloDest_grad_f32; //grados en int16
			deltaAng_dest = anguloDest_grad_i16 - direccion_i16;
			if (deltaAng_dest > 180) deltaAng_dest -= 360;
			if (deltaAng_dest < -180) deltaAng_dest += 360;

			if (deltaAng_dest > 17){
				if (deltaAng_dest > 45){
					velL = -4;
					velR = +4;
				}else{
					velL = +4;
					velR = +5;
				}
			}else if (deltaAng_dest < -17){
				if (deltaAng_dest < -45){
					velL = +4;
					velR = -4;
				}else{
					velL = +5;
					velR = +4;
				}
			}

		break;
		case 3: // orienta en la coordenada angulo del destino
			deltaAng_dest = direccion_dest - direccion_i16;
			if (deltaAng_dest > 180) deltaAng_dest -= 360;
			if (deltaAng_dest < -180) deltaAng_dest += 360;

			if (deltaAng_dest > 17){
				velL = -4;
				velR = +4;
			}else if (deltaAng_dest < -17){
				velL = +4;
				velR = -4;
			}else{
				velL = 0;
				velR = 0;
				estatusPuntoAPunto = 4;
			}
		break;
		case 4: // finaliza la secuencia
			flag_dest = 0;
			estatusPuntoAPunto = 0;
			sprintf((char*)txUart, "ENDEST");
			send_info(txUart);

		break;
	}

} //fin mov_puntoAPunto ()






void velocidades (int8_t vl, int8_t vr){

	if (flag_quieto != 0){
		HAL_GPIO_WritePin(OUT_in1_GPIO_Port, OUT_in1_Pin, 0);
		HAL_GPIO_WritePin(OUT_in2_GPIO_Port, OUT_in2_Pin, 0);
		HAL_GPIO_WritePin(OUT_in4_GPIO_Port, OUT_in4_Pin, 0);
		HAL_GPIO_WritePin(OUT_in3_GPIO_Port, OUT_in3_Pin, 0);
		return;
	}

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

void encoders (void){

	if (!flag_encoders) return;

	encoderL = __HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	encoderR = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	encoderL_delta = encoderL + encoderL_memPositivo + encoderL_memNegativo;
	encoderR_delta = encoderR + encoderR_memPositivo + encoderR_memNegativo;

	TIM4->CCR1 += velLFinal - encoderL_delta;
//	if (TIM4->CCR1 < 62) TIM4->CCR1 = 62;
//	if (TIM4->CCR1 > 82) TIM4->CCR1 = 82;

	TIM4->CCR2 += velRFinal - encoderR_delta;
//	if (TIM4->CCR2 < 62) TIM4->CCR2 = 62;
//	if (TIM4->CCR2 > 82) TIM4->CCR2 = 82;

	//avance del centro de masa
	if (velL > 0)
		distL = encoderL + encoderL_memPositivo - encoderL_memNegativo;
	else
		distL = -encoderL + encoderL_memPositivo - encoderL_memNegativo;

	if (velR > 0)
		distR = encoderR + encoderR_memPositivo - encoderR_memNegativo;
	else
		distR = -encoderR + encoderR_memPositivo - encoderR_memNegativo;

	distC += (distL + distR) >> 1;

	//acumula encoders
//	acum_encoderL += encoderL + encoderL_memPositivo - encoderL_memNegativo;
//	acum_encoderR += encoderR + encoderR_memPositivo - encoderR_memNegativo;
	acum_encoderL += encoderL_delta;
	acum_encoderR += encoderR_delta;

	encoderL_memNegativo = 0;
	encoderL_memPositivo = 0;
	encoderR_memNegativo = 0;
	encoderR_memPositivo = 0;
	flag_encoders = 0;

} //fin encoders()

void posicionamiento (void){

	//saco el angulo...


	//por magnetometro
	mpu9265_Read_Magnet(&mpu9265);
	magX = (float) (mpu9265.Magnet_X_RAW); //media empirica +359.0
	magY = (float) (mpu9265.Magnet_Y_RAW); //media empirica -159.0

	if (magX < magX_min) magX_min = magX;
	if (magX > magX_max) magX_max = magX;
	if (magY < magY_min) magY_min = magY;
	if (magY > magY_max) magY_max = magY;
	magX_media = (magX_min + magX_max) / 2.0;
	magY_media = (magY_min + magY_max) / 2.0;

	magX -= magX_media;
	magY -= magY_media;
	magX /= magX_media;
	magY /= magX_media;

	direccionMag_rad_f32 = atan2f(magY, magX); //radianes en float

	posX_f32 += (float) (distC * 0.1 * cosf(direccionMag_rad_f32)); //posicion X en float
	posY_f32 += (float) (distC * 0.1 * sinf(direccionMag_rad_f32)); //posicion Y en float
	distC = 0;
	direccionMag_grad_f32 = direccionMag_rad_f32 * 180.0 / M_PI; //grados en float
	direccionMag_grad_i16 = direccionMag_grad_f32; //grados en int16
	direccion_i16 = direccionMag_grad_i16;
	posX_i16 = posX_f32; //posicion X en int16
	posY_i16 = posY_f32; //posicion Y en int16
/*
	//por giroscopio
	mpu9265_Read_Gyro(&mpu9265);
	gyroZ = (float) (mpu9265.Gyro_Z_RAW / 131.0);
	modGyroZ = abs(gyroZ * 0.21);
	if (modGyroZ > 0.5){
		direccionGiro_grad_f32 += gyroZ * 0.21; //grados en float
	}
	if (direccionGiro_grad_f32 < 0){
		direccionGiro_grad_f32 += 360.0;
	}
	if (direccionGiro_grad_f32 > 360.0){
		direccionGiro_grad_f32 -= 360.0;
	}
	direccionGiro_rad_f32 = (direccionGiro_grad_f32 * M_PI / 180.0); //radianes en float

//	posX_f32 += (float) (distC * cosf(direccionGiro_rad_f32)); //posicion X en float
//	posY_f32 += (float) (distC * sinf(direccionGiro_rad_f32)); //posicion Y en float
//	distC = 0;
//	direccionGiro_grad_i16 = direccionGiro_grad_f32; //grados en int16
//	posX_i16 = posX_f32; //posicion X en int16
//	posY_i16 = posY_f32; //posicion Y en int16
*/
	/*
	//filtro complementario magnetometro y giroscopio
	direccion_rad = coef_gyro * direccionGiro_rad_f32 + coef_mag * direccionMag_rad_f32;

	//coordenadas
	direccion_grad = direccion_rad * 180.0 / M_PI; //grados en float
	direccion_i16 = direccion_grad; //grados en int16
	posX_f32 += (float) (distC * 0.1 * cosf(direccion_rad)); //posicion X en float
	posY_f32 += (float) (distC * 0.1 * sinf(direccion_rad)); //posicion Y en float
	posX_i16 = posX_f32; //posicion X en int16
	posY_i16 = posY_f32; //posicion Y en int16
	*/

/*
	//por odometria
	direccion_rad += (float) ( (distR - distL) * RADIANES_X_PULSO); //radianes en float

	if (direccion_rad < 0){
		direccion_rad += M_TWOPI;
	}
	if (direccion_rad > M_TWOPI){
		direccion_rad -= M_TWOPI;
	}

	posX_f32 += (float) (distC * cosf(direccion_rad)); //posicion X en float
	posY_f32 += (float) (distC * sinf(direccion_rad)); //posicion Y en float
	distC = 0;
	direccion_f32 = direccion_rad * (180.0/M_PI); //grados en float
	direccion_i16 = direccion_f32; //grados en int16
	posX_i16 = posX_f32; //posicion X en int16
	posY_i16 = posY_f32; //posicion Y en int16
*/

} //fin posicionamiento ()

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

void modo_funcionamiento (void){

	switch (modoFuncionamiento) {
		case AUTOMATICO:
			movimientoLibre();
		break;
		case MANUAL:
			movimientoRC();
		break;
		case CALIBRA_MAG:
			orientando();
		case PUNTO_A_PUNTO:
			mov_puntoAPunto();
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
