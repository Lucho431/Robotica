/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define encoder_R_Pin GPIO_PIN_0
#define encoder_R_GPIO_Port GPIOA
#define IN_sensorR_Pin GPIO_PIN_4
#define IN_sensorR_GPIO_Port GPIOA
#define IN_sensorL_Pin GPIO_PIN_5
#define IN_sensorL_GPIO_Port GPIOA
#define PWM_L_Pin GPIO_PIN_6
#define PWM_L_GPIO_Port GPIOA
#define PWM_R_Pin GPIO_PIN_7
#define PWM_R_GPIO_Port GPIOA
#define IC_Eco_Pin GPIO_PIN_0
#define IC_Eco_GPIO_Port GPIOB
#define OUT_in2_Pin GPIO_PIN_1
#define OUT_in2_GPIO_Port GPIOB
#define OUT_in3_Pin GPIO_PIN_10
#define OUT_in3_GPIO_Port GPIOB
#define OUT_in4_Pin GPIO_PIN_11
#define OUT_in4_GPIO_Port GPIOB
#define OUT_in1_Pin GPIO_PIN_14
#define OUT_in1_GPIO_Port GPIOB
#define OUT_Trig_Pin GPIO_PIN_15
#define OUT_Trig_GPIO_Port GPIOB
#define encoder_L_Pin GPIO_PIN_12
#define encoder_L_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
