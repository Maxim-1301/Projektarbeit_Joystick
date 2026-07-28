/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f3xx_hal.h"

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
#define MotorA_DE_Pin GPIO_PIN_6
#define MotorA_DE_GPIO_Port GPIOA
#define MotorA_NRE_Pin GPIO_PIN_7
#define MotorA_NRE_GPIO_Port GPIOA
#define MotorA_UART_TX_Pin GPIO_PIN_4
#define MotorA_UART_TX_GPIO_Port GPIOC
#define MotorA_UART_RX_Pin GPIO_PIN_5
#define MotorA_UART_RX_GPIO_Port GPIOC
#define MotorB_DE_Pin GPIO_PIN_1
#define MotorB_DE_GPIO_Port GPIOB
#define MotorB_NRE_Pin GPIO_PIN_2
#define MotorB_NRE_GPIO_Port GPIOB
#define MotorB_UART_TX_Pin GPIO_PIN_10
#define MotorB_UART_TX_GPIO_Port GPIOB
#define MotorB_UART_RX_Pin GPIO_PIN_11
#define MotorB_UART_RX_GPIO_Port GPIOB
#define Joystick_ADC_X_Pin GPIO_PIN_12
#define Joystick_ADC_X_GPIO_Port GPIOB
#define Joystick_Taster1_Pin GPIO_PIN_13
#define Joystick_Taster1_GPIO_Port GPIOB
#define Joystick_ADC_Y_Pin GPIO_PIN_14
#define Joystick_ADC_Y_GPIO_Port GPIOB
#define Joystick_ADC_Z_Pin GPIO_PIN_15
#define Joystick_ADC_Z_GPIO_Port GPIOB
#define Joystick_Taster2_Pin GPIO_PIN_6
#define Joystick_Taster2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
