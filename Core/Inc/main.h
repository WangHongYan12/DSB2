/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define WIND_PWM_Pin GPIO_PIN_1
#define WIND_PWM_GPIO_Port GPIOA
#define MOTORTX_Pin GPIO_PIN_2
#define MOTORTX_GPIO_Port GPIOA
#define MOTORRX_Pin GPIO_PIN_3
#define MOTORRX_GPIO_Port GPIOA
#define K1_Pin GPIO_PIN_4
#define K1_GPIO_Port GPIOA
#define K1_EXTI_IRQn EXTI4_IRQn
#define K2_Pin GPIO_PIN_5
#define K2_GPIO_Port GPIOA
#define K2_EXTI_IRQn EXTI9_5_IRQn
#define K3_Pin GPIO_PIN_6
#define K3_GPIO_Port GPIOA
#define K3_EXTI_IRQn EXTI9_5_IRQn
#define K4_Pin GPIO_PIN_7
#define K4_GPIO_Port GPIOA
#define K4_EXTI_IRQn EXTI9_5_IRQn
#define VISION_TX_Pin GPIO_PIN_10
#define VISION_TX_GPIO_Port GPIOB
#define VISION_RX_Pin GPIO_PIN_11
#define VISION_RX_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_8
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_11
#define SERVO2_GPIO_Port GPIOA
#define WIND_IN1_Pin GPIO_PIN_15
#define WIND_IN1_GPIO_Port GPIOA
#define IMU_TX_Pin GPIO_PIN_10
#define IMU_TX_GPIO_Port GPIOC
#define IMU_RX_Pin GPIO_PIN_11
#define IMU_RX_GPIO_Port GPIOC
#define SERVO3_Pin GPIO_PIN_6
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_7
#define SERVO4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
