/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define BUTTON_INFO_Pin GPIO_PIN_6
#define BUTTON_INFO_GPIO_Port GPIOA
#define BUTTON_INFO_EXTI_IRQn EXTI9_5_IRQn
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOB
#define BUTTON_START_Pin GPIO_PIN_8
#define BUTTON_START_GPIO_Port GPIOA
#define BUTTON_START_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_STOP_Pin GPIO_PIN_10
#define BUTTON_STOP_GPIO_Port GPIOA
#define BUTTON_STOP_EXTI_IRQn EXTI15_10_IRQn
#define POWER_KEY_Pin GPIO_PIN_12
#define POWER_KEY_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
