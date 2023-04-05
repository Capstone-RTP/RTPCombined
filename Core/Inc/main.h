/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define thetaDir_Pin GPIO_PIN_5
#define thetaDir_GPIO_Port GPIOA
#define yDir_Pin GPIO_PIN_6
#define yDir_GPIO_Port GPIOA
#define rDir_Pin GPIO_PIN_7
#define rDir_GPIO_Port GPIOA
#define thLim_Pin GPIO_PIN_12
#define thLim_GPIO_Port GPIOE
#define thLim_EXTI_IRQn EXTI15_10_IRQn
#define yLim_Pin GPIO_PIN_14
#define yLim_GPIO_Port GPIOE
#define yLim_EXTI_IRQn EXTI15_10_IRQn
#define rLim_Pin GPIO_PIN_15
#define rLim_GPIO_Port GPIOE
#define rLim_EXTI_IRQn EXTI15_10_IRQn
#define state3LED_Pin GPIO_PIN_14
#define state3LED_GPIO_Port GPIOB
#define loadCLK_Pin GPIO_PIN_8
#define loadCLK_GPIO_Port GPIOD
#define loadDATA_Pin GPIO_PIN_9
#define loadDATA_GPIO_Port GPIOD
#define state1LED_Pin GPIO_PIN_7
#define state1LED_GPIO_Port GPIOC
#define state2LED_Pin GPIO_PIN_7
#define state2LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
