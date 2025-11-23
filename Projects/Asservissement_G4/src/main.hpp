/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.hpp
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
#include "stm32g4xx_hal.h"

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

// Déclaration du callback UART (doit être en C)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_2A_Pin GPIO_PIN_0
#define ENC_2A_GPIO_Port GPIOA
#define ENC_2B_Pin GPIO_PIN_1
#define ENC_2B_GPIO_Port GPIOA
#define ENC_3B_Pin GPIO_PIN_4
#define ENC_3B_GPIO_Port GPIOA
#define ENC_3A_Pin GPIO_PIN_6
#define ENC_3A_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_7
#define PWM_3_GPIO_Port GPIOA
#define DIR_1_Pin GPIO_PIN_0
#define DIR_1_GPIO_Port GPIOB
#define ENC_1A_Pin GPIO_PIN_8
#define ENC_1A_GPIO_Port GPIOA
#define ENC_1B_Pin GPIO_PIN_9
#define ENC_1B_GPIO_Port GPIOA
#define DIR_2_Pin GPIO_PIN_4
#define DIR_2_GPIO_Port GPIOB
#define DIR_3_Pin GPIO_PIN_5
#define DIR_3_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_6
#define PWM_2_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_7
#define PWM_1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
