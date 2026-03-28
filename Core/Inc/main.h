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
#define Buzzer_Pin GPIO_PIN_0
#define Buzzer_GPIO_Port GPIOC
#define Duct_IN2_Pin GPIO_PIN_1
#define Duct_IN2_GPIO_Port GPIOA
#define Duct_IN1_Pin GPIO_PIN_2
#define Duct_IN1_GPIO_Port GPIOA
#define Light_CNV_2_Pin GPIO_PIN_3
#define Light_CNV_2_GPIO_Port GPIOA
#define Light_CNV_1_Pin GPIO_PIN_4
#define Light_CNV_1_GPIO_Port GPIOA
#define Light_SCK_Pin GPIO_PIN_5
#define Light_SCK_GPIO_Port GPIOA
#define Light_MISO_Pin GPIO_PIN_6
#define Light_MISO_GPIO_Port GPIOA
#define Light_MOSI_Pin GPIO_PIN_7
#define Light_MOSI_GPIO_Port GPIOA
#define Left_IN1_Pin GPIO_PIN_10
#define Left_IN1_GPIO_Port GPIOB
#define Left_IN2_Pin GPIO_PIN_11
#define Left_IN2_GPIO_Port GPIOB
#define ICM_SCK_Pin GPIO_PIN_13
#define ICM_SCK_GPIO_Port GPIOB
#define ICM_MISO_Pin GPIO_PIN_14
#define ICM_MISO_GPIO_Port GPIOB
#define ICM_MOSI_Pin GPIO_PIN_15
#define ICM_MOSI_GPIO_Port GPIOB
#define CS_ICM_Pin GPIO_PIN_6
#define CS_ICM_GPIO_Port GPIOC
#define Enable_Switch_Pin GPIO_PIN_8
#define Enable_Switch_GPIO_Port GPIOA
#define Right_IN1_Pin GPIO_PIN_15
#define Right_IN1_GPIO_Port GPIOA
#define Right_IN2_Pin GPIO_PIN_3
#define Right_IN2_GPIO_Port GPIOB
#define Encoder_Left_PWM_Pin GPIO_PIN_4
#define Encoder_Left_PWM_GPIO_Port GPIOB
#define Encoder_Left_IO_Pin GPIO_PIN_5
#define Encoder_Left_IO_GPIO_Port GPIOB
#define Encoder_Right_PWM_Pin GPIO_PIN_6
#define Encoder_Right_PWM_GPIO_Port GPIOB
#define Encoder_Right_IO_Pin GPIO_PIN_7
#define Encoder_Right_IO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
