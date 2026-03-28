/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin|CS_ICM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Light_CNV_2_Pin|Light_CNV_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Buzzer_Pin CS_ICM_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|CS_ICM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Light_CNV_2_Pin Light_CNV_1_Pin */
  GPIO_InitStruct.Pin = Light_CNV_2_Pin|Light_CNV_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Enable_Switch_Pin */
  GPIO_InitStruct.Pin = Enable_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Enable_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Encoder_Left_IO_Pin Encoder_Right_IO_Pin */
  GPIO_InitStruct.Pin = Encoder_Left_IO_Pin|Encoder_Right_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void SDA_GPIO_init(void)
{
	GPIO_InitTypeDef SDA_InitStruct = {0};
		
	SDA_InitStruct.Pin = OLED_IIC_SDA_PIN;
    SDA_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    SDA_InitStruct.Pull = GPIO_NOPULL;
    SDA_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &SDA_InitStruct);
		
	HAL_GPIO_WritePin(GPIOB, OLED_IIC_SDA_PIN, GPIO_PIN_SET);
}

void SCL_GPIO_init(void)
{
	GPIO_InitTypeDef SCL_InitStruct = {0};
		
	SCL_InitStruct.Pin = OLED_IIC_SCL_PIN;
    SCL_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    SCL_InitStruct.Pull = GPIO_NOPULL;
    SCL_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &SCL_InitStruct);
		
	HAL_GPIO_WritePin(GPIOB, OLED_IIC_SCL_PIN, GPIO_PIN_SET);
}

void SDA_GPIO_out(void)
{
	GPIO_InitTypeDef SDA_InitStruct = {0};
		
	SDA_InitStruct.Pin = OLED_IIC_SDA_PIN;
    SDA_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    SDA_InitStruct.Pull = GPIO_NOPULL;
    SDA_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &SDA_InitStruct);
		
	HAL_GPIO_WritePin(GPIOB, OLED_IIC_SDA_PIN, GPIO_PIN_SET);
}

void SDA_GPIO_in(void)
{
	GPIO_InitTypeDef SDA_InitStruct = {0};
		
	SDA_InitStruct.Pin = OLED_IIC_SDA_PIN;
    SDA_InitStruct.Mode = GPIO_MODE_INPUT;
    SDA_InitStruct.Pull = GPIO_NOPULL;
    SDA_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &SDA_InitStruct);
}
/* USER CODE END 2 */
