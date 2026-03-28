/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define USART_DMA_REC_SIE 256
#define USART_REC_SIE (USART_DMA_REC_SIE * 2)

typedef struct
{   
	uint8_t  UsartRecFlag;    // 串口接收完成标志位
	uint16_t UsartDMARecLen;  // 串口DMA接收长度
	uint16_t UsartRecLen;     // 串口接收完成长度
	uint8_t  UsartDMARecBuffer[USART_DMA_REC_SIE];  // 串口DMA接收缓冲区  
	uint8_t  UsartRecBuffer[USART_REC_SIE];         // 串口接收缓冲器
	int     frame_ok;                 // 解析成功标志
	uint8_t  raw[23];                 // 原始数据包
    int32_t  var_int[4];              // 解析出的 4 个 int32
	int16_t  var_short[3];            // 解析出的 3 个 short
} ts_Usart_type;
extern ts_Usart_type Usart1type;
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern void usart1_parse_frame(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

