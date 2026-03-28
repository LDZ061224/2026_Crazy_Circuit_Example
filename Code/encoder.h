/*************************************************
Copyright (C), 2016-2025, TYUT JBD TRoMaC
File name: control.h
Author: 
Version:               
Date: 2025.10.24
Description:  
Others:      
Function List:
History:
<author>   <time>   <version>  <desc>
**************************************************/

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "headfiles.h"

/*************************************宏定义************************************/
#define Encoder_Left 	htim4
#define Encoder_Right 	htim3
#define LeftForward 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1
#define RightForward 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1

/************************************全局变量***********************************/


/************************************函数声明***********************************/
void Encodeer_Init(void);
int Get_Count(TIM_HandleTypeDef *htim);
void Clear_Count(TIM_HandleTypeDef *htim);

#endif
