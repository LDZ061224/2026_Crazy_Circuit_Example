/*************************************************
Copyright (C), 2016-2025, TYUT JBD TRoMaC
File name: fun.h
Author: 
Version:               
Date: 2025.9.29
Description:  
Others:      
Function List:
History:
<author>   <time>   <version>  <desc>
**************************************************/

#ifndef __FUN_H__
#define __FUN_H__

#include "headfiles.h"

#define FILT_LEN   8          // 滑动平均深度，可改

typedef union
{
	float 		floatValue;
	uint8_t 	uint8Value[4];
}float2u8_union;

/*************************************宏定义*************************************/
#define  Delay_ms(ms) 	Delay_us(ms * 1000)

/************************************全局变量***********************************/
extern int Left_Arr[8];
extern int Right_Arr[8];
extern float ICM_Acc[3];
extern float ICM_Gyro[3];
extern int a;
extern TIM_HandleTypeDef htim7;
extern float Gyro_Z_Bias;
extern uint8_t Start_Flag;
extern uint32_t filt_L_Correct[8][FILT_LEN];   // 左 8 路队列
extern uint32_t filt_R_Correct[8][FILT_LEN];   // 右 8 路队列
extern uint8_t Start_Flag;  // 启动标志位
extern uint8_t dust_Flag;  // 负压标志位
/************************************函数声明***********************************/
void Delay_us(uint32_t nus);		
void Usart_SendArr(UART_HandleTypeDef *huart, uint8_t *arr, uint16_t size);
void VOFA_JustFloat(UART_HandleTypeDef *huart);
void Left_Threshold_Correct(void);
void Right_Threshold_Correct(void);
void Arr_Init(void);
void filt_scale_20000(int arr[8], uint32_t filt_q[8][FILT_LEN]);
extern float Sliding_Avg(int x);
extern void WitSD_Send(UART_HandleTypeDef *huart);
int16_t Map_Range(int16_t range1[2], int16_t range2[2], int16_t value);
float Gyro_Calibrate_Z_Bias(uint16_t sample_count, uint16_t sample_interval_ms);
float Gyro_Remove_Z_Bias(float gyro_z);
void Start(void);

#endif
