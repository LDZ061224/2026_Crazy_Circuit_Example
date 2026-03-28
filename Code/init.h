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

#ifndef __INIT_H__
#define __INIT_H__

#include "headfiles.h"


/*************************************宏定义************************************/
#define Amplitude_Limit(Value,Dowm,Up)  ((Value) < (Dowm) ? (Dowm) : ((Value) > (Up) ? (Up) : (Value)));
#define Step_Limit(New,Old,Range)       (New > (Old + Range) ? (Old + Range) : (New < (Old - Range) ? (Old - Range) : (New)))
#define ABS(x)                          (((x) >= (0.0f)) ? (x) : (-(x)))
#define Data_Limit(Value,Dowm,Up)       ((Value) < (Dowm) ? (Dowm) : ((Value) > (Up) ? (Up) : (Value)))
#define Limited_Add(Value,MAX)          ((++Value) > MAX ? MAX : Value)
#define SignOf(Value)                   ((Value < 0.0) ? (-1.0) : (1.0))
#define Increase_Limit(var, limit)      (var = (++var) > limit ? (limit+1) : var)

#define PI                              (3.1415926535897932384626433832795f)

#define Left_IN1  ((uint32_t *)&TIM2->CCR3)
#define Left_IN2  ((uint32_t *)&TIM2->CCR4)

#define Right_IN1 ((uint32_t *)&TIM2->CCR1)
#define Right_IN2 ((uint32_t *)&TIM2->CCR2)

#define Duct_IN1  ((uint32_t *)&TIM5->CCR2)
#define Duct_IN2  ((uint32_t *)&TIM5->CCR3)

#define Enable_Switch_ON	(HAL_GPIO_ReadPin(Enable_Switch_GPIO_Port, Enable_Switch_Pin) == 1)

/************************************全局变量***********************************/
extern int Left_Light[8];
extern int Right_Light[8];
extern int Left_Real_Spd;
extern int Right_Real_Spd;		

extern PID_HandleTypeDef Turn_PID;
extern PID_HandleTypeDef Left_PID;
extern PID_HandleTypeDef Right_PID;

extern float Turn_P_Init;
extern float Turn_D_Init;
extern float Left_P_Init;
extern float Left_I_Init;
extern float Left_D_Init;
extern float Right_P_Init;
extern float Right_I_Init;
extern float Right_D_Init;
extern float Gyro_P_Init;
extern float Gyro_D_Init;
extern int Gyro_Exp_Init;
extern int Left_Exp_Init;
extern int Right_Exp_Init;
extern int Dust_Exp_Init;
extern uint8_t Gyro_mode_Init;
	
extern int Left_Thres_Arr_Up[8]; 
extern int Right_Thres_Arr_Down[8];
extern int Left_Thres_Arr_Down[8]; 
extern int Right_Thres_Arr_Up[8];

/************************************函数声明***********************************/
void bsp_dwt_init(void);
void bsp_dwt_delay(uint32_t _delay_time);
void bsp_delay_us(uint32_t _delay_time);
void bsp_delay_ms(uint32_t _delay_time);
void Delay_us(uint32_t nus);

void Motor_Init(void);
void PID_Init(void);

#endif
