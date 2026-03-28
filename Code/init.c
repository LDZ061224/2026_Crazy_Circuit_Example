/*************************************************
Copyright (C), 2016-2025, TYUT JBD TRoMaC
File name: control.c
Author: 
Version:               
Date: 2025.10.24
Description:  
Others:      
Function List:
History:
<author>  	 <time>   	<version> 	 	<desc>
**************************************************/

#include "init.h"

int dbg_arr[2] = {0};

/************************************变量定义***********************************/
float Turn_P_Init = 0;
float Turn_D_Init = 0;
float Gyro_P_Init = 0;
float Gyro_D_Init = 0;
float Left_P_Init = 0;
float Left_I_Init = 0;
float Left_D_Init = 0;
float Right_P_Init = 0;
float Right_I_Init = 0;
float Right_D_Init = 0;
int Gyro_Exp_Init = 0;
int Left_Exp_Init = 0;
int Right_Exp_Init = 0;
int Dust_Exp_Init = 0;
uint8_t Gyro_mode_Init = 0;
/************************************函数定义***********************************/

/******************************************************
** Function: Motor_Init
** Description: PWM启动
** Others:
*******************************************************/
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	
	PWM_SetDuty(Duct_IN1, 0);
    PWM_SetDuty(Duct_IN2, 0);
}

/******************************************************
** Function: Delay_us
** Description:TIM7实现的us延时
** Others:
*******************************************************/
void Delay_us(uint32_t nus)
{
    uint16_t  differ = 0xffff - nus;
    //设置定时器7的计数初始值
	__HAL_TIM_SetCounter(&htim7, differ);
	//开启定时器
	HAL_TIM_Base_Start(&htim7);

	while (differ < 0x0000FFFC)
    {
		dbg_arr[0] = __HAL_TIM_GetCounter(&htim7);
        differ = __HAL_TIM_GetCounter(&htim7);
    };
    //关闭定时器
    HAL_TIM_Base_Stop(&htim7);
}

