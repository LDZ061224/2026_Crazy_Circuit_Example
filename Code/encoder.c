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

/************************************变量定义***********************************/

/************************************函数定义***********************************/

/******************************************************
** Function: Encodeer_Init
** Description: 测速初始化
** Others:
*******************************************************/
void Encodeer_Init(void)
{
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
}

/******************************************************
** Function: Get_Count
** Description: 获取计数器计数值
** Others:
*******************************************************/
int Get_Count(TIM_HandleTypeDef *htim)
{
	return __HAL_TIM_GET_COUNTER(htim);
}

/******************************************************
** Function: Clear_Count
** Description: 清空计数器计数值
** Others:
*******************************************************/
void Clear_Count(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
}

