/*************************************************
Copyright (C), 2016-2025, TYUT JBD TRoMaC
File name: AD7689.c
Author: 
Version: Demo              
Date: 2025.9.25
Description:  
Others:      
Function List: AD7689_init() | AD7689_Read() | AD7689_Trans() | CNV_H/L()
History:
<author> Cross_Z   <time>  2025.9.28    <version>  Demo <desc>
**************************************************/

#include "AD7689.h"

/************************************变量定义***********************************/
int ADC_Buff[IN_NUM] = {0};
int dbg = 0;

// AD7689配置寄存器
uint16_t AD_CH[8] = {INX_1, INX_2, INX_3, INX_4, INX_5, INX_6, INX_7, INX_0};

uint16_t AD7689_Config = (CFG_OVR | INCC_COM | INX_0 | BW_FULL | REF_INT_2p5 | SEQ_OFF | RB_DIS) << 2;

/************************************函数定义***********************************/

/******************************************************
** Function: AD7689_Left_Init
** Description: AD7689初始化
** Others:
*******************************************************/
void AD7689_Left_Init(void)
{
	Delay_ms(15);                // 等待15 ms

	/* ---第一次发--- */
	CNV_1_H();                     // 启动转换
	Delay_us(10);                 // 等待

	CNV_1_L();
	Delay_us(2);                 // 转换结束

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&AD7689_Config, 1, 100);
		
	CNV_1_H();                     // 锁存配置
	Delay_us(1);                 // 等待
	CNV_1_L();                     // 拉低等待下次通信
	
	/* ---第二次发--- */
	CNV_1_H();                     // 启动转换
	Delay_us(10);                 // 等待

	CNV_1_L();
	Delay_us(2);                 // 转换结束

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&AD7689_Config, 1, 100);
		
	CNV_1_H();                     // 锁存配置
	Delay_us(1);                 // 等待
	CNV_1_L();                     // 拉低等待下次通信
}

/******************************************************
** Function: AD7689_Right_Init
** Description:AD7689初始化
** Others:
*******************************************************/
void AD7689_Right_Init(void)
{	
	Delay_ms(15);                // 等待15 ms

	/* ---第一次发--- */
	CNV_2_H();                     // 启动转换
	Delay_us(10);                 // 等待

	CNV_2_L();
	Delay_us(2);                 // 转换结束

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&AD7689_Config, 1, 100);
		
	CNV_2_H();                     // 锁存配置
	Delay_us(1);                 // 等待
	CNV_2_L();                     // 拉低等待下次通信
	
	/* ---第二次发--- */
	CNV_2_H();                     // 启动转换
	Delay_us(10);                 // 等待

	CNV_2_L();
	Delay_us(2);                 // 转换结束

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&AD7689_Config, 1, 100);
		
	CNV_2_H();                     // 锁存配置
	Delay_us(1);                 // 等待
	CNV_2_L();                     // 拉低等待下次通信
}

/******************************************************
** Function: CNV_L / CNV_H
** Description:CNV拉高/拉低
** Others:
*******************************************************/
void CNV_1_H()
{
	  HAL_GPIO_WritePin(Light_CNV_1_GPIO_Port, Light_CNV_1_Pin, GPIO_PIN_SET);
}

void CNV_1_L()
{
	  HAL_GPIO_WritePin(Light_CNV_1_GPIO_Port, Light_CNV_1_Pin, GPIO_PIN_RESET);
}

void CNV_2_H()
{
	  HAL_GPIO_WritePin(Light_CNV_2_GPIO_Port, Light_CNV_2_Pin, GPIO_PIN_SET);
}

void CNV_2_L()
{
	  HAL_GPIO_WritePin(Light_CNV_2_GPIO_Port, Light_CNV_2_Pin, GPIO_PIN_RESET);
}

/******************************************************
** Function: AD7689_Read
** Description:AD7689读取
** Others:
*******************************************************/
void AD7689_Left_Read(int code[8])
{
	uint16_t dummy;

	/* 丢掉上电后的未知数据 */
	AD7689_Config = (CFG_OVR | INCC_COM | AD_CH[0] | BW_FULL |
	                 REF_INT_2p5 | SEQ_OFF | RB_DIS) << 2;
	CNV_2_H();
	Delay_us(3);
	CNV_2_L();
	Delay_us(1);
	HAL_SPI_TransmitReceive(&hspi1,
	                        (uint8_t *)&AD7689_Config,
	                        (uint8_t *)&dummy,
	                        1, 100);
	CNV_2_H();
	Delay_us(5);
	CNV_2_L();
	
	for (int i = 0; i < 8; i++)
	{
		// 配置字更新
		AD7689_Config = (CFG_OVR | INCC_COM | AD_CH[i] | BW_FULL | REF_INT_2p5 | SEQ_OFF | RB_DIS) << 2;
		
		CNV_2_H();                   // 拉高转化
		Delay_us(3);                 // 等待转化

		/* 发送新的配置字节，同时接收数据 */
		CNV_2_L();
		Delay_us(1);                 // tCNVH_L >= 10 ns
		
		dbg = HAL_SPI_TransmitReceive
		(				
			&hspi1,
			(uint8_t *)&AD7689_Config,
			(uint8_t *)&code[7 - i],
			1,
			100
		);
				
		/* 锁存配置，等待下次转换 */
		CNV_2_H();                     
		Delay_us(2);                 
		CNV_2_L();  
	}
}

void AD7689_Right_Read(int code[8])
{
	uint16_t dummy;

	/* 丢掉上电后的未知数据 */
	AD7689_Config = (CFG_OVR | INCC_COM | AD_CH[0] | BW_FULL |
	                 REF_INT_2p5 | SEQ_OFF | RB_DIS) << 2;
	CNV_1_H();
	Delay_us(3);
	CNV_1_L();
	Delay_us(1);
	HAL_SPI_TransmitReceive(&hspi1,
	                        (uint8_t *)&AD7689_Config,
	                        (uint8_t *)&dummy,
	                        1, 100);
	CNV_1_H();
	Delay_us(2);
	CNV_1_L();
	
	for (int i = 0; i < 8; i++)
	{
		// 配置字更新
		AD7689_Config = (CFG_OVR | INCC_COM | AD_CH[i] | BW_FULL | REF_INT_2p5 | SEQ_OFF | RB_DIS) << 2;
		
		CNV_1_H();                     // 拉高转化
		Delay_us(3);                 // 等待转化

		/* 发送新的配置字节，同时接收数据 */
		CNV_1_L();
		Delay_us(1);                 // tCNVH_L >= 10 ns
			
		dbg = HAL_SPI_TransmitReceive
		(				
			&hspi1,
			(uint8_t *)&AD7689_Config,
			(uint8_t *)&code[7 - i],
			1,
			100
		);
				
		/* 锁存配置，等待下次转换 */
		CNV_1_H();                     
		Delay_us(2);                 
		CNV_1_L(); 
		Delay_us(2); 		
	}	
}
