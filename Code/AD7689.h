/*************************************************
Copyright (C), 2016-2025, TYUT JBD TRoMaC
File name: AD7689.h
Author: 
Version: Demo              
Date: 2025.9.25
Description:  
Others:      
Function List:
History:
<author> Cross_Z   <time>  2025.9.28    <version> Demo <desc>
**************************************************/

#ifndef __AD7689_H__
#define __AD7689_H__

#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdint.h"
#include "headfiles.h"

/************************************宏定义************************************/
// 一共8个通道
#define IN_NUM  8          // IN0~IN7

/* 配置字节所在位置 */
#define RB_POS      0
#define SEQ_POS     1
#define REF_POS     3
#define BW_POS      6
#define INX_POS     7
#define INCC_POS    10
#define CFG_POS     13

/* ----------配置寄存器设置---------- */
#define RB_DIS       (1UL << RB_POS)        // 不回读
#define SEQ_OFF      (0UL << SEQ_POS)       // 0b00 = 禁用通道序列
#define SEQ_ON       (3UL << SEQ_POS)       // 0b11 = 使能通道序列
#define REF_INT_2p5  (0UL << REF_POS)       // 0b000 = 参考电压 = 2.50V
#define REF_INT_4p   (1UL << REF_POS)       // 0b001 = 参考电压 = 4.096V
#define REF_INT_OUT  (7UL << REF_POS)       // 0b111 = 外部参考电压
#define BW_FULL      (1UL << BW_POS)        // 全带宽
#define INCC_UNI     (7UL << INCC_POS)      // 共地
#define INCC_COM     (6UL << INCC_POS)      // 普通地
#define INX_0        (0UL << INX_POS)       // IN0
#define INX_1        (1UL << INX_POS)       // IN1
#define INX_2        (2UL << INX_POS)       // IN2
#define INX_3        (3UL << INX_POS)       // IN3
#define INX_4        (4UL << INX_POS)       // IN4
#define INX_5        (5UL << INX_POS)       // IN5
#define INX_6        (6UL << INX_POS)       // IN6
#define INX_7        (7UL << INX_POS)       // IN7
#define CFG_OVR      (1UL << CFG_POS)       // 每次更新配置寄存器
#define CFG_NO_OVR   (0UL << CFG_POS)  		// 每次不更新配置寄存器

/***********************************全局变量声明************************************/
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1;
extern int ADC_Buff[IN_NUM];

/************************************函数声明************************************/
void AD7689_Right_Init(void);
void AD7689_Left_Init(void);
void AD7689_Left_Read(int code[8]);
void AD7689_Right_Read(int code[8]);
void AD7689_Trans(void);
void CNV_1_L(void);		void CNV_2_L(void);
void CNV_1_H(void);		void CNV_2_H(void);

#endif
