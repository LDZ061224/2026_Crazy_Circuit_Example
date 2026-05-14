/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Fun.h
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description:  功能驱动声明头文件
Others:      无
Function List:
             1. 共用体、外设引脚宏定义
             2. 全局外部变量声明
             3. 驱动函数声明（ADC、电机、编码器、OLED等）
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30   0.0        创建初始版本
**************************************************/

// 防止头文件重复包含
#ifndef __FUN_H
#define __FUN_H

// 包含基础通用头文件
#include "zf_common_headfile.h"
// 包含全局宏定义与公共头文件
#include "headfiles.h"

/*********************************** 数据类型定义 ***********************************/
// 浮点数 <-> 4字节数组 共用体
// 用于串口发送浮点数（拆分float为4个uint8_t）
typedef union floatu8data
{
    float floatdata;       // 浮点数类型数据
    uint8 u8data[4];       // 对应4个字节数据
}floatu8data;

/*********************************** 硬件引脚宏定义 ***********************************/
// 左电机驱动PWM引脚定义
#define Left_Motor_IN1      ATOM0_CH7_P20_8
#define Left_Motor_IN2      ATOM2_CH6_P13_1

// 右电机驱动PWM引脚定义
#define Right_Motor_IN1     ATOM1_CH0_P02_8
#define Right_Motor_IN2     ATOM2_CH3_P00_12

// 吸风电机驱动PWM引脚定义
#define Suction_Motor_IN1   ATOM1_CH5_P00_6
#define Suction_Motor_IN2   ATOM1_CH3_P00_4

// 使能开关判断宏：P00_2引脚为高电平表示开启
#define EnableSwitch_ON     gpio_get_level(P00_2) == 1

/********************************* 全局外部变量声明 *********************************/
// 15路光敏传感器ADC原始值数组
extern uint16 Light_ADC[15];
// 15路光敏传感器阈值数组 [0]上阈值 [1]下阈值
extern float Light_Thr[15][2];
// ICM20602传感器状态标志
extern uint8 icm20602_Check;
// 陀螺仪Z轴角速度
extern float Gyro_Z;
// 电流检测值
extern float Current_Check;
// 两路电压检测值
extern float Voltage_Check[2];

/*********************************** 函数声明 ***********************************/
// VOFA上位机数据发送函数
void Vofa_Send_Data(void);
void Vofa_Send_Flash_Data(void);    // VOFA Flash数据导出
// 光敏传感器ADC初始化函数
void Light_Init(void);
// 编码器初始化函数
void Encoder_Init(void);
// 电机PWM初始化函数
void Motor_Init(void);
// 其他外设初始化（OLED、GPIO、ICM20602）
void Other_Init(void);
// 获取15路光敏传感器ADC值
void Get_Light(void);
// 获取光敏传感器阈值（自动校准）
void Get_Threshold(void);

#endif
