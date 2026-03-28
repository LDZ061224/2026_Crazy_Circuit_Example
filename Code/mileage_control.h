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

#ifndef __MILEAGE_CONTROL_H__
#define __MILEAGE_CONTROL_H__

#include "headfiles.h"

/*************************************宏定义*************************************/
#define NODE_NUM_MAX    30   // 节点数量最大值
#define ELEMENT_NUM_MAX 3   // 元器件数量最大值

/************************************PID初始化***********************************/
#define GYRO_PID { \
    .kp    		= 1, \
    .ki    		= 0, \
    .kd    		= 0, \
    .iOutMax    = 0, \
    .outMax		= 15, \
	.mode = PID_MODE_POSITION}	
	
#define LEFT_PID { \
    .kp    		= 3, \
    .ki    		= 1, \
    .kd    		= 0, \
    .iOutMax    = 4500, \
    .outMax		= 4500,\
	.mode = PID_MODE_ADD}

#define RIGHT_PID { \
    .kp    		= 3, \
    .ki    		= 1, \
    .kd    		= 0, \
    .iOutMax    = 4500, \
    .outMax		= 4500,\
	.mode = PID_MODE_ADD}

#define TURN_PID { \
    .kp    		= 1, \
    .ki    		= 0, \
    .kd    		= 0, \
    .iOutMax    = 0, \
    .outMax		= 4000 , \
	.mode = 1}

#define NODE_NUM 4

/************************************结构体***********************************/	
	typedef struct
{
    int Left;
    int Right;
    int Stop;
    float Mileage;
    int Straight;
    float Node_Mileage;
}Count_Typedef ;/*有关计数的*/

typedef struct
{
    int16_t Range_Target[2];   // 目标数组
    int16_t Range_Source[2];   // 源数组
}Speed_Adjust_Typedef;/*有关计数的*/

/**
 * @Racing_track_Typedef————参数说明
 * *******************************

 * Node_Num;												节点数量
 * Element_All_Num;		 								    元器件总数量（即节点之间）
 * Stop_Mode;												停止模式（最后一个是节点还是元器件）
 * Arr_Node_Dir[NODE_NUM_MAX];  			 				节点转向数组		
 * Arr_Element_Num[NODE_NUM_MAX];   						节点间元器件数量
 * Arr_Element_Dir[NODE_NUM_MAX][ELEMENT_NUM_MAX];			元器件方向数组
 * Arr_Mileage_Normal[NODE_NUM_MAX][ELEMENT_NUM_MAX];		正常行驶里程
 * Arr_Mileage_Element[NODE_NUM_MAX][ELEMENT_NUM_MAX];		元器件里程
 * Arr_Mileage_ALL[NODE_NUM_MAX];							路段总里程
 * Speed_Adjust_Typedef Speed_Adjust;						加减速结构体
 */                     
typedef struct          
{                       
	uint8_t Node_Num;   
    uint8_t Element_All_Num;
    uint8_t Stop_Mode;
    uint8_t Arr_Node_Dir[NODE_NUM_MAX];
    uint8_t Arr_Element_Num[NODE_NUM_MAX];
    uint8_t Arr_Element_Dir[NODE_NUM_MAX][ELEMENT_NUM_MAX];
    uint16_t Arr_Mileage_Normal[NODE_NUM_MAX][ELEMENT_NUM_MAX];
    uint16_t Arr_Mileage_Element[NODE_NUM_MAX][ELEMENT_NUM_MAX];
    uint16_t Arr_Mileage_ALL[NODE_NUM_MAX];
    Speed_Adjust_Typedef Speed_Adjust;
}Racing_track_Typedef;/*存赛道信息，便于切换赛道*/
	
typedef enum
{
    Normal_Mode,
    Turn_Left,
    Turn_Right,
    Mileage_Mode,
    Straight_Mode,
}Run_Mode_Enum;/*运行模式*/

/************************************全局变量***********************************/
extern PID_HandleTypeDef Turn_PID;
extern PID_HandleTypeDef Gyro_PID;
extern PID_HandleTypeDef Left_PID;
extern PID_HandleTypeDef Right_PID ;
	
extern int Image_Arr[16];

extern uint8_t Track_Num;		// 识别点数量
extern uint8_t Valid_Flag;		// 数据是否有效
extern uint8_t Left_Out;		// 左打死
extern uint8_t Right_Out;		// 右打死
extern int8_t Error;	
extern int Left_Exp_Spd;		// 左期望
extern int Right_Exp_Spd;		// 右期望
	
extern Count_Typedef Count;
/*---------------------------------keyboard------------------------------------*/
extern int L_exp;
extern int R_exp;
extern int dust_exp;
extern int Gyro_exp;
extern uint8_t Gyro_mode;
extern uint8_t Integral_Flag;
extern float Gyro_Integral;
extern uint8_t Node_Arr_Dir[NODE_NUM];	// 节点方向数组(1左2右0直行)
extern uint8_t Node_Arr_Angle[20];	// 节点角度数组(0->直行 1->45度 2->90度)
extern uint8_t Node_Times;		// 节点数量	
extern uint8_t Execute_Times;	// 执行数量
extern uint8_t Execute_All_Times;	// 执行总数量 
extern uint8_t Line_Num_Count;       // 有元器件的路段中正常线路的数量
extern uint8_t In_Line_Ele_Count;    // 一段连线中元器件计数
extern int Hollow_Flag;	
extern int	Debug_Mode;
extern float Gyro_PID_Out;

extern float  left_thr[8][2];  		// 左阈值
extern float  right_thr[8][2]; 		// 右阈值

extern Racing_track_Typedef Test1;
/************************************函数声明***********************************/
void Car_Go(void);
void Get_Speed(void);
void Get_Error(void);
void Get_IMU(void);
void PWM_SetDuty(uint32_t *Channel, int32_t Duty);
void Set_Out(void);
void Image_Process(void);   // 图形数组处理
void Normal_Run(void);
void Turn_Left_Run(void);
void Turn_Right_Run(void);
void Straight_Run(void);
void Mileage_Mode_Run(void);
#endif
