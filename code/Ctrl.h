/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.h
Author: Cross_Z
Version:0.0        Date: 2026.1.30
Description: 智能车核心控制头文件
             包含PID参数、运行模式、赛道结构体、全局变量、外部函数声明
Others:  基于中景智行底层库开发
Function List: 运动控制、速度闭环、循迹、里程、转向、建图/回放模式
History:
<author>    <time>       <version>    <desc>
Cross_Z     2026.1.30      0.0        创建初始版本
**************************************************/

#ifndef __CTRL_H
#define __CTRL_H

// 底层通用头文件 + 自定义总头文件
#include "zf_common_headfile.h"
#include "headfiles.h"

/**********************************************
* 宏定义
**********************************************/
#define NODE_NUM_MAX            20      // 赛道最大节点数
#define ELEMENT_NUM_MAX         5       // 单个节点最大元素数
#define TURN_MILEAGE_RECORD_MAX 120     // 转弯间距记录上限

// 陀螺仪 PID（位置式）
#define GYRO_PID { \
    .kp         = 0.008, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_POSITION \
}

// 左电机 PID（增量式）
#define LEFT_PID { \
    .kp         = 0, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 9500, \
    .mode       = PID_MODE_ADD \
}

// 右电机 PID（增量式）
#define RIGHT_PID { \
    .kp         = 250, \
    .ki         = 65, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 9500, \
    .mode       = PID_MODE_ADD \
}

// 转向 PID（增量式）
#define TURN_PID { \
    .kp         = 80, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 10000, \
    .mode       = PID_MODE_POSITION \
}

/**********************************************
* 枚举类型定义
**********************************************/

/**
 * @brief 小车运行模式
 */
typedef enum
{
    Normal_Mode,     // 常规循迹
    Turn_Left,       // 左转模式
    Turn_Right,      // 右转模式
    Mileage_Mode,    // 里程控制模式
    Straight_Mode,   // 直行模式
} Run_Mode_Enum;

/**
 * @brief 里程计运行阶段
 */
typedef enum
{
    Normal_Stage,    // 常规阶段
    Straight_Stage,  // 直行阶段
} Mileage_Stage_Enum;

/**
 * @brief 键盘显示工作模式
 */
typedef enum
{
    Remember_Mode,   // 回放模式
    Build_Mode,      // 建图模式
} Mode_Define;

/**********************************************
* 结构体定义
**********************************************/

/**
 * @brief 计数结构体（编码器、里程、状态计数）
 */
typedef struct
{
    int     Left;        // 左转出弯计数
    int     Right;       // 右转出弯计数
    int     Stop;        // 停车计数
    float   Mileage;     // 当前段里程
    float   Spd_Mileage; // 速度里程
    int     Straight;    // 直行计数
    int     Stall;       // 堵转计数
} Count_Typedef;

/**
 * @brief 赛道信息结构体（存储节点、元素、方向、里程）
 */
typedef struct
{
    uint8_t Node_Arr_Dir[NODE_NUM_MAX];                  // 各节点行驶方向
    uint8_t Node_Arr_Mileage_Num[NODE_NUM_MAX];         // 各节点里程段数量
    uint8_t Node_Arr_Mileage_Dir[NODE_NUM_MAX][ELEMENT_NUM_MAX];  // 各里程段行驶方向
    int     Node_Arr_Mileage_Normal[NODE_NUM_MAX][ELEMENT_NUM_MAX];   // 普通路段里程值
    int     Node_Arr_Mileage_Element[NODE_NUM_MAX][ELEMENT_NUM_MAX]; // 元素路段里程值
    uint8_t Node_Num;       // 有效节点总数
    uint8_t Element_Num;    // 有效元素总数
    uint8_t Stop_Mode;      // 停车模式
} Racing_track_Typedef;

/**********************************************
* 外部全局变量声明
**********************************************/
// 基础速度
extern int Basic_Speed;

// 期望速度
extern int Left_Exp_Spd;
extern int Right_Exp_Spd;
extern int Middle;
// 实际速度
extern int Left_Real_Spd;
extern int Right_Real_Spd;

// 平均速度
extern float Average_Speed;

// 循迹误差
extern int Error;

// 光敏传感器
extern uint16 Light_ADC[15];
extern uint8  Light_Convert[15];

// PID 输出
extern float Turn_PID_Out;
extern float Gyro_PID_Out;
extern float Left_PID_Out;
extern float Right_PID_Out;

// 状态标志
extern int  Stop_Flag;          // 停车标志
extern int  Finish_Flag;         // 任务完成标志
extern int  Finish_Count;        // 完成计数
extern int  Track_Num;           // 有效循迹传感器数

// 执行计数
extern int8_t  Execute_Times;    // 当前执行节点索引
extern int8_t  Mileage_Times;     // 当前节点里程段总数

// 元素/路线计数
extern uint8_t Line_Num_Count;    // 路线数量
extern uint8_t In_Line_Ele_Count; // 当前线路元素索引

// 陀螺仪相关
extern float Gyro_Integral;

// 回放/建图控制参数
extern int    Turn_Error_Value;
extern int16  Check_Edge_Count;    // Check_Edge触发次数
extern float  Remember_Mileage_Prepare_Distance;
extern float  Remember_Node_Prepare_Distance;
extern int    Remember_Turn_Error;
extern int    Remember_Speed_Min_Value;
extern int    Remember_Speed_Max_Value;
extern uint8  vofa_flash_dump_mode;     // VOFA Flash数据导出模式标志
extern float  Total_Run_Mileage;       // 总运行里程（回放模式里程对比基准）
// 结构体实例
extern Count_Typedef Count;
extern Mode_Define Mode;
extern float Gyro_Z_For_PID; // PID用陀螺仪Z轴数据
// 当前运行状态
extern Run_Mode_Enum       Run_Mode;
extern Mileage_Stage_Enum  Mileage_Stage;
extern Racing_track_Typedef Run_Track;

// Flash里程数据（VOFA导出用）
extern float Segment_Edge_Mileage_Record[NODE_NUM_MAX][ELEMENT_NUM_MAX];
extern float Segment_Total_Mileage[NODE_NUM_MAX + 1];         // 各段路实测总里程
extern float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX];
extern uint16_t Turn_Mileage_Record_Num;
extern float Remember_Next_Target_Mileage; // 记忆模式下的下一个目标里程   
// 预赛赛道
extern Racing_track_Typedef Pre_Contest_1;
extern Racing_track_Typedef Pre_Contest_2;
extern Racing_track_Typedef Pre_Contest_3;

// 决赛赛道
extern Racing_track_Typedef Final_Contest_1;
extern Racing_track_Typedef Final_Contest_2;
extern Racing_track_Typedef Final_Contest_3;

// PID 控制器
extern PID_HandleTypeDef Gyro_PID;
extern PID_HandleTypeDef Left_PID;
extern PID_HandleTypeDef Right_PID;
extern PID_HandleTypeDef Turn_PID;

/**********************************************
* 外部函数声明
**********************************************/
void Car_Go(void);                          // 小车主运行函数
void Set_Speed(void);                       // 设置目标速度
void Get_Speed(void);                       // 获取实际速度
uint8 Check_Edge(void);                      // 边缘检测
void Get_IMU(void);                         // 获取陀螺仪数据
void Set_Out(void);                         // 电机输出控制

void Normal_Run(void);                      // 常规循迹运行
void Straight_Run(void);                    // 直行运行
void Turn_Left_Run(void);                   // 左转控制
void Turn_Right_Run(void);                  // 右转控制

void Mileage_Mode_Run(void);                // 里程模式总控
void Mileage_Run_Stage_2(void);             // 里程阶段2

void Build_Mode_Get_Error(void);            // 建图模式获取误差
void Remember_Mode_Get_Error(void);         // 回放模式获取误差
void Load_All_Flash_Data_For_VOFA(void);    // VOFA导出：加载Flash全部地图数据

#endif
