/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Fun.h
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description:  通用宏定义、头文件包含
Others:      无
Function List:
             1. 数值限制、绝对值、符号等通用宏函数
             2. 工程所需头文件统一包含
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30   0.0        创建初始版本
**************************************************/

#ifndef __HEADFILES_H
#define __HEADFILES_H

/*********************************** 通用宏定义 ***********************************/
// 幅值限制宏：将Value限制在 [Down, Up] 之间，小于Down取Down，大于Up取Up
#define Amplitude_Limit(Value, Down, Up)  ((Value) < (Down) ? (Down) : ((Value) > (Up) ? (Up) : (Value)))

// 步长限制宏：限制New值相对于Old值的变化幅度不超过Range，防止突变
#define Step_Limit(New, Old, Range)       (New > (Old + Range) ? (Old + Range) : (New < (Old - Range) ? (Old - Range) : (New)))

// 绝对值宏：返回输入数值的绝对值
#define ABS(x)                          (((x) >= (0.0f)) ? (x) : (-(x)))

// 数据限制宏：同幅值限制，将Value限制在 [Down, Up] 区间内
#define Data_Limit(Value, Down, Up)       ((Value) < (Down) ? (Down) : ((Value) > (Up) ? (Up) : (Value)))

// 限制自增宏：变量自增后不超过最大值MAX，达到MAX后保持不变
#define Limited_Add(Value, MAX)          ((++Value) > MAX ? MAX : Value)

// 符号判断宏：返回数值的符号，负数返回-1.0，正数/0返回1.0
#define SignOf(Value)                   ((Value < 0.0) ? (-1.0) : (1.0))

// 增量限制宏：变量自增，超过limit后赋值为limit+1，用于状态计数
#define Increase_Limit(var, limit)      (var = (++var) > limit ? (limit + 1) : var)

// 圆周率常量定义（高精度浮点数）
#define PI                              (3.1415926535897932384626433832795f)

/*********************************** 头文件包含 ***********************************/
#include "zf_common_headfile.h"    // 智峰官方通用公共头文件
#include "pid.h"                   // PID算法相关头文件
#include "Fun.h"                   // 功能驱动实现头文件
#include "TCA9555.h"               // TCA9555 IO扩展芯片驱动头文件
#include "OLEDKeyboard.h"          // OLED显示屏与按键驱动头文件
#include "Ctrl.h"                  // 系统控制逻辑头文件

#endif