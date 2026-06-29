/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Uart_Adjust.h
Description:  UART 远程调参协议模块（可跨工程移植）
Others:      依赖 pid.h（PID_HandleTypeDef/PID_cleardata）、Flash 驱动
**************************************************/
#ifndef __UART_ADJUST_H
#define __UART_ADJUST_H

#include "zf_common_headfile.h"

/***********************************类型定义***********************************/
typedef struct
{
    uint8   valid;       // 1 = 有新命令待消费
    char    key[4];      // 3 字符键名 + '\0'
    float   value;       // 解析出的浮点值
} uart_tuning_cmd_t;

/*********************************全局变量声明*********************************/
extern uart_tuning_cmd_t g_tuning_cmd;

/* Flash 存储数组（定义在 OLEDKeyboard.c 中） */
extern uint32 PID_OKb[13];
extern uint32 Speed_OKb[1];
extern uint32 DBG_OKb[4];

/***********************************API 声明***********************************/
void Uart_Adjust_ParseByte(uint8 byte);    // 逐字节喂入帧解析状态机
void Uart_Adjust_Apply(void);              // 消费 g_tuning_cmd，修改 PID/目标/开关
void Uart_Adjust_SaveToFlash(void);        // 将当前 PID + 速度 + 调试参数存入 Flash

#endif
