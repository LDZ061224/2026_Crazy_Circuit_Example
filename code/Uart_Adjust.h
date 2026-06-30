/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Uart_Adjust.h
Description:  UART remote tuning protocol module (portable across projects)
Others:      Depends on pid.h (PID_HandleTypeDef/PID_cleardata), Flash driver
**************************************************/
#ifndef __UART_ADJUST_H
#define __UART_ADJUST_H

#include "zf_common_headfile.h"

/***********************************Type Definitions***********************************/
typedef struct
{
    uint8   valid;       // 1 = new command pending consumption
    char    key[4];      // 3-char key name + '\0'
    float   value;       // Parsed float value
} uart_tuning_cmd_t;

/*********************************Global Variable Declarations*********************************/
extern uart_tuning_cmd_t g_tuning_cmd;

/* Flash storage arrays (defined in OLEDKeyboard.c) */
extern uint32 PID_OKb[13];
extern uint32 Speed_OKb[1];
extern uint32 DBG_OKb[4];

/***********************************API Declarations***********************************/
void Uart_Adjust_ParseByte(uint8 byte);    // Feed one byte into the frame-parsing state machine
void Uart_Adjust_Apply(void);              // Consume g_tuning_cmd, modify PID/target/switches
void Uart_Adjust_SaveToFlash(void);        // Persist current PID + speed + debug params to Flash

#endif
