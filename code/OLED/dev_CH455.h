/*************************************************
Copyright (C), 2016-2014, TYUT JBD .
File name: CH455.h
Author: TYUT JBD
Version:1.0               Date: 2016.11.12
Description:  CH455.h
Others:      无
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
**************************************************/
#ifndef __CH455_H
#define __CH455_H

#include "OLEDkeyboard_Config.h"

/************************************宏定义************************************/

typedef enum
{
    KEY_0           = 0x00,
    KEY_1_          = 0x01,
    KEY_2_          = 0x02,
    KEY_3_          = 0x03,
    KEY_4_          = 0x04,
    KEY_5           = 0x05,
    KEY_6           = 0x06,
    KEY_7           = 0x07,
    KEY_8           = 0x08,
    KEY_9           = 0x09,
    KEY_BACK        = 0x0A,
    KEY_ENTER       = 0x0B,

    KEY_0_Long      = 0x0F,
    KEY_1_Long      = 0x1F,
    KEY_2_Long      = 0x2F,
    KEY_3_Long      = 0x3F,
    KEY_4_Long      = 0x4F,
    KEY_5_Long      = 0x5F,
    KEY_6_Long      = 0x6F,
    KEY_7_Long      = 0x7F,
    KEY_8_Long      = 0x8F,
    KEY_9_Long      = 0x9F,
    KEY_BACK_Long   = 0xAF,
    KEY_ENTER_Long  = 0xBF,

    // 当前被按下
    KEY_0_Down      = 0xFF - KEY_0,
    KEY_1_Down      = 0xFF - KEY_0,
    KEY_2_Down      = 0xFF - KEY_0,
    KEY_3_Down      = 0xFF - KEY_0,
    KEY_4_Down      = 0xFF - KEY_0,
    KEY_5_Down      = 0xFF - KEY_0,
    KEY_6_Down      = 0xFF - KEY_0,
    KEY_7_Down      = 0xFF - KEY_0,
    KEY_8_Down      = 0xFF - KEY_0,
    KEY_9_Down      = 0xFF - KEY_0,
    KEY_BACK_Down   = 0xFF - KEY_BACK,
    KEY_ENTER_Down  = 0xFF - KEY_ENTER,

    KEY_BLANK       = 0xFF,
}KeyValue_enum;

/*********************************全局变量声明*********************************/

//extern unsigned char keycode;

/***********************************函数声明***********************************/
void CH455_Init(void);
uint8 CH455_Read(void);
KeyValue_enum CH455_GetOneKey(void);
KeyValue_enum CH455_GetOneKey_GPS(uint8 Show, int32 Hyper_Select);
KeyValue_enum CH455_GetOneKey_RT(uint8 Show, uint8 Point_Num, int32 Hyper);
KeyValue_enum CH455_QueryOneKey(uint32 timeout);
#endif
