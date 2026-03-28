/*************************************************
Copyright (C), 2016-2017, TYUT JBD TEAM C.
File name: CH455.h
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
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

#include  "OLED.h"

/************************************宏定义************************************/
typedef enum
{
    KEY_0           = 0x00,
    KEY_1           = 0x01,
    KEY_2           = 0x02,
    KEY_3           = 0x03,
    KEY_4           = 0x04,
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
    KEY_1_Down      = 0xFF - KEY_1,
    KEY_2_Down      = 0xFF - KEY_2,
    KEY_3_Down      = 0xFF - KEY_3,
    KEY_4_Down      = 0xFF - KEY_4,
    KEY_5_Down      = 0xFF - KEY_5,
    KEY_6_Down      = 0xFF - KEY_6,
    KEY_7_Down      = 0xFF - KEY_7, 
    KEY_8_Down      = 0xFF - KEY_8,
    KEY_9_Down      = 0xFF - KEY_9,
    KEY_BACK_Down   = 0xFF - KEY_BACK,
    KEY_ENTER_Down  = 0xFF - KEY_ENTER,

    KEY_BLANK       = 0xFF,
}KeyValue_enum;

/*********************************全局变量声明*********************************/

extern unsigned char keycode;

/***********************************函数声明***********************************/
extern void CH455_Init(void);
extern unsigned char CH455_Read(void);
KeyValue_enum CH455_GetOneKey(void);

#endif
