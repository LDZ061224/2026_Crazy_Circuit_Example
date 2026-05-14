/*************************************************
Copyright (C), 2016-2024, TYUT JBD .
File name: OLEDKeyboard.h
Author: TEAM  A B C
Version:2.0               Date: 2024.12.07
Description:  OLEDKeyboard.h
Others:      无
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
SUV       2024.12.07  2.0        基于新库重构
**************************************************/
#ifndef __OLEDKEYBOARD_H
#define __OLEDKEYBOARD_H

#include "zf_common_headfile.h"
#include "OLEDkeyboard_Config.h"

/***********************************函数声明***********************************/
void OLED_Input(void);
void OLED_Display(void);
extern void OLED_Data_Load();

#endif

