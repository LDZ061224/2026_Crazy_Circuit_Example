/*************************************************
Copyright (C), 2016-2020, TYUT JBD TEAM C.
File name: ssd1306.h
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:  ssd1306.h
Others:      无
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
双车&信标   2020.6.1    儿童节版      兼容英飞凌
**************************************************/
#ifndef __SSD1306_H
#define __SSD1306_H

#include "OLED.h"

/************************************宏定义************************************/

/*********************************全局变量声明*********************************/
extern void OLED_Numbers(unsigned char x,unsigned char y,int Value);
extern void Image_Display(int kind);
extern void OrmFlash(unsigned char ModelFlash,unsigned char ParameterNum);
extern void OLED_Init(void);
extern void OLED_ShowStr(unsigned char x,unsigned char y,unsigned char ch[], unsigned char TextSize);
extern void OLED_Dat(unsigned char OLED_Cmd);
extern void OLED_SetPos(unsigned char x,unsigned char y);
extern void OLED_Fill(unsigned char fill_Data);

extern void before_image(void);

//extern unsigned char I2c_Read_OneByte(void);
/***********************************函数声明***********************************/

#endif
