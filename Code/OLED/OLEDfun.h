/*************************************************
Copyright (C), 2016-2020, TYUT JBD TEAM C.
File name: OLEDKeyboard.h
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:  OLEDKeyboard.h
Others:      无
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
双车&信标   2020.6.1    儿童节版      兼容英飞凌
**************************************************/
#ifndef __OLEDFUN_H
#define __OLEDFUN_H

#include  "OLED.h"
#include  "cmsis_armcc.h"

//-----变量声明-----
extern uint8_t Oled_Mode;
//extern uint8_t Run_Track;
extern float gfRoadOffset;
extern float gfObstacleOffsetL;
extern float gfObstacleOffsetR;
extern float gfLoopOffset;
extern short int Big_Entry;
extern short int Big_Exit;
extern short int Mid_Entry;
extern short int Mid_Exit;
extern short int Small_Entry;
extern short int Small_Exit;
extern short int giCurve_Redress;

/****************************函数*********************************************/
int KeyboardInput(unsigned char x,unsigned char y);
unsigned long KeyboardInput_with_Flash(unsigned char sector, unsigned short page, unsigned char x,unsigned char y); // (with Flash)
	
#endif
