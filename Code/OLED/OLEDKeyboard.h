/*************************************************
Copyright (C), 2016-2020, TYUT JBD TEAM C.
File name: OLEDKeyboard.h
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:  OLEDKeyboard.h
Others:      ??
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        ???
AmaZzzing 2016.11.12  1.0        ??????????
???&???   2020.6.1    ??????      ?????????
**************************************************/
#ifndef __OLEDKEYBOARD_H
#define __OLEDKEYBOARD_H

#include  "OLED.h"
#define ROW 15//???60
#define COL 30//???120
/*********************************??????????*********************************/
extern uint8_t map[3][ROW][COL];
extern uint8_t Image_dis[64][120];
extern int finish;
extern unsigned char Oled_Model_Choose;
extern int Completed_Quantity[3];
extern int Box_Quantity[3];

extern int L_MAX;
extern int R_MAX;
extern int L_MIN;
extern int R_MIN;
extern int L_THR;
extern int R_THR;
extern uint8_t Blue_Mode;
extern uint8_t Mileage_Test;	

/***********************************????????***********************************/
extern void Oled_Input(void);
void creat_map(void);
void fill_screen(int kind);
void up(int kind);
void left(int kind);
void right(int kind);
void down(int kind);
void Recovery(void);
void Oled_Data_Load(void);
void Oled_Show(void);
int KeyboardInput_Confirm(unsigned char x,unsigned char y); //??without Flash??


#endif

