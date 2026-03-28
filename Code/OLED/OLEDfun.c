/*************************************************
oled.fun.c
**************************************************/
#include "OLED.h"
#include "Flash.h"
#include <stdio.h>

uint8_t Oled_Mode = 0;
//extern uint8_t Run_Track;
float gfRoadOffset = 0;
float gfObstacleOffsetL = 0;
float gfObstacleOffsetR = 0;
float gfLoopOffset = 0;
short int Big_Entry = 0;
short int Big_Exit = 0;
short int Mid_Entry = 0;
short int Mid_Exit = 0;
short int Small_Entry = 0;
short int Small_Exit = 0;
short int giCurve_Redress = 0;
/*************************************************************************
*  函数名称：unsigned long KeyboardInput_with_Flash(unsigned char x,unsigned char y)
*  功能说明：键显输入且不存储
  * @param    x   ：  OLED显示坐标x
  * @param    y   ：  OLED显示坐标y
  *
*  函数返回：返回键显输入的数
*  修改时间：2020年6月6日
*************************************************************************/
int KeyboardInput(unsigned char x,unsigned char y) //（without Flash）
{
    int KeyCodeOld=0;
    int KeyCode=0;
    int KeyValue = 0;
    int KeyNumber=0;
    int RerturnKeyNumber = 0;
    int Time_One_Key_Confirmation=0;
    int One_Key_Confirmation=0;
 //   char num2str[8];
    while(KeyValue != KEY_ENTER && One_Key_Confirmation==0)
    {
        /*******************************按键识别*******************************/
        KeyCodeOld = KeyCode;
        KeyCode = CH455_Read();
        if(KeyCodeOld - KeyCode==0x40)
        {
            switch(KeyCode)
            {
                case 0x17 :     KeyValue = 1;   break;
                case 0x0f :     KeyValue = 2;   break;
                case 0x07 :     KeyValue = 3;   break;
                case 0x16 :     KeyValue = 4;   break;
                case 0x0e :     KeyValue = 5;   break;
                case 0x06 :     KeyValue = 6;   break;
                case 0x15 :     KeyValue = 7;   break;
                case 0x0d :     KeyValue = 8;   break;
                case 0x05 :     KeyValue = 9;   break;
                case 0x0c :     KeyValue = 0;   break;
                case 0x14 :     KeyValue = 10;  break;   //退格键
                case 0x04 :     KeyValue = 11;  break;   //确认键
                default:                        break;
            }
        }
        else
        {
            KeyValue = KEY_BLANK;
        }
        if(KeyCode==0x44)
        {
          Time_One_Key_Confirmation++;
          if(Time_One_Key_Confirmation > KEY_ENTER_TIME)
          {
            One_Key_Confirmation++;
          }
        }
        /****************************输入数字可退格****************************/

        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
            if(KeyNumber / 1000 != 0)
            {
                KeyNumber = KeyNumber % 1000;
            }
            KeyNumber = KeyNumber * 10 + KeyValue;
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            KeyNumber = KeyNumber / 10;
        }
			OLED_Numbers(x,y,KeyNumber);
    }
    RerturnKeyNumber = KeyNumber;
    return RerturnKeyNumber;
}

/*************************************************************************
*  函数名称：unsigned long KeyboardInput_with_Flash(unsigned char sector, unsigned short page, unsigned char x,unsigned char y)
*  功能说明：键显输入并存储到flash
  * @param    sector   ：  扇区   范围  0-11
  * @param    page   ：  页   范围  0-1023
  * @param    x   ：  OLED显示坐标x
  * @param    y   ：  OLED显示坐标y
*  函数返回：返回键显输入的数
*  修改时间：2020年6月6日
*************************************************************************/
unsigned long KeyboardInput_with_Flash(unsigned char sector, unsigned short page, unsigned char x,unsigned char y) // (with Flash)
{
	static uint8_t  firstDraw = 1;   // 1=还没按过数字
	static uint32_t flashBack = 0;   // 用来存Flash旧值
	
    int KeyCodeOld=0;              //键盘扫描
    int KeyCode=0;
    int KeyValue = 0;              //当次键入的值
    int KeyZero=0;                 //清零标志
    int KeyNumber=0;               //当前键入的值
 //   int KeyNumber_Flash = 0;       //Flash中的值
    unsigned long RerturnKeyNumber = 0;      //最终输出的值
    int One_Key_Confirmation=0;
    int Y_coordinate =0;   //参数所在Y值
    int Time_One_Key_Confirmation=0;
    char num2str[8];
    while(KeyValue != KEY_ENTER && One_Key_Confirmation==0)
    {
		if (firstDraw) 
		{                // 第一次进来
			firstDraw = 0;
			flashBack = FlashParam_Read(page);
			KeyNumber = flashBack;      // 先让屏幕显示旧值
			sprintf(num2str, "%5lu", KeyNumber);
			OLED_ShowStr(x, y, num2str, 2);
		}
		
        /*******************************按键识别*******************************/
        KeyCodeOld = KeyCode;
        KeyCode = CH455_Read();
        if(KeyCodeOld - KeyCode == 0x40)
        {
            switch(KeyCode)
            {
                case 0x17 :     KeyValue = 1;   break;
                case 0x0f :     KeyValue = 2;   break;
                case 0x07 :     KeyValue = 3;   break;
                case 0x16 :     KeyValue = 4;   break;
                case 0x0e :     KeyValue = 5;   break;
                case 0x06 :     KeyValue = 6;   break;
                case 0x15 :     KeyValue = 7;   break;
                case 0x0d :     KeyValue = 8;   break;
                case 0x05 :     KeyValue = 9;   break;
                case 0x0c :     KeyValue = 0;   break;
                case 0x14 :     KeyValue = KEY_BACK;  break;   //退格键
                case 0x04 :     KeyValue = KEY_ENTER;  break;   //确认键
                default:                        break;
            }
        }
        else
        {
            KeyValue = KEY_BLANK;
        }

        /****************************输入数字可退格****************************/

        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
			firstDraw = 0;                      // 标记“用户开始输入”
			
            if(KeyNumber / 1000 != 0)
            {
                KeyNumber = KeyNumber % 1000;
            }
            KeyNumber = KeyNumber * 10 + KeyValue;
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            KeyNumber = 0;
        }
        sprintf(num2str, "%5d", KeyNumber);
        OLED_ShowStr(x, y, num2str, 2);
		
		
		
		/***************************确认是否用Flash****************************/
		if(KeyNumber == 0)      // 输入数为 0 显示 Flash 中数
		{
			if(KeyValue == 0)
			{
				KeyZero = 1 ;
			}
			else if( KeyValue==10) // 退格
			{
				KeyZero = 0 ;
			}
			else if(KeyValue == KEY_ENTER)
			{
				if(KeyZero==0)     // 读 Flash
				{
					KeyNumber = FlashParam_Read(page);   // ← 直接读
					sprintf(num2str, "%5d", KeyNumber);
					OLED_ShowStr(x, y, num2str, 2);
				}
				else              // 写 0
				{
					FlashParam_Write(page, 0);           // ← 写 0
				}
			}
		}
		else                    // 显示实际数
		{
			if(KeyValue == KEY_ENTER)// 保存
			{
				FlashParam_Write(page, (uint32_t)KeyNumber); // ← 写当前值
			}
		}
    }

    RerturnKeyNumber = KeyNumber;

    return RerturnKeyNumber;
}

/*************************************************************************
*  函数名称：unsigned long KeyboardInput_with_Flash(unsigned char x,unsigned char y)
*  功能说明：键显输入且不存储
  * @param    x   ：  OLED显示坐标x
  * @param    y   ：  OLED显示坐标y
  *
*  函数返回：返回键显输入的数
*  修改时间：2020年6月6日
*************************************************************************/
int KeyboardInput_Confirm(unsigned char x,unsigned char y) //（without Flash）
{
    int KeyCodeOld=0;
    int KeyCode=0;
    int KeyValue = 0;
    int KeyNumber=0;
    int RerturnKeyNumber = 0;
    int Time_One_Key_Confirmation=0;
    int One_Key_Confirmation=0;
 //   char num2str[8];
    while(KeyValue != KEY_ENTER && One_Key_Confirmation==0)
    {
        /*******************************按键识别*******************************/
        KeyCodeOld = KeyCode;
        KeyCode = CH455_Read();
        if(KeyCodeOld - KeyCode==0x40)
        {
            switch(KeyCode)
            {
                case 0x17 :     KeyValue = 1;   break;
                case 0x0f :     KeyValue = 2;   break;
                case 0x07 :     KeyValue = 3;   break;
                case 0x16 :     KeyValue = 4;   break;
                case 0x0e :     KeyValue = 5;   break;
                case 0x06 :     KeyValue = 6;   break;
                case 0x15 :     KeyValue = 7;   break;
                case 0x0d :     KeyValue = 8;   break;
                case 0x05 :     KeyValue = 9;   break;
                case 0x0c :     KeyValue = 0;   break;
                case 0x14 :     KeyValue = 10;  break;   //退格键
                case 0x04 :     KeyValue = 11;  break;   //确认键
                default:                        break;
            }
        }
        else
        {
            KeyValue = KEY_BLANK;
        }
        if(KeyCode==0x44)
        {
          Time_One_Key_Confirmation++;
          if(Time_One_Key_Confirmation > KEY_ENTER_TIME)
          {
            One_Key_Confirmation++;
          }
        }
        /****************************输入数字可退格****************************/

        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
            if(KeyNumber / 1000 != 0)
            {
                KeyNumber = KeyNumber % 1000;
            }
            KeyNumber = KeyNumber * 10 + KeyValue;
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            KeyNumber = KeyNumber / 10;
        }
    }
    RerturnKeyNumber = KeyNumber;
    return RerturnKeyNumber;
}
