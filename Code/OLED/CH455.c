/*************************************************
Copyright (C), 2016-2017, TYUT JBD TEAM C.
File name: CH455.c
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:    CH455
Others:      无
Function List:    1.CH450_Write
                  2.CH450_Read
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0      初始
**************************************************/
#include "OLED.h"

void CH455_Init(void)
{
    I2c_Start();
    I2c_Write_OneByte(0x48);
    I2C_Wait_Ask();
    I2c_Write_OneByte(0x01);
    I2C_Wait_Ask();
    I2c_Stop();
}

unsigned char CH455_Read(void)
{
    unsigned char keycode;
    I2c_Start();
    I2c_Write_OneByte(0x4f);
    I2C_Wait_Ask();
    keycode=I2c_Read_OneByte(1);
    I2c_Stop();   //修改
    return(keycode);
}

KeyValue_enum CH455_GetOneKey(void)
{
    uint8_t  KeyCodeOld = KEY_BLANK;
    uint8_t  KeyCode = CH455_Read();
    uint16_t KeyValue = KEY_BLANK;  //短按时记录在低4位 长按记录在高4位
    uint16_t timeout = 15000;

    CH455_Init();

    do{
        KeyCodeOld = CH455_Read();
      }while(KeyCodeOld < 0x40); /*等待按键按下*/

    while(KeyValue == KEY_BLANK && timeout > 0)
    {
        KeyCode = CH455_Read();

        if(KeyCodeOld - KeyCode == 0x40)    //检测到按键抬起
        {
            switch(KeyCode)
            {
                case 0x17 :     KeyValue = KEY_1;       break;
                case 0x0f :     KeyValue = KEY_2;       break;
                case 0x07 :     KeyValue = KEY_3;       break;
                case 0x16 :     KeyValue = KEY_4;       break;
                case 0x0e :     KeyValue = KEY_5;       break;
                case 0x06 :     KeyValue = KEY_6;       break;
                case 0x15 :     KeyValue = KEY_7;       break;
                case 0x0d :     KeyValue = KEY_8;       break;
                case 0x05 :     KeyValue = KEY_9;       break;
                case 0x0c :     KeyValue = KEY_0;       break;
                case 0x14 :     KeyValue = KEY_BACK;    break;   //退格键
                case 0x04 :     KeyValue = KEY_ENTER;   break;   //确认键
                default:                                break;
            }
        }

        timeout--;
    }

    if(KeyValue != KEY_BLANK && timeout <= 100)
    {
        KeyValue <<= 4;
        KeyValue |= 0x0F;
    }

    if(KeyValue == KEY_BLANK && timeout == 0)  //由于超时跳出循环
    {
        /*根据当前按下的键返回数值*/
        switch(KeyCode)
        {
            case 0x57 :     KeyValue = KEY_1_Long;       break;
            case 0x4f :     KeyValue = KEY_2_Long;       break;
            case 0x47 :     KeyValue = KEY_3_Long;       break;
            case 0x56 :     KeyValue = KEY_4_Long;       break;
            case 0x4e :     KeyValue = KEY_5_Long;       break;
            case 0x46 :     KeyValue = KEY_6_Long;       break;
            case 0x55 :     KeyValue = KEY_7_Long;       break;
            case 0x4d :     KeyValue = KEY_8_Long;       break;
            case 0x45 :     KeyValue = KEY_9_Long;       break;
            case 0x4c :     KeyValue = KEY_0_Long;       break;
            case 0x54 :     KeyValue = KEY_BACK_Long;    break;   //退格键
            case 0x44 :     KeyValue = KEY_ENTER_Long;   break;   //确认键
            default:                                     break;
        }
    }
 
    return KeyValue;
}