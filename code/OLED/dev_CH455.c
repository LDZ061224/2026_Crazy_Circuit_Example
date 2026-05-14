/*************************************************
Copyright (C), 2016-2024, TYUT JBD .
File name: CH455.c
Author: TYUT JBD
Version:1.0               Date: 2016.11.12
Description: CH455 键盘/数码管驱动芯片底层驱动
Others:      基于JBD模拟IIC通信库
Function List:
             1. CH455_Init        - 初始化CH455芯片
             2. CH455_Read        - 读取CH455按键寄存器
             3. CH455_GetOneKey   - 阻塞式获取单个按键事件
             4. CH455_QueryOneKey - 轮询式获取按键事件
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
**************************************************/

#include "dev_CH455.h"
#include "dev_ssd1306.h"

/*************************************
** Function: CH455_Init
** Description: 初始化CH455芯片，使能按键扫描
** Others:     IIC通信，发送系统命令0x01
*************************************/
void CH455_Init(void)
{
    JBD_simiic_Start();              // 发起IIC起始信号
    JBD_simiic_Write_OneByte(0x48);  // 发送设备写地址（0x48 = 0x24<<1）
    JBD_simiic_Wait_Ask();           // 等待从机应答
    JBD_simiic_Write_OneByte(0x01);  // 发送系统命令：使能
    JBD_simiic_Wait_Ask();           // 等待从机应答
    JBD_simiic_Stop();               // 发送IIC停止信号
}

/*************************************
** Function: CH455_Read
** Description: 读取CH455按键寄存器当前值
** Return:     按键代码（bit6为按键状态，bit4-bit0为键值）
** Others:     读地址0x4F，带NACK结束
*************************************/
unsigned char CH455_Read(void)
{
    unsigned char keycode;

    JBD_simiic_Start();              // 发起IIC起始信号
    JBD_simiic_Write_OneByte(0x4f);  // 发送设备读地址
    JBD_simiic_Wait_Ask();           // 等待从机应答
    keycode = JBD_simiic_Read_OneByte(1); // 读取1字节（1=发送NACK）
    JBD_simiic_Stop();               // 发送IIC停止信号

    return(keycode);
}

/*************************************************
** Function: CH455_GetOneKey
** Description: 阻塞式获取一个CH455按键抬起事件
** Details:    进入函数后持续轮询CH455寄存器，直到检测到
**             有效的按键抬起事件才返回。支持短按/长按区分。
**             - 短按：返回低4位键值
**             - 长按：返回高4位键值 | 0x0F
**             - 超时未检测到抬起：根据当前按下的键返回长按键值
** Return:     KeyValue_enum 类型按键值
*************************************************/
KeyValue_enum CH455_GetOneKey(void)
{
    uint8  KeyCodeOld = KEY_BLANK;       // 上一周期按键代码
    uint8  KeyCode    = CH455_Read();     // 当前按键代码
    uint16 KeyValue   = KEY_BLANK;        // 返回值，低4位=短按，高4位=长按
    uint16 timeout    = 15000;            // 超时计数器

    CH455_Init();  // 初始化CH455

    // 等待按键按下（按键按下时 bit6=1，键值>=0x40）
    do
    {
        KeyCodeOld = CH455_Read();
    } while(KeyCodeOld < 0x40);

    // 等待按键抬起，或超时
    while(KeyValue == KEY_BLANK && timeout > 0)
    {
        KeyCode = CH455_Read();

        // 检测按键抬起：按下值 - 抬起值 == 0x40
        // 按下时 bit6=1，抬起后 bit6=0，差值即为 0x40
        if(KeyCodeOld - KeyCode == 0x40)
        {
            switch(KeyCode)
            {
                case 0x17: KeyValue = KEY_1_;    break;
                case 0x0f: KeyValue = KEY_2_;    break;
                case 0x07: KeyValue = KEY_3_;    break;
                case 0x16: KeyValue = KEY_4_;    break;
                case 0x0e: KeyValue = KEY_5;     break;
                case 0x06: KeyValue = KEY_6;     break;
                case 0x15: KeyValue = KEY_7;     break;
                case 0x0d: KeyValue = KEY_8;     break;
                case 0x05: KeyValue = KEY_9;     break;
                case 0x0c: KeyValue = KEY_0;     break;
                case 0x14: KeyValue = KEY_BACK;  break;  // 退格键
                case 0x04: KeyValue = KEY_ENTER; break;  // 确认键
                default:                         break;
            }
        }

        timeout--;
    }

    // 短按但超时较短，判定为长按
    if(KeyValue != KEY_BLANK && timeout <= 100)
    {
        KeyValue <<= 4;     // 移到高4位
        KeyValue |= 0x0F;   // 标记为长按
    }

    // 超时未检测到抬起：根据当前仍按下的键返回长按值
    if(KeyValue == KEY_BLANK && timeout == 0)
    {
        switch(KeyCode)
        {
            case 0x57: KeyValue = KEY_1_Long;    break;
            case 0x4f: KeyValue = KEY_2_Long;    break;
            case 0x47: KeyValue = KEY_3_Long;    break;
            case 0x56: KeyValue = KEY_4_Long;    break;
            case 0x4e: KeyValue = KEY_5_Long;    break;
            case 0x46: KeyValue = KEY_6_Long;    break;
            case 0x55: KeyValue = KEY_7_Long;    break;
            case 0x4d: KeyValue = KEY_8_Long;    break;
            case 0x45: KeyValue = KEY_9_Long;    break;
            case 0x4c: KeyValue = KEY_0_Long;    break;
            case 0x54: KeyValue = KEY_BACK_Long; break;  // 退格键
            case 0x44: KeyValue = KEY_ENTER_Long;break;  // 确认键
            default:                             break;
        }
    }

    return KeyValue;
}

/*************************************************
** Function: CH455_QueryOneKey
** Description: 轮询式获取一个CH455按键事件（非阻塞）
** Details:    每次调用与CH455通信一次，记录按键代码并与
**             历史值对比，检测按键抬起/长按事件。
**             状态机逻辑：
**             1. ValuableKeyCode == KEY_BLANK：等待按键按下
**             2. ValuableKeyCode != KEY_BLANK：等待按键抬起或超时
** Input:      timeout - 长按判定阈值（计数次数）
** Return:     - 按键按下中：返回 0xFF - ValuableKeyCode
**             - 短按抬起：  返回 ValuableKeyCode
**             - 长按/超时： 返回 (ValuableKeyCode << 4) | 0x0F
*************************************************/
KeyValue_enum CH455_QueryOneKey(uint32 timeout)
{
    static uint8 ValuableKeyCode = KEY_BLANK;  // 当前有效按键值
    static uint8 LastKeyCode     = KEY_BLANK;  // 上一周期按键代码
    static uint16 Count          = 0;          // 按键持续计数

    uint8 ThisKeyCode = CH455_Read();  // 读取当前按键寄存器

    //===== 状态1：当前无有效按键，等待按下 =====
    if(KEY_BLANK == ValuableKeyCode)
    {
        if(ThisKeyCode <= 0x40)
        {
            return KEY_BLANK;  // 无按键按下
        }
        else
        {
            // 按键按下：根据键值映射有效按键
            switch(ThisKeyCode)
            {
                case 0x57: ValuableKeyCode = KEY_1_;    break;
                case 0x4f: ValuableKeyCode = KEY_2_;    break;
                case 0x47: ValuableKeyCode = KEY_3_;    break;
                case 0x56: ValuableKeyCode = KEY_4_;    break;
                case 0x4e: ValuableKeyCode = KEY_5;     break;
                case 0x46: ValuableKeyCode = KEY_6;     break;
                case 0x55: ValuableKeyCode = KEY_7;     break;
                case 0x4d: ValuableKeyCode = KEY_8;     break;
                case 0x45: ValuableKeyCode = KEY_9;     break;
                case 0x4c: ValuableKeyCode = KEY_0;     break;
                case 0x54: ValuableKeyCode = KEY_BACK;  break;  // 退格键
                case 0x44: ValuableKeyCode = KEY_ENTER; break;  // 确认键
                default:   ValuableKeyCode = KEY_BLANK; break;
            }

            Count = 0;
            LastKeyCode = ThisKeyCode;

            return 0xFF - ValuableKeyCode;  // 返回"按下中"状态码
        }
    }
    //===== 状态2：已有有效按键，等待抬起或超时 =====
    else
    {
        // 本次查询到按键抬起
        if(ThisKeyCode < 0x40 && LastKeyCode - ThisKeyCode == 0x40)
        {
            uint8 temp;

            if(Count > timeout * 0.95)  // 长按判定
            {
                temp = (ValuableKeyCode << 4) | 0x0F;
            }
            else  // 短按
            {
                temp = ValuableKeyCode;
            }

            // 重置状态
            Count = 0;
            LastKeyCode = KEY_BLANK;
            ValuableKeyCode = KEY_BLANK;

            return temp;
        }
        else  // 按键仍在按下状态
        {
            uint8 temp;

            Count++;
            LastKeyCode = ThisKeyCode;

            if(Count > timeout)  // 超时强制转为长按
            {
                temp = (ValuableKeyCode << 4) | 0x0F;
            }
            else
            {
                temp = 0xFF - ValuableKeyCode;  // 继续返回"按下中"
            }

            return temp;
        }
    }
}
