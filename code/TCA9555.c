/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: OLEDKeyboard.h
Author: TEAM  A B C
Version:0.0               Date: 2026.1.27
Description:  TCA9555.c
Others:
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.27   0.0        ʼ
**************************************************/

#include "TCA9555.h"

soft_iic_info_struct TC9555_I2C_Struct;
TCA9555_LED_t LED[16] =  // 修正为16个元素
{
    LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7,
    LED_8, LED_9, LED_10, LED_11, LED_12, LED_13, LED_14, LED_15
};

/******************************************************************/

/*************************************
** Function: TCA9555_Init
** Description: TCA9555初始化
** Others: IO配置为输出模式
*************************************/
void TCA9555_Init()
{
    uint8_t cmd, data;

    // 初始化IIC
    soft_iic_init(&TC9555_I2C_Struct, TCA9555_BASE_ADDR, 10, P21_7, P20_6);

    // 配置 Port 0 为输出
    cmd = TCA9555_REG_CONFIG_P0;  // 选择配置寄存器0
    data = 0x00;                  // 1=输入, 0=输出 -> 全部配置为输出

    // I2C写设备地址 -> 命令字节(0x06) -> 数据(0x00)
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);

    // 配置 Port 1 为输出
    cmd = TCA9555_REG_CONFIG_P1;  // 选择配置寄存器1
    data = 0x00;                  // 全部配置为输出

    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);

    // 初始化输出寄存器，所有LED熄灭
    cmd = TCA9555_REG_OUTPUT_P0;
    data = 0x00;  // 高电平熄灭LED
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);

    cmd = TCA9555_REG_OUTPUT_P1;
    data = 0x00;  // 高电平熄灭LED
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);
}

/*************************************
** Function: TCA9555_LED_Ctrl
** Description: TCA9555 LED控制
** Others: pin为LED编号，state为LED状态 0-熄灭 1-点亮
*************************************/
void TCA9555_LED_Ctrl(TCA9555_LED_t pin, int state)
{
    uint8_t reg_addr;      // 寄存器地址
    uint8_t current_data;  // 当前寄存器值
    uint8_t pin_index;     // 端口内的位索引 (0-7)

    // 判断属于哪个端口，计算位索引
    if (pin < 8)
    {
        reg_addr = 0x02; // Output Port 0
        pin_index = pin; // P00-P07 对应位 0-7
    }
    else if (pin < 16)
    {
        reg_addr = 0x03; // Output Port 1
        pin_index = pin - 8; // P10-P17 对应位 0-7
    }
    else
    {
        return; // 无效的pin编号
    }

    // 读取当前端口状态
    current_data = soft_iic_read_8bit_register(&TC9555_I2C_Struct, reg_addr);

    // 设置状态位
    if (state == 0)
    {
        current_data &= ~(1 << pin_index); // 置0点亮LED（低电平有效）
    }
    else
    {
        current_data |= (1 << pin_index); // 置1熄灭LED（高电平有效）
    }

    // 写回寄存器
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, reg_addr, current_data);
}

/*************************************
** Function: TCA9555_Read_Input
** Description: 读取输入端口状态
** Others: port为端口号 0-Port0, 1-Port1
*************************************/
uint8_t TCA9555_Read_Input(uint8_t port)
{
    uint8_t reg_addr;

    if (port == 0)
    {
        reg_addr = TCA9555_REG_INPUT_P0;
    }
    else
    {
        reg_addr = TCA9555_REG_INPUT_P1;
    }

    return soft_iic_read_8bit_register(&TC9555_I2C_Struct, reg_addr);
}

/*************************************
** Function: TCA9555_Set_Polarity
** Description: 设置极性反转
** Others: port为端口号，polarity_mask为极性掩码
*************************************/
void TCA9555_Set_Polarity(uint8_t port, uint8_t polarity_mask)
{
    uint8_t cmd;

    if (port == 0)
    {
        cmd = TCA9555_REG_POLARITY_P0;
    }
    else
    {
        cmd = TCA9555_REG_POLARITY_P1;
    }

    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, polarity_mask);
}

/*************************************
** Function: TCA9555_All_LED_On
** Description: 点亮所有LED
** Others: 无
*************************************/
void TCA9555_All_LED_On(void)
{
    // Port 0 所有位清零
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x02, 0x00);
    // Port 1 所有位清零
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x03, 0x00);
}

/*************************************
** Function: TCA9555_All_LED_Off
** Description: 熄灭所有LED
** Others: 无
*************************************/
void TCA9555_All_LED_Off(void)
{
    // Port 0 所有位置1
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x02, 0xFF);
    // Port 1 所有位置1
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x03, 0xFF);
}
