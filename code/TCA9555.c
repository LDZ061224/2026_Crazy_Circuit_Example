/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: TCA9555.c
Author: TEAM  A B C
Version:0.0               Date: 2026.1.27
Description:  TCA9555 I2C IO expander driver implementation
Others:
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.27   0.0        Initial
**************************************************/

#include "TCA9555.h"

soft_iic_info_struct TC9555_I2C_Struct;
TCA9555_LED_t LED[16] =  // 16 elements
{
    LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7,
    LED_8, LED_9, LED_10, LED_11, LED_12, LED_13, LED_14, LED_15
};

/******************************************************************/

/*************************************
** Function: TCA9555_Init
** Description: TCA9555 initialization
** Others: IO configured as output mode
*************************************/
void TCA9555_Init()
{
    uint8_t cmd, data;

    // Initialize I2C
    soft_iic_init(&TC9555_I2C_Struct, TCA9555_BASE_ADDR, 10, P21_7, P20_6);

    // Configure Port 0 as output
    cmd = TCA9555_REG_CONFIG_P0;  // Select configuration register 0
    data = 0x00;                  // 1=input, 0=output -> all configured as output

    // I2C write device addr -> command byte(0x06) -> data(0x00)
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);

    // Configure Port 1 as output
    cmd = TCA9555_REG_CONFIG_P1;  // Select configuration register 1
    data = 0x00;                  // All configured as output

    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);

    // Initialize output registers, all LEDs off
    cmd = TCA9555_REG_OUTPUT_P0;
    data = 0x00;  // High level turns off LED
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);

    cmd = TCA9555_REG_OUTPUT_P1;
    data = 0x00;  // High level turns off LED
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, cmd, data);
}

/*************************************
** Function: TCA9555_LED_Ctrl
** Description: TCA9555 LED control
** Others: pin is LED number, state is LED state 0=off 1=on
*************************************/
void TCA9555_LED_Ctrl(TCA9555_LED_t pin, int state)
{
    uint8_t reg_addr;      // Register address
    uint8_t current_data;  // Current register value
    uint8_t pin_index;     // Bit index within port (0-7)

    // Determine which port, calculate bit index
    if (pin < 8)
    {
        reg_addr = 0x02; // Output Port 0
        pin_index = pin; // P00-P07 correspond to bits 0-7
    }
    else if (pin < 16)
    {
        reg_addr = 0x03; // Output Port 1
        pin_index = pin - 8; // P10-P17 correspond to bits 0-7
    }
    else
    {
        return; // Invalid pin number
    }

    // Read current port state
    current_data = soft_iic_read_8bit_register(&TC9555_I2C_Struct, reg_addr);

    // Set state bit
    if (state == 0)
    {
        current_data &= ~(1 << pin_index); // Clear bit to turn on LED (active low)
    }
    else
    {
        current_data |= (1 << pin_index); // Set bit to turn off LED (active high)
    }

    // Write back to register
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, reg_addr, current_data);
}

/*************************************
** Function: TCA9555_Read_Input
** Description: Read input port status
** Others: port is port number 0=Port0, 1=Port1
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
** Description: Set polarity inversion
** Others: port is port number, polarity_mask is polarity mask
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
** Description: Turn on all LEDs
** Others: None
*************************************/
void TCA9555_All_LED_On(void)
{
    // Port 0 all bits cleared
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x02, 0x00);
    // Port 1 all bits cleared
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x03, 0x00);
}

/*************************************
** Function: TCA9555_All_LED_Off
** Description: Turn off all LEDs
** Others: None
*************************************/
void TCA9555_All_LED_Off(void)
{
    // Port 0 all bits set
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x02, 0xFF);
    // Port 1 all bits set
    soft_iic_write_8bit_register(&TC9555_I2C_Struct, 0x03, 0xFF);
}
