/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: TCA9555.h
Author: TEAM  A B C
Version:0.0               Date: 2026.1.27
Description:  TCA9555 I2C IO expander driver header
Others:      None
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.27   0.0        Initial
**************************************************/
/*
* Chip model: TCA9555
* Core function: 16-bit I2C to parallel port expander, provides remote I/O expansion for MCUs.
*/
#ifndef __TCA9555_H
#define __TCA9555_H

#include "zf_common_headfile.h"
#include "headfiles.h"

/***********************************Macro Definitions***********************************/
#define TCA9555_BASE_ADDR      0x20  // Device write address (A0 A1 A2 all low)
#define TCA9555_REG_INPUT_P0   0x00  // Input port 0
#define TCA9555_REG_INPUT_P1   0x01  // Input port 1
#define TCA9555_REG_OUTPUT_P0  0x02  // Output port 0
#define TCA9555_REG_OUTPUT_P1  0x03  // Output port 1
#define TCA9555_REG_POLARITY_P0 0x04  // Polarity inversion port 0
#define TCA9555_REG_POLARITY_P1 0x05  // Polarity inversion port 1
#define TCA9555_REG_CONFIG_P0  0x06  // Configuration port 0
#define TCA9555_REG_CONFIG_P1  0x07  // Configuration port 1

typedef enum {
    // --- Port 0 (P00 - P07) ---
    LED_0,
    LED_1,
    LED_2,
    LED_3,
    LED_4,
    LED_5,
    LED_6,
    LED_7,

    // --- Port 1 (P10 - P17) ---
    LED_8,
    LED_9,
    LED_10,
    LED_11,
    LED_12,
    LED_13,
    LED_14,
    LED_15,

    LED_ALL  // Special value, usable for all-on / all-off operations

} TCA9555_LED_t;

/*********************************Global Variable Declarations*********************************/
extern TCA9555_LED_t LED[16];

/***********************************Function Declarations***********************************/
extern void TCA9555_Init();
extern void TCA9555_LED_Ctrl(TCA9555_LED_t pin, int state);
extern uint8_t TCA9555_Read_Input(uint8_t port);
extern void TCA9555_Set_Polarity(uint8_t port, uint8_t polarity_mask);
extern void TCA9555_All_LED_On(void);
extern void TCA9555_All_LED_Off(void);

#endif
