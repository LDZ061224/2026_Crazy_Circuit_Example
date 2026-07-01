/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Fun.h
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description:  Function driver declaration header
Others:      None
Function List:
             1. Union type, peripheral pin macro definitions
             2. Global external variable declarations
             3. Driver function declarations (ADC, motor, encoder, OLED, etc.)
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30   0.0        Initial version
**************************************************/

// Header guard
#ifndef __FUN_H
#define __FUN_H

// Include base common header
#include "zf_common_headfile.h"
// Include global macros and common header
#include "headfiles.h"

/*********************************** Data Type Definitions ***********************************/
// Float <-> 4-byte array union
// Used for UART float transmission (split float into 4 uint8_t bytes)
typedef union floatu8data
{
    float floatdata;       // Float type data
    uint8 u8data[4];       // Corresponding 4-byte data
}floatu8data;

/*********************************** Hardware Pin Macros ***********************************/
/*
 *  New car PWM drive method:
 *    _PWM = duty cycle channel
 *    _DIR = direction channel (PWM, 10000=forward/suction, 0=reverse)
 */
#define Left_Motor_PWM      ATOM3_CH1_P15_7
#define Left_Motor_DIR      ATOM3_CH0_P15_5
#define Right_Motor_PWM     ATOM1_CH3_P00_4
#define Right_Motor_DIR     ATOM1_CH5_P00_6
#define Suction_Motor_PWM   ATOM1_CH6_P00_7
#define Suction_Motor_DIR   ATOM3_CH3_P00_12

// Enable switch: P20_7 high = enabled
#define EnableSwitch_ON     gpio_get_level(P20_7) == 1

/********************************* Global External Variable Declarations *********************************/
// 15-channel light sensor raw ADC value array
extern uint16 Light_ADC[15];
// 15-channel light sensor threshold array [0]=upper threshold [1]=lower threshold
extern float Light_Thr[15][2];
// IMU660RB sensor status flag
extern uint8 imu660rb_Check;
// Gyro Z-axis angular velocity
extern float Gyro_Z;
// Current detection value
extern float Current_Check;
// Two-channel voltage detection values
extern float Voltage_Check[2];

/*********************************** Function Declarations ***********************************/
// VOFA host software data transmit function
void Vofa_Send_Data(void);
void Vofa_Send_Flash_Data(void);    // VOFA Flash data export
// Light sensor ADC initialization function
void Light_Init(void);
// Encoder initialization function
void Encoder_Init(void);
// Motor PWM initialization function
void Motor_Init(void);
// Other peripheral initialization (OLED, GPIO, IMU660RB)
void Other_Init(void);
// Get 15-channel light sensor ADC values
void Get_Light(void);
// Get light sensor thresholds (auto-calibration)
void Get_Threshold(void);

#endif
