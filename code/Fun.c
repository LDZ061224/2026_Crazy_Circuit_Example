/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Fun.c
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description: Peripheral driver, data acquisition, initialization functions
Others:      None
Function List:
              1. Vofa_Send_Data    - Send debug data to VOFA host software
              2. Wit_Send_Data     - Send sensor data via wireless module
              3. Light_Init        - Light sensor ADC initialization
              4. Encoder_Init      - Encoder initialization
              5. Motor_Init        - Motor PWM initialization
              6. Other_Init        - Other peripherals initialization
              7. Get_Light         - Read light sensor ADC values
              8. Get_Threshold     - Calculate light sensor thresholds
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30   0.0        Initial version
**************************************************/

#include "Fun.h"

/********************************* Global Variable Definitions *********************************/
uint8 imu660rb_Check = 0;                // IMU660RB sensor initialization status flag
uint16 Light_ADC[15] = {0};              // 15-channel light sensor raw ADC values
float Current_Check = 0;                 // Current detection value
float Voltage_Check[2] = {0};            // Two-channel voltage detection values

/* --------------- Light Sensor Calibration Parameters --------------- */
int Light_Raw_Min[15] =                  // 15-channel light sensor ADC minimum values (initialized to max)
{
    4096, 4096, 4096, 4096, 4096,
    4096, 4096, 4096, 4096, 4096,
    4096, 4096, 4096, 4096, 4096
};
int Light_Raw_Max[15] = {0};             // 15-channel light sensor ADC maximum values (initialized to 0)
float  Light_Thr[15][2];                 // 15-channel light sensor upper and lower thresholds

/********************************* Function Implementations *********************************/

/*************************************
** Function: Vofa_Send_Data
** Description: Send debug data to VOFA host software
** Input:      None
** Output:     None
** Return:     None
** Others:     UART_2 transmit, follows VOFA protocol frame format
*************************************/
void Vofa_Send_Data(void)
{
    floatu8data VOFA_data[20];
    uint8 frame[18 * 4 + 4];
    memset(VOFA_data, 0, sizeof(VOFA_data));

    // Common: wheel speeds (ch0~3)
     VOFA_data[0].floatdata  = Left_Exp_Spd;
     VOFA_data[1].floatdata  = Right_Exp_Spd;
     VOFA_data[2].floatdata  = Left_Real_Spd;
     VOFA_data[3].floatdata  = Right_Real_Spd;

#if USE_DEBUG_MODE
    switch (Debug_Sub_Mode)
    {
        case Debug_Sub_PI_Tuning: // ── right-wheel speed PI internals ──
            /* ch0 */ VOFA_data[0].floatdata  = Right_Exp_Spd;      // expected speed
            /* ch1 */ VOFA_data[1].floatdata  = Right_Real_Spd;     // actual speed
            /* ch2 */ VOFA_data[2].floatdata  = Right_PID.set;      // PID setpoint
            /* ch3 */ VOFA_data[3].floatdata  = Right_PID.err3[0];  // current error
            /* ch4 */ VOFA_data[4].floatdata  = Right_PID.pOut;     // P output
            /* ch5 */ VOFA_data[5].floatdata  = Right_PID.iOut;     // I output
            /* ch6 */ VOFA_data[6].floatdata  = Right_PID.dOut;     // D output
            /* ch7 */ VOFA_data[7].floatdata  = Right_PID.out;      // total PID output
            /* ch8 */ VOFA_data[8].floatdata  = Right_PID_Out;      // PWM duty written
            /* ch9 */ VOFA_data[9].floatdata  = Debug_Target_Speed;
            /* ch10*/ VOFA_data[10].floatdata = Voltage_Check[0];
            /* ch11*/ VOFA_data[11].floatdata = Right_PID.kp;
            /* ch12*/ VOFA_data[12].floatdata = Right_PID.ki;
            /* ch13*/ VOFA_data[13].floatdata = Right_PID.iOutMax;
            /* ch14*/ VOFA_data[14].floatdata = Right_PID.outMax;
            /* ch15*/ VOFA_data[15].floatdata = Debug_Motor_Enable;
            /* ch16*/ VOFA_data[16].floatdata = Debug_Which_Wheel;
            /* ch17*/ VOFA_data[17].floatdata = 0;
            break;

        case Debug_Sub_Ground_Test: // ── ground test (spin in place) ──
            VOFA_data[4].floatdata  = Gyro_Z;
            VOFA_data[5].floatdata  = Debug_Target_Speed;
            VOFA_data[6].floatdata  = Voltage_Check[0];
            VOFA_data[7].floatdata  = Debug_Ground_Dir;
            VOFA_data[8].floatdata  = Left_PID_Out;
            VOFA_data[9].floatdata  = Right_PID_Out;
            VOFA_data[10].floatdata = Debug_Motor_Enable;
            VOFA_data[11].floatdata = Debug_Fan_Duty;
            VOFA_data[12].floatdata = Gyro_Integral;
            break;

        case Debug_Sub_Angle: // ── angle / gyro rate tuning ──
            VOFA_data[4].floatdata  = Gyro_Z;
            VOFA_data[5].floatdata  = Debug_Angle_Vel_Target;
            VOFA_data[6].floatdata  = Debug_Angle_Mode;
            VOFA_data[7].floatdata  = Debug_Angle_Vel_Real;
            VOFA_data[8].floatdata  = Left_PID_Out;
            VOFA_data[9].floatdata  = Right_PID_Out;
            VOFA_data[12].floatdata = Gyro_Integral;
            VOFA_data[13].floatdata = Angle_PID.kp;
            VOFA_data[14].floatdata = Angle_PID.kd;
            VOFA_data[15].floatdata = Gyro_PID.kp;
            VOFA_data[16].floatdata = Gyro_PID.ki;
            VOFA_data[17].floatdata = Gyro_PID.kd;
            break;

        case Debug_Sub_NormalTrace: // ── normal trace debug ──
            VOFA_data[4].floatdata  = Gyro_Z;
            VOFA_data[5].floatdata  = Error;
            VOFA_data[6].floatdata  = Turn_PID_Out;
            VOFA_data[8].floatdata  = Left_PID_Out;
            VOFA_data[9].floatdata  = Right_PID_Out;
            VOFA_data[11].floatdata = Gyro_Integral;
            VOFA_data[13].floatdata = Debug_Target_Speed;
            break;

        default:
            break;
    }
#else
    /* Build mode: standard telemetry */
    VOFA_data[4].floatdata  = Gyro_Z;
    VOFA_data[5].floatdata  = Total_Run_Mileage;
    VOFA_data[6].floatdata  = Voltage_Check[0];
    VOFA_data[7].floatdata  = Count.Mileage;
    VOFA_data[8].floatdata  = Left_PID_Out;
    VOFA_data[9].floatdata  = Right_PID_Out;
    VOFA_data[10].floatdata = Current_Check;
    VOFA_data[11].floatdata = Error;
    VOFA_data[12].floatdata = Gyro_Integral;
    VOFA_data[13].floatdata = Debug_Angle_Vel_Target;
    VOFA_data[14].floatdata = Total_Angle;
    VOFA_data[15].floatdata = Build_Action_Index;
    VOFA_data[16].floatdata = Run_Mode;
    VOFA_data[17].floatdata = Count.Straight;
#endif

    int i;
    for(i = 0; i < 18; i++)
    {
        frame[i * 4 + 0] = VOFA_data[i].u8data[0];
        frame[i * 4 + 1] = VOFA_data[i].u8data[1];
        frame[i * 4 + 2] = VOFA_data[i].u8data[2];
        frame[i * 4 + 3] = VOFA_data[i].u8data[3];
    }

    // VOFA protocol fixed frame tail 00 00 80 7F
    frame[72] = 0x00;
    frame[73] = 0x00;
    frame[74] = 0x80;
    frame[75] = 0x7f;

    uart_write_buffer(UART_2, frame, sizeof(frame));
}

/*************************************
** Function: Vofa_Send_Flash_Data
** Description: VOFA Flash mileage data export -- send all valid mileage values in one frame
** Details:    Order: Turn_Mileage_Record[0..N-1] + segment edge mileages
**             One frame contains all data, cycles through
*************************************/
void Vofa_Send_Flash_Data(void)
{
    floatu8data vofa[50];
    uint8 frame[50 * 4 + 4];
    memset(vofa, 0, sizeof(vofa));

    uint16_t pos = 0;

    // Segment total mileages
    for (uint8_t r = 0; r < TRACK_SEGMENT_NUM_MAX && pos < 50; r++, pos++)
    {
        vofa[pos].floatdata = Segment_Total_Mileage[r];
    }
    // Segment edge mileages — 2D array: [segment_row][element_index]
    for (uint8_t r = 0; r < TRACK_SEGMENT_NUM_MAX && pos < 50; r++)
    {
        for (uint8_t c = 0; c < ELEMENT_NUM_MAX && pos < 50; c++, pos++)
        {
            vofa[pos].floatdata = Segment_Edge_Mileage_Record[r][c];
        }
    }

    // Send all valid data at once
    for (uint16_t i = 0; i < pos; i++)
    {
        frame[i * 4 + 0] = vofa[i].u8data[0];
        frame[i * 4 + 1] = vofa[i].u8data[1];
        frame[i * 4 + 2] = vofa[i].u8data[2];
        frame[i * 4 + 3] = vofa[i].u8data[3];
    }
    frame[pos * 4 + 0] = 0x00;
    frame[pos * 4 + 1] = 0x00;
    frame[pos * 4 + 2] = 0x80;
    frame[pos * 4 + 3] = 0x7f;
    uart_write_buffer(UART_0, frame, pos * 4 + 4);
}

/*************************************
** Function: Wit_Send_Data
** Description: Send sensor data via wireless module
** Input:      None
** Output:     None
** Return:     None
** Others:     UART_3 transmit, custom data frame format
*************************************/
void Wit_Send_Data(void)
{
    uint8 data[4];                       // 4-byte buffer for float decomposition
    floatu8data VOFA_data[20];           // Union for float and byte array conversion
    // Clear data buffer
    memset(VOFA_data, 0, sizeof(VOFA_data));

    int i = 0;

    // Assign 15-channel processed light sensor data
    for (int i = 0; i < 15; i++)
    {
        VOFA_data[i].floatdata = Light_Convert[i];
    }

    // Assign status parameters
    VOFA_data[15].floatdata = Run_Mode;   // Run mode
    VOFA_data[16].floatdata = Error;      // Error code
    VOFA_data[17].floatdata = 0;          // Reserved

    // Send data frame header 00 00 7F 80
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x7f);
    uart_write_byte(UART_3, 0x80);

    // Send 18 groups of float data in a loop
    for(i = 0; i < 18; i++)
    {
        // Extract the 4 bytes from the split float
        data[0] = VOFA_data[i].u8data[0];
        data[1] = VOFA_data[i].u8data[1];
        data[2] = VOFA_data[i].u8data[2];
        data[3] = VOFA_data[i].u8data[3];

        // Send byte by byte via UART_3
        uart_write_byte(UART_3, data[0]);
        uart_write_byte(UART_3, data[1]);
        uart_write_byte(UART_3, data[2]);
        uart_write_byte(UART_3, data[3]);
    }

    // Send data frame tail 00 00 80 7F
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x80);
    uart_write_byte(UART_3, 0x7f);

    return;
}

/*************************************
** Function: Light_Init
** Description: Initialize 15-channel light sensor ADCs
** Input:      None
** Output:     None
** Return:     None
** Others:     Configured for 12-bit ADC precision
*************************************/
void Light_Init()
{
    // Initialize ADC channels for 15 light sensors, 12-bit precision
    adc_init(ADC2_CH14_A48, ADC_12BIT);
    adc_init(ADC2_CH12_A46, ADC_12BIT);
    adc_init(ADC2_CH10_A44, ADC_12BIT);
    adc_init(ADC2_CH6_A38,  ADC_12BIT);
    adc_init(ADC2_CH4_A36,  ADC_12BIT);
    adc_init(ADC1_CH9_A25,  ADC_12BIT);
    adc_init(ADC1_CH5_A21,  ADC_12BIT);
    adc_init(ADC1_CH1_A17,  ADC_12BIT);
    adc_init(ADC0_CH13_A13, ADC_12BIT);
    adc_init(ADC0_CH11_A11, ADC_12BIT);
    adc_init(ADC0_CH8_A8,   ADC_12BIT);
    adc_init(ADC0_CH6_A6,   ADC_12BIT);
    adc_init(ADC0_CH4_A4,   ADC_12BIT);
    adc_init(ADC0_CH2_A2,   ADC_12BIT);
    adc_init(ADC0_CH0_A0,   ADC_12BIT);
}

/*************************************
** Function: Encoder_Init
** Description: Initialize left and right motor encoders
** Input:      None
** Output:     None
** Return:     None
** Others:     Quadrature encoder mode
*************************************/
void Encoder_Init()
{
    // Left motor encoder initialization
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    // Right motor encoder initialization (TIM3->TIM2: P02_7 wire broken, switched to P33_6/P33_7)
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
}

/*************************************
** Function: Motor_Init
** Description: Initialize motor driver PWM
** Input:      None
** Output:     None
** Return:     None
** Others:     Left/right motors 30KHz, suction motor 70KHz
*************************************/
void Motor_Init()
{
    // DIR PWM: Left 10000=forward/0=rev, Right 0=forward/10000=rev, 30kHz
    pwm_init(Left_Motor_DIR,  30000, 10000);
    pwm_init(Right_Motor_DIR, 30000, 0);
    // duty PWM: 30kHz, start at 0
    pwm_init(Left_Motor_PWM,  30000, 0);
    pwm_init(Right_Motor_PWM, 30000, 0);
    // Fan: initial full duty, run at 3000
    pwm_init(Suction_Motor_DIR, 100000, 10000);
    pwm_init(Suction_Motor_PWM, 100000, 10000);

    system_delay_ms(10);
}

/*************************************
** Function: Other_Init
** Description: Initialize other peripherals
** Input:      None
** Output:     None
** Return:     None
** Others:     Includes OLED, IMU660RB, GPIO initialization
*************************************/
void Other_Init()
{
    OLED_Init();                                  // OLED display initialization
//    gpio_init(P15_1,    GPO, 0, GPO_PUSH_PULL);
    gpio_init(P33_4,    GPO, 0, GPO_PUSH_PULL);   // Push-pull output GPIO initialization
    gpio_init(P20_7,    GPI, 0, GPI_PULL_DOWN);   // Enable switch input
    gpio_init(P22_3,    GPI, 0, GPI_PULL_DOWN);   // Custom button input
    adc_init(ADC0_CH5_A5,   ADC_12BIT); // Battery voltage detection ADC
    adc_init(ADC0_CH10_A10, ADC_12BIT); // Battery current detection ADC
}

/*************************************
** Function: Get_Light
** Description: Read 15-channel light sensor ADCs, battery voltage and current
** Input:      None
** Output:     Light_ADC array, Voltage_Check, Current_Check
** Return:     None
** Others:     Array index 0~14 corresponds to sensors 1~15
**             Voltage formula: raw * 3.3 * 11 / 4095 + 0.567, with first-order low-pass filter
**             Current formula: raw * 3.3 / (4095 * 20 * 0.015)
*************************************/
void Get_Light()
{
    // Read 15-channel ADC values into corresponding array
    Light_ADC[14]  = adc_convert(ADC2_CH14_A48);
    Light_ADC[13]  = adc_convert(ADC2_CH12_A46);
    Light_ADC[12]  = adc_convert(ADC2_CH10_A44);
    Light_ADC[11]  = adc_convert(ADC2_CH6_A38);
    Light_ADC[10]  = adc_convert(ADC2_CH4_A36);
    Light_ADC[9]   = adc_convert(ADC1_CH9_A25);
    Light_ADC[8]   = adc_convert(ADC1_CH5_A21);
    Light_ADC[7]   = adc_convert(ADC1_CH1_A17);
    Light_ADC[6]   = adc_convert(ADC0_CH13_A13);
    Light_ADC[5]   = adc_convert(ADC0_CH11_A11);
    Light_ADC[4]   = adc_convert(ADC0_CH8_A8);
    Light_ADC[3]   = adc_convert(ADC0_CH6_A6);
    Light_ADC[2]   = adc_convert(ADC0_CH4_A4);
    Light_ADC[1]   = adc_convert(ADC0_CH2_A2);
    Light_ADC[0]   = adc_convert(ADC0_CH0_A0);

    // Battery current detection (raw * 3.3 / (4095 * op-amp gain * sense resistor))
    Current_Check = (adc_convert(ADC0_CH10_A10) * 0.0008f * 0.41f) / (20.0f * 0.015f);

    // Battery voltage detection (11:1 divider + calibration offset + glitch filter + first-order LPF)
    float raw_voltage = (adc_convert(ADC0_CH5_A5) * 3.3 * 11 / 4095.0) + 0.567;
    Voltage_Check[1] = Voltage_Check[0];
    Voltage_Check[0] = raw_voltage;
    Voltage_Check[0] = 0.3 * Voltage_Check[1] + 0.7 * Voltage_Check[0];
}

/*************************************
** Function: Get_Threshold
** Description: Calculate and update light sensor thresholds
** Input:      None
** Output:     Light_Thr array
** Return:     None
** Others:     Auto-calibrate max/min values, calculate upper and lower thresholds
*************************************/
void Get_Threshold()
{
    Get_Light();  // Read latest light sensor ADC values first

    // Iterate 15-channel sensors, update ADC max and min values
    for (uint8_t ch = 0; ch < 15; ch++)
    {
        // Update max within valid range
        if (Light_ADC[ch] > Light_Raw_Max[ch] && Light_ADC[ch] < 5000)
        {
            Light_Raw_Max[ch] = Light_ADC[ch];
        }
        // Update min within valid range
        if (Light_ADC[ch] < Light_Raw_Min[ch] && Light_ADC[ch] > 0)
        {
            Light_Raw_Min[ch] = Light_ADC[ch];
        }
    }

    // Iterate 15-channel sensors, calculate upper and lower thresholds
    for (uint8_t ch = 0; ch < 15; ch++)
    {
        // Calculate ADC span, prevent division by zero
        uint16_t span = Light_Raw_Max[ch] - Light_Raw_Min[ch];
        if (span == 0)
        {
            span = 1;
        }
        // Calculate upper threshold: 6/9 position of span
        float mid_Up   = (float)(Light_Raw_Max[ch] - Light_Raw_Min[ch]) * 0.74 + Light_Raw_Min[ch];
        // Calculate lower threshold: 5/9 position of span
        float mid_Down = (float)(Light_Raw_Max[ch] - Light_Raw_Min[ch]) * 0.42 + Light_Raw_Min[ch];
        // Store into threshold array
        Light_Thr[ch][0] = mid_Up;
        Light_Thr[ch][1] = mid_Down;
    }
}
