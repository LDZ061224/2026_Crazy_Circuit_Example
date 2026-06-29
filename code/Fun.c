/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Fun.c
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description: 外设驱动、数据采集、初始化等功能实现
Others:      无
Function List:
              1. Vofa_Send_Data    - VOFA上位机数据发送
              2. Wit_Send_Data     - 无线模块数据发送
              3. Light_Init        - 光敏传感器ADC初始化
              4. Encoder_Init      - 编码器初始化
              5. Motor_Init        - 电机PWM初始化
              6. Other_Init        - 其他外设初始化
              7. Get_Light         - 读取光敏传感器ADC值
              8. Get_Threshold     - 计算光敏传感器阈值
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30   0.0        创建初始版本
**************************************************/

#include "Fun.h"

/********************************* 全局变量定义 *********************************/
uint8 imu660rb_Check = 0;                // IMU660RB传感器初始化状态标志
uint16 Light_ADC[15] = {0};              // 15路光敏传感器原始ADC值
float Current_Check = 0;                 // 电流检测值
float Voltage_Check[2] = {0};            // 两路电压检测值
int Dbg[10] = {0};

/* --------------- 光敏传感器校准参数 --------------- */
int Light_Raw_Min[15] =                  // 15路光敏传感器ADC最小值（初始化为最大值）
{
    4096, 4096, 4096, 4096, 4096,
    4096, 4096, 4096, 4096, 4096,
    4096, 4096, 4096, 4096, 4096
};
int Light_Raw_Max[15] = {0};             // 15路光敏传感器ADC最大值（初始化为0）
float  Light_Thr[15][2];                 // 15路光敏传感器上下阈值

/********************************* 函数实现 *********************************/

/*************************************
** Function: Vofa_Send_Data
** Description: 向VOFA上位机发送调试数据
** Input:      无
** Output:     无
** Return:     无
** Others:     串口0发送，遵循VOFA协议帧格式
*************************************/
void Vofa_Send_Data(void)
{
    floatu8data VOFA_data[20];           // 浮点数与字节数组共用体
    uint8 frame[15 * 4 + 4];
    // 清空数据缓存
    memset(VOFA_data, 0, sizeof(VOFA_data));

    int i = 0;
//       for (int i = 0; i < 15; i++)
//       {
//           VOFA_data[i].floatdata = Light_ADC[i];
//       }
    // 赋值需要发送的调试数据
     VOFA_data[0].floatdata  = Left_Exp_Spd;
     VOFA_data[1].floatdata  = Right_Exp_Spd;
     VOFA_data[2].floatdata  = Left_Real_Spd;
     VOFA_data[3].floatdata  = Right_Real_Spd;
     VOFA_data[4].floatdata  = Gyro_Z;
     VOFA_data[5].floatdata  = Total_Run_Mileage;
     VOFA_data[6].floatdata  = Voltage_Check[0];
     VOFA_data[7].floatdata  = Count.Mileage;
     VOFA_data[8].floatdata  = Left_PID_Out;
     VOFA_data[9].floatdata  = Right_PID_Out;
     VOFA_data[10].floatdata = Dbg[0];
     VOFA_data[11].floatdata = Error;
     VOFA_data[12].floatdata = Gyro_Integral;
     VOFA_data[13].floatdata = Debug_Angle_Vel_Target;
     VOFA_data[14].floatdata = Debug_Angle_Vel_Real;
    // 循环发送15组浮点数数据
    for(i = 0; i < 15; i++)
    {
        // 提取浮点数拆分后的4个字节
        frame[i * 4 + 0] = VOFA_data[i].u8data[0];
        frame[i * 4 + 1] = VOFA_data[i].u8data[1];
        frame[i * 4 + 2] = VOFA_data[i].u8data[2];
        frame[i * 4 + 3] = VOFA_data[i].u8data[3];
    }

    // 发送VOFA协议固定帧尾 00 00 80 7F
    frame[60] = 0x00;
    frame[61] = 0x00;
    frame[62] = 0x80;
    frame[63] = 0x7f;

    uart_write_buffer(UART_0, frame, sizeof(frame));

    return;
}

/*************************************
** Function: Vofa_Send_Flash_Data
** Description: VOFA Flash里程数据导出——单帧发送全部有效里程值
** Details:   顺序：Turn_Mileage_Record[0..N-1] + 各路段边缘里程
**            一帧包含所有数据，循环发送
*************************************/
void Vofa_Send_Flash_Data(void)
{
    floatu8data vofa[50];
    uint8 frame[50 * 4 + 4];
    memset(vofa, 0, sizeof(vofa));

    uint16_t turn_count = Turn_Mileage_Record_Num;
    uint16_t edge_count = 0;
    for (uint8_t r = 0; r <= Run_Track.Node_Num; r++)
    {
        edge_count += Run_Track.Node_Arr_Mileage_Num[r];
    }
    uint16_t pos = 0;

    // 转向间隔里程
    for (uint16_t i = 0; i < turn_count && pos < 50; i++, pos++)
    {
        vofa[pos].floatdata = Turn_Mileage_Record[i];
    }
    // 各路段边缘里程（按行展开，只取有效条目）
    for (uint8_t r = 0; r <= Run_Track.Node_Num && pos < 50; r++)
    {
        uint8_t num = Run_Track.Node_Arr_Mileage_Num[r];
        for (uint8_t c = 0; c < num && pos < 50; c++, pos++)
        {
            vofa[pos].floatdata = Segment_Edge_Mileage_Record[r][c];
        }
    }

    // 一次性发送全部有效数据
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
** Description: 通过无线模块发送传感器数据
** Input:      无
** Output:     无
** Return:     无
** Others:     串口3发送，自定义数据帧格式
*************************************/
void Wit_Send_Data(void)
{
    uint8 data[4];                       // 单精度浮点数拆分后的4字节缓存
    floatu8data VOFA_data[20];           // 浮点数与字节数组共用体
    // 清空数据缓存
    memset(VOFA_data, 0, sizeof(VOFA_data));

    int i = 0;

    // 赋值15路光敏传感器处理后数据
    for (int i = 0; i < 15; i++)
    {
        VOFA_data[i].floatdata = Light_Convert[i];
    }

    // 赋值状态参数
    VOFA_data[15].floatdata = Run_Mode;   // 运行模式
    VOFA_data[16].floatdata = Error;      // 错误码
    VOFA_data[17].floatdata = 0;          // 保留位

    // 发送数据帧头 00 00 7F 80
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x7f);
    uart_write_byte(UART_3, 0x80);

    // 循环发送18组浮点数数据
    for(i = 0; i < 18; i++)
    {
        // 提取浮点数拆分后的4个字节
        data[0] = VOFA_data[i].u8data[0];
        data[1] = VOFA_data[i].u8data[1];
        data[2] = VOFA_data[i].u8data[2];
        data[3] = VOFA_data[i].u8data[3];

        // 通过串口3逐字节发送
        uart_write_byte(UART_3, data[0]);
        uart_write_byte(UART_3, data[1]);
        uart_write_byte(UART_3, data[2]);
        uart_write_byte(UART_3, data[3]);
    }

    // 发送数据帧尾 00 00 80 7F
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x00);
    uart_write_byte(UART_3, 0x80);
    uart_write_byte(UART_3, 0x7f);

    return;
}

/*************************************
** Function: Light_Init
** Description: 15路光敏传感器ADC初始化
** Input:      无
** Output:     无
** Return:     无
** Others:     配置为12位ADC精度
*************************************/
void Light_Init()
{
    // 初始化15路光敏传感器对应的ADC通道，12位精度
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
** Description: 左右电机编码器初始化
** Input:      无
** Output:     无
** Return:     无
** Others:     正交编码器模式
*************************************/
void Encoder_Init()
{
    // 左电机编码器初始化
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    // 右电机编码器初始化
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_P02_6, TIM3_ENCODER_CH2_P02_7);
}

/*************************************
** Function: Motor_Init
** Description: 电机驱动PWM初始化
** Input:      无
** Output:     无
** Return:     无
** Others:     左右电机30KHz，吸风电机70KHz
*************************************/
void Motor_Init()
{
    // DIR PWM: 0=forward, 10000=reverse, 30kHz
    pwm_init(Left_Motor_DIR,  30000, 10000);
    pwm_init(Right_Motor_DIR, 30000, 10000);
    // duty PWM: 30kHz, start at 0
    pwm_init(Left_Motor_PWM,  30000, 10000);
    pwm_init(Right_Motor_PWM, 30000, 10000);
    // 风扇: DIR=0=吸风, duty=0
    pwm_init(Suction_Motor_DIR, 100000, 10000);
    pwm_init(Suction_Motor_PWM, 100000, 10000);
}

/*************************************
** Function: Other_Init
** Description: 其他外设初始化
** Input:      无
** Output:     无
** Return:     无
** Others:     包含OLED、IMU660RB、GPIO初始化
*************************************/
void Other_Init()
{
    OLED_Init();                                  // OLED显示屏初始化
//    gpio_init(P15_1,    GPO, 0, GPO_PUSH_PULL);
    gpio_init(P33_4,    GPO, 0, GPO_PUSH_PULL);   // 推挽输出GPIO初始化
    gpio_init(P20_7,    GPI, 0, GPI_PULL_DOWN);   // 使能开关输入
    gpio_init(P22_3,    GPI, 0, GPI_PULL_DOWN);   // 自定义按键输入
    adc_init(ADC0_CH5_A5,   ADC_12BIT); // 电池电压检测ADC
    adc_init(ADC0_CH10_A10, ADC_12BIT); // 电池电流检测ADC
}

/*************************************
** Function: Get_Light
** Description: 读取15路光敏传感器ADC、电池电压和电流
** Input:      无
** Output:     Light_ADC数组、Voltage_Check、Current_Check
** Return:     无
** Others:     数组下标0~14对应传感器1~15
**             电压公式：raw * 3.3 * 11 / 4095 + 0.567，经一阶低通滤波
**             电流公式：raw * 3.3 / (4095 * 20 * 0.015)
*************************************/
void Get_Light()
{
    // 读取15路ADC值并存入对应数组
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

    // 电池电流检测（raw * 3.3 / (4095 * 运放增益 * 采样电阻)）
    Current_Check = (adc_convert(ADC0_CH10_A10) * 3.3 / (4095 * 20 * 0.015));

    // 电池电压检测（分压比11:1 + 校准偏移 + 突变滤波 + 一阶低通滤波）
    {
        float raw_voltage = (adc_convert(ADC0_CH5_A5) * 3.3 * 11 / 4095.0) + 0.567;

        // 突变过滤：相邻两次差值 > 0.5V 则丢弃本次数据，保留上次值
        if (Voltage_Check[0] - raw_voltage > 0.5f)
        {
            raw_voltage = Voltage_Check[0];  // 认为是毛刺，使用上次有效值
        }

        Voltage_Check[1] = Voltage_Check[0];
        Voltage_Check[0] = raw_voltage;
    }
    Voltage_Check[0] = 0.3 * Voltage_Check[1] + 0.7 * Voltage_Check[0];
}

/*************************************
** Function: Get_Threshold
** Description: 计算并更新光敏传感器阈值
** Input:      无
** Output:     Light_Thr数组
** Return:     无
** Others:     自动校准最大值、最小值，计算上下阈值
*************************************/
void Get_Threshold()
{
    Get_Light();  // 先读取最新的光敏传感器ADC值

    // 遍历15路传感器，更新ADC最大值和最小值
    for (uint8_t ch = 0; ch < 15; ch++)
    {
        // 有效范围内更新最大值
        if (Light_ADC[ch] > Light_Raw_Max[ch] && Light_ADC[ch] < 5000)
        {
            Light_Raw_Max[ch] = Light_ADC[ch];
        }
        // 有效范围内更新最小值
        if (Light_ADC[ch] < Light_Raw_Min[ch] && Light_ADC[ch] > 0)
        {
            Light_Raw_Min[ch] = Light_ADC[ch];
        }
    }

    // 遍历15路传感器，计算上下阈值
    for (uint8_t ch = 0; ch < 15; ch++)
    {
        // 计算ADC量程，防止除0
        uint16_t span = Light_Raw_Max[ch] - Light_Raw_Min[ch];
        if (span == 0)
        {
            span = 1;
        }
        // 计算上阈值：量程6/9位置
        float mid_Up   = (float)(Light_Raw_Max[ch] - Light_Raw_Min[ch]) * 6 / 9.0f + Light_Raw_Min[ch];
        // 计算下阈值：量程5/9位置
        float mid_Down = (float)(Light_Raw_Max[ch] - Light_Raw_Min[ch]) * 5 / 9.0f + Light_Raw_Min[ch];
        // 存入阈值数组
        Light_Thr[ch][0] = mid_Up;
        Light_Thr[ch][1] = mid_Down;
    }
}
