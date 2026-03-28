/*************************************************
Copyright (C), 2016-2025, TYUT JBD TRoMaC
File name: fun.c
Author: 
Version:               
Date: 2025.9.29
Description:  
Others:      
Function List:
History:
<author>   <time>   <version>  <desc>
**************************************************/

#include "fun.h"

#define VOFA_JustFloat_index 		16
#define WitSD_Send_index 			19
#define N      8          // 窗口长度
// 串口发送数据的数量

/************************************变量定义***********************************/

uint8_t tail[4] = {0x00 , 0x00 , 0x80 , 0x7f};	// 帧尾

/*最大值数组 + 最小值数组 + 阈值*/
int Left_MAX_ARR[8][10] = {0};
int Left_MIN_ARR[8][10] = {20000};

int Right_MAX_ARR[8][10] = {0};
int Right_MIN_ARR[8][10] = {20000};

static uint32_t filt_L_Correct[8][FILT_LEN] = {0};   // 左 8 路队列
static uint32_t filt_R_Correct[8][FILT_LEN] = {0};   // 右 8 路队列
static uint8_t  filt_head = 0;               // 公共队首指针

int L_raw[8] = {0};
int R_raw[8] = {0};

/*--------------- 校准数组 ---------------*/
int L_raw_min[8] = {65535,65535,65535,65535,65535,65535,65535,65535};
int L_raw_max[8] = {0};
int R_raw_min[8] = {65535,65535,65535,65535,65535,65535,65535,65535};
int R_raw_max[8] = {0};
float  left_thr[8][2];  
float  right_thr[8][2];  

/************************************函数定义***********************************/

/******************************************************
** Function: Vofa_JustFloat
** Description:vofa发值
** Others:
*******************************************************/
void VOFA_JustFloat(UART_HandleTypeDef *huart)
{
    float data[VOFA_JustFloat_index] = {0.0};
    const uint8_t end[4] = {0x00, 0x00, 0x80, 0x7f};
	
	data[0] = Left_Light[0];
	data[1] = Left_Light[1];
	data[2] = Left_Light[2];
	data[3] = Left_Light[3];
	data[4] = Left_Light[4];
	data[5] = Left_Light[5];
	data[6] = Left_Light[6];
	data[7] = Left_Light[7];
	data[8] =  Right_Light[0];
	data[9] =  Right_Light[1];
	data[10] = Right_Light[2];
	data[11] = Right_Light[3];
	data[12] = Right_Light[4];
	data[13] = Right_Light[5];
	data[14] = Right_Light[6];
	data[15] = Right_Light[7];
	
	Usart_SendArr(huart, (uint8_t*)data, sizeof(float) * VOFA_JustFloat_index);

    Usart_SendArr(huart, (uint8_t*)end, 4);
}/*Vofa_JustFloat*/

/******************************************************
** Function: WitSD_Send
** Description:存储模块发送
** Others:
*******************************************************/
void WitSD_Send(UART_HandleTypeDef *huart)
{
    uint8_t data[WitSD_Send_index] = {0.0};
    const uint8_t end[2] = {0x80, 0x7f};
	const uint8_t start[2] = {0x7f, 0x80};
   
	for (int i = 0; i < 16; i++)
	{
		data[i] = Image_Arr[i];
	}
	
	// 补空位
	data[16] = 0;
	data[17] = 0;
	data[18] = 0;
	
	// 帧头
	Usart_SendArr(huart, (uint8_t*)start, 2);
	
	// 内容
    Usart_SendArr(huart, (uint8_t*)data, sizeof(uint8_t) * WitSD_Send_index);
	
	// 帧尾
	Usart_SendArr(huart, (uint8_t*)end, 2);
}/*WitSD_Send*/

/******************************************************
** Function: Image_Usart_Send
** Description:图像数据串口发送
** Others:
*******************************************************/
void Image_Usart_Send(UART_HandleTypeDef *huart)
{
    Usart_SendArr(huart, (uint8_t*)Image_Arr, sizeof(int) * 16);
}/*Image_Usart_Send*/

/******************************************************
** Function: Usart_SendArr
** Description:串口发值封装
** Others:
*******************************************************/
void Usart_SendArr(UART_HandleTypeDef *huart, uint8_t *arr, uint16_t size)
{
    HAL_UART_Transmit(huart, arr ,size ,0xFFFFFF);
}

/******************************************************
** Function: Left_Threshold_Correct
** Description:阈值校准
** Others:
*******************************************************/
void Left_Threshold_Correct(void)
{	
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        if (Left_Light[ch] > L_raw_max[ch] && Left_Light[ch] < 1650) L_raw_max[ch] = Left_Light[ch];
        if (Left_Light[ch] < L_raw_min[ch] && Left_Light[ch] > 700) L_raw_min[ch] = Left_Light[ch];
    }

    /* 2. 阈值计算同步 ×100 */
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        uint16_t span = L_raw_max[ch] - L_raw_min[ch];
        if (span == 0) span = 1;
        float mid_Up   = (float)(L_raw_max[ch] - L_raw_min[ch]) * 5 / 9.0f + L_raw_min[ch];
		float mid_Down = (float)(L_raw_max[ch] - L_raw_min[ch]) * 4 / 9.0f + L_raw_min[ch];
        left_thr[ch][0] = mid_Up;//(mid - L_raw_min[ch]) / span * SCALE;   /* 0-100 */
        left_thr[ch][1] = mid_Down;//(mid - L_raw_min[ch]) / span * SCALE;   /* 0-100 */
    }
}

/******************************************************
** Function: Right_Threshold_Correct
** Description:阈值校准
** Others:
*******************************************************/
void Right_Threshold_Correct(void)
{	
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        if (Right_Light[ch] > R_raw_max[ch] && Right_Light[ch] < 1700) R_raw_max[ch] = Right_Light[ch];
        if (Right_Light[ch] < R_raw_min[ch] && Right_Light[ch] > 700) R_raw_min[ch] = Right_Light[ch];
    }

    /* 2. 阈值计算同步 ×100 */
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        uint16_t span = R_raw_max[ch] - R_raw_min[ch];
        if (span == 0) span = 1;
        float mid_Up   = (float)(R_raw_max[ch] - R_raw_min[ch]) * 5 / 9.0f + R_raw_min[ch];
		float mid_Down = (float)(R_raw_max[ch] - R_raw_min[ch]) * 4 / 9.0f + R_raw_min[ch];
        right_thr[ch][0] = mid_Up;//(mid - R_raw_min[ch]) / span * SCALE;   /* 0-100 */
        right_thr[ch][1] = mid_Down;//(mid - R_raw_min[ch]) / span * SCALE;   /* 0-100 */
    }
}

/******************************************************
** Function: Arr_Init
** Description:存储数组初始化
** Others:
*******************************************************/
void Arr_Init()
{
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			Left_MIN_ARR[i][j] = 20000; 
			Right_MIN_ARR[i][j] = 20000; 
		}
	}
}

/******************************************************
** Function: filt_scale_20000
** Description:对任意 8 元素 int 数组做滑动平均 + 0..5000 归一化
** Others:
*******************************************************/
void filt_scale_20000(int arr[8], uint32_t filt_q[8][FILT_LEN])
{
    uint32_t sum[8] = {0};

    /* 1. 把新样本压入循环队列 */
    for (int i = 0; i < 8; i++)
    {
        filt_q[i][filt_head] = (uint32_t)arr[i];   // 原始 0..65535
    }
    filt_head = (filt_head + 1) % FILT_LEN;

    /* 2. 求 8 路各自的和 */
    for (int i = 0; i < 8; i++)
    {
        for (int k = 0; k < FILT_LEN; k++)
            sum[i] += filt_q[i][k] * (FILT_LEN - ((filt_head - k + FILT_LEN) % FILT_LEN));
    }

    /* 3. 归一化：先除 FILT_LEN 得平均值，再 *20000/65535 ≈ *0.305 */
    for (int i = 0; i < 8; i++)
    {
        uint32_t avg = sum[i] / 36;           // 0..65535
        arr[i] = (int)((avg * 5000UL) >> 16);     // 0..20000，误差<0.002%
    }
}

/******************************************************
** Function: Sliding_Avg
** Description:滑动均值
** Others:
*******************************************************/
float Sliding_Avg(int x)
{
	static float buf[N] = {0}; /* 环形缓冲区 */
	static float sum = 0;      /* 当前窗口的和 */
	static int idx = 0;      /* 写指针 */
	
    sum += x - buf[idx]; /* 去旧添新 */
    buf[idx] = x;        /* 保存新值 */
    idx = (idx + 1) & (N - 1); /* 环形指针，N 为 2 的幂时 & 最快 */
    return sum / N;      /* 当前均值 */
}

/*************************************
** Function: Map_Range
** Description: 数值映射函数
** @param range1 目标区间 {min1, max1}
** @param range2 原来区间 {min2, max2}
** @param value  要映射的值（必须在 range2 范围内）
** Others:
*************************************/
int16_t Map_Range(int16_t range1[2], int16_t range2[2], int16_t value)
{
    int16_t min1 = range1[0];
    int16_t max1 = range1[1];
    int16_t min2 = range2[0];
    int16_t max2 = range2[1];

    // 映射值限幅
    value = Data_Limit(value, min2, max2);

    // 计算比例
    float ratio = (max2 - value) * 1.0 / (max2 - min2);

    // 映射到目标区间
    int16_t mapped_value = min1 + ratio * (max1 - min1);

    return mapped_value;
}
