/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr_config.h"
#include "isr.h"
#include "Fun.h"

extern uint8 debug_uart_data;

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。

int Scan_Count = 0;         // 阈值校准计数
int Scan_Complete = 0;

// 串口调参：解析出的命令存这里，主循环检测 g_tuning_cmd.valid==1 即可读取
uart_tuning_cmd_t g_tuning_cmd = {0};

/*************************************
** Function: uart_tuning_atof
** Description: 手写字符串转浮点数，替代标准库 atof
** Parameters:  *str - 待转换字符串，如 "123.456" 或 "-0.008"
** Returns:     float 转换结果
** Notes:      嵌入式环境可能没有标准库 atof，故自行实现
*************************************/
static float uart_tuning_atof(const char *str)
{
    float result = 0.0f;        // 最终结果累加在这里
    float sign = 1.0f;          // 正负号，默认正
    float decimal = 1.0f;       // 小数权重：0.1 → 0.01 → 0.001 ...
    uint8 has_decimal = 0;      // 是否已经遇到小数点

    // 跳过符号位
    if (*str == '-') 
    {
         sign = -1.0f; 
         str++; 
    }
    else if (*str == '+') 
    {
         str++; 
    }

    // 逐字符扫描
    while (*str)
    {
        if (*str == '.')                    // 遇到小数点 → 切到小数模式
        {
            has_decimal = 1;
            str++;
            continue;
        }
        if (*str < '0' || *str > '9') break; // 非数字字符 → 停止

        if (has_decimal)                    // 小数模式：权重衰减 × 0.1
        {
            decimal *= 0.1f;               // 十分位 → 百分位 → 千分位 ...
            result += (float)(*str - '0') * decimal;
        }
        else                                // 整数模式：高位 × 10 + 新位
        {
            result = result * 10.0f + (float)(*str - '0');
        }
        str++;
    }
    return result * sign;                   // 最后乘上符号
}

/*************************************
** Function: uart_tuning_parse_frame
** Description: 解析完整帧 "KEY=value" 字符串，拆出键名和数值
** Parameters:  *frame - 帧字符串，如 "@LKP=80.5"
**             len    - 帧长度（不含结尾 '\0'）
** Details:    帧格式 @XXX=%f：XXX=3字符键名，=分隔，%f=浮点数值
**             位置索引: [0]='@' [1~3]=KEY [4]='=' [5~]=value
**             解析结果写入全局 g_tuning_cmd
*************************************/
static void uart_tuning_parse_frame(const char *frame, uint8 len)
{
    // 最短也至少是 @X=Y# → 去掉头尾中间至少 "X=Y" = 3字节, 加上@前缀共4+2=6
    if (len < 6) return;            // 帧太短，丢弃
    if (frame[0] != '@') return;    // 帧头不对，丢弃
    if (frame[4] != '=') return;    // 等号位置不对（KEY 不是3字符?），丢弃

    // --- 提取 KEY：@ 后面连续 3 个字符 ---
    //  位置:  0    1  2  3    4    5 ...
    //  帧:    @    L  K  P    =    8  0  .  5
    //         ↑                ↑
    //       frame[0]         frame[4]
    g_tuning_cmd.key[0] = frame[1];     // 第1个字符如 'L'
    g_tuning_cmd.key[1] = frame[2];     // 第2个字符如 'K'
    g_tuning_cmd.key[2] = frame[3];     // 第3个字符如 'P'
    g_tuning_cmd.key[3] = '\0';         // C 字符串结尾

    // --- 提取 value：等号后面一直到帧尾 ---
    // &frame[5] 指向 '8' 的位置，atof 一直读到非数字为止
    g_tuning_cmd.value = uart_tuning_atof(&frame[5]);

    // --- 标记有效，主循环可读 ---
    g_tuning_cmd.valid = 1;
}

/*************************************
** Function: uart_tuning_parse_byte
** Description: 逐字节喂入帧解析状态机
** Parameters:  byte - 串口收到的单个字节
** Details:    状态机行为：
**             1. 收到 '@' → 开始新帧，清空缓冲区
**             2. 收到 '#' 且正在接收 → 帧结束，调用 parse_frame 解析
**             3. 其他字节且正在接收 → 存入缓冲区（最长31字节）
**             未在接收中且非 '@' 的字节 → 丢弃（帧间垃圾数据）
** Notes:      buf/idx/receiving 为 static 局部变量，调用间保持值不变
*************************************/
static void uart_tuning_parse_byte(uint8 byte)
{
    static uint8 buf[32];       // 帧缓冲区，最长存31个字节 + 结束符
    static uint8 idx = 0;       // 当前写入位置（下一个字节放哪里）
    static uint8 receiving = 0; // 状态: 0=不在接收, 1=正在接收帧内容

    if (byte == '@')            // --- 帧头：开始新帧 ---
    {
        idx = 0;                // 清空缓冲区指针
        receiving = 1;          // 进入"接收中"状态
        buf[idx++] = byte;      // 也把 @ 存进去，方便帧解析时对齐位置
    }
    else if (receiving)         // --- 正在接收中 ---
    {
        if (byte == '#')        // 帧尾：一帧结束
        {
            buf[idx] = '\0';    // 字符串终止符
            receiving = 0;      // 退出接收状态
            uart_tuning_parse_frame((const char *)buf, idx); // 送去解析
            idx = 0;
        }
        else if (idx < sizeof(buf) - 1) // 普通字节：存入缓冲区
        {
            buf[idx++] = byte;
        }
        else                    // 缓冲区满了（超过31字节还没看到#）→ 丢弃
        {
            idx = 0;
            receiving = 0;
        }
    }
    // else: 不在接收状态也不是 '@' → 忽略（帧间的垃圾数据）
}

// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    // 阈值矫正
    if (Scan_Complete == 0) Scan_Count++;

    if (Scan_Count > 200 && Scan_Count < 800)
    {
        Get_Threshold();
    }
    else if (Scan_Count == 800)
    {
        Scan_Complete = 1;
    }

    // 跑车
    if (Scan_Complete)
    {
        Car_Go();
    }
}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);




}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
        uart_tuning_parse_byte(debug_uart_data);    // 同时喂给调参命令解析器
#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数



}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    gnss_uart_callback();                           // GNSS串口回调函数



}

// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
