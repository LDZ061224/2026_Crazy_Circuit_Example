/*********************************************************************************************************************
* TC264 Open Source Library
* Copyright (c) 2022 SEEKFREE
*
* 本文件是 CPU0 的启动入口，负责完成系统初始化、外设初始化和调试数据输出。
*
* 版权与许可说明请以 libraries/doc 目录中的 LICENSE 和
* GPL3_permission_statement.txt 为准。
*
* 文件名            cpu0_main
* 适用平台          TC264D
* 开发环境          ADS v1.10.2
* 项目主页          https://seekfree.taobao.com/
*
* 日期              作者                说明
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "headfiles.h"
#include "zf_driver_uart.h"
#pragma section all "cpu0_dsram"
// 将后续代码段放到 CPU0 的 DSRAM 中，便于快速访问。

/* CPU0 负责整车启动、外设初始化和持续调试输出。 */
int imu_Check = 1;  // IMU660RB初始化状态，1=未完成

/* ======================== CPU0 主入口 ======================== */
int core0_main(void)
{
    clock_init();                   // 初始化系统时钟
    debug_init();                   // 初始化调试串口和基础输出
    // 先完成底层外设初始化，再进入姿态模块和主循环。
    Encoder_Init();
    Motor_Init();
    Other_Init();
    Light_Init();
    /* IMU660RB 轮询初始化，直到姿态模块就绪后再继续。 */
    while (imu_Check == 1)
    {
        imu_Check = imu660rb_init();
        gpio_toggle_level(P33_4);
        system_delay_ms(200);
    }
    gpio_set_level(P33_4,0);
    TCA9555_Init();
    OLED_Input();
    OLED_Data_Load();
    uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);
    uart_rx_interrupt(UART_0, 1);           // 开启串口 0 接收中断
    interrupt_global_enable(0);             // 允许全局中断

    if (vofa_flash_dump_mode)
    {
        // Flash数据导出模式：不启动PIT，不跑车，只循环发送Flash数据
        while (TRUE)
        {
            Vofa_Send_Flash_Data();
        }
    }

    // PIT 定时器提供周期任务节拍，主循环只保留轻量输出。
    pit_ms_init(CCU60_CH0, 3);
    cpu_wait_event_ready();                 // 等待事件调度器进入就绪状态

    /* 主循环只做持续型调试发送，不阻塞控制中断。 */
    while (TRUE)
    {
        // 周期性发送调试数据，供上位机查看运行状态。
        //Wit_Send_Data();  // 无线模块未连接，使用VOFA有线调试
        Vofa_Send_Data();
        // system_delay_ms(1);
    }
}

#pragma section all restore
// 结束 CPU0 专用 RAM 段
