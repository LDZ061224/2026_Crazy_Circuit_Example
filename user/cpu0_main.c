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
* 2026-06-27       Claude             新增新车测试模式分支
********************************************************************************************************************/

// 底层驱动库（测试和正式模式都需要）
#include "zf_common_headfile.h"

/*
 *  编译模式控制：
 *    NEW_CAR_TEST_ENABLE = 1 → 新车基础功能测试模式（轻量，不依赖跑车代码）
 *    NEW_CAR_TEST_ENABLE = 0 → 原正式跑车代码
 */
#define NEW_CAR_TEST_ENABLE  1

#if NEW_CAR_TEST_ENABLE
    /* ========== 测试模式：只引入测试框架 ========== */
    #include "app_new_car_test.h"
#else
    /* ========== 正式模式：引入完整跑车代码 ========== */
    #include "headfiles.h"
    #include "zf_driver_uart.h"
    #include "isr.h"
#endif

#pragma section all "cpu0_dsram"
// 将后续代码段放到 CPU0 的 DSRAM 中，便于快速访问。

/* ======================== CPU0 主入口 ======================== */
int core0_main(void)
{
    clock_init();                   // 初始化系统时钟（必须）
    debug_init();                   // 初始化调试串口基础输出（必须）

#if NEW_CAR_TEST_ENABLE
    /* ========== 新车基础功能测试模式 ========== */
    NewCarTest_Init();

    while (TRUE)
    {
        NewCarTest_Loop();
    }

#else
    /* ========== 原正式跑车代码（完整保留） ========== */
    int imu_Check = 1;
    int i = 0;

    Encoder_Init();
    Motor_Init();
    Other_Init();
    Light_Init();
    while(1)
    {
       if (imu660rb_init()){}
       else
           break;
       gpio_toggle_level(P33_4);
    }
    gpio_set_level(P33_4, 0);
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
        Vofa_Send_Data();
    }
#endif
}

#pragma section all restore
// 结束 CPU0 专用 RAM 段
