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
* 2026-06-30       Claude             陀螺仪校准加缓冲, 扫线移到 PIT 之前, WS2812 启动阶段放 CPU0
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "headfiles.h"
#include "zf_driver_uart.h"
#include "isr.h"
#include "WS2812.h"

#pragma section all "cpu0_dsram"

/* CPU0 负责整车启动、外设初始化和持续调试输出。 */
int imu_Check = 1;

/* ======================== CPU0 主入口 ======================== */
int core0_main(void)
{
    clock_init();
    debug_init();
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
    Data_Load();
    WS2812_Init();

    // ========================= 启动缓冲：等 3 秒再校准 =========================
    // 给用户时间放稳车辆、打开使能开关，避免上车抖动干扰零漂采集
    // 此时 CPU1 阻塞在 cpu_wait_event_ready()，CPU0 独占灯板，无竞争
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_OFF });

#define CALIB_BUFFER_MS 1000
    for (int i = 0; i < CALIB_BUFFER_MS / 10; i++)
    {
        WS2812_Effect_Update();
        system_delay_ms(10);
    }

    // ========================= 陀螺仪除零漂校准 =========================
    // 负压开到最大，灯板红灯呼吸，提醒用户保持车辆静止
    pwm_set_duty(Suction_Motor_DIR, 10000);
    pwm_set_duty(Suction_Motor_PWM, 9500);

    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_BREATHING,
        .r = 255, .g = 0, .b = 0,
        .period_ms = 1000 });

#define GYRO_CALIB_SAMPLES 500
    int32_t gyro_z_sum = 0;
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++)
    {
        imu660rb_get_gyro();
        gyro_z_sum += imu660rb_gyro_z;
        WS2812_Effect_Update();
        system_delay_ms(5);
    }

    gyro_z_offset = imu660rb_gyro_transition((float)gyro_z_sum / GYRO_CALIB_SAMPLES);
//    pwm_set_duty(Suction_Motor_PWM, 0);

    // ========================= 扫线 =========================
    // 绿灯进度条，全程在 PIT 启动前完成
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_PROGRESS,
        .r = 0, .g = 255, .b = 0 });

#define SCAN_START 200
#define SCAN_END   1600
    for (int tick = 0; tick <= SCAN_END; tick++)
    {
        if (tick > SCAN_START && tick < SCAN_END)
        {
            Get_Threshold();
            g_scan_progress = (uint8_t)((tick - SCAN_START) * 100UL / (SCAN_END - SCAN_START));
            WS2812_Effect_SetProgress(g_scan_progress);
        }
        WS2812_Effect_Update();
        system_delay_ms(3);
    }

    g_scan_progress = 0;

    // 扫线结束：绿灯常亮
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_SOLID,
        .r = 0, .g = 255, .b = 0 });
    WS2812_Effect_Update();
    // ========================= 扫线 END =========================
    pwm_set_duty(Suction_Motor_PWM, 0);
    /* 串口2：调参命令接收 */
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
    uart_rx_interrupt(UART_2, 1);

    interrupt_global_enable(0);

    if (vofa_flash_dump_mode)
    {
        while (TRUE) { Vofa_Send_Flash_Data(); }
    }

    // PIT 启动后，ISR 调用 Car_Go；灯板由 CPU0 主循环统一驱动
    pit_ms_init(CCU60_CH0, 3);
    cpu_wait_event_ready();

    int last_led = -1;

    while (TRUE)
    {
        Vofa_Send_Data();
//        pwm_set_duty(Right_Motor_DIR, 0);
//        pwm_set_duty(Right_Motor_PWM, 6000);
//        pwm_set_duty(Left_Motor_PWM, 6000);
//        pwm_set_duty(Left_Motor_DIR, 0);

        // run-time LED: 0=green(normal) 1=blue(object) 2=yellow(low voltage)
        WS2812_Effect_Config cfg = { .type = EFF_SOLID };
        switch (g_led_flag)
        {
            case 2:  cfg.r = 255; cfg.g = 0; cfg.b = 255;   break; // yellow
            case 1:  cfg.r = 0;   cfg.g = 0;   cfg.b = 255; break; // blue
            default: cfg.r = 0;   cfg.g = 255; cfg.b = 0;   break; // green
        }
        WS2812_Effect_Set(cfg);
       WS2812_Effect_Update();
    }
}

#pragma section all restore
