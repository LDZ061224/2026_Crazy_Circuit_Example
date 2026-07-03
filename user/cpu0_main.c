/*********************************************************************************************************************
* TC264 Open Source Library
* Copyright (c) 2022 SEEKFREE
*
* This file is the CPU0 startup entry and is responsible for system initialization,
* peripheral initialization, and debug data output.
*
* Copyright and licensing details are governed by the LICENSE and
* GPL3_permission_statement.txt files in the libraries/doc directory.
*
* Filename           cpu0_main
* Platform           TC264D
* Dev environment    ADS v1.10.2
* Project home       https://seekfree.taobao.com/
*
* Date               Author              Description
* 2022-09-15         pudding             first version
* 2026-06-30         Claude              gyro calibration with buffer, line scan moved before PIT, WS2812 startup on CPU0
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "headfiles.h"
#include "zf_driver_uart.h"
#include "isr.h"
#include "WS2812.h"

#pragma section all "cpu0_dsram"

/* CPU0 handles vehicle startup, peripheral initialization, and continuous debug output. */
int imu_Check = 1;

/* ======================== CPU0 Main Entry ======================== */
int core0_main(void)
{
    /* ---- System & peripheral initialization ---- */
    clock_init();
    debug_init();
    Encoder_Init();
    Motor_Init();
    Other_Init();
    Light_Init();

    /* ---- IMU (gyro) initialization with retry ---- */
    while(1)
    {
       if (imu660rb_init()){}
       else
           break;
       gpio_toggle_level(P33_4);   // toggle LED on each retry attempt
    }
    gpio_set_level(P33_4, 0);      // turn LED off after successful init

    /* ---- saved data and LED strip ---- */
    Data_Load();
    WS2812_Init();

    // ========================= Startup Buffer: wait 3 seconds before calibration =========================
    // Give the user time to place the car steadily and turn on the enable switch,
    // so that vehicle shake does not corrupt the zero-drift acquisition.
    // CPU1 is blocked at cpu_wait_event_ready(), so CPU0 has exclusive access to the LED board.
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_OFF });

#define CALIB_BUFFER_MS 1000
    for (int i = 0; i < CALIB_BUFFER_MS / 10; i++)
    {
        WS2812_Effect_Update();
        system_delay_ms(10);
    }

    // ========================= Gyro Zero-Drift Calibration =========================
    // Turn on suction to maximum, LED board breathing red to remind the user to keep the vehicle still.
    pwm_set_duty(Suction_Motor_DIR, 10000);
    pwm_set_duty(Suction_Motor_PWM, 3000);

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
//    pwm_set_duty(Suction_Motor_PWM, 9500);

    // ========================= Line Scan =========================
    // Green progress bar on LED strip, completed entirely before PIT starts.
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

    // Line scan finished: solid green
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_SOLID,
        .r = 0, .g = 255, .b = 0 });
    WS2812_Effect_Update();
    // ========================= Line Scan END =========================
    pwm_set_duty(Suction_Motor_PWM, 10000);

    /* UART_2: tuning command receiver */
    uart_init(UART_2, 921600, UART2_TX_P33_9, UART2_RX_P33_8);
    uart_rx_interrupt(UART_2, 1);

    /* Enable interrupts on CPU0 */
    interrupt_global_enable(0);

    /* ---- VOFA flash dump mode: skip normal operation, dump stored data only ---- */
    if (vofa_flash_dump_mode)
    {
        while (TRUE) { Vofa_Send_Flash_Data(); }
    }

    // After PIT starts, ISR calls Car_Go; LED board is driven by the CPU0 main loop.
    pit_ms_init(CCU60_CH0, 3);
    cpu_wait_event_ready();

    int last_led = -1;

    /* ---- Main loop: telemetry output and runtime LED indication ---- */
    while (TRUE)
    {
        Vofa_Send_Data();
//        pwm_set_duty(Right_Motor_DIR, 10000);
//        pwm_set_duty(Right_Motor_PWM, 6000);
//        pwm_set_duty(Left_Motor_PWM, 6000);
//        pwm_set_duty(Left_Motor_DIR, 10000);

        // run-time LED: 0=green(normal) 1=blue(object) 2=purple(low voltage)
        WS2812_Effect_Config cfg = { .type = EFF_SOLID };
        switch (g_led_flag)
        {
            case 2:  cfg.r = 255; cfg.g = 0; cfg.b = 255;   break; // purple
            case 1:  cfg.r = 0;   cfg.g = 0;   cfg.b = 255; break; // blue
            default: cfg.r = 0;   cfg.g = 255; cfg.b = 0;   break; // green
        }
        WS2812_Effect_Set(cfg);
       WS2812_Effect_Update();
       system_delay_ms(1);
    }
}

#pragma section all restore


