/*********************************************************************************************************************
* TC264 Opensourec Library
* 文件名称          cpu1_main
* 修改记录
* 2022-09-15       pudding   first version
* 2026-06-29       Claude    Uart_Adjust + WS2812 (all on CPU1, no ISR, single thread)
*                            CPU0 stays clean for VOFA / Car_Go.
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "headfiles.h"
#include "Uart_Adjust.h"
#include "WS2812.h"
#pragma section all "cpu1_dsram"

extern uint8_t g_led_flag;   // from Ctrl.c, 1=blue, 0=green

void core1_main(void)
{
    disable_Watchdog();

    WS2812_Init();

    cpu_wait_event_ready();

    int last_led = -1;

    while (TRUE)
    {
        Uart_Adjust_Apply();

        if (g_led_flag != last_led)
        {
            last_led = g_led_flag;
            if (g_led_flag)
                WS2812_Effect_Set((WS2812_Effect_Config){
                    .type = EFF_SOLID, .r = 0, .g = 0, .b = 255 });   // blue
            else
                WS2812_Effect_Set((WS2812_Effect_Config){
                    .type = EFF_SOLID, .r = 0, .g = 255, .b = 0 });   // green
        }

        WS2812_Effect_Update();
        system_delay_ms(10);
    }
}
#pragma section all restore
