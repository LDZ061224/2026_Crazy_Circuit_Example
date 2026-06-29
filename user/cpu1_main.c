/*********************************************************************************************************************
* TC264 Opensourec Library
* Copyright (c) 2022 SEEKFREE 逐飞科技
* …[license truncated]…
* 文件名称          cpu1_main
* 修改记录
* 2022-09-15       pudding            first version
* 2026-06-28       Claude             OLED→串口调参 (UART_2 @XXX=val#)
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "headfiles.h"
#include "Uart_Adjust.h"
#pragma section all "cpu1_dsram"

void core1_main(void)
{
    disable_Watchdog();

    /* wait for CPU0 to finish initialization */
    cpu_wait_event_ready();

    while (TRUE)
    {
        /* consume serial tuning commands from UART_2 */
        Uart_Adjust_Apply();
    }
}
#pragma section all restore
