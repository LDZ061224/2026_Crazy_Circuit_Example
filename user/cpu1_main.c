/*********************************************************************************************************************
* TC264 Opensourec Library
* 文件名称          cpu1_main
* 修改记录
* 2022-09-15       pudding   first version
* 2026-06-30       Claude    Uart_Adjust only (WS2812 moved to CPU0)
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "headfiles.h"
#include "Uart_Adjust.h"
#pragma section all "cpu1_dsram"

void core1_main(void)
{
    disable_Watchdog();

    cpu_wait_event_ready();

    while (TRUE)
    {
        Uart_Adjust_Apply();
        system_delay_ms(10);
    }
}
#pragma section all restore
