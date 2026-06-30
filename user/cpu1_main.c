/*********************************************************************************************************************
* TC264 Open Source Library
* Filename           cpu1_main
* Modification history
* 2022-09-15         pudding   first version
* 2026-06-30         Claude    Uart_Adjust only (WS2812 moved to CPU0)
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "headfiles.h"
#include "Uart_Adjust.h"
#pragma section all "cpu1_dsram"

/* CPU1 main entry: dedicated to UART tuning parameter processing.
 * WS2812 LED control has been moved to CPU0 to avoid bus contention. */
void core1_main(void)
{
    /* Disable watchdog on CPU1 to prevent unexpected resets during tuning */
    disable_Watchdog();

    /* Wait for CPU0 to finish initialization (PIT + event broadcast) */
    cpu_wait_event_ready();

    /* Main loop: continuously apply UART tuning commands */
    while (TRUE)
    {
        Uart_Adjust_Apply();
        system_delay_ms(10);
    }
}
#pragma section all restore
