/*********************************************************************************************************************
* TC264 Open Source Library (third-party open source library based on official SDK interfaces)
* Copyright (c) 2022 SEEKFREE (Zhufei Technology)
* ... [license truncated] ...
* Filename           isr
* Company            Chengdu Zhufei Technology Co., Ltd.
* Dev environment    ADS v1.10.2
* Platform           TC264D
*
* Modification history
* Date               Author              Notes
* 2022-09-15         pudding             first version
* 2026-06-28         Claude              UART_2 serial tuning, removed duplicate parser
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr_config.h"
#include "isr.h"
#include "Fun.h"
#include "Uart_Adjust.h"
#include "Debug_Car.h"

extern uint8 debug_uart_data;

// Line scan is done before PIT starts in cpu0_main.c; ISR only needs to run the car control schedule.

// ============================ PIT Timer ISR ============================

/* CCU60 CH0: 3ms periodic interrupt — main car control tick (Car_Go core beat) */
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

#if USE_DEBUG_MODE
    Debug_Car_Go();
#else
    Car_Go();
#endif
}

/* CCU60 CH1: unused, closed */
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
}

/* CCU61 CH0: unused, closed */
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
}

/* CCU61 CH1: unused, closed */
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
}

// ============================ External Interrupt (ERU) ISR ============================

/* ERU channels 0 & 4 share one ISR;
 * actual pins: CH0_REQ0_P15_4 / CH4_REQ13_P15_5 (idle handling). */
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }
    if(exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

/* ERU channels 1 & 5 share one ISR;
 * CH1_REQ10_P14_3: ToF module (deprecated); CH5_REQ1_P15_8: idle handling. */
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        tof_module_exti_handler();
    }
    if(exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

/* ERU channels 3 & 7 share one ISR;
 * CH3_REQ6_P02_0: camera VSYNC; CH7_REQ16_P15_1: idle handling. */
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}

// ============================ DMA ISR ============================

/* DMA channel 5: camera DMA transfer complete handler */
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}

// ============================ UART ISR ============================

// --- UART_0: debug printf / VOFA output ---
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);

}

// --- UART_1: camera data ---
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_uart_handler();
}

// --- UART_2: serial tuning (replacement when OLED is unavailable on new car) ---
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    #if DEBUG_UART_USE_INTERRUPT
    debug_interrupr_handler();
    Uart_Adjust_ParseByte(debug_uart_data);
#endif
}

// --- UART_3: GNSS ---
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    gnss_uart_callback();
}

// ============================ UART Error ISR ============================

/* UART0 communication error handler */
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}

/* UART1 communication error handler */
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}

/* UART2 communication error handler */
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}

/* UART3 communication error handler */
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
