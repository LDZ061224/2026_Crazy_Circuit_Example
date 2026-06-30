/*********************************************************************************************************************
* TC264 Open Source Library (third-party open source library based on official SDK interfaces)
* Copyright (c) 2022 SEEKFREE (Zhufei Technology)
*
* This file is part of the TC264 Open Source Library
*
* TC264 Open Source Library is free software
* You may redistribute and/or modify it under the terms of the GNU General Public License
* version 3 (GPL3.0) or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GPL for more details.
*
* You should have received a copy of the GPL along with this library.
* If not, see <https://www.gnu.org/licenses/>
*
* Additional notes:
* This library uses the GPL3.0 open source license agreement; the above statement is a translated version.
* The English license statement is in libraries/doc/GPL3_permission_statement.txt
* The license copy is under the libraries folder, i.e. the LICENSE file in that folder.
* Welcome to use and share this program, but when modifying, you must retain Zhufei Technology's copyright statement (i.e., this statement).
*
* Filename           isr_config
* Company            Chengdu Zhufei Technology Co., Ltd.
* Version info       See libraries/doc/version file for version description
* Dev environment    ADS v1.10.2
* Platform           TC264D
* Store link         https://seekfree.taobao.com/
*
* Modification history
* Date               Author              Notes
* 2022-09-15         pudding             first version
********************************************************************************************************************/

#ifndef _isr_config_h
#define _isr_config_h


//====================================================================================================================
// IMPORTANT: Interrupt priorities MUST NOT be set to the same value — every interrupt must have a unique priority.
//====================================================================================================================
//====================================================================================================================
// IMPORTANT: Interrupt priorities MUST NOT be set to the same value — every interrupt must have a unique priority.
//====================================================================================================================
//====================================================================================================================
// IMPORTANT: Interrupt priorities MUST NOT be set to the same value — every interrupt must have a unique priority.
//====================================================================================================================

// ISR_PRIORITY:  TC264 provides 255 configurable interrupt priority levels (1-255). Priority 0 means interrupt disabled; 255 is the highest priority.
// INT_SERVICE:   Macro that determines who handles the interrupt, also known as the service provider.
//                (On TC264, interrupts are called "services"). Valid range: IfxSrc_Tos_cpu0, IfxSrc_Tos_cpu1, IfxSrc_Tos_dma.
//                Other values are not allowed.
//                If INT_SERVICE is set to IfxSrc_Tos_dma, ISR_PRIORITY range is 0-47.

//================================================== PIT interrupt parameter definitions ==============================================
#define CCU6_0_CH0_INT_SERVICE	IfxSrc_Tos_cpu0	    // CCU6_0 PIT channel 0 interrupt service type: defines which core handles this interrupt. IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma only.
#define CCU6_0_CH0_ISR_PRIORITY 10	                // 3ms PIT interrupt (Car_Go core beat)

#define CCU6_0_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 0                       // Unused: closed

#define CCU6_1_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 0                       // Unused: closed

#define CCU6_1_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 0                       // Unused: closed



//================================================== GPIO (ERU) interrupt parameter definitions ==============================================
// Channels 0 and 4 share one ISR function; inside the ISR, flag bits determine which channel triggered.
// Actual pins used: CH0_REQ0_P15_4 / CH4_REQ13_P15_5 (idle handling)
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0	// ERU channel 0 and channel 4 interrupt service type: defines which core handles. IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma only.
#define EXTI_CH0_CH4_INT_PRIO  	0	                // Unused: closed

// Channels 1 and 5 share one ISR function; inside the ISR, flag bits determine which channel triggered.
// CH1_REQ10_P14_3: ToF module (deprecated); CH5_REQ1_P15_8: idle handling
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0	// ERU channel 1 and channel 5 interrupt service type, same as above.
#define EXTI_CH1_CH5_INT_PRIO  	0	                // Unused: closed

// Channels 2 and 6 share one ISR function; inside the ISR, flag bits determine which channel triggered.
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma	    // ERU channel 2 and channel 6 interrupt service type, same as above.
#define EXTI_CH2_CH6_INT_PRIO  	5	                // ERU channel 2 and channel 6 interrupt priority. Valid range: 0-47.

// Channels 3 and 7 share one ISR function; inside the ISR, flag bits determine which channel triggered.
// Camera not in use
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0	// ERU channel 3 and channel 7 interrupt service type, same as above.
#define EXTI_CH3_CH7_INT_PRIO  	0	                // Unused: closed

// Channels 2 and 6 share one ISR function; inside the ISR, flag bits determine which channel triggered.
// Camera PCLK DMA not in use
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma	    // ERU channel 2 and channel 6 interrupt service type, same as above.
#define EXTI_CH2_CH6_INT_PRIO  	0	                // Unused: closed


//=================================================== DMA interrupt parameter definitions ===============================================
#define	DMA_INT_SERVICE         IfxSrc_Tos_cpu0	    // ERU-triggered DMA interrupt service type: defines which core handles. IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma only.
#define DMA_INT_PRIO  	        0	                // Camera DMA not in use: closed


//=================================================== UART interrupt parameter definitions ===============================================
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0	    // UART0 interrupt service type: defines which core handles. IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma only.
#define UART0_TX_INT_PRIO       0                    // UART0 TX interrupt priority. Range: 1-255, higher = higher priority. (RX @STOP# needs highest priority.)
#define UART0_RX_INT_PRIO       0	                // UART0 RX interrupt highest priority (emergency STOP# stop command)
#define UART0_ER_INT_PRIO       0	                // UART0 error interrupt, second-highest priority

// UART1-UART3 not in use, interrupts closed
#define	UART1_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART1_TX_INT_PRIO       0                   // Unused: closed
#define UART1_RX_INT_PRIO       0                   // Unused: closed
#define UART1_ER_INT_PRIO       0                   // Unused: closed

#define	UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       100                  // UART2 TX interrupt
#define UART2_RX_INT_PRIO       255                 // UART2 RX interrupt (serial tuning)
#define UART2_ER_INT_PRIO       254                 // UART2 error interrupt

#define	UART3_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO       0                   // Unused: closed
#define UART3_RX_INT_PRIO       0                   // Unused: closed
#define UART3_ER_INT_PRIO       0                   // Unused: closed


#endif
