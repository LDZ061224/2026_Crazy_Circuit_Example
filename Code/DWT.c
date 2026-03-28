#define  DWT_CYCCNT  *(volatile unsigned int *)0xE0001004
#define  DWT_CR      *(volatile unsigned int *)0xE0001000
#define  DEM_CR      *(volatile unsigned int *)0xE000EDFC

#define  DEM_CR_TRCENA               (1 << 24)
#define  DWT_CR_CYCCNTENA            (1 <<  0)

#include "headfiles.h"

/******************************************************************************************
******************************网上找的DWT方式实现的us延时************************************
******************************************************************************************/

/******************************************************************************
 * @brief  初始化 DWT
 * @return none
******************************************************************************/
void bsp_dwt_init(void)
{
    DEM_CR         |= (unsigned int)DEM_CR_TRCENA;   /* Enable Cortex-M4's DWT CYCCNT reg.  */
	DWT_CYCCNT      = (unsigned int)0u;
	DWT_CR         |= (unsigned int)DWT_CR_CYCCNTENA;
}

/******************************************************************************
 * @brief      
 * @param[in]  _delay_time    :    延时时间  
 * @return     none
******************************************************************************/
void bsp_dwt_delay(uint32_t _delay_time)
{
	uint32_t cnt, delay_cnt;
	uint32_t start;
		
	cnt = 0;
	delay_cnt = _delay_time;  /* 需要的节拍数 */ 		      
	start = DWT_CYCCNT;       /* 刚进入时的计数器值 */
	
	while(cnt < delay_cnt)
	{
		cnt = DWT_CYCCNT - start; /* 求减过程中，如果发生第一次32位计数器重新计数，依然可以正确计算 */	
	}
}

/******************************************************************************
 * @brief      这里的延时采用CPU的内部计数实现，32位计数器
 *             OSSchedLock(&err);
 *			   bsp_DelayUS(5);
 *			   OSSchedUnlock(&err); 根据实际情况看看是否需要加调度锁或选择关中断
 * @param[in]  _delay_time    :    延迟长度，单位1 us
 * @return     none
 * @note       
******************************************************************************/
void bsp_delay_us(uint32_t _delay_time)
{
	uint32_t cnt, delay_cnt;
	uint32_t start;
		
	start = DWT_CYCCNT;                                     /* 刚进入时的计数器值 */
	cnt = 0;
	delay_cnt = _delay_time * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */ 		      

	while(cnt < delay_cnt)
	{
		cnt = DWT_CYCCNT - start; /* 求减过程中，如果发生第一次32位计数器重新计数，依然可以正确计算 */	
	}
}

/******************************************************************************
 * @brief      为了让底层驱动在带RTOS和裸机情况下有更好的兼容性
 *             专门制作一个阻塞式的延迟函数，在底层驱动中ms毫秒延迟主要用于初始化，并不会影响实时性。 
 * @param[in]  _delay_time    :    延迟长度，单位1 ms
 * @return     none
******************************************************************************/
void bsp_delay_ms(uint32_t _delay_time)
{
	bsp_delay_us(1000 * _delay_time);
}

