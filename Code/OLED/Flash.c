#include "stm32f1xx_hal.h"
#include "Flash.h"

/*
 * Parameter storage region (top area of internal Flash).
 * STM32F103xE Flash page size is 2 KB.
 * Start address: 0x08036000, total pages: PARAM_PAGE_NUM(20) -> 40 KB.
 */
#define FLASH_PAGE_SIZE   2048U
#define PARAM_BASE_ADDR   0x08036000UL

static uint32_t PageToAddr(uint8_t page)
{
    /* Out-of-range page index falls back to page 0 for safety. */
    if (page >= PARAM_PAGE_NUM) page = 0;
    return PARAM_BASE_ADDR + page * FLASH_PAGE_SIZE;
}

uint32_t FlashParam_Read(uint8_t page)
{
    /* Read first 32-bit word in target parameter page. */
    return *(volatile uint32_t *)PageToAddr(page);
}

void FlashParam_Write(uint8_t page, uint32_t val)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t pageError;
    uint32_t addr = PageToAddr(page);

    HAL_FLASH_Unlock();

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = addr;
    erase.NbPages     = 1;
    
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
    {
        while (1);               /* Erase failed: stay here for debug. */           
    }

    /* Program first 32-bit word in target page. */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, val) != HAL_OK)
    {
        while (1);               /* Program failed: stay here for debug. */ 
    }

    HAL_FLASH_Lock();
}

void FlashParam_TrackConfig_Write(uint8_t page, uint32_t* val, uint8_t len)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t pageError;
    uint32_t addr = PageToAddr(page);

    HAL_FLASH_Unlock();

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = addr;
    erase.NbPages     = 1;
    
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
    {
        while (1);               /* Erase failed: stay here for debug. */           
    }

    /* Program first 32-bit word in target page. */
    for (uint8_t i = 0; i < len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, val[i]) != HAL_OK)
        {
            while (1);               /* Program failed: stay here for debug. */ 
        }
    }
    HAL_FLASH_Lock();
}