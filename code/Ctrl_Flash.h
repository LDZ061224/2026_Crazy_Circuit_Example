/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Flash.h
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Flash storage layer — mileage data persistence
             Ported from CarbonV2.0
**************************************************/

#ifndef __CTRL_FLASH_H
#define __CTRL_FLASH_H

#include "zf_common_headfile.h"
#include "Mode_Config.h"

/**********************************************
* Function Declarations (Build+Remember only)
**********************************************/

#if ACTIVE_MODE == MODE_BUILD || ACTIVE_MODE == MODE_REMEMBER

// Generic multi-page Flash read/write
void Save_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count,
                           uint16_t words_per_page, const uint32 *words);
void Load_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count,
                           uint16_t words_per_page, uint32 *words);

// Mileage data persistence (Build saves after finish, Remember loads at startup)
void Save_Mileage_Data_To_Flash(void);
void Load_Mileage_Data_From_Flash(void);

extern uint8_t Flash_Save_Pending;

#endif // ACTIVE_MODE == MODE_BUILD || ACTIVE_MODE == MODE_REMEMBER

#endif
