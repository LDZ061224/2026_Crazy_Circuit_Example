/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Flash.c
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Flash mileage persistence — Build saves, Remember loads
             Ported from CarbonV2.0
**************************************************/

#include "Ctrl_Flash.h"
#include "headfiles.h"
#include "Racing_Track.h"

// Only compile when Build or Remember mode is active (Debug doesn't need Flash mileage)
#if ACTIVE_MODE == MODE_BUILD || ACTIVE_MODE == MODE_REMEMBER

/*--------------- Flash Storage Struct -----------------*/

/**
 * @brief Mileage data Flash packed struct (edge mileage + turn spacing)
 * @note  Data size = 60×4 + 1 + 60×4 + 1 = 482 bytes, occupies 2 pages (512B)
 */
typedef struct
{
    float   Edge_Mileage_Flash[MAX_ACTIONS];    // Edge mileage array (240B)
    uint8_t Edge_Count;                          // Edge count
    float   Turn_Distance_Flash[MAX_ACTIONS];    // Turn spacing array (240B)
    uint8_t Turn_Count;                          // Turn spacing count
    uint8_t _pad;                                // Alignment padding (4-byte align)
} Mileage_Flash_Typedef;

/*--------------- Flash Sector/Page Config -----------------*/

/*
 * Mileage data Flash config
 * Sector 0, pages 4-5, 2 pages (512 bytes, enough for 60×2 float entries + 2×counts)
 */
#define MILEAGE_FLASH_SECTOR          0
#define MILEAGE_FLASH_START_PAGE      4
#define MILEAGE_FLASH_PAGE_COUNT      2
#define MILEAGE_FLASH_WORDS_PER_PAGE  64

uint8_t Flash_Save_Pending = 0;

/********************************* Flash R/W Functions *********************************/

/*************************************
** Function: Save_Flash_Page_Block
** Description: Generic multi-page Flash write — erase then write each page
*************************************/
void Save_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count,
                           uint16_t words_per_page, const uint32 *words)
{
    uint8_t page_index;

    for (page_index = 0; page_index < page_count; page_index++)
    {
        flash_erase_page(sector, start_page + page_index);
        flash_write_page(sector,
                         start_page + page_index,
                         &words[page_index * words_per_page],
                         words_per_page);
    }
}

/*************************************
** Function: Load_Flash_Page_Block
** Description: Generic multi-page Flash read
*************************************/
void Load_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count,
                           uint16_t words_per_page, uint32 *words)
{
    uint8_t page_index;

    for (page_index = 0; page_index < page_count; page_index++)
    {
        flash_read_page(sector,
                        start_page + page_index,
                        &words[page_index * words_per_page],
                        words_per_page);
    }
}

/*************************************
** Function: Save_Mileage_Data_To_Flash
** Description: Save edge mileage + turn spacing to Flash
** Details:   Called from main loop after Build mode finishes
**            Edge_Mileage_Rec + Turn_Distance_Rec → Flash sector 0, pages 4-5
*************************************/
void Save_Mileage_Data_To_Flash(void)
{
    Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[MILEAGE_FLASH_PAGE_COUNT * MILEAGE_FLASH_WORDS_PER_PAGE] = {0};

    memcpy(flash_log.Edge_Mileage_Flash,
           Edge_Mileage_Rec.Edge_Mileage,
           sizeof(Edge_Mileage_Rec.Edge_Mileage));
    flash_log.Edge_Count = Edge_Mileage_Rec.Edge_Count;

    memcpy(flash_log.Turn_Distance_Flash,
           Turn_Distance_Rec.Turn_Distance,
           sizeof(Turn_Distance_Rec.Turn_Distance));
    flash_log.Turn_Count = Turn_Distance_Rec.Turn_Count;

    memcpy(map_words, &flash_log, sizeof(flash_log));
    Save_Flash_Page_Block(MILEAGE_FLASH_SECTOR,
                          MILEAGE_FLASH_START_PAGE,
                          MILEAGE_FLASH_PAGE_COUNT,
                          MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);
}

#endif // ACTIVE_MODE == MODE_BUILD || ACTIVE_MODE == MODE_REMEMBER

/*************************************
** Function: Load_Mileage_Data_From_Flash
** Description: Load edge mileage + turn spacing from Flash
** Details:   Called at startup when Remember mode is active
**            Includes empty-chip (0xFF) and NaN/Inf validation
*************************************/
#if ACTIVE_MODE == MODE_REMEMBER

void Load_Mileage_Data_From_Flash(void)
{
    Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[MILEAGE_FLASH_PAGE_COUNT * MILEAGE_FLASH_WORDS_PER_PAGE] = {0};

    Load_Flash_Page_Block(MILEAGE_FLASH_SECTOR,
                          MILEAGE_FLASH_START_PAGE,
                          MILEAGE_FLASH_PAGE_COUNT,
                          MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);

    memcpy(&flash_log, map_words, sizeof(flash_log));

    // Empty chip (0xFF) check: abnormal count → treat as invalid, fallback to 0
    if (flash_log.Edge_Count > MAX_ACTIONS)
        flash_log.Edge_Count = 0;
    if (flash_log.Turn_Count > MAX_ACTIONS)
        flash_log.Turn_Count = 0;

    // Validate Turn_Distance[0] is a valid float (exclude NaN/Inf from empty chip)
    if (flash_log.Turn_Count > 0)
    {
        float v = flash_log.Turn_Distance_Flash[0];
        if (!(v > -1e9f && v < 1e9f))   // NaN/Inf fail this check
            flash_log.Turn_Count = 0;
    }

    Edge_Mileage_Rec.Edge_Count = flash_log.Edge_Count;
    memcpy(Edge_Mileage_Rec.Edge_Mileage,
           flash_log.Edge_Mileage_Flash,
           sizeof(Edge_Mileage_Rec.Edge_Mileage));

    Turn_Distance_Rec.Turn_Count = flash_log.Turn_Count;
    memcpy(Turn_Distance_Rec.Turn_Distance,
           flash_log.Turn_Distance_Flash,
           sizeof(Turn_Distance_Rec.Turn_Distance));
}

#endif // ACTIVE_MODE == MODE_REMEMBER
