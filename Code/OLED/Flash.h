#ifndef __FLASH_PARAM_H
#define __FLASH_PARAM_H

#include <stdint.h>

/*
 * Deprecated definitions (kept here only for historical reference).
 * These 3 macros are NOT used by Flash.c anymore.
 */
// #define USER_PAGE_SIZE   1024
// #define USER_PAGE_NUM    16
// #define USER_FLASH_BASE  0x0800C000

/*
 * Number of parameter pages used by this module.
 * Page index range: [0, PARAM_PAGE_NUM-1].
 * Current implementation uses 20 pages in total.
 */
#define PARAM_PAGE_NUM  20

/* Read one 32-bit parameter value from a page. */
uint32_t FlashParam_Read(uint8_t page);

/*
 * Write one 32-bit parameter value to a page.
 * Note: this operation erases the whole target page before programming.
 */
void     FlashParam_Write(uint8_t page, uint32_t val);
/*
 * Write multiple 32-bit parameter values to a page.
 * Note: this operation erases the whole target page before programming.
 *       the 'len' parameter specifies how many 32-bit words to write,
 *       starting from the target page's first word.
 */
void FlashParam_TrackConfig_Write(uint8_t page, uint32_t* val, uint8_t len);
#endif
