#ifndef __OLED_H__
#define __OLED_H__

#include "gpio.h"
#include <stdint.h>
#include "OLEDfun.h"
#include "OLEDKeyboard.h"
#include "ssd1306.h"
#include "IIC.h"
#include "CH455.h"
#include "headfiles.h"

// #define First_X                 50  //�ϴβ�����X����
// #define Second_X                85  //�������������X����
// #define MAX_PAGE_NUM 50
// #define BACKUP_SECTOR 11    //ʱݵ
#define OLED_CLS(void)  OLED_Fill(0x00)

#define KEY_ENTER_TIME  200    //ȷϼʱ

#endif
