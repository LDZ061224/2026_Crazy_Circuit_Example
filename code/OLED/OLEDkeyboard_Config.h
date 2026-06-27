#ifndef __OLEDKEYBOARD_CONFIG_H
#define __OLEDKEYBOARD_CONFIG_H

#include "zf_driver_gpio.h"
#include "JBD_simiic.h"
#include "Font.h"
#include "dev_ssd1306.h"
#include "dev_CH455.h"
#include "FlashFun.h"
#include "UI.h"
#include "OLEDkeyboard.h"

/*=====需要更改或显示的变量声明的头文件=====*/
//#include "zf_device_mt9v03x.h"
//#include "Image.h"
//#include "Getinfor.h"
/*================宏定义================*/

typedef enum
{
    Switch_OFF = 0,
    Switch_ON  = !Switch_OFF,
}SwitchStatus_typeDef;

/*==== simiic ====*/
#define JBD_simiic_SCL_PIN          P33_8
#define JBD_simiic_SDA_PIN          P33_6

/*==== OLED ====*/
#define Backup_Sector                   (0U)
/*===============外部变量===============*/

/*===============函数声明===============*/




#endif  /*__OLEDKEYBOARD_CONFIG_H*/

