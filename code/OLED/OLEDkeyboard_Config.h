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

/*=====闇�瑕佹洿鏀规垨鏄剧ず鐨勫彉閲忓０鏄庣殑澶存枃浠�=====*/
//#include "zf_device_mt9v03x.h"
//#include "Image.h"
//#include "Getinfor.h"
/*================瀹忓畾涔�================*/

typedef enum
{
    Switch_OFF = 0,
    Switch_ON  = !Switch_OFF,
}SwitchStatus_typeDef;

/*==== simiic ====*/
#define JBD_simiic_SCL_PIN          P21_5
#define JBD_simiic_SDA_PIN          P21_7

/*==== OLED ====*/
#define Backup_Sector                   (0U)
/*===============澶栭儴鍙橀噺===============*/

/*===============鍑芥暟澹版槑===============*/




#endif  /*__OLEDKEYBOARD_CONFIG_H*/

