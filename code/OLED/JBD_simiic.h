/*************************************************
Copyright (C), 2016-2024, TYUT JBD.
File name: JBD_simiic.h
Author: TYUT JBD
Version:0.0               Date: 2016.11.12
Description:  JBD_simiic.h
Others:      无
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
双车&信标   2020.6.1    儿童节版      兼容英飞凌
**************************************************/
#ifndef __JBD_simiic_H
#define __JBD_simiic_H

#include "OLEDkeyboard_Config.h"
#include "zf_driver_gpio.h"


/* 注意 IIC总线规定，IIC空闲时 SCL和SDA都为高电平 最好外部上拉（一定不能下拉） */
/* 模拟 IIC需要注意，IIC地址左移一位 例如MPU6050 模拟就是地址 0xD0 */
/* 想换用别的IO 直接修改宏定义 OLED_IIC_SCL_PIN 、 OLED_IIC_SDA_PIN 即可 */
#define JBD_simiic_SDA_OUT          gpio_set_dir(JBD_simiic_SDA_PIN, GPO,GPO_PUSH_PULL);
#define JBD_simiic_SDA_IN           gpio_set_dir(JBD_simiic_SDA_PIN, GPI,GPO_PUSH_PULL);

#define JBD_simiic_SCL_INIT         gpio_init(JBD_simiic_SCL_PIN,GPO, GPO_PUSH_PULL, 1);
#define JBD_simiic_SDA_INIT         gpio_init(JBD_simiic_SDA_PIN,GPO, GPO_PUSH_PULL, 1);

#define JBD_simiic_SCL_H            gpio_set_level(JBD_simiic_SCL_PIN, 1);
#define JBD_simiic_SCL_L            gpio_set_level(JBD_simiic_SCL_PIN, 0);

#define JBD_simiic_SDA_H            gpio_set_level(JBD_simiic_SDA_PIN, 1);
#define JBD_simiic_SDA_L            gpio_set_level(JBD_simiic_SDA_PIN, 0);

#define JBD_simiic_SDA_READ         gpio_get_level(JBD_simiic_SDA_PIN)

//#define OLED_RST_HIGH           gpio_set(P33_4, 1);
//#define OLED_RST_LOW            gpio_set(P33_4, 0);



/*********************************全局变量声明*********************************/

/***********************************函数声明***********************************/

static void JBD_simiic_delay(void);
void JBD_simiic_Start(void);
void JBD_simiic_Stop(void);
static void JBD_simiic_Ack(void);
static void JBD_simiic_NAck(void);
uint8 JBD_simiic_Wait_Ask(void);
void JBD_simiic_Write_OneByte(uint8 data);
uint8 JBD_simiic_Read_OneByte(uint8 ack);
#endif/* __JBD_simiic_H */
