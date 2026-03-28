/*************************************************
Copyright (C), 2016-2017, TYUT JBD TEAM C.
File name: I2C.h
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:  I2C.h
Others:      ??
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        ???
AmaZzzing 2016.11.12  1.0        ??????????
???&???   2020.6.1    ??????      ?????????
**************************************************/
#ifndef __IIC_H
#define __IIC_H

#include "headfiles.h"

///************************************????************************************/

/*********************************??????????*********************************/
/************************************????**********************************/

/* ??? IIC????уг??IIC????? SCL??SDA??????? ???????????????????????? */
/* ??? IIC??????IIC????????╕к ????MPU6050 ???????? 0xD0 */
/* ??????IO ????????? OLED_IIC_SCL_PIN ?? OLED_IIC_SDA_PIN ???? */

#define OLED_SDA_OUT        SDA_GPIO_out()
#define OLED_SDA_IN         SDA_GPIO_in()

#define OLED_IIC_SCL_INIT   SCL_GPIO_init()
#define OLED_IIC_SDA_INIT   SDA_GPIO_init()

#define OLED_IIC_SCL_H      HAL_GPIO_WritePin(GPIOB, OLED_IIC_SCL_PIN, GPIO_PIN_SET)
#define OLED_IIC_SCL_L      HAL_GPIO_WritePin(GPIOB, OLED_IIC_SCL_PIN, GPIO_PIN_RESET)

#define OLED_IIC_SDA_H      HAL_GPIO_WritePin(GPIOB, OLED_IIC_SDA_PIN, GPIO_PIN_SET)
#define OLED_IIC_SDA_L      HAL_GPIO_WritePin(GPIOB, OLED_IIC_SDA_PIN, GPIO_PIN_RESET)

#define OLED_IIC_SDA_READ   HAL_GPIO_ReadPin(GPIOB, OLED_IIC_SDA_PIN)

/***********************************????????***********************************/
void Delay_us(uint32_t nus);

extern void I2c_Start(void);
extern void I2c_Stop(void);
extern void I2c_Write_OneByte(unsigned char data_t);
extern unsigned char I2c_Read_OneByte(uint8_t ack);
extern void IIC_Ack(void);
extern void IIC_NAck(void);
extern void IIC_Delay_us (void);
extern void I2c_Start(void);
extern unsigned char I2C_Wait_Ask(void);
extern void I2c_Stop(void);
#endif
