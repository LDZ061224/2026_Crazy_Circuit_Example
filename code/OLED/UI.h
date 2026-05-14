#ifndef __UI_H
#define __UI_H

/*================宏定义================*/

/*===============外部变量===============*/

/*===============函数声明===============*/
extern int32 KeyboardInput(uint16 x,uint16 y, TextSize_TypeDef TextSize, float factor);
extern int32 KeyboardInput_GPS(uint16 x,uint16 y, TextSize_TypeDef TextSize, float factor,uint8 Point_Num, int32 Hyper_Select);
extern int32 KeyboardInput_RT(uint16 x,uint16 y, TextSize_TypeDef TextSize, float factor,uint8 Point_Num, int32 Hyper);

#endif /*__UI_H*/

