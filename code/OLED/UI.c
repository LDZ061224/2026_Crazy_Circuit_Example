#include "OLEDkeyboard_Config.h"
#include "UI.h"

int KeyValue_temp = KEY_BLANK;   //输入的新数字
int KeyValue_old_temp;
int NewVlaue_temp = 0;           //输入区显示出来的数字
int RT_show = 0;


///*************************************************
//Function: KeyboardInput
//Description:键盘输入    (最多5位)
//Details：factor 系数，输入的数在返回时将会乘的系数，1.0为不变
//*************************************************/
//int32 KeyboardInput_RT(uint16 x,uint16 y, TextSize_TypeDef TextSize, float factor) //（without Flash）
//{
//    int KeyValue = KEY_BLANK;   //输入的新数字
//    int KeyValue_old;
//    int NewVlaue = 0;           //输入区显示出来的数字
//
//    /*先在要输入信息的位置显示一行0*/
//    OLED_Show_Numbers(x, y, 0, TextSize);
//
//    if(CH455_Read() != 4)
//    {
//        while(KeyValue != KEY_ENTER)
//        {
//            KeyValue_old = KeyValue;
//            KeyValue = CH455_GetOneKey();
//
//            /*当前不对长按进行特殊处理*/
//            if(KeyValue > 0x0F && KeyValue != KEY_BLANK)
//            {
//                KeyValue = (KeyValue - 0x0F) >> 4;
//            }
//
//            /* 直接跳过的情况取 0 */
//            if(KeyValue_old == KEY_BLANK && (KeyValue == KEY_ENTER || KeyValue == KEY_ENTER_Long))
//            {
//                NewVlaue = 0;
//            }
//
//            /****************************输入数字可退格****************************/
//            if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
//            {
//                if(NewVlaue / 10000 != 0)       //只保留四位
//                {
//                    NewVlaue = NewVlaue % 10000;
//                }
//
//                NewVlaue = NewVlaue * 10 + KeyValue;      //新输入的数字放到个位
//            }
//            else if(KeyValue == KEY_BACK)    //退格
//            {
//                NewVlaue = NewVlaue / 10;     //最后一位去掉
//            }
//
//            OLED_Show_Numbers(x, y, NewVlaue, TextSize);   //显示当前值
//        }
//    }
//    else
//    {
//        return 111111;
//    }
//
//    NewVlaue *= factor;
//
//    return NewVlaue;
//}
//
/////*************************************************
////Function: KeyboardInput_RT
////Description:键盘实时输入    (最多5位)
////Details：factor 系数，输入的数在返回时将会乘的系数，1.0为不变
////*************************************************/
////int32 KeyboardInput_RT(uint16 x,uint16 y, TextSize_TypeDef TextSize, float factor) //（without Flash）
////{
////
////    /*先在要输入信息的位置显示一行0*/
////    if(RT_show == 0)
////    {
////        OLED_Show_Numbers(x, y, 0, TextSize);
////        NewVlaue_temp = 0;//清零
////        RT_show = 1;
////    }
////
////    KeyValue_old_temp = KeyValue_temp;//历史值
////    KeyValue_temp = CH455_QueryOneKey(15000);
////
////    /*当前不对长按进行特殊处理*/
////    if(KeyValue_temp > 0x0F && KeyValue_temp != KEY_BLANK)
////    {
////        KeyValue_temp = (KeyValue_temp - 0x0F) >> 4;
////    }
////
////    /* 直接跳过的情况取 0 */
//////    if(KeyValue_old_temp == KEY_BLANK && (KeyValue_temp == KEY_ENTER || KeyValue_temp == KEY_ENTER_Long))//上次空白，这次回车（没按数字的情况）
//////    {
//////        NewVlaue_temp = 0;
//////    }
////    if(NewVlaue_temp == 0 && KeyValue_old_temp == KEY_BLANK && (KeyValue_temp == KEY_ENTER || KeyValue_temp == KEY_ENTER_Long))
////    {
////        NewVlaue_temp = 0;
////    }
////
////    /****************************输入数字可退格****************************/
////    if(KeyValue_temp != KEY_BACK && KeyValue_temp != KEY_ENTER && KeyValue_temp != KEY_BLANK)   //输入数值
////    {
////        if(NewVlaue_temp / 10000 != 0)       //只保留四位
////        {
////            NewVlaue_temp = NewVlaue_temp % 10000;
////        }
////
////        NewVlaue_temp = NewVlaue_temp * 10 + KeyValue_temp;      //新输入的数字放到个位
////    }
////    else if(KeyValue_temp == KEY_BACK)    //退格
////    {
////        NewVlaue_temp = NewVlaue_temp / 10;     //最后一位去掉
////    }
////
////    OLED_Show_Numbers(x, y, NewVlaue_temp, TextSize);   //显示当前值
////
////    if(KeyValue_temp == KEY_ENTER)
////    {
////        return NewVlaue_temp;
////    }
////    else
////    {
////        return 111111;
////    }
////}
//
/*************************************************
Function: KeyboardInput
Description:键盘输入    (最多5位)
Details：factor 系数，输入的数在返回时将会乘的系数，1.0为不变
*************************************************/
int32 KeyboardInput(uint16 x ,uint16 y , TextSize_TypeDef TextSize, float factor) //（without Flash）
{
    int KeyValue = KEY_BLANK;   //输入的新数字
    int KeyValue_old;
    int NewVlaue = 0;           //输入区显示出来的数字

    /*先在要输入信息的位置显示一行0*/
    OLED_Show_Numbers(x, y, 0, TextSize);

    while(KeyValue != KEY_ENTER)
    {
        KeyValue_old = KeyValue;
        KeyValue = CH455_GetOneKey();

        /*当前不对长按进行特殊处理*/
        if(KeyValue > 0x0F && KeyValue != KEY_BLANK)
        {
            KeyValue = (KeyValue - 0x0F) >> 4;
        }

        /* 直接跳过的情况取 0 */
        if(KeyValue_old == KEY_BLANK && (KeyValue == KEY_ENTER || KeyValue == KEY_ENTER_Long))
        {
            NewVlaue = 0;
        }

        /****************************输入数字可退格****************************/
        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
            if(NewVlaue / 10000 != 0)       //只保留四位
            {
                NewVlaue = NewVlaue % 10000;
            }

            NewVlaue = NewVlaue * 10 + KeyValue;      //新输入的数字放到个位
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            NewVlaue = NewVlaue / 10;     //最后一位去掉
        }

        OLED_Show_Numbers(x, y, NewVlaue, TextSize);   //显示当前值
    }

    NewVlaue *= factor;

    return NewVlaue;
}

/*************************************************
Function: KeyboardInput_with_Flash
Description:键盘输入并存入Flash    (最多5位)
Details：大号字体显示四位数，小号字体显示五位
*************************************************/
int32 KeyboardInput_with_Flash(uint16 x,uint16 y, TextSize_TypeDef TextSize, float factor, uint16 sector, uint16 page)
{
    int KeyValue = KEY_BLANK;    //输入的新数字
    int KeyValue_old;
    uint32 NewVlaue = 0;           //输入区显示出来的数字

    /*先在要输入信息的位置显示一行0*/
    OLED_Show_Numbers(x, y, 0, TextSize);

    while(KeyValue != KEY_ENTER)
    {
        KeyValue_old = KeyValue;
        KeyValue = CH455_GetOneKey();

        /* 当前不对长按进行特殊处理 */
        if(KeyValue > 0x0F && KeyValue != KEY_BLANK)
        {
            KeyValue = (KeyValue - 0x0F) >> 4;
        }

        /* 直接跳过的情况保留原值 */
        if(KeyValue_old == KEY_BLANK && (KeyValue == KEY_ENTER || KeyValue == KEY_ENTER_Long))
        {
            flash_read_page(Backup_Sector, page, &NewVlaue, 256);
        }

        /****************************输入数字可退格****************************/
        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
            if(NewVlaue / 10000 != 0)       //只保留5位
            {
                NewVlaue = NewVlaue % 10000;
            }

            NewVlaue = NewVlaue * 10 + KeyValue;      //新输入的数字放到个位
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            NewVlaue = NewVlaue / 10;     //最后一位去掉
        }

        OLED_Show_Numbers(x, y, NewVlaue, TextSize);   //显示当前值
    }

    NewVlaue *= factor;

    flash_erase_page(sector, page);

    flash_write_page(sector, page, &NewVlaue, 9);
    return NewVlaue;
}

