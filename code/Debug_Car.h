/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Debug_Car.h
Author: Claude (extracted from Ctrl.c)
Version:0.0               Date: 2026.6.29
Description:  Debug car control -- PI tuning / ground test / angle / normal trace
              Separated from Ctrl.c, call Debug_Car_Go() instead of Car_Go()
              when USE_DEBUG_MODE is enabled.
Others:      Requires Ctrl.h externs for shared globals.
**************************************************/

#ifndef __DEBUG_CAR_H
#define __DEBUG_CAR_H

#include "headfiles.h"
#include "Ctrl.h"

/***********************************mode switch macro***********************************/
/*
 *  USE_DEBUG_MODE = 1   -> Car_Go() dispatches to Debug_Car_Go() when Mode==Debug_Mode
 *  USE_DEBUG_MODE = 0   -> Car_Go() runs only normal racing (compile out debug branch)
 */
#define USE_DEBUG_MODE  1


/*********************************** 闁喎瀹抽崜宥夘洯閻╃鍙х�规矮绠� ***********************************/

/* 閸撳秹顩悰銊ュ礋妞ょ櫢绱伴惄顔界垼闁喎瀹� -> 閸╄櫣顢� PWM */
typedef struct {
    float speed;   // 閻╊喗鐖ｉ柅鐔峰閿涘牏绱惍浣告珤 tick/3ms閿涳拷
    float pwm;     // 閸撳秹顩� PWM 閸楃姷鈹栧В鏃撶礄0~10000閿涳拷
} Speed_FF_Point_t;

/* 瑙掗�熷害鍓嶉琛ㄥ崟椤癸細鐩爣瑙掗�熷害 -> 宸﹀彸杞樊閫� */
typedef struct {
    float gyro_rate;   // 鐩爣瑙掗�熷害 (deg/s)
    float delta_v;     // 宸﹀彸杞樊閫燂紙缂栫爜鍣� tick/3ms锛�
} Gyro_FF_Point_t;

#define GYRO_FF_TABLE_SIZE  7

/* 瀹革箑褰告潪顔煎妫ｅ牐銆冪悰銊┿�嶉弫锟� */
#define LEFT_SPEED_FF_TABLE_SIZE   9
#define RIGHT_SPEED_FF_TABLE_SIZE  9

/* Voltage compensation — from Hardware_Config.h */
#define SPEED_FF_VOLTAGE_REF    DEBUG_SPEED_FF_REF
#define SPEED_FF_VOLTAGE_MIN    DEBUG_SPEED_FF_V_MIN
#define SPEED_FF_VOLTAGE_MAX    DEBUG_SPEED_FF_V_MAX
#define SPEED_FF_COMP_MIN       DEBUG_SPEED_FF_COMP_MIN
#define SPEED_FF_COMP_MAX       DEBUG_SPEED_FF_COMP_MAX

/* Voltage filter — from Hardware_Config.h */
#define VOLTAGE_FAST_ALPHA      DEBUG_VOLT_FAST_ALPHA
#define VOLTAGE_SLOW_ALPHA      DEBUG_VOLT_SLOW_ALPHA
#define VOLTAGE_SPIKE_LIMIT     DEBUG_VOLT_SPIKE_LIMIT

/***********************************public API***********************************/

/*  per-tick entry called from PIT ISR (replaces the Debug_Mode case in Car_Go) */
void Debug_Car_Go(void);

#endif  // __DEBUG_CAR_H
