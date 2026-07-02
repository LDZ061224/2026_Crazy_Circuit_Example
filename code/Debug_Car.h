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

/*********************************** 閫熷害鍓嶉鐩稿叧瀹氫箟 ***********************************/

/* 鍓嶉琛ㄥ崟椤癸細鐩爣閫熷害 -> 鍩虹 PWM */
typedef struct {
    float speed;   // 鐩爣閫熷害锛堢紪鐮佸櫒 tick/3ms锛�
    float pwm;     // 鍓嶉 PWM 鍗犵┖姣旓紙0~10000锛�
} Speed_FF_Point_t;

/* 角速度前馈表单项：目标角速度 -> 左右轮差速 */
typedef struct {
    float gyro_rate;   // 目标角速度 (deg/s)
    float delta_v;     // 左右轮差速（编码器 tick/3ms）
} Gyro_FF_Point_t;

#define GYRO_FF_TABLE_SIZE  7

/* 宸﹀彸杞墠棣堣〃琛ㄩ」鏁� */
#define LEFT_SPEED_FF_TABLE_SIZE   9
#define RIGHT_SPEED_FF_TABLE_SIZE  9

/* 鐢靛帇琛ュ伩鍙傛暟 */
#define SPEED_FF_VOLTAGE_REF    12.0f   // 鍓嶉琛ㄦ爣瀹氭椂鐨勫弬鑰冪數鍘�
#define SPEED_FF_VOLTAGE_MIN    11.1f   // 鐢垫満棰濆畾鐢靛帇锛堜笅闄愰挸浣嶏級
#define SPEED_FF_VOLTAGE_MAX    12.6f   // 3S 婊＄數鐢靛帇锛堜笂闄愰挸浣嶏級
#define SPEED_FF_COMP_MIN       0.90f   // 琛ュ伩鍊嶇巼涓嬮檺
#define SPEED_FF_COMP_MAX       1.12f   // 琛ュ伩鍊嶇巼涓婇檺

/* 鐢靛帇婊ゆ尝鍙傛暟 */
#define VOLTAGE_FAST_ALPHA      0.10f   // 蹇�� EMA 绯绘暟
#define VOLTAGE_SLOW_ALPHA      0.01f   // 鎱㈤�� EMA 绯绘暟
#define VOLTAGE_SPIKE_LIMIT     1.0f    // 鍗曟閲囨牱灏栧嘲鍓旈櫎闃堝�硷紙V锛�

/***********************************public API***********************************/

/*  per-tick entry called from PIT ISR (replaces the Debug_Mode case in Car_Go) */
void Debug_Car_Go(void);

#endif  // __DEBUG_CAR_H
