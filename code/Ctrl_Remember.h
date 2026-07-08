/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Remember.h
Author: Cross_Z
Version:4.1               Date: 2026.7.6
Description: Remember (replay) mode — simplified
             All actions triggered by Check_Edge (sensor), mileage for speed curve only
**************************************************/

#ifndef __CTRL_REMEMBER_H
#define __CTRL_REMEMBER_H

#include "zf_common_headfile.h"
#include "Ctrl.h"
#include "Racing_Track.h"

/**********************************************
* Speed Curve Macros — from Hardware_Config.h
**********************************************/
#define REMEMBER_SPEED_LOW_RATIO    REM_SPEED_LOW_RATIO
#define REMEMBER_SPEED_RAMP_RATIO   REM_SPEED_RAMP_RATIO
#define REMEMBER_SPEED_DECEL_RATIO  REM_SPEED_DECEL_RATIO

/**********************************************
* Heading Lock Macros
**********************************************/
#define HEADING_BUF_SIZE               HEADING_BUF_SIZE_VAL
#define STRAIGHT_HEADING_KP            STRAIGHT_HEADING_KP_VAL
#define STRAIGHT_HEADING_KD            STRAIGHT_HEADING_KD_VAL
#define HEADING_LOCK_MAX_ANGLE         HEADING_LOCK_MAX_ANGLE_VAL

/**********************************************
* Turn Macros
**********************************************/
#define REMEMBER_TURN_TARGET_ANGLE_DEG REM_TURN_TARGET_DEG
#define TURN_BASE_SPD_MIN              REM_TURN_BASE_MIN
#define TURN_BASE_SPD_MAX              REM_TURN_BASE_MAX
#define REMEMBER_TURN_INNER_SCALE      REM_TURN_INNER_SCALE
#define REMEMBER_TURN_OUTER_SCALE      REM_TURN_OUTER_SCALE

/**********************************************
* Speed / Turn Parameters
**********************************************/
#define REMEMBER_SPEED_MIN            REM_SPEED_MIN
#define REMEMBER_SPEED_MAX            REM_SPEED_MAX
#define REMEMBER_TURN_KP_AT_160       REM_TURN_KP_AT_160
#define REMEMBER_TURN_ERR             REM_TURN_ERR

/**********************************************
* Straight Mileage Presets
**********************************************/
#define STRAIGHT_SHORT_MILEAGE        REM_STRAIGHT_SHORT
#define STRAIGHT_MID_MILEAGE          REM_STRAIGHT_MID
#define STRAIGHT_LONG_MILEAGE         REM_STRAIGHT_LONG
#define STRAIGHT_NODE_MILEAGE         REM_STRAIGHT_NODE

/**********************************************
* Function Declarations + Variable Externs
**********************************************/
#if ACTIVE_MODE == MODE_REMEMBER

void Remember_Mode_Get_Error(void);
void Remember_Set_Speed(void);

extern float   gyro_buf[HEADING_BUF_SIZE];
extern uint8_t heading_buf_idx;
extern uint8_t heading_buf_full;
extern float   Remember_Section_Start_Mileage;
extern uint8_t Remember_Turn_Index;
extern float   Straight_Heading_Target;
extern uint8_t Straight_Heading_Locked;
extern float   Straight_Last_Heading_Err;
extern int     Remember_Turn_Error;
extern int     Remember_Speed_Min_Value;
extern int     Remember_Speed_Max_Value;

#endif // ACTIVE_MODE == MODE_REMEMBER

#endif
