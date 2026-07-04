/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Remember.h
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Remember (replay) mode macros and function declarations
             (Global variables are declared in Ctrl.h, defined in Ctrl.c)
**************************************************/

#ifndef __CTRL_REMEMBER_H
#define __CTRL_REMEMBER_H

#include "zf_common_headfile.h"
#include "Ctrl.h"
#include "Racing_Track.h"

/**********************************************
* Remember Mode Speed Curve Macros
**********************************************/
#define REMEMBER_SPEED_LOW_RATIO    0.05f
#define REMEMBER_SPEED_RAMP_RATIO   0.10f
#define REMEMBER_SPEED_DECEL_RATIO  0.80f

#define REMEMBER_CURVE_ADVANCE         4000.0f
#define REMEMBER_CURVE_RADIUS          REMEMBER_CURVE_ADVANCE
#define REMEMBER_CURVE_HEADING_KP      2.0f
#define HEADING_BUF_SIZE               11
#define STRAIGHT_HEADING_KP            1.0f
#define STRAIGHT_HEADING_KD            0.2f
#define HEADING_LOCK_MAX_ANGLE         2.0f
#define REMEMBER_CURVE_TARGET_DEG      90.0f
#define CURVE_BASE_SPEED               160
#define REMEMBER_TURN_TARGET_ANGLE_DEG 90.0f
#define TURN_BASE_SPD_MIN              140
#define TURN_BASE_SPD_MAX              180
#define REMEMBER_TURN_INNER_SCALE      1.4f
#define REMEMBER_TURN_OUTER_SCALE      0.6f

#define CURVE_DIFF_R5600   29
#define CURVE_DIFF_R5200   31
#define CURVE_DIFF_R4800   33
#define CURVE_DIFF_R4400   36
#define CURVE_DIFF_R4000   40
#define CURVE_DIFF_R3600   52

#define CURVE_VARIABLE_DIFF_START  70
#define CURVE_VARIABLE_DIFF_END    50
#define CURVE_VARIABLE_DIFF_KNEE   20.0f

#define REMEMBER_SPEED_MIN            160
#define REMEMBER_SPEED_MAX            160
#define REMEMBER_TURN_KP_AT_160       52.0f
#define REMEMBER_TURN_ERR             55

#define STRAIGHT_SHORT_MILEAGE        2700.0f
#define STRAIGHT_MID_MILEAGE          2800.0f
#define STRAIGHT_LONG_MILEAGE         2900.0f
#define STRAIGHT_NODE_MILEAGE         250.0f

#define NODE_TURN_PRE_STRAIGHT          1400.0f
#define ELEM_TURN_PRE_STRAIGHT_SMALL    2400.0f
#define ELEM_TURN_PRE_STRAIGHT_BIG      3000.0f
#define ELEM_TURN_POST_STRAIGHT_SMALL   350.0f
#define ELEM_TURN_POST_STRAIGHT_BIG     300.0f

#define TURN_APPROACH_MIN_SPEED         20
#define TURN_ROTATE_DIFF                125

/**********************************************
* Function Declarations + Variable Externs
**********************************************/
#if ACTIVE_MODE == MODE_REMEMBER
void Remember_Mode_Get_Error(void);
void Remember_Set_Speed(void);

// Variables defined in Ctrl_Remember.c, referenced by Ctrl.c #if REMEMBER branches
extern float gyro_buf[HEADING_BUF_SIZE];
extern uint8_t heading_buf_idx;
extern uint8_t heading_buf_full;
extern float Remember_Section_Start_Mileage;
extern uint8_t Remember_Turn_Index;
extern float Straight_Heading_Target;
extern uint8_t Straight_Heading_Locked;
extern float Straight_Last_Heading_Err;
extern float curve_odom_x;
extern float curve_odom_y;
extern float curve_last_odom_y;
extern float curve_current_advance;
extern uint8_t curve_variable_diff;
extern int   curve_diff;
extern uint8_t curve_direction;
extern uint8_t elem_approach_pending;
extern float Remember_Trigger_Value;
extern float Remember_Section_Total;
extern int   Remember_Turn_Error;
extern int   Remember_Speed_Min_Value;
extern int   Remember_Speed_Max_Value;
#endif

#endif
