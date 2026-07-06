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
* Speed Curve Macros
**********************************************/
#define REMEMBER_SPEED_LOW_RATIO    0.05f
#define REMEMBER_SPEED_RAMP_RATIO   0.10f
#define REMEMBER_SPEED_DECEL_RATIO  0.95f

/**********************************************
* Heading Lock Macros
**********************************************/
#define HEADING_BUF_SIZE               11
#define STRAIGHT_HEADING_KP            1.0f
#define STRAIGHT_HEADING_KD            0.2f
#define HEADING_LOCK_MAX_ANGLE         2.0f

/**********************************************
* Turn Macros
**********************************************/
#define REMEMBER_TURN_TARGET_ANGLE_DEG 90.0f
#define TURN_BASE_SPD_MIN              140
#define TURN_BASE_SPD_MAX              180
#define REMEMBER_TURN_INNER_SCALE      1.4f
#define REMEMBER_TURN_OUTER_SCALE      0.6f

/**********************************************
* Speed / Turn Parameters
**********************************************/
#define REMEMBER_SPEED_MIN            160
#define REMEMBER_SPEED_MAX            160
#define REMEMBER_TURN_KP_AT_160       52.0f
#define REMEMBER_TURN_ERR             55

/**********************************************
* Straight Mileage Presets
**********************************************/
#define STRAIGHT_SHORT_MILEAGE        2700.0f
#define STRAIGHT_MID_MILEAGE          2800.0f
#define STRAIGHT_LONG_MILEAGE         2900.0f
#define STRAIGHT_NODE_MILEAGE         250.0f

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
