/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Racing_Track.h
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Track definition — action list for Remember mode replay
Others:      Ported from CarbonV2.0
**************************************************/

#ifndef __RACING_TRACK_H
#define __RACING_TRACK_H

#include "zf_common_headfile.h"

/**********************************************
* Constants
**********************************************/

#define MAX_ACTIONS             60

/**********************************************
* Action Type Enumerations
**********************************************/

typedef enum
{
    ACTION_NONE           = 0,  // No action: keep tracing, Check_Edge for mileage recording only
    ACTION_TURN_LEFT      = 1,  // 90° left turn (node turn)
    ACTION_TURN_RIGHT     = 2,  // 90° right turn (node turn)
    ACTION_STRAIGHT_SHORT = 3,  // Short straight element
    ACTION_STRAIGHT_MID   = 4,  // Mid straight element
    ACTION_STRAIGHT_LONG  = 5,  // Long straight element
    ACTION_ELEM_LEFT      = 6,  // Element left turn (small)
    ACTION_ELEM_RIGHT     = 7,  // Element right turn (small)
    ACTION_ELEM_LEFT_BIG  = 8,  // Element left turn (big)
    ACTION_ELEM_RIGHT_BIG = 9,  // Element right turn (big)
} Action_Enum;

/**********************************************
* Data Structures
**********************************************/

typedef struct
{
    Action_Enum action;          // Action type
    uint8_t     heading_frames;  // Straight: N frames gyro integration (0 = fixed diff)
    int         fixed_error;     // Straight fixed diff mode: Error
    int         fixed_diff;      // Straight fixed diff mode: left-right differential
    float       curve_advance;   // Turn: curve advance distance (0 = use default)
} Action_Entry_Typedef;

typedef struct
{
    Action_Entry_Typedef Action_List[MAX_ACTIONS];
    uint8_t              Action_Count;
    uint8_t              Stop_Mode;       // 0 = serial track
} Simple_Track_Typedef;

// Recorded turn spacing (Build writes, Remember reads)
typedef struct
{
    float   Turn_Distance[MAX_ACTIONS];
    uint8_t Turn_Count;
} Turn_Distance_Typedef;

// Recorded edge mileage (Build writes, Remember uses for Snap correction)
typedef struct
{
    float   Edge_Mileage[MAX_ACTIONS];
    uint8_t Edge_Count;
} Edge_Mileage_Record_Typedef;

/**********************************************
* Global Variables (extern)
**********************************************/

extern Simple_Track_Typedef        Simple_Track;
extern Simple_Track_Typedef        Default_Simple_Track;
extern Turn_Distance_Typedef       Turn_Distance_Rec;
extern Edge_Mileage_Record_Typedef Edge_Mileage_Rec;
extern Action_Entry_Typedef        Current_Action;
extern uint8_t                     Edge_Index;

/**********************************************
* Track Definition Macros
**********************************************/

// T(action) = action only, no heading/error/diff/advance
#define T(action)              { action, 0, 0, 0, 0 }
// H(action, frames) = action with heading lock frames
#define H(action, frames)      { action, frames, 0, 0, 0 }
// F(action, err, diff) = fixed diff mode
#define F(action, err, diff)   { action, 0, err, diff, 0 }
// C(action, advance) = curve turn with custom advance
#define C(action, advance)     { action, 0, 0, 0, advance }

#endif
