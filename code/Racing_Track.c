/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Racing_Track.c
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Default track definition (28 actions, serial track)
             Ported from CarbonV2.0
Others:      Edit Action_List to match your track layout.
**************************************************/

#include "Racing_Track.h"
#include "headfiles.h"

/********************************* Default Track Data *********************************/

// Default track: serial layout, 28 actions
// Note: you should update this to match your actual track.
Simple_Track_Typedef Default_Simple_Track =
{
    .Action_Count = 28,
    .Stop_Mode = 0,  // serial track
    .Action_List =
    {
        H(ACTION_STRAIGHT_SHORT, 10), // [ 0]
        H(ACTION_STRAIGHT_SHORT, 10), // [ 1]
        T(ACTION_NONE),               // [ 2]
        C(ACTION_TURN_LEFT, 4000),    // [ 3]
        H(ACTION_STRAIGHT_SHORT, 10), // [ 4]
        H(ACTION_STRAIGHT_SHORT, 10), // [ 5]
        C(ACTION_TURN_LEFT, 4000),    // [ 6]
        T(ACTION_NONE),               // [ 7]
        H(ACTION_STRAIGHT_SHORT, 10), // [ 8]
        C(ACTION_TURN_LEFT, 4000),    // [ 9]
        T(ACTION_NONE),               // [10]
        H(ACTION_STRAIGHT_SHORT, 10), // [11]
        C(ACTION_TURN_LEFT, 3600),    // [12]
        T(ACTION_NONE),               // [13]
        T(ACTION_NONE),               // [14]
        C(ACTION_ELEM_LEFT_BIG, 3600),// [15]
        T(ACTION_NONE),               // [16]
        C(ACTION_ELEM_LEFT, 3800),    // [17]
        C(ACTION_TURN_LEFT, 3600),    // [18]
        H(ACTION_STRAIGHT_LONG, 10),  // [19]
        C(ACTION_TURN_RIGHT, 3600),   // [20]
        C(ACTION_TURN_RIGHT, 3600),   // [21]
        H(ACTION_STRAIGHT_LONG, 10),  // [22]
        C(ACTION_TURN_LEFT, -3600),   // [23] negative advance = variable diff mode
        C(ACTION_TURN_LEFT, -3600),   // [24]
        H(ACTION_STRAIGHT_LONG, 10),  // [25]
        T(ACTION_NONE),               // [26]
        T(ACTION_TURN_LEFT),          // [27] finish line
    }
};

// Runtime track (copied from Default_Simple_Track at startup)
Simple_Track_Typedef Simple_Track = {{0}};

// Mileage data (Build writes, Remember loads from Flash)
Turn_Distance_Typedef       Turn_Distance_Rec = {{0}};
Edge_Mileage_Record_Typedef Edge_Mileage_Rec   = {{0}};

// Current dispatched action
Action_Entry_Typedef Current_Action = {0};

// Edge index (shared by Build and Remember)
uint8_t Edge_Index = 0;
