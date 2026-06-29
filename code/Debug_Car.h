/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Debug_Car.h
Author: Claude (extracted from Ctrl.c)
Version:0.0               Date: 2026.6.29
Description:  Debug car control 鈥� PI tuning / ground test / angle / normal trace
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
 *  USE_DEBUG_MODE = 1  鈫� Car_Go() dispatches to Debug_Car_Go() when Mode==Debug_Mode
 *  USE_DEBUG_MODE = 0  鈫� Car_Go() runs only normal racing (compile out debug branch)
 */
#define USE_DEBUG_MODE  0

/***********************************public API***********************************/

/*  per-tick entry called from PIT ISR (replaces the Debug_Mode case in Car_Go) */
void Debug_Car_Go(void);

#endif  // __DEBUG_CAR_H
