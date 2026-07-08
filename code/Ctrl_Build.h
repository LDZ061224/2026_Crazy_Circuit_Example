/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Build.h
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Build mode function declarations
             (Global variables are declared in Ctrl.h)
**************************************************/

#ifndef __CTRL_BUILD_H
#define __CTRL_BUILD_H

#include "zf_common_headfile.h"
#include "Ctrl.h"

// All global variable externs are in Ctrl.h — this header only declares functions.

#if ACTIVE_MODE == MODE_BUILD
void Build_Mode_Get_Error(void);
void Normal_Run(void);
void Turn_Left_Run(void);
void Turn_Right_Run(void);
void Straight_Run(void);
// (Set_Mileage_Turn_Exp_Speed removed — Phase 1 uses fixed diff in Set_Speed)
uint8 Check_Edge(void);
void Load_All_Flash_Data_For_VOFA(void);
#endif

#endif
