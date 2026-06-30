/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: headfiles.h
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description:  Common macro definitions, header file includes
Others:      None
Function List:
             1. Universal macro functions: value limit, absolute value, sign, etc.
             2. Project-wide header file include list
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30   0.0        Initial version
**************************************************/

#ifndef __HEADFILES_H
#define __HEADFILES_H

/*********************************** Common Macro Definitions ***********************************/
// Absolute value macro: returns the absolute value of the input
#define ABS(x)                          (((x) >= (0.0f)) ? (x) : (-(x)))

// Sign judgment macro: returns the sign of the value, negative returns -1.0, positive/0 returns 1.0
#define SignOf(Value)                   ((Value < 0.0) ? (-1.0) : (1.0))

// Pi constant definition (high-precision float)
#define PI                              (3.1415926535897932384626433832795f)

/*********************************** Header File Includes ***********************************/
#include "zf_common_headfile.h"    // Zhifeng official common public header
#include "pid.h"                   // PID algorithm related header
#include "Fun.h"                   // Function driver implementation header
#include "TCA9555.h"               // TCA9555 IO expander chip driver header
#include "OLEDKeyboard.h"          // OLED display and keyboard driver header
#include "Ctrl.h"                  // System control logic header

#endif
