/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Mode_Config.h
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Compile-time mode selection
             Change ACTIVE_MODE to switch between Build / Remember / Debug.
             Only the active mode's code is compiled — zero runtime overhead.
Others:      Must be included before any mode-specific headers.
**************************************************/

#ifndef __MODE_CONFIG_H
#define __MODE_CONFIG_H

#define MODE_BUILD      1
#define MODE_REMEMBER   2
#define MODE_DEBUG      3

// ===== Change this line to switch modes =====
#define ACTIVE_MODE     MODE_BUILD

#endif
