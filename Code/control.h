#ifndef __CONTROL_H__
#define __CONTROL_H__

/*
 * Control implementation selector:
 *   default: mileage_control
 *   set USE_RECOGNIZE_CONTROL to switch to recognize_control
 */
#if defined(USE_RECOGNIZE_CONTROL)
#include "recognize_control.h"
#else
#include "mileage_control.h"
#endif

#endif
