/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: WS2812.h
Author: Cross_Z
Version:0.0               Date: 2026.6.23
Description:  WS2812 LED driver + light effect engine
Others:      GPIO bit-bang, configuration struct interface
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.6.23   0.0      Initial
**************************************************/
#ifndef __WS2812_H
#define __WS2812_H

#include "zf_common_headfile.h"
#include "headfiles.h"

/***********************************Hardware Configuration***********************************/
#define WS2812_MAX_LEDS         8
#define WS2812_DATA_PIN         P33_13          // Data pin

/***********************************Type Definitions***********************************/

// RGB color
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2812_Color_Typedef;

// Light effect types
typedef enum {
    EFF_OFF = 0,            // All off
    EFF_SOLID,              // Solid color
    EFF_BREATHING,          // Breathing (brightness fade)
    EFF_RAINBOW_FLOW,       // Rainbow flow with tail (colorful trailing loop)
    EFF_FLOW,               // Single-color flow (direction + tail configurable)
    EFF_CYCLE,              // Global hue cycle (all LEDs change color together)
    EFF_PROGRESS,           // Progress bar (from 0 to full ring)
} WS2812_Effect_Enum;

// Light effect configuration struct
// Usage: specify type, then fill corresponding fields as needed, fill 0 for unused fields
typedef struct {
    WS2812_Effect_Enum type;    // Light effect type (required)
    uint8_t  r, g, b;           // Color (for EFF_SOLID / EFF_BREATHING / EFF_FLOW)
    uint8_t  speed;             // Animation speed 1-10 (for EFF_RAINBOW_FLOW / EFF_CYCLE)
    uint8_t  tail;              // Tail length 2-8 (for EFF_RAINBOW_FLOW / EFF_FLOW)
    uint8_t  dir;               // Direction 0=forward 1=reverse (for EFF_FLOW)
    uint16_t period_ms;         // Period in ms (for EFF_BREATHING)
    uint8_t  progress;          // Progress 0-100 (for EFF_PROGRESS)
} WS2812_Effect_Config;

/***********************************Usage Example***********************************/
/*
    // Initialize
    WS2812_Init();

    // Set effect (call when switching effects, e.g. at init or state change)
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type  = EFF_BREATHING,
        .r = 0, .g = 255, .b = 0,  // Green
        .period_ms = 2000           // 2-second breathing cycle
    });

    // Call per frame in main loop (advance animation + refresh display)
    while (1) {
        WS2812_Effect_Update();     // Auto-update LEDs per current effect
        system_delay_ms(10);        // 10ms per frame, controls animation speed
    }
*/

/***********************************Function Declarations***********************************/

// Initialize (already called in Other_Init, no need to call manually)
void WS2812_Init(void);

// Set all LEDs to one color (direct refresh, does not affect current effect config)
void WS2812_Set_All(uint8_t r, uint8_t g, uint8_t b);

// Immediately refresh display (send current buffer data to LED strip)
void WS2812_Show(void);

// Turn off all LEDs
void WS2812_Clear(void);

// Set light effect (call when switching effects, resets animation state)
void WS2812_Effect_Set(WS2812_Effect_Config cfg);

// Directly update progress (ISR-safe, only modifies variable, does not send data)
void WS2812_Effect_SetProgress(uint8_t progress);

// Per-frame light effect update (call at fixed interval in main loop, e.g. 10ms)
void WS2812_Effect_Update(void);

// Scan complete -> yellow solid (waiting to launch)
void WS2812_ScanDone(void);

// Flash save success -> blue flow (call in main loop, ISR state machine does not override)
void WS2812_FlashSaved(void);

// Car LED state machine: color by Run_Mode, yellow breathing when stopped (call per tick in ISR)
//   enable_on: enable switch state (1=running 0=stopped)
void WS2812_UpdateCarLED(uint8_t enable_on);

#endif
