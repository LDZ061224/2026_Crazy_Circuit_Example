/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: WS2812.h
Author: Cross_Z
Version:0.0               Date: 2026.6.23
Description:  WS2812 LED驱动 + 灯效引擎
Others:      GPIO Bit-Bang, 配置结构体接口
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.6.23   0.0      初始
**************************************************/
#ifndef __WS2812_H
#define __WS2812_H

#include "zf_common_headfile.h"

/***********************************硬件配置***********************************/
#define WS2812_MAX_LEDS         8
#define WS2812_DATA_PIN         P20_9           // 数据引脚

/***********************************类型定义***********************************/

// RGB颜色
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2812_Color_Typedef;

// 灯效类型
typedef enum {
    EFF_OFF = 0,            // 全灭
    EFF_SOLID,              // 纯色常亮
    EFF_BREATHING,          // 呼吸灯（亮度渐变）
    EFF_RAINBOW_FLOW,       // 彩虹流水拖尾（彩色拖尾绕圈）
    EFF_FLOW,               // 单色流水（可选方向+拖尾）
    EFF_CYCLE,              // 全局色相循环（整体变色）
    EFF_PROGRESS,           // 进度条（从0到满圈）
} WS2812_Effect_Enum;

// 灯效配置结构体
// 用法：指定 type，然后按需填写对应字段，其余填0
typedef struct {
    WS2812_Effect_Enum type;    // 灯效类型（必填）
    uint8_t  r, g, b;           // 颜色（EFF_SOLID / EFF_BREATHING / EFF_FLOW 用）
    uint8_t  speed;             // 动画速度 1-10（EFF_RAINBOW_FLOW / EFF_CYCLE 用）
    uint8_t  tail;              // 拖尾长度 2-8（EFF_RAINBOW_FLOW / EFF_FLOW 用）
    uint8_t  dir;               // 方向 0=正向 1=反向（EFF_FLOW 用）
    uint16_t period_ms;         // 周期毫秒（EFF_BREATHING 用）
    uint8_t  progress;          // 进度 0-100（EFF_PROGRESS 用）
} WS2812_Effect_Config;

/***********************************使用示例***********************************/
/*
    // 初始化
    WS2812_Init();

    // 设置效果（在需要切换效果时调用，如初始化时或状态改变时）
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type  = EFF_BREATHING,
        .r = 0, .g = 255, .b = 0,  // 绿色
        .period_ms = 2000           // 2秒一个呼吸周期
    });

    // 主循环中每帧调用（推进动画 + 刷新显示）
    while (1) {
        WS2812_Effect_Update();     // 自动按当前效果更新LED
        system_delay_ms(10);        // 10ms一帧，控制动画速度
    }
*/

/***********************************函数声明***********************************/

// 初始化（在Other_Init中已调用，不需要手动调）
void WS2812_Init(void);

// 设置所有LED为同一颜色（直接刷新，不影响当前灯效配置）
void WS2812_Set_All(uint8_t r, uint8_t g, uint8_t b);

// 立即刷新显示（将当前缓冲区数据发送到灯板）
void WS2812_Show(void);

// 全部熄灭
void WS2812_Clear(void);

// 设置灯效（切换效果时调用，会重置动画状态）
void WS2812_Effect_Set(WS2812_Effect_Config cfg);

// 直接更新进度（ISR安全，只改变量不发送数据）
void WS2812_Effect_SetProgress(uint8_t progress);

// 灯效每帧更新（主循环中以固定间隔调用，如10ms）
void WS2812_Effect_Update(void);

// 扫描完成 → 黄色常亮（等发车）
void WS2812_ScanDone(void);

// Flash保存成功 → 蓝色流水（主循环调用，ISR状态机不覆盖）
void WS2812_FlashSaved(void);

// 跑车 LED 状态机：按 Run_Mode 变色，停车黄呼吸（ISR 中每 tick 调用）
//   enable_on: 使能开关状态（1=跑车 0=停车）
void WS2812_UpdateCarLED(uint8_t enable_on);

#endif
