/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: WS2812.c
Author: Cross_Z
Version:0.0               Date: 2026.6.23
Description:  WS2812 LED驱动 + 灯效引擎
Others:      GPIO Bit-Bang, NOP精确时序, 配置结构体接口
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.6.23   0.0      初始
**************************************************/
#include "zf_common_headfile.h"
#include "zf_driver_delay.h"
#include "headfiles.h"         // for Run_Mode / Stop_Flag / Count etc. in WS2812_UpdateCarLED
#include "WS2812.h"

/***********************************私有变量***********************************/
static WS2812_Color_Typedef  s_buf_a[WS2812_MAX_LEDS];
static WS2812_Color_Typedef  s_buf_b[WS2812_MAX_LEDS];
static WS2812_Color_Typedef * volatile s_front = s_buf_a;
static uint8_t               s_led_count = WS2812_MAX_LEDS;
static volatile uint8_t      s_tx_busy   = 0;

static WS2812_Effect_Config  s_cfg;
static uint32_t              s_phase     = 0;
static volatile uint8_t      s_progress  = 0;      // 进度 0-100（ISR可写）

/***********************************NOP时序 (5ns/个 @200MHz)*******************/
static inline void WS2812_SendBit(uint8_t bit)
{
    if (bit)
    {
        // '1': 800ns高 + 450ns低
        P20_OUT.B.P9 = 1;
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        P20_OUT.B.P9 = 0;
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
    }
    else
    {
        // '0': 400ns高 + 850ns低
        P20_OUT.B.P9 = 1;
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        P20_OUT.B.P9 = 0;
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
        __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); __asm("nop");
    }
}

static inline void WS2812_SendByte(uint8_t byte)
{
    WS2812_SendBit((byte >> 7) & 1);
    WS2812_SendBit((byte >> 6) & 1);
    WS2812_SendBit((byte >> 5) & 1);
    WS2812_SendBit((byte >> 4) & 1);
    WS2812_SendBit((byte >> 3) & 1);
    WS2812_SendBit((byte >> 2) & 1);
    WS2812_SendBit((byte >> 1) & 1);
    WS2812_SendBit((byte >> 0) & 1);
}

/***********************************内部工具函数********************************/
static WS2812_Color_Typedef WS2812_HSV(uint16_t h, uint8_t s, uint8_t v)
{
    uint8_t region = h / 256;
    uint8_t remainder = h % 256;
    uint8_t p = (uint16_t)v * (255 - s) / 255;
    uint8_t q = (uint16_t)v * (255 - (uint16_t)s * remainder / 255) / 255;
    uint8_t t = (uint16_t)v * (255 - (uint16_t)s * (255 - remainder) / 255) / 255;
    WS2812_Color_Typedef c;
    switch (region) {
        case 0:  c.r = v; c.g = t; c.b = p; break;
        case 1:  c.r = q; c.g = v; c.b = p; break;
        case 2:  c.r = p; c.g = v; c.b = t; break;
        case 3:  c.r = p; c.g = q; c.b = v; break;
        case 4:  c.r = t; c.g = p; c.b = v; break;
        default: c.r = v; c.g = p; c.b = q; break;
    }
    return c;
}

static uint16_t WS2812_Triangle(uint32_t phase, uint16_t peak)
{
    uint32_t p = phase % (peak * 2);
    return (p < peak) ? p : peak * 2 - p;
}

static void WS2812_ShowBuf(void)
{
    if (s_tx_busy) return;
    s_tx_busy = 1;
    uint32 primask = interrupt_global_disable();
    WS2812_Color_Typedef *front = s_front;
    for (uint8_t i = 0; i < s_led_count; i++)
    {
        WS2812_SendByte(front[i].g);
        WS2812_SendByte(front[i].r);
        WS2812_SendByte(front[i].b);
    }
    interrupt_global_enable(primask);
    system_delay_us(60);
    s_tx_busy = 0;
}

/***********************************基础函数***********************************/

void WS2812_Init(void)
{
    gpio_init(WS2812_DATA_PIN, GPO, 0, GPO_PUSH_PULL);
    s_tx_busy   = 0;
    s_led_count = WS2812_MAX_LEDS;
    s_phase     = 0;
    memset(&s_cfg, 0, sizeof(s_cfg));
    memset(s_buf_a, 0, sizeof(s_buf_a));
    memset(s_buf_b, 0, sizeof(s_buf_b));
    s_front = s_buf_a;
    WS2812_ShowBuf();
}

void WS2812_Set_All(uint8_t r, uint8_t g, uint8_t b)
{
    WS2812_Color_Typedef *back = (s_front == s_buf_a) ? s_buf_b : s_buf_a;
    for (uint8_t i = 0; i < s_led_count; i++)
    {
        back[i].r = r;
        back[i].g = g;
        back[i].b = b;
    }
    s_front = back;
}

void WS2812_Show(void) { WS2812_ShowBuf(); }

void WS2812_Clear(void) { WS2812_Set_All(0, 0, 0); WS2812_ShowBuf(); }

/***********************************灯效引擎***********************************/

void WS2812_Effect_Set(WS2812_Effect_Config cfg)
{
    s_cfg = cfg;
    s_phase = 0;
}

/*************************************
** Function: WS2812_Effect_SetProgress
** Description: 直接更新进度值（ISR安全，只改变量）
** Others: 在ISR中调用此函数更新进度
**         主循环中 Effect_Update 会自动刷新显示
*************************************/
void WS2812_Effect_SetProgress(uint8_t progress)
{
    if (progress > 100) progress = 100;
    s_progress = progress;
}

/*************************************
** Function: WS2812_Effect_Update
** Description: 灯效每帧更新
** Others: 在主循环中以固定间隔调用（如10ms）
**         自动根据当前配置渲染一帧并刷新显示
*************************************/
void WS2812_Effect_Update(void)
{
    WS2812_Color_Typedef *back = (s_front == s_buf_a) ? s_buf_b : s_buf_a;
    uint8_t n = s_led_count;

    // 原子快照 s_cfg，避免 ISR 里 Effect_Set 覆盖时读到半成品
    WS2812_Effect_Config cfg;
    {
        uint32 primask = interrupt_global_disable();
        cfg = s_cfg;
        interrupt_global_enable(primask);
    }

    switch (cfg.type)
    {
    case EFF_OFF:   // 全灭
        memset(back, 0, sizeof(WS2812_Color_Typedef) * n);
        break;

    case EFF_SOLID: // 纯色常亮
        for (uint8_t i = 0; i < n; i++)
            { back[i].r = cfg.r; back[i].g = cfg.g; back[i].b = cfg.b; }
        break;

    case EFF_BREATHING: // 呼吸灯：亮度按三角波 0→255→0 循环
    {
        uint16_t bright = WS2812_Triangle(s_phase, 255);
        for (uint8_t i = 0; i < n; i++)
        {
            back[i].r = (uint16_t)cfg.r * bright / 255;
            back[i].g = (uint16_t)cfg.g * bright / 255;
            back[i].b = (uint16_t)cfg.b * bright / 255;
        }
        uint16_t step = (cfg.period_ms > 0) ? cfg.period_ms / 20 : 50;
        s_phase += (step > 0) ? 255 / step : 5;
        break;
    }

    case EFF_RAINBOW_FLOW: // 彩虹流水拖尾：彩色拖尾绕环移动
    {
        uint8_t tail = cfg.tail ? cfg.tail : 4;
        uint8_t head = (s_phase / 4) % n;
        for (uint8_t i = 0; i < n; i++)
        {
            int16_t dist = (int16_t)head - (int16_t)i;
            if (dist < 0) dist += n;
            if (dist < tail)
            {
                uint8_t fade = (uint16_t)(tail - dist) * 255 / tail;
                uint16_t hue = (s_phase * 8 + i * (768 / n)) % 768;
                WS2812_Color_Typedef c = WS2812_HSV(hue, 255, fade);
                back[i] = c;
            }
            else
            {
                back[i].r = 0; back[i].g = 0; back[i].b = 0;
            }
        }
        s_phase += cfg.speed ? cfg.speed : 3;
        break;
    }

    case EFF_FLOW: // 单色流水：指定颜色+方向+拖尾长度
    {
        uint8_t tail = cfg.tail ? cfg.tail : 3;
        uint8_t head;
        if (cfg.dir == 0)
            head = (s_phase / 4) % n;
        else
            head = n - 1 - (s_phase / 4) % n;

        for (uint8_t i = 0; i < n; i++)
        {
            int16_t dist;
            if (cfg.dir == 0)
                dist = (int16_t)head - (int16_t)i;
            else
                dist = (int16_t)i - (int16_t)head;
            if (dist < 0) dist += n;
            if (dist < tail)
            {
                uint8_t fade = (uint16_t)(tail - dist) * 255 / tail;
                back[i].r = (uint16_t)cfg.r * fade / 255;
                back[i].g = (uint16_t)cfg.g * fade / 255;
                back[i].b = (uint16_t)cfg.b * fade / 255;
            }
            else
            {
                back[i].r = 0; back[i].g = 0; back[i].b = 0;
            }
        }
        s_phase++;
        break;
    }

    case EFF_CYCLE: // 全局色相循环：所有LED同步变色
    {
        WS2812_Color_Typedef c = WS2812_HSV(s_phase % 768, 255, 255);
        for (uint8_t i = 0; i < n; i++)
            back[i] = c;
        s_phase += cfg.speed ? cfg.speed : 3;
        break;
    }

    case EFF_PROGRESS: // 进度条：从第0颗开始逐颗点亮
    {
        uint8_t lit = (uint16_t)s_progress * n / 100;
        for (uint8_t i = 0; i < n; i++)
        {
            if (i < lit)
                { back[i].r = cfg.r; back[i].g = cfg.g; back[i].b = cfg.b; }
            else
                { back[i].r = 0; back[i].g = 0; back[i].b = 0; }
        }
        break;
    }
    }

    s_front = back;
    WS2812_ShowBuf();
}

/*********************************** 跑车 LED 状态机 ***********************************/

// led_state: 0=等发车(黄常亮) 1=跑车(按Run_Mode变色) 2=停车(黄呼吸) 3=Flash保存(蓝流水)
static uint8_t s_led_state          = 0;
static uint8_t s_last_run_mode      = 0xFF;  // 哨兵，强制首次更新
static uint8_t s_last_mileage_phase = 0;

void WS2812_ScanDone(void)
{
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_SOLID,
        .r = 0, .g = 255, .b = 0,
    });
}

void WS2812_FlashSaved(void)
{
    WS2812_Effect_Set((WS2812_Effect_Config){
        .type = EFF_FLOW,
        .r = 0, .g = 0, .b = 255,
        .tail = 4,
        .dir = 0,
    });
    s_led_state = 3;
}

void WS2812_UpdateCarLED(uint8_t enable_on)
{
#if 0  /* TODO: needs Work_Mode Remember_Mode Remember_Speed_Phase not yet defined */
    // 跑车 = 使能开 且 未停车(Stop_Flag=0)
    uint8_t car_running = enable_on && !Stop_Flag;

    // state 3 (Flash保存蓝流水) 不被停车覆盖，发车时才退出
    if (car_running && (s_led_state == 3 || s_led_state != 1))
    {
        s_led_state = 1;
        s_last_run_mode = 0xFF;
    }
    // 停车（使能关 或 自动停车）：切黄色呼吸灯（state 3 除外）
    else if (!car_running && (s_led_state == 1 || s_led_state == 0))
    {
        WS2812_Effect_Set((WS2812_Effect_Config){
            .type = EFF_BREATHING,
            .r = 255, .g = 255, .b = 0,
            .period_ms = 1500,
        });
        s_led_state = 2;
    }

    // 跑车中：按 Run_Mode 变色，仅在模式跳变时调 Effect_Set
    if (s_led_state == 1)
    {
        // Mileage_Mode 两阶段：0=循迹(绿) 1=特殊元素(紫)
        uint8_t mphase = 0;
        if (Run_Mode == Mileage_Mode)
            mphase = (Count.Mileage < Run_Track.Node_Mileage[Execute_Times][0]) ? 0 : 1;

        if (Run_Mode != s_last_run_mode || mphase != s_last_mileage_phase)
        {
            s_last_run_mode      = Run_Mode;
            s_last_mileage_phase = mphase;

            switch (Run_Mode)
            {
                case Normal_Mode: default:
                    // Remember 模式：按速度曲线相位变色
                    //   0=加速(黄) 1=匀速(绿) 2=减速(红) 3=转弯(红)
                    // 其他模式：灭
                    if (Work_Mode == Remember_Mode)
                    {
                        switch (Remember_Speed_Phase)
                        {
                            case 0:  // 加速
                                WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 255, .b = 0});
                                break;
                            case 1:  // 匀速
                                WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 0, .g = 255, .b = 0});
                                break;
                            case 2:  // 减速
                            default:
                                WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 0, .b = 0});
                                break;
                        }
                    }
                    else
                    {
                        WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_OFF});
                    }
                    break;
                case Turn_Left: case Turn_Right:        // 转弯：红
                    WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 0, .b = 0});
                    break;
                case Straight_Mode:                     // 直线：蓝
                    WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 0, .g = 0, .b = 255});
                    break;
                case Mileage_Mode:                      // 里程：阶段1绿 阶段2紫
                    if (mphase == 0)
                        WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 0, .g = 255, .b = 0});
                    else
                        WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 0, .b = 255});
                    break;
            }
        }
#endif
}


