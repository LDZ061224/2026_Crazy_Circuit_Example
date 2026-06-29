/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: WS2812.c
Author: Cross_Z
Version:0.0               Date: 2026.6.23
Description:  WS2812 LED椹卞姩 + 鐏晥寮曟搸
Others:      GPIO Bit-Bang, NOP绮剧‘鏃跺簭, 閰嶇疆缁撴瀯浣撴帴鍙�
Function List:
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.6.23   0.0      鍒濆
**************************************************/
#include "zf_common_headfile.h"
#include "zf_driver_delay.h"
#include "WS2812.h"

/***********************************绉佹湁鍙橀噺***********************************/
static WS2812_Color_Typedef  s_buf_a[WS2812_MAX_LEDS];
static WS2812_Color_Typedef  s_buf_b[WS2812_MAX_LEDS];
static WS2812_Color_Typedef * volatile s_front = s_buf_a;
static uint8_t               s_led_count = WS2812_MAX_LEDS;
static volatile uint8_t      s_tx_busy   = 0;

static WS2812_Effect_Config  s_cfg;
static uint32_t              s_phase     = 0;
static volatile uint8_t      s_progress  = 0;      // 杩涘害 0-100锛圛SR鍙啓锛�

/***********************************NOP鏃跺簭 (5ns/涓� @200MHz)*******************/
static inline void WS2812_SendBit(uint8_t bit)
{
    if (bit)
    {
        // '1': 800ns楂� + 450ns浣�
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
        // '0': 400ns楂� + 850ns浣�
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

/***********************************鍐呴儴宸ュ叿鍑芥暟********************************/
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

/***********************************鍩虹鍑芥暟***********************************/

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

/***********************************鐏晥寮曟搸***********************************/

void WS2812_Effect_Set(WS2812_Effect_Config cfg)
{
    s_cfg = cfg;
    s_phase = 0;
}

/*************************************
** Function: WS2812_Effect_SetProgress
** Description: 鐩存帴鏇存柊杩涘害鍊硷紙ISR瀹夊叏锛屽彧鏀瑰彉閲忥級
** Others: 鍦↖SR涓皟鐢ㄦ鍑芥暟鏇存柊杩涘害
**         涓诲惊鐜腑 Effect_Update 浼氳嚜鍔ㄥ埛鏂版樉绀�
*************************************/
void WS2812_Effect_SetProgress(uint8_t progress)
{
    if (progress > 100) progress = 100;
    s_progress = progress;
}

/*************************************
** Function: WS2812_Effect_Update
** Description: 鐏晥姣忓抚鏇存柊
** Others: 鍦ㄤ富寰幆涓互鍥哄畾闂撮殧璋冪敤锛堝10ms锛�
**         鑷姩鏍规嵁褰撳墠閰嶇疆娓叉煋涓�甯у苟鍒锋柊鏄剧ず
*************************************/
void WS2812_Effect_Update(void)
{
    WS2812_Color_Typedef *back = (s_front == s_buf_a) ? s_buf_b : s_buf_a;
    uint8_t n = s_led_count;

    // 鍘熷瓙蹇収 s_cfg锛岄伩鍏� ISR 閲� Effect_Set 瑕嗙洊鏃惰鍒板崐鎴愬搧
    WS2812_Effect_Config cfg;
    {
        uint32 primask = interrupt_global_disable();
        cfg = s_cfg;
        interrupt_global_enable(primask);
    }

    switch (cfg.type)
    {
    case EFF_OFF:   // 鍏ㄧ伃
        memset(back, 0, sizeof(WS2812_Color_Typedef) * n);
        break;

    case EFF_SOLID: // 绾壊甯镐寒
        for (uint8_t i = 0; i < n; i++)
            { back[i].r = cfg.r; back[i].g = cfg.g; back[i].b = cfg.b; }
        break;

    case EFF_BREATHING: // 鍛煎惛鐏細浜害鎸変笁瑙掓尝 0鈫�255鈫�0 寰幆
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

    case EFF_RAINBOW_FLOW: // 褰╄櫣娴佹按鎷栧熬锛氬僵鑹叉嫋灏剧粫鐜Щ鍔�
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

    case EFF_FLOW: // 鍗曡壊娴佹按锛氭寚瀹氶鑹�+鏂瑰悜+鎷栧熬闀垮害
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

    case EFF_CYCLE: // 鍏ㄥ眬鑹茬浉寰幆锛氭墍鏈塋ED鍚屾鍙樿壊
    {
        WS2812_Color_Typedef c = WS2812_HSV(s_phase % 768, 255, 255);
        for (uint8_t i = 0; i < n; i++)
            back[i] = c;
        s_phase += cfg.speed ? cfg.speed : 3;
        break;
    }

    case EFF_PROGRESS: // 杩涘害鏉★細浠庣0棰楀紑濮嬮�愰鐐逛寒
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

/*********************************** 璺戣溅 LED 鐘舵�佹満 ***********************************/

// led_state: 0=绛夊彂杞�(榛勫父浜�) 1=璺戣溅(鎸塕un_Mode鍙樿壊) 2=鍋滆溅(榛勫懠鍚�) 3=Flash淇濆瓨(钃濇祦姘�)
static uint8_t s_led_state          = 0;
static uint8_t s_last_run_mode      = 0xFF;  // 鍝ㄥ叺锛屽己鍒堕娆℃洿鏂�
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
    // 璺戣溅 = 浣胯兘寮� 涓� 鏈仠杞�(Stop_Flag=0)
    uint8_t car_running = enable_on && !Stop_Flag;

    // state 3 (Flash淇濆瓨钃濇祦姘�) 涓嶈鍋滆溅瑕嗙洊锛屽彂杞︽椂鎵嶉��鍑�
    if (car_running && (s_led_state == 3 || s_led_state != 1))
    {
        s_led_state = 1;
        s_last_run_mode = 0xFF;
    }
    // 鍋滆溅锛堜娇鑳藉叧 鎴� 鑷姩鍋滆溅锛夛細鍒囬粍鑹插懠鍚哥伅锛坰tate 3 闄ゅ锛�
    else if (!car_running && (s_led_state == 1 || s_led_state == 0))
    {
        WS2812_Effect_Set((WS2812_Effect_Config){
            .type = EFF_BREATHING,
            .r = 255, .g = 255, .b = 0,
            .period_ms = 1500,
        });
        s_led_state = 2;
    }

    // 璺戣溅涓細鎸� Run_Mode 鍙樿壊锛屼粎鍦ㄦā寮忚烦鍙樻椂璋� Effect_Set
    if (s_led_state == 1)
    {
        // Mileage_Mode 涓ら樁娈碉細0=寰抗(缁�) 1=鐗规畩鍏冪礌(绱�)
        uint8_t mphase = 0;
        if (Run_Mode == Mileage_Mode)
//            mphase = (Count.Mileage < Run_Track.Node_Mileage[Execute_Times][0]) ? 0 : 1;

        if (Run_Mode != s_last_run_mode || mphase != s_last_mileage_phase)
        {
            s_last_run_mode      = Run_Mode;
            s_last_mileage_phase = mphase;

            switch (Run_Mode)
            {
                case Normal_Mode: default:
                    // Remember 妯″紡锛氭寜閫熷害鏇茬嚎鐩镐綅鍙樿壊
                    //   0=鍔犻��(榛�) 1=鍖�閫�(缁�) 2=鍑忛��(绾�) 3=杞集(绾�)
                    // 鍏朵粬妯″紡锛氱伃
//                    if (Work_Mode == Remember_Mode)
//                    {
//                        switch (Remember_Speed_Phase)
//                        {
//                            case 0:  // 鍔犻��
//                                WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 255, .b = 0});
//                                break;
//                            case 1:  // 鍖�閫�
//                                WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 0, .g = 255, .b = 0});
//                                break;
//                            case 2:  // 鍑忛��
//                            default:
//                                WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 0, .b = 0});
//                                break;
//                        }
//                    }
//                    else
//                    {
//                        WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_OFF});
//                    }
                    break;
                case Turn_Left: case Turn_Right:        // 杞集锛氱孩
                    WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 0, .b = 0});
                    break;
                case Straight_Mode:                     // 鐩寸嚎锛氳摑
                    WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 0, .g = 0, .b = 255});
                    break;
                case Mileage_Mode:                      // 閲岀▼锛氶樁娈�1缁� 闃舵2绱�
                    if (mphase == 0)
                        WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 0, .g = 255, .b = 0});
                    else
                        WS2812_Effect_Set((WS2812_Effect_Config){.type = EFF_SOLID, .r = 255, .g = 0, .b = 255});
                    break;
            }
        }
    }
}
