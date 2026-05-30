#ifndef _isr_h
#define _isr_h

#include "zf_common_headfile.h"

// 串口调参命令解析结果
typedef struct
{
    uint8   valid;       // 1 = 有新命令
    char    key[4];      // 3字符命令键名 + 结束符 (LKP/LKI/RKP/RKI/TKP/TKD/GKP/GKD/YES)
    float   value;       // 解析出的浮点值
} uart_tuning_cmd_t;

extern uart_tuning_cmd_t g_tuning_cmd;

#endif
