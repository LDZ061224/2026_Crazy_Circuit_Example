/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Uart_Adjust.c
Description:  UART remote tuning protocol — implementation
             Frame: @XXX=value#  (3-char key, '=' separator, '#' terminator)
             Special: @STOP#  @RUN#
             Works on UART_2, ISR feeds bytes, CPU1 loop calls Apply.
Others:      依赖 headfiles.h (PID / Flash / Ctrl debug vars)
History:
Cross_Z   2026.1.30   0.0   initial
Claude    2026.6.28   0.1   add debug-mode commands (DLP/DLI/DRP/DRI/FAN/TGT/GDI/AMO/MEN/WHL/MOD/SAV)
**************************************************/

#include "Uart_Adjust.h"
#include "headfiles.h"

/*********************************全局变量定义*********************************/
uart_tuning_cmd_t g_tuning_cmd = {0};
uint16 g_tuning_msg_count = 0;

uint8 uart_tuning_mode = 1;
uint8 uart_track_mode = 0;
uint8 uart_enable = 0;

int16 uart_left_target = 0;
int16 uart_right_target = 0;
float uart_gyro_target = 0.0f;
int16 uart_suction_power = 0;

/**********************************static functions**********************************/

static float uart_tuning_atof(const char *str)
{
    float result = 0.0f;
    float sign = 1.0f;
    float decimal = 1.0f;
    uint8 has_decimal = 0;

    if (*str == '-') { sign = -1.0f; str++; }
    else if (*str == '+') { str++; }

    while (*str)
    {
        if (*str == '.') { has_decimal = 1; str++; continue; }
        if (*str < '0' || *str > '9') break;
        if (has_decimal) { decimal *= 0.1f; result += (float)(*str - '0') * decimal; }
        else             { result = result * 10.0f + (float)(*str - '0'); }
        str++;
    }
    return result * sign;
}

static void uart_tuning_parse_frame(const char *frame, uint8 len)
{
    /* ----- special: @STOP# (5 chars) ----- */
    if (len == 5 && frame[0] == '@' && frame[1] == 'S'
        && frame[2] == 'T' && frame[3] == 'O' && frame[4] == 'P')
    {
        g_tuning_cmd.key[0] = 'S'; g_tuning_cmd.key[1] = 'T';
        g_tuning_cmd.key[2] = 'P'; g_tuning_cmd.key[3] = '\0';
        g_tuning_cmd.value = 1;
        g_tuning_cmd.valid = 1;
        return;
    }
    /* ----- special: @RUN# (4 chars) ----- */
    if (len == 4 && frame[0] == '@' && frame[1] == 'R'
        && frame[2] == 'U' && frame[3] == 'N')
    {
        g_tuning_cmd.key[0] = 'R'; g_tuning_cmd.key[1] = 'U';
        g_tuning_cmd.key[2] = 'N'; g_tuning_cmd.key[3] = '\0';
        g_tuning_cmd.value = 0;
        g_tuning_cmd.valid = 1;
        return;
    }

    if (len < 6) return;
    if (frame[0] != '@') return;
    if (frame[4] != '=') return;

    g_tuning_cmd.key[0] = frame[1];
    g_tuning_cmd.key[1] = frame[2];
    g_tuning_cmd.key[2] = frame[3];
    g_tuning_cmd.key[3] = '\0';

    g_tuning_cmd.value = uart_tuning_atof(&frame[5]);
    g_tuning_cmd.valid = 1;
    g_tuning_msg_count++;
}

static void uart_tuning_parse_byte(uint8 byte)
{
    static uint8 buf[32];
    static uint8 idx = 0;
    static uint8 receiving = 0;

    if (byte == '@')
    {
        idx = 0;
        receiving = 1;
        buf[idx++] = byte;
        return;
    }
    else if (receiving)
    {
        if (byte == '#')
        {
            buf[idx] = '\0';
            receiving = 0;
            uart_tuning_parse_frame((const char *)buf, idx);
            idx = 0;
        }
        else if (idx < sizeof(buf) - 1)
        {
            buf[idx++] = byte;
        }
        else
        {
            idx = 0;
            receiving = 0;
        }
        return;
    }
}

/**********************************public API**********************************/

void Uart_Adjust_ParseByte(uint8 byte)
{
    uart_tuning_parse_byte(byte);
}

void Uart_Adjust_Apply(void)
{
    extern int Stop_Flag;  // from Ctrl.c

    if (!g_tuning_cmd.valid) return;

    /* ---------- 紧急停车 / 恢复 ---------- */
    if (strcmp(g_tuning_cmd.key, "STP") == 0)
    {
        Stop_Flag = 1;
    }
    else if (strcmp(g_tuning_cmd.key, "RUN") == 0)
    {
        Stop_Flag = 0;
    }
    /* ---------- 赛车 PID 参数 ---------- */
    else if (strcmp(g_tuning_cmd.key, "LKP") == 0)
    {
        Left_PID.kp = g_tuning_cmd.value;
        PID_cleardata(&Left_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "LKI") == 0)
    {
        Left_PID.ki = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Left_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "RKP") == 0)
    {
        Right_PID.kp = g_tuning_cmd.value;
        PID_cleardata(&Right_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "RKI") == 0)
    {
        Right_PID.ki = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Right_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "TKP") == 0)
    {
        Turn_PID.kp = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Turn_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "TKD") == 0)
    {
        Turn_PID.kd = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Turn_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "GKP") == 0)
        Gyro_PID.kp = g_tuning_cmd.value * 0.001f;
    else if (strcmp(g_tuning_cmd.key, "GKD") == 0)
        Gyro_PID.kd = g_tuning_cmd.value * 0.001f;
    /* ---------- 赛车模式 ---------- */
    else if (strcmp(g_tuning_cmd.key, "SPD") == 0)
        uart_suction_power = (int16)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "BSP") == 0)
    {
        Basic_Speed = (int16)g_tuning_cmd.value;
        Debug_Target_Speed = (int16)g_tuning_cmd.value;
    }
    else if (strcmp(g_tuning_cmd.key, "ENA") == 0)
        uart_enable = (uint8)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "EXL") == 0)
        uart_left_target = (int16)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "EXR") == 0)
        uart_right_target = (int16)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "GTR") == 0)
        uart_gyro_target = g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "TRK") == 0)
        uart_track_mode = (uint8)g_tuning_cmd.value;
    /* ---------- Debug 调参模式 ---------- */
    else if (strcmp(g_tuning_cmd.key, "DLP") == 0)
        Debug_Kp_Left = g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "DLI") == 0)
        Debug_Ki_Left = g_tuning_cmd.value * 0.01f;
    else if (strcmp(g_tuning_cmd.key, "DRP") == 0)
        Debug_Kp_Right = g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "DRI") == 0)
        Debug_Ki_Right = g_tuning_cmd.value * 0.01f;
    else if (strcmp(g_tuning_cmd.key, "FAN") == 0)
        Debug_Fan_Duty = (int)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "TGT") == 0)
        Debug_Target_Speed = (int)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "GDI") == 0)
        Debug_Ground_Dir = ((uint8)g_tuning_cmd.value == 2) ? 2 : 1;
    else if (strcmp(g_tuning_cmd.key, "AMO") == 0)
        Debug_Angle_Mode = ((uint8)g_tuning_cmd.value == 2) ? 2 : 1;
    else if (strcmp(g_tuning_cmd.key, "MEN") == 0)
        Debug_Motor_Enable = (uint8)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "WHL") == 0)
        Debug_Which_Wheel = (uint8)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "MOD") == 0)
    {
        switch ((uint8)g_tuning_cmd.value)
        {
        case 0: Debug_Sub_Mode = Debug_Sub_PI_Tuning;    break;
        case 1: Debug_Sub_Mode = Debug_Sub_Ground_Test;  break;
        case 2: Debug_Sub_Mode = Debug_Sub_Angle;        break;
        case 3: Debug_Sub_Mode = Debug_Sub_NormalTrace;  break;
        default: break;
        }
    }
    /* ---------- Flash 保存 ---------- */
    else if (strcmp(g_tuning_cmd.key, "SAV") == 0)
        Uart_Adjust_SaveToFlash();

    g_tuning_cmd.valid = 0;
}

void Uart_Adjust_SaveToFlash(void)
{
    PID_OKb[0] = (uint32)Debug_Kp_Left;
    PID_OKb[1] = (uint32)(Debug_Ki_Left * 100.0f);
    PID_OKb[2] = (uint32)Debug_Kp_Right;
    PID_OKb[3] = (uint32)(Debug_Ki_Right * 100.0f);
    PID_OKb[4] = (uint32)(Turn_PID.kp * 100.0f);
    PID_OKb[5] = (uint32)(Turn_PID.kd * 100.0f);
    PID_OKb[6] = (uint32)(Gyro_PID.kp * 1000.0f);
    PID_OKb[7] = (uint32)(Gyro_PID.kd * 1000.0f);

    Speed_OKb[0] = (uint32)Debug_Target_Speed;

    DBG_OKb[0] = (uint32)Debug_Target_Speed;
    DBG_OKb[1] = (uint32)Debug_Fan_Duty;
    DBG_OKb[2] = (uint32)Debug_Ground_Dir;
    DBG_OKb[3] = (uint32)Debug_Angle_Mode;

    flash_erase_page(0, 0);
    flash_write_page(0, 0, Speed_OKb, 1);

    flash_erase_page(0, 1);
    flash_write_page(0, 1, PID_OKb, 8);

    flash_erase_page(0, 2);
    flash_write_page(0, 2, DBG_OKb, 4);
}
