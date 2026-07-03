/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Uart_Adjust.c
Description:  UART remote tuning protocol -- implementation.
             Frame: @XXX=value# (3-char key, '=' separator, '#' terminator).
             Special: @STOP# -> key="STP" val=1 ; @RUN# -> key="RUN" val=0.
             Works on UART_2, ISR feeds bytes, CPU1 loop calls Apply.
Others:      All XXX keys are 3 chars (Apply uses strcmp on a 4-byte buffer).
History:
Cross_Z   2026.1.30   0.0  initial
Claude    2026.6.28   0.1  debug-mode + Flash save commands
Claude    2026.6.29   0.2  clean unused uart_* vars, fix BSP, add all keys
**************************************************/

#include "Uart_Adjust.h"
#include "headfiles.h"

/* ============================== supported commands ==============================

  Stop/Run  : @STOP#  @RUN#
  Speed PID : @LKP=val# / @DLP=val#   @LKI=val# / @DLI=val#   | kp*1  ki*0.01
              @RKP=val# / @DRP=val#   @RKI=val# / @DRI=val#
  Steer PID : @TKP=val#  @TKD=val#                          | *0.01
  Gyro PID  : @GKP=val#  @GKI=val#  @GKD=val#              | *0.001
  Base      : @BSP=val#   (Basic_Speed + Debug_Target_Speed)
  Debug     : @FAN=val#   @TGT=val#   @GDI=val#(1 or 2)
              @AMO=val#(1=sin 2=step 3=gyro rate)  @AVT=val#(gyro rate target deg/s)
              @MEN=val#(toggle if value matches)
              @WHL=val#   @MOD=val#(0=PI 1=GroundTest 2=Angle 3=NormTrace)
              @FFM=val#(0=PI only 1=speed FF+PI)  @GFM=val#(0=PID only 1=gyro FF+PID)
  Flash     : @SAV#   (no value, triggers save)
============================================================================= */

uart_tuning_cmd_t g_tuning_cmd = {0};

/* ============================== static helpers ============================== */

/* Convert a decimal string to float (lightweight, no stdlib dependency) */
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

/*
 *  Parse a complete frame (already delimited by '@' ... '#') into g_tuning_cmd.
 *
 *  Supported formats:
 *    @STOP#           -> key="STP"  val=1    (no '=')
 *    @RUN#            -> key="RUN"  val=0
 *    @XXX=value#      -> key="XXX"  val=parsed float  (3-char key, '=' required)
 *
 *  frame: points to '@' (byte-parser stored '@' in buf[0])
 *  len:   number of bytes before '#'
 */
static void uart_tuning_parse_frame(const char *frame, uint8 len)
{
    /* ---------- @STOP#  or  @STP# ---------- */
    if ((len == 5 && frame[0] == '@'
         && frame[1] == 'S' && frame[2] == 'T'
         && frame[3] == 'O' && frame[4] == 'P')
        ||
        (len == 4 && frame[0] == '@'
         && frame[1] == 'S' && frame[2] == 'T'
         && frame[3] == 'P'))
    {
        g_tuning_cmd.key[0] = 'S'; g_tuning_cmd.key[1] = 'T';
        g_tuning_cmd.key[2] = 'P'; g_tuning_cmd.key[3] = '\0';
        g_tuning_cmd.value = 1;
        g_tuning_cmd.valid = 1;
        return;
    }

    /* ---------- @RUN# ---------- */
    if (len == 4 && frame[0] == '@'
        && frame[1] == 'R' && frame[2] == 'U' && frame[3] == 'N')
    {
        g_tuning_cmd.key[0] = 'R'; g_tuning_cmd.key[1] = 'U';
        g_tuning_cmd.key[2] = 'N'; g_tuning_cmd.key[3] = '\0';
        g_tuning_cmd.value = 0;
        g_tuning_cmd.valid = 1;
        return;
    }

    /* ---------- @SAV# (no value, triggers flash save) ---------- */
    if (len == 4 && frame[0] == '@'
        && frame[1] == 'S' && frame[2] == 'A' && frame[3] == 'V')
    {
        g_tuning_cmd.key[0] = 'S'; g_tuning_cmd.key[1] = 'A';
        g_tuning_cmd.key[2] = 'V'; g_tuning_cmd.key[3] = '\0';
        g_tuning_cmd.value = 0;
        g_tuning_cmd.valid = 1;
        return;
    }

    /* ---------- @XXX=value# (generic) ---------- */
    /* minimum: @X=Y# -> 6 bytes.  frame[0]='@', frame[4]='=' */
    if (len < 6) return;
    if (frame[0] != '@') return;
    if (frame[4] != '=') return;

    g_tuning_cmd.key[0] = frame[1];
    g_tuning_cmd.key[1] = frame[2];
    g_tuning_cmd.key[2] = frame[3];
    g_tuning_cmd.key[3] = '\0';

    g_tuning_cmd.value = uart_tuning_atof(&frame[5]);
    g_tuning_cmd.valid = 1;
}

/*
 *  Per-byte state machine.  '@' starts a frame, '#' ends it.
 *  Buffer size 16 = enough for "@XXX=-123.456" (~14 bytes) + margin.
 *  Max frame len 14 bytes to reject noise: floating RX pin generates
 *  random '@'...'#' sequences that can fake @STP#.
 */
static void uart_tuning_parse_byte(uint8 byte)
{
    static uint8 buf[16];
    static uint8 idx = 0;
    static uint8 receiving = 0;

#define UART_FRAME_MAX_LEN 14

    if (byte == '@')
    {
        idx = 0;
        receiving = 1;
        buf[idx++] = byte;
        return;
    }

    if (!receiving) return;

    // Noise guard: discard partial frame if it runs too long
    if (idx >= UART_FRAME_MAX_LEN)
    {
        idx = 0;
        receiving = 0;
        return;
    }

    if (byte == '#')
    {
        buf[idx] = '\0';
        receiving = 0;
        uart_tuning_parse_frame((const char *)buf, idx);
        idx = 0;
    }
    else
    {
        buf[idx++] = byte;
    }
}

/* ============================== public API ============================== */

/* Feed one byte into the frame-parsing state machine (call from ISR) */
void Uart_Adjust_ParseByte(uint8 byte)
{
    uart_tuning_parse_byte(byte);
}

/* Consume g_tuning_cmd and apply the parameter change (call from main loop) */
void Uart_Adjust_Apply(void)
{
    extern int Stop_Flag;

    if (!g_tuning_cmd.valid) return;

    /* ---------- stop / resume ---------- */
    if (strcmp(g_tuning_cmd.key, "STP") == 0)
    {
        Stop_Flag = 1;
    }
    else if (strcmp(g_tuning_cmd.key, "RUN") == 0)
    {
        Stop_Flag = 0;
    }
    /* ---------- wheel-speed PID ---------- */
    else if (strcmp(g_tuning_cmd.key, "LKP") == 0 || strcmp(g_tuning_cmd.key, "DLP") == 0)
    {
        Left_PID.kp = g_tuning_cmd.value;
        PID_cleardata(&Left_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "LKI") == 0 || strcmp(g_tuning_cmd.key, "DLI") == 0)
    {
        Left_PID.ki = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Left_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "RKP") == 0 || strcmp(g_tuning_cmd.key, "DRP") == 0)
    {
        Right_PID.kp = g_tuning_cmd.value;
        PID_cleardata(&Right_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "RKI") == 0 || strcmp(g_tuning_cmd.key, "DRI") == 0)
    {
        Right_PID.ki = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Right_PID);
    }
    /* ---------- steering / gyro PID ---------- */
    else if (strcmp(g_tuning_cmd.key, "TKP") == 0)
    {
        Angle_PID.kp = g_tuning_cmd.value * 0.01f;
        PID_cleardata(&Angle_PID);
    }
    else if (strcmp(g_tuning_cmd.key, "TKD") == 0)
    {
        Angle_PID.kd = g_tuning_cmd.value * 0.01f;   // (Angle_PID.kd is Derivative-on-Measurement)
    }
    else if (strcmp(g_tuning_cmd.key, "GKP") == 0)
    {
        Gyro_PID.kp = g_tuning_cmd.value * 0.001f;
    }
    else if (strcmp(g_tuning_cmd.key, "GKI") == 0)
    { 
        Gyro_PID.ki = g_tuning_cmd.value * 0.001f;
    }
    else if (strcmp(g_tuning_cmd.key, "GKD") == 0)
    {
        Gyro_PID.kd = g_tuning_cmd.value * 0.001f;
    }
    /* ---------- base speed ---------- */
    else if (strcmp(g_tuning_cmd.key, "BSP") == 0)
    {
        Basic_Speed = (int16)g_tuning_cmd.value;
    }
    /* ---------- debug parameters ---------- */
    else if (strcmp(g_tuning_cmd.key, "FAN") == 0)
        Debug_Fan_Duty = (int)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "TGT") == 0)
        Debug_Target_Speed = (int)g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "GDI") == 0)
        Debug_Ground_Dir = ((uint8)g_tuning_cmd.value == 0) ? 0 : 1;
    else if (strcmp(g_tuning_cmd.key, "FFM") == 0)
        Debug_Ground_FF_Mode = (g_tuning_cmd.value > 0) ? 1 : 0;
    else if (strcmp(g_tuning_cmd.key, "GFM") == 0)
        Debug_Gyro_FF_Mode = (g_tuning_cmd.value > 0) ? 1 : 0;
    else if (strcmp(g_tuning_cmd.key, "AMO") == 0)
    {
        uint8 amo = (uint8)g_tuning_cmd.value;
        if (amo >= 1 && amo <= 3) Debug_Angle_Mode = amo;
    }
    else if (strcmp(g_tuning_cmd.key, "AVT") == 0)
        Debug_Angle_Vel_Target = g_tuning_cmd.value;
    else if (strcmp(g_tuning_cmd.key, "MEN") == 0)
    {
        Debug_Motor_Enable = (g_tuning_cmd.value > 0) ? 1 : 0;
        if (Debug_Motor_Enable) Stop_Flag = 0;  // clear stale stop
    }
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
    /* ---------- Flash save ---------- */
    else if (strcmp(g_tuning_cmd.key, "SAV") == 0)
        Uart_Adjust_SaveToFlash();

    g_tuning_cmd.valid = 0;
}

/* Persist current PID + speed + debug parameters to Flash */
void Uart_Adjust_SaveToFlash(void)
{
    PID_OKb[0] = (uint32)Left_PID.kp;
    PID_OKb[1] = (uint32)(Left_PID.ki * 100.0f);
    PID_OKb[2] = (uint32)Right_PID.kp;
    PID_OKb[3] = (uint32)(Right_PID.ki * 100.0f);
    PID_OKb[4] = (uint32)(Angle_PID.kp * 100.0f);
    PID_OKb[5] = (uint32)(Angle_PID.kd * 100.0f);
    PID_OKb[6] = (uint32)(Gyro_PID.kp * 1000.0f);
    PID_OKb[7] = (uint32)(Gyro_PID.ki * 1000.0f);
    PID_OKb[8] = (uint32)(Gyro_PID.kd * 1000.0f);

    Speed_OKb[0] = (uint32)Basic_Speed;

    DBG_OKb[0] = (uint32)Debug_Target_Speed;
    DBG_OKb[1] = (uint32)Debug_Fan_Duty;
    DBG_OKb[2] = (uint32)Debug_Ground_Dir;
    DBG_OKb[3] = (uint32)Debug_Angle_Mode;

    flash_erase_page(0, 0);
    flash_write_page(0, 0, Speed_OKb, 1);

    flash_erase_page(0, 1);
    flash_write_page(0, 1, PID_OKb, 13);

    flash_erase_page(0, 3);
    flash_write_page(0, 3, DBG_OKb, 4);
}
