/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Debug_Car.c
Author: Claude (extracted from Ctrl.c)
Version:0.0               Date: 2026.6.29
Description:  Debug car control implementation.
              Called from Car_Go() when USE_DEBUG_MODE=1 and Mode==Debug_Mode.
              Contains: Debug_Wheel_Tuning, Debug_Ground_Test,
                        Debug_Angle_Tuning, Debug_Normal_Trace, Debug_Set_Out
Others:      Shares global PID / speed / sensor variables via Ctrl.h externs.
**************************************************/

#include "Debug_Car.h"

/* globals from Ctrl.c (needed for preprocessing + NormalTrace) */
extern int   Track_Arr[15];
extern int16_t   Dir_Arr[15];
extern int   Left_Scan_Point;
extern int   Right_Scan_Point;
extern int   Last_Error;

/* file-scope in Ctrl.c -- needed for common preprocessing */
// Count.StartDelay now in Count_Typedef struct, no standalone extern needed
extern uint8_t  Last_EnableSwitch_ON;

/* from Ctrl.c -- speed feedforward mode flag */
extern uint8 Debug_Ground_FF_Mode;

/******************************** speed feedforward **********************************/

/* 左轮速度前馈表 —— PWM 值留空，实车标定后手动填入 */
static const Speed_FF_Point_t Left_Speed_FF_Table[LEFT_SPEED_FF_TABLE_SIZE] = {
    {0,   0},
    {10,  0},
    {20,  0},
    {30,  0},
    {50,  0},
    {80,  0},
    {120, 0},
    {160, 0},
    {200, 0},
};

/* 右轮速度前馈表 —— PWM 值留空，实车标定后手动填入 */
static const Speed_FF_Point_t Right_Speed_FF_Table[RIGHT_SPEED_FF_TABLE_SIZE] = {
    {0,   0},
    {10,  0},
    {20,  0},
    {30,  0},
    {50,  0},
    {80,  0},
    {120, 0},
    {160, 0},
    {200, 0},
};

/* 角速度前馈表 —— delta_V 值留空，实车标定后手动填入
   gyro_rate = 目标角速度 (deg/s), delta_v = 左右轮差速（编码器 tick/3ms） */
static const Gyro_FF_Point_t Gyro_FF_Table[GYRO_FF_TABLE_SIZE] = {
    {0,    0},
    {200,  0},
    {400,  0},
    {600,  0},
    {800,  0},
    {1000, 0},
    {1200, 0},
};

/* 电压滤波状态变量 */
static float Voltage_Raw  = SPEED_FF_VOLTAGE_REF;   // 原始 ADC 换算值（VOFA 观察用）
static float Voltage_Fast = SPEED_FF_VOLTAGE_REF;   // 快速 EMA（预留）
static float Voltage_Slow = SPEED_FF_VOLTAGE_REF;   // 慢速 EMA（前馈电压补偿用）

/********************************** 速度前馈辅助函数 **********************************/

/**
 * @brief  速度前馈查表 + 线性插值
 * @param  target_speed  带符号的目标速度（编码器 tick/3ms）
 * @param  table         前馈表指针（已按 speed 升序排列）
 * @param  table_size    表项数
 * @return 带符号的前馈 PWM（0~±10000）
 *
 * 逻辑：取 target_speed 绝对值查表 → 边界钳位 / 线性插值 → 恢复符号
 */
static float Speed_FF_GetPwm(float target_speed,
                             const Speed_FF_Point_t *table,
                             uint16_t table_size)
{
    float sign = (target_speed >= 0.0f) ? 1.0f : -1.0f;
    float abs_speed = fabsf(target_speed);

    // 小于表最小值 → 返回最小 PWM
    if (abs_speed <= table[0].speed)
        return sign * table[0].pwm;

    // 大于表最大值 → 返回最大 PWM
    if (abs_speed >= table[table_size - 1].speed)
        return sign * table[table_size - 1].pwm;

    // 线性插值
    for (uint16_t i = 0; i < table_size - 1; i++)
    {
        if (abs_speed >= table[i].speed && abs_speed <= table[i + 1].speed)
        {
            float t = (abs_speed - table[i].speed)
                    / (table[i + 1].speed - table[i].speed);
            float pwm = table[i].pwm + t * (table[i + 1].pwm - table[i].pwm);
            return sign * pwm;
        }
    }
    return 0.0f;  // 理论上不可达，消除编译器警告
}

/**
 * @brief  角速度前馈查表 + 线性插值
 * @param  target_gyro  带符号的目标角速度 (deg/s)
 * @return 带符号的左右轮差速 delta_V（编码器 tick/3ms）
 *
 * 逻辑与 Speed_FF_GetPwm 一致：取绝对值查表 → 边界钳位/线性插值 → 恢复符号
 */
static float Gyro_FF_GetDeltaV(float target_gyro)
{
    float sign = (target_gyro >= 0.0f) ? 1.0f : -1.0f;
    float abs_rate = fabsf(target_gyro);

    // 小于表最小值 → 返回最小 delta_V
    if (abs_rate <= Gyro_FF_Table[0].gyro_rate)
        return sign * Gyro_FF_Table[0].delta_v;

    // 大于表最大值 → 返回最大 delta_V
    if (abs_rate >= Gyro_FF_Table[GYRO_FF_TABLE_SIZE - 1].gyro_rate)
        return sign * Gyro_FF_Table[GYRO_FF_TABLE_SIZE - 1].delta_v;

    // 线性插值
    for (uint16_t i = 0; i < GYRO_FF_TABLE_SIZE - 1; i++)
    {
        if (abs_rate >= Gyro_FF_Table[i].gyro_rate
            && abs_rate <= Gyro_FF_Table[i + 1].gyro_rate)
        {
            float t = (abs_rate - Gyro_FF_Table[i].gyro_rate)
                    / (Gyro_FF_Table[i + 1].gyro_rate - Gyro_FF_Table[i].gyro_rate);
            float dv = Gyro_FF_Table[i].delta_v
                     + t * (Gyro_FF_Table[i + 1].delta_v - Gyro_FF_Table[i].delta_v);
            return sign * dv;
        }
    }
    return 0.0f;  // 理论上不可达，消除编译器警告
}

/**
 * @brief  对前馈 PWM 做电池电压补偿
 *
 * 只补偿前馈项，不补偿 PID 误差修正量。
 * ff_pwm_comp = ff_pwm * VOLTAGE_REF / voltage_slow（带上下限钳位）
 */
static float Speed_FF_VoltageComp(float ff_pwm, float voltage_slow)
{
    float v = voltage_slow;

    if (v < SPEED_FF_VOLTAGE_MIN) v = SPEED_FF_VOLTAGE_MIN;
    if (v > SPEED_FF_VOLTAGE_MAX) v = SPEED_FF_VOLTAGE_MAX;

    float comp = SPEED_FF_VOLTAGE_REF / v;

    if (comp < SPEED_FF_COMP_MIN) comp = SPEED_FF_COMP_MIN;
    if (comp > SPEED_FF_COMP_MAX) comp = SPEED_FF_COMP_MAX;

    return ff_pwm * comp;
}

/**
 * @brief  更新电压滤波链（尖峰剔除 + 双 EMA）
 *
 * 每控制周期（3ms）调用一次，在查前馈表之前调用。
 * 使用工程已有的 Voltage_Check[0] 作为原始电压来源，不重复写 ADC 驱动。
 */
static void Debug_Voltage_Filter_Update(float voltage_adc)
{
    float raw = voltage_adc;

    // 尖峰剔除：单次采样偏离慢速趋势超过阈值则钳位
    if (raw > Voltage_Slow + VOLTAGE_SPIKE_LIMIT)
        raw = Voltage_Slow + VOLTAGE_SPIKE_LIMIT;
    else if (raw < Voltage_Slow - VOLTAGE_SPIKE_LIMIT)
        raw = Voltage_Slow - VOLTAGE_SPIKE_LIMIT;

    Voltage_Raw  = voltage_adc;                                // 原始值，供 VOFA 观察
    Voltage_Fast += VOLTAGE_FAST_ALPHA * (raw - Voltage_Fast); // 快速 EMA（预留）
    Voltage_Slow += VOLTAGE_SLOW_ALPHA * (raw - Voltage_Slow); // 慢速 EMA（前馈补偿用）
}

/**********************************debug entry point**********************************/

/*
 *  Debug_Car_Go()
 *  Peer of Car_Go(), called directly from PIT ISR when Mode == Debug_Mode.
 *  Contains its own preprocessing (EnableSwitch / Get_Speed / IMU / Light / Safety)
 *  so it does not depend on Car_Go for anything.
 */
void Debug_Car_Go(void)
{
    /* --- EnableSwitch edge detection --- */
    if (EnableSwitch_ON == 1 && Last_EnableSwitch_ON == 0)
    {
        Count.StartDelay = 100;
    }
    Last_EnableSwitch_ON = EnableSwitch_ON;

    /* debug mode skips the startup delay */
    Count.StartDelay = 0;

    /* --- alternating speed read (every 3ms, same as Car_Go) --- */
    Get_Speed();

    /* --- IMU + sensors + safety --- */
    Get_IMU();
    Get_Light();
//    Light_Process();
    Safety_Check();

    if (Stop_Flag != 0)
    {
        return;
    }

    /* --- dispatch to specific debug sub-mode --- */
    switch (Debug_Sub_Mode)
    {
        case Debug_Sub_PI_Tuning:   Debug_Wheel_Tuning();   break;
        case Debug_Sub_Ground_Test: Debug_Ground_Test();     break;
        case Debug_Sub_Angle:       Debug_Angle_Tuning();   break;
        case Debug_Sub_NormalTrace: Debug_Normal_Trace();   break;
        default:                    Debug_Wheel_Tuning();   break;
    }

    // LED: green=motor off, blue=motor on (Safety_Check overrides on stop)
    g_led_flag = Debug_Motor_Enable ? 1 : 0;
}

/**********************************Debug_Wheel_Tuning**********************************/

void Debug_Wheel_Tuning(void)
{
    if (Debug_Motor_Enable == 0)
    {

        Left_PID_Out  = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
    }
    else
    {
        float kp, ki;

        if (Debug_Which_Wheel == 0)
        {
            Left_Exp_Spd = Debug_Target_Speed;
            Right_Exp_Spd = 0;

            Left_PID_Out  = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
            Right_PID_Out = 0;
            PID_cleardata(&Right_PID);
        }
        else
        {
            Right_Exp_Spd = Debug_Target_Speed;
            Left_Exp_Spd  = 0;

            Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
            Left_PID_Out  = 0;
            PID_cleardata(&Left_PID);
        }
    }

    Debug_Set_Out();
    pwm_set_duty(Suction_Motor_PWM, 0);
    pwm_set_duty(Suction_Motor_DIR, 0);
}

/**********************************Debug_Ground_Test**********************************/

void Debug_Ground_Test(void)
{
    if (Debug_Motor_Enable == 1 && EnableSwitch_ON == 1)
    {

        if(Debug_Ground_Dir == 1)
        {
            Left_Exp_Spd  =  Debug_Target_Speed;
            Right_Exp_Spd = -Debug_Target_Speed;
        }
        else  // Debug_Ground_Dir == 0
        {
            Left_Exp_Spd  = -Debug_Target_Speed;
            Right_Exp_Spd =  Debug_Target_Speed;
        }

        // ----- 速度 PI 计算（两种模式共用）-----
        float pid_out_left  = PID_calc(&Left_PID,  (float)Left_Exp_Spd,  (float)Left_Real_Spd);
        float pid_out_right = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);

        if (Debug_Ground_FF_Mode == 1)
        {
            // ----- 前馈模式：FF PWM + PI 修正 -----

            // 1. 更新电压滤波（使用工程已有的 Voltage_Check[0]）
            Debug_Voltage_Filter_Update(Voltage_Check[0]);

            // 2. 查前馈表
            float ff_left  = Speed_FF_GetPwm((float)Left_Exp_Spd,
                                             Left_Speed_FF_Table,
                                             LEFT_SPEED_FF_TABLE_SIZE);
            float ff_right = Speed_FF_GetPwm((float)Right_Exp_Spd,
                                             Right_Speed_FF_Table,
                                             RIGHT_SPEED_FF_TABLE_SIZE);

            // 3. 电压补偿（只补偿前馈项）
            ff_left  = Speed_FF_VoltageComp(ff_left,  Voltage_Slow);
            ff_right = Speed_FF_VoltageComp(ff_right, Voltage_Slow);

            // 4. 合成最终输出 = 前馈补偿 PWM + PID 误差修正
            Left_PID_Out  = ff_left  + pid_out_left;
            Right_PID_Out = ff_right + pid_out_right;
        }
        else
        {
            // ----- 原始纯 PI 模式（不变）-----
            Left_PID_Out  = pid_out_left;
            Right_PID_Out = pid_out_right;
        }
    }
    else
    {
        Left_Exp_Spd  = 0;
        Right_Exp_Spd = 0;
        Left_PID_Out  = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
    }
    Debug_Set_Out();
}

/**********************************Debug_Angle_Tuning**********************************/

void Debug_Angle_Tuning(void)
{
    static uint32 angle_tick = 0;
    float angle_target;
    float gyro_target;

    if (Debug_Motor_Enable == 0 || EnableSwitch_ON == 0)
    {
        angle_tick = 0;
        Gyro_Integral = 0;
        Turn_PID_Out = 0;
        Gyro_PID_Out = 0;
        Debug_Angle_Vel_Target = 0;
        Debug_Angle_Vel_Real = 0;
        Left_Exp_Spd = 0;
        Right_Exp_Spd = 0;
        Left_PID_Out = 0;
        Right_PID_Out = 0;
        Debug_Angle_D_First = 0;
        PID_cleardata(&Angle_PID);
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Gyro_PD_PID);
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        Debug_Set_Out();
        return;
    }

    if (Debug_Angle_Mode == 2)
    {
        // ----- angle tracking (Angle_PID outer + Gyro_PID inner) -----
        uint32 step_index = angle_tick / DEBUG_ANGLE_STEP_TICKS;
        uint32 phase = step_index % 8U;
        if (phase <= 4U)
        {
            angle_target = (float)phase * 90.0f;
        }
        else
        {
            angle_target = (float)(8U - phase) * 90.0f;
        }

        Debug_Angle_D_First = 1;
        Turn_PID_Out = PID_calc(&Angle_PID, angle_target, Gyro_Integral);
        gyro_target = Turn_PID_Out;
    }
    else if (Debug_Angle_Mode == 3)
    {
        // ----- direct gyro rate target (Gyro_PID only, set by serial AVT) -----
        // User sends @AVT=200# to set Debug_Angle_Vel_Target=200 deg/s
        // This bypasses Angle_PID -- pure rate-loop tuning
        angle_target = 0;
        Turn_PID_Out = 0;
        Debug_Angle_D_First = 0;
        gyro_target = Debug_Angle_Vel_Target;
    }
    else
    {
        // ----- sine rate target (default mode 1) -----
        angle_target = 0;
        Turn_PID_Out = 0;
        Debug_Angle_D_First = 0;
        gyro_target = 1200.0f * sinf(6.2831853f * (float)(angle_tick % 333U) / 333.0f);
    }

    Debug_Angle_Vel_Target = gyro_target;
    Debug_Angle_Vel_Real = Gyro_Z;

    // ----- 角速度内环 -----
    float gyro_pid_out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);

    if (Debug_Gyro_FF_Mode == 1)
    {
        // 前馈模式：查表得 delta_V + PID 修正
        float ff_delta_v = Gyro_FF_GetDeltaV(gyro_target);
        Gyro_PID_Out = ff_delta_v + gyro_pid_out;
    }
    else
    {
        // 原始纯 PID 模式
        Gyro_PID_Out = gyro_pid_out;
    }

    Left_Exp_Spd = Debug_Target_Speed + Gyro_PID_Out;
    Right_Exp_Spd = Debug_Target_Speed - Gyro_PID_Out;

    Left_PID_Out = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
    Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);

    angle_tick++;
    Debug_Set_Out();
}

/**********************************Debug_Normal_Trace**********************************/

void Debug_Normal_Trace(void)
{
    if (Debug_Motor_Enable == 0 || EnableSwitch_ON == 0)
    {
        Error = 0;
        Turn_PID_Out = 0;
        Gyro_PID_Out = 0;
        Debug_Angle_Vel_Target = 0;
        Debug_Angle_Vel_Real = 0;
        Left_Exp_Spd = 0;
        Right_Exp_Spd = 0;
        Left_PID_Out = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Turn_PID);
        PID_cleardata(&Angle_PID);
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Gyro_PD_PID);
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        Debug_Set_Out();
        return;
    }

    if (Track_Num > 0)
    {
        Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
        Last_Error = Error;
    }

    if (Track_Num < 2)
    {
        Error = 0;
    }
    else
    {
        Left_Scan_Point = Track_Arr[0];
        Right_Scan_Point = Track_Arr[Track_Num - 1];
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
    }

    {
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, (float)Error);
    }
    Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);

    Debug_Angle_Vel_Target = Turn_PID_Out;
    Debug_Angle_Vel_Real = Gyro_Z;

    Left_Exp_Spd = Debug_Target_Speed + Gyro_PID_Out;
    Right_Exp_Spd = Debug_Target_Speed - Gyro_PID_Out;

    Left_PID_Out = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
    Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
    Debug_Set_Out();
}

/**********************************Debug_Set_Out**********************************/

void Debug_Set_Out(void)
{
    if (Debug_Motor_Enable != 0)
    {
        pwm_set_duty(Suction_Motor_PWM, Debug_Fan_Duty);
        pwm_set_duty(Suction_Motor_DIR, 0);     // Same as Car_Go: 0=suction
    }
    else
    {
        pwm_set_duty(Suction_Motor_PWM, 0);
        pwm_set_duty(Suction_Motor_DIR, 0);
    }

    if (Debug_Motor_Enable == 0 || Left_PID_Out == 0)
    {
        pwm_set_duty(Left_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_PWM, 0);
    }
    else if (Left_PID_Out > 0)   // forward
    {
        pwm_set_duty(Left_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_PWM, fabs(Left_PID_Out));
    }
    else                         // reverse
    {
        pwm_set_duty(Left_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_PWM, fabs(Left_PID_Out));
    }


    if (Debug_Motor_Enable == 0 || Right_PID_Out == 0)
    {
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);
    }
    else if (Right_PID_Out > 0)  // forward
    {
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, fabs(Right_PID_Out));
    }
    else                         // reverse
    {
        pwm_set_duty(Right_Motor_DIR, 10000);
        pwm_set_duty(Right_Motor_PWM, fabs(Right_PID_Out));
    }
}
