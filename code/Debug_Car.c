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
extern int      Speed_Get_Count;
extern int      Enable_Start_Delay_Count;
extern uint8_t  Last_EnableSwitch_ON;

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
        Enable_Start_Delay_Count = 100;
    }
    Last_EnableSwitch_ON = EnableSwitch_ON;

    /* debug mode skips the startup delay */
    Enable_Start_Delay_Count = 0;

    /* --- alternating speed read (every 6ms, same as Car_Go) --- */
    if (Speed_Get_Count == 1)
    {
        Get_Speed();
    }
    Speed_Get_Count *= -1;

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
            Left_Exp_Spd  = Debug_Target_Speed;
            Right_Exp_Spd = -Debug_Target_Speed;
        }
        else if(Debug_Ground_Dir == 0)
        {
            Left_Exp_Spd  = -Debug_Target_Speed;
            Right_Exp_Spd = Debug_Target_Speed;
        }

        Left_PID_Out  = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
        Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
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
    Gyro_PID_Out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);

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
