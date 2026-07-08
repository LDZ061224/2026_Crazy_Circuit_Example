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

/* 宸﹁疆閫熷害鍓嶉琛� 鈥斺�� PWM 鍊肩暀绌猴紝瀹炶溅鏍囧畾鍚庢墜鍔ㄥ～鍏� */
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

/* 鍙宠疆閫熷害鍓嶉琛� 鈥斺�� PWM 鍊肩暀绌猴紝瀹炶溅鏍囧畾鍚庢墜鍔ㄥ～鍏� */
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

/* 瑙掗�熷害鍓嶉琛� 鈥斺�� delta_V 鍊肩暀绌猴紝瀹炶溅鏍囧畾鍚庢墜鍔ㄥ～鍏�
   gyro_rate = 鐩爣瑙掗�熷害 (deg/s), delta_v = 宸﹀彸杞樊閫燂紙缂栫爜鍣� tick/3ms锛� */
static const Gyro_FF_Point_t Gyro_FF_Table[GYRO_FF_TABLE_SIZE] = {
    {0,    0},
    {200,  0},
    {400,  0},
    {600,  0},
    {800,  0},
    {1000, 0},
    {1200, 0},
};

/* 鐢靛帇婊ゆ尝鐘舵�佸彉閲� */
static float Voltage_Raw  = SPEED_FF_VOLTAGE_REF;   // 鍘熷 ADC 鎹㈢畻鍊硷紙VOFA 瑙傚療鐢級
static float Voltage_Fast = SPEED_FF_VOLTAGE_REF;   // 蹇�� EMA锛堥鐣欙級
static float Voltage_Slow = SPEED_FF_VOLTAGE_REF;   // 鎱㈤�� EMA锛堝墠棣堢數鍘嬭ˉ鍋跨敤锛�

/********************************** 閫熷害鍓嶉杈呭姪鍑芥暟 **********************************/

/**
 * @brief  閫熷害鍓嶉鏌ヨ〃 + 绾挎�ф彃鍊�
 * @param  target_speed  甯︾鍙风殑鐩爣閫熷害锛堢紪鐮佸櫒 tick/3ms锛�
 * @param  table         鍓嶉琛ㄦ寚閽堬紙宸叉寜 speed 鍗囧簭鎺掑垪锛�
 * @param  table_size    琛ㄩ」鏁�
 * @return 甯︾鍙风殑鍓嶉 PWM锛�0~卤10000锛�
 *
 * 閫昏緫锛氬彇 target_speed 缁濆鍊兼煡琛� 鈫� 杈圭晫閽充綅 / 绾挎�ф彃鍊� 鈫� 鎭㈠绗﹀彿
 */
static float Speed_FF_GetPwm(float target_speed,
                             const Speed_FF_Point_t *table,
                             uint16_t table_size)
{
    float sign = (target_speed >= 0.0f) ? 1.0f : -1.0f;
    float abs_speed = fabsf(target_speed);

    // 灏忎簬琛ㄦ渶灏忓�� 鈫� 杩斿洖鏈�灏� PWM
    if (abs_speed <= table[0].speed)
        return sign * table[0].pwm;

    // 澶т簬琛ㄦ渶澶у�� 鈫� 杩斿洖鏈�澶� PWM
    if (abs_speed >= table[table_size - 1].speed)
        return sign * table[table_size - 1].pwm;

    // 绾挎�ф彃鍊�
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
    return 0.0f;  // 鐞嗚涓婁笉鍙揪锛屾秷闄ょ紪璇戝櫒璀﹀憡
}

/**
 * @brief  瑙掗�熷害鍓嶉鏌ヨ〃 + 绾挎�ф彃鍊�
 * @param  target_gyro  甯︾鍙风殑鐩爣瑙掗�熷害 (deg/s)
 * @return 甯︾鍙风殑宸﹀彸杞樊閫� delta_V锛堢紪鐮佸櫒 tick/3ms锛�
 *
 * 閫昏緫涓� Speed_FF_GetPwm 涓�鑷达細鍙栫粷瀵瑰�兼煡琛� 鈫� 杈圭晫閽充綅/绾挎�ф彃鍊� 鈫� 鎭㈠绗﹀彿
 */
static float Gyro_FF_GetDeltaV(float target_gyro)
{
    float sign = (target_gyro >= 0.0f) ? 1.0f : -1.0f;
    float abs_rate = fabsf(target_gyro);

    // 灏忎簬琛ㄦ渶灏忓�� 鈫� 杩斿洖鏈�灏� delta_V
    if (abs_rate <= Gyro_FF_Table[0].gyro_rate)
        return sign * Gyro_FF_Table[0].delta_v;

    // 澶т簬琛ㄦ渶澶у�� 鈫� 杩斿洖鏈�澶� delta_V
    if (abs_rate >= Gyro_FF_Table[GYRO_FF_TABLE_SIZE - 1].gyro_rate)
        return sign * Gyro_FF_Table[GYRO_FF_TABLE_SIZE - 1].delta_v;

    // 绾挎�ф彃鍊�
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
    return 0.0f;  // 鐞嗚涓婁笉鍙揪锛屾秷闄ょ紪璇戝櫒璀﹀憡
}

/**
 * @brief  瀵瑰墠棣� PWM 鍋氱數姹犵數鍘嬭ˉ鍋�
 *
 * 鍙ˉ鍋垮墠棣堥」锛屼笉琛ュ伩 PID 璇樊淇閲忋��
 * ff_pwm_comp = ff_pwm * VOLTAGE_REF / voltage_slow锛堝甫涓婁笅闄愰挸浣嶏級
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
 * @brief  鏇存柊鐢靛帇婊ゆ尝閾撅紙灏栧嘲鍓旈櫎 + 鍙� EMA锛�
 *
 * 姣忔帶鍒跺懆鏈燂紙3ms锛夎皟鐢ㄤ竴娆★紝鍦ㄦ煡鍓嶉琛ㄤ箣鍓嶈皟鐢ㄣ��
 * 浣跨敤宸ョ▼宸叉湁鐨� Voltage_Check[0] 浣滀负鍘熷鐢靛帇鏉ユ簮锛屼笉閲嶅鍐� ADC 椹卞姩銆�
 */
static void Debug_Voltage_Filter_Update(float voltage_adc)
{
    float raw = voltage_adc;

    // 灏栧嘲鍓旈櫎锛氬崟娆￠噰鏍峰亸绂绘參閫熻秼鍔胯秴杩囬槇鍊煎垯閽充綅
    if (raw > Voltage_Slow + VOLTAGE_SPIKE_LIMIT)
        raw = Voltage_Slow + VOLTAGE_SPIKE_LIMIT;
    else if (raw < Voltage_Slow - VOLTAGE_SPIKE_LIMIT)
        raw = Voltage_Slow - VOLTAGE_SPIKE_LIMIT;

    Voltage_Raw  = voltage_adc;                                // 鍘熷鍊硷紝渚� VOFA 瑙傚療
    Voltage_Fast += VOLTAGE_FAST_ALPHA * (raw - Voltage_Fast); // 蹇�� EMA锛堥鐣欙級
    Voltage_Slow += VOLTAGE_SLOW_ALPHA * (raw - Voltage_Slow); // 鎱㈤�� EMA锛堝墠棣堣ˉ鍋跨敤锛�
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
//    if (EnableSwitch_ON == 1 && Last_EnableSwitch_ON == 0)
//    {
//        Count.StartDelay = 100;
//    }
//    Last_EnableSwitch_ON = EnableSwitch_ON;
//
//    /* debug mode skips the startup delay */
//    Count.StartDelay = 0;

    /* --- alternating speed read (every 3ms, same as Car_Go) --- */
   Get_Speed();

    /* --- IMU + sensors + safety --- */
    if(EnableSwitch_ON == 1)
    {
        Get_IMU();
    }
    Get_Light();
//    Light_Process();
//    Safety_Check();
//
//    if (Stop_Flag != 0)
//    {
//        return;
//    }
//
//    /* --- dispatch to specific debug sub-mode --- */
//    switch (Debug_Sub_Mode)
//    {
//        case Debug_Sub_PI_Tuning:   Debug_Wheel_Tuning();   break;
//        case Debug_Sub_Ground_Test: Debug_Ground_Test();     break;
//        case Debug_Sub_Angle:       Debug_Angle_Tuning();   break;
//        case Debug_Sub_NormalTrace: Debug_Normal_Trace();   break;
//        case Debug_Sub_Curvature_Trace: Debug_Curvature_Trace(); break;
//        default:                    Debug_Wheel_Tuning();   break;
//    }

    // LED: only set when Stop_Flag==0. Safety_Check handles stop/low-voltage states.
//    if (g_led_flag != 2)
//        g_led_flag = Debug_Motor_Enable ? 1 : 0;
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
    pwm_set_duty(Suction_Motor_PWM, 500);
    pwm_set_duty(Suction_Motor_DIR, 10000);
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

        // ----- 閫熷害 PI 璁＄畻锛堜袱绉嶆ā寮忓叡鐢級-----
        float pid_out_left  = PID_calc(&Left_PID,  (float)Left_Exp_Spd,  (float)Left_Real_Spd);
        float pid_out_right = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);

        if (Debug_Ground_FF_Mode == 1)
        {
            // ----- 鍓嶉妯″紡锛欶F PWM + PI 淇 -----

            // 1. 鏇存柊鐢靛帇婊ゆ尝锛堜娇鐢ㄥ伐绋嬪凡鏈夌殑 Voltage_Check[0]锛�
            Debug_Voltage_Filter_Update(Voltage_Check[0]);

            // 2. 鏌ュ墠棣堣〃
            float ff_left  = Speed_FF_GetPwm((float)Left_Exp_Spd,
                                             Left_Speed_FF_Table,
                                             LEFT_SPEED_FF_TABLE_SIZE);
            float ff_right = Speed_FF_GetPwm((float)Right_Exp_Spd,
                                             Right_Speed_FF_Table,
                                             RIGHT_SPEED_FF_TABLE_SIZE);

            // 3. 鐢靛帇琛ュ伩锛堝彧琛ュ伩鍓嶉椤癸級
            ff_left  = Speed_FF_VoltageComp(ff_left,  Voltage_Slow);
            ff_right = Speed_FF_VoltageComp(ff_right, Voltage_Slow);

            // 4. 鍚堟垚鏈�缁堣緭鍑� = 鍓嶉琛ュ伩 PWM + PID 璇樊淇
            Left_PID_Out  = ff_left  + pid_out_left;
            Right_PID_Out = ff_right + pid_out_right;
        }
        else
        {
            // ----- 鍘熷绾� PI 妯″紡锛堜笉鍙橈級-----
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
        PID_cleardata(&Turn_PID);
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Gyro_PD_PID);
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        Debug_Set_Out();
        return;
    }

    if (Debug_Angle_Mode == 2)
    {
        // ----- angle tracking (Turn_PID outer + Gyro_PID inner) -----
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
        // This bypasses Turn_PID -- pure rate-loop tuning
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

    // ----- 瑙掗�熷害鍐呯幆 -----
    float gyro_pid_out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);

    if (Debug_Gyro_FF_Mode == 1)
    {
        // 鍓嶉妯″紡锛氭煡琛ㄥ緱 delta_V + PID 淇
        float ff_delta_v = Gyro_FF_GetDeltaV(gyro_target);
        Gyro_PID_Out = ff_delta_v + gyro_pid_out;
    }
    else
    {
        // 鍘熷绾� PID 妯″紡
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
        Turn_PID_Out = PID_calc(&Turn_PID, 0.0f, (float)Error);
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

/**********************************Debug_Curvature_Trace**********************************/

void Debug_Curvature_Trace(void)
{
    if (Debug_Motor_Enable == 0 || EnableSwitch_ON == 0)
    {
        Error = 0;
        Gyro_PID_Out = 0;
        Turn_PID_Out = 0;
        Left_Exp_Spd = 0;
        Right_Exp_Spd = 0;
        Left_PID_Out = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Gyro_PD_PID);
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        Debug_Set_Out();
        return;
    }

    // Compute Error (same as Normal_Trace)
    if (Track_Num > 0) {
        Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
        Last_Error = Error;
    }
    if (Track_Num < 2)
        Error = 0;
    else {
        Left_Scan_Point = Track_Arr[0];
        Right_Scan_Point = Track_Arr[Track_Num - 1];
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
    }

    // Parallel: curvature feedforward + gyro PD damping
    float e = (float)Error * SENSOR_PITCH_MM;
    float curvature = 2.0f * e / (LD_MM * LD_MM + e * e);
    float diff_track = Turn_PID.kp * curvature * TRACK_WIDTH_MM * (float)Debug_Target_Speed;
    float diff_damp  = PID_calc(&Gyro_PD_PID, 0.0f, Gyro_Z);

    // VOFA debug vars: channel 1=curvature diff, channel 2=actual gyro rate
    Debug_Angle_Vel_Target = diff_track;
    Debug_Angle_Vel_Real   = Gyro_Z;

    Left_Exp_Spd  = Debug_Target_Speed + (int)diff_track + (int)diff_damp;
    Right_Exp_Spd = Debug_Target_Speed - (int)diff_track - (int)diff_damp;

    Left_PID_Out  = PID_calc(&Left_PID,  (float)Left_Exp_Spd,  (float)Left_Real_Spd);
    Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
    Debug_Set_Out();
}

/**********************************Debug_Set_Out**********************************/

void Debug_Set_Out(void)
{
    if (Debug_Motor_Enable != 0)
    {
        pwm_set_duty(Suction_Motor_PWM, Debug_Fan_Duty);
        pwm_set_duty(Suction_Motor_DIR, 10000);     // Same as Car_Go: 0=suction
    }
    else
    {
        pwm_set_duty(Suction_Motor_PWM, 9500);
        pwm_set_duty(Suction_Motor_DIR, 10000);
    }

    if (Debug_Motor_Enable == 0 || Left_PID_Out == 0)
    {
        pwm_set_duty(Left_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_PWM, 0);
    }
    else if (Left_PID_Out > 0)   // forward
    {
        pwm_set_duty(Left_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_PWM, fabs(Left_PID_Out));
    }
    else                         // reverse: DIR=0
    {
        pwm_set_duty(Left_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_PWM, fabs(Left_PID_Out));
    }


    if (Debug_Motor_Enable == 0 || Right_PID_Out == 0)
    {
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);
    }
    else if (Right_PID_Out > 0)  // forward: DIR=0
    {
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, fabs(Right_PID_Out));
    }
    else                         // reverse: DIR=10000
    {
        pwm_set_duty(Right_Motor_DIR, 10000);
        pwm_set_duty(Right_Motor_PWM, fabs(Right_PID_Out));
    }
}
