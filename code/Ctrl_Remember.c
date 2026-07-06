/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Remember.c
Author: Cross_Z
Version:4.1               Date: 2026.7.6
Description: Remember (replay) mode — simplified V4.1
             - All actions triggered by Check_Edge (sensor), NOT mileage
             - Mileage used ONLY for trapezoidal speed curve
             - No curve turns, no odometry compensation, no Snap correction
             - Turn: PID diff right-angle turn with gyro integral check
             - Straight_Drive: simple mileage consumption between turns
             Only compiled when ACTIVE_MODE == MODE_REMEMBER
**************************************************/

#include "headfiles.h"
#include "Ctrl_Remember.h"
#include "Racing_Track.h"

#if ACTIVE_MODE == MODE_REMEMBER

/********************************* Remember Mode Global Variables *********************************/

// Gyro ring buffer for heading lock
float   gyro_buf[HEADING_BUF_SIZE] = {0};
uint8_t heading_buf_idx = 0;
uint8_t heading_buf_full = 0;

// Speed curve state
float   Remember_Section_Start_Mileage = 0;
uint8_t Remember_Turn_Index = 0;

// Straight drive heading lock
float   Straight_Heading_Target = 0;
uint8_t Straight_Heading_Locked = 0;
float   Straight_Last_Heading_Err = 0;

// Remember mode parameters
int Remember_Turn_Error = REMEMBER_TURN_ERR;
int Remember_Speed_Min_Value = REMEMBER_SPEED_MIN;
int Remember_Speed_Max_Value = REMEMBER_SPEED_MAX;

// Remember mode runtime state
static uint8_t Remember_First_Mode = 0;
static float   Straight_Drive_Base = 0;
static float   Straight_Drive_Target = 0;

// Heading lock state for straight drive
static int     straight_drive_heading_frames = 0;
static uint8_t straight_drive_heading_locked = 0;

/********************************* Static Function Declarations *********************************/

static void Remember_Normal_Run(void);
static void Remember_Check_Trigger(void);
static int  Remember_Get_Run_Speed(void);
static void Remember_Reset_Runtime_State(void);
static void Straight_Drive_Run(void);
static void Dispatch_Action(uint8_t edge_index);
static int  Get_Turn_Base_Speed(void);

/********************************* Static Helper Functions *********************************/

/*************************************
** Function: Get_Turn_Base_Speed
*************************************/
static int Get_Turn_Base_Speed(void)
{
    int speed = Remember_Get_Run_Speed();
    if (speed < TURN_BASE_SPD_MIN) speed = TURN_BASE_SPD_MIN;
    if (speed > TURN_BASE_SPD_MAX) speed = TURN_BASE_SPD_MAX;
    return speed;
}

/*************************************
** Function: Dispatch_Action
** Description: Dispatch an action from Simple_Track — all actions triggered by Check_Edge
*************************************/
static void Dispatch_Action(uint8_t edge_index)
{
    if (edge_index >= Simple_Track.Action_Count)
        return;

    Action_Entry_Typedef entry = Simple_Track.Action_List[edge_index];
    Current_Action = entry;
    Action_Enum action = entry.action;

    // ─── Compute heading target from gyro_buf for heading lock ───
    Straight_Heading_Locked = 0;
    if (heading_buf_full)
    {
        float sum = 0;
        uint8_t count = 0;
        for (uint8_t i = 1; i < HEADING_BUF_SIZE; i++)
        {
            if (i <= entry.heading_frames || entry.heading_frames == 0)
            {
                sum += gyro_buf[(heading_buf_idx + HEADING_BUF_SIZE - 1 - i) % HEADING_BUF_SIZE];
                count++;
            }
        }
        if (count > 0)
        {
            float heading = sum * GYRO_INTEGRATION_PERIOD_S;
            if (fabsf(heading) >= HEADING_LOCK_MAX_ANGLE)
            {
                Straight_Heading_Target = heading;
                Straight_Heading_Locked = 1;
            }
        }
    }

    // ─── Turn actions: PID diff right-angle turn ───
    if (action == ACTION_TURN_LEFT || action == ACTION_TURN_RIGHT ||
        action == ACTION_ELEM_LEFT || action == ACTION_ELEM_RIGHT ||
        action == ACTION_ELEM_LEFT_BIG || action == ACTION_ELEM_RIGHT_BIG)
    {
        uint8_t is_left_turn = (action == ACTION_TURN_LEFT ||
                                action == ACTION_ELEM_LEFT ||
                                action == ACTION_ELEM_LEFT_BIG) ? 1 : 0;

        // Clear gyro_buf after reading heading
        memset(gyro_buf, 0, sizeof(gyro_buf));
        heading_buf_idx = 0;
        heading_buf_full = 0;

        Turn_Decel_Phase = 0;
        Gyro_Integral = 0;
        Turn_Action_Done = 0;
        is_left = is_left_turn;
        is_right = !is_left_turn;
        Turn_Angle_Target = is_left_turn ? -REMEMBER_TURN_TARGET_ANGLE_DEG : REMEMBER_TURN_TARGET_ANGLE_DEG;
        Run_Mode = is_left_turn ? Turn_Left : Turn_Right;
        Count.Mileage = 0;
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Turn_PID);
        Error = is_left_turn ? -REMEMBER_TURN_ERR : REMEMBER_TURN_ERR;
    }
    // ─── Straight / NONE actions ───
    else
    {
        Straight_Drive_Base = Count.Mileage;
        Straight_Heading_Target = 0;
        Straight_Heading_Locked = 0;
        straight_drive_heading_locked = 0;

        switch (action)
        {
            case ACTION_STRAIGHT_SHORT: Straight_Drive_Target = STRAIGHT_SHORT_MILEAGE; break;
            case ACTION_STRAIGHT_MID:   Straight_Drive_Target = STRAIGHT_MID_MILEAGE;   break;
            case ACTION_STRAIGHT_LONG:  Straight_Drive_Target = STRAIGHT_LONG_MILEAGE;  break;
            case ACTION_NONE:
            default:                    Straight_Drive_Target = STRAIGHT_NODE_MILEAGE;  break;
        }

        straight_drive_heading_frames = entry.heading_frames;
        if (straight_drive_heading_frames > 0 && heading_buf_full)
        {
            float sum = 0;
            uint8_t count = 0;
            for (uint8_t i = 1; i < HEADING_BUF_SIZE; i++)
            {
                if (count < straight_drive_heading_frames)
                {
                    sum += gyro_buf[(heading_buf_idx + HEADING_BUF_SIZE - 1 - i) % HEADING_BUF_SIZE];
                    count++;
                }
            }
            if (count > 0)
            {
                Straight_Heading_Target = sum * GYRO_INTEGRATION_PERIOD_S;
                straight_drive_heading_locked = 1;
                Gyro_Integral = Straight_Heading_Target;
            }
        }

        memset(gyro_buf, 0, sizeof(gyro_buf));
        heading_buf_idx = 0;
        heading_buf_full = 0;
        Run_Mode = Straight_Drive;
    }
}

/*************************************
** Function: Remember_Reset_Runtime_State
*************************************/
static void Remember_Reset_Runtime_State(void)
{
    Edge_Index = 0;
    Run_Mode = Normal_Mode;
    Count.Left = 0;
    Count.Right = 0;
    Count.Mileage = 0;
    Count.StraightBase = 0;
    Count.Straight = 0;
    Count.Stall = 0;
    Count.Edge = 0;
    Count.Line = 0;
    Count.Element = 0;
    Count.Finish = 0;
    Count.StartDelay = 0;
    Finish_Flag = 0;
    Stop_Flag = 0;
    is_left = 0;
    is_right = 0;
    Turn_Action_Done = 0;
    Turn_Decel_Phase = 0;
    Gyro_Integral = 0;
    Total_Run_Mileage = 0;
    Last_Error = 0;
    Error = 0;
    Turn_PID_Out = 0;
    Gyro_PID_Out = 0;
    Left_Exp_Spd = 0;
    Right_Exp_Spd = 0;
    Left_PID_Out = 0;
    Right_PID_Out = 0;

    memset(gyro_buf, 0, sizeof(gyro_buf));
    heading_buf_idx = 0;
    heading_buf_full = 0;
    Remember_Section_Start_Mileage = 0;
    Remember_Turn_Index = 0;
    Straight_Heading_Target = 0;
    Straight_Heading_Locked = 0;
    Straight_Last_Heading_Err = 0;

    PID_cleardata(&Angle_PID);
    PID_cleardata(&Gyro_PID);
    PID_cleardata(&Gyro_PD_PID);
    PID_cleardata(&Turn_PID);
    PID_cleardata(&Left_PID);
    PID_cleardata(&Right_PID);
}

/*************************************
** Function: Remember_Normal_Run
*************************************/
static void Remember_Normal_Run(void)
{
    Last_Error = Error;

    if (Track_Num < 1)
        Error = Last_Error;
    else if (Track_Num < 5 && Track_Num >= 2)
    {
        Left_Scan_Point = Track_Arr[0];
        Right_Scan_Point = Track_Arr[Track_Num - 1];
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
    }
    else
    {
        Left_Scan_Point = Track_Arr[0];
        Right_Scan_Point = Track_Arr[Track_Num - 1];
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
    }

    gyro_buf[heading_buf_idx] = Gyro_Z;
    heading_buf_idx = (heading_buf_idx + 1) % HEADING_BUF_SIZE;
    if (heading_buf_idx == 0) heading_buf_full = 1;
}

/*************************************
** Function: Remember_Check_Trigger
** Description: All actions triggered by Check_Edge (sensor-based)
**              Mileage is NOT used for triggering, only for speed curve.
*************************************/
static void Remember_Check_Trigger(void)
{
    if (Run_Mode != Normal_Mode) return;
    if (Edge_Index >= Simple_Track.Action_Count) return;

    if (Check_Edge())
    {
        Edge_Index++;

        if (Edge_Index >= Simple_Track.Action_Count)
        {
            Finish_Flag = 1;    // all actions done → stop
        }
        else
        {
            Dispatch_Action(Edge_Index - 1);
        }
    }
}

/*************************************
** Function: Remember_Get_Run_Speed
** Description: Trapezoidal speed curve based on Turn_Distance_Rec
**              Section boundary set at turn completion (gyro integral check)
*************************************/
static int Remember_Get_Run_Speed(void)
{
    float section_total_mileage;
    float processed_mileage;
    float effective_total_mileage;

    if (Remember_Speed_Max_Value <= Remember_Speed_Min_Value)
        return Remember_Speed_Min_Value;

    if (Turn_Distance_Rec.Turn_Count == 0)
        return Remember_Speed_Min_Value;

    if (Remember_Turn_Index >= Turn_Distance_Rec.Turn_Count)
        return Remember_Speed_Min_Value;

    section_total_mileage = Turn_Distance_Rec.Turn_Distance[Remember_Turn_Index];

    if (section_total_mileage <= 0)
        return Remember_Speed_Min_Value;

    effective_total_mileage = section_total_mileage;

    processed_mileage = Total_Run_Mileage - Remember_Section_Start_Mileage;
    if (processed_mileage < 0) processed_mileage = 0;
    if (processed_mileage > effective_total_mileage) processed_mileage = effective_total_mileage;

    // Phase 1: Low [0% ~ 5%]
    if (processed_mileage <= effective_total_mileage * REMEMBER_SPEED_LOW_RATIO)
        return Remember_Speed_Min_Value;

    // Phase 2: Ramp [5% ~ 10%]
    if (processed_mileage <= effective_total_mileage * REMEMBER_SPEED_RAMP_RATIO)
    {
        float ratio = (processed_mileage - effective_total_mileage * REMEMBER_SPEED_LOW_RATIO)
                      / (effective_total_mileage * (REMEMBER_SPEED_RAMP_RATIO - REMEMBER_SPEED_LOW_RATIO));
        return Remember_Speed_Min_Value + (int)((Remember_Speed_Max_Value - Remember_Speed_Min_Value) * ratio);
    }

    // Phase 3: Cruise [10% ~ 95%]
    if (processed_mileage < effective_total_mileage * REMEMBER_SPEED_DECEL_RATIO)
        return Remember_Speed_Max_Value;

    // Phase 4: Decel [95% ~ 100%]
    {
        float ratio = (effective_total_mileage - processed_mileage)
                      / (effective_total_mileage * (1.0f - REMEMBER_SPEED_DECEL_RATIO));
        return Remember_Speed_Min_Value + (int)((Remember_Speed_Max_Value - Remember_Speed_Min_Value) * ratio);
    }
}

/*************************************
** Function: Remember_Set_Speed
*************************************/
void Remember_Set_Speed(void)
{
    static uint8_t straight_enter = 0;

    Run_Speed = Remember_Get_Run_Speed();
    Turn_PID.kp = (float)Run_Speed / 160.0f * REMEMBER_TURN_KP_AT_160;

    if (is_left == 1 || is_right == 1)
    {
        straight_enter = 0;
        // Right-angle turn: PID diff turning
        int base_spd = Get_Turn_Base_Speed();
        Gyro_PID_Out = PID_calc(&Gyro_PID, 0.0f, Gyro_Z);
        if (is_left == 1)
        {
            Left_Exp_Spd  = base_spd + (int)(Gyro_PID_Out * REMEMBER_TURN_INNER_SCALE);
            Right_Exp_Spd = base_spd - (int)(Gyro_PID_Out * REMEMBER_TURN_OUTER_SCALE);
        }
        else
        {
            Left_Exp_Spd  = base_spd - (int)(Gyro_PID_Out * REMEMBER_TURN_OUTER_SCALE);
            Right_Exp_Spd = base_spd + (int)(Gyro_PID_Out * REMEMBER_TURN_INNER_SCALE);
        }
    }
    else if (Run_Mode == Straight_Drive)
    {
        straight_enter = 1;
        if (straight_drive_heading_locked && fabsf(Gyro_Integral) < 1.5f)
        {
            float heading_err = Straight_Heading_Target - Gyro_Integral;
            float heading_err_d = heading_err - Straight_Last_Heading_Err;
            Straight_Last_Heading_Err = heading_err;
            Error = (int)(heading_err * STRAIGHT_HEADING_KP + heading_err_d * STRAIGHT_HEADING_KD);
        }
        else if (straight_drive_heading_frames == 0)
        {
            Error = Current_Action.fixed_error;
        }
        else
        {
            Error = 0;
        }

        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, (float)Error);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        Left_Exp_Spd  = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }
    else
    {
        straight_enter = 0;
        // Normal trace
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, (float)Error);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        Left_Exp_Spd = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }
}

/*************************************
** Function: Straight_Drive_Run
*************************************/
static void Straight_Drive_Run(void)
{
    if (straight_drive_heading_locked)
    {
        if (fabsf(Gyro_Integral) >= 1.5f)
            Error = 0;
        else
        {
            float heading_err = Straight_Heading_Target - Gyro_Integral;
            float heading_err_d = heading_err - Straight_Last_Heading_Err;
            Straight_Last_Heading_Err = heading_err;
            Error = (int)(heading_err * STRAIGHT_HEADING_KP + heading_err_d * STRAIGHT_HEADING_KD);
        }
    }
    else if (straight_drive_heading_frames == 0)
    {
        Error = Current_Action.fixed_error;
    }
    else
    {
        Error = 0;
    }

    gyro_buf[heading_buf_idx] = Gyro_Z;
    heading_buf_idx = (heading_buf_idx + 1) % HEADING_BUF_SIZE;
    if (heading_buf_idx == 0) heading_buf_full = 1;

    if (Count.Mileage - Straight_Drive_Base >= Straight_Drive_Target)
    {
        Run_Mode = Normal_Mode;
        memset(gyro_buf, 0, sizeof(gyro_buf));
        heading_buf_idx = 0;
        heading_buf_full = 0;
    }
}

/*************************************
** Function: Remember_Mode_Get_Error
** Description: Remember mode main dispatcher — called from Car_Go() every 3ms
*************************************/
void Remember_Mode_Get_Error(void)
{
    if (Remember_First_Mode == 0)
    {
        Remember_First_Mode = 1;
        memcpy(&Simple_Track, &Default_Simple_Track, sizeof(Simple_Track));
        Remember_Reset_Runtime_State();
    }

    Light_Process();

    if (Run_Mode == Normal_Mode)
    {
        Remember_Normal_Run();
        Remember_Check_Trigger();
    }

    switch (Run_Mode)
    {
        case Turn_Left:
        case Turn_Right:
            // PID diff right-angle turn — complete when gyro integral reaches target
            if (fabsf(Gyro_Integral) >= REMEMBER_TURN_TARGET_ANGLE_DEG)
            {
                // Section boundary: speed curve advances to next segment
                Remember_Section_Start_Mileage = Total_Run_Mileage;
                Remember_Turn_Index++;
                is_left = 0;
                is_right = 0;
                Gyro_Integral = 0;
                Turn_Action_Done = 1;
                Error = 0;
                PID_cleardata(&Gyro_PID);
                PID_cleardata(&Turn_PID);
                Run_Mode = Normal_Mode;
            }
            break;

        case Straight_Drive:
            Straight_Drive_Run();
            break;

        default:
            break;
    }
}

#endif // ACTIVE_MODE == MODE_REMEMBER
