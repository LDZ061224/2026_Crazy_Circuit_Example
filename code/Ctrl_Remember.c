/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Remember.c
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Remember (replay) mode — adapted from CarbonV2.0
             Mileage-based turn triggering, trapezoidal speed curve,
             curve turns with odometry compensation, Snap correction.
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

// Curve turn state
float   curve_odom_x = 0;
float   curve_odom_y = 0;
float   curve_last_odom_y = 0;
float   curve_current_advance = 0;
uint8_t curve_variable_diff = 0;
int     curve_diff = 0;
uint8_t curve_direction = 0;    // 0=left, 1=right

// Pending curve entry via straight drive
uint8_t elem_approach_pending = 0;  // 0=none, 1=left, 2=right

// VOFA debug
float Remember_Trigger_Value = 0;
float Remember_Section_Total = 0;

// Remember mode parameters
int Remember_Turn_Error = REMEMBER_TURN_ERR;
int Remember_Speed_Min_Value = REMEMBER_SPEED_MIN;
int Remember_Speed_Max_Value = REMEMBER_SPEED_MAX;

// Remember mode runtime state
static uint8_t Remember_First_Mode = 0;
static uint8_t Straight_Drive_Base = 0;
static float   Straight_Drive_Target = 0;

// Heading lock state for straight drive
static int     straight_drive_heading_frames = 0;
static uint8_t straight_drive_heading_locked = 0;

/********************************* Static Function Declarations *********************************/

static void Remember_Normal_Run(void);
static void Remember_Check_Trigger(void);
static int  Remember_Get_Run_Speed(void);
static void Remember_Reset_Runtime_State(void);
static void Enter_Curve_Turn(uint8_t is_left_turn, float advance_raw);
static void Curve_Turn_Run(void);
static void Straight_Drive_Run(void);
static void Dispatch_Action(uint8_t edge_index);
static int  Get_Curve_Diff(float advance_raw);
static int  Get_Turn_Base_Speed(void);

/********************************* Static Helper Functions *********************************/

/*************************************
** Function: Get_Curve_Diff
** Description: Lookup table mapping curve advance to wheel differential
*************************************/
static int Get_Curve_Diff(float advance_raw)
{
    float adv = (advance_raw < 0) ? -advance_raw : advance_raw;
    if (adv < 100) return CURVE_DIFF_R4000;       // default / ACTION_NONE
    if (adv <= 3600.0f) return CURVE_DIFF_R3600;
    if (adv <= 4000.0f) return CURVE_DIFF_R4000;
    if (adv <= 4400.0f) return CURVE_DIFF_R4400;
    if (adv <= 4800.0f) return CURVE_DIFF_R4800;
    if (adv <= 5200.0f) return CURVE_DIFF_R5200;
    return CURVE_DIFF_R5600;
}

/*************************************
** Function: Get_Turn_Base_Speed
** Description: Get base speed for Remember turn
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
** Description: Dispatch an action from Simple_Track to Current_Action + set Run_Mode
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
        for (uint8_t i = 1; i < HEADING_BUF_SIZE; i++)  // skip newest frame (i=0)
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

    // ─── Turn actions ───
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

        // Regular right-angle turn (非曲线转弯)
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
    // ─── Straight actions ───
    else
    {
        Straight_Drive_Base = Count.Mileage;
        Straight_Heading_Target = 0;
        Straight_Heading_Locked = 0;
        straight_drive_heading_locked = 0;

        switch (action)
        {
            case ACTION_STRAIGHT_SHORT:
                Straight_Drive_Target = STRAIGHT_SHORT_MILEAGE;
                break;
            case ACTION_STRAIGHT_MID:
                Straight_Drive_Target = STRAIGHT_MID_MILEAGE;
                break;
            case ACTION_STRAIGHT_LONG:
                Straight_Drive_Target = STRAIGHT_LONG_MILEAGE;
                break;
            case ACTION_NONE:
                Straight_Drive_Target = STRAIGHT_NODE_MILEAGE;
                break;
            default:
                Straight_Drive_Target = STRAIGHT_NODE_MILEAGE;
                break;
        }

        straight_drive_heading_frames = entry.heading_frames;

        // Heading lock init
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
** Description: Reset all runtime state for Remember mode startup
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
    curve_odom_x = 0;
    curve_odom_y = 0;
    curve_last_odom_y = 0;
    curve_current_advance = 0;
    curve_variable_diff = 0;
    curve_diff = 0;
    curve_direction = 0;
    elem_approach_pending = 0;

    PID_cleardata(&Angle_PID);
    PID_cleardata(&Gyro_PID);
    PID_cleardata(&Gyro_PD_PID);
    PID_cleardata(&Turn_PID);
    PID_cleardata(&Left_PID);
    PID_cleardata(&Right_PID);
}

/*************************************
** Function: Remember_Normal_Run
** Description: Remember mode line following — compute Error, update gyro_buf
**              Does NOT trigger Check_Edge or dispatch actions.
*************************************/
static void Remember_Normal_Run(void)
{
    Last_Error = Error;

    if (Track_Num < 1)
    {
        Error = Last_Error;
    }
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

    // Update gyro ring buffer every frame
    gyro_buf[heading_buf_idx] = Gyro_Z;
    heading_buf_idx = (heading_buf_idx + 1) % HEADING_BUF_SIZE;
    if (heading_buf_idx == 0) heading_buf_full = 1;
}

/*************************************
** Function: Remember_Check_Trigger
** Description: Remember mode trigger detection
**              - Turn actions: mileage-based trigger (Turn_Distance_Rec)
**              - Non-turn actions: physical Check_Edge trigger + Snap correction
*************************************/
static void Remember_Check_Trigger(void)
{
    if (Run_Mode != Normal_Mode) return;
    if (Edge_Index >= Simple_Track.Action_Count) return;

    Action_Enum next_action = Simple_Track.Action_List[Edge_Index].action;

    // ===== Turn actions: mileage-based trigger =====
    if (next_action == ACTION_TURN_LEFT  || next_action == ACTION_TURN_RIGHT ||
        next_action == ACTION_ELEM_LEFT || next_action == ACTION_ELEM_RIGHT ||
        next_action == ACTION_ELEM_LEFT_BIG || next_action == ACTION_ELEM_RIGHT_BIG)
    {
        if (Remember_Turn_Index < Turn_Distance_Rec.Turn_Count)
        {
            float traveled = Total_Run_Mileage - Remember_Section_Start_Mileage;
            float total    = Turn_Distance_Rec.Turn_Distance[Remember_Turn_Index];
            float adv_raw  = Simple_Track.Action_List[Edge_Index].curve_advance;
            float adv      = (adv_raw < 0) ? -adv_raw : adv_raw;
            if (adv == 0) adv = REMEMBER_CURVE_ADVANCE;
            uint8_t is_left_turn = (next_action == ACTION_TURN_LEFT ||
                                     next_action == ACTION_ELEM_LEFT ||
                                     next_action == ACTION_ELEM_LEFT_BIG) ? 1 : 0;

            float trigger = total - (Remember_Turn_Index == 0
                ? adv
                : curve_last_odom_y + adv);

            Remember_Trigger_Value = trigger;
            Remember_Section_Total = total;

            if (traveled >= trigger)
            {
                // Last turn → stop
                if (Edge_Index >= Simple_Track.Action_Count - 1)
                {
                    Finish_Flag = 1;
                }
                else
                {
                    Enter_Curve_Turn(is_left_turn, adv_raw);
                    Edge_Index++;
                }
            }
            // Mileage not reached but physical edge detected → straight drive remaining, then curve
            else if (Check_Edge())
            {
                float remaining = trigger - traveled;
                Straight_Drive_Base = Count.Mileage;
                Straight_Drive_Target = remaining;
                straight_drive_heading_locked = 0;
                elem_approach_pending = is_left_turn ? 1 : 2;
                Run_Mode = Straight_Drive;
            }
        }
        return;
    }

    // ===== Non-turn actions: physical Check_Edge trigger + Snap correction =====
    if (Check_Edge())
    {
        Edge_Index++;
        // Last Check_Edge → stop
        if (Edge_Index >= Simple_Track.Action_Count)
        {
            Finish_Flag = 1;
        }
        else
        {
            // Snap: ±2 window search for closest recorded mileage, correct index + mileage
            if (Edge_Mileage_Rec.Edge_Count > 0)
            {
                float best_diff = 999999.0f;
                uint8_t best_idx = Edge_Index;
                int16_t start = (int16_t)Edge_Index - 1;
                if (start < 0) start = 0;
                int16_t end = (int16_t)Edge_Index + 1;
                if (end >= Edge_Mileage_Rec.Edge_Count) end = Edge_Mileage_Rec.Edge_Count - 1;
                for (int16_t i = start; i <= end; i++)
                {
                    float diff = fabsf(Total_Run_Mileage - Edge_Mileage_Rec.Edge_Mileage[i]);
                    if (diff < best_diff) { best_diff = diff; best_idx = (uint8_t)i; }
                }
                Total_Run_Mileage = Edge_Mileage_Rec.Edge_Mileage[best_idx];
                Edge_Index = best_idx;
            }
            Dispatch_Action(Edge_Index - 1);
        }
    }
}

/*************************************
** Function: Remember_Get_Run_Speed
** Description: Trapezoidal speed curve based on Turn_Distance_Rec
**              4-phase: low[0-5%] → ramp[5-10%] → cruise[10-80%] → decel[80-100%]
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

    if (section_total_mileage <= REMEMBER_CURVE_ADVANCE)
        return Remember_Speed_Min_Value;

    effective_total_mileage = section_total_mileage - REMEMBER_CURVE_ADVANCE;

    processed_mileage = Total_Run_Mileage - Remember_Section_Start_Mileage;
    if (processed_mileage < 0) processed_mileage = 0;
    if (processed_mileage > effective_total_mileage) processed_mileage = effective_total_mileage;

    // Phase 1: Low speed [0% ~ 5%]
    if (processed_mileage <= effective_total_mileage * REMEMBER_SPEED_LOW_RATIO)
        return Remember_Speed_Min_Value;

    // Phase 2: Ramp up [5% ~ 10%]
    if (processed_mileage <= effective_total_mileage * REMEMBER_SPEED_RAMP_RATIO)
    {
        float ratio = (processed_mileage - effective_total_mileage * REMEMBER_SPEED_LOW_RATIO)
                      / (effective_total_mileage * (REMEMBER_SPEED_RAMP_RATIO - REMEMBER_SPEED_LOW_RATIO));
        return Remember_Speed_Min_Value + (int)((Remember_Speed_Max_Value - Remember_Speed_Min_Value) * ratio);
    }

    // Phase 3: Cruise [10% ~ 80%]
    if (processed_mileage < effective_total_mileage * REMEMBER_SPEED_DECEL_RATIO)
        return Remember_Speed_Max_Value;

    // Phase 4: Decel [80% ~ 100%]
    {
        float ratio = (effective_total_mileage - processed_mileage)
                      / (effective_total_mileage * (1.0f - REMEMBER_SPEED_DECEL_RATIO));
        return Remember_Speed_Min_Value + (int)((Remember_Speed_Max_Value - Remember_Speed_Min_Value) * ratio);
    }
}

/*************************************
** Function: Remember_Set_Speed
** Description: Remember mode speed control — trapezoidal curve + turning speed override
**              Called from Set_Speed() when ACTIVE_MODE == MODE_REMEMBER
*************************************/
void Remember_Set_Speed(void)
{
    static uint8_t straight_enter = 0;

    // Turn PID proportional scaling
    Run_Speed = Remember_Get_Run_Speed();
    Turn_PID.kp = (float)Run_Speed / 160.0f * REMEMBER_TURN_KP_AT_160;

    if (is_left == 1 || is_right == 1)
    {
        straight_enter = 0;
        // Right-angle turn: PID diff turning with inner/outer scale
        Gyro_PID_Out = PID_calc(&Gyro_PID, 0.0f, Gyro_Z);
        int base_spd = Get_Turn_Base_Speed();
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
        // Heading locked: PD control
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

        // Normal trace PID cascade
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
** Function: Enter_Curve_Turn
** Description: Initialize curve turn for Remember mode
*************************************/
static void Enter_Curve_Turn(uint8_t is_left_turn, float advance_raw)
{
    float adv = (advance_raw < 0) ? -advance_raw : advance_raw;
    if (adv < 100) adv = REMEMBER_CURVE_ADVANCE;

    curve_direction = is_left_turn ? 0 : 1;
    curve_current_advance = adv;
    curve_variable_diff = (advance_raw < 0) ? 1 : 0;

    if (curve_variable_diff)
        curve_diff = CURVE_VARIABLE_DIFF_START;
    else
        curve_diff = Get_Curve_Diff(advance_raw);

    Gyro_Integral = 0;
    is_left = is_left_turn;
    is_right = !is_left_turn;
    memset(gyro_buf, 0, sizeof(gyro_buf));
    heading_buf_idx = 0;
    heading_buf_full = 0;

    Run_Mode = Curve_Turn;
}

/*************************************
** Function: Curve_Turn_Run
** Description: Execute curve turn (fixed diff + variable diff modes)
*************************************/
static void Curve_Turn_Run(void)
{
    // Variable diff: linearly decrease diff as angle approaches target
    if (curve_variable_diff)
    {
        float angle = fabsf(Gyro_Integral);
        if (angle >= CURVE_VARIABLE_DIFF_KNEE && angle < REMEMBER_CURVE_TARGET_DEG)
        {
            float ratio = (REMEMBER_CURVE_TARGET_DEG - angle) / (REMEMBER_CURVE_TARGET_DEG - CURVE_VARIABLE_DIFF_KNEE);
            curve_diff = CURVE_VARIABLE_DIFF_END + (int)((CURVE_VARIABLE_DIFF_START - CURVE_VARIABLE_DIFF_END) * ratio);
        }
        else if (angle >= REMEMBER_CURVE_TARGET_DEG)
        {
            curve_diff = CURVE_VARIABLE_DIFF_END;
        }
    }

    Error = 0;
    if (curve_direction == 0)  // left
    {
        Left_Exp_Spd  = CURVE_BASE_SPEED - curve_diff;
        Right_Exp_Spd = CURVE_BASE_SPEED + curve_diff;
    }
    else  // right
    {
        Left_Exp_Spd  = CURVE_BASE_SPEED + curve_diff;
        Right_Exp_Spd = CURVE_BASE_SPEED - curve_diff;
    }

    // Check completion
    if (fabsf(Gyro_Integral) >= REMEMBER_CURVE_TARGET_DEG)
    {
        Count.Mileage = curve_current_advance;
        Total_Run_Mileage += curve_odom_x + curve_odom_y;
        curve_last_odom_y = curve_odom_y;
        Remember_Section_Start_Mileage = Total_Run_Mileage;
        Remember_Turn_Index++;

        // Reset curve state
        curve_odom_x = 0;
        curve_odom_y = 0;
        curve_variable_diff = 0;
        curve_diff = 0;
        is_left = 0;
        is_right = 0;
        Run_Mode = Normal_Mode;
    }
}

/*************************************
** Function: Straight_Drive_Run
** Description: Execute straight drive with heading lock or fixed diff
*************************************/
static void Straight_Drive_Run(void)
{
    // Heading-locked mode
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

    // Update gyro_buf during straight drive
    gyro_buf[heading_buf_idx] = Gyro_Z;
    heading_buf_idx = (heading_buf_idx + 1) % HEADING_BUF_SIZE;
    if (heading_buf_idx == 0) heading_buf_full = 1;

    // Check mileage completion
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
    // First-call initialization
    if (Remember_First_Mode == 0)
    {
        Remember_First_Mode = 1;
        memcpy(&Simple_Track, &Default_Simple_Track, sizeof(Simple_Track));
        Remember_Reset_Runtime_State();
    }

    // Sensor processing (shared with Build mode)
    Light_Process();

    // Normal mode: compute Error + check triggers
    if (Run_Mode == Normal_Mode)
    {
        Remember_Normal_Run();
        Remember_Check_Trigger();
    }

    // State machine dispatch
    switch (Run_Mode)
    {
        case Turn_Left:
        case Turn_Right:
            // Right-angle turns: PID diff turning (speed set in Remember_Set_Speed)
            // Check if gyro integral reached target
            if (fabsf(Gyro_Integral) >= REMEMBER_TURN_TARGET_ANGLE_DEG)
            {
                // Complete turn: reset state
                Remember_Section_Start_Mileage = Total_Run_Mileage;
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

        case Curve_Turn:
            Curve_Turn_Run();
            break;

        default:
            break;
    }

    // elem_approach_pending: straight drive completed → enter curve turn
    if (elem_approach_pending && Run_Mode == Normal_Mode)
    {
        uint8_t is_left_turn = (elem_approach_pending == 1) ? 1 : 0;
        elem_approach_pending = 0;
        float adv = Simple_Track.Action_List[Edge_Index].curve_advance;
        Enter_Curve_Turn(is_left_turn, adv);
    }
}

#endif // ACTIVE_MODE == MODE_REMEMBER
