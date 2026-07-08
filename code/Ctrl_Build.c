/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl_Build.c
Author: Cross_Z
Version:4.0               Date: 2026.7.4
Description: Build mode — all Build-specific functions extracted from Ctrl.c
             Only compiled when ACTIVE_MODE == MODE_BUILD
             Global variables remain in Ctrl.c (unconditional).
**************************************************/

#include "headfiles.h"
#include "Ctrl_Build.h"
#include "OLEDKeyboard.h"

#if ACTIVE_MODE == MODE_BUILD

/*--------------- Build Mode Static State ---------------*/
static uint8_t Straight_Node_Pending = 0;
static uint8_t Turn_Angle_Settle_Count = 0;
static uint8_t Build_Action_Active_Index = 0;

/*--------------- Flash Storage Layout ---------------*/
#define SEGMENT_EDGE_MILEAGE_FLASH_SECTOR 0
#define SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE 8
#define SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT 2
#define SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE 64

typedef struct
{
    float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];
    float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];
} Segment_Edge_Mileage_Flash_Typedef;

/*--------------- Static Function Declarations ---------------*/
static void Build_Load_Default_Action_List(void);
static void Build_Dispatch_Current_Action(void);
static void Build_Finish_Current_Action(void);
static int  Select_Run_Speed(void);
static void Set_Node_Run_Mode(uint8_t node_dir, float turn_delay);
static void Reset_Turn_Action_State(void);
static void Complete_Turn_Action(void);
static void Finish_Mileage_Section(void);
static void Save_Segment_Edge_Mileage_Record_To_Flash(void);
static void Load_Segment_Edge_Mileage_Record_From_Flash(void);
// (Save_Flash_Page_Block / Load_Flash_Page_Block shared from Ctrl_Flash.c)

/********************************* Static Helper Functions *********************************/

// Load the pre-defined default build action list (flat enum array copy)
static void Build_Load_Default_Action_List(void)
{
    Build_Action_Count = BUILD_ACTION_COUNT;
    Build_Action_Index = 0;
    Build_Action_Active_Index = 0;
    memcpy(Build_Action_List, Default_Build_Actions, sizeof(Default_Build_Actions));
}

// Mark the current action as finished — just check finish condition
static void Build_Finish_Current_Action(void)
{
    if (Build_Action_Index >= Build_Action_Count)
    {
        Finish_Flag = 1;
    }
}

// Dispatch next build action — 3 behavior groups (straight / left turn / right turn)
static void Build_Dispatch_Current_Action(void)
{
    uint8_t action;

    if (Build_Action_Index >= Build_Action_Count)
    {
        Finish_Flag = 1;
        return;
    }

    Build_Action_Active_Index = Build_Action_Index;
    action = Build_Action_List[Build_Action_Index];

    switch (action)
    {
        // ─── Straight (node + element short + element long) ───
        case BUILD_ACTION_NODE_STRAIGHT:
            Straight_Node_Pending = 1;
            Count.Straight = TUNE_NODE_STRAIGHT;
            Segment_Edge_Mileage_Record[Count.Line][Count.Element++] = Count.Last_Edge_Mileage;
            goto do_straight;
        case BUILD_ACTION_ELEM_STRAIGHT_SHORT:
            Count.Straight = TUNE_ELEM_STRAIGHT_SHORT;
            Segment_Edge_Mileage_Record[Count.Line][Count.Element++] = Count.Last_Edge_Mileage;
            goto do_straight;
        case BUILD_ACTION_ELEM_STRAIGHT_LONG:
            Count.Straight = TUNE_ELEM_STRAIGHT_LONG;
            Segment_Edge_Mileage_Record[Count.Line][Count.Element++] = Count.Last_Edge_Mileage;
        do_straight:
            Count.StraightBase = Count.Mileage;
            Run_Mode = Straight_Mode;
            break;

        // ─── Left turn (node + element) ───
        case BUILD_ACTION_NODE_TURN_LEFT:
            Count.is_elem_turn = 0;
            Count.Element = 0;
            goto do_left;
        case BUILD_ACTION_ELEM_TURN_LEFT:
            Count.is_elem_turn = 1;
            Count.Element = 0;
        do_left:
            Count.Line++;
            Set_Node_Run_Mode(1, (action == BUILD_ACTION_NODE_TURN_LEFT) ? TUNE_NODE_TURN_DELAY : TUNE_ELEM_TURN_DELAY);
            break;

        // ─── Right turn (node + element) ───
        case BUILD_ACTION_NODE_TURN_RIGHT:
            Count.is_elem_turn = 0;
            Count.Element = 0;
            goto do_right;
        case BUILD_ACTION_ELEM_TURN_RIGHT:
            Count.is_elem_turn = 1;
            Count.Element = 0;
        do_right:
            Count.Line++;
            Set_Node_Run_Mode(2, (action == BUILD_ACTION_NODE_TURN_RIGHT) ? TUNE_NODE_TURN_DELAY : TUNE_ELEM_TURN_DELAY);
            break;

        default:
            return;
    }
    Build_Action_Index++;
}

// Select the current run speed from the configured Basic_Speed
static int Select_Run_Speed(void)
{
    if (Basic_Speed < 0)
        return 0;
    return Basic_Speed;
}

/*************************************
** Function: Reset_Turn_Action_State
*************************************/
static void Reset_Turn_Action_State(void)
{
    Gyro_Integral = 0;
    Turn_Angle_Settle_Count = 0;
    Error = 0;
    Turn_PID_Out = 0;
    PID_cleardata(&Gyro_PID);
    PID_cleardata(&Gyro_PD_PID);
    PID_cleardata(&Turn_PID);
    Gyro_PID_Out = 0;
    Left_Exp_Spd = 0;
    Right_Exp_Spd = 0;
    Count.Left = 0;
    Count.Right = 0;
    Count.Mileage = 0;
    is_left = 0;
    is_right = 0;
    Turn_Decel_Phase = 0;
    Check_Edge_Skip_Mileage_Base = Count.Mileage;
    Check_Edge_Skip_Thresh = Count.is_elem_turn ? TUNE_COOLDOWN_ELEM_TURN : TUNE_COOLDOWN_NODE_TURN;
    Run_Mode = Normal_Mode;
}

/*************************************
** Function: Complete_Turn_Action
*************************************/
static void Complete_Turn_Action(void)
{
    Turn_Action_Done = 1;
    Segment_Total_Mileage[Count.Line] = Count.Mileage_Phase0;
    Total_Angle = 0;
    Build_Finish_Current_Action();
    Reset_Turn_Action_State();
}

// (Is_Turn_Angle_Settled removed — Phase 1 now uses direct Gyro_Integral >= ROTATE_TARGET_DEG check)

/*************************************
** Function: Set_Node_Run_Mode
*************************************/
static void Set_Node_Run_Mode(uint8_t node_dir, float turn_delay)
{
    Left_Exp_Spd = 0;
    Right_Exp_Spd = 0;
    Gyro_Integral = 0;
    Turn_Angle_Settle_Count = 0;
    PID_cleardata(&Turn_PID);
    PID_cleardata(&Gyro_PID);
    Turn_Action_Done = 0;
    Turn_Decel_Phase = 0;
    Count.Left = Count.Mileage;
    Count.Stall = turn_delay;

    switch (node_dir)
    {
        case 1:
            is_left = 1;
            is_right = 0;
            Turn_Angle_Target = -90.0f;
            Run_Mode = Turn_Left;
            break;
        case 2:
            is_left = 0;
            is_right = 1;
            Turn_Angle_Target = 90.0f;
            Run_Mode = Turn_Right;
            break;
    }
}

/*************************************
** Function: Finish_Mileage_Section
*************************************/
static void Finish_Mileage_Section(void)
{
    is_left = 0;
    is_right = 0;
    Mileage_Turn_Done = 0;
    Check_Edge_Skip_Mileage_Base = Count.Mileage;
    Check_Edge_Skip_Thresh = TUNE_COOLDOWN_STRAIGHT;
    Run_Mode = Normal_Mode;
    Build_Finish_Current_Action();
}

/*--------------- Flash Save/Load (currently stubbed) ---------------*/

static void Save_Segment_Edge_Mileage_Record_To_Flash(void)
{
    Segment_Edge_Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT * SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};
    memcpy(flash_log.Segment_Edge_Mileage_Record, Segment_Edge_Mileage_Record, sizeof(Segment_Edge_Mileage_Record));
    memcpy(flash_log.Segment_Total_Mileage, Segment_Total_Mileage, sizeof(Segment_Total_Mileage));
    memcpy(map_words, &flash_log, sizeof(flash_log));
    Save_Flash_Page_Block(SEGMENT_EDGE_MILEAGE_FLASH_SECTOR, SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE,
                          SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT, SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE, map_words);
}

static void Load_Segment_Edge_Mileage_Record_From_Flash(void)
{
    Segment_Edge_Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT * SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};
    Load_Flash_Page_Block(SEGMENT_EDGE_MILEAGE_FLASH_SECTOR, SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE,
                          SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT, SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE, map_words);
    memcpy(&flash_log, map_words, sizeof(flash_log));
    memcpy(Segment_Edge_Mileage_Record, flash_log.Segment_Edge_Mileage_Record, sizeof(Segment_Edge_Mileage_Record));
    memcpy(Segment_Total_Mileage, flash_log.Segment_Total_Mileage, sizeof(Segment_Total_Mileage));
}

// (Save_Flash_Page_Block / Load_Flash_Page_Block shared from Ctrl_Flash.c)

/*************************************
** Function: Load_All_Flash_Data_For_VOFA
*************************************/
void Load_All_Flash_Data_For_VOFA(void)
{
    // Load_Segment_Edge_Mileage_Record_From_Flash(); (FIXME: Flash format changed)
}

/********************************* Core Run-Time Functions *********************************/

/*************************************
** Function: Build_Mode_Get_Error
*************************************/
void Build_Mode_Get_Error(void)
{
    if (First_Mode == 0)
    {
        Run_Mode = Normal_Mode;
        First_Mode = 1;
        Count.Line = 0;
        Count.Element = 0;
        Build_Action_Index = 0;
        Build_Action_Active_Index = 0;
        Build_Load_Default_Action_List();
        memset(Segment_Edge_Mileage_Record, 0, sizeof(Segment_Edge_Mileage_Record));
    }

    switch (Run_Mode)
    {
        case Normal_Mode:   Normal_Run();       break;
        case Turn_Left:     Turn_Left_Run();    break;
        case Turn_Right:    Turn_Right_Run();   break;
        case Straight_Mode: Straight_Run();     break;
        default:            Normal_Run();       break;
    }

    if (g_led_flag != 2)
        g_led_flag = (Run_Mode == Normal_Mode) ? 0 : 1;
}

/*************************************
** Function: Normal_Run
*************************************/
void Normal_Run(void)
{
    if (Track_Num > 0)
    {
        Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
        Last_Error = Error;
    }

    if (Track_Num < 2)
        Error = 0;
    else
    {
        Left_Scan_Point = Track_Arr[0];
        Right_Scan_Point = Track_Arr[Track_Num - 1];
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
    }

    if (Check_Edge())
        Build_Dispatch_Current_Action();
}

/*************************************
** Function: Turn_Left_Run — 2-phase turn
*************************************/
void Turn_Left_Run(void)
{
    if (Turn_Action_Done) return;

    if (Turn_Decel_Phase == 0)
    {
        float traveled = Count.Mileage - Count.Left;
        float ratio = 1.0f - (traveled / Count.Stall);
        if (ratio < 0.0f) ratio = 0.0f;
        int speed = (int)(Basic_Speed * 1.0);
        Error = 0;
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, Total_Angle);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        Left_Exp_Spd = speed + (int)Gyro_PID_Out;
        Right_Exp_Spd = speed - (int)Gyro_PID_Out;

        if (traveled >= Count.Stall)
        {
            Turn_Decel_Phase = 1;
            Count.Mileage_Phase0 = Count.Mileage;
            Gyro_Integral = 0;
            Turn_Angle_Settle_Count = 0;
            PID_cleardata(&Turn_PID);
            PID_cleardata(&Gyro_PID);
        }
    }
    else
    {
        // Phase 1: fixed diff + gyro PD damping handled in Set_Speed()
        Error = 0;
        if (fabsf(Gyro_Integral) >= ROTATE_TARGET_DEG)
            Complete_Turn_Action();
    }
}

/*************************************
** Function: Turn_Right_Run — mirror of Turn_Left_Run
*************************************/
void Turn_Right_Run(void)
{
    if (Turn_Action_Done) return;

    if (Turn_Decel_Phase == 0)
    {
        float traveled = Count.Mileage - Count.Left;
        float ratio = 1.0f - (traveled / Count.Stall);
        if (ratio < 0.0f) ratio = 0.0f;
        int speed = (int)(Basic_Speed * 1.0);
        Error = 0;
        float delta = -Total_Angle;
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, delta);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        Left_Exp_Spd = speed + (int)Gyro_PID_Out;
        Right_Exp_Spd = speed - (int)Gyro_PID_Out;

        if (traveled >= Count.Stall)
        {
            Turn_Decel_Phase = 1;
            Count.Mileage_Phase0 = Count.Mileage;
            Gyro_Integral = 0;
            Turn_Angle_Settle_Count = 0;
            PID_cleardata(&Turn_PID);
            PID_cleardata(&Gyro_PID);
        }
    }
    else
    {
        // Phase 1: fixed diff + gyro PD damping handled in Set_Speed()
        Error = 0;
        if (fabsf(Gyro_Integral) >= ROTATE_TARGET_DEG)
            Complete_Turn_Action();
    }
}

// (Set_Mileage_Turn_Exp_Speed removed — Phase 1 now uses fixed diff in Set_Speed)

/*************************************
** Function: Straight_Run
*************************************/
void Straight_Run(void)
{
    float section_mileage = Count.Mileage - Count.StraightBase;
    Error = 0;
    if (section_mileage >= Count.Straight)
    {
        if (Straight_Node_Pending)
        {
            Finish_Mileage_Section();
            Straight_Node_Pending = 0;
        }
        else
        {
            Finish_Mileage_Section();
        }
    }
}

#endif // ACTIVE_MODE == MODE_BUILD
