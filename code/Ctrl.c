/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.c
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description: Main control logic for the smart car
- Track following and element recognition
- PID control for speed, steering, and angle
- Build mode (Build_Mode) and Flash parameter storage
Others:      Car_Go() is called every 3ms main loop tick
Function List:
Car_Go / Get_Speed / Get_IMU / Light_Process / Set_Speed / Set_Out
Normal_Run / Straight_Run / Turn_Left_Run / Turn_Right_Run
Mileage_Mode_Run / Mileage_Run_Stage_2
Build_Mode_Get_Error
Save_Turn_Mileage / Load_Turn_Mileage / Save_Segment_Edge / Load_Segment_Edge
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30    0.0
**************************************************/

#include "Ctrl.h"

#include "Debug_Car.h"
/********************************* Global Variables *********************************/

/*------------------------------*/
// Moving-window speed buffer (weighted IIR: 0.5/0.3/0.2)
int16 giSpeed_Left[3] = {0};
int16 giSpeed_Right[3] = {0};
int Left_Real_Spd = 0;
int Right_Real_Spd = 0;

int Left_Exp_Spd = 0;
int Right_Exp_Spd = 0;
int Basic_Speed = 70;   // TODO: hardcoded, restore flash read after tuning
int Run_Speed = 0;
float Average_Speed = 0;
int Speed_Get_Count = 1;
uint8 First_Mode = 0;

/*--------------- Light Sensor Data ---------------*/
uint8 Light_Convert[15] = {0};
uint8 Last_Light_Convert[15] = {0};

/*----------------- PID Controller Handles ----------------*/
float Gyro_Z = 0;
float Gyro_Z_For_PID = 0;
float gyro_z_offset = 0;   // Gyro Z-axis zero-drift offset, sampled during power-on calibration
PID_HandleTypeDef Gyro_PID = GYRO_PID;     // Gyro rate incremental PID
PID_HandleTypeDef Gyro_PD_PID = GYRO_PD_PID; // Gyro rate position PD for normal trace debug
PID_HandleTypeDef Angle_PID = ANGLE_PID;   // Angle PD with derivative on measurement
PID_HandleTypeDef Left_PID = LEFT_PID;

PID_HandleTypeDef Right_PID = RIGHT_PID;
PID_HandleTypeDef Turn_PID = TURN_PID;

/*--------------- Track Following State ---------------*/
int Left_Scan_Point = 0;
int Right_Scan_Point = 0;
int Error = 0;
int Last_Error = 0;
int Track_Arr[15] = {0};
int Last_Track_Arr[15] = {0};
int Initial_White_Num = 0;
int Track_Num = 0;
int Last_Track_Num = 0;
int Stop_Flag = 0;
int Finish_Count = 0;
int Finish_Flag = 0;
uint8_t Left_Num = 0;
uint8_t Right_Num = 0;
uint8_t Left_Flag = 0;
uint8_t Right_Flag = 0;
uint8_t is_left = 0;
uint8_t is_right = 0;
float  Turn_Angle_Target = 0;
float  Turn_Angle_Last_Real = 0;   // Last angle feedback for angle PD derivative
uint8_t Turn_Angle_D_First = 1;
uint8_t Turn_Decel_Phase = 0;
uint8_t Mileage_Turn_Done = 0;
uint8_t Turn_Action_Done = 0;
float Check_Edge_Skip_Thresh = 0;      // Edge-detect cooldown mileage threshold (encoder ticks)
float Check_Edge_Skip_Mileage_Base = 0; // Count.Mileage snapshot when cooldown started
int Enable_Start_Delay_Count = 0;
uint8_t Last_EnableSwitch_ON = 0;
uint8_t g_led_flag = 0;                        // 0=green(normal) 1=blue(object) 2=yellow(low voltage)
uint8_t g_scan_progress = 0;                  // Scan progress 0-100 (0=not scanning)
int Middle = 0;
float Gyro_Integral = 0;
float Mileage_Element_Base = 0;
float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX] = {{0}};
float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX] = {0};
float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX] = {0};
uint16_t Turn_Mileage_Record_Num = 0;
float Total_Run_Mileage = 0;
float Last_Turn_Mileage_Base = 0;
float Turn_Begin_Mileage = 0;

/*--------------- Direction Offset Table ---------------*/
// 15-element sensor direction weight array (mm-level offset mapping)



int16_t Dir_Arr[15] = {18, 16, 13, 9, 6, 3, 1, 0, -1, -3, -6, -9, -13, -16, -18};
int16 Check_Edge_Count = 0;
uint8_t Force_Straight_Speed = 0;
uint8_t Current_Element_Dir = 0;    // Direction of current element action (1=left, 2=right, 3=short-straight, 4=long-straight)

/*-------------------------------*/
// Single-row 15-sensor tracking: all indexes below participate in Track_Num.
#define TRACK_SENSOR_ACTIVE_NUM 15
static const uint8_t Track_Sensor_Active_Index[TRACK_SENSOR_ACTIVE_NUM] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

/*--------------- Default Build Action Table ---------------*/
// Default build actions (pre-computed from track map: 17 nodes + 14 elements = 31 actions)
const Build_Action_Typedef Default_Build_Actions[BUILD_ACTION_COUNT] =
{
    // seg 0: 2 element short-straights + node straight
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 0, 0},
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 0, 1},
    {BUILD_ACTION_NODE_STRAIGHT,        0, 0},
    // seg 1: no elements + node left
    {BUILD_ACTION_NODE_TURN_LEFT,       1, 0},
    // seg 2: 2 element short-straights + node left
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 2, 0},
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 2, 1},
    {BUILD_ACTION_NODE_TURN_LEFT,       2, 0},
    // seg 3: no elements + node straight
    {BUILD_ACTION_NODE_STRAIGHT,        3, 0},
    // seg 4: 1 element short-straight + node left
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 4, 0},
    {BUILD_ACTION_NODE_TURN_LEFT,       4, 0},
    // seg 5: no elements + node straight
    {BUILD_ACTION_NODE_STRAIGHT,        5, 0},
    // seg 6: 1 element short-straight + node left
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 6, 0},
    {BUILD_ACTION_NODE_TURN_LEFT,       6, 0},
    // seg 7: no elements + node straight
    {BUILD_ACTION_NODE_STRAIGHT,        7, 0},
    // seg 8: no elements + node straight
    {BUILD_ACTION_NODE_STRAIGHT,        8, 0},
    // seg 9: 1 element turn-left + node straight
    {BUILD_ACTION_ELEM_TURN_LEFT,       9, 0},
    {BUILD_ACTION_NODE_STRAIGHT,        9, 0},
    // seg 10: 1 element turn-left + node left
    {BUILD_ACTION_ELEM_TURN_LEFT,      10, 0},
    {BUILD_ACTION_NODE_TURN_LEFT,      10, 0},
    // seg 11: 1 element short-straight + node right
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 11, 0},
    {BUILD_ACTION_NODE_TURN_RIGHT,     11, 0},
    // seg 12: no elements + node right
    {BUILD_ACTION_NODE_TURN_RIGHT,     12, 0},
    // seg 13: 1 element short-straight + node left
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 13, 0},
    {BUILD_ACTION_NODE_TURN_LEFT,      13, 0},
    // seg 14: no elements + node left
    {BUILD_ACTION_NODE_TURN_LEFT,      14, 0},
    // seg 15: 1 element short-straight + node straight
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 15, 0},
    {BUILD_ACTION_NODE_STRAIGHT,       15, 0},
    // seg 16: no elements + node left
    {BUILD_ACTION_NODE_TURN_LEFT,      16, 0},
    // seg 17: 2 element short-straights (last segment, no node)
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 17, 0},
    {BUILD_ACTION_ELEM_STRAIGHT_SHORT, 17, 1},
};

// Per-segment element count (Mileage_Num from original track map)
const uint8 Mileage_Num_By_Segment[BUILD_NODE_NUM + 1] =
    {2,0,2,0,1,0,1,0,0,1,1,1,0,1,0,1,0,2};

int8_t Execute_Times = 0;
int8_t Mileage_Times = 0;
uint8_t Line_Num_Count = 0;
uint8_t In_Line_Ele_Count = 0;
Build_Action_Typedef Build_Action_List[BUILD_ACTION_MAX] = {0};
uint8_t Build_Action_Index = 0;
uint8_t Build_Action_Count = 0;
static uint8_t Build_Action_Active_Index = 0;

/*--------------- PID Output Values ---------------*/
float Turn_PID_Out = 0.0;
float Gyro_PID_Out = 0.0;
float Left_PID_Out = 0.0;
float Right_PID_Out = 0.0;

Run_Mode_Enum Run_Mode = Normal_Mode;
Run_Mode_Enum Last_Run_Mode = Normal_Mode;
Mileage_Stage_Enum Mileage_Stage = Normal_Stage;
Mode_Define Mode = Build_Mode;

/*--------------- Debug Sub-mode State ---------------*/
Debug_Sub_Mode_Enum Debug_Sub_Mode = Debug_Sub_PI_Tuning;
uint8  Debug_Motor_Enable = 0;
uint8  Debug_Which_Wheel = 0;
int    Debug_Target_Speed = 40;
int    Debug_Fan_Duty = 9500;
uint8  Debug_Ground_Dir = 1;
uint8  Debug_Angle_Mode = 2;                         // 1=sine rate, 2=step angle, 3=direct gyro rate
uint8  Debug_Angle_D_First = 0;                      // 0=error D, 1=measurement D
float  Debug_Angle_Vel_Target = 0.0f;
float  Debug_Angle_Vel_Real = 0.0f;


/*--------------- Build Mode Turn Control Parameters ---------------*/



#define BUILD_TURN_TARGET_ANGLE_DEG    90.0f
#define TURN_ANGLE_SETTLE_ERROR_DEG    5.0f
#define TURN_GYRO_SETTLE_RATE_DPS      45.0f
#define TURN_SETTLE_CYCLE_MIN          3

float Mileage_Element_Turn_Delay = 850.0f;          // Element turn pre-straight distance, OLED/Flash adjustable
float Mileage_Node_Turn_Delay = 75.0f;             // Node turn pre-straight distance, OLED/Flash adjustable
uint8 vofa_flash_dump_mode = 0;

// Mileage compensation for turn elements (negative = advance element edge)
#define MILEAGE_COMPENSATION_X (-100.0f)
// Short straight element mileage threshold (encoder ticks)
#define MILEAGE_STRAIGHT_SHORT 2300.0f
// Long straight element mileage threshold (0 = disabled / handled by node)
#define MILEAGE_STRAIGHT_LONG  0.0f


// Edge-detect cooldown distance after action completes (encoder ticks)
// Old cycle counts at ~2000 ticks/sec: 25cyc=150tk, 15cyc=90tk, 100cyc=600tk
#define BUILD_CHECK_EDGE_NODE_TURN_MILEAGE       150.0f
#define BUILD_CHECK_EDGE_MILEAGE_STRAIGHT_MILEAGE 100.0f
#define BUILD_CHECK_EDGE_MILEAGE_TURN_MILEAGE     400.0f

// Max consecutive cycles with all sensors on or all off before emergency stop
#define SAFETY_STOP_CYCLE_MAX         80

#define GYRO_INTEGRATION_PERIOD_S 0.003f
#define NORMAL_GYRO_OUT_STEP_MAX 12.0f   // Normal trace steering slew limit per 3ms tick

/*--------------- Flash Storage Layout ---------------*/
/*
* Flash sector and page layout:
* Sector 0, pages 5~7, each page holds 64 uint32_t words -> Turn_Mileage_Record
* Layout: Turn_Mileage_Record_Num(uint16), Turn_Mileage_Record[120](float)
* Corresponds to BUILD_MAP_FLASH_START_PAGE(4) in OLEDKeyboard.c
 */
#define TURN_MILEAGE_FLASH_SECTOR 0
#define TURN_MILEAGE_FLASH_START_PAGE 5
#define TURN_MILEAGE_FLASH_PAGE_COUNT 3
#define TURN_MILEAGE_FLASH_WORDS_PER_PAGE 64

/*
* Flash sector and page layout:
* Sector 0, pages 8~9, for Segment_Edge_Mileage_Record
* Storage size: NODE_NUM_MAX(20) * ELEMENT_NUM_MAX(5) * sizeof(float)(4) = 400 bytes
 */
#define SEGMENT_EDGE_MILEAGE_FLASH_SECTOR 0
#define SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE 8
#define SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT 2
#define SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE 64

/*--------------- Flash Data Structures ---------------*/

typedef struct
{
    uint16_t Turn_Mileage_Record_Num;
    float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX];
}Turn_Mileage_Flash_Typedef;


typedef struct
{
    float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];
    float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];
} Segment_Edge_Mileage_Flash_Typedef;


Count_Typedef Count =
{
    .Left = 0,
    .Right = 0,
    .Stop = 0,
    .Mileage = 0,
    .Straight = 0,
    .Spd_Mileage = 0,
};

static uint8_t Straight_Node_Pending = 0;
static uint8_t Turn_Angle_Settle_Count = 0;

/*--------------- Static Function Declarations ---------------*/
static void Load_Turn_Mileage_Record_From_Flash(void);
static void Save_Segment_Edge_Mileage_Record_To_Flash(void);
static void Load_Segment_Edge_Mileage_Record_From_Flash(void);
static void Save_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, const uint32_t *words);
static void Load_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, uint32_t *words);
static int Select_Run_Speed(void);
static void Advance_Turn_Section_Index(void);
static void Set_Node_Run_Mode(uint8_t node_dir);
static void Reset_Turn_Action_State(void);
static void Complete_Turn_Action(void);
static void Record_Turn_Mileage(void);
static void Record_Segment_Edge_Mileage(void);
static uint8_t Is_Turn_Angle_Settled(float angle_target);
static uint8_t Is_Track_Sensor_Adjacent(uint8_t left_index, uint8_t right_index);
static void Build_Load_Default_Action_List(void);
static void Build_Dispatch_Current_Action(void);
static void Build_Finish_Current_Action(void);
static uint8_t Build_Action_To_Element_Dir(Build_Action_Enum action);

/********************************* Static Helper Functions *********************************/

/*************************************
** Function: Is_Track_Sensor_Adjacent
** Description: Check if two track sensors are physically adjacent
**              (either consecutive indexes, or the center pair 5 and 9)
** Input:      left_index  - Left-side sensor index
**             right_index - Right-side sensor index
** Return:     1 if adjacent, 0 otherwise
*************************************/
static uint8_t Is_Track_Sensor_Adjacent(uint8_t left_index, uint8_t right_index)
{
    if (right_index == left_index + 1)
    {
        return 1;
    }


    if (left_index == 5 && right_index == 9)
    {
        return 1;
    }

    return 0;
}

// Extract element direction (1~4) from action type. Returns 0 for node actions.
static uint8_t Build_Action_To_Element_Dir(Build_Action_Enum action)
{
    switch (action)
    {
        case BUILD_ACTION_ELEM_TURN_LEFT:     return 1;
        case BUILD_ACTION_ELEM_TURN_RIGHT:    return 2;
        case BUILD_ACTION_ELEM_STRAIGHT_SHORT: return 3;
        case BUILD_ACTION_ELEM_STRAIGHT_LONG:  return 4;
        default:                               return 0;
    }
}

// Load the pre-defined default build action list for the current track
static void Build_Load_Default_Action_List(void)
{
    Build_Action_Count = BUILD_ACTION_COUNT;
    Build_Action_Index = 0;
    Build_Action_Active_Index = 0;
    memset(Build_Action_List, 0, sizeof(Build_Action_List));
    memcpy(Build_Action_List, Default_Build_Actions, sizeof(Default_Build_Actions));
}

// Mark the current action as finished and advance segment/element counters
static void Build_Finish_Current_Action(void)
{
    uint8_t finished_index = Build_Action_Active_Index;

    if (finished_index < Build_Action_Count)
    {
        Build_Action_Typedef *action = &Build_Action_List[finished_index];
        Execute_Times = action->segment_index;
        In_Line_Ele_Count = action->element_index;

        if (BUILD_ACTION_IS_ELEMENT(action->action))
        {
            In_Line_Ele_Count++;

            if (In_Line_Ele_Count >= Mileage_Num_By_Segment[Execute_Times])
            {
                Line_Num_Count++;
            }
        }
        else  // node action
        {
            Line_Num_Count++;
        }
    }

    if (Build_Action_Index >= Build_Action_Count)
    {
        Finish_Flag = 1;
    }
}

// Dispatch the next queued build action (node turn/straight or element)
static void Build_Dispatch_Current_Action(void)
{
    Build_Action_Typedef *action;

    if (Build_Action_Index >= Build_Action_Count)
    {
        Finish_Flag = 1;
        return;
    }

    Build_Action_Active_Index = Build_Action_Index;
    action = &Build_Action_List[Build_Action_Index];
    Execute_Times = action->segment_index;
    In_Line_Ele_Count = action->element_index;
    Mileage_Times = Mileage_Num_By_Segment[Execute_Times];

    //===== Dispatch by action type =====
    switch (action->action)
    {
        //--- Node actions ---
        case BUILD_ACTION_NODE_STRAIGHT:
            Segment_Total_Mileage[Execute_Times] = Count.Mileage;
            Build_Action_Index++;
            Straight_Node_Pending = 1;
            Run_Mode = Straight_Mode;
            return;

        case BUILD_ACTION_NODE_TURN_LEFT:
            Segment_Total_Mileage[Execute_Times] = Count.Mileage;
            Build_Action_Index++;
            Set_Node_Run_Mode(1);  // left turn
            return;

        case BUILD_ACTION_NODE_TURN_RIGHT:
            Segment_Total_Mileage[Execute_Times] = Count.Mileage;
            Build_Action_Index++;
            Set_Node_Run_Mode(2);  // right turn
            return;

        //--- Element actions ---
        case BUILD_ACTION_ELEM_STRAIGHT_SHORT:
        case BUILD_ACTION_ELEM_STRAIGHT_LONG:
        case BUILD_ACTION_ELEM_TURN_LEFT:
        case BUILD_ACTION_ELEM_TURN_RIGHT:
            Record_Segment_Edge_Mileage();
            Build_Action_Index++;
            Current_Element_Dir = Build_Action_To_Element_Dir(action->action);
            Mileage_Element_Base = Count.Mileage;
            Run_Mode = Mileage_Mode;
            return;

        //--- NONE (pass-through) ---
        default:
            Record_Segment_Edge_Mileage();
            Build_Action_Index++;
            In_Line_Ele_Count++;
            if (In_Line_Ele_Count >= Mileage_Times)
            {
                Segment_Total_Mileage[Execute_Times] = Count.Mileage;
                Build_Dispatch_Current_Action();
            }
            return;
    }
}

// Select the current run speed from the configured Basic_Speed
static int Select_Run_Speed(void)
{
    if (Basic_Speed < 0)
    {
        return 0;
    }

    return Basic_Speed;
}

/*************************************
** Function: Advance_Turn_Section_Index
** Description: Advance the turn section counter after a turn completes
**              (currently a placeholder; logic to be implemented if needed)
*************************************/
static void Advance_Turn_Section_Index(void)
{
}

/*************************************
** Function: Reset_Turn_Action_State
** Description: Reset all turn-related state variables to prepare for the next action
** Details:
**         Clears PID data, expected speeds, turn flags, and sets the
**         edge-check cooldown threshold to BUILD_CHECK_EDGE_NODE_TURN_MILEAGE.
**         Resets Run_Mode back to Normal_Mode.
*************************************/
static void Reset_Turn_Action_State(void)
{
    Gyro_Integral = 0;
    Turn_Angle_Last_Real = 0;
    Turn_Angle_Settle_Count = 0;
    Error = 0;
    Turn_PID_Out = 0;
    PID_cleardata(&Gyro_PID);
    PID_cleardata(&Gyro_PD_PID);
    PID_cleardata(&Angle_PID);
    PID_cleardata(&Turn_PID);
    Gyro_PID_Out = 0;
    Left_Exp_Spd = 0;
    Right_Exp_Spd = 0;
    Count.Left = 0;
    Count.Right = 0;
    is_left = 0;
    is_right = 0;
    Turn_Decel_Phase = 0;
    Check_Edge_Skip_Mileage_Base = Count.Mileage;
    Check_Edge_Skip_Thresh = BUILD_CHECK_EDGE_NODE_TURN_MILEAGE;
    Run_Mode = Normal_Mode;
}

/*************************************
** Function: Complete_Turn_Action
** Description: Finalize the current turn action
** Details:    1. Set Turn_Action_Done flag
**             2. Save turn mileage to Flash
**             3. Advance the action index via Build_Finish_Current_Action
**             4. Call Reset_Turn_Action_State to clear state
**    Called by Turn_Left_Run / Turn_Right_Run after angle settles.
**    Complete_Turn_Action also calls Record_Turn_Mileage.
*************************************/
static void Complete_Turn_Action(void)
{
    Turn_Action_Done = 1;
    Record_Turn_Mileage();

    Build_Finish_Current_Action();
    Reset_Turn_Action_State();
}

// Check if the accumulated gyro angle has settled within the target tolerance
static uint8_t Is_Turn_Angle_Settled(float angle_target)
{
    float angle_error = fabsf(angle_target - Gyro_Integral);

    if (angle_error <= TURN_ANGLE_SETTLE_ERROR_DEG && fabsf(Gyro_Z) <= TURN_GYRO_SETTLE_RATE_DPS)
    {
        if (Turn_Angle_Settle_Count < TURN_SETTLE_CYCLE_MIN)
        {
            Turn_Angle_Settle_Count++;
        }
    }
    else
    {
        Turn_Angle_Settle_Count = 0;
    }

    return (Turn_Angle_Settle_Count >= TURN_SETTLE_CYCLE_MIN);
}

/*************************************
** Function: Get_Track_Middle_Point
** Description: Compute the middle point of the detected track line
** Return:     If Track_Num > 0: average of first and last active sensor index
**             If Track_Num == 0: default center sensor index 7
** Details:    Uses Track_Arr[0] and Track_Arr[Track_Num-1] to compute midpoint.
*************************************/
static int Get_Track_Middle_Point(void)
{
    if (Track_Num > 0)
    {
        return (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
    }

    return 7;
}

/*************************************
** Function: Set_Node_Run_Mode
** Description: Configure the run mode based on the node direction at an intersection
** Input:      node_dir - 0=straight, 1=left turn, 2=right turn, 3/4=reserved
** Details:    Resets gyro integral, PID state, mileage counters, and sets Run_Mode
**             to the appropriate turning or straight mode.
*************************************/
static void Set_Node_Run_Mode(uint8_t node_dir)
{

    Gyro_Integral = 0;
    Turn_Angle_Last_Real = 0;
    Turn_Angle_Settle_Count = 0;
    PID_cleardata(&Angle_PID);
    Mileage_Turn_Done = 0;
    Turn_Action_Done = 0;
    Turn_Decel_Phase = 0;
    Count.Spd_Mileage = 0;
    Count.Mileage = 0;
    Mileage_Element_Base = 0;
    Straight_Node_Pending = 0;

    switch (node_dir)
    {
        case 1:
            Run_Mode = Turn_Left;
            is_left = 1;
            is_right = 0;
            Turn_Angle_Target = -90.0f;
            Turn_Begin_Mileage = Total_Run_Mileage;
            Advance_Turn_Section_Index();
            break;
        case 2:
            Run_Mode = Turn_Right;
            is_left = 0;
            is_right = 1;
            Turn_Angle_Target = 90.0f;
            Turn_Begin_Mileage = Total_Run_Mileage;
            Advance_Turn_Section_Index();
            break;
        case 0:
        default:
            Straight_Node_Pending = 1;
            Run_Mode = Straight_Mode;
            break;
    }
}

/*************************************
** Function: Finish_Mileage_Section
** Description: Complete the current mileage-based section (element or node segment)
** Details:    Resets turn flags, sets cooldown threshold for edge detection,
**             and transitions back to Normal_Mode. Calls Build_Finish_Current_Action
**             to advance the action queue.
*************************************/
static void Finish_Mileage_Section(void)
{

    is_left = 0;
    is_right = 0;
    Force_Straight_Speed = 0;
    Mileage_Turn_Done = 0;
    Check_Edge_Skip_Mileage_Base = Count.Mileage;
    Check_Edge_Skip_Thresh = BUILD_CHECK_EDGE_MILEAGE_STRAIGHT_MILEAGE;
    Run_Mode = Normal_Mode;

    Mileage_Element_Base = Count.Mileage;

    Build_Finish_Current_Action();
}

/*************************************
** Function: Record_Segment_Edge_Mileage
** Description: Record the mileage at each element edge within a segment for Segment_Total_Mileage
** Details:    Applies MILEAGE_COMPENSATION_X adjustment for turn elements (dir 1 or 2)
**             to mark the element boundary slightly before the actual edge.
*************************************/
static void Record_Segment_Edge_Mileage(void)
{
    if (Execute_Times < TRACK_SEGMENT_NUM_MAX && In_Line_Ele_Count < ELEMENT_NUM_MAX)
    {
        float recorded_mileage = Count.Mileage;

        if (Current_Element_Dir == 1 || Current_Element_Dir == 2)
        {
            recorded_mileage += MILEAGE_COMPENSATION_X;
        }

        Segment_Edge_Mileage_Record[Execute_Times][In_Line_Ele_Count] = recorded_mileage;
    }
}

/*************************************
** Function: Save_Segment_Edge_Mileage_Record_To_Flash
** Description: Save segment edge mileage records to Flash memory
** Details:    Copies Segment_Edge_Mileage_Record and Segment_Total_Mileage into a
**             packed struct, then writes to the designated Flash sector/page range.
*************************************/
static void Save_Segment_Edge_Mileage_Record_To_Flash(void)
{
    Segment_Edge_Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT * SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};


    memcpy(flash_log.Segment_Edge_Mileage_Record,
           Segment_Edge_Mileage_Record,
           sizeof(Segment_Edge_Mileage_Record));
    memcpy(flash_log.Segment_Total_Mileage,
           Segment_Total_Mileage,
           sizeof(Segment_Total_Mileage));


    memcpy(map_words, &flash_log, sizeof(flash_log));
    Save_Flash_Page_Block(SEGMENT_EDGE_MILEAGE_FLASH_SECTOR,
                          SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE,
                          SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT,
                          SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);
}

/*************************************
** Function: Load_Segment_Edge_Mileage_Record_From_Flash
** Description: Load segment edge mileage records from Flash memory
** Details:    Reads from the designated Flash sector/page range into a packed struct,
**             then copies back to Segment_Edge_Mileage_Record and Segment_Total_Mileage.
*************************************/
static void Load_Segment_Edge_Mileage_Record_From_Flash(void)
{
    Segment_Edge_Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT * SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};


    Load_Flash_Page_Block(SEGMENT_EDGE_MILEAGE_FLASH_SECTOR,
                          SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE,
                          SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT,
                          SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);


    memcpy(&flash_log, map_words, sizeof(flash_log));
    memcpy(Segment_Edge_Mileage_Record,
           flash_log.Segment_Edge_Mileage_Record,
           sizeof(Segment_Edge_Mileage_Record));
    memcpy(Segment_Total_Mileage,
           flash_log.Segment_Total_Mileage,
           sizeof(Segment_Total_Mileage));
}

/*************************************
** Function: Save_Turn_Mileage_Record_To_Flash
** Description: Save turn mileage interval records to Flash memory
** Details:    Copies Turn_Mileage_Record_Num and Turn_Mileage_Record into a packed
**             struct, then writes to the designated Flash sector/page range.
*************************************/
static void Save_Turn_Mileage_Record_To_Flash(void)
{
    Turn_Mileage_Flash_Typedef flash_log = {0, {0}};
    uint32 map_words[TURN_MILEAGE_FLASH_PAGE_COUNT * TURN_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};


    flash_log.Turn_Mileage_Record_Num = Turn_Mileage_Record_Num;
    memcpy(flash_log.Turn_Mileage_Record, Turn_Mileage_Record, sizeof(Turn_Mileage_Record));


    memcpy(map_words, &flash_log, sizeof(flash_log));
    Save_Flash_Page_Block(TURN_MILEAGE_FLASH_SECTOR,
                          TURN_MILEAGE_FLASH_START_PAGE,
                          TURN_MILEAGE_FLASH_PAGE_COUNT,
                          TURN_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);
}

/*************************************
** Function: Load_Turn_Mileage_Record_From_Flash
** Description: Load turn mileage interval records from Flash memory
** Details:    Reads from the designated Flash sector/page range into a packed struct,
**             performs bounds checking on Record_Num, then copies back.
*************************************/
static void Load_Turn_Mileage_Record_From_Flash(void)
{
    Turn_Mileage_Flash_Typedef flash_log = {0, {0}};
    uint32 map_words[TURN_MILEAGE_FLASH_PAGE_COUNT * TURN_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};


    Load_Flash_Page_Block(TURN_MILEAGE_FLASH_SECTOR,
                          TURN_MILEAGE_FLASH_START_PAGE,
                          TURN_MILEAGE_FLASH_PAGE_COUNT,
                          TURN_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);


    memcpy(&flash_log, map_words, sizeof(flash_log));


    if (flash_log.Turn_Mileage_Record_Num > TURN_MILEAGE_RECORD_MAX)
    {
        flash_log.Turn_Mileage_Record_Num = TURN_MILEAGE_RECORD_MAX;
    }


    Turn_Mileage_Record_Num = flash_log.Turn_Mileage_Record_Num;
    memcpy(Turn_Mileage_Record, flash_log.Turn_Mileage_Record, sizeof(Turn_Mileage_Record));
}

/*************************************
** Function: Save_Flash_Page_Block
** Description: Write data to a contiguous block of Flash pages
** Input:      sector       - Flash sector number
**             start_page   - First page within the sector
**             page_count   - Number of pages to write
**             words_per_page - Number of uint32_t words per page
**             words        - Pointer to source data array
** Details:    Uses the URIX Flash API: erase each page, then write.
*************************************/
static void Save_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, const uint32_t *words)
{
    uint8_t page_index;

    for (page_index = 0; page_index < page_count; page_index++)
    {
        flash_erase_page(sector, start_page + page_index);
        flash_write_page(sector,
                         start_page + page_index,
                         &words[page_index * words_per_page],
                         words_per_page);
    }
}

/*************************************
** Function: Load_Flash_Page_Block
** Description: Read data from a contiguous block of Flash pages
** Input:      sector/start_page/page_count/words_per_page - Same as Save counterpart
** Output:     words - Pointer to destination buffer (filled with uint32_t data)
** Details:    Reads uint32_t words from Flash pages into the output buffer.
*************************************/
static void Load_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, uint32_t *words)
{
    uint8_t page_index;

    for (page_index = 0; page_index < page_count; page_index++)
    {
        flash_read_page(sector,
                        start_page + page_index,
                        &words[page_index * words_per_page],
                        words_per_page);
    }
}

/*************************************
** Function: Record_Turn_Mileage
** Description: Record the mileage interval between turns into Turn_Mileage_Record
** Details:    Calculates turn_interval_mileage = Turn_Begin_Mileage - Last_Turn_Mileage_Base
**             as the distance traveled since the last recorded turn. Updates
**             Last_Turn_Mileage_Base to Total_Run_Mileage for the next interval.
**             Records are used for Flash-based self-learning.
*************************************/
static void Record_Turn_Mileage(void)
{
    float turn_interval_mileage;

    if (Turn_Mileage_Record_Num >= TURN_MILEAGE_RECORD_MAX)
    {
        return;
    }


    turn_interval_mileage = Turn_Begin_Mileage - Last_Turn_Mileage_Base;
    if (turn_interval_mileage < 0)
    {
        turn_interval_mileage = 0;
    }

    Turn_Mileage_Record[Turn_Mileage_Record_Num] = turn_interval_mileage;
    Turn_Mileage_Record_Num++;

    Last_Turn_Mileage_Base = Total_Run_Mileage;


}

/*************************************
** Function: Load_All_Flash_Data_For_VOFA
** Description: Load all Flash records for VOFA visualization/debugging tool
** Details:    Loads Turn_Mileage_Record and Segment_Edge_Mileage_Record from Flash
**             so they can be displayed and analyzed in VOFA.
*************************************/
void Load_All_Flash_Data_For_VOFA(void)
{
    Load_Turn_Mileage_Record_From_Flash();
    Load_Segment_Edge_Mileage_Record_From_Flash();
}

/********************************* Core Run-Time Functions *********************************/

/*************************************
** Function: Safety_Check
** Description: Safety monitoring — low voltage, stall detection, finish detection, LED indication
** Details:    1. Low voltage: if Voltage_Check[0] stays below SAFETY_LOW_VOLTAGE_THRESHOLD
**                for LOW_VOLT_FRAME_THRESH consecutive frames, trigger stop with yellow LED.
**             2. Stall detection: if Count.Stop exceeds SAFETY_STOP_CYCLE_MAX (all-on/all-off),
**                trigger Stop_Flag.
**             3. Finish detection: if Finish_Flag is set for over 200 cycles, stop and
**                save Flash records (Build_Mode only).
**             4. LED priority: yellow(low voltage) > blue(stopped beep) > green(normal).
**             5. On stop: zero all motor outputs and clear PID state.
*************************************/
void Safety_Check(void)
{
    static uint16_t stop_beep_count = 0;
    static uint8_t  low_voltage = 0;         // latch: stays 1 after confirmed trigger
    static uint8_t  low_volt_frames = 0;     // consecutive low-voltage frame counter

#define LOW_VOLT_FRAME_THRESH 1000  // frames of sustained low voltage before stop

    // Voltage check: debounce — require N consecutive frames below threshold
    if (Voltage_Check[0] < SAFETY_LOW_VOLTAGE_THRESHOLD)
    {
        low_volt_frames++;
        if (low_volt_frames >= LOW_VOLT_FRAME_THRESH)
        {
            Stop_Flag = 1;
            low_voltage = 1;
        }
    }
    else
    {
        low_volt_frames = 0;
    }

    if(Count.Stop > SAFETY_STOP_CYCLE_MAX)
    {
        Stop_Flag = 1;
    }

    if (Finish_Flag == 1)
    {
        Finish_Count++;
    }
    if (Finish_Count > 200 && Stop_Flag == 0)
    {
        Stop_Flag = 1;
        if (Mode == Build_Mode)
        {
            Save_Turn_Mileage_Record_To_Flash();
            Save_Segment_Edge_Mileage_Record_To_Flash();
        }
    }

    // LED priority: yellow(voltage) > blue(beep) > green(normal)
    if (Stop_Flag != 0)
    {
        pwm_set_duty(Suction_Motor_PWM, 0);
        pwm_set_duty(Suction_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_PWM, 0);
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);

        Left_Exp_Spd = 0;
        Right_Exp_Spd = 0;
        Left_PID_Out = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);

        if (low_voltage)
        {
            g_led_flag = 2;  // yellow: low voltage warning
        }
        else
        {
            stop_beep_count++;
            if (stop_beep_count >= 200)
            {
                stop_beep_count = 0;
            }
            g_led_flag = (stop_beep_count < 60) ? 1 : 0;  // blue beep / green idle
        }
    }
    else
    {
        stop_beep_count = 0;
        // g_led_flag set by run modes (Normal_Run=green, others=blue)
    }
}

/*************************************
** Function: Car_Go
** Description: Main control loop entry point, called every 3ms
** Call Order:  Safety_Check -> Get_Light (3ms) -> (alternating) Get_Speed(6ms) -> Get_IMU
**              -> Light_Process -> Build_Mode_Get_Error -> Set_Speed -> Set_Out
** Details:    Runs at 3ms interval. Speed sampling alternates every other tick (effective 6ms).
**             Left_Real_Spd/Right_Real_Spd are updated with weighted IIR filter.
**             On EnableSwitch ON rising edge, inserts 800-cycle (2.4s) start delay.
*************************************/
void Car_Go()
{

    if (EnableSwitch_ON == 1 && Last_EnableSwitch_ON == 0)
    {
        Enable_Start_Delay_Count = 800;
    }
    Last_EnableSwitch_ON = EnableSwitch_ON;


    if (Mode == Debug_Mode)
    {
        Enable_Start_Delay_Count = 0;
    }


    if (Speed_Get_Count == 1)
    {
        Get_Speed();
    }
    Speed_Get_Count *= -1;

    Get_IMU();

    Get_Light();

    Light_Process();

    Safety_Check();


    if (Stop_Flag != 0)
    {
        return;
    }


    if (Mode == Build_Mode)
    {
        if (EnableSwitch_ON)
        {
            Build_Mode_Get_Error();
        }
    }

    Set_Speed();

     Set_Out();
}


/* Debug functions moved to Debug_Car.c */

// Read encoder counts and compute filtered speed via weighted moving average
void Get_Speed()
{
    int left_raw, right_raw;
    float instant_speed;


    right_raw  = encoder_get_count(TIM4_ENCODER);
    left_raw =  encoder_get_count(TIM2_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
    encoder_clear_count(TIM2_ENCODER);


    giSpeed_Left[2] = giSpeed_Left[1];
    giSpeed_Left[1] = giSpeed_Left[0];
    giSpeed_Right[2] = giSpeed_Right[1];
    giSpeed_Right[1] = giSpeed_Right[0];

    giSpeed_Left[0]  = left_raw;
    giSpeed_Right[0] = right_raw;

    // Weighted IIR: 0.5*window[0] + 0.3*window[1] + 0.2*window[2]
    Left_Real_Spd  = (int)(0.5f * giSpeed_Left[0]  + 0.3f * giSpeed_Left[1]  + 0.2f * giSpeed_Left[2]);
    Right_Real_Spd = (int)(0.5f * giSpeed_Right[0] + 0.3f * giSpeed_Right[1] + 0.2f * giSpeed_Right[2]);


    if (EnableSwitch_ON)
    {
        instant_speed = (left_raw + right_raw) / 2.0f;
        Count.Mileage += instant_speed;
        Total_Run_Mileage += instant_speed;
    }
}

/*************************************
** Function: Get_IMU
** Description: Read and process IMU gyro data every 3ms
** Details:    Reads gyro Z-axis from IMU660RB, applies zero-drift offset.
**             Small values (|raw| < 2.0 deg/s) are clamped to zero.
**             Gyro_Integral is accumulated only during turns or Angle debug mode.
*************************************/
void Get_IMU()
{
    imu660rb_get_gyro();

    float gyro_raw = imu660rb_gyro_transition(imu660rb_gyro_z) - gyro_z_offset;
    uint8 debug_angle_run = (
        (Debug_Sub_Mode == Debug_Sub_Angle || Debug_Sub_Mode == Debug_Sub_NormalTrace)
        && Debug_Motor_Enable == 1);

    if (fabs(gyro_raw) < 2.0f)
    {
        Gyro_Z = 0;
        imu660rb_gyro_z = 0;
    }
    else
    {
        Gyro_Z = gyro_raw;
    }

        if (is_left == 1 || is_right == 1 || debug_angle_run)
    {
        Gyro_Integral += Gyro_Z * GYRO_INTEGRATION_PERIOD_S;
    }
    else
    {
        Gyro_Integral = 0;
    }


    // {
    //     float raw_z = (float)imu660rb_gyro_z;
    //     if (fabs(raw_z) < 30.0f)
    //     {
    //         Gyro_Z_For_PID = 0;
    //     }
    //     else
    //     {
    //         Gyro_Z_For_PID = raw_z / 1000.0f;
    //     }
    // }
}

/*************************************
** Function: Check_Edge
** Description: Detect track segment edges (intersections / element boundaries)
** Return:     0 = no edge detected, 1 = edge detected
** Details:    Respects mileage-based cooldown before allowing detection.
**             Edge is detected when either end sensor sees white AND at least
**             4 sensors are on, or when at least 5 sensors are on.
**             On detection, resets Count.Mileage and increments Check_Edge_Count.
*************************************/
uint8 Check_Edge()
{
    // Mileage-based cooldown: block edge detection until enough distance driven
    if (Check_Edge_Skip_Thresh > 0)
    {
        if ((Count.Mileage - Check_Edge_Skip_Mileage_Base) < Check_Edge_Skip_Thresh)
            return 0;
        Check_Edge_Skip_Thresh = 0;  // cooldown expired naturally
    }


    if (((Light_Convert[0] == 1 || Light_Convert[14] == 1)&&
        Initial_White_Num >= 4) || Initial_White_Num >= 5)
    {
        Check_Edge_Count++;
        Count.Mileage = 0;

        return 1;
    }

    return 0;
}

/*************************************
** Function: Light_Process
** Description: Process light sensor ADCs and update track line detection, called every 3ms
** Details:    Applies binary thresholding to each sensor based on Light_Thr array.
**             LEDs mirror sensor state when not in mileage/turn modes.
**             Builds Track_Arr of active (white) sensor indexes, validates contiguity
**             (adjacent only via Is_Track_Sensor_Adjacent), and updates Stop counter.
*************************************/
void Light_Process()
{
    memcpy(Last_Light_Convert, Light_Convert, sizeof(Light_Convert));
    uint8_t Led_Control_Enable = (Run_Mode != Mileage_Mode && Run_Mode != Turn_Left && Run_Mode != Turn_Right);
    uint8_t sensor_index;


    for (int i = 0; i < 15; i++)
    {
        if (Light_ADC[i] > Light_Thr[i][0])
        {
            Light_Convert[i] = 1;
            if (Led_Control_Enable)
            {
                TCA9555_LED_Ctrl(LED[14 - i], 1);
            }
        }
        if (Light_ADC[i] < Light_Thr[i][1])
        {
            Light_Convert[i] = 0;
            if (Led_Control_Enable)
            {
                TCA9555_LED_Ctrl(LED[14 - i], 0);
            }
        }
    }

    memcpy(Last_Track_Arr, Track_Arr, sizeof(Track_Arr));
    Last_Track_Num = Track_Num;
    for (int i = 0; i < 15; i++)
    {
        Track_Arr[i] = 0;
    }
    Initial_White_Num = 0;
    Track_Num = 0;
    Left_Num = 0;
    Right_Num = 0;

    for (uint8_t i = 0; i < TRACK_SENSOR_ACTIVE_NUM; i++)
    {
        sensor_index = Track_Sensor_Active_Index[i];
        if (Light_Convert[sensor_index] == 1)
        {
            Initial_White_Num++;
            Track_Arr[Track_Num++] = sensor_index;
        }
    }

    // Validate sensor contiguity: if a gap is found, revert to last valid frame
    for (int i = 0; i < Track_Num - 1; i++)
    {
        if (!Is_Track_Sensor_Adjacent((uint8_t)Track_Arr[i], (uint8_t)Track_Arr[i + 1]))
        {
            Track_Num = Last_Track_Num;
            memcpy(Track_Arr, Last_Track_Arr, sizeof(Last_Track_Arr));
            break;
        }
    }


    if (EnableSwitch_ON && Mode != Debug_Mode)
    {
        if ((Track_Num == TRACK_SENSOR_ACTIVE_NUM || Track_Num == 0) && is_left == 0 && is_right == 0)
        {
            Count.Stop++;
        }
        else
        {
            Count.Stop = 0;
        }
    }
    else
    {
        Count.Stop = 0;
    }

}

/*************************************
** Function: Build_Mode_Get_Error
** Description: Build mode main dispatcher — select run mode and compute tracking error
** Details:    On first call (First_Mode==0), initializes action list, counters, and
**             resets Flash-stored records.
**             Dispatches to Normal_Run, Turn_Left_Run, Turn_Right_Run, Mileage_Mode_Run,
**             or Straight_Run depending on current Run_Mode.
*************************************/
void Build_Mode_Get_Error()
{
    // (Check_Edge cooldown is now mileage-based, checked in Check_Edge())


    if (First_Mode == 0)
    {
        Run_Mode = Normal_Mode;                                       // Initialize to normal trace
        First_Mode = 1;                                               // Mark as initialized
        Execute_Times = 0;
        Mileage_Times = 0;
        Line_Num_Count = 0;
        In_Line_Ele_Count = 0;
        Build_Action_Index = 0;
        Build_Action_Active_Index = 0;
        Build_Load_Default_Action_List();                             // Load default build actions
        Mileage_Times = Mileage_Num_By_Segment[Execute_Times]; // Load first segment element count
        Turn_Mileage_Record_Num = 0;
        Last_Turn_Mileage_Base = 0;
        memset(Turn_Mileage_Record, 0, sizeof(Turn_Mileage_Record));
        memset(Segment_Edge_Mileage_Record, 0, sizeof(Segment_Edge_Mileage_Record));
        Save_Segment_Edge_Mileage_Record_To_Flash();
    }


    switch (Run_Mode)
    {
        case Normal_Mode:
            Normal_Run();
            break;
        case Turn_Left:
            Turn_Left_Run();
            break;
        case Turn_Right:
            Turn_Right_Run();
            break;
        case Mileage_Mode:
            Mileage_Mode_Run();
            break;
        case Straight_Mode:
            Straight_Run();
            break;
        default:
            Normal_Run();
            break;
    }

    // single-point LED: green when normal tracing, blue when object detected
    if (Mode == Build_Mode)
        g_led_flag = (Run_Mode == Normal_Mode) ? 0 : 1;

}

/*************************************
** Function: Normal_Run
** Description: Normal line-following mode: compute steering error from track position
** Details:    Calculates the midpoint of detected track line and the lateral error
**             (average of direction offset values for leftmost and rightmost sensors).
**             Checks for segment edges and dispatches the next build action if found.
*************************************/
void Normal_Run()
{
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

    if (Check_Edge())
    {
        Build_Dispatch_Current_Action();
    }
}
/*************************************
** Function: Turn_Left_Run
** Description: Execute a left turn (node or element), called every 3ms
** Details:    For Build_Mode: Phase 0 drives straight for Mileage_Node_Turn_Delay distance,
**             then Phase 1/2 uses angle PID to turn to the target angle (-90 deg).
**             Once the angle is settled, calls Complete_Turn_Action.
**             For non-Build_Mode: immediately executes the turn with Set_Mileage_Turn_Exp_Speed.
*************************************/
void Turn_Left_Run(void)
{
    TCA9555_All_LED_On();

    if (Turn_Action_Done)
        return;

    if (Mode == Build_Mode)
    {
        if (Turn_Decel_Phase == 0)
        {
            Error = 0;
            if (Count.Mileage >= Mileage_Node_Turn_Delay)
            {
                Turn_Decel_Phase = 1;
                is_left = 1;
                Gyro_Integral = 0;
                Turn_Angle_Last_Real = 0;
                Turn_Angle_Settle_Count = 0;
                PID_cleardata(&Angle_PID);
                Count.Spd_Mileage = 0;
            }
        }
        // else if (Turn_Decel_Phase == 1)
        // {
        //     Error = 0;
        //     Left_Exp_Spd = 0;
        //     Right_Exp_Spd = 0;
        //     if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
        //     {
        //         Turn_Decel_Phase = 2;
        //         Gyro_Integral = 0;
        //         Turn_Angle_Last_Real = 0;
        //         Turn_Angle_Settle_Count = 0;
        //         PID_cleardata(&Angle_PID);
        //     }
        // }
        else
        {
            Error = 0;
            Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target);
            if (Is_Turn_Angle_Settled(Turn_Angle_Target))
            {
                Complete_Turn_Action();
            }
        }
        return;
    }

    Error = 0;
    Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target);

    if (Is_Turn_Angle_Settled(Turn_Angle_Target))
    {
        Complete_Turn_Action();
    }
}

/*************************************
** Function: Turn_Right_Run
** Description: Execute a right turn (mirror of Turn_Left_Run) - same Build_Mode phases
*************************************/
void Turn_Right_Run(void)
{
    if (Turn_Action_Done)
        return;

    if (Mode == Build_Mode)
    {
        if (Turn_Decel_Phase == 0)
        {
            Error = 0;
            if (Count.Mileage >= Mileage_Node_Turn_Delay)
            {
                Turn_Decel_Phase = 1;
                is_right = 1;
                Gyro_Integral = 0;
                Turn_Angle_Last_Real = 0;
                Turn_Angle_Settle_Count = 0;
                PID_cleardata(&Angle_PID);
                Count.Spd_Mileage = 0;
            }
        }
        // else if (Turn_Decel_Phase == 1)
        // {
        //     Error = 0;
        //     Left_Exp_Spd = 0;
        //     Right_Exp_Spd = 0;
        //     if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
        //     {
        //         Turn_Decel_Phase = 2;
        //         Gyro_Integral = 0;
        //         Turn_Angle_Last_Real = 0;
        //         Turn_Angle_Settle_Count = 0;
        //         PID_cleardata(&Angle_PID);
        //     }
        // }
        else
        {
            Error = 0;
            Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target);
            if (Is_Turn_Angle_Settled(Turn_Angle_Target))
            {
                Complete_Turn_Action();
            }
        }
        return;
    }

    Error = 0;
    Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target);

    if (Is_Turn_Angle_Settled(Turn_Angle_Target))
    {
        Complete_Turn_Action();
    }
}
/*************************************
** Function: Mileage_Mode_Run
** Description: Mileage-based element execution, called every 3ms
** Details:    Dispatches based on Current_Element_Dir:
**             Dir 1/2 (turn elements): delegates to Mileage_Run_Stage_2, then finishes
**             section and sets cooldown for edge detection.
**             Dir 3 (short straight): forces straight with Error=0 until MILEAGE_STRAIGHT_SHORT reached.
**             Dir 4 (long straight): forces straight with Error=0 until MILEAGE_STRAIGHT_LONG reached.
**             Other: forces straight indefinitely.
*************************************/
void Mileage_Mode_Run()
{
    float section_mileage = Count.Mileage - Mileage_Element_Base;

    if (Current_Element_Dir == 1 || Current_Element_Dir == 2)
    {
        Force_Straight_Speed = 0;
        Mileage_Run_Stage_2();

        if (Mileage_Turn_Done == 1)
        {
            Record_Turn_Mileage();
            Finish_Mileage_Section();
            Check_Edge_Skip_Mileage_Base = Count.Mileage;
            Check_Edge_Skip_Thresh = BUILD_CHECK_EDGE_MILEAGE_TURN_MILEAGE;
        }
    }
    else if (Current_Element_Dir == 3)
    {
        Error = 0;
        Force_Straight_Speed = 1;

        if (section_mileage >= MILEAGE_STRAIGHT_SHORT)
        {
            Finish_Mileage_Section();
        }
    }
    else if (Current_Element_Dir == 4)
    {
        Error = 0;
        Force_Straight_Speed = 1;

        if (section_mileage >= MILEAGE_STRAIGHT_LONG)
        {
            Finish_Mileage_Section();
        }
    }
    else
    {
        Error = 0;
        Force_Straight_Speed = 1;
    }
}

/*************************************
** Function: Mileage_Run_Stage_2 / Set_Mileage_Turn_Exp_Speed
** Description: Stage 2 of element turn execution using angle PID cascade
*************************************/

/*************************************
** Function: Set_Mileage_Turn_Exp_Speed
** Description: Calculate expected wheel speeds for an angle-based turn
**              using cascaded PID: outer angle PID -> inner gyro PID -> wheel speeds
** Input:      angle_target - Target gyro angle (deg, e.g. +/-90)
*************************************/
void Set_Mileage_Turn_Exp_Speed(float angle_target)
{
    float gyro_target;

    gyro_target = PID_calc(&Angle_PID, angle_target, Gyro_Integral);
    Gyro_PID_Out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);
    Left_Exp_Spd = Basic_Speed + (int)Gyro_PID_Out;
    Right_Exp_Spd = Basic_Speed - (int)Gyro_PID_Out;
    Turn_Angle_Last_Real = Gyro_Integral;
}

// Stage 2 of element turn: deceleration phase, then angle PID turn with settle detection
void Mileage_Run_Stage_2()
{
    float section_mileage = Count.Mileage - Mileage_Element_Base;

    switch (Current_Element_Dir)
    {
        case 1:
            if (Mileage_Turn_Done == 0)
            {
                if (Turn_Decel_Phase == 0)
                {
                    Error = 0;
                    if (section_mileage >= Mileage_Element_Turn_Delay)
                    {
                        Turn_Decel_Phase = 1;
                        is_left = 1;
                        Gyro_Integral = 0;
                        Turn_Angle_Last_Real = 0;
                        Turn_Angle_Settle_Count = 0;
                        PID_cleardata(&Angle_PID);
                        Count.Spd_Mileage = 0;
                        // Advance_Turn_Section_Index();
                    }
                }
                // else if (Turn_Decel_Phase == 1)
                // {
                //     Error = 0;
                //     Left_Exp_Spd = 0;
                //     Right_Exp_Spd = 0;
                //     if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                //     {
                //         Turn_Decel_Phase = 2;
                //         Gyro_Integral = 0;
                //         Turn_Angle_Last_Real = 0;
                //         Turn_Angle_Settle_Count = 0;
                //         PID_cleardata(&Angle_PID);
                //     }
                // }
                else
                {
                    Set_Mileage_Turn_Exp_Speed(-BUILD_TURN_TARGET_ANGLE_DEG);
                    if (Is_Turn_Angle_Settled(-BUILD_TURN_TARGET_ANGLE_DEG))
                    {
                        Error = 0;
                        is_left = 0;
                        Mileage_Turn_Done = 1;
                        Turn_Decel_Phase = 0;
                        Gyro_Integral = 0;
                    }
                }
            }
            else
            {
                Error = 0;
            }
            break;
        case 2:
            if (Mileage_Turn_Done == 0)
            {
                if (Turn_Decel_Phase == 0)
                {
                    Error = 0;
                    if (section_mileage >= Mileage_Element_Turn_Delay)
                    {
                        Turn_Decel_Phase = 1;
                        is_right = 1;
                        Gyro_Integral = 0;
                        Turn_Angle_Last_Real = 0;
                        Turn_Angle_Settle_Count = 0;
                        PID_cleardata(&Angle_PID);
                        Count.Spd_Mileage = 0;
                        Advance_Turn_Section_Index();
                    }
                }
                // else if (Turn_Decel_Phase == 1)
                // {
                //     Error = 0;
                //     Left_Exp_Spd = 0;
                //     Right_Exp_Spd = 0;
                //     if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                //     {
                //         Turn_Decel_Phase = 2;
                //         Gyro_Integral = 0;
                //         Turn_Angle_Last_Real = 0;
                //         Turn_Angle_Settle_Count = 0;
                //         PID_cleardata(&Angle_PID);
                //     }
                // }
                else
                {
                    Set_Mileage_Turn_Exp_Speed(BUILD_TURN_TARGET_ANGLE_DEG);
                    if (Is_Turn_Angle_Settled(BUILD_TURN_TARGET_ANGLE_DEG))
                    {
                        Error = 0;
                        is_right = 0;
                        Mileage_Turn_Done = 1;
                        Turn_Decel_Phase = 0;
                        Gyro_Integral = 0;
                    }
                }
            }
            else
            {
                Error = 0;
            }
            break;
    }
}
/*************************************
** Function: Set_Speed
** Description: PID speed control — compute expected wheel speeds from tracking error
** Control Chain: Error -> Turn_PID -> Gyro_PID(+Gyro_Z damping) -> left/right speeds
** Details:    Either uses angle-based control (during turns) or normal trace control
**             with gyro rate as damping. Force_Straight_Speed overrides steering when set.
**             Outputs Left_Exp_Spd and Right_Exp_Spd, and computes wheel speed PID outputs.
*************************************/
void Set_Speed()
{
    static float normal_gyro_out_last = 0.0f;
    static uint8_t force_straight_last = 0;

    Left_PID_Out = 0;
    Right_PID_Out = 0;


    if (EnableSwitch_ON == 0)
    {
        normal_gyro_out_last = 0.0f;
        Turn_PID_Out = 0;
        Gyro_PID_Out = 0;
        force_straight_last = 0;
        Left_Exp_Spd = 0;
        Right_Exp_Spd = 0;
        PID_cleardata(&Turn_PID);
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Gyro_PD_PID);
        return;
    }


    Run_Speed = Select_Run_Speed();


    if (is_left == 1 || is_right == 1)
    {
        normal_gyro_out_last = 0.0f;
        force_straight_last = 0;
        PID_cleardata(&Gyro_PD_PID);
        PID_cleardata(&Turn_PID);
    }
    else
    {
        if (Force_Straight_Speed)
        {
            if (force_straight_last == 0)
            {
                PID_cleardata(&Gyro_PD_PID);
                PID_cleardata(&Turn_PID);
                normal_gyro_out_last = 0.0f;
            }
            force_straight_last = 1;
            Turn_PID_Out = 0;

            Gyro_PID_Out = PID_calc(&Gyro_PID, 0.0f, Gyro_Z);
        }
        else
        {
            force_straight_last = 0;

            // Normal trace uses gyro rate as damping, not as a stateful inner loop.
            Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, (float)Error);
            Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        }

        // if (Gyro_PID_Out > normal_gyro_out_last + NORMAL_GYRO_OUT_STEP_MAX)
        // {
        //     Gyro_PID_Out = normal_gyro_out_last + NORMAL_GYRO_OUT_STEP_MAX;
        // }
        // else if (Gyro_PID_Out < normal_gyro_out_last - NORMAL_GYRO_OUT_STEP_MAX)
        // {
        //     Gyro_PID_Out = normal_gyro_out_last - NORMAL_GYRO_OUT_STEP_MAX;
        // }
        normal_gyro_out_last = Gyro_PID_Out;


        Left_Exp_Spd = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }


    if (EnableSwitch_ON)
    {
        Average_Speed = (Left_Real_Spd + Right_Real_Spd) / 2.0;
        Count.Spd_Mileage += Average_Speed;
    }


    if (EnableSwitch_ON)
    {
        Left_PID_Out  = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
        Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
    }
}

/*************************************
** Function: Set_Out
** Description: Apply PWM outputs to motors based on PID results, called every 3ms
** Details:    Handles start delay (Enable_Start_Delay_Count) with suction motor on.
**             Sets suction motor PWM when enabled and not stopped.
**             Applies left/right motor direction and PWM based on sign of PID output:
**             positive = forward, negative = reverse.
*************************************/
void Set_Out(void)
{



    if (Enable_Start_Delay_Count > 0)
    {
        Enable_Start_Delay_Count--;


        pwm_set_duty(Suction_Motor_PWM, 9520);
        pwm_set_duty(Suction_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_PWM, 0);
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);

        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        return;
    }

    if (EnableSwitch_ON && Stop_Flag == 0)
    {
        pwm_set_duty(Suction_Motor_PWM, 9500);
        pwm_set_duty(Suction_Motor_DIR, 0);
    }
    else
    {
        pwm_set_duty(Suction_Motor_PWM, 0);
        pwm_set_duty(Suction_Motor_DIR, 0);
    }


    if (EnableSwitch_ON && Stop_Flag == 0)
    {

        if (Left_PID_Out == 0)
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


        if (Right_PID_Out == 0)
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
    else
    {
        pwm_set_duty(Left_Motor_DIR, 0);
        pwm_set_duty(Left_Motor_PWM, 0);
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);

        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
    }
}

/*************************************
** Function: Straight_Run
** Description: Straight line mode — maintain straight until track narrows enough, called every 3ms
** Details:    Waits until track narrows (Track_Num < 5) and the midpoint is near center,
**             counts consecutive straight-valid cycles, then either finishes a pending
**             node straight section or falls back to Normal_Mode.
*************************************/
void Straight_Run(void)
{
    Error = 0;
    Middle = Get_Track_Middle_Point();


    if (Track_Num < 5 && Track_Num > 1 && Middle > 3 && Middle < 11)
    {
        Count.Straight++;
    }
    else
    {
        Count.Straight = 0;
    }

    if (Count.Straight > 0)
    {
        if (Straight_Node_Pending != 0)
        {
            Finish_Mileage_Section();
            Straight_Node_Pending = 0;
            Count.Straight = 0;
        }
        else  // triggered by element, not a pending node; revert to normal
        {
            Run_Mode = Normal_Mode;
            Count.Straight = 0;
        }
    }
}
