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
int Finish_Flag = 0;
uint8_t Left_Num = 0;
uint8_t Right_Num = 0;
uint8_t Left_Flag = 0;
uint8_t Right_Flag = 0;
uint8_t is_left = 0;
uint8_t is_right = 0;
float  Turn_Angle_Target = 0;
uint8_t Turn_Angle_D_First = 1;
uint8_t Turn_Decel_Phase = 0;
uint8_t Mileage_Turn_Done = 0;
uint8_t Turn_Action_Done = 0;
float Check_Edge_Skip_Thresh = 0;      // Edge-detect cooldown mileage threshold (encoder ticks)
float Check_Edge_Skip_Mileage_Base = 0; // Count.Mileage snapshot when cooldown started
uint8_t Last_EnableSwitch_ON = 0;
uint8_t g_led_flag = 0;                        // 0=green(normal) 1=blue(object) 2=yellow(low voltage)
uint8_t g_scan_progress = 0;                  // Scan progress 0-100 (0=not scanning)
int Middle = 0;
float Gyro_Integral = 0;
float Total_Angle = 0;                         // Continuous gyro angle, corrected after each turn
static int16_t total_left_turns  = 0;          // Cumulative left turn count for angle correction
static int16_t total_right_turns = 0;          // Cumulative right turn count for angle correction
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
// (Current_Element_Dir removed — no longer needed for edge mileage compensation)

/*-------------------------------*/
// Single-row 15-sensor tracking: all indexes below participate in Track_Num.
#define TRACK_SENSOR_ACTIVE_NUM 15
static const uint8_t Track_Sensor_Active_Index[TRACK_SENSOR_ACTIVE_NUM] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

/*--------------- Default Build Action Table (flat enum array, 31 actions) ---------------*/
const uint8_t Default_Build_Actions[BUILD_ACTION_COUNT] =
{
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_ELEM_TURN_LEFT, BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_ELEM_TURN_LEFT, BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_TURN_RIGHT,
    BUILD_ACTION_NODE_TURN_RIGHT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_NODE_STRAIGHT,
    BUILD_ACTION_NODE_TURN_LEFT,
    BUILD_ACTION_ELEM_STRAIGHT_SHORT, BUILD_ACTION_ELEM_STRAIGHT_SHORT,
};

// (Execute_Times, Mileage_Times, Mileage_Num_By_Segment removed — now flat array + Count fields)
uint8_t Build_Action_List[BUILD_ACTION_MAX] = {0};
uint8_t Build_Action_Index = 0;
uint8_t Build_Action_Count = 0;
static uint8_t Build_Action_Active_Index = 0;

/*--------------- PID Output Values ---------------*/
float Turn_PID_Out = 0.0;
float Gyro_PID_Out = 0.0;
float Left_PID_Out = 0.0;
float Right_PID_Out = 0.0;

Run_Mode_Enum Run_Mode = Normal_Mode;

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
#define BUILD_TURN_DECEL_DISTANCE      300.0f  // encoder ticks: distance for decel+advance before in-place rotate
#define NODE_STRAIGHT_MILEAGE         100.0f  // encoder ticks: node straight pass-through distance

// Mileage_Element_Turn_Delay / Mileage_Node_Turn_Delay now #define macros in Ctrl.h
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

// Normalize an angle to [-180, 180] degrees
static inline float Normalize_Angle_180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

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
    .Left        = 0,
    .Right       = 0,
    .Stop        = 0,
    .Mileage     = 0,
    .Spd_Mileage = 0,
    .Straight    = 0,
    .Stall       = 0,
    .Edge        = 0,
    .Line        = 0,
    .Element     = 0,
    .Finish      = 0,
    .StartDelay  = 0,
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
static void Set_Node_Run_Mode(uint8_t node_dir);
static void Reset_Turn_Action_State(void);
static void Complete_Turn_Action(void);
static void Record_Turn_Mileage(void);
static uint8_t Is_Turn_Angle_Settled(float angle_target);
static uint8_t Is_Track_Sensor_Adjacent(uint8_t left_index, uint8_t right_index);
static void Build_Load_Default_Action_List(void);
static void Build_Dispatch_Current_Action(void);
static void Build_Finish_Current_Action(void);

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

// (Build_Action_To_Element_Dir removed — Build_Action_List is now flat enum array)

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
            Count.Straight = NODE_STRAIGHT_MILEAGE;
            Segment_Edge_Mileage_Record[Count.Line][Count.Element++] = Count.Mileage;
            goto do_straight;
        case BUILD_ACTION_ELEM_STRAIGHT_SHORT:
            Count.Straight = MILEAGE_STRAIGHT_SHORT;
            Segment_Edge_Mileage_Record[Count.Line][Count.Element++] = Count.Mileage;
            goto do_straight;
        case BUILD_ACTION_ELEM_STRAIGHT_LONG:
            Count.Straight = MILEAGE_STRAIGHT_LONG;
            Segment_Edge_Mileage_Record[Count.Line][Count.Element++] = Count.Mileage;
        do_straight:
            Count.Spd_Mileage = Count.Mileage;
            Run_Mode = Straight_Mode;
            break;

        // ─── Left turn (node + element) ───
        case BUILD_ACTION_NODE_TURN_LEFT:
            Segment_Total_Mileage[Count.Line] = Count.Mileage;
            Count.Element = 0;
            Count.Stall = Mileage_Node_Turn_Delay;          // ★ fix: was missing
            goto do_left;
        case BUILD_ACTION_ELEM_TURN_LEFT:
            Segment_Total_Mileage[Count.Line] = Count.Mileage;
            Count.Element = 0;
            Count.Stall = Mileage_Element_Turn_Delay;
        do_left:
            Count.Line++;
            Set_Node_Run_Mode(1);
            break;

        // ─── Right turn (node + element) ───
        case BUILD_ACTION_NODE_TURN_RIGHT:
            Segment_Total_Mileage[Count.Line] = Count.Mileage;
            Count.Element = 0;
            Count.Stall = Mileage_Node_Turn_Delay;
            goto do_right;
        case BUILD_ACTION_ELEM_TURN_RIGHT:
            Segment_Total_Mileage[Count.Line] = Count.Mileage;
            Count.Element = 0;
            Count.Stall = Mileage_Element_Turn_Delay;
        do_right:
            Count.Line++;
            Set_Node_Run_Mode(2);
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
    {
        return 0;
    }

    return Basic_Speed;
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
    Count.Mileage = 0;  // Zero segment mileage after Phase 1
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

    // Correct Total_Angle to exact turn sum after each turn
    if (Turn_Angle_Target < 0) total_left_turns++;
    else                       total_right_turns++;
    Total_Angle = Normalize_Angle_180(-90.0f * total_left_turns + 90.0f * total_right_turns);

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
    Left_Exp_Spd = 0;
    Right_Exp_Spd = 0;
    Gyro_Integral = 0;
    Turn_Angle_Settle_Count = 0;
    PID_cleardata(&Angle_PID);
    Turn_Action_Done = 0;
    Turn_Decel_Phase = 0;
    // Count.Mileage already zeroed by Reset_Turn_Action_State — no need to re-zero here

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
    Mileage_Turn_Done = 0;
    Check_Edge_Skip_Mileage_Base = Count.Mileage;
    Check_Edge_Skip_Thresh = BUILD_CHECK_EDGE_MILEAGE_STRAIGHT_MILEAGE;
    Run_Mode = Normal_Mode;

    Count.Spd_Mileage = Count.Mileage;

    Build_Finish_Current_Action();
}

// (Record_Segment_Edge_Mileage removed — edge mileage now recorded inline in Build_Dispatch)

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
    // Load_Segment_Edge_Mileage_Record_From_Flash(); (FIXME: Flash format changed)
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
        Count.Finish++;
    }
    if (Count.Finish > 200 && Stop_Flag == 0)
    {
        Stop_Flag = 1;
        Save_Turn_Mileage_Record_To_Flash();
        // Save_Segment_Edge_Mileage_Record_To_Flash(); (FIXME: Flash format changed)
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
        Count.StartDelay = 800;
    }
    Last_EnableSwitch_ON = EnableSwitch_ON;

    Get_Speed();

    Get_IMU();

    Get_Light();

    Light_Process();

    Safety_Check();


    if (Stop_Flag != 0)
    {
        return;
    }


    if (EnableSwitch_ON)
    {
        Build_Mode_Get_Error();
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

    // Continuous total angle: always accumulates, corrected after each turn
    Total_Angle += Gyro_Z * GYRO_INTEGRATION_PERIOD_S;
    Total_Angle  = Normalize_Angle_180(Total_Angle);


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
**             On detection, resets Count.Mileage and increments Count.Edge.
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
        Count.Edge++;
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
    uint8_t sensor_index;


    for (int i = 0; i < 15; i++)
    {
        if (Light_ADC[i] > Light_Thr[i][0])
        {
            Light_Convert[i] = 1;
        }
        if (Light_ADC[i] < Light_Thr[i][1])
        {
            Light_Convert[i] = 0;
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


    if (EnableSwitch_ON)
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
        Count.Line = 0;
        Count.Element = 0;
        Build_Action_Index = 0;
        Build_Action_Active_Index = 0;
        Build_Load_Default_Action_List();                             // Load default build actions
        Turn_Mileage_Record_Num = 0;
        Last_Turn_Mileage_Base = 0;
        memset(Turn_Mileage_Record, 0, sizeof(Turn_Mileage_Record));
        memset(Segment_Edge_Mileage_Record, 0, sizeof(Segment_Edge_Mileage_Record));
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
        case Straight_Mode:
            Straight_Run();
            break;
        default:
            Normal_Run();
            break;
    }

    // single-point LED: green when normal tracing, blue when object detected
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
** Description: 2-phase turn (works for both node and element turns, distinguished by Count.Stall)
**   Phase 0: decelerate+advance — speed ramps Basic_Speed→0 linearly over Count.Stall ticks
**   Phase 1: in-place rotate to target angle with speed=0, Angle_PID + Gyro_PID cascade
**   Non-Build_Mode: immediate turn at full Basic_Speed
*************************************/
void Turn_Left_Run(void)
{

    if (Turn_Action_Done)
        return;

    if (Turn_Decel_Phase == 0)
    {
        // Phase 0: decel+advance — speed ∝ remaining distance
        Count.Left = Count.Mileage;  // accumulated from 0 since turn start
        float ratio = 1.0f - (Count.Left / Count.Stall);
        if (ratio < 0.0f) ratio = 0.0f;
        int speed = (int)(Basic_Speed * ratio);

        Error = 0;
        Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target, speed);

        if (Count.Left >= Count.Stall)
        {
            Turn_Decel_Phase = 1;
            Gyro_Integral = 0;
            Turn_Angle_Settle_Count = 0;
            PID_cleardata(&Angle_PID);
        }
    }
    else // Phase 1: in-place rotate
    {
        Error = 0;
        Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target, 0);
        if (Is_Turn_Angle_Settled(Turn_Angle_Target))
        {
            Complete_Turn_Action();
        }
    }
}

/*************************************
** Function: Turn_Right_Run
** Description: Execute a right turn (mirror of Turn_Left_Run) — same 2-phase logic
*************************************/
void Turn_Right_Run(void)
{
    if (Turn_Action_Done)
        return;

    if (Turn_Decel_Phase == 0)
    {
        Count.Right = Count.Mileage;  // accumulated from 0 since turn start
        float ratio = 1.0f - (Count.Right / Count.Stall);
        if (ratio < 0.0f) ratio = 0.0f;
        int speed = (int)(Basic_Speed * ratio);

        Error = 0;
        Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target, speed);

        if (Count.Right >= Count.Stall)
        {
            Turn_Decel_Phase = 1;
            Gyro_Integral = 0;
            Turn_Angle_Settle_Count = 0;
            PID_cleardata(&Angle_PID);
        }
    }
    else
    {
        Error = 0;
        Set_Mileage_Turn_Exp_Speed(Turn_Angle_Target, 0);
        if (Is_Turn_Angle_Settled(Turn_Angle_Target))
        {
            Complete_Turn_Action();
        }
    }
}
/*************************************
** Function: Set_Mileage_Turn_Exp_Speed
** Description: Angle PID cascade for turns — outer Angle_PID → inner Gyro_PID → wheel speeds
*************************************/

/*************************************
** Function: Set_Mileage_Turn_Exp_Speed
** Description: Calculate expected wheel speeds for an angle-based turn
**              using cascaded PID: outer angle PID -> inner gyro PID -> wheel speeds
** Input:      angle_target - Target gyro angle (deg, e.g. +/-90)
*************************************/
void Set_Mileage_Turn_Exp_Speed(float angle_target, int base_speed)
{
    float gyro_target;

    gyro_target = PID_calc(&Angle_PID, angle_target, Gyro_Integral);
    Gyro_PID_Out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);
    Left_Exp_Spd = base_speed + (int)Gyro_PID_Out;
    Right_Exp_Spd = base_speed - (int)Gyro_PID_Out;
}

/*************************************
** Function: Set_Speed
** Description: PID speed control — compute expected wheel speeds from tracking error
** Control Chain: Error -> Turn_PID -> Gyro_PID(+Gyro_Z damping) -> left/right speeds
** Details:    Either uses angle-based control (during turns) or normal trace control
**             with gyro rate as damping. Straight_Mode uses gyro-damped straight control.
**             Outputs Left_Exp_Spd and Right_Exp_Spd, and computes wheel speed PID outputs.
*************************************/
void Set_Speed()
{
    static uint8_t straight_enter = 0;   // one-shot clear on entry to Straight_Mode

    Left_PID_Out = 0;
    Right_PID_Out = 0;

    if (EnableSwitch_ON == 0)
    {
        Turn_PID_Out = 0;
        Gyro_PID_Out = 0;
        straight_enter = 0;
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
        straight_enter = 0;
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Turn_PID);
    }
    else if (Run_Mode == Straight_Mode)
    {
        if (!straight_enter)
        {
            straight_enter = 1;
            PID_cleardata(&Gyro_PID);
            PID_cleardata(&Turn_PID);
        }
        Turn_PID_Out = 0;
        Gyro_PID_Out = PID_calc(&Gyro_PID, 0.0f, Gyro_Z);
        Left_Exp_Spd = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }
    else
    {
        straight_enter = 0;

        // Normal trace: Angle_PID(Error) → Gyro_PID rate damping → wheel speeds
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, (float)Error);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);

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
** Details:    Handles start delay (Count.StartDelay) with suction motor on.
**             Sets suction motor PWM when enabled and not stopped.
**             Applies left/right motor direction and PWM based on sign of PID output:
**             positive = forward, negative = reverse.
*************************************/
void Set_Out(void)
{
    if (Count.StartDelay > 0)
    {
        Count.StartDelay--;


        pwm_set_duty(Suction_Motor_PWM, 9520);
        pwm_set_duty(Suction_Motor_DIR, 10000);
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
        pwm_set_duty(Suction_Motor_DIR, 10000);
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
    float section_mileage = Count.Mileage - Count.Spd_Mileage;

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
