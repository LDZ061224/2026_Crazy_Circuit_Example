/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.h
Author: Cross_Z
Version:0.0        Date: 2026.1.30
Description: Main control module header - track following and element recognition
             PID control, speed control, turn control and mode definitions
Others:   All run modes declared - Normal/Straight/Turn/Mileage/Debug
Function List: Car_Go / Get_Speed / Get_IMU / Light_Process / Set_Speed / Set_Out
              Normal_Run / Straight_Run / Turn_Left_Run / Turn_Right_Run
              Mileage_Mode_Run / Mileage_Run_Stage_2 / Build_Mode_Get_Error
              Safety_Check / Load_All_Flash_Data_For_VOFA
              Debug_Wheel_Tuning / Debug_Ground_Test / Debug_Angle_Tuning / Debug_Normal_Trace
History:
<author>    <time>       <version>    <desc>
Cross_Z     2026.1.30      0.0         Initial creation
**************************************************/

#ifndef __CTRL_H
#define __CTRL_H

// Common headers: MCU peripheral + project-specific headfiles
#include "zf_common_headfile.h"
#include "headfiles.h"

/**********************************************
* Numeric Constants
**********************************************/
#define NODE_NUM_MAX            20
#define ELEMENT_NUM_MAX         5
#define TRACK_SEGMENT_NUM_MAX   (NODE_NUM_MAX + 1)
#define BUILD_ACTION_MAX        (NODE_NUM_MAX + (TRACK_SEGMENT_NUM_MAX * ELEMENT_NUM_MAX))
#define TURN_MILEAGE_RECORD_MAX 120
#define DEBUG_ANGLE_STEP_TICKS 667U          // Angle debug: 90-degree target changes every 2s

// Build mode default track constants
#define BUILD_NODE_NUM          17
#define BUILD_ACTION_COUNT      31

#define SAFETY_LOW_VOLTAGE_THRESHOLD    11.3f

// Gyro rate incremental PID — used in normal trace and turn inner loop
#define GYRO_PID { \
    .kp         = 0.16, \
    .ki         = 0.006, \
    .kd         = 0.075, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_ADD \
}

// Angle position PD with derivative on measurement — outer loop for turns
#define ANGLE_PID { \
    .kp         = 19.5, \
    .ki         = 0, \
    .kd         = 9, \
    .iOutMax    = 0, \
    .outMax     = 1500, \
    .mode       = PID_MODE_POSITION_D_ON_MEASUREMENT \
}

// Gyro position PD — for normal trace debug
#define GYRO_PD_PID { \
    .kp         = 0.008, \
    .ki         = 0, \
    .kd         = 0.002, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_POSITION \
}

// Left wheel speed incremental PID
#define LEFT_PID { \
    .kp         = 150, \
    .ki         = 1.2, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 9500, \
    .mode       = PID_MODE_ADD \
}

// Right wheel speed incremental PID
#define RIGHT_PID { \
    .kp         = 150, \
    .ki         = 2.1, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 9500, \
    .mode       = PID_MODE_ADD \
}

// Turn error position PID (outer loop for normal trace steering)
#define TURN_PID { \
    .kp         = 80, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 1200, \
    .mode       = PID_MODE_POSITION \
}

/**********************************************
* Mode and State Enumerations
**********************************************/

/**
 * @brief Run mode enumeration — main operating state machine
 */
typedef enum
{
    Normal_Mode,     // Normal line-following trace
    Turn_Left,       // Left turn mode
    Turn_Right,      // Right turn mode
    Mileage_Mode,    // Mileage-based element execution mode
    Straight_Mode,   // Straight line mode
} Run_Mode_Enum;

/**
 * @brief Mileage stage enumeration within Mileage_Mode
 */
typedef enum
{
    Normal_Stage,    // Normal stage
    Straight_Stage,  // Straight stage
} Mileage_Stage_Enum;

/**
 * @brief Global operating mode
 */
typedef enum
{
    Build_Mode,      // Build mode (autonomous track traversal)
    Debug_Mode,      // Debug mode (manual parameter tuning)
} Mode_Define;

// Build action type — defines what the car does at each track node/element
typedef enum
{
    BUILD_ACTION_NONE              = 0,  // unused / placeholder
    BUILD_ACTION_NODE_STRAIGHT     = 1,  // node -> straight mode
    BUILD_ACTION_NODE_TURN_LEFT    = 2,  // node -> left turn
    BUILD_ACTION_NODE_TURN_RIGHT   = 3,  // node -> right turn
    BUILD_ACTION_ELEM_STRAIGHT_SHORT = 4,  // element -> short straight (mileage mode, dir=3)
    BUILD_ACTION_ELEM_STRAIGHT_LONG  = 5,  // element -> long straight (mileage mode, dir=4)
    BUILD_ACTION_ELEM_TURN_LEFT    = 6,  // element -> left turn (mileage mode, dir=1)
    BUILD_ACTION_ELEM_TURN_RIGHT   = 7,  // element -> right turn (mileage mode, dir=2)
} Build_Action_Enum;

// Helper: action value 1-3 = node action, 4-7 = element action
#define BUILD_ACTION_IS_NODE(a)     ((a) >= BUILD_ACTION_NODE_STRAIGHT && (a) <= BUILD_ACTION_NODE_TURN_RIGHT)
#define BUILD_ACTION_IS_ELEMENT(a)  ((a) >= BUILD_ACTION_ELEM_STRAIGHT_SHORT && (a) <= BUILD_ACTION_ELEM_TURN_RIGHT)

/**
 * @brief Debug sub-mode enumeration
 *        Debug_Which_Wheel: 0=left wheel, 1=right wheel
 */
typedef enum
{
    Debug_Sub_PI_Tuning,     // Single-wheel PI tuning — uses Debug_Which_Wheel to select wheel
    Debug_Sub_Ground_Test,   // Ground/floor test mode
    Debug_Sub_Angle,         // Angle PID tuning with incremental step target
    Debug_Sub_NormalTrace,   // Normal trace debug with gyro PD
} Debug_Sub_Mode_Enum;

/**********************************************
* Runtime Data Structures
**********************************************/

/**
 * @brief Odometer/counter struct — encoder ticks and stall detection
 */
typedef struct
{
    int     Left;        // Left encoder count (per-cycle)
    int     Right;       // Right encoder count (per-cycle)
    int     Stop;        // Stall counter (consecutive all-on/all-off cycles)
    float   Mileage;     // Resettable mileage (encoder ticks, reset on edge)
    float   Spd_Mileage; // Speed-weighted mileage accumulator
    int     Straight;    // Consecutive straight-valid cycle counter
    int     Stall;       // Motor stall counter
} Count_Typedef;

// Build action descriptor: links an action type to a track segment and element
typedef struct
{
    Build_Action_Enum        action;
    uint8_t                  segment_index;
    uint8_t                  element_index;
} Build_Action_Typedef;

/**********************************************
* Global Variable Extern Declarations
**********************************************/
// Base speed
extern int Basic_Speed;

// Expected/target wheel speeds
extern int Left_Exp_Spd;
extern int Right_Exp_Spd;
extern int Middle;
// Actual measured wheel speeds
extern int Left_Real_Spd;
extern int Right_Real_Spd;

// Speed average
extern float Average_Speed;

// Track detection variables
extern int Error;
extern int   Track_Arr[15];
extern int16_t Dir_Arr[15];
extern int   Left_Scan_Point;
extern int   Right_Scan_Point;
extern int   Last_Error;

// Light sensor raw ADC and binary conversion
extern uint16 Light_ADC[15];
extern uint8  Light_Convert[15];

// PID output values
extern float Turn_PID_Out;
extern float Gyro_PID_Out;
extern float Left_PID_Out;
extern float Right_PID_Out;

// Stop and finish flags
extern int  Stop_Flag;          // Emergency stop flag
extern int  Finish_Flag;        // Finish flag (all actions completed)
extern int  Finish_Count;       // Finish confirmation counter
extern int  Track_Num;          // Number of sensors seeing the track line

// Segment and element counters
extern int8_t  Execute_Times;   // Current segment index (from action dispatch)
extern int8_t  Mileage_Times;   // Element count in current segment (from Mileage_Num_By_Segment)

// Lane and line element counters
extern uint8_t Line_Num_Count;    // Completed lane (full segment) counter
extern uint8_t In_Line_Ele_Count; // Current element counter within the lane
extern uint8_t Build_Action_Index;
extern uint8_t Build_Action_Count;

// Gyro angle integral for turn control
extern float Gyro_Integral;
// Turn target angle: left turn = -90, right turn = +90
extern float  Turn_Angle_Target;

// Build mode mileage tracking
extern int16  Check_Edge_Count;    // Check_Edge call count for diagnostics
extern float  Mileage_Element_Turn_Delay;
extern float  Mileage_Node_Turn_Delay;
extern uint8  Current_Element_Dir;      // Element direction of currently executing action (for Mileage_Mode_Run)
extern uint8  vofa_flash_dump_mode;     // VOFA Flash dump mode flag (OLED toggle)
extern float  Total_Run_Mileage;       // Total accumulated run mileage (never reset, for Flash records)
// Counters struct
extern Count_Typedef Count;
extern Mode_Define Mode;
extern float Gyro_Z_For_PID; // Gyro Z-axis angular velocity used by PID
extern float gyro_z_offset;  // Gyro Z-axis zero-drift (sampled at power-on calibration)
// Run mode and mileage stage
extern Run_Mode_Enum       Run_Mode;
extern Mileage_Stage_Enum  Mileage_Stage;
extern Build_Action_Typedef Build_Action_List[BUILD_ACTION_MAX];
extern const Build_Action_Typedef Default_Build_Actions[BUILD_ACTION_COUNT];
extern const uint8 Mileage_Num_By_Segment[BUILD_NODE_NUM + 1];

// Flash mileage records for VOFA display
extern float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];
extern float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];
extern float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX];
extern uint16_t Turn_Mileage_Record_Num;

// PID controller handles
extern PID_HandleTypeDef Gyro_PID;
extern PID_HandleTypeDef Left_PID;
extern PID_HandleTypeDef Right_PID;
extern PID_HandleTypeDef Turn_PID;
extern PID_HandleTypeDef Angle_PID;
extern PID_HandleTypeDef Gyro_PD_PID;

// Debug mode variables
extern Debug_Sub_Mode_Enum Debug_Sub_Mode;  // Current debug sub-mode
extern uint8  Debug_Motor_Enable;           // Debug mode motor enable flag: 0=disabled, 1=enabled
extern uint8  Debug_Which_Wheel;            // Debug wheel selection: 0=left wheel, 1=right wheel
extern int    Debug_Target_Speed;           // Debug target speed
extern int    Debug_Fan_Duty;               // Debug ground test fan duty cycle (suction motor)
extern uint8  Debug_Ground_Dir;             // 1=forward ground test, 2=reverse ground test
extern uint8  Debug_Angle_Mode;             // 1=sin target, 2=step angle, 3=direct gyro rate (AVT)
extern uint8  Debug_Angle_D_First;          // 0=error D, 1=measurement D
extern float  Debug_Angle_Vel_Target;       // VOFA: target angular velocity
extern uint8_t g_led_flag;                // 0=green(normal) 1=blue(object) 2=yellow(low voltage)
extern uint8_t g_scan_progress;            // Scan progress 0-100, 0=not scanning
extern float  Debug_Angle_Vel_Real;         // VOFA: actual angular velocity
extern float  Debug_Kp_Left;               // Left wheel debug Kp
extern float  Debug_Ki_Left;               // Left wheel debug Ki
extern float  Debug_Kp_Right;              // Right wheel debug Kp
extern float  Debug_Ki_Right;              // Right wheel debug Ki

/**********************************************
* Public Function Declarations
**********************************************/
void Car_Go(void);                          // Main control loop entry: called every 3ms from ISR
void Light_Process(void);
void Set_Speed(void);                       // Compute expected wheel speeds from tracking error
void Get_Speed(void);                       // Read encoder counts and compute filtered actual speeds
uint8 Check_Edge(void);                      // Edge detection for segment/element boundaries
void Get_IMU(void);                         // Read and process IMU gyro data
void Set_Out(void);                         // Apply PID output to motor PWM

void Normal_Run(void);                      // Normal line-following trace
void Straight_Run(void);                    // Straight line mode execution
void Turn_Left_Run(void);                   // Left turn execution
void Turn_Right_Run(void);                  // Right turn execution

void Mileage_Mode_Run(void);                // Mileage-based element mode dispatcher
void Mileage_Run_Stage_2(void);             // Mileage run stage 2 — turn element execution

void Safety_Check(void);                    // Safety monitoring: voltage, stall, finish detection; motor cutoff and LED indication
void Build_Mode_Get_Error(void);            // Build mode: get tracking error and dispatch run mode
void Load_All_Flash_Data_For_VOFA(void);    // VOFA: load all Flash mileage records for visualization

// Debug mode control functions
void Debug_Wheel_Tuning(void);              // Single-wheel PI tuning for left/right wheel
void Debug_Ground_Test(void);               // Ground test mode
void Debug_Angle_Tuning(void);              // Angle PID tuning with incremental step target
void Debug_Normal_Trace(void);              // Normal trace debug with PD control
void Debug_Set_Out(void);                   // Debug mode PWM output

void Set_Mileage_Turn_Exp_Speed(float angle_target);

#endif
