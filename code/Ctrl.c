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
int Basic_Speed = 45;   // TODO: hardcoded, restore flash read after tuning
int Run_Speed = 0;
float Average_Speed = 0;

Mode_Define Mode = Build_Mode;  // Runtime mode (overridden in cpu0_main.c at startup)
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
PID_HandleTypeDef Turn_PID = TURN_PID;   // Angle PD with derivative on measurement
PID_HandleTypeDef Left_PID = LEFT_PID;

PID_HandleTypeDef Right_PID = RIGHT_PID;
PID_HandleTypeDef Angle_PID = ANGLE_PID;     // heading hold (straight + full angle correction)

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
uint8_t g_led_flag = 0;                        // 0=green(normal) 1=blue(object) 2=purple(low voltage)
uint8_t g_scan_progress = 0;                  // Scan progress 0-100 (0=not scanning)
int Middle = 0;
float Gyro_Integral = 0;
float Total_Angle = 0;                         // Continuous gyro angle, corrected after each turn
// total_left/right_turns removed — Total_Angle is zeroed after each turn instead
// total_left/right_turns removed — Total_Angle is zeroed after each turn instead
float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX] = {{0}};
float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX] = {0};
float Total_Run_Mileage = 0;

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
int    Debug_Fan_Duty = 500;
uint8  Debug_Ground_Dir = 1;
uint8  Debug_Angle_Mode = 2;                         // 1=sine rate, 2=step angle, 3=direct gyro rate
uint8  Debug_Angle_D_First = 0;                      // 0=error D, 1=measurement D
float  Debug_Angle_Vel_Target = 0.0f;
float  Debug_Angle_Vel_Real = 0.0f;
uint8  Debug_Ground_FF_Mode = 0;                    // 地面测试: 0=纯PI, 1=速度前馈+PI修正
uint8  Debug_Gyro_FF_Mode = 0;                      // 角速度: 0=纯PID, 1=前馈+PID修正


/*--------------- Build Mode Tuning (see Ctrl.h TUNE_*) ---------------*/
// All TUNE_* macros defined in Ctrl.h

uint8 vofa_flash_dump_mode = 0;

// Max consecutive cycles with all sensors on or all off before emergency stop
#define SAFETY_STOP_CYCLE_MAX         80

// (GYRO_INTEGRATION_PERIOD_S moved to Ctrl.h)

// Normalize an angle to [-180, 180] degrees
static inline float Normalize_Angle_180(float angle)
{
    // while (angle > 180.0f)  angle -= 360.0f;
    // while (angle < -180.0f) angle += 360.0f;
    // while (angle > 180.0f)  angle -= 360.0f;
    // while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/*--------------- Flash Storage Layout ---------------*/
/*
* Flash sector and page layout:
* Corresponds to BUILD_MAP_FLASH_START_PAGE(4) in OLEDKeyboard.c
 */

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
    float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];
    float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];
} Segment_Edge_Mileage_Flash_Typedef;


Count_Typedef Count =
{
    .Left        = 0,
    .Right       = 0,
    .Stop        = 0,
    .Mileage     = 0,
    .StraightBase = 0,
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
static uint8_t Is_Track_Sensor_Adjacent(uint8_t left_index, uint8_t right_index);

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

// (Build-only helper functions moved to Ctrl_Build.c)

/********************************* Core Run-Time Functions *********************************/

/*************************************
** Function: Safety_Check
** Description: Safety monitoring — low voltage, stall detection, finish detection, LED indication
** Details:    1. Low voltage: if Voltage_Check[0] stays below TUNE_SAFE_VOLTAGE
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
    if (Voltage_Check[0] < TUNE_SAFE_VOLTAGE)
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
        // Save_Segment_Edge_Mileage_Record_To_Flash(); (FIXME: Flash format changed)
    }

    // LED priority: yellow(voltage) > blue(beep) > green(normal)
    if (Stop_Flag != 0)
    {
        pwm_set_duty(Suction_Motor_PWM, 9500);
        pwm_set_duty(Suction_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_DIR, 10000);
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
            g_led_flag = 2;  // purple: low voltage warning
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
#if ACTIVE_MODE == MODE_BUILD
        Build_Mode_Get_Error();
#elif ACTIVE_MODE == MODE_REMEMBER
        Remember_Mode_Get_Error();
#endif
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
#if ACTIVE_MODE == MODE_BUILD || ACTIVE_MODE == MODE_REMEMBER
        // Exclude in-place rotation mileage (Build Phase1 / Remember right-angle turns)
        if (!((Mode == Remember_Mode && (is_left == 1 || is_right == 1))
           || (Mode == Build_Mode && Turn_Decel_Phase == 1)))
        {
            Total_Run_Mileage += instant_speed;
        }
#else
        Total_Run_Mileage += instant_speed;
#endif
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
    // Mileage-based cooldown (Build mode only)
#if ACTIVE_MODE == MODE_BUILD
    if (Check_Edge_Skip_Thresh > 0)
    {
        if ((Count.Mileage - Check_Edge_Skip_Mileage_Base) < Check_Edge_Skip_Thresh)
            return 0;
        Check_Edge_Skip_Thresh = 0;
    }
#endif


    if (((Light_Convert[0] == 1 || Light_Convert[14] == 1)&&
        Initial_White_Num >= 4) || Initial_White_Num >= 5)
    {
        Count.Edge++;
#if ACTIVE_MODE == MODE_BUILD
        Count.Last_Edge_Mileage = Count.Mileage;  // snapshot before zero — used by Build_Dispatch_Current_Action
#endif
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

// (Mode-specific functions removed — Build in Ctrl_Build.c, Remember in Ctrl_Remember.c)
// (Set_Mileage_Turn_Exp_Speed removed — moved to Ctrl_Build.c)

/*************************************
** Function: Set_Speed
** Description: PID speed control — compute expected wheel speeds from tracking error
** Control Chain: Error -> Angle_PID -> Gyro_PID(+Gyro_Z damping) -> left/right speeds
** Control Chain: Error -> Angle_PID -> Gyro_PID(+Gyro_Z damping) -> left/right speeds
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
        PID_cleardata(&Gyro_PID);
        PID_cleardata(&Gyro_PD_PID);
        return;
    }

#if ACTIVE_MODE == MODE_BUILD
    Run_Speed = Basic_Speed;
    if (Run_Speed < 0) Run_Speed = 0;

    if (is_left == 1 || is_right == 1)
    {
        straight_enter = 0;
        // PID cleared once in Set_Node_Run_Mode / Phase transitions — do NOT clear per-tick
    }
    else if (Run_Mode == Straight_Mode)
    {
        if (!straight_enter)
        {
            straight_enter = 1;
            PID_cleardata(&Angle_PID);
            PID_cleardata(&Gyro_PID);
        }
        // Cascaded heading correction: angle → gyro rate → wheel speeds
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, Total_Angle);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        Left_Exp_Spd  = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }
    else
    {
        straight_enter = 0;
        // Normal trace: Turn_PID(Error) → Gyro_PID rate damping → wheel speeds
        Turn_PID_Out = PID_calc(&Angle_PID, 0.0f, (float)Error);
        Gyro_PID_Out = PID_calc(&Gyro_PID, Turn_PID_Out, Gyro_Z);
        Left_Exp_Spd = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }
#elif ACTIVE_MODE == MODE_REMEMBER
    Remember_Set_Speed();
#endif


    if (EnableSwitch_ON)
    {
        Average_Speed = (Left_Real_Spd + Right_Real_Spd) / 2.0;
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


        pwm_set_duty(Suction_Motor_PWM, 500);
        pwm_set_duty(Suction_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_PWM, 0);
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);

        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        return;
    }

    if (EnableSwitch_ON && Stop_Flag == 0)
    {
        pwm_set_duty(Suction_Motor_PWM, 500);
        pwm_set_duty(Suction_Motor_DIR, 10000);
    }
    else
    {
        pwm_set_duty(Suction_Motor_PWM, 500);
        pwm_set_duty(Suction_Motor_DIR, 10000);
    }


    if (EnableSwitch_ON && Stop_Flag == 0)
    {

        if (Left_PID_Out == 0)
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


        if (Right_PID_Out == 0)
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
    else
    {
        pwm_set_duty(Left_Motor_DIR, 10000);
        pwm_set_duty(Left_Motor_PWM, 0);
        pwm_set_duty(Right_Motor_DIR, 0);
        pwm_set_duty(Right_Motor_PWM, 0);

        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
    }
}

// Straight_Run removed — moved to Ctrl_Build.c
