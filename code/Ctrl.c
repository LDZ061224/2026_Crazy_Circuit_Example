/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.c
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description:  智能车核心控制程序
              - 光电传感器寻迹 + 编码器里程计 + 陀螺仪姿态控制
              - PID 闭环速度/转向控制
              - 建图模式(Build_Mode)：记录赛道转向里程到Flash
Others:      基于3ms定时中断调用 Car_Go() 主循环
Function List:
              主循环：Car_Go / Get_Speed / Get_IMU / Light_Process / Set_Speed / Set_Out
              寻迹：  Normal_Run / Straight_Run / Turn_Left_Run / Turn_Right_Run
              里程：  Mileage_Mode_Run / Mileage_Run_Stage_2
              建图：  Build_Mode_Get_Error
              存储：  Save_Turn_Mileage / Load_Turn_Mileage / Save_Segment_Edge / Load_Segment_Edge
History:
<author>  <time>      <version > <desc>
Cross_Z   2026.1.30    0.0        创建初始版本
**************************************************/

#include "Ctrl.h"   // 核心控制头文件（PID参数宏、运行模式枚举、赛道结构体、全局变量声明）

/********************************* 全局变量定义 *********************************/

/*---------------电机速度---------------*/
int16 giSpeed_Left[3] = {0};   // 左电机速度采样环形缓冲区
int16 giSpeed_Right[3] = {0};  // 右电机速度采样环形缓冲区
int Left_Real_Spd = 0;         // 左电机实际速度（6ms 3-tap FIR滤波值，用于速度环PID/里程/堵转）
int Right_Real_Spd = 0;        // 右电机实际速度（6ms 3-tap FIR滤波值，用于速度环PID/里程/堵转）

int Left_Exp_Spd = 0;          // 左电机期望速度（PID目标值）
int Right_Exp_Spd = 0;         // 右电机期望速度（PID目标值）
int Basic_Speed = 0;           // 基础速度设定值（由键显Flash加载）
int Run_Speed = 0;             // 当前运行速度（由 Select_Run_Speed 统一选择）
float Average_Speed = 0;       // 当前左右轮平均速度
int Speed_Get_Count = 1;       // 速度采集分频计数：每次 Car_Go 取反，==1时采集速度（6ms一次）
uint8 First_Mode = 0;          // 建图模式首次运行标志：0=未初始化，1=已完成初始化

/*---------------传感器数据----------------*/
uint8 Light_Convert[15] = {0};       // 15路光敏传感器二值化结果：0=黑线, 1=白线
uint8 Last_Light_Convert[15] = {0};  // 上一周期15路光敏传感器值

/*-----------------PID控制----------------*/
float Gyro_Z = 0;                          // 陀螺仪Z轴角速度（度/秒）
float Gyro_Z_For_PID = 0;                 // PID用陀螺仪值（raw/1000，与原工程一致）
PID_HandleTypeDef Gyro_PID = GYRO_PID;     // 陀螺仪PID实例（增量式PID, kp=0.008, 限幅±500）
PID_HandleTypeDef Gyro_PD_PID = GYRO_PD_PID; // Gyro rate incremental PD for trace/sine debug
PID_HandleTypeDef Angle_PID = ANGLE_PID;   // Angle PD with derivative on measurement
PID_HandleTypeDef Left_PID = LEFT_PID;     // 左电机PID实例（kp=0,ki=0,kd=0, 增量式, 限幅±9500）
                                           //  注：左电机kp/ki/kd=0，实际跟随右电机差值控制
PID_HandleTypeDef Right_PID = RIGHT_PID;   // 右电机PID实例（kp=250,ki=65,kd=0, 增量式, 限幅±9500）
PID_HandleTypeDef Turn_PID = TURN_PID;     // 转向PID实例（kp=80, 增量式, 限幅±10000）

/*----------------寻迹控制----------------*/
int Left_Scan_Point = 0;           // 左边界寻迹点（Track_Arr[0]传感器索引，对应赛道左边缘）
int Right_Scan_Point = 0;          // 右边界寻迹点（Track_Arr[Track_Num-1]传感器索引，对应赛道右边缘）
int Error = 0;                     // 寻迹偏差值（输入 Turn_PID + Gyro_PID，正值偏右/负值偏左）
int Last_Error = 0;                // 上一周期寻迹偏差（传感器丢失时用于保持方向）
int Track_Arr[15] = {0};           // 寻迹有效传感器索引数组（按从小到大排列白线传感器编号0~14）
int Last_Track_Arr[15] = {0};      // 上一周期寻迹有效传感器索引数组（用于异常回退）
int Initial_White_Num = 0;         // 初始白色区域数量（全部15路中检测为白色的传感器总数）
int Track_Num = 0;                 // 有效寻迹传感器数量（连续白线区域内的传感器个数）
int Last_Track_Num = 0;            // 上一周期有效寻迹传感器数量（用于异常回退）
int Stop_Flag = 0;                 // 停车标志：0=运行, 1=停车（触发条件：全白/全黑超时 或 任务完成）
int Finish_Count = 0;              // 任务完成计数（Finish_Flag=1后累计，>200触发停车）
int Finish_Flag = 0;               // 任务完成标志：0=未完成, 1=已完成（触发条件在Finish_Mileage_Section中）
uint8_t Left_Num = 0;              // 左侧传感器有效计数（转弯时统计传感器索引>7且有效的数量）
uint8_t Right_Num = 0;             // 右侧传感器有效计数（转弯时统计传感器索引<=7且有效的数量）
uint8_t Left_Flag = 0;             // 左转向标志
uint8_t Right_Flag = 0;            // 右转向标志
uint8_t is_left = 0;               // 正在执行左转动作标志：0=非左转, 1=左转中（控制陀螺仪积分和速度分配）
uint8_t is_right = 0;              // 正在执行右转动作标志：0=非右转, 1=右转中（控制陀螺仪积分和速度分配）
float  Turn_Angle_Target = 0;      // 转弯目标角度（°）：左转+90, 右转-90, 角度闭环PID用
float  Turn_Angle_Last_Real = 0;   // Last angle feedback for angle PD derivative
uint8_t Turn_Angle_D_First = 1;    // 0=error微分, 1=measurement微分（对Gyro_Integral微分，避免阶跃冲击）
uint8_t Turn_Decel_Phase = 0;      // 转弯减速阶段（Mileage_Run_Stage_2 里程模式转弯元件使用）
uint8_t Mileage_Turn_Done = 0;     // 里程计转向完成标志：0=转向中, 1=转向完成（里程模式下陀螺仪转角达标）
uint8_t Turn_Action_Done = 0;      // 转向动作完成标志：0=未完成, 1=已完成（传感器模式下出弯条件满足）
int Check_Edge_Skip_Count = 0;     // 边缘检测跳过计数（转向/里程切换后置20，逐周期递减，防误触发）（转向/里程切换后置20，逐周期递减，防误触发）
int Enable_Start_Delay_Count = 0;  // 启动延时计数（EnableSwitch_ON上升沿置100，递减期间电机锁定）
uint8_t Last_EnableSwitch_ON = 0;  // 上一周期使能开关状态（用于检测上升沿触发启动延时）
int Middle = 0;                    // 寻迹中心线位置（Track_Arr首尾传感器索引的平均值，0~14，理想值=7）
float Gyro_Integral = 0;           // 陀螺仪积分角度（转弯时累计角速度*3ms，出弯清零，单位：度）
float Mileage_Element_Base = 0;    // 里程计基础基准值
float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX] = {{0}}; // 各段各元素边缘里程记录
float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX] = {0};                      // 各段路实测总里程（段起始到接触下一节点的距离）
float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX] = {0};                 // 转向间距里程记录数组（元素=两次转弯之间的里程差）
uint16_t Turn_Mileage_Record_Num = 0;   // 转向里程记录有效数量（建图时递增，回放时从Flash加载）
float Total_Run_Mileage = 0;            // 总运行里程（从发车开始累计，不重置，用于回放模式里程对比）
float Last_Turn_Mileage_Base = 0;       // 上一次转向里程基准（计算Turn_Mileage_Record[i] = Turn_Begin_Mileage - Last_Turn_Mileage_Base）
float Turn_Begin_Mileage = 0;           // 转向开始时的Total_Run_Mileage（进入转弯时记录）

/*---------------方向映射表----------------*/
// Dir_Arr[传感器索引] = 该传感器偏离中心的权重值
// 索引0(最左): -22, 索引7(中心): 0, 索引14(最右): +22
// 非对称设计：左侧权重大于右侧（左侧-22~-2 vs 右侧+2~+22），用于补偿机械不对称
int8_t Dir_Arr[15] = {18, 16, 13, 9, 6, 3, 1, 0, -1, -3, -6, -9, -13, -16, -18};
int16 Check_Edge_Count = 0;   // Check_Edge触发次数（VOFA调试用）
uint8_t Force_Straight_Speed = 0;  // 直行元器件强制基础速度标志

/*---------------前瞻光电布局----------------*/
// Single-row 15-sensor tracking: all indexes below participate in Track_Num.
#define TRACK_SENSOR_ACTIVE_NUM 15
static const uint8_t Track_Sensor_Active_Index[TRACK_SENSOR_ACTIVE_NUM] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

/*-------------赛道运行状态结构体-------------*/
Racing_track_Typedef Run_Track;  // 当前赛道运行结构体（运行时数据，由键显输入或Flash加载填充）
int8_t Execute_Times = 0;        // 当前执行节点索引（0 ~ Node_Num，指向 Run_Track 的 Node_Arr_* 数组）
int8_t Mileage_Times = 0;        // 当前节点里程段总数（= Run_Track.Node_Arr_Mileage_Num[Execute_Times]）
uint8_t Line_Num_Count = 0;      // 已完成线路计数（累计通过的节点数，用于Stop_Mode==1时判断完成）
uint8_t In_Line_Ele_Count = 0;   // 当前线路内元素索引（0 ~ Mileage_Times-1，遍历每个节点的里程段）
Build_Action_Typedef Build_Action_List[BUILD_ACTION_MAX] = {0};
uint8_t Build_Action_Index = 0;
uint8_t Build_Action_Count = 0;
static uint8_t Build_Action_Active_Index = 0;

/*----------------PID输出----------------*/
float Turn_PID_Out = 0.0;    // 转向PID输出（Error → PID → 用于差速控制的基础值）
float Gyro_PID_Out = 0.0;    // 陀螺仪PID输出（Turn_PID_Out + Gyro_Z系数 → PID → 最终左右轮差速量）
float Left_PID_Out = 0.0;    // 左电机PID输出（期望速度 vs 实际速度 → PWM占空比调节量）
float Right_PID_Out = 0.0;   // 右电机PID输出（期望速度 vs 实际速度 → PWM占空比调节量）
// 运行模式
Run_Mode_Enum Run_Mode = Normal_Mode;         // 当前运行模式
Run_Mode_Enum Last_Run_Mode = Normal_Mode;     // 上一周期运行模式
Mileage_Stage_Enum Mileage_Stage = Normal_Stage; // 里程计运行阶段（Normal_Stage=正常, Straight_Stage=直行）
Mode_Define Mode = Build_Mode;              // 键盘显示工作模式（Build_Mode=建图, Debug_Mode=调试）

/*---------------调试模式全局变量----------------*/
Debug_Sub_Mode_Enum Debug_Sub_Mode = Debug_Sub_PI_Tuning;  // 当前子模式
uint8  Debug_Motor_Enable = 0;                         // 0=停转, 1=运行
uint8  Debug_Which_Wheel = 0;                          // 0=左轮, 1=右轮
int    Debug_Target_Speed = 40;                        // 目标速度
int    Debug_Fan_Duty = 2000;                          // 下地测试负压风扇占空比
uint8  Debug_Ground_Dir = 1;                         // 1=左正右反, 2=左反右正
uint8  Debug_Angle_Mode = 1;                         // 1=sin target, 2=step target
uint8  Debug_Angle_D_First = 0;                      // 0=error D, 1=measurement D
float  Debug_Angle_Vel_Target = 0.0f;                // VOFA: 角速度目标值
float  Debug_Angle_Vel_Real = 0.0f;                  // VOFA: 实际角速度
float  Debug_Kp_Left  = 250.0f;                        // 左轮Kp
float  Debug_Ki_Left  = 65.0f;                         // 左轮Ki
float  Debug_Kp_Right = 250.0f;                        // 右轮Kp
float  Debug_Ki_Right = 65.0f;                         // 右轮Ki

/*---------------转向控制参数（陀螺仪单位：真实 °/s 和 °）----------------*/
// IMU660RB: ±2000dps量程, transition_factor=14.3, 即 raw/14.3 = °/s
// Gyro_Integral = Σ(°/s × 0.003s) = 真实角度(°)
// Gyro_Z = imu660rb_gyro_transition(raw) = 真实 °/s
#define BUILD_TURN_TARGET_ANGLE_DEG    90.0f   // 里程模式转弯元件目标角度（建图）
#define TURN_ANGLE_SETTLE_ERROR_DEG    5.0f    // 转弯退出角度误差窗口
#define TURN_GYRO_SETTLE_RATE_DPS      45.0f   // 转弯退出角速度窗口
#define TURN_SETTLE_CYCLE_MIN          3       // 连续满足退出条件的3ms周期数

float Mileage_Element_Turn_Delay = 560.0f;          // Element turn pre-straight distance, OLED/Flash adjustable
float Mileage_Node_Turn_Delay = 260.0f;             // Node turn pre-straight distance, OLED/Flash adjustable
uint8 vofa_flash_dump_mode = 0;                     // VOFA Flash数据导出模式：0=正常, 1=导出

#define MILEAGE_COMPENSATION_X (-100.0f)  // 转弯元素里程补偿值（建图记录时减去此值，回放时提前触发）
#define MILEAGE_STRAIGHT_SHORT 900.0f    // 短直行元素里程
#define MILEAGE_STRAIGHT_LONG  0.0f    // 长直行元素里程

// Build模式去抖（可调参）
#define BUILD_CHECK_EDGE_NODE_TURN       25  // 节点转弯后
#define BUILD_CHECK_EDGE_MILEAGE_STRAIGHT 15 // 里程直行元器件后
#define BUILD_CHECK_EDGE_MILEAGE_TURN    100  // 里程转弯元器件后

#define SAFETY_STOP_CYCLE_MAX         80   // 堵转容忍周期数（3ms*100=300ms）

#define GYRO_INTEGRATION_PERIOD_S 0.003f // 陀螺仪积分周期（=3ms中断周期，Gyro_Z(°/s) × 0.003s = 本次角度增量(°)）
#define NORMAL_GYRO_OUT_STEP_MAX 12.0f   // Normal trace steering slew limit per 3ms tick
#define DEBUG_ANGLE_STEP_TICKS 667U      // 角度调试模式2：目标角度每2s变化一次（667×3ms≈2.001s）

/*---------------Flash存储分区定义----------------*/
/*
 * 转向里程Flash存储配置
 * 扇区0, 页5~7, 共3页(每页64个uint32_t)，存储 Turn_Mileage_Record 数组
 * 结构：{ Turn_Mileage_Record_Num(uint16), Turn_Mileage_Record[120](float) }
 * 注：与 OLEDKeyboard.c 中的 BUILD_MAP_FLASH_START_PAGE(4) 相邻但不重叠
 */
#define TURN_MILEAGE_FLASH_SECTOR 0              // Flash扇区编号
#define TURN_MILEAGE_FLASH_START_PAGE 5          // 起始页号
#define TURN_MILEAGE_FLASH_PAGE_COUNT 3          // 占用页数（3×64×4=768字节）
#define TURN_MILEAGE_FLASH_WORDS_PER_PAGE 64     // 每页uint32_t数量

/*
 * 路段边缘里程Flash存储配置
 * 扇区0, 页8~9, 共2页，存储 Segment_Edge_Mileage_Record 二维数组
 * 数据量：NODE_NUM_MAX(20) × ELEMENT_NUM_MAX(5) × sizeof(float)(4) = 400字节
 */
#define SEGMENT_EDGE_MILEAGE_FLASH_SECTOR 0              // Flash扇区编号
#define SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE 8          // 起始页号
#define SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT 2          // 占用页数
#define SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE 64     // 每页uint32_t数量

/*---------------Flash存储结构体----------------*/
// 转向里程Flash存储结构体（与Flash页面对齐的打包格式）
typedef struct
{
    uint16_t Turn_Mileage_Record_Num;                           // 有效转向里程记录数量
    float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX];         // 转向间距里程数组
}Turn_Mileage_Flash_Typedef;

// 路段边缘里程Flash存储结构体
typedef struct
{
    float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX]; // 边缘里程二维数组
    float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];                    // 各段路实测总里程
}Segment_Edge_Mileage_Flash_Typedef;

// 通用计数结构体实例（编码器/停止/里程/直行计数统一管理）
Count_Typedef Count =
{
    .Left = 0,       // 左转出弯计数（连续满足出弯条件次数）
    .Right = 0,      // 右转出弯计数
    .Stop = 0,       // 停车计数（全白/全黑持续周期数）
    .Mileage = 0,    // 当前段里程（进入新路段时清零）
    .Straight = 0,   // 直行稳定计数（居中持续周期数）
    .Spd_Mileage = 0,// 速度里程累计（用于速度曲线计算）
};

static uint8_t Straight_Node_Pending = 0;         // 直道节点等待标志：0=无等待, 1=等待直道稳定触发Finish
static uint8_t Turn_Angle_Settle_Count = 0;       // 角度闭环退出稳定计数

/*---------------内部静态函数声明---------------*/
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
static void Build_Rebuild_Action_List_From_RunTrack(void);
static void Build_Dispatch_Current_Action(void);
static void Build_Finish_Current_Action(void);
static Build_Action_Enum Build_Node_Dir_To_Action(uint8_t node_dir);
static Build_Action_Enum Build_Element_Dir_To_Action(uint8_t element_dir);
static uint8_t Build_Action_To_Node_Dir(Build_Action_Enum action);

/********************************* 内部静态函数实现 *********************************/

/*************************************
** Function: Is_Track_Sensor_Adjacent
** Description: 判断两个控制用光电在当前第一排布局中是否相邻
*************************************/
static uint8_t Is_Track_Sensor_Adjacent(uint8_t left_index, uint8_t right_index)
{
    if (right_index == left_index + 1)
    {
        return 1;
    }

    // 6/7/8 被屏蔽后，第一排中间的 5 和 9 是物理相邻。
    if (left_index == 5 && right_index == 9)
    {
        return 1;
    }

    return 0;
}

static Build_Action_Enum Build_Node_Dir_To_Action(uint8_t node_dir)
{
    switch (node_dir)
    {
        case 1:  return BUILD_ACTION_TURN_LEFT;
        case 2:  return BUILD_ACTION_TURN_RIGHT;
        default: return BUILD_ACTION_NONE;
    }
}

static Build_Action_Enum Build_Element_Dir_To_Action(uint8_t element_dir)
{
    switch (element_dir)
    {
        case 1:  return BUILD_ACTION_ELEM_LEFT;
        case 2:  return BUILD_ACTION_ELEM_RIGHT;
        case 3:  return BUILD_ACTION_STRAIGHT_SHORT;
        case 4:  return BUILD_ACTION_STRAIGHT_LONG;
        default: return BUILD_ACTION_NONE;
    }
}

static uint8_t Build_Action_To_Node_Dir(Build_Action_Enum action)
{
    switch (action)
    {
        case BUILD_ACTION_TURN_LEFT:  return 1;
        case BUILD_ACTION_TURN_RIGHT: return 2;
        default:                      return 0;
    }
}

static void Build_Rebuild_Action_List_From_RunTrack(void)
{
    uint8_t seg;
    Build_Action_Count = 0;
    Build_Action_Index = 0;
    Build_Action_Active_Index = 0;
    memset(Build_Action_List, 0, sizeof(Build_Action_List));

    for (seg = 0; seg <= Run_Track.Node_Num && seg < TRACK_SEGMENT_NUM_MAX; seg++)
    {
        uint8_t elem;
        uint8_t mileage_num = Run_Track.Node_Arr_Mileage_Num[seg];

        if (mileage_num > ELEMENT_NUM_MAX)
        {
            mileage_num = ELEMENT_NUM_MAX;
        }

        for (elem = 0; elem < mileage_num && Build_Action_Count < BUILD_ACTION_MAX; elem++)
        {
            Build_Action_List[Build_Action_Count].action =
                Build_Element_Dir_To_Action(Run_Track.Node_Arr_Mileage_Dir[seg][elem]);
            Build_Action_List[Build_Action_Count].source = BUILD_ACTION_SOURCE_ELEMENT;
            Build_Action_List[Build_Action_Count].segment_index = seg;
            Build_Action_List[Build_Action_Count].element_index = elem;
            Build_Action_Count++;
        }

        if (seg < Run_Track.Node_Num && Build_Action_Count < BUILD_ACTION_MAX)
        {
            Build_Action_List[Build_Action_Count].action = Build_Node_Dir_To_Action(Run_Track.Node_Arr_Dir[seg]);
            Build_Action_List[Build_Action_Count].source = BUILD_ACTION_SOURCE_NODE;
            Build_Action_List[Build_Action_Count].segment_index = seg;
            Build_Action_List[Build_Action_Count].element_index = 0;
            Build_Action_Count++;
        }
    }
}

static void Build_Finish_Current_Action(void)
{
    uint8_t finished_index = Build_Action_Active_Index;

    if (finished_index < Build_Action_Count)
    {
        Build_Action_Typedef *action = &Build_Action_List[finished_index];
        Execute_Times = action->segment_index;
        In_Line_Ele_Count = action->element_index;

        if (action->source == BUILD_ACTION_SOURCE_ELEMENT)
        {
            In_Line_Ele_Count++;

            if (In_Line_Ele_Count >= Run_Track.Node_Arr_Mileage_Num[Execute_Times])
            {
                Line_Num_Count++;
            }
        }
        else
        {
            Line_Num_Count++;
        }
    }

    if (Build_Action_Index >= Build_Action_Count ||
        (Run_Track.Stop_Mode == 1 && Line_Num_Count >= Run_Track.Node_Num))
    {
        Finish_Flag = 1;
    }
}

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
    Mileage_Times = Run_Track.Node_Arr_Mileage_Num[Execute_Times];

    if (action->source == BUILD_ACTION_SOURCE_NODE)
    {
        Segment_Total_Mileage[Execute_Times] = Count.Mileage;
        Build_Action_Index++;
        Set_Node_Run_Mode(Build_Action_To_Node_Dir(action->action));
        return;
    }

    Record_Segment_Edge_Mileage();
    Build_Action_Index++;

    if (action->action == BUILD_ACTION_NONE)
    {
        In_Line_Ele_Count++;
        if (In_Line_Ele_Count >= Mileage_Times)
        {
            Segment_Total_Mileage[Execute_Times] = Count.Mileage;
            Build_Dispatch_Current_Action();
        }
        return;
    }

    Mileage_Element_Base = Count.Mileage;
    Run_Mode = Mileage_Mode;
}

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
** Description: 转向路段索引递增（预留扩展点，当前为空）
*************************************/
static void Advance_Turn_Section_Index(void)
{
}

/*************************************
** Function: Reset_Turn_Action_State
** Description: 重置转向动作相关的所有状态变量
** Details:   转弯完成后调用，清除陀螺仪积分、编码器计数、方向标志，
**            并设置 Check_Edge_Skip_Count=30 防止转弯后立即误触发边缘检测
*************************************/
static void Reset_Turn_Action_State(void)
{
    Gyro_Integral = 0;           // 清零陀螺仪积分角度
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
    Count.Left = 0;              // 清零左转出弯计数
    Count.Right = 0;             // 清零右转出弯计数
    is_left = 0;                 // 清除左转标志
    is_right = 0;                // 清除右转标志
    Turn_Decel_Phase = 0;        // 清除转弯减速阶段
    Check_Edge_Skip_Count = BUILD_CHECK_EDGE_NODE_TURN;
    Run_Mode = Normal_Mode;      // 恢复到正常寻迹模式
}

/*************************************
** Function: Complete_Turn_Action
** Description: 完成转向动作的统一入口
** Details:   1. 标记 Turn_Action_Done
**            2. 建图模式下记录转向里程到Flash
**            3. 推进转向记录索引并切换到下一路段
**            4. 最后调用 Reset_Turn_Action_State 清理状态
** 调用链：   Turn_Left_Run / Turn_Right_Run → Complete_Turn_Action → Record_Turn_Mileage
*************************************/
static void Complete_Turn_Action(void)
{
    Turn_Action_Done = 1;        // 标记转向动作完成
    Record_Turn_Mileage();       // 记录本次转向里程

    Build_Finish_Current_Action();
    Reset_Turn_Action_State();   // 统一重置转向状态（Run_Mode变回Normal_Mode）
}

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
** Description: 计算寻迹中心点（首尾白线传感器索引的平均值）
** Return:     Track_Num>0 → (首+尾)/2,  Track_Num==0 → 默认7（中心）
** Details:   用于判断小车相对赛道中心的位置。7是物理中心传感器索引
*************************************/
static int Get_Track_Middle_Point(void)
{
    if (Track_Num > 0)  // 至少检测到1个白线传感器
    {
        return (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;  // 首尾平均值
    }

    return 7;  // 无传感器信号时默认在中心（安全兜底值）
}

/*************************************
** Function: Set_Node_Run_Mode
** Description: 根据节点方向设置运行模式
** Input:      node_dir - 节点方向：0=直行, 1=左转, 2=右转, 3=短直行, 4=长直行
** Details:   清零所有状态计数器，根据方向切换到对应的 Run_Mode
*************************************/
static void Set_Node_Run_Mode(uint8_t node_dir)
{
    // 统一清零状态变量（进入新路段需要全新状态）
    Gyro_Integral = 0;         // 清零陀螺仪积分
    Turn_Angle_Last_Real = 0;
    Turn_Angle_Settle_Count = 0;
    PID_cleardata(&Angle_PID);
    Mileage_Turn_Done = 0;     // 清零里程转向完成标志
    Turn_Action_Done = 0;      // 清零传感器转向完成标志
    Turn_Decel_Phase = 0;      // 清零转弯减速阶段
    Count.Spd_Mileage = 0;     // 清零速度里程
    Count.Mileage = 0;         // 清零当前段里程
    Mileage_Element_Base = 0;  // 清零里程基准值
    Straight_Node_Pending = 0; // 清零直道等待标志

    switch (node_dir)
    {
        case 1:  // 左转节点 → 角度闭环PID 目标+90°
            Run_Mode = Turn_Left;
            is_left = 1;
            is_right = 0;
            Turn_Angle_Target = -90.0f;
            Turn_Begin_Mileage = Total_Run_Mileage;
            Advance_Turn_Section_Index();
            break;
        case 2:  // 右转节点 → 角度闭环PID 目标-90°
            Run_Mode = Turn_Right;
            is_left = 0;
            is_right = 1;
            Turn_Angle_Target = 90.0f;
            Turn_Begin_Mileage = Total_Run_Mileage;
            Advance_Turn_Section_Index();
            break;
        case 0:   // 直行节点（0=无转向，fall-through）
        default:
            Straight_Node_Pending = 1;
            Run_Mode = Straight_Mode;
            break;
    }
}

/*************************************
** Function: Finish_Mileage_Section
** Description: 完成当前里程路段，推进元素/线路计数
** Details:
*************************************/
static void Finish_Mileage_Section(void)
{
    // 清除转向相关标志
    is_left = 0;
    is_right = 0;
    Force_Straight_Speed = 0;  // 清除直行强制速度标志
    Mileage_Turn_Done = 0;
    Check_Edge_Skip_Count = BUILD_CHECK_EDGE_MILEAGE_STRAIGHT;
    Run_Mode = Normal_Mode;      // 恢复到正常寻迹模式

    Mileage_Element_Base = Count.Mileage;  // 更新里程基准

    Build_Finish_Current_Action();
}

/*************************************
** Function: Record_Segment_Edge_MileageSegment_Total_Mileage
** Description: 记录当前路段边缘的里程值
** Details:
*************************************/
static void Record_Segment_Edge_Mileage(void)
{
    if (Execute_Times < TRACK_SEGMENT_NUM_MAX && In_Line_Ele_Count < ELEMENT_NUM_MAX)
    {
        float recorded_mileage = Count.Mileage;
        uint8_t mileage_dir = Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count];

        if (mileage_dir == 1 || mileage_dir == 2)
        {
            recorded_mileage += MILEAGE_COMPENSATION_X;
        }

        Segment_Edge_Mileage_Record[Execute_Times][In_Line_Ele_Count] = recorded_mileage;
        //Save_Segment_Edge_Mileage_Record_To_Flash();  // 立即持久化到Flash
    }
}

/*************************************
** Function: Save_Segment_Edge_Mileage_Record_To_Flash
** Description: 将路段边缘里程记录保存到Flash
** Details:
*************************************/
static void Save_Segment_Edge_Mileage_Record_To_Flash(void)
{
    Segment_Edge_Mileage_Flash_Typedef flash_log = {{0}};  // Flash打包结构体（栈上初始化）
    uint32 map_words[SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT * SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};

    // 将全局数据拷贝到Flash打包结构体
    memcpy(flash_log.Segment_Edge_Mileage_Record,
           Segment_Edge_Mileage_Record,
           sizeof(Segment_Edge_Mileage_Record));
    memcpy(flash_log.Segment_Total_Mileage,
           Segment_Total_Mileage,
           sizeof(Segment_Total_Mileage));

    // 将结构体按字节拷贝到uint32数组（Flash写入需要uint32对齐）
    memcpy(map_words, &flash_log, sizeof(flash_log));
    Save_Flash_Page_Block(SEGMENT_EDGE_MILEAGE_FLASH_SECTOR,      // 扇区0
                          SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE,   // 起始页8
                          SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT,   // 页数2
                          SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE, // 每页字数64
                          map_words);
}

/*************************************
** Function: Load_Segment_Edge_Mileage_Record_From_Flash
** Description: 从Flash加载路段边缘里程记录
** Details:
*************************************/
static void Load_Segment_Edge_Mileage_Record_From_Flash(void)
{
    Segment_Edge_Mileage_Flash_Typedef flash_log = {{0}};
    uint32 map_words[SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT * SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};

    // 从Flash读取原始uint32数据
    Load_Flash_Page_Block(SEGMENT_EDGE_MILEAGE_FLASH_SECTOR,
                          SEGMENT_EDGE_MILEAGE_FLASH_START_PAGE,
                          SEGMENT_EDGE_MILEAGE_FLASH_PAGE_COUNT,
                          SEGMENT_EDGE_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);

    // uint32数组 → 结构体 → 全局变量（两步拷贝确保类型安全）
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
** Description: 将转向里程记录保存到Flash
** Details:
*************************************/
static void Save_Turn_Mileage_Record_To_Flash(void)
{
    Turn_Mileage_Flash_Typedef flash_log = {0, {0}};
    uint32 map_words[TURN_MILEAGE_FLASH_PAGE_COUNT * TURN_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};

    // 打包转向里程记录数量和数组
    flash_log.Turn_Mileage_Record_Num = Turn_Mileage_Record_Num;
    memcpy(flash_log.Turn_Mileage_Record, Turn_Mileage_Record, sizeof(Turn_Mileage_Record));

    // 拷贝到uint32数组后写入Flash
    memcpy(map_words, &flash_log, sizeof(flash_log));
    Save_Flash_Page_Block(TURN_MILEAGE_FLASH_SECTOR,
                          TURN_MILEAGE_FLASH_START_PAGE,
                          TURN_MILEAGE_FLASH_PAGE_COUNT,
                          TURN_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);
}

/*************************************
** Function: Load_Turn_Mileage_Record_From_Flash
** Description: 从Flash加载转向里程记录
** Details:
*************************************/
static void Load_Turn_Mileage_Record_From_Flash(void)
{
    Turn_Mileage_Flash_Typedef flash_log = {0, {0}};
    uint32 map_words[TURN_MILEAGE_FLASH_PAGE_COUNT * TURN_MILEAGE_FLASH_WORDS_PER_PAGE] = {0};

    // 从Flash读取原始数据到uint32数组
    Load_Flash_Page_Block(TURN_MILEAGE_FLASH_SECTOR,
                          TURN_MILEAGE_FLASH_START_PAGE,
                          TURN_MILEAGE_FLASH_PAGE_COUNT,
                          TURN_MILEAGE_FLASH_WORDS_PER_PAGE,
                          map_words);

    // ★ 关键：将uint32数组数据拷贝到结构体（之前缺失此行导致flash_log始终为零）
    memcpy(&flash_log, map_words, sizeof(flash_log));

    // 边界保护：防止Flash中存储的数值超过数组容量
    if (flash_log.Turn_Mileage_Record_Num > TURN_MILEAGE_RECORD_MAX)
    {
        flash_log.Turn_Mileage_Record_Num = TURN_MILEAGE_RECORD_MAX;
    }

    // 恢复到全局变量
    Turn_Mileage_Record_Num = flash_log.Turn_Mileage_Record_Num;
    memcpy(Turn_Mileage_Record, flash_log.Turn_Mileage_Record, sizeof(Turn_Mileage_Record));
}

/*************************************
** Function: Save_Flash_Page_Block
** Description: 通用Flash多页写入函数
** Input:      sector       - Flash扇区号
**             start_page   - 起始页号
**             page_count   - 写入页数
**             words_per_page - 每页写入的uint32_t数量
**             words        - 待写入数据指针
** Details:   先擦除每页，再写入每页。AURIX Flash必须先擦后写
*************************************/
static void Save_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, const uint32_t *words)
{
    uint8_t page_index;

    for (page_index = 0; page_index < page_count; page_index++)
    {
        flash_erase_page(sector, start_page + page_index);            // 擦除目标页
        flash_write_page(sector,                                     // 写入目标页
                         start_page + page_index,                     // 页号
                         &words[page_index * words_per_page],         // 数据起始地址（跳过前面已写入页）
                         words_per_page);                             // 本页写入字数
    }
}

/*************************************
** Function: Load_Flash_Page_Block
** Description: 通用Flash多页读取函数
** Input:      sector/start_page/page_count/words_per_page 同写入
** Output:     words - 读取数据存放缓冲区
** Details:   逐页读取Flash数据到uint32_t数组
*************************************/
static void Load_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, uint32_t *words)
{
    uint8_t page_index;

    for (page_index = 0; page_index < page_count; page_index++)
    {
        flash_read_page(sector,                                     // 扇区号
                        start_page + page_index,                     // 页号
                        &words[page_index * words_per_page],         // 目标缓冲区地址
                        words_per_page);                             // 读取字数
    }
}

/*************************************
** Function: Record_Turn_Mileage
** Description: 记录两次转弯之间的里程间隔到 Turn_Mileage_Record 数组
** Details:   建图模式下在每次转弯完成时调用。
**            turn_interval_mileage = Turn_Begin_Mileage - Last_Turn_Mileage_Base
**            即：本次转弯起始里程 - 上次转弯记录基准里程 = 两次转弯间的行驶距离
**            记录后立即写入Flash。
*************************************/
static void Record_Turn_Mileage(void)
{
    float turn_interval_mileage;  // 两次转弯间的里程间隔

    if (Turn_Mileage_Record_Num >= TURN_MILEAGE_RECORD_MAX)  // 防止数组溢出
    {
        return;
    }

    // 计算本次转弯与上次转弯之间的里程间隔
    turn_interval_mileage = Turn_Begin_Mileage - Last_Turn_Mileage_Base;
    if (turn_interval_mileage < 0)  // 防御：里程不应为负
    {
        turn_interval_mileage = 0;
    }

    Turn_Mileage_Record[Turn_Mileage_Record_Num] = turn_interval_mileage; // 存入数组
    Turn_Mileage_Record_Num++;                                             // 计数+1

    Last_Turn_Mileage_Base = Total_Run_Mileage;  // 更新基准里程（为下一次记录做准备）

    //Save_Turn_Mileage_Record_To_Flash();  // 立即持久化到Flash
}

/*************************************
** Function: Load_All_Flash_Data_For_VOFA
** Description: VOFA导出模式：从Flash加载全部地图数据到全局变量
** Details:   加载 Turn_Mileage_Record 和 Segment_Edge_Mileage_Record
**            调用后全局变量即包含Flash中的建图数据
*************************************/
void Load_All_Flash_Data_For_VOFA(void)
{
    Load_Turn_Mileage_Record_From_Flash();
    Load_Segment_Edge_Mileage_Record_From_Flash();
}

/********************************* 公共函数实现 *********************************/

/*************************************
** Function: Safety_Check
** Description: 统一安全检测（每次中断最先调用，停车后接管PWM和蜂鸣器）
** Details:   1. 低压检测：电池电压 < SAFETY_LOW_VOLTAGE_THRESHOLD → 停车
**             2. 串口 @STOP# 命令或 Count.Stop 超限置 Stop_Flag
**             3. 完赛后保存Flash并延时停车
**             4. 停车响应：拉低全部占空比 + 蜂鸣器间歇响（200周期=600ms）
**               停车后不再进入后续模式逻辑，下游函数无需再判断 Stop_Flag
*************************************/
void Safety_Check(void)
{
    static uint16_t stop_beep_count = 0;  // 停车后蜂鸣器周期计数

    //===== 低压检测 =====
   if (Voltage_Check[0] < SAFETY_LOW_VOLTAGE_THRESHOLD)
   {
       Stop_Flag = 1;
   }

   if(Count.Stop > SAFETY_STOP_CYCLE_MAX)
   {
       Stop_Flag = 1;
   }
    //===== 完赛停车：Finish_Flag 置位后延迟 200 周期（600ms）停车 + 保存Flash =====
    if (Finish_Flag == 1)
    {
        Finish_Count++;
    }
    if (Finish_Count > 200 && Stop_Flag == 0)
    {
        Stop_Flag = 1;
        if (Mode == Build_Mode)
        {
            Save_Turn_Mileage_Record_To_Flash();         // 转弯间距里程存入Flash
            Save_Segment_Edge_Mileage_Record_To_Flash(); // 路段边缘里程存入Flash
        }
    }

    //===== 停车响应：拉低全部PWM + 蜂鸣器间歇报警 =====
    if (Stop_Flag != 0)
    {
        pwm_set_duty(Suction_Motor_IN1, 0);
        pwm_set_duty(Suction_Motor_IN2, 0);
        pwm_set_duty(Left_Motor_IN1, 0);
        pwm_set_duty(Left_Motor_IN2, 0);
        pwm_set_duty(Right_Motor_IN1, 0);
        pwm_set_duty(Right_Motor_IN2, 0);

        Left_Exp_Spd = 0;
        Right_Exp_Spd = 0;
        Left_PID_Out = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);

        stop_beep_count++;
        if (stop_beep_count >= 200)  // 200×3ms = 600ms 一个周期
        {
            stop_beep_count = 0;
        }
        gpio_set_level(P33_4, (stop_beep_count < 60) ? 1 : 0);  // 前180ms响，后420ms静
    }
    else
    {
        stop_beep_count = 0;
    }
}

/*************************************
** Function: Car_Go
** Description: 小车主运行函数（由3ms定时器中断调用）
** Call Order:  Safety_Check → Get_Light → 读编码器(3ms) → (每2次)Get_Speed(6ms) → Get_IMU → 模式Get_Error → Set_Out
** Details:    编码器每3ms读取一次原始值（无滤波），里程每3ms累加
**             Left_Real_Spd/Right_Real_Spd 每6ms计算一次（加权滤波保留）
**             启动延时：EnableSwitch_ON上升沿触发100周期延时（300ms）
*************************************/
void Car_Go()
{
    // 使能开关上升沿检测：从未按下→按下，触发启动延时
    if (EnableSwitch_ON == 1 && Last_EnableSwitch_ON == 0)
    {
        Enable_Start_Delay_Count = 100;  // 启动延时100周期（100×3ms=300ms）
    }
    Last_EnableSwitch_ON = EnableSwitch_ON;  // 更新上一周期状态

    // Debug模式不需要启动延时，直接解锁电机
    if (Mode == Debug_Mode)
    {
        Enable_Start_Delay_Count = 0;
    }

    // 每2次调用执行一次速度计算（6ms周期，编码器读取+FIR滤波+里程累加）
    if (Speed_Get_Count == 1)
    {
        Get_Speed();
    }
    Speed_Get_Count *= -1;  // 交替切换：1→-1→1→-1...

    Get_Light();  // 读取15路光敏传感器ADC值（每周3ms）

    Light_Process();  // 先处理传感器数据（更新 Light_Convert / Track_Arr）

    Get_IMU();  // 读取陀螺仪角速度 + 积分角度（每周3ms）

    Safety_Check();  // ★ 统一安全检测（低压+堵转+串口STOP），停车后直接接管PWM/蜂鸣器

    // 停车后跳过所有模式逻辑，Safety_Check已拉低占空比并启动蜂鸣器
    if (Stop_Flag != 0)
    {
        return;
    }

    // 根据模式选择对应的寻迹逻辑
    if (Mode == Build_Mode)
    {
        if (EnableSwitch_ON)
        {
            Build_Mode_Get_Error();     // 建图模式：传感器寻迹 + 记录里程数据
        }
    }

    if (Mode == Debug_Mode)
    {
        // 调试模式：根据子模式分发（各函数内部自行调用 Debug_Set_Out）
        switch (Debug_Sub_Mode)
        {
            case Debug_Sub_PI_Tuning:   Debug_Wheel_Tuning();   break;
            case Debug_Sub_Ground_Test: Debug_Ground_Test();     break;
            case Debug_Sub_Angle:       Debug_Angle_Tuning();   break;
            case Debug_Sub_NormalTrace: Debug_Normal_Trace();   break;
            default:                    Debug_Wheel_Tuning();   break;
        }
        return;  // 调试函数内部已调用 Debug_Set_Out，跳过 Set_Speed/Set_Out
    }

    Set_Speed();  // 计算PID输出速度

    Set_Out();  // PWM输出控制电机（每周3ms）
}


/*************************************
** Function: Debug_Wheel_Tuning
** Description: 轮子PI调参（左右轮共用，每3ms调用）
** Details:   Debug_Motor_Enable=0 → 停机 + 清零PID
**            Debug_Motor_Enable=1 → 读编码器、运行增量式PI、调用 Debug_Set_Out
**            通过 Debug_Which_Wheel 选择左轮(0)或右轮(1)
**            Kp/Ki 从全局变量 Debug_Kp_Left/Right 读取
**            目标速度从全局变量 Debug_Target_Speed 读取
** Note:     不读光电、不读IMU、不跑Set_Speed
*************************************/
void Debug_Wheel_Tuning(void)
{
    if (Debug_Motor_Enable == 0)
    {
        // 停机状态：清零PID，关闭电机
        Left_PID_Out  = 0;
        Right_PID_Out = 0;
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
    }
    else  // Debug_Motor_Enable == 1
    {
        float kp, ki;
//        Debug_Which_Wheel = 1;
        if (Debug_Which_Wheel == 0)
        {
            // 左轮调参：只用左电机
            kp = Debug_Kp_Left;
            ki = Debug_Ki_Left;

            Left_PID.kp  = kp;
            Left_PID.ki  = ki;
            Left_Exp_Spd = Debug_Target_Speed;
            Right_Exp_Spd = 0;

            Left_PID_Out  = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
            Right_PID_Out = 0;
            PID_cleardata(&Right_PID);
        }
        else
        {
            // 右轮调参：只用右电机
            kp = Debug_Kp_Right;
            ki = Debug_Ki_Right;

            Right_PID.kp  = kp;
            Right_PID.ki  = ki;
            Right_Exp_Spd = Debug_Target_Speed;
            Left_Exp_Spd  = 0;

            Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
            Left_PID_Out  = 0;
            PID_cleardata(&Left_PID);
        }
    }

    Debug_Set_Out();  // 精简PWM输出
    pwm_set_duty(Suction_Motor_IN1, 0);
    pwm_set_duty(Suction_Motor_IN2, 0);
}

/*************************************
** Function: Debug_Ground_Test
** Description: 下地测试：左右轮反向运行 + 负压风扇
*************************************/
void Debug_Ground_Test(void)
{
    if (Debug_Motor_Enable == 1 && EnableSwitch_ON == 1)  // Debug模式不需要使能开关
    {
        Left_PID.kp  = Debug_Kp_Left;
        Left_PID.ki  = Debug_Ki_Left;
        Right_PID.kp = Debug_Kp_Right;
        Right_PID.ki = Debug_Ki_Right;

        if(Debug_Ground_Dir == 1)
        {
            Left_Exp_Spd  = Debug_Target_Speed;
            Right_Exp_Spd = -Debug_Target_Speed;
        }
        else if(Debug_Ground_Dir == 2)
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
/*************************************
** Function: Debug_Angle_Tuning
** Description: 角度环调参（预留）
*************************************/
void Debug_Angle_Tuning(void)
{
    static uint32 angle_tick = 0;
    float angle_target;
    float gyro_target;

    if (Debug_Motor_Enable == 0 || EnableSwitch_ON == 0)  // 停机状态
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
        return;  // 停机时跳过角度环计算，不累计angle_tick
    }

    if (Debug_Angle_Mode == 2)
    {
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
    else
    {
        angle_target = 0;
        Turn_PID_Out = 0;
        Debug_Angle_D_First = 0;
        gyro_target = 800.0f * sinf(6.2831853f * (float)(angle_tick % 333U) / 333.0f);  // 1Hz正弦波: 333×3ms=999ms≈1s
    }

    Debug_Angle_Vel_Target = gyro_target;
    Debug_Angle_Vel_Real = Gyro_Z;
    if (Debug_Angle_Mode == 2)
    {
        Gyro_PID_Out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);      // 90-degree angle PD -> gyro PI
    }
    else
    {
        Gyro_PID_Out = PID_calc(&Gyro_PD_PID, gyro_target, Gyro_Z);   // sine gyro-rate target -> gyro PD
    }

    Left_Exp_Spd = Basic_Speed + Gyro_PID_Out;
    Right_Exp_Spd = Basic_Speed - Gyro_PID_Out;

    Left_PID.kp = Debug_Kp_Left;
    Left_PID.ki = Debug_Ki_Left;
    Right_PID.kp = Debug_Kp_Right;
    Right_PID.ki = Debug_Ki_Right;
    Left_PID_Out = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
    Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);

    angle_tick++;
    Debug_Set_Out();
}
/*************************************
** Function: Debug_Normal_Trace
** Description: 普通循迹（预留）
*************************************/
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
        Turn_PID_Out = PID_calc(&Turn_PID, 0.0f, (float)Error);
    }
    Gyro_PID_Out = PID_calc(&Gyro_PD_PID, Turn_PID_Out, Gyro_Z);

    Debug_Angle_Vel_Target = Turn_PID_Out;
    Debug_Angle_Vel_Real = Gyro_Z;

    Left_Exp_Spd = Basic_Speed + Gyro_PID_Out;
    Right_Exp_Spd = Basic_Speed - Gyro_PID_Out;

    Left_PID.kp = Debug_Kp_Left;
    Left_PID.ki = Debug_Ki_Left;
    Right_PID.kp = Debug_Kp_Right;
    Right_PID.ki = Debug_Ki_Right;

    Left_PID_Out = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
    Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
    Debug_Set_Out();
}
/*************************************
** Function: Debug_Set_Out
** Description: 调试模式专用精简PWM输出
** Details:   不走低压保护/启动延时/堵转检测/Stop_Flag等复杂逻辑
**            仅根据 Left_PID_Out / Right_PID_Out 驱动H桥
**            吸风电机常开（8000占空比）
**            Debug_Motor_Enable=0时全关
*************************************/
void Debug_Set_Out(void)
{
    // 吸风电机常开
//    pwm_set_duty(Suction_Motor_IN1, 0);
//    pwm_set_duty(Suction_Motor_IN2, 0);
    if (Debug_Motor_Enable != 0)
    {
        pwm_set_duty(Suction_Motor_IN1, Debug_Fan_Duty);
        pwm_set_duty(Suction_Motor_IN2, 10000);
    }
    else
    {
        pwm_set_duty(Suction_Motor_IN1, 0);
        pwm_set_duty(Suction_Motor_IN2, 0);
    }
    //===== 左电机 =====
    if (Debug_Motor_Enable == 0 || Left_PID_Out == 0)
    {
        pwm_set_duty(Left_Motor_IN1, 0);
        pwm_set_duty(Left_Motor_IN2, 0);
    }
    else if (Left_PID_Out < 0)
    {
        pwm_set_duty(Left_Motor_IN1, 10000);
        pwm_set_duty(Left_Motor_IN2, 10000 - fabs(Left_PID_Out));
    }
    else  // Left_PID_Out < 0
    {
        pwm_set_duty(Left_Motor_IN1, 10000 - fabs(Left_PID_Out));
        pwm_set_duty(Left_Motor_IN2, 10000);
    }

    //===== 右电机 =====
    if (Debug_Motor_Enable == 0 || Right_PID_Out == 0)
    {
        pwm_set_duty(Right_Motor_IN1, 0);
        pwm_set_duty(Right_Motor_IN2, 0);
    }
    else if (Right_PID_Out < 0)
    {
        pwm_set_duty(Right_Motor_IN1, 10000);
        pwm_set_duty(Right_Motor_IN2, 10000 - fabs(Right_PID_Out));
    }
    else  // Right_PID_Out < 0
    {
        pwm_set_duty(Right_Motor_IN1, 10000 - fabs(Right_PID_Out));
        pwm_set_duty(Right_Motor_IN2, 10000);
    }
//    pwm_set_duty(Left_Motor_IN1, 0);
//    pwm_set_duty(Left_Motor_IN2, 9000);
//    pwm_set_duty(Right_Motor_IN1, 0);
//    pwm_set_duty(Right_Motor_IN2, 6000);
}

/*************************************
** Function: Get_Speed
** Description: 速度更新（每6ms调用一次，编码器读值+FIR滤波+里程累加）
** Details:   与参考工程straight_longqiu_motor_car完全一致
**            6ms编码器读值 → 3-tap FIR → Left_Real_Spd/Right_Real_Spd
*************************************/
void Get_Speed()
{
    int left_raw, right_raw;
    float instant_speed;

    // 6ms编码器读值，与参考工程straight_longqiu_motor_car完全一致
    left_raw  =  1 * encoder_get_count(TIM3_ENCODER) / 3;
    right_raw = -1 * encoder_get_count(TIM2_ENCODER) / 3;
    encoder_clear_count(TIM3_ENCODER);
    encoder_clear_count(TIM2_ENCODER);

    // 3-tap FIR滤波
    giSpeed_Left[2] = giSpeed_Left[1];
    giSpeed_Left[1] = giSpeed_Left[0];
    giSpeed_Right[2] = giSpeed_Right[1];
    giSpeed_Right[1] = giSpeed_Right[0];

    giSpeed_Left[0]  = left_raw;
    giSpeed_Right[0] = right_raw;

    Left_Real_Spd  = (int)(0.5f * giSpeed_Left[0]  + 0.3f * giSpeed_Left[1]  + 0.2f * giSpeed_Left[2]);
    Right_Real_Spd = (int)(0.5f * giSpeed_Right[0] + 0.3f * giSpeed_Right[1] + 0.2f * giSpeed_Right[2]);

    // 里程累加（6ms周期）
    if (EnableSwitch_ON)
    {
        instant_speed = (left_raw + right_raw) / 2.0f;
        Count.Mileage += instant_speed;
        Total_Run_Mileage += instant_speed;
    }
}

/*************************************
** Function: Get_IMU
** Description: 获取IMU陀螺仪数据并处理（每3ms调用一次）
** Details:
*************************************/
void Get_IMU()
{
    imu660rb_get_gyro();  // 读取IMU660RB陀螺仪原始数据

    float gyro_raw = imu660rb_gyro_transition(imu660rb_gyro_z);
    uint8 debug_angle_run = (Mode == Debug_Mode
        && (Debug_Sub_Mode == Debug_Sub_Angle || Debug_Sub_Mode == Debug_Sub_NormalTrace)
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
        Gyro_Integral = 0;  // 非转弯状态清零积分角度
    }

    // PID用Gyro_Z：与原工程一致，raw/1000，死区<30 raw
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
** Description: 检测赛道边缘或特殊节点
** Return:     0=未检测到, 1=检测到
** Details:
*************************************/
uint8 Check_Edge()
{
    if (Check_Edge_Skip_Count > 0)  // 跳过计数生效中（转弯刚结束，车身未稳定）
    {
        return 0;
    }

    // 最左或最右传感器检测到白线 = 边界/路口
    if (((Light_Convert[0] == 1 || Light_Convert[14] == 1)&&
        Initial_White_Num >= 4) || Initial_White_Num >= 5)
    {
        Check_Edge_Count++;
        Count.Mileage = 0;  // 清段内里程，防止堵转误判
        // Mileage_Element_Base = 0;  // 已由 Set_Node_Run_Mode 清零
        return 1;
    }

    return 0;
}

/*************************************
** Function: Light_Process
** Description: 光电传感器数据处理（每3ms调用一次）
** Details:
*************************************/
void Light_Process()
{
    memcpy(Last_Light_Convert, Light_Convert, sizeof(Light_Convert));
    uint8_t Led_Control_Enable = (Run_Mode != Mileage_Mode && Run_Mode != Turn_Left && Run_Mode != Turn_Right);
    uint8_t sensor_index;

    //===== ADC二值化：比较ADC值与阈值，滞回比较器 =====
    for (int i = 0; i < 15; i++)
    {
        if (Light_ADC[i] > Light_Thr[i][0])  // ADC > 上阈值 → 判定为白线
        {
            Light_Convert[i] = 1;
            if (Led_Control_Enable)
            {
                TCA9555_LED_Ctrl(LED[14 - i], 1);
            }
        }
        if (Light_ADC[i] < Light_Thr[i][1])  // ADC < 下阈值 → 判定为黑线
        {
            Light_Convert[i] = 0;
            if (Led_Control_Enable)
            {
                TCA9555_LED_Ctrl(LED[14 - i], 0);  // 熄灭对应LED
            }
        }
    }

    memcpy(Last_Track_Arr, Track_Arr, sizeof(Track_Arr));
    Last_Track_Num = Track_Num;
    for (int i = 0; i < 15; i++)
    {
        Track_Arr[i] = 0;  // 全清零
    }
    Initial_White_Num = 0;
    Track_Num = 0;
    Left_Num = 0;
    Right_Num = 0;

    for (uint8_t i = 0; i < TRACK_SENSOR_ACTIVE_NUM; i++)
    {
        sensor_index = Track_Sensor_Active_Index[i];
        if (Light_Convert[sensor_index] == 1)  // 该控制用传感器检测到白线
        {
            Initial_White_Num++;              // 白线传感器总数+1
            Track_Arr[Track_Num++] = sensor_index; // 记录物理传感器索引，Track_Num自增
        }
    }

    for (int i = 0; i < Track_Num - 1; i++)
    {
        if (!Is_Track_Sensor_Adjacent((uint8_t)Track_Arr[i], (uint8_t)Track_Arr[i + 1]))
        {
            Track_Num = Last_Track_Num;                           // 回退数量
            memcpy(Track_Arr, Last_Track_Arr, sizeof(Last_Track_Arr)); // 回退整帧
            break;
        }
    }

    //===== 停车检测：控制用12路全白或全黑时累计 =====
    if (EnableSwitch_ON && Mode != Debug_Mode)
    {
        if ((Track_Num == TRACK_SENSOR_ACTIVE_NUM || Track_Num == 0) && is_left == 0 && is_right == 0)
        {
            Count.Stop++;  // 停车计数累加（80次 → Safety_Check 触发安全停车）
        }
        else
        {
            Count.Stop = 0;  // 恢复正常，清零计数
        }
    }
    else
    {
        Count.Stop = 0;  // 使能关闭时不累计停车计数
    }

}

/*************************************
** Function: Build_Mode_Get_Error
** Description: 建图模式：计算寻迹偏差 + 状态机调度
** Details:
*************************************/
void Build_Mode_Get_Error()
{
    if (Check_Edge_Skip_Count > 0)  // 边缘检测跳过计数递减
    {
        Check_Edge_Skip_Count--;
    }

    // 建图模式首次初始化
    if (First_Mode == 0)
    {
        Run_Mode = Normal_Mode;                                       // 初始为正常寻迹模式
        First_Mode = 1;                                               // 标记已初始化
        Execute_Times = 0;
        Mileage_Times = 0;
        Line_Num_Count = 0;
        In_Line_Ele_Count = 0;
        Build_Action_Index = 0;
        Build_Action_Active_Index = 0;
        Build_Rebuild_Action_List_From_RunTrack();                    // 将节点/元器件展开为统一动作表
        Mileage_Times = Run_Track.Node_Arr_Mileage_Num[Execute_Times]; // 加载第一个节点的里程段数
        Turn_Mileage_Record_Num = 0;                                   // 清零转向里程记录数
        Total_Run_Mileage = 0;                                         // 清零总里程
        Last_Turn_Mileage_Base = 0;                                    // 清零上次转向基准
        memset(Turn_Mileage_Record, 0, sizeof(Turn_Mileage_Record));           // 清零转向里程数组
        memset(Segment_Edge_Mileage_Record, 0, sizeof(Segment_Edge_Mileage_Record)); // 清零边缘里程数组
        Save_Segment_Edge_Mileage_Record_To_Flash();                   // 保存初始空数据到Flash
    }

    //===== 运行模式状态机 =====
    switch (Run_Mode)
    {
        case Normal_Mode:
            Normal_Run();     // 正常寻迹（检测边缘 → 触发里程或转向）
            break;
        case Turn_Left:
            Turn_Left_Run();  // 左转状态机（陀螺仪积分 + 传感器出弯判定）
            break;
        case Turn_Right:
            Turn_Right_Run(); // 右转状态机
            break;
        case Mileage_Mode:
            Mileage_Mode_Run(); // 里程计模式（按预设里程行驶/转向）
            break;
        case Straight_Mode:
            Straight_Run();   // 直道模式（居中稳定判定）
            break;
        default:
            Normal_Run();
            break;
    }

}

/*************************************
** Function: Normal_Run
** Description: 正常寻迹模式（建图模式使用）
** Details:
*************************************/
void Normal_Run()
{
    if (Track_Num > 0)
    {
        Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
        gpio_set_level(P33_4, 0);
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
** Description: 左转状态机（每3ms调用一次）
** Details:
*************************************/
void Turn_Left_Run(void)
{
    TCA9555_All_LED_On();

    if (Turn_Action_Done)
        return;

    gpio_set_level(P33_4, 1);

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
        else if (Turn_Decel_Phase == 1)
        {
            Error = 0;
            Left_Exp_Spd = 0;
            Right_Exp_Spd = 0;
            if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
            {
                Turn_Decel_Phase = 2;
                Gyro_Integral = 0;
                Turn_Angle_Last_Real = 0;
                Turn_Angle_Settle_Count = 0;
                PID_cleardata(&Angle_PID);
            }
        }
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
** Description: 右转状态机 - 节点转弯与元器件Build流程一致
*************************************/
void Turn_Right_Run(void)
{
    TCA9555_All_LED_On();

    if (Turn_Action_Done)
        return;

    gpio_set_level(P33_4, 1);

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
        else if (Turn_Decel_Phase == 1)
        {
            Error = 0;
            Left_Exp_Spd = 0;
            Right_Exp_Spd = 0;
            if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
            {
                Turn_Decel_Phase = 2;
                Gyro_Integral = 0;
                Turn_Angle_Last_Real = 0;
                Turn_Angle_Settle_Count = 0;
                PID_cleardata(&Angle_PID);
            }
        }
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
** Description: 里程计模式运行（每3ms调用一次）
** Details:
*************************************/
void Mileage_Mode_Run()
{
    gpio_set_level(P33_4, 1);    // 蜂鸣器响（里程模式中）
    TCA9555_All_LED_On();        // 点亮所有LED（可视化提示进入里程模式）

    uint8_t node_dir = Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count]; // 当前元素方向

    //===== 回放模式：直行元素边缘里程校准（转弯元素不需要）=====
    float section_mileage = Count.Mileage - Mileage_Element_Base;  // 当前段已行驶里程

    if (node_dir == 1 || node_dir == 2)  // 左转(1)或右转(2)元素
    {
        Force_Straight_Speed = 0;
        Mileage_Run_Stage_2();  // 陀螺仪角度控制转向

        if (Mileage_Turn_Done == 1)  // 转向完成
        {
            Record_Turn_Mileage();      // 记录转向间隔里程（建图模式）
            Finish_Mileage_Section();   // 完成当前里程段 → 推进计数
            Check_Edge_Skip_Count = BUILD_CHECK_EDGE_MILEAGE_TURN;
        }
    }
    else if (node_dir == 3)
    {
        Error = 0;
        Force_Straight_Speed = 1;

        if (section_mileage >= MILEAGE_STRAIGHT_SHORT)
        {
            Finish_Mileage_Section();
        }
    }
    else if (node_dir == 4)
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
        Force_Straight_Speed = 1;  // 强制基础速度直走
    }
}

/*************************************
** Function: Mileage_Run_Stage_2
** Description: 里程计模式二级控制——元器件转弯
** Details:
*************************************/

/*************************************
** Function: Set_Mileage_Turn_Exp_Speed
** Description: 里程元器件转弯专用角度闭环速度
*************************************/
void Set_Mileage_Turn_Exp_Speed(float angle_target)
{
    float gyro_target;

    gyro_target = PID_calc(&Angle_PID, angle_target, Gyro_Integral);
    Gyro_PID_Out = PID_calc(&Gyro_PID, gyro_target, Gyro_Z);
    Left_Exp_Spd = (int)Gyro_PID_Out;
    Right_Exp_Spd = - (int)Gyro_PID_Out;
    Turn_Angle_Last_Real = Gyro_Integral;
}

void Mileage_Run_Stage_2()
{
    float section_mileage = Count.Mileage - Mileage_Element_Base;

    switch (Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count])
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
                else if (Turn_Decel_Phase == 1)
                {
                    Error = 0;
                    Left_Exp_Spd = 0;
                    Right_Exp_Spd = 0;
                    if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                    {
                        Turn_Decel_Phase = 2;
                        Gyro_Integral = 0;
                        Turn_Angle_Last_Real = 0;
                        Turn_Angle_Settle_Count = 0;
                        PID_cleardata(&Angle_PID);
                    }
                }
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
                else if (Turn_Decel_Phase == 1)
                {
                    Error = 0;
                    Left_Exp_Spd = 0;
                    Right_Exp_Spd = 0;
                    if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                    {
                        Turn_Decel_Phase = 2;
                        Gyro_Integral = 0;
                        Turn_Angle_Last_Real = 0;
                        Turn_Angle_Settle_Count = 0;
                        PID_cleardata(&Angle_PID);
                    }
                }
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
** Description: 设置电机期望速度（PID级联控制）
** Control Chain: Error → Turn_PID → Gyro_PID(+Gyro_Z×系数) → 左右轮差速
** Details:
*************************************/
void Set_Speed()
{
    static float normal_gyro_out_last = 0.0f;
    static uint8_t force_straight_last = 0;

    Left_PID_Out = 0;   // 每周期重置PID输出
    Right_PID_Out = 0;

    //（Stop_Flag 判断已统一由 Safety_Check 处理，此处不再检查）
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

    //===== 运行速度选择 =====
    Run_Speed = Select_Run_Speed();

    //===== 转弯时：速度已由 Turn_Left_Run/Turn_Right_Run 角度闭环PID设好 =====
    if (is_left == 1 || is_right == 1)
    {
        normal_gyro_out_last = 0.0f;
        force_straight_last = 0;
        PID_cleardata(&Gyro_PD_PID);
        PID_cleardata(&Turn_PID);
        // Left_Exp_Spd / Right_Exp_Spd / Gyro_PID_Out 已在turn函数中计算
        // 跳过正常级联PID，直接进入电机PID
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

            Gyro_PID_Out = PID_calc(&Gyro_PD_PID, 0.0f, Gyro_Z);
        }
        else
        {
            force_straight_last = 0;

            // Normal trace uses gyro rate as damping, not as a stateful inner loop.
            Turn_PID_Out = PID_calc(&Turn_PID, 0.0f, (float)Error);
            Gyro_PID_Out = PID_calc(&Gyro_PD_PID, Turn_PID_Out, Gyro_Z);
        }

        if (Gyro_PID_Out > normal_gyro_out_last + NORMAL_GYRO_OUT_STEP_MAX)
        {
            Gyro_PID_Out = normal_gyro_out_last + NORMAL_GYRO_OUT_STEP_MAX;
        }
        else if (Gyro_PID_Out < normal_gyro_out_last - NORMAL_GYRO_OUT_STEP_MAX)
        {
            Gyro_PID_Out = normal_gyro_out_last - NORMAL_GYRO_OUT_STEP_MAX;
        }
        normal_gyro_out_last = Gyro_PID_Out;

        // 左右期望速度 = 基础速度 ± 陀螺仪PID差速量
        Left_Exp_Spd = Run_Speed + Gyro_PID_Out;
        Right_Exp_Spd = Run_Speed - Gyro_PID_Out;
    }

    //===== 更新平均速度（用于里程计算）=====
    if (EnableSwitch_ON)
    {
        Average_Speed = (Left_Real_Spd + Right_Real_Spd) / 2.0;
        Count.Spd_Mileage += Average_Speed;
    }

    //===== 电机PID闭环控制 =====
    if (EnableSwitch_ON)
    {
        Left_PID_Out  = PID_calc(&Left_PID, (float)Left_Exp_Spd, (float)Left_Real_Spd);
        Right_PID_Out = PID_calc(&Right_PID, (float)Right_Exp_Spd, (float)Right_Real_Spd);
    }
}

/*************************************
** Function: Set_Out
** Description: 电机PWM输出控制（每3ms调用一次）
** Details:
*************************************/
void Set_Out(void)
{
    //===== 低压保护/停车已统一由 Safety_Check 处理，此处仅输出 PWM =====

    //===== 启动延时：使能后等待电机和编码器稳定 =====
    if (Enable_Start_Delay_Count > 0)
    {
        Enable_Start_Delay_Count--;  // 递减延时计数

        // 延时期间所有电机锁定
        pwm_set_duty(Suction_Motor_IN1, 9520);
        pwm_set_duty(Suction_Motor_IN2, 10000);  // 吸风电机以80%占空比运转
        pwm_set_duty(Left_Motor_IN1, 0);
        pwm_set_duty(Left_Motor_IN2, 0);
        pwm_set_duty(Right_Motor_IN1, 0);
        pwm_set_duty(Right_Motor_IN2, 0);

        PID_cleardata(&Left_PID);   // 清零左电机PID历史数据
        PID_cleardata(&Right_PID);  // 清零右电机PID历史数据
        return;                     // 跳过正常输出
    }

    if (EnableSwitch_ON && Stop_Flag == 0)
    {
        pwm_set_duty(Suction_Motor_IN1, 9500);
        pwm_set_duty(Suction_Motor_IN2, 10000);
    }
    else
    {
        pwm_set_duty(Suction_Motor_IN1, 0);
        pwm_set_duty(Suction_Motor_IN2, 10000);
    }

    //===== 驱动电机输出 =====
    if (EnableSwitch_ON && Stop_Flag == 0)  // 使能 + 非停车状态
    {
        //--- 左电机 ---
        if (Left_PID_Out == 0)
        {
            // PID输出=0：H桥两路同时PWM（短路制动）
            pwm_set_duty(Left_Motor_IN1, 0);
            pwm_set_duty(Left_Motor_IN2, 0);
        }
        else if (Left_PID_Out < 0)
        {
            pwm_set_duty(Left_Motor_IN1, 10000);
            pwm_set_duty(Left_Motor_IN2, 10000 - fabs(Left_PID_Out));
        }
        else  // Left_PID_Out < 0
        {
            pwm_set_duty(Left_Motor_IN1, 10000 - fabs(Left_PID_Out));
            pwm_set_duty(Left_Motor_IN2, 10000);
        }

        //--- 右电机 ---
        if (Right_PID_Out == 0)
        {
            // PID输出=0：H桥短路制动
            pwm_set_duty(Right_Motor_IN2, 0);
            pwm_set_duty(Right_Motor_IN1, 0);
        }
        else if (Right_PID_Out > 0)
        {
            // 正向驱动
            pwm_set_duty(Right_Motor_IN2, 10000);
            pwm_set_duty(Right_Motor_IN1, 10000 - fabs(Right_PID_Out));
        }
        else  // Right_PID_Out < 0
        {
            // 反向驱动
            pwm_set_duty(Right_Motor_IN2, 10000 - fabs(Right_PID_Out));
            pwm_set_duty(Right_Motor_IN1, 10000);
        }
    }
    else  // 未使能 或 停车状态
    {
        // 四路全关：电机自由滑行
        pwm_set_duty(Left_Motor_IN1, 0);
        pwm_set_duty(Left_Motor_IN2, 0);
        pwm_set_duty(Right_Motor_IN1, 0);
        pwm_set_duty(Right_Motor_IN2, 0);

        // 清零PID历史（下次启动时从零开始）
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
    }
}

/*************************************
** Function: Straight_Run
** Description: 直道运行模式（每3ms调用一次）
** Details:
*************************************/
void Straight_Run(void)
{
    gpio_set_level(P33_4, 1);           // 蜂鸣器响（直道模式中）
    Error = 0;                          // 零偏差直行
    Middle = Get_Track_Middle_Point();  // 获取当前中心点

    // 直道稳定判定：传感器2~4个 && 中心在中间区域(3~11)
    if (Track_Num < 5 && Track_Num > 1 && Middle > 3 && Middle < 11)
    {
        Count.Straight++;  // 连续居中，计数+1
    }
    else
    {
        Count.Straight = 0;  // 不满足则清零
    }

    if (Count.Straight > 0)  // 连续1次居中 → 直道稳定
    {
        if (Straight_Node_Pending != 0)  // 由Set_Node_Run_Mode触发的直道等待
        {
            Finish_Mileage_Section();      // 完成当前里程段

            Straight_Node_Pending = 0;  // 清除等待标志
            Count.Straight = 0;         // 清零直道计数
        }
        else  //
        {
            Run_Mode = Normal_Mode;  // 切回正常寻迹
            Count.Straight = 0;
        }
    }
}
