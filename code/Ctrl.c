/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.c
Author: Cross_Z
Version:0.0               Date: 2026.1.30
Description:  智能车核心控制程序
              - 光电传感器寻迹 + 编码器里程计 + 陀螺仪姿态控制
              - PID 闭环速度/转向控制
              - 建图模式(Build_Mode)：记录赛道转向里程到Flash
              - 回放模式(Remember_Mode)：从Flash加载里程复现运行
Others:      基于3ms定时中断调用 Car_Go() 主循环
Function List:
              主循环：Car_Go / Get_Speed / Get_IMU / Light_Process / Set_Speed / Set_Out
              寻迹：  Normal_Run / Straight_Run / Turn_Left_Run / Turn_Right_Run
              里程：  Mileage_Mode_Run / Mileage_Run_Stage_2
              建图：  Build_Mode_Get_Error
              回放：  Remember_Mode_Get_Error / Remember_Normal_Run / Remember_Check_Trigger
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
int Basic_Speed = 0;           // 基础速度设定值（建图模式使用，由键显Flash加载）
int Run_Speed = 0;             // 当前运行速度（建图=Basic_Speed, 回放=Remember_Get_Run_Speed()）
float Average_Speed = 0;       // 当前左右轮平均速度
float Last_Average_Speed = 0;  // 上一周期平均速度（用于加权里程计算）
int Speed_Get_Count = 1;       // 速度采集分频计数：每次 Car_Go 取反，==1时采集速度（6ms一次）
uint8 First_Mode = 0;          // 建图模式首次运行标志：0=未初始化，1=已完成初始化

/*---------------传感器数据----------------*/
uint8 Light_Convert[15] = {0};       // 15路光敏传感器二值化结果：0=黑线, 1=白线
uint8 Last_Light_Convert[15] = {0};  // 上一周期15路光敏传感器值

/*-----------------PID控制----------------*/
float Gyro_Z = 0;                          // 陀螺仪Z轴角速度（度/秒）
float Gyro_Z_For_PID = 0;                 // PID用陀螺仪值（raw/1000，与原工程一致）
PID_HandleTypeDef Gyro_PID = GYRO_PID;     // 陀螺仪PID实例（kp=0.008, 位置式, 限幅±500）
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
uint8_t Turn_Decel_Phase = 0;      // Build直角转弯阶段：0=直行中, 1=减速停车中, 2=原地旋转中
uint8_t Mileage_Turn_Done = 0;     // 里程计转向完成标志：0=转向中, 1=转向完成（里程模式下陀螺仪转角达标）
uint8_t Turn_Action_Done = 0;      // 转向动作完成标志：0=未完成, 1=已完成（传感器模式下出弯条件满足）
uint8_t Low_Voltage_Protect_Flag = 0; // 低压保护锁存：触发后关闭所有电机，复位前保持
uint16_t Low_Voltage_Beep_Count = 0;  // 低压蜂鸣器间歇计数
int Check_Edge_Skip_Count = 0;     // 边缘检测跳过计数（转向/里程切换后置20，逐周期递减，防误触发）
int Enable_Start_Delay_Count = 0;  // 启动延时计数（EnableSwitch_ON上升沿置100，递减期间电机锁定）
uint8_t Last_EnableSwitch_ON = 0;  // 上一周期使能开关状态（用于检测上升沿触发启动延时）
int Middle = 0;                    // 寻迹中心线位置（Track_Arr首尾传感器索引的平均值，0~14，理想值=7）
float Gyro_Integral = 0;           // 陀螺仪积分角度（转弯时累计角速度*3ms，出弯清零，单位：度）
float Mileage_Element_Base = 0;    // 里程计基础基准值
float Segment_Edge_Mileage_Record[NODE_NUM_MAX][ELEMENT_NUM_MAX] = {{0}}; // 各节点各元素边缘里程记录
float Segment_Total_Mileage[NODE_NUM_MAX + 1] = {0};                      // 各段路实测总里程（段起始到接触下一节点的距离）
float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX] = {0};                 // 转向间距里程记录数组（元素=两次转弯之间的里程差）
uint16_t Turn_Mileage_Record_Num = 0;   // 转向里程记录有效数量（建图时递增，回放时从Flash加载）
float Total_Run_Mileage = 0;            // 总运行里程（从发车开始累计，不重置，用于回放模式里程对比）
float Last_Turn_Mileage_Base = 0;       // 上一次转向里程基准（计算Turn_Mileage_Record[i] = Turn_Begin_Mileage - Last_Turn_Mileage_Base）
float Turn_Begin_Mileage = 0;           // 转向开始时的Total_Run_Mileage（进入转弯时记录）

/*---------------方向映射表----------------*/
// Dir_Arr[传感器索引] = 该传感器偏离中心的权重值
// 索引0(最左): -22, 索引7(中心): 0, 索引14(最右): +22
// 非对称设计：左侧权重大于右侧（左侧-22~-2 vs 右侧+2~+22），用于补偿机械不对称
int8_t Dir_Arr[15] = {-22, -21, -20, -18, -14, -9, -2, 0, 2, 9, 14, 18, 20, 21, 22};
int Turn_Error_Value = 40;  // 转向PWM差值 / 转弯Error值（出弯时Error跳变产生反向阻尼）
int16 Check_Edge_Count = 0;   // Check_Edge触发次数（VOFA调试用）
uint8_t Force_Straight_Speed = 0;  // 直行元器件强制基础速度标志

/*---------------前瞻光电布局----------------*/
// 当前硬件为双排前瞻：6/7/8 在第二排且识别不稳定，暂不参与循迹/节点判定。
// 仍保留 Light_Convert[6..8] 供 VOFA/OLED 观察；控制只使用第一排 12 路。
#define TRACK_SENSOR_ACTIVE_NUM 12
static const uint8_t Track_Sensor_Active_Index[TRACK_SENSOR_ACTIVE_NUM] =
{
    0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14
};

/*-------------赛道运行状态结构体-------------*/
Racing_track_Typedef Run_Track;  // 当前赛道运行结构体（运行时数据，由键显输入或Flash加载填充）
int8_t Execute_Times = 0;        // 当前执行节点索引（0 ~ Node_Num，指向 Run_Track 的 Node_Arr_* 数组）
int8_t Mileage_Times = 0;        // 当前节点里程段总数（= Run_Track.Node_Arr_Mileage_Num[Execute_Times]）
uint8_t Line_Num_Count = 0;      // 已完成线路计数（累计通过的节点数，用于Stop_Mode==1时判断完成）
uint8_t In_Line_Ele_Count = 0;   // 当前线路内元素索引（0 ~ Mileage_Times-1，遍历每个节点的里程段）

/*----------------PID输出----------------*/
float Turn_PID_Out = 0.0;    // 转向PID输出（Error → PID → 用于差速控制的基础值）
float Gyro_PID_Out = 0.0;    // 陀螺仪PID输出（Turn_PID_Out + Gyro_Z系数 → PID → 最终左右轮差速量）
float Left_PID_Out = 0.0;    // 左电机PID输出（期望速度 vs 实际速度 → PWM占空比调节量）
float Right_PID_Out = 0.0;   // 右电机PID输出（期望速度 vs 实际速度 → PWM占空比调节量）
// 运行模式
Run_Mode_Enum Run_Mode = Normal_Mode;         // 当前运行模式
Run_Mode_Enum Last_Run_Mode = Normal_Mode;     // 上一周期运行模式
Mileage_Stage_Enum Mileage_Stage = Normal_Stage; // 里程计运行阶段（Normal_Stage=正常, Straight_Stage=直行）
Mode_Define Mode = Remember_Mode;              // 键盘显示工作模式（Build_Mode=建图, Remember_Mode=回放）

/*---------------转向控制参数（陀螺仪单位：真实 °/s 和 °）----------------*/
// IMU660RB: ±2000dps量程, transition_factor=14.3, 即 raw/14.3 = °/s
// Gyro_Integral = Σ(°/s × 0.003s) = 真实角度(°)
// Gyro_Z = imu660rb_gyro_transition(raw) = 真实 °/s
// Gyro_Z_PID_SCALE: Gyro_Z(°/s) 除以该系数后输入PID，保持与旧Gyro_Z(raw/1000)相同量级
// 旧: raw/1000 ≈ ±28.6, 新: raw/14.3 ≈ ±2000, 比值=2000/28.6≈70
#define GYRO_Z_PID_SCALE 70.0f             // Gyro_Z输入PID前的缩放系数
#define TURN_BASE_SPD_MIN 40               // 转向基础最小速度
#define TURN_BASE_SPD_MAX 40               // 转向基础最大速度（=MIN，转向速度恒定40）
#define BUILD_TURN_TARGET_ANGLE_DEG    75.0f   // Build模式转向目标角度
#define REMEMBER_TURN_TARGET_ANGLE_DEG 75.0f   // Remember模式转向目标角度
#define TURN_STRAIGHT_PRE_DISTANCE 300.0f  // 直角转弯前直行距离（可调参）
#define REMEMBER_TURN_INNER_SCALE 1.4f     // Remember转向内侧轮系数 (>1.0=多减速)
#define REMEMBER_TURN_OUTER_SCALE 0.6f     // Remember转向外侧轮系数 (<1.0=少加速)
// 约束: INNER+OUTER=2.0 保持总角速度不变
#define TURN_BASE_SPD_ZERO_ANGLE 10.0f     // 转向速度斜坡起始角度(°)：10°起开始加速
#define TURN_BASE_SPD_END_ANGLE 70.0f      // 转向速度斜坡结束角度(°)：70°达到最高速
#define TURN_BASE_PWM 0                    // 转弯基础PWM偏移（叠加到速度PID输出上，保证基础驱动力）
#define MIDDLE_COEFFICIENT_MIN 1.0f        // 中心线系数最小值
#define MIDDLE_COEFFICIENT_MAX 1.9f        // 中心线系数最大值
#define MIDDLE_COEFFICIENT_CENTER 7.0f     // 中心线基准值（传感器索引7=物理中心）

/*---------------记忆模式可调参数----------------*/
float Remember_Mileage_Prepare_Distance = 120.0f;  // 记忆模式路段内元素准备距离（提前120单位）
float Remember_Node_Prepare_Distance = 500.0f;      // 记忆模式节点准备距离（提前触发节点转弯）
int Remember_Turn_Error = 40;                       // 记忆模式直角转弯固定Error值
int Remember_Speed_Min_Value = 70;                  // 记忆模式最小速度（起步/减速末端速度）
int Remember_Speed_Max_Value = 85;                  // 记忆模式最大速度（中间匀速段速度）
uint8 vofa_flash_dump_mode = 0;                     // VOFA Flash数据导出模式：0=正常, 1=导出

#define MILEAGE_COMPENSATION_X (-100.0f)  // 转弯元素里程补偿值（建图记录时减去此值，回放时提前触发）
#define MILEAGE_STRAIGHT_SHORT 500.0f    // 短直行元素里程
#define MILEAGE_STRAIGHT_LONG  800.0f    // 长直行元素里程
#define MILEAGE_ELEMENT_TURN_DELAY 450.0f // 元器件转弯前直走延迟距离

#define CHECK_EDGE_SKIP_INIT            0   // 初始化/重置时清零

// Build模式去抖（可调参）
#define BUILD_CHECK_EDGE_NODE_TURN       1  // 节点转弯后
#define BUILD_CHECK_EDGE_MILEAGE_STRAIGHT 2 // 里程直行元器件后
#define BUILD_CHECK_EDGE_MILEAGE_TURN    25  // 里程转弯元器件后

// Remember模式去抖（可调参）
#define REMEMBER_CHECK_EDGE_NODE_TURN       6   // 节点转弯后
#define REMEMBER_CHECK_EDGE_MILEAGE_STRAIGHT 0  // 里程直行元器件后
#define REMEMBER_CHECK_EDGE_MILEAGE_TURN    0   // 里程转弯元器件后

#define LOW_VOLTAGE_PROTECT_VALUE      11.5f // 低压保护阈值
#define LOW_VOLTAGE_BEEP_PERIOD_COUNT  200   // 3ms*200=600ms
#define LOW_VOLTAGE_BEEP_ON_COUNT      60    // 3ms*60=180ms

/*---------------记忆模式速度曲线参数----------------*/
// 速度曲线：0%~5%低速 → 5%~10%加速 → 10%~95%匀速 → 95%~100%减速
#define REMEMBER_SPEED_LOW_RATIO 0.05f   // 低速段比例（前5%里程保持最低速度）
#define REMEMBER_SPEED_RAMP_RATIO 0.10f  // 加速段终点比例（5%~10%里程从最低速度线性加速到最高速度）
#define REMEMBER_SPEED_DECEL_RATIO 0.95f // 减速段起点比例（95%里程开始从最高速度线性减速到最低速度）
#define GYRO_INTEGRATION_PERIOD_S 0.003f // 陀螺仪积分周期（=3ms中断周期，Gyro_Z(°/s) × 0.003s = 本次角度增量(°)）

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
    float Segment_Edge_Mileage_Record[NODE_NUM_MAX][ELEMENT_NUM_MAX]; // 边缘里程二维数组
    float Segment_Total_Mileage[NODE_NUM_MAX + 1];                    // 各段路实测总里程
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

/*---------------记忆模式静态变量----------------*/
static uint8_t Remember_First_Mode = 0;           // 记忆模式首次运行标志：0=未初始化, 1=已从Flash加载并重置状态
static uint16_t Remember_Turn_Record_Index = 0;   // 记忆模式当前转向记录索引（指向 Turn_Mileage_Record[]）
float Remember_Next_Target_Mileage = 0;    // 记忆模式下一个目标里程（累计值，每次Advance累加）
static float Remember_Section_Base_Mileage = 0;   // 记忆模式当前路段基准里程（=进入路段时的Total_Run_Mileage）
static uint8_t Straight_Node_Pending = 0;         // 直道节点等待标志：0=无等待, 1=等待直道稳定触发Finish
static uint8_t Remember_Edge_Snap_Latched = 0;    // 记忆模式边缘锁定标志：0=未锁定, 1=已锁定（防重复Snap）

/*---------------内部静态函数声明---------------*/
static void Load_Turn_Mileage_Record_From_Flash(void);
static void Save_Segment_Edge_Mileage_Record_To_Flash(void);
static void Load_Segment_Edge_Mileage_Record_From_Flash(void);
static void Save_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, const uint32_t *words);
static void Load_Flash_Page_Block(uint8_t sector, uint8_t start_page, uint8_t page_count, uint16_t words_per_page, uint32_t *words);
static void Remember_Reset_Runtime_State(void);
static void Remember_Advance_Turn_Record(void);
static void Remember_Check_Trigger(void);
static void Remember_Normal_Run(void);
static int Remember_Get_Run_Speed(void);
static float Remember_Get_Current_Edge_Mileage(void);
static void Remember_Snap_Mileage_To_Current_Edge(void);
static void Advance_Turn_Section_Index(void);
static void Advance_To_Next_Track_Segment(void);
static void Reset_Turn_Action_State(void);
static void Complete_Turn_Action(void);
static void Record_Turn_Mileage(void);
static uint8_t Is_Track_Sensor_Adjacent(uint8_t left_index, uint8_t right_index);

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

/*************************************
** Function: Advance_Turn_Section_Index
** Description: 转向路段索引递增（预留扩展点，当前为空）
*************************************/
static void Advance_Turn_Section_Index(void)
{
}

/*************************************
** Function: Advance_To_Next_Track_Segment
** Description: 切换到下一个赛道路段
** Details:   1. Execute_Times = (Execute_Times + 1) % (Node_Num + 1)  循环推进节点索引
**            2. 更新 Mileage_Times 为下一节点的里程段数量
**            3. Stop_Mode==0（串行赛道）时：Execute_Times 回到 Node_Num 判定完成
**               Stop_Mode==1（并行赛道）时：由 Line_Num_Count 判断完成
*************************************/
static void Advance_To_Next_Track_Segment(void)
{
    // 循环递增节点索引（超出Node_Num后回到0，实现赛道循环）
    Execute_Times = (Execute_Times + 1) % (Run_Track.Node_Num + 1);
    // 加载新节点的里程段数量
    Mileage_Times = Run_Track.Node_Arr_Mileage_Num[Execute_Times];
    In_Line_Ele_Count = 0;  // 切换到新节点，元素序号归零

    // Remember模式：新增路段里程加入目标（直行/转弯节点统一走这里）
    if (Mode == Remember_Mode)
    {
        Remember_Next_Target_Mileage += Segment_Total_Mileage[Execute_Times];
    }

    // 串行赛道模式下：绕完一圈（Execute_Times回到Node_Num）即完成
    if (Run_Track.Stop_Mode == 0 && Execute_Times == Run_Track.Node_Num)
    {
        Finish_Flag = 1;  // 设置任务完成标志
    }
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
    Count.Left = 0;              // 清零左转出弯计数
    Count.Right = 0;             // 清零右转出弯计数
    is_left = 0;                 // 清除左转标志
    is_right = 0;                // 清除右转标志
    Turn_Decel_Phase = 0;        // 清除转弯减速阶段
    Check_Edge_Skip_Count = (Mode == Build_Mode) ? BUILD_CHECK_EDGE_NODE_TURN : REMEMBER_CHECK_EDGE_NODE_TURN;
    Run_Mode = Normal_Mode;      // 恢复到正常寻迹模式
}

/*************************************
** Function: Complete_Turn_Action
** Description: 完成转向动作的统一入口
** Details:   1. 标记 Turn_Action_Done
**            2. 建图模式下记录转向里程到Flash
**            3. 回放模式下推进转向记录索引并切换到下一路段
**            4. 最后调用 Reset_Turn_Action_State 清理状态
** 调用链：   Turn_Left_Run / Turn_Right_Run → Complete_Turn_Action → Record_Turn_Mileage
*************************************/
static void Complete_Turn_Action(void)
{
    Turn_Action_Done = 1;        // 标记转向动作完成
    Record_Turn_Mileage();       // 记录本次转向里程（建图模式才实际记录，回放模式直接return）

    if (Mode == Remember_Mode)   // 回放模式：补偿转弯内弧线里程 + 切换路段
    {
        if (Execute_Times < NODE_NUM_MAX)
        {
            Total_Run_Mileage += 2 * Remember_Node_Prepare_Distance; // 补偿转弯弧线缺失的里程
            Advance_To_Next_Track_Segment(); // 已含 In_Line_Ele_Count=0 + Target里程更新
            Remember_Section_Base_Mileage = Total_Run_Mileage;
        }
    }
    else  // 建图模式：同样在转弯完成后推进节点
    {
        if (Execute_Times < NODE_NUM_MAX)
        {
            Advance_To_Next_Track_Segment(); // 推进到下一节点，In_Line_Ele_Count 归零
        }
    }

    Reset_Turn_Action_State();   // 统一重置转向状态（Run_Mode变回Normal_Mode）
}

/*************************************
** Function: Get_Turn_Base_Speed
** Description: 获取转向基础速度（根据当前陀螺仪积分角度线性插值）
** Details:   角度 ≤ 10.0(TURN_BASE_SPD_ZERO_ANGLE) → 返回 MIN
**            角度 ≥ 70.0(TURN_BASE_SPD_END_ANGLE)  → 返回 MAX
**            角度在两者之间 → 线性插值
**            当前 MIN==MAX==40，实际始终返回40，斜坡逻辑为预留扩展
*************************************/
static int Get_Turn_Base_Speed(void)
{
    float turn_angle = fabsf(Gyro_Integral);  // 取陀螺仪积分角度的绝对值

    // 配置检查：结束角度不应小于等于起始角度（否则斜坡方向错误）
    if (TURN_BASE_SPD_END_ANGLE <= TURN_BASE_SPD_ZERO_ANGLE)
    {
        return TURN_BASE_SPD_MAX;  // 配置异常时返回最大速度兜底
    }

    if (turn_angle <= TURN_BASE_SPD_ZERO_ANGLE)
    {
        return TURN_BASE_SPD_MIN;  // 角度小 → 最低速度
    }

    if (turn_angle >= TURN_BASE_SPD_END_ANGLE)
    {
        return TURN_BASE_SPD_MAX;  // 角度大 → 最高速度
    }

    // 线性插值：speed = MIN + (MAX-MIN) × (当前角度 - 起始角度) / (结束角度 - 起始角度)
    return TURN_BASE_SPD_MIN + (int)((TURN_BASE_SPD_MAX - TURN_BASE_SPD_MIN)
                                     * (turn_angle - TURN_BASE_SPD_ZERO_ANGLE)
                                     / (TURN_BASE_SPD_END_ANGLE - TURN_BASE_SPD_ZERO_ANGLE));
}

/*************************************
** Function: Get_Middle_Coefficient
** Description: 获取陀螺仪角速度的中心线加权系数
** Details:   小车偏离中心越远，系数越大（1.0 ~ 1.9），Gyro_Z 被放大更多
**            计算公式：coeff = MAX - (MAX-MIN) × (|Middle-7|/7)^2
**            即中心(Middle=7)时系数=MAX=1.9，边缘时系数趋近MIN=1.0
**            使用平方函数使中心附近灵敏度更高（二次曲线非直线）
*************************************/
static float Get_Middle_Coefficient(void)
{
    int middle_offset = abs(Middle - 7);  // 当前中心点偏离理想中心(7)的距离
    float normalized_offset;

    if (middle_offset > 7)                // 最多偏离7（传感器范围 0~14，最大偏离=7）
    {
        middle_offset = 7;
    }

    normalized_offset = (float)middle_offset / MIDDLE_COEFFICIENT_CENTER; // 归一化到 [0, 1]

    // 二次函数：coeff = 1.9 - 0.9 × offset²
    // offset=0(中心) → coeff=1.9  offset=1(边缘) → coeff=1.0
    return MIDDLE_COEFFICIENT_MAX - (MIDDLE_COEFFICIENT_MAX - MIDDLE_COEFFICIENT_MIN)
                                   * normalized_offset * normalized_offset;
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
    Mileage_Turn_Done = 0;     // 清零里程转向完成标志
    Turn_Action_Done = 0;      // 清零传感器转向完成标志
    Turn_Decel_Phase = 0;      // 清零转弯减速阶段
    Count.Spd_Mileage = 0;     // 清零速度里程
    Count.Mileage = 0;         // 清零当前段里程
    Mileage_Element_Base = 0;  // 清零里程基准值
    Straight_Node_Pending = 0; // 清零直道等待标志

    switch (node_dir)
    {
        case 1:  // 左转节点
            Run_Mode = Turn_Left;                     // 切换到左转模式
            is_left = 1;                               // 设置左转标志（触发陀螺仪积分）
            is_right = 0;                              // 清除右转标志
            Turn_Begin_Mileage = Total_Run_Mileage;    // 记录转向起始时的总里程
            Advance_Turn_Section_Index();              // 直道节点计数+1
            break;
        case 2:  // 右转节点
            Run_Mode = Turn_Right;                    // 切换到右转模式
            is_left = 0;                               // 清除左转标志
            is_right = 1;                              // 设置右转标志（触发陀螺仪积分）
            Turn_Begin_Mileage = Total_Run_Mileage;    // 记录转向起始时的总里程
            Advance_Turn_Section_Index();              // 直道节点计数+1
            break;
        case 0:   // 直行节点（0=无转向，fall-through）
        default:
            Straight_Node_Pending = 1; // 设置直道等待标志（等直道稳定后触发Finish）
            Run_Mode = Straight_Mode;   // 切换到直道模式
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
    uint8_t mileage_num = Run_Track.Node_Arr_Mileage_Num[Execute_Times]; // 当前节点的里程段总数

    // 清除转向相关标志
    is_left = 0;
    is_right = 0;
    Force_Straight_Speed = 0;  // 清除直行强制速度标志
    Mileage_Turn_Done = 0;
    Check_Edge_Skip_Count = (Mode == Build_Mode) ? BUILD_CHECK_EDGE_MILEAGE_STRAIGHT : REMEMBER_CHECK_EDGE_MILEAGE_STRAIGHT;
    Run_Mode = Normal_Mode;      // 恢复到正常寻迹模式

    Mileage_Element_Base = Count.Mileage;  // 更新里程基准

    // 直道节点：无里程段( mileage_num==0 )且是由Set_Node_Run_Mode触发的(Straight_Node_Pending!=0)
    if (Straight_Node_Pending != 0 && mileage_num == 0)
    {
        Straight_Node_Pending = 0;  // 清除等待标志
        Line_Num_Count++;           // 直接完成一条线路
        return;
    }

    In_Line_Ele_Count++;  // 当前线路内的元素计数+1

    // 当前线路所有元素都完成后：线路+1
    // 注意：不在此处清零 In_Line_Ele_Count，让其保持 == mileage_num
    // 这样下次边缘检测时会走入 else 分支，正确触发节点动作
    // In_Line_Ele_Count 的清零由 Advance_To_Next_Track_Segment 负责
    if (In_Line_Ele_Count == mileage_num)
    {
        Line_Num_Count++;
    }

    // 并行赛道模式(Stop_Mode==1)：线路数达到总节点数即完成
    if (Run_Track.Stop_Mode == 1)
    {
        if (Line_Num_Count == Run_Track.Node_Num)
        {
            Finish_Flag = 1;  // 标记任务完成
        }
    }

    // 回放模式：仅转弯元器件完成后更新速度基准，直行元器件不需要
    if (Mode == Remember_Mode && In_Line_Ele_Count == mileage_num)
    {
        uint8_t finished_dir = Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count - 1];
        if (finished_dir == 1 || finished_dir == 2)
        {
            Remember_Section_Base_Mileage = Total_Run_Mileage;
        }
    }
}

/*************************************
** Function: Record_Segment_Edge_MileageSegment_Total_Mileage
** Description: 记录当前路段边缘的里程值
** Details:
*************************************/
static void Record_Segment_Edge_Mileage(void)
{
    if (Execute_Times < NODE_NUM_MAX && In_Line_Ele_Count < ELEMENT_NUM_MAX)
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
** Function: Remember_Advance_Turn_Record
** Description: 记忆模式转向记录推进
** Details:   根据 Turn_Mileage_Record 数组推进索引并更新目标里程：
**            1. 记录数为0 → 索引=0, 目标=0（无效状态）
**            2. 还有下一个记录 → 索引+1, 目标里程累加
**            3. 已是最后一个记录 → 索引置为总数, 目标里程设极大值(1000000)
**            每次推进后更新当前路段基准里程
*************************************/
static void Remember_Advance_Turn_Record(void)
{
    if (Turn_Mileage_Record_Num == 0)   // 无转向记录（Flash为空或建图未完成）
    {
        Remember_Turn_Record_Index = 0;
        Remember_Next_Target_Mileage = 0;
        return;
    }

    if (Remember_Turn_Record_Index + 1 < Turn_Mileage_Record_Num)  // 还有下一个记录
    {
        Remember_Turn_Record_Index++;                                // 索引推进
        Remember_Next_Target_Mileage += Turn_Mileage_Record[Remember_Turn_Record_Index]; // 累加目标里程
    }
    else  // 已经是最后一个记录
    {
        Remember_Turn_Record_Index = Turn_Mileage_Record_Num;        // 索引置为总数（超出有效范围）
        Remember_Next_Target_Mileage = 1000000.0f;                   // 设极大值（永远不会触发里程触发）
    }

    Remember_Section_Base_Mileage = Total_Run_Mileage;  // 更新路段基准里程
}

/*************************************
** Function: Remember_Get_Current_Edge_Mileage
** Description: 获取当前节点/元素对应的边缘里程记录值
** Return:     记录的边缘里程值（无有效记录时返回0）
** Details:   从 Segment_Edge_Mileage_Record 二维数组读取
*************************************/
static float Remember_Get_Current_Edge_Mileage(void)
{
    if (Execute_Times < NODE_NUM_MAX && In_Line_Ele_Count < ELEMENT_NUM_MAX)
    {
        return Segment_Edge_Mileage_Record[Execute_Times][In_Line_Ele_Count];
    }

    return 0.0f;  // 索引越界时返回0（安全兜底）
}

/*************************************
** Function: Remember_Snap_Mileage_To_Current_Edge
** Description: 将当前里程同步到记录的边缘里程值
** Details:   回放模式下检测到物理边缘时，将里程计数"跳变"到建图时记录的对应位置。
**            这样做是为了消除回放行驶与建图行驶之间的里程累积误差。
**            注意：Total_Run_Mileage 可能反向调整（target < current 时 mileage_offset 为负）
*************************************/
static void Remember_Snap_Mileage_To_Current_Edge(void)
{
    float target_mileage = Remember_Get_Current_Edge_Mileage();  // 获取建图时记录的边缘位置
    float mileage_offset;

    mileage_offset = target_mileage - Count.Mileage;  // 计算里程偏移量（可正可负）
    Count.Mileage = target_mileage;                   // 跳变到目标里程
    Total_Run_Mileage += mileage_offset;              // 同步调整总里程
    Remember_Next_Target_Mileage += mileage_offset;   // 同步调整目标里程
    Mileage_Element_Base = target_mileage;            // 更新里程基准
}

/*************************************
** Function: Remember_Reset_Runtime_State
** Description: 重置记忆模式的所有运行时状态
** Details:
*************************************/
static void Remember_Reset_Runtime_State(void)
{
    // 转向记录索引初始化：读第一条记录的目标里程
    Remember_Turn_Record_Index = 0;
    Remember_Next_Target_Mileage = Segment_Total_Mileage[0];  // 第一个路段的实测总里程
    Remember_Section_Base_Mileage = 0;

    // 路段索引初始化
    Execute_Times = 0;
    Mileage_Times = Run_Track.Node_Arr_Mileage_Num[Execute_Times];
    Line_Num_Count = 0;
    In_Line_Ele_Count = 0;

    // 运行模式初始化
    Run_Mode = Normal_Mode;
    Last_Run_Mode = Normal_Mode;

    // 计数结构体清零
    Count.Left = 0;
    Count.Right = 0;
    Count.Stop = 0;
    Count.Stall = 0;
    Count.Straight = 0;
    Count.Mileage = 0;
    Count.Spd_Mileage = 0;

    // 状态标志清零
    Finish_Count = 0;
    Finish_Flag = 0;
    Stop_Flag = 0;

    // 传感器/姿态状态清零
    Gyro_Integral = 0;
    Turn_Action_Done = 0;
    Mileage_Turn_Done = 0;
    is_left = 0;
    is_right = 0;
    Check_Edge_Skip_Count = CHECK_EDGE_SKIP_INIT;
    Enable_Start_Delay_Count = 0;
    Mileage_Element_Base = 0;
    Total_Run_Mileage = 0;
    Last_Turn_Mileage_Base = 0;
    Straight_Node_Pending = 0;
    Remember_Edge_Snap_Latched = 0;

    // PID输出清零
    Turn_PID_Out = 0.0f;
    Gyro_PID_Out = 0.0f;
    Left_PID_Out = 0.0f;
    Right_PID_Out = 0.0f;
}

/*************************************
** Function: Remember_Normal_Run
** Description: 记忆模式下的正常寻迹（替代 Normal_Run）
** Details:   与 Normal_Run 的区别：
**            1. Track_Num < 2 时：同上，用Last_Error推断方向
**            2. 2 <= Track_Num < 7 时：用边界传感器计算偏差
**            3. Track_Num >= 7 时：用 Left_Scan_Point/Right_Scan_Point 计算偏差
**            注意：不调用 Check_Edge()，边缘触发由 Remember_Check_Trigger 独立处理
**            Turn_Left/Right_Run等由 Remember_Mode_Get_Error 中的switch调度
*************************************/
static void Remember_Normal_Run(void)
{
    gpio_set_level(P33_4, 0);    // 蜂鸣器关
    Last_Error = Error;          // 保存上一周期偏差（用于传感器丢失时的方向保持）

    if (Track_Num < 2)           // 传感器<2个：几乎丢失赛道，沿用上次偏差方向
    {
        Error = (Last_Error >= 0) ? 30 : -30;  // 保持上一方向但不使用上次的精确值
    }
    else if (Track_Num < 5 && Track_Num >= 2)  // 传感器2~4个：少量白线，用边界计算偏差
    {
        Left_Scan_Point = Track_Arr[0];                       // 最左白线传感器索引
        Right_Scan_Point = Track_Arr[Track_Num - 1];          // 最右白线传感器索引
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2; // 首尾权重取平均
    }
    else  // Track_Num >= 5：正常寻迹，用历史边界点计算
    {
        // 注意：Left_Scan_Point/Right_Scan_Point 可能来自上一周期
        //       若首周期 Track_Num >= 5，则使用初始值0（传感器最左），偏差会偏向左侧
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
    }
}

/*************************************
** Function: Remember_Get_Run_Speed
** Description: 获取记忆模式下的运行速度（梯形速度曲线）
** Return:     当前应使用的速度值 [Remember_Speed_Min_Value ~ Remember_Speed_Max_Value]
** Details:   速度曲线分4段（基于已行驶里程占路段总里程的比例）：
**            [0%  ~ 5%]  低速段：Remember_Speed_Min_Value
**            [5%  ~ 10%] 加速段：线性从 Min 加速到 Max
**            [10% ~ 95%] 匀速段：Remember_Speed_Max_Value
**            [95% ~ 100%]减速段：线性从 Max 减速到 Min
**            路段总里程 = Remember_Next_Target_Mileage - Remember_Section_Base_Mileage
*************************************/
static int Remember_Get_Run_Speed(void)
{
    float section_total_mileage;     // 当前路段总里程
    float processed_mileage;         // 已行驶里程（已处理部分）
    float effective_total_mileage;   // 有效总里程（扣除准备距离后用于速度曲线计算）

    // 参数检查：最大速度必须大于最小速度
    if (Remember_Speed_Max_Value <= Remember_Speed_Min_Value)
    {
        return Remember_Speed_Min_Value;
    }

    // 计算当前路段总里程（目标里程 - 路段基准里程）
    section_total_mileage = Remember_Next_Target_Mileage - Remember_Section_Base_Mileage;
    if (section_total_mileage <= 0)
    {
        return Remember_Speed_Min_Value;  // 无效里程，安全低速
    }

    effective_total_mileage = section_total_mileage;
    if (Run_Mode == Normal_Mode)  // 仅在Normal_Mode下考虑准备距离
    {
        uint8_t mileage_num = Run_Track.Node_Arr_Mileage_Num[Execute_Times];

        if (In_Line_Ele_Count < mileage_num)
        {
            // 还在路段内部：检查下一个元素类型决定使用哪种准备距离
            uint8_t next_dir = Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count];
            if (next_dir == 0)
            {
                // 下一个是普通路段(dir=0)，说明前方直通节点 → 使用节点准备距离
                effective_total_mileage -= Remember_Node_Prepare_Distance;
            }
            else
            {
                // 下一个是元器件(1/2/3/4) → 使用元素准备距离
                effective_total_mileage -= Remember_Mileage_Prepare_Distance;
            }
        }
        else if (Run_Track.Node_Arr_Dir[Execute_Times] != 0)
        {
            // 已走完路段内元素且下一节点是转向：减去节点转向准备距离
            effective_total_mileage -= Remember_Node_Prepare_Distance;
        }
    }

    if (effective_total_mileage <= 0)
    {
        return Remember_Speed_Min_Value;  // 有效里程被准备距离消耗完，低速行驶
    }

    // 已行驶里程 = 当前总里程 - 路段基准里程，并钳位到有效范围
    processed_mileage = Total_Run_Mileage - Remember_Section_Base_Mileage;
    if (processed_mileage < 0)
    {
        processed_mileage = 0;  // 负值钳位到0（不应出现，但做防御）
    }
    if (processed_mileage > effective_total_mileage)
    {
        processed_mileage = effective_total_mileage;  // 超出钳位到最大值
    }

    //===== 第1段：低速段 [0% ~ 5%] =====
    if (processed_mileage <= effective_total_mileage * REMEMBER_SPEED_LOW_RATIO)
    {
        return Remember_Speed_Min_Value;
    }

    //===== 第2段：加速段 [5% ~ 10%] =====
    if (processed_mileage <= effective_total_mileage * REMEMBER_SPEED_RAMP_RATIO)
    {
        // 线性插值：ratio = (当前 - 低速终点) / (加速段长度)
        float ratio = (processed_mileage - effective_total_mileage * REMEMBER_SPEED_LOW_RATIO)
                      / (effective_total_mileage * (REMEMBER_SPEED_RAMP_RATIO - REMEMBER_SPEED_LOW_RATIO));
        return Remember_Speed_Min_Value + (int)((Remember_Speed_Max_Value - Remember_Speed_Min_Value) * ratio);
    }

    //===== 第3段：匀速段 [10% ~ 95%] =====
    if (processed_mileage < effective_total_mileage * REMEMBER_SPEED_DECEL_RATIO)
    {
        return Remember_Speed_Max_Value;
    }

    //===== 第4段：减速段 [95% ~ 100%] =====
    {
        // 线性插值：ratio = (剩余里程) / (减速段长度)
        float ratio = (effective_total_mileage - processed_mileage)
                      / (effective_total_mileage * (1.0f - REMEMBER_SPEED_DECEL_RATIO));
        return Remember_Speed_Min_Value + (int)((Remember_Speed_Max_Value - Remember_Speed_Min_Value) * ratio);
    }
}

/*************************************
** Function: Remember_Check_Trigger
** Description: 记忆模式下的触发检测（替代 Normal_Run 中的 Check_Edge 触发）
** Details:   两套触发机制并行：
**            1. 边缘触发：Check_Edge() 检测到物理边缘 → 立即触发里程模式/节点切换
**            2. 里程触发：Total_Run_Mileage >= 目标里程 - 准备距离 → 提前进入里程模式
**            逻辑分支：
**            - 路段内还有元素(In_Line_Ele_Count < mileage_num)：
**              · 元素方向=0且节点方向=0 → 仅边缘触发直道
**              · 元素方向=0且节点方向≠0 → 里程触发转向
**              · 元素方向≠0 → 边缘触发(立即)或里程触发(准备距离)
**            - 路段内元素已完(In_Line_Ele_Count == mileage_num)：
**              · 节点方向=0 → 仅边缘触发直道
**              · 节点方向≠0 → 里程触发转向
*************************************/
static void Remember_Check_Trigger(void)
{
    float trigger_mileage;     // 触发里程阈值
    uint8_t mileage_dir;       // 当前元素的方向类型
    uint8_t edge_hit;          // 边缘检测结果：0=未检测到, 1=检测到

    // 无建图路段里程数据：无效状态，直接返回
    if (Segment_Total_Mileage[0] == 0)
    {
        return;
    }

    edge_hit = Check_Edge();  // 执行一次边缘检测（复用 Check_Edge_Skip_Count 防抖）

    //===== 情况A：当前线路内还有元素未处理 =====
    if (In_Line_Ele_Count < Run_Track.Node_Arr_Mileage_Num[Execute_Times])
    {
        trigger_mileage = Remember_Next_Target_Mileage;  // 默认用总里程触发
        mileage_dir = Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count]; // 当前元素方向

        if (mileage_dir == 0)  // 元素方向=0（非转向元素/直行元素）
        {
            if (Run_Track.Node_Arr_Dir[Execute_Times] == 0)  // 节点方向也是0（纯直道）
            {
                if (Track_Num >= 5)  // Remember模式直道节点用Track_Num判定
                {
                    In_Line_Ele_Count = Run_Track.Node_Arr_Mileage_Num[Execute_Times]; // 标记所有路段已完成
                    Set_Node_Run_Mode(Run_Track.Node_Arr_Dir[Execute_Times]); // 触发直道模式
                }
                return;
            }
            // 节点方向≠0（当前段是直行，但下一节点是转向）
            if (Total_Run_Mileage >= trigger_mileage - Remember_Node_Prepare_Distance)
            {
                In_Line_Ele_Count = Run_Track.Node_Arr_Mileage_Num[Execute_Times]; // 标记所有路段已完成
                Set_Node_Run_Mode(Run_Track.Node_Arr_Dir[Execute_Times]); // 里程触发转向
            }
            return;
        }

        // 元素方向≠0（1左转/2右转/3短直行/4长直行）
        trigger_mileage = Remember_Get_Current_Edge_Mileage();  // 改用边缘里程作为触发基准

        if (edge_hit)  // 物理边缘优先触发（更精确）
        {
            Remember_Snap_Mileage_To_Current_Edge();  // 里程校准：跳变到建图记录位置
            Run_Mode = Mileage_Mode;                  // 进入里程模式
            Mileage_Element_Base = Count.Mileage;     // 记录里程基准
            Mileage_Turn_Done = 0;                    // 重置转向完成标志
            Turn_Action_Done = 0;                     // 重置动作完成标志
            return;
        }

        // 转弯元器件(1/2)：未检测到边缘时用里程提前触发；直行元器件(3/4)：只用边缘触发
        if ((mileage_dir == 1 || mileage_dir == 2)
            && Count.Mileage >= trigger_mileage - Remember_Mileage_Prepare_Distance)
        {
            Run_Mode = Mileage_Mode;
            Mileage_Element_Base = Count.Mileage;
            Mileage_Turn_Done = 0;
            Turn_Action_Done = 0;
        }
    }
    //===== 情况B：当前线路所有元素已完成 =====
    else
    {
        trigger_mileage = Remember_Next_Target_Mileage;

        if (Run_Track.Node_Arr_Dir[Execute_Times] == 0)  // 节点方向=0（直行）
        {
            if (Track_Num >= 5)  // Remember模式直道节点用Track_Num判定
            {
                Set_Node_Run_Mode(Run_Track.Node_Arr_Dir[Execute_Times]);
            }
            return;
        }

        // 节点方向≠0（转向）：里程触发
        if (Total_Run_Mileage >= trigger_mileage - Remember_Node_Prepare_Distance)
        {
            Set_Node_Run_Mode(Run_Track.Node_Arr_Dir[Execute_Times]);
        }
    }
}

/*************************************
** Function: Record_Turn_Mileage
** Description: 记录两次转弯之间的里程间隔到 Turn_Mileage_Record 数组
** Details:   建图模式下在每次转弯完成时调用。
**            turn_interval_mileage = Turn_Begin_Mileage - Last_Turn_Mileage_Base
**            即：本次转弯起始里程 - 上次转弯记录基准里程 = 两次转弯间的行驶距离
**            记录后立即写入Flash。
**            回放模式下直接return（不需要记录，应该从Flash读取）。
*************************************/
static void Record_Turn_Mileage(void)
{
    float turn_interval_mileage;  // 两次转弯间的里程间隔

    if (Mode == Remember_Mode)    // 回放模式不记录（是从Flash读取已有记录的）
    {
        return;
    }

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
** Function: Car_Go
** Description: 小车主运行函数（由3ms定时器中断调用）
** Call Order:  Get_Light → 读编码器(3ms) → (每2次)Get_Speed(6ms) → Get_IMU → 模式Get_Error → Set_Out
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

    Get_Light();  // 读取15路光敏传感器ADC值（每周3ms）

    // 每2次调用执行一次速度计算（6ms周期，编码器读取+FIR滤波+里程累加）
    if (Speed_Get_Count == 1)
    {
        Get_Speed();
    }
    Speed_Get_Count *= -1;  // 交替切换：1→-1→1→-1...

    Get_IMU();  // 读取陀螺仪角速度 + 积分角度（每周3ms）

    // 根据模式选择对应的寻迹逻辑
    if (Mode == Build_Mode)
    {
        Build_Mode_Get_Error();     // 建图模式：传感器寻迹 + 记录里程数据
    }

    if (Mode == Remember_Mode)
    {
        Remember_Mode_Get_Error();  // 回放模式：传感器寻迹 + Flash里程回放
    }

    Set_Speed();  // 计算PID输出速度
    
    Set_Out();  // PWM输出控制电机（每周3ms）
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
        // Remember模式转弯时走内弧线，编码器里程偏短不累加，转弯完成后统一补偿
        if (!(Mode == Remember_Mode && (is_left == 1 || is_right == 1)))
        {
            Total_Run_Mileage += instant_speed;
        }
    }
}

/*************************************
** Function: Get_IMU
** Description: 获取IMU陀螺仪数据并处理（每3ms调用一次）
** Details:
*************************************/
void Get_IMU()
{
    icm20602_get_gyro();  // 读取IMU20602陀螺仪原始数据

    float gyro_raw = icm20602_gyro_transition(icm20602_gyro_z);

    if (is_left == 1 || is_right == 1)
    {
        Gyro_Integral += gyro_raw * GYRO_INTEGRATION_PERIOD_S;
    }
    else
    {
        Gyro_Integral = 0;  // 非转弯状态清零积分角度
    }

    if (fabs(gyro_raw) < 2.0f)
    {
        Gyro_Z = 0;
        icm20602_gyro_z = 0;
    }
    else
    {
        Gyro_Z = gyro_raw;
    }

    // PID用Gyro_Z：与原工程一致，raw/1000，死区<30 raw
    {
        float raw_z = (float)icm20602_gyro_z;
        if (fabs(raw_z) < 30.0f)
        {
            Gyro_Z_For_PID = 0;
        }
        else
        {
            Gyro_Z_For_PID = raw_z / 1000.0f;
        }
    }
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
    if ((Light_Convert[0] == 1 || Light_Convert[1] == 1) ||     // 左边缘
        (Light_Convert[13] == 1 || Light_Convert[14] == 1) ||   // 右边缘
        Initial_White_Num >= 5)                                  // 白线数量≥5（十字路口/宽线）
    {
        Check_Edge_Count++;
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
    if (EnableSwitch_ON)
    {
        if ((Track_Num == TRACK_SENSOR_ACTIVE_NUM || Track_Num == 0) && is_left == 0 && is_right == 0)
        {
            Count.Stop++;  // 停车计数累加（连续80次 → Set_Out中触发Stop_Flag）
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

    if (Finish_Flag == 1)
    {
        Finish_Count++;
    }

    if (Finish_Count > 200 && Stop_Flag == 0)  // 首次触发：停车 + 保存里程数据到Flash
    {
        Stop_Flag = 1;
        if (Mode == Build_Mode)
        {
            Save_Turn_Mileage_Record_To_Flash();         // 转弯间距里程存入Flash
            Save_Segment_Edge_Mileage_Record_To_Flash(); // 路段边缘里程存入Flash
        }
    }
}

/*************************************
** Function: Build_Mode_Get_Error
** Description: 建图模式：计算寻迹偏差 + 状态机调度
** Details:
*************************************/
void Build_Mode_Get_Error()
{
    Light_Process();  // 先处理传感器数据（更新 Light_Convert / Track_Arr）

    if (Check_Edge_Skip_Count > 0)  // 边缘检测跳过计数递减
    {
        Check_Edge_Skip_Count--;
    }

    // 建图模式首次初始化
    if (First_Mode == 0)
    {
        Run_Mode = Normal_Mode;                                       // 初始为正常寻迹模式
        First_Mode = 1;                                               // 标记已初始化
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
** Function: Remember_Mode_Get_Error
** Description: 回放模式：计算寻迹偏差 + 状态机调度
** Details:
*************************************/
void Remember_Mode_Get_Error()
{
    Light_Process();  // 先处理传感器数据

    if (Check_Edge_Skip_Count > 0)  // 边缘检测跳过计数递减
    {
        Check_Edge_Skip_Count--;
    }

    // 回放模式首次初始化：从Flash加载建图数据
    if (Remember_First_Mode == 0)
    {
        Remember_First_Mode = 1;                            // 标记已初始化
        // 加载Flash页5-7 → Turn_Mileage_Record_Num(有效条数) + Turn_Mileage_Record[](各段转弯间距里程)
        Load_Turn_Mileage_Record_From_Flash();
        // 加载Flash页8-9 → Segment_Edge_Mileage_Record[节点][元素](建图时每个元素边缘触发的里程坐标)
        Load_Segment_Edge_Mileage_Record_From_Flash();
        Remember_Reset_Runtime_State();                      // 重置所有运行时变量
    }

    // Normal_Mode下额外执行回放专用的寻迹和触发检测
    if (Run_Mode == Normal_Mode)
    {
        Remember_Normal_Run();     // 回放专用寻迹（不调用Check_Edge）
        Remember_Check_Trigger();  // 回放专用触发检测（边缘+里程双触发）
    }

    //===== 运行模式状态机（与建图模式共用处理函数）=====
    switch (Run_Mode)
    {
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
        gpio_set_level(P33_4, 0);  // 蜂鸣器关（正常模式）
        Last_Error = Error;        // 保存当前偏差（用于传感器丢失时保持方向）
    }

    if (Track_Num < 2)  // 传感器<2个：基本丢失赛道
    {
        Error = 0;  // 丢线时直行，避免锁死转弯方向
    }
    else if (Track_Num < 4 && Track_Num >= 2)  // 传感器2~4个：少量白线
    {
        Left_Scan_Point = Track_Arr[0];                       // 当前帧最左传感器
        Right_Scan_Point = Track_Arr[Track_Num - 1];          // 当前帧最右传感器
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2; // 首尾平均偏差
    }
    if (Initial_White_Num >= 4)  // 真实白点≥4：正常寻迹（不依赖断点修正后的Track_Num）
    {
        // 使用已记录的边界点计算偏差（可能来自上一周期）
        Error = 0;

        // 检测边缘/路口 → 触发路段切换
        if (Check_Edge())
        {
            uint8_t mileage_num = Run_Track.Node_Arr_Mileage_Num[Execute_Times];

            if (In_Line_Ele_Count < mileage_num)
            {
                // 当前线路内还有元素
                uint8_t mileage_dir = Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count];

                if (mileage_dir != 0)  // 元素方向非零（1左转/2右转/3短直/4长直）
                {
                    Record_Segment_Edge_Mileage();         // 记录当前边缘的里程
                    Run_Mode = Mileage_Mode;               // 进入里程计模式
                    Remember_Edge_Snap_Latched = 0;        // 重置边缘锁定标志
                    Mileage_Element_Base = Count.Mileage;  // 记录里程基准
                }
                else  // 元素方向=0（普通路段，无元器件）
                {
                    // 记录边缘里程 + 推进元素计数，不触发节点
                    Record_Segment_Edge_Mileage();
                    In_Line_Ele_Count++;

                    if (In_Line_Ele_Count >= mileage_num)
                    {
                        Segment_Total_Mileage[Execute_Times] = Count.Mileage;
                        Set_Node_Run_Mode(Run_Track.Node_Arr_Dir[Execute_Times]);
                    }
                }
            }
            else
            {
                Segment_Total_Mileage[Execute_Times] = Count.Mileage;
                Set_Node_Run_Mode(Run_Track.Node_Arr_Dir[Execute_Times]);
            }
        }
    }
    else  // 断点修复后Track_Num≥5但真实白点<5，沿用边界点只算Error
    {
        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
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
    if (Turn_Action_Done == 0)
    {
        gpio_set_level(P33_4, 1);

        if (Mode == Build_Mode)
        {
            if (Turn_Decel_Phase == 0)          // 阶段0：直行
            {
                Error = 0;
                if (Count.Mileage >= TURN_STRAIGHT_PRE_DISTANCE)
                    Turn_Decel_Phase = 1;
                return;
            }
            else if (Turn_Decel_Phase == 1)     // 阶段1：减速停车
            {
                Error = -Turn_Error_Value;
                if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                    Turn_Decel_Phase = 2;
                return;
            }
            else                                // 阶段2：原地旋转
            {
                Error = -Turn_Error_Value;
            }
        }
        else
        {
            Error = -Remember_Turn_Error;   // 记忆模式用键显可调的固定Error
        }
        if (fabsf(Gyro_Integral) >= (Mode == Build_Mode ? BUILD_TURN_TARGET_ANGLE_DEG : REMEMBER_TURN_TARGET_ANGLE_DEG))
            Complete_Turn_Action();
    }
}

/*************************************
** Function: Turn_Right_Run
** Description: 右转状态机（每3ms调用一次）
** Details:
*************************************/
void Turn_Right_Run(void)
{
    TCA9555_All_LED_On();
    if (Turn_Action_Done == 0)
    {
        gpio_set_level(P33_4, 1);

        if (Mode == Build_Mode)
        {
            if (Turn_Decel_Phase == 0)          // 阶段0：直行
            {
                Error = 0;
                if (Count.Mileage >= TURN_STRAIGHT_PRE_DISTANCE)
                    Turn_Decel_Phase = 1;
                return;
            }
            else if (Turn_Decel_Phase == 1)     // 阶段1：减速停车
            {
                Error = Turn_Error_Value;
                if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                    Turn_Decel_Phase = 2;
                return;
            }
            else                                // 阶段2：原地旋转
            {
                Error = Turn_Error_Value;
            }
        }
        else
        {
            Error = Remember_Turn_Error;     // 记忆模式用键显可调的固定Error
        }
        if (fabsf(Gyro_Integral) >= (Mode == Build_Mode ? BUILD_TURN_TARGET_ANGLE_DEG : REMEMBER_TURN_TARGET_ANGLE_DEG))
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
            Check_Edge_Skip_Count = (Mode == Build_Mode) ? BUILD_CHECK_EDGE_MILEAGE_TURN : REMEMBER_CHECK_EDGE_MILEAGE_TURN;
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
void Mileage_Run_Stage_2()
{
    float section_mileage = Count.Mileage - Mileage_Element_Base; // 进入里程模式后已走距离

    switch (Run_Track.Node_Arr_Mileage_Dir[Execute_Times][In_Line_Ele_Count])
    {
        case 1:  // 左转元件
            if (Mileage_Turn_Done == 0)  // 转向未完成
            {
                if (Mode == Build_Mode)
                {
                    //===== Build模式：直行→减速停车→原地旋转（与节点转弯一致）=====
                    if (Turn_Decel_Phase == 0)          // 阶段0：直走延迟
                    {
                        Error = 0;
                        if (section_mileage >= MILEAGE_ELEMENT_TURN_DELAY)
                        {
                            Turn_Decel_Phase = 1;
                            is_left = 1;
                            Turn_Begin_Mileage = Total_Run_Mileage;
                            Count.Spd_Mileage = 0;
                            Advance_Turn_Section_Index();
                        }
                    }
                    else if (Turn_Decel_Phase == 1)     // 阶段1：减速停车
                    {
                        Error = -Turn_Error_Value;
                        if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                        {
                            Turn_Decel_Phase = 2;
                            Gyro_Integral = 0;
                        }
                    }
                    else                                // 阶段2：原地旋转
                    {
                        Error = -Turn_Error_Value;
                        if (fabsf(Gyro_Integral) >= BUILD_TURN_TARGET_ANGLE_DEG)
                        {
                            Error = 0;
                            is_left = 0;
                            Mileage_Turn_Done = 1;
                            Turn_Decel_Phase = 0;
                            Gyro_Integral = 0;
                        }
                    }
                }
                else  // Remember模式：两阶段差速转弯（不减速停车）
                {
                    if (is_left == 0 && is_right == 0)  // 阶段①：直走延迟
                    {
                        Error = 0;
                        if (section_mileage >= MILEAGE_ELEMENT_TURN_DELAY)
                        {
                            Turn_Begin_Mileage = Total_Run_Mileage;
                            Gyro_Integral = 0;
                            is_left = 1;
                            Count.Spd_Mileage = 0;
                            Advance_Turn_Section_Index();
                        }
                    }
                    else  // 阶段②：陀螺仪左转90°
                    {
                        if (fabsf(Gyro_Integral) >= REMEMBER_TURN_TARGET_ANGLE_DEG)
                        {
                            Error = 0;
                            is_left = 0;
                            Mileage_Turn_Done = 1;
                            Gyro_Integral = 0;
                        }
                        else
                        {
                            Error = -Turn_Error_Value;
                        }
                    }
                }
            }
            else
            {
                Error = 0;  // 已完成，不输出偏差
            }
            break;
        case 2:  // 右转元件
            if (Mileage_Turn_Done == 0)
            {
                if (Mode == Build_Mode)
                {
                    //===== Build模式：直行→减速停车→原地旋转（与节点转弯一致）=====
                    if (Turn_Decel_Phase == 0)          // 阶段0：直走延迟
                    {
                        Error = 0;
                        if (section_mileage >= MILEAGE_ELEMENT_TURN_DELAY)
                        {
                            Turn_Decel_Phase = 1;
                            is_right = 1;
                            Turn_Begin_Mileage = Total_Run_Mileage;
                            Count.Spd_Mileage = 0;
                            Advance_Turn_Section_Index();
                        }
                    }
                    else if (Turn_Decel_Phase == 1)     // 阶段1：减速停车
                    {
                        Error = Turn_Error_Value;
                        if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
                        {
                            Turn_Decel_Phase = 2;
                            Gyro_Integral = 0;
                        }
                    }
                    else                                // 阶段2：原地旋转
                    {
                        Error = Turn_Error_Value;
                        if (fabsf(Gyro_Integral) >= BUILD_TURN_TARGET_ANGLE_DEG)
                        {
                            Error = 0;
                            is_right = 0;
                            Mileage_Turn_Done = 1;
                            Turn_Decel_Phase = 0;
                            Gyro_Integral = 0;
                        }
                    }
                }
                else  // Remember模式：两阶段差速转弯
                {
                    if (is_left == 0 && is_right == 0)  // 阶段①：直走延迟
                    {
                        Error = 0;
                        if (section_mileage >= MILEAGE_ELEMENT_TURN_DELAY)
                        {
                            Turn_Begin_Mileage = Total_Run_Mileage;
                            Gyro_Integral = 0;
                            is_right = 1;
                            Count.Spd_Mileage = 0;
                            Advance_Turn_Section_Index();
                        }
                    }
                    else  // 阶段②：陀螺仪右转90°
                    {
                        if (fabsf(Gyro_Integral) >= REMEMBER_TURN_TARGET_ANGLE_DEG)
                        {
                            Error = 0;
                            is_right = 0;
                            Mileage_Turn_Done = 1;
                            Gyro_Integral = 0;
                        }
                        else
                        {
                            Error = Turn_Error_Value;
                        }
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
    Left_PID_Out = 0;   // 每周期重置PID输出
    Right_PID_Out = 0;

    //===== 停车检查 =====
    if (Stop_Flag != 0)  // 停车标志置位
    {
        Left_Exp_Spd = 0;   // 期望速度清零
        Right_Exp_Spd = 0;
        return;             // 跳过速度计算
    }

    //===== 模式速度选择 =====
    if (Mode == Build_Mode)
    {
        Run_Speed = Basic_Speed;  // 建图模式：固定速度（由键显设置）
    }
    else if (Mode == Remember_Mode)
    {
        Run_Speed = Remember_Get_Run_Speed();  // 回放模式：梯形速度曲线
    }

    //===== 级联PID：Error → Turn_PID → Gyro_PID =====
    Turn_PID_Out  = PID_calc(&Turn_PID, 0, (float)(Error / 100.0));

    Gyro_PID_Out = PID_calc(&Gyro_PID, 0, (-Turn_PID_Out)
        + ((Mode == Remember_Mode && (is_left == 1 || is_right == 1)) ? 0.0f : Gyro_Z_For_PID));

    //===== 更新平均速度（用于里程计算）=====
    if (EnableSwitch_ON)
    {
        Average_Speed = (Left_Real_Spd + Right_Real_Spd) / 2.0;
        Count.Spd_Mileage += Average_Speed;
    }

    // 左右期望速度 = 基础速度 ± 陀螺仪PID差速量
    Left_Exp_Spd = Run_Speed - Gyro_PID_Out;
    Right_Exp_Spd = Run_Speed + Gyro_PID_Out;

    //===== 转弯速度覆盖：使用转向专用基础速度 =====
    if (is_left == 1 || is_right == 1)
    {
        // Build模式：直行→减速停车→原地旋转
        if (Mode == Build_Mode && Turn_Decel_Phase == 0)
        {
            // 阶段0：直行，不覆盖速度（使用上方正常速度公式）
        }
        else if (Mode == Build_Mode && Turn_Decel_Phase == 1)
        {
            // 阶段1：减速停车
            Left_Exp_Spd = 0;
            Right_Exp_Spd = 0;
        }
        else if (Mode == Build_Mode && Turn_Decel_Phase == 2)
        {
            // 阶段2：原地旋转
            if (is_left == 1)
            {
                Left_Exp_Spd = -Turn_Error_Value;
                Right_Exp_Spd = Turn_Error_Value;
            }
            else
            {
                Left_Exp_Spd = Turn_Error_Value;
                Right_Exp_Spd = -Turn_Error_Value;
            }
        }
        else
        {
            // Remember模式：PID差速边走边转（Gyro_PID输入不含陀螺仪阻尼项）
            int Turn_Base_Spd = Get_Turn_Base_Speed();
            if (is_left == 1)
            {
                Left_Exp_Spd = Turn_Base_Spd - (int)(Gyro_PID_Out * REMEMBER_TURN_INNER_SCALE);
                Right_Exp_Spd = Turn_Base_Spd + (int)(Gyro_PID_Out * REMEMBER_TURN_OUTER_SCALE);
            }
            else
            {
                Left_Exp_Spd = Turn_Base_Spd - (int)(Gyro_PID_Out * REMEMBER_TURN_OUTER_SCALE);
                Right_Exp_Spd = Turn_Base_Spd + (int)(Gyro_PID_Out * REMEMBER_TURN_INNER_SCALE);
            }
        }
    }

    // 直行元器件：强制基础速度直走，不走差速
    if (Force_Straight_Speed)
    {
        Left_Exp_Spd = Basic_Speed;
        Right_Exp_Spd = Basic_Speed;
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
    //===== 低压保护：锁存后关闭所有电机，蜂鸣器间歇报警 =====
    if (Voltage_Check[0] < LOW_VOLTAGE_PROTECT_VALUE)
    {
        Low_Voltage_Protect_Flag = 1;
        Stop_Flag = 1;
    }

    if (Low_Voltage_Protect_Flag != 0)
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

        Low_Voltage_Beep_Count++;
        if (Low_Voltage_Beep_Count >= LOW_VOLTAGE_BEEP_PERIOD_COUNT)
        {
            Low_Voltage_Beep_Count = 0;
        }

        gpio_set_level(P33_4, (Low_Voltage_Beep_Count < LOW_VOLTAGE_BEEP_ON_COUNT) ? 1 : 0);
        return;
    }

    //===== 启动延时：使能后等待电机和编码器稳定 =====
    if (Enable_Start_Delay_Count > 0)
    {
        Enable_Start_Delay_Count--;  // 递减延时计数

        // 延时期间所有电机锁定
        pwm_set_duty(Suction_Motor_IN1, 0);
        pwm_set_duty(Suction_Motor_IN2, 8000);  // 吸风电机以80%占空比运转
        pwm_set_duty(Left_Motor_IN1, 0);
        pwm_set_duty(Left_Motor_IN2, 0);
        pwm_set_duty(Right_Motor_IN1, 0);
        pwm_set_duty(Right_Motor_IN2, 0);

        PID_cleardata(&Left_PID);   // 清零左电机PID历史数据
        PID_cleardata(&Right_PID);  // 清零右电机PID历史数据
        return;                     // 跳过正常输出
    }

//   if (Count.Stop > 100 && is_left == 0 && is_right == 0)
//   {
//       Stop_Flag = 1;  // 置位停车标志（下周期 Set_Speed 将设期望速度=0）
//   }

   //===== 堵转检测：使能+延时过后，连续20帧左右轮速度绝对值≤5 =====
//   if (EnableSwitch_ON && Enable_Start_Delay_Count == 0 && Stop_Flag == 0
//       && is_left == 0 && is_right == 0)  // 转弯时不检测堵转（减速/原地旋转速度天然为0）
//   {
//       if (abs(Left_Real_Spd) <= 5 && abs(Right_Real_Spd) <= 5)
//       {
//           Count.Stall++;
//           if (Count.Stall > 80)
//           {
//               Stop_Flag = 1;
//           }
//       }
//       else
//       {
//           Count.Stall = 0;
//       }
//   }

    if (EnableSwitch_ON && Stop_Flag == 0 && (is_left == 1 || is_right == 1))
    {
        pwm_set_duty(Suction_Motor_IN1, 0);
        pwm_set_duty(Suction_Motor_IN2, 9500);
    }
    else
    {
        pwm_set_duty(Suction_Motor_IN1, 0);
        pwm_set_duty(Suction_Motor_IN2, 9500);
    }

    //===== 驱动电机输出 =====
    if (EnableSwitch_ON && Stop_Flag == 0)  // 使能 + 非停车状态
    {
        //--- 左电机 ---
        if (Left_PID_Out == 0 || Stop_Flag == 1)
        {
            // PID输出=0：H桥两路同时PWM（短路制动）
            pwm_set_duty(Left_Motor_IN1, 0);
            pwm_set_duty(Left_Motor_IN2, 0);
        }
        else if (Left_PID_Out > 0)
        {
            // 正向驱动：IN1低, IN2按比例PWM
            pwm_set_duty(Left_Motor_IN1, 0);
            pwm_set_duty(Left_Motor_IN2, (Left_PID_Out + 10000) / 2);
        }
        else  // Left_PID_Out < 0
        {
            // 反向驱动：IN1按比例PWM, IN2低
            pwm_set_duty(Left_Motor_IN1, (fabs(Left_PID_Out) + 10000) / 2);
            pwm_set_duty(Left_Motor_IN2, 0);
        }

        //--- 右电机 ---
        if (Right_PID_Out == 0 || Stop_Flag == 1)
        {
            // PID输出=0：H桥短路制动
            pwm_set_duty(Right_Motor_IN2, 0);
            pwm_set_duty(Right_Motor_IN1, 0);
        }
        else if (Right_PID_Out > 0)
        {
            // 正向驱动
            pwm_set_duty(Right_Motor_IN2, 0);
            pwm_set_duty(Right_Motor_IN1, (Right_PID_Out + 10000) / 2);
        }
        else  // Right_PID_Out < 0
        {
            // 反向驱动
            pwm_set_duty(Right_Motor_IN2, (fabs(Right_PID_Out) + 10000) / 2);
            pwm_set_duty(Right_Motor_IN1, 0);
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
            Advance_To_Next_Track_Segment(); // 推进到下一路段

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
