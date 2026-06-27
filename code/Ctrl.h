/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.h
Author: Cross_Z
Version:0.0        Date: 2026.1.30
Description: 鏅鸿兘杞︽牳蹇冩帶鍒跺ご鏂囦欢
             鍖呭惈PID鍙傛暟銆佽繍琛屾ā寮忋€佽禌閬撶粨鏋勪綋銆佸叏灞€鍙橀噺銆佸閮ㄥ嚱鏁板０鏄?
Others:  鍩轰簬涓櫙鏅鸿搴曞眰搴撳紑鍙?
Function List: 杩愬姩鎺у埗銆侀€熷害闂幆銆佸惊杩广€侀噷绋嬨€佽浆鍚戙€佸缓鍥?鍥炴斁妯″紡
History:
<author>    <time>       <version>    <desc>
Cross_Z     2026.1.30      0.0        鍒涘缓鍒濆鐗堟湰
**************************************************/

#ifndef __CTRL_H
#define __CTRL_H

// 搴曞眰閫氱敤澶存枃浠?+ 鑷畾涔夋€诲ご鏂囦欢
#include "zf_common_headfile.h"
#include "headfiles.h"

/**********************************************
* 瀹忓畾涔?
**********************************************/
#define NODE_NUM_MAX            20
#define ELEMENT_NUM_MAX         5
#define TRACK_SEGMENT_NUM_MAX   (NODE_NUM_MAX + 1)
#define BUILD_ACTION_MAX        (NODE_NUM_MAX + (TRACK_SEGMENT_NUM_MAX * ELEMENT_NUM_MAX))
#define TURN_MILEAGE_RECORD_MAX 120

#define SAFETY_LOW_VOLTAGE_THRESHOLD    10.9f

#define GYRO_PID { \
    .kp         = 0.008, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_ADD \
}

#define ANGLE_PID { \
    .kp         = 80, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_POSITION_D_ON_MEASUREMENT \
}

#define GYRO_PD_PID { \
    .kp         = 0.008, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_POSITION \
}

#define LEFT_PID { \
    .kp         = 0, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 9500, \
    .mode       = PID_MODE_ADD \
}

#define RIGHT_PID { \
    .kp         = 250, \
    .ki         = 65, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 6000, \
    .mode       = PID_MODE_ADD \
}

#define TURN_PID { \
    .kp         = 80, \
    .ki         = 0, \
    .kd         = 0, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_POSITION \
}

/**********************************************
* 鏋氫妇绫诲瀷瀹氫箟
**********************************************/

/**
 * @brief 灏忚溅杩愯妯″紡
 */
typedef enum
{
    Normal_Mode,     // 甯歌寰抗
    Turn_Left,       // 宸﹁浆妯″紡
    Turn_Right,      // 鍙宠浆妯″紡
    Mileage_Mode,    // 閲岀▼鎺у埗妯″紡
    Straight_Mode,   // 鐩磋妯″紡
} Run_Mode_Enum;

/**
 * @brief 閲岀▼璁¤繍琛岄樁娈?
 */
typedef enum
{
    Normal_Stage,    // 甯歌闃舵
    Straight_Stage,  // 鐩磋闃舵
} Mileage_Stage_Enum;

/**
 * @brief 閿洏鏄剧ず宸ヤ綔妯″紡
 */
typedef enum
{
    Build_Mode,      // 寤哄浘妯″紡
    Debug_Mode,      // 璋冭瘯妯″紡
} Mode_Define;

typedef enum
{
    BUILD_ACTION_NONE           = 0,
    BUILD_ACTION_TURN_LEFT      = 1,
    BUILD_ACTION_TURN_RIGHT     = 2,
    BUILD_ACTION_STRAIGHT_SHORT = 3,
    BUILD_ACTION_STRAIGHT_LONG  = 4,
    BUILD_ACTION_ELEM_LEFT      = 5,
    BUILD_ACTION_ELEM_RIGHT     = 6,
} Build_Action_Enum;

typedef enum
{
    BUILD_ACTION_SOURCE_NODE = 0,
    BUILD_ACTION_SOURCE_ELEMENT = 1,
} Build_Action_Source_Enum;

/**
 * @brief 璋冭瘯瀛愭ā寮?
 */
typedef enum
{
    Debug_Sub_PI_Tuning,     // 杞瓙PI璋冨弬锛堝彲閫氳繃Debug_Which_Wheel鍒囨崲宸﹀彸锛?
    Debug_Sub_Ground_Test,   // 涓嬪湴娴嬭瘯璋冨弬锛堥鐣欙級
    Debug_Sub_Angle,         // 瑙掑害鐜皟鍙傦紙棰勭暀锛?
    Debug_Sub_NormalTrace,   // 鏅€氬惊杩癸紙棰勭暀锛?
} Debug_Sub_Mode_Enum;

/**********************************************
* 缁撴瀯浣撳畾涔?
**********************************************/

/**
 * @brief 璁℃暟缁撴瀯浣擄紙缂栫爜鍣ㄣ€侀噷绋嬨€佺姸鎬佽鏁帮級
 */
typedef struct
{
    int     Left;        // 宸﹁浆鍑哄集璁℃暟
    int     Right;       // 鍙宠浆鍑哄集璁℃暟
    int     Stop;        // 鍋滆溅璁℃暟
    float   Mileage;     // 褰撳墠娈甸噷绋?
    float   Spd_Mileage; // 閫熷害閲岀▼
    int     Straight;    // 鐩磋璁℃暟
    int     Stall;       // 鍫佃浆璁℃暟
} Count_Typedef;

/**
 * @brief 璧涢亾淇℃伅缁撴瀯浣擄紙瀛樺偍鑺傜偣銆佸厓绱犮€佹柟鍚戙€侀噷绋嬶級
 */
typedef struct
{
    uint8_t Node_Arr_Dir[NODE_NUM_MAX];                  // 鍚勮妭鐐硅椹舵柟鍚?
    uint8_t Node_Arr_Mileage_Num[TRACK_SEGMENT_NUM_MAX];         // 鍚勬閲岀▼娈垫暟閲?    uint8_t Node_Arr_Mileage_Dir[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];  // 鍚勯噷绋嬫琛岄┒鏂瑰悜
    int     Node_Arr_Mileage_Normal[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];   // 鏅€氳矾娈甸噷绋嬪€?    int     Node_Arr_Mileage_Element[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX]; // 鍏冪礌璺閲岀▼鍊?    uint8_t Node_Num;       // 鏈夋晥鑺傜偣鎬绘暟
    uint8_t Element_Num;    // 鏈夋晥鍏冪礌鎬绘暟
    uint8_t Stop_Mode;      // 鍋滆溅妯″紡
} Racing_track_Typedef;

typedef struct
{
    Build_Action_Enum        action;
    Build_Action_Source_Enum source;
    uint8_t                  segment_index;
    uint8_t                  element_index;
} Build_Action_Typedef;

/**********************************************
* 澶栭儴鍏ㄥ眬鍙橀噺澹版槑
**********************************************/
// 鍩虹閫熷害
extern int Basic_Speed;

// 鏈熸湜閫熷害
extern int Left_Exp_Spd;
extern int Right_Exp_Spd;
extern int Middle;
// 瀹為檯閫熷害
extern int Left_Real_Spd;
extern int Right_Real_Spd;

// 骞冲潎閫熷害
extern float Average_Speed;

// 寰抗璇樊
extern int Error;

// 鍏夋晱浼犳劅鍣?
extern uint16 Light_ADC[15];
extern uint8  Light_Convert[15];

// PID 杈撳嚭
extern float Turn_PID_Out;
extern float Gyro_PID_Out;
extern float Left_PID_Out;
extern float Right_PID_Out;

// 鐘舵€佹爣蹇?
extern int  Stop_Flag;          // 鍋滆溅鏍囧織
extern int  Finish_Flag;         // 浠诲姟瀹屾垚鏍囧織
extern int  Finish_Count;        // 瀹屾垚璁℃暟
extern int  Track_Num;           // 鏈夋晥寰抗浼犳劅鍣ㄦ暟

// 鎵ц璁℃暟
extern int8_t  Execute_Times;    // 褰撳墠鎵ц鑺傜偣绱㈠紩
extern int8_t  Mileage_Times;     // 褰撳墠鑺傜偣閲岀▼娈垫€绘暟

// 鍏冪礌/璺嚎璁℃暟
extern uint8_t Line_Num_Count;    // 璺嚎鏁伴噺
extern uint8_t In_Line_Ele_Count; // 褰撳墠绾胯矾鍏冪礌绱㈠紩
extern uint8_t Build_Action_Index;
extern uint8_t Build_Action_Count;

// 闄€铻轰华鐩稿叧
extern float Gyro_Integral;
// 杞集鐩爣瑙掑害锛堣搴﹂棴鐜疨ID浣跨敤锛氬乏杞?90, 鍙宠浆-90锛?
extern float  Turn_Angle_Target;

// 寤哄浘鎺у埗鍙傛暟
extern int16  Check_Edge_Count;    // Check_Edge瑙﹀彂娆℃暟
extern float  Mileage_Element_Turn_Delay;
extern float  Mileage_Node_Turn_Delay;
extern uint8  vofa_flash_dump_mode;     // VOFA Flash鏁版嵁瀵煎嚭妯″紡鏍囧織
extern float  Total_Run_Mileage;       // 鎬昏繍琛岄噷绋嬶紙鍥炴斁妯″紡閲岀▼瀵规瘮鍩哄噯锛?
// 缁撴瀯浣撳疄渚?
extern Count_Typedef Count;
extern Mode_Define Mode;
extern float Gyro_Z_For_PID; // PID鐢ㄩ檧铻轰华Z杞存暟鎹?
// 褰撳墠杩愯鐘舵€?
extern Run_Mode_Enum       Run_Mode;
extern Mileage_Stage_Enum  Mileage_Stage;
extern Racing_track_Typedef Run_Track;
extern Build_Action_Typedef Build_Action_List[BUILD_ACTION_MAX];

// Flash閲岀▼鏁版嵁锛圴OFA瀵煎嚭鐢級
extern float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];
extern float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];         // 鍚勬璺疄娴嬫€婚噷绋?extern float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX];
extern uint16_t Turn_Mileage_Record_Num;
// 棰勮禌璧涢亾
extern Racing_track_Typedef Pre_Contest_1;
extern Racing_track_Typedef Pre_Contest_2;
extern Racing_track_Typedef Pre_Contest_3;

// 鍐宠禌璧涢亾
extern Racing_track_Typedef Final_Contest_1;
extern Racing_track_Typedef Final_Contest_2;
extern Racing_track_Typedef Final_Contest_3;

// PID 鎺у埗鍣?
extern PID_HandleTypeDef Gyro_PID;
extern PID_HandleTypeDef Left_PID;
extern PID_HandleTypeDef Right_PID;
extern PID_HandleTypeDef Turn_PID;
extern PID_HandleTypeDef Angle_PID;
extern PID_HandleTypeDef Gyro_PD_PID;

// 璋冭瘯妯″紡
extern Debug_Sub_Mode_Enum Debug_Sub_Mode;  // 褰撳墠璋冭瘯瀛愭ā寮?
extern uint8  Debug_Motor_Enable;           // 璋冭瘯妯″紡鐢垫満浣胯兘鏍囧織锛?=鍋? 1=杞?
extern uint8  Debug_Which_Wheel;            // 褰撳墠璋冭瘯鐨勮疆瀛愶細0=宸﹁疆, 1=鍙宠疆
extern int    Debug_Target_Speed;           // 璋冭瘯鐩爣閫熷害
extern int    Debug_Fan_Duty;               // 涓嬪湴娴嬭瘯璐熷帇椋庢墖鍗犵┖姣?
extern uint8  Debug_Ground_Dir;             // 1=宸︽鍙冲弽, 2=宸﹀弽鍙虫
extern uint8  Debug_Angle_Mode;             // 1=sin target, 2=step target
extern uint8  Debug_Angle_D_First;          // 0=error D, 1=measurement D
extern float  Debug_Angle_Vel_Target;       // VOFA: 瑙掗€熷害鐩爣
extern float  Debug_Angle_Vel_Real;         // VOFA: 瀹為檯瑙掗€熷害
extern float  Debug_Kp_Left;               // 宸﹁疆璋冭瘯Kp
extern float  Debug_Ki_Left;               // 宸﹁疆璋冭瘯Ki
extern float  Debug_Kp_Right;              // 鍙宠疆璋冭瘯Kp
extern float  Debug_Ki_Right;              // 鍙宠疆璋冭瘯Ki

/**********************************************
* 澶栭儴鍑芥暟澹版槑
**********************************************/
void Car_Go(void);                          // 灏忚溅涓昏繍琛屽嚱鏁?
void Light_Process(void);
void Set_Speed(void);                       // 璁剧疆鐩爣閫熷害
void Get_Speed(void);                       // 鑾峰彇瀹為檯閫熷害
uint8 Check_Edge(void);                      // 杈圭紭妫€娴?
void Get_IMU(void);                         // 鑾峰彇闄€铻轰华鏁版嵁
void Set_Out(void);                         // 鐢垫満杈撳嚭鎺у埗

void Normal_Run(void);                      // 甯歌寰抗杩愯
void Straight_Run(void);                    // 鐩磋杩愯
void Turn_Left_Run(void);                   // 宸﹁浆鎺у埗
void Turn_Right_Run(void);                  // 鍙宠浆鎺у埗

void Mileage_Mode_Run(void);                // 閲岀▼妯″紡鎬绘帶
void Mileage_Run_Stage_2(void);             // 閲岀▼闃舵2

void Safety_Check(void);                    // 缁熶竴瀹夊叏妫€娴嬶紙浣庡帇+鍫佃浆锛屾瘡娆′腑鏂渶鍏堣皟鐢級
void Build_Mode_Get_Error(void);            // 寤哄浘妯″紡鑾峰彇璇樊
void Load_All_Flash_Data_For_VOFA(void);    // VOFA瀵煎嚭锛氬姞杞紽lash鍏ㄩ儴鍦板浘鏁版嵁

// 璋冭瘯妯″紡
void Debug_Wheel_Tuning(void);              // 杞瓙PI璋冨弬锛堝乏鍙宠疆鍏辩敤锛?
void Debug_Ground_Test(void);               // 涓嬪湴娴嬭瘯
void Debug_Angle_Tuning(void);              // 瑙掑害鐜皟鍙傦紙棰勭暀锛?
void Debug_Normal_Trace(void);              // 鏅€氬惊杩癸紙棰勭暀锛?
void Debug_Set_Out(void);                   // 璋冭瘯涓撶敤绮剧畝PWM杈撳嚭

void Set_Mileage_Turn_Exp_Speed(float angle_target);

#endif
