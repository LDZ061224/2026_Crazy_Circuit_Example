/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Ctrl.h
Author: Cross_Z
Version:0.0        Date: 2026.1.30
Description: 閺呴缚鍏樻潪锔界壋韫囧啯甯堕崚璺恒仈閺傚洣娆?
             閸栧懎鎯圥ID閸欏倹鏆熼妴浣界箥鐞涘本膩瀵繈鈧浇绂岄柆鎾剁波閺嬪嫪缍嬮妴浣稿弿鐏炩偓閸欐﹢鍣洪妴浣割樆闁劌鍤遍弫鏉匡紣閺?
Others:  閸╄桨绨稉顓熸珯閺呴缚顢戞惔鏇炵湴鎼存挸绱戦崣?
Function List: 鏉╂劕濮╅幒褍鍩楅妴渚€鈧喎瀹抽梻顓犲箚閵嗕礁鎯婃潻骞库偓渚€鍣风粙瀣ㄢ偓浣芥祮閸氭垯鈧礁缂撻崶?閸ョ偞鏂佸Ο鈥崇础
History:
<author>    <time>       <version>    <desc>
Cross_Z     2026.1.30      0.0        閸掓稑缂撻崚婵嗩潗閻楀牊婀?
**************************************************/

#ifndef __CTRL_H
#define __CTRL_H

// 鎼存洖鐪伴柅姘辨暏婢跺瓨鏋冩禒?+ 閼奉亜鐣炬稊澶嬧偓璇层仈閺傚洣娆?
#include "zf_common_headfile.h"
#include "headfiles.h"

/**********************************************
* 鐎瑰繐鐣炬稊?
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

#define SAFETY_LOW_VOLTAGE_THRESHOLD    10.9f

#define GYRO_PID { \
    .kp         = 0.16, \
    .ki         = 0.006, \
    .kd         = 0.075, \
    .iOutMax    = 0, \
    .outMax     = 50, \
    .mode       = PID_MODE_ADD \
}

#define ANGLE_PID { \
    .kp         = 19.5, \
    .ki         = 0, \
    .kd         = 9, \
    .iOutMax    = 0, \
    .outMax     = 900, \
    .mode       = PID_MODE_POSITION_D_ON_MEASUREMENT \
}

#define GYRO_PD_PID { \
    .kp         = 0.008, \
    .ki         = 0, \
    .kd         = 0.002, \
    .iOutMax    = 0, \
    .outMax     = 500, \
    .mode       = PID_MODE_POSITION \
}

#define LEFT_PID { \
    .kp         = 150, \
    .ki         = 1.2, \
    .kd         = 0, \
    .iOutMax    = 5000, \
    .outMax     = 9500, \
    .mode       = PID_MODE_ADD \
}

#define RIGHT_PID { \
    .kp         = 150, \
    .ki         = 2.1, \
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
    .outMax     = 1200, \
    .mode       = PID_MODE_POSITION \
}

/**********************************************
* 閺嬫矮濡囩猾璇茬€风€规矮绠?
**********************************************/

/**
 * @brief 鐏忓繗婧呮潻鎰攽濡€崇础
 */
typedef enum
{
    Normal_Mode,     // 鐢瓕顫夊顏囨姉
    Turn_Left,       // 瀹革箒娴嗗Ο鈥崇础
    Turn_Right,      // 閸欏疇娴嗗Ο鈥崇础
    Mileage_Mode,    // 闁插瞼鈻奸幒褍鍩楀Ο鈥崇础
    Straight_Mode,   // 閻╃顢戝Ο鈥崇础
} Run_Mode_Enum;

/**
 * @brief 闁插瞼鈻肩拋陇绻嶇悰宀勬▉濞?
 */
typedef enum
{
    Normal_Stage,    // 鐢瓕顫夐梼鑸殿唽
    Straight_Stage,  // 閻╃顢戦梼鑸殿唽
} Mileage_Stage_Enum;

/**
 * @brief 闁款喚娲忛弰鍓с仛瀹搞儰缍斿Ο鈥崇础
 */
typedef enum
{
    Build_Mode,      // 瀵ゅ搫娴樺Ο鈥崇础
    Debug_Mode,      // 鐠嬪啳鐦Ο鈥崇础
} Mode_Define;

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
 * @brief 鐠嬪啳鐦€涙劖膩瀵?
 */
typedef enum
{
    Debug_Sub_PI_Tuning,     // 鏉烆喖鐡橮I鐠嬪啫寮敍鍫濆讲闁俺绻僁ebug_Which_Wheel閸掑洦宕插锕€褰搁敍?
    Debug_Sub_Ground_Test,   // 娑撳婀村ù瀣槸鐠嬪啫寮敍鍫ヮ暕閻ｆ瑱绱?
    Debug_Sub_Angle,         // 鐟欐帒瀹抽悳顖濈殶閸欏偊绱欐０鍕殌閿?
    Debug_Sub_NormalTrace,   // 閺咁噣鈧艾鎯婃潻鐧哥礄妫板嫮鏆€閿?
} Debug_Sub_Mode_Enum;

/**********************************************
* 缂佹挻鐎担鎾崇暰娑?
**********************************************/

/**
 * @brief 鐠佲剝鏆熺紒鎾寸€担鎿勭礄缂傛牜鐖滈崳銊ｂ偓渚€鍣风粙瀣ㄢ偓浣哄Ц閹浇顓搁弫甯礆
 */
typedef struct
{
    int     Left;        // 瀹革箒娴嗛崙鍝勯泦鐠佲剝鏆?
    int     Right;       // 閸欏疇娴嗛崙鍝勯泦鐠佲剝鏆?
    int     Stop;        // 閸嬫粏婧呯拋鈩冩殶
    float   Mileage;     // 瑜版挸澧犲▓鐢稿櫡缁?
    float   Spd_Mileage; // 闁喎瀹抽柌宀€鈻?
    int     Straight;    // 閻╃顢戠拋鈩冩殶
    int     Stall;       // 閸絻娴嗙拋鈩冩殶
} Count_Typedef;

typedef struct
{
    Build_Action_Enum        action;
    uint8_t                  segment_index;
    uint8_t                  element_index;
} Build_Action_Typedef;

/**********************************************
* 婢舵牠鍎撮崗銊ョ湰閸欐﹢鍣烘竟鐗堟
**********************************************/
// 閸╄櫣顢呴柅鐔峰
extern int Basic_Speed;

// 閺堢喐婀滈柅鐔峰
extern int Left_Exp_Spd;
extern int Right_Exp_Spd;
extern int Middle;
// 鐎圭偤妾柅鐔峰
extern int Left_Real_Spd;
extern int Right_Real_Spd;

// 楠炲啿娼庨柅鐔峰
extern float Average_Speed;

// 瀵邦亣鎶楃拠顖氭▕
extern int Error;
extern int   Track_Arr[15];
extern int16_t Dir_Arr[15];
extern int   Left_Scan_Point;
extern int   Right_Scan_Point;
extern int   Last_Error;

// 閸忓鏅辨导鐘冲妳閸?
extern uint16 Light_ADC[15];
extern uint8  Light_Convert[15];

// PID 鏉堟挸鍤?
extern float Turn_PID_Out;
extern float Gyro_PID_Out;
extern float Left_PID_Out;
extern float Right_PID_Out;

// 閻樿埖鈧焦鐖ｈ箛?
extern int  Stop_Flag;          // 閸嬫粏婧呴弽鍥х箶
extern int  Finish_Flag;         // 娴犺濮熺€瑰本鍨氶弽鍥х箶
extern int  Finish_Count;        // 鐎瑰本鍨氱拋鈩冩殶
extern int  Track_Num;           // 閺堝鏅ュ顏囨姉娴肩姵鍔呴崳銊︽殶

// 閹笛嗩攽鐠佲剝鏆?
extern int8_t  Execute_Times;    // 瑜版挸澧犻幍褑顢戦懞鍌滃仯缁便垹绱?
extern int8_t  Mileage_Times;     // 瑜版挸澧犻懞鍌滃仯闁插瞼鈻煎▓鍨偓缁樻殶

// 閸忓啰绀?鐠侯垳鍤庣拋鈩冩殶
extern uint8_t Line_Num_Count;    // 鐠侯垳鍤庨弫浼村櫤
extern uint8_t In_Line_Ele_Count; // 瑜版挸澧犵痪鑳熅閸忓啰绀岀槐銏犵穿
extern uint8_t Build_Action_Index;
extern uint8_t Build_Action_Count;

// 闂勨偓閾昏桨鍗庨惄绋垮彠
extern float Gyro_Integral;
// 鏉烆剙闆嗛惄顔界垼鐟欐帒瀹抽敍鍫ｎ潡鎼达箓妫撮悳鐤↖D娴ｈ法鏁ら敍姘箯鏉?90, 閸欏疇娴?90閿?
extern float  Turn_Angle_Target;

// 瀵ゅ搫娴橀幒褍鍩楅崣鍌涙殶
extern int16  Check_Edge_Count;    // Check_Edge鐟欙箑褰傚▎鈩冩殶
extern float  Mileage_Element_Turn_Delay;
extern float  Mileage_Node_Turn_Delay;
extern uint8  Current_Element_Dir;      // Element direction of currently executing action (for Mileage_Mode_Run)
extern uint8  vofa_flash_dump_mode;     // VOFA Flash閺佺増宓佺€电厧鍤Ο鈥崇础閺嶅洤绻?
extern float  Total_Run_Mileage;       // 閹槒绻嶇悰宀勫櫡缁嬪绱欓崶鐐存杹濡€崇础闁插瞼鈻肩€佃鐦崺鍝勫櫙閿?
// 缂佹挻鐎担鎾崇杽娓?
extern Count_Typedef Count;
extern Mode_Define Mode;
extern float Gyro_Z_For_PID; // PID使用的陀螺仪Z轴角速度
extern float gyro_z_offset;  // 陀螺仪Z轴零漂（上电校准）
// 瑜版挸澧犳潻鎰攽閻樿埖鈧?
extern Run_Mode_Enum       Run_Mode;
extern Mileage_Stage_Enum  Mileage_Stage;
extern Build_Action_Typedef Build_Action_List[BUILD_ACTION_MAX];
extern const Build_Action_Typedef Default_Build_Actions[BUILD_ACTION_COUNT];
extern const uint8 Mileage_Num_By_Segment[BUILD_NODE_NUM + 1];

// Flash闁插瞼鈻奸弫鐗堝祦閿涘湸OFA鐎电厧鍤悽顭掔礆
extern float Segment_Edge_Mileage_Record[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX];
extern float Segment_Total_Mileage[TRACK_SEGMENT_NUM_MAX];
extern float Turn_Mileage_Record[TURN_MILEAGE_RECORD_MAX];
extern uint16_t Turn_Mileage_Record_Num;

// PID 閹貉冨煑閸?
extern PID_HandleTypeDef Gyro_PID;
extern PID_HandleTypeDef Left_PID;
extern PID_HandleTypeDef Right_PID;
extern PID_HandleTypeDef Turn_PID;
extern PID_HandleTypeDef Angle_PID;
extern PID_HandleTypeDef Gyro_PD_PID;

// 鐠嬪啳鐦Ο鈥崇础
extern Debug_Sub_Mode_Enum Debug_Sub_Mode;  // 瑜版挸澧犵拫鍐槸鐎涙劖膩瀵?
extern uint8  Debug_Motor_Enable;           // 鐠嬪啳鐦Ο鈥崇础閻㈠灚婧€娴ｈ儻鍏橀弽鍥х箶閿?=閸? 1=鏉?
extern uint8  Debug_Which_Wheel;            // 瑜版挸澧犵拫鍐槸閻ㄥ嫯鐤嗙€涙劧绱?=瀹革箒鐤? 1=閸欏疇鐤?
extern int    Debug_Target_Speed;           // 鐠嬪啳鐦惄顔界垼闁喎瀹?
extern int    Debug_Fan_Duty;               // 娑撳婀村ù瀣槸鐠愮喎甯囨搴㈠閸楃姷鈹栧В?
extern uint8  Debug_Ground_Dir;             // 1=瀹革附顒滈崣鍐插冀, 2=瀹革箑寮介崣铏劀
extern uint8  Debug_Angle_Mode;             // 1=sin target, 2=step target
extern uint8  Debug_Angle_D_First;          // 0=error D, 1=measurement D
extern float  Debug_Angle_Vel_Target;       // VOFA: 鐟欐帡鈧喎瀹抽惄顔界垼
extern uint8_t g_led_flag;                // 0=green(normal) 1=blue(object) 2=yellow(low voltage)
extern uint8_t g_scan_progress;            // Scan progress 0-100, 0=not scanning
extern float  Debug_Angle_Vel_Real;         // VOFA: 鐎圭偤妾憴鎺椻偓鐔峰
extern float  Debug_Kp_Left;               // 瀹革箒鐤嗙拫鍐槸Kp
extern float  Debug_Ki_Left;               // 瀹革箒鐤嗙拫鍐槸Ki
extern float  Debug_Kp_Right;              // 閸欏疇鐤嗙拫鍐槸Kp
extern float  Debug_Ki_Right;              // 閸欏疇鐤嗙拫鍐槸Ki

/**********************************************
* 婢舵牠鍎撮崙鑺ユ殶婢圭増妲?
**********************************************/
void Car_Go(void);                          // 鐏忓繗婧呮稉鏄忕箥鐞涘苯鍤遍弫?
void Light_Process(void);
void Set_Speed(void);                       // 鐠佸墽鐤嗛惄顔界垼闁喎瀹?
void Get_Speed(void);                       // 閼惧嘲褰囩€圭偤妾柅鐔峰
uint8 Check_Edge(void);                      // 鏉堝湱绱Λ鈧ù?
void Get_IMU(void);                         // 閼惧嘲褰囬梽鈧摶杞板崕閺佺増宓?
void Set_Out(void);                         // 閻㈠灚婧€鏉堟挸鍤幒褍鍩?

void Normal_Run(void);                      // 鐢瓕顫夊顏囨姉鏉╂劘顢?
void Straight_Run(void);                    // 閻╃顢戞潻鎰攽
void Turn_Left_Run(void);                   // 瀹革箒娴嗛幒褍鍩?
void Turn_Right_Run(void);                  // 閸欏疇娴嗛幒褍鍩?

void Mileage_Mode_Run(void);                // 闁插瞼鈻煎Ο鈥崇础閹粯甯?
void Mileage_Run_Stage_2(void);             // 闁插瞼鈻奸梼鑸殿唽2

void Safety_Check(void);                    // 缂佺喍绔寸€瑰鍙忓Λ鈧ù瀣剁礄娴ｅ骸甯?閸絻娴嗛敍灞剧槨濞嗏€茶厬閺傤厽娓堕崗鍫ｇ殶閻㈩煉绱?
void Build_Mode_Get_Error(void);            // 瀵ゅ搫娴樺Ο鈥崇础閼惧嘲褰囩拠顖氭▕
void Load_All_Flash_Data_For_VOFA(void);    // VOFA鐎电厧鍤敍姘鏉炵唇lash閸忋劑鍎撮崷鏉挎禈閺佺増宓?

// 鐠嬪啳鐦Ο鈥崇础
void Debug_Wheel_Tuning(void);              // 鏉烆喖鐡橮I鐠嬪啫寮敍鍫濅箯閸欏疇鐤嗛崗杈╂暏閿?
void Debug_Ground_Test(void);               // 娑撳婀村ù瀣槸
void Debug_Angle_Tuning(void);              // 鐟欐帒瀹抽悳顖濈殶閸欏偊绱欐０鍕殌閿?
void Debug_Normal_Trace(void);              // 閺咁噣鈧艾鎯婃潻鐧哥礄妫板嫮鏆€閿?
void Debug_Set_Out(void);                   // 鐠嬪啳鐦稉鎾舵暏缁墽鐣漃WM鏉堟挸鍤?

void Set_Mileage_Turn_Exp_Speed(float angle_target);

#endif
