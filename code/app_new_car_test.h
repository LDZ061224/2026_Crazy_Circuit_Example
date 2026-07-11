/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: app_new_car_test.h
Author: Claude (based on Cross_Z's framework)
Version:0.0               Date: 2026.6.27
Description: 閺傛媽婧呴崺铏诡攨閸旂喕鍏樺Λ锟藉ù锟� / 绾兛娆� Bring-Up 濞村鐦� 閳ユ柡锟斤拷 婢跺瓨鏋冩禒锟�
             闁劒閲滃ù瀣槸閺傛媽婧呯涵顑挎濡�虫健閿涙氨鏁搁張鎭掞拷浣芥腹妤︼絽娅掗妴涓U閵嗕竸DC閵嗭拷
             娑撴彃褰涢妴涓盠ED/閹稿鏁妴浣界閸樺顥撻幍鍥ワ拷浣虹椽閻礁娅掗妴浣峰▏閼宠棄绱戦崗宕囩搼閵嗭拷
Others:      濮ｅ繋閲滃ù瀣槸濡�崇础閻欘剛鐝涢敍灞藉涧閸掓繂顫愰崠鏍嚉濡�崇础閹碉拷闂囷拷婢舵牞顔曢妴锟�
             娑撳秶鐗崸蹇撳斧閺堝顒滃蹇氱獓鏉烇缚鍞惍浣碉拷锟�
             閺堫剙銇旈弬鍥︽娑撳秳绶风挧锟� headfiles.h閿涘苯褰ф笟婵婄 zf_common_headfile.h
             鎼存洖鐪版す鍗炲З鎼存搫绱濋柆鍨帳閹锋牕鍙� Ctrl/OLEDKeyboard 缁涘绐囨潪锕傦拷鏄忕帆閵嗭拷
Function List:
             1. NewCarTest_Init()  閳ワ拷 閺嶈宓� TEST_MODE 閸掓繂顫愰崠鏍ь嚠鎼存柨顦荤拋锟�
             2. NewCarTest_Loop()  閳ワ拷 閺嶈宓� TEST_MODE 瀵邦亞骞嗛幍褑顢戝ù瀣槸闁槒绶�
History:
<author>  <time>      <version > <desc>
Claude    2026.6.27   0.0        閸掓稑缂撻崚婵嗩潗閻楀牊婀�
Claude    2026.6.27   0.1        鐟欙綁娅� headfiles.h 娓氭繆绂嗛敍宀�绨跨粻锟芥稉锟� zf_common_headfile.h
**************************************************/

#ifndef __APP_NEW_CAR_TEST_H
#define __APP_NEW_CAR_TEST_H

// 閸欘亜瀵橀崥顐㈢俺鐏炲倿鈹嶉崝銊ョ氨閿涘奔绗夐崠鍛儓 headfiles.h閿涘牆鎮楅懓鍛窗閹锋牕鍙� Ctrl/OLEDKeyboard/pid 缁涘绐囨潪锔垮敩閻緤绱�
#include "zf_common_headfile.h"

/*********************************** 濞村鐦Ο鈥崇础閺嬫矮濡� ***********************************/
typedef enum
{
    TEST_NONE = 0,              // 閺冪姵绁寸拠鏇礉鐎瑰鍙忛悩鑸碉拷渚婄礉閹碉拷閺堝绶崙鍝勫彠闂傦拷
    TEST_MOTOR,                 // 閻㈠灚婧�妞瑰崬濮╁ù瀣槸
    TEST_BUZZER,                // 閾氬倿绂忛崳銊︾ゴ鐠囷拷
    TEST_IMU,                   // 闂勶拷閾昏桨鍗� / IMU 濞村鐦�
    TEST_ADC_FORWARD,           // 閸撳秶鐏� ADC / 閸忓鏁哥粻鈩冪ゴ鐠囷拷
    TEST_UART_VOFA,             // 娑撴彃褰涢柅姘繆濞村鐦�
    TEST_OLED_KEY,              // 闁款喗妯� / OLED 濞村鐦敍鍫ユ付鐟曪拷 OLED 妞瑰崬濮╅柧鎾呯礆
    TEST_FAN,                   // 鐠愮喎甯囨搴㈠濞村鐦�
    TEST_ENCODER,               // 閸忓鐖＄紓鏍垳閸ｏ拷 / 濞村锟界喐绁寸拠锟�
    TEST_ENABLE_SWITCH,         // 娴ｈ儻鍏樺锟介崗铏ゴ鐠囷拷
    TEST_BUTTON,                // 閼奉亜鐣炬稊澶嬪瘻闁款喗绁寸拠锟�
    TEST_WS2812,                // WS2812 LED test
    TEST_VOLTAGE_CURRENT,       // 閻㈤潧甯� / 閻㈠灚绁﹀Λ锟藉ù瀣ゴ鐠囷拷
    TEST_COUNT                  // 濞村鐦い鍦窗閹粯鏆熼敍鍫㈡暏娴滃氦绔熼悾灞绢梾閺屻儻绱�
} NewCarTestMode_e;

/*********************************** 濞村鐦Ο鈥崇础闁瀚� ***********************************/
/*
 *  閸︺劏绻栭柌灞兼叏閺�鐟扮暞鐎规矮绠熼敍宀勶拷澶嬪鐟曚胶鍎宠ぐ鏇熺ゴ鐠囨洜娈戦崝鐔诲厴閵嗭拷
 *  濮ｅ繑顐奸崣顏呯ゴ娑擄拷娑擃亝膩閸ф绱濋柆鍨帳婢舵矮閲滄妯哄閻滃洦膩閸ф鎮撻弮璺轰紣娴ｆ嚎锟斤拷
 *
 *  閸欘垶锟藉锟界》绱�
 *    TEST_NONE           - 鐎瑰鍙忛悩鑸碉拷渚婄礉CPU 缁屽搫鎯婇悳锟�
 *    TEST_MOTOR          - 閻㈠灚婧�妞瑰崬濮�
 *    TEST_BUZZER         - 閾氬倿绂忛崳锟�
 *    TEST_IMU            - 闂勶拷閾昏桨鍗� IMU
 *    TEST_ADC_FORWARD    - 閸撳秶鐏崗澶屾暩缁狅拷 ADC
 *    TEST_UART_VOFA      - 娑撴彃褰涢柅姘繆
 *    TEST_OLED_KEY       - OLED + 閹稿鏁�
 *    TEST_FAN            - 鐠愮喎甯囨搴㈠
 *    TEST_ENCODER        - 缂傛牜鐖滈崳銊︾ゴ闁拷
 *    TEST_ENABLE_SWITCH  - 娴ｈ儻鍏樺锟介崗锟�
 *    TEST_BUTTON         - 閼奉亜鐣炬稊澶嬪瘻闁匡拷
 *    TEST_VOLTAGE_CURRENT- 閻㈤潧甯囬悽鍨ウ濡拷濞达拷
 */
#define NEW_CAR_TEST_MODE   TEST_ENCODER
/*********************************** 鐎瑰鍙忛柊宥囩枂鐎癸拷 ***********************************/
/*
 *  娴犮儰绗呯�瑰繒鏁ゆ禍搴ｆ暩閺堬拷 / 妞嬪孩澧栧ù瀣槸閻ㄥ嫬鐣ㄩ崗銊ュ棘閺佽埇锟斤拷
 *  娣囶喗鏁兼潻娆庣昂閸婂吋娼电拫鍐╂殻濞村鐦弮鍓佹畱鏉堟挸鍤鍝勫閵嗭拷
 *  PWM_DUTY_MAX = 10000閿涘牆鐣炬稊澶婃躬 zf_driver_pwm.h閿涳拷
 */

// ---------- 閻㈠灚婧�濞村鐦� ----------
#define MOTOR_TEST_DUTY_LOW      6000    // 娴ｅ骸宕扮粚鐑樼槷 10%
#define MOTOR_TEST_DUTY_MID      5000    // 娑擃厼宕扮粚鐑樼槷 15%
#define MOTOR_TEST_DUTY_HIGH     8000    // 妤傛ê宕扮粚鐑樼槷 20%閿涘牆缂撶拋顔荤瑝鐡掑懓绻� 30%閿涳拷

// ---------- 鐠愮喎甯囨搴㈠濞村鐦� ----------
#define FAN_TEST_DUTY_LOW        2000    // 娴ｅ骸宕扮粚鐑樼槷 5%
#define FAN_TEST_DUTY_MID        5000    // 娑擃厼宕扮粚鐑樼槷 10%
#define FAN_TEST_DUTY_HIGH       8000    // 妤傛ê宕扮粚鐑樼槷 20%

// ---------- 濞村鐦顏嗗箚闂傛挳娈� ----------
#define TEST_LOOP_DELAY_MS         10    // 姒涙顓诲顏嗗箚闂傛挳娈ч敍鍧閿涳拷
#define TEST_FAST_LOOP_DELAY_MS     3    // 韫囶偊锟界喎鎯婇悳顖炴？闂呮棑绱檓s閿涘绱濋悽銊ょ艾缂傛牜鐖滈崳銊х搼

/*********************************** 绾兛娆㈠鏇″壖鐎瑰骏绱欐禒锟� Fun.h 婢跺秴鍩楅敍锟� ***********************************/
/*
 *  鏉╂瑤绨虹�瑰繐甯張顒�鐣炬稊澶婃躬 Fun.h 娑擃叏绱濋悽鍙樼艾濞村鐦禒锝囩垳娑撳秴瀵橀崥锟� headfiles.h
 *  閿涘牆鎮楅懓鍛窗闁炬儳绱￠崠鍛儓 Fun.h 閸欏﹥澧嶉張澶庣獓鏉烇箓锟芥槒绶敍澶涚礉閸ョ姵顒濋崷銊︻劃婢跺嫬宕熼悪顒�顦查崚鏈电娴犲锟斤拷
 *  婵″倹鐏夊鏇″壖閺堝褰夐崝顭掔礉鐠囧嘲鎮撳銉ゆ叏閺�骞匡拷锟�
 */
#define Left_Motor_PWM      ATOM3_CH1_P15_7
#define Left_Motor_DIR      ATOM3_CH0_P15_5
//#define Left_Motor_PWM      ATOM3_CH3_P00_12
//#define Left_Motor_DIR      ATOM2_CH0_P00_9
#define Motor_DIR      ATOM0_CH3_P00_4
#define Motor_PWM      ATOM1_CH6_P00_7
#define Right_Motor_PWM     ATOM1_CH3_P00_4
#define Right_Motor_DIR     ATOM1_CH5_P00_6
#define Suction_Motor_PWM   ATOM1_CH6_P00_7
#define Suction_Motor_DIR   ATOM3_CH3_P00_12

// 娴ｈ儻鍏樺锟介崗鍐茬穿閼存熬绱欐妯兼暩楠烇拷 = 瀵拷閸氼垽绱
#define ENABLE_SWITCH_PIN   P20_7

// 閼奉亜鐣炬稊澶嬪瘻闁款喖绱╅懘锟
#define BUTTON_PIN          P22_3

/*********************************** 閺佺増宓佺猾璇茬�烽敍鍫滅矤 Fun.h 婢跺秴鍩楅敍锟� ***********************************/
// 濞搭喚鍋ｉ弫锟� <-> 4鐎涙濡弫鎵矋 閸忚京鏁ゆ担鎿勭礉閻€劋绨� VOFA 娑撴彃褰涢崣鎴︼拷锟�
typedef union floatu8data
{
    float floatdata;
    uint8 u8data[4];
} floatu8data;

/*********************************** 閸戣姤鏆熸竟鐗堟 ***********************************/
void NewCarTest_Init(void);
void NewCarTest_Loop(void);

#endif  // __APP_NEW_CAR_TEST_H
