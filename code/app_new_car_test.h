/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: app_new_car_test.h
Author: Claude (based on Cross_Z's framework)
Version:0.0               Date: 2026.6.27
Description: 鏂拌溅鍩虹鍔熻兘妫�娴� / 纭欢 Bring-Up 娴嬭瘯 鈥斺�� 澶存枃浠�
             閫愪釜娴嬭瘯鏂拌溅纭欢妯″潡锛氱數鏈恒�佽渹楦ｅ櫒銆両MU銆丄DC銆�
             涓插彛銆丱LED/鎸夐敭銆佽礋鍘嬮鎵囥�佺紪鐮佸櫒銆佷娇鑳藉紑鍏崇瓑銆�
Others:      姣忎釜娴嬭瘯妯″紡鐙珛锛屽彧鍒濆鍖栬妯″紡鎵�闇�澶栬銆�
             涓嶇牬鍧忓師鏈夋寮忚窇杞︿唬鐮併��
             鏈ご鏂囦欢涓嶄緷璧� headfiles.h锛屽彧渚濊禆 zf_common_headfile.h
             搴曞眰椹卞姩搴擄紝閬垮厤鎷栧叆 Ctrl/OLEDKeyboard 绛夎窇杞﹂�昏緫銆�
Function List:
             1. NewCarTest_Init()  鈥� 鏍规嵁 TEST_MODE 鍒濆鍖栧搴斿璁�
             2. NewCarTest_Loop()  鈥� 鏍规嵁 TEST_MODE 寰幆鎵ц娴嬭瘯閫昏緫
History:
<author>  <time>      <version > <desc>
Claude    2026.6.27   0.0        鍒涘缓鍒濆鐗堟湰
Claude    2026.6.27   0.1        瑙ｉ櫎 headfiles.h 渚濊禆锛岀簿绠�涓� zf_common_headfile.h
**************************************************/

#ifndef __APP_NEW_CAR_TEST_H
#define __APP_NEW_CAR_TEST_H

// 鍙寘鍚簳灞傞┍鍔ㄥ簱锛屼笉鍖呭惈 headfiles.h锛堝悗鑰呬細鎷栧叆 Ctrl/OLEDKeyboard/pid 绛夎窇杞︿唬鐮侊級
#include "zf_common_headfile.h"

/*********************************** 娴嬭瘯妯″紡鏋氫妇 ***********************************/
typedef enum
{
    TEST_NONE = 0,              // 鏃犳祴璇曪紝瀹夊叏鐘舵�侊紝鎵�鏈夎緭鍑哄叧闂�
    TEST_MOTOR,                 // 鐢垫満椹卞姩娴嬭瘯
    TEST_BUZZER,                // 铚傞福鍣ㄦ祴璇�
    TEST_IMU,                   // 闄�铻轰华 / IMU 娴嬭瘯
    TEST_ADC_FORWARD,           // 鍓嶇灮 ADC / 鍏夌數绠℃祴璇�
    TEST_UART_VOFA,             // 涓插彛閫氫俊娴嬭瘯
    TEST_OLED_KEY,              // 閿樉 / OLED 娴嬭瘯锛堥渶瑕� OLED 椹卞姩閾撅級
    TEST_FAN,                   // 璐熷帇椋庢墖娴嬭瘯
    TEST_ENCODER,               // 鍏夋爡缂栫爜鍣� / 娴嬮�熸祴璇�
    TEST_ENABLE_SWITCH,         // 浣胯兘寮�鍏虫祴璇�
    TEST_BUTTON,                // 鑷畾涔夋寜閿祴璇�
    TEST_VOLTAGE_CURRENT,       // 鐢靛帇 / 鐢垫祦妫�娴嬫祴璇�
    TEST_COUNT                  // 娴嬭瘯椤圭洰鎬绘暟锛堢敤浜庤竟鐣屾鏌ワ級
} NewCarTestMode_e;

/*********************************** 娴嬭瘯妯″紡閫夋嫨 ***********************************/
/*
 *  鍦ㄨ繖閲屼慨鏀瑰畯瀹氫箟锛岄�夋嫨瑕佺儳褰曟祴璇曠殑鍔熻兘銆�
 *  姣忔鍙祴涓�涓ā鍧楋紝閬垮厤澶氫釜楂樺姛鐜囨ā鍧楀悓鏃跺伐浣溿��
 *
 *  鍙�夊�硷細
 *    TEST_NONE           - 瀹夊叏鐘舵�侊紝CPU 绌哄惊鐜�
 *    TEST_MOTOR          - 鐢垫満椹卞姩
 *    TEST_BUZZER         - 铚傞福鍣�
 *    TEST_IMU            - 闄�铻轰华 IMU
 *    TEST_ADC_FORWARD    - 鍓嶇灮鍏夌數绠� ADC
 *    TEST_UART_VOFA      - 涓插彛閫氫俊
 *    TEST_OLED_KEY       - OLED + 鎸夐敭
 *    TEST_FAN            - 璐熷帇椋庢墖
 *    TEST_ENCODER        - 缂栫爜鍣ㄦ祴閫�
 *    TEST_ENABLE_SWITCH  - 浣胯兘寮�鍏�
 *    TEST_BUTTON         - 鑷畾涔夋寜閿�
 *    TEST_VOLTAGE_CURRENT- 鐢靛帇鐢垫祦妫�娴�
 */
#define NEW_CAR_TEST_MODE   TEST_BUZZER
/*********************************** 瀹夊叏閰嶇疆瀹� ***********************************/
/*
 *  浠ヤ笅瀹忕敤浜庣數鏈� / 椋庢墖娴嬭瘯鐨勫畨鍏ㄥ弬鏁般��
 *  淇敼杩欎簺鍊兼潵璋冩暣娴嬭瘯鏃剁殑杈撳嚭寮哄害銆�
 *  PWM_DUTY_MAX = 10000锛堝畾涔夊湪 zf_driver_pwm.h锛�
 */

// ---------- 鐢垫満娴嬭瘯 ----------
#define MOTOR_TEST_DUTY_LOW      6000    // 浣庡崰绌烘瘮 10%
#define MOTOR_TEST_DUTY_MID      5000    // 涓崰绌烘瘮 15%
#define MOTOR_TEST_DUTY_HIGH     8000    // 楂樺崰绌烘瘮 20%锛堝缓璁笉瓒呰繃 30%锛�

// ---------- 璐熷帇椋庢墖娴嬭瘯 ----------
#define FAN_TEST_DUTY_LOW        2000    // 浣庡崰绌烘瘮 5%
#define FAN_TEST_DUTY_MID        5000    // 涓崰绌烘瘮 10%
#define FAN_TEST_DUTY_HIGH       8000    // 楂樺崰绌烘瘮 20%

// ---------- 娴嬭瘯寰幆闂撮殧 ----------
#define TEST_LOOP_DELAY_MS         10    // 榛樿寰幆闂撮殧锛坢s锛�
#define TEST_FAST_LOOP_DELAY_MS     3    // 蹇�熷惊鐜棿闅旓紙ms锛夛紝鐢ㄤ簬缂栫爜鍣ㄧ瓑

/*********************************** 纭欢寮曡剼瀹忥紙浠� Fun.h 澶嶅埗锛� ***********************************/
/*
 *  杩欎簺瀹忓師鏈畾涔夊湪 Fun.h 涓紝鐢变簬娴嬭瘯浠ｇ爜涓嶅寘鍚� headfiles.h
 *  锛堝悗鑰呬細閾惧紡鍖呭惈 Fun.h 鍙婃墍鏈夎窇杞﹂�昏緫锛夛紝鍥犳鍦ㄦ澶勫崟鐙鍒朵竴浠姐��
 *  濡傛灉寮曡剼鏈夊彉鍔紝璇峰悓姝ヤ慨鏀广��
 */
//#define Left_Motor_PWM      ATOM3_CH1_P15_7
//#define Left_Motor_DIR      ATOM3_CH0_P15_5
#define Left_Motor_PWM      ATOM3_CH3_P00_12
#define Left_Motor_DIR      ATOM2_CH0_P00_9
#define Right_Motor_PWM     ATOM1_CH3_P00_4
#define Right_Motor_DIR     ATOM1_CH5_P00_6
#define Suction_Motor_PWM   ATOM1_CH6_P00_7
#define Suction_Motor_DIR   ATOM3_CH3_P00_12

// 浣胯兘寮�鍏冲紩鑴氾紙楂樼數骞� = 寮�鍚級
#define ENABLE_SWITCH_PIN   P20_7

// 鑷畾涔夋寜閿紩鑴�
#define BUTTON_PIN          P22_3

/*********************************** 鏁版嵁绫诲瀷锛堜粠 Fun.h 澶嶅埗锛� ***********************************/
// 娴偣鏁� <-> 4瀛楄妭鏁扮粍 鍏辩敤浣擄紝鐢ㄤ簬 VOFA 涓插彛鍙戦��
typedef union floatu8data
{
    float floatdata;
    uint8 u8data[4];
} floatu8data;

/*********************************** 鍑芥暟澹版槑 ***********************************/
void NewCarTest_Init(void);
void NewCarTest_Loop(void);

#endif  // __APP_NEW_CAR_TEST_H
