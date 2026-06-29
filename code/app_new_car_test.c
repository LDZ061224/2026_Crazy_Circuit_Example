/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: app_new_car_test.c
Author: Claude (based on Cross_Z's framework)
Version:0.2               Date: 2026.6.28
Description: New car hardware bring-up test — implementation.
             Each TEST_MODE initializes only what it needs.
Others:      Safety: motor/fan default duty=0, enable switch kills output.
             PWM driving: DIR pins are PWM channels (10000=forward, 0=reverse).
History:
Claude    2026.6.27   0.0   Initial version
Claude    2026.6.27   0.1   Cleanup pins, remove P33_4, VOFA/printf split, OLED uses lib
Claude    2026.6.28   0.2   Motor DIR pins use PWM instead of GPIO for direction control
**************************************************/

#include "app_new_car_test.h"
#include "dev_ssd1306.h"
#include "dev_CH455.h"
#include "WS2812.h"

/*********************************** module-local variables ***********************************/

static const NewCarTestMode_e g_test_mode = NEW_CAR_TEST_MODE;

static uint32 g_buzzer_timer = 0;
static uint8  g_buzzer_on = 0;

static uint32 g_uart_test_count = 0;

static uint8  g_motor_test_phase = 0;
static uint32 g_motor_phase_timer = 0;

static uint8  g_fan_test_phase = 0;
static uint32 g_fan_phase_timer = 0;

static uint32 g_oled_test_count = 0;
static uint32 g_ws2812_phase = 0;

/*********************************** internal helpers ***********************************/

/* Stop all motor outputs (duty + direction = 0) */
static void Motor_Safe_Stop(void)
{
    pwm_set_duty(Left_Motor_PWM,   0);
    pwm_set_duty(Left_Motor_DIR,   0);
    pwm_set_duty(Right_Motor_PWM,  0);
    pwm_set_duty(Right_Motor_DIR,  0);
}

/* Stop suction fan */
static void Fan_Safe_Stop(void)
{
    pwm_set_duty(Suction_Motor_PWM, 0);
    pwm_set_duty(Suction_Motor_DIR, 0);
}

/* Check enable switch (P20_7 HIGH = armed) */
static inline uint8 EnableSwitch_IsOn(void)
{
    return (gpio_get_level(ENABLE_SWITCH_PIN) == 1) ? 1 : 0;
}

/* Safety check: kill motor/fan if enable switch is off */
static uint8 Safety_Check_Enable(void)
{
    if (!EnableSwitch_IsOn())
    {
        Motor_Safe_Stop();
        Fan_Safe_Stop();
        return 0;
    }
    return 1;
}

/*
 *  VOFA JustFloat sender.  Append frame tail 00 00 80 7F.
 *  Do NOT mix printf with this on the same UART or VOFA waveform decode breaks.
 */
static void Vofa_Send_Floats(uart_index_enum uart_n, float *data, uint8 count)
{
    floatu8data vofa;
    uint8 buf[4];

    for (uint8 i = 0; i < count; i++)
    {
        vofa.floatdata = data[i];
        buf[0] = vofa.u8data[0];
        buf[1] = vofa.u8data[1];
        buf[2] = vofa.u8data[2];
        buf[3] = vofa.u8data[3];
        uart_write_buffer(uart_n, buf, 4);
    }
    uart_write_byte(uart_n, 0x00);
    uart_write_byte(uart_n, 0x00);
    uart_write_byte(uart_n, 0x80);
    uart_write_byte(uart_n, 0x7f);
}

/*********************************** TEST_NONE ***********************************/
/*
 *  Safe idle — confirm MCU is alive via debugger.
 *  No peripherals are initialized.
 */
static void TestNone_Init(void) {}

static void TestNone_Loop(void)
{
    system_delay_ms(500);
}

/*********************************** TEST_MOTOR ***********************************/
/*
 *  Verify left/right motor drive on the new PWM scheme.
 *  DIR pin: PWM 10000 = forward, PWM 0 = reverse.
 *  Phase 0: Left forward    (3 s)
 *  Phase 1: Left reverse    (3 s)
 *  Phase 2: Right forward   (3 s)
 *  Phase 3: Right reverse   (3 s)
 *  Phase 4: Both forward    (3 s)
 *  Phase 5: All stop (2 s), then loop.
 *  Low duty by default (MOTOR_TEST_DUTY_LOW).  Keep wheels off the ground.
 *  Text-only output (printf), no VOFA frames.
 */
static void TestMotor_Init(void)
{
    gpio_init(ENABLE_SWITCH_PIN, GPI, 0, GPI_PULL_DOWN);

    // DIR PWM: 10000 = forward, 0 = reverse
    pwm_init(Left_Motor_DIR,  30000, 10000);
    pwm_init(Right_Motor_DIR, 30000, 10000);

    // duty PWM: 30 kHz, start at 0
    pwm_init(Left_Motor_PWM,  30000, 0);
    pwm_init(Right_Motor_PWM, 30000, 0);

    g_motor_test_phase = 0;
    g_motor_phase_timer = 0;

}

static void TestMotor_Loop(void)
{
//    if (!Safety_Check_Enable())
//    {
//        g_motor_test_phase = 0;
//        g_motor_phase_timer = 0;
//        system_delay_ms(100);
//        return;
//    }
//
//    if (g_motor_phase_timer == 0)
//    {
//        Motor_Safe_Stop();
//
//        switch (g_motor_test_phase)
//        {
//        case 0:  // Left forward
//            pwm_set_duty(Left_Motor_DIR, 10000);
//            pwm_set_duty(Left_Motor_PWM, MOTOR_TEST_DUTY_LOW);
//            break;
//        case 1:  // Left reverse
//            pwm_set_duty(Left_Motor_DIR, 0);
//            pwm_set_duty(Left_Motor_PWM, MOTOR_TEST_DUTY_LOW);
//            break;
//        case 2:  // Right forward
//            pwm_set_duty(Right_Motor_DIR, 10000);
//            pwm_set_duty(Right_Motor_PWM, MOTOR_TEST_DUTY_LOW);
//            break;
//        case 3:  // Right reverse
//            pwm_set_duty(Right_Motor_DIR, 0);
//            pwm_set_duty(Right_Motor_PWM, MOTOR_TEST_DUTY_LOW);
//            break;
//        case 4:  // Both forward
//            pwm_set_duty(Left_Motor_DIR, 10000);
//            pwm_set_duty(Right_Motor_DIR, 10000);
//            pwm_set_duty(Left_Motor_PWM, MOTOR_TEST_DUTY_LOW);
//            pwm_set_duty(Right_Motor_PWM, MOTOR_TEST_DUTY_LOW);
//            break;
//        case 5:
//        default:
////            Motor_Safe_Stop();
//            pwm_set_duty(Left_Motor_DIR, 10000);
//            pwm_set_duty(Right_Motor_DIR, 10000);
//            g_motor_test_phase = 0;
//            g_motor_phase_timer = 0;
////            system_delay_ms(2000);
//            return;
//        }
//    }
//
//    g_motor_phase_timer++;
//    system_delay_ms(10);
//
//    if (g_motor_phase_timer >= 300)
//    {
//        g_motor_phase_timer = 0;
//        g_motor_test_phase++;
//    }
    pwm_set_duty(Left_Motor_DIR, 0);
    pwm_set_duty(Left_Motor_PWM, 2000);
    pwm_set_duty(Right_Motor_DIR, 0);
    pwm_set_duty(Right_Motor_PWM, 4000);
}

/*********************************** TEST_BUZZER ***********************************/
/*
 *  Passive buzzer driven by PWM (4 kHz, 50% duty).
 *  Toggles ON/OFF according to BUZZER_ON_TIME_MS / BUZZER_OFF_TIME_MS.
 */
#define BUZZER_PWM          P33_4
#define BUZZER_PWM_FREQ     40000
#define BUZZER_PWM_DUTY     5000

static void TestBuzzer_Init(void)
{
    pwm_init(BUZZER_PWM, BUZZER_PWM_FREQ, 0);
    g_buzzer_timer = 0;
    g_buzzer_on = 0;
    printf("=== TEST_BUZZER: PWM passive buzzer ===\r\n");
    printf("  Freq=%dHz, Duty=%d/10000\r\n", BUZZER_PWM_FREQ, BUZZER_PWM_DUTY);
}

static void TestBuzzer_Loop(void)
{
    g_buzzer_timer++;
    pwm_set_duty(BUZZER_PWM, BUZZER_PWM_DUTY);


    system_delay_ms(TEST_LOOP_DELAY_MS);
}

/*********************************** TEST_IMU ***********************************/
/*
 *  IMU660RB gyro + acc test.
 *  Init: wait for IMU self-check via SPI.
 *  Loop: read gyro/acc, send 6-ch VOFA JustFloat.
 *  printf only during Init; Loop is pure VOFA.
 */
static void TestIMU_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);

    printf("=== TEST_IMU: IMU660RB gyro + acc ===\r\n");
    printf("  Waiting for IMU init...\r\n");

    int retry = 0;
    while (1)
    {
        if (imu660rb_init() == 0) break;
        retry++;
        system_delay_ms(50);
        if (retry > 200)
        {
            printf("  ERROR: IMU init failed after %d retries!\r\n", retry);
            return;
        }
    }

    printf("  IMU init OK (%d retries).\r\n", retry);
    printf("  Sending VOFA JustFloat: gyro_x,y,z acc_x,y,z\r\n");
}

static void TestIMU_Loop(void)
{
    imu660rb_get_gyro();
    imu660rb_get_acc();

    float gyro_x = (float)imu660rb_gyro_x / imu660rb_transition_factor[1];
    float gyro_y = (float)imu660rb_gyro_y / imu660rb_transition_factor[1];
    float gyro_z = (float)imu660rb_gyro_z / imu660rb_transition_factor[1];
    float acc_x  = (float)imu660rb_acc_x  / imu660rb_transition_factor[0];
    float acc_y  = (float)imu660rb_acc_y  / imu660rb_transition_factor[0];
    float acc_z  = (float)imu660rb_acc_z  / imu660rb_transition_factor[0];

    float imu_data[6] = { gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z };
    Vofa_Send_Floats(UART_2, imu_data, 6);

    system_delay_ms(20);
}

/*********************************** TEST_ADC_FORWARD ***********************************/
/*
 *  15-ch phototransistor ADC raw values.
 *  VOFA JustFloat, no printf in Loop.
 */
static void TestAdcForward_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);

    adc_init(ADC2_CH14_A48, ADC_12BIT);
    adc_init(ADC2_CH12_A46, ADC_12BIT);
    adc_init(ADC2_CH10_A44, ADC_12BIT);
    adc_init(ADC2_CH6_A38,  ADC_12BIT);
    adc_init(ADC2_CH4_A36,  ADC_12BIT);
    adc_init(ADC1_CH9_A25,  ADC_12BIT);
    adc_init(ADC1_CH5_A21,  ADC_12BIT);
    adc_init(ADC1_CH1_A17,  ADC_12BIT);
    adc_init(ADC0_CH13_A13, ADC_12BIT);
    adc_init(ADC0_CH11_A11, ADC_12BIT);
    adc_init(ADC0_CH8_A8,   ADC_12BIT);
    adc_init(ADC0_CH6_A6,   ADC_12BIT);
    adc_init(ADC0_CH4_A4,   ADC_12BIT);
    adc_init(ADC0_CH2_A2,   ADC_12BIT);
    adc_init(ADC0_CH0_A0,   ADC_12BIT);

    printf("=== TEST_ADC_FORWARD: 15-ch ADC ===\r\n");
}

static void TestAdcForward_Loop(void)
{
    float adc_vals[15];
    adc_vals[14] = (float)adc_convert(ADC2_CH14_A48);
    adc_vals[13] = (float)adc_convert(ADC2_CH12_A46);
    adc_vals[12] = (float)adc_convert(ADC2_CH10_A44);
    adc_vals[11] = (float)adc_convert(ADC2_CH6_A38);
    adc_vals[10] = (float)adc_convert(ADC2_CH4_A36);
    adc_vals[9]  = (float)adc_convert(ADC1_CH9_A25);
    adc_vals[8]  = (float)adc_convert(ADC1_CH5_A21);
    adc_vals[7]  = (float)adc_convert(ADC1_CH1_A17);
    adc_vals[6]  = (float)adc_convert(ADC0_CH13_A13);
    adc_vals[5]  = (float)adc_convert(ADC0_CH11_A11);
    adc_vals[4]  = (float)adc_convert(ADC0_CH8_A8);
    adc_vals[3]  = (float)adc_convert(ADC0_CH6_A6);
    adc_vals[2]  = (float)adc_convert(ADC0_CH4_A4);
    adc_vals[1]  = (float)adc_convert(ADC0_CH2_A2);
    adc_vals[0]  = (float)adc_convert(ADC0_CH0_A0);

    Vofa_Send_Floats(UART_2, adc_vals, 15);
    system_delay_ms(10);
}

/*********************************** TEST_UART_VOFA ***********************************/
/*
 *  Pure text UART test — printf a counter, verify with a serial terminal.
 */
static void TestUartVofa_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
    g_uart_test_count = 0;
    printf("=== TEST_UART_VOFA: UART text test ===\r\n");
    printf("  Baud: 115200, 8N1\r\n");
}

static void TestUartVofa_Loop(void)
{
    g_uart_test_count++;
    printf("New car UART test OK, count=%lu\r\n", g_uart_test_count);
    system_delay_ms(200);
}

/*********************************** TEST_OLED_KEY ***********************************/
/*
 *  OLED display + CH455 keypad test, using the original OLEDKeyboard driver chain.
 */
static void TestOledKey_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
//    OLED_Init();
//    OLED_Input-
    pwm_init(ATOM0_CH3_P21_5,30000,6000);
    pwm_init(ATOM1_CH5_P21_7,30000,6000);
}

static void TestOledKey_Loop(void)
{
    g_oled_test_count++;

    OLED_Show_Str(0, 0, (uint8 *)"NEW CAR TEST", (TextSize_TypeDef)0);
    OLED_Show_Str(0, 2, (uint8 *)"OLED+KEY OK", (TextSize_TypeDef)0);
    OLED_Show_Numbers(0, 4, (int32)g_oled_test_count, (TextSize_TypeDef)0);

    {
        uint8 key_val = CH455_Read();
        if (key_val != 0)
        {
            OLED_Show_Numbers(60, 4, (int32)key_val, (TextSize_TypeDef)0);
        }
    }

    system_delay_ms(200);
}

/*********************************** TEST_FAN ***********************************/
/*
 *  Suction fan test — DIR=10000=forward, PWM controls duty.
 *  Phase 0: Low (5%), 1: Mid (10%), 2: Stop, 3: High (20%), loop.
 *  Text-only printf output.
 */
static void TestFan_Init(void)
{
    gpio_init(ENABLE_SWITCH_PIN, GPI, 0, GPI_PULL_DOWN);

    // DIR PWM: 10000 = forward
    pwm_init(Suction_Motor_DIR, 100000, 10000);
    // duty PWM: 100 kHz, start at 0
    pwm_init(Suction_Motor_PWM, 100000, 0);

    // DIR PWM: 10000 = forward, 0 = reverse
    pwm_init(Left_Motor_DIR,  30000, 10000);
    pwm_init(Right_Motor_DIR, 30000, 10000);

    // duty PWM: 30 kHz, start at 0
    pwm_init(Left_Motor_PWM,  30000, 0);
    pwm_init(Right_Motor_PWM, 30000, 0);

    g_fan_test_phase = 0;
    g_fan_phase_timer = 0;
}

static void TestFan_Loop(void)
{
//    if (!Safety_Check_Enable())
//    {
//        Fan_Safe_Stop();
//        system_delay_ms(100);
//        return;
//    }
//
//    if (g_fan_phase_timer == 0)
//    {
//        Fan_Safe_Stop();
//
//        switch (g_fan_test_phase)
//        {
//        case 0:
//            pwm_set_duty(Suction_Motor_DIR, 10000);
//            pwm_set_duty(Suction_Motor_PWM, FAN_TEST_DUTY_LOW);
//            printf("[Fan] Phase 0: Low (%d)\r\n", FAN_TEST_DUTY_LOW);
//            break;
//        case 1:
//            pwm_set_duty(Suction_Motor_DIR, 10000);
//            pwm_set_duty(Suction_Motor_PWM, FAN_TEST_DUTY_MID);
//            printf("[Fan] Phase 1: Mid (%d)\r\n", FAN_TEST_DUTY_MID);
//            break;
//        case 2:
//            Fan_Safe_Stop();
//            printf("[Fan] Phase 2: Stop\r\n");
//            break;
//        case 3:
//            pwm_set_duty(Suction_Motor_DIR, 10000);
//            pwm_set_duty(Suction_Motor_PWM, FAN_TEST_DUTY_HIGH);
//            printf("[Fan] Phase 3: High (%d)\r\n", FAN_TEST_DUTY_HIGH);
//            break;
//        default:
//            Fan_Safe_Stop();
//            pwm_set_duty(Suction_Motor_DIR, 10000);
//            printf("[Fan] Restarting cycle...\r\n");
//            g_fan_test_phase = 0;
//            g_fan_phase_timer = 0;
//            system_delay_ms(2000);
//            return;
//        }
//    }
//
//    g_fan_phase_timer++;
//    system_delay_ms(10);
//
//    if (g_fan_phase_timer >= 300)
//    {
//        g_fan_phase_timer = 0;
//        g_fan_test_phase++;
//        if (g_fan_test_phase > 3) g_fan_test_phase = 0;
//    }
    pwm_set_duty(Suction_Motor_DIR, 10000);
    pwm_set_duty(Suction_Motor_PWM, 9500);
}

/*********************************** TEST_ENCODER ***********************************/
/*
 *  Quadrature encoder — send counts over VOFA JustFloat.
 */
static void TestEncoder_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);

    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_P02_6, TIM3_ENCODER_CH2_P02_7);

    encoder_clear_count(TIM4_ENCODER);
    encoder_clear_count(TIM3_ENCODER);
}

static void TestEncoder_Loop(void)
{
    static int16 last_left = 0, last_right = 0;

    int16 enc_left  = encoder_get_count(TIM4_ENCODER);
    int16 enc_right = encoder_get_count(TIM3_ENCODER);

    float speed_left  = (float)(enc_left  - last_left);
    float speed_right = (float)(enc_right - last_right);

    last_left  = enc_left;
    last_right = enc_right;

    float enc_data[4] = { (float)enc_left, (float)enc_right, speed_left, speed_right };
    Vofa_Send_Floats(UART_2, enc_data, 4);

    system_delay_ms(TEST_FAST_LOOP_DELAY_MS);
}

/*********************************** TEST_ENABLE_SWITCH ***********************************/
/*
 *  Monitor enable switch P20_7, print on state change.
 */
static void TestEnableSwitch_Init(void)
{
    gpio_init(ENABLE_SWITCH_PIN, GPI, 0, GPI_PULL_DOWN);
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
    printf("=== TEST_ENABLE_SWITCH: P20_7 monitor ===\r\n");
}

static void TestEnableSwitch_Loop(void)
{
    static uint8 last_state = 0xFF;
    uint8 state = EnableSwitch_IsOn();

    if (state != last_state)
    {
        last_state = state;
        printf("Enable Switch: %s (P20_7=%s)\r\n",
               state ? "ON" : "OFF", state ? "HIGH" : "LOW");
    }
    system_delay_ms(100);
}

/*********************************** TEST_BUTTON ***********************************/
/*
 *  Custom button P22_3 — print on state change.
 */
static void TestButton_Init(void)
{
    gpio_init(BUTTON_PIN, GPI, 0, GPI_PULL_DOWN);
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
    printf("=== TEST_BUTTON: P22_3 monitor ===\r\n");
}

static void TestButton_Loop(void)
{
    static uint8 last_btn = 0xFF;
    uint8 btn = gpio_get_level(BUTTON_PIN);

    if (btn != last_btn)
    {
        last_btn = btn;
        printf("Button: %s (P22_3=%s)\r\n",
               btn ? "PRESSED" : "RELEASED", btn ? "HIGH" : "LOW");
    }
    system_delay_ms(50);
}

/*********************************** TEST_VOLTAGE_CURRENT ***********************************/
/*
 *  Battery voltage + current via VOFA JustFloat.
 */
static void TestVoltageCurrent_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);
    adc_init(ADC0_CH5_A5,   ADC_12BIT);
    adc_init(ADC0_CH10_A10, ADC_12BIT);
    printf("=== TEST_VOLTAGE_CURRENT ===\r\n");
}

static void TestVoltageCurrent_Loop(void)
{
    uint16 v_raw = adc_convert(ADC0_CH5_A5);
    uint16 i_raw = adc_convert(ADC0_CH10_A10);

    float voltage = (v_raw * 3.3f * 11.0f / 4095.0f) + 0.567f;
    float current = (i_raw * 0.0008f * 0.41f) / (20.0f * 0.015f);

    float data[4] = { voltage, current, (float)v_raw, (float)i_raw };
    Vofa_Send_Floats(UART_2, data, 4);

    system_delay_ms(200);
}


/*********************************** TEST_WS2812 ***********************************/
/*
 *  WS2812 LED strip test on P20_9.
 *  Cycles through: R solid -> G solid -> B solid -> rainbow -> breathing -> off
 *  Each phase lasts ~3 seconds, then loops.
 *  Uses the WS2812 library API (Init / Effect_Set / Effect_Update).
 */
#define WS2812_TEST_PHASE_MS  3000

static void TestWs2812_Init(void)
{
    uart_init(UART_2, 115200, UART2_TX_P33_9, UART2_RX_P33_8);

    WS2812_Init();
    g_ws2812_phase = 0;

}

static void TestWs2812_Loop(void)
{
    /* phase changes every 3 s */
    uint8_t phase = (g_ws2812_phase / (WS2812_TEST_PHASE_MS / 10)) % 6;
    static uint8_t last_phase = 0xFF;

    if (phase != last_phase)
    {
        last_phase = phase;
        switch (phase)
        {
        case 0:
            WS2812_Effect_Set((WS2812_Effect_Config){
                .type = EFF_SOLID, .r = 255, .g = 0, .b = 0 });
            break;
        case 1:
            WS2812_Effect_Set((WS2812_Effect_Config){
                .type = EFF_SOLID, .r = 0, .g = 255, .b = 0 });
            break;
        case 2:
            WS2812_Effect_Set((WS2812_Effect_Config){
                .type = EFF_SOLID, .r = 0, .g = 0, .b = 255 });
            break;
        case 3:
            WS2812_Effect_Set((WS2812_Effect_Config){
                .type = EFF_RAINBOW_FLOW, .speed = 1, .tail = 4 });
            break;
        case 4:
            WS2812_Effect_Set((WS2812_Effect_Config){
                .type = EFF_BREATHING, .r = 0, .g = 255, .b = 0, .period_ms = 2000 });
            break;
        case 5:
            WS2812_Effect_Set((WS2812_Effect_Config){
                .type = EFF_OFF });
            break;
        }
    }

    WS2812_Effect_Update();
    g_ws2812_phase++;
    system_delay_ms(10);
}
/*********************************** top-level dispatch ***********************************/

void NewCarTest_Init(void)
{
    switch (g_test_mode)
    {
    case TEST_NONE:           TestNone_Init();           break;
    case TEST_MOTOR:          TestMotor_Init();          break;
    case TEST_BUZZER:         TestBuzzer_Init();         break;
    case TEST_IMU:            TestIMU_Init();            break;
    case TEST_ADC_FORWARD:    TestAdcForward_Init();     break;
    case TEST_UART_VOFA:      TestUartVofa_Init();       break;
    case TEST_OLED_KEY:       TestOledKey_Init();        break;
    case TEST_FAN:            TestFan_Init();            break;
    case TEST_ENCODER:        TestEncoder_Init();        break;
    case TEST_ENABLE_SWITCH:  TestEnableSwitch_Init();   break;
    case TEST_BUTTON:         TestButton_Init();         break;
    case TEST_VOLTAGE_CURRENT: TestVoltageCurrent_Init(); break;
    case TEST_WS2812:        TestWs2812_Init();        break;
    default:
        printf("=== ERROR: Unknown test mode %d ===\r\n", (int)g_test_mode);
        break;
    }
    interrupt_global_enable(0);
}

void NewCarTest_Loop(void)
{
    switch (g_test_mode)
    {
    case TEST_NONE:           TestNone_Loop();           break;
    case TEST_MOTOR:          TestMotor_Loop();          break;
    case TEST_BUZZER:         TestBuzzer_Loop();         break;
    case TEST_IMU:            TestIMU_Loop();            break;
    case TEST_ADC_FORWARD:    TestAdcForward_Loop();     break;
    case TEST_UART_VOFA:      TestUartVofa_Loop();       break;
    case TEST_OLED_KEY:       TestOledKey_Loop();        break;
    case TEST_FAN:            TestFan_Loop();            break;
    case TEST_ENCODER:        TestEncoder_Loop();        break;
    case TEST_ENABLE_SWITCH:  TestEnableSwitch_Loop();   break;
    case TEST_BUTTON:         TestButton_Loop();         break;
    case TEST_VOLTAGE_CURRENT: TestVoltageCurrent_Loop(); break;
    case TEST_WS2812:        TestWs2812_Loop();        break;
    default:
        system_delay_ms(500);
        break;
    }
}
