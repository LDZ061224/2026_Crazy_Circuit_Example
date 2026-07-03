# 新车基础功能测试 — 引脚配置

<!-- > 分支: `test/new-car`
> 切回正式跑车: `git checkout master` 或改 `cpu0_main.c` 中 `NEW_CAR_TEST_ENABLE = 0` -->

---

## 电机驱动 (TEST_MOTOR)

| 信号 | 引脚 | 类型 | 参数 |
|------|------|------|------|
| 左电机占空比 | P15_7 (ATOM3_CH1) | PWM | 30kHz |
| 左电机方向 | P15_5 (ATOM3_CH0) | PWM | 30kHz, 10000=反转, 0=正转 |
| 右电机占空比 | P00_4 (ATOM1_CH3) | PWM | 30kHz |
| 右电机方向 | P00_6 (ATOM1_CH5) | PWM | 30kHz, 10000=反转, 0=正转 |

## 负压风扇 (TEST_FAN)

| 信号 | 引脚 | 类型 | 参数 |
|------|------|------|------|
| 风扇占空比 | P00_7 (ATOM1_CH6) | PWM | 100kHz |
| 风扇方向 | P00_12 (ATOM3_CH3) | PWM | 100kHz, 10000=吸风 |

## 蜂鸣器 (TEST_BUZZER)

| 信号 | 引脚 | 类型 | 参数 |
|------|------|------|------|
| 无源蜂鸣器 | P33_4 | PWM | 40kHz, 50% 占空比 |

## 陀螺仪 (TEST_IMU)

| 信号 | 引脚 | 类型 | 备注 |
|------|------|------|------|
| SPI SCK | P15_8 (SPI2_SCLK) | 硬件 SPI | |
| SPI MOSI | P15_6 (SPI2_MOSI) | 硬件 SPI | |
| SPI MISO | P15_4 (SPI2_MISO) | 硬件 SPI | |
| CS | P15_2 | GPIO | |

> IMU660RB SPI 引脚由库 `zf_device_imu660rb.h` 宏定义，测试代码不直接引用。

## 前瞻光电管 ADC (TEST_ADC_FORWARD)

| 通道 | ADC 引脚 | 编号 |
|------|----------|------|
| ADC2_CH14 | A48 | adc14 |
| ADC2_CH12 | A46 | adc13 |
| ADC2_CH10 | A44 | adc12 |
| ADC2_CH6 | A38 | adc11 |
| ADC2_CH4 | A36 | adc10 |
| ADC1_CH9 | A25 | adc9 |
| ADC1_CH5 | A21 | adc8 |
| ADC1_CH1 | A17 | adc7 |
| ADC0_CH13 | A13 | adc6 |
| ADC0_CH11 | A11 | adc5 |
| ADC0_CH8 | A8 | adc4 |
| ADC0_CH6 | A6 | adc3 |
| ADC0_CH4 | A4 | adc2 |
| ADC0_CH2 | A2 | adc1 |
| ADC0_CH0 | A0 | adc0 |

> 12 位精度，原始范围 0~4095

## 编码器 (TEST_ENCODER)

| 信号 | 引脚 | 类型 |
|------|------|------|
| 左编码器 A | P02_8 (TIM4_CH1) | 正交编码 |
| 左编码器 B | P00_9 (TIM4_CH2) | 正交编码 |
| 右编码器 A | (TIM2_ENCODER_CH1_P33_7) | 正交编码 |
| 右编码器 B | (TIM2_ENCODER_CH2_P33_6) | 正交编码 |

## 串口 (TEST_UART_VOFA / 其他测试共用)

| 信号 | 引脚 | 参数 |
|------|------|------|
| TX | P33_9 (UART2_TX) | 115200, 8N1 |
| RX | P33_8 (UART2_RX) | 115200, 8N1 |

## OLED + 按键 (TEST_OLED_KEY)

| 信号 | 引脚 | 类型 | 备注 |
|------|------|------|------|
| OLED SCL | P33_8 | 软件 I2C | |
| OLED SDA | P33_6 | 软件 I2C | |
| CH455 键盘 | — | 软件 I2C | 与 OLED 同总线 |

## 使能开关 (TEST_ENABLE_SWITCH)

| 信号 | 引脚 | 类型 |
|------|------|------|
| 使能开关 | P20_7 | GPIO 下拉输入, HIGH=开启 |

## 自定义按键 (TEST_BUTTON)

| 信号 | 引脚 | 类型 |
|------|------|------|
| 按键 | P22_3 | GPIO 下拉输入 |

## 电压 / 电流检测 (TEST_VOLTAGE_CURRENT)

| 信号 | ADC 引脚 | 公式 |
|------|----------|------|
| 电池电压 | A5 (ADC0_CH5) | `raw × 3.3 × 11 / 4095 + 0.567` V |
| 电池电流 | A10 (ADC0_CH10) | `raw × 0.0008 × 0.41 / (20 × 0.015)` A |

## WS2812 灯板 (TEST_WS2812)

| 信号 | 引脚 | 类型 | 参数 |
|------|------|------|------|
| 数据线 | P20_9 | GPIO bit-bang | 8 颗灯珠 |

---
<!-- 
## 测试模式切换

修改 `code/app_new_car_test.h` 第 65 行:

```c
#define NEW_CAR_TEST_MODE   TEST_NONE  // 改成对应的 TEST_xxx
```

烧录后串口 `UART2` (P33_9/P33_8) 115200 观察输出。 -->
