/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Hardware_Config.h
Author: Cross_Z
Version:4.0               Date: 2026.7.8
Description: ALL tunable parameters in one place.
             Change values here — no need to dig through source files.
             Sensors / Motors / PID / Geometry / Timing / Safety
**************************************************/

#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

/**********************************************
* [Geometry] — fill in measured millimeter values
**********************************************/
#define TRACK_WIDTH_MM       0.0f   // TODO: 左右轮间距 (mm)
#define LD_MM                0.0f   // TODO: 前瞻距离 — 光电管到转轴 (mm)
#define SENSOR_PITCH_MM      0.0f   // TODO: 光电管两两间距 (mm)

/**********************************************
* [PID] Angle — heading hold (cascaded: TotalAngle → angle rate → diff)
**********************************************/
#define ANGLE_KP             3.0f
#define ANGLE_KI             0.0f
#define ANGLE_KD             1.2f
#define ANGLE_OUT_MAX        1500

/**********************************************
* [PID] Gyro rate — incremental inner loop
**********************************************/
#define GYRO_RATE_KP         0.15f
#define GYRO_RATE_KI         0.0048f
#define GYRO_RATE_KD         0.04f
#define GYRO_RATE_OUT_MAX    500

/**********************************************
* [PID] Gyro PD — position-mode gyro damping (parallel: curvature diff + PD damping)
**********************************************/
#define GYRO_PD_KP           0.008f
#define GYRO_PD_KI           0.0f
#define GYRO_PD_KD           0.002f
#define GYRO_PD_OUT_MAX      500

/**********************************************
* [PID] Turn — curvature gain for parallel trace + Phase1 diff speed
*        kp = curvature→diff gain (larger = sharper turns)
**********************************************/
#define TURN_KP              9.0f
#define TURN_KI              0.0f
#define TURN_KD              0.0f
#define TURN_OUT_MAX         1500

/**********************************************
* [PID] Left wheel speed PI
**********************************************/
#define LEFT_WHEEL_KP        90.5f
#define LEFT_WHEEL_KI        42.5f
#define LEFT_WHEEL_KD        0.0f
#define LEFT_WHEEL_IOUT_MAX  5000
#define LEFT_WHEEL_OUT_MAX   9500

/**********************************************
* [PID] Right wheel speed PI
**********************************************/
#define RIGHT_WHEEL_KP       100.5f
#define RIGHT_WHEEL_KI       37.2f
#define RIGHT_WHEEL_KD       0.0f
#define RIGHT_WHEEL_IOUT_MAX 5000
#define RIGHT_WHEEL_OUT_MAX  9500

/**********************************************
* [PID] Speed IIR filter weights (must sum to 1.0)
**********************************************/
#define SPEED_IIR_W0         0.5f
#define SPEED_IIR_W1         0.3f
#define SPEED_IIR_W2         0.2f

/**********************************************
* [Startup & Calibration]
**********************************************/
#define CALIB_BUFFER_MS      1000        // 陀螺校准前等待 (ms)
#define GYRO_CALIB_SAMPLES   500         // 陀螺零漂采样数
#define GYRO_CALIB_INTERVAL  5           // 采样间隔 (ms)
#define SCAN_START           200         // 阈值扫描起始 tick
#define SCAN_END             1600        // 阈值扫描结束 tick
#define SCAN_TICK_MS         3           // 扫描每 tick 延时 (ms)
#define START_DELAY_TICKS    800         // 使能开关按下后电机启动延迟 (3ms周期)
#define GYRO_INTEGRATION_PERIOD_S 0.003f // 控制循环周期 = PIT周期 (s)
#define GYRO_DEADZONE_DEG_S  2.0f        // 低于此值的陀螺原始读数归零 (deg/s)

/**********************************************
* [Build Mode] Turn & Straight distances (encoder ticks)
**********************************************/
#define TUNE_ELEM_TURN_DELAY      775.0f  // 元件转弯 Phase0 直行距离
#define TUNE_NODE_TURN_DELAY      445.0f  // 节点转弯 Phase0 直行距离
#define TUNE_NODE_STRAIGHT        200.0f  // 过节点后直行距离
#define TUNE_ELEM_STRAIGHT_SHORT  2650.0f // 短直行元素通过距离
#define TUNE_ELEM_STRAIGHT_LONG   0.0f    // 长直行元素通过距离

/**********************************************
* [Build Mode] Turn Phase1 — fixed diff rotation
**********************************************/
#define BUILD_TURN_DIFF_FIXED     125     // Phase1 原地旋转固定差速 (编码器tick/3ms)
#define ROTATE_TARGET_DEG         90.0f   // 旋转完成角度阈值

/**********************************************
* [Build Mode] Edge detect cooldown (encoder ticks)
**********************************************/
#define TUNE_COOLDOWN_NODE_TURN   150.0f  // 节点转弯后冷却距离
#define TUNE_COOLDOWN_STRAIGHT    100.0f  // 直行后冷却距离
#define TUNE_COOLDOWN_ELEM_TURN   400.0f  // 元件转弯后冷却距离

/**********************************************
* [Build Mode] Edge detect sensor threshold
**********************************************/
#define EDGE_MIN_WHITE_END       4       // 边沿检测最少末端白色传感器数
#define EDGE_MIN_WHITE_ALL       5       // 边沿检测最少总白色传感器数

/**********************************************
* [Build Mode] Turn settle (legacy, used by Is_Turn_Angle_Settled reference)
**********************************************/
#define TUNE_TURN_SETTLE_ERR     3.0f    // 稳定角度误差 (度)
#define TUNE_TURN_SETTLE_RATE    45.0f   // 稳定角速率 (度/秒)
#define TUNE_TURN_SETTLE_CYCLES  3       // 连续稳定周期数

/**********************************************
* [Build Mode] Array dimensions
**********************************************/
#define NODE_NUM_MAX             20
#define ELEMENT_NUM_MAX          5
#define TRACK_SEGMENT_NUM_MAX    (NODE_NUM_MAX + 1)
#define BUILD_ACTION_MAX         (NODE_NUM_MAX + (TRACK_SEGMENT_NUM_MAX * ELEMENT_NUM_MAX))
#define BUILD_NODE_NUM           17
#define BUILD_ACTION_COUNT       31

/**********************************************
* [Build Mode] Basic speed (encoder ticks/3ms)
**********************************************/
#define BUILD_BASIC_SPEED        45

/**********************************************
* [Remember Mode] Speed curve
**********************************************/
#define REM_SPEED_LOW_RATIO      0.05f   // 低速段比例 [0% ~ RATIO]
#define REM_SPEED_RAMP_RATIO     0.10f   // 加速段比例 [LOW ~ RAMP]
#define REM_SPEED_DECEL_RATIO    0.95f   // 减速起始比例 [0% ~ DECEL% 匀速, 之后减速]
#define REM_SPEED_MIN            160     // 回放最低速度 (编码器tick/3ms)
#define REM_SPEED_MAX            160     // 回放最高速度 (编码器tick/3ms)

/**********************************************
* [Remember Mode] Turn
**********************************************/
#define REM_TURN_KP_AT_160       52.0f   // 速度160时的转弯Kp基准
#define REM_TURN_ERR             55      // 转弯固定Error
#define REM_TURN_TARGET_DEG      90.0f   // 直角转弯目标角度
#define REM_TURN_INNER_SCALE     1.4f    // 内侧轮差速比例
#define REM_TURN_OUTER_SCALE     0.6f    // 外侧轮差速比例
#define REM_TURN_BASE_MIN        140     // 转弯基础速度下限
#define REM_TURN_BASE_MAX        180     // 转弯基础速度上限

/**********************************************
* [Remember Mode] Straight drive heading lock
**********************************************/
#define HEADING_BUF_SIZE         11      // 陀螺环形缓冲大小
#define STRAIGHT_HEADING_KP      1.0f    // 航向锁 P
#define STRAIGHT_HEADING_KD      0.2f    // 航向锁 D
#define HEADING_LOCK_MAX_ANGLE   2.0f    // 航向最大允许变化 (度)
#define HEADING_LOCK_DEADZONE    1.5f    // 航向锁死区 (度)

/**********************************************
* [Remember Mode] Straight mileage presets (encoder ticks)
**********************************************/
#define REM_STRAIGHT_SHORT       2700.0f
#define REM_STRAIGHT_MID         2800.0f
#define REM_STRAIGHT_LONG        2900.0f
#define REM_STRAIGHT_NODE        250.0f

/**********************************************
* [Safety]
**********************************************/
#define SAFE_VOLTAGE             11.3f   // 低压切断阈值 (V)
#define SAFETY_STOP_CYCLE_MAX    80      // 连续全亮/全暗 → 紧急停车
#define LOW_VOLT_FRAME_THRESH    1000    // 低压持续帧数 → 停车
#define FINISH_DEBOUNCE_TICKS    200     // Finish_Flag 保持 → 停车

/**********************************************
* [Debug / Speed FF]
**********************************************/
#define DEBUG_SPEED_FF_REF       12.0f   // 标称参考电压 (V)
#define DEBUG_SPEED_FF_V_MIN     11.1f   // 最低工作电压 (V)
#define DEBUG_SPEED_FF_V_MAX     12.6f   // 最高工作电压 (V)
#define DEBUG_SPEED_FF_COMP_MIN  0.90f   // 电压补偿最小乘数
#define DEBUG_SPEED_FF_COMP_MAX  1.12f   // 电压补偿最大乘数
#define DEBUG_VOLT_FAST_ALPHA    0.10f   // 快 EMA 系数
#define DEBUG_VOLT_SLOW_ALPHA    0.01f   // 慢 EMA 系数
#define DEBUG_VOLT_SPIKE_LIMIT   1.0f    // 尖峰过滤阈值 (V)

#define DEBUG_ANGLE_STEP_TICKS   667U    // 角度调试每步持续 (3ms周期)
#define DEBUG_ANGLE_SIN_AMP      1200.0f // 正弦模式峰值角速度 (deg/s)
#define DEBUG_DEFAULT_SPEED      40      // 调试默认目标速度
#define DEBUG_DEFAULT_FAN        500     // 调试默认风扇占空比

/**********************************************
* _VAL redirects for Ctrl_Remember.h compatibility
**********************************************/
#define HEADING_BUF_SIZE_VAL          HEADING_BUF_SIZE
#define STRAIGHT_HEADING_KP_VAL       STRAIGHT_HEADING_KP
#define STRAIGHT_HEADING_KD_VAL       STRAIGHT_HEADING_KD
#define HEADING_LOCK_MAX_ANGLE_VAL    HEADING_LOCK_MAX_ANGLE

#endif
