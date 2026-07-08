/*************************************************
Copyright (C), 2016-2026, TYUT JBD TEAM C.
File name: Hardware_Config.h
Author: Cross_Z
Version:4.0               Date: 2026.7.8
Description: Hardware geometry parameters — fill in measured values
             Used by curvature-based pure pursuit control.
**************************************************/

#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

// ===== 硬件几何参数 (mm) — 实测后填入 =====
#define TRACK_WIDTH_MM       0.0f   // TODO: 左右轮间距，测完填
#define LD_MM                0.0f   // TODO: 前瞻距离（光电管到前轴/转轴），测完填
#define SENSOR_PITCH_MM      0.0f   // TODO: 光电管两两间距，测完填

// ===== Build 转弯 Phase1 固定差速（原地旋转，编码器tick/3ms）=====
#define BUILD_TURN_DIFF_FIXED  125

// ===== 旋转退出角度 =====
#define ROTATE_TARGET_DEG      90.0f

#endif
