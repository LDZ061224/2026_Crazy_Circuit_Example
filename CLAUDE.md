# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

# 2026 Crazy Circuit 智能车竞速程序

## 硬件平台与工具链

- **芯片**: Infineon AURIX TC264D (TC26xB B-Step, Dual-core TriCore)
- **IDE**: AURIX Development Studio (Eclipse CDT, managed build)
- **编译器**: TASKING Tricore C/C++ Compiler (`ctc`)
- **调试器**: winIDEA (iSYSTEM) 通过 USB DAS 连接
- **依赖库**: SEEKFREE TC264 Open Source Library V3.4.1 + Infineon iLLD
- **启动**: 编译生成 `Debug/2026_Crazy_Circuit.elf`，通过 ADS → winIDEA 烧录至 Flash

> **注意**: `Readme.md` 中错误地写为 TC397，实际芯片为 **TC264D / TC26B**（以 `.cproject` 和 `Lcf_Tasking_Tricore_Tc.lsl` 为准）。

## 构建与烧录

- **IDE 构建**: ADS (AURIX Development Studio) → 右键项目 → Build Project
- **命令行构建**: `make -C Debug -j` (需要 TASKING ctc 工具链在 PATH 中)
- **产物**: `Debug/2026_Crazy_Circuit.elf` (调试) + `Debug/2026_Crazy_Circuit.hex` (烧录)
- **烧录**: winIDEA 通过 USB DAS 连接 → 加载 elf → 烧录至 Flash

---

## 项目目录结构

```
user/                   ← CPU 入口 + 中断服务
  cpu0_main.c           → Core 0: 硬件初始化 + 主循环(VOFA调试输出)
  cpu1_main.c           → Core 1: OLED 显示刷新循环
  isr.c                 → 所有ISR: 3ms PIT(Car_Go), EXTI, DMA, UART
  isr_config.h          → 中断优先级配置 (必须唯一!)
code/                   ← 应用层代码
  Ctrl.c / Ctrl.h       → 核心控制 (~1500行): 状态机, 寻迹, 转弯, 里程, 建图/回放
  Fun.c / Fun.h         → 外设初始化: ADC, 编码器, PWM, 电机, GPIO
  pid.c / pid.h         → PID 算法库 (位置式 + 增量式)
  Racing_Track.c        → 6张预赛/决赛赛道地图数据
  OLED/                 → OLED 显示 + CH455 键盘 + 模拟I2C + Flash UI
libraries/              ← 第三方库
  infineon_libraries/   → Infineon iLLD 底层驱动 (TC26B)
  zf_common/ / zf_driver/ / zf_device/ / zf_components/  → SEEKFREE 逐飞库
```

---

## 关键约束 (Gotchas)

1. **中断优先级必须唯一**: `isr_config.h` 中所有中断优先级 (1-255) 不能重复，否则硬件异常
2. **Boot 引脚禁止使用**: P14.2~P14.6, P10.5, P10.6 作为外设使用会导致芯片无法启动
3. **P20.2 仅输入**: 不能配置为输出；P21.6 在 TC264DA 上无法使用
4. **双核内存**: 全局变量默认分配到 CPU1 DSPRAM，CPU0 访问需注意链接脚本配置
5. **Flash 扇区 0**: 只使用页 0-9 (布局见下方 Flash 存储章节)，擦除需整扇操作
6. **DMA 中断**: EXTI / DMA 中断服务需要在 `isr.c` 中手动注册 CPU 亲和性
7. **头文件统一包含**: 所有 `.c` 文件只 include `headfiles.h`，该文件汇总了全部模块头文件。修改模块依赖时只需编辑 `headfiles.h`，无需逐个改源文件。

---

## 系统架构

```
cpu0_main.c (启动入口)
  ├── 硬件初始化: clock → Encoder → Motor → Light → IMU660RB → TCA9555 → OLED
  ├── OLED_Input()         ← 键盘菜单：模式选择 / 地图编辑 / 参数配置 / 查看数据
  ├── OLED_Data_Load()     ← 从 Flash 加载 PID/速度/控制参数到全局变量
  ├── pit_ms_init(3ms)     ← 启动 3ms 定时中断
  └── while(1): Vofa_Send_Data()  ← 每周期发送调试数据到上位机
```

**3ms 中断服务** (`user/isr.c`):
```
PIT_CH0_IRQ → Car_Go()  ← 整个控制核心的节拍
```

---

## 主控制循环：Car_Go() — Ctrl.c

```
Car_Go() [每 3ms 一次]
  ├── 启动延时检测 (EnableSwitch_ON 上升沿 → 100 周期延时)
  ├── Get_Light()           ← 读 15 路光电传感器 ADC
  ├── Get_Speed() [每 6ms]  ← 编码器读值(6ms清空) → 3-tap FIR → Left_Real_Spd/Right_Real_Spd + 里程累加
  ├── Get_IMU()             ← 陀螺仪角速度 → Gyro_Z(°/s) / Gyro_Integral(°)
  ├── [Build_Mode]  Build_Mode_Get_Error()
  │     ├── Light_Process()   ← ADC 二值化 → Track_Arr[] 寻迹数组
  │     ├── switch(Run_Mode):
  │     │     Normal_Mode   → Normal_Run()
  │     │     Turn_Left     → Turn_Left_Run()
  │     │     Turn_Right    → Turn_Right_Run()
  │     │     Mileage_Mode  → Mileage_Mode_Run()
  │     │     Straight_Mode → Straight_Run()
  │     └── Set_Speed()      ← PID 级联 → 左右轮期望速度 + 电机 PID
  ├── [Remember_Mode] Remember_Mode_Get_Error()  ← 回放专用逻辑
  └── Set_Out()              ← PWM 输出到电机 H 桥
```

---

## 工作模式 (Mode)

| 模式 | 枚举 | 说明 |
|------|------|------|
| `Build_Mode` | 0 | 建图：光电循迹，记录转弯/边缘里程到 Flash |
| `Remember_Mode` | 1 | 回放：从 Flash 加载里程数据，里程+边缘双触发 |

键盘选择：`OLED_Input()` 中 1=建图, 2=回放, 8=查看数据

---

## 运行模式状态机 (Run_Mode)

```
                   ┌──────────────────────────────────────┐
                   │            Normal_Mode                │
                   │  (传感器循迹, Check_Edge 触发路由)     │
                   └────┬──────┬──────┬──────┬────────────┘
                        │      │      │      │
          Check_Edge 后判断 mileage_dir:
          ┌─────────────┼──────┼──────┼──────┐
          │ dir=1/2     │dir=3/4│dir=0 │全部完成│
          ▼             ▼      ▼      ▼
   Mileage_Mode   Mileage_Mode  In_Line_Ele++  Set_Node_Run_Mode
   (转弯元件)     (直行元件)     若全部完成 →        │
       │              │         Set_Node_Run    ┌─node_dir=1/2: Turn_Left/Right
       │              │              │          └─node_dir=0:   Straight_Mode
       ▼              ▼              │                │
   Finish_Mileage  Finish_Mileage    │                ▼
       │              │              │         转弯完成 / 直道稳定
       └──────────────┴──────────────┘                │
              返回 Normal_Mode  ◄─────────────────────┘
```

---

## 赛道数据结构

```
赛道 = 电路原理图
  元器件 = 电路元件 (两节点之间的路段内)
  节点   = 电路交点 (决定方向：左转90°/右转90°/直行)

Racing_track_Typedef (Ctrl.h):
  Node_Arr_Dir[N]              ← 节点方向: 0=直行, 1=左转, 2=右转, 4=直行(不转弯)
  Node_Arr_Mileage_Num[N]      ← 第 N 段有多少个元器件（0表示无元器件，直接走节点方向）
  Node_Arr_Mileage_Dir[N][E]   ← 元器件方向: 0=普通, 1=左转, 2=右转, 3=短直, 4=长直
  Node_Arr_Mileage_Normal[N][E]← 普通路段里程
  Node_Arr_Mileage_Element[N][E]← 元器件里程
  Node_Num                     ← 有效节点数
  Stop_Mode                    ← 0=串行赛道, 1=并行赛道
```

关键索引变量：
- `Execute_Times` — 当前在第几个节点之间 (0 ~ Node_Num)
- `In_Line_Ele_Count` — 当前段内第几个元器件 (0 ~ Mileage_Num-1)

---

## 键显输入的"Default"默认地图 (OLEDKeyboard.c)

```
Node_Num = 11, Stop_Mode = ? (取决于OLED_Apply_Build_Mode_To_RunTrack)

Node_Dir  = {1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1}
            (左,直,左,直,左,直,左,左,左,直,左)

Mileage_Num = {0, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0}
              (段0无元素, 段1有1元素, 段2有2元素...)

Mileage_Dir:
  [0]={0}        ← 段0（无元素，直接走节点0方向=左转）
  [1]={4}        ← 段1有1个长直行元素(里程1800)
  [2]={3, 3}     ← 段2有2个短直行元素(各1400)
  [3]={0}        ← 段3有1个普通元素(里程走完触发节点3=左转)
  [4]={4}        ← 段4有1个长直行元素
  [5]={0}        ← 段5有1个普通元素(节点5=直行)
  [6]={3}        ← 段6有1个短直行元素
  [7]={4}        ← 段7有1个长直行元素
  [8]={1}        ← 段8有1个左转元器件
  [9]={3}        ← 段9有1个短直行元素
  [10]={0}       ← 段10有1个普通元素(节点10=左转)
  [11]={0}       ← 段11无元素(完赛)
```

---

## 传感器处理链

### Get_Light() → Light_Process()
```
15 路光电 ADC → 二值化(Light_Convert[15]) → 边界检测 → Track_Arr[] 有效传感器索引
  ↓
Initial_White_Num (实际白点数) / Track_Num (连续有效数)
  ↓
Left_Scan_Point = Track_Arr[0]        (最左白点)
Right_Scan_Point = Track_Arr[最后]    (最右白点)
Middle = (Left + Right) / 2
Error = (Dir_Arr[Left] + Dir_Arr[Right]) / 2   ← 偏差值, 输入 PID 级联
```

`Dir_Arr[15] = {-22, -21, -20, -18, -14, -9, -2, 0, 2, 9, 14, 18, 20, 21, 22}`

### Normal_Run() 三级判断
```
Track_Num < 2:                      Error=0 (丢线直行)
Track_Num 2~4:                      Error=当前帧首尾平均
Initial_White_Num >= 5:             Error沿用边界点 + Check_Edge → 路由决策
  断点修复后Track_Num高但白点<5:    Error沿用边界点（不做Check_Edge）
```

### Check_Edge() — Ctrl.c
触发条件：最左传感器白 || 最右传感器白 || 白点 ≥ 5
被 `Check_Edge_Skip_Count > 0` 抑制（转弯/里程切换后防误触发）

---

## PID 级联控制链 — Set_Speed()

```
Error(传感器偏差) ──→ Turn_PID(位置式, kp=80) ──→ Turn_PID_Out
                                                     │
Gyro_Z_For_PID(raw/1000) ──────────────────┬────────┤
                                            ↓
                             Gyro_PID(位置式, kp=0.008) ──→ Gyro_PID_Out
                                            │
                  ┌─────────────────────────┘
                  ↓
Left_Exp_Spd  = Run_Speed - Gyro_PID_Out
Right_Exp_Spd = Run_Speed + Gyro_PID_Out
                  │
  ┌───────────────┴───────────────┐
  ↓                               ↓
Left_PID(增量, kp=ki=kd=0)     Right_PID(增量, kp=250, ki=65)
  ↓                               ↓
Left_PID_Out (=0 恒为零)        Right_PID_Out → PWM
```

### 速度覆盖优先级 (Set_Speed 内部)
1. `Stop_Flag` → 期望速度 = 0, return
2. 转弯 (`is_left/is_right=1`) → Build 三阶段 / Remember PID差速

### PWM 输出 — Set_Out()
```
PID_Out == 0:  IN1=0, IN2=0 (滑行)
PID_Out > 0:   IN1=0, IN2=(PID_Out+10000)/2 (正向)
PID_Out < 0:   IN1=(|PID_Out|+10000)/2, IN2=0 (反向)
```

---

## Build 模式转弯三阶段

### 节点转弯 (Turn_Left_Run / Turn_Right_Run)
```
Turn_Decel_Phase=0: 直行 TURN_STRAIGHT_PRE_DISTANCE(100) 里程, Error=0 → Phase=1
Turn_Decel_Phase=1: Error=±40, 期望速度=0, 减速到轮速≤5 → Phase=2
Turn_Decel_Phase=2: Error=±40, 两轮反向±Turn_Error_Value, Gyro_Integral≥70° → Complete
陀螺仪积分从Set_Node_Run_Mode清零开始连续累积，中间不断开
```

---

## 里程与 Flash 存储

### 编码器测速 (Get_Speed, 每6ms)
```
encoder_get_count/3 → left_raw/right_raw → encoder_clear_count (6ms清空一次)
  → 滑入环形缓冲 giSpeed_{0,1,2}
  → 0.5×[0]+0.3×[1]+0.2×[2] → Left_Real_Spd/Right_Real_Spd (FIR滤波值，用于速度环PID)
  → (left_raw+right_raw)/2 → Count.Mileage + Total_Run_Mileage (里程累加)
```

### Flash 布局 (扇区 0)
| 页 | 内容 | 读写函数 |
|----|------|---------|
| 0 | Basic_Speed | flash_read/write_page |
| 1 | PID_OKb[8] | flash_read/write_page |
| 2 | DBG_OKb[3] | flash_read/write_page |
| 3 | Ctrl_OKb[5] | flash_read/write_page |
| 4 | 键盘输入的地图数据 | OLED Save/Load |
| 5-7 | Turn_Mileage_Record (转弯间距里程) | Save/Load_Turn_Mileage_Record_To_Flash |
| 8-9 | Segment_Edge_Mileage_Record (边缘里程) | Save/Load_Segment_Edge_Mileage_Record_To_Flash |

### 里程变量的含义
| 变量 | 单位 | 含义 |
|------|------|------|
| `Count.Mileage` | 段内里程 | 进入当前节点/段后累计，Set_Node_Run_Mode清零 |
| `Total_Run_Mileage` | 总里程 | 发车以来累计，永不清零 |
| `Turn_Mileage_Record[i]` | 总里程差 | 建图时两次转弯之间直道距离 = Turn_Begin_Mileage - Last_Turn_Mileage_Base |
| `Segment_Edge_Mileage_Record[node][ele]` | 段内里程 | 建图时该元素Check_Edge触发瞬间的 Count.Mileage |

### 存储时机
- **建图运行中**：Record_Turn_Mileage() / Record_Segment_Edge_Mileage() → 写 RAM + 立即写 Flash
- **启动时**：Build_Mode_Get_Error First_Mode==0 → 写空数据到 Flash

---

## Remember 模式 (回放) 完整执行流程

### 初始化 (首次进入 Remember_Mode_Get_Error)

```
Remember_First_Mode == 0:
  Load_Turn_Mileage_Record_From_Flash()       ← Flash页5-7 → Turn_Mileage_Record_Num + Turn_Mileage_Record[]
  Load_Segment_Edge_Mileage_Record_From_Flash() ← Flash页8-9 → Segment_Edge_Mileage_Record[][]
  Remember_Reset_Runtime_State()              ← 所有索引/计数/标志清零
    Remember_Turn_Record_Index = 0
    Remember_Next_Target_Mileage = Turn_Mileage_Record[0] (第一条转弯间距)
    Remember_Section_Base_Mileage = 0
    Execute_Times = 0, In_Line_Ele_Count = 0
    Run_Mode = Normal_Mode
    Total_Run_Mileage = 0, Count.Mileage = 0
```

### 每周期调度

```
Remember_Mode_Get_Error()
  ├── Light_Process()
  ├── if Run_Mode == Normal_Mode:
  │     ├── Remember_Normal_Run()      ← 只算Error，不调Check_Edge
  │     └── Remember_Check_Trigger()   ← 边缘+里程双触发 (内部调Check_Edge)
  ├── switch(Run_Mode):               ← 其他模式与Build共用
  │     Turn_Left/Right  → Turn_∗_Run()  (Remember模式：固定Error±40，边走边转)
  │     Mileage_Mode     → Mileage_Mode_Run()
  │     Straight_Mode    → Straight_Run()
  └── Set_Speed()                      ← Run_Speed = Remember_Get_Run_Speed() 梯形曲线
```

### Remember_Check_Trigger 核心逻辑

**前置条件**：`Turn_Mileage_Record_Num == 0` → 直接返回（无建图数据，纯循迹）

**情况A：路段内还有元素** (`In_Line_Ele_Count < Mileage_Num[Execute_Times]`)

| mileage_dir | Node_Dir | 触发方式 |
|------|------|------|
| 0 | 0 (纯直道) | 仅 `Check_Edge` 物理边缘触发 → Set_Node_Run_Mode(直行) |
| 0 | ≠0 (直行元素→转向节点) | `Total_Run_Mileage >= Remember_Next_Target_Mileage - 180` → 里程触发转向 |
| ≠0 (1/2/3/4) | 任意 | ① `edge_hit` 物理触发优先 → Snap里程校准 → Mileage_Mode；② `Count.Mileage >= Segment_Edge_Mileage[Exe][In_Line] - 120` → 里程提前进入 Mileage_Mode |

**情况B：路段元素已完成** (`In_Line_Ele_Count >= Mileage_Num[Execute_Times]`)

| Node_Dir | 触发方式 |
|------|------|
| 0 (直行节点) | 仅 `Check_Edge` 物理边缘触发 |
| ≠0 (转向节点) | `Total_Run_Mileage >= Remember_Next_Target_Mileage - 180` → 里程触发 Set_Node_Run_Mode |

### 转弯完成后推进 (Complete_Turn_Action in Remember mode)

```
Complete_Turn_Action:
  Record_Turn_Mileage()         ← Remember模式直接return，不记录
  Remember_Advance_Turn_Record():
    Remember_Turn_Record_Index++
    Remember_Next_Target_Mileage += Turn_Mileage_Record[Index]  ← 累加下一条
    Remember_Section_Base_Mileage = Total_Run_Mileage
  Advance_To_Next_Track_Segment():
    Execute_Times++, In_Line_Ele_Count = 0
  Reset_Turn_Action_State()     ← Run_Mode = Normal_Mode
```

### Remember 模式关键行为总结

- **段内元素**: `Remember_Check_Trigger` 提前触发（物理边缘 Snap 优先 + 里程预判 `Remember_Mileage_Prepare_Distance`(120) 兜底）
- **节点转弯**: 里程触发（距目标 `Remember_Node_Prepare_Distance`(180) 时进入转向）
- **转弯完成**: `Remember_Advance_Turn_Record` 累加下条记录，`Advance_To_Next_Track_Segment` 推进节点
- 完整逐段推演见 `Readme.md`"默认地图 Check-Edge 逐次路由表"

### Remember 模式速度曲线 (Remember_Get_Run_Speed)

```
路段总里程 = Remember_Next_Target_Mileage - Remember_Section_Base_Mileage
有效总里程 = 路段总里程 - 准备距离

[0% ~ 5%]:  最低速度 Remember_Speed_Min_Value(40)
[5% ~ 10%]: 线性加速 40→55
[10% ~ 95%]: 最高速度 Remember_Speed_Max_Value(55)
[95% ~ 100%]: 线性减速 55→40
```

### Remember 模式关键变量汇总

| 变量 | 初值 | 含义 |
|------|------|------|
| `Turn_Mileage_Record_Num` | Flash加载 | 有效转弯间距记录条数（=0则无建图数据） |
| `Turn_Mileage_Record[i]` | Flash加载 | 第i段直道的里程（两转弯之间） |
| `Remember_Turn_Record_Index` | 0 | 当前处理到第几条记录 |
| `Remember_Next_Target_Mileage` | Record[0] | 累计目标里程（每次Advance累加下一条） |
| `Remember_Section_Base_Mileage` | 0 | 本段起点 Total_Run_Mileage |
| `Segment_Edge_Mileage_Record[node][ele]` | Flash加载 | 建图时边缘触发的段内里程坐标 |
| `Remember_Mileage_Prepare_Distance` | 120 | 元素提前进入 Mileage_Mode 的距离 |
| `Remember_Node_Prepare_Distance` | 180 | 节点提前进入转向的距离 |

---

## 停车流程

```
Finish_Flag = 1  (Advance_To_Next_Track_Segment 或 Finish_Mileage_Section)
  ↓
Finish_Count++ (每 3ms, Light_Process 中)
  ↓
Finish_Count > 200 (600ms 延迟)
  ↓
Stop_Flag = 1
  ↓
Set_Speed: 期望速度=0
Set_Out: 四路 PWM=0
```

紧急停车：全白/全黑 > 100 周期 → Stop_Flag=1；堵转 > 20 周期 → Stop_Flag=1
转弯时(is_left||is_right)两种停车检测均抑制

---

## 关键宏定义速查

| 宏 | 值 | 说明 |
|----|-----|------|
| `TURN_TARGET_ANGLE_DEG` | 70.0f | 转弯目标角度 |
| `TURN_STRAIGHT_PRE_DISTANCE` | 100.0f | Build转弯前直行距离（可调参） |
| `TURN_INNER_WHEEL_SCALE` | 1.0f | 转向内侧轮速度系数 |
| `TURN_OUTER_WHEEL_SCALE` | 1.0f | 转向外侧轮速度系数 |
| `Turn_Error_Value` | 40 | 转弯 Error 固定值 |
| `GYRO_INTEGRATION_PERIOD_S` | 0.003f | 陀螺仪积分周期(3ms) |
| `Remember_Speed_Min_Value` | 40 | 回放模式最低速度 |
| `Remember_Speed_Max_Value` | 55 | 回放模式最高速度 |

## 函数调用关系速查

```
Car_Go
├── Get_Light → [读 ADC]
├── Get_Speed [6ms]
│   ├── encoder_get_count → left_raw/right_raw
│   ├── encoder_clear_count (6ms清空一次)
│   ├── FIR: giSpeed_Left/Right[3] → Left_Real_Spd/Right_Real_Spd
│   └── Count.Mileage += instant_speed
├── Get_IMU
│   ├── imu660rb_gyro_transition → Gyro_Z (°/s)
│   ├── 漂移滤波: |Gyro_Z|<2.0 → 清零
│   ├── Gyro_Integral += Gyro_Z × 0.003s (转弯时)
│   └── Gyro_Z_For_PID = raw/1000 (PID用，死区<30raw)
├── Build_Mode_Get_Error / Remember_Mode_Get_Error
│   ├── Light_Process → Track_Arr / 停车检测
│   ├── [Run_Mode dispatch]
│   │   ├── Normal_Run / Remember_Normal_Run
│   │   │   └── Remember_Check_Trigger (回放模式)
│   │   ├── Turn_Left_Run / Turn_Right_Run
│   │   ├── Mileage_Mode_Run → Mileage_Run_Stage_2
│   │   └── Straight_Run
│   └── Set_Speed
│       ├── PID_calc(&Turn_PID, 位置式) → Turn_PID_Out
│       ├── PID_calc(&Gyro_PID, 位置式) → Gyro_PID_Out
│       ├── Left_Exp_Spd / Right_Exp_Spd 计算 + 转弯覆盖
│       ├── PID_calc(&Left_PID) → Left_PID_Out (=0)
│       └── PID_calc(&Right_PID) → Right_PID_Out
└── Set_Out → pwm_set_duty (H 桥控制)
```

---

## 补充文档

`Readme.md` — 静态分析报告，含已修复 Bug 记录、已知局限列表、默认地图 Check-Edge 逐次路由表（19 步完整推演）、修改历史。
