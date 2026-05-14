# 2026 Crazy Circuit — 智能车竞速程序 静态分析报告

## 项目概述

本项目为 TYUT JBD TEAM C 智能车竞速程序，基于 Infineon AURIX TC397 平台开发，采用 ADS (AURIX Development Studio) 编译环境。实现功能包括：光电传感器循迹、编码器里程计、陀螺仪姿态控制、PID 闭环速度控制、赛道建图与记忆回放。

## 代码结构

```
code/
├── headfiles.h          # 通用宏定义（限幅、绝对值等）+ 所有头文件统一包含
├── pid.h / pid.c        # PID 算法库（位置式 + 增量式）
├── Fun.h / Fun.c        # 外设驱动（ADC、编码器、PWM、OLED、GPIO 初始化）
├── Ctrl.h / Ctrl.c      # 核心控制逻辑（寻迹、转向、里程、建图/回放模式）
├── TCA9555.h / TCA9555.c# LED 扩展芯片 I2C 驱动
├── Racing_Track.c       # 赛道地图数据定义（预赛/决赛共6张地图）
├── OLED/                # OLED 显示 + 键盘输入子模块
│   ├── OLEDKeyboard.h/c # 赛道编辑、参数配置、Flash 存储交互
│   ├── dev_CH455.h/c    # CH455 键盘/数码管芯片驱动
│   ├── dev_ssd1306.h/c  # SSD1306 OLED 显示屏驱动
│   ├── JBD_simiic.h/c   # 模拟 I2C 通信库
│   ├── FlashFun.h/c     # Flash 数据恢复工具
│   ├── UI.h/c           # 键盘输入 UI 界面
│   └── Font.h/c         # 显示字体库
```

### 核心控制流程

```
Car_Go() [主循环]
  ├── Get_Light()          # 读取15路光电传感器ADC
  ├── Get_Speed()          # 读取编码器速度 + 里程累计
  ├── Get_IMU()            # 读取陀螺仪角速度 + 积分角度
  ├── Build_Mode_Get_Error() 或 Remember_Mode_Get_Error()
  │     ├── Light_Process()   # ADC->二值化->寻迹数组
  │     ├── Normal_Run()      # 正常循迹 + 边缘检测触发
  │     ├── Turn_Left_Run()   # 左转状态机
  │     ├── Turn_Right_Run()  # 右转状态机
  │     ├── Mileage_Mode_Run()# 里程计模式
  │     ├── Straight_Run()    # 直道模式
  │     └── Set_Speed()       # PID 计算速度输出
  └── Set_Out()            # PWM 电机驱动输出
```

### 运行模式

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| Normal_Mode | 0 | 常规光电循迹，检测边缘触发转向/里程 |
| Turn_Left | 1 | 左转状态机，陀螺仪累计80度完成 |
| Turn_Right | 2 | 右转状态机，陀螺仪累计80度完成 |
| Mileage_Mode | 3 | 里程计控制模式，按预设里程行驶 |
| Straight_Mode | 4 | 直道稳定模式，传感器居中后切回正常 |

### 工作模式

- **Build_Mode（建图模式）**：首次运行，记录赛道转向里程数据到 Flash
- **Remember_Mode（回放模式）**：从 Flash 加载已记录的里程数据，复现运行

---

## 静态模拟仿真测试 - 漏洞分析

### 🔴 严重 Bug（已修复）

#### 1. Load_Turn_Mileage_Record_From_Flash 缺少 memcpy

- **位置**：Ctrl.c:438-456
- **问题**：从 Flash 读取 map_words 后，没有执行 memcpy(&flash_log, map_words, sizeof(flash_log)) 将数据拷贝到结构体。导致 flash_log 始终保持初始化时的全零值。
- **影响**：**回放模式 (Remember_Mode) 完全失效**。每次从 Flash 加载的转向里程记录始终为零，Turn_Mileage_Record_Num 始终为 0，小车无法按记录数据行驶。
- **对比**：Load_Segment_Edge_Mileage_Record_From_Flash 中正确包含了 memcpy 调用。
- **修复**：已在第449行插入缺失的 memcpy 调用。

```c
// 修复前（flash_log 始终为零）
Load_Flash_Page_Block(..., map_words);
if (flash_log.Turn_Mileage_Record_Num > TURN_MILEAGE_RECORD_MAX) // BUG: flash_log 全是0!

// 修复后
Load_Flash_Page_Block(..., map_words);
memcpy(&flash_log, map_words, sizeof(flash_log));  // <-- 添加此行
if (flash_log.Turn_Mileage_Record_Num > TURN_MILEAGE_RECORD_MAX)
```

### 🟡 中危问题

#### 2. ~~Pre_Contest_2 方向数组元素不足~~（经确认非 Bug，已撤回）

- **位置**：Racing_Track.c:56
- **初始判断**：Node_Num=16，Node_Arr_Dir 仅 15 个元素。
- **用户确认**：`Pre_Contest_2` 到 `Final_Contest_3` 均未被任何代码引用，运行时始终以 `Pre_Contest_1` 为模板，再通过键盘输入覆盖。缺元素不影响任何执行路径。此问题不成立。

#### 3. Light_Process 异常滤波逻辑缺陷

- **位置**：Ctrl.c:952-958
- **问题**：只要检测到任意一对相邻传感器索引差 > 1，就用上一周期数据完全替换当前数据。一次传感器毛刺就会丢弃整帧有效数据。
- **影响**：在高干扰环境下（如强光、反光），可能导致寻迹数据频繁回退，影响稳定性。
- **建议**：改为滑动窗口滤波或中值滤波，仅丢弃异常传感器而非整帧。

#### 4. ~~里程累计假设固定时间步长~~（经确认非 Bug，已撤回）

- **位置**：Ctrl.c:842-843
- **初始判断**：认为里程直接累加速度值未乘时间间隔，存在累积误差。
- **用户确认**：`Get_Speed()` 由 3ms 硬件定时器中断触发，调用间隔是硬件保证的固定值。编码器读数本身就是"3ms 内位移量"，直接累加即为正确里程，无需乘时间。此问题不成立。

#### 5. CH455_GetOneKey 阻塞式超时过长

- **位置**：dev_CH455.c:49
- **问题**：timeout = 15000 次 I2C 轮询，实际等待约1.5秒。
- **影响**：在按键卡住时，程序会在该函数阻塞约1.5秒，影响实时性。

### 🟢 建议改进

#### 6. 左电机 PID 参数全为零

- **位置**：Ctrl.h:40-47
- **现象**：LEFT_PID 的 kp=0, ki=0, kd=0，输出始终为0。左电机实际依赖 Right_PID 间接控制。
- **说明**：可能是硬件特性不同的有意配置，但建议添加注释说明原因。

#### 7. 电压检测无保护动作

- **位置**：Ctrl.c:1436-1439
- **问题**：Voltage_Check[0] < 11.5 时仅点亮指示灯，未切断电机输出。
- **建议**：电压低于阈值时应进入低功耗/停车状态，防止电池过放或电机失控。

#### 8. Turn_Left_Run 中右侧传感器计数逻辑冗余

- **位置**：Ctrl.c:1164
- **问题**：Track_Arr[i] >= 7 && Track_Arr[i] != 0 中 >= 7 已排除 0，!= 0 是冗余条件。
- **说明**：这是防御性编程（过滤未初始化元素），功能上无影响。

#### 9. KEY_1_ 命名不一致

- **位置**：dev_CH455.h:22-24
- **现象**：KEY_1_ 到 KEY_4_ 带下划线后缀，但 KEY_5 到 KEY_9 不带。推测因宏名冲突加后缀，风格不统一。

#### 10. 多张赛道数据重复

- **位置**：Racing_Track.c
- **现象**：Pre_Contest_3、Final_Contest_1/2/3 四张地图数据几乎完全相同（仅里程值个别差异）。疑似未完成配置的占位数据。

#### 11. Normal_Run/Remember_Normal_Run 中 Left_Scan_Point/Right_Scan_Point 未初始化路径（新增）

- **位置**：Ctrl.c Normal_Run() Track_Num >= 6 分支、Remember_Normal_Run() Track_Num >= 7 分支
- **问题**：如果首周期 Track_Num 直接 >= 6，Left_Scan_Point 和 Right_Scan_Point 使用初始值 0 计算 Error，导致 `(Dir_Arr[0]+Dir_Arr[0])/2 = -22`，小车会猛向左偏。
- **影响**：正常发车时 Track_Num 从 0 逐渐增加，此路径极少触发，但拔车/飞坡后可能复现。
- **建议**：在 Track_Num >= 6 分支中也用 Track_Arr 更新边界点。

#### 12. Light_Process 异常滤波缺少 break（新增）

- **位置**：Ctrl.c Light_Process() 异常滤波循环
- **问题**：检测到传感器间距异常后回退整帧数据，但循环没有 break。如果回退后的 Last_Track_Arr 中又存在不连续，会再次触发回退，造成不可控行为。
- **建议**：添加 break 或改用单次判定逻辑。

#### 13. Straight_Num_Count 只写不读（新增）

- **位置**：Ctrl.c Advance_Turn_Section_Index()
- **现象**：Straight_Num_Count 在 Advance_Turn_Section_Index 中递增，在 Remember_Reset_Runtime_State 中清零，但从不参与任何条件判断或输出。疑似调试遗留代码。

#### 14. Speed_Get_Count 2分频机制（新增）

- **位置**：Ctrl.c Car_Go()
- **现象**：Speed_Get_Count *= -1 实现 Get_Speed() 每 2 次 Car_Go 调用一次（6ms周期），而 Get_IMU/Light_Process/Set_Out 每 3ms 一次。速度采样与陀螺仪积分频率不对等。
- **说明**：这是有意设计——速度变化比角速度慢，6ms 采样足够。但里程计算和速度PID输出有 3ms 的额外延迟。

---

## 本次修改记录

### 1. Bug 修复

| 文件 | 行号 | 修改内容 |
|------|------|----------|
| Ctrl.c | 449 | 在 Load_Turn_Mileage_Record_From_Flash 中插入缺失的 memcpy |

### 1.5 编码器读取优化

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | Car_Go: 编码器改为每3ms读取一次原始值（无滤波），里程每3ms累加（精度翻倍） |
| Ctrl.c | Get_Speed: 改为使用6ms累加器值，保留加权滤波，Left_Real_Spd/Right_Real_Spd 不变 |
| Ctrl.c | 新增 Encoder_Left_Accum/Encoder_Right_Accum 静态累加器 |

### 1.6 速度/里程去滤波

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | Get_Speed: 移除加权滤波（0.5/0.3/0.2），Left_Real_Spd/Right_Real_Spd 直接用6ms累加值 |
| Ctrl.c | giSpeed_Left[3]/giSpeed_Right[3] 环形缓冲区保留但不再使用 |
| Ctrl.c | Mileage: 每3ms直接用原始编码器值累加（不做任何滤波） |

### 1.7 陀螺仪单位修正

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | Gyro_Z 改用 imu660rb_gyro_transition() → 真实 °/s（IMU660RB: raw/14.3=°/s） |
| Ctrl.c | 漂移滤波改为 |Gyro_Z| < 2.0f（真实 °/s），替换原来的 raw<30 |
| Ctrl.c | Gyro_Integral 公式不变（已经是 °/s × 0.003s = °） |
| Ctrl.c | TURN_BASE_SPD_ZERO_ANGLE: 1000→10（°），TURN_BASE_SPD_END_ANGLE: 3000→70（°） |
| Ctrl.c | 新增 GYRO_Z_PID_SCALE=70，Gyro_Z/70 后输入 PID，保持原有 PID 量级 |
| Ctrl.c | Soft_Start_Flag/Soft_Start_Count/Straight_Num_Count 删除，Segment_Total_Mileage_Record 删除 |

### 1.8 序号推进修复

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | Advance_To_Next_Track_Segment 新增 In_Line_Ele_Count=0（切节点归零） |
| Ctrl.c | Complete_Turn_Action 建图模式出弯 → In_Line_Ele_Count=0 |
| Ctrl.c | Complete_Turn_Action 回放模式 → Advance_To_Next_Track_Segment（已含归零） |

### 1.9 里程记录与回放优化（2026-05-01）

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | 新增 MILEAGE_COMPENSATION_X 宏（-100.0f），建图记录转弯元素里程时减去此值 |
| Ctrl.c | 新增 MILEAGE_STRAIGHT_SHORT（1000）/ MILEAGE_STRAIGHT_LONG（1400）宏 |
| Ctrl.c | Record_Segment_Edge_Mileage：转弯元素(mileage_dir=1/2)记录时加补偿x |
| Ctrl.c | Mileage_Mode_Run：Snap 里程校准仅对直行元素(3/4)生效，转弯元素不做 Snap |
| Ctrl.c | Mileage_Mode_Run：硬编码 1000/1400 替换为 MILEAGE_STRAIGHT_SHORT/LONG 宏 |

### 2.0 建图/回放模式节点推进与路段计数修复（2026-05-01）

#### 背景

用户详细描述了赛道模型：
- 赛道 = 电路原理图，元器件 = 电路元件，节点 = 电路交点
- 两个节点之间 = 一段路，段内可能有元器件也可能没有
- 节点行为（3种）：左转90°、右转90°、直走
- 元器件行为（4种）：左转90°、右转90°、直走短里程、直走长里程
- 建图模式：记录每个元器件和节点的里程（转弯元器件里程补偿 x=-100），完成后存入 Flash
- 回放模式：用记录的里程提前触发转弯，直行元器件检测边缘时刷新里程，动态速度规划（弯前减速）

#### 发现的逻辑漏洞

**Bug A: Normal_Run 中 dir=0 路段错误触发节点动作**

| 项目 | 内容 |
|------|------|
| 位置 | Ctrl.c `Normal_Run()` Check_Edge 分支 |
| 问题 | 当 `mileage_dir == 0`（普通路段，无元器件）时，代码直接调用 `Set_Node_Run_Mode(node_dir)` 触发节点转弯。导致每个物理边缘都触发一次节点动作，但节点序号从未推进，同一节点的转弯被反复执行。 |
| 影响 | **建图模式无法正常遍历赛道**。车在第一个节点反复转弯，无法推进到后续节点。 |
| 修复 | `mileage_dir == 0` 时仅记录边缘里程 + `In_Line_Ele_Count++`。只有当 `In_Line_Ele_Count >= mileage_num`（所有路段走完）时才触发节点动作。 |

**Bug A2: Complete_Turn_Action Build 分支粗暴清零 In_Line_Ele_Count**

| 项目 | 内容 |
|------|------|
| 位置 | Ctrl.c `Complete_Turn_Action()` Build 分支 |
| 问题 | `In_Line_Ele_Count = 0` 不区分转弯来源：节点转弯应归零（已进入新节点），元器件转弯不应归零（应继续计数剩余元器件）。当前 Build 模式下节点在转弯前推进（Normal_Run→Advance_To_Next_Track_Segment），Remember 模式在转弯后推进（Complete_Turn_Action→Advance_To_Next_Track_Segment），两模式不一致。 |
| 影响 | 若未来有非 Normal_Run 入口触发 Set_Node_Run_Mode 且未预先调用 Advance_To_Next_Track_Segment，In_Line_Ele_Count 会被错误清零，丢失当前节点剩余元器件的计数。 |
| 修复 | Build 分支也改为调用 `Advance_To_Next_Track_Segment()`，与 Remember 模式统一：节点推进统一发生在转弯完成后。同时移除 Normal_Run 中转弯前的 `Advance_To_Next_Track_Segment` 调用。 |

**Bug B: Finish_Mileage_Section 过早清零 In_Line_Ele_Count**

| 项目 | 内容 |
|------|------|
| 位置 | Ctrl.c `Finish_Mileage_Section()` |
| 问题 | 当 `In_Line_Ele_Count` 达到 `mileage_num` 时立即清零，导致下一边缘检测时 `In_Line_Ele_Count(0) < mileage_num` 为真，重新走入"还有元素"分支，同节点元素被反复处理。 |
| 影响 | 节点永远无法推进（`else` 分支 `In_Line_Ele_Count >= mileage_num` 不可达）。 |
| 修复 | 不在 `Finish_Mileage_Section` 中清零 `In_Line_Ele_Count`。让其保持 `== mileage_num`，下一边缘自然走入 `else` 分支触发节点。清零由 `Advance_To_Next_Track_Segment` 负责（仅在进入新节点时）。 |

**Bug C: Remember_Check_Trigger 中 dir=0 路段未跟踪元素计数**

| 项目 | 内容 |
|------|------|
| 位置 | Ctrl.c `Remember_Check_Trigger()` |
| 问题 | 回放模式下 `mileage_dir == 0` 时，代码直接触发节点动作（里程触发），但 `In_Line_Ele_Count` 保持为 0。速度曲线无法正确区分"路段内"和"接近节点"阶段。 |
| 影响 | 速度曲线始终使用 `Mileage_Prepare_Distance`（120），无法使用更大的 `Node_Prepare_Distance`（180），弯前减速距离不足。 |
| 修复 | 触发节点动作前设置 `In_Line_Ele_Count = mileage_num`，使后续速度曲线正确使用 `Node_Prepare_Distance`。 |

**Bug D: 速度曲线中 dir=0 路段使用错误的准备距离**

| 项目 | 内容 |
|------|------|
| 位置 | Ctrl.c `Remember_Get_Run_Speed()` |
| 问题 | 速度曲线仅根据 `In_Line_Ele_Count < mileage_num` 判断处于路段内，统一使用 `Mileage_Prepare_Distance`。当所有路段都是 dir=0（无元器件）时，实际是直通节点的，应使用更大的 `Node_Prepare_Distance`（180 > 120）来提供更长的减速距离。 |
| 影响 | 弯前减速距离偏短（120 vs 180），可能导致入弯速度偏高。 |
| 修复 | 增加检查：若下一个待处理元素的 `mileage_dir == 0`（普通路段），使用 `Node_Prepare_Distance`；若 `mileage_dir != 0`（元器件），使用 `Mileage_Prepare_Distance`。 |

**Bug E: Straight_Node_Pending 未在 Finish_Mileage_Section 中清除**

| 项目 | 内容 |
|------|------|
| 位置 | Ctrl.c `Finish_Mileage_Section()` |
| 问题 | 当 `Straight_Node_Pending != 0 && mileage_num == 0` 时函数直接 return，未清除 `Straight_Node_Pending`。虽然后续 `Straight_Run` 中会清除，但防御性不足。 |
| 修复 | return 前添加 `Straight_Node_Pending = 0`。 |

#### 修改文件清单

| 文件 | 函数/位置 | 修改内容 |
|------|----------|----------|
| Ctrl.c | `Normal_Run()` | dir=0 路段：记录边缘里程 + `In_Line_Ele_Count++`，计数达到 `mileage_num` 时触发节点（不在此处推进节点） |
| Ctrl.c | `Complete_Turn_Action()` | Build 分支：`In_Line_Ele_Count = 0` 改为 `Advance_To_Next_Track_Segment()`，与 Remember 模式统一 |
| Ctrl.c | `Finish_Mileage_Section()` | 移除 `In_Line_Ele_Count = 0` 清零；添加 `Straight_Node_Pending = 0` 清除 |
| Ctrl.c | `Remember_Check_Trigger()` | dir=0 且触发节点时：设置 `In_Line_Ele_Count = mileage_num` |
| Ctrl.c | `Remember_Get_Run_Speed()` | 检查 `mileage_dir == 0` 时使用 `Node_Prepare_Distance`，否则使用 `Mileage_Prepare_Distance` |

#### 已知局限

1. **回放模式下混合路段（dir=0 与 dir≠0 混合）的 In_Line_Ele_Count 跟踪不完整**：当前回放模式对 dir=0 路段采用里程触发而非边缘触发，元素计数被跳过。当赛道同时包含普通路段和元器件时，速度曲线可能在"路段准备"和"节点准备"之间切换不正确。当前所有赛道数据（Pre_Contest_1 等）的 `mileage_dir` 全部为 0，此局限不影响实际使用。
2. **Count.Spd_Mileage 累计但未读取**：`Spd_Mileage` 在 `Set_Speed` 中每周期累加，但不在任何控制逻辑中使用。疑似为未来功能预留。
3. **Advance_Turn_Section_Index 为空函数**：该函数体为空，在 `Set_Node_Run_Mode` 和 `Mileage_Run_Stage_2` 中被调用但无实际作用。

### 2. 格式对齐

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | Turn_Left_Run、Turn_Right_Run、Set_Out 函数缩进统一为标准 4 空格 |
| Ctrl.c | Set_Out 中各代码块间距统一，消除混合缩进（混用 tab/空格） |
| dev_CH455.c | 全文格式化，每行 I2C 操作添加行内注释 |

### 3. 注释增强

| 文件 | 修改内容 |
|------|----------|
| Ctrl.c | 关键判断逻辑添加说明（如记忆模式 80度 vs 建图模式 40度 的差异原因） |
| Ctrl.c | Set_Out 电压检测、启动延时等代码块添加功能注释 |
| dev_CH455.c | 函数头部补充完整的功能说明、参数文档、返回值文档 |
| dev_CH455.c | CH455_QueryOneKey 状态机逻辑添加分段注释 |

---

## 工作流程总结

1. **代码审查**：通读全部 15 个源文件（Ctrl.c/h、Fun.c/h、pid.c/h、Racing_Track.c、TCA9555.c/h、headfiles.h 及 OLED/ 子目录下 7 个文件）
2. **静态模拟仿真**：逐函数分析数据流、边界条件、Flash 读写配对、状态机完整性、I2C 通信时序
3. **发现问题**：发现 1 个严重 Bug（Flash 读取未 memcpy），1 个中危问题（滤波），1 个低危问题（阻塞超时），9 个建议改进（#2 #4 经确认不成立，已撤回；#11~#14 为逐行审查新增）
4. **修复严重 Bug**：补全 Load_Turn_Mileage_Record_From_Flash 缺失的 memcpy 调用
5. **格式对齐**：Ctrl.c 中 3 个函数缩进统一为 4 空格；dev_CH455.c 全文格式化
6. **注释增强**：Ctrl.c 全文逐行详细注释（全局变量用途、函数输入输出、控制链数据流、Flash布局、状态机转移条件、算法公式、已知缺陷标注）；dev_CH455.c 补充函数文档和状态机注释
7. **输出报告**：编写本 README 记录全部发现与修改

---

## 默认地图 Check-Edge 逐次路由表

当 `map_choose == 1` 时，加载默认地图 `DEFAULT_BUILD_MAP_NODE_NUM = 11`。小车从 `Execute_Times=0`（起点）出发，在 `Normal_Mode` 下每遇到一次物理边缘（`Check_Edge()` 返回 1），触发一次路由决策。下表按时间顺序列出每次 check-edge 时的路由行为和速度环（Left_PID / Right_PID）的参与情况。

### 默认地图数据回顾

```text
Node_Dir[11]    = {1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1}   // 1=左转90°, 0=直行通过
Mileage_Num[12] = {0, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0}  // 每段元器件数量
Mileage_Dir:    Row_0={0}  Row_1={4}   Row_2={3,3}  Row_3={0}  Row_4={4}  Row_5={0}
                Row_6={3}  Row_7={4}   Row_8={1}    Row_9={3}  Row_10={0} Row_11={0}

元器件方向: 0=普通路段(无元器件)  1=左转元件  2=右转元件  3=短直行(800)  4=长直行(1200)
```

### Check-Edge 逐次路由（建图模式 Build_Mode）

| # | Execute_Times | 路段 | node_dir | mileage_num | 当前元素 | 路由决策 | Run_Mode | 速度环 |
|---|--------------|------|----------|-------------|---------|---------|----------|--------|
| 1 | 0 | Row_0: 起点→Node1 | 左转(1) | 0 | — | 无元素，直接触发节点：**左转90°** | Turn_Left → Normal | 速度环差速控制 |
| 2 | 1 | Row_1: Node1→Node2 | 直行(0) | 1 | `mileage_dir[0]=4` | 检测到长直行元件：**里程模式走1200单位** | Mileage_Mode → Normal | Force_Straight_Speed=1, 两轮恒定Basic_Speed |
| 3 | 1 | Row_1: Node1→Node2 | 直行(0) | 1 | 元素全部完成 | **直行通过Node2**（Straight稳定后推进） | Straight_Mode → Normal | Error=0, 速度环差速微调 |
| 4 | 2 | Row_2: Node2→Node3 | 左转(1) | 2 | `mileage_dir[0]=3` | 检测到短直行元件：**里程模式走800单位** | Mileage_Mode → Normal | Force_Straight_Speed=1 |
| 5 | 2 | Row_2: Node2→Node3 | 左转(1) | 2 | `mileage_dir[1]=3` | 检测到第2个短直行元件：**里程模式走800单位** | Mileage_Mode → Normal | Force_Straight_Speed=1 |
| 6 | 2 | Row_2: Node2→Node3 | 左转(1) | 2 | 元素全部完成 | 触发节点：**左转90°到Node3** | Turn_Left → Normal | 速度环差速控制 |
| 7 | 3 | Row_3: Node3→Node4 | 直行(0) | 1 | `mileage_dir[0]=0` | 普通路段（无元件），In_Line_Ele_Count++后触发：**直行通过Node4** | Straight_Mode → Normal | Error=0, 速度环差速微调 |
| 8 | 4 | Row_4: Node4→Node5 | 左转(1) | 1 | `mileage_dir[0]=4` | 检测到长直行元件：**里程模式走1200单位** | Mileage_Mode → Normal | Force_Straight_Speed=1 |
| 9 | 4 | Row_4: Node4→Node5 | 左转(1) | 1 | 元素全部完成 | 触发节点：**左转90°到Node5** | Turn_Left → Normal | 速度环差速控制 |
| 10 | 5 | Row_5: Node5→Node6 | 直行(0) | 1 | `mileage_dir[0]=0` | 普通路段，推进后触发：**直行通过Node6** | Straight_Mode → Normal | Error=0, 速度环差速微调 |
| 11 | 6 | Row_6: Node6→Node7 | 左转(1) | 1 | `mileage_dir[0]=3` | 检测到短直行元件：**里程模式走800单位** | Mileage_Mode → Normal | Force_Straight_Speed=1 |
| 12 | 6 | Row_6: Node6→Node7 | 左转(1) | 1 | 元素全部完成 | 触发节点：**左转90°到Node7** | Turn_Left → Normal | 速度环差速控制 |
| 13 | 7 | Row_7: Node7→Node8 | 左转(1) | 1 | `mileage_dir[0]=4` | 检测到长直行元件：**里程模式走1200单位** | Mileage_Mode → Normal | Force_Straight_Speed=1 |
| 14 | 7 | Row_7: Node7→Node8 | 左转(1) | 1 | 元素全部完成 | 触发节点：**左转90°到Node8** | Turn_Left → Normal | 速度环差速控制 |
| 15 | 8 | Row_8: Node8→Node9 | 左转(1) | 1 | `mileage_dir[0]=1` | 检测到左转元件：**里程模式陀螺仪左转90°** | Mileage_Mode → Normal | 速度环差速控制 |
| 16 | 8 | Row_8: Node8→Node9 | 左转(1) | 1 | 元素全部完成 | 触发节点：**左转90°到Node9** | Turn_Left → Normal | 速度环差速控制 |
| 17 | 9 | Row_9: Node9→Node10 | 直行(0) | 1 | `mileage_dir[0]=3` | 检测到短直行元件：**里程模式走800单位** | Mileage_Mode → Normal | Force_Straight_Speed=1 |
| 18 | 9 | Row_9: Node9→Node10 | 直行(0) | 1 | 元素全部完成 | **直行通过Node10**（Straight稳定后推进） | Straight_Mode → Normal | Error=0, 速度环差速微调 |
| 19 | 10 | Row_10: Node10→Node11 | 左转(1) | 1 | `mileage_dir[0]=0` | 普通路段，推进后触发：**左转90°到Node11** | Turn_Left → Normal | 速度环差速控制 |

> 第19次 check-edge 触发 Node11 的左转后，`Advance_To_Next_Track_Segment` 使 `Execute_Times=11 == Node_Num`，`Finish_Flag=1`，任务完成。Row_11（终点段，mileage_num=0）正常情况下不会被遍历。

### 关键逻辑说明

1. **`mileage_dir == 0`（普通路段）**：不进入 Mileage_Mode，仅在 `Normal_Run` 中 `In_Line_Ele_Count++`，计数达到 `mileage_num` 后触发 `Set_Node_Run_Mode`（节点动作）。
2. **`mileage_dir == 3/4`（短/长直行）**：进入 `Mileage_Mode`，`Force_Straight_Speed=1` 强制两轮等速 `Basic_Speed`，绕过 PID 差速。行驶固定里程（800/1200）后 `Finish_Mileage_Section`。
3. **`mileage_dir == 1/2`（转弯元件）**：进入 `Mileage_Mode`，调用 `Mileage_Run_Stage_2()` 用陀螺仪控制转向，`Force_Straight_Speed=0`（保留差速控制）。
4. **节点直行（`node_dir == 0`）**：`Set_Node_Run_Mode` 设置 `Straight_Node_Pending=1` → `Straight_Run` 等待居中稳定 → `Finish_Mileage_Section` + `Advance_To_Next_Track_Segment` 推进。
5. **节点转弯（`node_dir == 1/2`）**：`Set_Node_Run_Mode` → `Turn_Left/Turn_Right` 状态机 → `Complete_Turn_Action` → `Advance_To_Next_Track_Segment` 推进。
6. **速度环（Left_PID / Right_PID）**：仅在以下情况被清零误差历史——回放模式直行元件期间 `Check_Edge()` 持续为真时，`PID_cleardata(&Turn_PID)` 和 `PID_cleardata(&Gyro_PID)` 每 3ms 清零一次（见 Mileage_Mode_Run:1640-1645）。`Left_PID` 和 `Right_PID` 的误差数组不受影响。
7. **节点推进统一在转弯完成或直道稳定后**：不在 `Normal_Run` 中直接调用 `Advance_To_Next_Track_Segment`，统一由 `Complete_Turn_Action` 或 `Straight_Run` 负责。

### 赛道示意

```text
起点 ──→ [Node1:左转] ──长直行1200──→ [Node2:直行] ──短直800+短直800──→ [Node3:左转]
  ──普通路段──→ [Node4:直行] ──长直行1200──→ [Node5:左转]
  ──普通路段──→ [Node6:直行] ──短直行800──→ [Node7:左转]
  ──长直行1200──→ [Node8:左转] ──左转元件──→ [Node9:左转]
  ──短直行800──→ [Node10:直行] ──普通路段──→ [Node11:左转] → 终点
```

*分析日期：2026-05-09*
