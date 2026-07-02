/**
 * @file        OLEDKeyboard.c
 * @author      TYUT JBD
 * @version     2.0
 * @date        2024.12.07
 * @brief       OLED键盘+OLED显示驱动，实现赛道地图编辑、参数配置、数据存储功�?
 * @details     依赖CH455键盘、OLED显示屏、Flash存储、PID控制模块
 * @copyright   Copyright (C) 2016-2024, TYUT JBD
 * @history
 *   JBD          2016.10.21     0.0        初始版本
 *   AmaZzzing    2016.11.12     1.0        预赛赛道配置完成
 *   SUV          2024.12.07     2.0        基于新库重构
 */

#include "OLEDKeyboard.h"
#include "Ctrl.h"

/*
 * 键盘显示模块核心功能�?
 * 1. 通过 CH455 键盘 + OLED 屏幕 输入/编辑赛道地图数据
 * 2. 将输入结果转换为 Run_Track 结构体，并写�?Flash 掉电保存
 * 3. 内置默认地图，开机可一键加�?
 */


/*===============================================================================
  全局变量定义
================================================================================*/
uint32 input = 0;                                     // 通用输入缓存
uint32 Speed_OKb[1] = {0};                            // 基础速度参数存储
uint32 PID_OKb[13] = {0};                             // [4:5]Angle PD [6:8]Gyro PID [9:10]Gyro PD
uint32 Ctrl_OKb[8] = {560, 260, 0, 0, 0, 0, 0, 0};         // [0]ElementTurnDelay [1]NodeTurnDelay
uint32 DBG_OKb[4] = {40, 8000, 1, 1};                              // 调试参数存储

// 键盘回放模式输入数据缓存
uint8 Flash_Node_Num = 0;                             // Flash存储的赛道节点总数
uint8 Flash_Node_Dir[NODE_NUM_MAX] = {0};             // 各节点行驶方�?
uint8 Flash_Node_Mileage_Num[TRACK_SEGMENT_NUM_MAX] = {0};  // Segment element count
uint8 mileage_dir[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX] = {{0}};  // Segment element direction

/*===============================================================================
  Flash 存储相关宏定�?
================================================================================*/
/*
 * 地图Flash分区分配说明�?
 * �?页：仅存储键盘输入的原始地图数据
 * 读取后与 Pre_Contest_1 结合，重建完�?Run_Track 运行赛道结构�?
 */
#define BUILD_MAP_FLASH_SECTOR        0                 // Flash �?扇区：当前工程参�?地图存储�?
#define BUILD_MAP_FLASH_START_PAGE    4                 // 地图数据起始页：�?页专门存键盘输入原始赛道数据
#define BUILD_MAP_FLASH_WORDS_PER_PAGE 64               // 每页64个uint32，与flash_write_page页宽一�?
#define BUILD_MAP_FLASH_PAGE_COUNT    1                 // 地图数据当前仅占1�?
#define BUILD_MAP_FLASH_WORD_COUNT    (BUILD_MAP_FLASH_WORDS_PER_PAGE * BUILD_MAP_FLASH_PAGE_COUNT) // 地图数据总长�?

/*===============================================================================
  Flash 地图存储结构�?
================================================================================*/
// 仅存储建图时真实输入的原始数据，不存储预设地图默认�?
typedef struct
{
    uint8 Node_Num;                                   // 节点总数
    uint8 Node_Arr_Dir[NODE_NUM_MAX];                 // 节点方向数组
    uint8 Node_Arr_Mileage_Num[TRACK_SEGMENT_NUM_MAX];     // 各段里程数量
    uint8 Node_Arr_Mileage_Dir[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX]; // 各段里程方向
} Build_Map_Flash_Typedef;

/*===============================================================================
  内部函数声明
================================================================================*/
static void OLED_Apply_Build_Mode(void);
static void OLED_Save_Build_Mode_Map_To_Flash(void);
static void OLED_Load_Build_Mode_Map_From_Flash(void);
static void OLED_Build_Mode_Input(void);

/*===============================================================================
  Default track map (used when key '1' is pressed in Build input)
  Now defined in Ctrl.c as Default_Build_Actions[] + Mileage_Num_By_Segment[].
================================================================================*/
// Segment 0 is "start -> node 0". If mileage_num[0]=0, run starts from the first node.

/**
 * @brief   将数字数组转换为字符串并在OLED显示
 * @param   x       显示横坐�?
 * @param   y       显示纵坐�?
 * @param   digits  数字数组
 * @param   len     数字长度
 */
static void OLED_Show_Digit_Buffer(uint16 x, uint16 y, uint8 digits[], uint8 len)
{
    uint8 show_str[NODE_NUM_MAX + 1] = {0};  
    uint8 i;

    // 逐位将数字转为ASCII字符
    for (i = 0; i < len && i < NODE_NUM_MAX; i++)
    {
        show_str[i] = digits[i] + '0';
    }
    show_str[i] = '\0';  // 添加字符串结束符

    OLED_Show_Str(x, y, show_str, TextSize_F6x8);
}

/**
 * @brief   加载默认地图到建图缓�?
 */
static void OLED_Load_Default_Build_Mode_Map(void)
{
#if 0   // (Mileage_Num_By_Segment removed — OLED disabled on new car)
    uint8 row;
    uint8 i;

    Flash_Node_Num = BUILD_NODE_NUM;

    // Load default per-segment counts from Ctrl.c constants
    for (row = 0; row <= BUILD_NODE_NUM; row++)
    {
        Flash_Node_Mileage_Num[row] = Mileage_Num_By_Segment[row];
    }

    // Copy default action list directly to runtime array
    Build_Action_Count = BUILD_ACTION_COUNT;
    memcpy(Build_Action_List, Default_Build_Actions, sizeof(Default_Build_Actions));

    // FIXME: flash map save skipped (format changed)
#endif
}

/**
 * @brief   单行数字输入，支持退格、确�?
 * @param   title         标题字符�?
 * @param   show_hint     是否显示提示信息
 * @param   show_row_index是否显示行索�?
 * @param   row_index     行索引�?
 * @param   input_x       输入区横坐标
 * @param   input_y       输入区纵坐标
 * @param   digits        输出数字缓存数组
 * @param   max_len       最大输入长�?
 * @return  实际输入长度
 */
static uint8 OLED_Read_Digit_Line(uint8 title[], uint8 show_hint, uint8 show_row_index, uint8 row_index,
                                  uint16 input_x, uint16 input_y, uint8 digits[], uint8 max_len)
{
    KeyValue_enum key_value = KEY_BLANK;
    uint8 len = 0;

    while (1)
    {
        OLED_CLS();  // 清屏刷新

        // 显示提示信息
        if (show_hint != 0)
        {
            OLED_Show_Str(0, 0, "N S=0 L=1 R=2", TextSize_F6x8);
            OLED_Show_Str(0, 1, "E 0=None 1=TL 2=TR 3=SS 4=LS", TextSize_F6x8);
            OLED_Show_Str(0, 2, title, TextSize_F6x8);

            if (show_row_index != 0)
            {
                OLED_Show_Numbers(54, 2, row_index, TextSize_F6x8);
            }
        }
        else
        {
            OLED_Show_Str(0, 0, title, TextSize_F6x8);
        }

        // 显示已输入内�?
        OLED_Show_Digit_Buffer(input_x, input_y, digits, len);

        // 获取按键
        key_value = CH455_GetOneKey();

        // 长按转普通键�?
        if (key_value > 0x0F && key_value != KEY_BLANK)
        {
            key_value = (key_value - 0x0F) >> 4;
        }

        if (key_value == KEY_BLANK) continue;

        // 确认键：退出输�?
        if (key_value == KEY_ENTER) break;

        // 退格键：删除最后一�?
        if (key_value == KEY_BACK)
        {
            if (len > 0) len--;
            continue;
        }

        // 数字键：添加到缓�?
        if (key_value <= KEY_9 && len < max_len)
        {
            digits[len] = (uint8)key_value;
            len++;
        }
    }

    return len;
}

/**
 * @brief   将建图缓存数据应用到运行赛道结构�?Run_Track
 */
static void OLED_Apply_Build_Mode(void)
{
#if 0  // (Build_Action_List now uint8_t[], OLED disabled on new car)
    uint8 seg, elem;
    uint8 count = 0;

    for (seg = 0; seg <= Flash_Node_Num && seg < TRACK_SEGMENT_NUM_MAX; seg++)
    {
        uint8 mnum = Flash_Node_Mileage_Num[seg];
        if (mnum > ELEMENT_NUM_MAX) mnum = ELEMENT_NUM_MAX;

        // Elements first
        for (elem = 0; elem < mnum && count < BUILD_ACTION_MAX; elem++)
        {
            switch (mileage_dir[seg][elem])
            {
                case 1: Build_Action_List[count] = BUILD_ACTION_ELEM_TURN_LEFT;     break;
                case 2: Build_Action_List[count] = BUILD_ACTION_ELEM_TURN_RIGHT;    break;
                case 3: Build_Action_List[count] = BUILD_ACTION_ELEM_STRAIGHT_SHORT; break;
                case 4: Build_Action_List[count] = BUILD_ACTION_ELEM_STRAIGHT_LONG;  break;
                default: Build_Action_List[count] = BUILD_ACTION_NONE; break;
            }
            count++;
        }

        // Then node (if within node count)
        if (seg < Flash_Node_Num && count < BUILD_ACTION_MAX)
        {
            switch (Flash_Node_Dir[seg])
            {
                case 0: Build_Action_List[count] = BUILD_ACTION_NODE_STRAIGHT;  break;
                case 1: Build_Action_List[count] = BUILD_ACTION_NODE_TURN_LEFT; break;
                case 2: Build_Action_List[count] = BUILD_ACTION_NODE_TURN_RIGHT; break;
                default: Build_Action_List[count] = BUILD_ACTION_NONE; break;
            }
            count++;
        }
    }

    Build_Action_Count = count;
#endif
}

/**
 * @brief   将当前建图数据分页保存到Flash
 */
static void OLED_Save_Build_Mode_Map_To_Flash(void)
{
    uint32 map_words[BUILD_MAP_FLASH_WORD_COUNT] = {0};
    Build_Map_Flash_Typedef flash_map = {0};
    uint8 page_index;
    uint8 i;
    uint8 row;

    // 打包原始输入数据
    flash_map.Node_Num = Flash_Node_Num;
    for (i = 0; i < Flash_Node_Num; i++)
        flash_map.Node_Arr_Dir[i] = Flash_Node_Dir[i];

    for (row = 0; row <= Flash_Node_Num; row++)
    {
        flash_map.Node_Arr_Mileage_Num[row] = Flash_Node_Mileage_Num[row];
        for (i = 0; i < Flash_Node_Mileage_Num[row]; i++)
            flash_map.Node_Arr_Mileage_Dir[row][i] = mileage_dir[row][i];
    }

    // 复制到uint32数组
    memcpy(map_words, &flash_map, sizeof(flash_map));

    // 擦除并写入Flash
    for (page_index = 0; page_index < BUILD_MAP_FLASH_PAGE_COUNT; page_index++)
    {
        flash_erase_page(BUILD_MAP_FLASH_SECTOR, BUILD_MAP_FLASH_START_PAGE + page_index);
        flash_write_page(BUILD_MAP_FLASH_SECTOR,
                         BUILD_MAP_FLASH_START_PAGE + page_index,
                         &map_words[page_index * BUILD_MAP_FLASH_WORDS_PER_PAGE],
                         BUILD_MAP_FLASH_WORDS_PER_PAGE);
    }
}

/**
 * @brief   从Flash加载已保存的地图
 */
static void OLED_Load_Build_Mode_Map_From_Flash(void)
{
    uint32 map_words[BUILD_MAP_FLASH_WORD_COUNT] = {0};
    Build_Map_Flash_Typedef flash_map = {0};
    uint8 page_index;

    // 读取Flash数据
    for (page_index = 0; page_index < BUILD_MAP_FLASH_PAGE_COUNT; page_index++)
    {
        flash_read_page(BUILD_MAP_FLASH_SECTOR,
                        BUILD_MAP_FLASH_START_PAGE + page_index,
                        &map_words[page_index * BUILD_MAP_FLASH_WORDS_PER_PAGE],
                        BUILD_MAP_FLASH_WORDS_PER_PAGE);
    }

    // 恢复到地图结构体
    memcpy(&flash_map, map_words, sizeof(flash_map));

    // 恢复到输入缓�?
    Flash_Node_Num = flash_map.Node_Num;
    if (Flash_Node_Num > NODE_NUM_MAX)        // Flash空片(0xFF)或损坏时防越�?
        Flash_Node_Num = NODE_NUM_MAX;

    for (uint8 i = 0; i < Flash_Node_Num; i++)
    {
        Flash_Node_Dir[i] = flash_map.Node_Arr_Dir[i];
    }

    for (uint8 row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = flash_map.Node_Arr_Mileage_Num[row];
        if (Flash_Node_Mileage_Num[row] > ELEMENT_NUM_MAX)
            Flash_Node_Mileage_Num[row] = ELEMENT_NUM_MAX;

        for (uint8 i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            mileage_dir[row][i] = flash_map.Node_Arr_Mileage_Dir[row][i];
        }
    }

    // 应用到运行赛�?
OLED_Apply_Build_Mode();
}

/**
 * @brief   地图编辑模式输入入口
 */
static void OLED_Build_Mode_Input(void)
{
    uint8 node_digits[NODE_NUM_MAX] = {0};
    uint8 line_digits[ELEMENT_NUM_MAX] = {0};
    uint8 row;
    int32 map_choose = 1;

    OLED_CLS();
    OLED_Show_Str(0, 0, "1Default 2New", TextSize_F6x8);
    OLED_Show_Str(0, 2, "Row0=Start->N1", TextSize_F6x8);
    OLED_Show_Str(0, 4, "Build Select", TextSize_F6x8);
    map_choose = KeyboardInput(88, 6, TextSize_F8x16, 1.0);
    OLED_CLS();

    // 加载默认地图
    if (map_choose == 1)
    {
        OLED_Load_Default_Build_Mode_Map();
        return;
    }

    // 清空缓存
    Flash_Node_Num = 0;
    for (uint8 i = 0; i < NODE_NUM_MAX; i++)
    {
        Flash_Node_Dir[i] = 0;
    }

    for (row = 0; row < TRACK_SEGMENT_NUM_MAX; row++)
    {
        Flash_Node_Mileage_Num[row] = 0;
        for (uint8 i = 0; i < ELEMENT_NUM_MAX; i++)
        {
            mileage_dir[row][i] = 0;
        }
    }
    // 输入节点�?
    Flash_Node_Num = OLED_Read_Digit_Line("Nodes:", 0, 0, 0, 0, 3, node_digits, NODE_NUM_MAX);
    for (uint8 i = 0; i < Flash_Node_Num; i++)
    {
        Flash_Node_Dir[i] = node_digits[i];
    }

    OLED_CLS();

    // 逐行输入里程方向
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = OLED_Read_Digit_Line("Mileage", 1, 1, row, 0, 5, line_digits, ELEMENT_NUM_MAX);
        for (uint8 i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            mileage_dir[row][i] = line_digits[i];
        }
        OLED_CLS();
    }

    OLED_Apply_Build_Mode();
    // FIXME: OLED_Save_Build_Mode_Map_To_Flash(); // flash format changed
}

/**
 * @brief   在OLED最后一行显示光敏传感器状态（0/1字符串）
 */
static void OLED_Show_Light_Row(void)
{
    uint8 light_str[16] = {0};
    for (uint8 i = 0; i < 15; i++)
    {
        light_str[i] = Light_Convert[i] ? '1' : '0';
    }
    OLED_Show_Str(0, 7, light_str, TextSize_F6x8);
}

/**
 * @brief   OLED查看里程数据（不调参不跑车）
 * @details 布局：P0=摘要(F8x16大字)、P1+=逐段(含空�?、P�?转弯间距(3�?�?
 */
static void OLED_View_Mileage_Data(void)
{
#if 0  // (Mileage_Num_By_Segment removed — OLED disabled on new car)
    uint16 seg, elem, ele_num, total_ele, tr_page, tr_pages;
    uint8  row;
    char   buf[22];

    Load_All_Flash_Data_For_VOFA();

    // 统计元素总数
    total_ele = 0;
    for (seg = 0; seg <= BUILD_NODE_NUM; seg++)
        total_ele += Mileage_Num_By_Segment[seg];

    //===== Page 0: 摘要 (F8x16 大字) =====
    OLED_CLS();
    sprintf(buf, "Nodes:%d", BUILD_NODE_NUM);
    OLED_Show_Str(0, 0, buf, TextSize_F8x16);
    sprintf(buf, "Segs:%d", BUILD_NODE_NUM + 1);
    OLED_Show_Str(64, 0, buf, TextSize_F8x16);

    sprintf(buf, "Turns:%d", Turn_Mileage_Record_Num);
    OLED_Show_Str(0, 2, buf, TextSize_F8x16);
    sprintf(buf, "Elems:%d", total_ele);
    OLED_Show_Str(64, 2, buf, TextSize_F8x16);

    sprintf(buf, "Actions:%d", BUILD_ACTION_COUNT);
    OLED_Show_Str(0, 4, buf, TextSize_F8x16);
    CH455_GetOneKey();

    //===== 逐段显示：所有段，一页一�?=====
    for (seg = 0; seg <= BUILD_NODE_NUM && seg < TRACK_SEGMENT_NUM_MAX; seg++)
    {
        ele_num = Mileage_Num_By_Segment[seg];
        OLED_CLS();

        // Row 0: 段标�?F8x16
        sprintf(buf, "Seg%d/%d", seg, BUILD_NODE_NUM);
        OLED_Show_Str(0, 0, buf, TextSize_F8x16);
        if (ele_num > 0)
            OLED_Show_Str(56, 0, "Elem", TextSize_F8x16);
        else
            OLED_Show_Str(56, 0, "Node", TextSize_F8x16);

        // Rows 2+: 元素详情 (F6x8)
        if (ele_num == 0)
        {
            // 无元器件段：显示实测总里�?
            if (Segment_Total_Mileage[seg] > 0)
            {
                sprintf(buf, "total:%.0f", Segment_Total_Mileage[seg]);
                OLED_Show_Str(0, 2, buf, TextSize_F6x8);
            }
            else
            {
                OLED_Show_Str(0, 2, "(no elements)", TextSize_F6x8);
            }
            row = 4;
        }
        else
        {
            row = 2;
            for (elem = 0; elem < ele_num && row <= 6; elem++)
            {
                sprintf(buf, "E%d @%.0f", elem + 1,
                        Segment_Edge_Mileage_Record[seg][elem]);
                OLED_Show_Str(1, row, buf, TextSize_F6x8);
                row++;
            }

            // 实测总里�?= 该段起点到接触下一节点的距�?
            if (row <= 7 && Segment_Total_Mileage[seg] > 0)
            {
                sprintf(buf, "total:%.0f", Segment_Total_Mileage[seg]);
                OLED_Show_Str(0, row, buf, TextSize_F6x8);
                row++;
            }
        }
        CH455_GetOneKey();
    }

    if (Turn_Mileage_Record_Num > 0)
    {
        tr_pages = (Turn_Mileage_Record_Num + 2) / 3;
        for (tr_page = 0; tr_page < tr_pages; tr_page++)
        {
            OLED_CLS();
            sprintf(buf, "Turns %d/%d", tr_page + 1, tr_pages);
            OLED_Show_Str(0, 0, buf, TextSize_F8x16);

            row = 2;
            for (elem = 0; elem < 3; elem++)
            {
                uint16 idx = tr_page * 3 + elem;
                if (idx >= Turn_Mileage_Record_Num) break;
                sprintf(buf, "#%d: %.0f", idx + 1, Turn_Mileage_Record[idx]);
                OLED_Show_Str(0, row, buf, TextSize_F6x8);
                row += 2;
            }
            CH455_GetOneKey();
        }
    }
#endif  // (Mileage_Num_By_Segment removed)
}

/**
 * @brief   键盘输入总入口：模式选择、地图编辑、参数配�?
 */
void OLED_Input(void)
{
    CH455_Init(); // 初始化CH455键盘(IIC通信)

    int32 OLED_Choose;
    uint8 mode_selected = 0;  // 模式已选择标志�?/3=跑车, 8=仅查看后回菜单）

    while (!mode_selected)
    {
        OLED_Show_Str(20, 3, "Nothing or Best.", TextSize_F6x8);
        OLED_Choose = KeyboardInput(88, 6, TextSize_F8x16, 1.0);

        //====== Mode selection (disabled — Mode removed, USE_DEBUG_MODE is the sole switch) ======
        switch (OLED_Choose)
        {
#if 0
            case 1:
                mode_selected = 1;
                break;
            case 2:
                OLED_Show_Str(0, 0, "Remember Off", TextSize_F6x8);
                break;
            case 3:
                mode_selected = 1;
                break;
#endif
            case 8:   // View mileage data mode (no tuning, no running, OLED display only)
                OLED_View_Mileage_Data();
                break;
        }
        OLED_CLS();
    }

#if 0   // Mode removed — OLED mode selection disabled
    // Enter build mode
    if (0)
    {
        OLED_Build_Mode_Input();
    }

    // Enter debug mode (select sub-mode + load parameters)
    if (0)
    {
        int32 dbg_choose;

        //---- 选择子模�?----
        OLED_CLS();
        OLED_Show_Str(0, 0, "Debug Mode", TextSize_F6x8);
        OLED_Show_Str(0, 2, "1 PI Tuning", TextSize_F6x8);
        OLED_Show_Str(0, 3, "2 GroundTest", TextSize_F6x8);
        OLED_Show_Str(0, 4, "3 Angle Loop", TextSize_F6x8);
        OLED_Show_Str(0, 5, "4 NormTrace", TextSize_F6x8);
        dbg_choose = KeyboardInput(88, 7, TextSize_F6x8, 1.0);

        switch (dbg_choose)
        {
            case 1: Debug_Sub_Mode  = Debug_Sub_PI_Tuning;   break;
            case 2: Debug_Sub_Mode  = Debug_Sub_Ground_Test; break;
            case 3: Debug_Sub_Mode  = Debug_Sub_Angle;       break;
            case 4: Debug_Sub_Mode  = Debug_Sub_NormalTrace; break;
            default:Debug_Sub_Mode  = Debug_Sub_PI_Tuning;   break;
        }

        //---- 从Flash恢复上次保存的调试参�?----
        flash_read_page(0, 1, PID_OKb, 13);
        if (PID_OKb[0] != 0) Left_PID.kp  = PID_OKb[0];
        if (PID_OKb[1] != 0) Left_PID.ki  = PID_OKb[1] * 0.01;
        if (PID_OKb[2] != 0) Right_PID.kp = PID_OKb[2];
        if (PID_OKb[3] != 0) Right_PID.ki = PID_OKb[3] * 0.01;
        flash_read_page(0, 3, DBG_OKb, 4);
        if (DBG_OKb[0] != 0) Debug_Target_Speed = DBG_OKb[0];
        if (DBG_OKb[1] != 0) Debug_Fan_Duty = DBG_OKb[1];
        if (DBG_OKb[2] == 1 || DBG_OKb[2] == 2) Debug_Ground_Dir = DBG_OKb[2];
        if (DBG_OKb[3] == 1 || DBG_OKb[3] == 2) Debug_Angle_Mode = DBG_OKb[3];
        // 电机初始为停，页面为编辑�?
        Debug_Motor_Enable = 0;

        // 跳过速度/PID参数配置，直接启�?
        Data_Load();
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        return;
    }
#endif  // Mode removed

    //====== 基础速度配置 ======
    flash_read_page(0, 0, Speed_OKb, 1);
    OLED_Show_Str(0, 0, "B_Spd", TextSize_F6x8);
    OLED_Show_Numbers(47, 0, Speed_OKb[0], TextSize_F6x8);

    input = KeyboardInput(90, 0, TextSize_F6x8, 1.0);
    if (input != 0)
    {
        Speed_OKb[0] = input;
    }
    OLED_CLS();

    // 擦除并写入速度参数
    flash_erase_page(0, 0);
    flash_write_page(0, 0, Speed_OKb, 1);

    // 菜单选择
    OLED_Choose = KeyboardInput(88, 6, TextSize_F8x16, 1.0);
    OLED_CLS();

    //====== 参数配置 ======
    switch (OLED_Choose)
    {
        case 1: // PID参数 + 控制参数配置
        {
            // 读取并配置左右电机PID
            flash_read_page(0, 1, PID_OKb, 13);
            if (PID_OKb[4] == 0 || PID_OKb[4] > 20000) PID_OKb[4] = (uint32)(Angle_PID.kp * 100.0f);
            if (PID_OKb[5] == 0 || PID_OKb[5] > 20000) PID_OKb[5] = (uint32)(Angle_PID.kd * 100.0f);
            if (PID_OKb[6] == 0 || PID_OKb[6] > 10000) PID_OKb[6] = (uint32)(Gyro_PID.kp * 1000.0f);
            if (PID_OKb[7] == 0 || PID_OKb[7] > 10000) PID_OKb[7] = (uint32)(Gyro_PID.ki * 1000.0f);
            if (PID_OKb[8] == 0 || PID_OKb[8] > 10000) PID_OKb[8] = (uint32)(Gyro_PD_PID.kp * 1000.0f);
            if (PID_OKb[9] == 0 || PID_OKb[9] > 10000) PID_OKb[9] = (uint32)(Gyro_PD_PID.kd * 1000.0f);
            if (PID_OKb[10] > 10000) PID_OKb[10] = (uint32)(Gyro_PID.kd * 1000.0f);
            OLED_Show_Str(0, 0, "L_P", TextSize_F6x8);
            OLED_Show_Numbers(47, 0, PID_OKb[0], TextSize_F6x8);
            OLED_Show_Str(0, 2, "L_I", TextSize_F6x8);
            OLED_Show_Numbers(47, 2, PID_OKb[1], TextSize_F6x8);
            OLED_Show_Str(0, 4, "R_P", TextSize_F6x8);
            OLED_Show_Numbers(47, 4, PID_OKb[2], TextSize_F6x8);
            OLED_Show_Str(0, 6, "R_I", TextSize_F6x8);
            OLED_Show_Numbers(47, 6, PID_OKb[3], TextSize_F6x8);

            for (uint16 i = 0; i < 4; i++)
            {
                input = KeyboardInput(90, 2 * i, TextSize_F6x8, 1.0);
                if (input != 0) PID_OKb[i] = input;
            }
            OLED_CLS();

            // 转向PID + 陀螺仪PID
            OLED_Show_Str(0, 0, "T_P", TextSize_F6x8);
            OLED_Show_Numbers(47, 0, PID_OKb[4], TextSize_F6x8);
            OLED_Show_Str(0, 1, "T_D", TextSize_F6x8);
            OLED_Show_Numbers(47, 1, PID_OKb[5], TextSize_F6x8);
            OLED_Show_Str(0, 2, "GI_P", TextSize_F6x8);
            OLED_Show_Numbers(47, 2, PID_OKb[6], TextSize_F6x8);
            OLED_Show_Str(0, 3, "GI_I", TextSize_F6x8);
            OLED_Show_Numbers(47, 3, PID_OKb[7], TextSize_F6x8);
            OLED_Show_Str(0, 4, "GI_D", TextSize_F6x8);
            OLED_Show_Numbers(47, 4, PID_OKb[10], TextSize_F6x8);
            OLED_Show_Str(0, 5, "GD_P", TextSize_F6x8);
            OLED_Show_Numbers(47, 5, PID_OKb[8], TextSize_F6x8);
            OLED_Show_Str(0, 6, "GD_D", TextSize_F6x8);
            OLED_Show_Numbers(47, 6, PID_OKb[9], TextSize_F6x8);

            {
                const uint16 pid_index[7] = {4, 5, 6, 7, 10, 8, 9};
                for (uint16 i = 0; i < 7; i++)
                {
                    input = KeyboardInput(90, i, TextSize_F6x8, 1.0);
                    if (input != 0) PID_OKb[pid_index[i]] = input;
                }
            }
            OLED_CLS();

            // 保存PID到Flash
            flash_erase_page(0, 1);
            flash_write_page(0, 1, PID_OKb, 13);

            // 读取并配置控制参�?
            flash_read_page(0, 2, Ctrl_OKb, 8);
            if (Ctrl_OKb[0] == 0 || Ctrl_OKb[0] > 5000) Ctrl_OKb[0] = (uint32)TUNE_ELEM_TURN_DELAY;
            if (Ctrl_OKb[1] == 0 || Ctrl_OKb[1] > 5000) Ctrl_OKb[1] = (uint32)TUNE_NODE_TURN_DELAY;
            OLED_Show_Str(0, 0, "E_Dly", TextSize_F6x8);
            OLED_Show_Numbers(47, 0, Ctrl_OKb[0], TextSize_F6x8);
            OLED_Show_Str(0, 1, "N_Dly", TextSize_F6x8);
            OLED_Show_Numbers(47, 1, Ctrl_OKb[1], TextSize_F6x8);

            input = KeyboardInput(90, 0, TextSize_F6x8, 1.0);
            if(input != 0) Ctrl_OKb[0] = input;
            input = KeyboardInput(90, 1, TextSize_F6x8, 1.0);
            if(input != 0) Ctrl_OKb[1] = input;

            OLED_CLS();
            flash_erase_page(0, 2);
            flash_write_page(0, 2, Ctrl_OKb, 8);
        }
        break;
    }
}

/**
 * @brief   从Flash加载所有配置参数到运行变量
 */
void Data_Load()
{
//    OLED_Load_Default_Build_Mode_Map();
//    // 加载PID参数
    flash_read_page(0, 1, PID_OKb, 13);
    if (PID_OKb[0] != 0 && PID_OKb[0] <= 2000) Left_PID.kp = PID_OKb[0];
    if (PID_OKb[1] != 0 && PID_OKb[1] <= 10000) Left_PID.ki = PID_OKb[1] * 0.01f;
    if (PID_OKb[2] != 0 && PID_OKb[2] <= 2000) Right_PID.kp = PID_OKb[2];
    if (PID_OKb[3] != 0 && PID_OKb[3] <= 10000) Right_PID.ki = PID_OKb[3] * 0.01f;
    if (PID_OKb[4] != 0 && PID_OKb[4] <= 20000) Angle_PID.kp = PID_OKb[4] * 0.01f;
    if (PID_OKb[5] != 0 && PID_OKb[5] <= 20000) Angle_PID.kd = PID_OKb[5] * 0.01f;
    Angle_PID.ki = 0;
    Angle_PID.mode = PID_MODE_POSITION_D_ON_MEASUREMENT;
    Turn_PID.ki = 0;
    Turn_PID.mode = PID_MODE_POSITION;
    if (PID_OKb[6] != 0 && PID_OKb[6] <= 10000) Gyro_PID.kp = PID_OKb[6] * 0.001f;
    if (PID_OKb[7] != 0 && PID_OKb[7] <= 10000) Gyro_PID.ki = PID_OKb[7] * 0.001f;
    if (PID_OKb[10] <= 10000) Gyro_PID.kd = PID_OKb[10] * 0.001f;
    Gyro_PID.mode = PID_MODE_ADD;
    if (PID_OKb[8] != 0 && PID_OKb[8] <= 10000) Gyro_PD_PID.kp = PID_OKb[8] * 0.001f;
    if (PID_OKb[9] != 0 && PID_OKb[9] <= 10000) Gyro_PD_PID.kd = PID_OKb[9] * 0.001f;
    Gyro_PD_PID.ki = 0;
    Gyro_PD_PID.mode = PID_MODE_POSITION;

    // 加载基础速度（暂时硬编码，调完后恢复）
//    flash_read_page(0, 0, Speed_OKb, 1);
//    if (Speed_OKb[0] != 0 && Speed_OKb[0] <= 300) Basic_Speed = Speed_OKb[0];

    // 加载控制参数（暂时硬编码，调完后恢复）
//    flash_read_page(0, 2, Ctrl_OKb, 8);
//    if (Ctrl_OKb[0] != 0 && Ctrl_OKb[0] <= 5000) TUNE_ELEM_TURN_DELAY = Ctrl_OKb[0];
//    if (Ctrl_OKb[1] != 0 && Ctrl_OKb[1] <= 5000) TUNE_NODE_TURN_DELAY = Ctrl_OKb[1];
}

/**
 * @brief   OLED实时显示运行数据 + 调试模式交互
 */
// void OLED_Display(void)
// {
//     if (Mode == Debug_Mode)
//     {
//         //===== 调试模式显示/交互 =====
//         if (Debug_Sub_Mode == Debug_Sub_PI_Tuning)
//         {
//             int32 key = KEY_BLANK;
//             static int32 edit_value = 0;  // 跨帧保存编辑中的数�?

//             if (Debug_Motor_Enable == 0)
//             {
//                 // ==== �?: 参数编辑页（四行从上到下依次输入，输完自动开电机�?====
//                 int32 cur_val[4];   // 0=Kp, 1=Ki, 2=Spd, 3=Wheel(0�?�?
//                 int i;

//                 OLED_CLS();
//                 cur_val[0] = (int)(Debug_Which_Wheel ? Debug_Kp_Right : Debug_Kp_Left);
//                 cur_val[1] = (int)((Debug_Which_Wheel ? Debug_Ki_Right : Debug_Ki_Left) * 100.0f);
//                 cur_val[2] = Debug_Target_Speed;
//                 cur_val[3] = Debug_Which_Wheel;

//                 OLED_Show_Str(0, 0, "Kp", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "Ki", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "Spd", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
//                 OLED_Show_Str(0, 3, "Wheel", TextSize_F6x8);
//                 OLED_Show_Str(45, 3, cur_val[3] ? "Right" : "Left", TextSize_F6x8);

//                 // 四行从上到下依次输入�?保持不变�?
//                 for (i = 0; i < 3; i++)
//                 {
//                     edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
//                     if (edit_value != 0) cur_val[i] = edit_value;
//                 }
//                 // Wheel: 输入2=�? 1=�?
//                 edit_value = KeyboardInput(45, 3, TextSize_F6x8, 1.0);
//                 if (edit_value == 2) cur_val[3] = 0;
//                 else if (edit_value == 1) cur_val[3] = 1;

//                 // 回写到全局变量
//                 Debug_Which_Wheel = cur_val[3];
//                 if (Debug_Which_Wheel == 0)
//                 {
//                     Debug_Kp_Left  = cur_val[0];
//                     Debug_Ki_Left  = cur_val[1] * 0.01;
//                 }
//                 else
//                 {
//                     Debug_Kp_Right = cur_val[0];
//                     Debug_Ki_Right = cur_val[1]*0.01;
//                 }
//                 Debug_Target_Speed = cur_val[2];

//                 // 输入完毕 �?自动开电机
//                 Debug_Motor_Enable = 1;
//                 PID_cleardata(&Left_PID);
//                 PID_cleardata(&Right_PID);
//                 OLED_CLS();
//             }
//             else  // Debug_Motor_Enable == 1
//             {
//                 // ==== �?: 运行页（Real/PWM + 1.Save 0.Stop�?====
//                 int real_spd = (Debug_Which_Wheel == 0) ? Left_Real_Spd : Right_Real_Spd;
//                 int pwm_out  = (Debug_Which_Wheel == 0) ? (int)Left_PID_Out : (int)Right_PID_Out;

//                 OLED_Show_Str(0, 0, "Run", TextSize_F6x8);
//                 OLED_Show_Str(55, 0, Debug_Which_Wheel ? "R" : "L", TextSize_F6x8);

//                 OLED_Show_Str(0, 1, "Real", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 1, real_spd, TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "PWM", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 2, pwm_out, TextSize_F6x8);

//                 OLED_Show_Str(0, 4, "1.Save", TextSize_F6x8);
//                 OLED_Show_Str(0, 5, "0.Stop", TextSize_F6x8);

//                 key = CH455_GetOneKey();
//                 if (key > 0x0F && key != KEY_BLANK)
//                     key = (key - 0x0F) >> 4;

//                 if (key == 1)  // 保存到Flash
//                 {
//                     PID_OKb[0] = (uint32)Debug_Kp_Left;
//                     PID_OKb[1] = (uint32)(Debug_Ki_Left * 100.0f);
//                     PID_OKb[2] = (uint32)Debug_Kp_Right;
//                     PID_OKb[3] = (uint32)(Debug_Ki_Right * 100.0f);
//                     flash_erase_page(0, 1);
//                     flash_write_page(0, 1, PID_OKb, 13);
//                 }
//                 else if (key == 0)  // 停止 �?回页1
//                 {
//                     Debug_Motor_Enable = 0;
//                     OLED_CLS();
//                 }
//             }
//         }
//         else if (Debug_Sub_Mode == Debug_Sub_Ground_Test)
//         {
//             int32 key = KEY_BLANK;
//             int32 edit_value = 0;

//             if (Debug_Motor_Enable == 0)
//             {
//                 int32 cur_val[8];
//                 int i;

//                 OLED_CLS();
//                 cur_val[0] = (int)Debug_Kp_Left;
//                 cur_val[1] = (int)(Debug_Ki_Left * 100.0f);
//                 cur_val[2] = (int)Debug_Kp_Right;
//                 cur_val[3] = (int)(Debug_Ki_Right * 100.0f);
//                 cur_val[4] = Debug_Target_Speed;
//                 cur_val[5] = Debug_Fan_Duty;
//                 cur_val[6] = Debug_Ground_Dir;

//                 OLED_Show_Str(0, 0, "L_P", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "L_I", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "R_P", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
//                 OLED_Show_Str(0, 3, "R_I", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 3, cur_val[3], TextSize_F6x8);
//                 OLED_Show_Str(0, 4, "Spd", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 4, cur_val[4], TextSize_F6x8);
//                 OLED_Show_Str(0, 5, "Fan", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 5, cur_val[5], TextSize_F6x8);
//                 OLED_Show_Str(0, 6, "Dir", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 6, cur_val[6], TextSize_F6x8);
//                 for (i = 0; i < 7; i++)
//                 {
//                     edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
//                     if (edit_value != 0) cur_val[i] = edit_value;
//                 }

//                 Debug_Kp_Left = cur_val[0];
//                 Debug_Ki_Left = cur_val[1] * 0.01f;
//                 Debug_Kp_Right = cur_val[2];
//                 Debug_Ki_Right = cur_val[3] * 0.01f;
//                 Debug_Target_Speed = cur_val[4];
//                 Debug_Fan_Duty = cur_val[5];
//                 Debug_Ground_Dir = (cur_val[6] == 2) ? 2 : 1;

//                 Debug_Motor_Enable = 1;
//                 PID_cleardata(&Left_PID);
//                 PID_cleardata(&Right_PID);
//                 OLED_CLS();
//             }
//             else
//             {
//                 OLED_Show_Str(0, 0, "Ground", TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "LReal", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 1, Left_Real_Spd, TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "RReal", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 2, Right_Real_Spd, TextSize_F6x8);
//                 OLED_Show_Str(0, 3, "Fan", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 3, Debug_Fan_Duty, TextSize_F6x8);
//                 OLED_Show_Str(0, 4, "Dir", TextSize_F6x8);
//                 OLED_Show_Str(45, 4, Debug_Ground_Dir == 2 ? "L-/R+" : "L+/R-", TextSize_F6x8);
//                 OLED_Show_Str(0, 5, "1.Save", TextSize_F6x8);
//                 OLED_Show_Str(0, 6, "0.Stop", TextSize_F6x8);

//                 key = CH455_GetOneKey();
//                 if (key > 0x0F && key != KEY_BLANK)
//                     key = (key - 0x0F) >> 4;

//                 if (key == 1)
//                 {
//                     PID_OKb[0] = (uint32)Debug_Kp_Left;
//                     PID_OKb[1] = (uint32)(Debug_Ki_Left * 100.0f);
//                     PID_OKb[2] = (uint32)Debug_Kp_Right;
//                     PID_OKb[3] = (uint32)(Debug_Ki_Right * 100.0f);
//                     DBG_OKb[0] = (uint32)Debug_Target_Speed;
//                     DBG_OKb[1] = (uint32)Debug_Fan_Duty;
//                     DBG_OKb[2] = (uint32)Debug_Ground_Dir;
//                     DBG_OKb[3] = (uint32)Debug_Angle_Mode;
//                     flash_erase_page(0, 0);
//                     flash_write_page(0, 0, Speed_OKb, 1);
//                     flash_erase_page(0, 1);
//                     flash_write_page(0, 1, PID_OKb, 13);
//                     flash_erase_page(0, 3);
//                     flash_write_page(0, 3, DBG_OKb, 4);
//                 }
//                 else if (key == 0)
//                 {
//                     Debug_Motor_Enable = 0;
//                     OLED_CLS();
//                 }
//             }
//         }
//         else if (Debug_Sub_Mode == Debug_Sub_Angle)
//         {
//             int32 key = KEY_BLANK;
//             int32 edit_value = 0;

//             if (Debug_Motor_Enable == 0)
//             {
//                 int32 cur_val[8];
//                 int i;

//                 OLED_CLS();
//                 cur_val[0] = (int)(Angle_PID.kp * 100.0f);
//                 cur_val[1] = (int)(Angle_PID.kd * 100.0f);
//                 cur_val[2] = (int)(Gyro_PID.kp * 1000.0f);
//                 cur_val[3] = (int)(Gyro_PID.ki * 1000.0f);
//                 cur_val[4] = (int)(Gyro_PID.kd * 1000.0f);
//                 cur_val[5] = Debug_Angle_Mode;
//                 cur_val[6] = Debug_Target_Speed;
//                 cur_val[7] = 0;

//                 OLED_Show_Str(0, 0, "A_P", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "A_D", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "GI_P", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
//                 OLED_Show_Str(0, 3, "GI_I", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 3, cur_val[3], TextSize_F6x8);
//                 OLED_Show_Str(0, 4, "GI_D", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 4, cur_val[4], TextSize_F6x8);
//                 OLED_Show_Str(0, 5, "Mode", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 5, cur_val[5], TextSize_F6x8);
//                 OLED_Show_Str(0, 6, "Spd", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 6, cur_val[6], TextSize_F6x8);

//                 for (i = 0; i < 7; i++)
//                 {
//                     edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
//                     if (edit_value != 0) cur_val[i] = edit_value;
//                 }

//                 Angle_PID.kp = cur_val[0] * 0.01f;
//                 Angle_PID.kd = cur_val[1] * 0.01f;
//                 Angle_PID.ki = 0;
//                 Angle_PID.mode = PID_MODE_POSITION_D_ON_MEASUREMENT;
//                 Gyro_PID.kp = cur_val[2] * 0.001f;
//                 Gyro_PID.ki = cur_val[3] * 0.001f;
//                 Gyro_PID.kd = cur_val[4] * 0.001f;
//                 Gyro_PID.mode = PID_MODE_ADD;
//                 Debug_Angle_Mode = (cur_val[5] == 2) ? 2 : 1;
//                 Debug_Target_Speed = (cur_val[6] == 1) ? 0 : cur_val[6];

//                 Debug_Motor_Enable = 1;
//                 Gyro_Integral = 0;
//                 Debug_Angle_D_First = 0;
//                 PID_cleardata(&Angle_PID);
//                 PID_cleardata(&Turn_PID);
//                 PID_cleardata(&Gyro_PID);
//                 PID_cleardata(&Gyro_PD_PID);
//                 PID_cleardata(&Left_PID);
//                 PID_cleardata(&Right_PID);
//                 OLED_CLS();
//             }
//             else
//             {
//                 OLED_Show_Str(0, 0, "Angle", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 0, (int)Gyro_Integral, TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "Out", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 1, (int)Gyro_PID_Out, TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "Mode", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 2, Debug_Angle_Mode, TextSize_F6x8);
//                 OLED_Show_Str(0, 3, "D1st", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 3, Debug_Angle_D_First, TextSize_F6x8);
//                 OLED_Show_Str(0, 4, "Fan", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 4, Debug_Fan_Duty, TextSize_F6x8);
//                 OLED_Show_Str(0, 5, "1.Save", TextSize_F6x8);
//                 OLED_Show_Str(0, 6, "0.Stop", TextSize_F6x8);

//                 key = CH455_GetOneKey();
//                 if (key > 0x0F && key != KEY_BLANK)
//                     key = (key - 0x0F) >> 4;

//                 if (key == 1)
//                 {
//                     PID_OKb[4] = (uint32)(Angle_PID.kp * 100.0f);
//                     PID_OKb[5] = (uint32)(Angle_PID.kd * 100.0f);
//                     PID_OKb[6] = (uint32)(Gyro_PID.kp * 1000.0f);
//                     PID_OKb[7] = (uint32)(Gyro_PID.ki * 1000.0f);
//                     PID_OKb[10] = (uint32)(Gyro_PID.kd * 1000.0f);
//                     DBG_OKb[0] = (uint32)Debug_Target_Speed;
//                     DBG_OKb[1] = (uint32)Debug_Fan_Duty;
//                     DBG_OKb[3] = (uint32)Debug_Angle_Mode;
//                     flash_erase_page(0, 1);
//                     flash_write_page(0, 1, PID_OKb, 13);
//                     flash_erase_page(0, 3);
//                     flash_write_page(0, 3, DBG_OKb, 4);
//                 }
//                 else if (key == 0)
//                 {
//                     Debug_Motor_Enable = 0;
//                     OLED_CLS();
//                 }
//             }
//         }

//         else if (Debug_Sub_Mode == Debug_Sub_NormalTrace)
//         {
//             int32 key = KEY_BLANK;
//             int32 edit_value = 0;

//             if (Debug_Motor_Enable == 0)
//             {
//                 int32 cur_val[8];
//                 int i;

//                 OLED_CLS();
//                 cur_val[0] = (int)(Angle_PID.kp * 100.0f);
//                 cur_val[1] = (int)(Angle_PID.kd * 100.0f);
//                 cur_val[2] = (int)(Gyro_PD_PID.kp * 1000.0f);
//                 cur_val[3] = (int)(Gyro_PD_PID.kd * 1000.0f);
//                 cur_val[4] = Debug_Target_Speed;
//                 cur_val[5] = 0;
//                 cur_val[6] = 0;
//                 cur_val[7] = 0;

//                 OLED_Show_Str(0, 0, "T_P", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "T_D", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
//                 OLED_Show_Str(0, 2, "GD_P", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
//                 OLED_Show_Str(0, 3, "GD_D", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 3, cur_val[3], TextSize_F6x8);
//                 OLED_Show_Str(0, 4, "Spd", TextSize_F6x8);
//                 OLED_Show_Numbers(40, 4, cur_val[4], TextSize_F6x8);

//                 for (i = 0; i < 5; i++)
//                 {
//                     edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
//                     if (edit_value != 0) cur_val[i] = edit_value;
//                 }

//                 Angle_PID.kp = cur_val[0] * 0.01f;
//                 Angle_PID.kd = cur_val[1] * 0.01f;
//                 Angle_PID.ki = 0;
//                 Angle_PID.mode = PID_MODE_POSITION_D_ON_MEASUREMENT;
//                 Gyro_PD_PID.kp = cur_val[2] * 0.001f;
//                 Gyro_PD_PID.kd = cur_val[3] * 0.001f;
//                 Gyro_PD_PID.ki = 0;
//                 Gyro_PD_PID.mode = PID_MODE_POSITION;
//                 Debug_Target_Speed = (cur_val[4] == 1) ? 0 : cur_val[4];

//                 Debug_Motor_Enable = 1;
//                 Gyro_Integral = 0;
//                 Debug_Angle_D_First = 0;
//                 PID_cleardata(&Angle_PID);
//                 PID_cleardata(&Turn_PID);
//                 PID_cleardata(&Gyro_PID);
//                 PID_cleardata(&Gyro_PD_PID);
//                 PID_cleardata(&Left_PID);
//                 PID_cleardata(&Right_PID);
//                 OLED_CLS();
//             }
//             else
//             {
//                 OLED_Show_Str(0, 0, "NTrace", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 0, (int)Gyro_Integral, TextSize_F6x8);
//                 OLED_Show_Str(0, 1, "Out", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 1, (int)Gyro_PID_Out, TextSize_F6x8);
//                 OLED_Show_Str(0, 4, "Fan", TextSize_F6x8);
//                 OLED_Show_Numbers(45, 4, Debug_Fan_Duty, TextSize_F6x8);
//                 OLED_Show_Str(0, 5, "1.Save", TextSize_F6x8);
//                 OLED_Show_Str(0, 6, "0.Stop", TextSize_F6x8);

//                 key = CH455_GetOneKey();
//                 if (key > 0x0F && key != KEY_BLANK)
//                     key = (key - 0x0F) >> 4;

//                 if (key == 1)
//                 {
//                     PID_OKb[4] = (uint32)(Angle_PID.kp * 100.0f);
//                     PID_OKb[5] = (uint32)(Angle_PID.kd * 100.0f);
//                     PID_OKb[8] = (uint32)(Gyro_PD_PID.kp * 1000.0f);
//                     PID_OKb[9] = (uint32)(Gyro_PD_PID.kd * 1000.0f);
//                     DBG_OKb[0] = (uint32)Debug_Target_Speed;
//                     DBG_OKb[1] = (uint32)Debug_Fan_Duty;
//                     DBG_OKb[2] = (uint32)Debug_Ground_Dir;
//                     DBG_OKb[3] = (uint32)Debug_Angle_Mode;
//                     flash_erase_page(0, 1);
//                     flash_write_page(0, 1, PID_OKb, 13);
//                     flash_erase_page(0, 3);
//                     flash_write_page(0, 3, DBG_OKb, 4);
//                 }
//                 else if (key == 0)
//                 {
//                     Debug_Motor_Enable = 0;
//                     OLED_CLS();
//                 }
//             }
//         }

//         else  // 其他调试子模式（预留�?
//         {
//             OLED_Show_Str(20, 0, "Debug Idle", TextSize_F6x8);
//             OLED_Show_Str(0, 4, "TODO", TextSize_F6x8);
//         }
//         return;
//     }

//     // 原有建图/回放模式显示
//     OLED_Show_Str(20, 0, "Nothing or Best.", TextSize_F6x8);

//     OLED_Show_Str(0, 2, "L_Spd", TextSize_F6x8);
//     OLED_Show_Numbers(77, 2, Left_Exp_Spd, TextSize_F6x8);

//     OLED_Show_Str(0, 4, "R_Spd", TextSize_F6x8);
//     OLED_Show_Numbers(77, 4, Right_Exp_Spd, TextSize_F6x8);

//     OLED_Show_Str(0, 6, "Err", TextSize_F6x8);
//     OLED_Show_Numbers(77, 6, Error, TextSize_F6x8);

//     OLED_Show_Light_Row();  // 显示光敏传感器状�?
// }
