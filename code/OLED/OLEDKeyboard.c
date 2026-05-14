/*************************************************
Copyright (C), 2016-2024, TYUT JBD .
File name: OLEDKeyboard.c
Author: TYUT JBD
Version:2.0               Date: 2024.12.07
Description:  OLED键盘+OLED显示驱动，实现赛道地图编辑、参数配置、数据存储功能
Others:      依赖CH455键盘、OLED显示屏、Flash存储、PID控制模块
Function List:
History:
<author>     <time>      <version>      <desc>
JBD          2016.10.21     0.0        初始版本
AmaZzzing    2016.11.12     1.0        预赛赛道配置完成
SUV          2024.12.07     2.0        基于新库重构
**************************************************/

#include "OLEDKeyboard.h"
#include "Ctrl.h"

/*
 * 键盘显示模块核心功能：
 * 1. 通过 CH455 键盘 + OLED 屏幕 输入/编辑赛道地图数据
 * 2. 将输入结果转换为 Run_Track 结构体，并写入 Flash 掉电保存
 * 3. 内置默认地图，开机可一键加载
 */

/*===============================================================================
  全局变量定义
================================================================================*/
uint32 input = 0;                                     // 通用输入缓存
uint32 Speed_OKb[1] = {0};                            // 基础速度参数存储
uint32 PID_OKb[8] = {0};                              // PID参数存储数组
uint32 Ctrl_OKb[6] = {30, 120, 180, 40, 85, 40};      // [0]Turn_Error [1]Mileage_Prep [2]Node_Prep [3]Speed_Min [4]Speed_Max [5]Remember_Turn_Error
uint32 DBG_OKb[3] = {0};                              // 调试参数存储

// 键盘回放模式输入数据缓存
uint8 Flash_Node_Num = 0;                             // Flash存储的赛道节点总数
uint8 Flash_Node_Dir[NODE_NUM_MAX] = {0};             // 各节点行驶方向
uint8 Flash_Node_Mileage_Num[NODE_NUM_MAX + 1] = {0};  // 各节点间里程计数量
uint8 mileage_dir[NODE_NUM_MAX + 1][ELEMENT_NUM_MAX] = {{0}};  // 各段里程行驶方向

/*===============================================================================
  Flash 存储相关宏定义
================================================================================*/
/*
 * 地图Flash分区分配说明：
 * 共1页：仅存储键盘输入的原始地图数据
 * 读取后与 Pre_Contest_1 结合，重建完整 Run_Track 运行赛道结构体
 */
#define BUILD_MAP_FLASH_SECTOR        0                 // Flash 第0扇区：当前工程参数/地图存储区
#define BUILD_MAP_FLASH_START_PAGE    4                 // 地图数据起始页：第4页专门存键盘输入原始赛道数据
#define BUILD_MAP_FLASH_WORDS_PER_PAGE 64               // 每页64个uint32，与flash_write_page页宽一致
#define BUILD_MAP_FLASH_PAGE_COUNT    1                 // 地图数据当前仅占1页
#define BUILD_MAP_FLASH_WORD_COUNT    (BUILD_MAP_FLASH_WORDS_PER_PAGE * BUILD_MAP_FLASH_PAGE_COUNT) // 地图数据总长度

/*===============================================================================
  Flash 地图存储结构体
================================================================================*/
// 仅存储建图时真实输入的原始数据，不存储预设地图默认值
typedef struct
{
    uint8 Node_Num;                                   // 节点总数
    uint8 Node_Arr_Dir[NODE_NUM_MAX];                 // 节点方向数组
    uint8 Node_Arr_Mileage_Num[NODE_NUM_MAX + 1];     // 各段里程数量
    uint8 Node_Arr_Mileage_Dir[NODE_NUM_MAX + 1][ELEMENT_NUM_MAX]; // 各段里程方向
} Build_Map_Flash_Typedef;

/*===============================================================================
  内部函数声明
================================================================================*/
static void OLED_Apply_Build_Mode_To_RunTrack(void);
static void OLED_Save_Build_Mode_Map_To_Flash(void);
static void OLED_Load_Build_Mode_Map_From_Flash(void);
static void OLED_Build_Mode_Input(void);

/*===============================================================================
  默认赛道地图参数
================================================================================*/
// 默认地图：第0段是“起点 -> 第1个节点”，0表示该段无元素
#define DEFAULT_BUILD_MAP_NODE_NUM    11

static const uint8 Default_Build_Map_Node_Dir[DEFAULT_BUILD_MAP_NODE_NUM] = {1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1};
static const uint8 Default_Build_Map_Mileage_Num[DEFAULT_BUILD_MAP_NODE_NUM + 1] = {0, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0};

// 每行元素类型定义：1=左转 2=右转 3=短直行 4=长直行
static const uint8 Default_Build_Map_Mileage_Dir[DEFAULT_BUILD_MAP_NODE_NUM + 1][ELEMENT_NUM_MAX] =
{
    {0},
    {4},
    {3, 3},
    {0},
    {4},
    {0},
    {3},
    {4},
    {1},
    {3},
    {0},
    {0}
};

/*===============================================================================
  函数名称：OLED_Show_Digit_Buffer
  函数功能：将数字数组转换为字符串并在OLED显示
  入口参数：x,y-显示坐标  digits-数字数组  len-数字长度
================================================================================*/
static void OLED_Show_Digit_Buffer(uint16 x, uint16 y, uint8 digits[], uint8 len)
{
    uint8 show_str[NODE_NUM_MAX + 1] = {0};  // 显示字符串缓存，末尾留1位放结束符
    uint8 i;

    // 逐位将数字转为ASCII字符
    for (i = 0; i < len && i < NODE_NUM_MAX; i++)
    {
        show_str[i] = digits[i] + '0';
    }
    show_str[i] = '\0';  // 添加字符串结束符

    OLED_Show_Str(x, y, show_str, TextSize_F6x8);
}

/*===============================================================================
  函数名称：OLED_Load_Default_Build_Mode_Map
  函数功能：加载默认地图到建图缓存
================================================================================*/
static void OLED_Load_Default_Build_Mode_Map(void)
{
    uint8 row;
    uint8 i;

    // 加载节点数量与方向
    Flash_Node_Num = DEFAULT_BUILD_MAP_NODE_NUM;
    for (i = 0; i < Flash_Node_Num; i++)
    {
        Flash_Node_Dir[i] = Default_Build_Map_Node_Dir[i];
    }

    // 加载各段里程数量与方向
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = Default_Build_Map_Mileage_Num[row];

        for (i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            mileage_dir[row][i] = Default_Build_Map_Mileage_Dir[row][i];
        }
    }

    // 应用到运行赛道并保存到Flash
    OLED_Apply_Build_Mode_To_RunTrack();
    OLED_Save_Build_Mode_Map_To_Flash();
}

/*===============================================================================
  函数名称：OLED_Read_Digit_Line
  函数功能：单行数字输入，支持退格、确认
  返 回 值：实际输入长度
================================================================================*/
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
            OLED_Show_Str(20, 0, "0node 1left 2right", TextSize_F6x8);
            OLED_Show_Str(20, 1, "3d=1000 4dd=1400", TextSize_F6x8);
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

        // 显示已输入内容
        OLED_Show_Digit_Buffer(input_x, input_y, digits, len);

        // 获取按键
        key_value = CH455_GetOneKey();

        // 长按转普通键值
        if (key_value > 0x0F && key_value != KEY_BLANK)
        {
            key_value = (key_value - 0x0F) >> 4;
        }

        if (key_value == KEY_BLANK) continue;

        // 确认键：退出输入
        if (key_value == KEY_ENTER) break;

        // 退格键：删除最后一位
        if (key_value == KEY_BACK)
        {
            if (len > 0) len--;
            continue;
        }

        // 数字键：添加到缓存
        if (key_value <= KEY_9 && len < max_len)
        {
            digits[len] = (uint8)key_value;
            len++;
        }
    }

    return len;
}

/*===============================================================================
  函数名称：OLED_Apply_Build_Mode_To_RunTrack
  函数功能：将建图缓存数据应用到运行赛道结构体 Run_Track
================================================================================*/
static void OLED_Apply_Build_Mode_To_RunTrack(void)
{
    uint8 row;
    uint8 i;
    uint8 total_element_num = 0;

    // 以预赛地图为基础，未定义字段保留原值
    Run_Track = Pre_Contest_1;

    // 更新节点数量与方向
    Run_Track.Node_Num = Flash_Node_Num;
    for (i = 0; i < Flash_Node_Num; i++)
    {
        Run_Track.Node_Arr_Dir[i] = Flash_Node_Dir[i];
    }

    // 更新各段里程数据
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Run_Track.Node_Arr_Mileage_Num[row] = Flash_Node_Mileage_Num[row];
        total_element_num += Flash_Node_Mileage_Num[row];

        for (i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            /*
             * 类型映射规则：
             * 1=左转  2=右转  里程=2000
             * 3=短直行 里程=1000
             * 4=长直行 里程=1400
             * 0=无元素
             */
            switch (mileage_dir[row][i])
            {
                case 1:
                    Run_Track.Node_Arr_Mileage_Dir[row][i] = 1;
                    Run_Track.Node_Arr_Mileage_Element[row][i] = 2000;
                    break;
                case 2:
                    Run_Track.Node_Arr_Mileage_Dir[row][i] = 2;
                    Run_Track.Node_Arr_Mileage_Element[row][i] = 2000;
                    break;
                case 3:
                    Run_Track.Node_Arr_Mileage_Dir[row][i] = 3;
                    Run_Track.Node_Arr_Mileage_Element[row][i] = 1400;
                    break;
                case 4:
                    Run_Track.Node_Arr_Mileage_Dir[row][i] = 4;
                    Run_Track.Node_Arr_Mileage_Element[row][i] = 1800;
                    break;
                default:
                    break;
            }
        }
    }

    // 总元素数量
    Run_Track.Element_Num = total_element_num;
}

/*===============================================================================
  函数名称：OLED_Save_Build_Mode_Map_To_Flash
  函数功能：将当前建图数据分页保存到Flash
================================================================================*/
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

/*===============================================================================
  函数名称：OLED_Load_Build_Mode_Map_From_Flash
  函数功能：从Flash加载已保存的地图
================================================================================*/
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

    // 恢复到输入缓存
    Flash_Node_Num = flash_map.Node_Num;
    for (uint8 i = 0; i < Flash_Node_Num; i++)
        Flash_Node_Dir[i] = flash_map.Node_Arr_Dir[i];

    for (uint8 row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = flash_map.Node_Arr_Mileage_Num[row];
        for (uint8 i = 0; i < Flash_Node_Mileage_Num[row]; i++)
            mileage_dir[row][i] = flash_map.Node_Arr_Mileage_Dir[row][i];
    }

    // 应用到运行赛道
    OLED_Apply_Build_Mode_To_RunTrack();
}

/*===============================================================================
  函数名称：OLED_Build_Mode_Input
  函数功能：地图编辑模式输入入口
================================================================================*/
static void OLED_Build_Mode_Input(void)
{
    uint8 node_digits[NODE_NUM_MAX] = {0};
    uint8 line_digits[ELEMENT_NUM_MAX] = {0};
    uint8 row;
    int32 map_choose;

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
        Flash_Node_Mileage_Num[i] = 0;
    }
    Flash_Node_Mileage_Num[NODE_NUM_MAX] = 0;

    for (row = 0; row < NODE_NUM_MAX + 1; row++)
        for (uint8 i = 0; i < ELEMENT_NUM_MAX; i++)
            mileage_dir[row][i] = 0;

    // 输入节点数
    Flash_Node_Num = OLED_Read_Digit_Line("Nodes:", 0, 0, 0, 0, 3, node_digits, NODE_NUM_MAX);
    for (uint8 i = 0; i < Flash_Node_Num; i++)
        Flash_Node_Dir[i] = node_digits[i];

    OLED_CLS();

    // 逐行输入里程方向
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = OLED_Read_Digit_Line("Mileage", 1, 1, row, 0, 5, line_digits, ELEMENT_NUM_MAX);
        for (uint8 i = 0; i < Flash_Node_Mileage_Num[row]; i++)
            mileage_dir[row][i] = line_digits[i];
        OLED_CLS();
    }

    // 应用并保存
    OLED_Apply_Build_Mode_To_RunTrack();
    OLED_Save_Build_Mode_Map_To_Flash();
}

/*===============================================================================
  函数名称：OLED_Show_Light_Row
  函数功能：在OLED最后一行显示光敏传感器状态（0/1字符串）
================================================================================*/
static void OLED_Show_Light_Row(void)
{
    uint8 light_str[16] = {0};
    for (uint8 i = 0; i < 15; i++)
    {
        light_str[i] = Light_Convert[i] ? '1' : '0';
    }
    OLED_Show_Str(0, 7, light_str, TextSize_F6x8);
}

/*===============================================================================
  函数名称：OLED_View_Mileage_Data
  函数功能：OLED查看里程数据（不调参不跑车）
  布局    ：P0=摘要(F8x16大字)  P1+=逐段(含空段)  P尾=转弯间距(3个/页)
================================================================================*/
static void OLED_View_Mileage_Data(void)
{
    uint16 seg, elem, ele_num, total_ele, tr_page, tr_pages;
    uint8  node_dir, edir, row;
    char   buf[22];
    const char *dname;

    OLED_Load_Build_Mode_Map_From_Flash();
    Load_All_Flash_Data_For_VOFA();

    // 统计元素总数
    total_ele = 0;
    for (seg = 0; seg <= Run_Track.Node_Num; seg++)
        total_ele += Run_Track.Node_Arr_Mileage_Num[seg];

    //===== Page 0: 摘要 (F8x16 大字) =====
    OLED_CLS();
    sprintf(buf, "Nodes:%d", Run_Track.Node_Num);
    OLED_Show_Str(0, 0, buf, TextSize_F8x16);
    sprintf(buf, "Segs:%d", Run_Track.Node_Num + 1);
    OLED_Show_Str(64, 0, buf, TextSize_F8x16);

    sprintf(buf, "Turns:%d", Turn_Mileage_Record_Num);
    OLED_Show_Str(0, 2, buf, TextSize_F8x16);
    sprintf(buf, "Elems:%d", total_ele);
    OLED_Show_Str(64, 2, buf, TextSize_F8x16);

    sprintf(buf, "Stop:%s", Run_Track.Stop_Mode ? "par" : "ser");
    OLED_Show_Str(0, 4, buf, TextSize_F8x16);
    CH455_GetOneKey();

    //===== 逐段显示：所有段，一页一段 =====
    for (seg = 0; seg <= Run_Track.Node_Num && seg < NODE_NUM_MAX; seg++)
    {
        ele_num = Run_Track.Node_Arr_Mileage_Num[seg];
        OLED_CLS();

        // Row 0: 段标题 F8x16 + 节点方向
        if (seg < Run_Track.Node_Num)
        {
            node_dir = Run_Track.Node_Arr_Dir[seg];
            dname = (node_dir == 0) ? "Straight" : (node_dir == 1) ? "Left90" :
                    (node_dir == 2) ? "Right90" : "?";
            sprintf(buf, "Seg%d/%d", seg, Run_Track.Node_Num);
        }
        else
        {
            dname = "End";
            sprintf(buf, "Seg%d/%d", seg, Run_Track.Node_Num);
        }
        OLED_Show_Str(0, 0, buf, TextSize_F8x16);
        OLED_Show_Str(56, 0, dname, TextSize_F8x16);

        // Rows 2+: 元素详情 (F6x8)
        if (ele_num == 0)
        {
            // 无元器件段：显示实测总里程
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
                edir = Run_Track.Node_Arr_Mileage_Dir[seg][elem];
                if (edir == 0) continue;  // 跳过普通路段，不显示

                switch (edir)
                {
                    case 1: dname = "left";   break;
                    case 2: dname = "right";  break;
                    case 3: dname = "short";  break;
                    case 4: dname = "long";   break;
                    default: dname = "?";     break;
                }
                sprintf(buf, "E%d %s @%.0f +%d", elem, dname,
                        Segment_Edge_Mileage_Record[seg][elem],
                        Run_Track.Node_Arr_Mileage_Element[seg][elem]);
                OLED_Show_Str(1, row, buf, TextSize_F6x8);
                row++;
            }

            // 实测总里程 = 该段起点到接触下一节点的距离
            if (row <= 7 && Segment_Total_Mileage[seg] > 0)
            {
                sprintf(buf, "total:%.0f", Segment_Total_Mileage[seg]);
                OLED_Show_Str(0, row, buf, TextSize_F6x8);
                row++;
            }
        }
        CH455_GetOneKey();
    }

    //===== 转弯间距总览：每页3个 =====
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
}

/*===============================================================================
  函数名称：OLED_Input
  函数功能：键盘输入总入口：模式选择、地图编辑、参数配置
================================================================================*/
void OLED_Input(void)
{
    CH455_Init(); // 初始化CH455键盘(IIC通信)

    int32 OLED_Choose;
    uint8 mode_selected = 0;  // 模式已选择标志（1/2=跑车, 8=仅查看后回菜单）

    while (!mode_selected)
    {
        OLED_Show_Str(20, 3, "Nothing or Best.", TextSize_F6x8);
        OLED_Choose = KeyboardInput(88, 6, TextSize_F8x16, 1.0);

        //====== 模式选择 ======
        switch (OLED_Choose)
        {
            case 1:   // 预赛---建图模式
                Mode = Build_Mode;
                mode_selected = 1;
                break;
            case 2:   // 预赛---回放模式
                Mode = Remember_Mode;
                mode_selected = 1;
                break;
            case 8:   // 查看里程数据模式（不调参不跑车，仅OLED显示）
                OLED_View_Mileage_Data();
                break;
        }
        OLED_CLS();
    }

    // 进入建图模式
    if (Mode == Build_Mode)
    {
        OLED_Build_Mode_Input();
    }

    // 进入回放模式（从Flash加载地图）
    if (Mode == Remember_Mode)
    {
        OLED_Load_Build_Mode_Map_From_Flash();
    }

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
            flash_read_page(0, 1, PID_OKb, 8);
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
            OLED_Show_Str(0, 2, "T_D", TextSize_F6x8);
            OLED_Show_Numbers(47, 2, PID_OKb[5], TextSize_F6x8);
            OLED_Show_Str(0, 4, "G_P", TextSize_F6x8);
            OLED_Show_Numbers(47, 4, PID_OKb[6], TextSize_F6x8);
            OLED_Show_Str(0, 6, "G_D", TextSize_F6x8);
            OLED_Show_Numbers(47, 6, PID_OKb[7], TextSize_F6x8);

            for (uint16 i = 4; i < 8; i++)
            {
                input = KeyboardInput(90, 2*(i-4), TextSize_F6x8, 1.0);
                if (input != 0) PID_OKb[i] = input;
            }
            OLED_CLS();

            // 保存PID到Flash
            flash_erase_page(0, 1);
            flash_write_page(0, 1, PID_OKb, 8);

            // 读取并配置控制参数
            flash_read_page(0, 3, Ctrl_OKb, 6);
            OLED_Show_Str(0, 0, "Turn_E", TextSize_F6x8);
            OLED_Show_Numbers(47, 0, Ctrl_OKb[0], TextSize_F6x8);
            OLED_Show_Str(0, 1, "R_Mile", TextSize_F6x8);
            OLED_Show_Numbers(47, 1, Ctrl_OKb[1], TextSize_F6x8);
            OLED_Show_Str(0, 2, "R_Node", TextSize_F6x8);
            OLED_Show_Numbers(47, 2, Ctrl_OKb[2], TextSize_F6x8);
            OLED_Show_Str(0, 3, "S_Min", TextSize_F6x8);
            OLED_Show_Numbers(47, 3, Ctrl_OKb[3], TextSize_F6x8);
            OLED_Show_Str(0, 4, "S_Max", TextSize_F6x8);
            OLED_Show_Numbers(47, 4, Ctrl_OKb[4], TextSize_F6x8);
            OLED_Show_Str(0, 5, "R_Turn", TextSize_F6x8);
            OLED_Show_Numbers(47, 5, Ctrl_OKb[5], TextSize_F6x8);

            for (uint8 i = 0; i < 6; i++)
            {
                input = KeyboardInput(90, i, TextSize_F6x8, 1.0);
                if(input != 0) Ctrl_OKb[i] = input;
            }

            OLED_CLS();
            flash_erase_page(0, 3);
            flash_write_page(0, 3, Ctrl_OKb, 6);
        }
        break;

        case 2: // 调试参数配置
        {
            flash_read_page(0, 2, DBG_OKb, 3);
            OLED_Show_Str(0, 0, "Exp_G", TextSize_F6x8);
            OLED_Show_Numbers(47, 0, DBG_OKb[0], TextSize_F6x8);
            OLED_Show_Str(0, 2, "Exp_L", TextSize_F6x8);
            OLED_Show_Numbers(47, 2, DBG_OKb[1], TextSize_F6x8);
            OLED_Show_Str(0, 4, "Exp_R", TextSize_F6x8);
            OLED_Show_Numbers(47, 4, DBG_OKb[2], TextSize_F6x8);

            for (uint16 i = 0; i < 3; i++)
            {
                input = KeyboardInput(90, 2*i, TextSize_F6x8, 1.0);
                if (input != 0) DBG_OKb[i] = input;
            }
            OLED_CLS();

            flash_erase_page(0, 2);
            flash_write_page(0, 2, DBG_OKb, 3);
        }
        break;
    }
}

/*===============================================================================
  函数名称：OLED_Data_Load
  函数功能：从Flash加载所有配置参数到运行变量
================================================================================*/
void OLED_Data_Load()
{
    // 加载PID参数
    flash_read_page(0, 1, PID_OKb, 8);
    Left_PID.kp = PID_OKb[0];
    Left_PID.ki = PID_OKb[1] * 0.01;
    Right_PID.kp = PID_OKb[2];
    Right_PID.ki = PID_OKb[3] * 0.01;
    Turn_PID.kp = PID_OKb[4] * 0.01;
    Turn_PID.kd = PID_OKb[5] * 0.01;
    Gyro_PID.kp = PID_OKb[6] * 0.001;
    Gyro_PID.kd = PID_OKb[7] * 0.001;

    // 加载基础速度
    flash_read_page(0, 0, Speed_OKb, 1);
    Basic_Speed = Speed_OKb[0];

    // 加载控制参数
    flash_read_page(0, 3, Ctrl_OKb, 6);
    if (Ctrl_OKb[0] != 0) Turn_Error_Value = Ctrl_OKb[0];
    if (Ctrl_OKb[1] != 0) Remember_Mileage_Prepare_Distance = Ctrl_OKb[1];
    if (Ctrl_OKb[2] != 0) Remember_Node_Prepare_Distance = Ctrl_OKb[2];
    if (Ctrl_OKb[3] != 0) Remember_Speed_Min_Value = Ctrl_OKb[3];
    if (Ctrl_OKb[4] != 0) Remember_Speed_Max_Value = Ctrl_OKb[4];
    if (Ctrl_OKb[5] != 0) Remember_Turn_Error = Ctrl_OKb[5];

    // 加载调试参数
    flash_read_page(0, 2, DBG_OKb, 3);
}

/*===============================================================================
  函数名称：OLED_Display
  函数功能：OLED实时显示运行数据
================================================================================*/
void OLED_Display(void)
{
    OLED_Show_Str(20, 0, "Nothing or Best.", TextSize_F6x8);

    OLED_Show_Str(0, 2, "L_Spd", TextSize_F6x8);
    OLED_Show_Numbers(77, 2, Left_Exp_Spd, TextSize_F6x8);

    OLED_Show_Str(0, 4, "R_Spd", TextSize_F6x8);
    OLED_Show_Numbers(77, 4, Right_Exp_Spd, TextSize_F6x8);

    OLED_Show_Str(0, 6, "Err", TextSize_F6x8);
    OLED_Show_Numbers(77, 6, Error, TextSize_F6x8);

    OLED_Show_Light_Row();  // 显示光敏传感器状态
}