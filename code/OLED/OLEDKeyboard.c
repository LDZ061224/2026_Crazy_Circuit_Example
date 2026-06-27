/**
 * @file        OLEDKeyboard.c
 * @author      TYUT JBD
 * @version     2.0
 * @date        2024.12.07
 * @brief       OLED閿洏+OLED鏄剧ず椹卞姩锛屽疄鐜拌禌閬撳湴鍥剧紪杈戙€佸弬鏁伴厤缃€佹暟鎹瓨鍌ㄥ姛鑳?
 * @details     渚濊禆CH455閿洏銆丱LED鏄剧ず灞忋€丗lash瀛樺偍銆丳ID鎺у埗妯″潡
 * @copyright   Copyright (C) 2016-2024, TYUT JBD
 * @history
 *   JBD          2016.10.21     0.0        鍒濆鐗堟湰
 *   AmaZzzing    2016.11.12     1.0        棰勮禌璧涢亾閰嶇疆瀹屾垚
 *   SUV          2024.12.07     2.0        鍩轰簬鏂板簱閲嶆瀯
 */

#include "OLEDKeyboard.h"
#include "Ctrl.h"

/*
 * 閿洏鏄剧ず妯″潡鏍稿績鍔熻兘锛?
 * 1. 閫氳繃 CH455 閿洏 + OLED 灞忓箷 杈撳叆/缂栬緫璧涢亾鍦板浘鏁版嵁
 * 2. 灏嗚緭鍏ョ粨鏋滆浆鎹负 Run_Track 缁撴瀯浣擄紝骞跺啓鍏?Flash 鎺夌數淇濆瓨
 * 3. 鍐呯疆榛樿鍦板浘锛屽紑鏈哄彲涓€閿姞杞?
 */


/*===============================================================================
  鍏ㄥ眬鍙橀噺瀹氫箟
================================================================================*/
uint32 input = 0;                                     // 閫氱敤杈撳叆缂撳瓨
uint32 Speed_OKb[1] = {0};                            // 鍩虹閫熷害鍙傛暟瀛樺偍
uint32 PID_OKb[11] = {0};                             // [4:5]Angle PD [6:8]Gyro PID [9:10]Gyro PD
uint32 Ctrl_OKb[8] = {560, 260, 0, 0, 0, 0, 0, 0};         // [0]ElementTurnDelay [1]NodeTurnDelay
uint32 DBG_OKb[4] = {40, 8000, 1, 1};                              // 璋冭瘯鍙傛暟瀛樺偍

// 閿洏鍥炴斁妯″紡杈撳叆鏁版嵁缂撳瓨
uint8 Flash_Node_Num = 0;                             // Flash瀛樺偍鐨勮禌閬撹妭鐐规€绘暟
uint8 Flash_Node_Dir[NODE_NUM_MAX] = {0};             // 鍚勮妭鐐硅椹舵柟鍚?
uint8 Flash_Node_Mileage_Num[TRACK_SEGMENT_NUM_MAX] = {0};  // Segment element count
uint8 mileage_dir[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX] = {{0}};  // Segment element direction

/*===============================================================================
  Flash 瀛樺偍鐩稿叧瀹忓畾涔?
================================================================================*/
/*
 * 鍦板浘Flash鍒嗗尯鍒嗛厤璇存槑锛?
 * 鍏?椤碉細浠呭瓨鍌ㄩ敭鐩樿緭鍏ョ殑鍘熷鍦板浘鏁版嵁
 * 璇诲彇鍚庝笌 Pre_Contest_1 缁撳悎锛岄噸寤哄畬鏁?Run_Track 杩愯璧涢亾缁撴瀯浣?
 */
#define BUILD_MAP_FLASH_SECTOR        0                 // Flash 绗?鎵囧尯锛氬綋鍓嶅伐绋嬪弬鏁?鍦板浘瀛樺偍鍖?
#define BUILD_MAP_FLASH_START_PAGE    4                 // 鍦板浘鏁版嵁璧峰椤碉細绗?椤典笓闂ㄥ瓨閿洏杈撳叆鍘熷璧涢亾鏁版嵁
#define BUILD_MAP_FLASH_WORDS_PER_PAGE 64               // 姣忛〉64涓猽int32锛屼笌flash_write_page椤靛涓€鑷?
#define BUILD_MAP_FLASH_PAGE_COUNT    1                 // 鍦板浘鏁版嵁褰撳墠浠呭崰1椤?
#define BUILD_MAP_FLASH_WORD_COUNT    (BUILD_MAP_FLASH_WORDS_PER_PAGE * BUILD_MAP_FLASH_PAGE_COUNT) // 鍦板浘鏁版嵁鎬婚暱搴?

/*===============================================================================
  Flash 鍦板浘瀛樺偍缁撴瀯浣?
================================================================================*/
// 浠呭瓨鍌ㄥ缓鍥炬椂鐪熷疄杈撳叆鐨勫師濮嬫暟鎹紝涓嶅瓨鍌ㄩ璁惧湴鍥鹃粯璁ゅ€?
typedef struct
{
    uint8 Node_Num;                                   // 鑺傜偣鎬绘暟
    uint8 Node_Arr_Dir[NODE_NUM_MAX];                 // 鑺傜偣鏂瑰悜鏁扮粍
    uint8 Node_Arr_Mileage_Num[TRACK_SEGMENT_NUM_MAX];     // 鍚勬閲岀▼鏁伴噺
    uint8 Node_Arr_Mileage_Dir[TRACK_SEGMENT_NUM_MAX][ELEMENT_NUM_MAX]; // 鍚勬閲岀▼鏂瑰悜
} Build_Map_Flash_Typedef;

/*===============================================================================
  鍐呴儴鍑芥暟澹版槑
================================================================================*/
static void OLED_Apply_Build_Mode_To_RunTrack(void);
static void OLED_Save_Build_Mode_Map_To_Flash(void);
static void OLED_Load_Build_Mode_Map_From_Flash(void);
static void OLED_Build_Mode_Input(void);

/*===============================================================================
  榛樿璧涢亾鍦板浘鍙傛暟
================================================================================*/
// 榛樿鍦板浘锛氱0娈垫槸鈥滆捣鐐?-> 绗?涓妭鐐光€濓紝0琛ㄧず璇ユ鏃犲厓绱?
#define DEFAULT_BUILD_MAP_NODE_NUM    20

static const uint8 Default_Build_Map_Node_Dir[DEFAULT_BUILD_MAP_NODE_NUM] = {0,0,1,0,1,1,0,1,0,1,0,0,0,1,2,2,1,1,0,1};
static const uint8 Default_Build_Map_Mileage_Num[DEFAULT_BUILD_MAP_NODE_NUM + 1] = {0,1,0,2,0,2,0,1,0,1,0,0,1,1,1,0,1,0,1,0,2};

// 姣忚鍏冪礌绫诲瀷瀹氫箟锛?=宸﹁浆 2=鍙宠浆 3=鐭洿琛?4=闀跨洿琛?
static const uint8 Default_Build_Map_Mileage_Dir[DEFAULT_BUILD_MAP_NODE_NUM + 1][ELEMENT_NUM_MAX] =
{
    {0},
    {3},
    {0},
    {3,3},
    {0},
    {3,3},
    {0},
    {3},
    {0},
    {3},
    {0},
    {0},
    {1},
    {1},
    {3},
    {0},
    {3},
    {0},
    {3},
    {0},
    {3,3}
};
/**
 * @brief   灏嗘暟瀛楁暟缁勮浆鎹负瀛楃涓插苟鍦∣LED鏄剧ず
 * @param   x       鏄剧ず妯潗鏍?
 * @param   y       鏄剧ず绾靛潗鏍?
 * @param   digits  鏁板瓧鏁扮粍
 * @param   len     鏁板瓧闀垮害
 */
static void OLED_Show_Digit_Buffer(uint16 x, uint16 y, uint8 digits[], uint8 len)
{
    uint8 show_str[NODE_NUM_MAX + 1] = {0};  
    uint8 i;

    // 閫愪綅灏嗘暟瀛楄浆涓篈SCII瀛楃
    for (i = 0; i < len && i < NODE_NUM_MAX; i++)
    {
        show_str[i] = digits[i] + '0';
    }
    show_str[i] = '\0';  // 娣诲姞瀛楃涓茬粨鏉熺

    OLED_Show_Str(x, y, show_str, TextSize_F6x8);
}

/**
 * @brief   鍔犺浇榛樿鍦板浘鍒板缓鍥剧紦瀛?
 */
static void OLED_Load_Default_Build_Mode_Map(void)
{
    uint8 row;
    uint8 i;

    // 鍔犺浇鑺傜偣鏁伴噺涓庢柟鍚?
    Flash_Node_Num = DEFAULT_BUILD_MAP_NODE_NUM;
    for (i = 0; i < Flash_Node_Num; i++)
    {
        Flash_Node_Dir[i] = Default_Build_Map_Node_Dir[i];
    }

    // 鍔犺浇鍚勬閲岀▼鏁伴噺涓庢柟鍚?
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = Default_Build_Map_Mileage_Num[row];

        for (i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            mileage_dir[row][i] = Default_Build_Map_Mileage_Dir[row][i];
        }
    }

    OLED_Apply_Build_Mode_To_RunTrack();
    OLED_Save_Build_Mode_Map_To_Flash();
}

/**
 * @brief   鍗曡鏁板瓧杈撳叆锛屾敮鎸侀€€鏍笺€佺‘璁?
 * @param   title         鏍囬瀛楃涓?
 * @param   show_hint     鏄惁鏄剧ず鎻愮ず淇℃伅
 * @param   show_row_index鏄惁鏄剧ず琛岀储寮?
 * @param   row_index     琛岀储寮曞€?
 * @param   input_x       杈撳叆鍖烘í鍧愭爣
 * @param   input_y       杈撳叆鍖虹旱鍧愭爣
 * @param   digits        杈撳嚭鏁板瓧缂撳瓨鏁扮粍
 * @param   max_len       鏈€澶ц緭鍏ラ暱搴?
 * @return  瀹為檯杈撳叆闀垮害
 */
static uint8 OLED_Read_Digit_Line(uint8 title[], uint8 show_hint, uint8 show_row_index, uint8 row_index,
                                  uint16 input_x, uint16 input_y, uint8 digits[], uint8 max_len)
{
    KeyValue_enum key_value = KEY_BLANK;
    uint8 len = 0;

    while (1)
    {
        OLED_CLS();  // 娓呭睆鍒锋柊

        // 鏄剧ず鎻愮ず淇℃伅
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

        // 鏄剧ず宸茶緭鍏ュ唴瀹?
        OLED_Show_Digit_Buffer(input_x, input_y, digits, len);

        // 鑾峰彇鎸夐敭
        key_value = CH455_GetOneKey();

        // 闀挎寜杞櫘閫氶敭鍊?
        if (key_value > 0x0F && key_value != KEY_BLANK)
        {
            key_value = (key_value - 0x0F) >> 4;
        }

        if (key_value == KEY_BLANK) continue;

        // 纭閿細閫€鍑鸿緭鍏?
        if (key_value == KEY_ENTER) break;

        // 閫€鏍奸敭锛氬垹闄ゆ渶鍚庝竴浣?
        if (key_value == KEY_BACK)
        {
            if (len > 0) len--;
            continue;
        }

        // 鏁板瓧閿細娣诲姞鍒扮紦瀛?
        if (key_value <= KEY_9 && len < max_len)
        {
            digits[len] = (uint8)key_value;
            len++;
        }
    }

    return len;
}

/**
 * @brief   灏嗗缓鍥剧紦瀛樻暟鎹簲鐢ㄥ埌杩愯璧涢亾缁撴瀯浣?Run_Track
 */
static void OLED_Apply_Build_Mode_To_RunTrack(void)
{
    uint8 row;
    uint8 i;
    uint8 total_element_num = 0;

    Run_Track = Pre_Contest_1;

    // 鏇存柊鑺傜偣鏁伴噺涓庢柟鍚?
    Run_Track.Node_Num = Flash_Node_Num;
    for (i = 0; i < Flash_Node_Num; i++)
    {
        Run_Track.Node_Arr_Dir[i] = Flash_Node_Dir[i];
    }

    // 鏇存柊鍚勬閲岀▼鏁版嵁
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Run_Track.Node_Arr_Mileage_Num[row] = Flash_Node_Mileage_Num[row];
        total_element_num += Flash_Node_Mileage_Num[row];

        for (i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            /*
             * 1=宸﹁浆  2=鍙宠浆  閲岀▼=2000
             * 3=鐭洿琛?閲岀▼=1000
             * 4=闀跨洿琛?閲岀▼=1400
             * 0=鏃犲厓绱?
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

    // 鎬诲厓绱犳暟閲?
    Run_Track.Element_Num = total_element_num;
}

/**
 * @brief   灏嗗綋鍓嶅缓鍥炬暟鎹垎椤典繚瀛樺埌Flash
 */
static void OLED_Save_Build_Mode_Map_To_Flash(void)
{
    uint32 map_words[BUILD_MAP_FLASH_WORD_COUNT] = {0};
    Build_Map_Flash_Typedef flash_map = {0};
    uint8 page_index;
    uint8 i;
    uint8 row;

    // 鎵撳寘鍘熷杈撳叆鏁版嵁
    flash_map.Node_Num = Flash_Node_Num;
    for (i = 0; i < Flash_Node_Num; i++)
        flash_map.Node_Arr_Dir[i] = Flash_Node_Dir[i];

    for (row = 0; row <= Flash_Node_Num; row++)
    {
        flash_map.Node_Arr_Mileage_Num[row] = Flash_Node_Mileage_Num[row];
        for (i = 0; i < Flash_Node_Mileage_Num[row]; i++)
            flash_map.Node_Arr_Mileage_Dir[row][i] = mileage_dir[row][i];
    }

    // 澶嶅埗鍒皍int32鏁扮粍
    memcpy(map_words, &flash_map, sizeof(flash_map));

    // 鎿﹂櫎骞跺啓鍏lash
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
 * @brief   浠嶧lash鍔犺浇宸蹭繚瀛樼殑鍦板浘
 */
static void OLED_Load_Build_Mode_Map_From_Flash(void)
{
    uint32 map_words[BUILD_MAP_FLASH_WORD_COUNT] = {0};
    Build_Map_Flash_Typedef flash_map = {0};
    uint8 page_index;

    // 璇诲彇Flash鏁版嵁
    for (page_index = 0; page_index < BUILD_MAP_FLASH_PAGE_COUNT; page_index++)
    {
        flash_read_page(BUILD_MAP_FLASH_SECTOR,
                        BUILD_MAP_FLASH_START_PAGE + page_index,
                        &map_words[page_index * BUILD_MAP_FLASH_WORDS_PER_PAGE],
                        BUILD_MAP_FLASH_WORDS_PER_PAGE);
    }

    // 鎭㈠鍒板湴鍥剧粨鏋勪綋
    memcpy(&flash_map, map_words, sizeof(flash_map));

    // 鎭㈠鍒拌緭鍏ョ紦瀛?
    Flash_Node_Num = flash_map.Node_Num;
    if (Flash_Node_Num > NODE_NUM_MAX)        // Flash绌虹墖(0xFF)鎴栨崯鍧忔椂闃茶秺鐣?
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

    // 搴旂敤鍒拌繍琛岃禌閬?
    OLED_Apply_Build_Mode_To_RunTrack();
}

/**
 * @brief   鍦板浘缂栬緫妯″紡杈撳叆鍏ュ彛
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

    // 鍔犺浇榛樿鍦板浘
    if (map_choose == 1)
    {
        OLED_Load_Default_Build_Mode_Map();
        return;
    }

    // 娓呯┖缂撳瓨
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
    // 杈撳叆鑺傜偣鏁?
    Flash_Node_Num = OLED_Read_Digit_Line("Nodes:", 0, 0, 0, 0, 3, node_digits, NODE_NUM_MAX);
    for (uint8 i = 0; i < Flash_Node_Num; i++)
    {
        Flash_Node_Dir[i] = node_digits[i];
    }

    OLED_CLS();

    // 閫愯杈撳叆閲岀▼鏂瑰悜
    for (row = 0; row <= Flash_Node_Num; row++)
    {
        Flash_Node_Mileage_Num[row] = OLED_Read_Digit_Line("Mileage", 1, 1, row, 0, 5, line_digits, ELEMENT_NUM_MAX);
        for (uint8 i = 0; i < Flash_Node_Mileage_Num[row]; i++)
        {
            mileage_dir[row][i] = line_digits[i];
        }
        OLED_CLS();
    }

    // 搴旂敤骞朵繚瀛?
    OLED_Apply_Build_Mode_To_RunTrack();
    OLED_Save_Build_Mode_Map_To_Flash();
}

/**
 * @brief   鍦∣LED鏈€鍚庝竴琛屾樉绀哄厜鏁忎紶鎰熷櫒鐘舵€侊紙0/1瀛楃涓诧級
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
 * @brief   OLED鏌ョ湅閲岀▼鏁版嵁锛堜笉璋冨弬涓嶈窇杞︼級
 * @details 甯冨眬锛歅0=鎽樿(F8x16澶у瓧)銆丳1+=閫愭(鍚┖娈?銆丳灏?杞集闂磋窛(3涓?椤?
 */
static void OLED_View_Mileage_Data(void)
{
    uint16 seg, elem, ele_num, total_ele, tr_page, tr_pages;
    uint8  node_dir, edir, row;
    char   buf[22];
    const char *dname;

    OLED_Load_Build_Mode_Map_From_Flash();
    Load_All_Flash_Data_For_VOFA();

    // 缁熻鍏冪礌鎬绘暟
    total_ele = 0;
    for (seg = 0; seg <= Run_Track.Node_Num; seg++)
        total_ele += Run_Track.Node_Arr_Mileage_Num[seg];

    //===== Page 0: 鎽樿 (F8x16 澶у瓧) =====
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

    //===== 閫愭鏄剧ず锛氭墍鏈夋锛屼竴椤典竴娈?=====
    for (seg = 0; seg <= Run_Track.Node_Num && seg < TRACK_SEGMENT_NUM_MAX; seg++)
    {
        ele_num = Run_Track.Node_Arr_Mileage_Num[seg];
        OLED_CLS();

        // Row 0: 娈垫爣棰?F8x16 + 鑺傜偣鏂瑰悜
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

        // Rows 2+: 鍏冪礌璇︽儏 (F6x8)
        if (ele_num == 0)
        {
            // 鏃犲厓鍣ㄤ欢娈碉細鏄剧ず瀹炴祴鎬婚噷绋?
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
                if (edir == 0) continue;

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

            // 瀹炴祴鎬婚噷绋?= 璇ユ璧风偣鍒版帴瑙︿笅涓€鑺傜偣鐨勮窛绂?
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
}

/**
 * @brief   閿洏杈撳叆鎬诲叆鍙ｏ細妯″紡閫夋嫨銆佸湴鍥剧紪杈戙€佸弬鏁伴厤缃?
 */
void OLED_Input(void)
{
    CH455_Init(); // 鍒濆鍖朇H455閿洏(IIC閫氫俊)

    int32 OLED_Choose;
    uint8 mode_selected = 0;  // 妯″紡宸查€夋嫨鏍囧織锛?/3=璺戣溅, 8=浠呮煡鐪嬪悗鍥炶彍鍗曪級

    while (!mode_selected)
    {
        OLED_Show_Str(20, 3, "Nothing or Best.", TextSize_F6x8);
        OLED_Choose = KeyboardInput(88, 6, TextSize_F8x16, 1.0);

        //====== 妯″紡閫夋嫨 ======
        switch (OLED_Choose)
        {
            case 1:   // 寤哄浘妯″紡
                Mode = Build_Mode;
                mode_selected = 1;
                break;
            case 2:   // 鍥炴斁妯″紡鏆傛椂绂佺敤锛屽悗缁噸鍐?
                OLED_Show_Str(0, 0, "Remember Off", TextSize_F6x8);
                break;
            case 3:   // 璋冭瘯妯″紡
                Mode = Debug_Mode;
                mode_selected = 1;
                break;
            case 8:   // 鏌ョ湅閲岀▼鏁版嵁妯″紡锛堜笉璋冨弬涓嶈窇杞︼紝浠匫LED鏄剧ず锛?
                OLED_View_Mileage_Data();
                break;
        }
        OLED_CLS();
    }

    // 杩涘叆寤哄浘妯″紡
    if (Mode == Build_Mode)
    {
        OLED_Build_Mode_Input();
    }

    // 杩涘叆璋冭瘯妯″紡锛堥€夋嫨瀛愭ā寮?+ 鍔犺浇鍙傛暟锛?
    if (Mode == Debug_Mode)
    {
        int32 dbg_choose;

        //---- 閫夋嫨瀛愭ā寮?----
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

        //---- 浠嶧lash鎭㈠涓婃淇濆瓨鐨勮皟璇曞弬鏁?----
        flash_read_page(0, 1, PID_OKb, 11);
        if (PID_OKb[0] != 0) Debug_Kp_Left  = PID_OKb[0];
        if (PID_OKb[1] != 0) Debug_Ki_Left  = PID_OKb[1] * 0.01;
        if (PID_OKb[2] != 0) Debug_Kp_Right = PID_OKb[2];
        if (PID_OKb[3] != 0) Debug_Ki_Right = PID_OKb[3] * 0.01;
        flash_read_page(0, 3, DBG_OKb, 4);
        if (DBG_OKb[0] != 0) Debug_Target_Speed = DBG_OKb[0];
        if (DBG_OKb[1] != 0) Debug_Fan_Duty = DBG_OKb[1];
        if (DBG_OKb[2] == 1 || DBG_OKb[2] == 2) Debug_Ground_Dir = DBG_OKb[2];
        if (DBG_OKb[3] == 1 || DBG_OKb[3] == 2) Debug_Angle_Mode = DBG_OKb[3];
        // 鐢垫満鍒濆涓哄仠锛岄〉闈负缂栬緫椤?
        Debug_Motor_Enable = 0;

        // 璺宠繃閫熷害/PID鍙傛暟閰嶇疆锛岀洿鎺ュ惎鍔?
        OLED_Data_Load();
        PID_cleardata(&Left_PID);
        PID_cleardata(&Right_PID);
        return;
    }

    //====== 鍩虹閫熷害閰嶇疆 ======
    flash_read_page(0, 0, Speed_OKb, 1);
    OLED_Show_Str(0, 0, "B_Spd", TextSize_F6x8);
    OLED_Show_Numbers(47, 0, Speed_OKb[0], TextSize_F6x8);

    input = KeyboardInput(90, 0, TextSize_F6x8, 1.0);
    if (input != 0)
    {
        Speed_OKb[0] = input;
    }
    OLED_CLS();

    // 鎿﹂櫎骞跺啓鍏ラ€熷害鍙傛暟
    flash_erase_page(0, 0);
    flash_write_page(0, 0, Speed_OKb, 1);

    // 鑿滃崟閫夋嫨
    OLED_Choose = KeyboardInput(88, 6, TextSize_F8x16, 1.0);
    OLED_CLS();

    //====== 鍙傛暟閰嶇疆 ======
    switch (OLED_Choose)
    {
        case 1: // PID鍙傛暟 + 鎺у埗鍙傛暟閰嶇疆
        {
            // 璇诲彇骞堕厤缃乏鍙崇數鏈篜ID
            flash_read_page(0, 1, PID_OKb, 11);
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

            // 杞悜PID + 闄€铻轰华PID
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

            // 淇濆瓨PID鍒癋lash
            flash_erase_page(0, 1);
            flash_write_page(0, 1, PID_OKb, 11);

            // 璇诲彇骞堕厤缃帶鍒跺弬鏁?
            flash_read_page(0, 2, Ctrl_OKb, 8);
            if (Ctrl_OKb[0] == 0 || Ctrl_OKb[0] > 5000) Ctrl_OKb[0] = (uint32)Mileage_Element_Turn_Delay;
            if (Ctrl_OKb[1] == 0 || Ctrl_OKb[1] > 5000) Ctrl_OKb[1] = (uint32)Mileage_Node_Turn_Delay;
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
 * @brief   浠嶧lash鍔犺浇鎵€鏈夐厤缃弬鏁板埌杩愯鍙橀噺
 */
void OLED_Data_Load()
{
//    OLED_Load_Default_Build_Mode_Map();
//    // 鍔犺浇PID鍙傛暟
    flash_read_page(0, 1, PID_OKb, 11);
    if (PID_OKb[0] != 0 && PID_OKb[0] <= 2000) Left_PID.kp = PID_OKb[0];
    if (PID_OKb[1] != 0 && PID_OKb[1] <= 10000) Left_PID.ki = PID_OKb[1] * 0.01f;
    if (PID_OKb[2] != 0 && PID_OKb[2] <= 2000) Right_PID.kp = PID_OKb[2];
    if (PID_OKb[3] != 0 && PID_OKb[3] <= 10000) Right_PID.ki = PID_OKb[3] * 0.01f;
    if (PID_OKb[4] != 0 && PID_OKb[4] <= 20000) Angle_PID.kp = PID_OKb[4] * 0.01f;
    if (PID_OKb[5] != 0 && PID_OKb[5] <= 20000) Angle_PID.kd = PID_OKb[5] * 0.01f;
    Angle_PID.ki = 0;
    Angle_PID.mode = PID_MODE_POSITION_D_ON_MEASUREMENT;
    Turn_PID.kp = Angle_PID.kp;
    Turn_PID.kd = Angle_PID.kd;
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

    // 鍔犺浇鍩虹閫熷害
    flash_read_page(0, 0, Speed_OKb, 1);
    if (Speed_OKb[0] != 0 && Speed_OKb[0] <= 300) Basic_Speed = Speed_OKb[0];

    // 鍔犺浇鎺у埗鍙傛暟
    flash_read_page(0, 2, Ctrl_OKb, 8);
    if (Ctrl_OKb[0] != 0 && Ctrl_OKb[0] <= 5000) Mileage_Element_Turn_Delay = Ctrl_OKb[0];
    if (Ctrl_OKb[1] != 0 && Ctrl_OKb[1] <= 5000) Mileage_Node_Turn_Delay = Ctrl_OKb[1];
}

/**
 * @brief   OLED瀹炴椂鏄剧ず杩愯鏁版嵁 + 璋冭瘯妯″紡浜や簰
 */
void OLED_Display(void)
{
    if (Mode == Debug_Mode)
    {
        //===== 璋冭瘯妯″紡鏄剧ず/浜や簰 =====
        if (Debug_Sub_Mode == Debug_Sub_PI_Tuning)
        {
            int32 key = KEY_BLANK;
            static int32 edit_value = 0;  // 璺ㄥ抚淇濆瓨缂栬緫涓殑鏁板€?

            if (Debug_Motor_Enable == 0)
            {
                // ==== 椤?: 鍙傛暟缂栬緫椤碉紙鍥涜浠庝笂鍒颁笅渚濇杈撳叆锛岃緭瀹岃嚜鍔ㄥ紑鐢垫満锛?====
                int32 cur_val[4];   // 0=Kp, 1=Ki, 2=Spd, 3=Wheel(0宸?鍙?
                int i;

                OLED_CLS();
                cur_val[0] = (int)(Debug_Which_Wheel ? Debug_Kp_Right : Debug_Kp_Left);
                cur_val[1] = (int)((Debug_Which_Wheel ? Debug_Ki_Right : Debug_Ki_Left) * 100.0f);
                cur_val[2] = Debug_Target_Speed;
                cur_val[3] = Debug_Which_Wheel;

                OLED_Show_Str(0, 0, "Kp", TextSize_F6x8);
                OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
                OLED_Show_Str(0, 1, "Ki", TextSize_F6x8);
                OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
                OLED_Show_Str(0, 2, "Spd", TextSize_F6x8);
                OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
                OLED_Show_Str(0, 3, "Wheel", TextSize_F6x8);
                OLED_Show_Str(45, 3, cur_val[3] ? "Right" : "Left", TextSize_F6x8);

                // 鍥涜浠庝笂鍒颁笅渚濇杈撳叆锛?淇濇寔涓嶅彉锛?
                for (i = 0; i < 3; i++)
                {
                    edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
                    if (edit_value != 0) cur_val[i] = edit_value;
                }
                // Wheel: 杈撳叆2=宸? 1=鍙?
                edit_value = KeyboardInput(45, 3, TextSize_F6x8, 1.0);
                if (edit_value == 2) cur_val[3] = 0;
                else if (edit_value == 1) cur_val[3] = 1;

                // 鍥炲啓鍒板叏灞€鍙橀噺
                Debug_Which_Wheel = cur_val[3];
                if (Debug_Which_Wheel == 0)
                {
                    Debug_Kp_Left  = cur_val[0];
                    Debug_Ki_Left  = cur_val[1] * 0.01;
                }
                else
                {
                    Debug_Kp_Right = cur_val[0];
                    Debug_Ki_Right = cur_val[1]*0.01;
                }
                Debug_Target_Speed = cur_val[2];

                // 杈撳叆瀹屾瘯 鈫?鑷姩寮€鐢垫満
                Debug_Motor_Enable = 1;
                PID_cleardata(&Left_PID);
                PID_cleardata(&Right_PID);
                OLED_CLS();
            }
            else  // Debug_Motor_Enable == 1
            {
                // ==== 椤?: 杩愯椤碉紙Real/PWM + 1.Save 0.Stop锛?====
                int real_spd = (Debug_Which_Wheel == 0) ? Left_Real_Spd : Right_Real_Spd;
                int pwm_out  = (Debug_Which_Wheel == 0) ? (int)Left_PID_Out : (int)Right_PID_Out;

                OLED_Show_Str(0, 0, "Run", TextSize_F6x8);
                OLED_Show_Str(55, 0, Debug_Which_Wheel ? "R" : "L", TextSize_F6x8);

                OLED_Show_Str(0, 1, "Real", TextSize_F6x8);
                OLED_Show_Numbers(40, 1, real_spd, TextSize_F6x8);
                OLED_Show_Str(0, 2, "PWM", TextSize_F6x8);
                OLED_Show_Numbers(40, 2, pwm_out, TextSize_F6x8);

                OLED_Show_Str(0, 4, "1.Save", TextSize_F6x8);
                OLED_Show_Str(0, 5, "0.Stop", TextSize_F6x8);

                key = CH455_GetOneKey();
                if (key > 0x0F && key != KEY_BLANK)
                    key = (key - 0x0F) >> 4;

                if (key == 1)  // 淇濆瓨鍒癋lash
                {
                    PID_OKb[0] = (uint32)Debug_Kp_Left;
                    PID_OKb[1] = (uint32)(Debug_Ki_Left * 100.0f);
                    PID_OKb[2] = (uint32)Debug_Kp_Right;
                    PID_OKb[3] = (uint32)(Debug_Ki_Right * 100.0f);
                    flash_erase_page(0, 1);
                    flash_write_page(0, 1, PID_OKb, 11);
                }
                else if (key == 0)  // 鍋滄 鈫?鍥為〉1
                {
                    Debug_Motor_Enable = 0;
                    OLED_CLS();
                }
            }
        }
        else if (Debug_Sub_Mode == Debug_Sub_Ground_Test)
        {
            int32 key = KEY_BLANK;
            int32 edit_value = 0;

            if (Debug_Motor_Enable == 0)
            {
                int32 cur_val[8];
                int i;

                OLED_CLS();
                cur_val[0] = (int)Debug_Kp_Left;
                cur_val[1] = (int)(Debug_Ki_Left * 100.0f);
                cur_val[2] = (int)Debug_Kp_Right;
                cur_val[3] = (int)(Debug_Ki_Right * 100.0f);
                cur_val[4] = Debug_Target_Speed;
                cur_val[5] = Debug_Fan_Duty;
                cur_val[6] = Debug_Ground_Dir;

                OLED_Show_Str(0, 0, "L_P", TextSize_F6x8);
                OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
                OLED_Show_Str(0, 1, "L_I", TextSize_F6x8);
                OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
                OLED_Show_Str(0, 2, "R_P", TextSize_F6x8);
                OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
                OLED_Show_Str(0, 3, "R_I", TextSize_F6x8);
                OLED_Show_Numbers(40, 3, cur_val[3], TextSize_F6x8);
                OLED_Show_Str(0, 4, "Spd", TextSize_F6x8);
                OLED_Show_Numbers(40, 4, cur_val[4], TextSize_F6x8);
                OLED_Show_Str(0, 5, "Fan", TextSize_F6x8);
                OLED_Show_Numbers(40, 5, cur_val[5], TextSize_F6x8);
                OLED_Show_Str(0, 6, "Dir", TextSize_F6x8);
                OLED_Show_Numbers(40, 6, cur_val[6], TextSize_F6x8);
                for (i = 0; i < 7; i++)
                {
                    edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
                    if (edit_value != 0) cur_val[i] = edit_value;
                }

                Debug_Kp_Left = cur_val[0];
                Debug_Ki_Left = cur_val[1] * 0.01f;
                Debug_Kp_Right = cur_val[2];
                Debug_Ki_Right = cur_val[3] * 0.01f;
                Debug_Target_Speed = cur_val[4];
                Debug_Fan_Duty = cur_val[5];
                Debug_Ground_Dir = (cur_val[6] == 2) ? 2 : 1;

                Debug_Motor_Enable = 1;
                PID_cleardata(&Left_PID);
                PID_cleardata(&Right_PID);
                OLED_CLS();
            }
            else
            {
                OLED_Show_Str(0, 0, "Ground", TextSize_F6x8);
                OLED_Show_Str(0, 1, "LReal", TextSize_F6x8);
                OLED_Show_Numbers(45, 1, Left_Real_Spd, TextSize_F6x8);
                OLED_Show_Str(0, 2, "RReal", TextSize_F6x8);
                OLED_Show_Numbers(45, 2, Right_Real_Spd, TextSize_F6x8);
                OLED_Show_Str(0, 3, "Fan", TextSize_F6x8);
                OLED_Show_Numbers(45, 3, Debug_Fan_Duty, TextSize_F6x8);
                OLED_Show_Str(0, 4, "Dir", TextSize_F6x8);
                OLED_Show_Str(45, 4, Debug_Ground_Dir == 2 ? "L-/R+" : "L+/R-", TextSize_F6x8);
                OLED_Show_Str(0, 5, "1.Save", TextSize_F6x8);
                OLED_Show_Str(0, 6, "0.Stop", TextSize_F6x8);

                key = CH455_GetOneKey();
                if (key > 0x0F && key != KEY_BLANK)
                    key = (key - 0x0F) >> 4;

                if (key == 1)
                {
                    PID_OKb[0] = (uint32)Debug_Kp_Left;
                    PID_OKb[1] = (uint32)(Debug_Ki_Left * 100.0f);
                    PID_OKb[2] = (uint32)Debug_Kp_Right;
                    PID_OKb[3] = (uint32)(Debug_Ki_Right * 100.0f);
                    DBG_OKb[0] = (uint32)Debug_Target_Speed;
                    DBG_OKb[1] = (uint32)Debug_Fan_Duty;
                    DBG_OKb[2] = (uint32)Debug_Ground_Dir;
                    DBG_OKb[3] = (uint32)Debug_Angle_Mode;
                    flash_erase_page(0, 0);
                    flash_write_page(0, 0, Speed_OKb, 1);
                    flash_erase_page(0, 1);
                    flash_write_page(0, 1, PID_OKb, 11);
                    flash_erase_page(0, 3);
                    flash_write_page(0, 3, DBG_OKb, 4);
                }
                else if (key == 0)
                {
                    Debug_Motor_Enable = 0;
                    OLED_CLS();
                }
            }
        }
        else if (Debug_Sub_Mode == Debug_Sub_Angle || Debug_Sub_Mode == Debug_Sub_NormalTrace)
        {
            int32 key = KEY_BLANK;
            int32 edit_value = 0;

            if (Debug_Motor_Enable == 0)
            {
                int32 cur_val[8];
                int i;

                OLED_CLS();
                cur_val[0] = (int)(Angle_PID.kp * 100.0f);
                cur_val[1] = (int)(Angle_PID.kd * 100.0f);
                cur_val[2] = (int)(Gyro_PID.kp * 1000.0f);
                cur_val[3] = (int)(Gyro_PID.ki * 1000.0f);
                cur_val[4] = (int)(Gyro_PID.kd * 1000.0f);
                cur_val[5] = (int)(Gyro_PD_PID.kp * 1000.0f);
                cur_val[6] = (int)(Gyro_PD_PID.kd * 1000.0f);
                cur_val[7] = Debug_Angle_Mode;

                OLED_Show_Str(0, 0, "T_P", TextSize_F6x8);
                OLED_Show_Numbers(40, 0, cur_val[0], TextSize_F6x8);
                OLED_Show_Str(0, 1, "T_D", TextSize_F6x8);
                OLED_Show_Numbers(40, 1, cur_val[1], TextSize_F6x8);
                OLED_Show_Str(0, 2, "GI_P", TextSize_F6x8);
                OLED_Show_Numbers(40, 2, cur_val[2], TextSize_F6x8);
                OLED_Show_Str(0, 3, "GI_I", TextSize_F6x8);
                OLED_Show_Numbers(40, 3, cur_val[3], TextSize_F6x8);
                OLED_Show_Str(0, 4, "GI_D", TextSize_F6x8);
                OLED_Show_Numbers(40, 4, cur_val[4], TextSize_F6x8);
                OLED_Show_Str(0, 5, "GD_P", TextSize_F6x8);
                OLED_Show_Numbers(40, 5, cur_val[5], TextSize_F6x8);
                OLED_Show_Str(0, 6, "GD_D", TextSize_F6x8);
                OLED_Show_Numbers(40, 6, cur_val[6], TextSize_F6x8);
                OLED_Show_Str(0, 7, "Mode", TextSize_F6x8);
                OLED_Show_Numbers(40, 7, cur_val[7], TextSize_F6x8);

                for (i = 0; i < 8; i++)
                {
                    edit_value = KeyboardInput(40, i, TextSize_F6x8, 1.0);
                    if (edit_value != 0) cur_val[i] = edit_value;
                }

                Angle_PID.kp = cur_val[0] * 0.01f;
                Angle_PID.kd = cur_val[1] * 0.01f;
                Angle_PID.ki = 0;
                Angle_PID.mode = PID_MODE_POSITION_D_ON_MEASUREMENT;
                Turn_PID.kp = Angle_PID.kp;
                Turn_PID.kd = Angle_PID.kd;
                Turn_PID.ki = 0;
                Turn_PID.mode = PID_MODE_POSITION;
                Gyro_PID.kp = cur_val[2] * 0.001f;
                Gyro_PID.ki = cur_val[3] * 0.001f;
                Gyro_PID.kd = cur_val[4] * 0.001f;
                Gyro_PID.mode = PID_MODE_ADD;
                Gyro_PD_PID.kp = cur_val[5] * 0.001f;
                Gyro_PD_PID.kd = cur_val[6] * 0.001f;
                Gyro_PD_PID.ki = 0;
                Gyro_PD_PID.mode = PID_MODE_POSITION;
                Debug_Angle_Mode = (cur_val[7] == 2) ? 2 : 1;

                Debug_Motor_Enable = 1;
                Gyro_Integral = 0;
                Debug_Angle_D_First = 0;
                PID_cleardata(&Gyro_PID);
                PID_cleardata(&Gyro_PD_PID);
                PID_cleardata(&Left_PID);
                PID_cleardata(&Right_PID);
                OLED_CLS();
            }
            else
            {
                OLED_Show_Str(0, 0, "Angle", TextSize_F6x8);
                OLED_Show_Numbers(45, 0, (int)Gyro_Integral, TextSize_F6x8);
                OLED_Show_Str(0, 1, "Out", TextSize_F6x8);
                OLED_Show_Numbers(45, 1, (int)Gyro_PID_Out, TextSize_F6x8);
                OLED_Show_Str(0, 2, "Mode", TextSize_F6x8);
                OLED_Show_Numbers(45, 2, Debug_Angle_Mode, TextSize_F6x8);
                OLED_Show_Str(0, 3, "D1st", TextSize_F6x8);
                OLED_Show_Numbers(45, 3, Debug_Angle_D_First, TextSize_F6x8);
                OLED_Show_Str(0, 4, "Fan", TextSize_F6x8);
                OLED_Show_Numbers(45, 4, Debug_Fan_Duty, TextSize_F6x8);
                OLED_Show_Str(0, 5, "1.Save", TextSize_F6x8);
                OLED_Show_Str(0, 6, "0.Stop", TextSize_F6x8);

                key = CH455_GetOneKey();
                if (key > 0x0F && key != KEY_BLANK)
                    key = (key - 0x0F) >> 4;

                if (key == 1)
                {
                    PID_OKb[4] = (uint32)(Angle_PID.kp * 100.0f);
                    PID_OKb[5] = (uint32)(Angle_PID.kd * 100.0f);
                    PID_OKb[6] = (uint32)(Gyro_PID.kp * 1000.0f);
                    PID_OKb[7] = (uint32)(Gyro_PID.ki * 1000.0f);
                    PID_OKb[8] = (uint32)(Gyro_PD_PID.kp * 1000.0f);
                    PID_OKb[9] = (uint32)(Gyro_PD_PID.kd * 1000.0f);
                    PID_OKb[10] = (uint32)(Gyro_PID.kd * 1000.0f);
                    Speed_OKb[0] = (uint32)Basic_Speed;
                    DBG_OKb[1] = (uint32)Debug_Fan_Duty;
                    DBG_OKb[3] = (uint32)Debug_Angle_Mode;
                    flash_erase_page(0, 0);
                    flash_write_page(0, 0, Speed_OKb, 1);
                    flash_erase_page(0, 1);
                    flash_write_page(0, 1, PID_OKb, 11);
                    flash_erase_page(0, 3);
                    flash_write_page(0, 3, DBG_OKb, 4);
                }
                else if (key == 0)
                {
                    Debug_Motor_Enable = 0;
                    OLED_CLS();
                }
            }
        }
        else  // 鍏朵粬璋冭瘯瀛愭ā寮忥紙棰勭暀锛?
        {
            OLED_Show_Str(20, 0, "Debug Idle", TextSize_F6x8);
            OLED_Show_Str(0, 4, "TODO", TextSize_F6x8);
        }
        return;
    }

    // 鍘熸湁寤哄浘/鍥炴斁妯″紡鏄剧ず
    OLED_Show_Str(20, 0, "Nothing or Best.", TextSize_F6x8);

    OLED_Show_Str(0, 2, "L_Spd", TextSize_F6x8);
    OLED_Show_Numbers(77, 2, Left_Exp_Spd, TextSize_F6x8);

    OLED_Show_Str(0, 4, "R_Spd", TextSize_F6x8);
    OLED_Show_Numbers(77, 4, Right_Exp_Spd, TextSize_F6x8);

    OLED_Show_Str(0, 6, "Err", TextSize_F6x8);
    OLED_Show_Numbers(77, 6, Error, TextSize_F6x8);

    OLED_Show_Light_Row();  // 鏄剧ず鍏夋晱浼犳劅鍣ㄧ姸鎬?
}
