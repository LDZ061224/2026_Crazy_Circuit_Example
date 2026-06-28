/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

/*
 *  编译模式控制（与 cpu0_main.c 保持一致）
 *  测试模式 = 1: CPU1 空循环，不依赖 OLED/headfiles.h
 *  正式模式 = 0: CPU1 运行 OLED_Display()
 */
#define NEW_CAR_TEST_ENABLE  1

#if !NEW_CAR_TEST_ENABLE
    #include "headfiles.h"
#endif

#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗

#if NEW_CAR_TEST_ENABLE
    /* ========== 测试模式：CPU1 空闲 ========== */
    // 所有测试逻辑在 CPU0 运行，CPU1 仅关看门狗后空转
    while (TRUE)
    {
    }
#else
    /* ========== 正式模式：CPU1 负责 OLED 显示刷新 ========== */
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        OLED_Display();
    }
#endif
}
#pragma section all restore
// **************************** 代码区域 ****************************
