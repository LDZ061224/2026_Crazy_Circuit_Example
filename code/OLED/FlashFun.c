#include "OLEDkeyboard_Config.h"
#include "FlashFun.h"


/*************************************************************************
*  函数名称：void Flash_Data_Recover(unsigned char desSector, unsigned char backupSector)
*  功能说明：按复位键后将数据从备份扇区恢复
* @param   desSector     ：  目的扇区   范围  0-11
* @param   backupSector  ：  扇区   范围  0-11, 默认11
*  函数返回：TRUE:恢复成功   FLASE:扇区不匹配
*  修改时间：2020年6月6日
*           2022年11月24日     //在键显初始化完成后先进行数据救援，全盘复制
*************************************************************************/
void Flash_Data_Recover(unsigned char backupSector)
{
//    uint32 sectorFlag = EEPROM_Read_32Bit(backupSector, 0);                             //备份扇区的0页存当前备份的扇区
//    uint32 finishFlag = EEPROM_Read_32Bit(backupSector, 1);                             //备份扇区的1页存完成标志

//    if(finishFlag != sectorFlag) //flash数据没有输入结束
//    {
//        Flash_Data_Move(Backup_Sector, sectorFlag);                                     //将数据恢复到原扇区
//    }
}













