/*************************************************
oled.show
**************************************************/
#include "OLEDfun.h"
#include "OLED.h"
#include <string.h>
#include "init.h"

unsigned char Oled_Model_Choose = 0;
uint8_t Image_dis[64][120] = {0};
int L_MAX = 0;
int R_MAX = 0;
int L_MIN = 0;
int R_MIN = 0;
int L_THR = 0;
int R_THR = 0;

uint8_t Mode_Choose = 0;
uint8_t Node_Movement = 0;
uint8_t Blue_Mode = 0;
uint8_t Mileage_Test = 0;	

/******************************************************
** Function: Oled_Input
** Description:OLED输入参数
** Others:
*******************************************************/
void Oled_Input(void) 
{
    CH455_Init();
    OLED_CLS();
	
    OLED_ShowStr(35,2,"TYUT-JBD",2);
	OLED_ShowStr(37,4,"Confirm",1);
	Mode_Choose =  KeyboardInput(88, 6);
	OLED_CLS();// 清屏
	
	if (Mode_Choose == 1)
	{
		OLED_ShowStr(0, 0, "Left_P", 2);	
		OLED_ShowStr(0, 2, "Left_I", 2);	
		OLED_ShowStr(0, 4, "Right_P", 2);	
		OLED_ShowStr(0, 6, "Right_I", 2);	
		
		Left_P_Init = (KeyboardInput_with_Flash(0, 0, 80, 0) * 1.0) / 10;
		Left_I_Init = (KeyboardInput_with_Flash(0, 1, 80, 2) * 1.0) / 10;
		Right_P_Init = (KeyboardInput_with_Flash(0, 2, 80, 4) * 1.0) / 10;
		Right_I_Init = (KeyboardInput_with_Flash(0, 3, 80, 6) * 1.0) / 10;
		
		OLED_CLS();// 清屏
		
		OLED_ShowStr(0, 0, "Turn_P", 2);
		OLED_ShowStr(0, 2, "Turn_D", 2);
		OLED_ShowStr(0, 4, "Gyro_P", 2);
		OLED_ShowStr(0, 6, "Gyro_D", 2);
		
		Turn_P_Init = (KeyboardInput_with_Flash(0, 4, 80, 0) * 1.0) / 100;
		Turn_D_Init = (KeyboardInput_with_Flash(0, 5, 80, 2) * 1.0) / 100;
		Gyro_P_Init = (KeyboardInput_with_Flash(0, 6, 80, 4) * 1.0) / 100;
		Gyro_D_Init = (KeyboardInput_with_Flash(0, 7, 80, 6) * 1.0) / 100;
		
		OLED_CLS();// 清屏
		
		OLED_ShowStr(0, 0, "v_Exp", 2);
		OLED_ShowStr(0, 2, "V_Exp", 2);
		OLED_ShowStr(0, 4, "dust", 2);
		OLED_ShowStr(0, 6, "Gyro_Ex", 2);
		
		Left_Exp_Init = KeyboardInput_with_Flash(0, 8, 80, 0);
		Right_Exp_Init = KeyboardInput_with_Flash(0, 17, 80, 2);
		Dust_Exp_Init = KeyboardInput_with_Flash(0, 9, 80, 4);
		Gyro_Exp_Init =  KeyboardInput_with_Flash(0, 10, 80, 6);

		OLED_CLS();// 清屏
		
		OLED_ShowStr(0, 0, "Execute", 2);
		OLED_ShowStr(0, 2, "Line", 2);
		OLED_ShowStr(0, 4, "Ele", 2);
		OLED_ShowStr(0, 6, "Mileage", 2);

	    Execute_Times = KeyboardInput_with_Flash(0, 12, 80, 0);
		Line_Num_Count = KeyboardInput_with_Flash(0, 13, 80, 2);
		In_Line_Ele_Count = KeyboardInput_with_Flash(0, 14, 80, 4);
		Count.Mileage = KeyboardInput_with_Flash(0, 15, 80, 6);

		OLED_CLS();// 清屏
		
		OLED_ShowStr(0, 0, "L_Mileage", 2);
		OLED_ShowStr(0, 6, "mode", 2);
		
	    Count.Node_Mileage = KeyboardInput_with_Flash(0, 16, 80, 0);
	    Gyro_mode_Init =  KeyboardInput_with_Flash(0, 11, 80, 6);
	}
	
	Left_P_Init = FlashParam_Read(0) * 1.0 / 10;
	Left_I_Init = FlashParam_Read(1) * 1.0 / 10;
	Right_P_Init = FlashParam_Read(2) * 1.0 / 10;
	Right_I_Init = FlashParam_Read(3) * 1.0 / 10;
	
	Turn_P_Init = FlashParam_Read(4) * 1.0 / 100;
	Turn_D_Init = FlashParam_Read(5) * 1.0 / 100;
	Gyro_P_Init = FlashParam_Read(6) * 1.0 / 1000;
	Gyro_D_Init = FlashParam_Read(7) * 1.0 / 1000;

	Left_Exp_Init = FlashParam_Read(8);
	Right_Exp_Init = FlashParam_Read(17);
  Dust_Exp_Init = FlashParam_Read(9);
	Gyro_Exp_Init = FlashParam_Read(10);
	Gyro_mode_Init = FlashParam_Read(11);

	Execute_Times = FlashParam_Read(12);
	Line_Num_Count = FlashParam_Read(13);
	In_Line_Ele_Count = FlashParam_Read(14);
	Count.Mileage = FlashParam_Read(15);

	Count.Node_Mileage = FlashParam_Read(16);
    OLED_CLS();// 清屏
}	 

/******************************************************
** Function: Oled_Data_Load
** Description:�������
** Others:
*******************************************************/
void Oled_Data_Load()
{	
	Turn_PID.mode = PID_MODE_POSITION;
	Gyro_PID.mode = PID_MODE_POSITION;
	PID_loadparam(&Turn_PID, Turn_P_Init, 0, Turn_D_Init);
	PID_loadparam(&Gyro_PID, Gyro_P_Init, 0, Gyro_D_Init);
	PID_loadparam(&Left_PID, Left_P_Init, Left_I_Init, Left_D_Init);
	PID_loadparam(&Right_PID, Right_P_Init, Right_I_Init, Right_D_Init);
	L_exp = Left_Exp_Init;
	R_exp = Right_Exp_Init;
	dust_exp = Dust_Exp_Init;
	Gyro_exp = Gyro_Exp_Init;
	map_mode = Gyro_mode_Init;
}


/******************************************************
** Function: Oled_Show
** Description:������ʾ
** Others:
*******************************************************/
void Oled_Show()
{
	for (int i = 0; i < 16; i++)
	{
		OLED_ONE_Number(0 + 8 * i, 0, Image_Arr[i]);
	}
	if(Error < 0) OLED_ShowStr(0,2,"-",2);
	else if(Error > 0) OLED_ShowStr(0,2,"+",2);
	else OLED_ShowStr(0,2," ",2);
	OLED_Numbers(50, 2, ABS(Error));
	OLED_Numbers(0, 4, Left_Exp_Spd);
	OLED_Numbers(50, 4, Right_Exp_Spd);
	if(ICM_Gyro[2] < 0) OLED_ShowStr(0,6,"-",2);
	else if(ICM_Gyro[2] > 0) OLED_ShowStr(0,6,"+",2);
	else OLED_ShowStr(0,6," ",2);
	OLED_Numbers(50 , 6, ABS(ICM_Gyro[2]));
}	

void Oled_ShowNum()
{
	for (int i = 0; i < 16; i++)
	{
		OLED_ONE_Number(0 + 8 * i, 0, Image_Arr[i]);
	}
}	
