///*************************************************
//Copyright (C), 2016-2025, TYUT JBD TRoMaC
//File name: control.c
//Author: 
//Version:               
//Date: 2025.10.24
//Description:  
//Others:      
//Function List:
//History:
//<author>  	 <time>   	<version> 	 	<desc>
//**************************************************/
//	
//#include "recognize_control.h"
//	
//#define LEN 16
//#define false 0
//#define true 1
//	
///************************************变量定义***********************************/
///*-------------标志位--------------*/
//uint8_t Stop_Flag = 0;       //停止标志位 
//uint8_t First_Mode = 0;      //初始模式标志位，初始化一些变量
//uint8_t Finish_Flag = 0;
//uint16_t Finish_Count = 0;
///*-------------循迹--------------*/
//int Image_Arr[16] = {0};	  // 图像数组
//int Last_Track_Arr[16] = {-1};	// 上次赛道
//int Track_Arr[16] = {-1};	// 赛道
//uint8_t Last_Track_Num = 0;	// 上次识别点数量
//uint8_t Track_Num = 0;		// 识别点数量
//int Left_Scan_Point = 0, Right_Scan_Point = 0; //左右扫描灯
//int Middle = 0;             //中点灯
//int Speed_Get_Count = 1;     
//int8_t Last_Error = 0;       //历史误差
//int8_t Error = 0;            //误差
//const int8_t Dir_Arr[16] = {-32, -26, -17, -14, -10, -5, -2, -1, 1, 2, 5, 10, 14, 17, 26, 32};
//uint8_t Left_Num = 0;       // 左侧点数量
//uint8_t Right_Num = 0;      // 右侧点数量
//	
///*--------------赛道结构体---------------*/
//Racing_track_Typedef Run_Track;     // 跑赛道结构体，键显选择
//uint8_t Line_Num_Count = 0;       // 有元器件的路段中正常线路的数量
//uint8_t In_Line_Ele_Count = 0;    // 一段连线中元器件计数
//uint8_t Execute_Times = 0;   //到第几个路段
//uint8_t Element_nums = 0;   //当前路段的元器件数量
//	
///*--------------速度---------------*/
//int giSpeed_Right[3] = {0};	// 右测速
//int giSpeed_Left[3] = {0};	// 左测速
//int Left_Real_Spd = 0;		// 左实际
//int Right_Real_Spd = 0;		// 右实际
//int Left_Spd = 0;			// 左实际
//int Right_Spd = 0;			// 右实际
//int Left_Exp_Spd = 0;		// 左期望
//int Right_Exp_Spd = 0;		// 右期望
//float Average_Speed = 0;    //左右平均，用来做积分
//float Last_Average_Speed = 0; //加权积分
//Speed_Adjust_Typedef Speed_Adjust;
///*---------------keyboard输出-------------*/
//int L_exp = 0;
//int R_exp = 0;
//int dust_exp = 0;
//int Gyro_exp = 0;
//uint8_t Gyro_mode = 0;
///*-------------陀螺仪--------------*/
//float ICM_Acc[3] = {0};
//float ICM_Gyro[3] = {0};
//float Gyro_Integral = 0;

///*-------------PID--------------*/
//PID_HandleTypeDef Gyro_PID = GYRO_PID;
//PID_HandleTypeDef Left_PID = LEFT_PID;
//PID_HandleTypeDef Right_PID = RIGHT_PID;
//PID_HandleTypeDef Turn_PID = TURN_PID;

//float Turn_PID_Out = 0;
//float Gyro_PID_Out = 0;
//float Left_PID_Out = 0;
//float Right_PID_Out = 0;
//	

///*-------------count-------------*/
//Count_Typedef Count =
//{
//    .Left = 0,
//    .Right = 0,
//    .Stop = 0,
//    .Mileage = 0,
//    .Straight = 0,
//};

///*-------------mode-------------*/
//Run_Mode_Enum Run_Mode = Mileage_Mode;

///************************************函数定义***********************************/
//	
///******************************************************
//** Function: Car_Go
//** Description: Go Go Go~
//** Others:
//*******************************************************/
//void Car_Go(void)
//{
//	Speed_Get_Count *= -1;
//	
//	if (Speed_Get_Count == 1)
//	{
//		Get_Speed();
//	}
//	//不知道是何意味
//	Left_Real_Spd = Left_Spd;
//	Right_Real_Spd = Right_Spd;
//	
//	Get_IMU();
//	Get_Error();
//	Set_Out();
//}	
//	
///******************************************************
//** Function: Get_Speed
//** Description: 测速函数
//** Others:
//*******************************************************/
//void Get_Speed()
//{	
//	giSpeed_Left[2] = giSpeed_Left[1];
//	giSpeed_Left[1] = giSpeed_Left[0];
//	giSpeed_Right[2] = giSpeed_Right[1];
//	giSpeed_Right[1] = giSpeed_Right[0];
//	
//	giSpeed_Left[0]  =  1 * Get_Count(&Encoder_Left) / 3;
//	giSpeed_Right[0] = -1 * Get_Count(&Encoder_Right) / 3;
//	
//	if(LeftForward == 1)
//	{
//		giSpeed_Left[0] = -giSpeed_Left[0];
//	}
//	
//	if(RightForward == 1)
//	{
//		giSpeed_Right[0] = -giSpeed_Right[0];
//	}
//	
//	Left_Spd  = 0.5 * giSpeed_Left[0] + 0.3 * giSpeed_Left[1] + 0.2 * giSpeed_Left[2];
//	Right_Spd = 0.5 * giSpeed_Right[0] + 0.3 * giSpeed_Right[1] + 0.2 * giSpeed_Right[2];
//	
//	Clear_Count(&Encoder_Left);
//	Clear_Count(&Encoder_Right);
//}	
//	
///******************************************************
//** Function: Get_Error
//** Description: 误差计算
//** Others:
//*******************************************************/
//void Get_Error(void)
//{
//	// 光电管数组处理
//	Image_Process();
//	if (First_Mode == 0)
//    {
//		Run_Track = Test1;
//        Run_Mode = Mileage_Mode;
//        First_Mode = 1;
//        Element_nums = Run_Track.Arr_Element_Num[Execute_Times];
//    }
//	
//    if (Element_nums == 0)
//    {
//        Run_Mode = Normal_Mode;
////        Mileage_Stage = Normal_Stage;
//        In_Line_Ele_Count = 0;
//    }
//	// 模式选择
//    switch (Run_Mode)
//    {
//        case Normal_Mode:
//                Normal_Run();
//                break;/*Normal_Mode*/
//        case Turn_Left:
//                Turn_Left_Run();
//                break;/*Turn_Left*/
//        case Turn_Right:
//                Turn_Right_Run();
//                break;/*Turn_Right*/
//        case Straight_Mode:
//                Straight_Run();
//                break;/*Straight*/
//        case Mileage_Mode:
//                Mileage_Mode_Run();
//                break;/*Mileage_Mode*/
//	}
//}	
//	
///******************************************************
//** Function: Get_IMU
//** Description: 陀螺仪数据获取
//** Others:
//*******************************************************/
//void Get_IMU(void)
//{	
//	icm20602_get_gyro(ICM_Gyro);
//	if(Gyro_mode == 1) ICM_Gyro[2] =  ICM_Gyro[2] + Gyro_exp;
//	else if(Gyro_mode == 2) ICM_Gyro[2] =  ICM_Gyro[2] - Gyro_exp;
//	
//	if (ABS(ICM_Gyro[2]) < 1)
//	{
//		ICM_Gyro[2] = 0;
//	}
//	// 积分∫
//    if (Run_Mode == Turn_Left || Run_Mode == Turn_Right)
//    {
//        Gyro_Integral += ICM_Gyro[2] * 1.0/100.0 ;
//    }
//}	
//	
///************************************************
//*Name: PWM_SetDuty 
//*Function: 占空比封装
//*Others: None
//*************************************************/
//void PWM_SetDuty(uint32_t *Channel, int32_t Duty)
//{	
//    Duty = ABS(Duty);
//    *Channel = Duty;
//}/* PWM_SetDuty() */
//	
///******************************************************
//** Function: Calcul_Out
//** Description: 计算输出
//** Others:
//*******************************************************/
//void Calcul_Out(void)
//{	
//	Turn_PID_Out  = PID_calc(&Turn_PID, 0, Error);
//	
//	// 陀螺仪可以先不用
//    Gyro_PID_Out = PID_calc(&Gyro_PID, 0, Turn_PID_Out + ICM_Gyro[2]);
//	
//	if(Gyro_mode == 0)
//	{
//		Left_Exp_Spd = L_exp - Turn_PID_Out;
//		Right_Exp_Spd = R_exp + Turn_PID_Out;
//	}
//	else 
//	{
//		Left_Exp_Spd = L_exp + Gyro_PID_Out;
//		Right_Exp_Spd = R_exp - Gyro_PID_Out;
//	}
//	
//	Left_PID_Out  = PID_calc(&Left_PID, Left_Exp_Spd, Left_Real_Spd);
//	Right_PID_Out = PID_calc(&Right_PID, Right_Exp_Spd, Right_Real_Spd);     
//}	
//	
///******************************************************
//** F unction: Set_Out
//** Description: 设置输出
//** Others:
//*******************************************************/
//void Set_Out(void)
//{	
//	// 计算输出
//    Calcul_Out();   
//	
//	// 风扇
//	PWM_SetDuty(Duct_IN1, 800 - dust_exp);
//	PWM_SetDuty(Duct_IN2, 800);
//	
//	if (Enable_Switch_ON)	// 使能开关
//	{	
//		if (Left_PID_Out == 0 || Stop_Flag == 1)
//		{
//			PWM_SetDuty(Left_IN1, 4800);
//			PWM_SetDuty(Left_IN2, 4800);
//		}
//		else if (Left_PID_Out > 0)	// 正转
//		{
//			PWM_SetDuty(Left_IN1, 4800);
//			PWM_SetDuty(Left_IN2, ABS(4800 - Left_PID_Out));
//		}
//		else					// 反转
//		{
//			PWM_SetDuty(Left_IN1, ABS(4800 + Left_PID_Out));
//			PWM_SetDuty(Left_IN2, 4800);
//		}
//		
//		if (Right_PID_Out == 0 || Stop_Flag == 1)
//		{
//			PWM_SetDuty(Right_IN1, 4800);
//			PWM_SetDuty(Right_IN2, 4800);
//		}
//		else if (Right_PID_Out < 0)	// 反转
//		{
//			PWM_SetDuty(Right_IN1, 4800);
//			PWM_SetDuty(Right_IN2, ABS(4800 + Right_PID_Out));
//		}
//		else					// 反转
//		{
//			PWM_SetDuty(Right_IN1, ABS(4800 - Right_PID_Out));
//			PWM_SetDuty(Right_IN2 , 4800);
//		}
//	}
//	else
//	{
//			PWM_SetDuty(Left_IN1, 0);
//			PWM_SetDuty(Left_IN2, 0);
//			PWM_SetDuty(Right_IN1, 0);
//			PWM_SetDuty(Right_IN2, 0);
//		
//			PID_cleardata(&Left_PID);
//			PID_cleardata(&Right_PID);
//	}
//}	
//	
///*******************************************************
//** Function: Image_Process
//** Description: 图像处理
//** Others:
//********************************************************/
//void Image_Process(void)
//{	
//	static int k = 0;
//	static int l = 0;
//    
//    for (int i = 0; i < 8; i++)
//    {
//        if (Left_Light[i]  > left_thr[i][0]) Image_Arr[i] = 1;
//        else if (Left_Light[i]  < left_thr[i][1]) Image_Arr[i] = 0;
//        // 否则保持 Image_Arr[i] 原值不变
//	
//        if (Right_Light[i] > right_thr[i][0]) Image_Arr[i + 8] = 1;
//        else if (Right_Light[i] < right_thr[i][1]) Image_Arr[i + 8] = 0;
//        // 否则保持 Image_Arr[i+8] 原值不变
//    }
//    
//	// 初始化变量
//	memcpy(Last_Track_Arr, Track_Arr, sizeof(Track_Arr));
//	for (int i = 0; i < 16; i++)
//	{
//		Track_Arr[i] = 0;
//	}
//	Last_Track_Num = Track_Num;
//	Track_Num = 0;
//	
//	// 记录当前白色数量和中点
//    for (int i = 0; i < 16; i++)
//	{
//		if (Image_Arr[i] == 1)
//		{
//			Track_Arr[Track_Num++] = i;
//		}
//	}
//	
//	// 检查点是否连续,不连续保持上次结果
//	for (int i = 0; i < Track_Num - 1; i++)
//	{	
//		if (Track_Arr[i + 1] - Track_Arr[i] > 1)
//		{
//			Track_Num = Last_Track_Num;
//			memcpy(Track_Arr, Last_Track_Arr, sizeof(Last_Track_Arr));
//		}
//	}
//	
//	 Left_Scan_Point = Track_Arr[0];                 // 左扫描点
//     Right_Scan_Point = Track_Arr[Track_Num - 1];    // 右扫描点
//	 Middle = (Left_Scan_Point + Right_Scan_Point) / 2;
//	 // 出赛道判断
////    if (Track_Num == 16 || Track_Num == 0)
////    {
////        Count.Stop++;
////    }
////    else
////    {
////        Count.Stop = 0;
////    }

////    if (Count.Stop > 100)
////       {
////           Stop_Flag = 1;
////       }
//	     if (Finish_Flag == 1)
//    {
//        Finish_Count++;
//    }

//    if (Finish_Count > 200)
//    {
//        Stop_Flag = 1;
//    }
//}	
//	

///*************************************
//** Function: Check_Edge
//** Description: 检查边界
//** Others:
//*************************************/
//uint8_t Check_Edge()
//{
////    for (int i = 0; i < 15; i++)
////    {
//        if (Track_Arr[0] == 0 || Track_Arr[Track_Num - 1] == 15)
//        {
//            return 1;
//        }
////    }

//    return 0;
//}

///******************************************************
//** Function: void Normal_Run 
//** Description: 正常模式
//** Others:
//*******************************************************/
//void Normal_Run(void)
//{	
////	static uint8_t count = 0;
////	if(Track_Num == 0 && !count ) 
////	{
////		count = 1;
////		Error = Last_Error * 1.3;
////		return;
////	}
////	if(count)
////	{
////		Error = Last_Error * 1.5;
////		count = 0;
////		return;
////	}
////	if (Track_Num < 12  && Track_Num > 0)	
////	{
////		Error = (Dir_Arr[Track_Arr[0]] + Dir_Arr[Track_Arr[Track_Num - 1]]) / 2;
////	}
////	else
////	{
////		Error = Last_Error;
////	}

//	/*更改为第一版*/
//	Last_Error = Error;
//	HAL_GPIO_WritePin (GPIOC , GPIO_PIN_0, 0);
//    if (Track_Num < 2)
//    {
//        Error = (Last_Error / ABS(Last_Error)) * 24;
//    }
//    else if (Track_Num <=  5 && Track_Num >= 2) // 赛道宽度小于6，正常循迹
//    {
////        Left_Scan_Point = Track_Arr[0];                 // 左扫描点
////        Right_Scan_Point = Track_Arr[Track_Num - 1];    // 右扫描点
//        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
//    }
//    else        // 异常，按规划跑
//    {
//        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
//		 if (Check_Edge())
//        {
//            switch (Run_Track.Arr_Node_Dir[Execute_Times])
//            {
//                case 1:     // 左转
//					Error = -25;
//                    Run_Mode = Turn_Left;
//                    /* code */
//                    break;
//                case 2:     // 右转
//					Error = 25;
//                    Run_Mode = Turn_Right;
//                    /* code */
//                    break;
//                case 4:     // 直行
//					Error = 0;
//                    Run_Mode = Straight_Mode;
//                    break;
//            }
//            Execute_Times = (Execute_Times + 1) % (Run_Track.Node_Num + 1);
//            Element_nums = Run_Track.Arr_Element_Num[Execute_Times];
//			
//			// 跑完一圈停车
//            if (Run_Track.Stop_Mode == 0)
//            {
//                if (Execute_Times == Run_Track.Node_Num)
//                {
//                    Finish_Flag = 1;
//                }
//            }
//		}
//	}
//}	

///*************************************
//** Function: Turn_Left_Run
//** Description: 左直角
//** Others:
//*************************************/
//void Turn_Left_Run(void)
//{
//    Error = -23;
////    Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
//    HAL_GPIO_WritePin (GPIOC ,GPIO_PIN_0 ,1);

////    for (int i = 0; i < 15; i++)
////    {
////        if (Track_Arr[i] >= 8 && Track_Arr[i] != 15)
////        {
////            Right_Num++;
////        }
////    }

//    if (Track_Num < 6 && Track_Num > 1 && Middle >= 0 && Middle <= 7 /* && Right_Num < 3&& Turn_Action == Straight_Stage*/)
//    {
//        Count.Left++;
//    }
//    else
//    {
//        Count.Left = 0;
//    }

//    if (Count.Left > 1 && Gyro_Integral < -38) 
//    {
//        Gyro_Integral = 0;
//        Run_Mode = Mileage_Mode;
//        Count.Left = 0;
//		Right_Num = 0;
//    }
//}

///*************************************
//** Function: Turn_Right_Run
//** Description: 右直角
//** Others:
//*************************************/
//void Turn_Right_Run(void)
//{
//    Error = 23;
////    Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;
//    HAL_GPIO_WritePin (GPIOC ,GPIO_PIN_0, 1); 
////    for (int i = 0; i < 15;  i++)
////    {
////        if (Track_Arr[i] <= 7 && Track_Arr[i] != 0)
////        {
////            Left_Num++;
////        }
////    }

//    if (Track_Num < 6 && Track_Num > 1 && Middle >= 7 && Middle < 15 /* && Left_Num < 3&& Turn_Action == Straight_Stage*/)
//    {
//        Count.Right++;
//    }
//    else
//    {
//        Count.Right = 0;
//    }

//    if (Count.Right > 1 && Gyro_Integral > 38)
//    {
//        Gyro_Integral = 0;
//        Run_Mode = Mileage_Mode;
//        Count.Right = 0;
//		Left_Num = 0;
//    }
//}
///*************************************
//** Function: Straight_Run
//** Description: 直行
//** Others:
//*************************************/
//void Straight_Run(void)
//{
//	HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_0 ,1);
//    Error = 0;
//    Middle = (Track_Arr[0] + Track_Arr[Track_Num - 1]) / 2;

//    if (Track_Num < 6 && Track_Num > 1 && Middle > 3 && Middle < 12)
//    {
//        Count.Straight++;
//    }
//    else
//    {
//        Count.Straight = 0;
//    }

//    if (Count.Straight > 5)
//    {
//        Run_Mode = Mileage_Mode;
//        Count.Straight = 0;
//    }
//}


///*************************************
//** Function:  Mileage_Normal_Run
//** Description: 里程计(1)
//** Others:
//*************************************/
//void Mileage_Normal_Run()
//{
// HAL_GPIO_WritePin (GPIOC , GPIO_PIN_0 , 0);
//   if (Track_Num < 2)
//    {
//        Error = (Last_Error / ABS(Last_Error)) * 24;
//    }
//    else if (Track_Num < 7 && Track_Num >= 2) // 赛道宽度小于6，正常循迹
//    {
////        Left_Scan_Point = Track_Arr[0];                 // 左扫描点
////        Right_Scan_Point = Track_Arr[Track_Num - 1];    // 右扫描点
//        Error = (Dir_Arr[Left_Scan_Point] + Dir_Arr[Right_Scan_Point]) / 2;
//    }
//    else
//    {
//        Error = Last_Error;
//    }
//}

///*************************************
//** Function: Mileage_Run_Stage_2
//** Description: 里程计(2)
//** Others:
//*************************************/
//void Mileage_Element_Run()
//{
//   HAL_GPIO_WritePin (GPIOC , GPIO_PIN_0 , 1);
//    switch (Run_Track.Arr_Element_Dir[Line_Num_Count][In_Line_Ele_Count])
//    {
//        case 0:
//            Error = 0;
//            break;
//        case 1:
//            Error = -16;
//            break;
//        case 2:
//            Error = 16 ;
//            break;
//    }
//}


///*************************************
//** Function: Mileage_Mode_Run
//** Description: 里程计
//** Others:
//*************************************/
//void Mileage_Mode_Run(void)
//{
//	// 动态速度调整
////        Run_Track.Speed_Adjust.Range_Target[0] = Basic_Speed;
////        Run_Track.Speed_Adjust.Range_Target[1] = 45;
////        Run_Track.Speed_Adjust.Range_Source[0] = 0;
////        Run_Track.Speed_Adjust.Range_Source[1] = Run_Track.Node_Mileage_ALL[Line_Num_Count];

//	// 里程计积分
//	if (Enable_Switch_ON)
//	{
//		Last_Average_Speed = Average_Speed;
//		Average_Speed = (Left_Real_Spd + Right_Real_Spd) / 2.0;             // 平均速度
//		Count.Mileage += (0.7 * Average_Speed + 0.3 * Last_Average_Speed);  // 累加里程
//	}
//	if (Count.Mileage < Run_Track.Arr_Mileage_Normal[Line_Num_Count][In_Line_Ele_Count])
//	{
//		// 正常循迹
//		Mileage_Normal_Run();
//	}
//	else if (Count.Mileage < (Run_Track.Arr_Mileage_Normal[Line_Num_Count][In_Line_Ele_Count]
//								  + Run_Track.Arr_Mileage_Element[Line_Num_Count][In_Line_Ele_Count]))
//	{
//		// 元素判断
//		Mileage_Element_Run();
//	}
//	else
//	{
//		In_Line_Ele_Count++;
//		Count.Mileage = 0;
//		if (In_Line_Ele_Count == Run_Track.Arr_Element_Num[Execute_Times])
//		{
//			Run_Mode = Normal_Mode;
//			Line_Num_Count++;
//			In_Line_Ele_Count = 0;
//		}

//		// 跑完一圈停车
//		if (Run_Track.Stop_Mode == 1)
//		{
//			if (Line_Num_Count == Run_Track.Element_All_Num - 1)
//			{
//				Finish_Flag = 1;
//			}
//		}
//	}
//}
