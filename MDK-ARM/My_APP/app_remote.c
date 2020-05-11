/** 
* @brief    遥控器模式设置
* @details  遥控器模式分发
* @author   Xu LiangPU
* @date      2019.12
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 日志
*				版本变更:
*				2020.1.16 |  v1.0  |  创建,将原来的app_mode文件去掉，用此文件代替
*/
#include "app_remote.h"
#include "bsp_dbus.h"
#include "global_define.h"
#include "app_imu.h"
#include "global_variable.h"
#include "bsp_can.hpp"
#include "bsp_adc_deal.h"


/*******************************底盘电机相关变量*****************************************************/
Motor_t DJI_Motor_3508(8192, 19);  //电机类型
pid PID_Chassis_Speed(5,0.1f,0,5000,16000,0,0,80);  //底盘电机PID
pid PID_Chassis_Follow_OUT(167,0,0,10000,10000,0,4,10);  //底盘跟随PID外环，用陀螺仪角度（无死区）
pid PID_Chassis_Follow_IN(1.3,0,0,10000,10000,0,0,1000);  //底盘跟随PID内环，用陀螺仪角加速度
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed,NULL);  //创建底盘类对象



#define MAGAZINE_AC_ON      		HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_SET);  //弹舱
#define MAGAZINE_AC_OFF      		HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_RESET);
#define RESCUE1_AC_ON      			HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_SET);  //救援
#define RESCUE1_AC_OFF      		HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_RESET);  //救援
#define RESCUE2_AC_ON      			HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_SET);  //救援
#define RESCUE2_AC_OFF      		HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_RESET);  //救援

int16_t Order_To_Slaver[4],Order_To_Cloud[4];
//////*******************************非模式控制函数但放在此方便管理的函数*******************************************************************************/
/*
* @brief  主控给副控发送信息
* @details Main函数中周期性执行
* @param  NULL
* @retval  NULL
*  2019.12.23 添加
*
*/
void Master_To_Slaver(void)
{

		Order_To_Slaver[0] = bsp_dbus_Data.Dial;  //拨轮
		Order_To_Slaver[1] = bsp_dbus_Data.CH_0;  //用于控制爪子抓取机构电机左右移动
		Order_To_Slaver[2] = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;  //用于判断副控模式
	  Order_To_Slaver[3] = bsp_dbus_Data.CH_1; 
	  bsp_can_Sendmessage(&COMMUNICATE_CAN,Master_To_Slaver_ID,Order_To_Slaver);  //发送
}

/*
* @brief  主控给云台发送信息
* @details Main函数中周期性执行
* @param  NULL
* @retval  NULL
*  2019.12.23 添加
*
*/
void Master_To_Cloud()
{
    Order_To_Cloud[0] = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;  //用于判断云台模式
		Order_To_Cloud[1] = bsp_dbus_Data.CH_0;  //用于控制云台Yaw
		Order_To_Cloud[2] = bsp_dbus_Data.CH_1;  //用于控制云台Pitch
	  Order_To_Cloud[3] = bsp_dbus_Data.Dial;  //拨轮;  
		bsp_can_Sendmessage(&COMMUNICATE_CAN,Master_To_Cloud_ID,Order_To_Cloud);  //发送
} 


////////******************************遥控器模式分发相关函数*****************************************************************************************/

               
/** 
    * @brief 安全模式
*/
static void Safe_Mode(uint8_t type)
{
	switch(type)
	{
		case STARTING:
			break;
		case RUNNING: 
					Chassis_Engineer.Safe();   
			break;
		case ENDING:
			break;
  }
}

/** 
* @brief 底盘独立模式
* @detail  运用底盘陀螺仪进行底盘控制
*/

static float Target_Angle_Gyro;    //陀螺仪目标角度
static float Turn_Angle_Gyro;  //陀螺仪计算的转弯角度
static void Chassis_Independent_Mode(uint8_t type)
{
	switch(type)
	{
		case STARTING:
					  Target_Angle_Gyro = app_imu_Data.integral.Yaw;  //记录当前时刻陀螺仪角度
			break;
		case RUNNING: 
					//遥控器通道控制
					Target_Angle_Gyro += (bsp_dbus_Data.CH_0)*0.0005f;
					Turn_Angle_Gyro = Target_Angle_Gyro - app_imu_Data.integral.Yaw;  //按道理应该用云台的机械角度换算，现在先凑合用着
					Vx = (bsp_dbus_Data.CH_3) * 14.f*(cos(Turn_Angle_Gyro*pi/180.f) - sin(Turn_Angle_Gyro*pi/180.f)); 
					Vy = (bsp_dbus_Data.CH_2) * (-14.f)*(sin(Turn_Angle_Gyro*pi/180.f) - cos(Turn_Angle_Gyro*pi/180.f));
					Vz = PID_Chassis_Follow_IN.pid_run(PID_Chassis_Follow_OUT.pid_run(Turn_Angle_Gyro) - app_imu_Data.Angle_Rate[2]);  //Yaw轴串级（无死区）
					Chassis_Engineer.Run(Vx,Vy,Vz);
					if(bsp_dbus_Data.Dial > 0)  
						{
							MAGAZINE_AC_ON;  //弹舱、救援同时开
							RESCUE1_AC_ON;
							RESCUE2_AC_ON;
						}
					if(bsp_dbus_Data.Dial < 0)  
						{
							RESCUE1_AC_OFF;  //弹舱、救援同时关
							RESCUE2_AC_OFF;
							MAGAZINE_AC_OFF;
						}
			break;
		case ENDING:
			break;
  }
}


/** 
* @brief 底盘对位
* @detail  
*/
#define LEFT_SHARP bsp_ADC1_Sharp_Distance[0]   //左后轮传感器
#define RIGHT_SHARP bsp_ADC1_Sharp_Distance[1]   //左前轮传感器
#define SHARP_BETWEEN 100  //车子上两个夏普传感器的距离单位为cm
#define AUTO_PARA_TARGET_DISTANCE 15  //目标距离
#define THRESHOLD_Z	 2.0f		//Z轴旋转阈值
#define THRESHOLD_Y  2.5f   //Y轴旋转阈值

pid Auto_Para_Z_PID(0,0,0,0,0,0);		//自动对位Z轴PID
//	pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1,uint16_t I_Limited=9999);//传统pid构造函数
pid Auto_Para_Y_PID(0,0,0,0,0,0);		//自动对位Y轴PID

float Speed_Z;  //Z轴旋转速度
float Speed_Y;	 //Y轴旋转速度
static uint8_t Flag_Auto_Para = 0;		//自动对位flag，LEFT_SHARP小于50开启
static void Chassis_Auto_Para_Mode(uint8_t type)
{
	switch(type)
	{
		case STARTING:
					  if(LEFT_SHARP > 50.f || RIGHT_SHARP >  50.f) Flag_Auto_Para = 0;	//不开启对位
						if(LEFT_SHARP < 50.f && RIGHT_SHARP < 50.f) Flag_Auto_Para = 1;	//开启对位
			break;
		case RUNNING: 
			    if(Flag_Auto_Para == 1)
					{
						if(ABS(LEFT_SHARP - RIGHT_SHARP) > THRESHOLD_Z)
						{
							Speed_Z = Auto_Para_Z_PID.pid_run(LEFT_SHARP - RIGHT_SHARP);
						}
						else Speed_Z = 0;
						if(ABS(LEFT_SHARP - AUTO_PARA_TARGET_DISTANCE) > AUTO_PARA_TARGET_DISTANCE)
						{
							Speed_Y = Auto_Para_Y_PID.pid_run(LEFT_SHARP - AUTO_PARA_TARGET_DISTANCE);
						}
						else Speed_Y = 0;
				  }
					else 
					{
						Speed_Z = 0;;
						Speed_Y = 0;
					}
					Chassis_Engineer.Run(0,Speed_Y,Speed_Z);
			break;
		case ENDING:
			break;
  }
}

#undef LEFT_SHARP 
#undef RIGHT_SHARP 
#undef SHARP_BETWEEN 
#undef AUTO_PARA_TARGET_DISTANCE 
#undef THRESHOLD_Z	 
#undef THRESHOLD_Y  
///** 
//* @brief 底盘跟随模式
//* @detail 运用云台机械角进行底盘跟随
//*/
//static float Target_Angle_Encoder;  //云台编码器调节目标角度
//static void Chassis_Follow_Mode(uint8_t type)
//{
//  case STARTING:
//					  Target_Angle_Encoder = app_imu_Data.integral.Yaw;  //记录当前时刻陀螺仪角度
//			break;
//		case RUNNING: 
//					//遥控器通道控制
//					TargetAngle_Gyro += (bsp_dbus_Data.CH_0)*0.0005f;
//					Turn_Angle_Gyro = TargetAngle_Gyro - app_imu_Data.integral.Yaw;  //按道理应该用云台的机械角度换算，现在先凑合用着
//					Vx = (bsp_dbus_Data.CH_3) * 14.f*(cos(Turn_Angle_Gyro*pi/180.f) - sin(Turn_Angle_Gyro*pi/180.f)); 
//					Vy = (bsp_dbus_Data.CH_2) * (-14.f)*(sin(Turn_Angle_Gyro*pi/180.f) - cos(Turn_Angle_Gyro*pi/180.f));
//					Vz = PID_Chassis_Follow_IN.pid_run(PID_Chassis_Follow_OUT.pid_run(Turn_Angle_Gyro) - app_imu_Data.Angle_Rate[2]);  //Yaw轴串级（无死区）
//					Chassis_Engineer.Run(Vx,Vy,Vz);
//					if(bsp_dbus_Data.Dial > 0)  
//						{
//							MAGAZINE_AC_ON;  //弹舱、救援同时开
//							RESCUE1_AC_ON;
//							RESCUE2_AC_ON;
//						}
//					if(bsp_dbus_Data.Dial < 0)  
//						{
//							RESCUE1_AC_OFF;  //弹舱、救援同时关
//							RESCUE2_AC_OFF;
//							MAGAZINE_AC_OFF;
//						}
//			break;
//		case ENDING:
//			break;
//}

/** 
    * @brief 抱死模式
*/
static void Lock_Mode(uint8_t type)
{
  switch(type)
	{
		case STARTING:
			break;
		case RUNNING: 
					Vx = 0;
					Vy = 0;
					Vz = 0;
					Chassis_Engineer.Run(Vx,Vy,Vz);   
			break;
		case ENDING:
			break;
  }
}
/** 
* @brief  遥控器mode分发
* @param [in]  mode 遥控器s1*10+s2
* @param [in]  type 运行参数
* @par 日志    
*							2020.1.16		|	创建
*
*/
static void Remote_Distribute(uint8_t mode,uint8_t type)
{
	
	switch(mode)
	{
		case 32:	//爪子电机初始化抱死
					Lock_Mode(type);
			break;
		case 12:	//一次取弹抱死
					Lock_Mode(type);
			break;
		case 11:  //二次取弹抱死
					Lock_Mode(type);
			break;
		case 33:  //底盘独立
					Chassis_Independent_Mode(type);
			break;
		case 22:	//安全
					Safe_Mode(type);
			break;
		case 23:	//自动对位
					Chassis_Auto_Para_Mode(type);
			break;
		default :		//安全
					Safe_Mode(type);
			break;
	}
}

/**
* @brief  遥控器控制函数句柄
* @details  周期执行该句柄
* @param  NULL
* @retval  NULL
*/
static int16_t remote_mode = 22;  //当前遥控器模式，默认为安全
static int16_t last_mode = 0xff;
void Remote_Control_Handle(void)
{
  remote_mode = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;  
	if(remote_mode != last_mode)  //如果当前模式和之前的模式不一样的话
	{
	  Remote_Distribute(last_mode,ENDING);  //退出之前的模式
		Remote_Distribute(remote_mode,STARTING);  //启用当前模式 开始部分
		Remote_Distribute(remote_mode,RUNNING);  //跑一次当前模式的running
		last_mode = remote_mode;
	}
	else Remote_Distribute(last_mode,RUNNING);  //持续当前模式
}

