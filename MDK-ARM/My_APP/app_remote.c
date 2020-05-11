/** 
* @brief    ң����ģʽ����
* @details  ң����ģʽ�ַ�
* @author   Xu LiangPU
* @date      2019.12
* @version  1.0
* @par Copyright (c):  RM2020���
* @par ��־
*				�汾���:
*				2020.1.16 |  v1.0  |  ����,��ԭ����app_mode�ļ�ȥ�����ô��ļ�����
*/
#include "app_remote.h"
#include "bsp_dbus.h"
#include "global_define.h"
#include "app_imu.h"
#include "global_variable.h"
#include "bsp_can.hpp"
#include "bsp_adc_deal.h"


/*******************************���̵����ر���*****************************************************/
Motor_t DJI_Motor_3508(8192, 19);  //�������
pid PID_Chassis_Speed(5,0.1f,0,5000,16000,0,0,80);  //���̵��PID
pid PID_Chassis_Follow_OUT(167,0,0,10000,10000,0,4,10);  //���̸���PID�⻷���������ǽǶȣ���������
pid PID_Chassis_Follow_IN(1.3,0,0,10000,10000,0,0,1000);  //���̸���PID�ڻ����������ǽǼ��ٶ�
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed,NULL);  //�������������



#define MAGAZINE_AC_ON      		HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_SET);  //����
#define MAGAZINE_AC_OFF      		HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_RESET);
#define RESCUE1_AC_ON      			HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_SET);  //��Ԯ
#define RESCUE1_AC_OFF      		HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_RESET);  //��Ԯ
#define RESCUE2_AC_ON      			HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_SET);  //��Ԯ
#define RESCUE2_AC_OFF      		HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_RESET);  //��Ԯ

int16_t Order_To_Slaver[4],Order_To_Cloud[4];
//////*******************************��ģʽ���ƺ��������ڴ˷������ĺ���*******************************************************************************/
/*
* @brief  ���ظ����ط�����Ϣ
* @details Main������������ִ��
* @param  NULL
* @retval  NULL
*  2019.12.23 ���
*
*/
void Master_To_Slaver(void)
{

		Order_To_Slaver[0] = bsp_dbus_Data.Dial;  //����
		Order_To_Slaver[1] = bsp_dbus_Data.CH_0;  //���ڿ���צ��ץȡ������������ƶ�
		Order_To_Slaver[2] = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;  //�����жϸ���ģʽ
	  Order_To_Slaver[3] = bsp_dbus_Data.CH_1; 
	  bsp_can_Sendmessage(&COMMUNICATE_CAN,Master_To_Slaver_ID,Order_To_Slaver);  //����
}

/*
* @brief  ���ظ���̨������Ϣ
* @details Main������������ִ��
* @param  NULL
* @retval  NULL
*  2019.12.23 ���
*
*/
void Master_To_Cloud()
{
    Order_To_Cloud[0] = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;  //�����ж���̨ģʽ
		Order_To_Cloud[1] = bsp_dbus_Data.CH_0;  //���ڿ�����̨Yaw
		Order_To_Cloud[2] = bsp_dbus_Data.CH_1;  //���ڿ�����̨Pitch
	  Order_To_Cloud[3] = bsp_dbus_Data.Dial;  //����;  
		bsp_can_Sendmessage(&COMMUNICATE_CAN,Master_To_Cloud_ID,Order_To_Cloud);  //����
} 


////////******************************ң����ģʽ�ַ���غ���*****************************************************************************************/

               
/** 
    * @brief ��ȫģʽ
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
* @brief ���̶���ģʽ
* @detail  ���õ��������ǽ��е��̿���
*/

static float Target_Angle_Gyro;    //������Ŀ��Ƕ�
static float Turn_Angle_Gyro;  //�����Ǽ����ת��Ƕ�
static void Chassis_Independent_Mode(uint8_t type)
{
	switch(type)
	{
		case STARTING:
					  Target_Angle_Gyro = app_imu_Data.integral.Yaw;  //��¼��ǰʱ�������ǽǶ�
			break;
		case RUNNING: 
					//ң����ͨ������
					Target_Angle_Gyro += (bsp_dbus_Data.CH_0)*0.0005f;
					Turn_Angle_Gyro = Target_Angle_Gyro - app_imu_Data.integral.Yaw;  //������Ӧ������̨�Ļ�е�ǶȻ��㣬�����ȴպ�����
					Vx = (bsp_dbus_Data.CH_3) * 14.f*(cos(Turn_Angle_Gyro*pi/180.f) - sin(Turn_Angle_Gyro*pi/180.f)); 
					Vy = (bsp_dbus_Data.CH_2) * (-14.f)*(sin(Turn_Angle_Gyro*pi/180.f) - cos(Turn_Angle_Gyro*pi/180.f));
					Vz = PID_Chassis_Follow_IN.pid_run(PID_Chassis_Follow_OUT.pid_run(Turn_Angle_Gyro) - app_imu_Data.Angle_Rate[2]);  //Yaw�ᴮ������������
					Chassis_Engineer.Run(Vx,Vy,Vz);
					if(bsp_dbus_Data.Dial > 0)  
						{
							MAGAZINE_AC_ON;  //���ա���Ԯͬʱ��
							RESCUE1_AC_ON;
							RESCUE2_AC_ON;
						}
					if(bsp_dbus_Data.Dial < 0)  
						{
							RESCUE1_AC_OFF;  //���ա���Ԯͬʱ��
							RESCUE2_AC_OFF;
							MAGAZINE_AC_OFF;
						}
			break;
		case ENDING:
			break;
  }
}


/** 
* @brief ���̶�λ
* @detail  
*/
#define LEFT_SHARP bsp_ADC1_Sharp_Distance[0]   //����ִ�����
#define RIGHT_SHARP bsp_ADC1_Sharp_Distance[1]   //��ǰ�ִ�����
#define SHARP_BETWEEN 100  //�������������մ������ľ��뵥λΪcm
#define AUTO_PARA_TARGET_DISTANCE 15  //Ŀ�����
#define THRESHOLD_Z	 2.0f		//Z����ת��ֵ
#define THRESHOLD_Y  2.5f   //Y����ת��ֵ

pid Auto_Para_Z_PID(0,0,0,0,0,0);		//�Զ���λZ��PID
//	pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1,uint16_t I_Limited=9999);//��ͳpid���캯��
pid Auto_Para_Y_PID(0,0,0,0,0,0);		//�Զ���λY��PID

float Speed_Z;  //Z����ת�ٶ�
float Speed_Y;	 //Y����ת�ٶ�
static uint8_t Flag_Auto_Para = 0;		//�Զ���λflag��LEFT_SHARPС��50����
static void Chassis_Auto_Para_Mode(uint8_t type)
{
	switch(type)
	{
		case STARTING:
					  if(LEFT_SHARP > 50.f || RIGHT_SHARP >  50.f) Flag_Auto_Para = 0;	//��������λ
						if(LEFT_SHARP < 50.f && RIGHT_SHARP < 50.f) Flag_Auto_Para = 1;	//������λ
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
//* @brief ���̸���ģʽ
//* @detail ������̨��е�ǽ��е��̸���
//*/
//static float Target_Angle_Encoder;  //��̨����������Ŀ��Ƕ�
//static void Chassis_Follow_Mode(uint8_t type)
//{
//  case STARTING:
//					  Target_Angle_Encoder = app_imu_Data.integral.Yaw;  //��¼��ǰʱ�������ǽǶ�
//			break;
//		case RUNNING: 
//					//ң����ͨ������
//					TargetAngle_Gyro += (bsp_dbus_Data.CH_0)*0.0005f;
//					Turn_Angle_Gyro = TargetAngle_Gyro - app_imu_Data.integral.Yaw;  //������Ӧ������̨�Ļ�е�ǶȻ��㣬�����ȴպ�����
//					Vx = (bsp_dbus_Data.CH_3) * 14.f*(cos(Turn_Angle_Gyro*pi/180.f) - sin(Turn_Angle_Gyro*pi/180.f)); 
//					Vy = (bsp_dbus_Data.CH_2) * (-14.f)*(sin(Turn_Angle_Gyro*pi/180.f) - cos(Turn_Angle_Gyro*pi/180.f));
//					Vz = PID_Chassis_Follow_IN.pid_run(PID_Chassis_Follow_OUT.pid_run(Turn_Angle_Gyro) - app_imu_Data.Angle_Rate[2]);  //Yaw�ᴮ������������
//					Chassis_Engineer.Run(Vx,Vy,Vz);
//					if(bsp_dbus_Data.Dial > 0)  
//						{
//							MAGAZINE_AC_ON;  //���ա���Ԯͬʱ��
//							RESCUE1_AC_ON;
//							RESCUE2_AC_ON;
//						}
//					if(bsp_dbus_Data.Dial < 0)  
//						{
//							RESCUE1_AC_OFF;  //���ա���Ԯͬʱ��
//							RESCUE2_AC_OFF;
//							MAGAZINE_AC_OFF;
//						}
//			break;
//		case ENDING:
//			break;
//}

/** 
    * @brief ����ģʽ
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
* @brief  ң����mode�ַ�
* @param [in]  mode ң����s1*10+s2
* @param [in]  type ���в���
* @par ��־    
*							2020.1.16		|	����
*
*/
static void Remote_Distribute(uint8_t mode,uint8_t type)
{
	
	switch(mode)
	{
		case 32:	//צ�ӵ����ʼ������
					Lock_Mode(type);
			break;
		case 12:	//һ��ȡ������
					Lock_Mode(type);
			break;
		case 11:  //����ȡ������
					Lock_Mode(type);
			break;
		case 33:  //���̶���
					Chassis_Independent_Mode(type);
			break;
		case 22:	//��ȫ
					Safe_Mode(type);
			break;
		case 23:	//�Զ���λ
					Chassis_Auto_Para_Mode(type);
			break;
		default :		//��ȫ
					Safe_Mode(type);
			break;
	}
}

/**
* @brief  ң�������ƺ������
* @details  ����ִ�иþ��
* @param  NULL
* @retval  NULL
*/
static int16_t remote_mode = 22;  //��ǰң����ģʽ��Ĭ��Ϊ��ȫ
static int16_t last_mode = 0xff;
void Remote_Control_Handle(void)
{
  remote_mode = bsp_dbus_Data.S1*10 + bsp_dbus_Data.S2;  
	if(remote_mode != last_mode)  //�����ǰģʽ��֮ǰ��ģʽ��һ���Ļ�
	{
	  Remote_Distribute(last_mode,ENDING);  //�˳�֮ǰ��ģʽ
		Remote_Distribute(remote_mode,STARTING);  //���õ�ǰģʽ ��ʼ����
		Remote_Distribute(remote_mode,RUNNING);  //��һ�ε�ǰģʽ��running
		last_mode = remote_mode;
	}
	else Remote_Distribute(last_mode,RUNNING);  //������ǰģʽ
}

