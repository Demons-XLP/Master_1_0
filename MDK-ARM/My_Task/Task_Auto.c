//#include "task_auto.h"
//#include "bsp_adc_deal.h"
//#include "math.h"
//#include "bsp_motor.hpp"
//#include "my_car.hpp"
//#include "global_variable.h"


//#define LeftSharp bsp_ADC1_Sharp_Distance[0]  //����ִ�����
//#define RightSharp bsp_ADC1_Sharp_Distance[1]  //��ǰ�ִ�����
//#define SharpBetween 100  //�������������մ������ľ��뵥λΪcm
//#define AutoPara_TargetDistance 10

//pid PID_AutoPara(0,0,0,10000,10000,0,0,0);
////	pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1,uint16_t I_Limited=9999);//��ͳpid���캯��

//float TurnAngle;
//void AutoTask()
//{
//	if(ABS(LeftSharp - RightSharp) > 3)  //�����ж��費��Ҫ��ת
//	{
//    TurnAngle = - asin((RightSharp - LeftSharp) / SharpBetween);  //���ڼ�����Ҫ��ת�ĽǶ�
//		Vz = TurnAngle*50;
//	}
//	else 
//	{
//		if((LeftSharp - AutoPara_TargetDistance) > 2.5f)  //��������2.5ʱ����PID����
//		{
//	   Vy = PID_AutoPara.pid_run(LeftSharp - AutoPara_TargetDistance);  
//		}
//		else Vy = 0;  //�����2.5�ڲ����е���
//	}
//	Chassis_Engineer.Run(0,Vy,Vz);
//	
//}
//	
//	
