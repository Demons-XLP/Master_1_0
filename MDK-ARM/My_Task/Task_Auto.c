//#include "task_auto.h"
//#include "bsp_adc_deal.h"
//#include "math.h"
//#include "bsp_motor.hpp"
//#include "my_car.hpp"
//#include "global_variable.h"


//#define LeftSharp bsp_ADC1_Sharp_Distance[0]  //左后轮传感器
//#define RightSharp bsp_ADC1_Sharp_Distance[1]  //左前轮传感器
//#define SharpBetween 100  //车子上两个夏普传感器的距离单位为cm
//#define AutoPara_TargetDistance 10

//pid PID_AutoPara(0,0,0,10000,10000,0,0,0);
////	pid(float P, float I, float D, float IMax, float PIDMax, uint16_t I_Time=1, uint16_t D_Time=1,uint16_t I_Limited=9999);//传统pid构造函数

//float TurnAngle;
//void AutoTask()
//{
//	if(ABS(LeftSharp - RightSharp) > 3)  //用于判断需不需要旋转
//	{
//    TurnAngle = - asin((RightSharp - LeftSharp) / SharpBetween);  //用于计算需要旋转的角度
//		Vz = TurnAngle*50;
//	}
//	else 
//	{
//		if((LeftSharp - AutoPara_TargetDistance) > 2.5f)  //当误差大于2.5时进行PID调节
//		{
//	   Vy = PID_AutoPara.pid_run(LeftSharp - AutoPara_TargetDistance);  
//		}
//		else Vy = 0;  //误差在2.5内不进行调节
//	}
//	Chassis_Engineer.Run(0,Vy,Vz);
//	
//}
//	
//	
