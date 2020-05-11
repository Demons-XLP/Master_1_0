//#include "my_car.hpp"
//#include "bsp_motor.hpp"

////底盘四个3508电机
//Motor_t DJI_Motor_3508(8192, 19);  //电机类型
//pid PID_Chassis_Speed(5,0.1f,0,5000,16000,0,0,80);  //底盘电机PID
//pid PID_Chassis_Follow_OUT(167,0,0,10000,10000,0,4,10);  //底盘跟随PID外环，用陀螺仪角度（无死区）
//pid PID_Chassis_Follow_IN(1.3,0,0,10000,10000,0,0,1000);  //底盘跟随PID内环，用陀螺仪角加速度
//chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed,NULL);  //创建底盘类对象

