#ifndef __MY_CAR_HPP
#define __MY_CAR_HPP
#include "bsp_motor.hpp"


extern pid PID_Chassis_Speed;  //底盘电机PID
extern pid PID_Chassis_Follow_OUT;  //底盘跟随PID外环，用陀螺仪角度（有死区）
extern pid PID_Chassis_Follow_IN;  //底盘跟随PID内环，用陀螺仪角加速度
extern chassis Chassis_Engineer;  //创建底盘类对象





#endif
