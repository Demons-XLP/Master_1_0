#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#include "bsp_motor.hpp"
void Remote_Control_Handle();  //遥控器控制函数句柄
void Master_To_Slaver(void);  //主控给云台发信息
void Master_To_Cloud();  //主控给副控发信息

extern Motor_t DJI_Motor_3508;  //电机类型
extern pid PID_Chassis_Speed;  //底盘电机PID
extern pid PID_Chassis_Follow_OUT;  //底盘跟随PID外环，用陀螺仪角度（无死区）
extern pid PID_Chassis_Follow_IN;  //底盘跟随PID内环，用陀螺仪角加速度
extern chassis Chassis_Engineer;  //创建底盘类对象

#endif
