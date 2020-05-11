#ifndef __GLOBAL_DEFINE_H
#define __GLOBAL_DEFINE_H
#include "can.h"

#define pi 3.1415926f
/*板子间通信相关*/
#define COMMUNICATE_CAN  hcan2
#define Master_To_Cloud_ID 0x100 //主控给云台发
#define Master_To_Slaver_ID 0x101  //主控给副控发
#define SlaverID 0x102  //副控标识符ID
#define CloudID  0x103  //云台标识符ID
/*资源岛对位相关宏定义*/
#define Distance_To_Island  10.f   //取弹时车子与资源岛之间的距离
#define Error_Between_Sharp   //两个传感器读取到的数据误差值，作为是否要转动yaw轴的判断条件

/*遥控器模式分发相关*/
#define STARTING  0  //启动时参数
#define RUNNING   1  //运行时参数
#define ENDING    2  //结束时参数

#endif
