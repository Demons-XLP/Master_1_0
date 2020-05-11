/** 
* @file	   CarDrv_config.hpp
* @brief    用户自定义参数头文件
* @details  该头文件下的参数与Car_Driver电机驱动库有关，用户可以自行配置，建议在此配置库选项，以保证库文件不改动 \n
*           当没有出现新的参数的时候，不要提交本文件避免专用参数提交到通用参数里
* @author   WMD
* @date     2019年1月17日19:58:05
* @version  0.1
* @par Copyright (c):  
*       WMD 
* @par 日志
*       V0.1	WMD觉得需要有这一个文件，所以这个文件就被创建了
*				V0.2	Evan-GH修改了这个文件的文件名来适应代码规范
*/  
#ifndef __BSP_CAR_CONFIG_HPP
#define __BSP_CAR_CONFIG_HPP

///选定使用的STM32的头文件
#include "stm32f4xx_hal.h"

///是否使用自定义的SYS函数（影响部分标准函数如printf）
//#define USE_OWM_SYS_DEF
//#include "usart.h"

#define LPF_NUM (0.3f)      //!<前馈低通滤波器的参数
#define ChassisMax 15000    //!<电机库中底盘的最大速度

typedef enum{
    Stop=0x00U,              //!<电机停止工作
    Speed_Ctl=0x01U,         //!<电机使用速度环工作
    Position_Ctl=0x02U,      //!<电机使用路程环工作
    Gyro_Position_Ctl=0x03U, //!<电机在陀螺仪环境下工作(Cloud子类专用)
    Gyro_Speed_Ctl=0x04U,    //!<电机在陀螺仪环境下工作(Cloud子类专用)
    ///在子类需要拓展运行模式时需要在这里加
}RunState_t;

//定义底盘功率限制
#define LIMIT_P (40)
#endif 
