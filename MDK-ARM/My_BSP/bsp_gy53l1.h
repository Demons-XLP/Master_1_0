/**
  * @file      bsp_gy53l1.h
  * @brief     低成本激光测距模块 GY-53/VL53L1x (支持UART通信) 驱动函数
  * @details   
  * @author   ThunderDoge
  * @date      2020-3-14
  * @version   1.1
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
  * Using encoding: UTF-8
  * v1.0    2020-3-21   根据手册实现了gy-53/vl53l1x的基本功能
  * v2.0    2020-4-11   为了照顾到使用多个gy53/vl53l1x的机器人。所以改成了用 .c 模拟面向对象
  */
#ifndef	__BSP_GY51VL1_H
#define	__BSP_GY51VL1_H

#include "usart.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define GY53L1_RXBUF_LEN 16



/**
 * @brief 模块返回当前状态的取值
 */
enum Gy53l1_RangeStatus {  
    GY53L1_RANGE_VALID  = 0,        //距离可靠
    GY53L1_SIGMA_FAIL   = 1,        //周围环境光影响
    GY53L1_OUTOFBOUNDS_FAIL = 4,    //超出测量量程
    GY53L1_HARDWARE_FAIL    = 5,    //硬件故障
    GY53L1_WARP_TARGET_FAIL = 7,    //周围环境有干扰噪声
    GY53L1_PROCESSING_FAIL  = 8,    //内部算法的错误或溢出
    GY53L1_RANGE_INVALID    = 14,   //无效测量
};



enum Gy53l1_RangeTime {
    GY53L1_TIME_110MS    = 0X01,
    GY53L1_TIME_200MS    = 0X02,    //默认
    GY53L1_TIME_300MS    = 0X03,
    GY53L1_TIME_55MS     = 0X00,
};



enum Gy53l1_RangeMode {
    GY53L1_SHORT_RANGE   = 0X01,
    GY53L1_MID_RANGE     = 0X02,
    GY53L1_LONG_RANGE    = 0X03,
};



/**
 * @brief 对模块串口发送指令 command 字段取值-枚举型定义
 */
enum Gy53l1_UartCommand{
    GY53L1_CONTINUOUS_OUTPUT    = 0x45,
    GY53L1_ONCE_OUTPUT          = 0X15,

    GY53L1_SAVE_CONFIG          = 0X25,

    GY53L1_SHORT_RANGE_CONFIG   = 0X51,
    GY53L1_MID_RANGE_CONFIG     = 0X52,
    GY53L1_LONG_RANGE_CONFIG    = 0X53,

    GY53L1_TIME_110MS_CONFIG    = 0X61,
    GY53L1_TIME_200MS_CONFIG    = 0X62,
    GY53L1_TIME_300MS_CONFIG    = 0X63,
    GY53L1_TIME_55MS_CONFIG     = 0X64,

    GY53L1_BAUD_9600_CONFIG     = 0XAE,
    GY53L1_BAUD_115200_CONFIG   = 0XAF,
};

// 
extern uint8_t Gy53l1_CheckBytes[4];

///模块数据结构体
typedef struct{
    uint32_t update_time;
	int16_t	distance;       ///<模块返回的距离，单位mm
	enum Gy53l1_RangeStatus		range_status;   ///<模块返回当前状态的取值 @see Gy53l1_RangeStatus
	enum Gy53l1_RangeTime		range_time;     ///<模块测量时间，该值越大，测距效果越好    取值 @see Gy53l1_RangeTime
	enum Gy53l1_RangeMode		range_mode;     ///<测量模式    取值 @see Gy53l1_RangeMode
}Gy53l1_DataType;

// extern Gy53l1_DataType Gy53l1_Data;	///模块数据

/// 激光测距模块GY53L1对象
typedef struct
{
    UART_HandleTypeDef* uart_interface;
    uint8_t init_flag;
    uint8_t RxBuffer[GY53L1_RXBUF_LEN];
	uint8_t RxBufferIT[GY53L1_RXBUF_LEN];
    Gy53l1_DataType     data;
}bsp_GY53L1_Object;

/// 激光测距模块GY53L1对象 初始化
void bsp_GY53L1_Object_Init(bsp_GY53L1_Object* object,UART_HandleTypeDef* uart_interface);

/// 向 激光测距模块GY53L1对象 发送命令
HAL_StatusTypeDef bsp_GY53L1_Object_SendCommand(bsp_GY53L1_Object* object,enum Gy53l1_UartCommand command);

/// 激光测距模块GY53L1对象 串口接收完成回调函数
void bsp_GY53L1_Object_Idle_RxCpltCallback(bsp_GY53L1_Object* object);




#endif	//__BSP_GY51VL1_H


