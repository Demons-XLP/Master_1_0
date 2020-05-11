#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "stm32f4xx.h"
//#include "app_motor.h"
#include "can.h"

//条件编译开关宏定义，不需要开启的就注释掉相关宏定义
#define	BSP_CAN_USE_FREERTOS
//外设相关宏定义,移植时如果修改了外设请在这里修改
#define BSP_CAN_USE_CAN1					hcan1
#define BSP_CAN_USE_CAN2					hcan2

void bsp_can_Init(void);	//CAN总线初始化函数
HAL_StatusTypeDef bsp_can_Sendmessage(CAN_HandleTypeDef* hcan,int16_t StdId,int16_t* Can_Send_Data);	//CAN总线数据发送函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);	//重定义的CAN中断回调函数
#endif
