
/** 
	* @brief    主控解析
	* @details  通过CAN接收并解析主控的数据，解析后存放在Master_Order[]里面
	* @author   Xu LiangPu
	* @date     2019.12.21 
	* @version  1.0
	* @par 
	* @par 日志
*/

#include "app_my_can.h"
#include "can.h"
#include "bsp_motor.hpp"
#include "string.h"
#include "global_define.h"

CAN_RxHeaderTypeDef RxHead;

int16_t Slaver_Feedback[4],Cloud_Feedback[4];  //解析后存放的主控数据
uint8_t data[8];//接收数据缓冲区


void Slaver_Feedback_Caculate(uint8_t data[]);	//副控反馈解析函数
void Cloud_Feedback_Caculate(uint8_t data[]);  //云台反馈解析函数



/**
* @brief  CAN接收中断
* @details  重新定义接收中断，会自动在CAN中断中调用
*           为了使副控能接收到主控的命令，在外部重新定义的CAN接收中断
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, (uint8_t*)data);
	if(hcan == &COMMUNICATE_CAN)
	{
		switch(RxHead.StdId)
		{
		  case SlaverID:
								Slaver_Feedback_Caculate(data);
				break;
			case CloudID:
								Cloud_Feedback_Caculate(data);
				break;
			default :
								manager::CANUpdate(hcan, &RxHead, (uint8_t*)data);
				break;
		}
	}
	else manager::CANUpdate(hcan, &RxHead, (uint8_t*)data);
}


/** 
* @brief   解析副控发送的数据
* @remarks 在主任务中周期执行
* 日志    
*/
void Slaver_Feedback_Caculate(uint8_t data[])  //主控解析函数
{
  Slaver_Feedback[0] = data[0]  << 8 | data[1];  //云台反馈的软件意义的机械角
	Slaver_Feedback[1] = data[2]  << 8 | data[3];  
	Slaver_Feedback[2] = data[4]  << 8 | data[5];  
	Slaver_Feedback[3] = data[6]  << 8 | data[7];  
}

/** 
* @brief   解析云台发送的数据
* @remarks 在主任务中周期执行
* 日志    
*/
void Cloud_Feedback_Caculate(uint8_t data[])  //主控解析函数
{
  Cloud_Feedback[0] = data[0]  << 8 | data[1];  //云台反馈的软件意义的机械角
	Cloud_Feedback[1] = data[2]  << 8 | data[3];  
	Cloud_Feedback[2] = data[4]  << 8 | data[5];  
	Cloud_Feedback[3] = data[6]  << 8 | data[7];  
}
