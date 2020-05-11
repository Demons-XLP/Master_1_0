
/** 
	* @brief    ���ؽ���
	* @details  ͨ��CAN���ղ��������ص����ݣ�����������Master_Order[]����
	* @author   Xu LiangPu
	* @date     2019.12.21 
	* @version  1.0
	* @par 
	* @par ��־
*/

#include "app_my_can.h"
#include "can.h"
#include "bsp_motor.hpp"
#include "string.h"
#include "global_define.h"

CAN_RxHeaderTypeDef RxHead;

int16_t Slaver_Feedback[4],Cloud_Feedback[4];  //�������ŵ���������
uint8_t data[8];//�������ݻ�����


void Slaver_Feedback_Caculate(uint8_t data[]);	//���ط�����������
void Cloud_Feedback_Caculate(uint8_t data[]);  //��̨������������



/**
* @brief  CAN�����ж�
* @details  ���¶�������жϣ����Զ���CAN�ж��е���
*           Ϊ��ʹ�����ܽ��յ����ص�������ⲿ���¶����CAN�����ж�
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
* @brief   �������ط��͵�����
* @remarks ��������������ִ��
* ��־    
*/
void Slaver_Feedback_Caculate(uint8_t data[])  //���ؽ�������
{
  Slaver_Feedback[0] = data[0]  << 8 | data[1];  //��̨�������������Ļ�е��
	Slaver_Feedback[1] = data[2]  << 8 | data[3];  
	Slaver_Feedback[2] = data[4]  << 8 | data[5];  
	Slaver_Feedback[3] = data[6]  << 8 | data[7];  
}

/** 
* @brief   ������̨���͵�����
* @remarks ��������������ִ��
* ��־    
*/
void Cloud_Feedback_Caculate(uint8_t data[])  //���ؽ�������
{
  Cloud_Feedback[0] = data[0]  << 8 | data[1];  //��̨�������������Ļ�е��
	Cloud_Feedback[1] = data[2]  << 8 | data[3];  
	Cloud_Feedback[2] = data[4]  << 8 | data[5];  
	Cloud_Feedback[3] = data[6]  << 8 | data[7];  
}
