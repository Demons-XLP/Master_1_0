#include "task_main.h"
#include "cmsis_os.h"
#include "freertos.h"
#include  "bsp_adc_deal.h"
#include "app_imu.h"
#include "app_math.h"
#include "task.h"
#include "can.h"
#include "bsp_motor.hpp"
#include "bsp_mpu9250.h"
#include "bsp_dbus.h"
#include "bsp_can.hpp"
#include  "TFmini.h"
#include "global_define.h"
#include "global_variable.h"
#include "app_remote.h"

uint8_t Omron[3] = {1,1,1};  //������ŷķ������һ���ӿڵ�����


void MainTask(void const * argument)
{ 
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡ��ǰ��ϵͳʱ��
	
	taskENTER_CRITICAL();  //�����ٽ�����
	bsp_mpu9250_Init();  //����9250Ӳ��
	app_imu_Init();  //�����ʼ��
	taskEXIT_CRITICAL();//�˳��ٽ���
	
	bsp_ADC_Sensor_Init(); //������
	TFmini_Init();  //���Ѵ�����
	bsp_dbus_Init();  //DBUS
	bsp_can_Init();  //CAN��ʼ��
	manager::CANSelect(&hcan1,&hcan2);  //ѡ��CAN1��CAN2
  for(;;)
	{
		app_imu_Calculate();   //��������̬����
		Master_To_Slaver();   //���ظ����ط�������
	  Master_To_Cloud();  //���ظ���̨��������
 		Chassis_Engineer.Handle();  //�����й�
		manager::CANSend();  //����й�
		Remote_Control_Handle(); //ң�������ƾ��
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS); //��������ʱ
	}
	
}





/** 
* @brief  GPIO�ⲿ�жϵĻص���������������У����ù���
* ��־    
*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
	{
		case Omron1_Pin:
			Omron[0] = HAL_GPIO_ReadPin(Omron1_GPIO_Port,Omron1_Pin);		//��ǰ��ŷķ��
		break;
		case Omron2_Pin:
			Omron[1] = HAL_GPIO_ReadPin(Omron2_GPIO_Port,Omron2_Pin);		//��ǰ��ŷķ��
		break;
		case Omron3_Pin:
			Omron[2] = HAL_GPIO_ReadPin(Omron3_GPIO_Port,Omron3_Pin);		//����
		break;
	  
	}
}	
