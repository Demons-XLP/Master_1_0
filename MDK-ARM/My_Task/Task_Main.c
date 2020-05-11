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

uint8_t Omron[3] = {1,1,1};  //底盘俩欧姆龙，多一个接口当备用


void MainTask(void const * argument)
{ 
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取当前的系统时间
	
	taskENTER_CRITICAL();  //进入临界区域
	bsp_mpu9250_Init();  //配置9250硬件
	app_imu_Init();  //解算初始化
	taskEXIT_CRITICAL();//退出临界区
	
	bsp_ADC_Sensor_Init(); //传感器
	TFmini_Init();  //北醒传感器
	bsp_dbus_Init();  //DBUS
	bsp_can_Init();  //CAN初始化
	manager::CANSelect(&hcan1,&hcan2);  //选择CAN1和CAN2
  for(;;)
	{
		app_imu_Calculate();   //陀螺仪姿态解算
		Master_To_Slaver();   //主控给副控发送数据
	  Master_To_Cloud();  //主控给云台发送数据
 		Chassis_Engineer.Handle();  //底盘托管
		manager::CANSend();  //电机托管
		Remote_Control_Handle(); //遥控器控制句柄
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS); //定周期延时
	}
	
}





/** 
* @brief  GPIO外部中断的回调函数，扔这里就行，不用管它
* 日志    
*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
	{
		case Omron1_Pin:
			Omron[0] = HAL_GPIO_ReadPin(Omron1_GPIO_Port,Omron1_Pin);		//左前轮欧姆龙
		break;
		case Omron2_Pin:
			Omron[1] = HAL_GPIO_ReadPin(Omron2_GPIO_Port,Omron2_Pin);		//右前轮欧姆龙
		break;
		case Omron3_Pin:
			Omron[2] = HAL_GPIO_ReadPin(Omron3_GPIO_Port,Omron3_Pin);		//备用
		break;
	  
	}
}	
