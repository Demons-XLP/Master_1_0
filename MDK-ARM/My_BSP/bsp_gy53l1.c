/**
  * @file      bsp_gy53l1.c
  * @brief     低成本激光测距模块 GY-53/VL53L1x (支持UART通信) 驱动函数
  * @details   使用方法：
  * 1. 初始化串口为 波特率9600 8位数据 无校验 1停止位 （即模块默认参数）
  * 2. 定义一个 bsp_GY53L1_Object 结构体，并且初始化 bsp_GY53L1_Object_Init()
  * 3. 根据 Gy53l1_UartCommand 向模块发送指令
  * @author   ThunderDoge
  * @date      2020-3-14
  * @version   1.0
  * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
                           Using encoding: UTF-8
        version     Date            Author          Details
        v1.0        2020-3-16       ThunderDoge
  实现了基础功能。运行测试通过。上传公共代码库。
  */
#include "bsp_gy53l1.h"

uint8_t Gy53l1_CheckBytes[4] = {0x5a, 0x5a, 0x15,
                                0x03}; /// 接收帧头标志对照表。Byte[0~3]


/**
 * @brief     激光测距模块GY53L1对象 初始化
 *
 * @param     object            模块对象的指针
 * @param     uart_interface    模块所用的串口句柄的指针
 */
void bsp_GY53L1_Object_Init(bsp_GY53L1_Object *object,
                            UART_HandleTypeDef *uart_interface)
{
	// 装载默认数值
    object->init_flag = 1;
    object->uart_interface = uart_interface;
    object->data.distance = 0;
    object->data.range_mode = GY53L1_SHORT_RANGE;
    object->data.range_status = GY53L1_RANGE_INVALID;
    object->data.range_time = GY53L1_TIME_200MS;
	
	__HAL_UART_ENABLE_IT(object->uart_interface,UART_IT_IDLE);	// 开启空闲中断
	
	// 确认初始化成功
    if(bsp_GY53L1_Object_SendCommand(object,GY53L1_CONTINUOUS_OUTPUT)!= HAL_OK)
		object->init_flag = 0;
		
	// 开启接收
	HAL_UART_Receive_DMA(object->uart_interface,object->RxBuffer,8);
}

/**
 * @brief     激光测距模块GY53L1 接收数据 格式分析
 * 
 * @param     object    模块对象的指针
 * @param     RxXfer    接收数据的缓存的指针
 */
static HAL_StatusTypeDef bsp_GY53L1_Object_DataAnalyze(bsp_GY53L1_Object* object)
{
    int frame_head_fitted = 0;
	uint8_t* pdata = object->RxBuffer;

    // 检查帧头格式符合
    for (int i = 0; i < 4; i++)
    {
        if (pdata[i] == Gy53l1_CheckBytes[i]) // 对照 Gy53l1_CheckBytes 进行检查
        {
            ++frame_head_fitted;
        }
    }

    if (frame_head_fitted < 4) // 若不符合
        return HAL_ERROR;      // 返回错误
    else
    { //写入数据。规则见模块手册
        object->data.distance = (enum Gy53l1_RangeStatus)pdata[4] << 8 | pdata[5];
        object->data.range_status = (enum Gy53l1_RangeStatus)(pdata[6] >> 4 & 0x0f); // |(x) &(√) 
        object->data.range_time = (enum Gy53l1_RangeTime)(pdata[6] >> 2 & 0x03);
        object->data.range_mode = (enum Gy53l1_RangeMode)(pdata[6] & 0x03);

        object->data.update_time = HAL_GetTick();
    }
    return HAL_OK; //返回正常
}

/**
 * @brief     向 激光测距模块GY53L1对象 发送命令
 * 
 * @param     object    模块对象的指针
 * @param     command   指令，枚举量
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef bsp_GY53L1_Object_SendCommand(bsp_GY53L1_Object* object,enum Gy53l1_UartCommand command)
{

    uint8_t data_to_send[3]; // 依格式发送数据
    data_to_send[0] = 0XA5;
    data_to_send[1] = (uint8_t)command;
    data_to_send[2] = (data_to_send[0] + data_to_send[1]) & 0XFF;
	
    return HAL_UART_Transmit(object->uart_interface, data_to_send, 3,100); //返回状态
}


/**
 * @brief 激光测距模块GY53L1对象 串口接收完成回调函数
 * 
 * @param     object 模块对象的指针
 */
void bsp_GY53L1_Object_Idle_RxCpltCallback(bsp_GY53L1_Object* object)
{
//	HAL_UART_AbortReceive_IT(object->uart_interface);
	
	if (bsp_GY53L1_Object_DataAnalyze(object) == HAL_OK) // 解析数据
	{
	
	}
	else
	{
		// 你在对该程序DEBUG时可以
		// while(1);
	}
		memset(object->RxBuffer, 0, 8); //清空缓存
		HAL_UART_Receive_DMA(object->uart_interface, object->RxBuffer,
							8); //重启接收

}



