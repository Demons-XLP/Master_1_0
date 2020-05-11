/** 
* @brief    Dbus板级支持包
* @details  Dbus数据接收，解析
* @author   Evan-GH
* @date      2019.10
* @version  1.3
* @par Copyright (c):  RM2020电控
* @par 	具体使用方法见Readme.md
				版本变更:
				1.0		|		调通Dbus总线相关中断处理函数,数据解析函数
				1.1		|		添加对接收数据的有效性和正确性的校验
				1.2		|		加入条件编译,分为使用信号量和普通中断两种
				1.3		|		增加对接收缓存数组处理之后的清0操作，确保数据校验准确
				1.4		|		根据代码规范修改了一些函数名称
				1.5		|		修改一些小bug
*/
#include "bsp_dbus.h"

bsp_dbus_RC_Data bsp_dbus_Data; //Dbus解算数据
static uint8_t Dbus_Rxbuffer[BSP_DBUS_BUFFER_SIZE]={0}; //Dbus接收数据缓存数组

/**
* @brief  Dbus总线初始化
* @details  初始化Dbus中断设置，开启DMA接收空闲中断
* @param  NULL
* @retval  NULL
*/
void bsp_dbus_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&BSP_DBUS_UART); //清除空闲中断位
	__HAL_UART_ENABLE_IT(&BSP_DBUS_UART,UART_IT_IDLE); //使能DMA接收空闲中断
	HAL_UART_Receive_DMA(&BSP_DBUS_UART,Dbus_Rxbuffer,BSP_DBUS_BUFFER_SIZE); //开始DMA接收
}

/**
* @brief  Dbus总线数据校验
* @details  校验接收数据的有效性和正确性
* @param  NULL
* @retval  HAL_OK 数据正常  HAL_ERROR 数据有误
*/
static HAL_StatusTypeDef bsp_dbus_Datacheck(void)
{
	int i=0;
	for(i=0;i<18;i++)				//首先检验前面18位数据是否有效，如果全为0显然是无效数据
	{
		if(Dbus_Rxbuffer[i]==0)
		{
			continue;					
		}
		else
		{
			break;
		}
	}
	if(i==18)
	{
		return HAL_ERROR;				//前18位数据全部为0，返回错误
	}
	for(i=18;i<25;i++)											//对后面的数据进行检验，如果不为0显然数据出错
	{
		if(Dbus_Rxbuffer[i]!=0)
		{
			return HAL_ERROR;			//后面几位数据出现了非0，说明数据出错
		}
	}
	return HAL_OK;
}

/**
* @brief  Dbus数据解析
* @details  解析Dbus接收到的数据并更新到结构体，并归一化
* @param  NULL
* @retval  NULL
*/
#ifdef BSP_DBUS_USE_SIGNAL
	void bsp_dbus_Analysis(void)
	{
		// 遥控数据原始解析，这部分代码直接参考官方示例代码即可
		//遥控器控制部分
		bsp_dbus_Data.CH_0 = ((int16_t)Dbus_Rxbuffer[0] | (((int16_t)Dbus_Rxbuffer[1]<<8))) & 0x07FF;
		bsp_dbus_Data.CH_1 = ((int16_t)Dbus_Rxbuffer[1] >> 3 | (((int16_t)Dbus_Rxbuffer[2]<<5))) & 0x07FF;
		bsp_dbus_Data.CH_2 = ((int16_t)Dbus_Rxbuffer[2] >> 6 | (((int16_t)Dbus_Rxbuffer[3]<<2)) | ((int16_t)Dbus_Rxbuffer[4])<<10) & 0x07FF;
		bsp_dbus_Data.CH_3 = ((int16_t)Dbus_Rxbuffer[4] >> 1 | (((int16_t)Dbus_Rxbuffer[5]<<7)) ) & 0x07FF;
		bsp_dbus_Data.S1 = ((Dbus_Rxbuffer[5] >> 4) & 0X000C) >> 2;
		bsp_dbus_Data.S2 = (Dbus_Rxbuffer[5] >> 4) & 0X0003;			
		bsp_dbus_Data.Dial = ((int16_t)Dbus_Rxbuffer[16] | ((int16_t)Dbus_Rxbuffer[17] << 8)) & 0x07FF;		//拨码盘数据，需要更新遥控器固件
		
		//键鼠控制部分
		bsp_dbus_Data.Mouse.X = ((int16_t)Dbus_Rxbuffer[6]) | ((int16_t)Dbus_Rxbuffer[7] << 8);
		bsp_dbus_Data.Mouse.Y = ((int16_t)Dbus_Rxbuffer[8]) | ((int16_t)Dbus_Rxbuffer[9] << 8);
		bsp_dbus_Data.Mouse.Z = ((int16_t)Dbus_Rxbuffer[10]) | ((int16_t)Dbus_Rxbuffer[11] << 8);
		bsp_dbus_Data.Mouse.Leftkey = Dbus_Rxbuffer[12];
		bsp_dbus_Data.Mouse.Rightkey = Dbus_Rxbuffer[13];
		bsp_dbus_Data.Keys = ((int16_t)Dbus_Rxbuffer[14] | (int16_t)Dbus_Rxbuffer[15]<<8);
	}
#else
	static void bsp_dbus_Analysis(void)
	{
		// 遥控数据原始解析，这部分代码直接参考官方示例代码即可
		//遥控器控制部分
		bsp_dbus_Data.CH_0 = (((int16_t)Dbus_Rxbuffer[0] | (((int16_t)Dbus_Rxbuffer[1]<<8))) & 0x07FF) - 1024;
		bsp_dbus_Data.CH_1 = (((int16_t)Dbus_Rxbuffer[1] >> 3 | (((int16_t)Dbus_Rxbuffer[2]<<5))) & 0x07FF) -1024;
		bsp_dbus_Data.CH_2 = (((int16_t)Dbus_Rxbuffer[2] >> 6 | (((int16_t)Dbus_Rxbuffer[3]<<2)) | ((int16_t)Dbus_Rxbuffer[4])<<10) & 0x07FF) -1024;
		bsp_dbus_Data.CH_3 = (((int16_t)Dbus_Rxbuffer[4] >> 1 | (((int16_t)Dbus_Rxbuffer[5]<<7)) ) & 0x07FF) -1024;
		bsp_dbus_Data.S1 = ((Dbus_Rxbuffer[5] >> 4) & 0X000C) >> 2;
		bsp_dbus_Data.S2 = (Dbus_Rxbuffer[5] >> 4) & 0X0003;			
		bsp_dbus_Data.Dial = (((int16_t)Dbus_Rxbuffer[16] | ((int16_t)Dbus_Rxbuffer[17] << 8)) & 0x07FF ) - 1024;		//拨码盘数据，需要更新遥控器固件
		
		//键鼠控制部分
		bsp_dbus_Data.Mouse.X = ((int16_t)Dbus_Rxbuffer[6]) | ((int16_t)Dbus_Rxbuffer[7] << 8);
		bsp_dbus_Data.Mouse.Y = ((int16_t)Dbus_Rxbuffer[8]) | ((int16_t)Dbus_Rxbuffer[9] << 8);
		bsp_dbus_Data.Mouse.Z = ((int16_t)Dbus_Rxbuffer[10]) | ((int16_t)Dbus_Rxbuffer[11] << 8);
		bsp_dbus_Data.Mouse.Leftkey = Dbus_Rxbuffer[12];
		bsp_dbus_Data.Mouse.Rightkey = Dbus_Rxbuffer[13];
		bsp_dbus_Data.Keys = ((int16_t)Dbus_Rxbuffer[14] | (int16_t)Dbus_Rxbuffer[15]<<8);
	}
#endif

/**
* @brief  Dbus中断处理函数
* @details  释放二值信号量以供任务进行同步处理，放在Dbus所使用的串口的总体中断中调用 注意对数据有效性和正确性进行检测
* @param  NULL
* @retval  NULL
*/
void bsp_dbus_It(void)
{
	if(__HAL_UART_GET_FLAG(&BSP_DBUS_UART,UART_FLAG_IDLE) != RESET)	//如果产生了空闲中断
	{		
		HAL_UART_DMAStop(&BSP_DBUS_UART); //关闭DMA
		//在解析数据之前首先确定数据的有效性和准确性
		if(bsp_dbus_Datacheck() == HAL_OK)
		{			
			#ifdef BSP_DBUS_USE_SIGNAL		//如果使用信号量
				static BaseType_t Dbus_xHigherPriorityTaskWoken;
				xSemaphoreGiveFromISR(Dbus_Update_Handle,&Dbus_xHigherPriorityTaskWoken); //释放信号量，更新Dbus数据
				xSemaphoreGiveFromISR(Dbus_Check_Handle, &Dbus_xHigherPriorityTaskWoken);	//释放Dbus离线检测二值信号量
			#else													//没有使用信号量
				bsp_dbus_Analysis(); //直接在中断里面解析数据
			#endif
		}
		memset(Dbus_Rxbuffer,0,BSP_DBUS_BUFFER_SIZE);	//对接收缓存数据清0
		//调试的时候可以取消下面语句注释把数据打印出来监控，注意printf是阻塞型函数，可能导致系统卡死，可以放进周期性函数中打印数据
		//注意：使用下面语句的前提是你已经重定向了printf，并且明确知道哪个串口是输出！
		//printf("CH0:%d CH1:%d CH2:%d CH3:%d\n",bsp_dbus_Data.CH_0,bsp_dbus_Data.CH_1,bsp_dbus_Data.CH_2,bsp_dbus_Data.CH_3);
		//printf("S1:%d S2:%d\n",bsp_dbus_Data.S1,bsp_dbus_Data.S2);
		__HAL_UART_CLEAR_IDLEFLAG(&BSP_DBUS_UART); //清除空闲中断标志位
		HAL_UART_Receive_DMA(&BSP_DBUS_UART, (uint8_t*)Dbus_Rxbuffer, BSP_DBUS_BUFFER_SIZE);//重新开启DMA接收传输
	}
}
