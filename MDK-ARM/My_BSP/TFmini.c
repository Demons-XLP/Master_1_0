#include "TFmini.h"
#include "usart.h"

#define TFmini_Offset  5

static unsigned char TFmini_Rx_buff[Data_Length]={0};//主机通信数据储存
uint16_t TFmini_Distance;
uint16_t Signal_Strength;  //在20-2000之间可信
uint8_t  TFmini_Online;

static uint8_t TFmini_RxData_Save(void)
{
	if(TFmini_Rx_buff[0] == TFmini_HeadByte && TFmini_Rx_buff[1] == TFmini_HeadByte){
		TFmini_Online = 1;
		TFmini_Distance = (TFmini_Rx_buff[3]<<8 | TFmini_Rx_buff[2]) + TFmini_Offset;
		Signal_Strength = TFmini_Rx_buff[5]<<8 | TFmini_Rx_buff[4];
		return 0;
	}
	else
		return 1;
}

/** 
* @brief   
* @remarks 
* @todo    
* @bug   1、
*/
void TFmini_Init(void)
{
	__HAL_UART_ENABLE_IT(&TFmini_USART,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&TFmini_USART,TFmini_Rx_buff,Data_Length);
}

/** 
* @brief   中断处理函数
* @remarks 需要放进PC_UART_IRQHandler中
* @todo    
* @bug   1、
*/
void TFmini_IT(void) 
{
	HAL_UART_DMAStop(&TFmini_USART);
  TFmini_RxData_Save();
	__HAL_UART_CLEAR_IDLEFLAG(&TFmini_USART);
  HAL_UART_Receive_DMA(&TFmini_USART,TFmini_Rx_buff,Data_Length);
}
