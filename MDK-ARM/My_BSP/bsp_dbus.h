#ifndef __BSP_DBUS_H
#define __BSP_DBUS_H
#include "stm32f4xx.h"
#include "string.h"
#include "usart.h"

//条件编译开关宏定义，不需要开启的就注释掉相关宏定义
//#define BSP_DBUS_USE_SIGNAL
//外设相关宏定义,移植时如果修改了外设请在这里修改
#define BSP_DBUS_UART						huart3
//Dbus接收缓存的数组大小，有需要请在这里修改,使用空闲中断要求这个数至少要大于18
#define BSP_DBUS_BUFFER_SIZE		25

//遥控器一些相关的宏定义,从官方手册看过来的，如果不需要就注释了吧
#define BSP_DBUS_RC_CH_MIN			((uint16_t)364)
#define BSP_DBUS_RC_CH_MID			((uint16_t)1024)
#define BSP_DBUS_RC_CH_MAX			((uint16_t)1684)
#define BSP_DBUS_RC_SW_UP				((uint8_t)1)
#define BSP_DBUS_RC_SW_MID			((uint8_t)3)
#define BSP_DBUS_RC_sW_DOWN			((uint8_t)2)
//参照了一下上届代码，此处按键定义可能和官方手册有差异，具体可以自己测试一下
#define BSP_DBUS_KEY_W					((uint16_t)0x01<<0) 
#define BSP_DBUS_KEY_S					((uint16_t)0x01<<1)
#define BSP_DBUS_KEY_A					((uint16_t)0x01<<2)
#define BSP_DBUS_KEY_D					((uint16_t)0x01<<3)
#define BSP_DBUS_KEY_Q					((uint16_t)0x01<<7)
#define BSP_DBUS_KEY_E					((uint16_t)0x01<<8)
#define BSP_DBUS_KEY_CTRL				((uint16_t)0x01<<6)
#define BSP_DBUS_KEY_SHIFT			((uint16_t)0x01<<5)
#define BSP_DBUS_KEY_R					((uint16_t)0x01<<9) 
#define BSP_DBUS_KEY_F					((uint16_t)0x01<<10)
#define BSP_DBUS_KEY_G					((uint16_t)0x01<<11)
#define BSP_DBUS_KEY_Z					((uint16_t)0x01<<12)
#define BSP_DBUS_KEY_X					((uint16_t)0x01<<13)
#define BSP_DBUS_KEY_C					((uint16_t)0x01<<14) 
#define BSP_DBUS_KEY_V					((uint16_t)0x01<<15)
#define BSP_DBUS_KEY_B					((uint16_t)0x01<<16)

/*遥控数据结构体*/
typedef struct rc_rec
{
		int16_t CH_0;		//通道0
		int16_t CH_1;		//通道1
		int16_t CH_2;		//通道2
		int16_t CH_3;		//通道3
		uint8_t S1;			//左拨码开关
		uint8_t S2;			//右拨码开关
		int16_t Dial;		//拨码盘
		struct 
		{
			int16_t X;				//X轴
			int16_t Y;				//Y轴
			int16_t Z;				//Z轴
			uint8_t Leftkey;	//右键
			uint8_t Rightkey;	//右键
		}Mouse;					//鼠标信息
		uint16_t Keys;	//按键信息
}bsp_dbus_RC_Data;

extern bsp_dbus_RC_Data bsp_dbus_Data;

//条件编译声明，如果使用信号量将多对外提供一个解析函数作为接口
#ifdef	BSP_DBUS_USE_SIGNAL
	void bsp_dbus_It(void);							//Dbus中断处理函数
	void bsp_dbus_Init(void);						//Dbus初始化函数
	void bsp_dbus_Analysis(void);				//Dbus数据解析函数，使用信号量时会对外开放接口
#else
	void bsp_dbus_It(void);							//Dbus中断处理函数
	void bsp_dbus_Init(void);						//Dbus初始化函数
#endif

#endif
