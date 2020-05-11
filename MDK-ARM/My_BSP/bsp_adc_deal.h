#ifndef  __BSP_ADC_DEAL_H
#define  __BSP_ADC_DEAL_H
#include "adc.h"
#define bsp_ADC_USE_ADC1 hadc1
//#define bsp_ADC_USE_ADC2 hadc2

extern uint16_t bsp_Final_ADC1_Volage[4];  // 最后处理出来的电压结果
extern uint16_t bsp_ADC1_Value[400];  //存储器中采集到的数据 
extern float    bsp_ADC1_Sharp_Distance[4];  // 夏普红外测距，经过拟合的

extern uint16_t bsp_Final_ADC2_Volage[4];// 最后处理出来的电压结果
extern uint16_t bsp_ADC2_Value[400];  //存储器中采集到的数据
extern float    bsp_ADC2_Sharp_Distance[4];  // 夏普红外测距，经过拟合的
void bsp_ADC_Sensor_Init();  //初始化ADC



#endif  /*__ADC_DEAL_H*/
