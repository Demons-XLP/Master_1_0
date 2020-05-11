/** 
* @brief    夏普红外测距主要对4个通道的电流数据进行采集处理
* @details  ADC中断回调函数进行数据接收、滤波（要拟合再根据数据处理）
* @author   TAOU Monkey
* @date      2019.11
* @version  1.0
* @par Copyright (c):  RM2020电控
* @par 
* @par     这个文件的输入为ADC1_DMA中断启动函数处得到的bsp_ADC_Value[400]
* @par     最后处理的结果为bsp_Final_ADC_Volage[4]
* @par     最后夏普红外拟合数据为bsp_ADC1_Sharp_Distance、bsp_ADC2_Sharp_Distance
*				使用方法：1.在头文件中定义自己想要的ADC1/2
                  2.将初始化函数bsp_ADC_Sensor_Init()初始化即可
*				版本变更:1.0相较于19届原始版本可以更加方便地控制自己想用的ADC通道
*       2019.11.27  9：35         1.1按照代码规范重新编写

*/
 
#include "bsp_adc_deal.h"
#include "math.h"

#ifdef bsp_ADC_USE_ADC1
uint16_t bsp_ADC1_Value[400];   //存储器中采集到的数据 
//分离出单个通道采集到的数据
uint16_t ADC1_ch1_value[100];
uint16_t ADC1_ch2_value[100];
uint16_t ADC1_ch3_value[100];
uint16_t ADC1_ch4_value[100];
float ADC1_fixed_ch[4];     //存放均值滤波后的4个电压数据  [4][1]
int   ADC1_adjust_offset=0; //初始电压读取次数计算用
uint16_t ADC1_ch_offset[4] = {0};  //放初始电压
float 	   ADC1_que[4][10];  //存放10次的均值滤波结果
float   ADC1_ch_sum[4]={0};  //存放每一个通道10次均值滤波结果的平均值
uint16_t bsp_Final_ADC1_Volage[4];  // 最后处理出来的电压结果
float    bsp_ADC1_Sharp_Distance[4];    // 夏普红外测距，经过拟合的

#endif

#ifdef bsp_ADC_USE_ADC2
uint16_t bsp_ADC2_Value[400];   //存储器中采集到的数据 

uint16_t ADC2_ch1_value[100];
uint16_t ADC2_ch2_value[100];
uint16_t ADC2_ch3_value[100];
uint16_t ADC2_ch4_value[100];
float ADC2_fixed_ch[4];     //存放均值滤波后的4个电压数据  [4][1]
int   ADC2_adjust_offset=0; //初始电压读取次数计算用
uint16_t ADC2_ch_offset[4] = {0};  //放初始电压
float 	   ADC2_que[4][10];  //存放10次的均值滤波结果
float   ADC2_ch_sum[4]={0};  //存放每一个通道10次均值滤波结果的平均值
uint16_t bsp_Final_ADC2_Volage[4];  // 最后处理出来的电压结果
float    bsp_ADC2_Sharp_Distance[4];    // 夏普红外测距，经过拟合的
#endif

/*******************************************************************************************
  * @Func			get_4ch(unsigned short int *signal,unsigned short int *channel,int length,unsigned char mask)
  * @Brief    根据mask的值获取四个通道的采样值(在ADC的回调函数中处理数据使用)
  * @Param		signal为采集到的数据，channel为分离出的单个通道的数据，length为单个通道的数据长度
  * @Retval		None
* @Date     2015/11/24    2018年10月30日16:49:07  Nankel修改
 *******************************************************************************************/
void get_4ch(unsigned short int *signal,unsigned short int *channel,int length,unsigned char mask)
{
	for(int j=0;j<length/4;j++)
		channel[j] = signal[j*4 + mask];
}

/**********去掉最大值和最小值后求平均值************/
float mid_sum_filter(uint16_t *data,uint16_t num)
{
	uint8_t i=0;
	uint16_t max=0,min=0;
	uint32_t sum=0;
	float average=0;
	
	max = *(data+0);
	min = *(data+0);
	sum += *(data+0);
	
	for (i=1; i<num; i++){
		if (max < *(data+i))
			max = *(data+i);
 		if (min > *(data+i))
			min = *(data+i);
		sum += *(data+i);
	}
	sum = sum - max - min;
	average = (float)sum/(num-2);
	return 	average;
}

/** 
  * @brief      ADC中断回调函数，进行数据接收和滤波
  * @param[in]  ADC_HandleTypeDef* hadc
  * @retval      
  * @retval       
  * @pa      
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	
#ifdef bsp_ADC_USE_ADC1
	if(hadc == &hadc1){
		get_4ch(bsp_ADC1_Value,ADC1_ch1_value,400,0);
		get_4ch(bsp_ADC1_Value,ADC1_ch2_value,400,1);
		get_4ch(bsp_ADC1_Value,ADC1_ch3_value,400,2);
		get_4ch(bsp_ADC1_Value,ADC1_ch4_value,400,3);
		ADC1_fixed_ch[0] = mid_sum_filter(ADC1_ch1_value, 100);         
		ADC1_fixed_ch[1] = mid_sum_filter(ADC1_ch2_value, 100);          
		ADC1_fixed_ch[2] = mid_sum_filter(ADC1_ch3_value, 100);
		ADC1_fixed_ch[3] = mid_sum_filter(ADC1_ch4_value, 100);
		

		for(int i=0; i<4; i++){
			for(int k=9; k>0; k--)   //将[4][10]的数组向后移位一格 
				ADC1_que[i][k] = ADC1_que[i][k-1];
			ADC1_que[i][0] = ADC1_fixed_ch[i];	//将ADC采样到的平均值插入第一列				
			for(int m=0; m<10; m++)   //将每行的元素加起来	
				ADC1_ch_sum[i] += ADC1_que[i][m];					
			ADC1_ch_sum[i] /= 10.0f;	
			bsp_Final_ADC1_Volage[i] =  (uint16_t) (ADC1_ch_sum[i] - ADC1_ch_offset[i]);    
		}
		for(uint8_t i=0;i<2;i++){
			if(bsp_Final_ADC1_Volage[i] < 750)  bsp_Final_ADC1_Volage[i] = 750;  // 对应60cm左右
			bsp_ADC1_Sharp_Distance[i] = 284486*pow(bsp_Final_ADC1_Volage[i],-1.275);
		}

  }
#endif
	
#ifdef bsp_ADC_USE_ADC2
	if(hadc == &hadc2){
		get_4ch(bsp_ADC2_Value,ADC2_ch1_value,400,0);
		get_4ch(bsp_ADC2_Value,ADC2_ch2_value,400,1);
		get_4ch(bsp_ADC2_Value,ADC2_ch3_value,400,2);
		get_4ch(bsp_ADC2_Value,ADC2_ch4_value,400,3);
		ADC2_fixed_ch[0] = mid_sum_filter(ADC2_ch1_value, 100);         
		ADC2_fixed_ch[1] = mid_sum_filter(ADC2_ch2_value, 100);          
		ADC2_fixed_ch[2] = mid_sum_filter(ADC2_ch3_value, 100);
		ADC2_fixed_ch[3] = mid_sum_filter(ADC2_ch4_value, 100);
		

		for(int i=0; i<4; i++){
			for(int k=9; k>0; k--)   //将[4][10]的数组向后移位一格 
				ADC2_que[i][k] = ADC2_que[i][k-1];
			ADC2_que[i][0] = ADC2_fixed_ch[i];	//将ADC采样到的平均值插入第一列				
			for(int m=0; m<10; m++)   //将每行的元素加起来	
				ADC2_ch_sum[i] += ADC2_que[i][m];					
			ADC2_ch_sum[i] /= 10.0f;	
			bsp_Final_ADC2_Volage[i] =  (uint16_t) (ADC2_ch_sum[i] - ADC2_ch_offset[i]);    
		}

	}
#endif 
}




/** 
  * @brief      传感器初始化函数
  * @param[in]  在某个地方初始化即可
  * @retval      
  * @retval       
  * @pa      
  */

void bsp_ADC_Sensor_Init()
{
	#ifdef bsp_ADC_USE_ADC1
 	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)bsp_ADC1_Value,400);//红外激光测距数据采集
	#endif
	#ifdef bsp_ADC_USE_ADC2
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)bsp_ADC2_Value,400);//红外激光测距数据采集
	#endif
}
