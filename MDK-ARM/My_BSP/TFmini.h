#ifndef  __TFMINI_H
#define  __TFMINI_H
#include "stm32f4xx_hal.h"

#define TFmini_USART huart2
#define Data_Length  9
#define TFmini_HeadByte 0x59

extern uint16_t TFmini_Distance;
extern uint8_t  TFmini_Online;

void TFmini_Init(void);
void TFmini_IT(void) ;
#endif
