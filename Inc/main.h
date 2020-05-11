/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Omron3_Pin GPIO_PIN_13
#define Omron3_GPIO_Port GPIOC
#define Omron3_EXTI_IRQn EXTI15_10_IRQn
#define Omron2_Pin GPIO_PIN_14
#define Omron2_GPIO_Port GPIOC
#define Omron2_EXTI_IRQn EXTI15_10_IRQn
#define Omron1_Pin GPIO_PIN_15
#define Omron1_GPIO_Port GPIOC
#define Omron1_EXTI_IRQn EXTI15_10_IRQn
#define NULL_Pin GPIO_PIN_2
#define NULL_GPIO_Port GPIOA
#define TFmini_RX_Pin GPIO_PIN_3
#define TFmini_RX_GPIO_Port GPIOA
#define SPI1_nCS_Pin GPIO_PIN_4
#define SPI1_nCS_GPIO_Port GPIOA
#define Camera2_Pin GPIO_PIN_5
#define Camera2_GPIO_Port GPIOC
#define Camera1_Pin GPIO_PIN_0
#define Camera1_GPIO_Port GPIOB
#define Screen_Pin GPIO_PIN_1
#define Screen_GPIO_Port GPIOB
#define NULLB10_Pin GPIO_PIN_10
#define NULLB10_GPIO_Port GPIOB
#define DBUS_Pin GPIO_PIN_11
#define DBUS_GPIO_Port GPIOB
#define Judgment_TX_Pin GPIO_PIN_6
#define Judgment_TX_GPIO_Port GPIOC
#define Judgment_RX_Pin GPIO_PIN_7
#define Judgment_RX_GPIO_Port GPIOC
#define MiniPC_TX_Pin GPIO_PIN_9
#define MiniPC_TX_GPIO_Port GPIOA
#define MiniPC_RX_Pin GPIO_PIN_10
#define MiniPC_RX_GPIO_Port GPIOA
#define Air_Cylinder3_Pin GPIO_PIN_3
#define Air_Cylinder3_GPIO_Port GPIOB
#define Air_Cylinder2_Pin GPIO_PIN_4
#define Air_Cylinder2_GPIO_Port GPIOB
#define Air_Cylinder1_Pin GPIO_PIN_5
#define Air_Cylinder1_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_6
#define BEEP_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_7
#define BLUE_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_8
#define RED_GPIO_Port GPIOB
#define GREEN_Pin GPIO_PIN_9
#define GREEN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
