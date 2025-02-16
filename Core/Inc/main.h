/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define Sen_5V_Fault_1_Pin GPIO_PIN_13
#define Sen_5V_Fault_1_GPIO_Port GPIOC
#define APPS2_VCC_Fault_Pin GPIO_PIN_14
#define APPS2_VCC_Fault_GPIO_Port GPIOC
#define APPS1_VCC_Fault_Pin GPIO_PIN_15
#define APPS1_VCC_Fault_GPIO_Port GPIOC
#define VoltSen5V_1_Pin GPIO_PIN_0
#define VoltSen5V_1_GPIO_Port GPIOC
#define VoltSen5V_2_Pin GPIO_PIN_1
#define VoltSen5V_2_GPIO_Port GPIOC
#define VoltSen5V_3_Pin GPIO_PIN_2
#define VoltSen5V_3_GPIO_Port GPIOC
#define VoltSen5V_4_Pin GPIO_PIN_3
#define VoltSen5V_4_GPIO_Port GPIOC
#define SpeedSen1_Pin GPIO_PIN_0
#define SpeedSen1_GPIO_Port GPIOA
#define SpeedSen2_Pin GPIO_PIN_1
#define SpeedSen2_GPIO_Port GPIOA
#define SpeedSen3_Pin GPIO_PIN_2
#define SpeedSen3_GPIO_Port GPIOA
#define VoltSen5V_5_Pin GPIO_PIN_3
#define VoltSen5V_5_GPIO_Port GPIOA
#define VoltSen5V_6_Pin GPIO_PIN_4
#define VoltSen5V_6_GPIO_Port GPIOA
#define APPS1_Pin GPIO_PIN_5
#define APPS1_GPIO_Port GPIOA
#define APPS2_Pin GPIO_PIN_6
#define APPS2_GPIO_Port GPIOA
#define RideHeight_Pin GPIO_PIN_7
#define RideHeight_GPIO_Port GPIOA
#define SalenKey_Pin GPIO_PIN_4
#define SalenKey_GPIO_Port GPIOC
#define AVoltSen5V_1_Pin GPIO_PIN_5
#define AVoltSen5V_1_GPIO_Port GPIOC
#define AVoltSen5V_2_Pin GPIO_PIN_0
#define AVoltSen5V_2_GPIO_Port GPIOB
#define AVoltSen5V_3_Pin GPIO_PIN_1
#define AVoltSen5V_3_GPIO_Port GPIOB
#define Pull_up_1_Pin GPIO_PIN_2
#define Pull_up_1_GPIO_Port GPIOB
#define Pull_up_2_Pin GPIO_PIN_10
#define Pull_up_2_GPIO_Port GPIOB
#define CANTX2_Pin GPIO_PIN_13
#define CANTX2_GPIO_Port GPIOB
#define Pull_up_3_Pin GPIO_PIN_14
#define Pull_up_3_GPIO_Port GPIOB
#define SDC2_MCU_Pin GPIO_PIN_8
#define SDC2_MCU_GPIO_Port GPIOA
#define SDC1_MCU_Pin GPIO_PIN_9
#define SDC1_MCU_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define HARDFAULT_LED_Pin GPIO_PIN_15
#define HARDFAULT_LED_GPIO_Port GPIOA
#define VecNav_TX_Pin GPIO_PIN_10
#define VecNav_TX_GPIO_Port GPIOC
#define VecNav_RX_Pin GPIO_PIN_11
#define VecNav_RX_GPIO_Port GPIOC
#define Gsense_Pin GPIO_PIN_12
#define Gsense_GPIO_Port GPIOC
#define HBeat_Pin GPIO_PIN_2
#define HBeat_GPIO_Port GPIOD
#define Fault_12V_LED_Pin GPIO_PIN_4
#define Fault_12V_LED_GPIO_Port GPIOB
#define USART_TX_Pin GPIO_PIN_6
#define USART_TX_GPIO_Port GPIOB
#define USART_RX_Pin GPIO_PIN_7
#define USART_RX_GPIO_Port GPIOB
#define Sen_12V_Fault_Pin GPIO_PIN_8
#define Sen_12V_Fault_GPIO_Port GPIOB
#define Sen_5V_Fault_2_Pin GPIO_PIN_9
#define Sen_5V_Fault_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
