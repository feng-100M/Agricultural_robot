/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "OLED.h"
#include "pid.h"
#include "vl53l0x.h"
#include "MPU6050.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern VL53L0X_Dev_t vl53l0x_dev; 
extern VL53L0X_Dev_t vl53l0x_dev1; 
extern VL53L0X_Dev_t vl53l0x_dev2; 

 //´®¿Ú
extern uint8_t Rx_String[100];
extern uint8_t Rx_Flag;
extern uint8_t Rx_buff;

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
#define E18_1_Pin GPIO_PIN_14
#define E18_1_GPIO_Port GPIOD
#define E18_3_Pin GPIO_PIN_7
#define E18_3_GPIO_Port GPIOC
#define E18_4_Pin GPIO_PIN_9
#define E18_4_GPIO_Port GPIOC
#define E18_5_Pin GPIO_PIN_9
#define E18_5_GPIO_Port GPIOA
#define E18_8_Pin GPIO_PIN_10
#define E18_8_GPIO_Port GPIOA
#define E18_6_Pin GPIO_PIN_11
#define E18_6_GPIO_Port GPIOA
#define E18_7_Pin GPIO_PIN_15
#define E18_7_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_10
#define CLK_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_11
#define CS_GPIO_Port GPIOC
#define CMD_Pin GPIO_PIN_12
#define CMD_GPIO_Port GPIOC
#define DAT_Pin GPIO_PIN_0
#define DAT_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
