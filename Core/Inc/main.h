/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define LED_System_Pin GPIO_PIN_2
#define LED_System_GPIO_Port GPIOC
#define TJA1028_EN_Pin GPIO_PIN_4
#define TJA1028_EN_GPIO_Port GPIOA
#define TJA1028_RSTN_Pin GPIO_PIN_0
#define TJA1028_RSTN_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_13
#define LCD_RST_GPIO_Port GPIOB
#define LCD_PSB_Pin GPIO_PIN_14
#define LCD_PSB_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_8
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_9
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_GPIO_Port GPIOC
#define LCD_D3_Pin GPIO_PIN_7
#define LCD_D3_GPIO_Port GPIOC
#define LCD_D2_Pin GPIO_PIN_8
#define LCD_D2_GPIO_Port GPIOD
#define LCD_D1_Pin GPIO_PIN_10
#define LCD_D1_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_11
#define LCD_D0_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_12
#define LCD_E_GPIO_Port GPIOA
#define LCD_WR_Pin GPIO_PIN_15
#define LCD_WR_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_GPIO_Port GPIOC
#define Finished_Key_Pin GPIO_PIN_5
#define Finished_Key_GPIO_Port GPIOD
#define Init_Key_Pin GPIO_PIN_3
#define Init_Key_GPIO_Port GPIOB
#define Step_Add_Pin GPIO_PIN_4
#define Step_Add_GPIO_Port GPIOB
#define Step_Sub_Pin GPIO_PIN_6
#define Step_Sub_GPIO_Port GPIOB
#define Loop_Add_Pin GPIO_PIN_7
#define Loop_Add_GPIO_Port GPIOB
#define Loop_Sub_Pin GPIO_PIN_8
#define Loop_Sub_GPIO_Port GPIOB
#define Start_Key_Pin GPIO_PIN_9
#define Start_Key_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
