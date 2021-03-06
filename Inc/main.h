/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define MPU6500_INT_Pin GPIO_PIN_8
#define MPU6500_INT_GPIO_Port GPIOB
#define MPU6500_INT_EXTI_IRQn EXTI9_5_IRQn
#define LCD_SCLK_Pin GPIO_PIN_3
#define LCD_SCLK_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_9
#define LCD_DC_GPIO_Port GPIOB
#define PWR_24V_1_Pin GPIO_PIN_2
#define PWR_24V_1_GPIO_Port GPIOH
#define PWR_24V_2_Pin GPIO_PIN_3
#define PWR_24V_2_GPIO_Port GPIOH
#define PWR_24V_3_Pin GPIO_PIN_4
#define PWR_24V_3_GPIO_Port GPIOH
#define PWR_24V_4_Pin GPIO_PIN_5
#define PWR_24V_4_GPIO_Port GPIOH
#define SHOOT_PWR_EN_Pin GPIO_PIN_4
#define SHOOT_PWR_EN_GPIO_Port GPIOC
#define LCD_BLK_Pin GPIO_PIN_6
#define LCD_BLK_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LCD_SDIN_Pin GPIO_PIN_7
#define LCD_SDIN_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
