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
#include "stm32f1xx_hal.h"

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
#define JOY1_X_Pin GPIO_PIN_0
#define JOY1_X_GPIO_Port GPIOC
#define JO1_Y_Pin GPIO_PIN_1
#define JO1_Y_GPIO_Port GPIOC
#define JOY2_X_Pin GPIO_PIN_2
#define JOY2_X_GPIO_Port GPIOC
#define JOY2_Y_Pin GPIO_PIN_3
#define JOY2_Y_GPIO_Port GPIOC
#define POT1_Pin GPIO_PIN_0
#define POT1_GPIO_Port GPIOA
#define CSN_Pin GPIO_PIN_1
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_4
#define CE_GPIO_Port GPIOA
#define POT2_Pin GPIO_PIN_5
#define POT2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_6
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_7
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_4
#define SW3_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_5
#define SW4_GPIO_Port GPIOC
#define BAT_LEVEL_Pin GPIO_PIN_0
#define BAT_LEVEL_GPIO_Port GPIOB
#define JOY1_PB_Pin GPIO_PIN_1
#define JOY1_PB_GPIO_Port GPIOB
#define JOY2_PB_Pin GPIO_PIN_2
#define JOY2_PB_GPIO_Port GPIOB
#define MPU_PWR_Pin GPIO_PIN_14
#define MPU_PWR_GPIO_Port GPIOB
#define RC_PWR_Pin GPIO_PIN_15
#define RC_PWR_GPIO_Port GPIOB
#define MPU_INT_Pin GPIO_PIN_6
#define MPU_INT_GPIO_Port GPIOC
#define LED_BAT_Pin GPIO_PIN_7
#define LED_BAT_GPIO_Port GPIOC
#define LED_EXT_Pin GPIO_PIN_8
#define LED_EXT_GPIO_Port GPIOC
#define LED_MODE_Pin GPIO_PIN_9
#define LED_MODE_GPIO_Port GPIOC
#define ENC_PB_Pin GPIO_PIN_10
#define ENC_PB_GPIO_Port GPIOA
#define ENC_PB_EXTI_IRQn EXTI15_10_IRQn
#define OLED_PWR_Pin GPIO_PIN_8
#define OLED_PWR_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_9
#define BUZZ_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define DEBUG_UART  huart3
#define BINDING_UART huart2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
