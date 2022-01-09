/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS_IP.h"
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
void print(char *buf, uint16_t len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_Pin GPIO_PIN_0
#define BTN_GPIO_Port GPIOA
#define ENC_INT_Pin GPIO_PIN_1
#define ENC_INT_GPIO_Port GPIOA
#define ENC_INT_EXTI_IRQn EXTI1_IRQn
#define ENC_RESET_Pin GPIO_PIN_2
#define ENC_RESET_GPIO_Port GPIOA
#define ENC_NSS_Pin GPIO_PIN_4
#define ENC_NSS_GPIO_Port GPIOA
#define ENC_SCK_Pin GPIO_PIN_5
#define ENC_SCK_GPIO_Port GPIOA
#define ENC_MISO_Pin GPIO_PIN_6
#define ENC_MISO_GPIO_Port GPIOA
#define ENC_MOSI_Pin GPIO_PIN_7
#define ENC_MOSI_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOE
#define LED5_Pin GPIO_PIN_13
#define LED5_GPIO_Port GPIOE
#define LED6_Pin GPIO_PIN_14
#define LED6_GPIO_Port GPIOE
#define LED7_Pin GPIO_PIN_15
#define LED7_GPIO_Port GPIOE
#define MIC_IN_Pin GPIO_PIN_12
#define MIC_IN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
