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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define NO_USB       //Disable USB for DEBUG build

//#define YOKE
//#define PEDALS
//#define THROTTLE
//#define GA_THROTTLE
//#define RACING_PEDALS
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
#define IN8_Pin GPIO_PIN_8
#define IN8_GPIO_Port GPIOB
#define IN9_Pin GPIO_PIN_0
#define IN9_GPIO_Port GPIOF
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOF
#define IN0_Pin GPIO_PIN_0
#define IN0_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOA
#define IN10_Pin GPIO_PIN_2
#define IN10_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_3
#define IN2_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_4
#define IN3_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_5
#define IN4_GPIO_Port GPIOA
#define IN5_Pin GPIO_PIN_6
#define IN5_GPIO_Port GPIOA
#define IN6_Pin GPIO_PIN_7
#define IN6_GPIO_Port GPIOA
#define IN7_Pin GPIO_PIN_1
#define IN7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
