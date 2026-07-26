/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "arm_math.h"
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
#define AD9910_CLK_Pin GPIO_PIN_2
#define AD9910_CLK_GPIO_Port GPIOE
#define AD9910_CSB_Pin GPIO_PIN_3
#define AD9910_CSB_GPIO_Port GPIOE
#define AD9910_DPH_Pin GPIO_PIN_4
#define AD9910_DPH_GPIO_Port GPIOE
#define AD9910_SDIO_Pin GPIO_PIN_5
#define AD9910_SDIO_GPIO_Port GPIOE
#define AD9910_OSK_Pin GPIO_PIN_1
#define AD9910_OSK_GPIO_Port GPIOD
#define AD9910_IOUP_Pin GPIO_PIN_5
#define AD9910_IOUP_GPIO_Port GPIOD
#define AD9910_DRC_Pin GPIO_PIN_7
#define AD9910_DRC_GPIO_Port GPIOD
#define AD9910_PF0_Pin GPIO_PIN_3
#define AD9910_PF0_GPIO_Port GPIOB
#define AD9910_PF1_Pin GPIO_PIN_4
#define AD9910_PF1_GPIO_Port GPIOB
#define AD9910_PF2_Pin GPIO_PIN_5
#define AD9910_PF2_GPIO_Port GPIOB
#define AD9910_PWR_Pin GPIO_PIN_0
#define AD9910_PWR_GPIO_Port GPIOE
#define AD9910_RST_Pin GPIO_PIN_1
#define AD9910_RST_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
