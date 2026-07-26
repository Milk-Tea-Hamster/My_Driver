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
#define AD9959_SP0_Pin GPIO_PIN_11
#define AD9959_SP0_GPIO_Port GPIOD
#define AD9959_PDC_Pin GPIO_PIN_12
#define AD9959_PDC_GPIO_Port GPIOD
#define AD9959_SP1_Pin GPIO_PIN_13
#define AD9959_SP1_GPIO_Port GPIOD
#define AD9959_RESET_Pin GPIO_PIN_14
#define AD9959_RESET_GPIO_Port GPIOD
#define AD9959_SP2_Pin GPIO_PIN_15
#define AD9959_SP2_GPIO_Port GPIOD
#define AD9959_UPDATE_Pin GPIO_PIN_6
#define AD9959_UPDATE_GPIO_Port GPIOC
#define AD9959_SP3_Pin GPIO_PIN_7
#define AD9959_SP3_GPIO_Port GPIOC
#define AD9959_CS_Pin GPIO_PIN_8
#define AD9959_CS_GPIO_Port GPIOC
#define AD9959_SDIO0_Pin GPIO_PIN_9
#define AD9959_SDIO0_GPIO_Port GPIOC
#define AD9959_SCLK_Pin GPIO_PIN_8
#define AD9959_SCLK_GPIO_Port GPIOA
#define AD9959_SDIO1_Pin GPIO_PIN_11
#define AD9959_SDIO1_GPIO_Port GPIOA
#define AD9959_SDIO3_Pin GPIO_PIN_12
#define AD9959_SDIO3_GPIO_Port GPIOA
#define AD9959_SDIO2_Pin GPIO_PIN_15
#define AD9959_SDIO2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
