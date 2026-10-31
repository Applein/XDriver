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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define POWER_U_Pin GPIO_PIN_0
#define POWER_U_GPIO_Port GPIOA
#define DRV_TEMP_Pin GPIO_PIN_1
#define DRV_TEMP_GPIO_Port GPIOA
#define HW_ELEC_BM_Pin GPIO_PIN_2
#define HW_ELEC_BM_GPIO_Port GPIOA
#define HW_ELEC_BP_Pin GPIO_PIN_3
#define HW_ELEC_BP_GPIO_Port GPIOA
#define HW_ELEC_AM_Pin GPIO_PIN_4
#define HW_ELEC_AM_GPIO_Port GPIOA
#define HW_ELEC_AP_Pin GPIO_PIN_5
#define HW_ELEC_AP_GPIO_Port GPIOA
#define debug_1_Pin GPIO_PIN_6
#define debug_1_GPIO_Port GPIOA
#define SIGNAL_COUNT_EN_Pin GPIO_PIN_0
#define SIGNAL_COUNT_EN_GPIO_Port GPIOB
#define SIGNAL_ALERT_Pin GPIO_PIN_1
#define SIGNAL_ALERT_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_2
#define BUTTON2_GPIO_Port GPIOB
#define HW_ELEC_BPWM_Pin GPIO_PIN_10
#define HW_ELEC_BPWM_GPIO_Port GPIOB
#define HW_ELEC_APWM_Pin GPIO_PIN_11
#define HW_ELEC_APWM_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_12
#define BUTTON1_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
