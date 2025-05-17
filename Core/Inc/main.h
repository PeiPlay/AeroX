/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define PE3_NRF_CE_Pin GPIO_PIN_3
#define PE3_NRF_CE_GPIO_Port GPIOE
#define PE3_NRF_CS_Pin GPIO_PIN_4
#define PE3_NRF_CS_GPIO_Port GPIOE
#define PC13_NRF_IRQ_Pin GPIO_PIN_13
#define PC13_NRF_IRQ_GPIO_Port GPIOC
#define PC0_SPI2_CS0_Pin GPIO_PIN_0
#define PC0_SPI2_CS0_GPIO_Port GPIOC
#define PC3_SPI2_CS1_Pin GPIO_PIN_3
#define PC3_SPI2_CS1_GPIO_Port GPIOC
#define PA0_SPI6_NSS_Pin GPIO_PIN_0
#define PA0_SPI6_NSS_GPIO_Port GPIOA
#define PA3_PMW_RST_Pin GPIO_PIN_3
#define PA3_PMW_RST_GPIO_Port GPIOA
#define PA4_SPI1_CS_Pin GPIO_PIN_4
#define PA4_SPI1_CS_GPIO_Port GPIOA
#define PB15_HC_SET_Pin GPIO_PIN_15
#define PB15_HC_SET_GPIO_Port GPIOE
#define PB13_SPI2_SCK_Pin GPIO_PIN_13
#define PB13_SPI2_SCK_GPIO_Port GPIOB
#define PA15_SPI3_NSS_Pin GPIO_PIN_15
#define PA15_SPI3_NSS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
