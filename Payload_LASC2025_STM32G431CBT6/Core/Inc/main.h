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
#include "stm32g4xx_hal.h"

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
#define LORA_ADDRESS_STM32G4   0x000A
#define LORA_ADDRESS_ESP32     0x000C
#define LORA_CHANNEL           23
#define RX_BUFFER_SIZE         256
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define LORA_TX_Pin GPIO_PIN_2
#define LORA_TX_GPIO_Port GPIOA
#define LORA_RX_Pin GPIO_PIN_3
#define LORA_RX_GPIO_Port GPIOA
#define SPI2_CS_BAR_Pin GPIO_PIN_4
#define SPI2_CS_BAR_GPIO_Port GPIOA
#define LORA_M1_Pin GPIO_PIN_6
#define LORA_M1_GPIO_Port GPIOA
#define SPI2_CS_IMU_Pin GPIO_PIN_0
#define SPI2_CS_IMU_GPIO_Port GPIOB
#define LORA_M0_Pin GPIO_PIN_1
#define LORA_M0_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_10
#define IMU_INT1_GPIO_Port GPIOA
#define IMU_FSYNC_Pin GPIO_PIN_11
#define IMU_FSYNC_GPIO_Port GPIOA
#define SPI3_CS_SD_Pin GPIO_PIN_15
#define SPI3_CS_SD_GPIO_Port GPIOA
#define TTL_USART_TX_Pin GPIO_PIN_6
#define TTL_USART_TX_GPIO_Port GPIOB
#define TTL_USART_RX_Pin GPIO_PIN_7
#define TTL_USART_RX_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define LED_BUILTIN_Pin GPIO_PIN_9
#define LED_BUILTIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
