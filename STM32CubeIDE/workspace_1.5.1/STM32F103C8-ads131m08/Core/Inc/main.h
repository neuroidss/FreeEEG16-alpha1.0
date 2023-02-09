/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define ADC1_SYNC_RESET_Pin GPIO_PIN_0
#define ADC1_SYNC_RESET_GPIO_Port GPIOB
#define ADC1_DRDY_Pin GPIO_PIN_1
#define ADC1_DRDY_GPIO_Port GPIOB
#define ADC1_DRDY_EXTI_IRQn EXTI1_IRQn
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
//#define SPI1_NSS2_Pin GPIO_PIN_1
//#define SPI1_NSS2_GPIO_Port GPIOB

#define FREEEEG16_TEST_COUNTER              (1LL<<4)
#define FREEEEG16_ADS131M08_SPI_TEST_INT (1LL<<5)
#define FREEEEG16_ADS131M08_SPI_TEXT_UART1_INT (1LL<<6)
#define FREEEEG16_ADS131M08_SPI_OPENVIBE_FREEEEG16_CUSTOM_INT (1LL<<7)
#define FREEEEG16_ADS131M08_SPI_TEST_REGISTERS_INT (1LL<<8)
#define FREEEEG16_ADS131M08_SPI_SAMPLECOUNT_INT (1LL<<9)

//#define FREEEEG16_OUT (0)

//#define FREEEEG16_OUT FREEEEG16_TEST_COUNTER
//#define FREEEEG16_OUT FREEEEG16_ADS131M08_SPI_TEST_INT
//#define FREEEEG16_OUT FREEEEG16_ADS131M08_SPI_TEST_REGISTERS_INT
//#define FREEEEG16_OUT FREEEEG16_ADS131M08_SPI_TEXT_UART1_INT
#define FREEEEG16_OUT FREEEEG16_ADS131M08_SPI_OPENVIBE_FREEEEG16_CUSTOM_INT
//#define FREEEEG16_OUT FREEEEG16_ADS131M08_SPI_SAMPLECOUNT_INT

#define FREESMARTEEG_ADC_NONE (0)
#define FREESMARTEEG_ADC_ADS131M08_SPI_READ_INT (1LL<<15)

#define FREESMARTEEG_ADC FREESMARTEEG_ADC_ADS131M08_SPI_READ_INT

#define FREESMARTEEG_SEND_UART1  (1<<0)
#define FREESMARTEEG_SEND_UART2  (1<<1)
//#define FREESMARTEEG_SEND FREESMARTEEG_SEND_UART1
#define FREESMARTEEG_SEND FREESMARTEEG_SEND_UART2

#define UART_DMA (1)
//#define UART_DMA (0)

#define SPI_DMA (1)
//#define SPI_DMA (0)

#define SPI_NSS_SOFTWARE (1)
//#define SPI_NSS_SOFTWARE (0)

//#define uint8_ad_adc_data_bytes_number (2)
#define uint8_ad_adc_data_bytes_number (3)
//#define uint8_ad_number ((1 + 3) * 1)
//#define uint8_ad_adc_number (1)
#define uint8_ad_adc_number (2)
//#define uint8_ad_adc_number (3)
//#define uint8_ad_adc_number (4)
//#define uint8_ad_adc_number (8)
//#define uint8_ad_adc_number (16)
//#define uint8_ad_chan_number (1)
//#define uint8_ad_chan_number (2)
//#define uint8_ad_chan_number (3)
//#define uint8_ad_chan_number (4)
//#define uint8_ad_chan_number (5)
//#define uint8_ad_chan_number (6)
//#define uint8_ad_chan_number (7)
#define uint8_ad_chan_number (8)
//#define uint8_accel_chan_number (1)
//#define uint8_ad_number ((1 + 3) * 8)
#define uint8_ad_number ((1 + 3) * uint8_ad_chan_number)
//#define uint8_accel_chan_number (1)
//#define uint8_accel_chan_number (0)
#define uint8_accel_chan_number (3)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
