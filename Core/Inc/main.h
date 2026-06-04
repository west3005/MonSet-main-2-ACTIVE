/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c/cpp file.
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
#include "stm32f4xx_hal.h"

/* ======== HAL-Р РЋРІР‚В¦Р РЋР РЉР В Р вЂ¦Р В РўвЂР В Р’В»Р РЋРІР‚в„– (extern) ======== */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;  ///< Датчиковый порт 0 — USART3 PB10(TX)/PB11(RX), DE=PB12
extern UART_HandleTypeDef huart4;  ///< Датчиковый порт 1 — UART4  PA0(TX)/PA1(RX)
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef  hi2c1;
extern SPI_HandleTypeDef  hspi1;
extern RTC_HandleTypeDef  hrtc;
extern TIM_HandleTypeDef  htim6;
/* Р вЂќР С•Р В±Р В°Р Р†Р С‘РЎвЂљРЎРЉ Р С” РЎРѓРЎС“РЎвЂ°Р ВµРЎРѓРЎвЂљР Р†РЎС“РЎР‹РЎвЂ°Р С‘Р С extern-Р С•Р В±РЎР‰РЎРЏР Р†Р В»Р ВµР Р…Р С‘РЎРЏР С: */

/* ======== Pin Defines Р Р†Р вЂљРІР‚Сњ Р В Р’В Р В РІР‚СћР В РІР‚вЂњР В Р’ВР В РЎС™Р В Р’В« ======== */
#define MODE_SWITCH_Pin      GPIO_PIN_0
#define MODE_SWITCH_GPIO_Port GPIOC

#define STATUS_LED_Pin       GPIO_PIN_1
#define STATUS_LED_GPIO_Port GPIOC

#define SIM800_PWR_Pin       GPIO_PIN_2
#define SIM800_PWR_GPIO_Port GPIOC

#define SD_CS_Pin            GPIO_PIN_4
#define SD_CS_GPIO_Port      GPIOA

#define RS485_DE_Pin         GPIO_PIN_12
#define RS485_DE_GPIO_Port   GPIOB

/* ======== UART1: Debug PA9(TX)-PA10(RX) ======== */
#define USART1_TX_Pin        GPIO_PIN_9
#define USART1_TX_GPIO_Port  GPIOA
#define USART1_RX_Pin        GPIO_PIN_10
#define USART1_RX_GPIO_Port  GPIOA

/* ======== UART2: SIM800L PA2(TX)-PA3(RX) ======== */
#define USART2_TX_Pin        GPIO_PIN_2
#define USART2_TX_GPIO_Port  GPIOA
#define USART2_RX_Pin        GPIO_PIN_3
#define USART2_RX_GPIO_Port  GPIOA

/* ======== UART3: RS485 PB10(TX)-PB11(RX) ======== */
#define USART3_TX_Pin        GPIO_PIN_10
#define USART3_TX_GPIO_Port  GPIOB
#define USART3_RX_Pin        GPIO_PIN_11
#define USART3_RX_GPIO_Port  GPIOB

/* ======== UART6: PC6(TX)-PC7(RX) ======== */
#define USART6_TX_Pin        GPIO_PIN_6
#define USART6_TX_GPIO_Port  GPIOC
#define USART6_RX_Pin        GPIO_PIN_7
#define USART6_RX_GPIO_Port  GPIOC

/* ======== I2C1: DS3231 PB6(SCL)-PB7(SDA) ======== */
#define I2C1_SCL_Pin         GPIO_PIN_6
#define I2C1_SCL_GPIO_Port   GPIOB
#define I2C1_SDA_Pin         GPIO_PIN_7
#define I2C1_SDA_GPIO_Port   GPIOB

/* ======== SPI1: SD Card PA5(SCK)-PA6(MISO)-PA7(MOSI) ======== */
#define SPI1_SCK_Pin         GPIO_PIN_5
#define SPI1_SCK_GPIO_Port   GPIOA
#define SPI1_MISO_Pin        GPIO_PIN_6
#define SPI1_MISO_GPIO_Port  GPIOA
#define SPI1_MOSI_Pin        GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port  GPIOA

/* Р В РІР‚СњР В РЎвЂўР В Р’В±Р В Р’В°Р В Р вЂ Р В РЎвЂР РЋРІР‚С™Р РЋР Р‰ Р В РЎвЂќ Р В РЎвЂ”Р В РЎвЂР В Р вЂ¦Р В Р’В°Р В РЎВ */
#define LED_BLUE_Pin        GPIO_PIN_13
#define LED_BLUE_GPIO_Port  GPIOC

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
