/**
 * @file    uart_extra.c
 * @brief   Определения UART_HandleTypeDef для UART4 и UART5.
 *          CubeMX не генерирует эти хэндлы так как UART4/UART5 настраиваются
 *          вручную в app.cpp (MX_UART4_Init / MX_UART5_Init).
 *          Файл не перегенерируется CubeMX — редактировать вручную при необходимости.
 */
#include "main.h"

UART_HandleTypeDef huart4;  ///< Датчиковый порт 1 — UART4 (PC10 TX / PC11 RX)
UART_HandleTypeDef huart5;  ///< Датчиковый порт 2 — UART5 (PC12 TX / PD2 RX)
