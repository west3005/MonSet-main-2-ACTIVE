/**
 * @file    uart_extra.c
 * @brief   Определения UART_HandleTypeDef для датчиковых портов 1 и 2.
 *
 *  Порт 0 — USART3 (huart3): определяется CubeMX в main.c, здесь не нужен.
 *  Порт 1 — UART4  (huart4): определяется здесь (CubeMX не включает UART4 в .ioc).
 *  Порт 2 — UART5  (huart5): определяется здесь (CubeMX не включает UART5 в .ioc).
 *
 *  Файл НЕ перегенерируется CubeMX — редактировать вручную при необходимости.
 *  Инициализация выполняется в app.cpp: MX_UART4_Init() / MX_UART5_Init().
 */
#include "main.h"

/* ── Датчиковый порт 0: USART3 (PB10 TX / PB11 RX, DE=PB12) ────────────────
 * Определяется CubeMX в main.c — здесь не дублируется.                       */

/* ── Датчиковый порт 1: UART4 (PC10 TX / PC11 RX, auto-direction 2126) ───── */
UART_HandleTypeDef huart4;

/* ── Датчиковый порт 2: UART5 (PC12 TX / PD2 RX,  auto-direction 2126) ───── */
UART_HandleTypeDef huart5;
