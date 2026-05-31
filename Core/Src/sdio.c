/**
 * @file sdio.c
 */
#include "sdio.h"
#include "stm32f4xx_hal_sd.h"
#include "main.h"
#include "debug_uart_c.h"

SD_HandleTypeDef hsd;

bool MX_SDIO_SD_Init(void)
{
    /* Небольшая пауза для стабилизации питания карты (≥1 мс по спеку) */
    HAL_Delay(10);

    hsd.Instance             = SDIO;
    hsd.Init.ClockEdge       = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass     = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave  = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide         = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE; /* STM32F4 errata: HWFC causes TXUNDERR in polling mode */
    hsd.Init.ClockDiv        = 118U; /* 48MHz/(118+2) = 400 кГц */

    uart_log_info("[SDIO] CLKCR=0x%08lX RCC_APB2ENR=0x%08lX",
                  (unsigned long)SDIO->CLKCR,
                  (unsigned long)RCC->APB2ENR);

    /* Три попытки с нарастающим delay */
    for (int attempt = 1; attempt <= 3; attempt++) {
        HAL_SD_DeInit(&hsd);
        HAL_StatusTypeDef hs = HAL_SD_Init(&hsd);
        uart_log_info("[SDIO] attempt %d: HAL=%d State=%d STA=0x%08lX ErrorCode=0x%08lX",
                      attempt, (int)hs, (int)hsd.State,
                      (unsigned long)SDIO->STA,
                      (unsigned long)hsd.ErrorCode);
        if (hs == HAL_OK) {
            goto init_ok;
        }
        SDIO->CLKCR &= ~SDIO_CLKCR_HWFC_EN;
        HAL_Delay(attempt * 20);
    }

    uart_log_error("[SDIO] all attempts failed");
    return false;

init_ok:
    /*
     * HAL_SD_ConfigWideBusOperation с SDIO_BUS_WIDE_1B на STM32F407
     * посылает ACMD6 и ждёт карту в состоянии TRANSFER.
     * После HAL_SD_Init() карта уже в 1-bit режиме — повторная команда
     * лишняя и может давать timeout если карта не готова немедленно.
     * Пропускаем — шина уже 1-bit после инициализации.
     */
    uart_log_info("[SDIO] init OK (1-bit, no ConfigWideBus needed)");
    return true;
}
