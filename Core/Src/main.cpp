/**
 * ================================================================
 * @file main.cpp
 * @brief Точка входа. Инициализация HAL, тактирования,
 *        всей периферии и запуск App.
 * ================================================================
 */
#include "main.h"
#include "config.hpp"
#include "debug_uart.hpp"
#include "app.hpp"
#include "spi.h"
#include "a7670c.hpp"

extern "C" {
#include "fatfs.h"
#include "sdio.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
}

/* ======== Глобальные флаги ======== */
bool g_sd_disabled = false;

// Причина последнего ресета — сохраняем до DBG.init()
static const char* g_reset_reason_str = "power-on";
static bool        g_reset_was_wdg    = false;

/* ======== HAL-хэндлы (глобальные) ======== */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
I2C_HandleTypeDef  hi2c1;
RTC_HandleTypeDef  hrtc;
TIM_HandleTypeDef  htim6;
RNG_HandleTypeDef  hrng;

/* ======== Внутренние прототипы ======== */
static void CheckResetReason(void);
static void MX_IWDG_Init(void);

/*
 * ВАЖНО:
 * Эти функции используются также из power_manager.cpp после выхода из STOP.
 * Поэтому они не static и объявлены с C linkage.
 */
extern "C" void SystemClock_Config(void);
extern "C" void MX_GPIO_Init(void);
extern "C" void MX_USART1_UART_Init(void);
extern "C" void MX_USART2_UART_Init(void);
extern "C" void MX_USART3_UART_Init(void);
extern "C" void MX_USART6_UART_Init(void);
extern "C" void MX_I2C1_Init(void);
extern "C" void MX_RTC_Init(void);
extern "C" void MX_TIM6_Init(void);
extern "C" void MX_RNG_Init(void);
extern "C" void MX_DMA_Init(void);
extern "C" bool HttpsW5500_tlsOneTimeInit(void);

/* =============================================================
 * main()
 * ============================================================= */
extern "C" int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();

    {
        const char boot1[] = "BOOT1 USART1 115200\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)boot1, sizeof(boot1) - 1, 1000);
        HAL_UART_Transmit(&huart6, (uint8_t*)boot1, sizeof(boot1) - 1, 1000);
    }

    CheckResetReason();

    // CircularLogBuffer lives in CCMRAM. Clear pointers before first UART/DBG print.
    CircularLogBuffer::init_ccmram();

    DBG.setMirror(&huart6);
    DBG.init();
    DBG.info("BOOT OK");

    // Логируем причину ресета — теперь DBG готов
    if (g_reset_was_wdg) DBG.warn("Reset reason: %s", g_reset_reason_str);
    else                 DBG.info("Reset reason: %s", g_reset_reason_str);

    DBG.info("HAL + Clock + GPIO + UART1/6 : OK");

    MX_I2C1_Init();
    DBG.info("I2C1 OK");

    MX_USART2_UART_Init();
    DBG.info("UART2 OK");

    MX_USART3_UART_Init();
    DBG.info("UART3 OK");

    MX_RNG_Init();
    DBG.info("RNG OK");

    // TLS pre-init ДО IWDG — теперь должен пройти быстро (<1с)
    // после исправления конфига

    DBG.info("TLS pre-init...");
    HttpsW5500_tlsOneTimeInit();   // <-- просто вызов, без объявления
    DBG.info("TLS pre-init DONE");

    MX_IWDG_Init();
    DBG.info("IWDG OK");

    MX_SPI1_Init();
    DBG.info("SPI1 OK");

    // DMA2 clock enable (даже без использования SDIO DMA)
    MX_DMA_Init();
    DBG.info("DMA OK");

    // Всегда пробуем инициализировать SD (один раз).
    // g_sd_disabled=true только при реальном сбое SDIO — не при типе ресета.
    DBG.info("MARK before SDIO");
    HAL_Delay(100);
    if (!MX_SDIO_SD_Init()) {
        DBG.error("SDIO: init failed — SD disabled for this boot");
        g_sd_disabled = true;
    } else {
        DBG.info("SDIO init done");
        DBG.info("MARK before FATFS");
        MX_FATFS_Init();
        DBG.info("FATFS OK");
    }

    MX_RTC_Init();
    DBG.info("RTC OK");

    MX_TIM6_Init();
    DBG.info("TIM6 OK");

    DBG.separator();
    DBG.info("APP START");
    DBG.separator();

    static App app;  // static: placed in BSS, not on main() stack (~100KB object)
    app.init();
    app.run();

    return 0;
}

static void CheckResetReason(void)
{
    uint32_t flags = RCC->CSR;
    bool iwdg_rst = (flags & RCC_CSR_IWDGRSTF) != 0;
    bool wwdg_rst = (flags & RCC_CSR_WWDGRSTF) != 0;
    bool soft_rst = (flags & RCC_CSR_SFTRSTF)  != 0;

    // Сохраняем строку — DBG ещё не инициализирован, вызов здесь вызовет HardFault.
    // Логируем позже, после DBG.init().
    if      (iwdg_rst) { g_reset_reason_str = "IWDG watchdog"; g_reset_was_wdg = true; }
    else if (wwdg_rst) { g_reset_reason_str = "WWDG watchdog"; g_reset_was_wdg = true; }
    else if (soft_rst) { g_reset_reason_str = "soft reset (flash/debug)"; }
    else               { g_reset_reason_str = "power-on"; }

    __HAL_RCC_CLEAR_RESET_FLAGS();
}
/* =============================================================
 * SystemClock_Config — HSE 8 MHz -> PLL -> 168 MHz
 * ============================================================= */
extern "C" void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clk = {};
    RCC_PeriphCLKInitTypeDef periph = {};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    osc.HSIState       = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSEState       = RCC_LSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM       = 16;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV4;
    osc.PLL.PLLQ       = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 |
                         RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    // RTC: LSE (32768 Гц)
    // SDIO: PLLQ = 48 МГц (PLLQ=7: 16/16*336/7 = 48 МГц — точное значение для SDIO)
    // SDIO на STM32F407 тактируется от PLLQ (CLK48) аппаратно.
    // RCC_PERIPHCLK_SDIO не определён для F407 — настройки не нужны.
    periph.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    periph.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;

    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
        Error_Handler();
    }
}




/* =============================================================
 * Watchdog: причина сброса + инициализация IWDG
 * ============================================================= */


static void MX_IWDG_Init(void)
{
    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) {
    }

    IWDG->KR = 0x5555;
    IWDG->PR = 6;        // делитель 256
    IWDG->RLR = 4095;    // максимум
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
}

/* =============================================================
 * MX_GPIO_Init
 * ============================================================= */
extern "C" void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Начальные уровни */
    HAL_GPIO_WritePin(GPIOC, STATUS_LED_Pin | SIM800_PWR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

    /* W5500: CS=1, RST=1 (не в reset) */
    HAL_GPIO_WritePin(PIN_W5500_CS_PORT,  PIN_W5500_CS_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(PIN_W5500_RST_PORT, PIN_W5500_RST_PIN, GPIO_PIN_SET);

    /* STATUS_LED + SIM800_PWR */
    g.Pin   = STATUS_LED_Pin | SIM800_PWR_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);

    /* RS485_DE */
    g.Pin   = RS485_DE_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &g);

    /* MODE_SWITCH (PC0) input pull-up */
    g.Pin   = MODE_SWITCH_Pin;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MODE_SWITCH_GPIO_Port, &g);

    /* NET_SELECT (PB0) input pull-up */
    g.Pin   = PIN_NET_SW_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_NET_SW_PORT, &g);

    /* W5500_CS (PA8) output */
    g.Pin   = PIN_W5500_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PIN_W5500_CS_PORT, &g);

    /* W5500_RST (PC3) output */
    g.Pin   = PIN_W5500_RST_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PIN_W5500_RST_PORT, &g);

    /* W5500_INT (PC4) input */
    g.Pin   = PIN_W5500_INT_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_W5500_INT_PORT, &g);
}

/* =============================================================
 * USART1 — Debug 115200 8N1 (PA9/PA10)
 * ============================================================= */
extern "C" void MX_USART1_UART_Init(void)
{
    huart1.Instance        = USART1;
    huart1.Init.BaudRate   = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits   = UART_STOPBITS_1;
    huart1.Init.Parity     = UART_PARITY_NONE;
    huart1.Init.Mode       = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* =============================================================
 * USART2 — (PA2/PA3)
 * ============================================================= */
extern "C" void MX_USART2_UART_Init(void)
{
    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    /*
     * КРИТИЧНО: включить RXNEIE ПОСЛЕ HAL_UART_Init().
     * HAL_UART_Init() перезаписывает CR1 (TE|RE без RXNEIE),
     * поэтому RXNEIE выставленный в MspInit — сбрасывается.
     * Сбрасываем флаги ошибок SR/DR накопившиеся при старте модема.
     */
    (void)USART2->SR;
    (void)USART2->DR;
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* =============================================================
 * USART3 — RS-485 Modbus 9600 8N1 (PB10/PB11)
 * ============================================================= */
extern "C" void MX_USART3_UART_Init(void)
{
    huart3.Instance        = USART3;
    huart3.Init.BaudRate   = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits   = UART_STOPBITS_1;
    huart3.Init.Parity     = UART_PARITY_NONE;
    huart3.Init.Mode       = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

/* =============================================================
 * USART6 — Mirror debug UART
 * ============================================================= */
extern "C" void MX_USART6_UART_Init(void)
{
    huart6.Instance        = USART6;
    huart6.Init.BaudRate   = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits   = UART_STOPBITS_1;
    huart6.Init.Parity     = UART_PARITY_NONE;
    huart6.Init.Mode       = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
}

/* =============================================================
 * I2C1 — DS3231 100 kHz (PB6/PB7)
 * ============================================================= */
extern "C" void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/* =============================================================
 * RNG — аппаратный генератор случайных чисел
 * ============================================================= */
extern "C" void MX_RNG_Init(void)
{
    __HAL_RCC_RNG_CLK_ENABLE();

    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) != HAL_OK) {
        Error_Handler();
    }
}

/* =============================================================
 * RTC — LSE 32.768 kHz, WakeUp Timer
 * ============================================================= */
extern "C" void MX_RTC_Init(void)
{
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;
    hrtc.Init.SynchPrediv    = 255;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

/* =============================================================
 * TIM6 — Modbus timeout 1 мс
 * ============================================================= */
extern "C" void MX_TIM6_Init(void)
{
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 999;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* =============================================================
 * Error_Handler — reset по фатальной ошибке
 * ============================================================= */
extern "C" void Error_Handler(void)
{
    const char msg[] = "\r\nFATAL ERROR\r\n";

    /* Пытаемся вывести хотя бы в USART1/USART6 */
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg) - 1, 200);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg) - 1, 200);

    HAL_Delay(100);
    NVIC_SystemReset();
}
