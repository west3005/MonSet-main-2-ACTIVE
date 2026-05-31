/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f4xx_it.h"
#include "dns.h"

/* USER CODE BEGIN Includes */
#include "uart_ringbuf.hpp"
/* USER CODE END Includes */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern SD_HandleTypeDef  hsd;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  while (1) {}
}

#include "dbg_cwrap.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* Extract stacked registers from exception frame for fault diagnosis */
void HardFault_Impl(uint32_t* sp);
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4          \n"  /* test EXC_RETURN bit 2: 0=MSP, 1=PSP */
        "ite eq              \n"
        "mrseq r0, msp       \n"
        "mrsne r0, psp       \n"
        "b HardFault_Impl    \n"
        :::"r0"
    );
}
void HardFault_Impl(uint32_t* sp) {
    __DSB();
    uint32_t stacked_r0  = sp[0];
    uint32_t stacked_r1  = sp[1];
    uint32_t stacked_r2  = sp[2];
    uint32_t stacked_r3  = sp[3];
    uint32_t stacked_r12 = sp[4];
    uint32_t stacked_lr  = sp[5];
    uint32_t stacked_pc  = sp[6];
    uint32_t stacked_xpsr= sp[7];
    (void)stacked_r0; (void)stacked_r1; (void)stacked_r2; (void)stacked_r3;
    (void)stacked_r12; (void)stacked_xpsr;
    char buf[128];
    snprintf(buf, sizeof(buf),
             "!!! HARDFAULT CFSR=0x%08lX HFSR=0x%08lX\r\n"
             "    PC=0x%08lX LR=0x%08lX SP=0x%08lX\r\n",
             (unsigned long)SCB->CFSR,
             (unsigned long)SCB->HFSR,
             (unsigned long)stacked_pc,
             (unsigned long)stacked_lr,
             (unsigned long)sp);
    SCB->CFSR = SCB->CFSR;
    SCB->HFSR = SCB->HFSR;
    dbg_puts(buf);
    HAL_Delay(20);
    NVIC_SystemReset();
}
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {}
}

void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */
  while (1) {}
}

void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {}
}

void SVC_Handler(void)      {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)   {}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_WKUP_IRQn 0 */
  /* USER CODE END RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_WKUP_IRQn 1 */
  /* USER CODE END RTC_WKUP_IRQn 1 */
}

void SDIO_IRQHandler(void)
{
  /* USER CODE BEGIN SDIO_IRQn 0 */
  /* USER CODE END SDIO_IRQn 0 */
  HAL_SD_IRQHandler(&hsd);
  /* USER CODE BEGIN SDIO_IRQn 1 */
  /* USER CODE END SDIO_IRQn 1 */
}

void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /* DNS_time_handler must be called every 1 second.
   * TIM6 fires every 0.5 ms (168MHz / 84 / 1000), so tick every 2000 calls = 1 sec. */
  static uint16_t s_dns_tick_cnt = 0;
  if (++s_dns_tick_cnt >= 2000u) {
      s_dns_tick_cnt = 0;
      DNS_time_handler();
  }
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
 * @brief USART2 IRQ — Air780E RX приёмник.
 *        Каждый принятый байт кладётся в кольцевой буфер g_air780_rxbuf.
 *        Драйвер air780e.cpp читает из буфера в polling-режиме.
 *        Используем C-обёртку air780_rxbuf_push() т.к. файл компилируется как C.
 */
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;   /* Читаем SR первым — обязательно для STM32F4 */

    if (sr & USART_SR_RXNE)
    {
        /* Штатный приём: чтение DR сбрасывает RXNE */
        uint8_t ch = (uint8_t)(USART2->DR & 0xFFU);
        air780_rxbuf_push(ch);
        return;
    }

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        /* Ошибочные флаги сбрасываются ТОЛЬКО последовательным чтением SR -> DR.
         * Если не прочитать DR — IRQ будет вызываться бесконечно (stall ISR).
         * NE/FE/PE возникают при старте модема (TX линия ещё не установилась).
         * ORE — байт при этом всё же есть в DR, сохраняем его. */
        uint8_t ch = (uint8_t)(USART2->DR & 0xFFU);
        if (sr & USART_SR_ORE)
        {
            air780_rxbuf_push(ch);
        }
        /* NE / FE / PE при старте — мусорный байт, игнорируем */
    }
}
/* USER CODE END 1 */
