/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file sd_diskio.c
  * @brief SD Disk I/O driver — polling HAL, no DMA, no IRQ disable.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "sdio.h"
#include "stm32f4xx_hal_sd.h"
#include "main.h"
#include "debug_uart_c.h"

#ifndef SD_TIMEOUT
#define SD_TIMEOUT   (5U * 1000U)
#endif

#define SD_DEFAULT_BLOCK_SIZE  512U

static volatile DSTATUS Stat = STA_NOINIT;

DSTATUS SD_initialize(BYTE);
DSTATUS SD_status    (BYTE);
DRESULT SD_read      (BYTE, BYTE *, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write     (BYTE, const BYTE *, DWORD, UINT);
#endif
#if _USE_IOCTL == 1
DRESULT SD_ioctl     (BYTE, BYTE, void *);
#endif

const Diskio_drvTypeDef SD_Driver = {
    SD_initialize,
    SD_status,
    SD_read,
#if _USE_WRITE == 1
    SD_write,
#endif
#if _USE_IOCTL == 1
    SD_ioctl,
#endif
};

static inline void SD_ClearFlags(void)
{
    __HAL_SD_CLEAR_FLAG(&hsd,
        SDIO_FLAG_CCRCFAIL | SDIO_FLAG_DCRCFAIL |
        SDIO_FLAG_CTIMEOUT | SDIO_FLAG_DTIMEOUT |
        SDIO_FLAG_TXUNDERR | SDIO_FLAG_RXOVERR  |
        SDIO_FLAG_CMDREND  | SDIO_FLAG_CMDSENT  |
        SDIO_FLAG_DATAEND  | SDIO_FLAG_STBITERR |
        SDIO_FLAG_DBCKEND);
}

static HAL_StatusTypeDef SD_WaitCardReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    HAL_SD_CardStateTypeDef st;
    while ((HAL_GetTick() - t0) < timeout_ms) {
        st = HAL_SD_GetCardState(&hsd);
        if (st == HAL_SD_CARD_TRANSFER) {
            return HAL_OK;
        }
        HAL_Delay(2);
    }
    st = HAL_SD_GetCardState(&hsd);
    uart_log_error("[DISKIO] WaitReady timeout: last CardState=%d hsd.State=%d ErrorCode=0x%08lX",
                   (int)st, (int)hsd.State, (unsigned long)hsd.ErrorCode);
    return HAL_TIMEOUT;
}

static DSTATUS SD_CheckStatus(BYTE lun)
{
    (void)lun;
    Stat = STA_NOINIT;
    HAL_SD_CardStateTypeDef cs = HAL_SD_GetCardState(&hsd);
    /* ErrorCode accumulates diagnostic bits during ReadBlocks while-loop.
     * Clear it here so the next CMD13 is not masked by stale error flags. */
    hsd.ErrorCode = HAL_SD_ERROR_NONE;
    if (cs == HAL_SD_CARD_TRANSFER) {
        Stat &= (DSTATUS)~STA_NOINIT;
    }
    return Stat;
}

DSTATUS SD_initialize(BYTE lun)
{
    (void)lun;
    Stat = STA_NOINIT;

    uart_log_info("[DISKIO] SD_initialize: hsd.State=%d CardState=%d",
                  (int)hsd.State, (int)HAL_SD_GetCardState(&hsd));

    /* Если карта уже в TRANSFER — просто фиксируем статус */
    if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) {
        uart_log_info("[DISKIO] SD_initialize: card already TRANSFER — fast path");
        Stat = SD_CheckStatus(lun);
        uart_log_info("[DISKIO] SD_initialize: Stat=0x%02X", (unsigned)Stat);
        return Stat;
    }

    /* Полная реинициализация — НЕ делаем DeInit чтобы не трогать RCC/GPIO */
    uart_log_info("[DISKIO] SD_initialize: full reinit path");
    SD_ClearFlags();

    /* Переинициализируем HAL без DeInit (GPIO и CLK уже настроены MspInit) */
    hsd.State = HAL_SD_STATE_RESET;
    if (!MX_SDIO_SD_Init()) {
        uart_log_error("[DISKIO] SD_initialize: MX_SDIO_SD_Init failed");
        return Stat;
    }

    uart_log_info("[DISKIO] SD_initialize: after MX_Init: hsd.State=%d CardState=%d",
                  (int)hsd.State, (int)HAL_SD_GetCardState(&hsd));

    if (SD_WaitCardReady(2000U) != HAL_OK) {
        uart_log_error("[DISKIO] SD_initialize: WaitCardReady failed");
        return Stat;
    }

    /* Переход на рабочую скорость: 24 МГц */
    hsd.Init.ClockDiv = 0U;
    MODIFY_REG(SDIO->CLKCR, SDIO_CLKCR_CLKDIV, 0U);

    Stat = SD_CheckStatus(lun);
    uart_log_info("[DISKIO] SD_initialize: done Stat=0x%02X ClockDiv=%lu",
                  (unsigned)Stat, (unsigned long)(SDIO->CLKCR & 0xFF));
    return Stat;
}

DSTATUS SD_status(BYTE lun)
{
    return SD_CheckStatus(lun);
}

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    HAL_StatusTypeDef hs;
    (void)lun;

    if (Stat & STA_NOINIT)               { return RES_NOTRDY; }
    if ((buff == NULL) || (count == 0U)) { return RES_PARERR; }

    uart_log_info("[DISKIO] read: sec=%lu cnt=%u", (unsigned long)sector, (unsigned)count);

    SD_ClearFlags();

    /* Сброс data path перед новой транзакцией */
    SDIO->DCTRL = 0U;
    __DSB(); __ISB();

    HAL_SD_CardStateTypeDef cs = HAL_SD_GetCardState(&hsd);
    uart_log_info("[DISKIO] read pre: CLKCR=0x%08lX CardState=%d RESP1=0x%08lX CardType=%lu RCA=0x%04lX",
                  (unsigned long)SDIO->CLKCR,
                  (int)cs,
                  (unsigned long)SDIO->RESP1,
                  (unsigned long)hsd.SdCard.CardType,
                  (unsigned long)hsd.SdCard.RelCardAdd);

    hs = HAL_SD_ReadBlocks(&hsd, (uint8_t *)buff,
                           (uint32_t)sector, (uint32_t)count, SD_TIMEOUT);
    if (hs != HAL_OK) {
        uart_log_error("[DISKIO] read: HAL=%d State=%d ErrorCode=0x%08lX STA=0x%08lX RESP1=0x%08lX",
                       (int)hs, (int)hsd.State,
                       (unsigned long)hsd.ErrorCode,
                       (unsigned long)SDIO->STA,
                       (unsigned long)SDIO->RESP1);
        hsd.State = HAL_SD_STATE_READY;
        SD_ClearFlags();
        return RES_ERROR;
    }

    if (SD_WaitCardReady(SD_TIMEOUT) != HAL_OK) {
        uart_log_error("[DISKIO] read: WaitReady timeout");
        SD_ClearFlags();
        return RES_ERROR;
    }

    return RES_OK;
}

#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    HAL_StatusTypeDef hs;
    (void)lun;

    if (Stat & STA_NOINIT)               { return RES_NOTRDY; }
    if ((buff == NULL) || (count == 0U)) { return RES_PARERR; }

    uart_log_info("[DISKIO] write: sec=%lu cnt=%u CardState=%d",
                  (unsigned long)sector, (unsigned)count,
                  (int)HAL_SD_GetCardState(&hsd));

    SD_ClearFlags();
    /*
     * TXUNDERR fix: STM32F4 polling write не поддерживает CMD25 (multiblock).
     * При CMD25 карта переходит в PRG после каждого блока — DPSM продолжает
     * тактировать шину пока карта не принимает → TXUNDERR на блоке 2+.
     * Решение: всегда писать одиночными CMD24, по одному блоку за раз.
     * Частота 24 МГц (ClockDiv=0), HWFC=0 (errata STM32F4).
     */
    MODIFY_REG(SDIO->CLKCR,
               SDIO_CLKCR_CLKDIV | SDIO_CLKCR_CLKEN | SDIO_CLKCR_HWFC_EN | SDIO_CLKCR_BYPASS,
               (0U) | SDIO_CLKCR_CLKEN);  /* ClockDiv=0 → 24 МГц */
    HAL_Delay(2);

    hs = HAL_OK;
    for (UINT blk = 0U; blk < count; blk++) {
        SDIO->DCTRL = 0U;
        __DSB(); __ISB();
        hs = HAL_SD_WriteBlocks(&hsd,
                                (uint8_t *)buff + blk * 512U,
                                (uint32_t)sector + blk,
                                1U,
                                SD_TIMEOUT);
        if (hs != HAL_OK) {
            uart_log_error("[DISKIO] write: blk=%u HAL=%d State=%d ErrorCode=0x%08lX STA=0x%08lX CardState=%d",
                           (unsigned)blk, (int)hs, (int)hsd.State,
                           (unsigned long)hsd.ErrorCode,
                           (unsigned long)SDIO->STA,
                           (int)HAL_SD_GetCardState(&hsd));
            hsd.State = HAL_SD_STATE_READY;
            SD_ClearFlags();
            return RES_ERROR;
        }
        /* Ждём готовности карты перед следующим блоком */
        if (blk + 1U < count) {
            if (SD_WaitCardReady(SD_TIMEOUT) != HAL_OK) {
                uart_log_error("[DISKIO] write: card not ready after blk=%u", (unsigned)blk);
                return RES_ERROR;
            }
        }
    }
    if (SD_WaitCardReady(SD_TIMEOUT) != HAL_OK) {
        uart_log_error("[DISKIO] write: WaitReady timeout");
        SD_ClearFlags();
        return RES_ERROR;
    }

    uart_log_info("[DISKIO] write: OK");
    return RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    (void)lun;
    DRESULT res = RES_ERROR;

    if (Stat & STA_NOINIT) { return RES_NOTRDY; }

    switch (cmd) {
        case CTRL_SYNC: {
            SD_ClearFlags();
            res = (SD_WaitCardReady(SD_TIMEOUT) == HAL_OK) ? RES_OK : RES_ERROR;
            break;
        }
        case GET_SECTOR_COUNT: {
            HAL_SD_CardInfoTypeDef info;
            HAL_SD_GetCardInfo(&hsd, &info);
            *(DWORD *)buff = info.LogBlockNbr;
            res = RES_OK;
            break;
        }
        case GET_SECTOR_SIZE: {
            HAL_SD_CardInfoTypeDef info;
            HAL_SD_GetCardInfo(&hsd, &info);
            *(WORD *)buff = (WORD)info.LogBlockSize;
            res = RES_OK;
            break;
        }
        case GET_BLOCK_SIZE: {
            HAL_SD_CardInfoTypeDef info;
            HAL_SD_GetCardInfo(&hsd, &info);
            *(DWORD *)buff = info.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
            res = RES_OK;
            break;
        }
        default:
            res = RES_PARERR;
            break;
    }
    return res;
}
#endif
