/**
 * @file sd_backup.cpp
 * @brief Резервное хранилище измерений на SD-карте (FATFS JSONL).
 */
#include "sd_backup.hpp"
#include "debug_uart.hpp"
#include "fatfs.h"
#include <cstring>
#include <cstdio>

extern "C" {
#include "sdio.h"

#include "stm32f4xx_hal_sd.h"
}

/* ------------------------------------------------------------------ */
static const char* frStr(FRESULT fr)
{
    switch (fr) {
        case FR_OK:                  return "FR_OK";
        case FR_DISK_ERR:            return "FR_DISK_ERR";
        case FR_INT_ERR:             return "FR_INT_ERR";
        case FR_NOT_READY:           return "FR_NOT_READY";
        case FR_NO_FILE:             return "FR_NO_FILE";
        case FR_NO_PATH:             return "FR_NO_PATH";
        case FR_INVALID_NAME:        return "FR_INVALID_NAME";
        case FR_DENIED:              return "FR_DENIED";
        case FR_EXIST:               return "FR_EXIST";
        case FR_INVALID_OBJECT:      return "FR_INVALID_OBJECT";
        case FR_WRITE_PROTECTED:     return "FR_WRITE_PROTECTED";
        case FR_INVALID_DRIVE:       return "FR_INVALID_DRIVE";
        case FR_NOT_ENABLED:         return "FR_NOT_ENABLED";
        case FR_NO_FILESYSTEM:       return "FR_NO_FILESYSTEM";
        case FR_MKFS_ABORTED:        return "FR_MKFS_ABORTED";
        case FR_TIMEOUT:             return "FR_TIMEOUT";
        case FR_LOCKED:              return "FR_LOCKED";
        case FR_NOT_ENOUGH_CORE:     return "FR_NOT_ENOUGH_CORE";
        case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";
        case FR_INVALID_PARAMETER:   return "FR_INVALID_PARAMETER";
        default:                     return "FR_???";
    }
}

/* ------------------------------------------------------------------ */
void SdBackup::make_drive(char* out, size_t out_sz) const
{
    if (out_sz < 3) return;
    out[0] = SDPath[0] ? SDPath[0] : '0';
    out[1] = ':';
    out[2] = '\0';
}

void SdBackup::make_full_path(char* out, size_t out_sz, const char* fname) const
{
    char drive[3];
    make_drive(drive, sizeof(drive));
    std::snprintf(out, out_sz, "%s/%s", drive, fname);
}

/* ------------------------------------------------------------------ */
bool SdBackup::init()
{
    if (m_broken) {
        DBG.warn("SD: previously marked as broken, skip init");
        m_mounted = false;
        return false;
    }

    char drive[3];
    make_drive(drive, sizeof(drive));

    // HAL_SD already initialised in main.cpp before SdBackup::init() is called.
    // Re-running DeInit/Init here resets the SDIO peripheral mid-flight and
    // causes the card to need extra settle time, leading to FR_DISK_ERR on
    // the first f_getfree / f_open call. Just mount the already-ready card.

    const uint32_t t0        = HAL_GetTick();
    const uint32_t timeoutMs = 5000;
    FRESULT fr = FR_INT_ERR;

    while ((HAL_GetTick() - t0) < timeoutMs) {
        fr = f_mount(&m_fatfs, drive, 1);
        if (fr == FR_OK) break;
        HAL_Delay(50);
    }

    if (fr != FR_OK) {
        DBG.error("SD: mount fail FR=%d %s", (int)fr, frStr(fr));
        m_mounted = false;
        m_broken  = true;
        return false;
    }

    m_mounted = true;
    DBG.info("SD: mounted drive=%s", drive);
    return true;
}

/* ------------------------------------------------------------------ */
void SdBackup::deinit()
{
    if (!m_mounted) return;
    char drive[3];
    make_drive(drive, sizeof(drive));
    f_mount(nullptr, drive, 0);
    m_mounted = false;
    DBG.info("SD: unmounted drive=%s", drive);
}

/* ------------------------------------------------------------------ */
bool SdBackup::exists() const
{
    if (!m_mounted) return false;
    char path[64];
    make_full_path(path, sizeof(path), Config::BACKUP_FILENAME);
    FILINFO fno;
    return (f_stat(path, &fno) == FR_OK);
}

/* ------------------------------------------------------------------ */
bool SdBackup::remove()
{
    if (!m_mounted) return false;
    char path[64];
    make_full_path(path, sizeof(path), Config::BACKUP_FILENAME);
    FRESULT fr = f_unlink(path);
    if (fr != FR_OK) {
        DBG.error("SD: unlink fail path=%s (FR=%d %s)", path, (int)fr, frStr(fr));
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
DWORD SdBackup::getFreeSpaceBytes() const
{
    char drive[3];
    make_drive(drive, sizeof(drive));
    FATFS* fs      = nullptr;
    DWORD fre_clust = 0;
    FRESULT fr = f_getfree(drive, &fre_clust, &fs);
    if (fr != FR_OK || fs == nullptr) {
        DBG.error("SD: f_getfree fail (FR=%d %s)", (int)fr, frStr(fr));
        return 0;
    }
    return fre_clust * fs->csize * 512UL;
}

/* ------------------------------------------------------------------ */
bool SdBackup::checkAndRotateFile(FIL& f, const char* path)
{
    FSIZE_t sz = f_size(&f);
    if (sz < MAX_BACKUP_FILE_SIZE) {
        return true;
    }
    DBG.warn("SD: backup file too big (%lu bytes) -> rotate (truncate)",
             (unsigned long)sz);
    f_close(&f);
    FRESULT fr = f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        DBG.error("SD: rotate reopen fail path=%s (FR=%d %s)",
                  path, (int)fr, frStr(fr));
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
static bool writeLineWithRetry(FIL& f, const char* data, UINT len,
                                uint8_t maxRetries)
{
    FRESULT fr = FR_OK;
    UINT    bw = 0;
    for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
        if (attempt > 0) {
            DBG.warn("SD: write retry %u/%u",
                     (unsigned)attempt, (unsigned)maxRetries);
            HAL_Delay(50);
        }
        fr = f_write(&f, data, len, &bw);
        if (fr == FR_OK && bw == len) {
            return true;
        }
        DBG.error("SD: write attempt %u failed (FR=%d %s bw=%u/%u)",
                  (unsigned)attempt + 1, (int)fr, frStr(fr),
                  (unsigned)bw, (unsigned)len);
        if (fr != FR_DISK_ERR) {
            break; /* Не дисковая ошибка — повтор бесполезен */
        }
        f_sync(&f);
    }
    return false;
}

/* ------------------------------------------------------------------ */
static bool syncWithRetry(FIL& f, uint8_t maxRetries)
{
    FRESULT fr = FR_OK;
    for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
        if (attempt > 0) {
            DBG.warn("SD: sync retry %u/%u",
                     (unsigned)attempt, (unsigned)maxRetries);
            HAL_Delay(20);
        }
        fr = f_sync(&f);
        if (fr == FR_OK) {
            return true;
        }
        DBG.error("SD: sync attempt %u failed (FR=%d %s)",
                  (unsigned)attempt + 1, (int)fr, frStr(fr));
    }
    return false;
}

/* ------------------------------------------------------------------ */
bool SdBackup::appendLine(const char* jsonLine)
{
    if (!m_mounted || !jsonLine) {
        DBG.error("SD: appendLine not mounted or null");
        return false;
    }

    const size_t n = std::strlen(jsonLine);
    if (n == 0 || n > Config::JSONL_LINE_MAX) {
        DBG.error("SD: JSONL line bad length (%u)", (unsigned)n);
        return false;
    }

    for (size_t i = 0; i < n; i++) {
        if (jsonLine[i] == '\r' || jsonLine[i] == '\n') {
            DBG.error("SD: JSONL line contains CR/LF at pos %u", (unsigned)i);
            return false;
        }
    }

    const DWORD freeBytes = getFreeSpaceBytes();
    const DWORD needBytes = (DWORD)n + 4U;
    if (freeBytes != 0 && freeBytes < needBytes) {
        DBG.error("SD: not enough free space (have=%lu need=%lu), skip line",
                  (unsigned long)freeBytes, (unsigned long)needBytes);
        return false;
    }

    char path[64];
    make_full_path(path, sizeof(path), Config::BACKUP_FILENAME);

    FIL     f{};
    FRESULT fr       = FR_INT_ERR;
    const uint8_t openRetries = 3;
    for (uint8_t attempt = 0; attempt < openRetries; attempt++) {
        if (attempt > 0) {
            DBG.warn("SD: open retry %u/%u",
                     (unsigned)attempt, (unsigned)openRetries);
            HAL_Delay(50);
        }
        fr = f_open(&f, path, FA_OPEN_ALWAYS | FA_WRITE);
        if (fr == FR_OK) break;
        DBG.error("SD: open for append fail attempt %u path=%s (FR=%d %s)",
                  (unsigned)attempt + 1, path, (int)fr, frStr(fr));
    }
    if (fr != FR_OK) {
        return false;
    }

    if (!checkAndRotateFile(f, path)) {
        f_close(&f);
        return false;
    }

    /* Seek to end — проверяем результат, иначе запишем в случайное место */
    fr = f_lseek(&f, f_size(&f));
    if (fr != FR_OK) {
        DBG.error("SD: lseek(end) fail path=%s (FR=%d %s)",
                  path, (int)fr, frStr(fr));
        f_close(&f);
        return false;
    }

    if (!writeLineWithRetry(f, jsonLine, (UINT)n, 3)) {
        DBG.error("SD: write line failed after retries");
        f_close(&f);
        return false;
    }

    const char eol[] = "\r\n";
    if (!writeLineWithRetry(f, eol, sizeof(eol) - 1, 3)) {
        DBG.error("SD: write EOL failed after retries");
        f_close(&f);
        return false;
    }

    if (!syncWithRetry(f, 3)) {
        DBG.error("SD: sync failed after retries");
        f_close(&f);
        return false;
    }

    f_close(&f);
    return true;
}

/* ------------------------------------------------------------------ */
bool SdBackup::readChunkAsJsonArray(char* out, uint32_t outSize,
                                     uint32_t maxPayloadBytes,
                                     uint32_t& linesRead,
                                     FSIZE_t& bytesUsedFromFile)
{
    linesRead        = 0;
    bytesUsedFromFile = 0;
    if (!m_mounted || !out || outSize < 4) return false;
    if (maxPayloadBytes >= outSize) maxPayloadBytes = outSize - 1;
    if (maxPayloadBytes < 4) return false;

    char path[64];
    make_full_path(path, sizeof(path), Config::BACKUP_FILENAME);

    FIL f{};
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        DBG.error("SD: open for read fail path=%s (FR=%d %s)",
                  path, (int)fr, frStr(fr));
        return false;
    }

    uint32_t off = 0;
    out[off++] = '[';

    char   line[Config::JSONL_LINE_MAX + 4];
    FSIZE_t lastPosAfterLine = 0;

    while (true) {
        char* s = f_gets(line, sizeof(line), &f);
        if (!s) break;
        lastPosAfterLine = f_tell(&f);

        size_t n = std::strlen(line);
        while (n > 0 && (line[n-1] == '\r' || line[n-1] == '\n' ||
                          line[n-1] == ' '  || line[n-1] == '\t')) {
            line[--n] = '\0';
        }
        if (n == 0) {
            bytesUsedFromFile = lastPosAfterLine;
            continue;
        }

        uint32_t need = (linesRead ? 1u : 0u) + (uint32_t)n + 2u;
        if (off + need > maxPayloadBytes) break;
        if (off + need > outSize)         break;

        if (linesRead) out[off++] = ',';
        std::memcpy(&out[off], line, n);
        off += (uint32_t)n;
        linesRead++;
        bytesUsedFromFile = lastPosAfterLine;
    }

    out[off++] = ']';
    out[off]   = '\0';
    f_close(&f);

    if (linesRead == 0 || bytesUsedFromFile == 0) {
        DBG.error("SD: readChunk got empty (lines=%u used=%lu) path=%s",
                  (unsigned)linesRead, (unsigned long)bytesUsedFromFile, path);
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
bool SdBackup::consumePrefix(FSIZE_t bytesUsedFromFile)
{
    if (!m_mounted || bytesUsedFromFile == 0) return true;

    char path[64];
    make_full_path(path, sizeof(path), Config::BACKUP_FILENAME);
    char tmpPath[64];
    make_full_path(tmpPath, sizeof(tmpPath), "backup.tmp");

    FIL src{};
    FRESULT fr = f_open(&src, path, FA_READ);
    if (fr != FR_OK) {
        DBG.error("SD: consume open src fail path=%s (FR=%d %s)",
                  path, (int)fr, frStr(fr));
        return false;
    }

    FSIZE_t sz = f_size(&src);
    if (bytesUsedFromFile >= sz) {
        f_close(&src);
        fr = f_unlink(path);
        if (fr != FR_OK) {
            DBG.error("SD: consume unlink fail path=%s (FR=%d %s)",
                      path, (int)fr, frStr(fr));
            return false;
        }
        return true;
    }

    FIL dst{};
    if (f_open(&dst, tmpPath, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        f_close(&src);
        return false;
    }

    fr = f_lseek(&src, bytesUsedFromFile);
    if (fr != FR_OK) {
        DBG.error("SD: consume lseek fail (FR=%d %s)", (int)fr, frStr(fr));
        f_close(&src);
        f_close(&dst);
        f_unlink(tmpPath);
        return false;
    }

    uint8_t buf[512];
    UINT br = 0, bw = 0;
    while (true) {
        fr = f_read(&src, buf, sizeof(buf), &br);
        if (fr != FR_OK) {
            DBG.error("SD: consume read fail (FR=%d %s)", (int)fr, frStr(fr));
            break;
        }
        if (br == 0) { fr = FR_OK; break; }
        fr = f_write(&dst, buf, br, &bw);
        if (fr != FR_OK || bw != br) {
            DBG.error("SD: consume write fail (FR=%d %s bw=%u/%u)",
                      (int)fr, frStr(fr), (unsigned)bw, (unsigned)br);
            fr = FR_DISK_ERR;
            break;
        }
    }

    f_close(&src);
    f_close(&dst);

    if (fr != FR_OK) {
        f_unlink(tmpPath);
        return false;
    }

    f_unlink(path);
    fr = f_rename(tmpPath, path);
    if (fr != FR_OK) {
        DBG.error("SD: consume rename fail (FR=%d %s)", (int)fr, frStr(fr));
        f_unlink(tmpPath);
        return false;
    }
    return true;
}
