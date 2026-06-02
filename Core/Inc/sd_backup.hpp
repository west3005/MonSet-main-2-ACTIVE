#ifndef SD_BACKUP_HPP
#define SD_BACKUP_HPP

#include "main.h"
#include "config.hpp"

#include <cstdint>
#include <cstddef>

extern "C" {
#include "ff.h"
#include "fatfs.h"
}

class SdBackup {
public:
    SdBackup() = default;

    /**
     * @brief Задать имя файла бэкапа (должен быть вызван до init).
     *
     * По умолчанию используется Config::BACKUP_FILENAME ("backup.jsn").
     * Для датчиковых портов вызывать с именем из rtu_ports[N].backup_filename:
     *   m_sdBackup0.setFilename("backup_p0.jsn");
     *   m_sdBackup1.setFilename("backup_p1.jsn");
     *   m_sdBackup2.setFilename("backup_p2.jsn");
     */
    void setFilename(const char* name);

    /** @brief Вернуть текущее имя файла бэкапа. */
    const char* filename() const { return m_filename; }

    bool init();
    void deinit();

    // JSONL: дописать одну строку (JSON-объект) + \r\n
    bool appendLine(const char* jsonLine);

    // Прочитать чанк из начала файла и собрать JSON-массив в out: [obj,obj,...]
    bool readChunkAsJsonArray(char* out,
                              uint32_t outSize,
                              uint32_t maxPayloadBytes,
                              uint32_t& linesRead,
                              FSIZE_t& bytesUsedFromFile);

    // Удалить из начала файла bytesUsedFromFile байт (отправленный префикс)
    bool consumePrefix(FSIZE_t bytesUsedFromFile);

    bool exists() const;
    bool remove();

    // Максимальный размер файла с бэкапами (ring-buffer по размеру)
    static constexpr FSIZE_t MAX_BACKUP_FILE_SIZE = 10 * 1024 * 1024; // 10 МБ

    /** @brief Returns full FATFS path to the backup file. */
    void getFilePath(char* out, size_t outSz) const {
        make_full_path(out, outSz, m_filename);
    }

private:
    bool  m_mounted  = false;
    bool  m_broken   = false;
    FATFS m_fatfs{};
    FIL   m_file{};

    // Текущее имя файла бэкапа; задаётся через setFilename() или дефолт из Config
    char m_filename[24] = {};   ///< Заполняется в конструкторе через setFilename

    void make_drive(char* out, size_t out_sz) const;
    void make_full_path(char* out, size_t out_sz, const char* fname) const;

    DWORD getFreeSpaceBytes() const;
    bool  checkAndRotateFile(FIL& f, const char* path);
};

#endif
