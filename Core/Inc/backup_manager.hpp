#pragma once
// ============================================================================
// backup_manager — попараметричный сегментный бэкап с курсорами отправки
// Этап 1 спецификации MonSet-PLC (docs/PRODUCT_SPEC.md, раздел 5)
//
// Хранение:  0:/backup/<id>.NNNN.jsonl  — сегменты по SEGMENT_MAX_BYTES
// Курсоры:   0:/backup/cursors.json     — позиция отправки на каждый параметр
//
// ВАЖНО: вызывающий код обязан убедиться, что SD смонтирована и доступна
// (после IWDG-сброса SD отключается — проверять состояние снаружи).
// Модуль сам JSON-конфиг не парсит: ParamConfig заполняет runtime_config.
// ============================================================================

#include <stdint.h>
#include <stddef.h>

namespace backup_mgr {

constexpr size_t   MAX_PARAMS          = 16;
constexpr uint32_t SEGMENT_MAX_BYTES   = 256u * 1024u;        // 256 KB сегмент
constexpr uint32_t DEFAULT_QUOTA_BYTES = 2048u * 1024u;       // 2 MB на параметр
constexpr uint32_t SD_MIN_FREE_BYTES   = 32u * 1024u * 1024u; // держим 32 MB свободными

struct ParamConfig {
    char     id[24];          // уникальный id параметра, идёт в имя файла ("level_1")
    char     metric[32];      // имя метрики для сервера ("water_level")
    bool     backup_enabled;  // галочка «создавать бэкап» в веб-интерфейсе
    uint32_t quota_bytes;     // 0 → DEFAULT_QUOTA_BYTES
};

struct Status {
    uint32_t total_bytes;     // суммарный размер всех сегментов параметра
    uint32_t pending_bytes;   // неотправленные байты (от курсора до конца)
    uint16_t segments;        // всего сегментов
    bool     active;          // параметр зарегистрирован и backup_enabled
};

// Регистрация параметров и загрузка курсоров. Вызвать один раз при старте
// после mount SD. Повторный вызов перечитывает конфиг и курсоры.
bool init(const ParamConfig* params, size_t count);

// Добавить измерение в архив параметра. iso_ts — "YYYY-MM-DDTHH:MM:SSZ".
// quality: 0=good, 1=sensor fail, 2=comm fail, 3=обрыв 4-20мА, 4=расчётное.
bool append(const char* id, const char* iso_ts, float value, uint8_t quality);

// Есть ли неотправленные данные по параметру.
bool hasPending(const char* id);

// Прочитать следующую порцию неотправленных данных (только целые строки JSONL).
// Возвращает число байт (0 — нет данных/ошибка). out_max должен превышать
// длину одной строки (строка ~60 байт; рекомендуется буфер >= 512).
uint16_t readPending(const char* id, char* out_buf, uint16_t out_max);

// Подтвердить отправку bytes байт, прочитанных readPending: сдвинуть курсор
// и сохранить cursors.json на SD. Вызывать ТОЛЬКО после успешной отправки.
bool commitSent(const char* id, uint16_t bytes);

// Статистика параметра для веб-API (Dashboard: записано/отправлено).
bool getStatus(const char* id, Status& st);

// Кольцевая перезапись: удаление старых сегментов при переполнении квоты
// параметра или нехватке места на SD. Сначала удаляются полностью
// отправленные сегменты. Вызывать после цикла append (не чаще раза в минуту).
void enforceQuotas();

} // namespace backup_mgr
