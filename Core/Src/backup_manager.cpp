// ============================================================================
// backup_manager — реализация. См. backup_manager.hpp и docs/PRODUCT_SPEC.md
// Только FatFS, без внешних JSON-зависимостей. Все буферы статические/на
// стеке — динамической памяти нет (важно для heap под mbedTLS).
// ============================================================================

#include "backup_manager.hpp"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern "C" {
#include "ff.h"   // FatFS (каталог FATFS/ в проекте)
}

namespace backup_mgr {

struct ParamState {
    ParamConfig cfg;
    uint16_t seg_read;    // курсор: сегмент, откуда читаем неотправленное
    uint32_t off_read;    // курсор: смещение внутри seg_read
    uint16_t seg_active;  // последний сегмент (куда пишем), 0 = нет ещё
    bool     used;
};

static ParamState  s_params[MAX_PARAMS];
static size_t      s_count = 0;
static const char* DIR = "0:/backup";

// ----------------------------------------------------------------- helpers

static ParamState* find(const char* id) {
    if (!id) return nullptr;
    for (size_t i = 0; i < s_count; ++i)
        if (s_params[i].used && strncmp(s_params[i].cfg.id, id, sizeof(s_params[i].cfg.id)) == 0)
            return &s_params[i];
    return nullptr;
}

static void seg_path(char* out, size_t out_sz, const char* id, uint16_t seg) {
    snprintf(out, out_sz, "%s/%s.%04u.jsonl", DIR, id, (unsigned)seg);
}

// Номер последнего существующего сегмента (0 — ни одного)
static uint16_t find_active_seg(const char* id) {
    char path[96];
    FILINFO fno;
    uint16_t hi = 0;
    for (uint16_t s = 1; s < 9999; ++s) {
        seg_path(path, sizeof(path), id, s);
        if (f_stat(path, &fno) != FR_OK) break;
        hi = s;
    }
    return hi;
}

static uint32_t seg_size(const char* id, uint16_t seg) {
    char path[96];
    FILINFO fno;
    seg_path(path, sizeof(path), id, seg);
    if (f_stat(path, &fno) != FR_OK) return 0;
    return (uint32_t)fno.fsize;
}

// ----------------------------------------------------------------- cursors

static bool save_cursors() {
    char path[96];
    snprintf(path, sizeof(path), "%s/cursors.json", DIR);
    FIL f;
    if (f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return false;
    bool ok = true;
    for (size_t i = 0; i < s_count && ok; ++i) {
        ParamState& ps = s_params[i];
        if (!ps.used) continue;
        char line[96];
        int n = snprintf(line, sizeof(line), "{\"id\":\"%s\",\"seg\":%u,\"off\":%lu}\n",
                         ps.cfg.id, (unsigned)ps.seg_read, (unsigned long)ps.off_read);
        UINT bw = 0;
        ok = (f_write(&f, line, (UINT)n, &bw) == FR_OK && bw == (UINT)n);
    }
    f_sync(&f);
    f_close(&f);
    return ok;
}

static void load_cursors() {
    char path[96];
    snprintf(path, sizeof(path), "%s/cursors.json", DIR);
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return;  // нет файла — курсоры по умолчанию
    static char buf[1024];                           // 16 параметров x ~45 байт
    UINT br = 0;
    f_read(&f, buf, sizeof(buf) - 1, &br);
    f_close(&f);
    buf[br] = 0;

    char* line = buf;
    while (line && *line) {
        char* eol = strchr(line, '\n');
        if (eol) *eol = 0;
        char* p = strstr(line, "\"id\":\"");
        if (p) {
            p += 6;
            char id[24]; size_t i = 0;
            while (*p && *p != '"' && i < sizeof(id) - 1) id[i++] = *p++;
            id[i] = 0;
            ParamState* ps = find(id);
            if (ps) {
                char* s = strstr(line, "\"seg\":");
                char* o = strstr(line, "\"off\":");
                if (s) ps->seg_read = (uint16_t)atoi(s + 6);
                if (o) ps->off_read = (uint32_t)strtoul(o + 6, nullptr, 10);
            }
        }
        line = eol ? eol + 1 : nullptr;
    }
}

// ------------------------------------------------------------------- public

bool init(const ParamConfig* params, size_t count) {
    if (!params || count == 0 || count > MAX_PARAMS) return false;
    memset(s_params, 0, sizeof(s_params));
    s_count = count;
    f_mkdir(DIR);
    for (size_t i = 0; i < count; ++i) {
        ParamState& ps = s_params[i];
        ps.used = true;
        ps.cfg  = params[i];
        if (ps.cfg.quota_bytes == 0) ps.cfg.quota_bytes = DEFAULT_QUOTA_BYTES;
        ps.seg_active = find_active_seg(ps.cfg.id);
        ps.seg_read   = ps.seg_active ? 1 : 0;  // по умолчанию — с начала архива
        ps.off_read   = 0;
    }
    load_cursors();
    return true;
}

bool append(const char* id, const char* iso_ts, float value, uint8_t quality) {
    ParamState* ps = find(id);
    if (!ps || !ps->cfg.backup_enabled) return false;
    if (ps->seg_active == 0) ps->seg_active = 1;

    char path[96];
    seg_path(path, sizeof(path), id, ps->seg_active);
    if (seg_size(id, ps->seg_active) >= SEGMENT_MAX_BYTES) {
        ps->seg_active++;
        seg_path(path, sizeof(path), id, ps->seg_active);
    }

    // Форматируем float вручную — без зависимости от printf-float в линкере
    long iv   = (long)value;
    long frac = (long)((value - (float)iv) * 1000.0f);
    if (frac < 0) frac = -frac;
    char line[96];
    int n = snprintf(line, sizeof(line), "{\"ts\":\"%s\",\"v\":%ld.%03lu,\"q\":%u}\n",
                     iso_ts ? iso_ts : "", iv, (unsigned long)frac, (unsigned)quality);

    FIL f;
    if (f_open(&f, path, FA_WRITE | FA_OPEN_APPEND) != FR_OK) return false;
    UINT bw = 0;
    FRESULT r = f_write(&f, line, (UINT)n, &bw);
    f_sync(&f);          // переживаем пропадание питания между опросами
    f_close(&f);
    return (r == FR_OK && bw == (UINT)n);
}

bool hasPending(const char* id) {
    ParamState* ps = find(id);
    if (!ps || ps->seg_read == 0) return false;
    if (ps->seg_read < ps->seg_active) return true;
    if (ps->seg_read == ps->seg_active)
        return ps->off_read < seg_size(id, ps->seg_read);
    return false;
}

uint16_t readPending(const char* id, char* out_buf, uint16_t out_max) {
    ParamState* ps = find(id);
    if (!ps || !out_buf || out_max < 64 || ps->seg_read == 0) return 0;

    char path[96];
    seg_path(path, sizeof(path), id, ps->seg_read);
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    if (f_lseek(&f, ps->off_read) != FR_OK) { f_close(&f); return 0; }
    UINT br = 0;
    FRESULT r = f_read(&f, out_buf, out_max, &br);
    f_close(&f);
    if (r != FR_OK || br == 0) return 0;

    uint16_t used = (uint16_t)br;
    while (used > 0 && out_buf[used - 1] != '\n') used--;  // только целые строки
    return used;
}

bool commitSent(const char* id, uint16_t bytes) {
    ParamState* ps = find(id);
    if (!ps || bytes == 0 || ps->seg_read == 0) return false;
    ps->off_read += bytes;

    // Сегмент дочитан и есть следующий → переходим на него
    if (ps->off_read >= seg_size(id, ps->seg_read) && ps->seg_read < ps->seg_active) {
        ps->seg_read++;
        ps->off_read = 0;
    }
    return save_cursors();
}

bool getStatus(const char* id, Status& st) {
    ParamState* ps = find(id);
    if (!ps) return false;
    memset(&st, 0, sizeof(st));
    st.active = ps->cfg.backup_enabled;
    for (uint16_t s = 1; s <= ps->seg_active; ++s) {
        uint32_t sz = seg_size(id, s);
        if (sz == 0 && s != ps->seg_active) continue;  // удалённый сегмент
        st.segments++;
        st.total_bytes += sz;
        if (s > ps->seg_read) st.pending_bytes += sz;
        else if (s == ps->seg_read && sz > ps->off_read) st.pending_bytes += sz - ps->off_read;
    }
    return true;
}

void enforceQuotas() {
    // 1) Квота на параметр: удаляем сегменты с начала, активный не трогаем
    for (size_t i = 0; i < s_count; ++i) {
        ParamState& ps = s_params[i];
        if (!ps.used || !ps.cfg.backup_enabled || ps.seg_active == 0) continue;

        uint32_t total = 0;
        for (uint16_t s = 1; s <= ps.seg_active; ++s) total += seg_size(ps.cfg.id, s);

        uint16_t oldest = 1;
        while (total > ps.cfg.quota_bytes && oldest < ps.seg_active) {
            char path[96];
            FILINFO fno;
            seg_path(path, sizeof(path), ps.cfg.id, oldest);
            if (f_stat(path, &fno) != FR_OK) { oldest++; continue; }  // уже удалён
            if (f_unlink(path) != FR_OK) break;
            total -= (uint32_t)fno.fsize;
            if (ps.seg_read == oldest) { ps.seg_read = oldest + 1; ps.off_read = 0; save_cursors(); }
            oldest++;
        }
    }

    // 2) Глобальный лимит свободного места на SD
    FATFS* fs; DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) != FR_OK) return;
    uint64_t free_bytes = (uint64_t)fre_clust * (uint64_t)fs->csize * 512ull;  // сектор 512

    for (int guard = 0; guard < 64 && free_bytes < SD_MIN_FREE_BYTES; ++guard) {
        // Жертва: самый старый сегмент; предпочитаем полностью отправленные
        ParamState* victim = nullptr;
        uint16_t victim_seg = 0;
        bool victim_sent = false;
        for (size_t i = 0; i < s_count; ++i) {
            ParamState& ps = s_params[i];
            if (!ps.used || ps.seg_active < 2) continue;
            for (uint16_t s = 1; s < ps.seg_active; ++s) {
                char path[96];
                FILINFO fno;
                seg_path(path, sizeof(path), ps.cfg.id, s);
                if (f_stat(path, &fno) != FR_OK) continue;
                bool sent = (s < ps.seg_read);
                if (!victim || (sent && !victim_sent)) {
                    victim = &ps; victim_seg = s; victim_sent = sent;
                }
                break;  // первый существующий сегмент параметра = самый старый
            }
            if (victim_sent) break;
        }
        if (!victim) break;

        char path[96];
        seg_path(path, sizeof(path), victim->cfg.id, victim_seg);
        uint32_t sz = seg_size(victim->cfg.id, victim_seg);
        if (f_unlink(path) != FR_OK) break;
        free_bytes += sz;
        if (victim->seg_read == victim_seg) { victim->seg_read = victim_seg + 1; victim->off_read = 0; }
        save_cursors();
    }
}

} // namespace backup_mgr
