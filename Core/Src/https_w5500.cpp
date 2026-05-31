#include "https_w5500.hpp"
#include "debug_uart.hpp"
#include <cstring>
#include <cstdio>
#include <cctype>

extern "C" {
#include "socket.h"
#include "dns.h"
#include "w5500.h"
#include "wizchip_conf.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_ciphersuites.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/error.h"
#include "stm32f4xx_hal.h"
// PSA УБРАН — не нужен без TLS1.3
extern volatile uint32_t dns_1s_tick; ///< incremented by DNS_time_handler() in TIM6 ISR
}

volatile bool g_web_exclusive = false;

#define IWDG_FEED()  do { IWDG->KR = 0xAAAAu; } while(0)

#define TLS_STEP(n, msg)  do { \
    DBG.info("TLS_STEP %d: " msg, (int)(n)); \
    HAL_Delay(5); \
    IWDG_FEED(); \
} while(0)

static const char* s_caPem = nullptr;
void HttpsW5500::setCaPem(const char* caPem) { s_caPem = caPem; }

// ================================================================
// Все TLS-структуры static — в .bss, не на стеке.
// PSA ПОЛНОСТЬЮ УБРАН: ssl_setup теперь использует классический
// mbedtls path без ECC/PSA таблиц -> работает мгновенно.
// ================================================================
static mbedtls_ssl_context       s_ssl;
static mbedtls_ssl_config        s_conf;
static mbedtls_entropy_context   s_entropy;
static mbedtls_ctr_drbg_context  s_ctr;

static bool s_tls_ready = false;

static char s_hdr[512];
static char s_rx[512];

struct TlsSockCtx { uint8_t sn; uint32_t deadline; };
static TlsSockCtx s_bio;

static void logMbedtlsErr(const char* tag, int rc) {
    char buf[128]{};
    mbedtls_strerror(rc, buf, sizeof(buf));
    DBG.error("%s rc=-0x%04X (%s)", tag, (unsigned)(-rc), buf[0] ? buf : "?");
}

static bool resolveHost(const char* host, uint8_t outIp[4]) {
    // Быстрая проверка — может это уже IP-адрес
    bool isNum = true;
    for (const char* p = host; *p; ++p)
        if (!std::isdigit((unsigned char)*p) && *p != '.') { isNum = false; break; }
    if (isNum) {
        uint32_t a=0,b=0,c=0,d=0;
        if (std::sscanf(host, "%lu.%lu.%lu.%lu",&a,&b,&c,&d) != 4) return false;
        outIp[0]=(uint8_t)a; outIp[1]=(uint8_t)b;
        outIp[2]=(uint8_t)c; outIp[3]=(uint8_t)d;
        return true;
    }

    static uint8_t dnsBuf[512];
    wiz_NetInfo ni{}; wizchip_getnetinfo(&ni);
    DBG.info("DNS: resolving [%s] via %u.%u.%u.%u",
             host, ni.dns[0],ni.dns[1],ni.dns[2],ni.dns[3]);

    /* fix: ATTEMPT_GUARD_MS снижен 25000→8000ms.
     * После fix DNS_time_handler (1мс*1000=1с), DNS_run max = MAX_DNS_RETRY(2)*DNS_WAIT_TIME(3)*1с = 6с.
     * 3 попытки * 8с = 24с < IWDG timeout(32с) — безопасно.
     * IWDG_FEED() между попытками (в начале цикла) не даёт WDG сработать. */
    static const uint8_t  MAX_ATTEMPTS = 4;
    static const uint32_t ATTEMPT_GUARD_MS = 8000UL;
    /* Fallback DNS servers: attempt 0,1 → DHCP DNS; 2 → 8.8.8.8; 3 → 1.1.1.1 */
    static const uint8_t FALLBACK_DNS[2][4] = {{8,8,8,8},{1,1,1,1}};

    for (uint8_t attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        IWDG_FEED();
        dns_1s_tick = 0;  // сброс таймера перед запросом
        close(1);         // fix: освобождаем сокет перед DNS_init
        HAL_Delay(5);
        DNS_init(1, dnsBuf);

        /* выбираем DNS сервер */
        uint8_t* dnsToUse = ni.dns;
        if (attempt == 2) dnsToUse = (uint8_t*)FALLBACK_DNS[0]; /* 8.8.8.8 */
        if (attempt == 3) dnsToUse = (uint8_t*)FALLBACK_DNS[1]; /* 1.1.1.1 */
        if (attempt >= 2) {
            DBG.info("DNS: fallback to %u.%u.%u.%u",
                     dnsToUse[0],dnsToUse[1],dnsToUse[2],dnsToUse[3]);
        }

        uint8_t ip[4]{};
        uint32_t t0 = HAL_GetTick();
        int8_t r = DNS_run(dnsToUse, (uint8_t*)host, ip);
        uint32_t elapsed = HAL_GetTick() - t0;
        IWDG_FEED();

        DBG.info("DNS: attempt %d/%d r=%d elapsed=%lums ip=%u.%u.%u.%u",
                 (int)attempt+1, (int)MAX_ATTEMPTS, (int)r,
                 (unsigned long)elapsed, ip[0],ip[1],ip[2],ip[3]);

        if (r == 1 && (ip[0]||ip[1]||ip[2]||ip[3])) {
            std::memcpy(outIp, ip, 4);
            DBG.info("DNS: OK %u.%u.%u.%u", ip[0],ip[1],ip[2],ip[3]);
            close(1); HAL_Delay(20); // освобождаем UDP сокет перед TCP
            return true;
        }
        if (r == 1) {
            // r=1 но ip пустой — ioLibrary баг, retry
            DBG.warn("DNS: r=1 but ip empty, retry");
            close(1); HAL_Delay(100);
            continue;
        }
        // r==0 (pending/no answer) или r<0 (error): проверяем guard только здесь
        if (r == 0 && elapsed >= ATTEMPT_GUARD_MS) {
            DBG.warn("DNS: attempt %d no answer (%lums), trying next server",
                     (int)attempt + 1, (unsigned long)elapsed);
            close(1);
            continue;  /* не abort — переходим к fallback 8.8.8.8 / 1.1.1.1 */
        }
        if (r < 0) {
            DBG.error("DNS: DNS_run error r=%d", (int)r);
            close(1); break;
        }
        if (attempt + 1 < MAX_ATTEMPTS) HAL_Delay(200);
    }
    DBG.error("DNS: failed to resolve [%s]", host);
    close(1);
    return false;
}

static int w5500_send_cb(void* ctx, const unsigned char* buf, size_t len) {
    auto* c = (TlsSockCtx*)ctx;
    if (len == 0) return 0;
    uint16_t fsr = getSn_TX_FSR(c->sn);
    if (fsr == 0) return MBEDTLS_ERR_SSL_WANT_WRITE;
    uint16_t toSend = (len > fsr) ? fsr : (uint16_t)len;
    int32_t r = send(c->sn, (uint8_t*)buf, toSend);
    if (r < 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    if (r == 0) return MBEDTLS_ERR_SSL_WANT_WRITE;
    return (int)r;
}

static int w5500_recv_cb(void* ctx, unsigned char* buf, size_t len) {
    auto* c = (TlsSockCtx*)ctx;
    if (len == 0) return 0;
    uint16_t rsr = getSn_RX_RSR(c->sn);
    if (rsr == 0) {
        if (HAL_GetTick() > c->deadline) return MBEDTLS_ERR_SSL_TIMEOUT;
        IWDG_FEED();
        return MBEDTLS_ERR_SSL_WANT_READ;
    }
    uint16_t toRead = (len > rsr) ? rsr : (uint16_t)len;
    int32_t r = recv(c->sn, (uint8_t*)buf, toRead);
    if (r < 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    return (int)r;
}

bool HttpsW5500::parseHttpsUrl(const char* url, UrlParts& out) {
    if (!url) return false;
    std::memset(&out, 0, sizeof(out));
    out.port = 443;
    const char* p = url;
    if (std::strncmp(p, "https://", 8) != 0) return false;
    p += 8;
    const char* hb = p;
    while (*p && *p != '/' && *p != ':') p++;
    size_t hl = (size_t)(p - hb);
    if (hl == 0 || hl >= sizeof(out.host)) return false;
    std::memcpy(out.host, hb, hl); out.host[hl] = 0;
    if (*p == ':') {
        p++; uint32_t port = 0;
        while (*p && std::isdigit((unsigned char)*p))
            { port = port * 10u + (uint32_t)(*p - '0'); p++; }
        if (port == 0 || port > 65535) return false;
        out.port = (uint16_t)port;
    }
    if (*p == 0) { std::strcpy(out.path, "/"); return true; }
    if (*p != '/') return false;
    size_t pl = std::strlen(p);
    if (pl == 0 || pl >= sizeof(out.path)) return false;
    std::memcpy(out.path, p, pl + 1);
    return true;
}

// ================================================================
// Однократная инициализация TLS.
// БЕЗ PSA: только entropy + ctr_drbg + ssl_config + ssl_setup.
// ssl_setup без PSA/ECDHE выполняется за ~1мс.
// ================================================================
static bool tlsOneTimeInit() {
    IWDG_FEED();

    DBG.info("TLS_INIT: entropy + ctr_drbg_init");
    mbedtls_entropy_init(&s_entropy);
    mbedtls_ctr_drbg_init(&s_ctr);
    IWDG_FEED();

    DBG.info("TLS_INIT: ctr_drbg_seed START");
    const char* pers = "w5500tls";
    int rc = mbedtls_ctr_drbg_seed(&s_ctr, mbedtls_entropy_func, &s_entropy,
                                    (const unsigned char*)pers,
                                    std::strlen(pers));
    IWDG_FEED();
    if (rc != 0) { logMbedtlsErr("TLS_INIT: seed", rc); return false; }
    DBG.info("TLS_INIT: ctr_drbg_seed OK");
    IWDG_FEED();

    DBG.info("TLS_INIT: ssl_config_defaults");
    mbedtls_ssl_config_init(&s_conf);
    rc = mbedtls_ssl_config_defaults(&s_conf,
                                      MBEDTLS_SSL_IS_CLIENT,
                                      MBEDTLS_SSL_TRANSPORT_STREAM,
                                      MBEDTLS_SSL_PRESET_DEFAULT);
    IWDG_FEED();
    if (rc != 0) { logMbedtlsErr("TLS_INIT: config_defaults", rc); return false; }

    // Явно ограничиваем TLS 1.2
    mbedtls_ssl_conf_min_tls_version(&s_conf, MBEDTLS_SSL_VERSION_TLS1_2);
    mbedtls_ssl_conf_max_tls_version(&s_conf, MBEDTLS_SSL_VERSION_TLS1_2);

    mbedtls_ssl_conf_rng(&s_conf, mbedtls_ctr_drbg_random, &s_ctr);
    mbedtls_ssl_conf_authmode(&s_conf, MBEDTLS_SSL_VERIFY_NONE);

    // Явный список cipher suites — только RSA/AES без PSA/ECC зависимостей
    // Убирает HardFault в ssl_setup при широком mbedtls_config.h
    static const int s_ciphersuites[] = {
        MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA,
        MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA,
        MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA256,
        MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA256,
        MBEDTLS_TLS_RSA_WITH_AES_128_GCM_SHA256,
        MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384,
        0
    };
    mbedtls_ssl_conf_ciphersuites(&s_conf, s_ciphersuites);
    IWDG_FEED();

    DBG.info("TLS_INIT: ssl_init");
    mbedtls_ssl_init(&s_ssl);
    IWDG_FEED();

    // Без PSA ssl_setup — просто calloc(4096) + calloc(4096) + memset
    DBG.info("TLS_INIT: ssl_setup START");
    rc = mbedtls_ssl_setup(&s_ssl, &s_conf);
    IWDG_FEED();
    if (rc != 0) { logMbedtlsErr("TLS_INIT: ssl_setup", rc); return false; }
    DBG.info("TLS_INIT: ssl_setup DONE");
    IWDG_FEED();

    s_tls_ready = true;
    return true;
}

// C-wrapper для вызова из main() до старта IWDG
extern "C" bool HttpsW5500_tlsOneTimeInit(void) {
    if (s_tls_ready) return true;
    return tlsOneTimeInit();
}

int HttpsW5500::postJson(const char* httpsUrl, const char* authB64,
                         const char* json, uint16_t jsonLen, uint32_t timeoutMs)
{
    if (g_web_exclusive) { DBG.warn("HTTPS: skip (web)"); return -99; }

    UrlParts u{};

    TLS_STEP(1, "parse URL");
    if (!parseHttpsUrl(httpsUrl, u)) {
        DBG.error("HTTPS: bad URL"); return -10;
    }

    TLS_STEP(2, "DNS resolve");
    static uint8_t s_ip[4];
    if (!resolveHost(u.host, s_ip)) {
        DBG.error("HTTPS: DNS fail"); return -11;
    }
    IWDG_FEED();

    TLS_STEP(3, "TCP socket+connect");
    DBG.info("HTTPS: connect %s:%u", u.host, (unsigned)u.port);
    DBG.info("HTTPS: ip %u.%u.%u.%u", s_ip[0],s_ip[1],s_ip[2],s_ip[3]);
    const uint8_t sn = 1;  // sn1=HTTPS/TLS 2KB TX/RX (wizchip_init карта)
    if (socket(sn, Sn_MR_TCP, 50001, 0) != sn) {
        DBG.error("HTTPS: socket() fail"); close(sn); return -20;
    }
    DBG.info("HTTPS: after socket: Sn_SR=0x%02X Sn_IR=0x%02X",
             getSn_SR(sn), getSn_IR(sn));
    // connect() ioLibrary блокирующий — добавляем watchdog и таймаут
    connect(sn, s_ip, u.port); // инициирует SYN
    {
        uint32_t t0 = HAL_GetTick();
        uint8_t sr;
        while (true) {
            IWDG_FEED();
            sr = getSn_SR(sn);
            if (sr == SOCK_ESTABLISHED) break;
            if (sr == SOCK_CLOSED || sr == SOCK_CLOSE_WAIT) {
                DBG.error("HTTPS: connect fail SR=0x%02X", sr);
                close(sn); return -21;
            }
            if ((HAL_GetTick() - t0) > 8000) {
                DBG.error("HTTPS: connect timeout SR=0x%02X", sr);
                disconnect(sn); close(sn); return -22;
            }
            HAL_Delay(5);
        }
    }
    DBG.info("HTTPS: connected: Sn_SR=0x%02X Sn_IR=0x%02X",
             getSn_SR(sn), getSn_IR(sn));
    IWDG_FEED();

    if (g_web_exclusive) {
        DBG.warn("HTTPS: abort after connect"); disconnect(sn); close(sn); return -99;
    }

    if (!s_tls_ready) {
        TLS_STEP(4, "TLS one-time init START");
        if (!tlsOneTimeInit()) {
            DBG.error("TLS: init failed");
            disconnect(sn); close(sn); return -30;
        }
        DBG.info("TLS_STEP 5: TLS one-time init DONE");
        IWDG_FEED();
    } else {
        TLS_STEP(4, "ssl_session_reset (reuse context)");
        IWDG_FEED();
        int sres = mbedtls_ssl_session_reset(&s_ssl);
        IWDG_FEED();
        if (sres != 0) {
            logMbedtlsErr("TLS: session_reset", sres);
            s_tls_ready = false;
            disconnect(sn); close(sn); return -31;
        }
        DBG.info("TLS: session_reset OK");
    }

    TLS_STEP(6, "ssl_set_hostname + set_bio");
    (void)mbedtls_ssl_set_hostname(&s_ssl, u.host);
    s_bio.sn       = sn;
    s_bio.deadline = HAL_GetTick() + timeoutMs;
    mbedtls_ssl_set_bio(&s_ssl, &s_bio, w5500_send_cb, w5500_recv_cb, nullptr);
    IWDG_FEED();

    TLS_STEP(7, "handshake loop START");
    int rc = 0;
    while (true) {
        if (g_web_exclusive) {
            DBG.warn("HTTPS: handshake abort (web)"); rc = MBEDTLS_ERR_SSL_TIMEOUT; break;
        }
        IWDG_FEED();
        rc = mbedtls_ssl_handshake(&s_ssl);
        if (rc == 0) { DBG.info("TLS: handshake OK"); break; }
        if (rc == MBEDTLS_ERR_SSL_WANT_READ || rc == MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (HAL_GetTick() > s_bio.deadline) {
                DBG.warn("TLS: handshake timeout"); rc = MBEDTLS_ERR_SSL_TIMEOUT; break;
            }
            IWDG_FEED();
            HAL_Delay(1);
            continue;
        }
        logMbedtlsErr("TLS: handshake error", rc);
        break;
    }
    if (rc != 0) { disconnect(sn); close(sn); return -40; }

    TLS_STEP(8, "HTTP write");
    int hdrLen = std::snprintf(s_hdr, sizeof(s_hdr),
        "POST %s HTTP/1.1\r\nHost: %s\r\n"
        "Authorization: Basic %s\r\n"
        "Content-Type: application/json\r\nContent-Length: %u\r\n"
        "Connection: close\r\n\r\n",
        u.path, u.host,
        authB64 ? authB64 : "", (unsigned)jsonLen);

    auto sslWriteAll = [&](const uint8_t* p, uint32_t n) -> int {
        uint32_t off = 0;
        while (off < n) {
            if (g_web_exclusive) return -1;
            IWDG_FEED();
            int w = mbedtls_ssl_write(&s_ssl, p + off, (size_t)(n - off));
            if (w > 0) { off += (uint32_t)w; continue; }
            if (w == MBEDTLS_ERR_SSL_WANT_READ || w == MBEDTLS_ERR_SSL_WANT_WRITE) {
                IWDG_FEED(); HAL_Delay(1); continue;
            }
            logMbedtlsErr("TLS: write", w); return -1;
        }
        return 0;
    };
    if (hdrLen > 0 && sslWriteAll((const uint8_t*)s_hdr,  (uint32_t)hdrLen) != 0) return -51;
    if (             sslWriteAll((const uint8_t*)json,     (uint32_t)jsonLen) != 0) return -52;

    TLS_STEP(9, "HTTP read");
    int used = 0; int httpCode = -1;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeoutMs) {
        if (g_web_exclusive) { DBG.warn("HTTPS: rx abort (web)"); break; }
        IWDG_FEED();
        int r = mbedtls_ssl_read(&s_ssl,
                                  (unsigned char*)s_rx + used,
                                  sizeof(s_rx) - 1 - (size_t)used);
        if (r > 0) {
            used += r; s_rx[used] = 0;
            const char* p2 = std::strstr(s_rx, "HTTP/1.1 ");
            if (!p2) p2 = std::strstr(s_rx, "HTTP/1.0 ");
            if (p2) {
                int code = 0;
                if (std::sscanf(p2, "HTTP/%*s %d", &code) == 1 && code > 0) {
                    httpCode = code; break;
                }
            }
            if (used >= (int)(sizeof(s_rx) - 1)) break;
            continue;
        }
        if (r == 0 || r == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) break;
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
            IWDG_FEED(); HAL_Delay(2); continue;
        }
        logMbedtlsErr("TLS: read", r); break;
    }
    IWDG_FEED();

    (void)mbedtls_ssl_close_notify(&s_ssl);
    disconnect(sn); close(sn);

    if (httpCode > 0) DBG.info("HTTPS: done, code=%d", httpCode);
    else              DBG.warn("HTTPS: no code (rx=%d)", used);
    return httpCode;
}
