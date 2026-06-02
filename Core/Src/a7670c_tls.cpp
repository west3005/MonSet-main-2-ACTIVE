/**
 * ================================================================
 * @file    a7670c_tls.cpp
 * @brief   TLS-туннель поверх A7670C через AT+CIPOPEN/CIPSEND/CIPRXGET.
 *
 * Транспортный слой:
 *   mbedTLS → biosend/biorecv → AT+CIPSEND/AT+CIPRXGET → A7670C UART
 *
 * AT-команды (A76XX Series AT Command Manual):
 *   Буферизация : AT+CIPRXGET=1          (один раз перед CIPOPEN)
 *   Открытие    : AT+CIPOPEN=<id>,"TCP","<host>",<port>
 *   Отправка    : AT+CIPSEND=<id>,<len>  → prompt ">" → данные → OK
 *   Приём       : AT+CIPRXGET=2,<id>,<len> → +CIPRXGET:2,<id>,<actual>,<pending>\r\n<data>
 *   Закрытие    : AT+CIPCLOSE=<id>
 *
 * Отличие от CSOC-подхода:
 *   AT+CSOC требует AT+NETOPEN + отдельного IP-стека.
 *   AT+CIPOPEN работает поверх PDP-контекста (CGACT) напрямую — надёжнее.
 * ================================================================
 */
#include "a7670c_tls.hpp"
#include "debug_uart.hpp"
#include "runtime_config.hpp"

extern "C" {
#include "mbedtls/net_sockets.h"
#include "mbedtls/error.h"
}

#include <cstdio>
#include <cstring>
#include <cctype>

// ============================================================================
// Константы
// ============================================================================
static constexpr uint16_t MODEM_MAX_SEND = 512;
static constexpr uint16_t MODEM_MAX_RECV = 512;
static constexpr uint8_t  CIP_LINK       = 0;    ///< A7670C: первый доступный link

// ============================================================================
// Конструктор / Деструктор
// ============================================================================
A7670CTls::A7670CTls(A7670C& modem, uint32_t timeoutMs)
    : m_modem(modem), m_timeoutMs(timeoutMs)
{
    mbedtls_ssl_init(&m_ssl);
    mbedtls_ssl_config_init(&m_conf);
    mbedtls_entropy_init(&m_entropy);
    mbedtls_ctr_drbg_init(&m_ctr);
    mbedtls_x509_crt_init(&m_cacert);
    m_tlsInited = true;
}

A7670CTls::~A7670CTls()
{
    close();
    tlsFree();
}

// ============================================================================
// Вспомогательные
// ============================================================================
void A7670CTls::logMbedtlsErr(const char* tag, int rc)
{
    char buf[128];
    std::memset(buf, 0, sizeof(buf));
    mbedtls_strerror(rc, buf, sizeof(buf));
    DBG.error("%s rc=-0x%04X (%s)", tag, (unsigned)(-rc), buf[0] ? buf : "?");
}

void A7670CTls::tlsFree()
{
    if (!m_tlsInited) return;
    mbedtls_ssl_free(&m_ssl);
    mbedtls_ssl_config_free(&m_conf);
    mbedtls_x509_crt_free(&m_cacert);
    mbedtls_ctr_drbg_free(&m_ctr);
    mbedtls_entropy_free(&m_entropy);
    mbedtls_ssl_init(&m_ssl);
    mbedtls_ssl_config_init(&m_conf);
    mbedtls_entropy_init(&m_entropy);
    mbedtls_ctr_drbg_init(&m_ctr);
    mbedtls_x509_crt_init(&m_cacert);
}

bool A7670CTls::parseHttpsUrl(const char* url, UrlParts& out)
{
    if (!url) return false;
    out = UrlParts{};
    out.port = 443;

    const char* prefix = "https://";
    size_t prefixLen = std::strlen(prefix);
    if (std::strncmp(url, prefix, prefixLen) != 0) return false;

    const char* p = url + prefixLen;
    const char* hb = p;
    while (*p && *p != '/' && *p != ':') p++;
    size_t hl = (size_t)(p - hb);
    if (hl == 0 || hl >= sizeof(out.host)) return false;
    std::memcpy(out.host, hb, hl);
    out.host[hl] = '\0';

    if (*p == ':') {
        p++;
        uint32_t port = 0;
        while (*p && std::isdigit((unsigned char)*p))
            port = port * 10u + (uint32_t)(*p++ - '0');
        if (port == 0 || port > 65535) return false;
        out.port = (uint16_t)port;
    }

    if (*p == '\0') { std::strcpy(out.path, "/"); return true; }
    if (*p != '/')   return false;
    size_t pl = std::strlen(p);
    if (pl == 0 || pl >= sizeof(out.path)) return false;
    std::memcpy(out.path, p, pl + 1);
    return true;
}

// ============================================================================
// BIO: отправка через AT+CIPSEND=<id>,<len>
// Prompt ">", затем бинарные данные, затем OK
// ============================================================================
int A7670CTls::modemWriteRaw(const uint8_t* buf, uint16_t len)
{
    if (!buf || len == 0) return 0;

    uint16_t sent = 0;
    char cmd[64], r[128];

    while (sent < len) {
        uint16_t chunk = len - sent;
        if (chunk > MODEM_MAX_SEND) chunk = MODEM_MAX_SEND;

        std::snprintf(cmd, sizeof(cmd),
                      "AT+CIPSEND=%hhu,%u\r\n", m_sockId, (unsigned)chunk);
        m_modem.flushRx_pub();
        m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));

        m_modem.waitFor_pub(r, sizeof(r), ">", 3000);
        if (!std::strstr(r, ">")) {
            DBG.error("TLS BIO: нет prompt \'>\'  для CIPSEND (chunk=%u)", (unsigned)chunk);
            return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
        }

        m_modem.sendRaw_pub(reinterpret_cast<const char*>(buf + sent), chunk);

        // A7670C отвечает "OK" (не "SEND OK" как Air780E)
        m_modem.waitFor_pub(r, sizeof(r), "OK", 5000);
        if (!std::strstr(r, "OK") && !std::strstr(r, "SEND OK")) {
            DBG.warn("TLS BIO: CIPSEND нет OK (chunk=%u)", (unsigned)chunk);
        }

        sent += chunk;
        IWDG->KR = 0xAAAA;
    }
    return (int)sent;
}

// ============================================================================
// BIO: приём через AT+CIPRXGET=2,<id>,<len>
// Ответ: +CIPRXGET: 2,<id>,<actual>,<pending>\r\n<data>\r\nOK
// ============================================================================
int A7670CTls::modemReadRaw(uint8_t* buf, uint16_t len)
{
    if (!buf || len == 0) return 0;

    uint16_t want = (len > MODEM_MAX_RECV) ? MODEM_MAX_RECV : len;
    char cmd[64];
    char r[MODEM_MAX_RECV + 128];

    std::snprintf(cmd, sizeof(cmd),
                  "AT+CIPRXGET=2,%hhu,%u\r\n", m_sockId, (unsigned)want);
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));

    uint16_t rxLen = m_modem.waitFor_pub(r, sizeof(r), "+CIPRXGET:", 3000);

    if (!std::strstr(r, "+CIPRXGET:")) {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }

    // Парсим: +CIPRXGET: 2,<id>,<actual>,<pending>
    const char* p = std::strstr(r, "+CIPRXGET:");
    int mode = 0, id = 0, actual = 0, pending = 0;
    if (std::sscanf(p, "+CIPRXGET: %d,%d,%d,%d",
                    &mode, &id, &actual, &pending) < 3 || actual <= 0) {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }

    const char* dataStart = std::strchr(p, '\n');
    if (!dataStart) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    dataStart++;

    uint16_t copy = (uint16_t)actual;
    if (copy > len) copy = (uint16_t)len;
    size_t available = rxLen - (size_t)(dataStart - r);
    if (available < copy) copy = (uint16_t)available;

    std::memcpy(buf, dataStart, copy);
    IWDG->KR = 0xAAAA;
    return (int)copy;
}

// ============================================================================
// mbedTLS BIO callbacks
// ============================================================================
int A7670CTls::biosend(void* ctx, const unsigned char* buf, size_t len)
{
    auto* self = static_cast<A7670CTls*>(ctx);
    if (!self || !buf || len == 0) return 0;
    if (HAL_GetTick() > self->m_deadline) return MBEDTLS_ERR_SSL_TIMEOUT;
    uint16_t toSend = (len > MODEM_MAX_SEND) ? MODEM_MAX_SEND : (uint16_t)len;
    int r = self->modemWriteRaw(buf, toSend);
    if (r <= 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    return r;
}

int A7670CTls::biorecv(void* ctx, unsigned char* buf, size_t len)
{
    auto* self = static_cast<A7670CTls*>(ctx);
    if (!self || !buf || len == 0) return 0;
    if (HAL_GetTick() > self->m_deadline) return MBEDTLS_ERR_SSL_TIMEOUT;
    uint16_t toRead = (len > MODEM_MAX_RECV) ? MODEM_MAX_RECV : (uint16_t)len;
    int r = self->modemReadRaw(buf, toRead);
    if (r == MBEDTLS_ERR_SSL_WANT_READ) return r;
    if (r < 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    if (r == 0) return MBEDTLS_ERR_SSL_WANT_READ;
    return r;
}

// ============================================================================
// connect(): TCP open (CIPOPEN) + режим буферизации + TLS handshake
// ============================================================================
int A7670CTls::connect(const char* host, uint16_t port)
{
    DBG.info("TLS: connect %s:%u (via CIPOPEN)", host, port);
    m_deadline = HAL_GetTick() + m_timeoutMs;
    m_sockId   = CIP_LINK;

    char r[256], cmd[128];

    // Закрыть предыдущий линк (может не существовать — ERROR допустим)
    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%hhu\r\n", m_sockId);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);
    HAL_Delay(200);

    // DNS-резолв через модем (AT+CDNSGIP) — некоторые прошивки A7670C
    // не резолвят hostname внутри CIPOPEN, требуют IP напрямую
    char connectAddr[64];
    std::strncpy(connectAddr, host, sizeof(connectAddr) - 1);
    connectAddr[sizeof(connectAddr) - 1] = '\0';

    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd), "AT+CDNSGIP=\"%s\"\r\n", host);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    DBG.info("TLS: DNS resolving %s...", host);
    // Формат A7670C: +CDNSGIP: 1,<count>,"host","ip1","ip2",...
    // Успех = "+CDNSGIP: 1", ошибка = "+CDNSGIP: 0,<errcode>"
    uint16_t dnsRxLen = m_modem.waitFor_pub(r, sizeof(r), "+CDNSGIP:", 10000);
    (void)dnsRxLen;
    if (std::strstr(r, "+CDNSGIP: 1")) {
        // Пропускаем три запятые: "1,<count>,"host","ip"
        const char* p1 = std::strstr(r, "+CDNSGIP: 1");
        const char* c1 = std::strchr(p1, ',');           // после "1"
        if (c1) c1 = std::strchr(c1 + 1, ',');          // после count
        if (c1) c1 = std::strchr(c1 + 1, ',');          // после "host"
        if (c1) {
            c1++;
            while (*c1 == ' ' || *c1 == '"') c1++;
            size_t ipLen = 0;
            while (c1[ipLen] && c1[ipLen] != '"' && c1[ipLen] != '\r' &&
                   c1[ipLen] != '\n' && ipLen < sizeof(connectAddr) - 1) {
                connectAddr[ipLen] = c1[ipLen];
                ipLen++;
            }
            connectAddr[ipLen] = '\0';
        }
        if (connectAddr[0] && std::strcmp(connectAddr, host) != 0)
            DBG.info("TLS: DNS OK %s -> %s", host, connectAddr);
        else
            DBG.warn("TLS: DNS parse fail, using hostname");
    } else {
        DBG.warn("TLS: DNS fail [%.40s], using hostname", r);
    }

    // AT+CIPOPEN=<id>,"TCP","<ip_or_host>",<port>
    // Модем сначала отвечает "OK", затем асинхронно "+CIPOPEN: <id>,<err>"
    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd),
                  "AT+CIPOPEN=%u,\"TCP\",\"%s\",%u\r\n",
                  (unsigned)m_sockId, connectAddr, (unsigned)port);
    DBG.info("TLS: CIPOPEN to %s:%u", connectAddr, (unsigned)port);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));

    // Ждём +CIPOPEN: — ответ асинхронный, между OK и +CIPOPEN: может быть пауза >50мс.
    // waitFor прерывается по 50мс тишины и не захватывает +CIPOPEN: если он пришёл позже.
    // Решение: накапливающий буфер + повторные дочитывания до таймаута 18с.
    {
        char cipBuf[256] = {};
        uint16_t cipLen = 0;
        uint32_t cipStart = HAL_GetTick();
        constexpr uint32_t CIP_TIMEOUT_MS = 18000;

        while ((HAL_GetTick() - cipStart) < CIP_TIMEOUT_MS) {
            // Дочитываем в остаток буфера
            char* tail   = cipBuf + cipLen;
            uint16_t rem = static_cast<uint16_t>(sizeof(cipBuf) - 1 - cipLen);
            if (rem == 0) break;
            uint16_t got = m_modem.waitFor_pub(tail, rem + 1, "+CIPOPEN:", 500);
            cipLen = static_cast<uint16_t>(std::strlen(cipBuf));
            IWDG->KR = 0xAAAA;
            if (std::strstr(cipBuf, "+CIPOPEN:")) break;
            if (std::strstr(cipBuf, "+CME ERROR")) break;
            (void)got;
        }

        if (!std::strstr(cipBuf, "+CIPOPEN:")) {
            DBG.error("TLS: CIPOPEN no response [%.80s]", cipBuf);
            return -1;
        }
        int id = 0, err = -1;
        const char* p = std::strstr(cipBuf, "+CIPOPEN:");
        std::sscanf(p, "+CIPOPEN: %d,%d", &id, &err);
        if (err != 0) {
            DBG.error("TLS: CIPOPEN err=%d addr=%s:%u", err, connectAddr, (unsigned)port);
            return -2;
        }
        std::memcpy(r, cipBuf, sizeof(r));
    }
    DBG.info("TLS: TCP open OK (link=%u)", (unsigned)m_sockId);

    // ---- mbedTLS seed ----
    const char* pers = "a7670c_tls";
    int rc = mbedtls_ctr_drbg_seed(&m_ctr,
                                    mbedtls_entropy_func, &m_entropy,
                                    reinterpret_cast<const unsigned char*>(pers),
                                    std::strlen(pers));
    if (rc != 0) { logMbedtlsErr("TLS: ctr_drbg_seed", rc); close(); return -10; }

    // ---- CA cert ----
    if (m_caPem) {
        rc = mbedtls_x509_crt_parse(&m_cacert,
                                     reinterpret_cast<const unsigned char*>(m_caPem),
                                     std::strlen(m_caPem) + 1);
        if (rc < 0) { logMbedtlsErr("TLS: x509_crt_parse", rc); close(); return -11; }
    }

    // ---- SSL config ----
    rc = mbedtls_ssl_config_defaults(&m_conf,
                                      MBEDTLS_SSL_IS_CLIENT,
                                      MBEDTLS_SSL_TRANSPORT_STREAM,
                                      MBEDTLS_SSL_PRESET_DEFAULT);
    if (rc != 0) { logMbedtlsErr("TLS: config_defaults", rc); close(); return -12; }

    mbedtls_ssl_conf_rng(&m_conf, mbedtls_ctr_drbg_random, &m_ctr);

    if (m_caPem) {
        mbedtls_ssl_conf_authmode(&m_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
        mbedtls_ssl_conf_ca_chain(&m_conf, &m_cacert, nullptr);
    } else {
        mbedtls_ssl_conf_authmode(&m_conf, MBEDTLS_SSL_VERIFY_NONE);
        DBG.warn("TLS: VERIFY_NONE (CA cert не задан)");
    }

    // ---- SSL setup ----
    rc = mbedtls_ssl_setup(&m_ssl, &m_conf);
    if (rc != 0) { logMbedtlsErr("TLS: ssl_setup", rc); close(); return -13; }

    rc = mbedtls_ssl_set_hostname(&m_ssl, host);
    if (rc != 0) { logMbedtlsErr("TLS: set_hostname", rc); close(); return -14; }

    mbedtls_ssl_set_bio(&m_ssl, this,
                         A7670CTls::biosend,
                         A7670CTls::biorecv,
                         nullptr);

    // ---- TLS handshake ----
    DBG.info("TLS: handshake start...");
    while (true) {
        rc = mbedtls_ssl_handshake(&m_ssl);
        if (rc == 0) break;
        if (rc == MBEDTLS_ERR_SSL_WANT_READ ||
            rc == MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (HAL_GetTick() > m_deadline) {
                DBG.error("TLS: handshake timeout");
                close(); return -20;
            }
            HAL_Delay(5);
            IWDG->KR = 0xAAAA;
            continue;
        }
        logMbedtlsErr("TLS: handshake", rc);
        close(); return -21;
    }

    DBG.info("TLS: OK, cipher=%s", mbedtls_ssl_get_ciphersuite(&m_ssl));
    m_connected = true;
    return 0;
}

// ============================================================================
// close
// ============================================================================
void A7670CTls::close()
{
    if (m_connected) {
        mbedtls_ssl_close_notify(&m_ssl);
        m_connected = false;
    }

    char cmd[32], r[64];
    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%hhu\r\n", m_sockId);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);

    tlsFree();
}

// ============================================================================
// httpsPost — полный цикл: connect + POST + read response + close
// ============================================================================
int A7670CTls::httpsPost(const char* url, const char* json, uint16_t jsonLen)
{
    UrlParts u{};
    if (!parseHttpsUrl(url, u)) {
        DBG.error("TLS: bad URL: %s", url ? url : "(null)");
        return -1;
    }

    int rc = connect(u.host, u.port);
    if (rc != 0) {
        DBG.error("TLS: connect failed rc=%d", rc);
        return -2;
    }

    // HTTP-заголовок
    char hdr[700];
    int hdrLen;
    const char* auth = Cfg().server_auth_b64;
    if (auth && auth[0]) {
        hdrLen = std::snprintf(hdr, sizeof(hdr),
            "POST %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Authorization: Basic %s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "Connection: close\r\n"
            "\r\n",
            u.path, u.host, auth, (unsigned)jsonLen);
    } else {
        hdrLen = std::snprintf(hdr, sizeof(hdr),
            "POST %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "Connection: close\r\n"
            "\r\n",
            u.path, u.host, (unsigned)jsonLen);
    }

    if (hdrLen <= 0 || (size_t)hdrLen >= sizeof(hdr)) {
        DBG.error("TLS HTTP: header overflow");
        close(); return -50;
    }

    // Отправляем заголовок + тело через mbedTLS
    int w;
    size_t off = 0;
    const uint8_t* hdrBuf = reinterpret_cast<const uint8_t*>(hdr);
    while (off < (size_t)hdrLen) {
        w = mbedtls_ssl_write(&m_ssl, hdrBuf + off, (size_t)hdrLen - off);
        if (w > 0) { off += (size_t)w; continue; }
        if (w == MBEDTLS_ERR_SSL_WANT_READ || w == MBEDTLS_ERR_SSL_WANT_WRITE) {
            HAL_Delay(2); IWDG->KR = 0xAAAA; continue;
        }
        logMbedtlsErr("TLS HTTP: write header", w);
        close(); return -51;
    }

    off = 0;
    const uint8_t* jsonBuf = reinterpret_cast<const uint8_t*>(json);
    while (off < (size_t)jsonLen) {
        w = mbedtls_ssl_write(&m_ssl, jsonBuf + off, (size_t)jsonLen - off);
        if (w > 0) { off += (size_t)w; continue; }
        if (w == MBEDTLS_ERR_SSL_WANT_READ || w == MBEDTLS_ERR_SSL_WANT_WRITE) {
            HAL_Delay(2); IWDG->KR = 0xAAAA; continue;
        }
        logMbedtlsErr("TLS HTTP: write body", w);
        close(); return -52;
    }

    // Читаем HTTP-ответ
    static char rx[1024];
    int used = 0, httpCode = -1;
    const uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < m_timeoutMs) {
        int r = mbedtls_ssl_read(&m_ssl,
                                  reinterpret_cast<unsigned char*>(rx + used),
                                  (size_t)(sizeof(rx) - 1 - used));
        if (r > 0) {
            used += r;
            rx[used] = '\0';
            const char* p = std::strstr(rx, "HTTP/1.");
            if (p) {
                int code = 0;
                if (std::sscanf(p, "HTTP/%*s %d", &code) == 1) {
                    httpCode = code;
                    DBG.info("TLS HTTP: code=%d", httpCode);
                    break;
                }
            }
            continue;
        }
        if (r == 0) break;
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
            HAL_Delay(5); IWDG->KR = 0xAAAA; continue;
        }
        logMbedtlsErr("TLS HTTP: read response", r);
        break;
    }

    close();
    return httpCode;
}
