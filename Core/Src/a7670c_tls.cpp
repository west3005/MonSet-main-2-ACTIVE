/**
 * ================================================================
 * @file    a7670c_tls.cpp
 * @brief   TLS-туннель поверх A7670C TCP-сокета.
 *
 * Транспортный слой:
 *   mbedTLS вызывает biosend/biorecv, которые отправляют/читают
 *   данные через AT+CSOSEND / AT+CSORCVDATA (A7670C TCP socket).
 *
 * Отличия от SIM7020C (sim7020c_tls.cpp):
 *   - AT+CSOSEND=<id>,<len>  — A7670C принимает бинарный поток
 *     (prompt '>', затем N байт, затем ОК)
 *   - AT+CSORCVDATA=<id>,<maxLen> — возвращает данные в бинаре
 *   - Максимальный чанк: 1460 байт (MTU) на CSOSEND
 *   - Ответ CSORCVDATA: "+CSORCVDATA: <id>,<len>\r\n<data>"
 *
 * Память:
 *   mbedTLS требует ~40-80 КБ RAM для TLS 1.2.
 *   STM32F407 192 КБ RAM — достаточно.
 *   При нехватке: MBEDTLS_SSL_MAX_CONTENT_LEN = 4096
 * ================================================================
 */
#include "a7670c_tls.hpp"
#include "debug_uart.hpp"

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
// Транспорт: отправка через AT+CSOSEND (A7670C)
// Формат: AT+CSOSEND=<id>,<len>\r\n  → prompt '>' → <binary data> → OK
// ============================================================================
int A7670CTls::modemWriteRaw(const uint8_t* buf, uint16_t len)
{
    if (!buf || len == 0) return 0;

    uint16_t sent = 0;
    char cmd[64];
    char r[64];

    while (sent < len) {
        uint16_t chunk = len - sent;
        if (chunk > MODEM_MAX_SEND) chunk = MODEM_MAX_SEND;

        std::snprintf(cmd, sizeof(cmd),
                      "AT+CSOSEND=%hhu,%u\r\n", m_sockId, (unsigned)chunk);
        m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));

        // Ждём prompt '>'
        m_modem.waitFor_pub(r, sizeof(r), ">", 3000);
        if (!std::strstr(r, ">")) {
            DBG.error("TLS BIO: нет prompt '>' для CSOSEND");
            return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
        }

        // Отправляем бинарные данные прямо в UART
        m_modem.sendRaw_pub(reinterpret_cast<const char*>(buf + sent), chunk);

        // Ждём OK
        m_modem.waitFor_pub(r, sizeof(r), "OK", 3000);
        if (!std::strstr(r, "OK")) {
            DBG.warn("TLS BIO: CSOSEND не ответил OK (chunk=%u)", (unsigned)chunk);
        }

        sent += chunk;
        IWDG->KR = 0xAAAA;
    }

    return (int)sent;
}

// ============================================================================
// Транспорт: приём через AT+CSORCVDATA (A7670C)
// Ответ: "+CSORCVDATA: <id>,<len>\r\n<binary data>\r\nOK"
// ============================================================================
int A7670CTls::modemReadRaw(uint8_t* buf, uint16_t len)
{
    if (!buf || len == 0) return 0;

    uint16_t want = (len > MODEM_MAX_RECV) ? MODEM_MAX_RECV : len;
    char cmd[64];
    char r[MODEM_MAX_RECV + 128];

    std::snprintf(cmd, sizeof(cmd),
                  "AT+CSORCVDATA=%hhu,%u\r\n", m_sockId, (unsigned)want);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));

    uint16_t rxLen = m_modem.waitFor_pub(r, sizeof(r), "+CSORCVDATA:", 3000);
    (void)rxLen;

    if (!std::strstr(r, "+CSORCVDATA:")) {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }

    const char* p = std::strstr(r, "+CSORCVDATA:");
    int id = 0, recvLen = 0;
    if (std::sscanf(p, "+CSORCVDATA: %d,%d", &id, &recvLen) != 2 || recvLen <= 0) {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }

    const char* dataStart = std::strchr(p, '\n');
    if (!dataStart) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    dataStart++;

    uint16_t copy = (uint16_t)recvLen;
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

    if (HAL_GetTick() > self->m_deadline)
        return MBEDTLS_ERR_SSL_TIMEOUT;

    uint16_t toSend = (len > MODEM_MAX_SEND) ? MODEM_MAX_SEND : (uint16_t)len;
    int r = self->modemWriteRaw(buf, toSend);
    if (r <= 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    return r;
}

int A7670CTls::biorecv(void* ctx, unsigned char* buf, size_t len)
{
    auto* self = static_cast<A7670CTls*>(ctx);
    if (!self || !buf || len == 0) return 0;

    if (HAL_GetTick() > self->m_deadline)
        return MBEDTLS_ERR_SSL_TIMEOUT;

    uint16_t toRead = (len > MODEM_MAX_RECV) ? MODEM_MAX_RECV : (uint16_t)len;
    int r = self->modemReadRaw(buf, toRead);

    if (r == MBEDTLS_ERR_SSL_WANT_READ) return r;
    if (r < 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    if (r == 0) return MBEDTLS_ERR_SSL_WANT_READ;
    return r;
}

// ============================================================================
// connect(): TCP + TLS handshake
// ============================================================================
int A7670CTls::connect(const char* host, uint16_t port)
{
    DBG.info("TLS: connect %s:%u", host, port);
    m_deadline = HAL_GetTick() + m_timeoutMs;

    char r[256], cmd[128];

    // Закрыть предыдущий сокет (может не существовать — OK и ERROR оба допустимы)
    m_modem.flushRx_pub();  // сброс буфера перед командой
    std::snprintf(cmd, sizeof(cmd), "AT+CSOCL=%hhu\r\n", m_sockId);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000); // drain: OK или ERROR
    HAL_Delay(200);

    // Создать TCP-сокет
    m_modem.flushRx_pub();  // сброс остатков ответа CSOCL
    m_modem.sendRaw_pub("AT+CSOC=1,1,1\r\n", 16);
    m_modem.waitFor_pub(r, sizeof(r), "+CSOC:", 5000);
    if (!std::strstr(r, "+CSOC:")) {
        DBG.error("TLS: CSOC create failed [%.80s]", r);
        return -1;
    }
    const char* sp = std::strstr(r, "+CSOC:");
    if (sp) std::sscanf(sp, "+CSOC: %hhu", &m_sockId);

    // Подключиться к серверу
    std::snprintf(cmd, sizeof(cmd),
                  "AT+CSOCON=%hhu,%u,\"%s\"\r\n", m_sockId, port, host);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    m_modem.waitFor_pub(r, sizeof(r), "OK", 15000);
    if (!std::strstr(r, "OK")) {
        DBG.error("TLS: CSOCON failed host=%s:%u", host, port);
        return -2;
    }
    DBG.info("TLS: TCP socket open OK (id=%hhu)", m_sockId);

    // mbedTLS seed
    const char* pers = "a7670c_tls";
    int rc = mbedtls_ctr_drbg_seed(&m_ctr,
                                    mbedtls_entropy_func,
                                    &m_entropy,
                                    reinterpret_cast<const unsigned char*>(pers),
                                    std::strlen(pers));
    if (rc != 0) { logMbedtlsErr("TLS: ctr_drbg_seed", rc); close(); return -10; }

    // CA cert
    if (m_caPem) {
        rc = mbedtls_x509_crt_parse(&m_cacert,
                                     reinterpret_cast<const unsigned char*>(m_caPem),
                                     std::strlen(m_caPem) + 1);
        if (rc < 0) { logMbedtlsErr("TLS: x509_crt_parse", rc); close(); return -11; }
    }

    // SSL config
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

    rc = mbedtls_ssl_setup(&m_ssl, &m_conf);
    if (rc != 0) { logMbedtlsErr("TLS: ssl_setup", rc); close(); return -13; }

    rc = mbedtls_ssl_set_hostname(&m_ssl, host);
    if (rc != 0) { logMbedtlsErr("TLS: set_hostname", rc); close(); return -14; }

    mbedtls_ssl_set_bio(&m_ssl, this, A7670CTls::biosend, A7670CTls::biorecv, nullptr);

    // TLS Handshake
    DBG.info("TLS: handshake...");
    while ((rc = mbedtls_ssl_handshake(&m_ssl)) != 0) {
        if (rc != MBEDTLS_ERR_SSL_WANT_READ && rc != MBEDTLS_ERR_SSL_WANT_WRITE) {
            logMbedtlsErr("TLS: handshake failed", rc);
            close();
            return -20;
        }
        if (HAL_GetTick() > m_deadline) {
            DBG.error("TLS: handshake timeout");
            close();
            return -21;
        }
        IWDG->KR = 0xAAAA;
    }

    DBG.info("TLS: handshake OK, cipher=%s",
             mbedtls_ssl_get_ciphersuite(&m_ssl));
    m_connected = true;
    return 0;
}

// ============================================================================
// HTTPS POST
// ============================================================================
uint16_t A7670CTls::httpsPost(const char* url, const char* json, uint16_t len)
{
    if (!url || !json || len == 0) return 0;

    UrlParts u{};
    if (!parseHttpsUrl(url, u)) {
        DBG.error("A7670C HTTPS: плохой URL: %s", url);
        return 0;
    }

    if (!m_connected) {
        if (connect(u.host, u.port) != 0) return 0;
    }

    // Формируем HTTP-запрос
    char hdr[600];
    int hdrLen = std::snprintf(hdr, sizeof(hdr),
        "POST %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "\r\n",
        u.path, u.host, (unsigned)len);

    // Отправить заголовок через TLS
    int rc = mbedtls_ssl_write(&m_ssl,
                                reinterpret_cast<const unsigned char*>(hdr),
                                (size_t)hdrLen);
    if (rc < 0) {
        logMbedtlsErr("A7670C HTTPS: write header", rc);
        close(); return 0;
    }

    // Отправить тело
    rc = mbedtls_ssl_write(&m_ssl,
                            reinterpret_cast<const unsigned char*>(json),
                            len);
    if (rc < 0) {
        logMbedtlsErr("A7670C HTTPS: write body", rc);
        close(); return 0;
    }

    // Читаем ответ
    uint8_t rxBuf[512]{};
    uint16_t code = 0;
    uint32_t deadline = HAL_GetTick() + 15000;

    while (HAL_GetTick() < deadline) {
        rc = mbedtls_ssl_read(&m_ssl, rxBuf, sizeof(rxBuf) - 1);
        if (rc == MBEDTLS_ERR_SSL_WANT_READ) {
            IWDG->KR = 0xAAAA;
            continue;
        }
        if (rc <= 0) break;

        rxBuf[rc] = '\0';
        DBG.data("A7670C HTTPS rx: %s", (char*)rxBuf);

        if (code == 0) {
            const char* p = std::strstr((char*)rxBuf, "HTTP/1.");
            if (p) std::sscanf(p, "HTTP/1.%*c %hu", &code);
        }
        if (code != 0) break;
        IWDG->KR = 0xAAAA;
    }

    close();
    DBG.info("A7670C HTTPS: response code %u", (unsigned)code);
    return code;
}

// ============================================================================
// close()
// ============================================================================
void A7670CTls::close()
{
    if (m_connected) {
        mbedtls_ssl_close_notify(&m_ssl);
        m_connected = false;
    }
    char r[64], cmd[32];
    std::snprintf(cmd, sizeof(cmd), "AT+CSOCL=%hhu\r\n", m_sockId);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    HAL_Delay(100);
    DBG.info("TLS: соединение закрыто");
}
