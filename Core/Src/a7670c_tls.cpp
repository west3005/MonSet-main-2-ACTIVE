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
static constexpr uint16_t MODEM_MAX_SEND = 256;  // A7670C: >256 иногда ERROR
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

        // A7670C: после AT+CIPSEND сначала шлёт "OK\r\n", потом ">"
        // waitFor_pub останавливается на 50мс паузе — используем waitForUrc_pub
        m_modem.waitForUrc_pub(r, sizeof(r), ">", 5000);
        if (!std::strstr(r, ">")) {
            DBG.error("TLS BIO: no CIPSEND prompt, chunk=%u raw=[%.40s]",
                      (unsigned)chunk, r);
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

    // AT+CDNSGIP не поддерживается данной прошивкой A7670C.
    // Передаём hostname напрямую — A7670C резолвит его внутри стека при CIPOPEN.
    DBG.info("TLS: skip DNS, use hostname %s", connectAddr);

    // AT+CIPOPEN=<id>,"TCP","<ip_or_host>",<port>
    // Модем сначала отвечает "OK", затем асинхронно "+CIPOPEN: <id>,<err>"
    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd),
                  "AT+CIPOPEN=%u,\"TCP\",\"%s\",%u\r\n",
                  (unsigned)m_sockId, connectAddr, (unsigned)port);
    DBG.info("TLS: CIPOPEN to %s:%u", connectAddr, (unsigned)port);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));

    // +CIPOPEN: приходит асинхронно после OK — waitFor прерывается по 50мс тишины
    // и не ждёт URC. Используем waitForUrc_pub — без 50мс ограничения, только
    // по маркеру или таймауту 18с.
    {
        char cipBuf[256] = {};
        // waitForUrc_pub останавливается на маркере "+CIPOPEN:" не дочитав числа.
        // Дочитываем "\r\n" чтобы получить полный URC: "+CIPOPEN: 0,0\r\n"
        m_modem.waitForUrc_pub(cipBuf, sizeof(cipBuf), "+CIPOPEN:", 18000);
        {
            uint16_t already = (uint16_t)std::strlen(cipBuf);
            if (already < sizeof(cipBuf) - 1)
                m_modem.waitFor_pub(cipBuf + already,
                                    (uint16_t)(sizeof(cipBuf) - already - 1),
                                    "\r\n", 500);
        }
        DBG.info("TLS: CIPOPEN raw [%.80s]", cipBuf);
        if (!std::strstr(cipBuf, "+CIPOPEN:")) {
            DBG.error("TLS: CIPOPEN no URC, raw=[%.80s]", cipBuf);
            return -1;
        }
        int id = 0, err = -1;
        const char* p = std::strstr(cipBuf, "+CIPOPEN:");
        std::sscanf(p, "+CIPOPEN: %d,%d", &id, &err);
        if (err != 0) {
            DBG.error("TLS: CIPOPEN err=%d addr=%s:%u raw=[%.60s]", err, connectAddr, (unsigned)port, cipBuf);
            return -2;
        }
        std::memcpy(r, cipBuf, sizeof(r));
    }
    DBG.info("TLS: TCP open OK (link=%u)", (unsigned)m_sockId);
    HAL_Delay(300);  // A7670C: пауза после CIPOPEN перед первым CIPSEND

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

    char cmd[256], r[256];

    // --- Запускаем SSL сервис ---
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CCHSTART\r\n", 14);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 5000);
    // +CCHSTART: 0 = уже запущен, тоже OK
    if (!std::strstr(r, "OK") && !std::strstr(r, "+CCHSTART: 0")) {
        DBG.error("TLS: CCHSTART fail [%.40s]", r);
        return -2;
    }
    DBG.info("TLS CCH: CCHSTART OK");

    // --- Настраиваем SSL контекст (ctx 0) ---
    // sslversion: 4 = TLS (auto-negotiate)
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CSSLCFG=\"sslversion\",0,4\r\n", 28);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);
    // authmode: 0 = no auth (без проверки CA сертификата)
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CSSLCFG=\"authmode\",0,0\r\n", 27);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);
    // ignorelocaltime: игнорировать локальное время при проверке сертификата
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CSSLCFG=\"ignorelocaltime\",0,1\r\n", 34);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);
    // enableSNI: 1 = включить SNI (требуется для hostname-based серверов)
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CSSLCFG=\"enableSNI\",0,1\r\n", 29);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);
    // Привязываем SSL контекст 0 к CCH сессии 0
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CCHSSLCFG=0,0\r\n", 19);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 2000);
    DBG.info("TLS CCH: SSL ctx configured");

    // --- Открываем SSL соединение ---
    // AT+CCHOPEN=<session>,<host>,<port>  (без ssl_type — задан через CCHSSLCFG)
    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd), "AT+CCHOPEN=0,\"%s\",%u\r\n", u.host, (unsigned)u.port);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    DBG.info("TLS CCH: CCHOPEN %s:%u ...", u.host, (unsigned)u.port);
    // Ответ: +CCHOPEN: 0,0 (успех) или +CCHOPEN: 0,<err>
    m_modem.waitForUrc_pub(r, sizeof(r), "+CCHOPEN:", 30000);
    {
        uint16_t already = (uint16_t)std::strlen(r);
        if (already < sizeof(r) - 1)
            m_modem.waitFor_pub(r + already, (uint16_t)(sizeof(r) - already - 1), "\r\n", 1000);
    }
    DBG.info("TLS CCH: CCHOPEN raw [%.60s]", r);
    int sess = -1, cerr = -1;
    const char* cp = std::strstr(r, "+CCHOPEN:");
    if (cp) std::sscanf(cp, "+CCHOPEN: %d,%d", &sess, &cerr);
    if (cerr != 0) {
        DBG.error("TLS CCH: CCHOPEN err=%d", cerr);
        m_modem.flushRx_pub();
        m_modem.sendRaw_pub("AT+CCHSTOP\r\n", 13);
        m_modem.waitFor_pub(r, sizeof(r), "OK", 3000);
        return -3;
    }
    DBG.info("TLS CCH: SSL open OK");

    // --- Формируем HTTP запрос ---
    char hdr[512];
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
    uint16_t totalLen = (uint16_t)hdrLen + jsonLen;

    // --- Отправляем через AT+CCHSEND ---
    // AT+CCHSEND=<session>,<len> → prompt ">" → данные → +CCHSEND: 0,0
    m_modem.flushRx_pub();
    std::snprintf(cmd, sizeof(cmd), "AT+CCHSEND=0,%u\r\n", (unsigned)totalLen);
    m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
    DBG.info("TLS CCH: CCHSEND %u bytes...", (unsigned)totalLen);

    m_modem.waitForUrc_pub(r, sizeof(r), ">", 5000);
    if (!std::strstr(r, ">")) {
        DBG.error("TLS CCH: no CCHSEND prompt raw=[%.40s]", r);
        cchClose();
        return -4;
    }

    // Отправляем заголовок и тело одним потоком
    m_modem.sendRaw_pub(hdr, (uint16_t)hdrLen);
    m_modem.sendRaw_pub(json, jsonLen);

    // Ждём ответ модема после отправки данных.
    // A7670C может прислать:
    //   a) "+CCHSEND: 0,0" (ack отправки), затем отдельно "+CCHRECV: DATA,..."
    //   b) Сразу "+CCHRECV: DATA,0,N\r\nHTTP/1.1 ..." без CCHSEND ack
    // Ждём любой из маркеров, таймаут 10с.
    static char rx[1024];
    int used = 0, httpCode = -1;

    m_modem.waitForUrc_pub(r, sizeof(r), "+CCH", 10000);
    {
        uint16_t already = (uint16_t)std::strlen(r);
        if (already < sizeof(r) - 1)
            m_modem.waitFor_pub(r + already, (uint16_t)(sizeof(r) - already - 1), "\r\n", 1000);
    }
    DBG.info("TLS CCH: after-send buf [%.80s]", r);

    // Случай (b): HTTP ответ уже в буфере вместе с CCHRECV
    if (std::strstr(r, "HTTP/1.")) {
        // Извлекаем HTTP часть
        const char* hp = std::strstr(r, "HTTP/1.");
        int hlen = (int)std::strlen(hp);
        if (hlen > (int)sizeof(rx) - 1) hlen = (int)sizeof(rx) - 1;
        std::memcpy(rx, hp, (size_t)hlen);
        rx[hlen] = '\0';
        used = hlen;
        DBG.info("TLS CCH: HTTP in ack buf");
    }
    // Случай (a): пришёл CCHRECV URC — нужно запросить данные
    // r[] уже содержит начало ответа (after-send buf прочитал первый чанк).
    // НЕ делаем flush — копируем r[] в rx и дочитываем остаток.
    else if (std::strstr(r, "+CCHRECV:")) {
        int recvLen = 0;
        const char* rp = std::strstr(r, "DATA,");
        if (rp) std::sscanf(rp, "DATA,%*d,%d", &recvLen);
        if (recvLen > 0) {
            // Шаг 1: копируем уже прочитанный буфер r[] в rx
            used = (int)std::strlen(r);
            if (used > (int)sizeof(rx) - 1) used = (int)sizeof(rx) - 1;
            std::memcpy(rx, r, (size_t)used);
            rx[used] = '\0';
            // Шаг 2: дочитываем остаток из UART (без flush!) до финального OK
            std::snprintf(cmd, sizeof(cmd), "AT+CCHRECV=0,%d\r\n", recvLen);
            m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
            uint16_t avail = (uint16_t)(sizeof(rx) - 1 - (uint16_t)used);
            if (avail > 0) {
                m_modem.waitFor_pub(rx + used, avail, "\r\nOK\r\n", 6000);
                used = (int)std::strlen(rx);
            }
            DBG.info("TLS CCH: raw recv len=%d", used);
            // Ищем HTTP/1. в любом месте объединённого буфера
            const char* hstart = std::strstr(rx, "HTTP/1.");
            if (hstart) {
                int hl = (int)std::strlen(hstart);
                std::memmove(rx, hstart, (size_t)hl + 1);
                used = hl;
            } else {
                used = (int)std::strlen(rx);
            }
            DBG.info("TLS CCH: CCHRECV data [%.60s]", rx);
        }
    }

    // Дочитываем если HTTP ответ ещё не полный
    const uint32_t t0 = HAL_GetTick();
    while (!std::strstr(rx, "HTTP/1.") &&
           (HAL_GetTick() - t0) < 8000 &&
           used < (int)sizeof(rx) - 1) {
        char urc[128] = {};
        m_modem.waitForUrc_pub(urc, sizeof(urc), "+CCHRECV:", 3000);
        if (!std::strstr(urc, "+CCHRECV:")) break;
        int recvLen = 0;
        const char* rp2 = std::strstr(urc, "DATA,");
        if (rp2) std::sscanf(rp2, "DATA,%*d,%d", &recvLen);
        if (recvLen <= 0) break;
        std::snprintf(cmd, sizeof(cmd), "AT+CCHRECV=0,%d\r\n", recvLen);
        m_modem.flushRx_pub();
        m_modem.sendRaw_pub(cmd, (uint16_t)std::strlen(cmd));
        char* dst = rx + used;
        m_modem.waitFor_pub(dst, (uint16_t)(sizeof(rx) - 1 - used), "\r\n\r\n", 3000);
        const char* ds2 = std::strstr(dst, "\r\n");
        if (ds2) { ds2 += 2; int dl = (int)std::strlen(ds2); std::memmove(dst, ds2, (size_t)dl + 1); used += dl; }
        else { used += (int)std::strlen(dst); }
        rx[used] = '\0';
        IWDG->KR = 0xAAAA;
    }

    // Парсим HTTP код
    const char* p = std::strstr(rx, "HTTP/1.");
    if (p) {
        std::sscanf(p, "HTTP/1.%*d %d", &httpCode);
        DBG.info("TLS CCH: HTTP %d", httpCode);
    } else {
        DBG.error("TLS CCH: нет HTTP ответа [%.60s]", rx);
    }

    cchClose();
    return httpCode;
}

void A7670CTls::cchClose()
{
    char r[64];
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CCHCLOSE=0\r\n", 16);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 3000);
    m_modem.flushRx_pub();
    m_modem.sendRaw_pub("AT+CCHSTOP\r\n", 13);
    m_modem.waitFor_pub(r, sizeof(r), "OK", 3000);
    DBG.info("TLS CCH: closed");
}
