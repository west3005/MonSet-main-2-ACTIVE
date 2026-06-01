/**
 * @file a7670c_tls.hpp
 * @brief HTTPS через mbedTLS поверх TCP-сокета A7670C.
 *
 * Схема:
 *   STM32 mbedTLS → AT+CSOSEND (бинарный поток) → A7670C → сервер 443
 *   A7670C — прозрачный TCP-туннель, TLS делает STM32.
 *
 * Отличие от SIM7020C версии:
 *   - modemWriteRaw: AT+CSOSEND  (A7670C) вместо AT+CSODSEND (SIM7020C)
 *   - modemReadRaw:  AT+CSORCVDATA (A7670C) вместо AT+CSORCV (SIM7020C)
 */
#pragma once
#include "a7670c.hpp"

extern "C" {
#include "mbedtls/ssl.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/x509_crt.h"
}

class A7670CTls {
public:
    explicit A7670CTls(A7670C& modem, uint32_t timeoutMs = 30000);
    ~A7670CTls();

    // CA cert в PEM формате (nullptr = VERIFY_NONE)
    void setCaCert(const char* pem) { m_caPem = pem; }

    // Установить TCP+TLS соединение
    int  connect(const char* host, uint16_t port);

    // HTTPS POST (вызывать после connect)
    int      httpsPost(const char* url, const char* json, uint16_t len);

    // Закрыть соединение
    void close();

    bool isConnected() const { return m_connected; }

private:
    A7670C&   m_modem;
    uint32_t  m_timeoutMs;
    uint32_t  m_deadline   = 0;
    uint8_t   m_sockId     = 1;
    bool      m_connected  = false;
    bool      m_tlsInited  = false;
    const char* m_caPem    = nullptr;

    mbedtls_ssl_context    m_ssl;
    mbedtls_ssl_config     m_conf;
    mbedtls_entropy_context m_entropy;
    mbedtls_ctr_drbg_context m_ctr;
    mbedtls_x509_crt       m_cacert;

    int  modemWriteRaw(const uint8_t* buf, uint16_t len);
    int  modemReadRaw(uint8_t* buf, uint16_t len);

    static int biosend(void* ctx, const unsigned char* buf, size_t len);
    static int biorecv(void* ctx, unsigned char* buf, size_t len);

    void logMbedtlsErr(const char* tag, int rc);
    void tlsFree();

    struct UrlParts {
        char     host[64]{};
        char     path[128]{};
        uint16_t port = 443;
    };
    bool parseHttpsUrl(const char* url, UrlParts& out);
};
