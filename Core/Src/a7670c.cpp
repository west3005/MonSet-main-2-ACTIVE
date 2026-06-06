/**
 * ================================================================
 * @file    a7670c.cpp
 * @brief   Реализация драйвера A7670C LTE Cat-1 / GSM.
 *          Наследует Air780E, переопределяет AT-команды.
 *
 * Отличия от Air780E (Air780E):
 *   - Ожидание загрузки: "+CGEV: ME PDN ACT" (не "RDY")
 *   - PWRKEY: 1500 мс
 *   - Регистрация: CREG + CEREG (GSM fallback)
 *   - TCP send: AT+CSOSEND (не CIPSEND)
 *   - TCP recv: AT+CSORCVDATA (не CIPRXGET)
 *   - MQTT: AT+CMQNEW/CMQCON/CMQPUB (не CMQTTSTART)
 * ================================================================
 */
#include "a7670c.hpp"
#include "debug_uart.hpp"
#include "board_pins.hpp"
#include "config.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>

// ============================================================================
// Вспомогательный парсинг HTTP URL
// ============================================================================
namespace {
struct UrlParts {
    char     host[64]{};
    char     path[128]{};
    uint16_t port = 80;
};

static bool parseHttpUrl(const char* url, UrlParts& out)
{
    if (!url) return false;
    out = UrlParts{};
    out.port = 80;
    const char* prefix = "http://";
    if (std::strncmp(url, prefix, std::strlen(prefix)) != 0) return false;
    const char* p = url + std::strlen(prefix);
    const char* hb = p;
    while (*p && *p != '/' && *p != ':') p++;
    size_t hl = (size_t)(p - hb);
    if (hl == 0 || hl >= sizeof(out.host)) return false;
    std::memcpy(out.host, hb, hl); out.host[hl] = '\0';
    if (*p == ':') {
        p++;
        uint32_t port = 0;
        while (*p && std::isdigit((unsigned char)*p))
            port = port * 10u + (uint32_t)(*p++ - '0');
        if (port == 0 || port > 65535) return false;
        out.port = (uint16_t)port;
    }
    if (*p == '\0') { std::strcpy(out.path, "/"); return true; }
    if (*p != '/') return false;
    size_t pl = std::strlen(p);
    if (pl == 0 || pl >= sizeof(out.path)) return false;
    std::memcpy(out.path, p, pl + 1);
    return true;
}
} // namespace

// ============================================================================
// Конструктор — делегируем в Air780E
// ============================================================================
A7670C::A7670C(UART_HandleTypeDef* uart,
               GPIO_TypeDef*        pwrPort,
               uint16_t             pwrPin)
    : Air780E(uart, pwrPort, pwrPin)
{}

// ============================================================================
// Ожидание готовности (ждём "+CGEV: ME PDN ACT")
// ============================================================================
bool A7670C::waitRdyA7670(uint32_t timeoutMs)
{
    char buf[256]{};
    uint16_t idx = 0;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeoutMs) {
        uint8_t ch;
        if (HAL_UART_Receive(m_uart, &ch, 1, 20) == HAL_OK) {
            if (idx < sizeof(buf) - 1) buf[idx++] = static_cast<char>(ch);
            buf[idx] = '\0';
            if (std::strstr(buf, "ME PDN ACT")) {
                DBG.info("A7670C: загрузка завершена за %lu мс",
                         (unsigned long)(HAL_GetTick() - start));
                HAL_Delay(2000);
                return true;
            }
            if (idx >= sizeof(buf) - 2) {
                std::memmove(buf, buf + 64, sizeof(buf) - 64);
                idx -= 64;
            }
        }
        IWDG->KR = 0xAAAA;
    }
    DBG.warn("A7670C: таймаут ожидания ME PDN ACT");
    return false;
}

// ============================================================================
// Управление питанием
// ============================================================================
void A7670C::powerOn()
{
    DBG.info("A7670C: включение...");
    HAL_GPIO_WritePin(m_pwrPort, m_pwrPin, GPIO_PIN_SET);
    HAL_Delay(300);

    GPIO_PinState status = HAL_GPIO_ReadPin(PIN_CELL_STATUS_PORT, PIN_CELL_STATUS_PIN);
    if (status == GPIO_PIN_SET) {
        DBG.info("A7670C: уже включён (STATUS=HIGH)");
        return;
    }

    DBG.info("A7670C: PWRKEY 1500 мс...");
    HAL_GPIO_WritePin(PIN_CELL_PWRKEY_PORT, PIN_CELL_PWRKEY_PIN, GPIO_PIN_RESET);
    HAL_Delay(1500);
    HAL_GPIO_WritePin(PIN_CELL_PWRKEY_PORT, PIN_CELL_PWRKEY_PIN, GPIO_PIN_SET);

    if (!waitRdyA7670(25000)) {
        DBG.warn("A7670C: ME PDN ACT не получен — пробуем AT");
        // Дополнительная пауза: модем ещё загружается, SIM не инициализирована
        HAL_Delay(3000);
    }
    // Сбрасываем буфер UART после загрузки модема
    __HAL_UART_FLUSH_DRREGISTER(m_uart);
    g_air780_rxbuf.clear();
}

void A7670C::powerOff()
{
    DBG.info("A7670C: выключение...");
    GPIO_PinState status = HAL_GPIO_ReadPin(PIN_CELL_STATUS_PORT, PIN_CELL_STATUS_PIN);
    if (status == GPIO_PIN_RESET) {
        DBG.info("A7670C: уже выключен");
        HAL_GPIO_WritePin(m_pwrPort, m_pwrPin, GPIO_PIN_RESET);
        return;
    }
    char r[64];
    sendCommand("+CPOWD=1", r, sizeof(r), 3000);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(m_pwrPort, m_pwrPin, GPIO_PIN_RESET);
}

void A7670C::hardReset()
{
    DBG.warn("A7670C: аппаратный сброс");
    HAL_GPIO_WritePin(PIN_CELL_RESET_PORT, PIN_CELL_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(PIN_CELL_RESET_PORT, PIN_CELL_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(500);
    waitRdyA7670(25000);
}

// ============================================================================
// Активация PDN
// ============================================================================
GsmStatus A7670C::activatePdnA7670()
{
    const RuntimeConfig& c = Cfg();
    char r[256], cmd[128];

    // Если APN не задан — используем Tele2 (временно для отладки)
    const char* apn = (c.gsm_apn[0] != '\0') ? c.gsm_apn : "internet.tele2.ru";
    std::snprintf(cmd, sizeof(cmd), "+CGDCONT=1,\"IP\",\"%s\"", apn);
    sendCommand(cmd, r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);

    if (sendCommand("+CGACT=1,1", r, sizeof(r),
                    Config::SIM7020_PDN_TIMEOUT_MS) != GsmStatus::Ok) {
        sendCommand("+CGACT?", r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);
        if (!std::strstr(r, ",1")) {
            DBG.error("A7670C: PDN activation failed");
            return GsmStatus::PdnErr;
        }
    }

    // Включить буферизацию приёма (обязательно перед AT+CIPOPEN)
    sendCommand("+CIPRXGET=1", r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);

    // Ждём IP-адрес (CGACT OK не гарантирует мгновенного IP)
    // Ответ A7670C: +CGPADDR: 1,10.x.x.x  (без кавычек)
    {
        char ip[64] = {};
        const uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 6000) {
            sendCommand("+CGPADDR=1", r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);
            const char* q = std::strstr(r, "+CGPADDR:");
            if (q) {
                // Пропускаем до запятой после cid: "+CGPADDR: 1,10.x.x.x"
                const char* comma = std::strchr(q, ',');
                if (comma) {
                    comma++;
                    // Пропускаем пробелы и кавычки (на случай прошивок с кавычками)
                    while (*comma == ' ' || *comma == '"') comma++;
                    size_t ipLen = 0;
                    while (comma[ipLen] && comma[ipLen] != '\r' &&
                           comma[ipLen] != '\n' && comma[ipLen] != '"' &&
                           ipLen < sizeof(ip) - 1) {
                        ip[ipLen] = comma[ipLen];
                        ipLen++;
                    }
                    ip[ipLen] = '\0';
                }
            }
            if (ip[0] && std::strcmp(ip, "0.0.0.0") != 0) {
                DBG.info("A7670C: IP=%s", ip);
                break;
            }
            ip[0] = '\0';
            HAL_Delay(500);
            IWDG->KR = 0xAAAA;
        }
        if (!ip[0] || std::strcmp(ip, "0.0.0.0") == 0) {
            DBG.warn("A7670C: IP не получен за 6с, продолжаем...");
        }
    }

    // AT+NETOPEN — активирует TCP/IP стек, обязателен для AT+CIPOPEN
    // "+NETOPEN: 1" означает "уже открыт" — не ошибка
    sendCommand("+NETOPEN", r, sizeof(r), 5000);
    if (std::strstr(r, "ERROR") && !std::strstr(r, "+NETOPEN: 1")) {
        DBG.warn("A7670C: NETOPEN warn [%.40s]", r);
    } else {
        DBG.info("A7670C: NETOPEN OK");
    }
    HAL_Delay(300);

    return GsmStatus::Ok;
}

// ============================================================================
// Инициализация
// ============================================================================
GsmStatus A7670C::init()
{
    char r[256];

    __HAL_UART_FLUSH_DRREGISTER(m_uart);
    HAL_Delay(100);

    bool alive = false;
    for (uint8_t i = 0; i < 10 && !alive; i++) {
        if (sendCommand("", r, sizeof(r), 2000) == GsmStatus::Ok) alive = true;
        else HAL_Delay(500);
        IWDG->KR = 0xAAAA;
    }
    if (!alive) { DBG.error("A7670C: нет ответа на AT"); return GsmStatus::Timeout; }

    sendCommand("E0", r, sizeof(r), 2000);
    sendCommand("E0", r, sizeof(r), 2000);
    sendCommand("+IPR=115200", r, sizeof(r), 2000);
    // AT&W не поддерживается A7670C — пропускаем
    sendCommand("+CTZU=1", r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);

    // Ждём SIM READY до 10 секунд — после включения SIM инициализируется с задержкой
    {
        bool simReady = false;
        for (uint8_t attempt = 0; attempt < 10 && !simReady; attempt++) {
            sendCommand("+CPIN?", r, sizeof(r), 3000);
            if (std::strstr(r, "READY")) { simReady = true; break; }
            DBG.warn("A7670C: SIM не готова (попытка %u/10): %.30s", attempt+1, r);
            HAL_Delay(1000);
            IWDG->KR = 0xAAAA;
        }
        if (!simReady) {
            DBG.error("A7670C: SIM не ответила за 10 с");
            return GsmStatus::NoSim;
        }
    }

    // Регистрация: CREG (GSM) + CEREG (LTE), ждём до 60 сек
    bool registered = false;
    for (uint8_t i = 0; i < 60 && !registered; i++) {
        sendCommand("+CREG?", r, sizeof(r), 2000);
        if (std::strstr(r, ",1") || std::strstr(r, ",5")) { registered = true; break; }
        sendCommand("+CEREG?", r, sizeof(r), 2000);
        if (std::strstr(r, ",1") || std::strstr(r, ",5")) { registered = true; break; }
        HAL_Delay(1000);
        IWDG->KR = 0xAAAA;
    }
    if (!registered) { DBG.error("A7670C: нет регистрации"); return GsmStatus::NoReg; }

    activatePdnA7670();
    DBG.info("A7670C: init OK");
    return GsmStatus::Ok;
}

// ============================================================================
// Отключение
// ============================================================================
void A7670C::disconnect()
{
    char r[64];
    // AT+CIPCLOSE — закрыть TCP сокет (CIPOPEN стек)
    sendCommand("+CIPCLOSE=0", r, sizeof(r), 2000);
    // AT+NETCLOSE — закрыть TCP/IP стек
    sendCommand("+NETCLOSE", r, sizeof(r), 3000);
    sendCommand("+CGACT=0,1", r, sizeof(r), 5000);
    DBG.info("A7670C: отключён");
}

// ============================================================================
// Уровень сигнала
// ============================================================================
uint8_t A7670C::getSignalQuality()
{
    char r[64];
    sendCommand("+CSQ", r, sizeof(r), 2000);
    const char* p = std::strstr(r, "+CSQ:");
    if (p) {
        uint8_t v = 99;
        std::sscanf(p, "+CSQ: %hhu", &v);
        return v;
    }
    return 99;
}

// ============================================================================
// HTTP POST через TCP socket (порт 80)
// AT+CSOC / CSOCON / CSOSEND / CSORCVDATA
// ============================================================================
uint16_t A7670C::httpPost(const char* url, const char* json, uint16_t len)
{
    if (!url || !json || len == 0) return 0;

    const RuntimeConfig& c = Cfg();
    UrlParts u{};
    if (!parseHttpUrl(url, u)) {
        DBG.error("A7670C HTTP: плохой URL: %s", url);
        return 0;
    }

    char r[512], cmd[256];

    // Создать TCP-сокет
    if (sendCommand("+CSOC=1,1,1", r, sizeof(r),
                    Config::SIM7020_CMD_TIMEOUT_MS) != GsmStatus::Ok) {
        DBG.error("A7670C HTTP: CSOC create failed");
        return 0;
    }
    uint8_t sockId = 0;
    { const char* p = std::strstr(r, "+CSOC:"); if (p) std::sscanf(p, "+CSOC: %hhu", &sockId); }

    // Подключиться к хосту
    std::snprintf(cmd, sizeof(cmd), "+CSOCON=%hhu,%u,\"%s\"", sockId, u.port, u.host);
    if (sendCommand(cmd, r, sizeof(r), Config::SIM7020_TCP_TIMEOUT_MS) != GsmStatus::Ok) {
        DBG.error("A7670C HTTP: CSOCON failed %s:%u", u.host, u.port);
        sendCommand("+CSOCL=0", r, sizeof(r), 2000);
        return 0;
    }

    // HTTP-заголовок
    char hdr[600];
    int hdrLen;
    if (c.server_auth_b64[0]) {
        hdrLen = std::snprintf(hdr, sizeof(hdr),
            "POST %s HTTP/1.1\r\nHost: %s\r\n"
            "Authorization: Basic %s\r\n"
            "Content-Type: application/json\r\nContent-Length: %u\r\n"
            "Connection: close\r\n\r\n",
            u.path, u.host, c.server_auth_b64, (unsigned)len);
    } else {
        hdrLen = std::snprintf(hdr, sizeof(hdr),
            "POST %s HTTP/1.1\r\nHost: %s\r\n"
            "Content-Type: application/json\r\nContent-Length: %u\r\n"
            "Connection: close\r\n\r\n",
            u.path, u.host, (unsigned)len);
    }

    // Отправить заголовок
    std::snprintf(cmd, sizeof(cmd), "+CSOSEND=%hhu,%d", sockId, hdrLen);
    sendCommand(cmd, r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);
    sendRaw(hdr, (uint16_t)hdrLen);
    readResponse(r, sizeof(r), 3000);

    // Отправить тело
    std::snprintf(cmd, sizeof(cmd), "+CSOSEND=%hhu,%u", sockId, (unsigned)len);
    sendCommand(cmd, r, sizeof(r), Config::SIM7020_CMD_TIMEOUT_MS);
    sendRaw(json, len);
    readResponse(r, sizeof(r), 3000);

    // Читаем HTTP-ответ
    std::snprintf(cmd, sizeof(cmd), "+CSORCVDATA=%hhu,512", sockId);
    waitFor(r, sizeof(r), "HTTP/1.", Config::SIM7020_TCP_TIMEOUT_MS);

    uint16_t code = 0;
    const char* p = std::strstr(r, "HTTP/1.");
    if (p) std::sscanf(p, "HTTP/1.%*c %hu", &code);
    DBG.info("A7670C HTTP: code=%u", (unsigned)code);

    sendCommand("+CSOCL=0", r, sizeof(r), 2000);
    return code;
}

// ============================================================================
// MQTT (AT+CMQNEW / CMQCON / CMQPUB)
// ============================================================================
GsmStatus A7670C::mqttConnect(const char* broker, uint16_t port,
                               const char* clientId,
                               const char* user, const char* pass)
{
    char r[256], cmd[256];

    // Создать MQTT клиент
    std::snprintf(cmd, sizeof(cmd),
        "+CMQNEW=\"%s\",%u,10000,1024", broker, port);
    if (sendCommand(cmd, r, sizeof(r), 15000) != GsmStatus::Ok) {
        DBG.error("A7670C MQTT: CMQNEW failed");
        return GsmStatus::MqttErr;
    }

    uint8_t mqttId = 0;
    {
        const char* p = std::strstr(r, "+CMQNEW:");
        if (p) std::sscanf(p, "+CMQNEW: %hhu", &mqttId);
    }

    // Подключиться
    if (user && user[0]) {
        std::snprintf(cmd, sizeof(cmd),
            "+CMQCON=%hhu,3,\"%s\",60,1,0,\"%s\",\"%s\"",
            mqttId, clientId, user, pass ? pass : "");
    } else {
        std::snprintf(cmd, sizeof(cmd),
            "+CMQCON=%hhu,3,\"%s\",60,0",
            mqttId, clientId);
    }

    if (sendCommand(cmd, r, sizeof(r), 15000) != GsmStatus::Ok) {
        DBG.error("A7670C MQTT: CMQCON failed");
        return GsmStatus::MqttErr;
    }

    DBG.info("A7670C MQTT: подключён к %s:%u", broker, port);
    return GsmStatus::Ok;
}

GsmStatus A7670C::mqttPublish(const char* topic, const char* payload, uint16_t len)
{
    char r[256], cmd[512];
    std::snprintf(cmd, sizeof(cmd),
        "+CMQPUB=0,\"%s\",1,0,0,%u,\"%.*s\"",
        topic, (unsigned)len, (int)len, payload);

    if (sendCommand(cmd, r, sizeof(r), 10000) != GsmStatus::Ok) {
        DBG.error("A7670C MQTT: CMQPUB failed");
        return GsmStatus::MqttErr;
    }
    return GsmStatus::Ok;
}
void A7670C::mqttDisconnect()
{
    char r[64];
    sendCommand("+CMQDISCON=0", r, sizeof(r), 5000);
    DBG.info("A7670C MQTT: отключён");
}
