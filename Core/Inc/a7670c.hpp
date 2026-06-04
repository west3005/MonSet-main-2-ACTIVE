/**
 * @file a7670c.hpp
 * @brief Драйвер A7670C LTE Cat-1 / GSM.
 *
 * Наследует Air780E — совместим с MqttClient, Webhook, Air780eTls.
 * Переопределяет методы под AT-команды A7670C:
 *   powerOn/powerOff/hardReset — другой PWRKEY (1500 мс) и URC "ME PDN ACT"
 *   init        — CREG+CEREG, autobaud, фиксация скорости
 *   httpPost    — AT+CSOC/CSOCON/CSOSEND/CSORCVDATA (не HTTPINIT)
 *   disconnect  — AT+CSOCL + CGACT=0
 *
 * AT-команды: A76XX Series AT Command Manual
 */
#pragma once
#include "air780e.hpp"

class A7670C : public Air780E {
public:
    A7670C(UART_HandleTypeDef* uart,
           GPIO_TypeDef*        pwrPort,
           uint16_t             pwrPin);

    // --- Переопределяем под A7670C ---
    void      powerOn();
    void      powerOff();
    void      hardReset();

    GsmStatus init();
    void      disconnect();

    uint16_t  httpPost(const char* url, const char* json, uint16_t len);

    uint8_t   getSignalQuality();

    // --- Публичные методы для A7670CTls ---
    void      sendRaw_pub(const char* data, uint16_t len) { sendRaw(data, len); }
    uint16_t  waitFor_pub(char* buf, uint16_t bsize,
                          const char* expected, uint32_t timeout)
              { return waitFor(buf, bsize, expected, timeout); }
    void      flushRx_pub() { g_air780_rxbuf.clear(); }

    /// waitFor без 50мс ограничения простоя — для асинхронных URC типа +CIPOPEN.
    /// Читает до нахождения expected, ERROR или истечения timeout.
    uint16_t  waitForUrc_pub(char* buf, uint16_t bsize,
                              const char* expected, uint32_t timeout)
    {
        if (!buf || !expected || bsize < 2) return 0;
        uint16_t idx   = 0;
        uint32_t start = HAL_GetTick();
        std::memset(buf, 0, bsize);
        while (idx < (uint16_t)(bsize - 1) && (HAL_GetTick() - start) < timeout) {
            uint8_t ch;
            if (g_air780_rxbuf.pop(ch)) {
                buf[idx++] = static_cast<char>(ch);
                buf[idx]   = '\0';
                if (std::strstr(buf, expected))    break;
                if (std::strstr(buf, "+CME ERROR")) break;
                if (std::strstr(buf, "ERROR\r\n")) break;
            } else {
                HAL_Delay(1);
            }
            IWDG->KR = 0xAAAA;
        }
        buf[idx] = '\0';
        return idx;
    }

    static constexpr uint8_t HTTP_SOCK_IDX = 0;
    static constexpr uint8_t TLS_SOCK_IDX  = 1;

    // --- MQTT (переопределяем под A7670C AT+CMQNEW/CMQCON/CMQPUB) ---
    GsmStatus mqttConnect(const char* broker, uint16_t port,
                          const char* clientId,
                          const char* user, const char* pass);
    GsmStatus mqttPublish(const char* topic, const char* payload, uint16_t len);
    void      mqttDisconnect();

private:
    bool      waitRdyA7670(uint32_t timeoutMs);
    GsmStatus activatePdnA7670();
};
