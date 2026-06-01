/**
 * ================================================================
 * @file    air780e.hpp
 * @brief   Драйвер Air780E 4G CAT1 модема.
 *
 * Поддерживаемые протоколы:
 *   HTTP POST  — AT+HTTPINIT / HTTPPARA / HTTPDATA / HTTPACTION
 *   MQTT       — AT+CMQTTSTART / CMQTTACCQ / CMQTTCONNECT / CMQTTPUB
 *
 * Аппаратная привязка (board_pins.hpp):
 *   UART    : huart2 (PA2/PA3, 115200 8N1)
 *   PWR_EN  : PIN_CELL_PWR_EN  (PC2, PP)
 *   PWRKEY  : PIN_CELL_PWRKEY  (PD6, OD, active-low)
 *   RESET   : PIN_CELL_RESET   (PD7, OD, active-low)
 *   STATUS  : PIN_CELL_STATUS  (PD9, вход, HIGH=включён)
 *
 * PWRKEY (Air780E Hardware Design Manual v1.2.4):
 *   Включение : LOW ≥ 1000 мс
 *   Выключение: LOW ≥ 1500 мс
 * ================================================================
 */
#pragma once

#include "main.h"
#include "config.hpp"
#include "runtime_config.hpp"
#include <cstdint>
#include <cstddef>
#include "gsm_common.hpp"
#include "uart_ringbuf.hpp"

// ================================================================
// Air780E
// ================================================================
class Air780E {
public:
    Air780E(UART_HandleTypeDef* uart,
            GPIO_TypeDef*        pwrPort,
            uint16_t             pwrPin);

    // ---- Питание ----
    void powerOn();
    void powerOff();
    void hardReset();

    // ---- Инициализация ----
    GsmStatus init();
    void      disconnect();

    // ---- HTTP POST ----
    uint16_t httpPost(const char* url, const char* json, uint16_t len);

    // ---- MQTT ----
    GsmStatus mqttConnect(const char* broker, uint16_t port);
    GsmStatus mqttPublish(const char* topic, const char* payload, uint8_t qos = 1);
    void      mqttDisconnect();

    // ---- Утилиты ----
    uint8_t   getSignalQuality();
    GsmStatus sendCommand(const char* cmd, char* resp,
                          uint16_t rsize, uint32_t timeout);

    // ---- Публичный низкоуровневый доступ (для TLS BIO) ----
    void     sendRaw_pub(const char* data, uint16_t len) { sendRaw(data, len); }
    uint16_t waitFor_pub(char* buf, uint16_t bsize,
                         const char* expected, uint32_t timeout)
    { return waitFor(buf, bsize, expected, timeout); }
    /** Сбросить RX-буфер — вызывать перед новой AT-командой через sendRaw_pub */
    void     flushRx_pub() { g_air780_rxbuf.clear(); }

protected:
    UART_HandleTypeDef* m_uart;
    GPIO_TypeDef*       m_pwrPort;
    uint16_t            m_pwrPin;
    bool                m_mqttStarted = false;

    static constexpr uint8_t MQTT_IDX = 0;

    // ---- Низкоуровневый I/O ----
    void     sendRaw(const char* data, uint16_t len);
    uint16_t readResponse(char* buf, uint16_t bsize, uint32_t timeout);
    uint16_t waitFor(char* buf, uint16_t bsize,
                     const char* expected, uint32_t timeout);
    bool     waitRdy(uint32_t timeoutMs);

    // ---- Инициализация сети ----
    GsmStatus activatePdn();

    // ---- Вспомогательная для синхронизации времени ----
    bool     waitForValidTime(uint32_t timeoutMs);
};

// ================================================================
// Алиас для обратной совместимости с main.cpp
// ================================================================
using SIM7020C = Air780E;
