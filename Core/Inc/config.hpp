/**
 * ================================================================
 * @file    config.hpp
 * @brief   Константы времени компиляции (железо, сеть, буферы).
 *          Изменяемые в рантайме параметры — в runtime_config.hpp.
 * ================================================================
 */
#pragma once

#include "stm32f4xx_hal.h"
#include "board_pins.hpp"

#ifdef __cplusplus
#include <cstdint>
#include <cstddef>
#else
#include <stdint.h>
#include <stddef.h>
#endif

// ================================================================
// GPIO алиасы (обратная совместимость с устаревшим кодом)
// Источник истины: board_pins.hpp
// ================================================================
#ifndef PIN_LED_PORT
#define PIN_LED_PORT  PIN_STATUS_LED_PORT
#endif
#ifndef PIN_LED_PIN
#define PIN_LED_PIN   PIN_STATUS_LED_PIN
#endif

#ifndef PIN_SIM_PWR_PORT
#define PIN_SIM_PWR_PORT  PIN_CELL_PWR_EN_PORT
#endif
#ifndef PIN_SIM_PWR_PIN
#define PIN_SIM_PWR_PIN   PIN_CELL_PWR_EN_PIN
#endif

#ifndef PIN_NET_SW_PORT
#define PIN_NET_SW_PORT  PIN_NET_SELECT_PORT
#endif
#ifndef PIN_NET_SW_PIN
#define PIN_NET_SW_PIN   PIN_NET_SELECT_PIN
#endif

namespace Config {

// ================================================================
// W5500 Ethernet
// ================================================================
constexpr uint8_t  W5500_MAC[6]           = {0x02, 0x30, 0x05, 0x00, 0x00, 0x01};

enum class NetMode : uint8_t { DHCP = 0, Static = 1 };
constexpr NetMode  NET_MODE               = NetMode::Static;

constexpr uint8_t  NET_IP[4]              = {192, 168, 31, 122};
constexpr uint8_t  NET_SN[4]              = {255, 255, 255,   0};
constexpr uint8_t  NET_GW[4]              = {192, 168, 31,   1};
constexpr uint8_t  NET_DNS[4]             = {192, 168, 31,   1};

constexpr uint32_t W5500_DHCP_TIMEOUT_MS  = 8000;

// ================================================================
// Сеть: DNS / HTTP / HTTPS
// ================================================================
constexpr uint32_t DNS_TIMEOUT_MS         = 5000;
constexpr uint16_t DNS_BUFFER_SIZE        = 512;

constexpr uint32_t HTTP_POST_TIMEOUT_MS   = 15000;
constexpr uint32_t HTTPS_POST_TIMEOUT_MS  = 20000;
constexpr uint16_t HTTP_LOCAL_PORT        = 50000;

// ================================================================
// Периферия
// ================================================================
constexpr uint8_t  DS3231_ADDR            = 0x68 << 1;

// ================================================================
// UART назначения (источник истины — board_pins.hpp и MonSet.ioc)
// ================================================================
#define UART_DEBUG    huart1   ///< Debug UART → Serial Monitor (PA9/PA10, 115200)
#define UART_GSM      huart2   ///< GSM модем A7670C / Air780E (PA2/PA3, 115200)
// Датчиковые порты — настраиваются в runtime_config.json / веб-интерфейсе
#define UART_PORT_0   huart3   ///< Датчик порт 0 — USART3 (PB10/PB11, RS-485 DE=PB12)
#define UART_PORT_1   huart4   ///< Датчик порт 1 — UART4  (PC10/PC11, RS-485/RS-232)
#define UART_PORT_2   huart5   ///< Датчик порт 2 — UART5  (PC12/PD2,  RS-485/RS-232 через MAX3232)
#define UART_WIFI     huart6   ///< ESP8266/ESP32 WiFi (PC6/PC7, 115200)

// ================================================================
// Дефолты датчиковых портов (применяются в setDefaultsFromConfig)
// ================================================================
// Порт 0 (USART3) — дефолтный Modbus RTU, обратная совместимость с текущим конфигом
constexpr uint32_t PORT0_DEFAULT_BAUD      = 9600;
constexpr uint8_t  PORT0_DEFAULT_STOP      = 1;
constexpr uint8_t  PORT0_DEFAULT_PARITY    = 0;     ///< 0=None
constexpr uint8_t  PORT0_DEFAULT_AVG       = 1;
constexpr const char* PORT0_BACKUP_FILE    = "backup_p0.jsn";

// Порт 1 (UART4) — универсальный, по умолчанию выключен
constexpr uint32_t PORT1_DEFAULT_BAUD      = 9600;
constexpr uint8_t  PORT1_DEFAULT_STOP      = 1;
constexpr uint8_t  PORT1_DEFAULT_PARITY    = 0;
constexpr uint8_t  PORT1_DEFAULT_AVG       = 1;
constexpr const char* PORT1_BACKUP_FILE    = "backup_p1.jsn";

// Порт 2 (UART5) — универсальный, по умолчанию выключен
constexpr uint32_t PORT2_DEFAULT_BAUD      = 9600;
constexpr uint8_t  PORT2_DEFAULT_STOP      = 1;
constexpr uint8_t  PORT2_DEFAULT_PARITY    = 0;
constexpr uint8_t  PORT2_DEFAULT_AVG       = 1;
constexpr const char* PORT2_BACKUP_FILE    = "backup_p2.jsn";

// ================================================================
// Modbus RTU
// ================================================================
constexpr uint8_t  MODBUS_SLAVE           = 1;
constexpr uint8_t  MODBUS_FUNC_CODE       = 4;
constexpr uint16_t MODBUS_START_REG       = 0;
constexpr uint16_t MODBUS_NUM_REGS        = 2;
constexpr uint32_t MODBUS_TIMEOUT_MS      = 1000;

// ================================================================
// Масштабирование сенсора
// ================================================================
constexpr float    SENSOR_ZERO_LEVEL      = 0.0f;
constexpr float    SENSOR_DIVIDER         = 1000.0f;

// ================================================================
// ID метрик (телеметрия)
// ================================================================
constexpr const char* METRIC_ID           = "f2656f53-463c-4d66-8ab1-e86fb11549b1";
constexpr const char* COMPLEX_ID          = "21100e69-b08b-45d1-ab1f-0adca0f0f909";

// ================================================================
// GSM / NB-IoT (SIM7020C)
// ================================================================
constexpr const char* GSM_APN             = "internet";
constexpr const char* GSM_APN_USER        = "";
constexpr const char* GSM_APN_PASS        = "";

/// Таймаут одной AT-команды (мс)
constexpr uint32_t SIM7020_CMD_TIMEOUT_MS = 5000;
/// Таймаут ожидания "RDY" после включения питания (мс)
constexpr uint32_t SIM7020_BOOT_MS        = 8000;
/// Таймаут активации PDN (мс)
constexpr uint32_t SIM7020_PDN_TIMEOUT_MS = 30000;
/// Таймаут TCP HTTP-операции (мс)
constexpr uint32_t SIM7020_TCP_TIMEOUT_MS = 20000;
/// Таймаут MQTT-операции (мс)
constexpr uint32_t SIM7020_MQTT_TIMEOUT_MS = 15000;
/// Таймаут TLS-хендшейка + HTTPS-запроса (мс)
/// TLS 1.2 handshake на NB-IoT занимает ~5-15 с (зависит от сигнала)
constexpr uint32_t SIM7020_TLS_TIMEOUT_MS  = 40000;
/// Размер RX-буфера UART модема
constexpr uint16_t GSM_RX_BUF_SIZE        = 512;

// ================================================================
// Сервер (значения по умолчанию; перекрываются runtime_config)
// ================================================================
constexpr const char* SERVER_URL           =
    "";  ///< Default server URL (empty — use server_host/server_path in runtime_config)
constexpr const char* SERVER_AUTH          = "";

// ================================================================
// Циклы опроса / отправки
// ================================================================
constexpr uint32_t POLL_INTERVAL_SEC       = 5;
constexpr uint32_t SEND_INTERVAL_POLLS     = 2;

// ================================================================
// Буферы
// ================================================================
constexpr uint8_t  MEAS_BUFFER_SIZE        = 64;
constexpr uint16_t JSON_BUFFER_SIZE        = 8192;

// ================================================================
// SD / backup
// ================================================================
constexpr const char* BACKUP_FILENAME      = "backup.jsn";
constexpr uint16_t    JSONL_LINE_MAX       = 240;
constexpr uint16_t    HTTP_CHUNK_MAX       = 1800;

} // namespace Config
