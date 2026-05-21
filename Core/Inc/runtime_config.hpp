/**
 * ================================================================
 * @file    runtime_config.hpp
 * @brief   Runtime configuration loaded from JSON on SD card.
 *
 * @note    All fields have sane defaults from config.hpp.
 *          Extended with channel management, MQTT, webhook,
 *          modbus map, per-UART config, battery, web auth,
 *          Iridium settings, and structured sub-configs.
 * ================================================================
 */
#pragma once
#include "config.hpp"
#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cstddef>

/// Maximum entries in the legacy modbus register map
static constexpr uint8_t MAX_MODBUS_ENTRIES = 16;

/// Maximum channels in the priority chain
static constexpr uint8_t MAX_CHAIN_ORDER = 4;

/// Maximum UART ports configurable
static constexpr uint8_t MAX_UART_PORTS = 3;

/// Maximum physical RS-485 (Modbus RTU) ports
static constexpr uint8_t MAX_RTU_PORTS = 3;

// ================================================================
// Protocol selection
// ================================================================
/**
 * @brief Selects the telemetry transmission protocol.
 *
 * @note HTTPS_GENERIC (value 0) — generic HTTPS for any server. Backward-compatible JSON storage.
 *       Numeric values are preserved for backward-compatible JSON/EEPROM
 *       storage.
 */
enum class ProtocolMode : uint8_t {
    HTTPS_GENERIC    = 0, ///< Generic HTTPS POST to any server
    MQTT_GENERIC      = 1, ///< Generic MQTT broker
    WEBHOOK_HTTP      = 2, ///< Arbitrary HTTP webhook
    MQTT_THINGSBOARD  = 3  ///< MQTT with TB-style topic (legacy)
};

// ================================================================
// Webhook trigger mode
// ================================================================
/**
 * @brief Determines when the webhook fires.
 */
enum class WebhookTrigger : uint8_t {
    Event    = 0, ///< Triggered on a data event
    Schedule = 1  ///< Triggered on a fixed schedule
};

// ================================================================
// Per-UART port configuration (backward compat — do not remove)
// ================================================================
/**
 * @brief Legacy per-UART port configuration.
 *
 * Kept for backward compatibility with existing call sites.
 * New code should prefer ModbusRtuPortConfig.
 */
struct UartPortCfg {
    uint32_t baud     = 9600; ///< Baud rate in bps
    uint8_t  parity   = 2;    ///< 0=none, 1=odd, 2=even
    uint8_t  stopbits = 2;    ///< Stop bits: 1 or 2
    uint8_t  mode     = 0;    ///< 0=disabled, 1=modbus, 2=raw
};

// ================================================================
// Modbus register map entry (backward compat — used by sensor_reader)
// ================================================================
/**
 * @brief One entry in the legacy flat modbus register map.
 *
 * Used by sensor_reader; do not remove.
 */
struct ModbusRegEntry {
    uint8_t  port_idx    = 0;     ///< Physical port: 0=USART3, 1=UART4, 2=UART5
    uint8_t  slave_id    = 1;     ///< Modbus slave address
    uint8_t  function    = 4;     ///< Function code: 3 or 4
    uint16_t start_reg   = 0;     ///< First register address
    uint16_t count       = 1;     ///< Number of registers to read
    uint8_t  data_type   = 0;     ///< 0=int16, 1=uint32, 2=float
    float    scale       = 1.0f;  ///< Multiplicative scale applied to raw value
    float    zero_offset = 0.0f;  ///< Additive offset applied after scale
    float    multiplier  = 1.0f;  ///< Additional multiplier (legacy field)
    char     unit[16]    {};      ///< Engineering unit string
    char     name[32]    {};      ///< Human-readable channel name
};

// ================================================================
// ChannelConfig — transmission channel settings
// ================================================================
/**
 * @brief Controls which physical transmission channels are active and
 *        how the failover chain behaves.
 */
struct ChannelConfig {
    bool     eth_enabled          = true;         ///< Enable W5500 Ethernet channel
    bool     gsm_enabled          = true;         ///< Enable GSM/GPRS channel
    bool     wifi_enabled         = false;        ///< Enable Wi-Fi channel
    bool     iridium_enabled      = false;        ///< Enable Iridium satellite channel
    bool     chain_mode           = true;         ///< true = try channels in priority order
    uint8_t  chain_order[4]       = {0, 1, 2, 3}; ///< 0=ETH, 1=GSM, 2=WIFI, 3=IRIDIUM
    uint8_t  chain_count          = 4;            ///< Number of active entries in chain_order
    uint16_t channel_timeout_s    = 30;           ///< Per-channel attempt timeout in seconds
    uint16_t channel_retry_s      = 300;          ///< Cooldown before retrying a failed channel (s)
    uint8_t  channel_max_retries  = 3;            ///< Maximum consecutive retries per channel
};

// ================================================================
// MeasurementConfig
// ================================================================
/**
 * @brief Measurement scheduling and averaging parameters.
 */
struct MeasurementConfig {
    uint16_t poll_interval_s     = 60;     ///< Sensor poll period in seconds
    uint16_t send_interval_s     = 300;    ///< Telemetry transmit period in seconds
    uint16_t backup_retry_s      = 600;    ///< Retry period after a failed send (s)
    uint8_t  avg_count           = 5;      ///< Number of samples averaged per reading
    bool     deep_sleep_enabled  = false;  ///< Enter deep sleep between polls
    uint16_t deep_sleep_s        = 3600;   ///< Deep sleep duration in seconds
    bool     schedule_enabled    = false;  ///< Restrict operation to a daily window
    char     schedule_start[6]   = "06:00"; ///< Window open time  — format HH:MM
    char     schedule_stop[6]    = "22:00"; ///< Window close time — format HH:MM
};

// ================================================================
// WebConfig
// ================================================================
/**
 * @brief Configuration for the embedded HTTP configuration portal.
 */
struct WebConfig {
    char     web_user[32]         = "admin"; ///< HTTP Basic-Auth username
    char     web_pass[32]         = "monset"; ///< HTTP Basic-Auth password
    bool     web_auth_enabled     = true;    ///< Require authentication to access web UI
    uint16_t web_port             = 80;      ///< TCP port the web server listens on
    uint16_t web_idle_timeout_s   = 30;      ///< Close idle connection after this many seconds
    bool     web_exclusive_mode   = true;    ///< Suspend telemetry while web session is active
};

// ================================================================
// ProtocolConfig
// ================================================================
/**
 * @brief Protocol-level settings; the active sub-section is selected by
 *        the @p mode field.
 */
struct ProtocolConfig {
    ProtocolMode mode = ProtocolMode::HTTPS_GENERIC;    ///< Active protocol

    // --- MQTT broker settings ---
    char     mqtt_host[64]  = "";                          ///< Broker hostname or IP
    uint16_t mqtt_port      = 1883;                        ///< Broker TCP port
    char     mqtt_user[32]  = "";                          ///< MQTT username (empty = anonymous)
    char     mqtt_pass[32]  = "";                          ///< MQTT password
    char     mqtt_topic[64] = "v1/devices/me/telemetry";  ///< Publish topic
    uint8_t  mqtt_qos       = 1;                           ///< QoS level: 0, 1, or 2
    bool     mqtt_tls       = false;                       ///< Enable TLS for MQTT connection

    // --- Universal HTTPS server ---
    char     server_host[64]  = "";             ///< Hostname, e.g. "monset.ru"
    char     server_token[128]= "";             ///< Auth token (X-Device-Token header)
    char     server_path[128] = "/api/ingest";  ///< URL path
    uint16_t server_port      = 443;            ///< TCP port

    // --- Legacy server aliases (backward compat) ---
    char     tb_host[64]   = "";  ///< @deprecated use server_host
    char     tb_token[64]  = "";  ///< @deprecated use server_token
    uint16_t tb_port       = 443; ///< @deprecated use server_port

    // --- Webhook (WEBHOOK_HTTP) ---
    char     webhook_url[128]  = "";     ///< Full URL for the webhook endpoint
    char     webhook_method[8] = "POST"; ///< HTTP method: GET or POST
};

// ================================================================
// TimeConfig
// ================================================================
/**
 * @brief NTP and timezone settings.
 */
struct TimeConfig {
    bool   ntp_enabled     = true;          ///< Enable NTP time synchronisation
    char   ntp_server[64]  = "pool.ntp.org"; ///< NTP server hostname or IP
    int8_t timezone_offset = 3;             ///< UTC offset in whole hours (e.g. 3 = UTC+3 Moscow)
};

// ================================================================
// AlertConfig
// ================================================================
/**
 * @brief Threshold-based alerting configuration.
 */
struct AlertConfig {
    bool  alerts_enabled               = false;  ///< Master enable for alert processing
    float battery_low_threshold_pct    = 20.0f;  ///< Battery % below which a low-battery alert fires
    float sensor_min[4]                = {0.0f, 0.0f, 0.0f, 0.0f}; ///< Per-channel low-limit (index 0..3)
    float sensor_max[4]                = {0.0f, 0.0f, 0.0f, 0.0f}; ///< Per-channel high-limit (index 0..3)
    bool  alert_on_channel_fail        = true;   ///< Fire alert when a transmission channel fails
    bool  alert_on_sensor_fail         = true;   ///< Fire alert when a sensor read fails
    char  alert_webhook_url[128]       = "";     ///< Webhook URL for alert delivery
};

// ================================================================
// ModbusDeviceCfg — one sensor on a RTU port
// ================================================================
/**
 * @brief Describes a single Modbus RTU slave device polled on a port.
 */
struct ModbusDeviceCfg {
    bool     enabled       = false; ///< Include this device in the poll cycle
    uint8_t  slave_addr    = 1;     ///< Modbus slave address (1..247)
    char     name[32]      = "";    ///< Human-readable device/channel name
    char     unit[16]      = "";    ///< Engineering unit string (e.g. "°C")
    uint16_t reg_start     = 0x0000; ///< First register address to read
    uint8_t  reg_count     = 2;     ///< Number of consecutive registers to read
    uint8_t  func_code     = 3;     ///< Modbus function: 3=Read Holding, 4=Read Input
    uint8_t  data_type     = 0;     ///< 0=INT16, 1=UINT16, 2=INT32_BE, 3=UINT32_BE, 4=FLOAT32_BE
    float    scale         = 1.0f;  ///< Multiplicative scale applied to raw register value
    float    offset        = 0.0f;  ///< Additive offset applied after scale
    uint8_t  channel_idx   = 0;     ///< Destination channel index in the telemetry payload
};

// ================================================================
// ModbusRtuPortConfig — one physical UART port
// ================================================================
/**
 * @brief Configuration for one physical RS-485 / Modbus RTU port.
 */
struct ModbusRtuPortConfig {
    bool     enabled             = false;    ///< Enable polling on this port
    char     uart_name[8]        = "USART3"; ///< Peripheral name used for logging ("USART3", "UART4", …)
    uint32_t baudrate            = 9600;     ///< Baud rate in bps
    uint8_t  data_bits           = 8;        ///< Data bits per frame (typically 8)
    uint8_t  stop_bits           = 1;        ///< Stop bits: 1 or 2
    uint8_t  parity              = 0;        ///< 0=None, 1=Even, 2=Odd
    uint16_t response_timeout_ms = 500;      ///< Wait time for slave response in ms
    uint16_t inter_frame_ms      = 10;       ///< Silent gap between frames in ms

    static constexpr uint8_t MAX_DEVICES = 8; ///< Maximum slave devices on one port
    ModbusDeviceCfg devices[MAX_DEVICES];      ///< Device configurations
    uint8_t         device_count = 0;          ///< Number of valid entries in @p devices
};

// ================================================================
// ModbusTcpDeviceCfg — one Modbus TCP slave device to poll
// ================================================================
/**
 * @brief Describes a single Modbus TCP slave device to poll as a master.
 */
struct ModbusTcpDeviceCfg {
    bool     enabled            = false;         ///< Include this device in the poll cycle
    char     ip[16]             = "192.168.1.200"; ///< IPv4 address of the slave (dotted-decimal)
    uint16_t port               = 502;           ///< TCP port of the slave (default 502)
    uint8_t  unit_id            = 1;             ///< Modbus unit identifier
    char     name[32]           = "";            ///< Human-readable device/channel name
    char     unit[16]           = "";            ///< Engineering unit string
    uint16_t reg_start          = 0x0000;        ///< First register address
    uint8_t  reg_count          = 2;             ///< Number of consecutive registers
    uint8_t  func_code          = 3;             ///< Modbus function: 3=Read Holding, 4=Read Input
    uint8_t  data_type          = 4;             ///< 0=INT16, 1=UINT16, 2=INT32_BE, 3=UINT32_BE, 4=FLOAT32_BE
    float    scale              = 1.0f;          ///< Multiplicative scale applied to raw value
    float    offset             = 0.0f;          ///< Additive offset applied after scale
    uint8_t  channel_idx        = 0;             ///< Destination channel index in telemetry payload
    uint16_t poll_timeout_ms    = 1000;          ///< Read operation timeout in ms
    uint16_t connect_timeout_ms = 3000;          ///< TCP connection timeout in ms
    uint8_t  w5500_socket       = 2;             ///< W5500 hardware socket index (0..7)
};

// ================================================================
// ModbusTcpMasterConfig
// ================================================================
/**
 * @brief Groups all Modbus TCP master polling parameters.
 */
struct ModbusTcpMasterConfig {
    bool    enabled = false;                           ///< Enable the Modbus TCP master
    static constexpr uint8_t MAX_TCP_DEVICES = 8;     ///< Maximum TCP slave devices
    ModbusTcpDeviceCfg devices[MAX_TCP_DEVICES];       ///< Per-device configurations
    uint8_t device_count = 0;                          ///< Number of valid entries in @p devices
};

// ================================================================
// ModbusTcpSlaveRegEntry — one register in the slave register map
// ================================================================
/**
 * @brief Maps one holding register in the device's Modbus TCP slave
 *        register table to an internal data source.
 */
struct ModbusTcpSlaveRegEntry {
    uint16_t reg_addr;    ///< Holding register address (0-based)
    uint8_t  source;      ///< Data source: 0..3=sensor channel, 4=battery_pct, 5=battery_v, 6=channel_status
    uint8_t  data_type;   ///< Encoding: 0=INT16, 1=UINT16, 4=FLOAT32 (occupies 2 registers)
    float    scale;       ///< Multiplier applied before integer encoding
    char     name[32];    ///< Human-readable register name
    char     unit[16];    ///< Engineering unit string
};

// ================================================================
// ModbusTcpSlaveConfig
// ================================================================
/**
 * @brief Configuration for the on-device Modbus TCP slave (server).
 */
struct ModbusTcpSlaveConfig {
    bool     enabled               = false; ///< Enable the Modbus TCP slave
    uint16_t listen_port           = 502;   ///< TCP port to listen on
    uint8_t  w5500_socket          = 3;     ///< W5500 hardware socket index (0..7)
    uint8_t  unit_id               = 1;     ///< Modbus unit identifier returned in responses
    uint16_t connection_timeout_ms = 5000;  ///< Close inactive connection after this many ms

    static constexpr uint8_t MAX_REGS = 32;  ///< Maximum register map entries
    ModbusTcpSlaveRegEntry reg_map[MAX_REGS]; ///< Register definitions
    uint8_t reg_count = 0;                    ///< Number of valid entries in @p reg_map
};

// ================================================================
// RuntimeConfig
// ================================================================
/**
 * @brief Top-level runtime configuration structure.
 *
 * Persisted as JSON on the SD card. All fields carry sane defaults
 * so the device is operational immediately after a factory reset.
 * VERSION must be bumped whenever the layout changes in a
 * backward-incompatible way.
 */
struct RuntimeConfig
{
    static constexpr uint32_t VERSION = 2; ///< Schema version; mismatch causes reset to defaults

    // --- telemetry ids ---
    bool complex_enabled = false;  ///< Enable complex/aggregate metric upload
    char metric_id[64]   {};       ///< Primary metric identifier sent to the server
    char complex_id[64]  {};       ///< Complex metric identifier (used when complex_enabled=true)

    // --- server ---
    char server_url[192]       {}; ///< Base URL of the telemetry server (legacy field)
    char server_auth_b64[128]  {}; ///< HTTP Basic Auth credentials, Base64-encoded

    // --- TLS (GSM / HTTPS) ---
    /// CA certificate in PEM format for HTTPS over GSM (A7670CTls).
    /// Leave empty to use VERIFY_NONE (encrypted but no server authentication).
    char tls_ca_cert[2048] {}; ///< CA certificate PEM (empty = VERIFY_NONE)

    // --- ETH (W5500) ---
    /**
     * @brief Selects static IP or DHCP assignment for the W5500 interface.
     */
    enum class EthMode : uint8_t {
        Static = 0, ///< Use statically configured IP parameters
        Dhcp   = 1  ///< Obtain IP via DHCP
    };
    EthMode eth_mode = EthMode::Static; ///< IP address assignment mode

    uint8_t w5500_mac[6] {}; ///< W5500 MAC address (6 bytes)
    uint8_t eth_ip[4]    {}; ///< Static IPv4 address
    uint8_t eth_sn[4]    {}; ///< Subnet mask
    uint8_t eth_gw[4]    {}; ///< Default gateway
    uint8_t eth_dns[4]   {}; ///< DNS server address

    // --- GSM ---
    char gsm_apn[32]  {}; ///< GSM/GPRS APN string
    char gsm_user[32] {}; ///< APN authentication username (empty = none)
    char gsm_pass[32] {}; ///< APN authentication password

    // --- timing (legacy flat fields) ---
    uint32_t poll_interval_sec   = 5; ///< Legacy poll interval in seconds (superseded by meas.poll_interval_s)
    uint32_t send_interval_polls = 2; ///< Legacy send cadence in poll counts (superseded by meas.send_interval_s)

    // --- Legacy Modbus (backward compat) ---
    uint8_t  modbus_slave     = 1; ///< Default slave address for single-device setups
    uint8_t  modbus_func      = 4; ///< Default function code (3 or 4)
    uint16_t modbus_start_reg = 0; ///< Default first register address
    uint16_t modbus_num_regs  = 2; ///< Default register count

    // --- Legacy sensor scale ---
    float sensor_zero_level = 0.0f;    ///< Zero-level offset for legacy sensor scaling
    float sensor_divider    = 1000.0f; ///< Divisor for legacy sensor scaling

    // --- time / NTP (legacy flat fields) ---
    bool     ntp_enabled    = true;  ///< Enable NTP (legacy; superseded by time_cfg.ntp_enabled)
    char     ntp_host[64]   {};      ///< NTP server hostname (legacy; superseded by time_cfg.ntp_server)
    uint32_t ntp_resync_sec = 86400; ///< NTP resynchronisation period in seconds

    // --- Channel enable/disable (legacy flat flags) ---
    bool eth_enabled     = true;  ///< Ethernet enabled (legacy; superseded by channels.eth_enabled)
    bool gsm_enabled     = true;  ///< GSM enabled      (legacy; superseded by channels.gsm_enabled)
    bool wifi_enabled    = false; ///< Wi-Fi enabled    (legacy; superseded by channels.wifi_enabled)
    bool iridium_enabled = false; ///< Iridium enabled  (legacy; superseded by channels.iridium_enabled)

    // --- Priority chain (legacy flat fields) ---
    bool    chain_enabled               = false;         ///< Enable failover chain (legacy)
    uint8_t chain_order[MAX_CHAIN_ORDER] = {0, 1, 2, 3}; ///< Channel priority order (legacy)
    uint8_t chain_count                 = 2;             ///< Active entries in chain_order (legacy)

    // --- MQTT broker (legacy flat fields) ---
    char     mqtt_host[128] {}; ///< Broker hostname (legacy; superseded by proto.mqtt_host)
    uint16_t mqtt_port      = 1883; ///< Broker port (legacy; superseded by proto.mqtt_port)
    char     mqtt_user[64]  {}; ///< Broker username (legacy)
    char     mqtt_pass[64]  {}; ///< Broker password (legacy)
    char     mqtt_topic[128]{}; ///< Publish topic   (legacy)
    uint8_t  mqtt_qos       = 1;    ///< QoS level (legacy)
    bool     mqtt_tls       = false; ///< Enable TLS (legacy)

    // --- Protocol selection (legacy flat field) ---
    ProtocolMode protocol = ProtocolMode::HTTPS_GENERIC;    ///< Active protocol (legacy)

    // --- HTTP Webhook (legacy flat fields) ---
    char           webhook_url[192]              {}; ///< Webhook endpoint URL (legacy)
    char           webhook_method[8]             = "POST"; ///< HTTP method (legacy)
    char           webhook_headers[256]          {}; ///< Custom HTTP headers (legacy)
    char           webhook_payload_template[256] {}; ///< Payload template string (legacy)
    WebhookTrigger webhook_trigger               = WebhookTrigger::Event; ///< Trigger mode (legacy)
    uint32_t       webhook_interval_sec          = 300; ///< Webhook send interval (legacy)

    // --- Per-UART port config (legacy flat array) ---
    UartPortCfg uart_ports[MAX_UART_PORTS]; ///< Legacy per-port UART settings

    // --- Modbus register map (legacy flat array) ---
    ModbusRegEntry modbus_map[MAX_MODBUS_ENTRIES]; ///< Legacy flat register map
    uint8_t        modbus_map_count = 0;            ///< Number of valid entries in modbus_map

    // --- Averaging (legacy) ---
    uint8_t avg_count = 1; ///< Sample average count (legacy; superseded by meas.avg_count)

    // --- Backup send interval (legacy) ---
    uint32_t backup_send_interval_sec = 600; ///< Backup retry period (legacy)

    // --- Battery threshold (legacy) ---
    uint8_t battery_low_pct = 20; ///< Battery low % threshold (legacy; superseded by alerts.battery_low_threshold_pct)

    // --- Web auth (legacy flat fields) ---
    // --- WiFi (legacy flat fields) ---
    char wifi_ssid[64] {}; ///< Wi-Fi SSID
    char wifi_pass[64] {}; ///< Wi-Fi passphrase

    // ================================================================
    // Structured sub-configs (new)
    // ================================================================

    uint8_t config_version = 2; ///< Schema version — mismatch triggers reset to defaults

    WebConfig           web;       ///< Embedded web portal settings
    ChannelConfig       channels;  ///< Transmission channel and failover settings
    MeasurementConfig   meas;      ///< Measurement scheduling and averaging
    ProtocolConfig      proto;     ///< Protocol selection and connection parameters
    TimeConfig          time_cfg;  ///< NTP and timezone settings ('time' is a reserved identifier)
    AlertConfig         alerts;    ///< Threshold alerting configuration

    ModbusRtuPortConfig rtu_ports[MAX_RTU_PORTS]; ///< Per-port Modbus RTU configuration

    ModbusTcpMasterConfig tcp_master; ///< Modbus TCP master (client) configuration
    ModbusTcpSlaveConfig  tcp_slave;  ///< Modbus TCP slave (server) configuration

    // ================================================================
    // Methods
    // ================================================================

    /// @brief Fill defaults from compile-time Config::... constants.
    void setDefaultsFromConfig();

    /// @brief Clamp or correct all fields that are out of valid range.
    /// @return true if no corrections were needed, false if any value was fixed.
    bool validateAndFix();

    /// @brief Check all parameter ranges without modifying any field.
    /// @return true if every value is within its valid range, false otherwise.
    bool validate() const;

    /// @brief Load configuration from a JSON file on the SD card.
    /// @param filename  Path to the JSON file (e.g. "runtime_config.json").
    /// @return true on success.
    bool loadFromSd(const char* filename);

    /// @brief Serialise the current configuration to a JSON file on the SD card.
    /// @param filename  Destination file path.
    /// @return true on success.
    bool saveToSd(const char* filename) const;

    /// @brief Populate fields by parsing a JSON string in memory.
    /// @param json  Pointer to a null-terminated JSON buffer.
    /// @param len   Length of the JSON buffer in bytes.
    /// @return true on success.
    bool loadFromJson(const char* json, size_t len);

    /// @brief Log all current configuration parameters via the debug UART.
    void log() const;

    /// @brief Build the effective HTTPS URL from server_host/server_path/server_port
    ///        or fall back to legacy server_url if set.
    void buildServerUrl(char* out, size_t outSz) const;
};

/// @brief Access the singleton RuntimeConfig instance.
/// @return Reference to the global Cfg singleton.
RuntimeConfig& Cfg();
RuntimeConfig& CfgBackup();  ///< Second singleton used as scratch/rollback buffer in handlePostConfig

/// Default filename used for SD card persistence.
constexpr const char* RUNTIME_CONFIG_FILENAME = "0:/runtime_config.json";
