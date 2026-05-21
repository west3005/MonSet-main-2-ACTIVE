/**
 * ================================================================
 * @file    app.hpp
 * @brief   Main application class — coordinates all modules.
 *
 * @note    Modes (PIN_MODE_SW):
 *            Debug — LED ON, no STOP mode
 *            Sleep — Stop mode between poll cycles
 *
 *          Data channel managed by ChannelManager with priority
 *          chain: ETH → GSM → WIFI → IRIDIUM.
 *
 *          Supports MQTT and HTTPS protocols, webhook triggers,
 *          multi-sensor Modbus, battery monitoring, web server,
 *          and captive portal setup.
 * ================================================================
 */
#pragma once

#include "config.hpp"
#include "runtime_config.hpp"
#include "debug_uart.hpp"
#include "ds3231.hpp"
#include "modbus_rtu.hpp"
#include "air780e_tls.hpp"
#include "a7670c_tls.hpp"
#include "sd_backup.hpp"
#include "modbus_tcp_master.hpp"
#include "modbus_tcp_slave.hpp"
#include "circular_log.hpp"
#include "sensor_reader.hpp"
#include "data_buffer.hpp"
#include "power_manager.hpp"
#include "channel_manager.hpp"
#include "mqtt_client.hpp"
#include "webhook.hpp"
#include "iridium.hpp"
#include "web_server.hpp"
#include "esp8266.hpp"
#include "captive_portal.hpp"
#include "battery_monitor.hpp"

// ================================================================
// System mode
// ================================================================
enum class SystemMode : uint8_t {
    Sleep = 0,  ///< Stop mode between polls
    Debug = 1,  ///< Always on, LED active
};

// ================================================================
// Link channel (legacy — kept for backward compat)
// ================================================================
enum class LinkChannel : uint8_t {
    Eth = 0,
    Gsm = 1,
};

// ================================================================
// App class
// ================================================================
class App {
public:
    App();

    /** Initialize all subsystems. Called once from main(). */
    void init();

    /** Main infinite loop. */
    [[noreturn]] void run();

private:
    // ---- Hardware modules ----
    DS3231         m_rtc;
    ModbusRTU      m_modbusPort0;  ///< USART3
    ModbusRTU*     m_modbusPorts[3]; ///< Array of port pointers
    A7670C         m_gsm;
    SdBackup       m_sdBackup;
    SensorReader   m_sensor;
    DataBuffer     m_buffer;
    PowerManager   m_power;

    // ---- New modules ----
    ChannelManager m_channelMgr;
    MqttClient     m_mqtt;
    Webhook        m_webhook;
    Iridium        m_iridium;
    WebServer      m_webServer;
    ESP8266        m_esp;
    CaptivePortal  m_captivePortal;
    BatteryMonitor m_battery;

    // ---- State ----
    SystemMode  m_mode        = SystemMode::Sleep;
    uint32_t    m_pollCounter = 0;
    uint32_t    m_lastBackupSendTick = 0;
    bool        m_mqttConnected = false;

    // ---- JSON work buffer ----
    char m_json[Config::JSON_BUFFER_SIZE]{};

    // ---- SD state ----
    bool m_sdOk = false;

    // ---- Alive flags for software watchdog ----
    bool m_sensorAlive  = false;
    bool m_channelAlive = false;

    // ---- Web exclusive mode ----
    bool     m_webActive      = false;  ///< True while HTTP client is connected
    uint32_t m_webLastReqTick = 0;      ///< HAL_GetTick() of last HTTP request
    bool     m_webStartPending = false; ///< Deferred web server start (eth not ready at boot)

    // ---- Test send state ----
    enum class TestSendState : uint8_t { Idle, Reading, Building, Sending, Success, Fail };
    volatile TestSendState m_testState = TestSendState::Idle;
    uint32_t m_testElapsedMs = 0;
    int      m_testHttpCode  = 0;
    char     m_testChannel[16] = "";

    // ---- Channel status for Modbus TCP Slave ----
    uint8_t m_channelStatus = 0; ///< bit0=ETH,bit1=GSM,bit2=WIFI,bit3=IRIDIUM

    // ---- Modbus TCP Master / Slave + readings ----
    ModbusTcpMaster m_tcpMaster;
    ModbusTcpSlave  m_tcpSlave;
    float           m_tcpReadings[ModbusTcpMasterConfig::MAX_TCP_DEVICES] = {};

public:
    /**
     * @brief Notify web activity — call from WebServer on each valid request.
     *        Resets web idle timer. Non-blocking, safe to call from tick().
     */
    void notifyWebActivity();

    /**
     * @brief Get web active flag — true if HTTP session is active.
     */
    bool isWebActive() const { return m_webActive; }

    /**
     * @brief Get remaining idle time in seconds before web mode expires.
     */
    uint16_t webIdleRemainingS() const;

    /**
     * @brief Trigger a test send — non-blocking, result polled via getTestResult().
     */
    void triggerTestSend();

    /**
     * @brief Get test send result for polling.
     * @param state      Output state string: "pending","reading","building","sending","success","fail"
     * @param channel    Output channel name
     * @param httpCode   Output HTTP code (0 if not applicable)
     * @param elapsedMs  Output elapsed time in ms
     */
    void getTestResult(char* state, char* channel, int* httpCode, uint32_t* elapsedMs) const;

    /**
     * @brief Get current channel status bitmask (for Modbus TCP Slave).
     */
    uint8_t getChannelStatus() const { return m_channelStatus; }

private:
    /**
     * @brief Check web idle timeout and clear m_webActive if expired.
     *        Must be called every main loop iteration.
     */
    void checkWebTimeout();

    /**
     * @brief Write measurement to backup.jsn with "src":"web_q" tag.
     * @param payload  JSON string to write
     */
    void writeToBackup(const char* payload);

    /**
     * @brief Execute test send synchronously (called when m_testState==Reading).
     */
    void runTestSend();
    void processTestSend();

    // ---- LED ----
    void ledOn();
    void ledOff();
    void ledBlink(uint8_t count, uint32_t ms);

    // ---- Mode / channel ----
    SystemMode  readMode();
    LinkChannel readChannel();

    // ---- NTP sync ----
    bool syncRtcWithNtpIfNeeded(const char* tag, bool verbose);

    // ---- Data transmission ----
    void transmitBuffer();
    void transmitSingle(float value, const DateTime& dt);
    void retransmitBackup();

    // ---- Channel send callbacks ----
    static int sendViaEth(const char* json, uint16_t len, void* ctx);
    static int sendViaGsm(const char* json, uint16_t len, void* ctx);
    static int sendViaWifi(const char* json, uint16_t len, void* ctx);
    static int sendViaIridium(const char* json, uint16_t len, void* ctx);

    // ---- ETH helpers ----
    bool ensureEthReady();
    int  postViaEth(const char* json, uint16_t len);
    int  postViaGsm(const char* json, uint16_t len);

    // ---- MQTT send ----
    bool sendViaMqtt(const char* json, uint16_t len);

    // ---- JSON builders ----
    int buildPayload(char* buf, size_t bsz,
                     const char* tsStr, float val, const DateTime& dt,
                     bool asArray);
    int buildMultiSensorPayload(char* buf, size_t bsz,
                                 const char* tsStr, const DateTime& dt,
                                 bool asArray);

    // ---- Init helpers ----
    void initModbusPorts();
    void initChannelManager();
};
