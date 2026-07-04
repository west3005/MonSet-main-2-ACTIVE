/**
 * ================================================================
 * @file    sensor_reader.hpp
 * @brief   Multi-sensor, multi-port Modbus reader with averaging,
 *          per-sensor calibration, and data type conversion.
 *
 * @note    Supports USART3, UART4, UART5 via separate ModbusRTU
 *          instances. Falls back to legacy single-sensor mode
 *          when modbus_map_count == 0.
 *
 *          Extended with pollRtuPorts() for multi-device RTU mode
 *          driven by ModbusRtuPortConfig / ModbusDeviceCfg descriptors
 *          (see sensor_reader_ext.cpp).
 * ================================================================
 */
#ifndef SENSOR_READER_HPP
#define SENSOR_READER_HPP

#include "modbus_rtu.hpp"
#include "modbus_map.hpp"
#include "ds3231.hpp"
#include "runtime_config.hpp"
#include <cstdint>

/// Maximum sensor readings stored
static constexpr uint8_t MAX_SENSOR_READINGS = MAX_MODBUS_ENTRIES;

// ================================================================
// SensorReading
// ================================================================

/// A single sensor reading result
struct SensorReading {
    char     name[64]{};         ///< Metric identifier (UUID или имя); 64 = полный UUID (36 символов) + запас
                                 ///< Этап 4: расширено с 32 до 64 — иначе UUID (36 симв.) обрезался
    float    value     = 0.0f;   ///< Calibrated value
    float    raw_value = 0.0f;   ///< Raw value before calibration
    char     unit[16]{};
    bool     valid     = false;
    DateTime timestamp{};
};

// ================================================================
// SensorReader
// ================================================================

class SensorReader {
public:
    /**
     * @brief Construct with references to up to 3 ModbusRTU instances and RTC.
     * @param ports Array of 3 ModbusRTU pointers (USART3, UART4, UART5).
     *              nullptr entries are skipped.
     * @param rtc   DS3231 RTC reference.
     */
    SensorReader(ModbusRTU* ports[3], DS3231& rtc);

    /// @brief Legacy constructor for backward compat (single port)
    SensorReader(ModbusRTU& modbus, DS3231& rtc);

    /// @brief Read all sensors from modbus_map (or legacy single)
    /// @param timestamp  Output: current timestamp from RTC
    /// @return First sensor's calibrated value (legacy compat)
    float read(DateTime& timestamp);

    /// @brief Get reading by index
    const SensorReading& getReading(uint8_t idx) const;

    /// @brief Number of valid readings from last read() cycle
    uint8_t getReadingCount() const { return m_readingCount; }

    /// @brief Get pointer to all readings array
    const SensorReading* getReadings() const { return m_readings; }

    /// @brief Last legacy value (backward compat)
    float lastValue() const { return m_lastValue; }

    /**
     * @brief Poll all devices in rtu_ports config (new multi-device mode).
     *        For each enabled port: iterate devices, call readModbusDevice().
     *        Stores results in m_readings[] at dev.channel_idx.
     * @param rtu_ports  Array of ModbusRtuPortConfig (from RuntimeConfig)
     * @param portCount  Number of ports (up to MAX_RTU_PORTS=3)
     */
    void pollRtuPorts(const ModbusRtuPortConfig* rtu_ports, uint8_t portCount);

    /**
     * @brief Этап 7: true если показание канала было СВЕЖИМ (реально опрошено)
     *        именно в последнем вызове pollRtuPorts()/read(). Используется
     *        в App::run() как гейт перед записью в backup_ch{N}.jsn — иначе
     *        "медленные" датчики (poll_interval_polls > 1) дублировали бы
     *        одно и то же старое значение в бэкап на каждом тике.
     * @param channelIdx Индекс канала (== dev.channel_idx)
     */
    bool isFreshThisCycle(uint8_t channelIdx) const {
        return (channelIdx < MAX_SENSOR_READINGS) ? m_freshThisCycle[channelIdx] : false;
    }

private:
    ModbusRTU* m_ports[3] = {nullptr, nullptr, nullptr};
    DS3231&    m_rtc;
    float      m_lastValue = -9999.0f;

    SensorReading m_readings[MAX_SENSOR_READINGS];
    uint8_t       m_readingCount = 0;

    // Этап 7: персональные счётчики опроса на канал (индекс == channel_idx)
    // и флаг "было ли реально опрошено в этом цикле" (для гейта записи бэкапа).
    uint16_t m_pollCounters[MAX_SENSOR_READINGS]    = {};
    bool     m_freshThisCycle[MAX_SENSOR_READINGS]  = {};

    /// Legacy single-sensor convert (backward compat)
    static float convertLegacy(uint16_t reg0, uint16_t reg1);

    /// Read a single modbus map entry with averaging
    bool readEntry(const ModbusRegEntry& entry, SensorReading& out,
                   const DateTime& ts);

    /// Average N readings of a single register set
    float readWithAveraging(ModbusRTU& port, const ModbusRegEntry& entry,
                            uint8_t avgCount);

    /**
     * @brief Read a single ModbusDeviceCfg from port, apply scale/offset.
     * @param port    ModbusRTU port instance
     * @param dev     Device config
     * @param portCfg RTU port config (for response_timeout_ms, inter_frame_ms)
     * @return Calibrated float value, or -9999.0f on error
     */
    float readModbusDevice(ModbusRTU& port, const ModbusDeviceCfg& dev,
                           const ModbusRtuPortConfig& portCfg);
};

#endif /* SENSOR_READER_HPP */
