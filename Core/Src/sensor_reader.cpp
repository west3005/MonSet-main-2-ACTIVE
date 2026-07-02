/**
 * ================================================================
 * @file    sensor_reader.cpp
 * @brief   Multi-sensor reader with averaging and calibration.
 *
 * @note    When modbus_map_count == 0, falls back to legacy
 *          single-register read on port 0 (USART3).
 *          Estimated Flash: ~1.5 KB, RAM: ~1.6 KB (readings array).
 * ================================================================
 */
#include "sensor_reader.hpp"
#include "debug_uart.hpp"
#include <cstring>

// Static dummy reading for out-of-range access
static const SensorReading s_emptyReading{};

// ================================================================
// Constructors
// ================================================================
SensorReader::SensorReader(ModbusRTU* ports[3], DS3231& rtc)
    : m_rtc(rtc)
{
    for (int i = 0; i < 3; i++) m_ports[i] = ports[i];
}

SensorReader::SensorReader(ModbusRTU& modbus, DS3231& rtc)
    : m_rtc(rtc)
{
    m_ports[0] = &modbus;
    m_ports[1] = nullptr;
    m_ports[2] = nullptr;
}

// ================================================================
// Legacy convert (backward compat)
// ================================================================
float SensorReader::convertLegacy(uint16_t reg0, uint16_t reg1) {
    const RuntimeConfig& c = Cfg();
    float raw = static_cast<float>(
        static_cast<uint32_t>(reg0) * 65536UL + static_cast<uint32_t>(reg1)
    ) / 10.0f;
    return c.sensor_zero_level - raw / c.sensor_divider;
}

// ================================================================
// Read with averaging
// ================================================================
float SensorReader::readWithAveraging(ModbusRTU& port, const ModbusRegEntry& entry,
                                       uint8_t avgCount) {
    if (avgCount == 0) avgCount = 1;

    float sum = 0.0f;
    uint8_t validCount = 0;
    uint16_t regs[30];

    for (uint8_t a = 0; a < avgCount; a++) {
        std::memset(regs, 0, sizeof(regs));

        auto status = port.readRegisters(
            entry.slave_id,
            entry.function,
            entry.start_reg,
            entry.count,
            regs
        );

        if (status == ModbusStatus::Ok) {
            float raw = ModbusMap::convertByType(regs, entry.data_type);
            sum += raw;
            validCount++;
        } else {
            DBG.warn("Modbus: avg read %u/%u failed (status=%d)",
                     (unsigned)(a+1), (unsigned)avgCount, (int)status);
        }

        // Small delay between averaging reads
        if (avgCount > 1 && a < avgCount - 1) {
            HAL_Delay(50);
        }
    }

    if (validCount == 0) return -9999.0f;
    return sum / static_cast<float>(validCount);
}

// ================================================================
// Read a single entry
// ================================================================
bool SensorReader::readEntry(const ModbusRegEntry& entry, SensorReading& out,
                              const DateTime& ts) {
    // Check port available
    if (entry.port_idx >= 3 || m_ports[entry.port_idx] == nullptr) {
        DBG.error("Sensor '%s': port %u not available", entry.name, (unsigned)entry.port_idx);
        out.valid = false;
        return false;
    }

    // Check port mode is enabled for modbus
    if (Cfg().uart_ports[entry.port_idx].mode != 1) {
        DBG.warn("Sensor '%s': port %u not in modbus mode", entry.name, (unsigned)entry.port_idx);
        out.valid = false;
        return false;
    }

    ModbusRTU& port = *m_ports[entry.port_idx];

    float raw = readWithAveraging(port, entry, Cfg().avg_count);
    if (raw <= -9998.0f) {
        out.valid = false;
        return false;
    }

    float calibrated = ModbusMap::applyCalibration(raw, entry);

    std::strncpy(out.name, entry.name, sizeof(out.name) - 1);
    out.name[sizeof(out.name) - 1] = 0;
    std::strncpy(out.unit, entry.unit, sizeof(out.unit) - 1);
    out.unit[sizeof(out.unit) - 1] = 0;
    out.raw_value = raw;
    out.value     = calibrated;
    out.valid     = true;
    out.timestamp = ts;

    DBG.info("Sensor '%s': raw=%.3f cal=%.3f %s (port%u slave%u)",
             out.name, (double)raw, (double)calibrated, out.unit,
             (unsigned)entry.port_idx, (unsigned)entry.slave_id);

    return true;
}

// ================================================================
// Main read() — reads all sensors or legacy single
// ================================================================
float SensorReader::read(DateTime& timestamp) {
    m_lastValue = -9999.0f;
    m_readingCount = 0;

    // Read RTC time
    if (!m_rtc.getTime(timestamp)) {
        DBG.error("DS3231: time read error");
        timestamp = DateTime{};
    }

    const RuntimeConfig& c = Cfg();

    // --- Multi-sensor mode (modbus_map configured) ---
    if (c.modbus_map_count > 0) {
        for (uint8_t i = 0; i < c.modbus_map_count && i < MAX_SENSOR_READINGS; i++) {
            SensorReading& rdg = m_readings[m_readingCount];
            rdg = SensorReading{}; // reset

            if (readEntry(c.modbus_map[i], rdg, timestamp)) {
                m_readingCount++;
            } else {
                // Still count it as a reading slot (invalid)
                std::strncpy(rdg.name, c.modbus_map[i].name, sizeof(rdg.name) - 1);
                rdg.valid = false;
                rdg.timestamp = timestamp;
                m_readingCount++;
            }
        }

        // Return first valid reading for backward compat
        for (uint8_t i = 0; i < m_readingCount; i++) {
            if (m_readings[i].valid) {
                m_lastValue = m_readings[i].value;
                break;
            }
        }
        return m_lastValue;
    }

    // --- Structured RTU device mode (JSON rtu[] via ModbusDeviceCfg) ---
    // Использует существующий pollRtuPorts()/readModbusDevice() из
    // sensor_reader_ext.cpp, который уже корректно применяет data_type
    // (включая FLOAT32_BE), scale, divider и offset.
    bool anyRtuDevice = false;
    for (uint8_t port = 0; port < MAX_RTU_PORTS && !anyRtuDevice; ++port) {
        const auto& rtu = c.rtu_ports[port];
        if (!rtu.enabled) continue;
        for (uint8_t dev = 0; dev < rtu.device_count; ++dev) {
            if (rtu.devices[dev].enabled) { anyRtuDevice = true; break; }
        }
    }

    if (anyRtuDevice) {
        for (uint8_t i = 0; i < MAX_SENSOR_READINGS; ++i) {
            m_readings[i] = SensorReading{};
        }
        m_readingCount = 0;

        pollRtuPorts(c.rtu_ports, MAX_RTU_PORTS);

        uint8_t maxIdx = 0;
        for (uint8_t port = 0; port < MAX_RTU_PORTS; ++port) {
            const auto& rtu = c.rtu_ports[port];
            if (!rtu.enabled) continue;
            for (uint8_t dev = 0; dev < rtu.device_count; ++dev) {
                const auto& d = rtu.devices[dev];
                if (d.enabled && d.channel_idx < MAX_SENSOR_READINGS) {
                    m_readings[d.channel_idx].timestamp = timestamp;
                    if (d.channel_idx + 1 > maxIdx) maxIdx = d.channel_idx + 1;
                }
            }
        }
        m_readingCount = maxIdx;

        for (uint8_t i = 0; i < m_readingCount; i++) {
            if (m_readings[i].valid) {
                m_lastValue = m_readings[i].value;
                break;
            }
        }
        return m_lastValue;
    }

    // --- Legacy single-sensor mode ---
    uint16_t regs[2] = {0, 0};

    auto status = m_ports[0]->readRegisters(
        c.modbus_slave,
        c.modbus_func,
        c.modbus_start_reg,
        c.modbus_num_regs,
        regs
    );

    if (status == ModbusStatus::Ok) {
        m_lastValue = convertLegacy(regs[0], regs[1]);
        DBG.info("Modbus: [0x%04X,0x%04X] -> %.3f", regs[0], regs[1], m_lastValue);

        SensorReading& rdg = m_readings[0];
        rdg = SensorReading{};
        std::strncpy(rdg.name, c.metric_id, sizeof(rdg.name) - 1);
        rdg.name[sizeof(rdg.name) - 1] = 0;
        rdg.value     = m_lastValue;
        rdg.raw_value = m_lastValue;
        rdg.unit[0]   = 0;
        rdg.valid     = true;
        rdg.timestamp = timestamp;
        m_readingCount = 1;
    } else {
        DBG.error("Modbus: error %d", static_cast<int>(status));
    }

    return m_lastValue;
}

// ================================================================
// getReading
// ================================================================
const SensorReading& SensorReader::getReading(uint8_t idx) const {
    if (idx >= m_readingCount) return s_emptyReading;
    return m_readings[idx];
}

// Estimated Flash: ~1.5 KB  |  RAM: ~1.6 KB (readings array)
