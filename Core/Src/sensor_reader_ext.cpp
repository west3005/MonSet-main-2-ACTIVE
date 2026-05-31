// sensor_reader_ext.cpp — extension: multi-device RTU polling
#include "sensor_reader.hpp"
#include "modbus_rtu.hpp"
#include "debug_uart.hpp"
#include "stm32f4xx_hal.h"
#include <cstring>

// ================================================================
// Internal helper: parse raw big-endian bytes into float
// ================================================================

/**
 * @brief Convert big-endian raw bytes to a float value.
 *
 * @param raw   Byte buffer (big-endian, at least 2 or 4 bytes depending on dtype)
 * @param dtype 0=INT16, 1=UINT16, 2=INT32_BE, 3=UINT32_BE, 4=FLOAT32_BE
 * @return Parsed value as float, or 0.0f for unknown dtype
 */
static float parseModbusRegisters(const uint8_t* raw, uint8_t dtype) {
    switch (dtype) {
        case 0:  // INT16 (signed)
            return (float)(int16_t)((raw[0] << 8) | raw[1]);

        case 1:  // UINT16 (unsigned)
            return (float)(uint16_t)((raw[0] << 8) | raw[1]);

        case 2: {  // INT32 big-endian
            int32_t v = (int32_t)(
                ((uint32_t)raw[0] << 24) |
                ((uint32_t)raw[1] << 16) |
                ((uint32_t)raw[2] <<  8) |
                 (uint32_t)raw[3]
            );
            return (float)v;
        }

        case 3: {  // UINT32 big-endian
            uint32_t v = ((uint32_t)raw[0] << 24) |
                         ((uint32_t)raw[1] << 16) |
                         ((uint32_t)raw[2] <<  8) |
                          (uint32_t)raw[3];
            return (float)v;
        }

        case 4: {  // IEEE-754 FLOAT32 big-endian
            uint32_t u = ((uint32_t)raw[0] << 24) |
                         ((uint32_t)raw[1] << 16) |
                         ((uint32_t)raw[2] <<  8) |
                          (uint32_t)raw[3];
            float f;
            std::memcpy(&f, &u, sizeof(f));
            return f;
        }

        default:
            return 0.0f;
    }
}

// ================================================================
// SensorReader::readModbusDevice
// ================================================================

float SensorReader::readModbusDevice(ModbusRTU& port,
                                     const ModbusDeviceCfg& dev,
                                     const ModbusRtuPortConfig& portCfg) {
    // Guard: only FC 3 (holding) and FC 4 (input) are supported
    if (dev.func_code != 3 && dev.func_code != 4) {
        DBG.warn("RTU: %s (addr=%u) unsupported fc=%u",
                 dev.name, (unsigned)dev.slave_addr, (unsigned)dev.func_code);
        return -9999.0f;
    }

    // Guard: reg_count must be 1..4 (we only buffer 8 bytes = 4 registers)
    const uint8_t count = (dev.reg_count == 0) ? 1u
                        : (dev.reg_count  > 4)  ? 4u
                        : dev.reg_count;

    uint16_t regs[4] = {};

    // Use the ModbusRTU::readRegisters() API with portCfg.response_timeout_ms
    ModbusStatus status = port.readRegisters(
        dev.slave_addr,
        dev.func_code,
        dev.reg_start,
        count,
        regs,
        portCfg.response_timeout_ms
    );

    if (status != ModbusStatus::Ok) {
        DBG.warn("RTU: %s (addr=%u fc=%u reg=%u) READ FAIL status=%d",
                 dev.name,
                 (unsigned)dev.slave_addr,
                 (unsigned)dev.func_code,
                 (unsigned)dev.reg_start,
                 (int)status);
        return -9999.0f;
    }

    // Convert uint16_t[] to raw bytes (big-endian) for parseModbusRegisters
    uint8_t raw[8] = {};
    for (uint8_t i = 0; i < count && i < 4; i++) {
        raw[i * 2]     = (uint8_t)(regs[i] >> 8);
        raw[i * 2 + 1] = (uint8_t)(regs[i] & 0xFF);
    }

    float val = parseModbusRegisters(raw, dev.data_type);
    // Этап 3: формула масштабирования — аналог ocean-station ScalingConfig
    // ocean-station: value = (multiplier * raw) / divider - reference_level
    // MonSet:        val   = (scale      * raw) / divider - offset
    val = (val * dev.scale) / dev.divider - dev.offset;

    DBG.info("RTU: %s=%.3f %s", dev.name, (double)val, dev.unit);
    return val;
}

// ================================================================
// SensorReader::pollRtuPorts
// ================================================================

void SensorReader::pollRtuPorts(const ModbusRtuPortConfig* rtu_ports,
                                 uint8_t portCount) {
    for (uint8_t p = 0; p < portCount && p < MAX_RTU_PORTS; ++p) {
        const ModbusRtuPortConfig& pCfg = rtu_ports[p];

        if (!pCfg.enabled || m_ports[p] == nullptr) {
            continue;
        }

        ModbusRTU& port = *m_ports[p];

        for (uint8_t d = 0;
             d < pCfg.device_count && d < ModbusRtuPortConfig::MAX_DEVICES;
             ++d) {
            const ModbusDeviceCfg& dev = pCfg.devices[d];

            if (!dev.enabled) {
                continue;
            }

            // Inter-frame delay before each transaction
            if (pCfg.inter_frame_ms > 0) {
                HAL_Delay(pCfg.inter_frame_ms);
            }

            float val = readModbusDevice(port, dev, pCfg);

            if (dev.channel_idx < MAX_SENSOR_READINGS) {
                SensorReading& slot = m_readings[dev.channel_idx];
                slot.value = val;
                slot.valid = (val > -9998.0f);

                // Этап 4: приоритет per-device metric_id над именем устройства
                // Аналог ocean-station: fields[].metric_id → payload metricId
                // Если metric_id непустой — он идёт в slot.name и затем в payload
                // Если пустой — используется dev.name (как раньше, обратная совместимость)
                const char* mid = (dev.metric_id[0] != '\0') ? dev.metric_id : dev.name;
                std::strncpy(slot.name, mid, sizeof(slot.name) - 1);
                slot.name[sizeof(slot.name) - 1] = '\0';

                std::strncpy(slot.unit, dev.unit, sizeof(slot.unit) - 1);
                slot.unit[sizeof(slot.unit) - 1] = '\0';
            }
        }
    }
}
