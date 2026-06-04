/**
 * ================================================================
 * @file    modbus_rtu.cpp
 * @brief   Реализация Modbus RTU Master.
 * ================================================================
 */
#include "modbus_rtu.hpp"
#include "debug_uart.hpp"

/* ========= Конструктор ========= */
ModbusRTU::ModbusRTU(UART_HandleTypeDef* uart,
                      GPIO_TypeDef* dePort, uint16_t dePin)
    : m_uart(uart), m_dePort(dePort), m_dePin(dePin)
{
}

/* ========= configure() — для объектов, созданных дефолтным конструктором ========= */
void ModbusRTU::configure(UART_HandleTypeDef* uart,
                           GPIO_TypeDef* dePort, uint16_t dePin)
{
    m_uart   = uart;
    m_dePort = dePort;
    m_dePin  = dePin;
}

/* ========= configure(uart) — для auto-direction конвертеров без DE (напр. 2126) ========= */
void ModbusRTU::configure(UART_HandleTypeDef* uart)
{
    m_uart   = uart;
    m_dePort = nullptr;   // DE не управляется — конвертер сам переключает направление
    m_dePin  = 0;
}

/* ========= DE/RE управление ========= */
void ModbusRTU::setTransmit() {
    if (m_dePort) HAL_GPIO_WritePin(m_dePort, m_dePin, GPIO_PIN_SET);
}

void ModbusRTU::setReceive() {
    if (m_dePort) HAL_GPIO_WritePin(m_dePort, m_dePin, GPIO_PIN_RESET);
}

/* ========= CRC16 ========= */
uint16_t ModbusRTU::crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else          crc >>= 1;
        }
    }
    return crc;
}

/* ========= Init ========= */
void ModbusRTU::init()
{
    if (!m_uart) {
        DBG.error("ModbusRTU::init() — UART не назначен, пропуск");
        return;
    }
    setReceive();   // no-op если dePort=nullptr (auto-direction конвертер)
    DBG.info("ModbusRTU: инициализирован uart=0x%p de=%s",
             static_cast<void*>(m_uart), m_dePort ? "GPIO" : "auto");
}

/* ========= Чтение регистров ========= */
ModbusStatus ModbusRTU::readRegisters(uint8_t slave, uint8_t fc,
                                       uint16_t start, uint16_t count,
                                       uint16_t* outRegs, uint32_t timeout)
{
    if (!m_uart) {
        DBG.error("ModbusRTU::readRegisters() — UART не назначен, slave=%d", slave);
        return ModbusStatus::Timeout;
    }

    /* 1. Формируем запрос */
    uint8_t tx[8];
    tx[0] = slave;
    tx[1] = fc;
    tx[2] = static_cast<uint8_t>(start >> 8);
    tx[3] = static_cast<uint8_t>(start & 0xFF);
    tx[4] = static_cast<uint8_t>(count >> 8);
    tx[5] = static_cast<uint8_t>(count & 0xFF);

    uint16_t c = crc16(tx, 6);
    tx[6] = static_cast<uint8_t>(c & 0xFF);
    tx[7] = static_cast<uint8_t>(c >> 8);

    DBG.hexDump("MB TX", tx, 8);

    /* 2. Отправляем */
    setTransmit();
    HAL_Delay(1);
    HAL_UART_Transmit(m_uart, tx, 8, 100);
    HAL_Delay(2);
    setReceive();

    /* 3. Принимаем */
    const uint16_t expLen = 3 + count * 2 + 2;
    uint8_t rx[64];
    std::memset(rx, 0, sizeof(rx));

    auto hs = HAL_UART_Receive(m_uart, rx, expLen, timeout);

    if (hs == HAL_TIMEOUT) {
        DBG.error("Modbus: ТАЙМАУТ от slave %d", slave);
        return ModbusStatus::Timeout;
    }
    if (hs != HAL_OK) {
        DBG.error("Modbus: HAL ошибка %d", static_cast<int>(hs));
        return ModbusStatus::FrameErr;
    }

    DBG.hexDump("MB RX", rx, expLen);

    /* 4. Валидация */
    if (rx[0] != slave)         return ModbusStatus::FrameErr;
    if (rx[1] & 0x80)           return ModbusStatus::SlaveErr;
    if (rx[1] != fc)            return ModbusStatus::FrameErr;
    if (rx[2] != count * 2)     return ModbusStatus::FrameErr;

    uint16_t rcrc = crc16(rx, expLen - 2);
    uint16_t mcrc = static_cast<uint16_t>(rx[expLen - 2]) |
                    (static_cast<uint16_t>(rx[expLen - 1]) << 8);

    if (rcrc != mcrc) {
        DBG.error("Modbus: CRC mismatch (0x%04X vs 0x%04X)", rcrc, mcrc);
        return ModbusStatus::CrcError;
    }

    /* 5. Данные */
    for (uint16_t i = 0; i < count; i++) {
        outRegs[i] = (static_cast<uint16_t>(rx[3 + i * 2]) << 8) |
                      static_cast<uint16_t>(rx[3 + i * 2 + 1]);
    }

    DBG.info("Modbus: OK, %d рег. от slave %d", count, slave);
    return ModbusStatus::Ok;
}
