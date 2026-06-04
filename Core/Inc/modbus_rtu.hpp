/**
 * ================================================================
 * @file    modbus_rtu.hpp
 * @brief   C++ класс Modbus RTU Master (USART3 + RS-485).
 * ================================================================
 */
#ifndef MODBUS_RTU_HPP
#define MODBUS_RTU_HPP

#include "main.h"
#include "config.hpp"
#include <cstdint>
#include <cstring>

enum class ModbusStatus : uint8_t {
    Ok       = 0,
    Timeout  = 1,
    CrcError = 2,
    SlaveErr = 3,
    FrameErr = 4
};

class ModbusRTU {
public:
    /**
     * @brief  Конструктор с параметрами (для m_modbusPort0 — инициализируется сразу).
     * @param  uart   — USART handle (huart3 / huart4 / huart5)
     * @param  dePort — GPIO порт пина DE/RE
     * @param  dePin  — номер пина DE/RE
     */
    ModbusRTU(UART_HandleTypeDef* uart,
              GPIO_TypeDef* dePort, uint16_t dePin);

    /**
     * @brief  Дефолтный конструктор — для хранения в классе App как член.
     *         UART не назначен; требует вызова configure() перед init().
     */
    ModbusRTU() = default;

    /**
     * @brief  Назначить UART и пин DE/RE после дефолтной конструкции.
     *         Вызывать перед init() для портов 1 и 2.
     * @param  uart   — UART handle (например &huart4)
     * @param  dePort — GPIO порт пина DE/RE
     * @param  dePin  — номер пина DE/RE
     */
    void configure(UART_HandleTypeDef* uart,
                   GPIO_TypeDef* dePort, uint16_t dePin);

    /** @brief Возвращает true если UART назначен через конструктор или configure(). */
    bool isConfigured() const { return m_uart != nullptr; }

    /** Инициализация (DE/RE в режим приёма). Вызывать после configure(). */
    void init();

    /**
     * @brief  Чтение регистров
     * @param  slave   — адрес (1..247)
     * @param  fc      — функция (3 или 4)
     * @param  start   — начальный регистр
     * @param  count   — количество
     * @param  outRegs — массив для результата
     * @param  timeout — мс
     * @retval Статус (Timeout если порт не сконфигурирован)
     */
    ModbusStatus readRegisters(uint8_t slave, uint8_t fc,
                                uint16_t start, uint16_t count,
                                uint16_t* outRegs,
                                uint32_t timeout = Config::MODBUS_TIMEOUT_MS);

    /** Статический расчёт CRC16 */
    static uint16_t crc16(const uint8_t* data, uint16_t len);

private:
    UART_HandleTypeDef* m_uart   = nullptr;  ///< nullptr пока не назначен
    GPIO_TypeDef*       m_dePort = nullptr;
    uint16_t            m_dePin  = 0;

    void setTransmit();
    void setReceive();
};

#endif /* MODBUS_RTU_HPP */
