/**
 * ================================================================
 * @file    channel_manager.hpp
 * @brief   Communication channel manager with priority chain,
 *          failover, watchdog, and auto-recovery.
 *
 * @note    Channels: ETH (W5500), GSM (Air780E), WIFI (ESP8266),
 *          IRIDIUM (NR9602G). Tries channels in configured order,
 *          falls back on failure. Saves to SD backup if all fail.
 * ================================================================
 */
#pragma once

#include "runtime_config.hpp"
#include "sd_backup.hpp"
#include <cstdint>

// Forward declarations — avoid circular includes
class W5500Net;
class Air780E;
class MqttClient;

/// Channel identifier
enum class Channel : uint8_t {
    ETHERNET = 0,
    GSM      = 1,
    WIFI     = 2,
    IRIDIUM  = 3,
    COUNT    = 4
};

/// Per-channel state
struct ChannelState {
    bool     enabled          = false;
    bool     alive            = false;
    uint32_t last_success_tick = 0;
    uint32_t fail_count       = 0;
    uint32_t total_sends      = 0;
    uint32_t total_fails      = 0;
};

/// Send result
enum class SendResult : uint8_t {
    Ok           = 0,
    AllFailed    = 1,
    SavedBackup  = 2
};

/// Callback type for sending data via a specific channel
/// Returns HTTP status code (200=ok) or negative on error
typedef int (*ChannelSendFn)(const char* json, uint16_t len, void* ctx);

class ChannelManager {
public:
    ChannelManager();

    /// @brief Initialize from RuntimeConfig
    void init(SdBackup* backup);

    /// @brief Register a send function for a channel
    void registerChannel(Channel ch, ChannelSendFn fn, void* ctx);

    /// @brief Send data through the priority chain
    /// @param json  JSON payload
    /// @param len   payload length
    /// @return SendResult
    SendResult sendData(const char* json, uint16_t len);

    /// @brief Periodic tick — check watchdog timers, attempt recovery
    void tick();

    /// @brief Get channel state
    const ChannelState& getState(Channel ch) const;

    /// @brief Mark a channel as alive (call after successful init/connection)
    void markAlive(Channel ch);

    /// @brief Mark a channel as dead
    void markDead(Channel ch);

    /// @brief Check if a channel is enabled in config
    bool isEnabled(Channel ch) const;

    /// @brief Get the number of active (enabled+alive) channels
    uint8_t activeCount() const;

    /// @brief Get the channel that was actually used in last sendData()
    Channel getLastSentChannel() const { return m_lastSentChannel; }

private:
    ChannelState  m_states[4];
    ChannelSendFn m_sendFns[4] = {nullptr};
    void*         m_sendCtx[4] = {nullptr};
    SdBackup*     m_backup     = nullptr;

    /// Watchdog timeout: if no success in this time, mark channel dead
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 120000; // 2 min

    /// Auto-recovery interval: try higher-priority channels periodically
    static constexpr uint32_t RECOVERY_INTERVAL_MS = 300000; // 5 min
    uint32_t m_lastRecoveryTick = 0;

    /// @brief Try to send via a specific channel
    bool trySend(Channel ch, const char* json, uint16_t len);

    /// @brief Load channel enable flags from config
    void loadConfig();

    /// Last channel that successfully sent data
    Channel m_lastSentChannel = Channel::ETHERNET;
};
