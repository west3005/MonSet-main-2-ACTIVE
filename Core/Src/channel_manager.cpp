/**
 * ================================================================
 * @file    channel_manager.cpp
 * @brief   Priority chain channel manager with failover & recovery.
 *
 * @note    Estimated Flash: ~1.2 KB, RAM: ~100 bytes.
 * ================================================================
 */
#include "channel_manager.hpp"
#include "debug_uart.hpp"
#include <cstring>

static const char* chName(Channel ch) {
    switch (ch) {
        case Channel::ETHERNET:     return "ETH";
        case Channel::GSM:     return "GSM";
        case Channel::WIFI:    return "WIFI";
        case Channel::IRIDIUM: return "IRIDIUM";
        default:               return "?";
    }
}

// ================================================================
// Constructor
// ================================================================
ChannelManager::ChannelManager() {
    for (auto& s : m_states) s = ChannelState{};
}

// ================================================================
// Init
// ================================================================
void ChannelManager::init(SdBackup* backup) {
    m_backup = backup;
    loadConfig();
    m_lastRecoveryTick = HAL_GetTick();

    for (uint8_t i = 0; i < (uint8_t)Channel::COUNT; i++) {
        if (m_states[i].enabled) {
            DBG.info("ChMgr: %s enabled", chName((Channel)i));
        }
    }
}

// ================================================================
// Load config
// ================================================================
void ChannelManager::loadConfig() {
    const RuntimeConfig& c = Cfg();
    m_states[(uint8_t)Channel::ETHERNET].enabled     = c.eth_enabled;
    m_states[(uint8_t)Channel::GSM].enabled     = c.gsm_enabled;
    m_states[(uint8_t)Channel::WIFI].enabled    = c.wifi_enabled;
    m_states[(uint8_t)Channel::IRIDIUM].enabled = c.iridium_enabled;
}

// ================================================================
// Register channel
// ================================================================
void ChannelManager::registerChannel(Channel ch, ChannelSendFn fn, void* ctx) {
    uint8_t idx = (uint8_t)ch;
    if (idx >= (uint8_t)Channel::COUNT) return;
    m_sendFns[idx] = fn;
    m_sendCtx[idx] = ctx;
}

// ================================================================
// Get state
// ================================================================
const ChannelState& ChannelManager::getState(Channel ch) const {
    return m_states[(uint8_t)ch];
}

void ChannelManager::markAlive(Channel ch) {
    auto& s = m_states[(uint8_t)ch];
    s.alive = true;
    s.last_success_tick = HAL_GetTick();
    s.fail_count = 0;
}

void ChannelManager::markDead(Channel ch) {
    m_states[(uint8_t)ch].alive = false;
}

bool ChannelManager::isEnabled(Channel ch) const {
    return m_states[(uint8_t)ch].enabled;
}

uint8_t ChannelManager::activeCount() const {
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < (uint8_t)Channel::COUNT; i++) {
        if (m_states[i].enabled && m_states[i].alive) cnt++;
    }
    return cnt;
}

// ================================================================
// Try send via one channel
// ================================================================
bool ChannelManager::trySend(Channel ch, const char* json, uint16_t len) {
    uint8_t idx = (uint8_t)ch;
    if (!m_states[idx].enabled) return false;
    if (!m_sendFns[idx]) return false;

    DBG.info("ChMgr: try %s...", chName(ch));
    m_states[idx].total_sends++;

    int code = m_sendFns[idx](json, len, m_sendCtx[idx]);

    if (code >= 200 && code < 300) {
        DBG.info("ChMgr: %s OK (HTTP %d)", chName(ch), code);
        m_states[idx].alive = true;
        m_states[idx].last_success_tick = HAL_GetTick();
        m_states[idx].fail_count = 0;
        return true;
    }

    DBG.warn("ChMgr: %s FAIL (code=%d)", chName(ch), code);
    m_states[idx].fail_count++;
    m_states[idx].total_fails++;

    if (m_states[idx].fail_count >= 3) {
        m_states[idx].alive = false;
    }

    return false;
}

// ================================================================
// sendData — priority chain
// ================================================================
SendResult ChannelManager::sendData(const char* json, uint16_t len) {
    const RuntimeConfig& c = Cfg();

    // Всегда используем chain_order — приоритет: channels.chain_order (новое),
    // затем legacy chain_order. chain_enabled больше не блокирует логику.
    uint8_t count = 0;
    const uint8_t* order = nullptr;

    if (c.channels.chain_count > 0) {
        order = c.channels.chain_order;
        count = c.channels.chain_count;
    } else if (c.chain_count > 0) {
        order = c.chain_order;
        count = c.chain_count;
    }

    if (order && count > 0) {
        for (uint8_t i = 0; i < count; i++) {
            Channel ch = (Channel)order[i];
            if ((uint8_t)ch >= (uint8_t)Channel::COUNT) continue;
            if (trySend(ch, json, len)) return SendResult::Ok;
        }
    } else {
        // Нет chain_order вообще — перебираем в порядке enum
        for (uint8_t i = 0; i < (uint8_t)Channel::COUNT; i++) {
            if (trySend((Channel)i, json, len)) return SendResult::Ok;
        }
    }

    // All channels failed — save to backup
    DBG.error("ChMgr: ALL channels failed, saving to backup");
    if (m_backup) {
        if (m_backup->appendLine(json)) {
            return SendResult::SavedBackup;
        }
    }
    return SendResult::AllFailed;
}

// ================================================================
// Periodic tick — watchdog & recovery
// ================================================================
void ChannelManager::tick() {
    uint32_t now = HAL_GetTick();

    // Watchdog: mark channels dead if no success in timeout
    for (uint8_t i = 0; i < (uint8_t)Channel::COUNT; i++) {
        auto& s = m_states[i];
        if (!s.enabled || !s.alive) continue;

        if (s.last_success_tick != 0 &&
            (now - s.last_success_tick) > WATCHDOG_TIMEOUT_MS) {
            DBG.warn("ChMgr: %s watchdog timeout -> dead", chName((Channel)i));
            s.alive = false;
        }
    }

    // Auto-recovery: periodically try higher-priority channels
    if ((now - m_lastRecoveryTick) > RECOVERY_INTERVAL_MS) {
        m_lastRecoveryTick = now;

        const RuntimeConfig& c = Cfg();
        if (c.chain_enabled && c.chain_count > 1) {
            // Find the first enabled-but-dead channel that's higher priority
            // than current best alive channel
            for (uint8_t i = 0; i < c.chain_count; i++) {
                Channel ch = (Channel)c.chain_order[i];
                uint8_t idx = (uint8_t)ch;
                if (idx >= (uint8_t)Channel::COUNT) continue;

                if (m_states[idx].enabled && !m_states[idx].alive) {
                    DBG.info("ChMgr: recovery attempt for %s", chName(ch));
                    // Reset fail count to allow retry
                    m_states[idx].fail_count = 0;
                    break; // only try one recovery per tick
                }

                if (m_states[idx].enabled && m_states[idx].alive) {
                    // Found a working channel at higher priority — no need to recover
                    break;
                }
            }
        }
    }
}

// Estimated Flash: ~1.2 KB  |  RAM: ~100 bytes
