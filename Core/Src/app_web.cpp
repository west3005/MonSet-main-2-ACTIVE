extern volatile bool g_web_exclusive;

/**
 * @file    app_web.cpp
 * @brief   App web exclusive mode and test-send implementation.
 */
#include "app.hpp"
#include "debug_uart.hpp"
#include "runtime_config.hpp"
#include "circular_log.hpp"
#include "sd_backup.hpp"
#include <cstdio>
#include <cstring>

// ============================================================================
// notifyWebActivity
// ============================================================================
void App::notifyWebActivity() {
    m_webActive      = true;
    m_webLastReqTick = HAL_GetTick();
}

// ============================================================================
// webIdleRemainingS
// ============================================================================
uint16_t App::webIdleRemainingS() const {
    if (!m_webActive) return 0;
    const RuntimeConfig& cfg = Cfg();
    uint32_t timeoutMs = (uint32_t)cfg.web.web_idle_timeout_s * 1000U;
    uint32_t elapsed   = (uint32_t)(HAL_GetTick() - m_webLastReqTick);
    if (elapsed >= timeoutMs) return 0;
    return (uint16_t)((timeoutMs - elapsed) / 1000U);
}

// ============================================================================
// checkWebTimeout — call each main loop
// ============================================================================
void App::checkWebTimeout() {
    if (!m_webActive) return;
    const RuntimeConfig& cfg = Cfg();
    uint32_t timeoutMs = (uint32_t)cfg.web.web_idle_timeout_s * 1000U;
    if ((uint32_t)(HAL_GetTick() - m_webLastReqTick) >= timeoutMs) {
        m_webActive = false;
        DBG.info("[WEB_IDLE] resuming data transmission");
        CircularLogBuffer::instance().write("INFO", "WEB_IDLE: resuming data transmission");
    }
}

// ============================================================================
// writeToBackup — stores payload as a JSONL line with "src":"web_q" tag
// ============================================================================
void App::writeToBackup(const char* payload) {
    if (!m_sdOk) return;
    // appendLine() writes a JSON object line followed by \r\n
    if (!m_sdBackup.appendLine(payload)) {
        DBG.error("[WEB_ACTIVE] backup appendLine failed");
        return;
    }
    DBG.info("[WEB_ACTIVE] measurement queued to backup");
    CircularLogBuffer::instance().write("INFO", "WEB_ACTIVE: measurement queued to backup");
}

// ============================================================================
// triggerTestSend
// ============================================================================
void App::triggerTestSend() {
    if (m_testState == TestSendState::Idle ||
        m_testState == TestSendState::Success ||
        m_testState == TestSendState::Fail) {
        m_testState      = TestSendState::Reading;
        m_testElapsedMs  = 0;
        m_testHttpCode   = 0;
        m_testChannel[0] = '\0';
        DBG.info("[TEST] test send triggered");
    }
}

// ============================================================================
// getTestResult
// ============================================================================
void App::getTestResult(char* state, char* channel, int* httpCode, uint32_t* elapsedMs) const {
    switch (m_testState) {
        case TestSendState::Idle:     std::strncpy(state, "idle",     15); break;
        case TestSendState::Reading:  std::strncpy(state, "reading",  15); break;
        case TestSendState::Building: std::strncpy(state, "building", 15); break;
        case TestSendState::Sending:  std::strncpy(state, "sending",  15); break;
        case TestSendState::Success:  std::strncpy(state, "success",  15); break;
        case TestSendState::Fail:     std::strncpy(state, "fail",     15); break;
        default:                      std::strncpy(state, "unknown",  15); break;
    }
    state[15] = '\0';
    std::strncpy(channel, m_testChannel, 15); channel[15] = '\0';
    *httpCode  = m_testHttpCode;
    *elapsedMs = m_testElapsedMs;
}

// ============================================================================
// runTestSend — executes test in the main loop when state==Reading
// ============================================================================
void App::runTestSend() {
    uint32_t start = HAL_GetTick();
    m_testState = TestSendState::Building;

    // 1. Build payload
    DateTime dt{};
    float val = m_sensor.read(dt);
    (void)val;

    char payload[512];
    int plen = buildMultiSensorPayload(payload, sizeof(payload), "test", dt, false);
    if (plen <= 0) {
        m_testState = TestSendState::Fail;
        DBG.warn("[TEST] payload build failed");
        return;
    }

    // 2. Send via channel manager (test send ignores web_exclusive_mode)
    m_testState = TestSendState::Sending;
    std::strncpy(m_testChannel, "eth", sizeof(m_testChannel) - 1);
    m_testChannel[sizeof(m_testChannel) - 1] = '\0';

    bool prevWebExclusive = g_web_exclusive;
    g_web_exclusive = false;
    SendResult result = m_channelMgr.sendData(payload, (uint16_t)plen);
    g_web_exclusive = prevWebExclusive;
    m_testElapsedMs = (uint32_t)(HAL_GetTick() - start);

    // Map SendResult to HTTP-style code for the caller
    if (result == SendResult::Ok) {
        m_testHttpCode = 200;
        m_testState    = TestSendState::Success;
        DBG.info("[TEST] success, elapsed=%lu ms", (unsigned long)m_testElapsedMs);
        CircularLogBuffer::instance().writef("INFO", "TEST: success HTTP 200 [%lu ms]",
                                              (unsigned long)m_testElapsedMs);
    } else {
        m_testHttpCode = -1;
        m_testState    = TestSendState::Fail;
        DBG.warn("[TEST] fail, result=%d", (int)result);
        CircularLogBuffer::instance().writef("WARN", "TEST: fail result=%d [%lu ms]",
                                              (int)result, (unsigned long)m_testElapsedMs);
    }
}


// ============================================================================
// processTestSend — call periodically from main loop
// ============================================================================
void App::processTestSend() {
    if (m_testState == TestSendState::Reading) {
        DBG.info("[TEST] starting runTestSend() from processTestSend");
        runTestSend();
    }
}
