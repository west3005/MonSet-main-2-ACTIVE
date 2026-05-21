/**
 * ================================================================
 * @file    captive_portal.cpp
 * @brief   Captive portal — ESP8266 AP + setup wizard.
 *
 * @note    Serves a minimal HTML form via ESP8266 TCP server.
 *          Form fields are URL-encoded and parsed into RuntimeConfig.
 *          Estimated Flash: ~3 KB, RAM: ~256 bytes.
 * ================================================================
 */
#include "captive_portal.hpp"
#include "debug_uart.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>

// ================================================================
// Constructor
// ================================================================
CaptivePortal::CaptivePortal() {}

// ================================================================
// Init
// ================================================================
void CaptivePortal::init(ESP8266* esp) {
    m_esp = esp;
}

// ================================================================
// Start
// ================================================================
bool CaptivePortal::start(const char* apSsid, const char* apPass) {
    if (!m_esp) return false;

    m_esp->enterEspMode();
    if (!m_esp->init()) {
        m_esp->exitEspMode();
        return false;
    }

    if (!m_esp->startAP(apSsid, apPass)) {
        m_esp->exitEspMode();
        return false;
    }

    if (!m_esp->startServer(80)) {
        m_esp->stopAP();
        m_esp->exitEspMode();
        return false;
    }

    m_active = true;
    m_configSaved = false;
    DBG.info("CaptivePortal: started on %s", apSsid);
    return true;
}

// ================================================================
// Stop
// ================================================================
void CaptivePortal::stop() {
    if (m_esp) {
        m_esp->stopServer();
        m_esp->stopAP();
        m_esp->exitEspMode();
    }
    m_active = false;
    DBG.info("CaptivePortal: stopped");
}

// ================================================================
// URL decode
// ================================================================
void CaptivePortal::urlDecode(char* str) {
    char* out = str;
    while (*str) {
        if (*str == '%' && str[1] && str[2]) {
            char hex[3] = {str[1], str[2], 0};
            *out++ = (char)std::strtoul(hex, nullptr, 16);
            str += 3;
        } else if (*str == '+') {
            *out++ = ' ';
            str++;
        } else {
            *out++ = *str++;
        }
    }
    *out = 0;
}

// ================================================================
// Get form field from URL-encoded body
// ================================================================
bool CaptivePortal::getFormField(const char* data, const char* key,
                                   char* out, size_t outSz) {
    if (!data || !key) return false;

    char search[64];
    std::snprintf(search, sizeof(search), "%s=", key);

    const char* p = std::strstr(data, search);
    if (!p) return false;
    p += std::strlen(search);

    size_t i = 0;
    while (*p && *p != '&' && i < outSz - 1) {
        out[i++] = *p++;
    }
    out[i] = 0;
    urlDecode(out);
    return true;
}

// ================================================================
// Parse form data into config
// ================================================================
bool CaptivePortal::parseFormData(const char* body) {
    if (!body) return false;

    RuntimeConfig& c = Cfg();
    char tmp[128];

    // Simple mode fields
    if (getFormField(body, "wifi_ssid", tmp, sizeof(tmp)))
        std::strncpy(c.wifi_ssid, tmp, sizeof(c.wifi_ssid) - 1);

    if (getFormField(body, "wifi_pass", tmp, sizeof(tmp)))
        std::strncpy(c.wifi_pass, tmp, sizeof(c.wifi_pass) - 1);

    if (getFormField(body, "gsm_apn", tmp, sizeof(tmp)))
        std::strncpy(c.gsm_apn, tmp, sizeof(c.gsm_apn) - 1);

    if (getFormField(body, "mqtt_host", tmp, sizeof(tmp)))
        std::strncpy(c.mqtt_host, tmp, sizeof(c.mqtt_host) - 1);

    if (getFormField(body, "mqtt_topic", tmp, sizeof(tmp)))
        std::strncpy(c.mqtt_topic, tmp, sizeof(c.mqtt_topic) - 1);

    if (getFormField(body, "server_url", tmp, sizeof(tmp)))
        std::strncpy(c.server_url, tmp, sizeof(c.server_url) - 1);

    if (getFormField(body, "mqtt_port", tmp, sizeof(tmp)))
        c.mqtt_port = (uint16_t)std::strtoul(tmp, nullptr, 10);

    if (getFormField(body, "poll_interval_sec", tmp, sizeof(tmp)))
        c.poll_interval_sec = (uint32_t)std::strtoul(tmp, nullptr, 10);

    // Advanced mode fields
    if (getFormField(body, "mqtt_user", tmp, sizeof(tmp)))
        std::strncpy(c.mqtt_user, tmp, sizeof(c.mqtt_user) - 1);

    if (getFormField(body, "mqtt_pass", tmp, sizeof(tmp)))
        std::strncpy(c.mqtt_pass, tmp, sizeof(c.mqtt_pass) - 1);

    if (getFormField(body, "web_user", tmp, sizeof(tmp)))
        std::strncpy(c.web.web_user, tmp, sizeof(c.web.web_user) - 1);

    if (getFormField(body, "web_pass", tmp, sizeof(tmp)))
        std::strncpy(c.web.web_pass, tmp, sizeof(c.web.web_pass) - 1);

    // Protocol mode
    if (getFormField(body, "protocol", tmp, sizeof(tmp))) {
        if (std::strcmp(tmp, "mqtt") == 0) c.protocol = ProtocolMode::MQTT_GENERIC;
        else c.protocol = ProtocolMode::HTTPS_GENERIC;
    }

    // Enable WiFi channel if WiFi configured
    if (c.wifi_ssid[0]) c.wifi_enabled = true;

    c.validateAndFix();

    // Save to SD
    if (c.saveToSd(RUNTIME_CONFIG_FILENAME)) {
        DBG.info("CaptivePortal: config saved to SD");
        m_configSaved = true;
        return true;
    }

    DBG.error("CaptivePortal: save to SD failed");
    return false;
}

// ================================================================
// Generate setup page HTML
// ================================================================
int CaptivePortal::generateSetupPage(char* out, size_t outSz) {
    const RuntimeConfig& c = Cfg();
    return std::snprintf(out, outSz,
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>MonSet Setup</title>"
        "<style>body{font-family:sans-serif;max-width:480px;margin:0 auto;padding:16px;background:#222;color:#eee}"
        "h1{font-size:20px;color:#4fc3f7}input{width:100%%;padding:8px;margin:4px 0 12px;background:#333;"
        "color:#eee;border:1px solid #555;border-radius:4px;box-sizing:border-box}"
        "label{font-size:13px;color:#aaa}button{background:#4fc3f7;color:#222;border:none;"
        "padding:10px 20px;cursor:pointer;font-weight:bold;border-radius:4px;width:100%%}"
        "details{margin-top:16px;border:1px solid #444;padding:8px;border-radius:4px}"
        "summary{cursor:pointer;color:#4fc3f7}</style></head>"
        "<body><h1>MonSet Setup</h1>"
        "<form method='POST' action='/save'>"
        "<label>WiFi SSID</label><input name='wifi_ssid' value='%s'>"
        "<label>WiFi Password</label><input name='wifi_pass' type='password' value='%s'>"
        "<label>GSM APN</label><input name='gsm_apn' value='%s'>"
        "<label>Server URL</label><input name='server_url' value='%s'>"
        "<label>MQTT Host</label><input name='mqtt_host' value='%s'>"
        "<label>MQTT Port</label><input name='mqtt_port' value='%u'>"
        "<label>MQTT Topic</label><input name='mqtt_topic' value='%s'>"
        "<details><summary>Advanced</summary>"
        "<label>MQTT User</label><input name='mqtt_user' value='%s'>"
        "<label>MQTT Pass</label><input name='mqtt_pass' type='password'>"
        "<label>Protocol</label><input name='protocol' value='%s'>"
        "<label>Poll Interval (sec)</label><input name='poll_interval_sec' value='%lu'>"
        "<label>Web User</label><input name='web_user' value='%s'>"
        "<label>Web Pass</label><input name='web_pass' type='password'>"
        "</details><br>"
        "<button type='submit'>Save &amp; Reboot</button>"
        "</form></body></html>",
        c.wifi_ssid, c.wifi_pass, c.gsm_apn, c.server_url,
        c.mqtt_host, (unsigned)c.mqtt_port, c.mqtt_topic,
        c.mqtt_user,
        (c.protocol == ProtocolMode::MQTT_GENERIC || c.protocol == ProtocolMode::MQTT_THINGSBOARD) ? "mqtt" : "https",
        (unsigned long)c.poll_interval_sec,
        c.web.web_user);
}

// ================================================================
// Handle incoming request
// ================================================================
void CaptivePortal::handleRequest(const char* data, uint16_t len) {
    if (!m_esp || len < 4) return;

    if (std::strncmp(data, "GET", 3) == 0) {
        // Serve setup page
        static char page[2048];
        int pageLen = generateSetupPage(page, sizeof(page));
        if (pageLen > 0) {
            m_esp->sendData((const uint8_t*)page, (uint16_t)pageLen);
        }
    } else if (std::strncmp(data, "POST", 4) == 0) {
        // Find body
        const char* body = std::strstr(data, "\r\n\r\n");
        if (body) {
            body += 4;
            parseFormData(body);
        }

        // Send redirect response
        const char* resp =
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
            "<!DOCTYPE html><html><body style='background:#222;color:#eee;"
            "font-family:sans-serif;text-align:center;padding:40px'>"
            "<h1 style='color:#4fc3f7'>Config Saved!</h1>"
            "<p>Device will reboot in 5 seconds...</p>"
            "</body></html>";
        m_esp->sendData((const uint8_t*)resp, (uint16_t)std::strlen(resp));
    }
}

// ================================================================
// Non-blocking tick
// ================================================================
void CaptivePortal::tick() {
    if (!m_active || !m_esp) return;

    uint8_t buf[1024];
    uint16_t len = m_esp->receiveData(buf, sizeof(buf) - 1, 100);
    if (len > 0) {
        buf[len] = 0;
        handleRequest((const char*)buf, len);
    }
}

// Estimated Flash: ~3 KB  |  RAM: ~256 bytes (static page buffer in function)
