/**
 * ================================================================
 * @file runtime_config.cpp
 * @brief Runtime configuration — JSON parse/save with all fields.
 *
 * @note Uses a minimal hand-rolled JSON parser (no heap).
 *       Modbus map array parsed via manual bracket scanning.
 *       Estimated Flash: ~6 KB, RAM: ~4.5 KB (g_cfg singleton).
 * ================================================================
 */
#include "runtime_config.hpp"
#include "debug_uart.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>
extern "C" {
#include "ff.h"
}

static RuntimeConfig g_cfg;
static RuntimeConfig g_cfg_backup;  // scratch buffer for handlePostConfig rollback
RuntimeConfig& Cfg()       { return g_cfg; }
RuntimeConfig& CfgBackup() { return g_cfg_backup; }

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static void copyStr(char* dst, size_t dstSz, const char* src) {
    if (!dst || dstSz == 0) return;
    if (!src) { dst[0] = 0; return; }
    std::strncpy(dst, src, dstSz - 1);
    dst[dstSz - 1] = 0;
}

// -----------------------------------------------------------------------------
// Defaults from config.hpp
// -----------------------------------------------------------------------------
void RuntimeConfig::setDefaultsFromConfig() {
    complex_enabled = false;
    copyStr(metric_id,      sizeof(metric_id),      Config::METRIC_ID);
    copyStr(complex_id,     sizeof(complex_id),      Config::COMPLEX_ID);
    copyStr(server_url,     sizeof(server_url),      Config::SERVER_URL);
    copyStr(server_auth_b64,sizeof(server_auth_b64), Config::SERVER_AUTH);

    eth_mode = (Config::NET_MODE == Config::NetMode::DHCP) ? EthMode::Dhcp : EthMode::Static;
    std::memcpy(w5500_mac, Config::W5500_MAC, 6);
    std::memcpy(eth_ip,    Config::NET_IP,    4);
    std::memcpy(eth_sn,    Config::NET_SN,    4);
    std::memcpy(eth_gw,    Config::NET_GW,    4);
    std::memcpy(eth_dns,   Config::NET_DNS,   4);

    copyStr(gsm_apn,  sizeof(gsm_apn),  Config::GSM_APN);
    copyStr(gsm_user, sizeof(gsm_user), Config::GSM_APN_USER);
    copyStr(gsm_pass, sizeof(gsm_pass), Config::GSM_APN_PASS);

    poll_interval_sec   = Config::POLL_INTERVAL_SEC;
    send_interval_polls = Config::SEND_INTERVAL_POLLS;
    modbus_slave        = Config::MODBUS_SLAVE;
    modbus_func         = Config::MODBUS_FUNC_CODE;
    modbus_start_reg    = Config::MODBUS_START_REG;
    modbus_num_regs     = Config::MODBUS_NUM_REGS;
    sensor_zero_level   = Config::SENSOR_ZERO_LEVEL;
    sensor_divider      = Config::SENSOR_DIVIDER;

    ntp_enabled = true;
    copyStr(ntp_host, sizeof(ntp_host), "pool.ntp.org");
    ntp_resync_sec = 86400;

    // Channel defaults
    eth_enabled     = true;
    gsm_enabled     = true;
    wifi_enabled    = false;
    iridium_enabled = false;
    chain_enabled   = false;
    chain_order[0]  = 0; chain_order[1] = 1;
    chain_order[2]  = 2; chain_order[3] = 3;
    chain_count     = 2;

    // MQTT defaults
    mqtt_host[0] = 0;
    mqtt_port    = 1883;
    mqtt_user[0] = 0;
    mqtt_pass[0] = 0;
    copyStr(mqtt_topic, sizeof(mqtt_topic), "v1/devices/me/telemetry");
    mqtt_qos = 1;
    mqtt_tls = false;
    protocol = ProtocolMode::HTTPS_GENERIC;

    // Webhook defaults
    webhook_url[0] = 0;
    copyStr(webhook_method, sizeof(webhook_method), "POST");
    webhook_headers[0]          = 0;
    webhook_payload_template[0] = 0;
    webhook_trigger              = WebhookTrigger::Event;
    webhook_interval_sec         = 300;

    // UART port defaults: port 0 = USART3 modbus, others disabled
    uart_ports[0] = {9600, 2, 2, 1};
    uart_ports[1] = {9600, 0, 1, 0};
    uart_ports[2] = {9600, 0, 1, 0};

    modbus_map_count       = 0;
    avg_count              = 1;
    backup_send_interval_sec = 600;
    battery_low_pct        = 20;
    wifi_ssid[0]           = 0;
    wifi_pass[0]           = 0;
}

// -----------------------------------------------------------------------------
// Validation
// -----------------------------------------------------------------------------
bool RuntimeConfig::validateAndFix() {
    bool ok = true;
    if (metric_id[0] == 0)  ok = false;
    if (server_url[0] == 0) ok = false;
    if (poll_interval_sec == 0)  poll_interval_sec = 1;
    if (send_interval_polls == 0) send_interval_polls = 1;
    if (modbus_slave == 0) modbus_slave = 1;
    if (modbus_func != 3 && modbus_func != 4) modbus_func = 4;
    if (modbus_num_regs == 0)  modbus_num_regs = 2;
    if (modbus_num_regs > 30)  modbus_num_regs = 30;
    if (sensor_divider == 0.0f) sensor_divider = 1000.0f;
    if (ntp_resync_sec < 3600)  ntp_resync_sec = 3600;
    if (ntp_host[0] == 0) copyStr(ntp_host, sizeof(ntp_host), "pool.ntp.org");
    if (avg_count == 0)   avg_count = 1;
    if (avg_count > 100)  avg_count = 100;
    if (modbus_map_count > MAX_MODBUS_ENTRIES) modbus_map_count = MAX_MODBUS_ENTRIES;
    if (chain_count > MAX_CHAIN_ORDER) chain_count = MAX_CHAIN_ORDER;
    if (mqtt_qos > 2) mqtt_qos = 1;
    if (backup_send_interval_sec < 60) backup_send_interval_sec = 60;

    for (uint8_t i = 0; i < MAX_UART_PORTS; i++) {
        if (uart_ports[i].baud < 1200) uart_ports[i].baud = 9600;
        if (uart_ports[i].parity > 2)  uart_ports[i].parity = 0;
        if (uart_ports[i].stopbits < 1 || uart_ports[i].stopbits > 2) uart_ports[i].stopbits = 1;
        if (uart_ports[i].mode > 2) uart_ports[i].mode = 0;
    }
    for (uint8_t i = 0; i < modbus_map_count; i++) {
        auto& e = modbus_map[i];
        if (e.port_idx >= MAX_UART_PORTS) e.port_idx = 0;
        if (e.slave_id == 0) e.slave_id = 1;
        if (e.function != 3 && e.function != 4) e.function = 4;
        if (e.count == 0)  e.count = 1;
        if (e.count > 30)  e.count = 30;
        if (e.data_type > 2) e.data_type = 0;
        if (e.scale == 0.0f)      e.scale = 1.0f;
        if (e.multiplier == 0.0f) e.multiplier = 1.0f;
    }
    return ok;
}

// -----------------------------------------------------------------------------
// Mini JSON parser
// -----------------------------------------------------------------------------
static const char* skipWs(const char* p) {
    while (p && *p && std::isspace((unsigned char)*p)) p++;
    return p;
}
static const char* findKey(const char* json, const char* key) {
    if (!json || !key) return nullptr;
    char pat[80];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = std::strstr(json, pat);
    if (!p) return nullptr;
    p += std::strlen(pat);
    p = skipWs(p);
    if (*p != ':') return nullptr;
    p++;
    return skipWs(p);
}
static bool jsonGetString(const char* json, const char* key, char* out, size_t outSz) {
    const char* p = findKey(json, key);
    if (!p) return false;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && (i + 1) < outSz) out[i++] = *p++;
    out[i] = 0;
    return (*p == '"');
}
static bool jsonGetU32(const char* json, const char* key, uint32_t& out) {
    const char* p = findKey(json, key);
    if (!p) return false;
    out = (uint32_t)std::strtoul(p, nullptr, 10);
    return true;
}
static bool jsonGetU16(const char* json, const char* key, uint16_t& out) {
    const char* p = findKey(json, key);
    if (!p) return false;
    out = (uint16_t)std::strtoul(p, nullptr, 10);
    return true;
}
static bool jsonGetU8(const char* json, const char* key, uint8_t& out) {
    const char* p = findKey(json, key);
    if (!p) return false;
    out = (uint8_t)std::strtoul(p, nullptr, 10);
    return true;
}
static void normalizeHost(char* s, size_t sz) {
    if (!s || sz == 0 || s[0] == 0) return;
    const char* p = s;
    if (std::strncmp(p, "https://", 8) == 0) p += 8;
    else if (std::strncmp(p, "http://", 7) == 0) p += 7;
    char tmp[128]{};
    size_t i = 0;
    while (p[i] && p[i] != '/' && i < sizeof(tmp) - 1) {
        tmp[i] = p[i];
        ++i;
    }
    tmp[i] = 0;
    std::strncpy(s, tmp, sz - 1);
    s[sz - 1] = 0;
}

static bool jsonGetBool(const char* json, const char* key, bool& out) {
    const char* p = findKey(json, key);
    if (!p) return false;
    if (std::strncmp(p, "true",  4) == 0) { out = true;  return true; }
    if (std::strncmp(p, "false", 5) == 0) { out = false; return true; }
    return false;
}
static bool jsonGetF32(const char* json, const char* key, float& out) {
    const char* p = findKey(json, key);
    if (!p) return false;
    out = std::strtof(p, nullptr);
    return true;
}
static bool jsonGetI32(const char* json, const char* key, int& out) {
    const char* p = findKey(json, key);
    if (!p) return false;
    out = std::atoi(p);
    return true;
}
static bool parseIpv4Str(const char* s, uint8_t out[4]) {
    if (!s) return false;
    unsigned a=0,b=0,c=0,d=0;
    if (std::sscanf(s, "%u.%u.%u.%u", &a,&b,&c,&d) != 4) return false;
    if (a>255||b>255||c>255||d>255) return false;
    out[0]=(uint8_t)a; out[1]=(uint8_t)b; out[2]=(uint8_t)c; out[3]=(uint8_t)d;
    return true;
}
static bool jsonGetIpv4(const char* json, const char* key, uint8_t out[4]) {
    char tmp[32];
    if (!jsonGetString(json, key, tmp, sizeof(tmp))) return false;
    return parseIpv4Str(tmp, out);
}
static bool parseMacStr(const char* s, uint8_t out[6]) {
    if (!s) return false;
    unsigned m[6]{};
    if (std::sscanf(s, "%x:%x:%x:%x:%x:%x", &m[0],&m[1],&m[2],&m[3],&m[4],&m[5]) != 6) return false;
    for (int i=0;i<6;i++) if (m[i] > 0xFF) return false;
    for (int i=0;i<6;i++) out[i] = (uint8_t)m[i];
    return true;
}
static bool jsonGetMac(const char* json, const char* key, uint8_t out[6]) {
    char tmp[32];
    if (!jsonGetString(json, key, tmp, sizeof(tmp))) return false;
    return parseMacStr(tmp, out);
}
static void ftoa6(float x, char* out, size_t outSz) {
    if (!out || outSz < 4) return;
    bool neg = (x < 0.0f);
    if (neg) x = -x;
    uint32_t ip = (uint32_t)x;
    float frac  = x - (float)ip;
    uint32_t fp = (uint32_t)(frac * 1000000.0f + 0.5f);
    if (fp >= 1000000UL) { ip += 1; fp = 0; }
    if (neg) std::snprintf(out, outSz, "-%lu.%06lu", (unsigned long)ip, (unsigned long)fp);
    else     std::snprintf(out, outSz,  "%lu.%06lu", (unsigned long)ip, (unsigned long)fp);
}

// -----------------------------------------------------------------------------
// JSON object/array helpers
// -----------------------------------------------------------------------------
static const char* findMatchingBrace(const char* p) {
    if (!p || *p != '{') return nullptr;
    int depth = 0; bool inStr = false;
    while (*p) {
        if (*p == '"' && (p == nullptr || *(p-1) != '\\')) inStr = !inStr;
        if (!inStr) {
            if (*p == '{') depth++;
            else if (*p == '}') { depth--; if (depth == 0) return p + 1; }
        }
        p++;
    }
    return nullptr;
}
static const char* findArrayStart(const char* json, const char* key) {
    const char* p = findKey(json, key);
    if (!p || *p != '[') return nullptr;
    return p + 1;
}

// -----------------------------------------------------------------------------
// parseUartPortCfg — handles both canonical names and UI aliases
// -----------------------------------------------------------------------------
static void parseUartPortCfg(const char* obj, UartPortCfg& cfg) {
    jsonGetU32(obj, "baud",     cfg.baud);
    jsonGetU8 (obj, "parity",   cfg.parity);
    jsonGetU8 (obj, "stopbits", cfg.stopbits);
    jsonGetU8 (obj, "mode",     cfg.mode);
    // UI aliases: rtu[].sb = stopbits, rtu[].par = parity string
    jsonGetU8(obj, "sb", cfg.stopbits);
    char parStr[8]{};
    if (jsonGetString(obj, "par", parStr, sizeof(parStr))) {
        if      (std::strcmp(parStr,"None")==0 || std::strcmp(parStr,"none")==0) cfg.parity = 0;
        else if (std::strcmp(parStr,"Odd") ==0 || std::strcmp(parStr,"odd") ==0) cfg.parity = 1;
        else if (std::strcmp(parStr,"Even")==0 || std::strcmp(parStr,"even")==0) cfg.parity = 2;
    }
    // rms / fms — нет полей в UartPortCfg, игнорируются
}

static void parseModbusRegEntry(const char* obj, ModbusRegEntry& e) {
    jsonGetU8 (obj, "port_idx",   e.port_idx);
    jsonGetU8 (obj, "slave_id",   e.slave_id);
    jsonGetU8 (obj, "function",   e.function);
    jsonGetU16(obj, "start_reg",  e.start_reg);
    jsonGetU16(obj, "count",      e.count);
    jsonGetU8 (obj, "data_type",  e.data_type);
    jsonGetF32(obj, "scale",      e.scale);
    jsonGetF32(obj, "zero_offset",e.zero_offset);
    jsonGetF32(obj, "multiplier", e.multiplier);
    jsonGetString(obj, "unit", e.unit, sizeof(e.unit));
    jsonGetString(obj, "name", e.name, sizeof(e.name));
}

static uint8_t parseObjectArray(const char* json, const char* key,
    void (*parseFn)(const char*, void*),
    void* arr, size_t elemSize, uint8_t maxCount)
{
    const char* p = findArrayStart(json, key);
    if (!p) return 0;
    uint8_t count = 0;
    while (*p && *p != ']' && count < maxCount) {
        p = skipWs(p);
        if (*p == ',') { p++; p = skipWs(p); }
        if (*p != '{') break;
        const char* end = findMatchingBrace(p);
        if (!end) break;
        size_t objLen = (size_t)(end - p);
        if (objLen > 512) objLen = 512;
        char objBuf[513];
        std::memcpy(objBuf, p, objLen);
        objBuf[objLen] = 0;
        parseFn(objBuf, (uint8_t*)arr + count * elemSize);
        count++;
        p = end;
    }
    return count;
}
static void parseUartCb  (const char* obj, void* dest) { parseUartPortCfg  (obj, *(UartPortCfg*)dest); }

static void parseModbusCb(const char* obj, void* dest) { parseModbusRegEntry(obj, *(ModbusRegEntry*)dest); }

// -----------------------------------------------------------------------------
// RTU device / port parsers
// -----------------------------------------------------------------------------
static const char* dtToStr(uint8_t dt) {
    switch(dt) {
        case 0: return "INT16";
        case 1: return "UINT16";
        case 2: return "INT32_BE";
        case 3: return "UINT32_BE";
        case 4: return "FLOAT32_BE";
        default: return "INT16";
    }
}
static uint8_t strToDt(const char* s) {
    if (std::strcmp(s,"UINT16")    ==0) return 1;
    if (std::strcmp(s,"INT32_BE")  ==0) return 2;
    if (std::strcmp(s,"UINT32_BE") ==0) return 3;
    if (std::strcmp(s,"FLOAT32_BE")==0) return 4;
    return 0; // INT16 default
}

static void parseRtuDeviceCfg(const char* obj, ModbusDeviceCfg& d) {
    jsonGetBool(obj, "en", d.enabled);
    jsonGetU8  (obj, "sa", d.slave_addr);
    jsonGetString(obj, "nm", d.name, sizeof(d.name));
    jsonGetU8  (obj, "fc", d.func_code);
    jsonGetU16 (obj, "rs", d.reg_start);
    jsonGetU8  (obj, "rc", d.reg_count);
    jsonGetF32 (obj, "sc", d.scale);
    jsonGetF32 (obj, "of", d.offset);
    jsonGetString(obj, "un", d.unit, sizeof(d.unit));
    jsonGetU8  (obj, "ci", d.channel_idx);
    char dtStr[16]{};
    if (jsonGetString(obj, "dt", dtStr, sizeof(dtStr)))
        d.data_type = strToDt(dtStr);
}

static void parseRtuPortInner(const char* obj, ModbusRtuPortConfig& rp) {
    jsonGetBool(obj, "en",  rp.enabled);
    jsonGetU32 (obj, "baud",rp.baudrate);
    jsonGetU8  (obj, "sb",  rp.stop_bits);
    jsonGetU16 (obj, "rms", rp.response_timeout_ms);
    jsonGetU16 (obj, "fms", rp.inter_frame_ms);
    char parStr[8]{};
    if (jsonGetString(obj, "par", parStr, sizeof(parStr))) {
        if      (std::strcmp(parStr,"None")==0 || std::strcmp(parStr,"none")==0) rp.parity = 0;
        else if (std::strcmp(parStr,"Even")==0 || std::strcmp(parStr,"even")==0) rp.parity = 1;
        else if (std::strcmp(parStr,"Odd") ==0 || std::strcmp(parStr,"odd") ==0) rp.parity = 2;
    }
    // parse devs[] array inline
    const char* devsKey = "\"devs\"";
    const char* dp = std::strstr(obj, devsKey);
    if (!dp) return;
    dp += std::strlen(devsKey);
    while (*dp && *dp != '[') dp++;
    if (!dp || *dp != '[') return;
    dp++;
    rp.device_count = 0;
    while (*dp && *dp != ']' && rp.device_count < ModbusRtuPortConfig::MAX_DEVICES) {
        while (*dp && *dp != '{' && *dp != ']') dp++;
        if (!*dp || *dp == ']') break;
        // find matching '}'
        const char* objStart = dp;
        int depth = 0;
        const char* p = dp;
        while (*p) {
            if (*p == '{') depth++;
            else if (*p == '}') { depth--; if (depth==0){p++;break;} }
            p++;
        }
        static char devBuf[256];
        size_t devLen = (size_t)(p - objStart);
        if (devLen >= sizeof(devBuf)) devLen = sizeof(devBuf)-1;
        std::memcpy(devBuf, objStart, devLen);
        devBuf[devLen] = '\0';
        parseRtuDeviceCfg(devBuf, rp.devices[rp.device_count]);
        rp.device_count++;
        dp = p;
        while (*dp && *dp == ',') dp++;
    }
}
static void parseRtuPortCb(const char* obj, void* dest) {
    parseRtuPortInner(obj, *(ModbusRtuPortConfig*)dest);
}

static uint8_t parseU8Array(const char* json, const char* key, uint8_t* out, uint8_t maxCount) {
    const char* p = findKey(json, key);
    if (!p || *p != '[') return 0;
    p++;
    uint8_t count = 0;
    while (*p && *p != ']' && count < maxCount) {
        p = skipWs(p);
        if (*p == ',') { p++; p = skipWs(p); }
        if (std::isdigit((unsigned char)*p)) {
            out[count++] = (uint8_t)std::strtoul(p, nullptr, 10);
            while (*p && std::isdigit((unsigned char)*p)) p++;
        } else break;
    }
    return count;
}

static bool parseBoolArray(const char* json, const char* key, bool* out, uint8_t maxLen) {
    char needle[48];
    std::snprintf(needle, sizeof(needle), "\"%s\":[", key);
    const char* p = std::strstr(json, needle);
    if (!p) return false;
    p += std::strlen(needle);
    uint8_t idx = 0;
    while (*p && *p != ']' && idx < maxLen) {
        while (*p == ' ' || *p == '\t' || *p == ',') ++p;
        if      (std::strncmp(p,"true", 4)==0) { out[idx++]=true;  p+=4; }
        else if (std::strncmp(p,"false",5)==0) { out[idx++]=false; p+=5; }
        else ++p;
    }
    return idx > 0;
}

// -----------------------------------------------------------------------------
// loadFromJson
// -----------------------------------------------------------------------------
bool RuntimeConfig::loadFromJson(const char* json, size_t len) {
    if (!json || len < 2) return false;
    // Use global scratch buffer to avoid placing a full RuntimeConfig copy (~8KB) on the task stack.
    RuntimeConfig& tmp = CfgBackup();
    tmp = *this;

    // --- Legacy fields (backward compat) ---
    (void)jsonGetBool  (json, "complex_enabled", tmp.complex_enabled);
    (void)jsonGetString(json, "metric_id",  tmp.metric_id,  sizeof(tmp.metric_id));
    (void)jsonGetString(json, "complex_id", tmp.complex_id, sizeof(tmp.complex_id));

    // ETH mode alias: UI sends eth_dhcp=bool
    { bool dhcp = false;
      if (jsonGetBool(json, "eth_dhcp", dhcp))
          tmp.eth_mode = dhcp ? EthMode::Dhcp : EthMode::Static; }

    // Chain order / enable arrays from UI
    { uint8_t ord[MAX_CHAIN_ORDER]{};
      uint8_t cnt = parseU8Array(json, "chain_order", ord, MAX_CHAIN_ORDER);
      if (cnt > 0) {
          std::memcpy(tmp.channels.chain_order, ord, cnt);
          tmp.chain_count = cnt;
          tmp.channels.chain_count = cnt;
      } }

    // co_en: boolean array [true,false,...] -> channels.eth/gsm/...
    { bool en[4]{};
      if (parseBoolArray(json, "co_en", en, 4)) {
          tmp.channels.eth_enabled     = en[0];
          tmp.channels.gsm_enabled     = en[1];
          tmp.channels.wifi_enabled    = en[2];
          tmp.channels.iridium_enabled = en[3];
      } }

    (void)jsonGetString(json, "server_url",      tmp.server_url,      sizeof(tmp.server_url));
    (void)jsonGetString(json, "server_auth_b64", tmp.server_auth_b64, sizeof(tmp.server_auth_b64));

    char ethMode[16]{};
    if (jsonGetString(json, "eth_mode", ethMode, sizeof(ethMode))) {
        if (std::strcmp(ethMode,"dhcp")   == 0) tmp.eth_mode = EthMode::Dhcp;
        if (std::strcmp(ethMode,"static") == 0) tmp.eth_mode = EthMode::Static;
    }
    (void)jsonGetMac (json, "w5500_mac", tmp.w5500_mac);
    (void)jsonGetIpv4(json, "eth_ip",    tmp.eth_ip);
    (void)jsonGetIpv4(json, "eth_sn",    tmp.eth_sn);
    (void)jsonGetIpv4(json, "eth_gw",    tmp.eth_gw);
    (void)jsonGetIpv4(json, "eth_dns",   tmp.eth_dns);

    (void)jsonGetString(json, "gsm_apn",  tmp.gsm_apn,  sizeof(tmp.gsm_apn));
    (void)jsonGetString(json, "gsm_user", tmp.gsm_user, sizeof(tmp.gsm_user));
    (void)jsonGetString(json, "gsm_pass", tmp.gsm_pass, sizeof(tmp.gsm_pass));

    (void)jsonGetU32(json, "poll_interval_sec",   tmp.poll_interval_sec);
    (void)jsonGetU32(json, "send_interval_polls",  tmp.send_interval_polls);
    (void)jsonGetU8 (json, "modbus_slave",         tmp.modbus_slave);
    (void)jsonGetU8 (json, "modbus_func",          tmp.modbus_func);
    (void)jsonGetU16(json, "modbus_start_reg",     tmp.modbus_start_reg);
    (void)jsonGetU16(json, "modbus_num_regs",      tmp.modbus_num_regs);
    (void)jsonGetF32(json, "sensor_zero_level",    tmp.sensor_zero_level);
    (void)jsonGetF32(json, "sensor_divider",       tmp.sensor_divider);

    (void)jsonGetBool  (json, "ntp_enabled",   tmp.ntp_enabled);
    (void)jsonGetString(json, "ntp_host",      tmp.ntp_host,      sizeof(tmp.ntp_host));
    (void)jsonGetU32   (json, "ntp_resync_sec",tmp.ntp_resync_sec);

    // Channel flags
    (void)jsonGetBool(json, "eth_enabled",     tmp.eth_enabled);
    (void)jsonGetBool(json, "gsm_enabled",     tmp.gsm_enabled);
    (void)jsonGetBool(json, "wifi_enabled",    tmp.wifi_enabled);
    (void)jsonGetBool(json, "iridium_enabled", tmp.iridium_enabled);

    // Priority chain
    (void)jsonGetBool(json, "chain_enabled", tmp.chain_enabled);
    { uint8_t cnt = parseU8Array(json, "chain_order", tmp.chain_order, MAX_CHAIN_ORDER);
      if (cnt > 0) tmp.chain_count = cnt; }

    // MQTT
    (void)jsonGetString(json, "mqtt_host",  tmp.proto.mqtt_host,  sizeof(tmp.proto.mqtt_host));
    (void)jsonGetU16   (json, "mqtt_port",  tmp.proto.mqtt_port);
    (void)jsonGetString(json, "mqtt_user",  tmp.proto.mqtt_user,  sizeof(tmp.proto.mqtt_user));
    (void)jsonGetString(json, "mqtt_pass",  tmp.proto.mqtt_pass,  sizeof(tmp.proto.mqtt_pass));
    (void)jsonGetString(json, "mqtt_topic", tmp.proto.mqtt_topic, sizeof(tmp.proto.mqtt_topic));
    (void)jsonGetU8    (json, "mqtt_qos",   tmp.proto.mqtt_qos);
    (void)jsonGetBool  (json, "mqtt_tls",   tmp.mqtt_tls);

    // Protocol (long form)
    { char proto[16]{};
      if (jsonGetString(json, "protocol", proto, sizeof(proto))) {
          if (std::strcmp(proto,"mqtt")==0 || std::strcmp(proto,"MQTT")==0)
              tmp.protocol = ProtocolMode::MQTT_GENERIC;
          else
              tmp.protocol = ProtocolMode::HTTPS_GENERIC;
      } }

    // Webhook
    (void)jsonGetString(json, "webhook_url",              tmp.webhook_url,              sizeof(tmp.webhook_url));
    (void)jsonGetString(json, "webhook_method",           tmp.webhook_method,           sizeof(tmp.webhook_method));
    (void)jsonGetString(json, "webhook_headers",          tmp.webhook_headers,          sizeof(tmp.webhook_headers));
    (void)jsonGetString(json, "webhook_payload_template", tmp.webhook_payload_template, sizeof(tmp.webhook_payload_template));
    (void)jsonGetU32   (json, "webhook_interval_sec",     tmp.webhook_interval_sec);
    { char trigStr[16]{};
      if (jsonGetString(json, "webhook_trigger", trigStr, sizeof(trigStr)))
          tmp.webhook_trigger = (std::strcmp(trigStr,"schedule")==0)
              ? WebhookTrigger::Schedule : WebhookTrigger::Event; }

    // UART ports — canonical name, then UI alias "rtu"
    tmp.uart_ports[0] = {9600, 2, 2, 1};
    tmp.uart_ports[1] = {9600, 0, 1, 0};
    tmp.uart_ports[2] = {9600, 0, 1, 0};
    uint8_t uartCnt = parseObjectArray(json, "uart_ports", parseUartCb,
                                       tmp.uart_ports, sizeof(UartPortCfg), MAX_UART_PORTS);
    if (uartCnt == 0)
        uartCnt = parseObjectArray(json, "rtu", parseUartCb,
                                   tmp.uart_ports, sizeof(UartPortCfg), MAX_UART_PORTS);
    (void)uartCnt;

    // rtu[] -> rtu_ports[] (ModbusRtuPortConfig — new structure)
    { uint8_t rtuCnt = parseObjectArray(json, "rtu", parseRtuPortCb,
                                        tmp.rtu_ports, sizeof(ModbusRtuPortConfig), MAX_RTU_PORTS);
      (void)rtuCnt; }

    // Modbus map — canonical name, then UI alias "regs"
    tmp.modbus_map_count = parseObjectArray(json, "modbus_map", parseModbusCb,
                                            tmp.modbus_map, sizeof(ModbusRegEntry), MAX_MODBUS_ENTRIES);
    if (tmp.modbus_map_count == 0)
        tmp.modbus_map_count = parseObjectArray(json, "regs", parseModbusCb,
                                                tmp.modbus_map, sizeof(ModbusRegEntry), MAX_MODBUS_ENTRIES);

    // Misc
    (void)jsonGetU8 (json, "avg_count",              tmp.avg_count);
    (void)jsonGetU32(json, "backup_send_interval_sec",tmp.backup_send_interval_sec);
    (void)jsonGetU8 (json, "battery_low_pct",         tmp.battery_low_pct);
    (void)jsonGetString(json, "web_user", tmp.web.web_user, sizeof(tmp.web.web_user));
    (void)jsonGetString(json, "web_pass", tmp.web.web_pass, sizeof(tmp.web.web_pass));
    (void)jsonGetBool  (json, "web_auth_enabled", tmp.web.web_auth_enabled);
    (void)jsonGetString(json, "wifi_ssid", tmp.wifi_ssid, sizeof(tmp.wifi_ssid));
    (void)jsonGetString(json, "wifi_pass", tmp.wifi_pass, sizeof(tmp.wifi_pass));

    // Sync legacy -> nested structs
    if (tmp.poll_interval_sec  > 0) tmp.meas.poll_interval_s  = tmp.poll_interval_sec;
    if (tmp.send_interval_polls> 0) tmp.meas.send_interval_s  = tmp.send_interval_polls * tmp.poll_interval_sec;
    if (tmp.backup_send_interval_sec > 0) tmp.meas.backup_retry_s = tmp.backup_send_interval_sec;
    tmp.channels.eth_enabled     = tmp.eth_enabled;
    tmp.channels.gsm_enabled     = tmp.gsm_enabled;
    tmp.channels.wifi_enabled    = tmp.wifi_enabled;
    tmp.channels.iridium_enabled = tmp.iridium_enabled;
    tmp.channels.chain_mode      = tmp.chain_enabled;

    // ==========================================================================
    // UI field aliases — placed LAST to override any legacy keys above
    // ==========================================================================

    // Channel enable flags: ch_eth / ch_gsm / ch_wifi / ch_iridium
    { bool en=false; if(jsonGetBool(json,"ch_eth",     en)){ tmp.eth_enabled     =en; tmp.channels.eth_enabled     =en; } }
    { bool en=false; if(jsonGetBool(json,"ch_gsm",     en)){ tmp.gsm_enabled     =en; tmp.channels.gsm_enabled     =en; } }
    { bool en=false; if(jsonGetBool(json,"ch_wifi",    en)){ tmp.wifi_enabled    =en; tmp.channels.wifi_enabled    =en; } }
    { bool en=false; if(jsonGetBool(json,"ch_iridium", en)){ tmp.iridium_enabled =en; tmp.channels.iridium_enabled =en; } }
    { bool en=false; if(jsonGetBool(json,"chain_mode", en)){ tmp.chain_enabled   =en; tmp.channels.chain_mode      =en; } }

    // Timing: UI sends _s suffix
    { uint32_t v=0;
      if (jsonGetU32(json,"poll_interval_s",v) && v>0) {
          tmp.poll_interval_sec = v; tmp.meas.poll_interval_s = (uint16_t)v; }
      if (jsonGetU32(json,"send_interval_s",v) && v>0) {
          tmp.meas.send_interval_s = (uint16_t)v;
          tmp.send_interval_polls  = (tmp.poll_interval_sec>0)
              ? (uint32_t)(v/tmp.poll_interval_sec) : 1;
          if (tmp.send_interval_polls < 1) tmp.send_interval_polls = 1;
      }
      if (jsonGetU32(json,"backup_retry_s",v) && v>0) {
          tmp.backup_send_interval_sec = v; tmp.meas.backup_retry_s = (uint16_t)v; }
    }
    { uint8_t v8=0;
      if (jsonGetU8(json,"avg_count",v8) && v8>0) {
          tmp.avg_count = v8; tmp.meas.avg_count = v8; } }

    // Protocol alias: "proto" -> ProtocolMode enum
    { char pv[16]{};
      if (jsonGetString(json,"proto",pv,sizeof(pv))) {
          if      (std::strcmp(pv,"https_generic")==0) { tmp.protocol=ProtocolMode::HTTPS_GENERIC;    tmp.proto.mode=ProtocolMode::HTTPS_GENERIC;    }
          else if (std::strcmp(pv,"https_tb")     ==0) { tmp.protocol=ProtocolMode::HTTPS_GENERIC;    tmp.proto.mode=ProtocolMode::HTTPS_GENERIC;    }
          else if (std::strcmp(pv,"mqtt_tb")      ==0) { tmp.protocol=ProtocolMode::MQTT_THINGSBOARD; tmp.proto.mode=ProtocolMode::MQTT_THINGSBOARD; }
          else if (std::strcmp(pv,"mqtt_gen")     ==0) { tmp.protocol=ProtocolMode::MQTT_GENERIC;     tmp.proto.mode=ProtocolMode::MQTT_GENERIC;     }
          else if (std::strcmp(pv,"webhook")      ==0) { tmp.protocol=ProtocolMode::WEBHOOK_HTTP;     tmp.proto.mode=ProtocolMode::WEBHOOK_HTTP;     }
      } }

    // Legacy server aliases: tb_host / tb_token / tb_port (backward compat)
    { char s[64]{};
      if (jsonGetString(json,"tb_host", s,sizeof(s))) { normalizeHost(s, sizeof(s)); copyStr(tmp.proto.tb_host, sizeof(tmp.proto.tb_host), s); } }
    { char s[128]{};
      if (jsonGetString(json,"tb_token",s,sizeof(s))) copyStr(tmp.proto.tb_token,sizeof(tmp.proto.tb_token),s); }
    { uint16_t v16=0;
      if (jsonGetU16(json,"tb_port",v16) && v16>0) tmp.proto.tb_port = v16; }

    // Universal server fields
    { char s[64]{};
      if (jsonGetString(json,"server_host",s,sizeof(s))) { normalizeHost(s, sizeof(s)); copyStr(tmp.proto.server_host,sizeof(tmp.proto.server_host),s); } }
    { char s[128]{};
      if (jsonGetString(json,"server_token",s,sizeof(s))) copyStr(tmp.proto.server_token,sizeof(tmp.proto.server_token),s); }
    { char s[128]{};
      if (jsonGetString(json,"server_path",s,sizeof(s))) copyStr(tmp.proto.server_path,sizeof(tmp.proto.server_path),s); }
    { uint16_t v16=0;
      if (jsonGetU16(json,"server_port",v16) && v16>0) tmp.proto.server_port = v16; }

    // Backward compat: tb_host/tb_token -> server_host/server_token if server_host not set
    if (tmp.proto.tb_host[0] && !tmp.proto.server_host[0])
        copyStr(tmp.proto.server_host, sizeof(tmp.proto.server_host), tmp.proto.tb_host);
    normalizeHost(tmp.proto.tb_host, sizeof(tmp.proto.tb_host));
    normalizeHost(tmp.proto.server_host, sizeof(tmp.proto.server_host));
    if (tmp.proto.tb_token[0] && !tmp.proto.server_token[0])
        copyStr(tmp.proto.server_token, sizeof(tmp.proto.server_token), tmp.proto.tb_token);
    if (tmp.proto.tb_port && tmp.proto.tb_port != 443 && tmp.proto.server_port == 443)
        tmp.proto.server_port = tmp.proto.tb_port;

    // NTP: ntp_server / ntp_enabled / tz_off
    { char s[64]{};
      if (jsonGetString(json,"ntp_server",s,sizeof(s))) {
          copyStr(tmp.ntp_host,            sizeof(tmp.ntp_host),            s);
          copyStr(tmp.time_cfg.ntp_server, sizeof(tmp.time_cfg.ntp_server), s);
      } }
    { bool en=false;
      if (jsonGetBool(json,"ntp_enabled",en)) {
          tmp.ntp_enabled = en; tmp.time_cfg.ntp_enabled = en; } }
    { int iv=0;
      if (jsonGetI32(json,"tz_off",iv)) tmp.time_cfg.timezone_offset = (int8_t)iv; }

    // Deep sleep / schedule
    { bool en=false; if(jsonGetBool(json,"deep_sleep_enabled",en)) tmp.meas.deep_sleep_enabled=en; }
    { uint32_t v=0;  if(jsonGetU32 (json,"deep_sleep_s",      v))  tmp.meas.deep_sleep_s=(uint16_t)v; }
    { bool en=false; if(jsonGetBool(json,"schedule_enabled",  en)) tmp.meas.schedule_enabled=en; }
    { char s[16]{};
      if (jsonGetString(json,"schedule_start",s,sizeof(s))) {
          copyStr(tmp.meas.schedule_start,sizeof(tmp.meas.schedule_start),s); } }
    { char s[16]{};
      if (jsonGetString(json,"schedule_stop", s,sizeof(s))) {
          copyStr(tmp.meas.schedule_stop, sizeof(tmp.meas.schedule_stop), s); } }

    // Channel timing
    { uint32_t v=0;
      if (jsonGetU32(json,"channel_timeout_s",   v)) tmp.channels.channel_timeout_s    = (uint16_t)v;
      if (jsonGetU32(json,"channel_retry_s",     v)) tmp.channels.channel_retry_s      = (uint16_t)v;
      if (jsonGetU32(json,"channel_max_retries", v)) tmp.channels.channel_max_retries  = (uint8_t)v; }

    // Web UI settings
    { uint16_t v16=0;
      if (jsonGetU16(json,"web_port",          v16) && v16>0) tmp.web.web_port          = v16; }
    { uint16_t v16=0;
      if (jsonGetU16(json,"web_idle_timeout_s",v16) && v16>0) tmp.web.web_idle_timeout_s= v16; }
    { bool en=false;
      if (jsonGetBool(json,"web_exclusive_mode",en)) tmp.web.web_exclusive_mode = en; }

    // Modbus TCP master/slave
    { bool en=false; if(jsonGetBool(json,"mtcpm_en",en)) tmp.tcp_master.enabled       = en; }
    { bool en=false; if(jsonGetBool(json,"mtcps_en",en)) tmp.tcp_slave.enabled        = en; }
    { uint16_t v16=0;
      if (jsonGetU16(json,"sl_port",v16) && v16>0) tmp.tcp_slave.listen_port         = v16; }
    { uint8_t v8=0;
      if (jsonGetU8 (json,"sl_uid", v8))            tmp.tcp_slave.unit_id            = v8; }
    { uint8_t v8=0;
      if (jsonGetU8 (json,"sl_sock",v8))            tmp.tcp_slave.w5500_socket       = v8; }
    { uint16_t v16=0;
      if (jsonGetU16(json,"sl_ctms",v16))           tmp.tcp_slave.connection_timeout_ms = v16; }

    // Modbus TCP Master devices array
    {
        const char* tcpKey = "\"tcp_devs\":[";
        const char* tp = std::strstr(json, tcpKey);
        if (tp) {
            tp += std::strlen(tcpKey);
            tmp.tcp_master.device_count = 0;
            while (*tp && *tp != ']' &&
                   tmp.tcp_master.device_count < ModbusTcpMasterConfig::MAX_TCP_DEVICES) {
                while (*tp && *tp != '{' && *tp != ']') tp++;
                if (!*tp || *tp == ']') break;
                const char* os = tp; int dep = 0; const char* q = tp;
                while (*q) {
                    if (*q=='{') dep++;
                    else if (*q=='}') { dep--; if(dep==0){q++;break;} }
                    q++;
                }
                static char tb[512];
                size_t tl = (size_t)(q-os); if(tl>=sizeof(tb)) tl=sizeof(tb)-1;
                std::memcpy(tb,os,tl); tb[tl]='\0';
                ModbusTcpDeviceCfg& d = tmp.tcp_master.devices[tmp.tcp_master.device_count];
                jsonGetBool  (tb,"en",   d.enabled);
                jsonGetString(tb,"ip",   d.ip,   sizeof(d.ip));
                jsonGetU16   (tb,"port", d.port);
                jsonGetU8    (tb,"uid",  d.unit_id);
                jsonGetString(tb,"nm",   d.name, sizeof(d.name));
                jsonGetString(tb,"un",   d.unit, sizeof(d.unit));
                jsonGetU8    (tb,"fc",   d.func_code);
                jsonGetU16   (tb,"rs",   d.reg_start);
                jsonGetU8    (tb,"rc",   d.reg_count);
                { uint8_t dtv=0; if(jsonGetU8(tb,"dt",dtv)) d.data_type=dtv; }
                jsonGetF32   (tb,"sc",   d.scale);
                jsonGetF32   (tb,"of",   d.offset);
                jsonGetU8    (tb,"ci",   d.channel_idx);
                jsonGetU16   (tb,"pms",  d.poll_timeout_ms);
                jsonGetU16   (tb,"cms",  d.connect_timeout_ms);
                jsonGetU8    (tb,"sk",   d.w5500_socket);
                tmp.tcp_master.device_count++;
                tp = q;
                while (*tp == ',') tp++;
            }
        }
    }

    // Modbus TCP Slave register map
    {
        const char* slKey = "\"sl_regs\":[";
        const char* sp = std::strstr(json, slKey);
        if (sp) {
            sp += std::strlen(slKey);
            tmp.tcp_slave.reg_count = 0;
            while (*sp && *sp != ']' &&
                   tmp.tcp_slave.reg_count < ModbusTcpSlaveConfig::MAX_REGS) {
                while (*sp && *sp != '{' && *sp != ']') sp++;
                if (!*sp || *sp == ']') break;
                const char* os2 = sp; int dep2 = 0; const char* q2 = sp;
                while (*q2) {
                    if (*q2=='{') dep2++;
                    else if (*q2=='}') { dep2--; if(dep2==0){q2++;break;} }
                    q2++;
                }
                static char rb[256];
                size_t rl = (size_t)(q2-os2); if(rl>=sizeof(rb)) rl=sizeof(rb)-1;
                std::memcpy(rb,os2,rl); rb[rl]='\0';
                ModbusTcpSlaveRegEntry& re = tmp.tcp_slave.reg_map[tmp.tcp_slave.reg_count];
                jsonGetU16   (rb,"ad", re.reg_addr);
                jsonGetU8    (rb,"si", re.source);
                { uint8_t dtv=0; if(jsonGetU8(rb,"dt",dtv)) re.data_type=dtv; }
                jsonGetF32   (rb,"sc", re.scale);
                jsonGetString(rb,"un", re.unit, sizeof(re.unit));
                jsonGetString(rb,"nm", re.name, sizeof(re.name));
                tmp.tcp_slave.reg_count++;
                sp = q2;
                while (*sp == ',') sp++;
            }
        }
    }

    // Alerts
    { bool en=false;  if(jsonGetBool  (json,"alerts_enabled",          en)) tmp.alerts.alerts_enabled           = en; }
    { float fv=0.0f;  if(jsonGetF32   (json,"batt_low",               fv)) tmp.alerts.battery_low_threshold_pct= fv; }
    { bool en=false;  if(jsonGetBool  (json,"alert_on_channel_fail",   en)) tmp.alerts.alert_on_channel_fail    = en; }
    { bool en=false;  if(jsonGetBool  (json,"alert_on_sensor_fail",    en)) tmp.alerts.alert_on_sensor_fail     = en; }
    { char s[160]{};
      if (jsonGetString(json,"alert_webhook_url",s,sizeof(s)))
          copyStr(tmp.alerts.alert_webhook_url,sizeof(tmp.alerts.alert_webhook_url),s); }

    // ==========================================================================
    // Final sync: nested proto -> legacy root fields (for saveToSd compat)
    // ==========================================================================
    tmp.proto.mode = tmp.protocol;
    copyStr(tmp.mqtt_host,  sizeof(tmp.mqtt_host),  tmp.proto.mqtt_host);
    tmp.mqtt_port  = tmp.proto.mqtt_port;
    copyStr(tmp.mqtt_user,  sizeof(tmp.mqtt_user),  tmp.proto.mqtt_user);
    copyStr(tmp.mqtt_pass,  sizeof(tmp.mqtt_pass),  tmp.proto.mqtt_pass);
    copyStr(tmp.mqtt_topic, sizeof(tmp.mqtt_topic), tmp.proto.mqtt_topic);
    tmp.mqtt_qos   = tmp.proto.mqtt_qos;
    tmp.mqtt_tls   = tmp.proto.mqtt_tls;
    copyStr(tmp.webhook_url,    sizeof(tmp.webhook_url),    tmp.proto.webhook_url);
    copyStr(tmp.webhook_method, sizeof(tmp.webhook_method), tmp.proto.webhook_method);
    tmp.time_cfg.ntp_enabled = tmp.ntp_enabled;
    copyStr(tmp.time_cfg.ntp_server, sizeof(tmp.time_cfg.ntp_server), tmp.ntp_host);

    tmp.validateAndFix();
    *this = tmp;
    return true;
}

// -----------------------------------------------------------------------------
// loadFromSd
// -----------------------------------------------------------------------------
bool RuntimeConfig::loadFromSd(const char* filename) {
    setDefaultsFromConfig();
    FIL f;
    FRESULT fr = f_open(&f, filename, FA_READ);
    if (fr == FR_NO_FILE) {
        DBG.warn("CFG: %s not found, creating with defaults", filename);
        validateAndFix();
        if (saveToSd(filename)) DBG.info("CFG: %s created OK", filename);
        else DBG.error("CFG: failed to create %s on SD", filename);
        return false;
    }
    if (fr != FR_OK) {
        DBG.warn("CFG: f_open error %d -> defaults", (int)fr);
        validateAndFix();
        return false;
    }
    static char buf[10240];
    UINT br = 0;
    fr = f_read(&f, buf, sizeof(buf) - 1, &br);
    f_close(&f);
    if (fr != FR_OK || br == 0) {
        DBG.error("CFG: read fail fr=%d br=%u", (int)fr, (unsigned)br);
        validateAndFix();
        return false;
    }
    DBG.info("CFG: read %u bytes from %s", (unsigned)br, filename);
    buf[br] = 0;
    if (!loadFromJson(buf, (size_t)br)) {
        DBG.error("CFG: JSON parse fail -> defaults");
        setDefaultsFromConfig();
        validateAndFix();
        return false;
    }
    DBG.info("CFG: loaded from %s (%u bytes)", filename, (unsigned)br);
    return true;
}

// -----------------------------------------------------------------------------
// saveToSd
// -----------------------------------------------------------------------------
bool RuntimeConfig::saveToSd(const char* filename) const {
    char macStr[24];
    std::snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
        w5500_mac[0],w5500_mac[1],w5500_mac[2],
        w5500_mac[3],w5500_mac[4],w5500_mac[5]);

    auto ipStr = [](const uint8_t ip[4], char* out, size_t outSz) {
        std::snprintf(out, outSz, "%u.%u.%u.%u", ip[0],ip[1],ip[2],ip[3]);
    };
    char ip[16], sn[16], gw[16], dns[16];
    ipStr(eth_ip, ip, sizeof(ip));
    ipStr(eth_sn, sn, sizeof(sn));
    ipStr(eth_gw, gw, sizeof(gw));
    ipStr(eth_dns,dns,sizeof(dns));

    const char* ethModeStr = (eth_mode == EthMode::Dhcp) ? "dhcp" : "static";
    char zStr[48], dStr[48];
    ftoa6(sensor_zero_level, zStr, sizeof(zStr));
    ftoa6(sensor_divider,    dStr, sizeof(dStr));

    static char json[20480];
    int n = 0;

    n += std::snprintf(json+n, sizeof(json)-n,
        "{\"v\":%lu,"
        "\"complex_enabled\":%s,"
        "\"metric_id\":\"%s\","
        "\"complex_id\":\"%s\","
        "\"server_url\":\"%s\","
        "\"server_auth_b64\":\"%s\","
        "\"eth_mode\":\"%s\","
        "\"w5500_mac\":\"%s\","
        "\"eth_ip\":\"%s\","
        "\"eth_sn\":\"%s\","
        "\"eth_gw\":\"%s\","
        "\"eth_dns\":\"%s\","
        "\"gsm_apn\":\"%s\","
        "\"gsm_user\":\"%s\","
        "\"gsm_pass\":\"%s\","
        "\"poll_interval_sec\":%lu,"
        "\"send_interval_polls\":%lu,"
        "\"modbus_slave\":%u,"
        "\"modbus_func\":%u,"
        "\"modbus_start_reg\":%u,"
        "\"modbus_num_regs\":%u,"
        "\"sensor_zero_level\":%s,"
        "\"sensor_divider\":%s,"
        "\"ntp_enabled\":%s,"
        "\"ntp_host\":\"%s\","
        "\"ntp_resync_sec\":%lu,",
        (unsigned long)VERSION,
        complex_enabled?"true":"false",
        metric_id, complex_id, server_url, server_auth_b64,
        ethModeStr, macStr, ip, sn, gw, dns,
        gsm_apn, gsm_user, gsm_pass,
        (unsigned long)poll_interval_sec,
        (unsigned long)send_interval_polls,
        (unsigned)modbus_slave, (unsigned)modbus_func,
        (unsigned)modbus_start_reg, (unsigned)modbus_num_regs,
        zStr, dStr,
        ntp_enabled?"true":"false", ntp_host,
        (unsigned long)ntp_resync_sec);
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n, sizeof(json)-n,
        "\"eth_enabled\":%s,"
        "\"gsm_enabled\":%s,"
        "\"wifi_enabled\":%s,"
        "\"iridium_enabled\":%s,",
        eth_enabled?"true":"false", gsm_enabled?"true":"false",
        wifi_enabled?"true":"false", iridium_enabled?"true":"false");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n, sizeof(json)-n,
        "\"chain_enabled\":%s,\"chain_order\":[",
        chain_enabled?"true":"false");
    for (uint8_t i=0;i<chain_count;i++)
        n += std::snprintf(json+n,sizeof(json)-n,"%s%u",(i>0)?",":"",(unsigned)chain_order[i]);
    n += std::snprintf(json+n,sizeof(json)-n,"],");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n,sizeof(json)-n,
        "\"mqtt_host\":\"%s\","
        "\"mqtt_port\":%u,"
        "\"mqtt_user\":\"%s\","
        "\"mqtt_pass\":\"%s\","
        "\"mqtt_topic\":\"%s\","
        "\"mqtt_qos\":%u,"
        "\"mqtt_tls\":%s,",
        mqtt_host,(unsigned)mqtt_port,
        mqtt_user,mqtt_pass,mqtt_topic,
        (unsigned)mqtt_qos, mqtt_tls?"true":"false");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n,sizeof(json)-n,
        "\"protocol\":\"%s\",",
        (protocol==ProtocolMode::MQTT_GENERIC||protocol==ProtocolMode::MQTT_THINGSBOARD)
            ?"mqtt":"https");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n,sizeof(json)-n,
        "\"webhook_url\":\"%s\","
        "\"webhook_method\":\"%s\","
        "\"webhook_headers\":\"%s\","
        "\"webhook_payload_template\":\"%s\","
        "\"webhook_trigger\":\"%s\","
        "\"webhook_interval_sec\":%lu,",
        webhook_url, webhook_method,
        webhook_headers, webhook_payload_template,
        (webhook_trigger==WebhookTrigger::Schedule)?"schedule":"event",
        (unsigned long)webhook_interval_sec);
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n,sizeof(json)-n,"\"uart_ports\":[");
    for (uint8_t i=0;i<MAX_UART_PORTS;i++)
        n += std::snprintf(json+n,sizeof(json)-n,
            "%s{\"baud\":%lu,\"parity\":%u,\"stopbits\":%u,\"mode\":%u}",
            (i>0)?",":"",
            (unsigned long)uart_ports[i].baud,
            (unsigned)uart_ports[i].parity,
            (unsigned)uart_ports[i].stopbits,
            (unsigned)uart_ports[i].mode);
    n += std::snprintf(json+n,sizeof(json)-n,"],");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    // rtu[] from rtu_ports[] (ModbusRtuPortConfig — new structure with devices)
    n += std::snprintf(json+n,sizeof(json)-n,"\"rtu\":[");
    for (uint8_t i=0;i<MAX_RTU_PORTS;i++) {
        const ModbusRtuPortConfig& rp = rtu_ports[i];
        const char* parStr = (rp.parity==1)?"Even":(rp.parity==2)?"Odd":"None";
        n += std::snprintf(json+n,sizeof(json)-n,
            "%s{\"en\":%s,\"baud\":%lu,\"sb\":%u,\"par\":\"%s\","
            "\"rms\":%u,\"fms\":%u,\"devs\":[",
            i==0?"":",",
            rp.enabled?"true":"false",
            (unsigned long)rp.baudrate,
            (unsigned)rp.stop_bits, parStr,
            (unsigned)rp.response_timeout_ms,
            (unsigned)rp.inter_frame_ms);
        if (n<0||n>=(int)sizeof(json)) goto overflow;
        for (uint8_t j=0;j<rp.device_count && j<ModbusRtuPortConfig::MAX_DEVICES;j++) {
            const ModbusDeviceCfg& d = rp.devices[j];
            n += std::snprintf(json+n,sizeof(json)-n,
                "%s{\"en\":%s,\"sa\":%u,\"nm\":\"%s\",\"fc\":%u,"
                "\"rs\":%u,\"rc\":%u,\"dt\":\"%s\","
                "\"sc\":%f,\"of\":%f,\"un\":\"%s\",\"ci\":%u}",
                j==0?"":",",
                d.enabled?"true":"false",
                (unsigned)d.slave_addr, d.name, (unsigned)d.func_code,
                (unsigned)d.reg_start, (unsigned)d.reg_count,
                dtToStr(d.data_type),
                (double)d.scale, (double)d.offset,
                d.unit, (unsigned)d.channel_idx);
            if (n<0||n>=(int)sizeof(json)) goto overflow;
        }
        n += std::snprintf(json+n,sizeof(json)-n,"]}");
        if (n<0||n>=(int)sizeof(json)) goto overflow;
    }
    n += std::snprintf(json+n,sizeof(json)-n,"],");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n,sizeof(json)-n,"\"modbus_map\":[");
    for (uint8_t i=0;i<modbus_map_count;i++) {
        const auto& e = modbus_map[i];
        char sStr[24],zoStr[24],mStr[24];
        ftoa6(e.scale,      sStr, sizeof(sStr));
        ftoa6(e.zero_offset,zoStr,sizeof(zoStr));
        ftoa6(e.multiplier, mStr, sizeof(mStr));
        n += std::snprintf(json+n,sizeof(json)-n,
            "%s{\"port_idx\":%u,\"slave_id\":%u,\"function\":%u,"
            "\"start_reg\":%u,\"count\":%u,\"data_type\":%u,"
            "\"scale\":%s,\"zero_offset\":%s,\"multiplier\":%s,"
            "\"unit\":\"%s\",\"name\":\"%s\"}",
            (i>0)?",":"",
            (unsigned)e.port_idx,(unsigned)e.slave_id,(unsigned)e.function,
            (unsigned)e.start_reg,(unsigned)e.count,(unsigned)e.data_type,
            sStr,zoStr,mStr,e.unit,e.name);
        if (n>=(int)sizeof(json)) goto overflow;
    }
    n += std::snprintf(json+n,sizeof(json)-n,"],");
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    n += std::snprintf(json+n,sizeof(json)-n,
        "\"avg_count\":%u,"
        "\"backup_send_interval_sec\":%lu,"
        "\"battery_low_pct\":%u,"
        "\"web_user\":\"%s\","
        "\"web_pass\":\"%s\","
        "\"wifi_ssid\":\"%s\","
        "\"wifi_pass\":\"%s\",",
        (unsigned)avg_count,
        (unsigned long)backup_send_interval_sec,
        (unsigned)battery_low_pct,
        web.web_user, web.web_pass,
        wifi_ssid, wifi_pass);
    if (n<0||n>=(int)sizeof(json)) goto overflow;

    {
        const char* protoStr = "https_generic";
        if      (proto.mode==ProtocolMode::MQTT_GENERIC)     protoStr = "mqtt_gen";
        else if (proto.mode==ProtocolMode::MQTT_THINGSBOARD) protoStr = "mqtt_tb";
        else if (proto.mode==ProtocolMode::WEBHOOK_HTTP)     protoStr = "webhook";

        n += std::snprintf(json+n,sizeof(json)-n,
            "\"proto\":\"%s\","
            "\"server_host\":\"%s\","
            "\"server_token\":\"%s\","
            "\"server_path\":\"%s\","
            "\"server_port\":%u,"
            "\"tb_host\":\"%s\","
            "\"tb_token\":\"%s\","
            "\"tb_port\":%u,"
            "\"poll_interval_s\":%u,"
            "\"send_interval_s\":%u,"
            "\"backup_retry_s\":%u,"
            "\"deep_sleep_enabled\":%s,"
            "\"deep_sleep_s\":%u,"
            "\"schedule_enabled\":%s,"
            "\"schedule_start\":\"%s\","
            "\"schedule_stop\":\"%s\","
            "\"chain_mode\":%s,"
            "\"channel_timeout_s\":%u,"
            "\"channel_retry_s\":%u,"
            "\"channel_max_retries\":%u,"
            "\"tz_off\":%d,"
            "\"ntp_server\":\"%s\","
            "\"ntp_enabled\":%s,"
            "\"web_auth_enabled\":%s,"
            "\"web_port\":%u,"
            "\"web_idle_timeout_s\":%u,"
            "\"web_exclusive_mode\":%s,"
            "\"alerts_enabled\":%s,"
            "\"batt_low\":%.2f,"
            "\"alert_on_channel_fail\":%s,"
            "\"alert_on_sensor_fail\":%s,"
            "\"alert_webhook_url\":\"%s\","
            "\"ch_eth\":%s,"
            "\"ch_gsm\":%s,"
            "\"ch_wifi\":%s,"
            "\"ch_iridium\":%s,"
            "\"mtcpm_en\":%s,"
            "\"mtcps_en\":%s,"
            "\"sl_port\":%u,"
            "\"sl_uid\":%u,"
            "\"sl_sock\":%u,"
            "\"sl_ctms\":%u"
            "}\n",
            protoStr,
            proto.server_host, proto.server_token, proto.server_path, (unsigned)proto.server_port,
            proto.server_host, proto.server_token, (unsigned)proto.server_port,
            (unsigned)meas.poll_interval_s, (unsigned)meas.send_interval_s,
            (unsigned)meas.backup_retry_s,
            meas.deep_sleep_enabled?"true":"false", (unsigned)meas.deep_sleep_s,
            meas.schedule_enabled?"true":"false",
            meas.schedule_start, meas.schedule_stop,
            channels.chain_mode?"true":"false",
            (unsigned)channels.channel_timeout_s,
            (unsigned)channels.channel_retry_s,
            (unsigned)channels.channel_max_retries,
            (int)time_cfg.timezone_offset, time_cfg.ntp_server,
            time_cfg.ntp_enabled?"true":"false",
            web.web_auth_enabled?"true":"false",
            (unsigned)web.web_port, (unsigned)web.web_idle_timeout_s,
            web.web_exclusive_mode?"true":"false",
            alerts.alerts_enabled?"true":"false",
            alerts.battery_low_threshold_pct,
            alerts.alert_on_channel_fail?"true":"false",
            alerts.alert_on_sensor_fail?"true":"false",
            alerts.alert_webhook_url,
            channels.eth_enabled?"true":"false",
            channels.gsm_enabled?"true":"false",
            channels.wifi_enabled?"true":"false",
            channels.iridium_enabled?"true":"false",
            tcp_master.enabled?"true":"false",
            tcp_slave.enabled?"true":"false",
            (unsigned)tcp_slave.listen_port,
            (unsigned)tcp_slave.unit_id,
            (unsigned)tcp_slave.w5500_socket,
            (unsigned)tcp_slave.connection_timeout_ms);
    }
    if (n<=0||n>=(int)sizeof(json)) goto overflow;

    // ── tcp_devs[] ────────────────────────────────────────────────────────────
    {
        char tb[256];
        int k = std::snprintf(json+n, sizeof(json)-n, ",\"tcp_devs\":[");
        if (k<0||n+k>=(int)sizeof(json)) goto overflow; n+=k;
        for (uint8_t i=0; i<tcp_master.device_count && i<ModbusTcpMasterConfig::MAX_TCP_DEVICES; i++) {
            const ModbusTcpDeviceCfg& d = tcp_master.devices[i];
            k = std::snprintf(tb, sizeof(tb),
                "%s{\"en\":%s,\"ip\":\"%s\",\"port\":%u,\"uid\":%u,\"nm\":\"%s\","
                "\"fc\":%u,\"rs\":%u,\"rc\":%u,\"dt\":%u,\"sc\":%.6g,\"of\":%.6g,"
                "\"un\":\"%s\",\"ci\":%u,\"pms\":%u,\"cms\":%u,\"sk\":%u}",
                i==0?"":",",
                d.enabled?"true":"false", d.ip,
                (unsigned)d.port, (unsigned)d.unit_id, d.name,
                (unsigned)d.func_code, (unsigned)d.reg_start, (unsigned)d.reg_count,
                (unsigned)d.data_type, (double)d.scale, (double)d.offset,
                d.unit, (unsigned)d.channel_idx,
                (unsigned)d.poll_timeout_ms, (unsigned)d.connect_timeout_ms,
                (unsigned)d.w5500_socket);
            if (k<0||n+k>=(int)sizeof(json)) goto overflow;
            std::memcpy(json+n, tb, (size_t)k); n+=k;
        }
        k = std::snprintf(json+n, sizeof(json)-n, "]");
        if (k<0||n+k>=(int)sizeof(json)) goto overflow; n+=k;
    }

    // ── sl_regs[] ─────────────────────────────────────────────────────────────
    {
        char rb[128];
        int k = std::snprintf(json+n, sizeof(json)-n, ",\"sl_regs\":[");
        if (k<0||n+k>=(int)sizeof(json)) goto overflow; n+=k;
        for (uint8_t i=0; i<tcp_slave.reg_count && i<ModbusTcpSlaveConfig::MAX_REGS; i++) {
            const ModbusTcpSlaveRegEntry& re = tcp_slave.reg_map[i];
            k = std::snprintf(rb, sizeof(rb),
                "%s{\"ad\":%u,\"si\":%u,\"dt\":%u,\"sc\":%.6g,\"un\":\"%s\",\"nm\":\"%s\"}",
                i==0?"":",",
                (unsigned)re.reg_addr, (unsigned)re.source, (unsigned)re.data_type,
                (double)re.scale, re.unit, re.name);
            if (k<0||n+k>=(int)sizeof(json)) goto overflow;
            std::memcpy(json+n, rb, (size_t)k); n+=k;
        }
        k = std::snprintf(json+n, sizeof(json)-n, "]");
        if (k<0||n+k>=(int)sizeof(json)) goto overflow; n+=k;
    }

    if (n<=0||n>=(int)sizeof(json)) goto overflow;

    // ── Запись на SD по чанкам 512 байт ──────────────────────────────────────
    {
        FIL f;
        char tmpName[64];
        std::snprintf(tmpName, sizeof(tmpName), "%s.tmp", filename);
        f_unlink(tmpName);
        FRESULT fr = f_open(&f, tmpName, FA_CREATE_ALWAYS|FA_WRITE);
        if (fr != FR_OK) { DBG.error("CFG: open temp file fail fr=%d",(int)fr); return false; }
        const uint32_t total = (uint32_t)n;
        uint32_t written = 0;
        bool wok = true;
        while (written < total) {
            UINT chunk = (UINT)((total - written) > 512u ? 512u : (total - written));
            UINT bw = 0;
            fr = f_write(&f, json + written, chunk, &bw);
            if (fr != FR_OK || bw != chunk) {
                DBG.error("CFG: write fail at %lu fr=%d bw=%u", (unsigned long)written, (int)fr, (unsigned)bw);
                wok = false; break;
            }
            written += bw;
        }
        f_close(&f);
        if (!wok) { f_unlink(tmpName); return false; }
        f_unlink(filename);
        fr = f_rename(tmpName, filename);
        if (fr!=FR_OK) { DBG.error("CFG: rename fail %s->%s fr=%d",tmpName,filename,(int)fr); return false; }
        DBG.info("CFG: saved %s (%lu bytes)", filename, (unsigned long)written);
        return true;
    }
overflow:
    DBG.error("CFG: save JSON overflow (n=%d max=%d)", n, (int)sizeof(json));
    return false;
}

// -----------------------------------------------------------------------------
// log
// -----------------------------------------------------------------------------
void RuntimeConfig::log() const {
    DBG.info("CFG: metric_id=%s", metric_id);
    DBG.info("CFG: complex_enabled=%d complex_id=%s", complex_enabled?1:0, complex_id);
    DBG.info("CFG: server_url=%s", server_url);
    DBG.info("CFG: eth_mode=%s", (eth_mode==EthMode::Dhcp)?"dhcp":"static");
    DBG.info("CFG: eth_ip=%u.%u.%u.%u", eth_ip[0],eth_ip[1],eth_ip[2],eth_ip[3]);
    DBG.info("CFG: poll=%lu sec, send_each=%lu polls",
        (unsigned long)poll_interval_sec,(unsigned long)send_interval_polls);
    DBG.info("CFG: modbus slave=%u fc=%u reg=%u n=%u",
        (unsigned)modbus_slave,(unsigned)modbus_func,
        (unsigned)modbus_start_reg,(unsigned)modbus_num_regs);
    DBG.info("CFG: zero=%.3f divider=%.3f",(double)sensor_zero_level,(double)sensor_divider);
    DBG.info("CFG: ntp=%d host=%s period=%lu sec",
        ntp_enabled?1:0, ntp_host,(unsigned long)ntp_resync_sec);
    DBG.info("CFG: proto=%d, ch: eth=%d gsm=%d wifi=%d iridium=%d",
        (int)protocol, eth_enabled?1:0, gsm_enabled?1:0,
        wifi_enabled?1:0, iridium_enabled?1:0);
    DBG.info("CFG: avg_count=%u backup=%lu sec",
        (unsigned)avg_count,(unsigned long)backup_send_interval_sec);
    DBG.info("CFG: modbus_map_count=%u",(unsigned)modbus_map_count);
    for (uint8_t i=0;i<modbus_map_count;i++) {
        const auto& e=modbus_map[i];
        DBG.info("  reg[%u]: port=%u slave=%u fc=%u start=%u cnt=%u name=%s",
            (unsigned)i,(unsigned)e.port_idx,(unsigned)e.slave_id,
            (unsigned)e.function,(unsigned)e.start_reg,(unsigned)e.count,e.name);
    }
}

// ============================================================================
// buildServerUrl — build effective HTTPS URL from config
// ============================================================================
void RuntimeConfig::buildServerUrl(char* out, size_t outSz) const {
    if (!out || outSz == 0) return;
    // If legacy server_url is set explicitly, use it as-is
    if (server_url[0]) {
        std::strncpy(out, server_url, outSz - 1);
        out[outSz - 1] = 0;
        return;
    }
    const char* host   = proto.server_host;
    const char* path   = proto.server_path[0] ? proto.server_path : "/api/ingest";
    uint16_t    port   = proto.server_port    ? proto.server_port  : 443;
    if (!host[0]) {
        out[0] = 0;
        DBG.error("CFG: buildServerUrl failed: empty server_host");
        return;
    }
    const char* scheme = (port == 80) ? "http" : "https";
    if (port == 443 || port == 80) {
        std::snprintf(out, outSz, "%s://%s%s", scheme, host, path);
    } else {
        std::snprintf(out, outSz, "%s://%s:%u%s", scheme, host, (unsigned)port, path);
    }
}
