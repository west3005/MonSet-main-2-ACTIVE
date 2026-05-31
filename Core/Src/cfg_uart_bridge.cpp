#include "cfg_uart_bridge.hpp"

#include "debug_uart.hpp"
#include "runtime_config.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "stm32f4xx_hal.h"
}

extern UART_HandleTypeDef huart6;

static char s_line[256];
static uint32_t s_lineLen = 0;

static uint32_t s_jsonNeed = 0;
static uint32_t s_jsonGot  = 0;
static char s_jsonBuf[2048];

static void uartSend(const char* s) {
  if (!s) return;
  HAL_UART_Transmit(&huart6, (uint8_t*)s, (uint16_t)std::strlen(s), 200);
}

static void uartSendLn(const char* s) {
  uartSend(s);
  uartSend("\n");
}

static void ipToStr(const uint8_t ip[4], char* out, size_t outSz) {
  std::snprintf(out, outSz, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

static void macToStr(const uint8_t mac[6], char* out, size_t outSz) {
  std::snprintf(out, outSz, "%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Мини‑конвертер float -> "[-]I.FFFFFF" без printf-float
static void ftoa6(float x, char* out, size_t outSz) {
  if (!out || outSz < 4) return;

  bool neg = (x < 0.0f);
  if (neg) x = -x;

  uint32_t ip = (uint32_t)x;
  float frac = x - (float)ip;
  uint32_t fp = (uint32_t)(frac * 1000000.0f + 0.5f);

  if (fp >= 1000000UL) { ip += 1; fp = 0; }

  if (neg) std::snprintf(out, outSz, "-%lu.%06lu", (unsigned long)ip, (unsigned long)fp);
  else     std::snprintf(out, outSz, "%lu.%06lu",  (unsigned long)ip, (unsigned long)fp);
}

static void emitKv(const char* key, const char* val) {
  char b[320];
  std::snprintf(b, sizeof(b), "%s=%s", key, val ? val : "");
  uartSendLn(b);
}

static void emitU32(const char* key, uint32_t v) {
  char b[32];
  std::snprintf(b, sizeof(b), "%lu", (unsigned long)v);
  emitKv(key, b);
}

static void emitU16(const char* key, uint16_t v) {
  char b[32];
  std::snprintf(b, sizeof(b), "%u", (unsigned)v);
  emitKv(key, b);
}

static void emitU8(const char* key, uint8_t v) {
  char b[32];
  std::snprintf(b, sizeof(b), "%u", (unsigned)v);
  emitKv(key, b);
}

static void emitF(const char* key, float v) {
  char b[48];
  ftoa6(v, b, sizeof(b));
  emitKv(key, b);
}

static void doGetCfg(void) {
  const RuntimeConfig& c = Cfg();

  uartSendLn("OK");

  emitKv("METRIC_ID", c.metric_id);
  emitKv("COMPLEX_ID", c.complex_id);
  emitKv("COMPLEX_ENABLED", c.complex_enabled ? "1" : "0");

  emitKv("SERVER_URL", c.server_url);
  emitKv("SERVER_AUTH_B64", c.server_auth_b64);

  emitKv("ETH_MODE", (c.eth_mode == RuntimeConfig::EthMode::Dhcp) ? "dhcp" : "static");

  char tmp[64];
  macToStr(c.w5500_mac, tmp, sizeof(tmp)); emitKv("W5500_MAC", tmp);
  ipToStr(c.eth_ip,  tmp, sizeof(tmp)); emitKv("ETH_IP", tmp);
  ipToStr(c.eth_sn,  tmp, sizeof(tmp)); emitKv("ETH_SN", tmp);
  ipToStr(c.eth_gw,  tmp, sizeof(tmp)); emitKv("ETH_GW", tmp);
  ipToStr(c.eth_dns, tmp, sizeof(tmp)); emitKv("ETH_DNS", tmp);

  emitKv("GSM_APN", c.gsm_apn);
  emitKv("GSM_USER", c.gsm_user);
  emitKv("GSM_PASS", c.gsm_pass);

  emitU32("POLL_INTERVAL_SEC", c.poll_interval_sec);
  emitU32("SEND_INTERVAL_POLLS", c.send_interval_polls);

  emitU8("MODBUS_SLAVE", c.modbus_slave);
  emitU8("MODBUS_FUNC", c.modbus_func);
  emitU16("MODBUS_START_REG", c.modbus_start_reg);
  emitU16("MODBUS_NUM_REGS", c.modbus_num_regs);

  emitF("SENSOR_ZERO_LEVEL", c.sensor_zero_level);
  emitF("SENSOR_DIVIDER", c.sensor_divider);

  emitKv("NTP_ENABLED", c.ntp_enabled ? "1" : "0");
  emitKv("NTP_HOST", c.ntp_host);
  emitU32("NTP_RESYNC_SEC", c.ntp_resync_sec);

  uartSendLn("END");
}

static void finishAndReboot(bool ok) {
  if (!ok) {
    uartSendLn("ERR");
    return;
  }

  (void)Cfg().validateAndFix();

  if (!Cfg().saveToSd(RUNTIME_CONFIG_FILENAME)) {
    uartSendLn("ERR SAVE");
    return;
  }

  uartSendLn("OK");
  HAL_Delay(50);
  NVIC_SystemReset();
}

static void handleLine(const char* s) {
  if (!s || !*s) return;

  if (std::strcmp(s, "GETCFG") == 0) {
    doGetCfg();
    return;
  }

  if (std::strncmp(s, "SETCFG_JSON ", 11) == 0) {
    uint32_t n = (uint32_t)std::strtoul(s + 11, nullptr, 10);
    if (n == 0 || n > sizeof(s_jsonBuf)) {
      uartSendLn("ERR LEN");
      return;
    }
    s_jsonNeed = n;
    s_jsonGot = 0;
    uartSendLn("OK");
    return;
  }

  if (std::strcmp(s, "REBOOT") == 0) {
    uartSendLn("OK");
    HAL_Delay(50);
    NVIC_SystemReset();
    return;
  }

  uartSendLn("ERR CMD");
}

void CfgUartBridge_Init(void) {
  s_lineLen = 0;
  s_jsonNeed = s_jsonGot = 0;
  DBG.info("CFG bridge: USART6 ready");
}

void CfgUartBridge_Tick(void) {
  uint8_t b = 0;
  if (HAL_UART_Receive(&huart6, &b, 1, 0) != HAL_OK) return;

  if (s_jsonNeed) {
    if (s_jsonGot < sizeof(s_jsonBuf)) s_jsonBuf[s_jsonGot] = (char)b;
    s_jsonGot++;

    if (s_jsonGot >= s_jsonNeed) {
      bool ok = Cfg().loadFromJson(s_jsonBuf, (size_t)s_jsonNeed);
      s_jsonNeed = s_jsonGot = 0;
      finishAndReboot(ok);
    }
    return;
  }

  if (b == '\r') return;

  if (b == '\n') {
    s_line[s_lineLen] = 0;
    handleLine(s_line);
    s_lineLen = 0;
    return;
  }

  if (s_lineLen + 1 < sizeof(s_line)) {
    s_line[s_lineLen++] = (char)b;
  } else {
    s_lineLen = 0;
    uartSendLn("ERR LINE");
  }
}

void CfgUartBridge_DelayMs(uint32_t ms) {
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms) {
    CfgUartBridge_Tick();
    IWDG->KR = 0xAAAAU;   /* feed IWDG -- poll_interval can be arbitrarily long */
    HAL_Delay(1);
  }
}
