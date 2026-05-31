#include "w5500_net.hpp"
#include "w5500_port.h"
#include "runtime_config.hpp"
#include "debug_uart.hpp"

#include <cstring>

extern "C" {
#include "w5500.h"
#include "dhcp.h"
}

static volatile bool s_dhcpAssigned = false;
static volatile bool s_dhcpConflict = false;

static void logMac(const uint8_t mac[6])
{
  DBG.info("W5500: MAC %02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool w5500SpiSelfTest()
{
  uint8_t ver = 0;
  for (int i = 0; i < 5; i++) { ver = getVERSIONR(); HAL_Delay(2); }

  DBG.info("W5500: VERSIONR=0x%02X", ver);
  if (ver != 0x04) {
    DBG.error("W5500: SPI self-test FAIL (VERSIONR != 0x04)");
    return false;
  }
  DBG.info("W5500: SPI self-test OK");
  return true;
}

void W5500Net::dhcpCbAssigned()  { s_dhcpAssigned = true; }
void W5500Net::dhcpCbConflict()  { s_dhcpConflict = true; }

void W5500Net::getNetInfo(wiz_NetInfo& out) const
{
  out = m_info;
}

bool W5500Net::init(SPI_HandleTypeDef* hspi, uint32_t dhcpTimeoutMs)
{
  const RuntimeConfig& c = Cfg();

  m_ready = false;
  m_mode = Mode::Static;
  s_dhcpAssigned = false;
  s_dhcpConflict = false;

  W5500_PortInit(hspi);
  W5500_HardReset();

  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
  reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
  reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);

  uint8_t txsize[8] = {2,2,2,2,2,2,2,2};
  uint8_t rxsize[8] = {2,2,2,2,2,2,2,2};

  if (wizchip_init(txsize, rxsize) != 0) {
    DBG.error("W5500: wizchip_init failed");
    return false;
  }

  if (!w5500SpiSelfTest()) return false;

  wiz_NetInfo ni0{};
  std::memcpy(ni0.mac, c.w5500_mac, 6);
  ni0.dhcp = NETINFO_DHCP;
  wizchip_setnetinfo(&ni0);
  m_info = ni0;
  logMac(ni0.mac);

  uint8_t link = 0;
  uint32_t t0 = HAL_GetTick();
  do {
    ctlwizchip(CW_GET_PHYLINK, (void*)&link);
    if (link) break;
    HAL_Delay(50);
  } while ((HAL_GetTick() - t0) < 3000);

  if (c.eth_mode == RuntimeConfig::EthMode::Dhcp) {
    if (tryDhcp(dhcpTimeoutMs)) {
      m_mode = Mode::Dhcp;
      m_ready = true;
      return true;
    }
  }

  applyStatic();
  return m_ready;
}

bool W5500Net::tryDhcp(uint32_t timeoutMs)
{
  const RuntimeConfig& c = Cfg();

  s_dhcpAssigned = false;
  s_dhcpConflict = false;

  DHCP_init(0, m_dhcpBuf);
  reg_dhcp_cbfunc(dhcpCbAssigned, dhcpCbAssigned, dhcpCbConflict);

  m_lastDhcp1s = HAL_GetTick();

  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < timeoutMs) {
    uint32_t now = HAL_GetTick();
    if ((now - m_lastDhcp1s) >= 1000) {
      DHCP_time_handler();
      m_lastDhcp1s += 1000;
    }

    uint8_t r = DHCP_run();
    if (r == DHCP_IP_LEASED || r == DHCP_IP_CHANGED || s_dhcpAssigned) {
      wiz_NetInfo ni{};
      std::memcpy(ni.mac, c.w5500_mac, 6);
      getIPfromDHCP(ni.ip);
      getGWfromDHCP(ni.gw);
      getSNfromDHCP(ni.sn);
      getDNSfromDHCP(ni.dns);
      ni.dhcp = NETINFO_DHCP;

      wizchip_setnetinfo(&ni);
      m_info = ni;

      DBG.info("W5500: DHCP OK %d.%d.%d.%d", ni.ip[0], ni.ip[1], ni.ip[2], ni.ip[3]);
      DBG.info("W5500: GW  %d.%d.%d.%d", ni.gw[0],  ni.gw[1],  ni.gw[2],  ni.gw[3]);
      DBG.info("W5500: SN  %d.%d.%d.%d", ni.sn[0],  ni.sn[1],  ni.sn[2],  ni.sn[3]);
      DBG.info("W5500: DNS %d.%d.%d.%d", ni.dns[0], ni.dns[1], ni.dns[2], ni.dns[3]);
      DHCP_stop();
      return true;
    }

    if (s_dhcpConflict) {
      DBG.error("W5500: DHCP conflict");
      DHCP_stop();
      return false;
    }

    HAL_Delay(1);
  }

  DBG.error("W5500: DHCP timeout -> static");
  DHCP_stop();
  return false;
}

void W5500Net::applyStatic()
{
  const RuntimeConfig& c = Cfg();

  wiz_NetInfo ni{};
  std::memcpy(ni.mac, c.w5500_mac, 6);
  std::memcpy(ni.ip,  c.eth_ip,  4);
  std::memcpy(ni.sn,  c.eth_sn,  4);
  std::memcpy(ni.gw,  c.eth_gw,  4);
  std::memcpy(ni.dns, c.eth_dns, 4);
  ni.dhcp = NETINFO_STATIC;

  wizchip_setnetinfo(&ni);
  m_info = ni;

  m_mode = Mode::Static;
  m_ready = true;

  DBG.info("W5500: STATIC %d.%d.%d.%d", ni.ip[0], ni.ip[1], ni.ip[2], ni.ip[3]);
}

void W5500Net::tick()
{
  if (m_mode != Mode::Dhcp) return;

  uint32_t now = HAL_GetTick();
  if ((now - m_lastDhcp1s) >= 1000) {
    DHCP_time_handler();
    m_lastDhcp1s += 1000;
  }

  uint8_t r = DHCP_run();
  if (r == DHCP_IP_CHANGED) {
    const RuntimeConfig& c = Cfg();

    wiz_NetInfo ni{};
    std::memcpy(ni.mac, c.w5500_mac, 6);
    getIPfromDHCP(ni.ip);
    getGWfromDHCP(ni.gw);
    getSNfromDHCP(ni.sn);
    getDNSfromDHCP(ni.dns);
    ni.dhcp = NETINFO_DHCP;

    wizchip_setnetinfo(&ni);
    m_info = ni;

    DBG.info("W5500: DHCP changed %d.%d.%d.%d", ni.ip[0], ni.ip[1], ni.ip[2], ni.ip[3]);
  }
}
