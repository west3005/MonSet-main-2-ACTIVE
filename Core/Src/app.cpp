/**
 * ================================================================
 * @file app.cpp
 * @brief Main application — full integration of all modules.
 *
 * ИСПРАВЛЕНИЕ "Loading..." / медленного веб-интерфейса:
 *
 *  Причина: m_webServer.tick() вызывался раз в ~6 секунд
 *  (Modbus timeout 1с + poll_delay 5с). Браузер делал
 *  fetch('/api/sensors') через 50мс → не получал ответа → timeout.
 *
 *  Решения:
 *  1. handleIndex() теперь самодостаточная страница — вторых
 *     запросов нет (см. web_server.cpp).
 *  2. Когда m_webActive=true, run() входит в TIGHT WEB LOOP:
 *     tick() вызывается каждые 5мс — мгновенный отклик браузера.
 *     Modbus и poll_delay пропускаются на время веб-сессии.
 *  3. Данные накапливаются в m_lastVal/m_lastTs для отображения
 *     без необходимости опроса Modbus во время веб-работы.
 * ================================================================
 */
#include "app.hpp"
#include "w5500_net.hpp"
#include "https_w5500.hpp"
#include "air780e_tls.hpp"
#include "a7670c_tls.hpp"
#include "runtime_config.hpp"
#include "cfg_uart_bridge.hpp"
#include "board_pins.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

extern "C" {
    extern I2C_HandleTypeDef  hi2c1;
    extern UART_HandleTypeDef huart2;
    extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
    extern UART_HandleTypeDef huart6;
    extern SPI_HandleTypeDef  hspi1;
    extern RTC_HandleTypeDef  hrtc;
    extern bool g_sd_disabled;
#include "socket.h"
#include "dns.h"
#include "w5500.h"
#include "wizchip_conf.h"
// ================================================================
// Накопители усреднения (Этап 2 — агрегация по send_interval)
// Аналог value_buffer: HashMap<metric_id, Vec<f64>> из ocean-station.
// Вместо полного массива отсчётов — только sum + count на канал.
// RAM: 16 * (4 + 1) = 80 байт. Корректное среднее без heap.
// ================================================================
#ifndef MAX_AVG_CHANNELS
#define MAX_AVG_CHANNELS  MAX_MODBUS_ENTRIES  ///< = 16, по числу каналов
#endif

static float   s_avgSum[MAX_AVG_CHANNELS]{};   ///< накопленная сумма по каналу
static uint8_t s_avgCount[MAX_AVG_CHANNELS]{};  ///< число накопленных отсчётов

// Этап 6: раздельные таймеры retry backup — GSM/ETH и Iridium
// m_lastBackupSendTick (в app.hpp) используется для GSM/ETH
// s_lastIridiumRetryTick — static, не трогает app.hpp
static uint32_t s_lastIridiumRetryTick = 0; ///< HAL_GetTick последней попытки Iridium retry

/// Добавить значение val в накопитель канала ch
static inline void avgPush(uint8_t ch, float val) {
    if (ch >= MAX_AVG_CHANNELS) return;
    s_avgSum[ch] += val;
    s_avgCount[ch]++;
}

/// Вычислить среднее по накопителю канала ch (возвращает 0.0 если пусто)
static inline float avgGet(uint8_t ch) {
    if (ch >= MAX_AVG_CHANNELS || s_avgCount[ch] == 0) return 0.0f;
    return s_avgSum[ch] / (float)s_avgCount[ch];
}

/// Сбросить накопитель канала ch
static inline void avgClear(uint8_t ch) {
    if (ch >= MAX_AVG_CHANNELS) return;
    s_avgSum[ch] = 0.0f; s_avgCount[ch] = 0;
}

/// Сбросить все накопители
static inline void avgClearAll() {
    for (uint8_t i = 0; i < MAX_AVG_CHANNELS; i++) {
        s_avgSum[i] = 0.0f; s_avgCount[i] = 0;
    }
}


}

#ifdef W5500
# undef W5500
#endif

extern volatile bool g_web_exclusive;

static W5500Net eth;
static UART_HandleTypeDef huart5;

/**
 * @brief Инициализация UART4 (PC10 TX / PC11 RX) — датчик порт 1.
 *        Baudrate/parity/stopbits берутся из rtu_ports[1].
 */
static void MX_UART4_Init(const ModbusRtuPortConfig& p) {
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_10; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_11; gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &gpio);
    const uint32_t parLut[3] = { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };
    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = p.baudrate;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = (p.stop_bits == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    huart4.Init.Parity       = (p.parity < 3) ? parLut[p.parity] : UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart4);
}

/**
 * @brief Инициализация UART5 (PC12 TX / PD2 RX) — датчик порт 2 / Iridium.
 * @param p  конфиг порта 2; nullptr — Iridium defaults (19200 8N1).
 */
static void MX_UART5_Init(const ModbusRtuPortConfig* p = nullptr) {
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_12; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_2; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &gpio);
    const uint32_t parLut5[3] = { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };
    huart5.Instance          = UART5;
    huart5.Init.BaudRate     = p ? p->baudrate : 19200;
    huart5.Init.WordLength   = UART_WORDLENGTH_8B;
    huart5.Init.StopBits     = (p && p->stop_bits == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    huart5.Init.Parity       = (p && p->parity < 3) ? parLut5[p->parity] : UART_PARITY_NONE;
    huart5.Init.Mode         = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart5);
}

static void InitIridiumGpio(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin  = GPIO_PIN_3; gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_4; gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN; HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Pin = GPIO_PIN_5; HAL_GPIO_Init(GPIOD, &gpio);
}

static void InitEspGpio(void) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_0; gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &gpio);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_1; HAL_GPIO_Init(GPIOE, &gpio);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_2; HAL_GPIO_Init(GPIOE, &gpio);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
}

static bool startsWith(const char* s, const char* prefix) {
    if (!s || !prefix) return false;
    return std::strncmp(s, prefix, std::strlen(prefix)) == 0;
}
static bool isLeapYear(int y) {
    return ((y%4)==0&&(y%100)!=0)||((y%400)==0);
}
static uint64_t toUnixMs(const DateTime& dt) {
    int y = 2000 + (int)dt.year, m = (int)dt.month;
    static const uint16_t cumDays[12]={0,31,59,90,120,151,181,212,243,273,304,334};
    uint32_t days = 0;
    for (int yy=1970; yy<y; yy++) days += isLeapYear(yy)?366:365;
    days += cumDays[m-1];
    if (m>2 && isLeapYear(y)) days++;
    days += (uint32_t)(dt.date-1);
    uint64_t sec = (uint64_t)days*86400ULL
                 + (uint64_t)dt.hours*3600ULL
                 + (uint64_t)dt.minutes*60ULL
                 + (uint64_t)dt.seconds;
    return sec*1000ULL;
}
static void u64ToDec(char* out, size_t outSz, uint64_t v) {
    if (!out||outSz==0) return;
    char tmp[32]; size_t n=0;
    do { tmp[n++]=char('0'+(v%10)); v/=10; } while(v&&n<sizeof(tmp));
    size_t pos=0;
    while(n&&(pos+1)<outSz) out[pos++]=tmp[--n];
    out[pos]='\0';
}

// NTP
static constexpr uint16_t NTP_PORT       = 123;
static constexpr uint32_t NTP_TIMEOUT_MS = 3000;
static constexpr uint32_t BKP_MAGIC      = 0x4E545031;
static constexpr uint32_t BKP_MAGIC_REG  = RTC_BKP_DR0;
static constexpr uint32_t BKP_SYNC_REG   = RTC_BKP_DR1;
static uint32_t bkpRead(uint32_t reg)            { return HAL_RTCEx_BKUPRead(&hrtc,reg); }
static void     bkpWrite(uint32_t reg,uint32_t v){ HAL_RTCEx_BKUPWrite(&hrtc,reg,v); }
static uint32_t loadLastSyncUnix()  { return (bkpRead(BKP_MAGIC_REG)==BKP_MAGIC)?bkpRead(BKP_SYNC_REG):0; }
static void     storeLastSyncUnix(uint32_t u)    { bkpWrite(BKP_MAGIC_REG,BKP_MAGIC); bkpWrite(BKP_SYNC_REG,u); }
static bool rtcIsInvalid(const DateTime& dt) {
    if (dt.year<24||dt.year>60) return true;
    if (dt.month<1||dt.month>12) return true;
    if (dt.date<1||dt.date>31) return true;
    if (dt.hours>23||dt.minutes>59||dt.seconds>59) return true;
    return false;
}
static void unixToDateTime(uint32_t unixSec, DateTime& out) {
    uint32_t sec = unixSec;
    out.seconds=(uint8_t)(sec%60); sec/=60;
    out.minutes=(uint8_t)(sec%60); sec/=60;
    out.hours=(uint8_t)(sec%24); sec/=24;
    uint32_t days=sec; int y=1970;
    while(true){ uint32_t diy=isLeapYear(y)?366u:365u; if(days<diy) break; days-=diy; y++; }
    const uint8_t mdays[12]={31,28,31,30,31,30,31,31,30,31,30,31};
    uint8_t mo=0;
    while(mo<12){ uint8_t md=mdays[mo]; if(mo==1&&isLeapYear(y)) md=29; if(days<md) break; days-=md; mo++; }
    out.year=(uint8_t)(y-2000); out.month=(uint8_t)(mo+1); out.date=(uint8_t)(days+1);
}
static bool sntpGetUnixTime(const char* host, uint32_t& unixSec) {
    uint8_t ip[4]{};
    bool isNumeric=true;
    for(const char* p=host;*p;p++)
        if(!std::isdigit((unsigned char)*p)&&*p!='.'){isNumeric=false;break;}
    if(isNumeric){
        uint32_t a,b,c,d;
        if(std::sscanf(host,"%lu.%lu.%lu.%lu",&a,&b,&c,&d)!=4) return false;
        ip[0]=(uint8_t)a;ip[1]=(uint8_t)b;ip[2]=(uint8_t)c;ip[3]=(uint8_t)d;
    } else {
        wiz_NetInfo ni{}; wizchip_getnetinfo(&ni);
        static uint8_t dnsBuf[Config::DNS_BUFFER_SIZE];
        DNS_init(1,dnsBuf); bool resolved=false;
        uint32_t t0=HAL_GetTick();
        while((HAL_GetTick()-t0)<Config::DNS_TIMEOUT_MS){
            int8_t r=DNS_run(ni.dns,(uint8_t*)host,ip);
            if(r==1){resolved=true;break;} if(r<0) return false;
            HAL_Delay(50); IWDG->KR=0xAAAA;
        }
        if(!resolved) return false;
    }
    for(int attempt=1;attempt<=3;attempt++){
        const uint8_t sn=2; const uint16_t lport=(uint16_t)(40000+attempt);
        uint8_t pkt[48]{}; pkt[0]=0x1B;
        if(socket(sn,Sn_MR_UDP,lport,0)!=sn){close(sn);continue;}
        if(sendto(sn,pkt,sizeof(pkt),ip,NTP_PORT)!=(int32_t)sizeof(pkt)){close(sn);continue;}
        uint32_t t0=HAL_GetTick();
        while((HAL_GetTick()-t0)<NTP_TIMEOUT_MS){
            uint8_t rx[48]; uint8_t rip[4]; uint16_t rport=0;
            int32_t r=recvfrom(sn,rx,sizeof(rx),rip,&rport);
            if(r>=48){
                close(sn);
                uint32_t ntpSec=((uint32_t)rx[40]<<24)|((uint32_t)rx[41]<<16)|
                                 ((uint32_t)rx[42]<<8)|rx[43];
                constexpr uint32_t NTP2UNIX=2208988800UL;
                if(ntpSec<NTP2UNIX) break;
                unixSec=ntpSec-NTP2UNIX; return true;
            }
            HAL_Delay(10); IWDG->KR=0xAAAA;
        }
        close(sn);
    }
    return false;
}

static int httpPostPlainW5500(const char* url,const char* authB64,
                               const char* json,uint16_t len,uint32_t timeoutMs)
{
    struct UrlParts{char host[64]{};char path[128]{};uint16_t port=80;} u{};
    {
        const char* prefix="http://";
        if(std::strncmp(url,prefix,7)!=0) return -10;
        const char* p=url+7; const char* hb=p;
        while(*p&&*p!='/'&&*p!=':') p++;
        size_t hl=(size_t)(p-hb);
        if(hl==0||hl>=sizeof(u.host)) return -10;
        std::memcpy(u.host,hb,hl);
        if(*p==':'){p++;u.port=(uint16_t)std::strtoul(p,nullptr,10);while(*p&&*p!='/') p++;}
        if(*p==0) std::strcpy(u.path,"/"); else std::strcpy(u.path,p);
    }
    uint8_t dstIp[4]{};
    {
        bool isNum=true;
        for(const char* p=u.host;*p;p++)
            if(!std::isdigit((unsigned char)*p)&&*p!='.'){isNum=false;break;}
        if(isNum){
            uint32_t a,b,c,d;
            std::sscanf(u.host,"%lu.%lu.%lu.%lu",&a,&b,&c,&d);
            dstIp[0]=(uint8_t)a;dstIp[1]=(uint8_t)b;dstIp[2]=(uint8_t)c;dstIp[3]=(uint8_t)d;
        } else {
            wiz_NetInfo ni{}; wizchip_getnetinfo(&ni);
            static uint8_t dnsBuf[Config::DNS_BUFFER_SIZE];
            DNS_init(1,dnsBuf); bool ok=false;
            uint32_t t0=HAL_GetTick();
            while((HAL_GetTick()-t0)<Config::DNS_TIMEOUT_MS){
                int8_t r=DNS_run(ni.dns,(uint8_t*)u.host,dstIp);
                if(r==1){ok=true;break;} if(r<0) break;
                HAL_Delay(50); IWDG->KR=0xAAAA;
            }
            if(!ok) return -11;
        }
    }
    const uint8_t sn=0;
    if(socket(sn,Sn_MR_TCP,Config::HTTP_LOCAL_PORT,0)!=sn){close(sn);return -20;}
    if(connect_3(sn,dstIp,u.port)!=SOCK_OK){close(sn);return -21;}
    char hdr[600]; int hdrLen;
    if(authB64&&authB64[0]){
        hdrLen=std::snprintf(hdr,sizeof(hdr),
            "POST %s HTTP/1.1\r\nHost: %s\r\nAuthorization: Basic %s\r\n"
            "Content-Type: application/json\r\nContent-Length: %u\r\nConnection: close\r\n\r\n",
            u.path,u.host,authB64,(unsigned)len);
    } else {
        hdrLen=std::snprintf(hdr,sizeof(hdr),
            "POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: application/json\r\n"
            "Content-Length: %u\r\nConnection: close\r\n\r\n",
            u.path,u.host,(unsigned)len);
    }
    if(hdrLen<=0||(size_t)hdrLen>=sizeof(hdr)){close(sn);return -22;}
    auto sendAll=[&](const uint8_t* p,uint32_t n)->bool{
        uint32_t off=0;
        while(off<n){int32_t r=send(sn,(uint8_t*)p+off,(uint16_t)(n-off));if(r<=0) return false;off+=(uint32_t)r;}
        return true;
    };
    if(!sendAll((const uint8_t*)hdr,(uint32_t)hdrLen)){close(sn);return -23;}
    if(!sendAll((const uint8_t*)json,(uint32_t)len)){close(sn);return -24;}
    static char rx[768]; int rxUsed=0;
    uint32_t t0=HAL_GetTick();
    while((HAL_GetTick()-t0)<timeoutMs){
        int32_t rlen=recv(sn,(uint8_t*)rx+rxUsed,(uint16_t)(sizeof(rx)-1-rxUsed));
        if(rlen>0){
            rxUsed+=(int)rlen; rx[rxUsed]=0;
            const char* p=std::strstr(rx,"HTTP/1.");
            if(p){int code=0;if(std::sscanf(p,"HTTP/%*s %d",&code)==1){disconnect(sn);close(sn);return code;}}
        } else {HAL_Delay(2); IWDG->KR=0xAAAA;}
    }
    disconnect(sn); close(sn); return -30;
}

// ============================================================================
// Constructor
// ============================================================================
App::App()
    : m_rtc(&hi2c1)
    , m_modbusPort0(&huart3, PIN_RS485_DE_PORT, PIN_RS485_DE_PIN)
    , m_gsm(&huart2, PIN_SIM_PWR_PORT, PIN_SIM_PWR_PIN)
    , m_sdBackup()
    , m_sensor(m_modbusPort0, m_rtc)
    , m_buffer()
    , m_power(&hrtc, m_sdBackup)
    , m_channelMgr()
    , m_mqtt()
    , m_webhook()
    , m_iridium(&huart5)
    , m_webServer()
    , m_esp(&huart6)
    , m_captivePortal()
    , m_battery()
{
    m_modbusPorts[0] = &m_modbusPort0;
    m_modbusPorts[1] = &m_modbusPort1;
    m_modbusPorts[2] = &m_modbusPort2;
    // Имена файлов бэкапа (дефолты); обновятся в setup() после loadFromSd()
    m_sdBackup.setFilename(Config::PORT0_BACKUP_FILE);
    m_sdBackup1.setFilename(Config::PORT1_BACKUP_FILE);
    m_sdBackup2.setFilename(Config::PORT2_BACKUP_FILE);
}

SystemMode App::readMode() {
    return (HAL_GPIO_ReadPin(PIN_MODE_SW_PORT,PIN_MODE_SW_PIN)==GPIO_PIN_SET)
           ? SystemMode::Debug : SystemMode::Sleep;
}
LinkChannel App::readChannel() {
    return (HAL_GPIO_ReadPin(PIN_NET_SW_PORT,PIN_NET_SW_PIN)==GPIO_PIN_RESET)
           ? LinkChannel::Eth : LinkChannel::Gsm;
}
void App::ledOn()  { HAL_GPIO_WritePin(PIN_LED_PORT,PIN_LED_PIN,GPIO_PIN_SET); }
void App::ledOff() { HAL_GPIO_WritePin(PIN_LED_PORT,PIN_LED_PIN,GPIO_PIN_RESET); }
void App::ledBlink(uint8_t count,uint32_t ms) {
    for(uint8_t i=0;i<count;i++){ledOn();HAL_Delay(ms);ledOff();HAL_Delay(ms);}
}

bool App::ensureEthReady() {
    if(!eth.ready()){ if(!eth.init(&hspi1,Config::W5500_DHCP_TIMEOUT_MS)) return false; }
    uint8_t link=0;
    for(int i=0;i<50;i++){
        if(ctlwizchip(CW_GET_PHYLINK,(void*)&link)!=0) return false;
        if(link!=PHY_LINK_OFF) return true;
        HAL_Delay(100); IWDG->KR=0xAAAA;
    }
    return false;
}

int App::sendViaEth(const char* json,uint16_t len,void* ctx)     { return static_cast<App*>(ctx)->postViaEth(json,len); }
int App::sendViaGsm(const char* json,uint16_t len,void* ctx)     { return static_cast<App*>(ctx)->postViaGsm(json,len); }
int App::sendViaWifi(const char* json,uint16_t len,void* ctx)    { (void)json;(void)len;(void)ctx; DBG.warn("WiFi not impl"); return -1; }
int App::sendViaIridium(const char* json,uint16_t len,void* ctx) {
    App* self=static_cast<App*>(ctx);
    float val=self->m_sensor.lastValue();
    uint32_t unixSec=(uint32_t)(toUnixMs(self->m_sensor.getReading(0).timestamp)/1000ULL);
    uint8_t sbdData[64];
    uint16_t sbdLen=Iridium::packSensorData(sbdData,sizeof(sbdData),&val,1,unixSec);
    if(sbdLen==0) return -1;
    self->m_iridium.powerOn();
    if(!self->m_iridium.init()){self->m_iridium.powerOff();return -1;}
    IridiumStatus st=self->m_iridium.sendSBD(sbdData,sbdLen);
    self->m_iridium.powerOff();
    return (st==IridiumStatus::Ok)?200:-1;
}

int App::postViaEth(const char* json,uint16_t len) {
    if(!ensureEthReady()) return -1;
    const RuntimeConfig& c=Cfg();
    if((c.protocol==ProtocolMode::MQTT_GENERIC||c.protocol==ProtocolMode::MQTT_THINGSBOARD)&&c.mqtt_host[0])
        return sendViaMqtt(json,len)?200:-1;
    char url[192]{};
    c.buildServerUrl(url, sizeof(url));
    // Ocean Monitor: авторизация через Authorization: Basic header
    const char* auth = nullptr;
    if (c.protocol == ProtocolMode::OCEAN_MONITOR) {
        auth = c.server_auth_b64[0] ? c.server_auth_b64 : nullptr;
    } else {
        auth = c.server_auth_b64[0] ? c.server_auth_b64 : nullptr;
    }
    if(startsWith(url,"https://"))
        return HttpsW5500::postJson(url, auth, json, len, Config::HTTPS_POST_TIMEOUT_MS);
    if(startsWith(url,"http://"))
        return httpPostPlainW5500(url, auth, json, len, Config::HTTP_POST_TIMEOUT_MS);
    return -1;
}
int App::postViaGsm(const char* json,uint16_t len) {
    m_gsm.powerOn();
    if(m_gsm.init()!=GsmStatus::Ok){m_gsm.powerOff();return -1;}
    const RuntimeConfig& c=Cfg(); int code=-1;
    if((c.protocol==ProtocolMode::MQTT_GENERIC||c.protocol==ProtocolMode::MQTT_THINGSBOARD)&&c.mqtt_host[0]){
        if(!m_mqtt.isConnected()) m_mqtt.mqttConnect(MqttBackend::GSM);
        if(m_mqtt.publish(nullptr,json,len)) code=200;
    } else {
        char url[192]{};
        c.buildServerUrl(url, sizeof(url));
        // Если gsm_server_ip задан — заменяем hostname на IP
        // (DNS недоступен у некоторых операторов через AT+CDNSGIP)
        char gsmUrl[192]{};
        std::strncpy(gsmUrl, url, sizeof(gsmUrl)-1);
        if (c.gsm_server_ip[0]) {
            const char* schemeEnd = std::strstr(url, "://");
            if (schemeEnd) {
                const char* pathStart = std::strchr(schemeEnd + 3, '/');
                if (pathStart)
                    std::snprintf(gsmUrl, sizeof(gsmUrl),
                                 "https://%s%s", c.gsm_server_ip, pathStart);
            }
        }
        if(startsWith(gsmUrl,"https://")) {
            A7670CTls tls(m_gsm);
            if(c.tls_ca_cert[0])
                tls.setCaCert(c.tls_ca_cert);
            code=(int)tls.httpsPost(gsmUrl,json,len);
        } else {
            code=(int)m_gsm.httpPost(gsmUrl,json,len);
        }
    }
    m_gsm.disconnect(); m_gsm.powerOff(); return code;
}
bool App::sendViaMqtt(const char* json,uint16_t len) {
    if(!m_mqtt.isConnected()){
        MqttBackend backend=(readChannel()==LinkChannel::Eth)?MqttBackend::WIZNET:MqttBackend::GSM;
        if(!m_mqtt.mqttConnect(backend)) return false;
    }
    return m_mqtt.publish(nullptr,json,(uint16_t)len);
}

// ----------------------------------------------------------------------------
// buildOceanPayload — формирует JSON для ocean-monitor.ru /api/rest/measures
//
// Формат ocean-station (плоский массив, без обёртки url/username/password):
//   [{"metricId":"...","value":"%.3f","measureTime":"YYYY-MM-DDTHH:MM:SS.mmmZ"},...]
//
// - value передаётся как СТРОКА с 3 знаками после точки (требование протокола)
// - measureTime: ISO 8601 UTC с реальными миллисекундами (вычисляются из Unix timestamp)
// - все валидные SensorReading включаются в массив (multi-metric)
// - если ни одного валидного reading нет — используется переданный val как fallback
// - авторизация передаётся через Authorization: Basic HTTP-заголовок (не в теле)
// ----------------------------------------------------------------------------
int App::buildOceanPayload(char* buf, size_t bsz, float val, const DateTime& dt) {
    const RuntimeConfig& c = Cfg();

    // Вычисляем реальные миллисекунды из RTC timestamp
    uint64_t unixMs = toUnixMs(dt);
    uint16_t ms = (uint16_t)(unixMs % 1000ULL);

    int n = 0;
    n += std::snprintf(buf + n, bsz - n, "[");
    if (n < 0 || (size_t)n >= bsz) return -1;

    bool added = false;
    uint8_t readingCount = m_sensor.getReadingCount();

    for (uint8_t i = 0; i < readingCount; i++) {
        const SensorReading& r = m_sensor.getReading(i);
        if (!r.valid) continue;

        // Этап 2: используем среднее из буфера если есть накопленные отсчёты,
        // иначе — текущее значение (первый poll или буфер пуст)
        float sendVal = (s_avgCount[i] > 0) ? avgGet(i) : r.value;

        // Выбираем metric_id: из reading.name если не пустой, иначе глобальный
        const char* mid = (r.name[0] != '\0') ? r.name : c.proto.ocean_metric_id;
        if (mid[0] == '\0') mid = c.metric_id;

        if (added) {
            n += std::snprintf(buf + n, bsz - n, ",");
            if (n < 0 || (size_t)n >= bsz) return -1;
        }
        n += std::snprintf(buf + n, bsz - n,
            "{\"metricId\":\"%s\","
            "\"value\":\"%.3f\","
            "\"measureTime\":\"20%02u-%02u-%02uT%02u:%02u:%02u.%03uZ\"}",
            mid,
            (double)sendVal,
            (unsigned)dt.year, (unsigned)dt.month,  (unsigned)dt.date,
            (unsigned)dt.hours,(unsigned)dt.minutes,(unsigned)dt.seconds,
            (unsigned)ms);
        if (n < 0 || (size_t)n >= bsz) return -1;
        added = true;
    }

    // Fallback: если ни одного валидного reading нет — используем переданный val
    if (!added) {
        float sendVal = (s_avgCount[0] > 0) ? avgGet(0) : val;
        const char* mid = c.proto.ocean_metric_id[0] ? c.proto.ocean_metric_id : c.metric_id;
        n += std::snprintf(buf + n, bsz - n,
            "{\"metricId\":\"%s\","
            "\"value\":\"%.3f\","
            "\"measureTime\":\"20%02u-%02u-%02uT%02u:%02u:%02u.%03uZ\"}",
            mid,
            (double)sendVal,
            (unsigned)dt.year, (unsigned)dt.month,  (unsigned)dt.date,
            (unsigned)dt.hours,(unsigned)dt.minutes,(unsigned)dt.seconds,
            (unsigned)ms);
        if (n < 0 || (size_t)n >= bsz) return -1;
    }

    n += std::snprintf(buf + n, bsz - n, "]");
    if (n < 0 || (size_t)n >= bsz) return -1;

    return n;
}

int App::buildPayload(char* buf,size_t bsz,const char* tsStr,float val,const DateTime& dt,bool asArray) {
    // Ocean Monitor использует собственный формат — делегируем
    if (Cfg().protocol == ProtocolMode::OCEAN_MONITOR)
        return buildOceanPayload(buf, bsz, val, dt);
    return std::snprintf(buf,bsz,
        "%s{\"ts\":%s,\"values\":{\"metricId\":\"%s\",\"value\":%.3f,"
        "\"measureTime\":\"20%02u-%02u-%02uT%02u:%02u:%02u.000Z\"}}%s",
        asArray?"[":"", tsStr,Cfg().metric_id,val,
        (unsigned)dt.year,(unsigned)dt.month,(unsigned)dt.date,
        (unsigned)dt.hours,(unsigned)dt.minutes,(unsigned)dt.seconds,
        asArray?"]":"");
}
int App::buildMultiSensorPayload(char* buf,size_t bsz,const char* tsStr,const DateTime& dt,bool asArray) {
    // Ocean Monitor: используем buildOceanPayload (multi-metric, плоский формат)
    if (Cfg().protocol == ProtocolMode::OCEAN_MONITOR) {
        float val = 0.0f;
        for (uint8_t i = 0; i < m_sensor.getReadingCount(); i++) {
            const SensorReading& r = m_sensor.getReading(i);
            if (r.valid) { val = r.value; break; }
        }
        return buildOceanPayload(buf, bsz, val, dt);
    }
    int n=0;
    if(asArray) n+=std::snprintf(buf+n,bsz-n,"[");
    n+=std::snprintf(buf+n,bsz-n,"{\"ts\":%s,\"values\":{",tsStr);
    for(uint8_t i=0;i<m_sensor.getReadingCount();i++){
        const SensorReading& r=m_sensor.getReading(i);
        if(!r.valid) continue;
        if(i>0) n+=std::snprintf(buf+n,bsz-n,",");
        n+=std::snprintf(buf+n,bsz-n,"\"%s\":%.3f",
            r.name[0]?r.name:Cfg().metric_id,(double)r.value);
    }
    n+=std::snprintf(buf+n,bsz-n,
        ",\"battery_pct\":%u,\"battery_v\":%.2f"
        ",\"measureTime\":\"20%02u-%02u-%02uT%02u:%02u:%02u.000Z\"}}",
        (unsigned)m_battery.getPercent(),(double)m_battery.getVoltage(),
        (unsigned)dt.year,(unsigned)dt.month,(unsigned)dt.date,
        (unsigned)dt.hours,(unsigned)dt.minutes,(unsigned)dt.seconds);
    if(asArray) n+=std::snprintf(buf+n,bsz-n,"]");
    return n;
}

// ============================================================================
// Init
// ============================================================================
void App::init() {
    DBG.info("=== APP INIT %s %s ===", __DATE__, __TIME__);
    DBG.info("[1/9] RTC init"); m_rtc.init();
    DBG.info("[2/9] Modbus init");
    // Порт 0 (USART3) — всегда активен
    m_modbusPort0.init();
    // Порт 1 (UART4) — если enabled в конфиге
    if (Cfg().rtu_ports[1].enabled) {
        MX_UART4_Init(Cfg().rtu_ports[1]);
        m_modbusPort1.configure(&huart4);  // auto-direction конвертер 2126, DE не нужен
        m_modbusPort1.init();
        DBG.info("[2/9] UART4 port1 OK baud=%lu", (unsigned long)Cfg().rtu_ports[1].baudrate);
    } else {
        DBG.info("[2/9] UART4 port1 disabled");
    }
    // Порт 2 (UART5) — если enabled И Iridium выключен
    if (Cfg().rtu_ports[2].enabled && !Cfg().iridium_enabled) {
        MX_UART5_Init(&Cfg().rtu_ports[2]);
        m_modbusPort2.configure(&huart5);  // auto-direction конвертер 2126, DE не нужен
        m_modbusPort2.init();
        DBG.info("[2/9] UART5 port2 OK baud=%lu", (unsigned long)Cfg().rtu_ports[2].baudrate);
    } else if (Cfg().iridium_enabled) {
        DBG.info("[2/9] UART5 reserved for Iridium — port2 skipped");
    } else {
        DBG.info("[2/9] UART5 port2 disabled");
    }
    // Имена файлов бэкапа: берём из config.hpp (константы).
    // backup_filename в rtu_ports дублирует их для UI (/api/config),
    // но для SdBackup используем compile-time константы — стабильнее.
    m_sdBackup.setFilename(Config::PORT0_BACKUP_FILE);
    m_sdBackup1.setFilename(Config::PORT1_BACKUP_FILE);
    m_sdBackup2.setFilename(Config::PORT2_BACKUP_FILE);
    DBG.info("[3/9] SD init");
    // SD инициализируется всегда — g_sd_disabled выставляется только
    // если MX_SDIO_SD_Init() реально упал (см. main.cpp)
    m_sdOk = !g_sd_disabled && m_sdBackup.init();
    if (g_sd_disabled) {
        DBG.warn("[3/9] SD skipped (SDIO hardware failure on boot)");
    } else {
        DBG.info("[3/9] SD init: %s", m_sdOk ? "OK" : "FAIL");
    }
    DBG.info("[4/9] Load runtime config");
    bool cfgLoaded=false;
    if(!g_sd_disabled&&m_sdOk) cfgLoaded=Cfg().loadFromSd(RUNTIME_CONFIG_FILENAME);
    if(!cfgLoaded) Cfg().setDefaultsFromConfig();
    Cfg().log();
    DBG.info("[5/9] Modem power OFF (cold start)"); m_gsm.powerOff();
    DBG.info("[6/9] Iridium GPIO + UART5 init");
    if(Cfg().iridium_enabled){ InitIridiumGpio(); MX_UART5_Init(nullptr); DBG.info("Iridium UART5 OK (19200 8N1)"); }
    DBG.info("[7/9] ESP8266 GPIO init"); InitEspGpio();
    DBG.info("[8/9] Battery monitor init");
    m_battery.init(); m_battery.update();
    DBG.info("Battery: %.1fV %u%%",(double)m_battery.getVoltage(),(unsigned)m_battery.getPercent());
    DBG.info("[9/9] Channel manager + modules init");
    m_mqtt.init(&m_gsm); m_webhook.init(&m_gsm);
    m_captivePortal.init(&m_esp); initChannelManager();
    m_mode=readMode();
    DBG.info("Mode: %s",m_mode==SystemMode::Debug?"DEBUG":"SLEEP");
    DBG.info("Poll: %lu s | Send every: %lu polls",
             (unsigned long)Cfg().poll_interval_sec,(unsigned long)Cfg().send_interval_polls);
    // Web-сервер стартует НЕЗАВИСИМО от mode и eth_enabled.
    // Если W5500 ещё не поднят (DHCP не ответил) — init() выставит m_running=false
    // и флаг m_webStartPending=true. Повторная попытка каждую итерацию run().
    m_webServer.init(&m_sensor, &m_sdBackup, &m_battery, this);
    m_webServer.setRtc(&m_rtc);
    m_webServer.setSdOk(m_sdOk);
    if (!m_webServer.isRunning()) {
        DBG.info("[9/9] WebServer: deferred start pending (eth not ready)");
        m_webStartPending = true;
    }
    if(m_mode==SystemMode::Debug) ledOn(); else ledBlink(3,200);
    m_lastBackupSendTick=HAL_GetTick();
}

void App::initChannelManager() {
    m_channelMgr.init(&m_sdBackup);
    if(Cfg().eth_enabled)     m_channelMgr.registerChannel(Channel::ETHERNET,sendViaEth,    this);
    if(Cfg().gsm_enabled)     m_channelMgr.registerChannel(Channel::GSM,     sendViaGsm,    this);
    if(Cfg().wifi_enabled)    m_channelMgr.registerChannel(Channel::WIFI,    sendViaWifi,   this);
    if(Cfg().iridium_enabled) m_channelMgr.registerChannel(Channel::IRIDIUM, sendViaIridium,this);
    if(Cfg().eth_enabled&&eth.ready()) m_channelMgr.markAlive(Channel::ETHERNET);
}

void App::reinitChannelManager() {
    m_channelMgr.init(&m_sdBackup);  // сброс + повторная регистрация
    if(Cfg().eth_enabled)     m_channelMgr.registerChannel(Channel::ETHERNET,sendViaEth,    this);
    if(Cfg().gsm_enabled)     m_channelMgr.registerChannel(Channel::GSM,     sendViaGsm,    this);
    if(Cfg().wifi_enabled)    m_channelMgr.registerChannel(Channel::WIFI,    sendViaWifi,   this);
    if(Cfg().iridium_enabled) m_channelMgr.registerChannel(Channel::IRIDIUM, sendViaIridium,this);
    if(Cfg().eth_enabled&&eth.ready()) m_channelMgr.markAlive(Channel::ETHERNET);
    DBG.info("[CFG] ChMgr reinit: eth=%d gsm=%d wifi=%d irid=%d",
        (int)Cfg().eth_enabled,(int)Cfg().gsm_enabled,
        (int)Cfg().wifi_enabled,(int)Cfg().iridium_enabled);
}

bool App::syncRtcWithNtpIfNeeded(const char* tag,bool verbose) {
    const RuntimeConfig& c=Cfg();
    if(!c.ntp_enabled) return false;
    DateTime cur{};
    if(!m_rtc.getTime(cur)) return false;
    const bool invalid=rtcIsInvalid(cur);
    const uint32_t lastSy=loadLastSyncUnix();
    const uint32_t nowSec=(uint32_t)(toUnixMs(cur)/1000ULL);
    bool needSync=false;
    if(invalid) needSync=true;
    else if(lastSy==0) needSync=true;
    else if((nowSec-lastSy)>=c.ntp_resync_sec) needSync=true;
    else {if(verbose) DBG.info("[%s] NTP: skip",tag); return false;}
    (void)needSync; ///< suppress -Wunused-but-set-variable
    if(!c.eth_enabled||!ensureEthReady()) return false;
    uint32_t unixSec=0;
    if(!sntpGetUnixTime(c.ntp_host,unixSec)) return false;
    DateTime ntpDt{};
    unixToDateTime(unixSec,ntpDt);
    if(!m_rtc.setTimeAutoDOW(ntpDt)) return false;
    storeLastSyncUnix(unixSec);
    DBG.info("[%s] NTP: synced",tag);
    return true;
}

// ============================================================================
// Main loop
// ============================================================================
[[noreturn]] void App::run() {
    bool wokeFromStop = false;
    bool firstCycle   = true;

    while (true) {
        // ── Отложенный старт веб-сервера ──────────────────────────────────
        // Если init() не смог открыть сокет (eth не поднят на boot),
        // пробуем повторно каждую итерацию главного цикла.
        if (m_webStartPending) {
            if (ensureEthReady()) {
                m_webServer.init(&m_sensor, &m_sdBackup, &m_battery, this);
                m_webServer.setRtc(&m_rtc);
                m_webServer.setSdOk(m_sdOk);
                if (m_webServer.isRunning()) {
                    m_webStartPending = false;
                    DBG.info("WebServer: deferred start OK");
                }
            }
        }
        // ─────────────────────────────────────────────────────────────────

        CfgUartBridge_Tick();

        if (eth.ready()) eth.tick();

        // Веб-сервер tick
        if (m_webServer.isRunning()) m_webServer.tick();

        // FIX: синхронизируем глобальный флаг
        g_web_exclusive = m_webActive;
        // SD статус синхронизируем в WebServer каждый цикл (SD может восстановиться)
        m_webServer.setSdOk(m_sdOk);

        m_channelMgr.tick();
        m_mode = readMode();

        const bool isWake = firstCycle || wokeFromStop;
        const char* tag   = firstCycle ? "BOOT" : (wokeFromStop ? "WAKE" : "RUN");

        if (isWake) syncRtcWithNtpIfNeeded(tag, true);

        m_battery.update();
        if (m_battery.isLow())
            DBG.warn("Battery LOW: %u%% (%.1fV)",
                     (unsigned)m_battery.getPercent(), (double)m_battery.getVoltage());

        // Poll sensors
        DateTime ts{};
        float val = m_sensor.read(ts);
        m_sensorAlive = (val > -9998.0f);

        // Накапливаем значения в буфер усреднения (Этап 2)
        // Аналог ocean-station: value_buffer[metric_id].push(val)
        {
            uint8_t cnt = m_sensor.getReadingCount();
            for (uint8_t _i = 0; _i < cnt && _i < MAX_AVG_CHANNELS; _i++) {
                const SensorReading& _r = m_sensor.getReading(_i);
                if (_r.valid) avgPush(_i, _r.value);
            }
            // Fallback: если нет multi-sensor readings — копим legacy val
            if (cnt == 0 && m_sensorAlive) avgPush(0, val);
        }
        const uint64_t unixMs = toUnixMs(ts);
        char tsStr[24]; u64ToDec(tsStr, sizeof(tsStr), unixMs);
        char timeStr[32]{}; ts.formatISO8601(timeStr);
        DBG.data("[%s] val=%.3f t=%s bat=%u%%",
                 tag, val, timeStr, (unsigned)m_battery.getPercent());

        ledBlink(1, 50);

        if (m_sdOk) {
            char line[Config::JSONL_LINE_MAX]; int l;
            if (m_sensor.getReadingCount() > 1 || Cfg().modbus_map_count > 0)
                l = buildMultiSensorPayload(line, sizeof(line), tsStr, ts, false);
            else
                l = buildPayload(line, sizeof(line), tsStr, val, ts, false);
            if (l > 0 && l < (int)sizeof(line)) {
                if (!m_sdBackup.appendLine(line)) {
                    DBG.error("SD: appendLine failed"); m_sdOk = false;
                    m_webServer.setSdOk(false); // sync SD status to web
                }
            }
        }

        if (m_webhook.isConfigured() && Cfg().webhook_trigger == WebhookTrigger::Event) {
            const SensorReading& r = m_sensor.getReading(0);
            m_webhook.sendTemplated(r.value, timeStr, r.name);
        }

        // ================================================================
        // Periodic send
        // ================================================================
        m_pollCounter++;
        if (m_pollCounter >= Cfg().send_interval_polls) {
            m_pollCounter = 0;
            int jsonLen;
            if (m_sensor.getReadingCount() > 1 || Cfg().modbus_map_count > 0)
                jsonLen = buildMultiSensorPayload(m_json, sizeof(m_json), tsStr, ts, true);
            else
                jsonLen = buildPayload(m_json, sizeof(m_json), tsStr, val, ts, true);

            if (jsonLen > 0 && jsonLen < (int)sizeof(m_json)) {
                // FIX: не трогаем W5500 сокеты пока веб активен
                if (m_webActive) {
                    DBG.info("[WEB_ACTIVE] send skipped, data queued to backup");
                    if (m_sdOk) m_sdBackup.appendLine(m_json);
                } else {
                    SendResult result = m_channelMgr.sendData(m_json, (uint16_t)jsonLen);
                    m_channelAlive = (result == SendResult::Ok);
                    if (result == SendResult::Ok) {
                        DBG.info("Data sent OK");
                        // Этап 2: сброс буферов усреднения после успешной отправки
                        // Аналог ocean-station: value_buffer.clear()
                        avgClearAll();
                    } else if (result == SendResult::SavedBackup) {
                        DBG.warn("Data saved to backup");
                        // Сбрасываем буфер и при сохранении в backup — данные уже записаны
                        avgClearAll();
                    } else {
                        DBG.error("Data send FAILED");
                        // Буфер НЕ сбрасываем при ошибке — накапливаем дальше
                    }
                }
            }
        }

        // FIX: проверяем таймаут веб-бездействия
        checkWebTimeout();
                processTestSend();

        // Backup retransmit — Этап 6: раздельные интервалы GSM/ETH и Iridium
        // GSM/ETH: backup_retry_gsm_sec (default 60, аналог ocean-station retry_all)
        // Iridium:  backup_retry_iridium_sec (default 600, аналог ocean-station retry_iridium)
        if (m_sdBackup.exists()) {
            uint32_t now = HAL_GetTick();
            const RuntimeConfig& rc = Cfg();
            // GSM/ETH retry — используем m_lastBackupSendTick (уже в app.hpp)
            uint32_t gsmInterval = (rc.backup_retry_gsm_sec > 0)
                ? rc.backup_retry_gsm_sec * 1000UL
                : rc.backup_send_interval_sec * 1000UL;
            if ((now - m_lastBackupSendTick) >= gsmInterval) {
                m_lastBackupSendTick = now;
                if (!m_webActive && !rc.iridium_enabled) retransmitBackup();
                else if (!m_webActive && !rc.channels.iridium_enabled) retransmitBackup();
            }
            // Iridium retry — отдельный static таймер
            if (rc.iridium_enabled || rc.channels.iridium_enabled) {
                uint32_t iridInterval = (rc.backup_retry_iridium_sec > 0)
                    ? rc.backup_retry_iridium_sec * 1000UL
                    : 600000UL; // fallback 600 сек
                if ((now - s_lastIridiumRetryTick) >= iridInterval) {
                    s_lastIridiumRetryTick = now;
                    if (!m_webActive) retransmitBackup();
                }
            }
        }

        if (m_webhook.isConfigured() && m_webhook.isScheduledDue()) {
            const SensorReading& r = m_sensor.getReading(0);
            m_webhook.sendTemplated(r.value, timeStr, r.name);
            m_webhook.markScheduledSent();
        }

        // ================================================================
        // Sleep / wait
        // FIX: когда веб активен — TIGHT WEB LOOP вместо 5-секундного ожидания
        // tick() вызывается каждые 5мс → мгновенный отклик браузера
        // ================================================================
        if (m_webActive) {
            // Tight web service loop — обслуживаем браузер без задержек
            uint32_t loopStart = HAL_GetTick();
            const uint32_t maxLoopMs = Cfg().poll_interval_sec * 1000UL;
            while ((HAL_GetTick() - loopStart) < maxLoopMs) {
                CfgUartBridge_Tick();
                if (eth.ready()) eth.tick();
                if (m_webServer.isRunning()) m_webServer.tick();
                g_web_exclusive = m_webActive;
                checkWebTimeout();
                processTestSend();
                IWDG->KR = 0xAAAA;
                HAL_Delay(5); // 200 запросов/сек — мгновенный отклик
                if (!m_webActive) {
                    // Веб-бездействие закончилось — выходим из tight loop
                    DBG.info("[WEB_IDLE] resuming data transmission");
                    break;
                }
            }
            wokeFromStop = false;
        } else if (m_mode == SystemMode::Sleep) {
            DBG.info("STOP %lu s...", (unsigned long)Cfg().poll_interval_sec);
            ledOff();
            m_mqtt.disconnect();
            m_power.enterStopMode(Cfg().poll_interval_sec);
            DBG.info("...wake");
            wokeFromStop = true;
        } else {
            // Debug mode: wait poll_interval but keep web + IWDG alive
            uint32_t t0 = HAL_GetTick();
            const uint32_t waitMs = Cfg().poll_interval_sec * 1000UL;
            while ((HAL_GetTick() - t0) < waitMs) {
                CfgUartBridge_Tick();
                if (eth.ready()) eth.tick();
                if (m_webServer.isRunning()) m_webServer.tick();
                checkWebTimeout();
                processTestSend();
                IWDG->KR = 0xAAAAU;
                // Выходим немедленно при входящем TCP-соединении (до первого запроса)
                // или когда браузер уже активен (m_webActive выставлен handleRequest)
                if (m_webActive
                    || getSn_SR(WebServer::HTTP_SOCKET) == SOCK_ESTABLISHED
                    || getSn_RX_RSR(WebServer::HTTP_SOCKET) > 0) break;
                HAL_Delay(5);
            }
            wokeFromStop = false;
        }

        firstCycle = false;
        IWDG->KR = 0xAAAA;
    }
}

void App::transmitSingle(float value,const DateTime& dt) {
    char tsStr[24]; u64ToDec(tsStr,sizeof(tsStr),toUnixMs(dt));
    char j[Config::JSON_BUFFER_SIZE]; int len=buildPayload(j,sizeof(j),tsStr,value,dt,true);
    if(len<=0||len>=(int)sizeof(j)) return;
    m_channelMgr.sendData(j,(uint16_t)len);
}

void App::transmitBuffer() { retransmitBackup(); }

void App::retransmitBackup() {
    // Отправляем последовательно: порт 0 → порт 1 → порт 2
    // Каждый файл передаётся полностью до перехода к следующему.
    SdBackup* backs[3] = { &m_sdBackup, &m_sdBackup1, &m_sdBackup2 };
    const uint32_t maxPayload = (Config::HTTP_CHUNK_MAX < Config::JSON_BUFFER_SIZE)
                                 ? Config::HTTP_CHUNK_MAX
                                 : (Config::JSON_BUFFER_SIZE - 1);
    for (uint8_t p = 0; p < 3; p++) {
        SdBackup* bk = backs[p];
        if (!bk->exists()) continue;
        DBG.info("Backup retransmit port%u file=%s", (unsigned)p, bk->filename());
        while (bk->exists()) {
            uint32_t lines = 0; FSIZE_t used = 0;
            bool ok = bk->readChunkAsJsonArray(m_json, sizeof(m_json), maxPayload, lines, used);
            if (!ok || lines == 0 || used == 0) {
                DBG.warn("Backup port%u: empty or unreadable, skip", (unsigned)p);
                break;
            }
            SendResult result = m_channelMgr.sendData(m_json, (uint16_t)std::strlen(m_json));
            if (result == SendResult::Ok) {
                bk->consumePrefix(used);
            } else {
                DBG.error("Backup retransmit port%u failed, abort", (unsigned)p);
                return;  // прерываем всю цепочку — связь пропала
            }
        }
        DBG.info("Backup port%u fully transmitted", (unsigned)p);
    }
    DBG.info("All backups transmitted (ports 0-2)");
}

