/*
 * ============================================================================
 *  极米 Play5  ×  米家 / 小爱同学  —— ESP32 桥接固件
 * ============================================================================
 *
 *  工作方式：
 *    米家/小爱  --(巴法云 MQTT)-->  ESP32  --+-- BLE 广播重放  --> 投影仪开机
 *                                            +-- WiFi UDP 指令 --> 投影仪关机
 *
 *  为什么开机要用蓝牙：极米关机后 Wi-Fi 全部断电，UDP 叫不醒它；
 *  遥控器的电源键其实是发一包 BLE 广播（含本机专属 token），所以 ESP32
 *  只需要"学"到这包广播，之后原样重放即可。
 *
 *  三个来源的协议依据：
 *    1. 关机：UDP -> 投影仪:16735  "KEYPRESSES:30"（关机键）
 *              UDP -> 投影仪:16750  {"action":20000,...}（快速关机）
 *    2. 开机：BLE 广播，AD 结构 = 02 01 06 / 03 03 12 18 / 13 FF 46 00 <token>
 *             （46 00 = 极米的厂商 ID 0x0046）
 *    3. 云端：巴法云 MQTT，mqtt.bemfa.com:9501，clientID 填私钥，
 *             订阅主题收到 on / off
 *
 *  首次使用（不用改任何代码）：
 *    1. 上电后手机连 Wi-Fi "JimiPlay5-Setup"（密码 12345678），会自动弹出配网页
 *    2. 填：家里 Wi-Fi、巴法云私钥、巴法云主题、投影仪 IP → 保存
 *    3. 打开 http://jimiplay5.local → 学习遥控器 → 绑定米家，详见 README
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <NimBLEDevice.h>

// ------------------------------ 可改的默认值 --------------------------------
#define FIRMWARE_VERSION     "v1.0.0"
//  出于安全考虑这里全部留空：首次上电会进入配网热点（JimiPlay5-Setup / 12345678），
//  在配网页面里填家里 Wi-Fi、巴法云私钥、主题、投影仪 IP，存进 Flash，掉电不丢。
//  如果你想免配网、直接把 Wi-Fi 写死在固件里，也可以在下面填上（不要把填好的代码传到公开仓库）。
#define DEFAULT_WIFI_SSID    ""               // 家里 Wi-Fi 名称（2.4GHz），留空 = 走配网
#define DEFAULT_WIFI_PASS    ""               // Wi-Fi 密码
#define DEFAULT_BEMFA_UID    ""               // 巴法云私钥（cloud.bemfa.com 控制台右上角）
#define DEFAULT_BEMFA_TOPIC  ""               // 巴法云主题，结尾必须是 006（= 开关设备）
#define BEMFA_HOST           "mqtt.bemfa.com"
#define BEMFA_PORT           9501

#define UDP_KEY_PORT         16735            // 简单按键 API
#define UDP_FAST_PORT        16750            // 复杂指令 API（快速关机）
#define LED_PIN              2                // 板载 LED
#define BLE_ADV_MS           4000             // 开机广播持续时长
#define LEARN_WINDOW_MS      90000UL          // 学习窗口 90 秒（抓到后 2.5 秒自动停）

// 极米广播的固定前缀：Flags + 16bit Service UUID(0x1812 HID) + Appearance
// 只在「手动粘贴纯厂商数据」时用来补齐，完整报文不会走这里
static const char *XGIMI_ADV_PREFIX = "020105030312180319c103";
#define XGIMI_COMPANY_ID 0x0046

// ------------------------------ 全局对象 ------------------------------------
Preferences prefs;
WebServer   server(80);
WiFiClient  netClient;
PubSubClient mqtt(netClient);

NimBLEScan *bleScan = nullptr;
bool bleReady = false;

// ------------------------------ 运行状态 ------------------------------------
String cfgIp;        // 投影仪局域网 IP
String cfgUid;       // 巴法云私钥
String cfgTopic;     // 巴法云主题
String cfgAdvHex;    // 学到的 BLE 广播（十六进制）

bool          projectorOn   = false;
unsigned long lastActionMs  = 0;
String        lastReported  = "";
unsigned long lastReportMs  = 0;
unsigned long ignoreReplayUntil = 0;   // 连上云端后忽略历史消息的时间点

bool          learning      = false;
unsigned long learnStartMs  = 0;
String        learnHitHex   = "";
int           learnHitRssi  = 0;
String        learnHitMac   = "";
unsigned long learnHitCount = 0;
unsigned long learnLastHitMs = 0;
String        lastEvent     = "上电启动";

// ============================================================================
//  小工具
// ============================================================================
static String bytesToHex(const uint8_t *buf, size_t len) {
    static const char *H = "0123456789abcdef";
    String s;
    s.reserve(len * 2);
    for (size_t i = 0; i < len; i++) {
        s += H[buf[i] >> 4];
        s += H[buf[i] & 0x0F];
    }
    return s;
}

static size_t hexToBytes(const String &hex, uint8_t *out, size_t maxLen) {
    size_t n = 0;
    for (size_t i = 0; i + 1 < hex.length() && n < maxLen; i += 2) {
        char b[3] = {hex[i], hex[i + 1], 0};
        out[n++] = (uint8_t)strtol(b, nullptr, 16);
    }
    return n;
}

static void saveStr(const char *key, const String &val) {
    prefs.begin("jimi", false);
    prefs.putString(key, val);
    prefs.end();
}

static void loadConfig() {
    prefs.begin("jimi", true);
    cfgIp     = prefs.getString("ip", "");
    cfgUid    = prefs.getString("uid", DEFAULT_BEMFA_UID);
    cfgTopic  = prefs.getString("topic", DEFAULT_BEMFA_TOPIC);
    cfgAdvHex = prefs.getString("adv", "");
    prefs.end();
    if (cfgUid.isEmpty())   cfgUid = DEFAULT_BEMFA_UID;
    if (cfgTopic.isEmpty()) cfgTopic = DEFAULT_BEMFA_TOPIC;
}

// ============================================================================
//  Wi-Fi 断开原因（排查用）
// ============================================================================
static void onWifiEvent(WiFiEvent_t ev, WiFiEventInfo_t info) {
    if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        uint8_t r = info.wifi_sta_disconnected.reason;
        const char *why = "未知";
        switch (r) {
            case 2:   why = "认证超时（AP 拒绝）"; break;
            case 3:   why = "AP 主动断开"; break;
            case 4:   why = "关联超时"; break;
            case 15:  why = "四次握手超时 = 密码多半不对"; break;
            case 201: why = "找不到这个 AP"; break;
            case 202: why = "认证失败"; break;
            case 203: why = "关联失败"; break;
            case 204: why = "握手超时"; break;
            case 205: why = "连接失败"; break;
            case 8:   why = "AP 离开"; break;
        }
        Serial.printf("[WiFi] 断开，原因码=%d（%s）\n", r, why);
    }
}

// ============================================================================
//  周边 Wi-Fi 扫描（排查连不上用）
// ============================================================================
static const char *authName(wifi_auth_mode_t m) {
    switch (m) {
        case WIFI_AUTH_OPEN:            return "OPEN";
        case WIFI_AUTH_WEP:             return "WEP";
        case WIFI_AUTH_WPA_PSK:         return "WPA";
        case WIFI_AUTH_WPA2_PSK:        return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-企业";
        case WIFI_AUTH_WPA3_PSK:        return "WPA3(不支持)";
        case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
        default:                        return "其他";
    }
}

static void dumpScan() {
    // 连接过程中扫描会失败，先断开
    WiFi.disconnect(true, false);
    delay(400);
    int n = WiFi.scanNetworks(false, true);   // 同步扫描，包含隐藏 SSID
    Serial.printf("[扫描] 发现 %d 个 Wi-Fi\n", n);
    if (n <= 0) {
        Serial.printf("       扫描返回 %d：天线/距离问题，或 2.4G 已关闭\n", n);
        return;
    }
    for (int i = 0; i < n; i++) {
        int ch = WiFi.channel(i);
        String ssid = WiFi.SSID(i);
        Serial.printf("       %-26s RSSI=%4d  信道=%2d(%s)  %s\n",
                      ssid.c_str(), WiFi.RSSI(i), ch,
                      ch <= 14 ? "2.4G" : "5G", authName(WiFi.encryptionType(i)));
        // 可疑项打印真实字节，排查隐藏空格/全角字符
        if (ssid == DEFAULT_WIFI_SSID || ssid.length() <= 5) {
            Serial.printf("           ^ 长度=%u 字节=", ssid.length());
            for (unsigned k = 0; k < ssid.length(); k++)
                Serial.printf("%02X ", (uint8_t)ssid[k]);
            Serial.println();
        }
    }
    WiFi.scanDelete();
}

// 先扫描拿到 BSSID + 信道，再定向连接：可绕开 "NO_AP_FOUND"
static bool connectViaScan(unsigned long timeoutMs) {
    int n = WiFi.scanNetworks(false, true);
    int best = -1, bestRssi = -1000;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) != DEFAULT_WIFI_SSID) continue;
        Serial.printf("[WiFi] 扫描命中 %s  RSSI=%d  CH=%d\n",
                      WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
        if (WiFi.RSSI(i) > bestRssi) { bestRssi = WiFi.RSSI(i); best = i; }
    }
    if (best < 0) {
        Serial.println("[WiFi] 扫描结果里没有这个 SSID（名称可能不完全一致）");
        WiFi.scanDelete();
        return false;
    }
    int ch = WiFi.channel(best);
    const uint8_t *bssid = WiFi.BSSID(best);
    WiFi.scanDelete();
    delay(100);

    WiFi.begin(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS, ch, bssid);
    unsigned long t0 = millis();
    Serial.print("[WiFi] 定向连接中 ");
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
}

// ============================================================================
//  BLE：学习 + 重放开机广播
// ============================================================================
// 解析 AD 结构，找厂商 ID 为 0x0046（极米）的那一段
static bool extractXgimiPayload(const std::string &payload, std::string &out) {
    const uint8_t *p = (const uint8_t *)payload.data();
    size_t len = payload.length(), i = 0;
    while (i + 1 < len) {
        uint8_t alen = p[i];
        if (alen == 0) break;
        if (i + 1 + alen > len) break;
        uint8_t type = p[i + 1];
        if (type == 0xFF && alen >= 4) {
            uint16_t cid = (uint16_t)p[i + 2] | ((uint16_t)p[i + 3] << 8);
            if (cid == XGIMI_COMPANY_ID) {
                out.assign(payload);
                return true;
            }
        }
        i += alen + 1;
    }
    return false;
}

class XgimiScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice *dev) override {
        if (!learning) return;

        uint8_t *rawPtr = dev->getPayload();
        size_t   rawLen = dev->getPayloadLength();
        if (!rawPtr || rawLen < 4) return;
        std::string raw((const char *)rawPtr, rawLen);
        std::string hit;
        if (!extractXgimiPayload(raw, hit)) return;

        learnHitHex   = bytesToHex((const uint8_t *)hit.data(), hit.length());
        learnHitRssi  = dev->getRSSI();
        learnHitMac   = String(dev->getAddress().toString().c_str());
        learnHitCount++;
        learnLastHitMs = millis();

        Serial.printf("[BLE] 捕获极米广播 #%lu  MAC=%s RSSI=%d  %s\n",
                      learnHitCount, learnHitMac.c_str(), learnHitRssi,
                      learnHitHex.c_str());
    }
};

static void bleInit() {
    if (bleReady) return;

    // ★ 关键：ESP32 的 Wi-Fi 与 BLE 共用同一套射频，靠「软件共存」分时调度。
    // 如果 Wi-Fi 关掉了 modem sleep（WIFI_PS_NONE，即 WiFi.setSleep(false)），
    // ESP-IDF 在启用蓝牙控制器时会直接在 coex_core_enable() 里 abort 重启。
    // 所以开蓝牙之前必须把 Wi-Fi 省电模式恢复成 MIN_MODEM。
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    delay(50);

    NimBLEDevice::init("JimiBridge");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // 最大发射功率，穿透力强一点
    bleReady = true;
    Serial.println("[BLE] 协议栈就绪（Wi-Fi 已切到共存模式）");
}

// 把学到的报文原样广播出去
static bool bleWake() {
    if (cfgAdvHex.length() < 8) {
        Serial.println("[BLE] 还没学到遥控器报文，无法开机");
        return false;
    }
    bleInit();

    // 学到的报文已经是完整 AD 结构（Flags 开头 02 01 xx），必须原样重放。
    // 只有手动粘贴的「纯厂商数据」才补前缀——补错了会超过 31 字节上限被截断。
    String hex = cfgAdvHex;
    if (!hex.startsWith("0201")) hex = String(XGIMI_ADV_PREFIX) + hex;

    uint8_t buf[64];
    size_t n = hexToBytes(hex, buf, sizeof(buf));
    if (n < 8) return false;
    if (n > 31) {
        // BLE 4.x 传统广播 payload 上限 31 字节，超了控制器会截断
        Serial.printf("[BLE] 注意：报文 %u 字节超过 31，已按上限截断、可能无效\n", (unsigned)n);
        n = 31;
    }

    NimBLEAdvertisementData advData;
    advData.addData(std::string((const char *)buf, n));

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->stop();
    adv->setAdvertisementData(advData);
    adv->setMinInterval(32);   // 20ms
    adv->setMaxInterval(32);
    if (!adv->start(0)) {
        Serial.println("[BLE] 广播启动失败");
        return false;
    }
    Serial.printf("[BLE] 重放开机广播 %u 字节，持续 %d ms\n", (unsigned)n, BLE_ADV_MS);
    delay(BLE_ADV_MS);
    adv->stop();
    Serial.println("[BLE] 广播结束");
    return true;
}

static void bleLearnStart() {
    bleInit();
    if (!bleScan) {
        bleScan = NimBLEDevice::getScan();
        bleScan->setAdvertisedDeviceCallbacks(new XgimiScanCB(), false);
        bleScan->setActiveScan(false);
    }
    // Wi-Fi 和 BLE 共用射频：扫描占空比必须留出空口给 Wi-Fi，
    // 否则网页打不开、MQTT 断线。100ms 周期扫 32ms，约 1/3 占空比。
    if (bleScan->isScanning()) bleScan->stop();
    bleScan->setInterval(160);
    bleScan->setWindow(48);
    bleScan->clearResults();
    learnHitHex = "";
    learnHitCount = 0;
    learnHitMac = "";
    learnHitRssi = 0;
    learning = true;
    learnStartMs = millis();

    if (!bleScan->start(0, nullptr, false)) {
        learning = false;
        Serial.println("[BLE] 扫描启动失败");
        return;
    }
    Serial.printf("[BLE] 学习模式：请对着 ESP32 按遥控器电源键（%lu 秒内）\n",
                  LEARN_WINDOW_MS / 1000);
    lastEvent = "学习中：请按遥控器电源键";
}

static void bleLearnStop(bool persist) {
    if (bleScan) bleScan->stop();
    learning = false;

    if (persist && learnHitHex.length() > 8) {
        cfgAdvHex = learnHitHex;
        saveStr("adv", cfgAdvHex);
        Serial.printf("[BLE] 已保存广播报文：%s\n", cfgAdvHex.c_str());
        lastEvent = "已学到遥控器开机广播";
    } else if (learnHitCount == 0) {
        Serial.println("[BLE] 没抓到极米广播，请靠近一点再试");
        lastEvent = "学习失败：没抓到广播";
    }
}

// ============================================================================
//  投影仪控制
// ============================================================================
static bool projectorOff() {
    if (cfgIp.isEmpty()) {
        Serial.println("[UDP] 没有配置投影仪 IP，无法关机");
        return false;
    }
    WiFiUDP udp;
    udp.beginPacket(cfgIp.c_str(), UDP_KEY_PORT);
    udp.write((const uint8_t *)"KEYPRESSES:30", 13);   // 关机键
    udp.endPacket();
    delay(120);

    // 复杂指令 API 的快速关机，作为兜底
    const char *fast =
        "{\"action\":20000,\"controlCmd\":{\"delayTime\":0,\"mode\":6,\"time\":0,\"type\":0},\"msgid\":\"2\"}";
    udp.beginPacket(cfgIp.c_str(), UDP_FAST_PORT);
    udp.write((const uint8_t *)fast, strlen(fast));
    udp.endPacket();
    udp.stop();

    Serial.printf("[UDP] 已向 %s 发送关机指令\n", cfgIp.c_str());
    return true;
}

static void reportState() {
    if (!mqtt.connected()) return;
    const char *st = projectorOn ? "on" : "off";
    mqtt.publish(cfgTopic.c_str(), st, false);
    lastReported = st;
    lastReportMs = millis();
}

static void doPowerOn(const char *from) {
    Serial.printf("[动作] 开机（来源：%s）\n", from);
    if (bleWake()) {
        projectorOn = true;
        lastActionMs = millis();
        lastEvent = String("已发送开机广播（") + from + "）";
        reportState();
    } else {
        lastEvent = "开机失败：未学到遥控器报文";
    }
}

static void doPowerOff(const char *from) {
    Serial.printf("[动作] 关机（来源：%s）\n", from);
    if (projectorOff()) {
        projectorOn = false;
        lastActionMs = millis();
        lastEvent = String("已发送关机指令（") + from + "）";
        reportState();
    } else {
        lastEvent = "关机失败：未配置投影仪 IP";
    }
}

// ============================================================================
//  MQTT（巴法云）
// ============================================================================
static void mqttCallback(char *topic, byte *payload, unsigned int len) {
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)tolower(payload[i]);
    msg.trim();
    Serial.printf("[MQTT] 收到 [%s] -> %s\n", topic, msg.c_str());

    if (msg != "on" && msg != "off") return;

    // 巴法云会把主题上的最后一条消息回放给新订阅者，
    // 刚连上时收到的是历史消息，不能当成新指令（否则每次重启都会开机）
    if (millis() < ignoreReplayUntil) {
        Serial.println("[MQTT] 忽略连接后回放的历史消息");
        return;
    }

    // 抑制自己上报状态时被云端回推造成的重复触发
    if (msg == lastReported && millis() - lastReportMs < 5000) return;
    // 2 秒内重复指令直接忽略
    if (millis() - lastActionMs < 2000) return;

    if (msg == "on")  doPowerOn("米家/小爱");
    else              doPowerOff("米家/小爱");
}

static void mqttEnsure() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt.connected()) { mqtt.loop(); return; }

    static unsigned long nextTry = 0;
    if (millis() < nextTry) return;
    nextTry = millis() + 5000;

    if (cfgUid.isEmpty() || cfgTopic.isEmpty()) {
        static bool warned = false;
        if (!warned) {
            Serial.println("[MQTT] 未配置私钥/主题，云端功能暂不可用；请到网页控制台「参数设置」里填写");
            warned = true;
        }
        return;
    }

    Serial.printf("[MQTT] 连接 %s:%d （clientID=%s）...", BEMFA_HOST, BEMFA_PORT, cfgUid.c_str());
    // 巴法云：clientID 就是私钥，用户名密码留空
    if (mqtt.connect(cfgUid.c_str())) {
        Serial.println(" 成功");
        mqtt.subscribe(cfgTopic.c_str());
        ignoreReplayUntil = millis() + 4000;
        Serial.printf("[MQTT] 已订阅主题 %s\n", cfgTopic.c_str());
        reportState();
    } else {
        Serial.printf(" 失败 rc=%d，5 秒后重试\n", mqtt.state());
    }
}

// ============================================================================
//  Web 控制台
// ============================================================================
static const char *PAGE_HEAD =
    "<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>极米投影仪米家桥</title><style>"
    "body{font-family:-apple-system,'PingFang SC',sans-serif;margin:0;background:#f5f6f8;color:#23262b}"
    ".wrap{max-width:560px;margin:0 auto;padding:18px 16px 40px}"
    "h1{font-size:19px;margin:4px 0 16px;font-weight:650}"
    ".card{background:#fff;border-radius:14px;padding:14px 16px;margin-bottom:14px;"
    "box-shadow:0 1px 3px rgba(0,0,0,.06)}"
    ".row{display:flex;justify-content:space-between;gap:12px;padding:7px 0;"
    "border-bottom:1px solid #f1f2f4;font-size:14px}.row:last-child{border:0}"
    ".k{color:#878c96}.v{font-weight:600;text-align:right;word-break:break-all}"
    "button{appearance:none;border:0;border-radius:11px;padding:13px;font-size:15px;"
    "background:#2f6fed;color:#fff;width:100%;margin-top:10px;font-weight:600}"
    "button.gray{background:#eceef3;color:#333}button.red{background:#e8543f}"
    "input{width:100%;box-sizing:border-box;padding:10px 11px;border:1px solid #dcdfe6;"
    "border-radius:9px;font-size:15px;margin:4px 0 10px}"
    "label{font-size:13px;color:#6b7280}"
    ".tag{display:inline-block;font-size:12px;padding:2px 8px;border-radius:20px}"
    ".on{background:#e6f6ec;color:#0a8a3f}.off{background:#f1f2f4;color:#6b7280}"
    ".hint{font-size:13px;color:#878c96;line-height:1.6;margin-top:6px}"
    "</style></head><body><div class=\"wrap\"><h1>极米投影仪 · 米家桥 " FIRMWARE_VERSION "</h1>";

static String pageBody() {
    String h;
    bool wifiOk = WiFi.status() == WL_CONNECTED;

    h += "<div class=\"card\">";
    h += "<div class=\"row\"><span class=\"k\">Wi-Fi</span><span class=\"v\">";
    h += wifiOk ? WiFi.SSID() + "<br>" + WiFi.localIP().toString() : "未连接";
    h += "</span></div>";
    h += "<div class=\"row\"><span class=\"k\">巴法云</span><span class=\"v\">";
    h += mqtt.connected() ? "<span class=\"tag on\">已连接</span>"
         : (cfgUid.isEmpty() ? "未配置（去下面 ③ 填写）" : "<span class=\"tag off\">未连接</span>");
    h += "</span></div>";
    h += "<div class=\"row\"><span class=\"k\">主题</span><span class=\"v\">" + cfgTopic + "</span></div>";
    h += "<div class=\"row\"><span class=\"k\">投影仪 IP</span><span class=\"v\">" +
         (cfgIp.isEmpty() ? "未配置" : cfgIp) + "</span></div>";
    h += "<div class=\"row\"><span class=\"k\">开机广播</span><span class=\"v\">" +
         String(cfgAdvHex.length() > 8 ? "已学习" : "未学习") + "</span></div>";
    h += "<div class=\"row\"><span class=\"k\">当前状态</span><span class=\"v\">" +
         String(projectorOn ? "开" : "关") + "</span></div>";
    h += "<div class=\"row\"><span class=\"k\">最近事件</span><span class=\"v\">" + lastEvent + "</span></div>";
    h += "</div>";

    // 学习
    h += "<div class=\"card\"><div style=\"font-weight:650;margin-bottom:6px\">① 学习遥控器开机广播</div>";
    h += "<div class=\"hint\">点下面按钮后，把极米遥控器靠近 ESP32（10cm 内）按一下<b>电源键</b>，"
         "投影仪关机状态也可以。最多等 90 秒，抓到会自动停并存进 Flash，掉电不丢。</div>";
    if (learning) {
        unsigned long left = (millis() - learnStartMs < LEARN_WINDOW_MS)
                                 ? (LEARN_WINDOW_MS - (millis() - learnStartMs)) / 1000 : 0;
        h += "<div class=\"row\" style=\"margin-top:8px\"><span class=\"k\">剩余</span><span class=\"v\">" +
             String(left) + " 秒</span></div>";
        h += "<div class=\"row\"><span class=\"k\">已捕获</span><span class=\"v\">" +
             String(learnHitCount) + " 包</span></div>";
        if (learnHitHex.length()) {
            h += "<div class=\"row\"><span class=\"k\">最新报文</span><span class=\"v\" style=\"font-size:12px\">" +
                 learnHitHex + "</span></div>";
        }
    } else if (cfgAdvHex.length() > 8) {
        h += "<div class=\"row\" style=\"margin-top:8px\"><span class=\"k\">已存报文</span>"
             "<span class=\"v\" style=\"font-size:12px\">" + cfgAdvHex + "</span></div>";
    }
    h += "<form method=\"POST\" action=\"/learn\"><button>开始学习（按一下遥控器电源键）</button></form>";
    h += "</div>";

    // 测试
    h += "<div class=\"card\"><div style=\"font-weight:650;margin-bottom:6px\">② 手动测试</div>";
    h += "<form method=\"POST\" action=\"/on\"><button>测试开机（发蓝牙广播）</button></form>";
    h += "<form method=\"POST\" action=\"/off\"><button class=\"gray\">测试关机（发 UDP）</button></form>";
    h += "<div class=\"hint\">开机后投影仪要 20~30 秒才连上 Wi-Fi，关机请等它亮着的时候测。</div>";
    h += "</div>";

    // 配置
    h += "<div class=\"card\"><div style=\"font-weight:650;margin-bottom:10px\">③ 参数设置</div>";
    h += "<form method=\"POST\" action=\"/config\">";
    h += "<label>投影仪局域网 IP</label><input name=\"ip\" value=\"" + cfgIp + "\" placeholder=\"192.168.1.50\">";
    h += "<label>巴法云私钥（cloud.bemfa.com 控制台右上角，32 位）</label>"
         "<input name=\"uid\" value=\"" + cfgUid + "\" placeholder=\"留空则保持不变\">";
    h += "<label>巴法云主题（结尾必须是 006）</label>"
         "<input name=\"topic\" value=\"" + cfgTopic + "\" placeholder=\"例如 myproj006\">";
    h += "<button>保存</button></form>";
    h += "<form method=\"POST\" action=\"/reset\"><button class=\"red\">清除开机广播 / 重启</button></form>";
    h += "</div>";

    h += "<div class=\"hint\" style=\"text-align:center\">固件 " FIRMWARE_VERSION " · ESP32 " +
         String(ESP.getFreeHeap() / 1024) + " KB 可用内存 · 运行 " + String(millis() / 1000) + " 秒</div>";
    return h;
}

static void handleRoot() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/html; charset=utf-8", String(PAGE_HEAD) + pageBody() + "</div></body></html>");
}

static void redirectHome() {
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "");
}

static void handleConfig() {
    if (server.hasArg("ip"))    saveStr("ip", server.arg("ip"));
    if (server.hasArg("uid"))   saveStr("uid", server.arg("uid"));
    if (server.hasArg("topic")) saveStr("topic", server.arg("topic"));
    loadConfig();
    mqtt.disconnect();
    lastEvent = "参数已保存";
    redirectHome();
}

static void setupWeb() {
    server.on("/", handleRoot);
    server.on("/on", HTTP_POST, []() { doPowerOn("网页"); redirectHome(); });
    server.on("/off", HTTP_POST, []() { doPowerOff("网页"); redirectHome(); });
    server.on("/learn", HTTP_POST, []() {
        if (learning) bleLearnStop(true);
        // 先把响应发出去，再启动蓝牙扫描（初始化要几百毫秒，别让浏览器干等）
        redirectHome();
        bleLearnStart();
    });
    server.on("/config", HTTP_POST, handleConfig);
    server.on("/reset", HTTP_POST, []() {
        saveStr("adv", "");
        cfgAdvHex = "";
        lastEvent = "已清除，重启中";
        redirectHome();
        delay(300);
        ESP.restart();
    });
    server.on("/api/status", []() {
        String j = "{\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
                   ",\"ip\":\"" + WiFi.localIP().toString() + "\"" +
                   ",\"mqtt\":" + String(mqtt.connected() ? "true" : "false") +
                   ",\"topic\":\"" + cfgTopic + "\"" +
                   ",\"projector\":\"" + cfgIp + "\"" +
                   ",\"adv\":\"" + cfgAdvHex + "\"" +
                   ",\"on\":" + String(projectorOn ? "true" : "false") + "}";
        server.send(200, "application/json", j);
    });
    server.onNotFound(handleRoot);
    server.begin();
    Serial.println("[WEB] http://jimiplay5.local 或 http://" + WiFi.localIP().toString());
}

// ============================================================================
//  板载 LED：Wi-Fi 未连=快闪，MQTT 未连=慢闪，全连=常亮
// ============================================================================
static void updateLed() {
    static unsigned long t = 0;
    bool on = WiFi.status() == WL_CONNECTED;
    bool ok = mqtt.connected();
    if (on && ok) {
        digitalWrite(LED_PIN, HIGH);
        return;
    }
    unsigned long period = on ? 1200 : 220;
    if (millis() - t > period / 2) {
        t = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

// ============================================================================
//  串口命令（调试用）
// ============================================================================
static void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    if (line == "on")        doPowerOn("串口");
    else if (line == "off")  doPowerOff("串口");
    else if (line == "learn") bleLearnStart();
    else if (line == "save") bleLearnStop(true);
    else if (line == "scan") dumpScan();
    else if (line.startsWith("setip ")) {
        cfgIp = line.substring(6); cfgIp.trim();
        saveStr("ip", cfgIp);
        Serial.println("[CFG] 投影仪 IP 已设为 " + cfgIp);
    }
    else if (line.startsWith("settopic ")) {
        cfgTopic = line.substring(9); cfgTopic.trim();
        saveStr("topic", cfgTopic);
        mqtt.disconnect();
        Serial.println("[CFG] 主题已设为 " + cfgTopic + "（正在重新连接）");
    }
    else if (line == "clear") { saveStr("adv", ""); cfgAdvHex = ""; Serial.println("[CFG] 已清除广播"); }
    else if (line == "ip")   Serial.println("[CFG] 投影仪 IP = " + cfgIp);
    else if (line == "adv")  Serial.println("[CFG] 广播 = " + cfgAdvHex);
    else if (line == "info") {
        Serial.printf("[INFO] WiFi=%d IP=%s MQTT=%d 主题=%s heap=%u\n",
                      WiFi.status(), WiFi.localIP().toString().c_str(),
                      mqtt.connected(), cfgTopic.c_str(), ESP.getFreeHeap());
    } else {
        Serial.println("[串口] 可用命令：on / off / learn / save / clear / ip / adv / info");
    }
}

// ============================================================================
//  初始化
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n\n=== 极米投影仪米家桥 " FIRMWARE_VERSION " 启动 ===");
    {
        esp_reset_reason_t r = esp_reset_reason();
        const char *why = (r == ESP_RST_PANIC) ? "崩溃重启（panic）"
                        : (r == ESP_RST_POWERON) ? "上电"
                        : (r == ESP_RST_SW) ? "软件重启"
                        : (r == ESP_RST_EXT) ? "外部复位"
                        : (r == ESP_RST_BROWNOUT) ? "电压不足"
                        : "其它";
        Serial.printf("[系统] 本次启动原因：%s (%d)\n", why, (int)r);
    }

    pinMode(LED_PIN, OUTPUT);
    loadConfig();

    // ---- Wi-Fi：先用固件里写死的，连不上再开配网热点 ----
    //     想换路由器又进不去热点时：按住板上的 BOOT 键上电，强制进配网
    pinMode(0, INPUT_PULLUP);
    bool forcePortal = (digitalRead(0) == LOW);
    if (forcePortal) Serial.println("[WiFi] 检测到 BOOT 键按住，强制进入配网模式");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    // 默认只接受 WPA2 及以上，会把老式 WPA(TKIP) 的 AP 直接过滤掉（表现为"找不到 AP"）
    WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
    WiFi.onEvent(onWifiEvent);

    // 固件里写死了 Wi-Fi 才先直连；留空则直接走配网热点（推荐小白用户）
    bool hasBuiltinWifi = strlen(DEFAULT_WIFI_SSID) > 0;
    if (!forcePortal && hasBuiltinWifi) {
        if (!connectViaScan(15000)) {
            Serial.println("[WiFi] 定向连接失败，改用普通方式再试一次");
            WiFi.disconnect(true, false);
            delay(300);
            WiFi.begin(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS);
            unsigned long wifiT0 = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - wifiT0 < 15000) {
                delay(250);
                Serial.print(".");
            }
            Serial.println();
        }
    }

    if (forcePortal || !hasBuiltinWifi || WiFi.status() != WL_CONNECTED) {
        // 连不上时先扫一遍周边，方便排查是不是 5GHz / 加密方式不支持
        dumpScan();
        // 连不上（密码改了/路由器换了）才开门户，连热点可重新配网
        Serial.println("[WiFi] 连不上，打开配网热点 JimiPlay5-Setup / 密码 12345678");
        WiFiManager wm;
        wm.setConfigPortalTimeout(240);
        WiFiManagerParameter pIp("ip", "投影仪局域网 IP（可留空）", cfgIp.c_str(), 15);
        WiFiManagerParameter pUid("uid", "巴法云私钥", cfgUid.c_str(), 40);
        WiFiManagerParameter pTopic("topic", "巴法云主题（建议以 006 结尾）", cfgTopic.c_str(), 24);
        wm.addParameter(&pIp);
        wm.addParameter(&pUid);
        wm.addParameter(&pTopic);
        wm.setSaveParamsCallback([&]() {
            if (strlen(pIp.getValue()))    saveStr("ip", String(pIp.getValue()));
            if (strlen(pUid.getValue()))   saveStr("uid", String(pUid.getValue()));
            if (strlen(pTopic.getValue())) saveStr("topic", String(pTopic.getValue()));
        });
        if (!wm.autoConnect("JimiPlay5-Setup", "12345678")) {
            Serial.println("[WiFi] 配网超时，重启");
            ESP.restart();
        }
    }
    loadConfig();
    Serial.printf("[WiFi] 已连接 %s，IP=%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    if (MDNS.begin("jimiplay5")) MDNS.addService("http", "tcp", 80);

    // ---- 巴法云 ----
    mqtt.setServer(BEMFA_HOST, BEMFA_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(30);
    mqtt.setBufferSize(256);

    setupWeb();
    // BLE 等真正要用时再初始化，省内存
    Serial.println("[系统] 就绪，等待云端指令...");
    lastEvent = "就绪，等待云端指令";
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient();
        mqttEnsure();
    }

    if (learning) {
        bool timeout = millis() - learnStartMs > LEARN_WINDOW_MS;
        bool gotIt   = learnHitCount > 0 && millis() - learnLastHitMs > 2500;
        if (timeout || gotIt) bleLearnStop(true);
    }

    updateLed();
    handleSerial();
    delay(2);
}
