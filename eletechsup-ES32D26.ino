// WiFi+MQTT+DogStatsD + Relay control (74HC595) for ES32D26
// - Static IP
// - Samples Vi1..Vi4 every 15s (brief WiFi-off windows for ADC2)
// - Publishes iot.pressure (PSI) gauges via DogStatsD with tags
// - Subscribes to relay topics and energizes/de-energizes relays 1..6 (+7 pump)
// - Starts a 300s master timer on first command; after 300s all relays turn off

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <stdarg.h>

// Forward declaration so helpers can reference the global MQTT client
class PubSubClient; extern PubSubClient mqtt;

// Feature flags
#define FLOW_METRICS_ENABLED 1

// ------------ Network (static IP) ------------
const char* WIFI_SSID = "PrediccioNet";
const char* WIFI_PASS = "emulsify.impearl.tress";
IPAddress STATIC_IP(192,168,88,206);
IPAddress GATEWAY(192,168,88,1);
IPAddress SUBNET(255,255,255,0);
IPAddress DNS1(8,8,8,8);

// ------------ MQTT ------------
const char* MQTT_HOST = "192.168.88.205";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "eletechsup";
const char* MQTT_PASS = "ngv1udw0YBZ!ygk.tru";

// Topics (subscribe to both with and without leading slash for safety)
const char* topics_slash[] = {
  "/eletechsup/prefilter","/eletechsup/postfilter","/eletechsup/500","/eletechsup/250","/eletechsup/100","/eletechsup/50","/eletechsup/pump"
};
const char* topics[] = {
  "eletechsup/prefilter","eletechsup/postfilter","eletechsup/500","eletechsup/250","eletechsup/100","eletechsup/50","eletechsup/pump"
};
// Base topic to publish input terminal voltages
const char* INPUT_BASE_TOPIC = "/eletechsup/inputs";
// channel index map (0-based relay number)
// ch1=prefilter, ch2=postfilter, ch3=500, ch4=250, ch5=100, ch6=50

// ------------ Home Assistant Discovery ------------
static const char* FW_VERSION = "0.2.0";
static const char* HA_PREFIX = "homeassistant";
static const char* DEVICE_ID = "esp32_water";
static const char* AVAIL_TOPIC = "esp32_water/status";

// ------------ DogStatsD ------------
// DogStatsD disabled; using HTTPS metrics intake instead
const char* DD_HOST = "192.168.88.204"; // DogStatsD Agent host
const uint16_t DD_PORT = 8125;     // unused
WiFiUDP ddUdp;                     // unused
// Debug counters for UDP DogStatsD sends
volatile uint32_t dd_udp_ok = 0;
volatile uint32_t dd_udp_err = 0;

// Safe snprintf appender to avoid buffer overflows when batching metrics
static inline void add_snprintf(char* buf, size_t size, size_t& n, const char* fmt, ...){
  if(n >= size) return;
  va_list ap; va_start(ap, fmt);
  int ret = vsnprintf(buf + n, size - n, fmt, ap);
  va_end(ap);
  if(ret < 0) return;
  if((size_t)ret >= (size - n)) n = size - 1; else n += (size_t)ret;
}

// Datadog HTTPS intake (US1 site)
static const char* DD_API_HOST = "https://api.datadoghq.com";
static const char* DD_SERIES_PATH = "/api/v2/series";
static const char* DD_API_KEY = "<REDACTED_DD_API_KEY>";

// Common tags
static const char* DD_COMMON_TAGS = "env:prod,sensor:transducer,source:eletechsup,service:water";

// ------------ Status LEDs ------------
const int LED_WIFI = 2;
const int LED_MQTT = 4;

// ------------ Analog mapping ------------
const int VI1_PIN = 14; // ADC2
const int VI2_PIN = 33; // ADC1
const int VI3_PIN = 27; // ADC2
const int VI4_PIN = 32; // ADC1
static constexpr float ADC_TO_TERMINAL_GAIN = 5.0f;   // observed board scaling ~5x
static constexpr float SENSOR_FS_VOLTS      = 5.0f;   // 0..5V sensor
static constexpr float PSI_FS[4]            = {5.0f, 60.0f, 60.0f, 100.0f};
int vi_mV[4] = {0,0,0,0};

// ------------ Flow meters (pulse inputs) ------------
// IO18 and IO19 as interrupt-capable pulse counters
const int FLOW1_PIN = 18;
const int FLOW2_PIN = 19;
// K-factor: pulses per liter (update to your sensor's spec). Using placeholder default.
static constexpr float FLOW_K_PPL = 450.0f; // pulses/L (typical YF-S201 ~450-480)
volatile uint32_t flow1_pulses = 0;
volatile uint32_t flow2_pulses = 0;
unsigned long lastFlowPublishMs = 0;
float flow1_hz_last = 0.0f;
float flow2_hz_last = 0.0f;
float flow1_lpm_last = 0.0f;
float flow2_lpm_last = 0.0f;

void IRAM_ATTR flow1_isr(){ flow1_pulses++; }
void IRAM_ATTR flow2_isr(){ flow2_pulses++; }

// ------------ Relays via 74HC595 + ULN2803 ------------
const int SR_DATA  = 12;
const int SR_CLK   = 22;
const int SR_LATCH = 23;
const int SR_OE    = 13; // active LOW
uint8_t srState = 0x00; // bits 0..7 => outputs
// Map logical channels 1..6 to physical bit indices 2..7 (hardware wiring offset)
static const uint8_t RELAY_BIT_FOR_CHANNEL[6] = {2,3,4,5,6,7};

// Resolve the shift-register bit used for a given logical channel
// Channels 1..6 map via RELAY_BIT_FOR_CHANNEL; channel 7 (pump) uses bit 1; channel 8 uses bit 0
static inline uint8_t relayBitForChannel(int ch){
  if(ch>=1 && ch<=6) return RELAY_BIT_FOR_CHANNEL[ch-1];
  if(ch==7) return 1; // dedicate Q1 to pump to avoid collisions with channels 1..6
  if(ch==8) return 0; // optional spare on Q0
  return 0;
}

inline void srLatch(){ digitalWrite(SR_LATCH, LOW); digitalWrite(SR_LATCH, HIGH); }
inline void srWrite(uint8_t value){
  for(int i=0;i<8;++i){
    digitalWrite(SR_CLK, LOW);
    digitalWrite(SR_DATA, (value>>i)&1);
    digitalWrite(SR_CLK, HIGH);
  }
  srLatch();
}
void setRelay(int ch, bool on){ // ch:1..8
  uint8_t before = srState;
  if(ch>=1 && ch<=8){
    uint8_t bit = relayBitForChannel(ch);
    if(on) srState |=  (1 << bit); else srState &= ~(1 << bit);
  }
  if(srState != before){ srWrite(srState); }
}

static void publishRelayStateTopic(const char* name, int ch){
  // state topics separate from command topics for HA
  String topic = String("/eletechsup/") + name + "/state";
  uint8_t bit = relayBitForChannel(ch);
  const char* payload = ((srState >> bit) & 1) ? "1" : "0";
  mqtt.publish(topic.c_str(), payload, true);
}

static void publishAllRelayStates(){
  publishRelayStateTopic("prefilter", 1);
  publishRelayStateTopic("postfilter", 2);
  publishRelayStateTopic("500", 3);
  publishRelayStateTopic("250", 4);
  publishRelayStateTopic("100", 5);
  publishRelayStateTopic("50", 6);
  publishRelayStateTopic("pump", 7);
}

static void publishHADiscovery(){
  if(!mqtt.connected()) return;
  auto pubcfg = [&](const char* comp, const char* obj, const char* json){
    String t = String(HA_PREFIX) + "/" + comp + "/" + DEVICE_ID + "/" + obj + "/config";
    mqtt.publish(t.c_str(), json, true);
  };
  char buf[640];
  // Switches 1..6 + pump (7)
  struct Item{ const char* name; const char* cmd; const char* st; const char* uid; };
  Item sws[] = {
    {"prefilter","/eletechsup/prefilter","/eletechsup/prefilter/state","esp32_water_prefilter"},
    {"postfilter","/eletechsup/postfilter","/eletechsup/postfilter/state","esp32_water_postfilter"},
    {"500","/eletechsup/500","/eletechsup/500/state","esp32_water_500"},
    {"250","/eletechsup/250","/eletechsup/250/state","esp32_water_250"},
    {"100","/eletechsup/100","/eletechsup/100/state","esp32_water_100"},
    {"50","/eletechsup/50","/eletechsup/50/state","esp32_water_50"},
    {"pump","/eletechsup/pump","/eletechsup/pump/state","esp32_water_pump"}
  };
  for(auto &it: sws){
    snprintf(buf, sizeof(buf),
      "{\"name\":\"%s\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"pl_on\":\"1\",\"pl_off\":\"0\",\"avty_t\":\"%s\",\"uniq_id\":\"%s\",\"qos\":1,\"ret\":true,\"device\":{\"ids\":[\"%s\"],\"name\":\"ES32D26\",\"mf\":\"eletechsup\",\"mdl\":\"ES32D26\",\"sw\":\"%s\"}}",
      it.name, it.cmd, it.st, AVAIL_TOPIC, it.uid, DEVICE_ID, FW_VERSION);
    pubcfg("switch", it.name, buf);
  }
  // Flow sensors (tank/house), LPM and Hz
  struct Sen{ const char* name; const char* st; const char* uid; const char* unit; };
  Sen sens_lpm[] = {
    {"flow_tank_lpm","/eletechsup/flow1_lpm","esp32_water_flow_tank_lpm","L/min"},
    {"flow_house_lpm","/eletechsup/flow2_lpm","esp32_water_flow_house_lpm","L/min"}
  };
  for(auto &s: sens_lpm){
    snprintf(buf, sizeof(buf),
      "{\"name\":\"%s\",\"stat_t\":\"%s\",\"unit_of_meas\":\"%s\",\"avty_t\":\"%s\",\"uniq_id\":\"%s\",\"state_class\":\"measurement\",\"device\":{\"ids\":[\"%s\"],\"name\":\"ES32D26\",\"mf\":\"eletechsup\",\"mdl\":\"ES32D26\",\"sw\":\"%s\"}}",
      s.name, s.st, s.unit, AVAIL_TOPIC, s.uid, DEVICE_ID, FW_VERSION);
    pubcfg("sensor", s.name, buf);
  }
  Sen sens_hz[] = {
    {"flow_tank_hz","/eletechsup/flow1_hz","esp32_water_flow_tank_hz","Hz"},
    {"flow_house_hz","/eletechsup/flow2_hz","esp32_water_flow_house_hz","Hz"}
  };
  for(auto &s: sens_hz){
    snprintf(buf, sizeof(buf),
      "{\"name\":\"%s\",\"stat_t\":\"%s\",\"unit_of_meas\":\"%s\",\"avty_t\":\"%s\",\"uniq_id\":\"%s\",\"state_class\":\"measurement\",\"device\":{\"ids\":[\"%s\"],\"name\":\"ES32D26\",\"mf\":\"eletechsup\",\"mdl\":\"ES32D26\",\"sw\":\"%s\"}}",
      s.name, s.st, s.unit, AVAIL_TOPIC, s.uid, DEVICE_ID, FW_VERSION);
    pubcfg("sensor", s.name, buf);
  }
}

// ------------ Globals ------------
WiFiClient net;
PubSubClient mqtt(net);
unsigned long lastSampleMs = 0;
bool timeSynced = false;
unsigned long timerStartMs = 0; // 0 means not started
bool mqttSubscribed = false;    // subscribe only once with persistent session
bool flowDiscoveryPublished = false; // publish HA discovery for flow sensors once per session
bool ha_announced = false;      // HA discovery published once per connection
bool avail_online = false;      // availability online flag

static int readAvgMilliVolts(int pin, int samples=8){ long acc=0; for(int i=0;i<samples;i++){ acc += analogReadMilliVolts(pin); delay(2);} return (int)(acc/samples); }

// Batch-read ADC2 inputs (Vi1, Vi3) during a single WiFi-off window
static void readADC2Pair(){
  // Turn WiFi off once
  // Ensure MQTT is cleanly disconnected before tearing down Wi‑Fi to avoid lwIP pbuf issues
  if(mqtt.connected()){
    mqtt.disconnect();
    mqttSubscribed = false;
    ha_announced = false;
    avail_online = false;
    delay(20);
  }
  WiFi.disconnect(true,true); WiFi.mode(WIFI_OFF); delay(120);
  // Read ADC2 channels
  vi_mV[0] = readAvgMilliVolts(VI1_PIN, 8);
  vi_mV[2] = readAvgMilliVolts(VI3_PIN, 8);
  // Reconnect WiFi once
  WiFi.mode(WIFI_STA);
  WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS1);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-t<8000){ delay(80);} 
  digitalWrite(LED_WIFI, WiFi.isConnected()?HIGH:LOW);
  // Re-bind UDP socket after radio cycle
  ddUdp.stop(); ddUdp.begin(0);
}

void ensureWiFi(){
  if(WiFi.status()==WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS1);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-start<10000){ delay(150);} 
  digitalWrite(LED_WIFI, WiFi.isConnected()?HIGH:LOW);
  ddUdp.stop(); ddUdp.begin(0); // ensure UDP ready (legacy)
}

// Utilities for HTTPS metrics
static void ensureTime(){
  if(timeSynced) return;
  configTime(0,0,"pool.ntp.org","time.nist.gov");
  for(int i=0;i<50;i++){ if(time(nullptr) > 1600000000){ timeSynced=true; break; } delay(100); }
}

static bool postDatadogSeries(const String& json){
  ensureTime();
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  String url = String(DD_API_HOST) + String(DD_SERIES_PATH);
  if(!http.begin(client, url)) return false;
  http.addHeader("Content-Type","application/json");
  http.addHeader("DD-API-KEY", DD_API_KEY);
  int code = http.POST(json);
  String resp = http.getString();
  http.end();
  Serial.printf("DD HTTP %d %s\n", code, resp.c_str());
  return (code>=200 && code<300);
}

// Build a JSON tags array string from fixed and optional tags, skipping empties
static String buildTagsArray(const char* channelTag, const char* locationTag, bool includeUnitPsi){
  String out;
  auto appendKV = [&](const char* kv){
    if(kv && *kv){ if(out.length()>0) out += ","; out += "\""; out += kv; out += "\""; }
  };
  // expand DD_COMMON_TAGS (comma-separated) into individual entries
  {
    String common = String(DD_COMMON_TAGS);
    int start = 0;
    while(start < common.length()){
      int comma = common.indexOf(',', start);
      String part = (comma == -1) ? common.substring(start) : common.substring(start, comma);
      part.trim();
      if(part.length()>0){ appendKV(part.c_str()); }
      if(comma == -1) break; else start = comma + 1;
    }
  }
  // dynamic tags
  if(channelTag && *channelTag){ String kv = String("channel:") + channelTag; appendKV(kv.c_str()); }
  if(locationTag && *locationTag){ String kv = String("location:") + locationTag; appendKV(kv.c_str()); }
  if(includeUnitPsi){ appendKV("unit:psi"); }
  return out;
}

void ddMetricGaugePsi(const char* channelTag, const char* locationTag, float psi){
  unsigned long ts = (unsigned long)time(nullptr);
  String tags = buildTagsArray(channelTag, locationTag, true);
  char body[640];
  int n = snprintf(body, sizeof(body),
    "{\"series\":[{\"metric\":\"iot.pressure\",\"type\":0,\"points\":[[%lu,%0.3f]],\"tags\":[%s]}]}",
    ts, psi, tags.c_str());
  postDatadogSeries(String(body));
}

// minimal series without location
void ddMetricGaugePsiMinimal(const char* channelTag, float psi){
  unsigned long ts = (unsigned long)time(nullptr);
  String tags = buildTagsArray(channelTag, nullptr, true);
  char body[640];
  int n = snprintf(body, sizeof(body),
    "{\"series\":[{\"metric\":\"iot.pressure\",\"type\":0,\"points\":[[%lu,%0.3f]],\"tags\":[%s]}]}",
    ts, psi, tags.c_str());
  postDatadogSeries(String(body));
}

void ddMetricGaugeNoTags(float psi){
  unsigned long ts = (unsigned long)time(nullptr);
  char body[256];
  int n = snprintf(body, sizeof(body),
    "{\"series\":[{\"metric\":\"iot.pressure\",\"type\":0,\"points\":[[%lu,%0.3f]]}]}",
    ts, psi);
  postDatadogSeries(String(body));
}

void ddEvent(const char* title, const char* channelTag){
  // DogStatsD event: _e{title_len,text_len}:title|text|#tags
  const char* text = "relay_state_change";
  char buf[300];
  int n = snprintf(buf, sizeof(buf), "_e{%d,%d}:%s|%s|#%s,channel:%s\n", (int)strlen(title), (int)strlen(text), title, text, DD_COMMON_TAGS, channelTag);
  ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); ddUdp.endPacket();
}

// DogStatsD gauge emitters for iot.pressure
static void ddMetricGaugePsiDog(const char* channelTag, const char* locationTag, float psi){
  char buf[256];
  if(locationTag && *locationTag){
    int n = snprintf(buf, sizeof(buf), "iot.pressure:%0.3f|g|#%s,channel:%s,location:%s,unit:psi\n", psi, DD_COMMON_TAGS, channelTag, locationTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); int rc = ddUdp.endPacket(); if(rc==1) dd_udp_ok++; else dd_udp_err++;
  } else {
    int n = snprintf(buf, sizeof(buf), "iot.pressure:%0.3f|g|#%s,channel:%s,unit:psi\n", psi, DD_COMMON_TAGS, channelTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); int rc = ddUdp.endPacket(); if(rc==1) dd_udp_ok++; else dd_udp_err++;
  }
}

// DogStatsD gauge emitters for iot.voltage (terminal volts)
static void ddMetricGaugeVoltsDog(const char* channelTag, const char* locationTag, float volts){
  char buf[256];
  if(locationTag && *locationTag){
    int n = snprintf(buf, sizeof(buf), "iot.voltage:%0.3f|g|#%s,channel:%s,location:%s,unit:v\n", volts, DD_COMMON_TAGS, channelTag, locationTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); int rc = ddUdp.endPacket(); if(rc==1) dd_udp_ok++; else dd_udp_err++;
  } else {
    int n = snprintf(buf, sizeof(buf), "iot.voltage:%0.3f|g|#%s,channel:%s,unit:v\n", volts, DD_COMMON_TAGS, channelTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); int rc = ddUdp.endPacket(); if(rc==1) dd_udp_ok++; else dd_udp_err++;
  }
}

void ensureMqtt(){
  if(mqtt.connected()) return;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  String clientId = String("esp32_water-") + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqtt.setBufferSize(512);
  mqtt.setCallback([](char* topic, byte* payload, unsigned int len){
    String t(topic);
    String p; p.reserve(len); for(unsigned int i=0;i<len;i++) p += (char)payload[i];
    p.trim();
    auto handle = [&](int ch, const char* name){
      // Accept exactly '1' or '0'
      if(p == "1"){ setRelay(ch, true); Serial.printf("MQTT %s -> ON\n", name); ddEvent((String("ch")+ch+" energized").c_str(), name); }
      else if(p == "0"){ setRelay(ch, false); Serial.printf("MQTT %s -> OFF\n", name); ddEvent((String("ch")+ch+" deenergized").c_str(), name); }
      if(timerStartMs==0) timerStartMs = millis();
      // Publish state topic for HA
      String st = String("/eletechsup/") + name + "/state";
      mqtt.publish(st.c_str(), (p=="1")?"1":"0", true);
    };
    if(t.endsWith("prefilter"))   handle(1, "prefilter");
    else if(t.endsWith("postfilter")) handle(2, "postfilter");
    else if(t.endsWith("500"))    handle(3, "500");
    else if(t.endsWith("250"))    handle(4, "250");
    else if(t.endsWith("100"))    handle(5, "100");
    else if(t.endsWith("50"))     handle(6, "50");
    else if(t.endsWith("pump"))   handle(7, "pump");
  });
  // persistent session: cleanSession=false so existing subs are preserved
  if(mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, nullptr, 0, false, nullptr, false)){
    digitalWrite(LED_MQTT, HIGH);
    if(!avail_online){ mqtt.publish(AVAIL_TOPIC, "online", true); avail_online = true; }
    if(!mqttSubscribed){
      for(const char* s: topics_slash) mqtt.subscribe(s, 1);
      for(const char* s: topics)      mqtt.subscribe(s, 1);
      mqttSubscribed = true;
    }
    if(!ha_announced){ publishHADiscovery(); ha_announced = true; }
    publishAllRelayStates();
    // Publish HA discovery for flow sensors (retained) once per session
    if(!flowDiscoveryPublished){
      const char* tank_cfg =
        "{\"name\":\"flow_tank_lpm\",\"stat_t\":\"/eletechsup/flow1_lpm\",\"unit_of_meas\":\"L/min\",\"avty_t\":\"esp32_water/status\",\"uniq_id\":\"esp32_water_flow_tank_lpm\",\"state_class\":\"measurement\",\"device\":{\"ids\":[\"esp32_water\"],\"name\":\"ES32D26\",\"mf\":\"eletechsup\",\"mdl\":\"ES32D26\"}}";
      const char* house_cfg =
        "{\"name\":\"flow_house_lpm\",\"stat_t\":\"/eletechsup/flow2_lpm\",\"unit_of_meas\":\"L/min\",\"avty_t\":\"esp32_water/status\",\"uniq_id\":\"esp32_water_flow_house_lpm\",\"state_class\":\"measurement\",\"device\":{\"ids\":[\"esp32_water\"],\"name\":\"ES32D26\",\"mf\":\"eletechsup\",\"mdl\":\"ES32D26\"}}";
      mqtt.publish("homeassistant/sensor/esp32_water/flow_tank_lpm/config", tank_cfg, true);
      mqtt.publish("homeassistant/sensor/esp32_water/flow_house_lpm/config", house_cfg, true);
      flowDiscoveryPublished = true;
    }
  } else {
    digitalWrite(LED_MQTT, LOW);
    avail_online = false; // don't publish while disconnected
  }
}

static float mvToPsi(int mv, int idx){
  float adcVolts = mv / 1000.0f;
  float terminalVolts = adcVolts * ADC_TO_TERMINAL_GAIN;
  return (terminalVolts / SENSOR_FS_VOLTS) * PSI_FS[idx];
}

void sampleAndPublish(){
  // Read ADC1 while WiFi is up
  vi_mV[1] = readAvgMilliVolts(VI2_PIN, 8);
  vi_mV[3] = readAvgMilliVolts(VI4_PIN, 8);
  // Read ADC2 in a single WiFi-off window, then reconnect once
  readADC2Pair();
  // Allow connection to settle and ensure MQTT is connected before publishing
  unsigned long settleStart = millis();
  while(millis() - settleStart < 1200){
    ensureMqtt();
    if(mqtt.connected()) mqtt.loop(); else delay(50);
    delay(50);
  }

  // Publish terminal voltages per input to MQTT (retained)
  if(mqtt.connected()){
    auto publishVolts = [&](const char* chan, int mv){
      float adcVolts = mv / 1000.0f;
      float terminalVolts = adcVolts * ADC_TO_TERMINAL_GAIN;
      char payload[24];
      snprintf(payload, sizeof(payload), "%0.3f", terminalVolts);
      String topic = String(INPUT_BASE_TOPIC) + "/" + chan; // e.g., /eletechsup/inputs/vi1
      mqtt.publish(topic.c_str(), payload, true);
    };
    publishVolts("vi1", vi_mV[0]);
    publishVolts("vi2", vi_mV[1]);
    publishVolts("vi3", vi_mV[2]);
    publishVolts("vi4", vi_mV[3]);
    // Also publish flow cached values to MQTT
    char buf[24];
    snprintf(buf, sizeof(buf), "%0.3f", flow1_lpm_last); mqtt.publish("/eletechsup/flow1_lpm", buf, true); // tank
    snprintf(buf, sizeof(buf), "%0.3f", flow2_lpm_last); mqtt.publish("/eletechsup/flow2_lpm", buf, true); // house
    snprintf(buf, sizeof(buf), "%0.1f", flow1_hz_last);  mqtt.publish("/eletechsup/flow1_hz", buf, true);
    snprintf(buf, sizeof(buf), "%0.1f", flow2_hz_last);  mqtt.publish("/eletechsup/flow2_hz", buf, true);
  }

  // Metrics via DogStatsD (UDP): pressure and voltage (BATCHEd into ONE packet)
  float v1 = (vi_mV[0] / 1000.0f) * ADC_TO_TERMINAL_GAIN;
  float v2 = (vi_mV[1] / 1000.0f) * ADC_TO_TERMINAL_GAIN;
  float v3 = (vi_mV[2] / 1000.0f) * ADC_TO_TERMINAL_GAIN;
  float v4 = (vi_mV[3] / 1000.0f) * ADC_TO_TERMINAL_GAIN;

  float p1 = mvToPsi(vi_mV[0],0);
  float p2 = mvToPsi(vi_mV[1],1);
  float p3 = mvToPsi(vi_mV[2],2);
  float p4 = mvToPsi(vi_mV[3],3);

  // Per-metric packets to guarantee visibility and avoid truncation
  auto send_line = [&](const char* fmt, ...){
    char line[200];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if(n > 0){ ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)line, (size_t)n); int rc=ddUdp.endPacket(); if(rc==1) dd_udp_ok++; else dd_udp_err++; Serial.print("DD: "); Serial.print(line); }
  };
  send_line("iot.pressure:%0.3f|g|#%s,channel:vi1,location:tank,unit:psi\n", p1, DD_COMMON_TAGS);
  send_line("iot.pressure:%0.3f|g|#%s,channel:vi2,location:house,unit:psi\n", p2, DD_COMMON_TAGS);
  send_line("iot.pressure:%0.3f|g|#%s,channel:vi3,location:prefilter,unit:psi\n", p3, DD_COMMON_TAGS);
  send_line("iot.pressure:%0.3f|g|#%s,channel:vi4,location:postfilter,unit:psi\n", p4, DD_COMMON_TAGS);
  send_line("iot.voltage:%0.3f|g|#%s,channel:vi1,location:tank,unit:v\n", v1, DD_COMMON_TAGS);
  send_line("iot.voltage:%0.3f|g|#%s,channel:vi2,location:house,unit:v\n", v2, DD_COMMON_TAGS);
  send_line("iot.voltage:%0.3f|g|#%s,channel:vi3,location:prefilter,unit:v\n", v3, DD_COMMON_TAGS);
  send_line("iot.voltage:%0.3f|g|#%s,channel:vi4,location:postfilter,unit:v\n", v4, DD_COMMON_TAGS);

  // Add flow metrics (Flow1=tank, Flow2=house)
  #if FLOW_METRICS_ENABLED
  {
    auto send_line = [&](const char* fmt, ...){
      char line[160];
      va_list ap; va_start(ap, fmt);
      int n = vsnprintf(line, sizeof(line), fmt, ap);
      va_end(ap);
      if(n > 0){ ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)line, (size_t)n); int rc=ddUdp.endPacket(); if(rc==1) dd_udp_ok++; else dd_udp_err++; Serial.print("DD: "); Serial.print(line); }
    };
    // Flow1=tank, Flow2=house
    send_line("iot.flow:%0.3f|g|#%s,channel:tank,unit:l_per_min\n", flow1_lpm_last, DD_COMMON_TAGS);
    send_line("iot.flow_hz:%0.1f|g|#%s,channel:tank,unit:hz\n", flow1_hz_last, DD_COMMON_TAGS);
    send_line("iot.flow:%0.3f|g|#%s,channel:house,unit:l_per_min\n", flow2_lpm_last, DD_COMMON_TAGS);
    send_line("iot.flow_hz:%0.1f|g|#%s,channel:house,unit:hz\n", flow2_hz_last, DD_COMMON_TAGS);
  }
  #endif

  Serial.printf("PSI vi1:%0.2f vi2:%0.2f vi3:%0.2f vi4:%0.2f | V vi1:%0.3f vi2:%0.3f vi3:%0.3f vi4:%0.3f\n",
    p1, p2, p3, p4, v1, v2, v3, v4);
  Serial.printf("Flow tank LPM:%0.3f Hz:%0.1f | house LPM:%0.3f Hz:%0.1f\n", flow1_lpm_last, flow1_hz_last, flow2_lpm_last, flow2_hz_last);
  Serial.printf("DD UDP ok:%lu err:%lu host:%s\n", (unsigned long)dd_udp_ok, (unsigned long)dd_udp_err, DD_HOST);
}

// Publish flow meter metrics approximately every second while MQTT is connected
static void publishFlowIfDue(){
  unsigned long now = millis();
  if(now - lastFlowPublishMs < 1000) return;
  lastFlowPublishMs = now;
  // Atomically read and reset pulse counters
  uint32_t p1, p2;
  noInterrupts(); p1 = flow1_pulses; flow1_pulses = 0; p2 = flow2_pulses; flow2_pulses = 0; interrupts();
  // Convert pulses in last interval to frequency (Hz) and flow (L/min)
  float seconds = (now % 1000 == 0) ? 1.0f : ( (now - (now - 1000)) / 1000.0f ); // ~1s
  float f1_hz = p1 / seconds;
  float f2_hz = p2 / seconds;
  float f1_lpm = (p1 / FLOW_K_PPL) * 60.0f / seconds;
  float f2_lpm = (p2 / FLOW_K_PPL) * 60.0f / seconds;
  flow1_hz_last = f1_hz; flow2_hz_last = f2_hz; flow1_lpm_last = f1_lpm; flow2_lpm_last = f2_lpm;
  if(mqtt.connected()){
    char buf[32];
    snprintf(buf, sizeof(buf), "%0.3f", f1_lpm); mqtt.publish("/eletechsup/flow1_lpm", buf, true);
    snprintf(buf, sizeof(buf), "%0.3f", f2_lpm); mqtt.publish("/eletechsup/flow2_lpm", buf, true);
    snprintf(buf, sizeof(buf), "%0.1f", f1_hz);  mqtt.publish("/eletechsup/flow1_hz", buf, true);
    snprintf(buf, sizeof(buf), "%0.1f", f2_hz);  mqtt.publish("/eletechsup/flow2_hz", buf, true);
  }
}

void setup(){
  Serial.begin(115200); delay(150);
  pinMode(LED_WIFI, OUTPUT); pinMode(LED_MQTT, OUTPUT); digitalWrite(LED_WIFI, LOW); digitalWrite(LED_MQTT, LOW);
  // Relays
  pinMode(SR_DATA, OUTPUT); pinMode(SR_CLK, OUTPUT); pinMode(SR_LATCH, OUTPUT); pinMode(SR_OE, OUTPUT);
  digitalWrite(SR_DATA, LOW); digitalWrite(SR_CLK, LOW); digitalWrite(SR_LATCH, HIGH); digitalWrite(SR_OE, LOW);
  srWrite(srState);
  // Hostname
  WiFi.setHostname("esp32_water");
  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(VI1_PIN, ADC_11db);
  analogSetPinAttenuation(VI2_PIN, ADC_11db);
  analogSetPinAttenuation(VI3_PIN, ADC_11db);
  analogSetPinAttenuation(VI4_PIN, ADC_11db);
  // Flow meters
  pinMode(FLOW1_PIN, INPUT_PULLUP);
  pinMode(FLOW2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW1_PIN), flow1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW2_PIN), flow2_isr, RISING);
  // Net
  ensureWiFi();
  // Stagger MQTT connect a bit to let Wi-Fi settle
  delay(300);
  ensureMqtt();
  ddUdp.begin(0);
  // Prime metrics
  sampleAndPublish();
}

void loop(){
  ensureWiFi(); ensureMqtt(); if(mqtt.connected()) mqtt.loop();
  unsigned long now = millis();
  if(now - lastSampleMs >= 15000){ lastSampleMs = now; sampleAndPublish(); }
  if(timerStartMs && (now - timerStartMs >= 300000UL)){
    // Auto de-energize all
    const char* chTag[6] = {"prefilter","postfilter","500","250","100","50"};
    for(int ch=1; ch<=6; ++ch){ bool wasOn = ((srState >> relayBitForChannel(ch)) & 1); setRelay(ch, false); if(wasOn) ddEvent((String("ch")+ch+" deenergized").c_str(), chTag[ch-1]); }
    timerStartMs = 0; // stop timer until next command
  }
  // LWT availability heartbeat
  static unsigned long lastAvail = 0; if(now - lastAvail > 30000UL){ lastAvail = now; if(mqtt.connected()) mqtt.publish(AVAIL_TOPIC, "online", true); }
  publishFlowIfDue();
}
