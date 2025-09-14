// WiFi+MQTT+DogStatsD + Relay control (74HC595) for ES32D26
// - Static IP
// - Samples Vi1..Vi4 every 15s (brief WiFi-off windows for ADC2)
// - Publishes iot.pressure (PSI) gauges via DogStatsD with tags
// - Subscribes to relay topics and energizes/de-energizes relays 1..6
// - Starts a 300s master timer on first command; after 300s all relays turn off

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

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
  "/eletechsup/prefilter","/eletechsup/postfilter","/eletechsup/500","/eletechsup/250","/eletechsup/100","/eletechsup/50"
};
const char* topics[] = {
  "eletechsup/prefilter","eletechsup/postfilter","eletechsup/500","eletechsup/250","eletechsup/100","eletechsup/50"
};
// Base topic to publish input terminal voltages
const char* INPUT_BASE_TOPIC = "/eletechsup/inputs";
// channel index map (0-based relay number)
// ch1=prefilter, ch2=postfilter, ch3=500, ch4=250, ch5=100, ch6=50

// ------------ DogStatsD ------------
// DogStatsD disabled; using HTTPS metrics intake instead
IPAddress DD_HOST(192,168,88,205); // retained for compatibility (unused)
const uint16_t DD_PORT = 8125;     // unused
WiFiUDP ddUdp;                     // unused

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

// ------------ Relays via 74HC595 + ULN2803 ------------
const int SR_DATA  = 12;
const int SR_CLK   = 22;
const int SR_LATCH = 23;
const int SR_OE    = 13; // active LOW
uint8_t srState = 0x00; // bits 0..7 => outputs
// Map logical channels 1..6 to physical bit indices 2..7 (hardware wiring offset)
static const uint8_t RELAY_BIT_FOR_CHANNEL[6] = {2,3,4,5,6,7};

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
  if(ch>=1 && ch<=6){
    uint8_t bit = RELAY_BIT_FOR_CHANNEL[ch-1];
    if(on) srState |=  (1 << bit); else srState &= ~(1 << bit);
  } else if(ch>=7 && ch<=8){
    // pass-through for 7..8
    uint8_t bit = (uint8_t)(ch-1);
    if(on) srState |=  (1 << bit); else srState &= ~(1 << bit);
  }
  if(srState != before){ srWrite(srState); }
}

// ------------ Globals ------------
WiFiClient net;
PubSubClient mqtt(net);
unsigned long lastSampleMs = 0;
bool timeSynced = false;
unsigned long timerStartMs = 0; // 0 means not started

static int readAvgMilliVolts(int pin, int samples=8){ long acc=0; for(int i=0;i<samples;i++){ acc += analogReadMilliVolts(pin); delay(2);} return (int)(acc/samples); }

// Batch-read ADC2 inputs (Vi1, Vi3) during a single WiFi-off window
static void readADC2Pair(){
  // Turn WiFi off once
  WiFi.disconnect(true,true); WiFi.mode(WIFI_OFF); delay(120);
  // Read ADC2 channels
  vi_mV[0] = readAvgMilliVolts(VI1_PIN, 8);
  vi_mV[2] = readAvgMilliVolts(VI3_PIN, 8);
  // Reconnect WiFi once
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-t<8000){ delay(80);} 
  digitalWrite(LED_WIFI, WiFi.isConnected()?HIGH:LOW);
  // Re-bind UDP socket after radio cycle
  ddUdp.stop(); ddUdp.begin(0);
}

void ensureWiFi(){
  if(WiFi.status()==WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
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
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); ddUdp.endPacket();
  } else {
    int n = snprintf(buf, sizeof(buf), "iot.pressure:%0.3f|g|#%s,channel:%s,unit:psi\n", psi, DD_COMMON_TAGS, channelTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); ddUdp.endPacket();
  }
}

// DogStatsD gauge emitters for iot.voltage (terminal volts)
static void ddMetricGaugeVoltsDog(const char* channelTag, const char* locationTag, float volts){
  char buf[256];
  if(locationTag && *locationTag){
    int n = snprintf(buf, sizeof(buf), "iot.voltage:%0.3f|g|#%s,channel:%s,location:%s,unit:v\n", volts, DD_COMMON_TAGS, channelTag, locationTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); ddUdp.endPacket();
  } else {
    int n = snprintf(buf, sizeof(buf), "iot.voltage:%0.3f|g|#%s,channel:%s,unit:v\n", volts, DD_COMMON_TAGS, channelTag);
    ddUdp.beginPacket(DD_HOST, DD_PORT); ddUdp.write((uint8_t*)buf, n); ddUdp.endPacket();
  }
}

void ensureMqtt(){
  if(mqtt.connected()) return;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  String clientId = String("es32d26-") + String((uint32_t)ESP.getEfuseMac(), HEX);
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
    };
    if(t.endsWith("prefilter"))   handle(1, "prefilter");
    else if(t.endsWith("postfilter")) handle(2, "postfilter");
    else if(t.endsWith("500"))    handle(3, "500");
    else if(t.endsWith("250"))    handle(4, "250");
    else if(t.endsWith("100"))    handle(5, "100");
    else if(t.endsWith("50"))     handle(6, "50");
  });
  // persistent session to receive retained msgs during reconnects
  if(mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, nullptr, 0, false, nullptr, true)){
    digitalWrite(LED_MQTT, HIGH);
    for(const char* s: topics_slash) mqtt.subscribe(s, 1);
    for(const char* s: topics)      mqtt.subscribe(s, 1);
  } else {
    digitalWrite(LED_MQTT, LOW);
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
    mqtt.loop();
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
  }

  // Metrics via DogStatsD (UDP): pressure and voltage
  float v1 = (vi_mV[0] / 1000.0f) * ADC_TO_TERMINAL_GAIN;
  float v2 = (vi_mV[1] / 1000.0f) * ADC_TO_TERMINAL_GAIN;
  float v3 = (vi_mV[2] / 1000.0f) * ADC_TO_TERMINAL_GAIN;
  float v4 = (vi_mV[3] / 1000.0f) * ADC_TO_TERMINAL_GAIN;

  ddMetricGaugePsiDog("vi1", "tank",      mvToPsi(vi_mV[0],0));
  ddMetricGaugePsiDog("vi2", "house",     mvToPsi(vi_mV[1],1));
  ddMetricGaugePsiDog("vi3", "prefilter", mvToPsi(vi_mV[2],2));
  ddMetricGaugePsiDog("vi4", "postfilter", mvToPsi(vi_mV[3],3));

  ddMetricGaugeVoltsDog("vi1", "tank",      v1);
  ddMetricGaugeVoltsDog("vi2", "house",     v2);
  ddMetricGaugeVoltsDog("vi3", "prefilter", v3);
  ddMetricGaugeVoltsDog("vi4", "postfilter", v4);

  Serial.printf("PSI vi1:%0.2f vi2:%0.2f vi3:%0.2f vi4:%0.2f | V vi1:%0.3f vi2:%0.3f vi3:%0.3f vi4:%0.3f\n",
    mvToPsi(vi_mV[0],0), mvToPsi(vi_mV[1],1), mvToPsi(vi_mV[2],2), mvToPsi(vi_mV[3],3), v1, v2, v3, v4);
}

void setup(){
  Serial.begin(115200); delay(150);
  pinMode(LED_WIFI, OUTPUT); pinMode(LED_MQTT, OUTPUT); digitalWrite(LED_WIFI, LOW); digitalWrite(LED_MQTT, LOW);
  // Relays
  pinMode(SR_DATA, OUTPUT); pinMode(SR_CLK, OUTPUT); pinMode(SR_LATCH, OUTPUT); pinMode(SR_OE, OUTPUT);
  digitalWrite(SR_DATA, LOW); digitalWrite(SR_CLK, LOW); digitalWrite(SR_LATCH, HIGH); digitalWrite(SR_OE, LOW);
  srWrite(srState);
  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(VI1_PIN, ADC_11db);
  analogSetPinAttenuation(VI2_PIN, ADC_11db);
  analogSetPinAttenuation(VI3_PIN, ADC_11db);
  analogSetPinAttenuation(VI4_PIN, ADC_11db);
  // Net
  ensureWiFi(); ensureMqtt();
  ddUdp.begin(0);
  // Prime metrics
  sampleAndPublish();
}

void loop(){
  ensureWiFi(); ensureMqtt(); mqtt.loop();
  unsigned long now = millis();
  if(now - lastSampleMs >= 15000){ lastSampleMs = now; sampleAndPublish(); }
  if(timerStartMs && (now - timerStartMs >= 300000UL)){
    // Auto de-energize all
    const char* chTag[6] = {"prefilter","postfilter","500","250","100","50"};
    for(int ch=1; ch<=6; ++ch){ bool wasOn = (srState >> (ch-1)) & 1; setRelay(ch, false); if(wasOn) ddEvent((String("ch")+ch+" deenergized").c_str(), chTag[ch-1]); }
    timerStartMs = 0; // stop timer until next command
  }
}
