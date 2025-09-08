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
#include "secrets.h"  // defines WIFI_SSID, WIFI_PASS, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, DD_API_KEY

// ------------ Network (static IP) ------------
const char* WIFI_SSID = WIFI_SSID_SECRET;
const char* WIFI_PASS = WIFI_PASS_SECRET;
IPAddress STATIC_IP(192,168,88,206);
IPAddress GATEWAY(192,168,88,1);
IPAddress SUBNET(255,255,255,0);
IPAddress DNS1(8,8,8,8);

// ------------ MQTT ------------
const char* MQTT_HOST = MQTT_HOST_SECRET;
const uint16_t MQTT_PORT = MQTT_PORT_SECRET;
const char* MQTT_USER = MQTT_USER_SECRET;
const char* MQTT_PASS = MQTT_PASS_SECRET;

// Topics (subscribe to both with and without leading slash for safety)
const char* topics_slash[] = {
  "/eletechsup/prefilter","/eletechsup/postfilter","/eletechsup/500","/eletechsup/250","/eletechsup/100","/eletechsup/50"
};
const char* topics[] = {
  "eletechsup/prefilter","eletechsup/postfilter","eletechsup/500","eletechsup/250","eletechsup/100","eletechsup/50"
};
// channel index map (0-based relay number)
// ch1=prefilter, ch2=postfilter, ch3=500, ch4=250, ch5=100, ch6=50

// ------------ DogStatsD ------------
// DogStatsD disabled; using HTTPS metrics intake instead
IPAddress DD_HOST(192,168,88,205); // retained for compatibility (unused)
const uint16_t DD_PORT = 8125;     // unused
WiFiUDP ddUdp;                     // unused

// Datadog HTTPS intake (US1 site)
static const char* DD_API_HOST = "https://api.datadoghq.com";
static const char* DD_SERIES_PATH = "/api/v1/series";
static const char* DD_API_KEY = DD_API_KEY_SECRET;

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
uint8_t srState = 0x00; // bits 0..7 => ch1..ch8 (1 = ON)

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
  if(on) srState |=  (1 << (ch-1)); else srState &= ~(1 << (ch-1));
  if(srState != before){ srWrite(srState); }
}

// ------------ Globals ------------
WiFiClient net;
PubSubClient mqtt(net);
unsigned long lastADC1Ms = 0;
unsigned long lastADC2Ms = 0;
bool timeSynced = false;
unsigned long timerStartMs = 0; // 0 means not started
unsigned long lastCommandMs = 0; // suppress ADC2 windows briefly after commands

static int readAvgMilliVolts(int pin, int samples=8){ long acc=0; for(int i=0;i<samples;i++){ acc += analogReadMilliVolts(pin); delay(2);} return (int)(acc/samples); }

int readADC2MilliVolts(int pin){
  WiFi.disconnect(true,true); WiFi.mode(WIFI_OFF); delay(120);
  int mv = readAvgMilliVolts(pin, 8);
  WiFi.mode(WIFI_STA); WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS1); WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-t<6000){ delay(80);} digitalWrite(LED_WIFI, WiFi.isConnected()?HIGH:LOW);
  // re-bind UDP socket after radio cycle
  ddUdp.stop(); ddUdp.begin(0);
  return mv;
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
  // v1 intake uses api_key as query parameter
  String url = String(DD_API_HOST) + String(DD_SERIES_PATH) + String("?api_key=") + String(DD_API_KEY);
  if(!http.begin(client, url)) return false;
  http.addHeader("Content-Type","application/json");
  int code = http.POST(json);
  String resp = http.getString();
  http.end();
  Serial.printf("DD HTTP %d %s\n", code, resp.c_str());
  return (code>=200 && code<300);
}

void ddMetricGaugePsi(const char* channelTag, const char* locationTag, float psi){
  unsigned long ts = (unsigned long)time(nullptr);
  char body[512];
  // Build one-series payload
  int n = snprintf(body, sizeof(body),
    "{\"series\":[{\"metric\":\"iot.pressure\",\"type\":\"gauge\",\"points\":[[%lu,%0.3f]],\"tags\":[\"%s\",\"channel:%s\",\"location:%s\",\"unit:psi\"]}]}",
    ts, psi, DD_COMMON_TAGS, channelTag, locationTag);
  postDatadogSeries(String(body));
}

// minimal series without location
void ddMetricGaugePsiMinimal(const char* channelTag, float psi){
  unsigned long ts = (unsigned long)time(nullptr);
  char body[512];
  int n = snprintf(body, sizeof(body),
    "{\"series\":[{\"metric\":\"iot.pressure\",\"type\":\"gauge\",\"points\":[[%lu,%0.3f]],\"tags\":[\"%s\",\"channel:%s\",\"unit:psi\"]}]}",
    ts, psi, DD_COMMON_TAGS, channelTag);
  postDatadogSeries(String(body));
}

void ddMetricGaugeNoTags(float psi){
  unsigned long ts = (unsigned long)time(nullptr);
  char body[256];
  int n = snprintf(body, sizeof(body),
    "{\"series\":[{\"metric\":\"iot.pressure\",\"type\":\"gauge\",\"points\":[[%lu,%0.3f]]}]}",
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
      if(p == "1"){ setRelay(ch, true); lastCommandMs = millis(); Serial.printf("MQTT %s -> ON\n", name); ddEvent((String("ch")+ch+" energized").c_str(), name); }
      else if(p == "0"){ setRelay(ch, false); lastCommandMs = millis(); Serial.printf("MQTT %s -> OFF\n", name); ddEvent((String("ch")+ch+" deenergized").c_str(), name); }
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
  // Build a single JSON with up to four series to minimize blocking
  unsigned long ts = (unsigned long)time(nullptr);
  // Update ADC1 now
  vi_mV[1] = readAvgMilliVolts(VI2_PIN, 8);
  vi_mV[3] = readAvgMilliVolts(VI4_PIN, 8);
  // Optionally update ADC2 if not within suppression window
  if(millis() - lastCommandMs > 5000) {
    vi_mV[0] = readADC2MilliVolts(VI1_PIN);
    vi_mV[2] = readADC2MilliVolts(VI3_PIN);
  }
  float psi[4] = { mvToPsi(vi_mV[0],0), mvToPsi(vi_mV[1],1), mvToPsi(vi_mV[2],2), mvToPsi(vi_mV[3],3) };
  String json = "{\"series\":[";
  auto appendSeries = [&](const char* channel, const char* tagval, float val){
    if(json.charAt(json.length()-1) != '[') json += ",";
    json += "{\"metric\":\"iot.pressure\",\"type\":\"gauge\",\"points\":[[";
    json += String(ts);
    json += ","; json += String(val,3);
    json += "]],\"tags\":[\""; json += DD_COMMON_TAGS; json += "\",\"channel:"; json += channel; json += "\",\"tag:"; json += tagval; json += "\",\"unit:psi\"]}";
  };
  appendSeries("Vi1","tank",psi[0]);
  appendSeries("Vi2","house",psi[1]);
  appendSeries("Vi3","prefilter",psi[2]);
  appendSeries("Vi4","postfilter",psi[3]);
  json += "]}";
  postDatadogSeries(json);
  Serial.printf("PSI vi1:%0.2f vi2:%0.2f vi3:%0.2f vi4:%0.2f\n", psi[0], psi[1], psi[2], psi[3]);
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
  // Prime metrics (without ADC2 window delay)
  lastCommandMs = millis(); // suppress ADC2 for 5s at boot
  sampleAndPublish();
}

void loop(){
  ensureWiFi(); ensureMqtt(); mqtt.loop();
  unsigned long now = millis();
  if(now - lastADC1Ms >= 15000){ lastADC1Ms = now; sampleAndPublish(); }
  if(timerStartMs && (now - timerStartMs >= 300000UL)){
    // Auto de-energize all
    const char* chTag[6] = {"prefilter","postfilter","500","250","100","50"};
    for(int ch=1; ch<=6; ++ch){ bool wasOn = (srState >> (ch-1)) & 1; setRelay(ch, false); if(wasOn) ddEvent((String("ch")+ch+" deenergized").c_str(), chTag[ch-1]); }
    timerStartMs = 0; // stop timer until next command
  }
}
