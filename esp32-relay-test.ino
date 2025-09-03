// Serial-only Vi1..Vi4 publisher (mV) for ES32D26 — labeled output only
#include <Arduino.h>
#include <WiFi.h>

// Final mapping:
// Vi1 -> GPIO14 (ADC2)
// Vi2 -> GPIO33 (ADC1)
// Vi3 -> GPIO27 (ADC2)
// Vi4 -> GPIO32 (ADC1)
const int VI1_PIN = 14; // ADC2
const int VI2_PIN = 33; // ADC1
const int VI3_PIN = 27; // ADC2
const int VI4_PIN = 32; // ADC1

static int readAvgMilliVolts(int pin, int samples=8){
  long acc=0; for(int i=0;i<samples;i++){ acc += analogReadMilliVolts(pin); delay(2);} return (int)(acc/samples);
}

void setup(){
  Serial.begin(115200);
  delay(150);
  // Ensure radio is off so ADC2 reads are reliable
  WiFi.mode(WIFI_OFF);
  analogReadResolution(12);
  analogSetPinAttenuation(VI1_PIN, ADC_11db);
  analogSetPinAttenuation(VI2_PIN, ADC_11db);
  analogSetPinAttenuation(VI3_PIN, ADC_11db);
  analogSetPinAttenuation(VI4_PIN, ADC_11db);
}

void loop(){
  int vi1 = readAvgMilliVolts(VI1_PIN, 8);
  int vi2 = readAvgMilliVolts(VI2_PIN, 8);
  int vi3 = readAvgMilliVolts(VI3_PIN, 8);
  int vi4 = readAvgMilliVolts(VI4_PIN, 8);

  Serial.print("Vi1: "); Serial.print(vi1); Serial.println(" mV");
  Serial.print("Vi2: "); Serial.print(vi2); Serial.println(" mV");
  Serial.print("Vi3: "); Serial.print(vi3); Serial.println(" mV");
  Serial.print("Vi4: "); Serial.print(vi4); Serial.println(" mV\n");
  delay(2000);
}
