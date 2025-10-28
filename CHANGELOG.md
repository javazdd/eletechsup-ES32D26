## 1.0.0 - 2025-10-28
- ESPHome support added (`esphome/es32d26.yaml`) with OTA, HA API, flow meters, DogStatsD UDP, and SN74HC595 mapping
- Finalized channel labels (ESPHome): ch1=50, ch2=100, ch3=250, ch4=500, ch5=Postfilter, ch6=Prefilter, ch7=Pump, ch8=UV
- Removed auto-shutoff behavior from ESPHome path to avoid HA interference
- Added HA-editable settings for MQTT and DogStatsD (ports integer only)
- README updated with precise mapping and flashing instructions

## 0.2.0 - 2025-10-12
- Add two flow meters on IO18 (tank) and IO19 (house), publish LPM and Hz
- Add Home Assistant MQTT Discovery for flow sensors and pump switch
- Add peristaltic pump on channel 7 with unique shift-register bit mapping
- Publish pump retained state and discovery; ensure HA availability topic
- Fix MQTT/Wi‑Fi stability: static IP reconnects, guard mqtt.loop(), clean disconnect before Wi‑Fi off
- DogStatsD: send per‑metric packets for pressure/voltage/flow; add serial debug lines
- Hostname set to `esp32_water`
- README updated for pump/flow and networking

## 0.1.0 - 2025-10-10
- Initial ESP32 firmware for ES32D26: relays 1..6 over MQTT, Vi1..Vi4 sampling, DogStatsD pressure metrics
