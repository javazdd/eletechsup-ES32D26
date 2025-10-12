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
