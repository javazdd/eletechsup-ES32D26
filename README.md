# eletechsup-ES32D26 (ESP32 firmware)

Version: 1.0.0

Firmware for the Eletechsup 2AO-8AI-8DI-8DO board (ES32D26) using an ESP32-DevKitC.

## What it does
- Connects to Wi‑Fi and MQTT
- Subscribes to topics and energizes/de‑energizes relays based on MQTT payloads
- Samples 4 pressure transducers (Vi1..Vi4) every 15s
- Counts 2 flow meters (IO18=tank, IO19=house) and publishes L/min and Hz
- Controls a peristaltic pump on channel 7 (relay bit mapped uniquely)
- Publishes `iot.pressure` as DogStatsD metrics to a Datadog Agent with rich tags

## Channels and relay mapping
- Relays are driven via 74HC595 -> ULN2803A
- Shift register pins to ESP32: DATA=GPIO12, SRCLK=GPIO22, LATCH=GPIO23, OE(LOW)=GPIO13
- Channel labels (ESPHome):
  - ch1 → `50`
  - ch2 → `100`
  - ch3 → `250`
  - ch4 → `500`
  - ch5 → `Postfilter`
  - ch6 → `Prefilter`
  - ch7 → `Pump`
  - ch8 → `UV`

MQTT topics (Arduino firmware):
  - `/eletechsup/prefilter`, `/eletechsup/postfilter`, `/eletechsup/500`, `/eletechsup/250`, `/eletechsup/100`, `/eletechsup/50`, `/eletechsup/pump`

## MQTT control (relays)

## MQTT input voltages (subscribe)
- Host: <MQTT_HOST>
- Port: <MQTT_PORT>
- User: <MQTT_USER>
- Pass: <MQTT_PASS>

All inputs (retained shown immediately):
```bash
mosquitto_sub -h <MQTT_HOST> -p <MQTT_PORT> -u "<MQTT_USER>" -P "<MQTT_PASS>" -t "/eletechsup/inputs/#" -v
```

Live-only (ignore retained):
```bash
mosquitto_sub -R -h <MQTT_HOST> -p <MQTT_PORT> -u "<MQTT_USER>" -P "<MQTT_PASS>" -t "/eletechsup/inputs/#" -v
```

Single channel example (Vi4):
```bash
mosquitto_sub -h <MQTT_HOST> -p <MQTT_PORT> -u "<MQTT_USER>" -P "<MQTT_PASS>" -t "/eletechsup/inputs/vi4" -v
```


- Host: `<MQTT_HOST>` (e.g., 192.168.88.205)
- Port: `<MQTT_PORT>` (e.g., 1883)
- User: `<MQTT_USER>`
- Pass: `<MQTT_PASS>`
- QoS: 1 | Retained: yes | Payloads: `1` (ON), `0` (OFF)

Examples (mosquitto_pub):
```bash
HOST=<MQTT_HOST>; PORT=<MQTT_PORT>; USER="<MQTT_USER>"; PASS="<MQTT_PASS>"
# ON
for T in prefilter postfilter 500 250 100 50; do mosquitto_pub -h "$HOST" -p "$PORT" -u "$USER" -P "$PASS" -t "/eletechsup/$T" -m '1' -q 1 -r; done
# OFF
for T in prefilter postfilter 500 250 100 50; do mosquitto_pub -h "$HOST" -p "$PORT" -u "$USER" -P "$PASS" -t "/eletechsup/$T" -m '0' -q 1 -r; done
```

## Flow meters (MQTT + HA Discovery)
- Topics (retained):
  - Tank LPM: `/eletechsup/flow1_lpm`
  - House LPM: `/eletechsup/flow2_lpm`
  - Tank Hz: `/eletechsup/flow1_hz`
  - House Hz: `/eletechsup/flow2_hz`
- Home Assistant Discovery (retained):
  - `homeassistant/sensor/esp32_water/flow_tank_lpm/config`
  - `homeassistant/sensor/esp32_water/flow_house_lpm/config`
  - `homeassistant/sensor/esp32_water/flow_tank_hz/config`
  - `homeassistant/sensor/esp32_water/flow_house_hz/config`

## Metrics and tags (DogStatsD)
- UDP to Datadog Agent: host `<AGENT_HOSTNAME_OR_IP>`, port `8125`
- Metric: `iot.pressure` (gauge), one per input:
  - Vi1 → `channel:vi1`, `location:tank`
  - Vi2 → `channel:vi2`, `location:house`
  - Vi3 → `channel:vi3`, `location:prefilter`
  - Vi4 → `channel:vi4`, `location:postfilter`
- Common tags: `env:prod`, `sensor:transducer`, `source:eletechsup`, `service:water`, `unit:psi`

## Build & upload (Arduino CLI)
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 .
```

## Notes
- Ensure the Datadog Agent exposes DogStatsD on UDP 8125 and allows non‑local traffic.
- Keep credentials out of commits; use placeholders in code or untracked config.
- ADC pins: Vi1=GPIO14 (ADC2), Vi2=GPIO33 (ADC1), Vi3=GPIO27 (ADC2), Vi4=GPIO32 (ADC1)

## Networking
- Static IP: 192.168.88.206 (configured in firmware)
- Hostname: esp32_water (set via WiFi.setHostname)

## ESPHome (alternative firmware)
- Config file: `esphome/es32d26.yaml`
- Features:
  - Relays via SN74HC595 with mapping ch1..ch8 to Q0..Q7
  - Flow meters: IO18 (tank) and IO19 (house) LPM + derived Hz
  - Vi2 (GPIO33) and Vi4 (GPIO32) ADC with PSI conversion
  - OTA + API for Home Assistant
  - HA-editable settings: MQTT Host/User/Password/Port and DogStatsD Host/Port (ports are integers)
- Flash:
  ```bash
  esphome compile esphome/es32d26.yaml
  esphome upload esphome/es32d26.yaml --device /dev/cu.usbserial-0001  # first flash
  esphome upload esphome/es32d26.yaml --device 192.168.88.206          # OTA updates
  ```

