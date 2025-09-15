# eletechsup-ES32D26 (ESP32 firmware)

Firmware for the Eletechsup 2AO-8AI-8DI-8DO board (ES32D26) using an ESP32-DevKitC.

## What it does
- Connects to Wi‑Fi and MQTT
- Subscribes to topics and energizes/de‑energizes relays based on MQTT payloads
- Samples 4 pressure transducers (Vi1..Vi4) every 15s
- Publishes `iot.pressure` as DogStatsD metrics to a Datadog Agent with rich tags
- Auto safety: after 300s from first MQTT command, turns all relays OFF

## Channels and relay mapping
- Relays are driven via 74HC595 -> ULN2803A
- Shift register pins to ESP32: DATA=GPIO12, SRCLK=GPIO22, LATCH=GPIO23, OE(LOW)=GPIO13
- Channel tags and topics:
  - ch1 → `prefilter`   → topic `/eletechsup/prefilter`
  - ch2 → `postfilter`  → topic `/eletechsup/postfilter`
  - ch3 → `500`         → topic `/eletechsup/500`
  - ch4 → `250`         → topic `/eletechsup/250`
  - ch5 → `100`         → topic `/eletechsup/100`
  - ch6 → `50`          → topic `/eletechsup/50`

## MQTT control

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
- Static IP: 192.168.88.203 (configured in firmware)
- Hostname: eletechsup (set via WiFi.setHostname)

