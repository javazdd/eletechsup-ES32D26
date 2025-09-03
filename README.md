# Eletechsup ES32D26 Relay Test (ESP32)

This project loads an ESP32-DevKitC on the Eletechsup 2AO-8AI-8DI-8DO board and pulses all 8 relays one-by-one for 1s to verify hardware.

## Confirmed relay drive path
- Relays are driven through a 74HC595 shift register into a ULN2803A driver array.
- The shift register pins map to the ESP32 as follows:
  - SER / DATA (74HC595 pin 14)  -> ESP32 GPIO12
  - SRCLK / SHIFT CLK (pin 11)   -> ESP32 GPIO22
  - RCLK / LATCH (pin 12)        -> ESP32 GPIO23
  - OE (pin 13, active LOW)      -> ESP32 GPIO13 (held LOW to enable outputs)
  - MR (pin 10, active LOW)      -> tied HIGH on board (not controlled by firmware)

Notes:
- Bit order is LSB-first in firmware. Physical relay order may differ from bit order; this sketch just proves each channel actuates.
- DIP SW1 must enable digital outputs (Io1=ON, Io2=ON, Vo1=OFF, Vo2=OFF). Board must be powered with 12V (or 24V as supported) for relays to actuate. USB alone is not enough.

## Firmware behavior
- Each relay turns ON for 1 second, then OFF, sweeping through 8 relays repeatedly.
- Implementation sends bits to the 74HC595, then latches with RCLK.

## Build and upload (Arduino CLI)
Prereqs:
- Homebrew installed
- Arduino CLI installed: `brew install arduino-cli`
- ESP32 core installed once: `arduino-cli core update-index --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
  and `arduino-cli core install esp32:esp32 --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

Commands:
```bash
cd ~/esp32-relay-test
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 .
```
Adjust the serial port path if different.

## File of interest
- `esp32-relay-test.ino` – main sketch driving the relays via 74HC595.

## Troubleshooting
- If no clicks: ensure 12V power present; DIP SW1 set to Io1/Io2 ON; reseat ESP32; check that `/dev/cu.usbserial-...` exists; try pressing EN after upload.
- If only one channel toggles: wiring may differ; verify continuity from ESP32 pins to 74HC595 pins as above.
