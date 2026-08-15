# Waveshare ESP32-S3 Touch LCD 3.5-C Firmware

Firmware and hardware notes for the Waveshare ESP32-S3-Touch-LCD-3.5-C, including its ST7796 display, touch controller, sensors, audio, and nRF24/UART expansion interfaces.

## Find this repo again

Search for `Waveshare ESP32-S3-Touch-LCD-3.5-C`, `ST7796`, `TCA9554`, `NRF52_UART_RX`, or `rear-board-label-photo`.

See [Repository identity and recovery details](docs/REPOSITORY-IDENTITY.md).

## Build

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```
