# Repository Identity

- Canonical name: Waveshare ESP32-S3 Touch LCD 3.5 Radio UI
- Purpose: PlatformIO touchscreen firmware for Waveshare board peripherals and attached nRF radios.
- Remote: `leafedbug-cmd/waveshare-esp32-3.5-touch`
- Required base branch: `main`; recovery branch: `agent/update-waveshare-ui`
- Recovery commit: `dc3a565` (`Update UI and add hardware photos`)
- Local alias: the `/home/bug/GitHub` checkout; it differs from the Desktop checkout on another recovery branch.
- Entry point: `src/main.cpp`
- Manifest: `platformio.ini`; hardware instructions: `AGENTS.md`
- Build/flash/monitor: `pio run`; `pio run -t upload`; `pio device monitor -b 115200`
- Generated artifacts: `.pio/build/`; ignored.

Search terms: waveshare-esp32-3.5-touch Waveshare ESP32-S3-Touch-LCD-3.5-C TARGET_SSID NRF52_UART_RX TCA9554 RF24.
