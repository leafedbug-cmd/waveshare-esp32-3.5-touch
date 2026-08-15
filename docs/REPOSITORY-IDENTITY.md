# Repository Identity

- Canonical name: Waveshare ESP32-S3 Touch LCD 3.5-C Firmware
- Purpose: PlatformIO firmware and hardware reference for the Waveshare ESP32-S3-Touch-LCD-3.5-C and attached nRF radios.
- Remote: `leafedbug-cmd/waveshare-esp32-3.5-touch`
- Required base branch: `main`; recovery branch: `agent/refactor-waveshare-display`
- Recovery commit: `f128fb1` (`Refactor Waveshare display firmware`)
- Local alias: the Desktop checkout of `waveshare-esp32-3.5-touch`; it differs from the checkout under `/home/bug/GitHub`.
- Entry points: `src/main.cpp`, `src/display_driver.cpp`
- Manifest/config: `platformio.ini`, `boards/waveshare_esp32_s3_touch_lcd_3_5_c.json`, `include/pins_config.h`
- Build/flash/monitor: `pio run`; `pio run -t upload`; `pio device monitor -b 115200`
- Generated artifacts: `.pio/build/`; ignored and not versioned.
- Hardware reference: `buildsheet.md`, `docs/rear-board-label-photo.jpg`, and `AGENTS.md`.

Search terms: Waveshare ESP32-S3-Touch-LCD-3.5-C ST7796 TCA9554 NRF52_UART_RX rear-board-label-photo waveshare-esp32-3.5-touch.
