# AGENTS.md

These instructions apply to work in this repository.

**Target Hardware**
- Board: Waveshare ESP32-S3-Touch-LCD-3.5-C
- MCU: ESP32-S3, 16MB flash, 8MB PSRAM
- LCD: ST7796, SPI, 320x480
- IO expander: TCA9554 (I2C). Backlight + LCD reset are controlled here on some revisions.

**Onboard Peripherals**
- Camera: OV5640 (rear camera module)
- Audio Codec: ES8311
- IMU: QMI8658 (6-axis accelerometer + gyroscope)
- PMIC: AXP2101
- RTC: PCF85063
- USB: USB-C
- Storage: Micro SD slot
- Buttons: `PWR`, `RST`, `BOOT`

**Known-Good Pin Map (Waveshare Demo)**
- SPI: `SCLK=GPIO5`, `MOSI=GPIO1`, `MISO=GPIO2`, `DC=GPIO3`, `CS=NC`
- I2C: `SDA=GPIO8`, `SCL=GPIO7`
- TCA9554 I2C address: `0x20`
- TCA9554 pins: `P0=Backlight`, `P1=LCD Reset` (some revisions may swap)
- Direct backlight fallback: `GPIO6` (some revisions drive BL directly)

**Required Init Sequence**
1. Initialize I2C on GPIO8/7.
2. Initialize TCA9554 at `0x20`.
3. Set TCA `P0` and `P1` as outputs.
4. Backlight ON: `P0=1` (also drive `GPIO6=HIGH` as fallback).
5. LCD reset pulse: `P1=1 -> 0 -> 1` with small delays.
6. Initialize ST7796 over SPI.

**PlatformIO Baseline**
- Use Arduino framework.
- Use Arduino_GFX (ST7796) and TCA9554 libraries.
- If build fails with missing `esp32-hal-periman.h`, pin Arduino_GFX to `1.5.5`.
- Example config lives in `platformio.ini`.

**Minimal Working Sketch**
- `src/main.cpp` draws a red rectangle with white border and yellow text.
- It enables both TCA backlight and `GPIO6` backlight fallback.

**Troubleshooting Checklist**
- If screen stays black, confirm serial output and that backlight is ON.
- If backlight is OFF, toggle TCA `P0` and `P1` (some boards swap BL/RST).
- If display is washed out or mirrored, adjust ST7796 rotation/invert.
- If SPI errors, double-check `SCLK/MOSI/DC` pins and that `CS` is `-1`.

**Hardware Configuration: NRF24L01 Wiring**

### NRF24L01+ to Waveshare ESP32-S3 Header
| Wire Color | NRF Function | ESP32 Pin | Function |
| :--- | :--- | :--- | :--- |
| **Blue** | VCC | **3V3** | Power (3.3V) |
| **Black** | GND | **G** | Ground |
| **White** | CE | **38** | Chip Enable |
| **Green** | CSN | **39** | Chip Select |
| **Grey** | SCK | **40** | SPI Clock |
| **Yellow** | MOSI | **41** | SPI MOSI |
| **Purple** | MISO | **42** | SPI MISO |
| **Orange** | IRQ | **N/C** | Not Connected |

**Driver Implementation Notes:**
- Initialize SPI bus with: `SCK=40, MOSI=41, MISO=42`
- Initialize Radio with: `CE=38, CSN=39`
- WARNING: Do not use 5V on the VCC line.
