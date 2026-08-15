# Waveshare ESP32-S3-Touch-LCD-3.5-C Build Sheet

Canonical hardware and bring-up reference for this repo.

## Official Sources

- Waveshare wiki: <https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.5>
- Waveshare demo bundle: <https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.5/ESP32-S3-Touch-LCD-3.5-Demo.zip>
- Local rear-label photo: [`docs/rear-board-label-photo.jpg`](docs/rear-board-label-photo.jpg)
- Repo source of truth for pins used by this project: [`include/pins_config.h`](include/pins_config.h)

The goal of this sheet is simple:

- use Waveshare's official board data wherever possible
- document the current local wiring exactly as built
- call out every known mismatch or pin conflict directly

## 1. Board Identity

| Item | Value |
| --- | --- |
| Board | `ESP32-S3-Touch-LCD-3.5-C` |
| MCU | ESP32-S3 |
| Flash | `16MB` |
| PSRAM | `8MB` |
| LCD | `ST7796`, `320x480`, SPI |
| Touch | `FT6336` |
| PMIC | `AXP2101` |
| RTC | `PCF85063` |
| IMU | `QMI8658` |
| IO expander | `TCA9554` |
| Audio codec | `ES8311` |
| Camera | `OV5640` |
| USB | USB-C power/data |
| Storage | Micro SD |

## 2. Physical Orientation

Use the board in this orientation for every pin reference below:

- rear label upright
- dual-row header at the bottom edge
- LCD/front treated as USB-C on the left

Quick rear-label sanity check:

- top row starts `5V, G, 19, 20`
- bottom row starts `BAT, G, 21, 38`

Rotation policy for this repo:

| Item | Value |
| --- | --- |
| Vendor baseline | portrait, `rotation 0` in Waveshare Arduino examples |
| Repo UI | landscape |
| Checked-in default | `LCD_ROTATION = 1` in [`include/pins_config.h`](include/pins_config.h) |
| Locked known-good mapping | rotation `1` + MADCTL `MY | MV | BGR` (non-mirrored text) |
| If a different panel revision mirrors text | adjust MADCTL bits in [`src/main.cpp`](src/main.cpp) case `1`, not just rotation |

The touch remap in [`src/main.cpp`](src/main.cpp) follows `LCD_ROTATION`.

## 3. Rear Header Pinout

Read exactly as printed on the back label photo.

| Position | Top Row | Bottom Row |
| --- | --- | --- |
| 1 | `5V` | `BAT` |
| 2 | `G` | `G` |
| 3 | `19` | `21` |
| 4 | `20` | `38` |
| 5 | `11` | `39` |
| 6 | `10` | `40` |
| 7 | `9` | `41` |
| 8 | `17` | `42` |
| 9 | `18` | `45` |
| 10 | `0` | `46` |
| 11 | `RST` | `47` |
| 12 | `PWR` | `48` |
| 13 | `SCL` | `TX` |
| 14 | `SDA` | `RX` |
| 15 | `G` | `G` |
| 16 | `3V3` | `3V3` |

## 4. Vendor-Verified Pin Map

### LCD / Touch / I2C

| Function | Pin / Value | Notes |
| --- | --- | --- |
| LCD SCK | `GPIO5` | Waveshare Arduino + IDF demos |
| LCD MOSI | `GPIO1` | Waveshare Arduino + IDF demos |
| LCD MISO | `GPIO2` | Waveshare Arduino + IDF demos |
| LCD DC | `GPIO3` | Waveshare Arduino + IDF demos |
| LCD CS | `NC` / `-1` | Waveshare demos use no dedicated CS |
| LCD RST | `NC` / `-1` | reset driven through TCA9554 |
| LCD backlight | `GPIO6` | primary vendor backlight path |
| I2C SDA | `GPIO8` | shared onboard peripheral bus |
| I2C SCL | `GPIO7` | shared onboard peripheral bus |
| TCA9554 address | `0x20` | vendor demos |
| TCA9554 `P1` | LCD reset | consistent across vendor Arduino demos |
| TCA9554 `P0` | extra high in some IDF demos | safe compatibility output, not the primary vendor BL path |

### Audio / Storage / Camera

| Function | Pin / Value | Notes |
| --- | --- | --- |
| I2S MCK | `GPIO12` | ES8311 path |
| I2S BCK | `GPIO13` | ES8311 path |
| I2S LRCK | `GPIO15` | ES8311 path |
| I2S DOUT | `GPIO16` | ESP32 -> codec |
| I2S DIN | `GPIO14` | codec -> ESP32 |
| SDMMC CLK | `GPIO11` | vendor IDF |
| SDMMC CMD | `GPIO10` | vendor IDF |
| SDMMC D0 | `GPIO9` | vendor IDF |
| Camera XCLK | `GPIO38` | OV5640 |
| Camera SIOD | `GPIO8` | OV5640 SCCB/I2C |
| Camera SIOC | `GPIO7` | OV5640 SCCB/I2C |
| Camera Y9 | `GPIO21` | OV5640 |
| Camera Y8 | `GPIO39` | OV5640 |
| Camera Y7 | `GPIO40` | OV5640 |
| Camera Y6 | `GPIO42` | OV5640 |
| Camera Y5 | `GPIO46` | OV5640 |
| Camera Y4 | `GPIO48` | OV5640 |
| Camera Y3 | `GPIO47` | OV5640 |
| Camera Y2 | `GPIO45` | OV5640 |
| Camera VSYNC | `GPIO17` | OV5640 |
| Camera HREF | `GPIO18` | OV5640 |
| Camera PCLK | `GPIO41` | OV5640 |

## 5. Current NRF24L01+PA+LNA Wiring

This is the current physical build, not a vendor feature.

| Wire Color | NRF24 Signal | Board Pin | Notes |
| --- | --- | --- | --- |
| Blue | `VCC` | `3V3` | 3.3V only |
| Black | `GND` | `G` | common ground |
| White | `CE` | `38` | bottom row |
| Green | `CSN` | `39` | bottom row |
| Grey | `SCK` | `40` | bottom row |
| Yellow | `MOSI` | `41` | bottom row |
| Purple | `MISO` | `42` | bottom row |
| Orange | `IRQ` | not connected | optional, unused |

Electrical rule:

- never power the NRF24 from `5V`

Code alignment:

- [`include/pins_config.h`](include/pins_config.h) is the canonical repo definition
- [`src/main.cpp`](src/main.cpp) uses that header directly

## 6. Critical Pin Conflict

The current NRF24 wiring consumes these ESP32 pins:

- `38`
- `39`
- `40`
- `41`
- `42`

Those same pins are used by the onboard OV5640 camera:

- `38` = `XCLK`
- `39` = `Y8`
- `40` = `Y7`
- `41` = `PCLK`
- `42` = `Y6`

Practical result:

- the NRF24 build works
- the onboard camera is effectively unavailable while the NRF24 is attached to those pins

This is now called out in both the build sheet and the boot serial logs.

## 7. Required LCD Bring-Up Sequence

For this board family, the safe init order is:

1. Start I2C on `GPIO8/7`.
2. Set `GPIO6` high for the vendor backlight path.
3. Initialize `TCA9554` at `0x20`.
4. Configure `P0` and `P1` as outputs.
5. Drive `P0` high as a compatibility output.
6. Pulse LCD reset on `P1`: `1 -> 0 -> 1`.
7. Start `ST7796` over SPI on `5/1/2/3`.

Repo status:

- [`src/main.cpp`](src/main.cpp) now follows this sequence
- boot serial now prints whether TCA detection succeeded
- firmware applies a custom ST7796 MADCTL override for this panel so landscape text is not mirrored
- display SPI clock is `40 MHz`

## 8. Repo Reality

| Area | Current State |
| --- | --- |
| PlatformIO board | [`boards/waveshare_esp32_s3_touch_lcd_3_5_c.json`](boards/waveshare_esp32_s3_touch_lcd_3_5_c.json) |
| PlatformIO env | `esp32-s3-touch-lcd-3_5` in [`platformio.ini`](platformio.ini) |
| Framework | Arduino |
| Display lib | `Arduino_GFX 1.5.5` |
| Key libs | `TCA9554`, `SensorLib`, `RF24` |
| LCD/backlight pins | centralized in [`include/pins_config.h`](include/pins_config.h) |
| Touch rotation remap | implemented in [`src/main.cpp`](src/main.cpp) |
| ES8311 codec support | stubbed by [`src/es8311_stub.cpp`](src/es8311_stub.cpp) |
| PMU / battery UI | not implemented yet in the checked-in app |
| Camera support | not implemented in the checked-in app, and blocked by current NRF24 wiring |

## 9. Flash And Monitor

User-validated flash command:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor -b 115200
```

If auto-detect ever misses the port on this Linux box, fall back to:

```bash
pio run -t upload --upload-port /dev/ttyACM0
pio device monitor -p /dev/ttyACM0 -b 115200
```

Compatibility note:

- if `Arduino_GFX` ever trips on `esp32-hal-periman.h`, keep it pinned to `1.5.5`

## 10. Bring-Up Checklist

1. Verify the board is oriented with the rear label upright and the header at the bottom.
2. Power from USB-C.
3. Flash with `pio run -t upload`.
4. Check boot serial for TCA, LCD, touch, IMU, and NRF status lines.
5. Confirm the backlight turns on.
6. Confirm the UI is upright at `LCD_ROTATION = 3`.
7. If a different unit is flipped, change `LCD_ROTATION` to `1`.
8. Do not expect the OV5640 camera to work while the NRF24 is wired to `38-42`.

## 11. Analyzer Color Code

| Color | Meaning |
| --- | --- |
| Blue | Wi-Fi CH1 area, about `2412 MHz` |
| Green | Wi-Fi CH6 area, about `2437 MHz` |
| Yellow | Wi-Fi CH11 area, about `2462 MHz` |
| Magenta | BLE advertising channels `2402 / 2426 / 2480 MHz` |
| Cyan | other channels in the normal 2.4 GHz ISM band |
| Dark Gray | above about `2483 MHz` |
