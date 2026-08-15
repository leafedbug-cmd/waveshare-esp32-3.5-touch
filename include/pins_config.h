#pragma once

#include <stdint.h>

namespace boardcfg {

// Waveshare ESP32-S3-Touch-LCD-3.5-C
// Physical reference: rear label upright, dual-row header at bottom edge.
// Locked display defaults for this hardware setup.
// - LCD_ROTATION controls 0/90/180/270 orientation.
// - LCD_MIRROR_X controls horizontal mirror independently.
// Flip 180deg to correct upside-down panel orientation on this build.
static constexpr int LCD_ROTATION = 3;
static constexpr bool LCD_MIRROR_X = false;

// ST7796 SPI interface
static constexpr uint32_t LCD_SPI_HZ = 40000000UL;
static constexpr int SPI_SCLK = 5;
static constexpr int SPI_MOSI = 1;
static constexpr int SPI_MISO = 2;
static constexpr int LCD_DC = 3;
static constexpr int LCD_CS = -1;
static constexpr int LCD_RST = -1;
static constexpr int LCD_HOR_RES = 320;
static constexpr int LCD_VER_RES = 480;

// Shared I2C bus and IO expander (TCA9554)
static constexpr uint32_t I2C_FREQ_HZ = 400000UL;
static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 7;
static constexpr uint8_t TCA_ADDR = 0x20;

// TCA outputs used by Waveshare examples
static constexpr uint8_t TCA_PIN_LCD_COMPAT_BL = 0;
static constexpr uint8_t TCA_PIN_LCD_RST = 1;

// Direct backlight fallback path seen on some revisions
static constexpr int LCD_BL_GPIO = 6;

static constexpr int BOOT_BUTTON_PIN = 0;

// SD/MMC 1-bit mode (vendor IDF demos)
static constexpr int SD_CLK = 11;
static constexpr int SD_CMD = 10;
static constexpr int SD_D0  = 9;

// FT6336 capacitive touch (on shared I2C bus)
static constexpr uint8_t TOUCH_ADDR = 0x38;

// QMI8658 6-axis IMU (on shared I2C bus)
static constexpr uint8_t IMU_ADDR = 0x6B;

// NRF24L01+ wiring (rear header)
static constexpr int NRF_CE = 38;
static constexpr int NRF_CSN = 39;
static constexpr int NRF_SCK = 40;
static constexpr int NRF_MOSI = 41;
static constexpr int NRF_MISO = 42;
static constexpr uint8_t NRF_NUM_CHANNELS = 126;

}  // namespace boardcfg
