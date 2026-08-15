#include "display_driver.h"
#include "pins_config.h"
#include <Arduino.h>
#include <SPI.h>
#include <esp_heap_caps.h>

namespace hw = boardcfg;

/* ST7796 command set (relevant subset) */
#define ST7796_NOP       0x00
#define ST7796_SWRESET   0x01
#define ST7796_SLPOUT    0x11
#define ST7796_INVON     0x21
#define ST7796_DISPOFF   0x28
#define ST7796_DISPON    0x29
#define ST7796_CASET     0x2A
#define ST7796_RASET     0x2B
#define ST7796_RAMWR     0x2C
#define ST7796_MADCTL    0x36
#define ST7796_COLMOD    0x3A

/* MADCTL bits */
#define MADCTL_MY   0x80
#define MADCTL_MX   0x40
#define MADCTL_MV   0x20
#define MADCTL_ML   0x10
#define MADCTL_BGR  0x08
#define MADCTL_MH   0x04

/* Display dimensions (native portrait) */
#define LCD_WIDTH   320
#define LCD_HEIGHT  480

/* Draw buffer: 40 lines of landscape width (480 px * 2 bytes * 40 lines = 38400 bytes) */
#define DRAW_BUF_LINES  40

static SPIClass *lcd_spi = nullptr;

/* Landscape resolution based on rotation */
static uint16_t disp_hor_res;
static uint16_t disp_ver_res;

DisplayDriver Display;

/* ---- Low-level SPI helpers ---- */

void DisplayDriver::writeCommand(uint8_t cmd) {
    digitalWrite(hw::LCD_DC, LOW);
    lcd_spi->beginTransaction(SPISettings(hw::LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
    lcd_spi->transfer(cmd);
    lcd_spi->endTransaction();
    digitalWrite(hw::LCD_DC, HIGH);
}

void DisplayDriver::writeData(const uint8_t *data, size_t len) {
    lcd_spi->beginTransaction(SPISettings(hw::LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
    lcd_spi->transferBytes(data, nullptr, len);
    lcd_spi->endTransaction();
}

void DisplayDriver::writeData8(uint8_t val) {
    lcd_spi->beginTransaction(SPISettings(hw::LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
    lcd_spi->transfer(val);
    lcd_spi->endTransaction();
}

void DisplayDriver::writeData16(uint16_t val) {
    lcd_spi->beginTransaction(SPISettings(hw::LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
    lcd_spi->transfer16(val);
    lcd_spi->endTransaction();
}

/* ---- ST7796 register initialization ---- */

void DisplayDriver::initST7796() {
    /* Software reset */
    writeCommand(ST7796_SWRESET);
    delay(120);

    /* Sleep out */
    writeCommand(ST7796_SLPOUT);
    delay(120);

    /* Pixel format: 16-bit RGB565 */
    writeCommand(ST7796_COLMOD);
    writeData8(0x55);

    /* MADCTL: rotation and color order.
     * Ported from the existing apply_waveshare_st7796_madctl() in the old main.cpp.
     * This board's IPS panel uses BGR subpixel order. */
    uint8_t madctl;
    switch (hw::LCD_ROTATION & 3) {
        case 0:
            madctl = MADCTL_BGR;
            disp_hor_res = LCD_WIDTH;
            disp_ver_res = LCD_HEIGHT;
            break;
        case 1:
            madctl = MADCTL_MY | MADCTL_MV | MADCTL_BGR;
            disp_hor_res = LCD_HEIGHT;
            disp_ver_res = LCD_WIDTH;
            break;
        case 2:
            madctl = MADCTL_MY | MADCTL_BGR;
            disp_hor_res = LCD_WIDTH;
            disp_ver_res = LCD_HEIGHT;
            break;
        case 3:
        default:
            madctl = MADCTL_MV | MADCTL_BGR;
            disp_hor_res = LCD_HEIGHT;
            disp_ver_res = LCD_WIDTH;
            break;
    }

    if (hw::LCD_MIRROR_X) {
        madctl |= MADCTL_MX;
    }

    writeCommand(ST7796_MADCTL);
    writeData8(madctl);
    Serial.printf("[LCD] MADCTL=0x%02X rotation=%d res=%dx%d\n",
                  madctl, hw::LCD_ROTATION & 3, disp_hor_res, disp_ver_res);

    /* Inversion on (required for IPS panels to show correct colors) */
    writeCommand(ST7796_INVON);

    /* Display on */
    writeCommand(ST7796_DISPON);
    delay(20);
}

/* ---- Backlight control ---- */

void DisplayDriver::setBacklight(bool on) {
    digitalWrite(hw::LCD_BL_GPIO, on ? HIGH : LOW);
}

/* ---- LVGL flush callback ---- */

void DisplayDriver::flushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    /* Set column address (CASET) */
    writeCommand(ST7796_CASET);
    writeData16(x1);
    writeData16(x2);

    /* Set row address (RASET) */
    writeCommand(ST7796_RASET);
    writeData16(y1);
    writeData16(y2);

    /* Write pixels (RAMWR) */
    writeCommand(ST7796_RAMWR);

    uint32_t pixel_count = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);
    uint32_t byte_count = pixel_count * 2;  /* RGB565 = 2 bytes per pixel */

    /* Stream pixel data with DC high (data mode) */
    lcd_spi->beginTransaction(SPISettings(hw::LCD_SPI_HZ, MSBFIRST, SPI_MODE0));
    lcd_spi->transferBytes(px_map, nullptr, byte_count);
    lcd_spi->endTransaction();

    /* Tell LVGL flush is done */
    lv_display_flush_ready(disp);
}

/* ---- Public API ---- */

bool DisplayDriver::begin() {
    /* Set up SPI bus for ST7796 */
    lcd_spi = new SPIClass(HSPI);
    lcd_spi->begin(hw::SPI_SCLK, hw::SPI_MISO, hw::SPI_MOSI, -1);

    /* DC pin: manual GPIO toggle for command vs data */
    pinMode(hw::LCD_DC, OUTPUT);
    digitalWrite(hw::LCD_DC, HIGH);

    /* Initialize ST7796 registers */
    initST7796();
    Serial.println("[LCD] ST7796 init complete");

    /* --- LVGL display setup --- */

    /* Allocate two partial draw buffers in PSRAM for double-buffering */
    size_t buf_size = disp_hor_res * DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    void *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!buf1 || !buf2) {
        Serial.printf("[LCD] PSRAM draw buffer alloc failed (requested %u bytes each)\n",
                      (unsigned)buf_size);
        return false;
    }

    Serial.printf("[LCD] Draw buffers: 2 x %u bytes in PSRAM (%d lines)\n",
                  (unsigned)buf_size, DRAW_BUF_LINES);

    _disp = lv_display_create(disp_hor_res, disp_ver_res);
    if (!_disp) {
        Serial.println("[LCD] lv_display_create failed");
        return false;
    }

    lv_display_set_flush_cb(_disp, flushCb);
    lv_display_set_buffers(_disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(_disp, LV_COLOR_FORMAT_RGB565);

    Serial.printf("[LCD] LVGL display ready: %dx%d\n", disp_hor_res, disp_ver_res);
    return true;
}
