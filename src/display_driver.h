#pragma once

#include <lvgl.h>

/**
 * LVGL display driver for ST7796 on the Waveshare ESP32-S3-Touch-LCD-3.5-C.
 *
 * Handles:
 *  - TCA9554 backlight + LCD reset (I2C must be initialized first)
 *  - ST7796 SPI initialization (SWRESET, SLPOUT, COLMOD, MADCTL, DISPON)
 *  - LVGL display creation with PSRAM-backed double draw buffers
 *  - Flush callback that streams RGB565 pixels via SPI DMA
 */
class DisplayDriver {
public:
    /**
     * Full hardware + LVGL init. Call AFTER Wire.begin() and TCA9554 init.
     * Performs: SPI bus setup -> ST7796 register init -> MADCTL for rotation ->
     *          lv_display_create -> allocate PSRAM draw buffers.
     * Returns true on success.
     */
    bool begin();

    lv_display_t *getDisplay() const { return _disp; }

    /** Control backlight via TCA9554 P0 + GPIO6 fallback. */
    static void setBacklight(bool on);

private:
    lv_display_t *_disp = nullptr;

    /** Initialize ST7796 registers over SPI. */
    void initST7796();

    /** LVGL flush callback: sets address window and streams pixels. */
    static void flushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

    /** Send a command byte (DC low). */
    static void writeCommand(uint8_t cmd);

    /** Send data bytes (DC high). */
    static void writeData(const uint8_t *data, size_t len);
    static void writeData8(uint8_t val);
    static void writeData16(uint16_t val);
};

extern DisplayDriver Display;
