#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include "pins_config.h"
#include "TCA9554.h"
#include "display_driver.h"

namespace hw = boardcfg;

static TCA9554 tca(hw::TCA_ADDR);

/* ---- LVGL tick provider ---- */

static uint32_t lvgl_tick_cb(void) {
    return millis();
}

/* ---- TCA9554 + backlight/reset sequence ---- */

static bool init_tca_backlight_and_reset() {
    Wire.begin(hw::I2C_SDA, hw::I2C_SCL);
    Wire.setClock(hw::I2C_FREQ_HZ);

    /* Direct GPIO backlight fallback (always enable) */
    pinMode(hw::LCD_BL_GPIO, OUTPUT);
    digitalWrite(hw::LCD_BL_GPIO, HIGH);

    if (!tca.begin()) {
        Serial.printf("[LCD] TCA9554 not found at 0x%02X, using GPIO%d backlight only\n",
                      hw::TCA_ADDR, hw::LCD_BL_GPIO);
        delay(120);
        return false;
    }

    /* Configure TCA outputs */
    tca.pinMode1(hw::TCA_PIN_LCD_COMPAT_BL, OUTPUT);
    tca.pinMode1(hw::TCA_PIN_LCD_RST, OUTPUT);
    tca.write1(hw::TCA_PIN_LCD_COMPAT_BL, HIGH);

    /* LCD reset pulse: HIGH -> LOW -> HIGH */
    tca.write1(hw::TCA_PIN_LCD_RST, HIGH);
    delay(10);
    tca.write1(hw::TCA_PIN_LCD_RST, LOW);
    delay(10);
    tca.write1(hw::TCA_PIN_LCD_RST, HIGH);
    delay(120);

    Serial.printf("[LCD] TCA9554 ready @ 0x%02X (BL=P%d, RST=P%d)\n",
                  hw::TCA_ADDR, hw::TCA_PIN_LCD_COMPAT_BL, hw::TCA_PIN_LCD_RST);
    return true;
}

/* ---- Simple LVGL test UI ---- */

static void create_test_ui() {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003040), LV_PART_MAIN);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text_fmt(label, "Waveshare ESP32-S3-Touch-LCD-3.5-C\n"
                                "LVGL v%d.%d.%d\n\n"
                                "SD + PSRAM Asset System",
                                LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    lv_obj_set_style_text_color(label, lv_color_hex(0x00E0FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_center(label);
}

/* ---- Arduino entry points ---- */

void setup() {
    Serial.begin(115200);
    delay(120);

    Serial.println();
    Serial.println("[BOOT] Waveshare ESP32-S3-Touch-LCD-3.5-C");
    Serial.printf("[LCD]  SPI SCK=%d MOSI=%d MISO=%d DC=%d\n",
                  hw::SPI_SCLK, hw::SPI_MOSI, hw::SPI_MISO, hw::LCD_DC);
    Serial.printf("[I2C]  SDA=%d SCL=%d TCA=0x%02X\n",
                  hw::I2C_SDA, hw::I2C_SCL, hw::TCA_ADDR);

    /* 1. I2C + TCA + backlight + LCD reset */
    init_tca_backlight_and_reset();

    /* 2. LVGL core init */
    lv_init();
    lv_tick_set_cb(lvgl_tick_cb);

    /* 3. Display driver (SPI + ST7796 + LVGL display) */
    if (!Display.begin()) {
        Serial.println("[BOOT] Display init FAILED — halting");
        while (true) { delay(1000); }
    }

    /* 4. Show test UI */
    create_test_ui();

    Serial.println("[BOOT] Setup complete");
}

void loop() {
    lv_timer_handler();
    delay(5);
}
