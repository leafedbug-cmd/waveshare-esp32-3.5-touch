#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <Arduino_GFX_Library.h>
#include "TCA9554.h"
#include "es8311.h"
#include "SensorQMI8658.hpp"
#include "TouchDrvFT6X36.hpp"

// --- NRF24 Includes ---
#include <SPI.h>
#include <RF24.h>

extern "C" {
#include "driver/i2s.h"
}

// ==========================================
// HARDWARE CONFIGURATION (Waveshare S3)
// ==========================================
#define SPI_SCLK 5
#define SPI_MOSI 1
#define SPI_MISO 2

#define LCD_CS   -1
#define LCD_DC   3
#define LCD_RST  -1
#define LCD_HOR_RES 320
#define LCD_VER_RES 480

#define I2C_SDA 8
#define I2C_SCL 7

#define TCA_ADDR 0x20
#define TCA_PIN_BL 0
#define TCA_PIN_RST 1
#define GPIO_BL 6

#define BOOT_BUTTON_PIN 0

// NRF24 Pins
#define NRF_CE   38
#define NRF_CSN  39
#define NRF_SCK  40
#define NRF_MOSI 41
#define NRF_MISO 42

// Audio
#define I2S_PORT        I2S_NUM_0
#define I2S_MCK_PIN     12
#define I2S_BCK_PIN     13
#define I2S_LRCK_PIN    15
#define I2S_DOUT_PIN    16
#define I2S_DIN_PIN     14
#define AUDIO_SAMPLE_HZ 16000
#define AUDIO_MCLK_HZ   (AUDIO_SAMPLE_HZ * 256)

#define IMU_HEADING_SIGN -1.0f

// --- TUNING CONSTANTS ---
static const char *TARGET_SSID = "";
static const char *TARGET_BSSID = "";
static const int RSSI_NEAR_DBM = -45;
static const int RSSI_FAR_DBM = -95;
static const uint32_t FRAME_INTERVAL_MS = 90;
static const uint8_t MAX_TRACKED_DEVICES = 24;
static const uint32_t DEVICE_HIDE_MS = 10000;
static const uint32_t DEVICE_PRUNE_MS = 45000;
static const uint16_t LEVEL_CAL_SAMPLES = 42;
static const uint8_t MAX_LIST_ROWS = 8;
static const uint16_t BLE_SCAN_SECONDS = 1;
static const uint32_t BLE_SCAN_INTERVAL_MS = 5000;
static const uint32_t SCAN_INTERVAL_SIGNAL_FINDER_MS = 2000;
static const uint32_t MODE_TOUCH_GUARD_MS = 400;

// NRF Constants
#define NRF_NUM_CHANNELS 128
#define MAX_SNIFFER_LOGS 12

// UI Layout (Base)
static int BTN_BACK_X = 236;
static int BTN_BACK_Y = 6;
static int BTN_BACK_W = 76;
static int BTN_BACK_H = 28;
static int BTN_CAL_X = 236;
static int BTN_CAL_Y = 38;
static int BTN_CAL_W = 76;
static int BTN_CAL_H = 28;
static int BTN_LBEEP_X = 236;
static int BTN_LBEEP_Y = 70;
static int BTN_LBEEP_W = 76;
static int BTN_LBEEP_H = 28;
static int RADAR_LIST_X = 8;
static int RADAR_LIST_Y = 96;
static int RADAR_LIST_W = 304;
static int RADAR_ROW_H = 44;

// --- APP MODES ---
enum AppMode : uint8_t
{
  APP_WIFI_SCAN = 0,
  APP_BLE_SCAN = 1,
  APP_NRF = 2,
  APP_TRI_BAND = 3,  
  APP_SNIFFER = 4,   
  APP_LEVEL_ONLY = 5 
};

enum TrendState : uint8_t { TREND_UNKNOWN = 0, TREND_CLOSER, TREND_FARTHER, TREND_STABLE };
enum SignalSource : uint8_t { SIGNAL_WIFI = 0, SIGNAL_BLE = 1 };
enum LevelBeepMode : uint8_t { LEVEL_BEEP_OFF = 0, LEVEL_BEEP_WHEN_LEVEL = 1, LEVEL_BEEP_WHEN_NOT_LEVEL = 2 };
enum ScanSourceMode : uint8_t { SCAN_WIFI_ONLY = 0, SCAN_WIFI_BLE = 1, SCAN_BLE_ONLY = 2 };

struct RadarDevice
{
  bool used = false;
  bool seen_this_scan = false;
  bool visible = false;
  SignalSource source = SIGNAL_WIFI;
  String ssid;
  String bssid;
  int32_t channel = 0;
  uint32_t last_seen_ms = 0;
  float raw_rssi = -100.0f;
  float smooth_rssi = -100.0f;
  float prev_smooth_rssi = -100.0f;
  bool has_rssi = false;
  TrendState trend = TREND_UNKNOWN;
  float signal_01 = 0.0f;
};

// --- GLOBAL OBJECTS ---
TCA9554 tca(TCA_ADDR);
SensorQMI8658 qmi;
TouchDrvFT6X36 touch;
IMUdata acc;
IMUdata gyr;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, SPI_SCLK, SPI_MOSI, SPI_MISO);
Arduino_GFX *gfx = new Arduino_ST7796(bus, LCD_RST, 0, true, LCD_HOR_RES, LCD_VER_RES);

// NRF Object
RF24 radio(NRF_CE, NRF_CSN);
static bool g_nrf_ready = false;
static uint8_t g_nrf_channel_buckets[NRF_NUM_CHANNELS];
static String g_sniffer_logs[MAX_SNIFFER_LOGS]; // For Sniffer Mode

static es8311_handle_t g_es8311 = nullptr;
static bool g_audio_ready = false;
static bool g_i2s_ready = false;
static bool g_imu_ready = false;
static bool g_touch_ready = false;
static bool g_ble_ready = false;
static bool g_ble_init_attempted = false;
static BLEScan *g_ble_scan = nullptr;
static AppMode g_app_mode = APP_WIFI_SCAN;
static ScanSourceMode g_scan_source_mode = SCAN_WIFI_ONLY;

static RadarDevice g_devices[MAX_TRACKED_DEVICES];
static int g_selected_device = -1;
static bool g_touch_was_pressed = false;
static int16_t g_touch_last_x = 0;
static int16_t g_touch_last_y = 0;
static int g_touch_press_device = -1;
static bool g_touch_press_back = false;
static bool g_touch_press_src = false;
static bool g_radar_ui_static_ready = false;
static bool g_radar_ui_has_sig = false;
static uint32_t g_radar_ui_last_sig = 0;
static uint8_t g_display_rotation = 0;
static int g_last_tap_device = -1;
static bool g_last_tap_selected = false;
static uint32_t g_last_tap_feedback_until_ms = 0;
static int g_visible_count = 0;
static int g_sorted_count = 0;
static int g_sorted_indices[MAX_TRACKED_DEVICES];
static int g_row_to_device[MAX_LIST_ROWS];

static float g_heading_deg = 0.0f;
static float g_level_x_deg = 0.0f;
static float g_level_y_deg = 0.0f;
static bool g_level_valid = false;
static float g_level_zero_x_deg = 0.0f;
static float g_level_zero_y_deg = 0.0f;
static bool g_level_zero_set = false;
static bool g_level_ui_static_ready = false;
static bool g_level_calibrating = false;
static uint16_t g_level_cal_count = 0;
static float g_level_cal_sum_x = 0.0f;
static float g_level_cal_sum_y = 0.0f;
static bool g_level_prev_dot_valid = false;
static int16_t g_level_prev_dot_x = 0;
static int16_t g_level_prev_dot_y = 0;
static uint16_t g_level_prev_dot_color = BLACK;
static LevelBeepMode g_level_beep_mode = LEVEL_BEEP_OFF;
static bool g_level_flat_state_valid = false;
static bool g_level_is_flat = false;
static uint32_t g_last_level_beep_ms = 0;

static uint32_t g_last_scan_ms = 0;
static uint32_t g_last_ble_scan_ms = 0;
static uint32_t g_last_frame_ms = 0;
static uint32_t g_last_beep_ms = 0;
static uint32_t g_last_imu_us = 0;
static uint32_t g_mode_enter_ms = 0;
static uint32_t g_last_audio_init_attempt_ms = 0;
static bool g_prev_boot_pressed = false;

// --- FORWARD DECLARATIONS ---
static void play_beep_tone(uint16_t freq_hz, uint16_t duration_ms, float gain);
static bool ble_scanner_init();
static void draw_radar_ui();
static void draw_level_only_ui();
static void draw_nrf_ui(bool full_redraw);
static void draw_tri_band_ui(bool full_redraw);
static void draw_sniffer_ui(bool full_redraw);
static void nrf_scan_channels();

// --- HELPER FUNCTIONS ---
static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static uint32_t hash_mix_u32(uint32_t h, uint32_t v) {
  h ^= v; h *= 16777619u; return h;
}

static uint32_t hash_text_prefix(const String &s, size_t max_chars) {
  uint32_t h = 2166136261u;
  const size_t n = (s.length() < max_chars) ? s.length() : max_chars;
  for (size_t i = 0; i < n; ++i) h = hash_mix_u32(h, uint8_t(s.charAt(i)));
  return h;
}

static bool is_landscape_rotation(uint8_t rot) { return (rot == 1u) || (rot == 3u); }

static int radar_rows_visible() {
  const int usable_h = gfx->height() - RADAR_LIST_Y - 6;
  const int rows = (usable_h > 0) ? (usable_h / RADAR_ROW_H) : 0;
  if (rows <= 0) return 1;
  return (rows < int(MAX_LIST_ROWS)) ? rows : int(MAX_LIST_ROWS);
}

static const char *app_mode_title(AppMode mode) {
  if (mode == APP_WIFI_SCAN) return "WiFi Scan";
  if (mode == APP_BLE_SCAN) return "BLE Scan";
  if (mode == APP_NRF) return "NRF Spec";
  if (mode == APP_TRI_BAND) return "Tri-Band";
  if (mode == APP_SNIFFER) return "Sniffer";
  return "Level Mode";
}

static ScanSourceMode scan_source_for_mode(AppMode mode) {
  return (mode == APP_BLE_SCAN) ? SCAN_BLE_ONLY : SCAN_WIFI_ONLY;
}

// ** MISSING HELPERS RESTORED HERE **
static bool scan_mode_uses_wifi() { return g_app_mode == APP_WIFI_SCAN; }
static bool scan_mode_uses_ble() { return g_app_mode == APP_BLE_SCAN; }

static void update_ui_layout_for_rotation() {
  const int w = gfx->width();
  const int h = gfx->height();
  if (is_landscape_rotation(g_display_rotation)) {
    BTN_BACK_W = 84; BTN_BACK_H = 28;
    BTN_BACK_X = w - BTN_BACK_W - 8; BTN_BACK_Y = 6;
    BTN_CAL_W = 84; BTN_CAL_H = 28;
    BTN_CAL_X = BTN_BACK_X; BTN_CAL_Y = 38;
    BTN_LBEEP_W = 84; BTN_LBEEP_H = 28;
    BTN_LBEEP_X = BTN_BACK_X; BTN_LBEEP_Y = 70;
    RADAR_LIST_X = 8; RADAR_LIST_Y = 56;
    RADAR_LIST_W = w - 16; RADAR_ROW_H = 42;
  } else {
    BTN_BACK_X = 236; BTN_BACK_Y = 6; BTN_BACK_W = 76; BTN_BACK_H = 28;
    BTN_CAL_X = 236; BTN_CAL_Y = 38; BTN_CAL_W = 76; BTN_CAL_H = 28;
    BTN_LBEEP_X = 236; BTN_LBEEP_Y = 70; BTN_LBEEP_W = 76; BTN_LBEEP_H = 28;
    RADAR_LIST_X = 8; RADAR_LIST_Y = 96; RADAR_LIST_W = 304; RADAR_ROW_H = 44;
  }
}

static void reset_ui_draw_state() {
  g_touch_was_pressed = false;
  g_touch_press_device = -1;
  g_touch_press_back = false;
  g_touch_press_src = false;
  g_radar_ui_static_ready = false;
  g_radar_ui_has_sig = false;
  g_level_ui_static_ready = false;
  g_level_prev_dot_valid = false;
}

static void apply_display_rotation(uint8_t rotation, bool force_redraw = true) {
  rotation &= 0x03;
  if (rotation == g_display_rotation) return;
  g_display_rotation = rotation;
  gfx->setRotation(g_display_rotation);
  update_ui_layout_for_rotation();
  reset_ui_draw_state();
  g_mode_enter_ms = millis();
  g_last_frame_ms = g_mode_enter_ms - FRAME_INTERVAL_MS;

  if (force_redraw) {
    if (g_app_mode == APP_TRI_BAND) draw_tri_band_ui(true);
    else if (g_app_mode == APP_SNIFFER) draw_sniffer_ui(true);
    else if (g_app_mode == APP_NRF) draw_nrf_ui(true);
    else if (g_app_mode == APP_LEVEL_ONLY) draw_level_only_ui();
    else draw_radar_ui();
  }
}

static bool read_touch_point(int16_t &x, int16_t &y) {
  int16_t tx[1] = {0}; int16_t ty[1] = {0};
  if (touch.getPoint(tx, ty, 1) <= 0) return false;
  int16_t mx = tx[0]; int16_t my = ty[0];
  if (g_display_rotation == 1) { mx = ty[0]; my = int16_t((LCD_HOR_RES - 1) - tx[0]); }
  else if (g_display_rotation == 2) { mx = int16_t((LCD_HOR_RES - 1) - mx); my = int16_t((LCD_VER_RES - 1) - my); }
  else if (g_display_rotation == 3) { mx = int16_t((LCD_VER_RES - 1) - ty[0]); my = tx[0]; }
  
  if (mx < 0) mx = 0; if (mx >= gfx->width()) mx = gfx->width() - 1;
  if (my < 0) my = 0; if (my >= gfx->height()) my = gfx->height() - 1;
  x = mx; y = my;
  return true;
}

static float wrap360(float deg) {
  while (deg < 0.0f) deg += 360.0f; while (deg >= 360.0f) deg -= 360.0f; return deg;
}

static float rssi_to_signal01(float rssi) {
  const float denom = float(RSSI_NEAR_DBM - RSSI_FAR_DBM);
  if (denom <= 0.0f) return 0.0f;
  return clamp01((rssi - float(RSSI_FAR_DBM)) / denom);
}

static const char *trend_to_text(TrendState trend) {
  switch (trend) { case TREND_CLOSER: return "closer"; case TREND_FARTHER: return "farther"; case TREND_STABLE: return "stable"; default: return "unknown"; }
}

static uint16_t source_to_color(SignalSource source) { return (source == SIGNAL_BLE) ? MAGENTA : CYAN; }

static bool has_cfg_target_bssid() { return (TARGET_BSSID != nullptr) && (TARGET_BSSID[0] != '\0'); }
static bool has_cfg_target_ssid() { return (TARGET_SSID != nullptr) && (TARGET_SSID[0] != '\0'); }

static bool bssid_equals_ignore_case(const String &a, const char *b) {
  if (b == nullptr || b[0] == '\0') return false;
  String rhs(b); return a.equalsIgnoreCase(rhs);
}

static bool point_in_rect(int16_t x, int16_t y, int rx, int ry, int rw, int rh) {
  return (x >= rx) && (x < (rx + rw)) && (y >= ry) && (y < (ry + rh));
}

static void draw_touch_button(int x, int y, int w, int h, const char *label, uint16_t fill, uint16_t border, uint16_t text) {
  gfx->fillRoundRect(x, y, w, h, 6, fill);
  gfx->drawRoundRect(x, y, w, h, 6, border);
  gfx->setTextColor(text); gfx->setTextSize(1);
  const int text_x = x + (w / 2) - (int(strlen(label)) * 3);
  const int text_y = y + (h / 2) - 3;
  gfx->setCursor(text_x, text_y);
  gfx->print(label);
}

// --- INIT FUNCTIONS ---
static void tca9554_init_and_lcd_reset() {
  Wire.begin(I2C_SDA, I2C_SCL);
  tca.begin();
  tca.pinMode1(TCA_PIN_BL, OUTPUT); tca.pinMode1(TCA_PIN_RST, OUTPUT);
  tca.write1(TCA_PIN_BL, 1);
  pinMode(GPIO_BL, OUTPUT); digitalWrite(GPIO_BL, HIGH);
  tca.write1(TCA_PIN_RST, 1); delay(10);
  tca.write1(TCA_PIN_RST, 0); delay(10);
  tca.write1(TCA_PIN_RST, 1); delay(120);
}

static bool imu_init() {
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS)) return false;
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_112_1Hz, SensorQMI8658::LPF_MODE_3);
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_0);
  qmi.enableGyroscope(); qmi.enableAccelerometer();
  g_last_imu_us = micros();
  return true;
}

static bool touch_init() {
  if (!touch.begin(Wire, FT6X36_SLAVE_ADDRESS)) return false;
  return true;
}

static bool nrf_init() {
  if (g_nrf_ready) return true;
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  if (!radio.begin(&SPI)) return false;
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(false);
  radio.startListening();
  radio.stopListening();
  g_nrf_ready = true;
  return true;
}

// --- IMU LOGIC ---
static void update_imu_heading() {
  if (!g_imu_ready) return;
  const uint32_t now_us = micros();
  float dt = float(now_us - g_last_imu_us) * 1.0e-6f;
  g_last_imu_us = now_us;
  if (dt <= 0.0f || dt > 0.25f) return;
  if (!qmi.getDataReady()) return;

  if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
    float z_dps = gyr.z * IMU_HEADING_SIGN;
    if (fabsf(z_dps) < 0.9f) z_dps = 0.0f;
    g_heading_deg = wrap360(g_heading_deg + z_dps * dt);
  }

  if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
    const float roll_deg = atan2f(acc.y, acc.z) * 57.29578f;
    const float pitch_deg = atan2f(-acc.x, sqrtf((acc.y * acc.y) + (acc.z * acc.z))) * 57.29578f;
    if (!g_level_valid) { g_level_x_deg = roll_deg; g_level_y_deg = pitch_deg; g_level_valid = true; } 
    else { g_level_x_deg = (0.88f * g_level_x_deg) + (0.12f * roll_deg); g_level_y_deg = (0.88f * g_level_y_deg) + (0.12f * pitch_deg); }

    if (!g_level_zero_set) { g_level_zero_x_deg = g_level_x_deg; g_level_zero_y_deg = g_level_y_deg; g_level_zero_set = true; }
    if (g_level_calibrating) {
      g_level_cal_sum_x += g_level_x_deg; g_level_cal_sum_y += g_level_y_deg; g_level_cal_count++;
      if (g_level_cal_count >= LEVEL_CAL_SAMPLES) {
        g_level_zero_x_deg = g_level_cal_sum_x / float(g_level_cal_count);
        g_level_zero_y_deg = g_level_cal_sum_y / float(g_level_cal_count);
        g_level_zero_set = true; g_level_calibrating = false;
        play_beep_tone(1320, 48, 0.22f);
      }
    }
  }
}

// --- MENU & STARTUP ---
static int startup_mode_hit_test(int16_t x, int16_t y) {
  const int rotate_w = 84; const int rotate_h = 28;
  const int rotate_x = gfx->width() - rotate_w - 8; const int rotate_y = 14;
  if (point_in_rect(x, y, rotate_x, rotate_y, rotate_w, rotate_h)) return 100;

  const int w = gfx->width();
  if (is_landscape_rotation(g_display_rotation)) {
    const int card_w = (w - 40) / 3;
    const int card_h = (gfx->height() - 126) / 2 - 5;
    const int y0 = 108;
    for (int i = 0; i < 3; ++i) if (point_in_rect(x, y, 8 + (i * (card_w + 12)), y0, card_w, card_h)) return i;
    for (int i = 0; i < 3; ++i) if (point_in_rect(x, y, 8 + (i * (card_w + 12)), y0 + card_h + 10, card_w, card_h)) return i+3;
  }
  return -1;
}

static void draw_startup_mode_menu(int highlighted_mode) {
  const int w = gfx->width();
  const int h = gfx->height();
  const bool landscape = is_landscape_rotation(g_display_rotation);

  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE); gfx->setTextSize(2);
  gfx->setCursor(16, 18); gfx->println("Select Tool");

  const int rotate_w = 84; const int rotate_h = 28;
  const int rotate_x = w - rotate_w - 8; const int rotate_y = 14;
  gfx->drawRoundRect(rotate_x, rotate_y, rotate_w, rotate_h, 6, LIGHTGREY);
  gfx->setTextSize(1); gfx->setCursor(rotate_x + 16, rotate_y + 10); gfx->print("ROTATE");

  const char *title[] = {"WiFi Scan", "BLE Scan", "NRF Spec", "Tri-Band", "Sniffer", "Level"};
  const char *sub[] = {"2.4GHz APs", "BLE Tags", "Spectrum", "All-Eye", "Packet Log", "Bubble"};
  const uint16_t fill_lo[] = {0x1082, 0x0822, 0x2104, MAROON, DARKGREEN, NAVY};
  const uint16_t fill_hi[] = {DARKGREEN, 0x03A8, ORANGE, RED, GREEN, BLUE};

  if (landscape) {
     const int card_w = (w - 40) / 3;
     const int card_h = (h - 126) / 2 - 5;
     const int y0 = 108;
     for(int i=0; i<6; i++) {
        int r = (i < 3) ? 0 : 1;
        int c = (i < 3) ? i : (i-3);
        int x = 8 + (c * (card_w + 12));
        int y = y0 + (r * (card_h + 10));
        bool hi = (highlighted_mode == i);
        gfx->fillRoundRect(x, y, card_w, card_h, 8, hi ? fill_hi[i] : fill_lo[i]);
        gfx->drawRoundRect(x, y, card_w, card_h, 8, WHITE);
        gfx->setTextSize(2); gfx->setTextColor(WHITE);
        gfx->setCursor(x+5, y+10); gfx->println(title[i]);
        gfx->setTextSize(1); gfx->setCursor(x+5, y+30); gfx->println(sub[i]);
     }
  }
}

static AppMode select_startup_mode_touch() {
  if (!g_touch_ready) return APP_WIFI_SCAN;
  bool menu_drawn = false;
  int highlighted = -1;
  bool was_pressed = false;
  int press_mode = -1;

  while (true) {
    int16_t tx = 0, ty = 0;
    const bool touched = read_touch_point(tx, ty);
    const int hit_mode = touched ? startup_mode_hit_test(tx, ty) : -1;
    if (!menu_drawn || (hit_mode != highlighted)) {
      highlighted = hit_mode;
      draw_startup_mode_menu(highlighted);
      menu_drawn = true;
    }
    if (touched) { was_pressed = true; press_mode = hit_mode; }
    else if (was_pressed) {
      was_pressed = false;
      if (press_mode == 100) { apply_display_rotation((g_display_rotation + 1) & 0x03, false); menu_drawn = false; press_mode = -1; continue; }
      if (press_mode >= 0 && press_mode <= 5) return (AppMode)press_mode;
    }
    delay(20);
  }
}

// --- TRI-BAND LOGIC ---
static void draw_tri_band_ui(bool full_redraw) {
  if (full_redraw) {
     gfx->fillScreen(BLACK);
     gfx->drawFastHLine(0, gfx->height()/3, gfx->width(), WHITE);
     gfx->drawFastHLine(0, (gfx->height()/3)*2, gfx->width(), WHITE);
     draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);
  }
  
  int w = gfx->width();
  int h_third = gfx->height() / 3;

  // 1. WiFi Zone (Top)
  static unsigned long last_wifi = 0;
  if (millis() - last_wifi > 5000) { WiFi.scanNetworks(true); last_wifi = millis(); }
  int n = WiFi.scanComplete();
  if (n >= 0) {
    gfx->fillRect(0, 30, w, h_third - 30, BLACK);
    gfx->setTextColor(GREEN); gfx->setTextSize(1);
    gfx->setCursor(5, 5); gfx->print("ZONE 1: Wi-Fi APs");
    for (int i=0; i<min(n, 5); i++) {
       gfx->setCursor(5, 25 + (i*15));
       gfx->print(WiFi.SSID(i).substring(0, 18)); gfx->print(" "); gfx->print(WiFi.RSSI(i));
    }
    WiFi.scanDelete();
  }

  // 2. BLE Zone (Middle)
  if (!g_ble_ready) ble_scanner_init();
  static unsigned long last_ble = 0;
  if (millis() - last_ble > 2000) {
    BLEScanResults r = g_ble_scan->start(1, false);
    gfx->fillRect(0, h_third + 20, w, h_third - 20, BLACK);
    gfx->setTextColor(CYAN);
    gfx->setCursor(5, h_third + 5); gfx->print("ZONE 2: BLE Devices");
    for (int i=0; i<min(r.getCount(), 5); i++) {
       BLEAdvertisedDevice d = r.getDevice(i);
       gfx->setCursor(5, h_third + 25 + (i*15));
       if (d.haveName()) gfx->print(d.getName().c_str()); else gfx->print(d.getAddress().toString().c_str());
       gfx->print(" "); gfx->print(d.getRSSI());
    }
    g_ble_scan->clearResults();
    last_ble = millis();
  }

  // 3. NRF Spectrum (Bottom)
  if (!g_nrf_ready) nrf_init();
  nrf_scan_channels();
  int base_y = h_third * 2;
  int chart_h = h_third - 20;
  int bar_w = w / NRF_NUM_CHANNELS;
  if (bar_w < 2) bar_w = 2;
  gfx->setTextColor(YELLOW); gfx->setCursor(5, base_y + 5); gfx->print("ZONE 3: 2.4GHz Spectrum");

  for (int i = 0; i < NRF_NUM_CHANNELS; i++) {
     int x = i * bar_w;
     int val = g_nrf_channel_buckets[i];
     int bar_h = map(val, 0, 255, 0, chart_h);
     uint16_t col = (val > 150) ? RED : (val > 50 ? YELLOW : DARKGREEN);
     gfx->drawFastVLine(x, base_y + 20, chart_h, BLACK);
     if (bar_h > 0) gfx->fillRect(x, (base_y + 20 + chart_h) - bar_h, bar_w-1, bar_h, col);
  }
}

// --- SNIFFER LOGIC ---
static void nrf_sniffer_loop() {
  if (!g_nrf_ready) nrf_init();
  for (int i=0; i<NRF_NUM_CHANNELS; i+=2) {
    radio.setChannel(i);
    radio.startListening();
    delayMicroseconds(120);
    bool hit = radio.testCarrier();
    radio.stopListening();
    if (hit) {
      String l = "EVENT: Ch " + String(i) + " [" + String(millis()) + "]";
      for(int k=MAX_SNIFFER_LOGS-1; k>0; k--) g_sniffer_logs[k] = g_sniffer_logs[k-1];
      g_sniffer_logs[0] = l;
      delay(2);
    }
  }
}

static void draw_sniffer_ui(bool full_redraw) {
  if (full_redraw) {
    gfx->fillScreen(BLACK);
    gfx->setTextColor(GREEN); gfx->setTextSize(2);
    gfx->setCursor(10, 10); gfx->print("HACKER TERMINAL");
    gfx->drawFastHLine(0, 35, gfx->width(), GREEN);
    draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);
  }
  nrf_sniffer_loop();
  gfx->setTextSize(2);
  for (int i=0; i<MAX_SNIFFER_LOGS; i++) {
    int y = 50 + (i * 25);
    gfx->fillRect(0, y, gfx->width(), 22, BLACK);
    uint16_t col = (i==0) ? GREEN : 0x0600;
    gfx->setTextColor(col);
    gfx->setCursor(10, y);
    gfx->print(g_sniffer_logs[i]);
  }
}

// --- NRF SCANNER LOGIC ---
static void nrf_scan_channels() {
  if (!g_nrf_ready) nrf_init();
  for (int i = 0; i < 128; i++) {
    radio.setChannel(i);
    radio.startListening();
    delayMicroseconds(50);
    if (radio.testCarrier()) { if (g_nrf_channel_buckets[i] < 250) g_nrf_channel_buckets[i] += 10; } 
    else { if (g_nrf_channel_buckets[i] > 2) g_nrf_channel_buckets[i] -= 3; }
    radio.stopListening();
  }
}

static void draw_nrf_ui(bool full_redraw) {
  if (full_redraw) {
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE); gfx->setTextSize(2); gfx->setCursor(8, 8); gfx->println("2.4GHz Spectrum");
    draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);
    gfx->drawFastHLine(10, gfx->height() - 20, 300, WHITE);
    gfx->drawFastVLine(10, 60, gfx->height() - 80, WHITE);
    gfx->setTextSize(1); gfx->setTextColor(LIGHTGREY);
    gfx->setCursor(10, gfx->height() - 10); gfx->print("2.400");
    gfx->setCursor(260, gfx->height() - 10); gfx->print("2.525");
  }
  int chart_bottom = gfx->height() - 21;
  int chart_h = gfx->height() - 100;
  for (int i = 0; i < 128; i++) {
    int x = 12 + (i * 2.3);
    int h = map(g_nrf_channel_buckets[i], 0, 255, 0, chart_h);
    uint16_t color = (h > chart_h/2) ? RED : (h > 10 ? YELLOW : GREEN);
    gfx->drawFastVLine(x, chart_bottom - chart_h, chart_h, BLACK);
    gfx->drawFastVLine(x+1, chart_bottom - chart_h, chart_h, BLACK);
    if (h > 0) gfx->fillRect(x, chart_bottom - h, 2, h, color);
  }
}

// --- DEVICE MANAGEMENT ---
static int find_device_slot(SignalSource source, const String &id) {
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i) {
    if (g_devices[i].used && g_devices[i].source == source && g_devices[i].bssid.equalsIgnoreCase(id)) return i;
  }
  return -1;
}

static int alloc_device_slot(uint32_t now_ms) {
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i) if (!g_devices[i].used) return i;
  int replace_idx = 0; uint32_t oldest_age = 0;
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i) {
    const uint32_t age = now_ms - g_devices[i].last_seen_ms;
    if (age > oldest_age) { oldest_age = age; replace_idx = i; }
  }
  return replace_idx;
}

static void upsert_device(SignalSource source, const String &name, const String &id, int32_t channel, float raw_rssi, uint32_t now_ms) {
  int slot = find_device_slot(source, id);
  if (slot < 0) {
    slot = alloc_device_slot(now_ms);
    if (slot == g_selected_device) g_selected_device = -1;
    g_devices[slot] = RadarDevice(); g_devices[slot].used = true; g_devices[slot].source = source; g_devices[slot].bssid = id;
  }
  RadarDevice &d = g_devices[slot];
  d.seen_this_scan = true; d.visible = true; d.last_seen_ms = now_ms; d.source = source; d.ssid = name; d.channel = channel;
  d.raw_rssi = raw_rssi;
  if (!d.has_rssi) { d.smooth_rssi = raw_rssi; d.prev_smooth_rssi = raw_rssi; d.trend = TREND_UNKNOWN; d.has_rssi = true; } 
  else { d.prev_smooth_rssi = d.smooth_rssi; d.smooth_rssi = (0.74f * d.smooth_rssi) + (0.26f * raw_rssi);
         const float delta = d.smooth_rssi - d.prev_smooth_rssi;
         if (delta > 1.0f) d.trend = TREND_CLOSER; else if (delta < -1.0f) d.trend = TREND_FARTHER; else d.trend = TREND_STABLE; }
  d.signal_01 = rssi_to_signal01(d.smooth_rssi);
}

static void scan_wifi_devices(uint32_t now_ms) {
  const int16_t count = WiFi.scanComplete();
  if (count == WIFI_SCAN_RUNNING) return;
  if (count == WIFI_SCAN_FAILED) { WiFi.scanDelete(); WiFi.scanNetworks(true, true, false, 90); return; }
  for (int i = 0; i < count; ++i) {
    String bssid = WiFi.BSSIDstr(i);
    if (has_cfg_target_bssid() && !bssid_equals_ignore_case(bssid, TARGET_BSSID)) continue;
    String ssid = WiFi.SSID(i);
    if (has_cfg_target_ssid() && !ssid.equals(TARGET_SSID)) continue;
    upsert_device(SIGNAL_WIFI, ssid, bssid, WiFi.channel(i), float(WiFi.RSSI(i)), now_ms);
  }
  WiFi.scanDelete(); WiFi.scanNetworks(true, true, false, 90);
}

static void scan_ble_devices(uint32_t now_ms) {
  if (!g_ble_ready || !g_ble_scan) return;
  BLEScanResults results = g_ble_scan->start(BLE_SCAN_SECONDS, false);
  for (int i = 0; i < results.getCount(); ++i) {
    BLEAdvertisedDevice dev = results.getDevice(i);
    upsert_device(SIGNAL_BLE, dev.haveName() ? String(dev.getName().c_str()) : "", String(dev.getAddress().toString().c_str()), 0, float(dev.getRSSI()), now_ms);
  }
  g_ble_scan->clearResults();
}

static void refresh_visibility_and_sort(uint32_t now_ms) {
  g_visible_count = 0; g_sorted_count = 0;
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i) {
    if (!g_devices[i].used) continue;
    if ((now_ms - g_devices[i].last_seen_ms) > DEVICE_PRUNE_MS) { g_devices[i] = RadarDevice(); if (g_selected_device == i) g_selected_device = -1; continue; }
    g_devices[i].visible = ((now_ms - g_devices[i].last_seen_ms) <= DEVICE_HIDE_MS);
    if (g_devices[i].visible) { g_sorted_indices[g_sorted_count++] = i; g_visible_count++; }
  }
  for (int i = 0; i < g_sorted_count - 1; ++i) {
    for (int j = i + 1; j < g_sorted_count; ++j) {
      if (g_devices[g_sorted_indices[j]].signal_01 > g_devices[g_sorted_indices[i]].signal_01) { int t = g_sorted_indices[i]; g_sorted_indices[i] = g_sorted_indices[j]; g_sorted_indices[j] = t; }
    }
  }
}

static void process_scan_results(uint32_t now_ms) {
  if (scan_mode_uses_wifi()) scan_wifi_devices(now_ms);
  if (scan_mode_uses_ble() && g_ble_ready && !g_touch_was_pressed && ((now_ms - g_last_ble_scan_ms) >= BLE_SCAN_INTERVAL_MS)) { scan_ble_devices(now_ms); g_last_ble_scan_ms = now_ms; }
  refresh_visibility_and_sort(now_ms);
}

static bool ble_scanner_init() {
  if (g_ble_ready) return true;
  BLEDevice::init("");
  g_ble_scan = BLEDevice::getScan();
  if (!g_ble_scan) return false;
  g_ble_scan->setActiveScan(true); g_ble_scan->setInterval(120); g_ble_scan->setWindow(90);
  return true;
}

static bool audio_beeper_init() {
  if (g_audio_ready) return true;
  if (g_es8311) es8311_delete(g_es8311);
  if (g_i2s_ready) { i2s_driver_uninstall(I2S_PORT); }
  i2s_config_t i2s_cfg = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = AUDIO_SAMPLE_HZ, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 6, .dma_buf_len = 256, .use_apll = false, .tx_desc_auto_clear = true, .fixed_mclk = AUDIO_MCLK_HZ };
  if (i2s_driver_install(I2S_PORT, &i2s_cfg, 0, nullptr) != ESP_OK) return false;
  g_i2s_ready = true;
  i2s_pin_config_t pin_cfg = { .bck_io_num = I2S_BCK_PIN, .ws_io_num = I2S_LRCK_PIN, .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_DIN_PIN }; pin_cfg.mck_io_num = I2S_MCK_PIN;
  i2s_set_pin(I2S_PORT, &pin_cfg);
  g_es8311 = es8311_create((i2c_port_t)0, ES8311_ADDRRES_0);
  es8311_clock_config_t clk_cfg = { .mclk_inverted = false, .sclk_inverted = false, .mclk_from_mclk_pin = true, .mclk_frequency = AUDIO_MCLK_HZ, .sample_frequency = AUDIO_SAMPLE_HZ };
  es8311_init(g_es8311, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
  es8311_voice_volume_set(g_es8311, 70, nullptr); es8311_voice_mute(g_es8311, false);
  g_audio_ready = true; return true;
}

static bool ensure_audio_ready(uint32_t now_ms, bool force = false) {
  if (g_audio_ready) return true;
  if (!force && ((now_ms - g_last_audio_init_attempt_ms) < 2500)) return false;
  g_last_audio_init_attempt_ms = now_ms;
  return audio_beeper_init();
}

static void play_beep_tone(uint16_t freq_hz, uint16_t duration_ms, float gain) {
  if (freq_hz == 0 || duration_ms == 0) return;
  if (!ensure_audio_ready(millis())) return;
  const int total_frames = (AUDIO_SAMPLE_HZ * int(duration_ms)) / 1000;
  const int chunk_frames = 128;
  int16_t pcm[chunk_frames * 2];
  float phase = 0.0f;
  const float phase_inc = (2.0f * PI * freq_hz) / AUDIO_SAMPLE_HZ;
  int remaining = total_frames;
  while (remaining > 0) {
    const int frames = (remaining > chunk_frames) ? chunk_frames : remaining;
    for (int i = 0; i < frames; ++i) {
      const int16_t sample = (int16_t)(sinf(phase) * 32767.0f * gain);
      phase += phase_inc; if (phase > 2.0f * PI) phase -= 2.0f * PI;
      pcm[2 * i] = sample; pcm[2 * i + 1] = sample;
    }
    size_t written; i2s_write(I2S_PORT, pcm, frames * 4, &written, portMAX_DELAY);
    remaining -= frames;
  }
}

static int device_from_touch(int16_t x, int16_t y) {
  const int rows = radar_rows_visible();
  if (!point_in_rect(x, y, RADAR_LIST_X, RADAR_LIST_Y, RADAR_LIST_W, RADAR_ROW_H * rows)) return -1;
  const int row = (y - RADAR_LIST_Y) / RADAR_ROW_H;
  return (row >= 0 && row < rows) ? g_row_to_device[row] : -1;
}

static void handle_radar_touch_release(int16_t x, int16_t y) {
  int hit = device_from_touch(x, y);
  if (hit < 0 && g_touch_press_device >= 0) hit = g_touch_press_device;
  if (hit < 0) return;
  if (g_selected_device == hit) { g_selected_device = -1; g_last_tap_device = hit; g_last_tap_selected = false; play_beep_tone(500, 36, 0.20f); } 
  else { g_selected_device = hit; g_last_tap_device = hit; g_last_tap_selected = true; play_beep_tone(1180, 48, 0.24f); }
  g_last_tap_feedback_until_ms = millis() + 850;
}

static void update_beeper(uint32_t now_ms) {
  if (g_selected_device < 0 || !g_devices[g_selected_device].visible) return;
  const RadarDevice &d = g_devices[g_selected_device];
  uint32_t interval = 980 - (d.signal_01 * 780);
  if (interval < 110) interval = 110;
  if ((now_ms - g_last_beep_ms) < interval) return;
  uint16_t freq = 620 + (d.signal_01 * 1380);
  if (d.trend == TREND_CLOSER) freq += 180; else if (d.trend == TREND_FARTHER && freq > 140) freq -= 180;
  play_beep_tone(freq, 20 + (d.signal_01 * 44), 0.18f + (d.signal_01 * 0.14f));
  g_last_beep_ms = now_ms;
}

static void draw_level_grid(int cx, int cy, int r) {
  gfx->drawCircle(cx, cy, r, CYAN); if (r > 55) gfx->drawCircle(cx, cy, r - 45, DARKGREY);
  gfx->drawFastHLine(cx - r, cy, r * 2, DARKGREY); gfx->drawFastVLine(cx, cy - r, r * 2, DARKGREY);
}

static void draw_level_only_ui() {
  const int w = gfx->width(), h = gfx->height();
  const int cx = w / 2, cy = 84 + (h - 124) / 2;
  int r = (h - 124) / 2 - 8; if (r > (w / 2 - 10)) r = w / 2 - 10;

  if (!g_level_ui_static_ready) {
    gfx->fillScreen(BLACK); gfx->setTextColor(WHITE); gfx->setTextSize(2); gfx->setCursor(8, 8); gfx->println("Level Mode");
    gfx->setTextSize(1); gfx->setTextColor(LIGHTGREY); gfx->setCursor(8, 30); gfx->println("BOOT: recenter level | Touch CAL");
    g_level_ui_static_ready = true; g_level_prev_dot_valid = false;
    gfx->fillCircle(cx, cy, r + 1, BLACK); draw_level_grid(cx, cy, r);
  }
  draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);
  draw_touch_button(BTN_CAL_X, BTN_CAL_Y, BTN_CAL_W, BTN_CAL_H, "CAL", g_level_calibrating ? DARKGREEN : 0x1082, g_level_calibrating ? GREEN : LIGHTGREY, WHITE);
  
  if (g_level_valid) {
    const float rel_x = g_level_x_deg - g_level_zero_x_deg; const float rel_y = g_level_y_deg - g_level_zero_y_deg;
    float nx = clamp01((rel_x / 25.0f + 1.0f) * 0.5f) * 2.0f - 1.0f; float ny = clamp01((rel_y / 25.0f + 1.0f) * 0.5f) * 2.0f - 1.0f;
    const int dx = cx + int(nx * (r - 18)), dy = cy + int(ny * (r - 18));
    const uint16_t c = (sqrtf(rel_x*rel_x + rel_y*rel_y) < 2.0f) ? GREEN : YELLOW;
    if (!g_level_prev_dot_valid || dx != g_level_prev_dot_x || dy != g_level_prev_dot_y || c != g_level_prev_dot_color) {
      if (g_level_prev_dot_valid) { gfx->fillCircle(g_level_prev_dot_x, g_level_prev_dot_y, 13, BLACK); draw_level_grid(cx, cy, r); }
      gfx->fillCircle(dx, dy, 10, c); gfx->drawCircle(dx, dy, 12, WHITE);
      g_level_prev_dot_x = dx; g_level_prev_dot_y = dy; g_level_prev_dot_color = c; g_level_prev_dot_valid = true;
    }
  }
}

static void draw_radar_ui() {
  if (!g_radar_ui_static_ready) {
    gfx->fillScreen(BLACK); gfx->setTextColor(LIGHTGREY); gfx->setTextSize(1);
    gfx->setCursor(8, gfx->height() - 28); gfx->println("Beep rises with signal");
    gfx->setCursor(8, gfx->height() - 14); gfx->println("BACK: menu | BOOT: clear");
    g_radar_ui_static_ready = true; g_radar_ui_has_sig = false;
  }
  const int rows = radar_rows_visible();
  const int shown = (g_sorted_count < rows) ? g_sorted_count : rows;
  gfx->fillRect(0, 0, gfx->width(), 90, BLACK);
  gfx->setTextColor(WHITE); gfx->setTextSize(2); gfx->setCursor(8, 8); gfx->println(app_mode_title(g_app_mode));
  draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);
  gfx->setTextSize(1); gfx->setTextColor(CYAN); gfx->setCursor(8, 30); gfx->printf("%s devices: %d", scan_mode_uses_ble() ? "BLE" : "WiFi", g_visible_count);
  if (g_selected_device >= 0 && g_devices[g_selected_device].used) {
    gfx->setTextColor(YELLOW); gfx->setCursor(8, 56); gfx->printf("Target: %s RSSI %.0f", g_devices[g_selected_device].ssid.c_str(), g_devices[g_selected_device].smooth_rssi);
  }
  gfx->fillRect(RADAR_LIST_X - 2, RADAR_LIST_Y - 2, RADAR_LIST_W + 4, (RADAR_ROW_H * rows) + 4, BLACK);
  for (int row = 0; row < shown; ++row) {
    const int idx = g_sorted_indices[row]; g_row_to_device[row] = idx;
    const RadarDevice &d = g_devices[idx];
    int y = RADAR_LIST_Y + (row * RADAR_ROW_H); bool sel = (idx == g_selected_device);
    gfx->fillRoundRect(RADAR_LIST_X, y, RADAR_LIST_W, RADAR_ROW_H - 4, 6, sel ? 0x0841 : 0x0000);
    gfx->drawRoundRect(RADAR_LIST_X, y, RADAR_LIST_W, RADAR_ROW_H - 4, 6, sel ? YELLOW : DARKGREY);
    gfx->setTextSize(2); gfx->setTextColor(source_to_color(d.source));
    gfx->setCursor(RADAR_LIST_X + 8, y + 6); gfx->printf("%d %s", row + 1, d.ssid.substring(0, 10).c_str());
    gfx->setTextSize(1); gfx->setTextColor(WHITE);
    gfx->setCursor(RADAR_LIST_X + 8, y + 30); gfx->printf("RSSI %.0f %s", d.smooth_rssi, trend_to_text(d.trend));
  }
}

// --- APP STATE MANAGEMENT ---
static void apply_app_mode(AppMode mode) {
  g_app_mode = mode;
  g_touch_was_pressed = false;
  reset_ui_draw_state();
  if (mode == APP_NRF) { WiFi.mode(WIFI_OFF); nrf_init(); draw_nrf_ui(true); }
  else if (mode == APP_TRI_BAND) { nrf_init(); draw_tri_band_ui(true); }
  else if (mode == APP_SNIFFER) { nrf_init(); draw_sniffer_ui(true); }
  else if (mode == APP_LEVEL_ONLY) { WiFi.mode(WIFI_OFF); draw_level_only_ui(); }
  else {
    if (mode == APP_BLE_SCAN && !g_ble_init_attempted) { g_ble_init_attempted=true; ble_scanner_init(); }
    if (mode == APP_WIFI_SCAN) { WiFi.mode(WIFI_STA); WiFi.disconnect(); WiFi.scanNetworks(true); }
    draw_radar_ui();
  }
}

static void switch_mode_from_touch_menu() {
  if (!g_touch_ready) return;
  const AppMode selected = select_startup_mode_touch();
  apply_app_mode(selected);
}

// --- MAIN ---
void setup() {
  Serial.begin(115200);
  tca9554_init_and_lcd_reset();
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  gfx->begin(40000000);
  gfx->setRotation(g_display_rotation);
  update_ui_layout_for_rotation();
  g_touch_ready = touch_init();
  g_imu_ready = imu_init();
  g_app_mode = select_startup_mode_touch();
  apply_app_mode(g_app_mode);
}

void loop() {
  const uint32_t now = millis();
  update_imu_heading();

  if (g_touch_ready && ((now - g_mode_enter_ms) >= MODE_TOUCH_GUARD_MS)) {
    int16_t tx, ty;
    if (read_touch_point(tx, ty)) {
       g_touch_was_pressed = true; g_touch_last_x = tx; g_touch_last_y = ty;
       if (point_in_rect(tx, ty, BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H)) { switch_mode_from_touch_menu(); return; }
    } else if (g_touch_was_pressed) {
       g_touch_was_pressed = false;
       if (g_app_mode == APP_WIFI_SCAN || g_app_mode == APP_BLE_SCAN) handle_radar_touch_release(g_touch_last_x, g_touch_last_y);
    }
  }

  if (g_app_mode == APP_NRF) {
     nrf_scan_channels();
     if ((now - g_last_frame_ms) > 50) { draw_nrf_ui(false); g_last_frame_ms = now; }
  }
  else if (g_app_mode == APP_TRI_BAND) { draw_tri_band_ui(false); }
  else if (g_app_mode == APP_SNIFFER) { draw_sniffer_ui(false); }
  else if (g_app_mode == APP_LEVEL_ONLY) {
     if ((now - g_last_frame_ms) > FRAME_INTERVAL_MS) { draw_level_only_ui(); g_last_frame_ms = now; }
  }
  else {
     if ((now - g_last_scan_ms) >= SCAN_INTERVAL_SIGNAL_FINDER_MS) { process_scan_results(now); g_last_scan_ms = now; }
     if ((now - g_last_frame_ms) >= FRAME_INTERVAL_MS) { draw_radar_ui(); g_last_frame_ms = now; }
     update_beeper(now);
  }
  delay(1);
}