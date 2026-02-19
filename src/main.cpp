#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <Arduino_GFX_Library.h>
#include "TCA9554.h"
#include "es8311.h"
#include "SensorQMI8658.hpp"
#include "TouchDrvFT6X36.hpp"

extern "C" {
#include "driver/i2s.h"
}

// Waveshare ESP32-S3-Touch-LCD-3.5-C
// ST7796 over SPI + TCA9554 for BL/RST + ES8311 audio for beeps.

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

// ES8311 + I2S pins from Waveshare demo.
#define I2S_PORT        I2S_NUM_0
#define I2S_MCK_PIN     12
#define I2S_BCK_PIN     13
#define I2S_LRCK_PIN    15
#define I2S_DOUT_PIN    16
#define I2S_DIN_PIN     14
#define AUDIO_SAMPLE_HZ 16000
#define AUDIO_MCLK_HZ   (AUDIO_SAMPLE_HZ * 256)

#define IMU_HEADING_SIGN -1.0f

// Optional scan filters. Leave empty to include all visible APs.
static const char *TARGET_SSID = "";
static const char *TARGET_BSSID = "";

static const int RSSI_NEAR_DBM = -45;
static const int RSSI_FAR_DBM = -95;
static const uint32_t SCAN_INTERVAL_MS = 1200;
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

static const int BTN_BACK_X = 236;
static const int BTN_BACK_Y = 6;
static const int BTN_BACK_W = 76;
static const int BTN_BACK_H = 28;
static const int BTN_SRC_X = 142;
static const int BTN_SRC_Y = 6;
static const int BTN_SRC_W = 90;
static const int BTN_SRC_H = 28;
static const int BTN_CAL_X = 236;
static const int BTN_CAL_Y = 38;
static const int BTN_CAL_W = 76;
static const int BTN_CAL_H = 28;
static const int BTN_LBEEP_X = 236;
static const int BTN_LBEEP_Y = 70;
static const int BTN_LBEEP_W = 76;
static const int BTN_LBEEP_H = 28;
static const int RADAR_LIST_X = 8;
static const int RADAR_LIST_Y = 96;
static const int RADAR_LIST_W = 304;
static const int RADAR_ROW_H = 44;

enum AppMode : uint8_t
{
  APP_WIFI_RADAR = 0,
  APP_LEVEL_ONLY = 1
};

enum TrendState : uint8_t
{
  TREND_UNKNOWN = 0,
  TREND_CLOSER,
  TREND_FARTHER,
  TREND_STABLE
};

enum SignalSource : uint8_t
{
  SIGNAL_WIFI = 0,
  SIGNAL_BLE = 1
};

enum LevelBeepMode : uint8_t
{
  LEVEL_BEEP_OFF = 0,
  LEVEL_BEEP_WHEN_LEVEL = 1,
  LEVEL_BEEP_WHEN_NOT_LEVEL = 2
};

enum ScanSourceMode : uint8_t
{
  SCAN_WIFI_ONLY = 0,
  SCAN_WIFI_BLE = 1,
  SCAN_BLE_ONLY = 2
};

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

  int16_t draw_x = 0;
  int16_t draw_y = 0;
};

TCA9554 tca(TCA_ADDR);
SensorQMI8658 qmi;
TouchDrvFT6X36 touch;
IMUdata acc;
IMUdata gyr;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
    LCD_DC /* DC */, LCD_CS /* CS */, SPI_SCLK /* SCK */, SPI_MOSI /* MOSI */, SPI_MISO /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7796(
    bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */, LCD_HOR_RES, LCD_VER_RES);

static es8311_handle_t g_es8311 = nullptr;
static bool g_audio_ready = false;
static bool g_i2s_ready = false;
static bool g_imu_ready = false;
static bool g_touch_ready = false;
static bool g_ble_ready = false;
static bool g_ble_init_attempted = false;
static BLEScan *g_ble_scan = nullptr;
static AppMode g_app_mode = APP_WIFI_RADAR;
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

static void play_beep_tone(uint16_t freq_hz, uint16_t duration_ms, float gain);
static bool ble_scanner_init();

static float clamp01(float v)
{
  if (v < 0.0f)
    return 0.0f;
  if (v > 1.0f)
    return 1.0f;
  return v;
}

static float wrap360(float deg)
{
  while (deg < 0.0f)
    deg += 360.0f;
  while (deg >= 360.0f)
    deg -= 360.0f;
  return deg;
}

static float rssi_to_signal01(float rssi)
{
  const float denom = float(RSSI_NEAR_DBM - RSSI_FAR_DBM);
  if (denom <= 0.0f)
    return 0.0f;
  return clamp01((rssi - float(RSSI_FAR_DBM)) / denom);
}

static const char *trend_to_text(TrendState trend)
{
  switch (trend)
  {
    case TREND_CLOSER: return "closer";
    case TREND_FARTHER: return "farther";
    case TREND_STABLE: return "stable";
    default: return "unknown";
  }
}

static uint16_t trend_to_color(TrendState trend)
{
  switch (trend)
  {
    case TREND_CLOSER: return GREEN;
    case TREND_FARTHER: return ORANGE;
    case TREND_STABLE: return CYAN;
    default: return LIGHTGREY;
  }
}

static const char *source_to_text(SignalSource source)
{
  return (source == SIGNAL_BLE) ? "BLE" : "WiFi";
}

static uint16_t source_to_color(SignalSource source)
{
  return (source == SIGNAL_BLE) ? MAGENTA : CYAN;
}

static const char *level_beep_mode_text(LevelBeepMode mode)
{
  switch (mode)
  {
    case LEVEL_BEEP_WHEN_LEVEL: return "LVL";
    case LEVEL_BEEP_WHEN_NOT_LEVEL: return "NOT";
    default: return "OFF";
  }
}

static const char *scan_mode_text(ScanSourceMode mode)
{
  switch (mode)
  {
    case SCAN_WIFI_BLE: return "BOTH";
    case SCAN_BLE_ONLY: return "BLE";
    default: return "WIFI";
  }
}

static bool scan_mode_uses_wifi()
{
  return g_scan_source_mode != SCAN_BLE_ONLY;
}

static bool scan_mode_uses_ble()
{
  return g_scan_source_mode != SCAN_WIFI_ONLY;
}

static bool has_cfg_target_bssid()
{
  return (TARGET_BSSID != nullptr) && (TARGET_BSSID[0] != '\0');
}

static bool has_cfg_target_ssid()
{
  return (TARGET_SSID != nullptr) && (TARGET_SSID[0] != '\0');
}

static bool bssid_equals_ignore_case(const String &a, const char *b)
{
  if (b == nullptr || b[0] == '\0')
    return false;
  String rhs(b);
  return a.equalsIgnoreCase(rhs);
}

static bool point_in_rect(int16_t x, int16_t y, int rx, int ry, int rw, int rh)
{
  return (x >= rx) && (x < (rx + rw)) && (y >= ry) && (y < (ry + rh));
}

static void draw_touch_button(int x, int y, int w, int h, const char *label, uint16_t fill, uint16_t border, uint16_t text)
{
  gfx->fillRoundRect(x, y, w, h, 6, fill);
  gfx->drawRoundRect(x, y, w, h, 6, border);
  gfx->setTextColor(text);
  gfx->setTextSize(1);
  const int text_x = x + (w / 2) - (int(strlen(label)) * 3);
  const int text_y = y + (h / 2) - 3;
  gfx->setCursor(text_x, text_y);
  gfx->print(label);
}

static void start_level_calibration()
{
  g_level_calibrating = true;
  g_level_cal_count = 0;
  g_level_cal_sum_x = 0.0f;
  g_level_cal_sum_y = 0.0f;
}

static void cycle_level_beep_mode()
{
  if (g_level_beep_mode == LEVEL_BEEP_OFF)
    g_level_beep_mode = LEVEL_BEEP_WHEN_LEVEL;
  else if (g_level_beep_mode == LEVEL_BEEP_WHEN_LEVEL)
    g_level_beep_mode = LEVEL_BEEP_WHEN_NOT_LEVEL;
  else
    g_level_beep_mode = LEVEL_BEEP_OFF;
}

static void tca9554_init_and_lcd_reset()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  tca.begin();

  tca.pinMode1(TCA_PIN_BL, OUTPUT);
  tca.pinMode1(TCA_PIN_RST, OUTPUT);

  tca.write1(TCA_PIN_BL, 1);
  pinMode(GPIO_BL, OUTPUT);
  digitalWrite(GPIO_BL, HIGH);

  tca.write1(TCA_PIN_RST, 1);
  delay(10);
  tca.write1(TCA_PIN_RST, 0);
  delay(10);
  tca.write1(TCA_PIN_RST, 1);
  delay(120);
}

static bool imu_init()
{
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS))
  {
    Serial.println("QMI8658 init failed");
    return false;
  }

  qmi.configGyroscope(
      SensorQMI8658::GYR_RANGE_512DPS,
      SensorQMI8658::GYR_ODR_112_1Hz,
      SensorQMI8658::LPF_MODE_3);
  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_125Hz,
      SensorQMI8658::LPF_MODE_0);
  qmi.enableGyroscope();
  qmi.enableAccelerometer();
  g_last_imu_us = micros();
  return true;
}

static bool touch_init()
{
  if (!touch.begin(Wire, FT6X36_SLAVE_ADDRESS))
  {
    Serial.println("FT6X36 touch init failed");
    return false;
  }
  return true;
}

static void update_imu_heading()
{
  if (!g_imu_ready)
    return;

  const uint32_t now_us = micros();
  float dt = float(now_us - g_last_imu_us) * 1.0e-6f;
  g_last_imu_us = now_us;

  if (dt <= 0.0f || dt > 0.25f)
    return;

  if (!qmi.getDataReady())
    return;

  if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z))
  {
    float z_dps = gyr.z * IMU_HEADING_SIGN;
    if (fabsf(z_dps) < 0.9f)
    {
      z_dps = 0.0f;
    }
    g_heading_deg = wrap360(g_heading_deg + z_dps * dt);
  }

  if (qmi.getAccelerometer(acc.x, acc.y, acc.z))
  {
    const float ax = acc.x;
    const float ay = acc.y;
    const float az = acc.z;

    const float roll_deg = atan2f(ay, az) * 57.29578f;
    const float pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 57.29578f;

    if (!g_level_valid)
    {
      g_level_x_deg = roll_deg;
      g_level_y_deg = pitch_deg;
      g_level_valid = true;
    }
    else
    {
      g_level_x_deg = (0.88f * g_level_x_deg) + (0.12f * roll_deg);
      g_level_y_deg = (0.88f * g_level_y_deg) + (0.12f * pitch_deg);
    }

    if (!g_level_zero_set)
    {
      g_level_zero_x_deg = g_level_x_deg;
      g_level_zero_y_deg = g_level_y_deg;
      g_level_zero_set = true;
    }

    if (g_level_calibrating)
    {
      g_level_cal_sum_x += g_level_x_deg;
      g_level_cal_sum_y += g_level_y_deg;
      g_level_cal_count++;
      if (g_level_cal_count >= LEVEL_CAL_SAMPLES)
      {
        const float inv_count = 1.0f / float(g_level_cal_count);
        g_level_zero_x_deg = g_level_cal_sum_x * inv_count;
        g_level_zero_y_deg = g_level_cal_sum_y * inv_count;
        g_level_zero_set = true;
        g_level_calibrating = false;
        play_beep_tone(1320, 48, 0.22f);
      }
    }
  }
}

static void clear_direction_map()
{
  g_heading_deg = 0.0f;
}

static void clear_tracked_devices()
{
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i)
  {
    g_devices[i] = RadarDevice();
  }
  for (int i = 0; i < MAX_LIST_ROWS; ++i)
  {
    g_row_to_device[i] = -1;
  }
  g_visible_count = 0;
  g_sorted_count = 0;
  g_selected_device = -1;
  g_touch_press_device = -1;
  g_last_tap_device = -1;
  g_last_tap_feedback_until_ms = 0;
}

static void apply_scan_source_mode(uint32_t now_ms)
{
  if (scan_mode_uses_ble() && !g_ble_ready)
  {
    if (!g_ble_init_attempted)
    {
      g_ble_init_attempted = true;
      g_ble_ready = ble_scanner_init();
      Serial.printf("BLE scan: %s\n", g_ble_ready ? "READY" : "OFF");
    }
  }

  if (scan_mode_uses_wifi())
  {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    WiFi.setSleep(false);
    WiFi.scanDelete();
    WiFi.scanNetworks(true, true, false, 90);
  }
  else
  {
    WiFi.scanDelete();
    WiFi.mode(WIFI_OFF);
  }

  g_last_ble_scan_ms = 0;
  g_last_scan_ms = now_ms - SCAN_INTERVAL_SIGNAL_FINDER_MS;
}

static void cycle_scan_source_mode()
{
  if (g_scan_source_mode == SCAN_WIFI_ONLY)
    g_scan_source_mode = SCAN_WIFI_BLE;
  else if (g_scan_source_mode == SCAN_WIFI_BLE)
    g_scan_source_mode = SCAN_BLE_ONLY;
  else
    g_scan_source_mode = SCAN_WIFI_ONLY;

  if (scan_mode_uses_ble() && !g_ble_ready)
    g_ble_init_attempted = false;

  clear_tracked_devices();
  apply_scan_source_mode(millis());
  Serial.printf("Signal source mode: %s\n", scan_mode_text(g_scan_source_mode));
  play_beep_tone(820, 32, 0.20f);
}

static int startup_mode_hit_test(int16_t x, int16_t y)
{
  const int box_x = 24;
  const int box_w = 272;
  const int box_h = 110;
  const int radar_y = 140;
  const int level_y = 272;

  if (x >= box_x && x < (box_x + box_w))
  {
    if (y >= radar_y && y < (radar_y + box_h))
      return int(APP_WIFI_RADAR);
    if (y >= level_y && y < (level_y + box_h))
      return int(APP_LEVEL_ONLY);
  }
  return -1;
}

static void draw_startup_mode_menu(int highlighted_mode)
{
  const int box_x = 24;
  const int box_w = 272;
  const int box_h = 110;
  const int radar_y = 140;
  const int level_y = 272;

  const bool radar_hi = (highlighted_mode == int(APP_WIFI_RADAR));
  const bool level_hi = (highlighted_mode == int(APP_LEVEL_ONLY));

  gfx->fillScreen(BLACK);

  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(28, 20);
  gfx->println("Select Startup Mode");

  gfx->setTextSize(1);
  gfx->setTextColor(LIGHTGREY);
  gfx->setCursor(28, 48);
  gfx->println("Tap a card to start");
  gfx->setCursor(28, 62);
  gfx->println("Waiting for selection...");

  uint16_t radar_fill = radar_hi ? DARKGREEN : 0x1082;
  uint16_t level_fill = level_hi ? 0x0410 : 0x1082;
  uint16_t radar_border = radar_hi ? GREEN : LIGHTGREY;
  uint16_t level_border = level_hi ? CYAN : LIGHTGREY;

  gfx->fillRoundRect(box_x, radar_y, box_w, box_h, 12, radar_fill);
  gfx->drawRoundRect(box_x, radar_y, box_w, box_h, 12, radar_border);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(box_x + 18, radar_y + 30);
  gfx->println("Signal Finder");
  gfx->setTextSize(1);
  gfx->setCursor(box_x + 18, radar_y + 66);
  gfx->println("List BLE+Wi-Fi signals, tap to guide by beeps");

  gfx->fillRoundRect(box_x, level_y, box_w, box_h, 12, level_fill);
  gfx->drawRoundRect(box_x, level_y, box_w, box_h, 12, level_border);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(box_x + 18, level_y + 30);
  gfx->println("Level Mode");
  gfx->setTextSize(1);
  gfx->setCursor(box_x + 18, level_y + 66);
  gfx->println("Full-screen leveling bubble only");
}

static AppMode select_startup_mode_touch()
{
  if (!g_touch_ready)
  {
    return APP_WIFI_RADAR;
  }

  bool menu_drawn = false;
  int highlighted = -1;
  bool was_pressed = false;
  int press_mode = -1;

  while (true)
  {
    int16_t tx[1] = {0};
    int16_t ty[1] = {0};
    const bool touched = touch.getPoint(tx, ty, 1) > 0;
    const int hit_mode = touched ? startup_mode_hit_test(tx[0], ty[0]) : -1;

    if (!menu_drawn || (hit_mode != highlighted))
    {
      highlighted = hit_mode;
      draw_startup_mode_menu(highlighted);
      menu_drawn = true;
    }

    if (touched)
    {
      was_pressed = true;
      press_mode = hit_mode;
    }
    else if (was_pressed)
    {
      was_pressed = false;
      if (press_mode == int(APP_LEVEL_ONLY))
      {
        return APP_LEVEL_ONLY;
      }
      if (press_mode == int(APP_WIFI_RADAR))
      {
        return APP_WIFI_RADAR;
      }
    }

    delay(20);
  }

  return APP_WIFI_RADAR;
}

static int find_device_slot(SignalSource source, const String &id)
{
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i)
  {
    if (!g_devices[i].used)
      continue;
    if (g_devices[i].source != source)
      continue;
    if (g_devices[i].bssid.equalsIgnoreCase(id))
      return i;
  }
  return -1;
}

static int alloc_device_slot(uint32_t now_ms)
{
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i)
  {
    if (!g_devices[i].used)
      return i;
  }

  int replace_idx = 0;
  uint32_t oldest_age = 0;
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i)
  {
    const uint32_t age = now_ms - g_devices[i].last_seen_ms;
    if (age > oldest_age)
    {
      oldest_age = age;
      replace_idx = i;
    }
  }
  return replace_idx;
}

static void update_device_rssi(RadarDevice &d, float raw_rssi)
{
  d.raw_rssi = raw_rssi;
  if (!d.has_rssi)
  {
    d.smooth_rssi = raw_rssi;
    d.prev_smooth_rssi = raw_rssi;
    d.trend = TREND_UNKNOWN;
    d.has_rssi = true;
  }
  else
  {
    d.prev_smooth_rssi = d.smooth_rssi;
    d.smooth_rssi = (0.74f * d.smooth_rssi) + (0.26f * raw_rssi);
    const float delta = d.smooth_rssi - d.prev_smooth_rssi;
    if (delta > 1.0f)
      d.trend = TREND_CLOSER;
    else if (delta < -1.0f)
      d.trend = TREND_FARTHER;
    else
      d.trend = TREND_STABLE;
  }
  d.signal_01 = rssi_to_signal01(d.smooth_rssi);
}

static void upsert_device(SignalSource source, const String &name, const String &id, int32_t channel, float raw_rssi, uint32_t now_ms)
{
  int slot = find_device_slot(source, id);
  if (slot < 0)
  {
    slot = alloc_device_slot(now_ms);
    if (slot == g_selected_device)
      g_selected_device = -1;
    g_devices[slot] = RadarDevice();
    g_devices[slot].used = true;
    g_devices[slot].source = source;
    g_devices[slot].bssid = id;
  }

  RadarDevice &d = g_devices[slot];
  d.seen_this_scan = true;
  d.visible = true;
  d.last_seen_ms = now_ms;
  d.source = source;
  d.ssid = name;
  d.channel = channel;
  update_device_rssi(d, raw_rssi);
}

static void scan_wifi_devices(uint32_t now_ms)
{
  const int16_t count = WiFi.scanComplete();
  if (count == WIFI_SCAN_RUNNING)
    return;

  if (count > 0)
  {
    for (int i = 0; i < count; ++i)
    {
      const String ssid = WiFi.SSID(i);
      const String bssid = WiFi.BSSIDstr(i);
      if (has_cfg_target_bssid() && !bssid_equals_ignore_case(bssid, TARGET_BSSID))
        continue;
      if (has_cfg_target_ssid() && !ssid.equals(TARGET_SSID))
        continue;
      upsert_device(SIGNAL_WIFI, ssid, bssid, WiFi.channel(i), float(WiFi.RSSI(i)), now_ms);
    }
  }
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true, false, 90);
}

static void scan_ble_devices(uint32_t now_ms)
{
  if (!g_ble_ready || (g_ble_scan == nullptr))
    return;

  BLEScanResults results = g_ble_scan->start(BLE_SCAN_SECONDS, false);
  const int count = results.getCount();
  for (int i = 0; i < count; ++i)
  {
    BLEAdvertisedDevice dev = results.getDevice(i);
    String mac = String(dev.getAddress().toString().c_str());
    String name = dev.haveName() ? String(dev.getName().c_str()) : String("");
    upsert_device(SIGNAL_BLE, name, mac, 0, float(dev.getRSSI()), now_ms);
  }
  g_ble_scan->clearResults();
}

static void refresh_visibility_and_sort(uint32_t now_ms)
{
  g_visible_count = 0;
  g_sorted_count = 0;
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i)
  {
    if (!g_devices[i].used)
      continue;

    const uint32_t age = now_ms - g_devices[i].last_seen_ms;
    if (age > DEVICE_PRUNE_MS)
    {
      g_devices[i] = RadarDevice();
      if (g_selected_device == i)
        g_selected_device = -1;
      continue;
    }

    g_devices[i].visible = (age <= DEVICE_HIDE_MS);
    if (g_devices[i].visible)
    {
      g_sorted_indices[g_sorted_count++] = i;
      g_visible_count++;
    }
  }

  for (int i = 0; i < (g_sorted_count - 1); ++i)
  {
    for (int j = (i + 1); j < g_sorted_count; ++j)
    {
      const int a = g_sorted_indices[i];
      const int b = g_sorted_indices[j];
      if (g_devices[b].signal_01 > g_devices[a].signal_01)
      {
        const int t = g_sorted_indices[i];
        g_sorted_indices[i] = g_sorted_indices[j];
        g_sorted_indices[j] = t;
      }
    }
  }
}

static void process_scan_results(uint32_t now_ms)
{
  for (int i = 0; i < MAX_TRACKED_DEVICES; ++i)
  {
    if (g_devices[i].used)
      g_devices[i].seen_this_scan = false;
  }

  if (scan_mode_uses_wifi())
    scan_wifi_devices(now_ms);
  if (scan_mode_uses_ble() && g_ble_ready && !g_touch_was_pressed && ((now_ms - g_last_ble_scan_ms) >= BLE_SCAN_INTERVAL_MS))
  {
    scan_ble_devices(now_ms);
    g_last_ble_scan_ms = now_ms;
  }
  refresh_visibility_and_sort(now_ms);
}

static bool ble_scanner_init()
{
  if (g_ble_ready && (g_ble_scan != nullptr))
    return true;

  BLEDevice::init("");
  g_ble_scan = BLEDevice::getScan();
  if (g_ble_scan == nullptr)
    return false;

  g_ble_scan->setActiveScan(true);
  g_ble_scan->setInterval(120);
  g_ble_scan->setWindow(90);
  g_ble_ready = true;
  return true;
}

static void audio_beeper_release()
{
  if (g_es8311 != nullptr)
  {
    es8311_delete(g_es8311);
    g_es8311 = nullptr;
  }

  if (g_i2s_ready)
  {
    i2s_zero_dma_buffer(I2S_PORT);
    i2s_driver_uninstall(I2S_PORT);
    g_i2s_ready = false;
  }

  g_audio_ready = false;
}

static bool audio_beeper_init()
{
  audio_beeper_release();

  i2s_config_t i2s_cfg = {};
  i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_cfg.sample_rate = AUDIO_SAMPLE_HZ;
  i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_cfg.dma_buf_count = 6;
  i2s_cfg.dma_buf_len = 256;
  i2s_cfg.use_apll = false;
  i2s_cfg.tx_desc_auto_clear = true;
  i2s_cfg.fixed_mclk = AUDIO_MCLK_HZ;
  i2s_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  i2s_cfg.bits_per_chan = I2S_BITS_PER_CHAN_16BIT;

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_cfg, 0, nullptr);
  if (err != ESP_OK)
  {
    Serial.printf("I2S install failed: %d\n", err);
    return false;
  }
  g_i2s_ready = true;

  i2s_pin_config_t pin_cfg = {};
  pin_cfg.mck_io_num = I2S_MCK_PIN;
  pin_cfg.bck_io_num = I2S_BCK_PIN;
  pin_cfg.ws_io_num = I2S_LRCK_PIN;
  pin_cfg.data_out_num = I2S_DOUT_PIN;
  pin_cfg.data_in_num = I2S_DIN_PIN;

  err = i2s_set_pin(I2S_PORT, &pin_cfg);
  if (err != ESP_OK)
  {
    Serial.printf("I2S pin set failed: %d\n", err);
    audio_beeper_release();
    return false;
  }
  i2s_zero_dma_buffer(I2S_PORT);

  g_es8311 = es8311_create((i2c_port_t)0, ES8311_ADDRRES_0);
  if (g_es8311 == nullptr)
  {
    Serial.println("ES8311 handle create failed");
    audio_beeper_release();
    return false;
  }

  es8311_clock_config_t clk_cfg = {};
  clk_cfg.mclk_inverted = false;
  clk_cfg.sclk_inverted = false;
  clk_cfg.mclk_from_mclk_pin = true;
  clk_cfg.mclk_frequency = AUDIO_MCLK_HZ;
  clk_cfg.sample_frequency = AUDIO_SAMPLE_HZ;

  err = es8311_init(g_es8311, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
  if (err != ESP_OK)
  {
    Serial.printf("ES8311 init failed: %d\n", err);
    audio_beeper_release();
    return false;
  }

  es8311_voice_volume_set(g_es8311, 70, nullptr);
  es8311_microphone_config(g_es8311, false);
  es8311_voice_mute(g_es8311, false);

  g_audio_ready = true;
  return true;
}

static bool ensure_audio_ready(uint32_t now_ms, bool force = false)
{
  if (g_audio_ready)
    return true;

  const uint32_t retry_ms = 2500;
  if (!force && ((now_ms - g_last_audio_init_attempt_ms) < retry_ms))
    return false;

  g_last_audio_init_attempt_ms = now_ms;
  const bool ok = audio_beeper_init();
  Serial.printf("Audio beeper: %s\n", ok ? "READY" : "OFF");
  return ok;
}

static void play_beep_tone(uint16_t freq_hz, uint16_t duration_ms, float gain = 0.18f)
{
  if (freq_hz == 0 || duration_ms == 0)
    return;

  if (!g_audio_ready && !ensure_audio_ready(millis()))
    return;

  if (gain < 0.01f)
    gain = 0.01f;
  if (gain > 0.9f)
    gain = 0.9f;

  const int total_frames = (AUDIO_SAMPLE_HZ * int(duration_ms)) / 1000;
  if (total_frames <= 0)
    return;

  const int chunk_frames = 128;
  int16_t pcm[chunk_frames * 2];
  const float phase_inc = (2.0f * PI * float(freq_hz)) / float(AUDIO_SAMPLE_HZ);
  const int ramp_frames = max(12, total_frames / 14);
  float phase = 0.0f;
  int remaining = total_frames;
  int frame_pos = 0;

  while (remaining > 0)
  {
    const int frames = (remaining > chunk_frames) ? chunk_frames : remaining;
    for (int i = 0; i < frames; ++i)
    {
      float env = 1.0f;
      if (frame_pos < ramp_frames)
      {
        env = float(frame_pos) / float(ramp_frames);
      }
      else if (frame_pos > (total_frames - ramp_frames))
      {
        env = float(total_frames - frame_pos) / float(ramp_frames);
      }
      if (env < 0.0f)
        env = 0.0f;

      const int16_t sample = (int16_t)(sinf(phase) * 32767.0f * gain * env);
      phase += phase_inc;
      if (phase > (2.0f * PI))
      {
        phase -= (2.0f * PI);
      }
      pcm[2 * i] = sample;
      pcm[2 * i + 1] = sample;
      frame_pos++;
    }

    size_t written = 0;
    i2s_write(I2S_PORT, pcm, size_t(frames * 2 * sizeof(int16_t)), &written, portMAX_DELAY);
    remaining -= frames;
  }
}

static int device_from_touch(int16_t x, int16_t y)
{
  if (!point_in_rect(x, y, RADAR_LIST_X, RADAR_LIST_Y, RADAR_LIST_W, RADAR_ROW_H * MAX_LIST_ROWS))
    return -1;

  const int row = (int(y) - RADAR_LIST_Y) / RADAR_ROW_H;
  if (row < 0 || row >= MAX_LIST_ROWS)
    return -1;
  return g_row_to_device[row];
}

static void handle_radar_touch_release(int16_t x, int16_t y)
{
  int hit_idx = device_from_touch(x, y);
  if ((hit_idx < 0) && (g_touch_press_device >= 0))
  {
    // Keep taps reliable even while the sorted row map changes during scans.
    hit_idx = g_touch_press_device;
  }
  if (hit_idx < 0)
    return;

  if (g_selected_device == hit_idx)
  {
    g_selected_device = -1;
    g_last_tap_device = hit_idx;
    g_last_tap_selected = false;
    g_last_tap_feedback_until_ms = millis() + 850;
    play_beep_tone(500, 36, 0.20f);
  }
  else
  {
    g_selected_device = hit_idx;
    g_last_tap_device = hit_idx;
    g_last_tap_selected = true;
    g_last_tap_feedback_until_ms = millis() + 850;
    play_beep_tone(1180, 48, 0.24f);
  }
}

static bool selected_device_active()
{
  if (g_selected_device < 0 || g_selected_device >= MAX_TRACKED_DEVICES)
    return false;
  if (!g_devices[g_selected_device].used)
    return false;
  return true;
}

static void update_beeper(uint32_t now_ms)
{
  if (!selected_device_active())
    return;
  if (!ensure_audio_ready(now_ms))
    return;

  RadarDevice &d = g_devices[g_selected_device];
  if (!d.visible)
    return;

  const float signal = d.signal_01;
  uint32_t interval_ms = uint32_t(980.0f - (signal * 780.0f));
  if (interval_ms < 110)
    interval_ms = 110;
  if ((now_ms - g_last_beep_ms) < interval_ms)
    return;

  uint16_t freq_hz = uint16_t(620.0f + (signal * 1380.0f));
  if (d.trend == TREND_CLOSER)
  {
    freq_hz += 180;
  }
  else if (d.trend == TREND_FARTHER && freq_hz > 140)
  {
    freq_hz -= 180;
  }

  const uint16_t duration_ms = uint16_t(20.0f + (signal * 44.0f));
  const float gain = 0.18f + (signal * 0.14f);
  play_beep_tone(freq_hz, duration_ms, gain);
  g_last_beep_ms = now_ms;
}

static void update_level_beeper(uint32_t now_ms)
{
  if (!g_level_valid || (g_level_beep_mode == LEVEL_BEEP_OFF))
    return;
  if (!ensure_audio_ready(now_ms))
    return;

  const float rel_x_deg = g_level_x_deg - g_level_zero_x_deg;
  const float rel_y_deg = g_level_y_deg - g_level_zero_y_deg;
  const float tilt_mag = sqrtf((rel_x_deg * rel_x_deg) + (rel_y_deg * rel_y_deg));

  const bool now_flat = (tilt_mag <= 1.6f);
  const bool now_not_flat = (tilt_mag >= 2.2f);
  if (!g_level_flat_state_valid)
  {
    g_level_is_flat = now_flat;
    g_level_flat_state_valid = true;
  }
  else
  {
    if (now_flat)
      g_level_is_flat = true;
    else if (now_not_flat)
      g_level_is_flat = false;
  }

  const bool should_beep = (g_level_beep_mode == LEVEL_BEEP_WHEN_LEVEL) ? g_level_is_flat : !g_level_is_flat;
  if (!should_beep)
    return;

  const uint32_t interval_ms = g_level_is_flat ? 650 : 320;
  if ((now_ms - g_last_level_beep_ms) < interval_ms)
    return;

  const uint16_t freq_hz = g_level_is_flat ? 1080 : 480;
  const uint16_t dur_ms = g_level_is_flat ? 42 : 38;
  const float gain = g_level_is_flat ? 0.12f : 0.10f;
  play_beep_tone(freq_hz, dur_ms, gain);
  g_last_level_beep_ms = now_ms;
}

static void draw_level_grid(int cx, int cy, int r)
{
  gfx->drawCircle(cx, cy, r, CYAN);
  gfx->drawCircle(cx, cy, r - 45, DARKGREY);
  gfx->drawCircle(cx, cy, r - 90, DARKGREY);
  gfx->drawFastHLine(cx - r, cy, r * 2, DARKGREY);
  gfx->drawFastVLine(cx, cy - r, r * 2, DARKGREY);
}

static void draw_level_only_ui()
{
  const int cx = 160;
  const int cy = 255;
  const int r = 145;
  const float max_tilt = 25.0f;

  if (!g_level_ui_static_ready)
  {
    gfx->fillScreen(BLACK);

    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(8, 8);
    gfx->println("Level Mode");

    gfx->setTextSize(1);
    gfx->setTextColor(LIGHTGREY);
    gfx->setCursor(8, 30);
    gfx->println("BOOT: recenter level | Touch CAL when flat");
    g_level_ui_static_ready = true;
    g_level_prev_dot_valid = false;

    // Clear and draw the full level arena once.
    gfx->fillCircle(cx, cy, r + 1, BLACK);
    draw_level_grid(cx, cy, r);
  }

  draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);
  const uint16_t cal_fill = g_level_calibrating ? DARKGREEN : 0x1082;
  const uint16_t cal_border = g_level_calibrating ? GREEN : LIGHTGREY;
  draw_touch_button(BTN_CAL_X, BTN_CAL_Y, BTN_CAL_W, BTN_CAL_H, "CAL", cal_fill, cal_border, WHITE);
  const uint16_t beep_fill = (g_level_beep_mode == LEVEL_BEEP_OFF) ? 0x1082 : 0x0430;
  const uint16_t beep_border = (g_level_beep_mode == LEVEL_BEEP_OFF) ? LIGHTGREY : GREEN;
  draw_touch_button(BTN_LBEEP_X, BTN_LBEEP_Y, BTN_LBEEP_W, BTN_LBEEP_H, "BEEP", beep_fill, beep_border, WHITE);

  // Only clear status text rows each frame.
  gfx->fillRect(8, 438, 304, 28, BLACK);
  char line[96];

  if (g_level_valid)
  {
    const float rel_x_deg = g_level_x_deg - g_level_zero_x_deg;
    const float rel_y_deg = g_level_y_deg - g_level_zero_y_deg;

    float nx = rel_x_deg / max_tilt;
    float ny = rel_y_deg / max_tilt;
    nx = clamp01((nx + 1.0f) * 0.5f) * 2.0f - 1.0f;
    ny = clamp01((ny + 1.0f) * 0.5f) * 2.0f - 1.0f;

    const int dot_limit = r - 18;
    const int dx = cx + int(nx * dot_limit);
    const int dy = cy + int(ny * dot_limit);

    const float tilt_mag = sqrtf((rel_x_deg * rel_x_deg) + (rel_y_deg * rel_y_deg));
    const uint16_t c = (tilt_mag < 2.0f) ? GREEN : YELLOW;

    const bool dot_changed = (!g_level_prev_dot_valid) ||
                             (dx != g_level_prev_dot_x) ||
                             (dy != g_level_prev_dot_y) ||
                             (c != g_level_prev_dot_color);

    if (dot_changed)
    {
      if (g_level_prev_dot_valid)
      {
        gfx->fillCircle(g_level_prev_dot_x, g_level_prev_dot_y, 13, BLACK);
        draw_level_grid(cx, cy, r);
      }
      gfx->fillCircle(dx, dy, 10, c);
      gfx->drawCircle(dx, dy, 12, WHITE);
      g_level_prev_dot_x = int16_t(dx);
      g_level_prev_dot_y = int16_t(dy);
      g_level_prev_dot_color = c;
      g_level_prev_dot_valid = true;
    }

    gfx->setTextColor(g_level_calibrating ? GREEN : LIGHTGREY);
    gfx->setCursor(8, 438);
    if (g_level_calibrating)
      snprintf(line, sizeof(line), "Calibrating... %u/%u", (unsigned)g_level_cal_count, (unsigned)LEVEL_CAL_SAMPLES);
    else
      snprintf(line, sizeof(line), "Tap CAL to set level | Beep mode: %s", level_beep_mode_text(g_level_beep_mode));
    gfx->println(line);

    gfx->setTextColor(WHITE);
    gfx->setCursor(8, 452);
    snprintf(line, sizeof(line), "Tilt X: %.1f deg  Y: %.1f deg", rel_x_deg, rel_y_deg);
    gfx->println(line);
  }
  else
  {
    if (g_level_prev_dot_valid)
    {
      gfx->fillCircle(g_level_prev_dot_x, g_level_prev_dot_y, 13, BLACK);
      draw_level_grid(cx, cy, r);
      g_level_prev_dot_valid = false;
    }
    gfx->setTextColor(ORANGE);
    gfx->setCursor(8, 438);
    gfx->println("Waiting for IMU...");
    gfx->setTextColor(LIGHTGREY);
    gfx->setCursor(8, 452);
    snprintf(line, sizeof(line), "Tap CAL flat | Beep mode: %s", level_beep_mode_text(g_level_beep_mode));
    gfx->println(line);
  }
}

static void draw_radar_ui()
{
  if (!g_radar_ui_static_ready)
  {
    gfx->fillScreen(BLACK);
    gfx->setTextSize(1);
    gfx->setTextColor(LIGHTGREY);
    gfx->setCursor(8, 452);
    gfx->println("Beep speed/pitch rises as selected target gets stronger");
    gfx->setCursor(8, 466);
    gfx->println("BACK: mode menu | BOOT: clear selected target");
    g_radar_ui_static_ready = true;
  }

  if (g_selected_device >= 0)
  {
    if (g_selected_device >= MAX_TRACKED_DEVICES || !g_devices[g_selected_device].used)
      g_selected_device = -1;
  }

  for (int i = 0; i < MAX_LIST_ROWS; ++i)
  {
    g_row_to_device[i] = -1;
  }

  // Redraw only dynamic regions to avoid full-screen flicker.
  gfx->fillRect(0, 0, LCD_HOR_RES, 90, BLACK);
  gfx->fillRect(RADAR_LIST_X - 2, RADAR_LIST_Y - 2, RADAR_LIST_W + 4, (RADAR_ROW_H * MAX_LIST_ROWS) + 4, BLACK);

  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(8, 8);
  gfx->println("Signal Finder");
  const uint16_t src_fill = scan_mode_uses_ble() ? 0x032F : 0x18A3;
  const uint16_t src_border = scan_mode_uses_ble() ? CYAN : GREEN;
  char src_label[16];
  snprintf(src_label, sizeof(src_label), "SRC:%s", scan_mode_text(g_scan_source_mode));
  draw_touch_button(BTN_SRC_X, BTN_SRC_Y, BTN_SRC_W, BTN_SRC_H, src_label, src_fill, src_border, WHITE);
  draw_touch_button(BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H, "BACK", 0x2104, LIGHTGREY, WHITE);

  gfx->setTextSize(1);
  gfx->setTextColor(CYAN);
  char line[128];
  if (g_scan_source_mode == SCAN_WIFI_BLE)
    snprintf(line, sizeof(line), "Nearby Wi-Fi + BLE: %d", g_visible_count);
  else if (g_scan_source_mode == SCAN_BLE_ONLY)
    snprintf(line, sizeof(line), "Nearby BLE: %d", g_visible_count);
  else
    snprintf(line, sizeof(line), "Nearby Wi-Fi: %d", g_visible_count);
  gfx->setCursor(8, 30);
  gfx->println(line);
  gfx->setTextColor(LIGHTGREY);
  gfx->setCursor(8, 44);
  gfx->println("Tap SRC to switch scan source. Tap a row to guide.");

  const bool show_tap_feedback = (millis() < g_last_tap_feedback_until_ms) &&
                                 (g_last_tap_device >= 0) &&
                                 (g_last_tap_device < MAX_TRACKED_DEVICES) &&
                                 g_devices[g_last_tap_device].used;
  if (selected_device_active())
  {
    const RadarDevice &sel = g_devices[g_selected_device];
    const String shown = (sel.ssid.length() > 0) ? sel.ssid : String((sel.source == SIGNAL_WIFI) ? "<hidden SSID>" : "<no name>");
    gfx->setCursor(8, 56);
    gfx->setTextColor(YELLOW);
    snprintf(line, sizeof(line), "Target: %s %s  RSSI: %.1f  %s",
             source_to_text(sel.source), shown.c_str(), sel.smooth_rssi, trend_to_text(sel.trend));
    gfx->println(line);
  }
  else
  {
    gfx->setCursor(8, 56);
    gfx->setTextColor(ORANGE);
    gfx->println("No target selected");
  }
  if (show_tap_feedback)
  {
    const RadarDevice &tap = g_devices[g_last_tap_device];
    String shown = (tap.ssid.length() > 0) ? tap.ssid : String((tap.source == SIGNAL_WIFI) ? "<hidden SSID>" : "<no name>");
    if (shown.length() > 20)
      shown = shown.substring(0, 20);
    gfx->setCursor(8, 68);
    gfx->setTextColor(g_last_tap_selected ? GREEN : ORANGE);
    snprintf(line, sizeof(line), "%s: %s %s",
             g_last_tap_selected ? "Selected" : "Cleared",
             source_to_text(tap.source), shown.c_str());
    gfx->println(line);
  }

  const int shown_rows = (g_sorted_count < MAX_LIST_ROWS) ? g_sorted_count : MAX_LIST_ROWS;
  for (int row = 0; row < shown_rows; ++row)
  {
    const int idx = g_sorted_indices[row];
    g_row_to_device[row] = idx;
    const RadarDevice &d = g_devices[idx];
    const int y = RADAR_LIST_Y + (row * RADAR_ROW_H);
    const bool is_selected = (idx == g_selected_device);
    const bool is_pressed = g_touch_was_pressed && (idx == g_touch_press_device);

    const uint16_t row_fill = is_selected ? 0x0841 : (is_pressed ? 0x10A2 : 0x0000);
    const uint16_t row_border = is_pressed ? CYAN : (is_selected ? YELLOW : DARKGREY);
    gfx->fillRoundRect(RADAR_LIST_X, y, RADAR_LIST_W, RADAR_ROW_H - 4, 6, row_fill);
    gfx->drawRoundRect(RADAR_LIST_X, y, RADAR_LIST_W, RADAR_ROW_H - 4, 6, row_border);

    String shown = (d.ssid.length() > 0) ? d.ssid : String((d.source == SIGNAL_WIFI) ? "<hidden SSID>" : "<no name>");
    if (shown.length() > 12)
      shown = shown.substring(0, 12);

    gfx->setTextSize(2);
    gfx->setCursor(RADAR_LIST_X + 8, y + 6);
    gfx->setTextColor(source_to_color(d.source));
    snprintf(line, sizeof(line), "%d %c %s", row + 1, (d.source == SIGNAL_WIFI) ? 'W' : 'B', shown.c_str());
    gfx->println(line);

    gfx->setTextSize(1);
    gfx->setCursor(RADAR_LIST_X + 8, y + 30);
    gfx->setTextColor(WHITE);
    if (d.source == SIGNAL_WIFI)
      snprintf(line, sizeof(line), "RSSI %.1f dBm  %3d%%  CH %ld  %s", d.smooth_rssi, int(d.signal_01 * 100.0f), (long)d.channel, trend_to_text(d.trend));
    else
      snprintf(line, sizeof(line), "RSSI %.1f dBm  %3d%%  %s", d.smooth_rssi, int(d.signal_01 * 100.0f), trend_to_text(d.trend));
    gfx->println(line);
  }

  if (shown_rows == 0)
  {
    gfx->setTextSize(2);
    gfx->setTextColor(ORANGE);
    gfx->setCursor(12, RADAR_LIST_Y + 14);
    gfx->println("Scanning...");
    gfx->setTextSize(1);
  }

  // Footer is static and drawn once when entering radar mode.
}

static void apply_app_mode(AppMode mode)
{
  const uint32_t now_ms = millis();
  g_app_mode = mode;
  g_touch_was_pressed = false;
  g_touch_press_device = -1;
  g_touch_press_back = false;
  g_touch_press_src = false;
  g_radar_ui_static_ready = false;
  g_last_tap_device = -1;
  g_last_tap_feedback_until_ms = 0;
  g_last_frame_ms = now_ms - FRAME_INTERVAL_MS;
  g_mode_enter_ms = now_ms;
  g_level_ui_static_ready = false;
  g_level_flat_state_valid = false;
  g_last_level_beep_ms = 0;

  if (!g_audio_ready)
  {
    ensure_audio_ready(now_ms, true);
  }

  if (mode == APP_WIFI_RADAR)
  {
    apply_scan_source_mode(now_ms);
    draw_radar_ui();
  }
  else
  {
    WiFi.mode(WIFI_OFF);
    g_selected_device = -1;
    start_level_calibration();
    draw_level_only_ui();
  }
}

static void switch_mode_from_touch_menu()
{
  if (!g_touch_ready)
    return;

  const AppMode selected = select_startup_mode_touch();
  Serial.printf("Selected mode: %s\n", (selected == APP_LEVEL_ONLY) ? "LEVEL" : "WIFI_RADAR");
  apply_app_mode(selected);
  g_prev_boot_pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
}

void setup()
{
  Serial.begin(115200);
  delay(120);
  Serial.println("Waveshare ESP32-S3 Touch 3.5 app");

  tca9554_init_and_lcd_reset();
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  if (!gfx->begin(40000000))
  {
    Serial.println("gfx->begin() failed");
    while (true)
    {
      delay(1000);
    }
  }

  g_touch_ready = touch_init();
  Serial.printf("Touch: %s\n", g_touch_ready ? "READY" : "OFF");

  g_app_mode = select_startup_mode_touch();
  g_prev_boot_pressed = false;
  Serial.printf("Selected mode: %s\n", (g_app_mode == APP_LEVEL_ONLY) ? "LEVEL" : "WIFI_RADAR");

  g_imu_ready = imu_init();
  Serial.printf("IMU heading: %s\n", g_imu_ready ? "READY" : "OFF");

  apply_app_mode(g_app_mode);
}

void loop()
{
  const uint32_t now = millis();
  update_imu_heading();

  const bool boot_pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  const bool boot_edge = boot_pressed && !g_prev_boot_pressed;
  g_prev_boot_pressed = boot_pressed;

  if (boot_edge && g_level_valid)
  {
    g_level_zero_x_deg = g_level_x_deg;
    g_level_zero_y_deg = g_level_y_deg;
    g_level_zero_set = true;
    g_level_calibrating = false;
    g_level_cal_count = 0;
  }

  if (g_app_mode == APP_LEVEL_ONLY)
  {
    if (g_touch_ready && ((now - g_mode_enter_ms) >= MODE_TOUCH_GUARD_MS))
    {
      int16_t tx[1] = {0};
      int16_t ty[1] = {0};
      const bool touched = touch.getPoint(tx, ty, 1) > 0;
      if (touched)
      {
        if (point_in_rect(tx[0], ty[0], BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H))
        {
          g_touch_was_pressed = false;
          switch_mode_from_touch_menu();
          delay(5);
          return;
        }
        g_touch_was_pressed = true;
        g_touch_last_x = tx[0];
        g_touch_last_y = ty[0];
      }
      else if (g_touch_was_pressed)
      {
        g_touch_was_pressed = false;
        if (point_in_rect(g_touch_last_x, g_touch_last_y, BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H))
        {
          switch_mode_from_touch_menu();
          delay(5);
          return;
        }
        if (point_in_rect(g_touch_last_x, g_touch_last_y, BTN_CAL_X, BTN_CAL_Y, BTN_CAL_W, BTN_CAL_H))
        {
          start_level_calibration();
          play_beep_tone(920, 36, 0.20f);
        }
        if (point_in_rect(g_touch_last_x, g_touch_last_y, BTN_LBEEP_X, BTN_LBEEP_Y, BTN_LBEEP_W, BTN_LBEEP_H))
        {
          cycle_level_beep_mode();
          ensure_audio_ready(now, true);
          play_beep_tone(760, 28, 0.20f);
        }
      }
    }
    else
    {
      g_touch_was_pressed = false;
    }

    if ((now - g_last_frame_ms) >= FRAME_INTERVAL_MS)
    {
      draw_level_only_ui();
      g_last_frame_ms = now;
    }
    update_level_beeper(now);
    delay(5);
    return;
  }

  if (g_touch_ready && ((now - g_mode_enter_ms) >= MODE_TOUCH_GUARD_MS))
  {
    int16_t tx[1] = {0};
    int16_t ty[1] = {0};
    const bool touched = touch.getPoint(tx, ty, 1) > 0;
    if (touched)
    {
      if (!g_touch_was_pressed)
      {
        g_touch_press_back = point_in_rect(tx[0], ty[0], BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H);
        g_touch_press_src = point_in_rect(tx[0], ty[0], BTN_SRC_X, BTN_SRC_Y, BTN_SRC_W, BTN_SRC_H);
        if (g_touch_press_back || g_touch_press_src)
          g_touch_press_device = -1;
        else
          g_touch_press_device = device_from_touch(tx[0], ty[0]);
      }
      g_touch_was_pressed = true;
      g_touch_last_x = tx[0];
      g_touch_last_y = ty[0];
    }
    else if (g_touch_was_pressed)
    {
      g_touch_was_pressed = false;
      const bool release_on_back = point_in_rect(g_touch_last_x, g_touch_last_y, BTN_BACK_X, BTN_BACK_Y, BTN_BACK_W, BTN_BACK_H);
      const bool release_on_src = point_in_rect(g_touch_last_x, g_touch_last_y, BTN_SRC_X, BTN_SRC_Y, BTN_SRC_W, BTN_SRC_H);
      if (g_touch_press_back && release_on_back)
      {
        g_touch_press_device = -1;
        g_touch_press_back = false;
        g_touch_press_src = false;
        switch_mode_from_touch_menu();
        delay(5);
        return;
      }
      if (g_touch_press_src && release_on_src)
      {
        cycle_scan_source_mode();
        g_touch_press_device = -1;
        g_touch_press_back = false;
        g_touch_press_src = false;
        draw_radar_ui();
        delay(5);
        return;
      }
      handle_radar_touch_release(g_touch_last_x, g_touch_last_y);
      g_touch_press_device = -1;
      g_touch_press_back = false;
      g_touch_press_src = false;
    }
  }
  else
  {
    g_touch_was_pressed = false;
    g_touch_press_device = -1;
    g_touch_press_back = false;
    g_touch_press_src = false;
  }

  if (boot_edge)
  {
    clear_direction_map();
    g_selected_device = -1;
    play_beep_tone(500, 45, 0.20f);
  }

  if ((now - g_last_scan_ms) >= SCAN_INTERVAL_SIGNAL_FINDER_MS)
  {
    process_scan_results(now);
    g_last_scan_ms = now;

    if (selected_device_active())
    {
      const RadarDevice &d = g_devices[g_selected_device];
      Serial.printf("Visible: %d | Selected: %s | RSSI: %.1f dBm | %s\n",
                    g_visible_count, source_to_text(d.source), d.smooth_rssi, d.bssid.c_str());
    }
    else
    {
      Serial.printf("Visible: %d | No selected target\n", g_visible_count);
    }
  }

  update_beeper(now);

  if ((now - g_last_frame_ms) >= FRAME_INTERVAL_MS)
  {
    draw_radar_ui();
    g_last_frame_ms = now;
  }

  delay(5);
}
