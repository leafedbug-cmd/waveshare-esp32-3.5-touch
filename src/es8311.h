#pragma once

#include <Arduino.h>
#include "esp_err.h"
#include "driver/i2c.h"

typedef struct es8311_dev_t *es8311_handle_t;

typedef enum {
  ES8311_ADDRRES_0 = 0
} es8311_addr_res_t;

typedef enum {
  ES8311_RESOLUTION_16 = 16
} es8311_resolution_t;

typedef struct {
  bool mclk_inverted;
  bool sclk_inverted;
  bool mclk_from_mclk_pin;
  uint32_t mclk_frequency;
  uint32_t sample_frequency;
} es8311_clock_config_t;

es8311_handle_t es8311_create(i2c_port_t port, es8311_addr_res_t addr_res);
esp_err_t es8311_delete(es8311_handle_t handle);
esp_err_t es8311_init(es8311_handle_t handle,
                      const es8311_clock_config_t *clk_cfg,
                      es8311_resolution_t in_bits,
                      es8311_resolution_t out_bits);
esp_err_t es8311_voice_volume_set(es8311_handle_t handle, uint8_t volume, uint8_t *out_volume);
esp_err_t es8311_microphone_config(es8311_handle_t handle, bool enable);
esp_err_t es8311_voice_mute(es8311_handle_t handle, bool mute);

