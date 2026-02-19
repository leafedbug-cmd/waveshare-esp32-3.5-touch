#include "es8311.h"

struct es8311_dev_t {
  i2c_port_t port;
  es8311_addr_res_t addr_res;
};

es8311_handle_t es8311_create(i2c_port_t port, es8311_addr_res_t addr_res)
{
  es8311_dev_t *dev = new es8311_dev_t();
  if (dev == nullptr)
    return nullptr;
  dev->port = port;
  dev->addr_res = addr_res;
  return dev;
}

esp_err_t es8311_delete(es8311_handle_t handle)
{
  if (handle != nullptr)
    delete handle;
  return ESP_OK;
}

esp_err_t es8311_init(es8311_handle_t handle,
                      const es8311_clock_config_t *clk_cfg,
                      es8311_resolution_t in_bits,
                      es8311_resolution_t out_bits)
{
  (void)clk_cfg;
  (void)in_bits;
  (void)out_bits;
  return (handle != nullptr) ? ESP_OK : ESP_FAIL;
}

esp_err_t es8311_voice_volume_set(es8311_handle_t handle, uint8_t volume, uint8_t *out_volume)
{
  (void)volume;
  if (out_volume != nullptr)
    *out_volume = volume;
  return (handle != nullptr) ? ESP_OK : ESP_FAIL;
}

esp_err_t es8311_microphone_config(es8311_handle_t handle, bool enable)
{
  (void)enable;
  return (handle != nullptr) ? ESP_OK : ESP_FAIL;
}

esp_err_t es8311_voice_mute(es8311_handle_t handle, bool mute)
{
  (void)mute;
  return (handle != nullptr) ? ESP_OK : ESP_FAIL;
}

