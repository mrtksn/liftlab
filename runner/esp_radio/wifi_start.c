/* Wi-Fi up: see wifi_start.h. */
#include "wifi_start.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"

esp_err_t wifi_start(wifi_mode_t mode, wifi_config_t *cfg) {
  esp_err_t e = esp_netif_init();
  if (e == ESP_OK) { e = esp_event_loop_create_default(); if (e == ESP_ERR_INVALID_STATE) e = ESP_OK; }   /* (made already: fine) */
  if (e != ESP_OK) return e;
  if (!(mode == WIFI_MODE_AP ? esp_netif_create_default_wifi_ap() : esp_netif_create_default_wifi_sta())) return ESP_FAIL;
  esp_log_level_set("wifi", ESP_LOG_WARN);              /* (its chatter would go into the serial link) */
  wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
  e = esp_wifi_init(&ic);
  if (e == ESP_OK) e = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (e == ESP_OK) e = esp_wifi_set_mode(mode);
  if (e == ESP_OK && cfg) e = esp_wifi_set_config(mode == WIFI_MODE_AP ? WIFI_IF_AP : WIFI_IF_STA, cfg);
  if (e == ESP_OK) e = esp_wifi_start();
  if (e == ESP_OK) e = esp_wifi_set_ps(WIFI_PS_NONE);
  return e;
}
