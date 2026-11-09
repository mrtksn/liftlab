/* The companion link over MQTT, beside the UART: see link_mqtt.h. */
#include "link_mqtt.h"
#include "mqtt_min.h"
#include "wifi_start.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>

#define TXCAP 16384                              /* the outgoing queue: LTEL at 200 Hz and the telemetry fit with room */
static mq_client *MQ;
static SemaphoreHandle_t mu;                     /* (link_task and the radio task send; this file's task polls) */
static StreamBufferHandle_t rxq;
static char *to_fc, *from_fc, *net;           /* (on the heap: static DRAM is short on the ESP32) */
static esp_radio_say SAY;
static volatile int joined, resync;

static void sayf(const char *fmt, ...) { if (!SAY) return; char s[128]; va_list a; va_start(a, fmt); vsnprintf(s, sizeof s, fmt, a); va_end(a); SAY(s); }
static double now_s(void) { return esp_timer_get_time() * 1e-6; }
static void on_bytes(void *c, const uint8_t *b, int n) { (void)c; if (xStreamBufferSend(rxq, b, (size_t)n, 0) != (size_t)n) MQ->dropped++; }
static void on_say(void *c, const char *s) { (void)c; if (strstr(s, "connected to")) resync = 1; if (SAY) SAY(s); }

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
  (void)arg;
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
  else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *d = data;
    if (joined) sayf("link: left Wi-Fi %s (reason %d); joining it again", net, d ? d->reason : 0);
    else { static int said; if (!said++) sayf("link: can't join Wi-Fi %s yet (reason %d): is it there, the password right? Trying on", net, d ? d->reason : 0); }
    joined = 0; esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t *g = data; char ip[16]; esp_ip4addr_ntoa(&g->ip_info.ip, ip, sizeof ip);
    joined = 1; sayf("link: joined Wi-Fi %s as %s", net, ip);
  }
}

static void task(void *arg) {
  (void)arg;
  for (;;) {
    int fd = -1;
    xSemaphoreTake(mu, portMAX_DELAY);
    if (joined) fd = mq_poll(MQ, now_s());
    else if (MQ->fd >= 0) mq_close(MQ, "Wi-Fi lost", now_s());
    xSemaphoreGive(mu);
    if (fd >= 0) { fd_set r; FD_ZERO(&r); FD_SET(fd, &r); struct timeval tv = { 0, 2000 }; select(fd + 1, &r, 0, 0, &tv); }   /* (up to 2 ms: what comes is taken at once) */
    else vTaskDelay(pdMS_TO_TICKS(20));
  }
}

int link_mqtt_start(const char *ssid, const char *pass, const char *host, int port, const char *prefix, esp_radio_say say) {
  SAY = say;
  if (!ssid || !ssid[0]) { sayf("link=mqtt: which network? set wifi=SSID,PASSWORD (the broker's network)"); return -1; }
  MQ = calloc(1, sizeof *MQ); uint8_t *tx = malloc(TXCAP); rxq = xStreamBufferCreate(8192, 1); mu = xSemaphoreCreateMutex();
  to_fc = malloc(112); from_fc = malloc(112); net = malloc(33);
  if (!MQ || !tx || !rxq || !mu || !to_fc || !from_fc || !net) { sayf("link=mqtt: no memory"); return -1; }
  uint8_t mac[6] = { 0 }; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char id[24]; snprintf(id, sizeof id, "liftlab-fc-%02x%02x%02x", mac[3], mac[4], mac[5]);
  mq_link_topics(prefix, to_fc, 112, from_fc, 112);
  mq_init(MQ, host, port, id, to_fc, tx, TXCAP); MQ->on_bytes = on_bytes; MQ->on_say = on_say;
  snprintf(net, 33, "%s", ssid);
  wifi_config_t c; memset(&c, 0, sizeof c);
  memcpy(c.sta.ssid, net, strlen(net)); memcpy(c.sta.password, pass, strnlen(pass, sizeof c.sta.password - 1));
  c.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;   /* (an empty password: an open network, as a toy camera board's) */
  esp_err_t e = esp_event_loop_create_default(); if (e == ESP_ERR_INVALID_STATE) e = ESP_OK;
  if (e == ESP_OK) e = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, 0);
  if (e == ESP_OK) e = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, 0);
  if (e == ESP_OK) e = wifi_start(WIFI_MODE_STA, &c);
  if (e != ESP_OK) { sayf("link=mqtt: Wi-Fi didn't start: %s", esp_err_to_name(e)); return -1; }
  if (xTaskCreatePinnedToCore(task, "mqtt", 6144, 0, 5, 0, 0) != pdPASS) { sayf("link=mqtt: no task"); return -1; }
  sayf("link: MQTT broker %s:%d over Wi-Fi %s as %s; frames in on %s, out on %s", host, port, net, id, to_fc, from_fc);
  return 0;
}
int link_mqtt_read(uint8_t *b, int max) { return rxq ? (int)xStreamBufferReceive(rxq, b, (size_t)max, 0) : 0; }
int link_mqtt_resync(void) { int r = resync; resync = 0; return r; }
void link_mqtt_send(const uint8_t *frame, uint32_t n) {
  if (!MQ || !mu || xSemaphoreTake(mu, pdMS_TO_TICKS(2)) != pdTRUE) return;   /* (never holds up the sender for long) */
  mq_publish(MQ, from_fc, frame, n, now_s());
  xSemaphoreGive(mu);
}
