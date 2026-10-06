/* ESP-NOW: see esp_radio.h. */
#include "radio_packet.h"
#include "wifi_start.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

typedef struct { uint8_t n; int8_t rssi; uint8_t src[ESP_NOW_ETH_ALEN]; uint8_t p[PLINK_MTU]; } rx_pkt;
#define RXQ 16
static QueueHandle_t rxq;
static volatile uint32_t rx_lost;
static const uint8_t bcast[ESP_NOW_ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
/* Where ours go: to everyone until the other end is known, then to it alone. A broadcast frame goes once, unanswered;
 * one sent to an address is acknowledged and tried again by the radio when it isn't, which is most of what a weak
 * link loses. The other end is the sender of the last packet plink took (signed with our phrase, its session the
 * one we talk to): a new command module is taken, as with Wi-Fi, once the old one has been quiet for half a second. */
static uint8_t last_src[ESP_NOW_ETH_ALEN], peer[ESP_NOW_ETH_ALEN];
static int have_peer;

/* (the Wi-Fi task: only into the queue; the radio task takes it from there) */
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int n) {
  if (n < 1 || n > PLINK_MTU) return;
  rx_pkt r; r.n = (uint8_t)n; r.rssi = (int8_t)(info && info->rx_ctrl ? info->rx_ctrl->rssi : 0);
  if (info && info->src_addr) memcpy(r.src, info->src_addr, sizeof r.src); else memset(r.src, 0, sizeof r.src);
  memcpy(r.p, data, (size_t)n);
  if (xQueueSend(rxq, &r, 0) != pdTRUE) rx_lost++;
}
static int en_recv(pk_link *K, uint8_t *p, int cap, int *rssi, int wait_ms) {
  (void)K; rx_pkt r;
  if (xQueueReceive(rxq, &r, wait_ms > 0 ? pdMS_TO_TICKS(wait_ms) : 0) != pdTRUE) return 0;
  int n = r.n < cap ? r.n : cap; memcpy(p, r.p, (size_t)n); *rssi = r.rssi; memcpy(last_src, r.src, sizeof last_src); return n;
}
static int en_send(pk_link *K, const uint8_t *p, int n) { (void)K; return esp_now_send(have_peer ? peer : bcast, p, (size_t)n) == ESP_OK ? 0 : -1; }
static void en_taken(pk_link *K) {                      /* (the radio task, as plink takes a packet) */
  if (have_peer && !memcmp(peer, last_src, sizeof peer)) return;
  if (!memcmp(last_src, bcast, sizeof bcast) || !(last_src[0] | last_src[1] | last_src[2] | last_src[3] | last_src[4] | last_src[5])) return;
  esp_now_peer_info_t pi; memset(&pi, 0, sizeof pi);
  memcpy(pi.peer_addr, last_src, sizeof last_src); pi.channel = 0; pi.ifidx = WIFI_IF_STA; pi.encrypt = false;
  esp_err_t e = esp_now_is_peer_exist(last_src) ? ESP_OK : esp_now_add_peer(&pi);
  if (e != ESP_OK) { pk_say(K, "ESP-NOW: can't add the other end (%s): broadcasting", esp_err_to_name(e)); have_peer = 0; return; }
  if (have_peer) esp_now_del_peer(peer);
  memcpy(peer, last_src, sizeof peer); have_peer = 1;
  pk_say(K, "ESP-NOW: talking to %02x:%02x:%02x:%02x:%02x:%02x", peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
}

static pk_link *KP;                     /* (on the heap: static DRAM is short on the ESP32) */
radio_io *radio_espnow_start(const rlink_cfg *L, int role, const char *bind, esp_radio_say say) {
  if (!KP && !(KP = calloc(1, sizeof *KP))) { if (say) say("ESP-NOW: no memory"); return 0; }
  pk_init(KP, L->lr ? "ESP-NOW, long range" : "ESP-NOW", role, bind, say);
  rxq = xQueueCreate(RXQ, sizeof(rx_pkt));
  esp_err_t e = rxq ? wifi_start(WIFI_MODE_STA, 0) : ESP_ERR_NO_MEM;
  if (e == ESP_OK) e = esp_wifi_set_channel((uint8_t)L->channel, WIFI_SECOND_CHAN_NONE);
  if (e == ESP_OK && L->lr) e = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (e == ESP_OK) e = esp_now_init();
  if (e == ESP_OK) e = esp_now_register_recv_cb(on_recv);
  if (e == ESP_OK) {
    esp_now_peer_info_t peer; memset(&peer, 0, sizeof peer);
    memcpy(peer.peer_addr, bcast, sizeof bcast); peer.channel = 0; peer.ifidx = WIFI_IF_STA; peer.encrypt = false;   /* (channel 0: the one we're on) */
    e = esp_now_add_peer(&peer);
  }
  if (e != ESP_OK) { pk_say(KP, "ESP-NOW didn't start: %s", esp_err_to_name(e)); return 0; }
  KP->recv = en_recv; KP->send = en_send; KP->taken = en_taken; have_peer = 0;
  uint8_t mac[6] = { 0 }; esp_wifi_get_mac(WIFI_IF_STA, mac);
  pk_say(KP, "radio: ESP-NOW on Wi-Fi channel %d%s, as %02x:%02x:%02x:%02x:%02x:%02x", L->channel,
         L->lr ? ", long range (the other end must be lr too)" : "", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return &KP->io;
}
