/* Drones talking to each other over ESP-NOW (fc/peer.h): see esp_radio.h. The radio's part only: the packets
 * peer.c has due go out (a beacon to everyone, the rest to one drone, which the radio then tries up to a few times),
 * the ones that come go in. ESP-NOW is shared with the pilot's link when that is ESP-NOW too (radio_espnow.c hands
 * over what starts with peer.c's mark); otherwise this starts it, a station on the peers' channel. */
#include "esp_radio.h"
#include "radio_cfg.h"
#include "peer.h"
#include "fleet.h"
#include "wifi_start.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { uint8_t n; int8_t rssi; uint8_t src[6]; uint8_t p[PEER_MTU]; } peer_rx;
#define RXQ 12
static QueueHandle_t rxq;
static SemaphoreHandle_t lock;                          /* (the radio task polls; the link task asks for the table) */
static peer_net *PN;                                    /* (on the heap: static DRAM is short with Wi-Fi) */
static fleet_link FK;                                   /* the fleet program's link (fleet.h): what the Pi's program says */
static volatile uint32_t rx_lost, send_fail, to_all;
static esp_radio_say SAY;
static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
/* The drones we added as ESP-NOW peers (a packet to an address needs one; at most 20 in all, the pilot's link's among
 * them): the one unused longest makes room. Without room, a packet goes to everyone: peer.c's header names the drone
 * it's for, and the others drop it. */
#define ADDED 8
static struct { uint8_t a[6]; int used; int64_t t; } added[ADDED];

static double now_s(void) { return esp_timer_get_time() * 1e-6; }
static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *fmt, ...) { if (!SAY) return; char s[96]; va_list a; va_start(a, fmt); vsnprintf(s, sizeof s, fmt, a); va_end(a); SAY(s); }

/* (the Wi-Fi task: only into the queue) */
void radio_peer_rx(const uint8_t src[6], const uint8_t *p, int n, int rssi) {
  if (!rxq || n < 1 || n > PEER_MTU) return;
  peer_rx r; r.n = (uint8_t)n; r.rssi = (int8_t)rssi; memcpy(r.src, src, 6); memcpy(r.p, p, (size_t)n);
  if (xQueueSend(rxq, &r, 0) != pdTRUE) rx_lost++;
}
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int n) {   /* (ESP-NOW started here: all of it is ours) */
  if (info && info->src_addr) radio_peer_rx(info->src_addr, data, n, info->rx_ctrl ? info->rx_ctrl->rssi : 0);
}
static int reach(const uint8_t a[6]) {                  /* an ESP-NOW peer for a, if there's room: 1 */
  int64_t t = esp_timer_get_time(), old = 0; int k = -1;
  for (int i = 0; i < ADDED; i++) if (added[i].used && !memcmp(added[i].a, a, 6)) { added[i].t = t; return 1; }
  if (esp_now_is_peer_exist(a)) return 1;               /* (the pilot's link's: not ours to manage) */
  for (int i = 0; i < ADDED; i++) { if (!added[i].used) { k = i; break; } if (k < 0 || added[i].t < old) { k = i; old = added[i].t; } }
  if (added[k].used) { esp_now_del_peer(added[k].a); added[k].used = 0; }
  esp_now_peer_info_t pi; memset(&pi, 0, sizeof pi);
  memcpy(pi.peer_addr, a, 6); pi.channel = 0; pi.ifidx = WIFI_IF_STA; pi.encrypt = false;   /* (channel 0: the one we're on) */
  if (esp_now_add_peer(&pi) != ESP_OK) return 0;
  memcpy(added[k].a, a, 6); added[k].used = 1; added[k].t = t; return 1;
}

int radio_peer_start(int channel, const char *fleet, esp_radio_say s) {
  SAY = s;
  if (!PN && !(PN = calloc(1, sizeof *PN))) { say("peers: no memory"); return -1; }
  if (!(rxq = xQueueCreate(RXQ, sizeof(peer_rx))) || !(lock = xSemaphoreCreateMutex())) { say("peers: no memory"); return -1; }
  int shared = radio_espnow_channel();
  esp_err_t e = ESP_OK;
  if (shared && shared != channel) { say("peers: on channel %d, but the ESP-NOW link is on %d: off", channel, shared); return -1; }
  if (!shared) {                                        /* ESP-NOW for the peers alone */
    e = wifi_start(WIFI_MODE_STA, 0);
    if (e == ESP_OK) e = esp_wifi_set_channel((uint8_t)channel, WIFI_SECOND_CHAN_NONE);
    if (e == ESP_OK) e = esp_now_init();
    if (e == ESP_OK) e = esp_now_register_recv_cb(on_recv);
    if (e == ESP_OK) {
      esp_now_peer_info_t pi; memset(&pi, 0, sizeof pi);
      memcpy(pi.peer_addr, bcast, 6); pi.channel = 0; pi.ifidx = WIFI_IF_STA; pi.encrypt = false;
      e = esp_now_add_peer(&pi);
    }
    if (e != ESP_OK) { say("peers: ESP-NOW didn't start: %s", esp_err_to_name(e)); return -1; }
  }
  uint8_t mac[6] = { 0 }; esp_wifi_get_mac(WIFI_IF_STA, mac);
  char name[PEER_NAME]; snprintf(name, sizeof name, "drone %02X%02X", mac[4], mac[5]);
  peer_net *N = PN; peer_init(N, peer_id_of(mac), esp_radio_session(), name, rcfg_bind(fleet));
  say("peers: on channel %d%s as \"%s\" (node %08lx): other drones of this fleet are found and talked to", channel, shared ? " (with the ESP-NOW link)" : "", name, (unsigned long)N->id);
  if (rcfg_bind_default(fleet)) say("peers: the fleet phrase is the default (liftlab): set fleet=YOUR PHRASE on each of your drones");
  return 0;
}
int radio_peer_on(void) { return PN && lock; }

void radio_peer_poll(const float *vals, int n) {
  if (!radio_peer_on()) return;
  if (xSemaphoreTake(lock, 0) != pdTRUE) return;        /* (the table being read: next time) */
  peer_net *N = PN; double t = now_s();
  if (vals && n >= FLEET_HEAD) fleet_link_publish(&FK, N, vals, t);   /* (the flight core's three, then the navigation's and its program's) */
  else if (vals) peer_publish(N, vals, n);
  peer_rx r; while (xQueueReceive(rxq, &r, 0) == pdTRUE) peer_from_air(N, r.src, r.p, r.n, r.rssi, t);
  static uint8_t p[PEER_MTU]; uint8_t a[6]; int m;
  for (int k = 0; k < 8 && (m = peer_to_air(N, t, a, p, sizeof p)) > 0; k++) {
    const uint8_t *to = a;
    if (memcmp(a, bcast, 6) && !reach(a)) { to = bcast; to_all++; }
    if (esp_now_send(to, p, (size_t)m) != ESP_OK) send_fail++;
  }
  xSemaphoreGive(lock);
}

static const char *const STATE[] = { "lost", "heard", "stale", "connected" };
int radio_peer_status(char *out, int n) {
  if (!radio_peer_on()) { snprintf(out, (size_t)n, "peers: off (peers=CHANNEL to find the other drones of the fleet)"); return 0; }
  xSemaphoreTake(lock, portMAX_DELAY);
  peer_net *N = PN; double t = now_s(); int k = 0, c = 0;
  #define APP(...) do { if (k < n) { int w = snprintf(out + k, (size_t)(n - k), __VA_ARGS__); if (w > 0) k += w < n - k ? w : n - k; } } while (0)
  for (int i = 0; i < PEER_MAX; i++) c += N->P[i].used;
  APP("peers: \"%s\" (node %08lx), %d in the table; packets sent %lu (failed %lu, to everyone for want of room %lu), taken %lu, bad %lu, replays %lu, resent %lu, dropped %lu (queue full %lu)",
      N->name, (unsigned long)N->id, c, (unsigned long)N->N.sent, (unsigned long)send_fail, (unsigned long)to_all, (unsigned long)N->N.got, (unsigned long)N->N.bad,
      (unsigned long)N->N.replays, (unsigned long)N->N.resent, (unsigned long)N->N.dropped, (unsigned long)rx_lost);
  for (int i = 0; i < PEER_MAX; i++) {
    const peer_t *P = &N->P[i]; int s = peer_state(N, i, t); if (s < 0) continue;
    APP("\n  %08lx \"%s\": %s, heard %.1f s ago, LQ %d%% (it hears us %d%%) %d dBm", (unsigned long)P->id, P->name, STATE[s], t - P->t_heard, peer_lq(N, i, t), P->heard_us, P->rssi);
    if (P->rtt > 0) APP(", ping %.1f ms", P->rtt * 1000);
    if (P->nvals) { APP(", values"); for (int v = 0; v < P->nvals && v < 6; v++) APP(" %g", (double)P->vals[v]); if (P->nvals > 6) APP(" …"); APP(" (%.1f s old)", t - P->t_vals); }
  }
  #undef APP
  xSemaphoreGive(lock);
  return c;
}
int radio_peer_pack(const float head[3], float *out) {
  if (!radio_peer_on()) return 0;
  xSemaphoreTake(lock, portMAX_DELAY); int n = fleet_link_pack(PN, now_s(), head, out); xSemaphoreGive(lock); return n;
}
int radio_peer_apply(const float *in, int n) {
  if (!radio_peer_on()) return -1;
  xSemaphoreTake(lock, portMAX_DELAY); int e = fleet_link_apply(&FK, PN, in, n, now_s()); xSemaphoreGive(lock); return e;
}
int radio_peer_ping(uint32_t id) {
  if (!radio_peer_on()) return -1;
  xSemaphoreTake(lock, portMAX_DELAY); int e = peer_ping(PN, id, now_s()); xSemaphoreGive(lock); return e;
}
int radio_peer_send(uint32_t to, const uint8_t *msg, int n) {
  if (!radio_peer_on()) return -1;
  xSemaphoreTake(lock, portMAX_DELAY); int e = peer_send(PN, to, msg, n); xSemaphoreGive(lock); return e;
}
int radio_peer_recv(uint32_t *from, uint8_t *msg, int cap) {
  if (!radio_peer_on()) return -1;
  xSemaphoreTake(lock, portMAX_DELAY); int e = peer_recv(PN, from, msg, cap); xSemaphoreGive(lock); return e;
}
