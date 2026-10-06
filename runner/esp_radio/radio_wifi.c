/* Wi-Fi, UDP: see esp_radio.h. */
#include "radio_packet.h"
#include "radio_cfg.h"
#include "wifi_start.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "lwip/sockets.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct {
  int sock;
  struct sockaddr_in from;     /* where the packet recv gave last came from */
  struct sockaddr_in to; int have_to;   /* where ours go: the drone's address (the command module), the sender of the last packet taken (the drone) */
  int rssi; double t_rssi; int ap;
  char ssid[RCFG_SSID_N];
  volatile int joined;
} udp_t;
static udp_t U;
static pk_link *KP;                     /* (on the heap: static DRAM is short on the ESP32) */

static int ud_recv(pk_link *k, uint8_t *p, int cap, int *rssi, int wait_ms) {
  udp_t *u = k->t;
  double t = pk_now();
  if (t - u->t_rssi > 0.5) {                            /* the signal, now and then (the driver knows it per station) */
    u->t_rssi = t; u->rssi = 0;
    if (u->ap) { wifi_sta_list_t l; if (esp_wifi_ap_get_sta_list(&l) == ESP_OK && l.num > 0) u->rssi = l.sta[0].rssi; }
    else { wifi_ap_record_t a; if (esp_wifi_sta_get_ap_info(&a) == ESP_OK) u->rssi = a.rssi; }
  }
  if (wait_ms > 0) {
    fd_set r; FD_ZERO(&r); FD_SET(u->sock, &r); struct timeval tv = { 0, wait_ms * 1000 };
    if (select(u->sock + 1, &r, 0, 0, &tv) <= 0) return 0;
  }
  socklen_t fl = sizeof u->from;
  int n = recvfrom(u->sock, p, (size_t)cap, MSG_DONTWAIT, (struct sockaddr *)&u->from, &fl);
  if (n <= 0) return 0;
  *rssi = u->rssi; return n;
}
static void ud_taken(pk_link *k) {                      /* (the drone: it answers whoever plink took a packet from last) */
  udp_t *u = k->t;
  if (k->L.C.role != PLINK_DRONE) return;
  if (!u->have_to || u->to.sin_addr.s_addr != u->from.sin_addr.s_addr || u->to.sin_port != u->from.sin_port) {
    u->to = u->from; u->have_to = 1;
    char ip[16]; inet_ntoa_r(u->from.sin_addr, ip, sizeof ip);
    pk_say(k, "Wi-Fi: the pilot's end is %s, port %d", ip, ntohs(u->from.sin_port));
  }
}
static int ud_send(pk_link *k, const uint8_t *p, int n) {
  udp_t *u = k->t;
  if (!u->have_to) return 0;                            /* (the drone, before anyone has talked to it) */
  if (!u->ap && !u->joined) return -1;
  return sendto(u->sock, p, (size_t)n, 0, (struct sockaddr *)&u->to, sizeof u->to) == n ? 0 : -1;
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
  (void)arg;
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
  else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *d = data;
    if (U.joined) pk_say(KP, "Wi-Fi: left %s (reason %d); joining it again", U.ssid, d ? d->reason : 0);
    else { static int said; if (!said++) pk_say(KP, "Wi-Fi: can't join %s yet (reason %d): is it there, the password right? Trying on", U.ssid, d ? d->reason : 0); }
    U.joined = 0; esp_wifi_connect();
  }
  else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t *g = data; U.joined = 1;
    char ip[16]; esp_ip4addr_ntoa(&g->ip_info.ip, ip, sizeof ip);
    pk_say(KP, "Wi-Fi: joined %s as %s", U.ssid, ip);
  }
  else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
    const wifi_event_ap_staconnected_t *c = data;
    pk_say(KP, "Wi-Fi: %02x:%02x:%02x:%02x:%02x:%02x joined %s", c->mac[0], c->mac[1], c->mac[2], c->mac[3], c->mac[4], c->mac[5], U.ssid);
  }
  else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
    const wifi_event_ap_stadisconnected_t *c = data;
    pk_say(KP, "Wi-Fi: %02x:%02x:%02x:%02x:%02x:%02x left %s", c->mac[0], c->mac[1], c->mac[2], c->mac[3], c->mac[4], c->mac[5], U.ssid);
  }
}

radio_io *radio_wifi_start(const rlink_cfg *L, int role, const char *phrase, const char *ssid, const char *pass, const char *drone_ip, esp_radio_say say) {
  int ap = role == PLINK_DRONE && !L->sta;
  if (!KP && !(KP = calloc(1, sizeof *KP))) { if (say) say("Wi-Fi: no memory"); return 0; }
  pk_init(KP, ap ? "Wi-Fi (UDP), its own network" : "Wi-Fi (UDP)", role, phrase, say);
  memset(&U, 0, sizeof U); U.sock = -1; U.ap = ap; U.t_rssi = -1e9;
  KP->t = &U; KP->recv = ud_recv; KP->send = ud_send; KP->taken = ud_taken;
  int weak = 0; const char *pw = rcfg_pass(pass, phrase, &weak);
  if (ap) {
    if (ssid[0]) snprintf(U.ssid, sizeof U.ssid, "%s", ssid);
    else { uint8_t mac[6] = { 0 }; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP); rcfg_default_ssid(U.ssid, mac); }
  } else {
    if (!ssid[0]) { pk_say(KP, role == PLINK_DRONE ? "radio=wifi,sta: which network? set wifi=SSID,PASSWORD" : "Wi-Fi: which network? set wifi=SSID[,PASSWORD] (the drone's: LiftLab-XXXX, as it says at power-on)"); return 0; }
    snprintf(U.ssid, sizeof U.ssid, "%s", ssid);
  }
  struct sockaddr_in peer = { 0 };
  if (role == PLINK_GROUND) {
    uint8_t q[4]; if (rcfg_ip_parse(0, q, drone_ip && drone_ip[0] ? drone_ip : RCFG_DRONE_DEFAULT, 0, 0)) { pk_say(KP, "Wi-Fi: drone=%s isn't an address", drone_ip); return 0; }
    peer.sin_family = AF_INET; peer.sin_port = htons(RLINK_UDP_PORT);
    peer.sin_addr.s_addr = htonl((uint32_t)q[0] << 24 | (uint32_t)q[1] << 16 | (uint32_t)q[2] << 8 | q[3]);
  }

  wifi_config_t c; memset(&c, 0, sizeof c);
  if (ap) {
    memcpy(c.ap.ssid, U.ssid, strlen(U.ssid)); c.ap.ssid_len = (uint8_t)strlen(U.ssid);
    memcpy(c.ap.password, pw, strlen(pw)); c.ap.channel = (uint8_t)L->channel;
    c.ap.authmode = WIFI_AUTH_WPA2_PSK; c.ap.max_connection = 4;
  } else {
    memcpy(c.sta.ssid, U.ssid, strlen(U.ssid)); memcpy(c.sta.password, pw, strlen(pw));
  }
  esp_err_t e = esp_event_loop_create_default(); if (e == ESP_ERR_INVALID_STATE) e = ESP_OK;
  if (e == ESP_OK) e = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, 0);
  if (e == ESP_OK) e = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, 0);
  if (e == ESP_OK) e = wifi_start(ap ? WIFI_MODE_AP : WIFI_MODE_STA, &c);
  if (e != ESP_OK) { pk_say(KP, "Wi-Fi didn't start: %s", esp_err_to_name(e)); return 0; }

  U.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in me = { 0 }; me.sin_family = AF_INET; me.sin_addr.s_addr = htonl(INADDR_ANY);
  me.sin_port = htons(role == PLINK_DRONE ? RLINK_UDP_PORT : 0);
  if (U.sock < 0 || bind(U.sock, (struct sockaddr *)&me, sizeof me) < 0) { pk_say(KP, "Wi-Fi: no UDP socket (%d)", errno); return 0; }
  fcntl(U.sock, F_SETFL, O_NONBLOCK);
  if (role == PLINK_GROUND) { U.to = peer; U.have_to = 1; }

  if (ap) pk_say(KP, "radio: Wi-Fi network %s on channel %d, password %s; this drone is 192.168.4.1, UDP port %d", U.ssid, L->channel,
                 pass[0] ? "as set (wifi=)" : weak ? "liftlab1 (WEAK: set bind= to 8+ characters, or wifi=SSID,PASSWORD)" : "the binding phrase", RLINK_UDP_PORT);
  else if (role == PLINK_DRONE) pk_say(KP, "radio: Wi-Fi, joining %s; UDP port %d (the command module needs this drone's address: it says it once joined)", U.ssid, RLINK_UDP_PORT);
  else { char ip[16]; inet_ntoa_r(peer.sin_addr, ip, sizeof ip); pk_say(KP, "radio: Wi-Fi, joining %s; the drone at %s, UDP port %d", U.ssid, ip, RLINK_UDP_PORT); }
  return &KP->io;
}
