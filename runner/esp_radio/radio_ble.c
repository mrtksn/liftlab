/* Bluetooth LE (radio=ble): see esp_radio.h. The drone's ESP32 is a peripheral with one GATT service: the ground
 * writes its packets to one characteristic (write without response), the drone notifies its own on the other. The
 * command module's ESP32 is the central: it scans for that service with this binding phrase's mark in the
 * advertisement (so it finds your drone, not another's), connects (7.5–15 ms connection interval, 1 s supervision
 * timeout), asks for a 256-byte MTU, finds the two characteristics and subscribes. Lost, it scans again; the drone
 * advertises again. The packets are plink's (radio_packet.c), up to the MTU less 3 bytes; the radio's own link layer
 * acknowledges and retries within the connection. NimBLE, Espressif's lighter Bluetooth host. On the ESP32-S3 and C3
 * (the ESP32's Bluetooth controller takes 64 KB of static memory the flight code doesn't leave): elsewhere radio=ble
 * says so. */
#include "radio_packet.h"
#include "radio_cfg.h"
#include "sdkconfig.h"
#if CONFIG_BT_NIMBLE_ENABLED
#include "esp_bt.h"
#include "esp_mac.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* the service and its two characteristics: LiftLab's own 128-bit UUIDs (only their being the same at both ends matters) */
static const ble_uuid128_t SVC = BLE_UUID128_INIT(0x62, 0x6c, 0x74, 0x66, 0x69, 0x6c, 0x7e, 0x9a, 0x8b, 0x4e, 0x3d, 0x6b, 0x01, 0x00, 0x1f, 0x6c);
static const ble_uuid128_t CHR_UP = BLE_UUID128_INIT(0x62, 0x6c, 0x74, 0x66, 0x69, 0x6c, 0x7e, 0x9a, 0x8b, 0x4e, 0x3d, 0x6b, 0x02, 0x00, 0x1f, 0x6c);
static const ble_uuid128_t CHR_DOWN = BLE_UUID128_INIT(0x62, 0x6c, 0x74, 0x66, 0x69, 0x6c, 0x7e, 0x9a, 0x8b, 0x4e, 0x3d, 0x6b, 0x03, 0x00, 0x1f, 0x6c);
#define MFG_ID 0xFFFF            /* (the "no company" ID: the mark is ours) */

typedef struct { uint8_t n; int8_t rssi; uint8_t p[PLINK_MTU]; } rx_pkt;
static QueueHandle_t rxq;
static pk_link *KP;
static int role;
static uint8_t own_addr_type, mark[4];
static volatile uint16_t conn = BLE_HS_CONN_HANDLE_NONE, h_up, h_down, h_down_end;
static volatile int ready;       /* connected, and (the ground) found the characteristics and subscribed */
static volatile int rssi_now;
static char devname[20];
static void start(void);

static void got(const uint8_t *p, int n) {
  if (n < 1 || n > PLINK_MTU) return;
  rx_pkt r; r.n = (uint8_t)n; r.rssi = (int8_t)rssi_now; memcpy(r.p, p, (size_t)n);
  xQueueSend(rxq, &r, 0);
}
static void got_mbuf(struct os_mbuf *om) { uint8_t b[PLINK_MTU]; uint16_t n = 0; if (!ble_hs_mbuf_to_flat(om, b, sizeof b, &n)) got(b, n); }
static void use_mtu(uint16_t mtu) { int m = mtu - 3; if (m > PLINK_MTU) m = PLINK_MTU; if (m < 64) m = 64; KP->L.C.mtu = m; }

/* ── the drone: a peripheral ── */
static int chr_access(uint16_t c, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
  (void)c; (void)attr; (void)arg;
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) got_mbuf(ctxt->om);
  return 0;
}
static uint16_t down_val;
static const struct ble_gatt_svc_def svcs[] = {
  { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &SVC.u, .characteristics = (struct ble_gatt_chr_def[]) {
      { .uuid = &CHR_UP.u, .access_cb = chr_access, .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE },
      { .uuid = &CHR_DOWN.u, .access_cb = chr_access, .val_handle = &down_val, .flags = BLE_GATT_CHR_F_NOTIFY },
      { 0 } } },
  { 0 } };
static int gap_event(struct ble_gap_event *ev, void *arg);
static void advertise(void) {
  struct ble_hs_adv_fields f; memset(&f, 0, sizeof f);
  static uint8_t mfg[6]; mfg[0] = MFG_ID & 0xFF; mfg[1] = MFG_ID >> 8; memcpy(mfg + 2, mark, 4);
  f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  f.uuids128 = (ble_uuid128_t *)&SVC; f.num_uuids128 = 1; f.uuids128_is_complete = 1;
  f.mfg_data = mfg; f.mfg_data_len = sizeof mfg;
  ble_gap_adv_set_fields(&f);
  struct ble_hs_adv_fields r; memset(&r, 0, sizeof r);
  r.name = (uint8_t *)devname; r.name_len = (uint8_t)strlen(devname); r.name_is_complete = 1;
  ble_gap_adv_rsp_set_fields(&r);
  struct ble_gap_adv_params ap; memset(&ap, 0, sizeof ap);
  ap.conn_mode = BLE_GAP_CONN_MODE_UND; ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ap.itvl_min = BLE_GAP_ADV_ITVL_MS(30); ap.itvl_max = BLE_GAP_ADV_ITVL_MS(50);     /* (found within a tenth of a second) */
  ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &ap, gap_event, NULL);
}

/* ── the ground: a central ── */
static int adv_ours(const struct ble_gap_disc_desc *d) {
  struct ble_hs_adv_fields f;
  if (ble_hs_adv_parse_fields(&f, d->data, d->length_data)) return 0;
  int svc = 0; for (int i = 0; i < f.num_uuids128; i++) if (!ble_uuid_cmp(&f.uuids128[i].u, &SVC.u)) svc = 1;
  return svc && f.mfg_data_len == 6 && f.mfg_data[0] == (MFG_ID & 0xFF) && f.mfg_data[1] == (MFG_ID >> 8) && !memcmp(f.mfg_data + 2, mark, 4);
}
static void scan(void) {
  struct ble_gap_disc_params dp; memset(&dp, 0, sizeof dp); dp.passive = 1; dp.filter_duplicates = 0; dp.itvl = 0x60; dp.window = 0x50;
  ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &dp, gap_event, NULL);
}
static int on_subscribed(uint16_t c, const struct ble_gatt_error *e, struct ble_gatt_attr *a, void *arg) {
  (void)c; (void)a; (void)arg;
  if (e->status == 0) { ready = 1; pk_say(KP, "Bluetooth LE: connected to the drone (packets up to %d bytes)", KP->L.C.mtu); }
  else ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
  return 0;
}
static int on_chr(uint16_t c, const struct ble_gatt_error *e, const struct ble_gatt_chr *chr, void *arg) {
  (void)arg;
  if (e->status == 0 && chr) {
    if (!ble_uuid_cmp(&chr->uuid.u, &CHR_UP.u)) h_up = chr->val_handle;
    if (!ble_uuid_cmp(&chr->uuid.u, &CHR_DOWN.u)) h_down = chr->val_handle;
    return 0;
  }
  if (e->status == BLE_HS_EDONE && h_up && h_down) {          /* subscribe: its CCCD follows its value (NimBLE's layout, as the drone's) */
    uint8_t v[2] = { 1, 0 }; ble_gattc_write_flat(c, (uint16_t)(h_down + 1), v, 2, on_subscribed, NULL); return 0;
  }
  ble_gap_terminate(c, BLE_ERR_REM_USER_CONN_TERM);
  return 0;
}
static int on_svc(uint16_t c, const struct ble_gatt_error *e, const struct ble_gatt_svc *s, void *arg) {
  (void)arg;
  if (e->status == 0 && s) { h_down_end = s->end_handle; ble_gattc_disc_all_chrs(c, s->start_handle, s->end_handle, on_chr, NULL); }
  return 0;
}
static int on_mtu(uint16_t c, const struct ble_gatt_error *e, uint16_t mtu, void *arg) {
  (void)arg; if (e->status == 0) use_mtu(mtu);
  ble_gattc_disc_svc_by_uuid(c, &SVC.u, on_svc, NULL);
  return 0;
}

static int gap_event(struct ble_gap_event *ev, void *arg) {
  (void)arg;
  switch (ev->type) {
    case BLE_GAP_EVENT_DISC:
      if (role == PLINK_GROUND && conn == BLE_HS_CONN_HANDLE_NONE && adv_ours(&ev->disc)) {
        ble_gap_disc_cancel();
        struct ble_gap_conn_params cp; memset(&cp, 0, sizeof cp);
        cp.scan_itvl = 0x10; cp.scan_window = 0x10; cp.itvl_min = 6; cp.itvl_max = 12; cp.latency = 0; cp.supervision_timeout = 100;   /* (7.5–15 ms; 1 s) */
        if (ble_gap_connect(own_addr_type, &ev->disc.addr, 3000, &cp, gap_event, NULL)) scan();
      }
      return 0;
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status) { start(); return 0; }
      conn = ev->connect.conn_handle; h_up = h_down = 0; rssi_now = 0;
      if (role == PLINK_GROUND) ble_gattc_exchange_mtu(conn, on_mtu, NULL);
      else pk_say(KP, "Bluetooth LE: the command module connected");
      return 0;
    case BLE_GAP_EVENT_DISCONNECT:
      conn = BLE_HS_CONN_HANDLE_NONE; ready = 0;
      pk_say(KP, "Bluetooth LE: disconnected (reason 0x%x); %s", ev->disconnect.reason, role == PLINK_GROUND ? "looking for the drone again" : "advertising again");
      start(); return 0;
    case BLE_GAP_EVENT_DISC_COMPLETE: case BLE_GAP_EVENT_ADV_COMPLETE:
      if (conn == BLE_HS_CONN_HANDLE_NONE) start();
      return 0;
    case BLE_GAP_EVENT_MTU: use_mtu(ev->mtu.value); return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
      if (ev->subscribe.attr_handle == down_val) ready = ev->subscribe.cur_notify;
      return 0;
    case BLE_GAP_EVENT_NOTIFY_RX:
      if (role == PLINK_GROUND && ev->notify_rx.attr_handle == h_down) got_mbuf(ev->notify_rx.om);
      return 0;
  }
  return 0;
}
static void start(void) { if (role == PLINK_GROUND) scan(); else advertise(); }
static void on_sync(void) {
  ble_hs_util_ensure_addr(0); ble_hs_id_infer_auto(0, &own_addr_type);
  pk_say(KP, "radio: Bluetooth LE, %s", role == PLINK_GROUND ? "looking for the drone (its service, this binding phrase's mark)" : devname);
  start();
}
static void on_reset(int reason) { (void)reason; conn = BLE_HS_CONN_HANDLE_NONE; ready = 0; }
static void host_task(void *p) { (void)p; nimble_port_run(); nimble_port_freertos_deinit(); }

/* ── the transport for plink ── */
static int bt_recv(pk_link *K, uint8_t *p, int cap, int *rssi, int wait_ms) {
  (void)K; rx_pkt r;
  static double t_rssi; double t = pk_now();
  if (ready && conn != BLE_HS_CONN_HANDLE_NONE && t - t_rssi > 0.5) { int8_t v; if (!ble_gap_conn_rssi(conn, &v)) rssi_now = v; t_rssi = t; }
  if (xQueueReceive(rxq, &r, wait_ms > 0 ? pdMS_TO_TICKS(wait_ms) : 0) != pdTRUE) return 0;
  int n = r.n < cap ? r.n : cap; memcpy(p, r.p, (size_t)n); *rssi = r.rssi; return n;
}
static int bt_send(pk_link *K, const uint8_t *p, int n) {
  (void)K; if (!ready || conn == BLE_HS_CONN_HANDLE_NONE) return -1;
  if (role == PLINK_GROUND) return ble_gattc_write_no_rsp_flat(conn, h_up, p, (uint16_t)n) ? -1 : 0;
  struct os_mbuf *om = ble_hs_mbuf_from_flat(p, (uint16_t)n); if (!om) return -1;
  return ble_gatts_notify_custom(conn, down_val, om) ? -1 : 0;
}

radio_io *radio_ble_start(const rlink_cfg *L, int r, const char *bind, esp_radio_say say) {
  if (L->kind != RLINK_BLE) return 0;
  if (!KP && !(KP = calloc(1, sizeof *KP))) { if (say) say("Bluetooth LE: no memory"); return 0; }
  role = r;
  pk_init(KP, "Bluetooth LE", r, bind, say);
  KP->L.C.mtu = 64;                                             /* (until the MTU is agreed: BLE's least, 23, won't carry a channel packet) */
  uint8_t m[3] = { 'a', 'd', 'v' }; uint64_t h = plink_siphash(KP->L.C.k0, KP->L.C.k1, m, 3);
  for (int i = 0; i < 4; i++) mark[i] = (uint8_t)(h >> (8 * i));
  uint8_t mac[6] = { 0 }; esp_read_mac(mac, ESP_MAC_BT);
  snprintf(devname, sizeof devname, "LiftLab-%02X%02X", mac[4], mac[5]);
  rxq = xQueueCreate(16, sizeof(rx_pkt));
  esp_err_t e = rxq ? nimble_port_init() : ESP_ERR_NO_MEM;
  if (e != ESP_OK) { pk_say(KP, "Bluetooth LE didn't start: %s", esp_err_to_name(e)); return 0; }
  ble_hs_cfg.sync_cb = on_sync; ble_hs_cfg.reset_cb = on_reset;
  ble_att_set_preferred_mtu(256);
  if (r == PLINK_DRONE) {
    ble_svc_gap_init(); ble_svc_gatt_init();
    if (ble_gatts_count_cfg(svcs) || ble_gatts_add_svcs(svcs)) { pk_say(KP, "Bluetooth LE: the service didn't register"); return 0; }
  }
  ble_svc_gap_device_name_set(devname);
  KP->recv = bt_recv; KP->send = bt_send;
  nimble_port_freertos_init(host_task);
  return &KP->io;
}
/* Not Bluetooth this time: its memory back to the heap (on the ESP32, the controller's ~50 KB). Before anything else
 * of Bluetooth's, once. */
void radio_ble_release(void) { esp_bt_controller_mem_release(ESP_BT_MODE_BTDM); }
#else
/* (a chip built without Bluetooth: the classic ESP32) */
radio_io *radio_ble_start(const rlink_cfg *L, int r, const char *bind, esp_radio_say say) {
  (void)L; (void)r; (void)bind;
  if (say) say("Bluetooth LE: not on this chip's firmware (the ESP32's Bluetooth takes memory the flight code needs): use an ESP32-S3 or C3 at each end, or ESP-NOW");
  return 0;
}
void radio_ble_release(void) {}
#endif
