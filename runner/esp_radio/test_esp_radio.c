/* Tests for the ESP32 packet links' portable parts: the USB port's text and frames apart (usb_split.c), the settings
 * (radio_cfg.h).
 *   cc -O2 -Wall -Wextra -I.. -I../fc -o test_esp_radio test_esp_radio.c usb_split.c ../fc/crsf.c -lm && ./test_esp_radio */
#include "usb_split.h"
#include "radio_cfg.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

typedef struct { int frames, lines; uint8_t f[16][CRSF_MAX_FRAME]; int fn[16]; char l[16][USB_SPLIT_LINE]; } got;
static void on_frame(void *c, const uint8_t *f, int n) { got *G = c; if (G->frames < 16) { memcpy(G->f[G->frames], f, (size_t)n); G->fn[G->frames] = n; } G->frames++; }
static void on_line(void *c, char *l) { got *G = c; if (G->lines < 16) snprintf(G->l[G->lines], USB_SPLIT_LINE, "%s", l); G->lines++; }

static void feed(usb_split *S, got *G, const void *b, int n, double t) {
  usb_split_out O = { on_frame, on_line, G };
  usb_split_feed(S, (const uint8_t *)b, n, t, &O);
}
static void feed_bytewise(usb_split *S, got *G, const uint8_t *b, int n, double t) { for (int i = 0; i < n; i++) feed(S, G, b + i, 1, t); }

int main(void) {
  uint8_t rc[CRSF_MAX_FRAME], cmd[CRSF_MAX_FRAME]; float ch[16] = { 0.5f, -0.25f };
  int nrc = crsf_rc(rc, CRSF_ADDR_FC, ch);
  uint8_t pl[8] = { CRSF_EXT_CMD, 0xEA, 7, 1, 2, 3 }; int ncmd = crsf_frame(cmd, CRSF_ADDR_FC, CRSF_EXT, pl, 6);

  printf("usb_split: text and frames\n");
  { usb_split S; usb_split_init(&S); got G = { 0 };
    feed(&S, &G, "show\r\nset bind=my phrase\n\n", 26, 0);
    CHECK(G.lines == 2 && !strcmp(G.l[0], "show") && !strcmp(G.l[1], "set bind=my phrase") && !G.frames, "lines (CR LF, empty lines skipped): %d", G.lines);
    CHECK(!usb_split_frames_now(&S, 0), "no frames: not bridging");
    feed(&S, &G, rc, nrc, 1.0); feed(&S, &G, cmd, ncmd, 1.0);
    CHECK(G.frames == 2 && G.fn[0] == nrc && !memcmp(G.f[0], rc, (size_t)nrc) && G.fn[1] == ncmd && !memcmp(G.f[1], cmd, (size_t)ncmd), "two frames back to back");
    CHECK(usb_split_frames_now(&S, 1.5) && !usb_split_frames_now(&S, 2.1), "bridging for a second after a frame");
    G = (got){ 0 }; feed_bytewise(&S, &G, rc, nrc, 3.0);
    CHECK(G.frames == 1 && !G.lines, "a frame a byte at a time");
    /* text between frames, and a frame in the middle of a line */
    G = (got){ 0 }; uint8_t mix[256]; int k = 0;
    memcpy(mix + k, "stat", 4); k += 4; memcpy(mix + k, rc, (size_t)nrc); k += nrc; memcpy(mix + k, "us\n", 3); k += 3;
    feed(&S, &G, mix, k, 4.0);
    CHECK(G.frames == 1 && G.lines == 1 && !strcmp(G.l[0], "us"), "a frame inside a line of text: the frame whole, the line's start dropped");
  }
  printf("usb_split: what isn't a frame\n");
  { usb_split S; usb_split_init(&S); got G = { 0 };
    uint8_t bad[CRSF_MAX_FRAME]; memcpy(bad, rc, (size_t)nrc); bad[nrc - 1] ^= 0x55;   /* a wrong CRC */
    feed(&S, &G, bad, nrc, 0); feed(&S, &G, rc, nrc, 0);
    CHECK(G.frames == 1 && !memcmp(G.f[0], rc, (size_t)nrc) && S.bad >= 1, "a frame with a bad CRC is dropped; the good one after it is found (bad %u)", S.bad);
    /* a bad frame's tail hides a good one: found by looking again from the byte after the address */
    G = (got){ 0 }; uint8_t hid[128]; hid[0] = CRSF_ADDR_FC; hid[1] = (uint8_t)(nrc + 3); hid[2] = CRSF_RC; memcpy(hid + 3, rc, (size_t)nrc);
    int hn = 3 + nrc; while (hn < 2 + hid[1]) hid[hn++] = 'x';
    feed(&S, &G, hid, hn, 1); feed(&S, &G, "\n", 1, 1);   /* (the broken frame's text-like bytes: a line of junk) */
    CHECK(G.frames == 1 && !memcmp(G.f[0], rc, (size_t)nrc), "a good frame inside a bad one's bytes is found");
    /* UTF-8 typed in a terminal (0xC8 and 0xEE start two-byte and three-byte characters) */
    G = (got){ 0 }; const char *u = "goto 1 2 3 \xc8\x80 \xee\x80\x80 ok\n"; feed(&S, &G, u, (int)strlen(u), 2);
    CHECK(G.lines == 1 && !G.frames && !strcmp(G.l[0], "goto 1 2 3   ok"), "UTF-8 dropped, the line kept: '%s'", G.lines ? G.l[0] : "");
    /* a stray address byte, then text: the frame stalls and is given up, the text comes through */
    G = (got){ 0 }; uint8_t st[2] = { CRSF_ADDR_FC, 20 }; feed(&S, &G, st, 2, 3.0);
    feed(&S, &G, "show\n", 5, 3.01);
    CHECK(G.lines == 0, "text right after a stray frame start is held (it could be the frame's)");
    feed(&S, &G, 0, 0, 3.2);
    CHECK(G.lines == 1 && !strcmp(G.l[0], "show"), "given up after %.1f s: the text comes through", USB_SPLIT_STALL);
    /* a line too long: cut */
    G = (got){ 0 }; char lng[300]; memset(lng, 'a', sizeof lng); lng[299] = '\n'; feed(&S, &G, lng, 300, 4);
    CHECK(G.lines == 2 && (int)strlen(G.l[0]) == USB_SPLIT_LINE - 1, "a too-long line is cut, the rest the next line");
    /* noise: random bytes never crash it, and a good frame after them is found */
    G = (got){ 0 }; uint32_t x = 12345; uint8_t nz[4000];
    for (int i = 0; i < 4000; i++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; nz[i] = (uint8_t)x; }
    feed(&S, &G, nz, 4000, 5); feed(&S, &G, 0, 0, 6); int before = G.frames; feed(&S, &G, rc, nrc, 6.0);
    CHECK(G.frames == before + 1 && S.fn == 0, "after 4000 random bytes a good frame is still found");
  }
  printf("radio_cfg: settings\n");
  { char b[RCFG_BIND_N] = "x", ssid[RCFG_SSID_N] = "", pass[RCFG_PASS_N] = "", err[120];
    CHECK(!rcfg_bind_parse(b, "my secret phrase", err, sizeof err) && !strcmp(b, "my secret phrase"), "bind=my secret phrase");
    CHECK(rcfg_bind_parse(b, "", err, sizeof err) && rcfg_bind_parse(b, "0123456789012345678901234567890x", err, sizeof err) && !strcmp(b, "my secret phrase"), "bind: empty and 32 characters refused, unchanged");
    CHECK(!rcfg_bind_parse(b, "0123456789012345678901234567890", err, sizeof err), "bind: 31 characters");
    CHECK(rcfg_bind_parse(b, " lead", err, sizeof err) && rcfg_bind_parse(b, "tab\there", err, sizeof err) && rcfg_bind_parse(b, "\xc3\xa9t\xc3\xa9", err, sizeof err), "bind: leading space, control, non-ASCII refused");
    CHECK(rcfg_bind_default("liftlab") && rcfg_bind_default("") && !rcfg_bind_default("liftlab2"), "the default phrase is told");
    CHECK(!rcfg_wifi_parse(ssid, pass, "Home Net,hunter2hunter2", err, sizeof err) && !strcmp(ssid, "Home Net") && !strcmp(pass, "hunter2hunter2"), "wifi=SSID,PASSWORD");
    CHECK(!rcfg_wifi_parse(ssid, pass, "LiftLab-A1B2", err, sizeof err) && !strcmp(ssid, "LiftLab-A1B2") && !pass[0], "wifi=SSID: the default password");
    CHECK(!rcfg_wifi_parse(ssid, pass, "Net,pa,ss,word", err, sizeof err) && !strcmp(ssid, "Net") && !strcmp(pass, "pa,ss,word"), "wifi: the password may hold commas");
    CHECK(rcfg_wifi_parse(ssid, pass, "Net,short", err, sizeof err) && rcfg_wifi_parse(ssid, pass, ",password1", err, sizeof err)
          && rcfg_wifi_parse(ssid, pass, "012345678901234567890123456789012,password1", err, sizeof err) && !strcmp(ssid, "Net"), "wifi: short password, no SSID, 33-character SSID refused");
    CHECK(!rcfg_wifi_parse(ssid, pass, "", err, sizeof err) && !ssid[0] && !pass[0], "wifi= alone: the defaults");
    int weak = 0;
    CHECK(!strcmp(rcfg_pass("", "liftlab", &weak), "liftlab1") && weak, "default password with a short phrase: liftlab1, weak");
    CHECK(!strcmp(rcfg_pass("", "long enough phrase", &weak), "long enough phrase") && !weak, "default password: the phrase (8+ characters)");
    CHECK(!strcmp(rcfg_pass("explicit1", "liftlab", &weak), "explicit1") && !weak, "a password set wins");
    uint8_t mac[6] = { 0x24, 0x6f, 0x28, 0x01, 0xa1, 0x0b }; char d[RCFG_SSID_N]; rcfg_default_ssid(d, mac);
    CHECK(!strcmp(d, "LiftLab-A10B"), "default network name: %s", d);
    char ip[RCFG_IP_N]; uint8_t q[4];
    CHECK(!rcfg_ip_parse(ip, q, "192.168.4.1", err, sizeof err) && !strcmp(ip, "192.168.4.1") && q[0] == 192 && q[3] == 1, "drone=192.168.4.1");
    CHECK(rcfg_ip_parse(ip, q, "192.168.4", err, sizeof err) && rcfg_ip_parse(ip, q, "256.1.1.1", err, sizeof err) && rcfg_ip_parse(ip, q, "1.2.3.4.5", err, sizeof err)
          && rcfg_ip_parse(ip, q, "1.2.3.4x", err, sizeof err) && rcfg_ip_parse(ip, q, "", err, sizeof err), "drone: bad addresses refused");
    char t1[4] = { 'a', 'b', 'c', 'd' }, t2[4] = { 'a', 0, 'c', 'd' };
    CHECK(!rcfg_terminated(t1, 4) && rcfg_terminated(t2, 4), "a stored string must end within its room");
  }
  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
