/* Files up the radio as text commands: see file_rx.h. No C library (the simulator's boards are freestanding). */
#include "file_rx.h"
#include "fc_core.h"     /* fc_fmt */
#include "../rn.h"       /* rn_crc32 */

static const char *after(const char *s, const char *p) { while (*p) if (*s++ != *p++) return 0; return s; }
static const char *sp(const char *s) { while (*s == ' ') s++; return s; }
static int eq(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return *a == *b; }
static const char *dec(const char *s, uint32_t *v) {
  s = sp(s); if (*s < '0' || *s > '9') return 0;
  uint64_t x = 0; while (*s >= '0' && *s <= '9') { x = 10 * x + (uint64_t)(*s++ - '0'); if (x > 0xFFFFFFFFu) return 0; }
  *v = (uint32_t)x; return s;
}
static const char *hex(const char *s, uint32_t *v) {
  s = sp(s); uint32_t x = 0; int n = 0;
  for (;; s++, n++) {
    int d = *s >= '0' && *s <= '9' ? *s - '0' : *s >= 'a' && *s <= 'f' ? *s - 'a' + 10 : *s >= 'A' && *s <= 'F' ? *s - 'A' + 10 : -1;
    if (d < 0) break;
    if (n == 8) return 0;
    x = x << 4 | (uint32_t)d;
  }
  if (!n) return 0; *v = x; return s;
}
static int b64v(char c) { return c >= 'A' && c <= 'Z' ? c - 'A' : c >= 'a' && c <= 'z' ? c - 'a' + 26 : c >= '0' && c <= '9' ? c - '0' + 52 : c == '+' ? 62 : c == '/' ? 63 : -1; }
/* base64 to bytes: how many, or −1 if it isn't base64 or won't fit */
static int unb64(const char *s, uint8_t *out, int cap) {
  int n = 0, bits = 0; uint32_t acc = 0;
  for (; *s && *s != ' '; s++) {
    if (*s == '=') { while (*s == '=') s++; break; }
    int v = b64v(*s); if (v < 0) return -1;
    acc = acc << 6 | (uint32_t)v; bits += 6;
    if (bits >= 8) { bits -= 8; if (n >= cap) return -1; out[n++] = (uint8_t)(acc >> bits); }
  }
  return *sp(s) ? -1 : n;
}

uint32_t file_rx_pieces(uint32_t size) { return (size + FILE_RX_PIECE - 1) / FILE_RX_PIECE; }
int file_rx_name_ok(const char *s) {
  int n = 0;
  if (*s == '.' || !*s) return 0;
  for (; s[n]; n++) {
    char c = s[n];
    if (n >= FILE_RX_NAME || !((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.')) return 0;
  }
  return 1;
}
int file_rx_begins(const char *line, char *name, uint32_t *size, uint32_t *crc) {
  const char *a = after(sp(line), "file ");
  if (!a) return 0;
  a = sp(a); int n = 0;
  while (a[n] && a[n] != ' ' && n < FILE_RX_NAME) { name[n] = a[n]; n++; }
  name[n] = 0;
  if (a[n] != ' ' || eq(name, "end")) return 0;
  const char *e = dec(a + n, size); if (!e) return 0;
  e = hex(e, crc); if (!e || *sp(e)) return 0;
  return 1;
}

static int has(const file_rx *R, uint32_t k) { return R->got[k >> 3] >> (k & 7) & 1; }

int file_rx_line(file_rx *R, const char *line, char *r, int cap) {
  const char *a;
  line = sp(line);
  if ((a = after(line, "fd ")) != 0) {
    uint32_t off; a = dec(a, &off);
    r[0] = 0;
    if (R->active != 1) { fc_fmt(r, cap, "fd: no file is coming (file NAME SIZE CRC first)"); return 1; }
    if (!a || *a != ' ' || off % FILE_RX_PIECE || off >= R->size) { fc_fmt(r, cap, "fd: not a piece of %s", R->name); return 1; }
    uint32_t want = R->size - off < FILE_RX_PIECE ? R->size - off : FILE_RX_PIECE;
    uint8_t b[FILE_RX_PIECE + 3]; int n = unb64(sp(a), b, sizeof b);
    if (n != (int)want) { fc_fmt(r, cap, "fd: the piece at %d of %s is damaged", (int)off, R->name); return 1; }
    for (int i = 0; i < n; i++) R->buf[off + (uint32_t)i] = b[i];
    uint32_t k = off / FILE_RX_PIECE; if (!has(R, k)) { R->got[k >> 3] |= (uint8_t)(1u << (k & 7)); R->n_got++; }
    return 1;
  }
  if (eq(line, "file")) {
    if (!R->active) fc_fmt(r, cap, "file: none coming");
    else if (R->active == 2) fc_fmt(r, cap, "file %s: done, %d bytes", R->name, (int)R->size);
    else fc_fmt(r, cap, "file %s: %d of %d pieces", R->name, (int)R->n_got, (int)file_rx_pieces(R->size));
    return 1;
  }
  if (eq(line, "file end")) {
    if (R->active != 1) { fc_fmt(r, cap, R->active ? "file %s: done already" : "file end: no file is coming", R->name); return 1; }
    uint32_t np = file_rx_pieces(R->size);
    if (R->n_got < np) {
      if (cap > FILE_RX_SAY + 1) cap = FILE_RX_SAY + 1;
      fc_fmt(r, cap, "file %s: missing %d of %d:", R->name, (int)(np - R->n_got), (int)np);
      int at = 0; while (r[at]) at++;
      for (uint32_t k = 0; k < np; k++) if (!has(R, k)) {
        char o[16]; fc_fmt(o, sizeof o, " %d", (int)(k * FILE_RX_PIECE));
        int m = 0; while (o[m]) m++;
        if (at + m >= cap) break;
        for (int i = 0; i <= m; i++) r[at + i] = o[i];
        at += m;
      }
      return 1;
    }
    if (rn_crc32(R->buf, R->size) != R->crc) {
      for (unsigned i = 0; i < sizeof R->got; i++) R->got[i] = 0;
      R->n_got = 0; fc_fmt(r, cap, "file %s: damaged (its CRC is wrong): send it again", R->name); return 1;
    }
    R->active = 2; fc_fmt(r, cap, "file %s: whole, %d bytes", R->name, (int)R->size);
    return 2;
  }
  if (!after(line, "file ")) return 0;
  char name[FILE_RX_NAME + 1]; uint32_t size, crc;
  if (!file_rx_begins(line, name, &size, &crc)) { fc_fmt(r, cap, "file NAME SIZE CRC (hex), fd OFFSET BASE64, file end"); return 1; }
  if (!file_rx_name_ok(name)) { fc_fmt(r, cap, "file: a name of letters, digits, - _ and . (no folders), up to %d", FILE_RX_NAME); return 1; }
  if (!size || size > FILE_RX_MAX) { fc_fmt(r, cap, "file %s: 1 to %d bytes", name, FILE_RX_MAX); return 1; }
  for (int i = 0; i <= FILE_RX_NAME; i++) { R->name[i] = name[i]; if (!name[i]) break; }
  R->size = size; R->crc = crc; R->n_got = 0; R->active = 1;
  for (unsigned i = 0; i < sizeof R->got; i++) R->got[i] = 0;
  fc_fmt(r, cap, "file %s: %d bytes in %d pieces, send them", name, (int)size, (int)file_rx_pieces(size));
  return 1;
}
