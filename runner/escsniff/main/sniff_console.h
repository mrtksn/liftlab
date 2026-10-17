/* Command line framing shared by the UART/native USB readers and host tests. */
#ifndef SNIFF_CONSOLE_H
#define SNIFF_CONSOLE_H
#include <string.h>

#define SNIFF_LINE_SIZE 64
typedef struct {
  char text[SNIFF_LINE_SIZE];
  unsigned len;
  int after_cr;
} sniff_console;

/* Returns one complete line, including an empty Enter. Treat CRLF as one
 * terminator even when its two bytes arrive in different reads. */
static inline int sniff_console_feed(sniff_console *s, unsigned char c, char out[SNIFF_LINE_SIZE]) {
  if (c == '\n' && s->after_cr) { s->after_cr = 0; return 0; }
  s->after_cr = c == '\r';
  if (c == '\r' || c == '\n') {
    memcpy(out, s->text, s->len);
    out[s->len] = 0;
    s->len = 0;
    return 1;
  }
  if (s->len < SNIFF_LINE_SIZE - 2) s->text[s->len++] = (char)c;
  return 0;
}
#endif
