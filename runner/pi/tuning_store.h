#ifndef PI_TUNING_STORE_H
#define PI_TUNING_STORE_H
#include "pid_tuning.h"
#include "rn.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
/* Explicitly saved, CRC-protected gains bound to both the airframe and
 * navigation config. Atomic replacement preserves the previous file if saving
 * fails. Not a firmware flash write. */
static int tuning_file(const char *path, float *g, uint32_t af, uint32_t nav,
                       int save) {
  uint32_t words[17] = {0x54504644u, PID_VERSION, af, nav}; /* DFPT */
  float pg[9];
  for (int i = 0; i < 9; i++)
    pg[i] = g[9 + i / 3];
  if (save && (!pid_valid(g, 0) || !pid_valid(pg, 1)))
    return -1;
  if (save) {
    memcpy(words + 4, g, 48);
    words[16] = rn_crc32((uint8_t *)words, 64);
    char tmp[1024];
    if (snprintf(tmp, sizeof tmp, "%s.tmp", path) >= (int)sizeof tmp)
      return -1;
    FILE *f = fopen(tmp, "wb");
    if (!f)
      return -1;
    int ok = fwrite(words, 1, sizeof words, f) == sizeof words;
    ok = fflush(f) == 0 && ok;
    ok = fsync(fileno(f)) == 0 && ok;
    ok = fclose(f) == 0 && ok;
    if (ok)
      ok = rename(tmp, path) == 0;
    if (!ok)
      unlink(tmp);
    return ok ? 0 : -1;
  }
  FILE *f = fopen(path, "rb");
  if (!f)
    return -1;
  int ok = fread(words, 1, sizeof words, f) == sizeof words && fgetc(f) == EOF;
  fclose(f);
  if (!ok || words[0] != 0x54504644u || words[1] != PID_VERSION ||
      words[2] != af || words[3] != nav ||
      words[16] != rn_crc32((uint8_t *)words, 64))
    return -1;
  memcpy(g, words + 4, 48);
  for (int i = 0; i < 9; i++)
    pg[i] = g[9 + i / 3];
  return pid_valid(g, 0) && pid_valid(pg, 1) ? 0 : -1;
}
#endif
