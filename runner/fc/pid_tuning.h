/* Hardware PID transactions for the bundled standard controller only.
 * Header-only so every native/WASM test uses the same lease and gain validation
 * without new link dependencies. */
#ifndef PID_TUNING_H
#define PID_TUNING_H
#include <stdint.h>
#define PID_VERSION 1
#define PID_LEASE .25f
#define PID_FRAME 12
/* Status: version/session/phase/support, gains[9], CRC halves, guided. */
#define PID_STATUS 16
/* gain order: P[3], D[3], I[3]. Position repeats its scalar gains across axes.
 */
typedef struct {
  float accepted[9], trial[9], left;
  uint32_t session;
  int enabled, pending;
} pid_tuning;
static inline void pid_defaults(float *g, int pos) {
  const float a[9] = {100, 100, 40, 16, 16, 10, 80, 80, 20};
  for (int i = 0; i < 9; i++)
    g[i] = pos ? (i < 3 ? 4 : i < 6 ? 3.6f : 1) : a[i];
}
static inline int pid_valid(const float *g, int pos) {
  for (int i = 0; i < 9; i++) {
    float lo = pos ? (i < 3 ? .05f : i < 6 ? .1f : 0) : (i < 3 ? 1 : 0);
    float hi = pos ? (i < 3   ? 50
                      : i < 6 ? 40
                              : 25)
                   : (i < 3   ? 2500
                      : i < 6 ? 200
                              : 2500);
    if (!((g[i] - g[i]) == 0) || g[i] < lo || g[i] > hi)
      return 0;
    if (pos && g[i] != g[i / 3 * 3])
      return 0;
  }
  return 1;
}
static inline const float *pid_gains(const pid_tuning *T) {
  return T->pending ? T->trial : T->accepted;
}
/* 0 cancel, 1 trial/refresh (same session and gains), 2 commit, 3 load while
 * disarmed. No implicit commit on expiry; a supervisor intervention also
 * cancels the trial. */
static inline int pid_frame(pid_tuning *T, const float *p, int n, int pos,
                            int supported, int flying, int disarmed) {
  if (n != PID_FRAME || p[0] != PID_VERSION || !supported ||
      !pid_valid(p + 3, pos))
    return -1;
  if (!(p[1] >= 0 && p[1] <= 3) || (int)p[1] != p[1] ||
      !(p[2] >= 1 && p[2] <= 16777215) || (uint32_t)p[2] != p[2])
    return -1;
  int action = (int)p[1];
  uint32_t id = (uint32_t)p[2];
  if (action == 0) {
    if (T->session == id)
      T->pending = 0;
    return 0;
  }
  if (action == 1) {
    if (!T->pending) {
      uint32_t newer = (id - T->session) & 0xffffffu;
      if (T->session && (!newer || newer > 0x7fffffu))
        return -1;
    }
    if (!flying || (T->pending && T->session != id) ||
        (!T->pending && T->session == id))
      return -1;
    if (T->pending)
      for (int i = 0; i < 9; i++)
        if (T->trial[i] != p[i + 3])
          return -1;
    T->session = id;
    for (int i = 0; i < 9; i++)
      T->trial[i] = p[i + 3];
    T->pending = 1;
    T->left = PID_LEASE;
    return 0;
  }
  if (action == 2) {
    if (flying && !T->pending && T->enabled && T->session == id) {
      for (int i = 0; i < 9; i++)
        if (T->accepted[i] != p[i + 3])
          return -1;
      return 0;
    }
    if (!flying || !T->pending || T->session != id || T->left <= 0)
      return -1;
    for (int i = 0; i < 9; i++)
      if (T->trial[i] != p[i + 3])
        return -1;
  } else if (!disarmed || T->pending)
    return -1;
  for (int i = 0; i < 9; i++)
    T->accepted[i] = p[i + 3];
  T->enabled = 1;
  T->pending = 0;
  T->session = id;
  return 0;
}
static inline void pid_tick(pid_tuning *T, float dt, int supported, int safe,
                            int pos) {
  if (!supported) {
    T->pending = T->enabled = 0;
    pid_defaults(T->accepted, pos);
    return;
  }
  if (!(dt > 0 && dt < 1)) {
    T->pending = 0;
    return;
  }
  if (T->pending) {
    T->left -= dt;
    if (T->left <= 0 || !safe)
      T->pending = 0;
  }
}
static inline void pid_status(const pid_tuning *T, int supported, float *p) {
  p[0] = PID_VERSION;
  p[1] = (float)T->session;
  p[2] = T->pending ? 1 : T->enabled ? 2 : 0;
  p[3] = (float)supported;
  const float *g = pid_gains(T);
  for (int i = 0; i < 9; i++)
    p[4 + i] = g[i];
  p[13] = p[14] = p[15] = 0;
}
#endif
