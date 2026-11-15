/* The flight core's text commands: see fc_text.h. No C library (the simulator's boards are freestanding). */
#include "fc_text.h"
#include "wasm_math.h"

static int eq(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return *a == *b; }
static const char *after(const char *s, const char *p) { while (*p) if (*s++ != *p++) return 0; return s; }
static const char *sp(const char *s) { while (*s == ' ') s++; return s; }
/* a number ([-+]digits[.digits]); 0 if there's none */
static const char *num(const char *s, float *v) {
  s = sp(s); int neg = *s == '-'; if (*s == '-' || *s == '+') s++;
  if (!((*s >= '0' && *s <= '9') || (*s == '.' && s[1] >= '0' && s[1] <= '9'))) return 0;
  double x = 0, f = 0.1; while (*s >= '0' && *s <= '9') x = 10 * x + (*s++ - '0');
  if (*s == '.') { s++; while (*s >= '0' && *s <= '9') { x += f * (*s++ - '0'); f *= 0.1; } }
  *v = (float)(neg ? -x : x); return s;
}
static void level_said(const fc_state *F, char *r, int cap) {
  if (!F->level_kind) { fc_fmt(r, cap, "level: none, the IMU's mount as the airframe says"); return; }
  fc_fmt(r, cap, "level: trim roll %.1f°, pitch %.1f°, yaw %.0f° (%s)", (double)F->level_deg[0], (double)F->level_deg[1], (double)F->level_deg[2], F->level_kind == 1 ? "measured" : "set");
}

int fc_text(fc_state *F, const char *line, const float *acc, int still, char *r, int cap) {
  const char *a;
  line = sp(line);
  if (eq(line, "status")) {
    const float *R = F->R;
    float roll = atan2f(R[7], R[8]) * 57.2958f, pitch = atan2f(-R[6], sqrtf(R[7] * R[7] + R[8] * R[8])) * 57.2958f;
    fc_fmt(r, cap, "%s; attitude roll %.1f pitch %.1f; ", fc_state_name(F->state), (double)roll, (double)pitch);
    int at = 0; while (r[at]) at++;
    level_said(F, r + at, cap - at);
    fc_fmt_add(r, cap, "; safety %s; %s", F->safety_off ? "OFF" : "on", F->why);
    return 1;
  }
  if (eq(line, "safety")) { fc_fmt(r, cap, F->safety_off ? "safety off: no level check when arming, no crash cut-off" : "safety on"); return 1; }
  if (eq(line, "safety on")) { fc_safety(F, 0); fc_fmt(r, cap, "%s", F->why); return 1; }
  if (eq(line, "safety off")) { fc_safety(F, 1); fc_fmt(r, cap, "%s", F->why); return 1; }
  if (!(a = after(line, "level")) || (*a && *a != ' ')) return 0;
  a = sp(a);
  if (F->state != FC_DISARMED && *a) { fc_fmt(r, cap, "level: disarm first"); return 1; }
  if (!*a) {
    if (F->state != FC_DISARMED) { level_said(F, r, cap); return 1; }   /* (armed: what it is, nothing changes) */
    if (!acc) { fc_fmt(r, cap, "level: no IMU readings"); return 1; }
    if (!still) { fc_fmt(r, cap, "level: it's moving; keep it still and try again"); return 1; }
    if (fc_level_imu(F, acc)) { fc_fmt(r, cap, "%s", F->why); return 1; }
    level_said(F, r, cap); return 1;
  }
  if (eq(a, "none")) { fc_level_none(F); level_said(F, r, cap); return 1; }
  float v[3] = { 0, 0, F->level_deg[2] };
  if ((a = after(a, "yaw"))) {
    const char *e = num(a, &v[2]);
    if (!e || *sp(e)) { fc_fmt(r, cap, "level yaw DEGREES"); return 1; }
    if (fc_level_yaw(F, v[2])) { fc_fmt(r, cap, "%s", F->why); return 1; }
    level_said(F, r, cap); return 1;
  }
  a = sp(after(line, "level")); int n = 0;
  for (const char *e; n < 3 && (e = num(a, &v[n])); n++) { a = sp(e); if (*a == ',') a = sp(a + 1); }
  if (n < 2 || *a) { fc_fmt(r, cap, "level, level none, level ROLL,PITCH[,YAW] or level yaw DEGREES"); return 1; }
  if (fc_level_deg(F, v[0], v[1], v[2])) { fc_fmt(r, cap, "%s", F->why); return 1; }
  level_said(F, r, cap); return 1;
}
