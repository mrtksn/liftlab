/* The command module's text commands: see ground_text.h. */
#include "ground_text.h"
#include "rc_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *const gnd_button_names[GB_N] = { "right", "left", "fwd", "back", "up", "down", "yawr", "yawl", "arm", "fly", "hold", "home", "gentle", "normal", "sport", "cal" };
#define STICK_BUTTONS 0xFFu                          /* GB_RIGHT … GB_YAWL */
#define LAPSE 1.0                                    /* [s] */
int gnd_button(const char *s) { for (int i = 0; i < GB_N; i++) if (!strcmp(s, gnd_button_names[i])) return i; return -1; }

void gnd_text_inputs(const gnd_text_in *I, double t, gnd_input *in) {
  in->held |= I->held;
  for (int b = 0; b < GB_N; b++) if (t < I->until[b]) in->held |= GB(b);
  for (int a = 0; a < GND_AXES; a++) if (((I->has >> a) & 1) && t < I->axis_until[a]) { in->axis[a] = I->axis[a]; in->has_axis |= 1u << a; }
}

void gnd_status(const gnd_state *G, double t, char *out, int n) {
  const gnd_view *V = &G->V; int why, lvl = gnd_alert(G, &why); char lq[24] = "?";
  if (V->t_link >= 0 && t - V->t_link < 2) snprintf(lq, sizeof lq, "%.0f%%", (double)V->link.up_lq);
  snprintf(out, (size_t)n, "%s | uplink %s | %s | %.1f V %d%% | height %.1f m | %s%s",
           V->t_any >= 0 && t - V->t_any < 1.5 ? "telemetry" : "NO TELEMETRY", lq, V->mode[0] ? V->mode : "?", (double)V->volts, V->pct, (double)V->baro_alt,
           lvl == 2 ? "ALARM: " : lvl ? "warning: " : "", lvl ? gnd_why_text[why] : "all fine");
}

int gnd_text(gnd_state *G, gnd_text_in *I, char *s, double t, char *reply, int rn) {
  char *w[10], *save = 0; int n = 0;
  for (char *p = strtok_r(s, " \t\r\n", &save); p && n < 10; p = strtok_r(0, " \t\r\n", &save)) w[n++] = p;
  reply[0] = 0; if (!n) return 1;
  if ((!strcmp(w[0], "press") || !strcmp(w[0], "release") || !strcmp(w[0], "tap")) && n >= 2) {
    int b = gnd_button(w[1]); if (b < 0) { snprintf(reply, (size_t)rn, "no button %s", w[1]); return 1; }
    if (w[0][0] == 'r') { I->held &= ~GB(b); I->until[b] = 0; }
    else if (w[0][0] == 't') I->until[b] = t + 0.25;
    else if (GB(b) & STICK_BUTTONS) I->until[b] = t + LAPSE;  /* (lapses unless sent again) */
    else I->held |= GB(b);
  } else if (!strcmp(w[0], "stick") && n >= 3) {
    static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" }; int a = -1; for (int i = 0; i < 4; i++) if (!strcmp(w[1], ax[i])) a = i;
    if (a < 0) { snprintf(reply, (size_t)rn, "stick roll|pitch|throttle|yaw V"); return 1; }
    if (!strcmp(w[2], "off")) I->has &= ~(1u << a);
    else { I->axis[a] = (float)atof(w[2]); I->has |= 1u << a; I->axis_until[a] = t + LAPSE; }
  } else if (!strcmp(w[0], "goto") && n >= 4) {
    float h = n >= 5 ? (float)(atof(w[4]) * 3.14159265358979 / 180) : 0;
    if (gnd_goto(G, (float)atof(w[1]), (float)atof(w[2]), (float)atof(w[3]), h)) snprintf(reply, (size_t)rn, "too many commands waiting");
    else snprintf(reply, (size_t)rn, "going to %.1f %.1f %.1f", atof(w[1]), atof(w[2]), atof(w[3]));
  } else if (!strcmp(w[0], "calibrate")) { float c = 1; gnd_command(G, RC_CMD_LEARN, &c, 1); snprintf(reply, (size_t)rn, "asked the learning to calibrate"); }
  else if (!strcmp(w[0], "cmd") && n >= 2) {
    float v[6]; int m = 0; for (int i = 2; i < n && m < 6; i++) v[m++] = (float)atof(w[i]);
    if (gnd_command(G, atoi(w[1]), v, m)) snprintf(reply, (size_t)rn, "too many commands waiting");
  } else if (!strcmp(w[0], "status")) gnd_status(G, t, reply, rn);
  else if (!strcmp(w[0], "messages")) {
    int k = 0; uint32_t from = G->V.nmsg > GND_MSGS ? G->V.nmsg - GND_MSGS : 0;
    for (uint32_t i = from; i < G->V.nmsg && k + 70 < rn; i++) k += snprintf(reply + k, (size_t)(rn - k), "%s%s", k ? "\n" : "", G->V.msg[i % GND_MSGS].s);
    if (!k) snprintf(reply, (size_t)rn, "no messages yet");
  } else if (!strcmp(w[0], "quit")) return 2;
  else return 0;
  return 1;
}
