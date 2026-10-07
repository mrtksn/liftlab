/* The command module's text commands: see ground_text.h. */
#include "ground_text.h"
#include "rc_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *const gnd_button_names[GB_N] = { "right", "left", "fwd", "back", "up", "down", "yawr", "yawl", "arm", "fly", "hold", "home", "gentle", "normal", "sport", "cal" };
#define STICK_BUTTONS 0xFFu                          /* GB_RIGHT … GB_YAWL */
#define LAPSE 1.0                                    /* [s] */
#define TAP 0.25                                     /* [s] a tap, and a press of hold, home or cal (momentary) */
int gnd_button(const char *s) { for (int i = 0; i < GB_N; i++) if (!strcmp(s, gnd_button_names[i])) return i; return -1; }
/* a number, all of the word, and a finite one ("home", "nan", "1e999" aren't) */
static int num(const char *w, double *x) { char *e; *x = strtod(w, &e); return e != w && !*e && isfinite(*x); }

void gnd_text_inputs(const gnd_text_in *I, double t, gnd_input *in) {
  in->sw |= I->held;                                 /* (switch states as set; latching buttons are set in the core: gnd_latch) */
  for (int b = 0; b < GB_N; b++) if (t < I->until[b]) in->sw |= GB(b);
  for (int a = 0; a < GND_AXES; a++) if (((I->has >> a) & 1) && t < I->axis_until[a]) { in->axis[a] = I->axis[a]; in->has_axis |= 1u << a; }
}

void gnd_status(const gnd_state *G, double t, char *out, int n) {
  const gnd_view *V = &G->V; int why, lvl = gnd_alert(G, &why); char lq[24] = "?";
  if (V->t_link >= 0 && t - V->t_link < 2) snprintf(lq, sizeof lq, "%.0f%%", (double)V->link.up_lq);
  int k = snprintf(out, (size_t)n, "%s | uplink %s | %s | %.1f V %d%% | height %.1f m | %s%s",
           V->t_any >= 0 && t - V->t_any < 1.5 ? "telemetry" : "NO TELEMETRY", lq, V->mode[0] ? V->mode : "?", (double)V->volts, V->pct, (double)V->baro_alt,
           lvl == 2 ? "ALARM: " : lvl ? "warning: " : "", lvl ? gnd_why_text[why] : "all fine");
  if (G->qn && k > 0 && k < n) snprintf(out + k, (size_t)(n - k), " | %d command%s waiting%s", G->qn, G->qn > 1 ? "s" : "", gnd_link_up(G, t) ? "" : " for the link");
}

/* what a switch is now, after a text command changed it */
static void switch_said(const gnd_state *G, const gnd_text_in *I, int b, char *reply, int rn) {
  uint32_t m = GB(b); int latch = (G->C.latch & m) != 0;
  int want = latch ? (G->latched & m) != 0 : (I->held & m) != 0, hw = !latch && (G->held_was & m);
  if (want && (m & (GB(GB_ARM) | GB(GB_FLY))) && !(G->seen_off & m)) snprintf(reply, (size_t)rn, "%s on, but held back until it has been off (switch warning)", gnd_button_names[b]);
  else if (!want && hw) snprintf(reply, (size_t)rn, "%s off here, but a switch holds it on", gnd_button_names[b]);
  else snprintf(reply, (size_t)rn, "%s %s", gnd_button_names[b], want || hw ? "on" : "off");
}

int gnd_text(gnd_state *G, gnd_text_in *I, char *s, double t, char *reply, int rn) {
  char *w[10], *save = 0; int n = 0;
  for (char *p = strtok_r(s, " \t\r\n", &save); p && n < 10; p = strtok_r(0, " \t\r\n", &save)) w[n++] = p;
  reply[0] = 0; if (!n) return 1;
  if ((!strcmp(w[0], "press") || !strcmp(w[0], "release") || !strcmp(w[0], "tap")) && n >= 2) {
    int b = gnd_button(w[1]); if (b < 0) { snprintf(reply, (size_t)rn, "no button %s", w[1]); return 1; }
    if (G->C.latch & GB(b)) {                        /* a latching button: set as said (tap: a push, toggles it) */
      I->held &= ~GB(b); I->until[b] = 0;
      gnd_latch(G, b, w[0][0] == 'p' ? 1 : w[0][0] == 'r' ? 0 : !(G->latched & GB(b)));
      switch_said(G, I, b, reply, rn);
    }
    else if (w[0][0] == 'r') { I->held &= ~GB(b); I->until[b] = 0; if (b == GB_ARM || b == GB_FLY) switch_said(G, I, b, reply, rn); }
    else if (w[0][0] == 't') I->until[b] = t + TAP;
    else if (GB(b) & STICK_BUTTONS) I->until[b] = t + LAPSE;  /* (lapses unless sent again) */
    else if (b == GB_GENTLE || b == GB_NORMAL || b == GB_SPORT) I->until[b] = t + 0.05;   /* (a speed level: a moment is enough) */
    else if (b == GB_ARM || b == GB_FLY) { I->held |= GB(b); switch_said(G, I, b, reply, rn); }
    else I->until[b] = t + TAP;                      /* (hold, home, cal: momentary, so each press is one) */
  } else if (!strcmp(w[0], "stick") && n >= 3) {
    static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" }; int a = -1; for (int i = 0; i < 4; i++) if (!strcmp(w[1], ax[i])) a = i;
    double v;
    if (a < 0) { snprintf(reply, (size_t)rn, "stick roll|pitch|throttle|yaw V"); return 1; }
    if (!strcmp(w[2], "off")) I->has &= ~(1u << a);
    else if (!num(w[2], &v) || v < -1 || v > 1) snprintf(reply, (size_t)rn, "stick %s: a number from -1 to 1, or off (not %.20s): unchanged", ax[a], w[2]);
    else { I->axis[a] = (float)v; I->has |= 1u << a; I->axis_until[a] = t + LAPSE; }
  } else if (!strcmp(w[0], "goto") && n >= 4) {
    double x, y, z, h = 0; int r;
    if (!num(w[1], &x) || !num(w[2], &y) || !num(w[3], &z) || (n >= 5 && !num(w[4], &h))) snprintf(reply, (size_t)rn, "goto X Y Z [HEADING]: numbers, metres from home (and degrees): not sent");
    else if (fabs(x) > GND_GOTO_MAX || fabs(y) > GND_GOTO_MAX || fabs(z) > GND_GOTO_MAX) snprintf(reply, (size_t)rn, "goto: X, Y and Z within %.0f m of home (what the command carries): not sent", (double)GND_GOTO_MAX);
    else if ((r = gnd_goto(G, (float)x, (float)y, (float)z, (float)(fmod(h, 360) * 3.14159265358979 / 180)))) snprintf(reply, (size_t)rn, r == -1 ? "too many commands waiting" : "goto: can't send that");
    else snprintf(reply, (size_t)rn, "going to %.1f %.1f %.1f%s", x, y, z, gnd_link_up(G, t) ? "" : " (once the link is back)");
  } else if (!strcmp(w[0], "latch") && n >= 3) {           /* the cargo task's latches (cargo_core.h): latch 1 open, latch all close */
    double x = 0; int all = !strcmp(w[1], "all"), a = !strcmp(w[2], "open") ? 0 : !strcmp(w[2], "close") ? 1 : !strcmp(w[2], "toggle") ? 2 : -1;
    if (a < 0 || (!all && (!num(w[1], &x) || x < 1 || x > 8 || x != floor(x)))) snprintf(reply, (size_t)rn, "latch N|all open|close|toggle: N from 1 to 8: not sent");
    else { float v[2] = { all ? -1.0f : (float)(x - 1), (float)a }; int r = gnd_command(G, RC_CMD_LATCH, v, 2);
      snprintf(reply, (size_t)rn, r ? "too many commands waiting" : "latch %s: %s%s", w[1], w[2], r || gnd_link_up(G, t) ? "" : " (once the link is back)"); }
  } else if (!strcmp(w[0], "pickup") && n >= 4) {          /* fly the hook onto a thing at X Y Z (its top, from home) and close latch N */
    double x, y, z, l = 1; int r;
    if (!num(w[1], &x) || !num(w[2], &y) || !num(w[3], &z) || (n >= 5 && (!num(w[4], &l) || l < 1 || l > 8 || l != floor(l)))) snprintf(reply, (size_t)rn, "pickup X Y Z [LATCH]: the thing's top, metres from home, and the latch 1 to 8: not sent");
    else { float top[3] = { (float)x, (float)y, (float)z };
      r = gnd_pickup(G, top, (int)l - 1, t);
      if (r == -3) snprintf(reply, (size_t)rn, "pickup: no attitude from the drone to work out where its hook goes: not sent");
      else if (r) snprintf(reply, (size_t)rn, r == -1 ? "too many commands waiting" : "pickup: out of range: not sent");
      else snprintf(reply, (size_t)rn, "pickup at %.2f %.2f %.2f with latch %d%s", x, y, z, (int)l, gnd_link_up(G, t) ? "" : " (once the link is back)"); }
  } else if (!strcmp(w[0], "fleet") && n >= 2 && (!strcmp(w[1], "on") || !strcmp(w[1], "off"))) {   /* let the drone's fleet program fly it (fc/fleet.h), or not */
    float v = w[1][1] == 'n' ? 1.0f : 0.0f; int r = gnd_command(G, RC_CMD_FLEET, &v, 1);
    snprintf(reply, (size_t)rn, r ? "too many commands waiting" : "fleet program %s%s", w[1], r || gnd_link_up(G, t) ? "" : " (once the link is back)");
  } else if (!strcmp(w[0], "calibrate")) { float c = 1; snprintf(reply, (size_t)rn, gnd_command(G, RC_CMD_LEARN, &c, 1) ? "too many commands waiting" : "asked the learning to calibrate"); }
  else if (!strcmp(w[0], "cmd") && n >= 2) {
    char *e; long id = strtol(w[1], &e, 10); float v[6]; int m = 0, bad = e == w[1] || *e || id < 1 || id > 255 || n - 2 > 6;
    for (int i = 2; i < n && !bad; i++) { double x; if (num(w[i], &x)) v[m++] = (float)x; else bad = 1; }
    int r = bad ? -2 : gnd_command(G, (int)id, v, m);
    if (r == -2) snprintf(reply, (size_t)rn, "cmd ID V1 … V6: ID 1 to 255, values numbers within what the command carries: not sent");
    else snprintf(reply, (size_t)rn, r ? "too many commands waiting" : "command %ld queued", id);
  } else if (!strcmp(w[0], "status")) gnd_status(G, t, reply, rn);
  else if (!strcmp(w[0], "messages")) {
    int k = 0; uint32_t from = G->V.nmsg > GND_MSGS ? G->V.nmsg - GND_MSGS : 0;
    for (uint32_t i = from; i < G->V.nmsg && k + 70 < rn; i++) k += snprintf(reply + k, (size_t)(rn - k), "%s%s", k ? "\n" : "", G->V.msg[i % GND_MSGS].s);
    if (!k) snprintf(reply, (size_t)rn, "no messages yet");
  } else if (!strcmp(w[0], "quit")) return 2;
  else return 0;
  return 1;
}
