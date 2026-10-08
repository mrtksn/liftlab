/* Production hardware tuner on the independent motor/rigid-body test plant.
 * UART-framed core transactions and commands arrive 6 ms later; nav is 100 Hz
 * and LTEL 200 Hz. */
#define main nav_original_main
#include "../runner/fc/test_nav.c"
#undef main
#include "../runner/fc/autotune_core.h"
#include "../runner/pi/autotune_worker.h"
#include <assert.h>
static autotune_state AT;
static autotune_worker AW;
static learn_state LS;
static super_state SS;
static int AT_live, wire_rejected;
static double last_nav_at;
static struct {
  double at;
  int type, n;
  float p[32];
} wq[64];
static int wq_n;
static struct {
  double at;
  int n;
  float p[AT_LTEL_MAX];
} atq[8];
static int atq_n;
static int skip_tuning_telemetry;
static void sink(void *ctx, uint8_t type, const float *p, int n) {
  (void)ctx;
  assert(n <= 32 && wq_n < 64);
  uint8_t packet[256], buf[256];
  rn_link R;
  rn_link_init(&R, buf, sizeof buf);
  int bytes = rn_link_frame(packet, sizeof packet, type, (const uint8_t *)p,
                            (uint32_t)n * 4),
      got = 0;
  for (int i = 0; i < bytes; i++)
    got = rn_link_feed(&R, packet[i]);
  assert(got == type && R.len == (uint32_t)n * 4);
  wq[wq_n].at = B.t + .006;
  wq[wq_n].type = type;
  wq[wq_n].n = n;
  memcpy(wq[wq_n++].p, R.buf, (size_t)n * 4);
}
static void hardware_fly(double seconds) {
  for (int n = 0; n < (int)(seconds * 1000 + 0.5); n++) {
    double t = B.t;
    long ms = lround(t * 1000);
    while (wq_n && wq[0].at <= t) {
      float *p = wq[0].p;
      int kind = wq[0].type;
      if (kind == RN_LINK_EXC)
        fc_exc(&F, p, wq[0].n);
      else if (kind == RN_LINK_TUNE) {
        if (fc_tune(&F, p, wq[0].n))
          wire_rejected++;
      }
      memmove(wq, wq + 1, sizeof wq[0] * --wq_n);
    }
    /* commands arriving at the flight core */
    while (cq_n && cq[0].at <= t) {
      fc_command(&F, &cq[0].c);
      memmove(cq, cq + 1, sizeof cq[0] * --cq_n);
    }
    fc_imu m;
    for (int k = 0; k < 3; k++) {
      m.gyro[k] = (float)(B.w[k] + 0.002 * gauss());
      m.acc[k] = (float)(acc_b[k] + 0.03 * gauss());
    }
    if (B.t == 0) {
      double R[9];
      qm(R, B.q);
      for (int i = 0; i < 3; i++)
        m.acc[i] = (float)(R[6 + i] * 9.81);
    }
    m.have_gyro = ms != drop_gyro_at;
    m.have_mag = 0;
    m.have_baro = ms % 40 == 0;
    m.baro_alt = (float)(B.p[2] + 50 + 0.001 * gauss());
    rn_host_tick(&HF, 0.001f);
    fc_step(&F, &m, 0.001f, 0, &O);
    plant_step(&B, &F.A, &O, 0.001);
    /* every 10 ms the flight core sends its telemetry to the Pi */
    if (ms % 10 == 0 && link_on) {
      nav_in T;
      memset(&T, 0, sizeof T);
      memcpy(T.q, F.q, sizeof T.q);
      memcpy(T.w, F.w, sizeof T.w);
      for (int k = 0; k < 3; k++)
        T.acc[k] = m.acc[k];
      T.have_att = F.att_ok;
      T.have_baro = F.have_alt;
      T.baro_alt = F.alt_e;
      T.baro_age = 0;
      if (tq_n < QN) {
        tq[tq_n].at = t + DELAY;
        tq[tq_n].m = T;
        tq_n++;
      }
    }
    if (AT_live) {
      autotune_guard(&AT, t, 1, &SP, &NO, last_nav_at, 0);
      if (ms % 5 == 0 && !skip_tuning_telemetry) {
        float p[AT_LTEL_MAX];
        int n = fc_ltel(&F, p);
        n += fc_tuning_sample(&F, p + n);
        n += fc_tune_status(&F, p + n);
        uint8_t frame[AT_LTEL_MAX * 4 + 32], buf[AT_LTEL_MAX * 4];
        rn_link R;
        rn_link_init(&R, buf, sizeof buf);
        int bytes = rn_link_frame(frame, sizeof frame, RN_LINK_TUNE_LTEL,
                                  (const uint8_t *)p, (uint32_t)n * 4),
            got = 0;
        for (int j = 0; j < bytes; j++)
          got = rn_link_feed(&R, frame[j]);
        assert(got == RN_LINK_TUNE_LTEL && R.len == (uint32_t)n * 4);
        assert(atq_n < 8);
        atq[atq_n].at = t + .006;
        atq[atq_n].n = n;
        memcpy(atq[atq_n++].p, R.buf, (size_t)n * 4);
      }
      while (atq_n && atq[0].at <= t) {
        autotune_sample(&AT, atq[0].p, atq[0].n, t);
        memmove(atq, atq + 1, sizeof atq[0] * --atq_n);
      }
    }
    if (AT_live)
      autotune_work_poll(&AW, &AT);
    /* the GPS, wired to the Pi */
    if (ms % 50 == 0) {
      for (int k = 0; k < 3; k++)
        gps_w[k] += (-gps_w[k] * 0.2 + 0.3 * gauss()) * 0.2;
      if (gq_n < 8) {
        gq[gq_n].at = t + 0.02;
        for (int k = 0; k < 3; k++) {
          gq[gq_n].p[k] = B.p[k];
          gq[gq_n].v[k] = B.v[k];
        }
        gq_n++;
      }
    }
    /* the Pi: navigation at 100 Hz on the newest telemetry */
    while (tq_n && tq[0].at <= t) {
      latest = tq[0].m;
      have_latest = 1;
      memmove(tq, tq + 1, sizeof tq[0] * --tq_n);
    }
    static double fix_t = -1, fix_p[3], fix_v[3], fix_meas_t;
    while (gq_n && gq[0].at <= t) {
      fix_t = t;
      fix_meas_t = gq[0].at - 0.02;
      memcpy(fix_p, gq[0].p, sizeof fix_p);
      memcpy(fix_v, gq[0].v, sizeof fix_v);
      memmove(gq, gq + 1, sizeof gq[0] * --gq_n);
    }
    if (ms % 10 == 5 && have_latest) {
      nav_in in = latest;
      nav_out o;
      in.baro_age = (float)DELAY;
      if (use_fix && fix_t >= 0 && t - fix_t < 0.3) {
        in.have_fix = 1;
        in.fix_age = (float)(t - fix_meas_t);
        for (int k = 0; k < 3; k++) {
          in.fix_p[k] = (float)fix_p[k];
          in.fix_v[k] = (float)fix_v[k];
        }
      }
      if (use_flow) { /* a downward camera on the Pi: flow and range now (and
                         the gyro it removes is the telemetry's) */
        double R[9];
        qm(R, B.q);
        double range = B.p[2] + 0.05;
        range /= R[8] > 0.3 ? R[8] : 0.3;
        double vs[3];
        for (int i = 0; i < 3; i++)
          vs[i] = R[i] * B.v[0] + R[3 + i] * B.v[1] + R[6 + i] * B.v[2];
        in.have_flow = 1;
        in.range = (float)(range + 0.01 * gauss());
        in.flow_q = 1;
        in.flow_age = 0.02f;
        in.flow[0] = (float)(B.w[1] - vs[0] / range + 0.05 * gauss());
        in.flow[1] = (float)(-B.w[0] - vs[1] / range + 0.05 * gauss());
      }
      if (drop_att > 0) {
        drop_att--;
        in.have_att = 0;
      }
      int e = nav_step(&N, &in, &SP, 0.01f, &o);
      NO = o;
      last_nav_at = t;
      fc_cmd c;
      memset(&c, 0, sizeof c);
      c.arm = 1;
      c.test_motor = -1;
      c.guided = 1;
      memcpy(c.acc, o.acc, sizeof c.acc);
      c.heading = o.heading;
      c.throttle = o.fly ? 1 : 0;
      if (!o.fly)
        idle_cmds++;
      if (link_on && cq_n < QN && !(e > 0 && o.fly)) {
        cq[cq_n].at = t + DELAY;
        cq[cq_n].c = c;
        cq_n++;
      } /* (as dfb_pi: nothing to step on in the air, no command) */
    }
  }
}

static void wait_hover(void) {
  for (int i = 0; i < 200; i++) {
    hardware_fly(.1);
    if (AT.settled)
      return;
  }
  assert(AT.settled);
}
static void command(const char *s) {
  char reply[512];
  assert(autotune_command(&AT, s, reply, sizeof reply));
  printf("%s -> %s\n", s, reply);
}
static void trial(void) {
  wait_hover();
  memcpy(AT.before, AT.current, sizeof AT.before);
  memcpy(AT.candidate, AT.current, sizeof AT.current);
  AT.candidate[0] *= .98f;
  AT.candidate[9] *= .98f;
  AT.loop = 1;
  AT.phase = AT_REVIEW;
  command("autotune apply");
  hardware_fly(.05);
  assert(AT.phase == AT_VERIFY && F.tuning.pending && N.tuning.pending);
}
static void fault_tests(void) {
  float att[9], pos[9];
  memcpy(att, F.tuning.accepted, sizeof att);
  memcpy(pos, N.tuning.accepted, sizeof pos);
  trial();
  command("autotune stop");
  hardware_fly(.02);
  assert(AT.phase == AT_STOPPED && !F.tuning.pending && !N.tuning.pending);
  trial();
  skip_tuning_telemetry = 1;
  hardware_fly(.08);
  skip_tuning_telemetry = 0;
  assert(AT.phase == AT_STOPPED && !F.tuning.pending && !N.tuning.pending);
  hardware_fly(.1);
  trial();
  drop_gyro_at = lround(B.t * 1000);
  hardware_fly(.02);
  drop_gyro_at = -1;
  assert(AT.phase == AT_STOPPED && !F.tuning.pending && !N.tuning.pending);
  trial();
  AT_live = 0;
  atq_n = 0;
  hardware_fly(.3);
  assert(!F.tuning.pending && !N.tuning.pending);
  AT_live = 1;
  hardware_fly(.03);
  assert(AT.phase == AT_STOPPED);
  assert(!memcmp(att, F.tuning.accepted, sizeof att) &&
         !memcmp(pos, N.tuning.accepted, sizeof pos));
  wait_hover();
  command("autotune attitude");
  LS.flyB[0][0] += .001f;
  hardware_fly(.02);
  assert(AT.phase == AT_STOPPED);
  LS.flyB[0][0] -= .001f;
  wait_hover();
  command("autotune attitude");
  N.sup_mode = 1;
  hardware_fly(.02);
  assert(AT.phase == AT_STOPPED);
  N.sup_mode = 0;
  wait_hover();
  command("autotune attitude");
  float p[AT_LTEL_MAX];
  int n = fc_ltel(&F, p);
  n += fc_tuning_sample(&F, p + n);
  n += fc_tune_status(&F, p + n);
  p[n - 3] += 1;
  autotune_sample(&AT, p, n, B.t);
  assert(AT.phase == AT_STOPPED && !AT.have_status);
  hardware_fly(.02);
  wait_hover();
  LS.have_fit = 0;
  command("autotune attitude");
  assert(!autotune_busy(&AT));
  LS.have_fit = 1;
  AT.phase = AT_ANALYZE;
  autotune_analysis obsolete;
  autotune_analysis_input(&AT, &obsolete);
  obsolete.ok = 1;
  AT.measurement++;
  autotune_analysis_finish(&AT, &obsolete);
  assert(AT.phase == AT_ANALYZE);
  autotune_stop(&AT, "analysis cancelled");
  autotune_analysis_finish(&AT, &obsolete);
  assert(AT.phase == AT_STOPPED);
  puts("Stop, telemetry loss, IMU loss, companion loss, "
       "model/supervisor/airframe changes and missing calibration guards "
       "passed");
}
int main(void) {
  uint32_t len;
  uint8_t *quad = read_file("runner/fc/testdata/quadx.dfa", &len);
  start(quad, len, 0);
  arm();
  SP.fly = 1;
  SP.target[2] = 1.5f;
  hardware_fly(12);
  /* Accepted calibration fixture: model identification has separate
   * native/end-to-end tests. */
  LS.FA.A = F.A;
  LS.FA.airframe_crc = F.airframe_crc;
  LS.keep = 0;
  LS.have_fit = LS.accepted = 1;
  LS.state = FC_ARMED;
  NO.ready = 1;
  N.last = NO;
  autotune_init(&AT, &N, &LS, &SS, sink, 0);
  AT_live = 1;
  last_nav_at = B.t;
  hardware_fly(.2);
  wait_hover();
  command("autotune attitude");
  assert(AT.phase == AT_MEASURE);
  for (int k = 0; k < 250 && autotune_busy(&AT); k++)
    hardware_fly(1);
  printf("measurement phase %d: %s index %d pos %.3f %.3f %.3f target %.3f "
         "%.3f %.3f v %.3f %.3f %.3f rates %.3f %.3f %.3f\n",
         AT.phase, AT.message, AT.index, N.last.p[0], N.last.p[1], N.last.p[2],
         SP.target[0], SP.target[1], SP.target[2], N.last.v[0], N.last.v[1],
         N.last.v[2], F.w[0], F.w[1], F.w[2]);
  assert(AT.phase == AT_REVIEW);
  command("autotune apply");
  for (int k = 0; k < 250 && autotune_busy(&AT); k++)
    hardware_fly(1);
  printf("verification phase %d: %s\n", AT.phase, AT.message);
  assert(AT.phase == AT_DONE && AT.att_verified && F.tuning.enabled);
  command("autotune position");
  for (int k = 0; k < 250 && autotune_busy(&AT); k++)
    hardware_fly(1);
  for (int i = 0; i < 3; i++)
    printf("pos bin %.2f q %.3f T %.3f %.3f G %.3f %.3f fit %.3f %.3f %.3f err "
           "%.3f\n",
           AT.bins[i].hz, AT.bins[i].quality, AT.bins[i].T.r, AT.bins[i].T.i,
           AT.bins[i].plant.r, AT.bins[i].plant.i, AT.models[0].gain,
           AT.models[0].lag, AT.models[0].delay, AT.models[0].error);
  printf("position phase %d: %s\n", AT.phase, AT.message);
  assert(AT.phase == AT_REVIEW);
  command("autotune apply");
  for (int k = 0; k < 250 && autotune_busy(&AT); k++)
    hardware_fly(1);
  printf("position verification %d: %s\n", AT.phase, AT.message);
  assert(AT.phase == AT_DONE && N.tuning.enabled);
  assert(autotune_save_ready(&AT));
  fault_tests();
  float accepted[9];
  memcpy(accepted, F.tuning.accepted, sizeof accepted);
  wait_hover();
  command("autotune attitude");
  hardware_fly(.1);
  SP.target[0] += .1f;
  hardware_fly(.1);
  assert(AT.phase == AT_STOPPED && !F.tuning.pending);
  SP.target[0] -= .1f;
  hardware_fly(1);
  wait_hover();
  command("autotune attitude");
  AT.last_packet = B.t - .05;
  autotune_guard(&AT, B.t, 1, &SP, &NO, B.t, 0);
  assert(AT.phase == AT_STOPPED);
  /* Isolate the flight-controller lease from continued normal navigation
   * commands. */
  float tx[PID_FRAME] = {PID_VERSION, 1, 999};
  memcpy(tx + 3, accepted, sizeof accepted);
  tx[3] *= .9f;
  assert(!fc_tune(&F, tx, PID_FRAME));
  AT_live = 0;
  hardware_fly(.3);
  assert(!F.tuning.pending &&
         !memcmp(accepted, F.tuning.accepted, sizeof accepted));
  assert(fc_tune(
      &F, tx,
      PID_FRAME)); /* a stale refresh cannot restart the expired trial */
  tx[2] = 1000;
  tx[3] = NAN;
  assert(fc_tune(&F, tx, PID_FRAME));
  tx[3] = accepted[0] * .9f;
  assert(!fc_tune(&F, tx, PID_FRAME));
  F.sup_mode = 1;
  hardware_fly(.01);
  assert(!F.tuning.pending);
  F.sup_mode = 0;
  /* Late lease refreshes after cancellation are intentionally rejected. */
  assert(!fails);
  puts("Hardware attitude/position measurement + verification, framed "
       "readback, pilot takeover, telemetry loss, FC lease, stale refresh, NaN "
       "and supervisor rollback passed");
  return 0;
}
