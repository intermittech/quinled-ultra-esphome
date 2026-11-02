// Hub <-> board contract, end to end on a PC (review round 6): the component's Scheduler (ultra_core.h) builds the
// frames, and every board runs them through the board firmware's own receive path (frame_rx.c / proto.c: its slot from
// the frame start, the header trailer from the frame end, the protocol lock) into its light engine (ultra.c), with its
// own HSI error and tick phase, as main.c's handle_ultra does. Needs the board firmware (ULTRA_FIRMWARE).
// Build: python tests/run_ultra_contract_test.py
//  - every command type on a 50-board pin with 20 % frame loss, then a signal loss off / fade off on every board;
//  - one group start over two pins (a SET fade on 260 lights, calls over 8 ms): one start;
//  - LIGHT_DEF gamma 0 = linear (ESPHome's gamma_correct: 0; board firmware > 0.6.6).

#include "../components/ultra_line/ultra_core.h"

extern "C" {
#include "frame_rx.h"
#include "ultra.h"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace esphome::ultra_line;

static int failures = 0;
#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      failures++; \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
    } \
  } while (0)

static const uint16_t Z5[5] = {0, 0, 0, 0, 0};
static const uint16_t ONE5[5] = {65535, 65535, 65535, 65535, 65535};
static const uint8_t MONO1[5] = {ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0};
static const uint8_t RGBWW[5] = {ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_COLD_WHITE,
                                 ULTRA_ROLE_WARM_WHITE};

/// One board: its receive path and engine, its oscillator (HSI error, tick phase).
struct Board {
  frx_t rx{};
  ultra_t u{};
  double drift{0}, phase{0};
  uint32_t frames{0}, slots{0};
  uint32_t tick(double real_us) const {
    return static_cast<uint32_t>(static_cast<uint64_t>((real_us * (1.0 + drift) + phase) / 50.0));   // (wraps as the board's)
  }
};

/// A pin: the hub's scheduler and its boards, all of them sticky Ultra (they ran Ultra before this power-up).
struct Pin {
  Scheduler s;
  std::vector<Board> boards;
  std::vector<uint8_t> f;
  std::mt19937 rng;
  double loss{0};        // share of frames a board loses (its slot and header)
  double now_us{0};      // real time = the hub's clock (us)
  uint32_t frame_id{0};
  void init(uint16_t n, uint32_t seed, double drift_max = 0.02) {
    this->s.init(n, seed, 3);
    this->boards.assign(n, Board{});
    this->rng.seed(seed);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    for (uint16_t b = 0; b < n; b++) {
      Board &bd = this->boards[b];
      frx_init(&bd.rx, PMODE_AUTO, PROTO_ULTRA);
      ultra_init(&bd.u, 1000u + b + seed);
      bd.drift = uni(this->rng) * drift_max;
      bd.phase = (uni(this->rng) + 1.0) * 25.0;
    }
    this->f.assign(this->s.frame_bytes(), 0);
  }
  /// The frame in f ended (its last bit) at end_us: every board takes it as main.c does (its own slot from the
  /// start of what it sees, the header from the end).
  void deliver(double end_us) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const size_t n = this->boards.size();
    this->frame_id++;
    for (size_t k = 0; k < n; k++) {
      Board &bd = this->boards[k];
      if (this->loss > 0 && uni(this->rng) < this->loss)
        continue;
      const uint8_t *bits = this->f.data() + ULTRA_SLOT_BYTES * k;   // the boards before it took their slots
      const uint32_t total = static_cast<uint32_t>(ULTRA_SLOT_BITS * (n - k) + ULTRA_HDR_BITS);
      slot_t ws{};
      (void) frx_capture(&bd.rx, this->frame_id, proto_active(&bd.rx.ps), bits, ULTRA_SLOT_BITS, ULTRA_SLOT_BITS, 0,
                         &ws);
      frx_end_t e{};
      frx_frame_end(&bd.rx, this->frame_id, total, this->f.data() + this->f.size() - ULTRA_HDR_BYTES, &e);
      const uint32_t tk = bd.tick(end_us + uni(this->rng) * 50.0);
      if (e.ultra_sync) {
        ultra_on_header(&bd.u, tk, e.host_us);
        bd.frames++;
        if (e.ultra_slot) {
          ultra_apply_slot(&bd.u, bd.rx.slot, tk);
          bd.slots++;
        }
      }
    }
  }
  /// One frame now: the preamble's DARK frame or the scheduler's; then the line's reset and the hub's loop gap.
  void frame(bool dark = false) {
    const int64_t start = static_cast<int64_t>(this->now_us);
    const int64_t end = dark ? this->s.build_dark_frame(start, this->f.data()) : this->s.build_frame(start, this->f.data());
    this->deliver(static_cast<double>(end));
    this->now_us = static_cast<double>(end) + 300.0 + 500.0;
  }
  /// The hub's boot: the start-up preamble (DARK frames)
  void boot() {
    for (uint32_t i = 0; i < preamble_frames(static_cast<uint16_t>(this->boards.size())); i++)
      this->frame(true);
  }
  /// Frames back to back until the queues are empty
  void drain(int max_frames = 100000) {
    for (int i = 0; this->s.pending() && i < max_frames; i++)
      this->frame();
  }
  /// At rest: a frame every refresh_interval for ms
  void rest(double ms, double refresh_ms = 200.0) {
    const double until = this->now_us + ms * 1000.0;
    while (this->now_us < until) {
      const bool busy = this->s.pending();
      this->frame();
      if (!busy)
        this->now_us += refresh_ms * 1000.0 - 800.0;
    }
  }
};

// ----------------------------------------------------------------------------------------- every command, 50 boards

// Valid fixtures for every current board CONFIG and the ten scoped channel fields.
static uint16_t config_value(uint8_t id, uint16_t board) {
  const uint16_t v = static_cast<uint16_t>((id * 37 + board) % 97 + 1);
  switch (id) {
    case 1: return v % 15;
    case 2: return 1440 + v;
    case 3: case 5: case 10: return v % 3;
    case 6: return 5 + v % 96;
    case 7: return v % 5;
    case 8: case 22: return v % 4;
    case 9: return 5 + v;
    case 23: return v % 2;
    case 25: return v & 1 ? ULTRA_SKIP_HZ_SLOW : ULTRA_SKIP_HZ_DEFAULT;   // the pulse-skip rate: 2000 / 1000 only
    default: return v;
  }
}

/// Every board holds what the hub sent (test_every_command): CONFIG, IDENTIFY, the light definitions, options, scenes,
/// both stored levels and states, the unused light indices cleared. Returns the boards that differ.
static int check_boards(Pin &p, const std::vector<uint8_t> &lights) {
  int bad = 0;
  for (auto &bd : p.boards) {
    uint16_t out[5];
    ultra_render(&bd.u, bd.tick(p.now_us), out);   // (an effect switches at its start time, in the render)
  }
  for (uint16_t b = 0; b < p.boards.size(); b++) {
    const ultra_t &u = p.boards[b].u;
    bool ok = true;
    int line = 0;
#define OK(c) \
  do { \
    if (!(c) && ok) { \
      ok = false; \
      line = __LINE__; \
    } \
  } while (0)
    OK(u.identify == ULTRA_IDENT_VERSION);
    for (uint8_t id = 1; id < HOST_CONFIG_COUNT; id++) {
      uint16_t expected = config_value(id, b);
      if (id == ULTRA_CFG_PWM_PERIOD) expected = static_cast<uint16_t>(mcu_pwm_units_10ns(expected));
      if (id == ULTRA_CFG_RETRY && !(expected >> 8)) expected |= 0x100u;
      const uint8_t slot = id == ULTRA_CFG_SKIP_RATE ? ULTRA_CFG_SLOT_SKIP_RATE : id;   // (25: the board's slot 26)
      OK((u.cfg_valid & (1u << slot)) && u.cfg[slot] == expected);
    }
    for (uint8_t l = 0; l < HOST_LIGHTS; l++) {
      const ultra_light_t &L = u.light[l];
      const uint8_t nl = lights[b];
      if (l >= nl) {
        OK(L.defined && L.mask == 0 && L.scene_have == 3 && L.scene_v[0].m == 0 && L.scene_v[1].m == 0);
        continue;
      }
      OK(L.defined && L.mask == (nl == 1 ? 0x1F : 1u << l) && L.g_q16 == ((2200u + 100u * l) * 65536u + 500u) / 1000u);
      OK(L.flags == (nl == 1 ? ULTRA_DEF_CONSTANT_BRIGHTNESS : 0) && L.min_p[2] == 300 && L.max_p[4] == 64000);
      OK(L.opt_flags == (nl == 1 ? ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS : 0) && L.opt_n == (nl == 1 ? 3 : 0));
      if (nl == 1)
        OK(L.opt_pm[0] == 60000 && L.opt_pt[2] == 5556 && L.opt_warm == 5556 && L.opt_assist == 20000);
      OK(L.scene_have == 3);
      for (uint8_t level = 0; level < 2; level++)
        OK(L.scene_fade[level] == (level == 0 ? 7 : 3) && L.scene_v[level].m == 30000 + b + 1000 * level &&
           L.scene_v[level].a[4] == 55 && L.scene_v[level].b[2] == 60000);
      OK(L.cur.to.m == 40000 + l && L.cur.to.a[1] == 30000 && L.cur.smooth == 1);

    }
#undef OK
    if (!ok && bad++ < 3)
      std::printf("  board %u: state differs from the hub's at line %d (frames %u, slots %u)\n", b, line,
                  p.boards[b].frames, p.boards[b].slots);
  }
  return bad;
}

/// Supported commands on 50 boards with 20% loss: definitions, options, power-on levels, SET fades,
/// configuration and unused-light clearing. Released-wire loss recovery restores SET states; obsolete
/// analog effect calls are inert. IDENTIFY is a transient action and is established without injected loss.
static void test_every_command() {
  const uint16_t N = 50;
  Pin p;
  p.init(N, 77);
  p.loss = 0.2;
  p.now_us = 5e9;   // the hub's clock: 5000 s (the 32-bit TIME wrapped)
  std::vector<uint8_t> lights(N);
  for (uint16_t b = 0; b < N; b++) {
    for (uint8_t id = 1; id < HOST_CONFIG_COUNT; id++) {
      if (id > 10 && id < 21) continue;
      p.s.set_config(b, id, config_value(id, b));
    }
    for (uint8_t ch = 0; ch < 5; ch++) {
      p.s.set_channel_config(b, ch, 11, config_value(11 + ch, b));
      p.s.set_channel_config(b, ch, 12, config_value(16 + ch, b));
    }
  }
  p.s.flush_config();
  for (uint16_t b = 0; b < N; b++) {
    const uint8_t nl = static_cast<uint8_t>(b % 5 + 1);
    lights[b] = nl;
    p.s.clear_unused_lights(b, nl);
    for (uint8_t l = 0; l < nl; l++) {
      uint8_t roles[5] = {0, 0, 0, 0, 0};
      if (nl == 1) {
        std::copy(RGBWW, RGBWW + 5, roles);
      } else {
        roles[l] = ULTRA_ROLE_BRIGHTNESS;
      }
      const uint16_t mn[5] = {100, 200, 300, 400, 500}, mx[5] = {60000, 61000, 62000, 63000, 64000};
      p.s.light_def(b, l, roles, static_cast<uint16_t>(2200 + 100 * l), nl == 1 ? ULTRA_DEF_CONSTANT_BRIGHTNESS : 0,
                    mn, mx);
      const uint16_t pm[3] = {60000, 20000, 655}, pt[3] = {3704, 4545, 5556};
      p.s.light_opt(b, l, nl == 1 ? ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS : 0, 1538, 3704, 5556, 20000,
                    nl == 1 ? 3 : 0, pm, pt);
      const uint16_t sa[5] = {11, 22, 33, 44, 55}, sb[5] = {65535, 65535, 60000, 65535, 65535};
      p.s.light_scene(b, l, 0, 7, static_cast<uint16_t>(30000 + b), sa, sb);
      p.s.light_scene(b, l, 1, 3, static_cast<uint16_t>(31000 + b), sa, sb);
      const uint16_t a[5] = {65535, 30000, 20000, 10000, 5000};
      p.s.light_set(b, l, ULTRA_SET_SMOOTH, static_cast<uint16_t>(40000 + l), a, ONE5, 500,
                    static_cast<int64_t>(p.now_us) + 100000);
    }
  }
  p.boot();
  p.drain();
  // IDENTIFY is a transient action (not refresh state); establish it without simulated loss.
  p.loss = 0;
  for (uint16_t b = 0; b < N; b++) p.s.show_version(b);
  p.drain();
  p.loss = 0.2;
  p.rest(60000);   // refresh cycles: allow several complete rounds even at 20% loss
  int bad = check_boards(p, lights);
  CHECK(bad == 0, "50 boards, 20 %% of the frames lost: every board holds every command as the hub sent it (%d differ)",
        bad);
  // signal loss off / fade off on every board (main.c's signal_loss_check: ultra_all_off), then the frames are back
  for (uint16_t b = 0; b < N; b++) {
    Board &bd = p.boards[b];
    ultra_all_off(&bd.u, bd.tick(p.now_us + 4e6), b & 1 ? 1000 : 0);
    uint16_t out[5];
    ultra_render(&bd.u, bd.tick(p.now_us + 5.5e6), out);
    const uint16_t ch5 = lights[b] == 1 || lights[b] == 5 ? 500 : 0;   // (CH5 in a light: its min_power)
    CHECK(out[0] == 100 && out[4] == ch5, "board %u off after the signal loss: min_power (%u, %u)", b, out[0], out[4]);
  }
  p.now_us += 6e6;
  p.rest(60000);
  const int bad2 = check_boards(p, lights);
  CHECK(bad2 == 0, "after a signal loss off / fade off the refresh restores every board (%d differ)", bad2);
  std::printf("  every command type, 50 boards through frame_rx + the engine, 20 %% loss: %d of %u boards differ; after "
              "a signal loss off / fade off on all of them, %d\n", bad, N, bad2);
}

// ----------------------------------------------------------------------------------------- group start, two pins
/// One Home Assistant fade on two pins (50 boards x 5 lights and 10 boards x 1 light), calls over 8 ms:
/// grouped SETs interpolate at the same real time despite each receiver's HSI error.
static void test_group_phase() {
  Pin a, c;
  a.init(50, 5);
  c.init(10, 6);
  a.now_us = c.now_us = 7e9;
  for (Pin *p : {&a, &c}) {
    const uint16_t lights = p == &a ? 5 : 1;
    for (uint16_t b = 0; b < p->boards.size(); b++) {
      for (uint8_t l = 0; l < lights; l++) {
        uint8_t roles[5] = {0, 0, 0, 0, 0};
        roles[l] = ULTRA_ROLE_BRIGHTNESS;
        p->s.light_def(b, l, roles, 1000, 0, Z5, ONE5);
        p->s.light_set(b, l, 0, 65535, ONE5, ONE5, 0, static_cast<int64_t>(p->now_us));
      }
    }
    p->boot();
    p->drain();
    p->rest(3000);
  }
  const double t0 = std::max(a.now_us, c.now_us);
  a.now_us = c.now_us = t0;
  uint8_t breathe[7];
  ultra_put24(breathe, 2000);
  ultra_put16(breathe + 3, 0);
  ultra_put16(breathe + 5, 65535);
  GroupStart group;
  // the calls: lights 0-1 (and the other pin) at t0, lights 2-3 4 ms later, light 4 8 ms later; frames go on between
  const struct {
    double at;
    uint8_t first, last;
  } calls[] = {{0, 0, 1}, {4000, 2, 3}, {8000, 4, 4}};
  const uint32_t frame_a = static_cast<uint32_t>(a.s.build_idle_frame(0, a.f.data())) + 800;
  const uint32_t frame_c = static_cast<uint32_t>(c.s.build_idle_frame(0, c.f.data())) + 800;
  for (const auto &call : calls) {
    for (Pin *p : {&a, &c}) {
      while (p->now_us < t0 + call.at)
        p->frame();
    }
    const int64_t now = static_cast<int64_t>(t0 + call.at);
    const int64_t lead = std::max(start_lead_us(frame_a, 5, a.s.waiting()), start_lead_us(frame_c, 1, c.s.waiting()));
    for (Pin *p : {&a, &c}) {
      const uint16_t lights = p == &a ? 5 : 1;
      const uint32_t fr = p == &a ? frame_a : frame_c;
      for (uint16_t b = 0; b < p->boards.size(); b++) {
        for (uint8_t l = call.first; l <= call.last && l < lights; l++) {
          const int64_t at = group.start(now, lead, start_lead_us(fr, 1, p->s.waiting(b)) - START_SLACK_US);
          p->s.light_set(b, l, 0, 0, ONE5, ONE5, 10000, at);
        }
      }
    }
  }
  for (Pin *p : {&a, &c}) {
    p->drain();
    p->rest(2000);
  }
  // render every board at the same real times
  double worst = 0;
  bool mid_fade = false;
  for (int k = 0; k < 40; k++) {
    const double t = std::max(a.now_us, c.now_us) + 1000.0 + k * 37.0 * 1000.0;
    uint16_t lo = 65535, hi = 0;
    for (Pin *p : {&a, &c}) {
      for (auto &bd : p->boards) {
        uint16_t out[5];
        ultra_render(&bd.u, bd.tick(t), out);
        const uint16_t lights = p == &a ? 5 : 1;
        for (uint16_t l = 0; l < lights; l++) {
          mid_fade |= out[l] > 100 && out[l] < 65000;
          lo = std::min(lo, out[l]);
          hi = std::max(hi, out[l]);
        }
      }
    }
    worst = std::max(worst, static_cast<double>(hi - lo));
  }
  // a breathe over 2 s moves at most pi / 2 s x 65535 = ~103 counts per ms
  CHECK(mid_fade && worst <= 2 * 103 + 40, "two pins, 260 lights started by calls over 8 ms: one SET fade (spread %.0f counts)",
        worst);
  std::printf("  group start over two pins (50 x 5 + 10 x 1 lights, HSI +-2 %%): SET fade levels within %.0f counts\n",
              worst);
}

// ----------------------------------------------------------------------------------------- long-running effects
/// A candle the host started, left on for weeks. (a) The board that ran it all along: the flame still moves after
/// 2^31 ms (24.86 days; board firmware 0.6.6 takes the effect time as negative then and holds it at its start: frozen
/// until day 49.7). (b) A board that powers up on day 25 (replaced, a breaker): the hub's refresh EFFECT carries a start
/// offset of int32 ms, clipped at -24.86 days, so the effect's start seems 2^31 ms back: the board never starts it (the
/// plain state shows). Expected: the candle flickers on both.




// ----------------------------------------------------------------------------------------- gamma_correct: 0
/// ESPHome's gamma_correct: 0 is linear (its gamma table: i / 255). The light sends gamma x 1000 in LIGHT_DEF (1000 for
/// 0 with this component's fix); a LIGHT_DEF with gamma 0 must render linear too (the board's x^0 showed every level
/// that is on at 100 %).
static void test_gamma_zero() {
  for (uint16_t g : {static_cast<uint16_t>(0), static_cast<uint16_t>(1000)}) {
    Pin p;
    p.init(1, 9, 0.0);
    p.s.light_def(0, 0, MONO1, g, 0, Z5, ONE5);
    p.s.light_set(0, 0, 0, 16384, ONE5, ONE5, 0, 0);
    p.boot();
    p.drain();
    uint16_t out[5];
    ultra_render(&p.boards[0].u, p.boards[0].tick(p.now_us + 1000), out);
    CHECK(std::abs(static_cast<int>(out[0]) - 16384) <= 1,
          "LIGHT_DEF gamma %u (ESPHome gamma_correct %s): 25 %% shows as 25 %% (%u)", g, g ? "1.0" : "0 = linear",
          out[0]);
  }
}

// ----------------------------------------------------------------------------------------- the power-off notice
/// The Diff-Power port's notice (protocol 0.3): IDENTIFY 7 POWER_OFF in every board's slot, 3 copies 50 ms apart
/// (frames at rest between them). A board without standby support (the Dig2Analog-Plus-5ch engine built here; firmware
/// before 0.6.10 on any board) ignores it: its light and its output stay as they were, and an IDENTIFY sent with it
/// still works. (The Diff-Power receiver's own handling: the firmware's tests.)
static void test_power_off_ignored() {
  const uint16_t N = 4;
  Pin p;
  p.init(N, 31, 0.0);
  for (uint16_t b = 0; b < N; b++) {
    p.s.light_def(b, 0, MONO1, 1000, 0, Z5, ONE5);
    p.s.light_set(b, 0, 0, static_cast<uint16_t>(20000 + 1000 * b), ONE5, ONE5, 0, 0);
  }
  p.boot();
  p.drain();
  p.rest(1000);
  std::vector<uint16_t> before(N);
  std::vector<uint8_t> ident(N);
  for (uint16_t b = 0; b < N; b++) {
    uint16_t out[5];
    ultra_render(&p.boards[b].u, p.boards[b].tick(p.now_us), out);
    before[b] = out[0];
    ident[b] = p.boards[b].u.identify;
  }
  p.s.identify(1, true);   // queued with the notice
  int notice_slots = 0;
  for (int k = 0; k < 3; k++) {
    for (uint16_t b = 0; b < N; b++)
      p.s.power_off(b, k == 0);
    p.frame();
    for (uint16_t b = 0; b < N; b++) {
      const uint8_t *sl = p.f.data() + ULTRA_SLOT_BYTES * b;
      notice_slots += sl[0] == ULTRA_IDENTIFY && sl[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF && ultra_slot_check(sl) == 0;
    }
    p.rest(50);
  }
  bool same = true;
  for (uint16_t b = 0; b < N; b++) {
    uint16_t out[5];
    ultra_render(&p.boards[b].u, p.boards[b].tick(p.now_us), out);
    same = same && out[0] == before[b] && (b == 1 || p.boards[b].u.identify == ident[b]);
  }
  CHECK(notice_slots == 3 * N && same, "the notice in %d of %d slots; the boards ignore it (light, identify unchanged)",
        notice_slots, 3 * N);
  CHECK(p.boards[1].u.identify == ULTRA_IDENT_ON, "the IDENTIFY sent with the notice works (board 1: %u)",
        p.boards[1].u.identify);
  std::printf("  the power-off notice (IDENTIFY 7, 0.2.6): 3 copies, every board; a board without standby support "
              "ignores it\n");
}

static void test_pixel_calibration_contract() {
  static_assert(ULTRA_FEAT_PIXEL_CALIBRATION == 0x00800000u && ULTRA_UC_FORMAT == 2);
  uint8_t slot[ULTRA_SLOT_BYTES];
  for (unsigned seq = 0; seq < 256; seq++) {
    for (uint8_t action : {ULTRA_IDENT_CAL, ULTRA_IDENT_RELEASE, ULTRA_IDENT_PIXEL_CLEAR}) {
      std::fill(slot, slot + sizeof(slot), 0xFF);
      ultra_slot_pixel_calibration(slot, 0, static_cast<uint8_t>(seq), action);
      CHECK(slot[0] == 0x06 && slot[1] == 0 && slot[2] == seq && slot[3] == action,
            "digital IDENTIFY header/action/SEQ");
      CHECK(ultra_slot_check(slot) == 0, "digital request CRC");
      for (unsigned i = 4; i < 30; i++) CHECK(slot[i] == 0, "digital argument/reserved byte %u zero", i);
    }
  }
  Scheduler scheduler; scheduler.init(2, 99, 3);
  std::vector<uint8_t> frame(scheduler.frame_bytes());
  scheduler.build_calibration_frame(0, frame.data(), 1, ULTRA_IDENT_CAL);
  CHECK(frame[0] == ULTRA_NOP && frame[32] == ULTRA_IDENTIFY, "quiet request targets only its receiver");
  scheduler.build_calibration_frame(100000, frame.data(), 1, 0);
  CHECK(frame[0] == ULTRA_NOP && frame[32] == ULTRA_NOP, "sparse keepalive cannot repeat calibration automatically");
  std::printf("  digital calibration wire contract: all SEQs/actions, reserved zero, CRC and no unsolicited repeat\n");
}

int main() {
  test_pixel_calibration_contract();
  test_every_command();
  test_group_phase();
  test_gamma_zero();
  test_power_off_ignored();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA CONTRACT TESTS PASSED", failures);
  return failures ? 1 : 0;
}
