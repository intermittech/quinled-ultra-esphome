// Host test for components/dig2analog_ultra/ultra_core.h: ESPHome value mapping, PWM presets, the frame scheduler
// (copies, ordering, replacement, refresh, start offsets, the power-on effect, a full queue, streams taking turns) and
// the bit encoder. With the board firmware next to this repo (../quinled-ultra-firmware or
// ../dig2analog-ultra-5ch/firmware, or $ULTRA_FIRMWARE), the frames also go through the boards' real engine: a chain
// of boards with drifting clocks, lost frames and a board that reboots; a board replaced during a stream, a long
// transition joined late, a light removed from the YAML, the frames back after a signal-loss shutdown.
// Build: python tests/run_ultra_host_test.py   (uses the ziglang pip package as C/C++ compiler)

#include "../components/dig2analog_ultra/ultra_core.h"

#ifdef WITH_ENGINE
extern "C" {
#include "config.h"
#include "ultra.h"
}
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace esphome::dig2analog_ultra;

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

static const uint8_t RGBWW[5] = {ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_COLD_WHITE,
                                 ULTRA_ROLE_WARM_WHITE};
static const uint8_t MONO1[5] = {0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS};
static constexpr uint8_t MODE_RGB = 0x23, MODE_CWW = 0x13, MODE_RGB_CWW = 0x33, MODE_BRIGHTNESS = 0x03;

static void test_mapping() {
  uint16_t m, a[5], b[5];
  // RGB mode on an RGBWW light: colors and color brightness, whites 0
  ColorValues v{1.0f, 0.5f, 0.8f, 1.0f, 0.25f, 0.0f, 0.0f, 0.7f, 0.3f, MODE_RGB};
  map_values(RGBWW, v, m, a, b);
  CHECK(m == to16(0.5f) && a[0] == 65535 && a[1] == to16(0.25f) && a[2] == 0, "rgb values");
  CHECK(b[0] == to16(0.8f) && b[1] == to16(0.8f) && b[3] == 65535, "color brightness in B");
  CHECK(a[3] == 0 && a[4] == 0, "white off in RGB mode");
  v.caps = MODE_CWW;
  map_values(RGBWW, v, m, a, b);
  CHECK(a[0] == 0 && a[3] == to16(0.7f) && a[4] == to16(0.3f), "cw / ww mode");
  v.caps = MODE_RGB_CWW;
  map_values(RGBWW, v, m, a, b);
  CHECK(a[0] == 65535 && a[3] == to16(0.7f), "rgbww mode");
  ColorValues off{0.0f, 0.5f, 1, 1, 1, 1, 1, 1, 1, MODE_BRIGHTNESS};
  map_values(MONO1, off, m, a, b);
  CHECK(m == 0 && a[4] == 65535 && a[0] == 0, "off = M 0, brightness channel A = 1");

  uint16_t preset, period;
  pwm_for_frequency(25000.0f, preset, period);
  CHECK(preset == 8 && period == 0, "25 kHz = preset 8");
  pwm_for_frequency(2197.3f, preset, period);
  CHECK(preset == 0 && period == 0, "2.197 kHz = preset 0");
  pwm_for_frequency(33000.0f, preset, period);
  CHECK(period == 4364, "33 kHz = period 4364 (%u)", period);
  pwm_for_frequency(100000.0f, preset, period);
  CHECK(preset == 14 && period == 0, "100 kHz = preset 14");
}

// layout (wire VERSION 2): slot i at 32 i, the header (trailer) after the slots
static const uint8_t *slot_of(const std::vector<uint8_t> &f, int i) { return f.data() + ULTRA_SLOT_BYTES * i; }
static const uint8_t *hdr_of(const std::vector<uint8_t> &f) { return f.data() + f.size() - ULTRA_HDR_BYTES; }

static bool frame_ok(const std::vector<uint8_t> &f, uint16_t boards) {
  uint32_t t;
  if (f.size() != ultra_frame_bytes(boards) || ultra_parse_header(hdr_of(f), &t) != 0)
    return false;
  for (uint16_t i = 0; i < boards; i++) {
    if (ultra_slot_check(slot_of(f, i)) != 0)
      return false;
  }
  return true;
}

// The compatibility rules (ultra_protocol.h): the hub sends only defined values. A slot of a normal frame: a known
// type (never POLL, never 0x0B-0xFF), LIGHT 0-4 or 15, FLAGS only on SET (and only SMOOTH / VIA_OFF), and per type no
// unknown flag bits, roles, ids, effect ids or IDENTIFY / SCENE values, and spare bytes 0. A DARK slot (all zero) is
// fine. Returns a short reason, nullptr = defined.
static const char *slot_undefined(const uint8_t *s) {
  bool zero = true;
  for (uint32_t i = 0; i < ULTRA_SLOT_BYTES; i++)
    zero &= s[i] == 0;
  if (zero)
    return nullptr;  // DARK
  if (ultra_slot_check(s) != 0)
    return "CRC";
  const uint8_t type = s[0], light = s[1] >> 4, fl = s[1] & 0x0F, *p = s + ULTRA_PAYLOAD;
  auto spare = [&](uint32_t from) {
    for (uint32_t i = from; i < ULTRA_PAYLOAD_LEN; i++) {
      if (p[i] != 0)
        return true;
    }
    return false;
  };
  if (type >= ULTRA_TYPES || type == ULTRA_POLL)
    return "type";
  const bool board = type == ULTRA_CONFIG || type == ULTRA_IDENTIFY;
  if (type == ULTRA_NOP ? (s[1] != 0 || s[2] != 0) : board ? light != ULTRA_LIGHT_BOARD : light >= ULTRA_LIGHTS)
    return "light";
  if (fl & ~(type == ULTRA_SET ? ULTRA_SET_FLAGS : 0u))
    return "slot flags";
  switch (type) {
    case ULTRA_NOP:
      return spare(0) ? "NOP payload" : nullptr;
    case ULTRA_LIGHT_DEF:
      for (int i = 0; i < 5; i++) {
        if (((p[i >> 1] >> ((i & 1) ? 0 : 4)) & 0x0F) > ULTRA_ROLE_WARM_WHITE)
          return "role";
      }
      if ((p[2] & 0x0F) || (p[5] & ~ULTRA_DEF_FLAGS) || spare(26))
        return "LIGHT_DEF reserved bits / spare";
      return nullptr;
    case ULTRA_EFFECT:
      if (p[0] >= ULTRA_FX_COUNT)
        return "effect id";
      return spare(5 + ultra_fx_param_bytes(p[0])) ? "EFFECT spare params" : nullptr;
    case ULTRA_SCENE_EFFECT:
      if (p[0] >= ULTRA_FX_COUNT)
        return "power-on effect id";
      return spare(1 + ultra_fx_param_bytes(p[0])) ? "SCENE_EFFECT spare params" : nullptr;
    case ULTRA_STEP:
      return ((p[0] & 0x0F) == 0 || (p[0] & 0x0F) > ULTRA_MAX_STEPS || (p[0] >> 4) >= (p[0] & 0x0F)) ? "STEP index"
                                                                                                    : nullptr;
    case ULTRA_CONFIG:
      for (int i = 0; i < 9; i++) {
        const uint8_t id = p[3 * i];
        if (id >= ULTRA_CFG_COUNT || (id == ULTRA_CFG_NONE && ultra_get16(p + 3 * i + 1) != 0))
          return "CONFIG id";
      }
      return nullptr;
    case ULTRA_IDENTIFY:
      if (p[0] > ULTRA_IDENT_RELEASE && p[0] != ULTRA_IDENT_VERSION)
        return "IDENTIFY value";
      if (p[0] == ULTRA_IDENT_CAL && p[1] != 0 && (p[1] < ULTRA_IDENT_CAL_MIN || p[1] > ULTRA_IDENT_CAL_MAX))
        return "CAL argument";
      return spare(p[0] == ULTRA_IDENT_CAL || p[0] == ULTRA_IDENT_DARK ? 2 : 1) ? "IDENTIFY spare" : nullptr;
    case ULTRA_SCENE:
      return (p[0] > 1 || spare(24)) ? "SCENE mode / spare" : nullptr;
    case ULTRA_LIGHT_OPT:
      return ((p[0] & ~ULTRA_OPT_FLAGS) || spare(25)) ? "LIGHT_OPT flags / spare" : nullptr;
    default:
      return nullptr;  // SET: values only
  }
}

// A normal frame of the hub: header VERSION 2, FLAGS 0, every slot defined.
static const char *frame_undefined(const uint8_t *f, size_t n) {
  uint32_t t;
  if (ultra_parse_header(f + ULTRA_SLOT_BYTES * n, &t) != 0)
    return "header";
  if (ultra_header_flags(f + ULTRA_SLOT_BYTES * n) != 0)
    return "header flags";
  for (size_t i = 0; i < n; i++) {
    const char *why = slot_undefined(f + ULTRA_SLOT_BYTES * i);
    if (why != nullptr)
      return why;
  }
  return nullptr;
}

// Every Scheduler call, also with values that aren't defined (flags 0xFF, roles 8-15, effect ids >= 10, CONFIG ids
// 0 / 23-255, IDENTIFY / CAL / SCENE values out of range, too many points or steps): every frame it builds carries only
// defined values.
static void test_defined_values() {
  const uint16_t N = 3;
  Scheduler s;
  s.init(N, 2468, 3);
  std::mt19937 rng(1357);
  auto r8 = [&]() { return static_cast<uint8_t>(rng() & 0xFF); };
  auto r16 = [&]() { return static_cast<uint16_t>(rng() & 0xFFFF); };
  std::vector<uint8_t> f(s.frame_bytes());
  int frames = 0, bad = 0;
  const char *first = nullptr;
  int64_t now = 1000000;
  for (int it = 0; it < 20000; it++) {
    const uint16_t b = static_cast<uint16_t>(rng() % (N + 1));   // N: out of range, ignored
    const uint8_t l = static_cast<uint8_t>(rng() % 7);            // 5, 6: out of range, ignored
    uint16_t a[5], bb[5], mn[5], mx[5], pm[6], pt[6];
    uint8_t roles[5], par[ULTRA_FX_PARAMS];
    for (int i = 0; i < 5; i++) {
      a[i] = r16();
      bb[i] = r16();
      mn[i] = r16();
      mx[i] = r16();
      roles[i] = static_cast<uint8_t>(rng() % 16);
    }
    for (int i = 0; i < 6; i++) {
      pm[i] = r16();
      pt[i] = r16();
    }
    for (auto &x : par)
      x = r8();
    switch (rng() % 14) {
      case 0:
        s.light_def(b, l, roles, r16(), r8(), mn, mx);
        break;
      case 1:
        s.light_set(b, l, r8(), r16(), a, bb, rng() & 0x1FFFFFF, now + 30000);
        break;
      case 2:
        s.light_stream(b, l, r16(), a, bb, rng() % 3000, now);
        break;
      case 3:
        s.light_opt(b, l, r8(), r16(), r16(), r16(), r16(), static_cast<uint8_t>(rng() % 7), pm, pt);
        break;
      case 4:
        s.light_scene(b, l, r8(), r8(), r16(), a, bb);
        break;
      case 5: {
        std::vector<Step> steps(rng() % 11);
        for (auto &st : steps) {
          st.duration_ms = r16();
          st.transition_ms = r16();
          st.m = r16();
          for (int i = 0; i < 5; i++) {
            st.a[i] = r16();
            st.b[i] = r16();
          }
        }
        s.light_effect(b, l, r8() % 2 ? r8() : static_cast<uint8_t>(rng() % ULTRA_FX_COUNT), par, rng() % 23,
                       now + 30000, rng() % 2 ? &steps : nullptr);
        break;
      }
      case 6:
        s.set_config(b, r8(), r16());
        s.flush_config();
        break;
      case 7:
        s.identify(b, rng() % 2);
        break;
      case 8:
        s.hold_dark(b, r8());
        break;
      case 9:
        s.release_dark(b);
        break;
      case 10:
        s.calibrate_current(b, r8());
        break;
      case 11:
        if (rng() % 2)
          s.hold_effect(b, l, now);
        else
          s.release_effect(b, l);
        break;
      case 12:
        if (rng() % 20 == 0)
          s.replay(now);
        break;
      case 13:
        s.light_scene_effect(b, l, r8() % 2 ? r8() : static_cast<uint8_t>(rng() % ULTRA_FX_COUNT), par, rng() % 23);
        break;
      default:
        break;
    }
    for (int k = static_cast<int>(rng() % 3); k > 0; k--) {
      const int kind = static_cast<int>(rng() % 10);
      if (kind == 0)
        s.build_idle_frame(now, f.data());
      else if (kind == 1)
        s.build_dark_frame(now, f.data());
      else
        s.build_frame(now, f.data());
      now += 2000 + static_cast<int64_t>(rng() % 200000);
      frames++;
      const char *why = frame_undefined(f.data(), N);
      if (why != nullptr) {
        bad++;
        if (first == nullptr)
          first = why;
      }
    }
  }
  std::printf("  defined values only: %d hub frames from 20000 random Scheduler calls (undefined inputs included), "
              "%d with an undefined value\n", frames, bad);
  CHECK(bad == 0, "the hub sent an undefined value (%s)", first ? first : "");
}

static void test_scheduler() {
  Scheduler s;
  s.init(3, 12345, 3);
  std::vector<uint8_t> f(s.frame_bytes());
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};

  // two commands for board 1: first copies first (A B A B A B), then idle -> refresh / NOP
  s.light_set(1, 0, ULTRA_SET_SMOOTH, 1000, one, one, 500, 1000000);
  s.light_set(1, 1, 0, 2000, one, one, 0, 1000000);
  std::vector<uint8_t> order;
  for (int i = 0; i < 6; i++) {
    const int64_t start = 900000 + i * 2000;
    const int64_t hdr = s.build_frame(start, f.data());
    CHECK(frame_ok(f, 3), "frame CRCs");
    const uint8_t *slot = slot_of(f, 1);
    CHECK(slot[0] == ULTRA_SET, "board 1 gets its SETs");
    order.push_back(static_cast<uint8_t>(slot[1] >> 4));
    // TIME = the frame's last bit: start + its bits on the wire (default timing 250/1050, 700/650 ns)
    uint32_t hus = 0;
    ultra_parse_header(hdr_of(f), &hus);
    const int64_t wire = static_cast<int64_t>((s.wire_ns(f.data(), f.size()) + 500) / 1000);
    CHECK(hus == static_cast<uint32_t>(hdr) && std::llabs(hdr - start - wire) <= 1 && wire > 1050 && wire < 1130,
          "TIME at the frame end (%lld us after the start)", (long long) (hdr - start));
    // start offset: TIME + offset = the absolute start (1 s), within 1 ms
    const int16_t off = static_cast<int16_t>(ultra_get16(slot + ULTRA_PAYLOAD + 25));
    CHECK(std::llabs(hdr + off * 1000 - 1000000) <= 500, "start offset %d at TIME %lld", off, (long long) hdr);
    CHECK(f[0] == ULTRA_NOP || f[0] == ULTRA_SET, "board 0: nothing to do");
  }
  CHECK(order == (std::vector<uint8_t>{0, 1, 0, 1, 0, 1}), "first copies first");
  CHECK(!s.pending(), "queue empty after 3 copies each");

  // a newer SET for the same light replaces the queued one
  s.light_set(2, 0, 0, 100, one, one, 0, 0);
  s.light_set(2, 0, 0, 200, one, one, 0, 0);
  CHECK(s.queued(2) == 1, "replaced");
  s.build_frame(0, f.data());
  CHECK(ultra_get16(slot_of(f, 2) + ULTRA_PAYLOAD) == 200, "the newer one");

  // current calibration (current-sense boards): an IDENTIFY slot with ULTRA_IDENT_CAL + the duty
  while (s.pending())
    s.build_frame(0, f.data());
  s.calibrate_current(0, 40);
  s.build_frame(0, f.data());
  CHECK(frame_ok(f, 3) && f[0] == ULTRA_IDENTIFY && f[ULTRA_PAYLOAD] == ULTRA_IDENT_CAL && f[ULTRA_PAYLOAD + 1] == 40,
        "calibrate_current slot");
  // the firmware version as blinks (protocol 0.2.1): an IDENTIFY slot with ULTRA_IDENT_VERSION, argument 0
  while (s.pending())
    s.build_frame(0, f.data());
  s.show_version(1);
  s.build_frame(0, f.data());
  CHECK(frame_ok(f, 3) && slot_of(f, 1)[0] == ULTRA_IDENTIFY && slot_of(f, 1)[ULTRA_PAYLOAD] == ULTRA_IDENT_VERSION &&
            slot_of(f, 1)[ULTRA_PAYLOAD + 1] == 0 && slot_undefined(slot_of(f, 1)) == nullptr,
        "show_version slot (board 1), a defined value");
  // chain-wide (calibrate_boards, all at once): every board's slot of the same frame carries the standard test
  // (duty 0), each with its own SEQ; the copies that follow repeat that SEQ, so no board starts twice
  while (s.pending())
    s.build_frame(0, f.data());
  for (uint16_t b = 0; b < 3; b++)
    s.calibrate_current(b, 0);
  s.build_frame(0, f.data());
  {
    bool all = frame_ok(f, 3);
    uint8_t seq[3];
    for (uint16_t b = 0; b < 3; b++) {
      const uint8_t *sl = slot_of(f, b);
      all &= sl[0] == ULTRA_IDENTIFY && sl[ULTRA_PAYLOAD] == ULTRA_IDENT_CAL && sl[ULTRA_PAYLOAD + 1] == 0;
      seq[b] = sl[2];
    }
    CHECK(all, "every board calibrates from the same frame");
    s.build_frame(0, f.data());
    bool same = true;
    for (uint16_t b = 0; b < 3; b++)
      same &= slot_of(f, b)[0] == ULTRA_IDENTIFY && slot_of(f, b)[2] == seq[b];
    CHECK(same, "the next copy repeats each board's SEQ");
  }

  // refresh: an idle board cycles through its CONFIG, DEF, SET and EFFECT
  Scheduler r;
  r.init(1, 1, 1);
  std::vector<uint8_t> g(r.frame_bytes());
  r.set_config(0, ULTRA_CFG_PWM_PRESET, 8);
  r.set_config(0, ULTRA_CFG_BOARD_MAX, 5000);
  r.flush_config();
  r.light_def(0, 0, RGBWW, 2800, 0, z, one);
  r.light_set(0, 0, 0, 1000, one, one, 0, 0);
  const uint8_t fx[3] = {0, 0x0F, 0xA0};
  r.light_effect(0, 0, ULTRA_FX_BREATHE, fx, 3, 0);
  int seen[ULTRA_TYPES] = {};
  for (int i = 0; i < 80; i++) {
    r.build_frame(i * 1000, g.data());
    seen[g[0] < ULTRA_TYPES ? g[0] : 0]++;
  }
  CHECK(seen[ULTRA_CONFIG] >= 2 && seen[ULTRA_LIGHT_DEF] >= 2 && seen[ULTRA_SET] >= 2 && seen[ULTRA_EFFECT] >= 2,
        "refresh cycles through everything (%d %d %d %d)", seen[ULTRA_CONFIG], seen[ULTRA_LIGHT_DEF], seen[ULTRA_SET],
        seen[ULTRA_EFFECT]);
  CHECK(seen[ULTRA_NOP] == 0, "no NOP while there is state to refresh");

  // the same setting again is not re-sent
  r.set_config(0, ULTRA_CFG_PWM_PRESET, 8);
  r.flush_config();
  CHECK(!r.pending(), "unchanged CONFIG not queued");

  // boot protocol (persisted id 22): only sent when set; it lands in CONFIG page 3 (ids 19-27)
  CHECK(!r.has_config(0, ULTRA_CFG_BOOT_PROTOCOL), "boot protocol not set by default");
  r.set_config(0, ULTRA_CFG_BOOT_PROTOCOL, 3);
  r.flush_config();
  bool found = false;
  for (int i = 0; i < 10 && !found; i++) {
    r.build_frame(100000 + i * 1000, g.data());
    const uint8_t *s = slot_of(g, 0);
    for (int k = 0; s[0] == ULTRA_CONFIG && k < 9; k++)
      found |= s[ULTRA_PAYLOAD + 3 * k] == ULTRA_CFG_BOOT_PROTOCOL && ultra_get16(s + ULTRA_PAYLOAD + 3 * k + 1) == 3;
  }
  CHECK(found && r.has_config(0, ULTRA_CFG_BOOT_PROTOCOL), "boot protocol sent in a CONFIG slot");

  // preamble: 4N + 2 full frames of DARK slots (all zero, never valid) + the header
  CHECK(preamble_frames(1) == 6 && preamble_frames(8) == 34, "preamble 4N + 2");
  {
    Scheduler d;
    d.init(4, 3, 3);
    std::vector<uint8_t> df(d.frame_bytes(), 0x55);
    const int64_t t = d.build_dark_frame(123456789, df.data());
    uint32_t hus = 0;
    bool zero = true, invalid = true;
    for (size_t i = 0; i < 4 * ULTRA_SLOT_BYTES; i++)
      zero &= df[i] == 0;
    for (int i = 0; i < 4; i++)
      invalid &= ultra_slot_check(slot_of(df, i)) != 0;
    CHECK(zero && invalid, "DARK slots: all zero, no valid slot");
    CHECK(ultra_parse_header(hdr_of(df), &hus) == 0 && hus == static_cast<uint32_t>(t) && t > 123456789 + 1000,
          "DARK frame: header at the end, TIME at the frame end");
    // idle frames: NOP slots (valid)
    d.build_idle_frame(5000, df.data());
    CHECK(frame_ok(df, 4) && df[0] == ULTRA_NOP && df[3 * ULTRA_SLOT_BYTES] == ULTRA_NOP, "idle frame: NOP slots");
    // other bit timing: TIME follows it
    d.set_bit_timing(400, 850, 800, 450);
    const int64_t t2 = d.build_idle_frame(0, df.data());
    CHECK(std::llabs(t2 - static_cast<int64_t>((d.wire_ns(df.data(), df.size()) + 500) / 1000)) <= 1 && t2 >= 1355 &&
              t2 <= 1360,
          "TIME with 1250 ns bits: 1088 bits = 1360 us minus the last low time (%lld us)", (long long) t2);
  }
}

template<typename T> static std::vector<T> encode_all(const FrameSymbols<T> &p, const std::vector<uint8_t> &bytes,
                                                      std::mt19937 &rng) {
  std::vector<T> out;
  bool done = false;
  std::uniform_int_distribution<int> chunk(1, 64);
  while (!done) {
    std::vector<T> buf(chunk(rng));
    const size_t got = encode_symbols(p, bytes.data(), bytes.size(), out.size(), buf.size(), buf.data(), &done);
    out.insert(out.end(), buf.begin(), buf.begin() + got);
  }
  return out;
}

static void test_encoder() {
  std::mt19937 rng(7);
  const FrameSymbols<uint8_t> p{0, 1, 2};
  std::vector<uint8_t> bytes(8 + 32 * 3);
  for (auto &b : bytes)
    b = static_cast<uint8_t>(rng());
  auto sym = encode_all(p, bytes, rng);
  CHECK(sym.size() == bytes.size() * 8 + 1 && sym.back() == 2, "bits + reset");
  bool ok = true;
  for (size_t i = 0; i < bytes.size() * 8; i++)
    ok &= sym[i] == ((bytes[i >> 3] >> (7 - (i & 7))) & 1);
  CHECK(ok, "MSB first");
}

#ifdef WITH_ENGINE
/// Chain simulation: host scheduler -> frames -> board k takes slot k (its gate removes it for the rest).
struct SimBoard {
  ultra_t u;
  double drift;   // HSI error
  double phase;
};

static void test_chain() {
  const int N = 4;
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  Scheduler s;
  s.init(N, 4242, 3);
  std::vector<SimBoard> boards(N);
  for (int i = 0; i < N; i++) {
    ultra_init(&boards[i].u, 100 + i);
    boards[i].drift = (uni(rng) - 0.5) * 0.04;
    boards[i].phase = uni(rng) * 50;
  }
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  for (int b = 0; b < N; b++) {
    s.set_config(b, ULTRA_CFG_LOSS, 0);
    s.light_def(b, 0, MONO1, 1000, 0, z, one);   // gamma 1: output = M
  }
  s.flush_config();
  std::vector<uint8_t> f(s.frame_bytes());
  const int64_t T0 = 5000000000LL;  // host clock 5000 s (unwrap past 2^32 us)
  auto tick_of = [&](const SimBoard &b, double t_us) { return static_cast<uint32_t>((t_us * (1 + b.drift) + b.phase) / 50.0); };
  double t = 0, next_frame = 0;
  int frames = 0, lost = 0;
  uint32_t reboot_done = 0;
  // light 0 of every board: fade to 60 % over 2 s at t = 3 s, all boards at the same moment
  bool sent_fade = false, sent_off = false;
  uint16_t out_at[N][5];
  double spread = 0;
  while (t < 20e6) {
    if (!sent_fade && t >= 3e6) {
      sent_fade = true;
      for (int b = 0; b < N; b++)
        s.light_set(b, 0, ULTRA_SET_SMOOTH, 39321, one, one, 2000, T0 + static_cast<int64_t>(t) + 30000);
    }
    if (!sent_off && t >= 12e6) {
      sent_off = true;
      for (int b = 0; b < N; b++)
        s.light_set(b, 0, ULTRA_SET_SMOOTH, 0, one, one, 1000, T0 + static_cast<int64_t>(t) + 30000);
    }
    if (reboot_done == 0 && t >= 8e6) {  // board 2 loses power: its state comes back from the refresh
      reboot_done = 1;
      ultra_init(&boards[2].u, 777);
    }
    if (t >= next_frame) {
      const int64_t now = T0 + static_cast<int64_t>(t);
      const double end = t + static_cast<double>(s.build_frame(now, f.data()) - now);   // the frame's last bit
      frames++;
      CHECK(frame_undefined(f.data(), N) == nullptr, "chain frame: undefined value");
      const double frame_us = end - t + 300;
      for (int b = 0; b < N; b++) {
        if (uni(rng) < 0.2) {  // 20 % of frames lost / corrupted at this board
          lost++;
          continue;
        }
        // the slot from the frame start, the header from the frame end: the board's clock reference is the tick
        // at which it saw the last edge (0-50 us later), then the slot is applied
        uint32_t hus;
        CHECK(ultra_parse_header(hdr_of(f), &hus) == 0, "header");
        const uint32_t tk = tick_of(boards[b], end + uni(rng) * 50);
        ultra_on_header(&boards[b].u, tk, hus);
        ultra_on_slot(&boards[b].u, slot_of(f, b), tk);
      }
      next_frame = t + (s.pending() ? frame_us : 200000.0);
    }
    // render all boards at this real time
    if (static_cast<int64_t>(t) % 1000 < 250) {
      for (int b = 0; b < N; b++)
        ultra_render(&boards[b].u, tick_of(boards[b], t), out_at[b]);
      if (t > 3.1e6 && t < 5.2e6) {
        int lo = 65535, hi = 0;
        for (int b = 0; b < N; b++) {
          lo = out_at[b][4] < lo ? out_at[b][4] : lo;
          hi = out_at[b][4] > hi ? out_at[b][4] : hi;
        }
        spread = hi - lo > spread ? hi - lo : spread;
      }
      if (t > 6e6 && t < 7.9e6) {
        for (int b = 0; b < N; b++)
          CHECK(std::abs(out_at[b][4] - 39321) <= 1, "board %d at 60 %% (%u)", b, out_at[b][4]);
      }
      if (t > 11.5e6 && t < 11.9e6)
        CHECK(std::abs(out_at[2][4] - 39321) <= 1, "rebooted board 2 restored by the refresh (%u)", out_at[2][4]);
    }
    t += 250;
  }
  for (int b = 0; b < N; b++)
    CHECK(out_at[b][4] == 0, "board %d off at the end (%u)", b, out_at[b][4]);
  std::printf("  chain of %d boards, %d frames, %d of %d slots lost (20 %%): fade in step within %.0f counts "
              "(%.2f %%), rebooted board restored, all off\n",
              N, frames, lost, frames * N, spread, spread * 100.0 / 65535);
  CHECK(spread < 400, "boards in step during the fade (%.0f counts apart)", spread);
}
#endif

/// switch_boards_to_ultra: the WS config frames "protocol = Ultra" (ultra_switch_frame) - WS2805 then WS2915, each
/// ULTRA_WSCFG_COPIES copies and a normal (all zero) frame, lengths alternating by one byte.
static void test_switch_frames() {
  bool ok = true;
  for (uint32_t n = 1; n <= 10; n++) {
    std::vector<uint8_t> f(ultra_switch_frame_bytes(n) + 4, 0xEE);
    uint32_t prev = 0;
    for (uint32_t i = 0; i < ULTRA_SWITCH_WS_FRAMES; i++) {
      const bool ws2915 = i > ULTRA_WSCFG_COPIES, normal = i % (ULTRA_WSCFG_COPIES + 1) == ULTRA_WSCFG_COPIES;
      std::fill(f.begin(), f.end(), 0xEE);
      const uint32_t len = ultra_switch_frame(f.data(), i, n), body = ws2915 ? 4 + 10 * n : 5 * n;
      ok &= len == body + 1 + (i & 1) && len <= ultra_switch_frame_bytes(n) && f[len] == 0xEE;
      ok &= (i == 0 || i == ULTRA_WSCFG_COPIES + 1) || len == prev + 1 || len + 1 == prev;   // 8 bits apart
      prev = len;
      for (uint32_t j = body; j < len; j++)
        ok &= f[j] == 0;  // padding
      if (ws2915)
        ok &= f[0] == 0xFF && f[1] == 0xFF && f[2] == 0xFF && f[3] == 0x80;
      const uint32_t bpc = ws2915 ? 10 : 5;
      for (uint32_t b = 0; b < n; b++) {
        const uint8_t *m = f.data() + (ws2915 ? 4 : 0) + bpc * b;
        if (normal) {
          for (uint32_t j = 0; j < bpc; j++)
            ok &= m[j] == 0;
        } else {
          ok &= m[0] == 0xD2 && m[1] == 0xA5 && m[2] == ULTRA_WSCFG_PROTOCOL && m[3] == 3 && m[4] == ultra_crc8(m, 4);
          for (uint32_t j = 0; ws2915 && j < 5; j++)
            ok &= m[5 + j] == static_cast<uint8_t>(~m[j]);
        }
      }
    }
  }
  CHECK(ok, "switch frames: layout, padding, config messages");
}

#ifdef WITH_ENGINE
/// switch_boards_to_ultra's WS config frames are what the board firmware accepts (config_process: 3 identical
/// frames). (The same message from the ws2915 component's ws2915.send_config is checked in esphome-ws2915-ws2805.)
static void test_config_frames() {
  for (int is16 = 0; is16 < 2; is16++) {
    const size_t bpc = is16 ? 10 : 5;   // bytes per board: WS2915 (the message and its complement), WS2805
    // switch_boards_to_ultra's frames of this format: the message, taken by every board after 3 copies
    std::vector<uint8_t> sw(ultra_switch_frame_bytes(4));
    const uint32_t first = is16 ? ULTRA_WSCFG_COPIES + 1 : 0;
    for (uint32_t b = 0; b < 4; b++) {
      settings_t s2;
      config_rx_t rx2;
      settings_default(&s2);
      s2.protocol = is16 ? 1 : 2;
      config_rx_init(&rx2);
      int acks = 0, at = -1;
      for (uint32_t i = first; i <= first + ULTRA_WSCFG_COPIES; i++) {
        ultra_switch_frame(sw.data(), i, 4);
        slot_t sl2{};
        sl2.raw_len = static_cast<uint8_t>(bpc);
        std::memcpy(sl2.raw, sw.data() + (is16 ? 4 : 0) + bpc * b, bpc);
        const cfg_result_t r2 = config_process(&rx2, &s2, &sl2);
        if (r2.event == CFG_ACK) {
          acks++;
          at = static_cast<int>(i - first);
        }
      }
      CHECK(acks == 1 && at == 2 && s2.protocol == 3, "%s switch frames: board %u takes protocol 3 at the 3rd copy (%d, %d)",
            is16 ? "WS2915" : "WS2805", b, acks, at);
    }
  }
}
#endif

/// Power-on scene: the scheduler's SCENE item (with refresh) reaches the board, and the scene the board computes
/// equals what it renders once ESPHome restores the same state at boot (SET without transition).
static void test_scene_path() {
  Scheduler s;
  s.init(1, 77, 1);
  std::vector<uint8_t> f(s.frame_bytes());
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  ColorValues v{1.0f, 0.6f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.8f, 0.4f, MODE_CWW};
  uint16_t m, a[5], b[5];
  map_values(RGBWW, v, m, a, b);
  s.set_config(0, ULTRA_CFG_CH_CURRENT + 3, 4000);
  s.set_config(0, ULTRA_CFG_BOARD_MAX, 3000);
  s.flush_config();
  s.light_def(0, 0, RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, z, one);
  s.light_scene(0, 0, 1, 10, m, a, b);
  ultra_t u;
  ultra_init(&u, 9);
  uint32_t tick = 1000, evs = 0;
  int scenes = 0;
  for (int i = 0; i < 12; i++, tick += 4000) {
    s.build_frame(1000000 + i * 200000, f.data());
    uint32_t hus;
    ultra_parse_header(hdr_of(f), &hus);
    ultra_on_header(&u, tick, hus);
    const uint32_t ev = ultra_on_slot(&u, slot_of(f, 0), tick);
    evs |= ev;
    scenes += f[0] == ULTRA_SCENE;
  }
  CHECK((evs & ULTRA_EV_SCENE) && scenes >= 2, "SCENE sent and refreshed (%d)", scenes);
  scene_t stored{}, sc{};
  ultra_scene(&u, &stored, &sc);
  CHECK(sc.mask == 0x1F && sc.fade == 10, "scene covers the light");
  s.light_set(0, 0, 0, m, a, b, 0, 1000000 + 12 * 200000);
  s.build_frame(1000000 + 12 * 200000, f.data());
  uint32_t hus;
  ultra_parse_header(hdr_of(f), &hus);
  ultra_on_header(&u, tick, hus);
  ultra_on_slot(&u, slot_of(f, 0), tick);
  uint16_t out[5];
  ultra_render(&u, tick + 100, out);
  bool same = true;
  for (int i = 0; i < 5; i++)
    same &= out[i] == sc.v[i];
  CHECK(same, "scene == render after the boot SET (%u/%u vs %u/%u)", out[3], out[4], sc.v[3], sc.v[4]);
  std::printf("  power-on scene: SCENE item refreshed, board scene WW/CW %u/%u = render after the boot state "
              "(board limit 3 A applied)\n", sc.v[3], sc.v[4]);
}

/// Dim to warm: the scheduler's LIGHT_OPT reaches the board (and is refreshed); at full brightness nothing changes,
/// dimmed the CW / WW mix is warmer than without it, with red assist on the red / green channels.
static void test_dtw_path() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  // ESPHome's 4000 K on a 6500 / 2700 K light
  ColorValues v{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, MODE_CWW};
  const float cwm = 1e6f / 6500, wwm = 1e6f / 2700, ctm = 1e6f / 4000;
  const float ww = (ctm - cwm) / (wwm - cwm), cw = 1.0f - ww, mx = ww > cw ? ww : cw;
  v.cold_white = std::pow(cw / mx, 1.0f / 2.8f);
  v.warm_white = std::pow(ww / mx, 1.0f / 2.8f);
  uint16_t m, a[5], b[5];
  uint16_t out[2][2][5];
  for (int dtw = 0; dtw < 2; dtw++) {
    Scheduler s;
    s.init(1, 5, 1);
    std::vector<uint8_t> f(s.frame_bytes());
    s.light_def(0, 0, RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, z, one);
    if (dtw)
      s.light_opt(0, 0, ULTRA_OPT_DIM_TO_WARM, static_cast<uint16_t>(cwm * 10 + 0.5f),
                  static_cast<uint16_t>(wwm * 10 + 0.5f), static_cast<uint16_t>(1e7f / 1800 + 0.5f), 19661);
    ultra_t u;
    ultra_init(&u, 3);
    uint32_t tick = 1000;
    int opts = 0;
    for (int k = 0; k < 2; k++) {
      v.brightness = k ? 0.2f : 1.0f;
      map_values(RGBWW, v, m, a, b);
      s.light_set(0, 0, 0, m, a, b, 0, 1000000 + k * 12 * 200000);
      for (int i = 0; i < 12; i++, tick += 4000) {
        s.build_frame(1000000 + (k * 12 + i) * 200000, f.data());
        uint32_t hus;
        ultra_parse_header(hdr_of(f), &hus);
        ultra_on_header(&u, tick, hus);
        ultra_on_slot(&u, slot_of(f, 0), tick);
        opts += f[0] == ULTRA_LIGHT_OPT;
      }
      ultra_render(&u, tick, out[dtw][k]);
    }
    if (dtw)
      CHECK(opts >= 2, "LIGHT_OPT sent and refreshed (%d)", opts);
  }
  // RGBWW roles: CH1-3 RGB, CH4 cold, CH5 warm
  bool same_top = true;
  for (int i = 0; i < 5; i++)
    same_top &= std::abs(out[0][0][i] - out[1][0][i]) <= 2;
  CHECK(same_top, "full brightness: same as without dim to warm");
  const float share0 = out[0][1][4] / float(out[0][1][3] + out[0][1][4]);
  const float share1 = out[1][1][4] / float(out[1][1][3] + out[1][1][4]);
  CHECK(share1 > share0 + 0.2f, "at 20 %% brightness the warm share rises (%.2f -> %.2f)", share0, share1);
  CHECK(out[1][1][0] > 0 && out[1][1][1] == out[1][1][0] / 4 && out[1][1][2] == 0, "red assist: red, green = red / 4");
  std::printf("  dim to warm: LIGHT_OPT refreshed; at 20 %% the warm share goes %.2f -> %.2f, red assist %u\n", share0,
              share1, out[1][1][0]);
}

/// Dim to warm with set points (YAML points: 100 % 4000 K, 20 % 2700 K): the scheduler's LIGHT_OPT reaches the board,
/// which interpolates in mireds; the light's own colour temperature (here 5000 K) never makes it colder.
static void test_dtw_points_path() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  const float cwm = 1e6f / 6500, wwm = 1e6f / 2700, ctm = 1e6f / 5000;
  const float ww = (ctm - cwm) / (wwm - cwm), cw = 1.0f - ww, mx = ww > cw ? ww : cw;
  ColorValues v{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, std::pow(cw / mx, 1.0f / 2.8f), std::pow(ww / mx, 1.0f / 2.8f),
                MODE_CWW};
  const uint16_t pm[2] = {65535, 13107}, pt[2] = {2500, 3704};   // 100 % 4000 K, 20 % 2700 K (mireds x 10)
  Scheduler s;
  s.init(1, 5, 1);
  std::vector<uint8_t> f(s.frame_bytes());
  s.light_def(0, 0, RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, z, one);
  s.light_opt(0, 0, ULTRA_OPT_DIM_TO_WARM | ULTRA_OPT_POINTS, static_cast<uint16_t>(cwm * 10 + 0.5f),
              static_cast<uint16_t>(wwm * 10 + 0.5f), 3704, 0, 2, pm, pt);
  ultra_t u;
  ultra_init(&u, 3);
  uint32_t tick = 1000;
  const float br[3] = {1.0f, 0.6f, 0.2f};
  const float want[3] = {4000.0f, 1e6f / (250.0f + (370.37f - 250.0f) * 0.5f), 2700.0f};
  bool ok = true;
  for (int k = 0; k < 3; k++) {
    uint16_t m, a[5], b[5], out[5];
    v.brightness = br[k];
    map_values(RGBWW, v, m, a, b);
    s.light_set(0, 0, 0, m, a, b, 0, 1000000 + k * 12 * 200000);
    for (int i = 0; i < 12; i++, tick += 4000) {
      s.build_frame(1000000 + (k * 12 + i) * 200000, f.data());
      uint32_t hus;
      ultra_parse_header(hdr_of(f), &hus);
      ultra_on_header(&u, tick, hus);
      ultra_on_slot(&u, slot_of(f, 0), tick);
    }
    ultra_render(&u, tick, out);
    const float sw = out[4] / float(out[3] + out[4]);   // RGBWW roles: CH4 cold, CH5 warm
    const float kk = 1e6f / (cwm + (wwm - cwm) * sw);
    ok &= std::fabs(kk - want[k]) < 25.0f;
    if (std::fabs(kk - want[k]) >= 25.0f)
      std::printf("  set points at %.0f %%: %.0f K, expected %.0f K\n", br[k] * 100, kk, want[k]);
  }
  CHECK(ok, "dim to warm set points: 4000 K at 100 %%, interpolated at 60 %%, 2700 K at 20 %%");
  std::printf("  dim to warm set points (100 %% 4000 K, 20 %% 2700 K) on a 5000 K light: board follows the points\n");
}

// Protocol test vector slot_scene_effect (quinled-ultra-protocol test-vectors/vectors.json; run_ultra_host_test.py
// checks this copy against it): candle (intensity 0.5, speed 100 %) as light 0's power-on effect, SEQ 0x79.
static const char *const VEC_SLOT_SCENE_EFFECT = "0a007905800000640000000000000000000000000000000000000000000000cd";

static std::vector<uint8_t> from_hex(const char *h) {
  std::vector<uint8_t> out;
  for (; h[0] && h[1]; h += 2)
    out.push_back(static_cast<uint8_t>(std::strtoul(std::string(h, 2).c_str(), nullptr, 16)));
  return out;
}

/// Power-on effect (SCENE_EFFECT, protocol 0.2.3; YAML power_on_effect): the scheduler's slot equals the protocol's test
/// vector and goes out after the light's SCENE (copies, refresh, replay with the same SEQ); a new SEQ only when it
/// changes; a light without one gets none (no effect, no parameter bytes). With the board engine: the board takes it,
/// a board powered up from the stored head runs the candle on the SCENE values, and none / strobe store no effect.
static void test_scene_effect() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  uint8_t p[ULTRA_FX_PARAMS] = {};
  ultra_put16(p, to16(0.5f));  // as UltraCandleEffect sends intensity 50 %, speed 100 %
  ultra_put16(p + 2, 100);
  ColorValues v{1.0f, 0.6f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.4f, 1.0f, MODE_CWW};
  uint16_t m, a[5], b[5];
  map_values(RGBWW, v, m, a, b);
  Scheduler s;
  s.init(1, 31, 3);
  std::vector<uint8_t> f(s.frame_bytes());
#ifdef WITH_ENGINE
  ultra_t u;
  ultra_init(&u, 9);
#endif
  int64_t now = 1000000;
  uint32_t tick = 1000;
  auto frame = [&]() {
    s.build_frame(now, f.data());
    CHECK(frame_undefined(f.data(), 1) == nullptr, "power-on effect frame: undefined value");
#ifdef WITH_ENGINE
    uint32_t hus;
    ultra_parse_header(hdr_of(f), &hus);
    ultra_on_header(&u, tick, hus);
    ultra_on_slot(&u, slot_of(f, 0), tick);
#endif
    now += 200000;
    tick += 4000;
    return f[0];
  };
  s.light_def(0, 0, RGBWW, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS, z, one);
  s.light_scene(0, 0, 1, 10, m, a, b);
  s.light_scene_effect(0, 0, ULTRA_FX_CANDLE, p, 4);
  std::vector<uint8_t> types;
  uint8_t sent[ULTRA_SLOT_BYTES] = {};
  while (s.pending() && types.size() < 20) {
    types.push_back(frame());
    if (f[0] == ULTRA_SCENE_EFFECT)
      std::memcpy(sent, f.data(), ULTRA_SLOT_BYTES);
  }
  const std::vector<uint8_t> want = {ULTRA_LIGHT_DEF, ULTRA_SCENE, ULTRA_SCENE_EFFECT, ULTRA_LIGHT_DEF, ULTRA_SCENE,
                                     ULTRA_SCENE_EFFECT, ULTRA_LIGHT_DEF, ULTRA_SCENE, ULTRA_SCENE_EFFECT};
  CHECK(types == want, "SCENE_EFFECT goes out after the SCENE, 3 copies each");
  const std::vector<uint8_t> vec = from_hex(VEC_SLOT_SCENE_EFFECT);
  uint8_t t[ULTRA_SLOT_BYTES];
  std::memcpy(t, sent, sizeof t);
  t[2] = 0x79;  // the vector's SEQ
  ultra_slot_seal(t);
  CHECK(vec.size() == ULTRA_SLOT_BYTES && std::memcmp(t, vec.data(), ULTRA_SLOT_BYTES) == 0 &&
            ultra_slot_check(sent) == 0,
        "the slot = test vector slot_scene_effect (with its SEQ)");
  const uint8_t seq = sent[2];

  // unchanged: nothing queued; the refresh repeats it (same SEQ), so does a replay
  s.light_scene_effect(0, 0, ULTRA_FX_CANDLE, p, 4);
  CHECK(!s.pending(), "an unchanged power-on effect queues nothing");
  int refreshed = 0;
  bool same = true;
  for (int i = 0; i < 40; i++) {
    if (frame() == ULTRA_SCENE_EFFECT) {
      refreshed++;
      same &= std::memcmp(f.data(), sent, ULTRA_SLOT_BYTES) == 0;
    }
  }
  CHECK(refreshed >= 2 && same, "refreshed with the same SEQ (%d times)", refreshed);
  s.replay(now);
  types.clear();
  same = true;
  while (s.pending() && types.size() < 40) {
    types.push_back(frame());
    if (f[0] == ULTRA_SCENE_EFFECT)
      same &= f[2] == seq;
  }
  const auto pos = [&](uint8_t type) { return std::find(types.begin(), types.end(), type) - types.begin(); };
  CHECK(pos(ULTRA_SCENE_EFFECT) < static_cast<long>(types.size()) && pos(ULTRA_SCENE) < pos(ULTRA_SCENE_EFFECT) && same,
        "replay: again after the SCENE, same SEQ");

#ifdef WITH_ENGINE
  CHECK(u.light[0].scene_fx_have && u.light[0].scene_fx == ULTRA_FX_CANDLE && ultra_scene_fx_ok(&u.light[0]) &&
            std::memcmp(u.light[0].scene_fxp, p, 4) == 0,
        "the board took the candle as the light's power-on effect");
  // the board's power-up: its stored head (main.c / hal/hw.c), no host; the candle flickers below the scene's outputs
  scene_t stored{}, sc{};
  ultra_scene(&u, &stored, &sc);
  ultra_t up;
  ultra_init(&up, 9);
  std::memcpy(&up.light[0], &u.light[0], ULTRA_HEAD_BYTES);
  ultra_scene_fx_start(&up, 1, 0);
  uint16_t out[5], lo = 65535, hi = 0;
  for (uint32_t ms = 0; ms < 3000; ms++) {
    ultra_render(&up, ms * 20u, out);
    lo = std::min(lo, out[4]);
    hi = std::max(hi, out[4]);
  }
  CHECK(up.light[0].fx == ULTRA_FX_CANDLE && hi <= sc.v[4] + 1 && lo < sc.v[4] * 9 / 10 && hi > lo,
        "power-up without a host: candle on WW %u..%u (scene %u)", lo, hi, sc.v[4]);
  std::printf("  power-on effect: SCENE_EFFECT = test vector, after the SCENE, refreshed; board power-up runs the candle "
              "(WW %u..%u of the scene's %u)\n", lo, hi, sc.v[4]);
#endif

  // changed: a new SEQ
  ultra_put16(p + 2, 50);
  s.light_scene_effect(0, 0, ULTRA_FX_CANDLE, p, 4);
  CHECK(s.pending(), "a changed power-on effect is queued");
  while (s.pending() && frame() != ULTRA_SCENE_EFFECT) {
  }
  CHECK(f[0] == ULTRA_SCENE_EFFECT && f[2] != seq && ultra_get16(f.data() + ULTRA_PAYLOAD + 3) == 50,
        "changed: new SEQ, the new parameters");

  // a light without power_on_effect: none, without parameter bytes (the board stores no effect for it, and drops one an
  // earlier configuration stored); strobe isn't stored either (its steps aren't): the plain values
  s.light_scene_effect(0, 1, ULTRA_FX_NONE, p, 4);
  bool none = false;
  for (int i = 0; i < 8 && s.pending(); i++) {
    if (frame() == ULTRA_SCENE_EFFECT && (f[1] >> 4) == 1) {
      none = f[ULTRA_PAYLOAD] == ULTRA_FX_NONE;
      for (uint32_t k = 1; k < ULTRA_PAYLOAD_LEN; k++)
        none &= f[ULTRA_PAYLOAD + k] == 0;
    }
  }
  CHECK(none, "no power_on_effect: SCENE_EFFECT none, no parameter bytes");
#ifdef WITH_ENGINE
  CHECK(u.light[1].scene_fx_have && u.light[1].scene_fx == ULTRA_FX_NONE && !ultra_scene_fx_ok(&u.light[1]),
        "the board stores no effect for it");
  uint8_t sl[ULTRA_SLOT_BYTES];
  const uint8_t steps[1] = {2};
  ultra_slot_scene_effect(sl, 0, static_cast<uint8_t>(seq + 50), ULTRA_FX_STROBE, steps, 1);
  ultra_on_slot(&u, sl, tick);
  CHECK(u.light[0].scene_fx == ULTRA_FX_STROBE && !ultra_scene_fx_ok(&u.light[0]), "strobe: not stored (plain values)");
  ultra_slot_scene_effect(sl, 0, static_cast<uint8_t>(seq + 51), ULTRA_FX_NONE, nullptr, 0);
  ultra_on_slot(&u, sl, tick);
  CHECK(u.light[0].scene_fx == ULTRA_FX_NONE && !ultra_scene_fx_ok(&u.light[0]), "none clears the light's effect");
#endif
}

// ---------------------------------------------------------------------------------------- pin calibration sequence
// CalSequencer on a Scheduler, frames back to back while commands are pending (as the hub sends them): when each
// board sees DARK / CAL / RELEASE (first and last copy), and with the board engine which IDENTIFY events a board
// actually takes (a board whose slots are all lost takes none, and the run goes on without it).
struct CalTrace {
  double dark_first = -1, dark_last = -1, cal_first = -1, cal_last = -1, rel_first = -1;
  int cal_copies = 0;
  std::vector<uint8_t> events;  // with the engine: the IDENTIFY values the board took, in order
};

struct CalRun {
  std::vector<CalTrace> tr;
  double done_ms = -1;
  int boards_started = 0;
};

// n boards, test `boards` (empty = all); drop: a board whose slots are all lost; cancel_after: cancel once this many
// boards were started (0 = never); frames: false = the line can't send (no frames at all)
static CalRun run_cal(int n, const std::vector<uint16_t> &boards, int drop, int cancel_after, bool frames,
                      Scheduler *other = nullptr, int *other_identify = nullptr) {
  Scheduler s;
  s.init(static_cast<uint16_t>(n), 77, 3);
  CalSequencer q;
  CalRun r;
  r.tr.resize(static_cast<size_t>(n));
#ifdef WITH_ENGINE
  std::vector<ultra_t> u(static_cast<size_t>(n));
  for (int b = 0; b < n; b++)
    ultra_init(&u[static_cast<size_t>(b)], 300u + static_cast<uint32_t>(b));
#endif
  std::vector<uint8_t> f(s.frame_bytes()), g(other ? other->frame_bytes() : 0);
  const int64_t T0 = 1000000000LL;
  double next_frame = 0;
  CHECK(q.start(s, static_cast<uint16_t>(n), boards, 0, 0), "start");
  CHECK(!q.start(s, static_cast<uint16_t>(n), boards, 0, 0), "a second start while running is ignored");
  for (uint32_t now = 0; now < 120000 && q.running(); now++) {
    const CalSequencer::Event ev = q.poll(s, now);
    if (ev == CalSequencer::EV_BOARD) {
      r.boards_started++;
      if (cancel_after && r.boards_started == cancel_after + 1) {
        q.cancel(s);
      }
    }
    if (ev == CalSequencer::EV_DONE)
      r.done_ms = now;
    if (other && now % 200 == 0) {  // another pin: its normal frames, nothing of this run
      other->build_frame(T0 + now * 1000LL, g.data());
      for (size_t b = 0; b < other->frame_bytes() / ULTRA_SLOT_BYTES - 1; b++)
        *other_identify += slot_of(g, static_cast<int>(b))[0] == ULTRA_IDENTIFY;
    }
    if (!frames || !s.pending() || now < next_frame)
      continue;
    const int64_t hdr = s.build_frame(T0 + now * 1000LL, f.data());
    next_frame = now + static_cast<double>(hdr - (T0 + now * 1000LL)) / 1000.0 + 0.3;
    CHECK(frame_undefined(f.data(), static_cast<size_t>(n)) == nullptr, "calibration frame: undefined value");
    for (int b = 0; b < n; b++) {
      const uint8_t *sl = slot_of(f, b);
      CalTrace &t = r.tr[static_cast<size_t>(b)];
      if (b == drop)
        continue;
      if (sl[0] == ULTRA_IDENTIFY) {
        const uint8_t what = sl[ULTRA_PAYLOAD];
        if (what == ULTRA_IDENT_DARK) {
          if (t.dark_first < 0) t.dark_first = now;
          t.dark_last = now;
        } else if (what == ULTRA_IDENT_CAL) {
          if (t.cal_first < 0) t.cal_first = now;
          t.cal_last = now;
          t.cal_copies++;
        } else if (what == ULTRA_IDENT_RELEASE && t.rel_first < 0) {
          t.rel_first = now;
        }
      }
#ifdef WITH_ENGINE
      ultra_on_header(&u[static_cast<size_t>(b)], now * 20u, static_cast<uint32_t>(hdr));
      if (ultra_on_slot(&u[static_cast<size_t>(b)], sl, now * 20u) & ULTRA_EV_IDENTIFY)
        t.events.push_back(u[static_cast<size_t>(b)].identify);
#endif
    }
  }
  if (r.done_ms < 0 && !q.running())
    r.done_ms = -2;  // ended by cancel
  return r;
}

static void test_cal_sequencer() {
  const int N = 4;
  std::printf("pin calibration: dark, one board after the other, release\n");
  {
    Scheduler other;
    other.init(2, 5, 3);
    const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
    other.light_def(0, 0, MONO1, 1000, 0, one, one);
    other.light_set(0, 0, 0, 30000, one, one, 0, 0);
    int other_identify = 0;
    CalRun r = run_cal(N, {}, -1, 0, true, &other, &other_identify);
    double dark_done = 0, prev_end = -1;
    bool order = true, dark_before = true, no_overlap = true, released = true;
    for (int b = 0; b < N; b++) {
      const CalTrace &t = r.tr[static_cast<size_t>(b)];
      dark_done = std::max(dark_done, t.dark_last);
      CHECK(t.dark_first >= 0 && t.cal_first >= 0 && t.rel_first >= 0 && t.cal_copies == 3,
            "board %d: dark %.0f, test %.0f (%d copies), release %.0f", b, t.dark_first, t.cal_first, t.cal_copies,
            t.rel_first);
      if (b > 0 && t.cal_first <= r.tr[static_cast<size_t>(b - 1)].cal_first)
        order = false;
      if (prev_end >= 0 && t.cal_first < prev_end)
        no_overlap = false;
      prev_end = t.cal_last + CalSequencer::TEST_MS;     // the latest a test that started on its last copy ends
    }
    for (int b = 0; b < N; b++) {
      if (r.tr[static_cast<size_t>(b)].cal_first < dark_done + 300)
        dark_before = false;
      if (r.tr[static_cast<size_t>(b)].rel_first < prev_end)
        released = false;
    }
    CHECK(dark_before, "every board is dark (fade done) before the first test");
    CHECK(order && no_overlap, "chain order, one test at a time (no overlap even for a board taking its last copy)");
    CHECK(released, "every board released after the last test");
    CHECK(r.done_ms > 0 && r.done_ms < 350 + N * 1150 + 200, "done after %.0f ms", r.done_ms);
    CHECK(other_identify == 0, "another pin: no dark / test commands, its frames go on");
    std::printf("  4 boards: dark at %.0f ms, tests at %.0f / %.0f / %.0f / %.0f ms, released at %.0f ms (%.1f s)\n",
                r.tr[0].dark_first, r.tr[0].cal_first, r.tr[1].cal_first, r.tr[2].cal_first, r.tr[3].cal_first,
                r.tr[0].rel_first, r.done_ms / 1000.0);
  }
  {
    CalRun r = run_cal(N, {1, 3}, -1, 0, true);
    CHECK(r.tr[0].dark_first >= 0 && r.tr[2].dark_first >= 0 && r.tr[0].cal_first < 0 && r.tr[2].cal_first < 0 &&
              r.tr[1].cal_first >= 0 && r.tr[3].cal_first >= 0 && r.tr[0].rel_first >= 0,
          "boards [1, 3]: the whole pin dark and released, only those two tested");
  }
  {
    CalRun r = run_cal(N, {}, -1, 1, true);   // cancel when the second board starts
    bool all_rel = true;
    for (int b = 0; b < N; b++)
      all_rel &= r.tr[static_cast<size_t>(b)].rel_first >= 0;
    CHECK(r.done_ms == -2 && all_rel && r.tr[2].cal_first < 0 && r.tr[3].cal_first < 0,
          "cancel: every board released at once, no further tests");
    CHECK(r.tr[1].cal_copies <= 3, "cancel: the pending test command is replaced by the release");
  }
  {
    CalRun r = run_cal(N, {}, 2, 0, true);    // board 2's slots are all lost: its command never arrives
    CHECK(r.done_ms > 0 && r.tr[3].cal_first >= 0, "a board that misses its command doesn't block the run");
#ifdef WITH_ENGINE
    const std::vector<uint8_t> want = {ULTRA_IDENT_DARK, ULTRA_IDENT_CAL, ULTRA_IDENT_RELEASE};
    CHECK(r.tr[0].events == want && r.tr[3].events == want && r.tr[2].events.empty(),
          "board engine: dark, one test, release (repeats ignored); the lost board took nothing");
#endif
  }
  {
    CalRun r = run_cal(N, {}, -1, 0, false);  // the line can't send at all (e.g. an update session)
    CHECK(r.done_ms > 0 && r.done_ms < 2000 + N * (2000 + 1100) + 1000, "no frames: the run still ends (%.0f ms)",
          r.done_ms);
  }
  CHECK(CalSequencer::hold_s(64, 0) >= 80 && CalSequencer::hold_s(4, 0) >= 15 && CalSequencer::hold_s(250, 0) == 255,
        "dark hold covers the whole run (+10 s), at most 255 s");
}

// 50 boards on one pin (officially supported, research/chain-50-boards.md): frame, timing, preambles, the hub's RAM
// per board, and a pin calibration of all 50.
struct SchedPeek : Scheduler {
  static size_t board_bytes() { return sizeof(Board); }
  static size_t light_bytes() { return sizeof(LightItems); }
  static size_t cmd_bytes() { return sizeof(Cmd); }
  void set_order(uint32_t order) { this->order_ = order; }
  size_t lights_allocated() const {
    size_t n = 0;
    for (const auto &b : this->boards_) {
      for (const auto &l : b.light)
        n += l ? 1 : 0;
    }
    return n;
  }
  size_t queue_capacity() const {
    size_t n = 0;
    for (const auto &b : this->boards_)
      n += b.queue.capacity();
    return n;
  }
};

/// The refresh rotation (the rule shared with the sender library): idle frames take each board's refresh list; the
/// clearing slots of unused light indices are in rounds 0, 1, 2, 6, 10, ... after the init, and again after a change
/// of the board's lights (from round 0); the used lights' items in every round. 3 boards with 1 / 3 / 5 lights, 20
/// rounds; then board 0 to 2 lights and 10 more. The rounds go to the cross-check trace (the sender's in the same
/// folder) when that folder exists. And a 1-light board's SET refresh period before / now.
struct RotPeek : Scheduler {
  /// board b's next refresh slot (as an idle frame takes it) and its position in the list
  bool next(uint16_t b, uint8_t *s, uint16_t *pos) {
    Board &bd = this->boards_[b];
    if (!this->take_refresh_(bd, s, 0, 0))
      return false;
    const uint16_t total = CONFIG_PAGES + ULTRA_LIGHTS * (6 + ULTRA_MAX_STEPS);
    *pos = static_cast<uint16_t>((bd.rr + total - 1) % total);
    return true;
  }
};

static void rot_lights(RotPeek &s, uint16_t b, uint8_t from, uint8_t n) {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  for (uint8_t l = from; l < n; l++) {
    s.light_def(b, l, MONO1, 1000, 0, z, one);
    s.light_opt(b, l, 0, 0, 0, 0, 0);
    s.light_scene(b, l, 0, 0, 0, z, one);
    s.light_scene_effect(b, l, ULTRA_FX_NONE, nullptr, 0);
    s.light_set(b, l, 0, 30000, one, one, 0, 0);
  }
  s.clear_unused_lights(b, n);
}

static void test_refresh_rotation() {
  std::printf("the refresh rotation: the clearing slots in rounds 0, 1, 2, 6, 10, ...\n");
  uint8_t lights[3] = {1, 3, 5};
  RotPeek s;
  s.init(3, 11, 3);
  for (uint16_t b = 0; b < 3; b++) {
    s.set_config(b, 1, 25000);
    s.set_config(b, 2, 1);
  }
  s.flush_config();
  for (uint16_t b = 0; b < 3; b++)
    rot_lights(s, b, 0, lights[b]);
  // per board: its rounds, each the (type, light) pairs of its refresh slots; a round ends where the list restarts
  std::vector<std::vector<std::vector<std::pair<int, int>>>> rounds(3);
  std::vector<int> last(3, -1);
  auto restart = [&](uint16_t b) {   // (a change: the list from its start, a new round)
    rounds[b].emplace_back();
    last[b] = -1;
  };
  for (uint16_t b = 0; b < 3; b++)
    restart(b);
  auto run = [&](const size_t want[3]) {   // until board b has want[b] complete rounds (and the next one begun)
    for (bool more = true; more;) {
      more = false;
      for (uint16_t b = 0; b < 3; b++) {
        if (rounds[b].size() > want[b])
          continue;
        uint8_t slot[ULTRA_SLOT_BYTES];
        uint16_t pos;
        if (!s.next(b, slot, &pos))
          continue;
        if (static_cast<int>(pos) <= last[b])
          rounds[b].emplace_back();
        last[b] = pos;
        rounds[b].back().push_back({slot[0], slot[1] >> 4});
        more = true;
      }
    }
  };
  // one line per round, as the sender library's trace: the cleared light indices, the used lights' DEF / SET
  std::vector<std::string> trace;
  auto line = [&](const char *phase, uint16_t b, size_t r, size_t label) {
    const auto &v = rounds[b][r];
    std::string clear, used;
    for (int l = 0; l < static_cast<int>(ULTRA_LIGHTS); l++) {
      bool cl = false, def = false, set = false;
      for (const auto &p : v) {
        if (p.second != l)
          continue;
        cl |= l >= lights[b];
        def |= l < lights[b] && p.first == ULTRA_LIGHT_DEF;
        set |= l < lights[b] && p.first == ULTRA_SET;
      }
      if (cl)
        clear += (clear.empty() ? "" : ",") + std::to_string(l);
      if (def)
        used += " DEF" + std::to_string(l);
      if (set)
        used += " SET" + std::to_string(l);
    }
    trace.push_back(std::string(phase) + " b" + std::to_string(b) + " r" + std::to_string(label) + " clear " +
                    (clear.empty() ? "-" : clear) + " |" + (used.empty() ? "" : " used" + used));
  };
  auto clears_ok = [&](uint16_t b, size_t from, size_t n) {   // rounds from..from+n-1: rounds 0, 1, 2, 6, ... clear
    bool ok = true;
    for (size_t r = 0; r < n; r++) {
      int c = 0, sets = 0;
      for (const auto &p : rounds[b][from + r]) {
        c += p.second >= lights[b] && p.second < static_cast<int>(ULTRA_LIGHTS);
        sets += p.first == ULTRA_SET;
      }
      ok &= c == ((r < 3 || r % 4 == 2) ? 3 * (ULTRA_LIGHTS - lights[b]) : 0) && sets == lights[b];
    }
    return ok;
  };
  // init: 20 rounds each
  const size_t w1[3] = {20, 20, 20};
  run(w1);
  bool rule = true;
  for (uint16_t b = 0; b < 3; b++) {
    rule &= clears_ok(b, 0, 20);
    for (size_t r = 0; r < 20; r++)
      line("init", b, r, r);
  }
  CHECK(rule, "init, 20 rounds: the clearing slots in rounds 0, 1, 2, 6, 10, 14, 18 only, every used light's SET in "
        "every round");
  const size_t with = rounds[0][0].size(), without = rounds[0][3].size();
  std::printf("  a 1-light board's SET every %zu idle frames in the first rounds, %zu after (at 200 ms: %.1f s -> "
              "%.1f s; 3 of 4 rounds)\n", with, without, with * 0.2, without * 0.2);
  // set_lights: board 0 to 2 lights (the clears from round 0 again, for 2-4); the others go on (rounds 20-29)
  rot_lights(s, 0, 1, 2);
  lights[0] = 2;
  const size_t m0 = rounds[0].size();
  restart(0);
  const size_t w2[3] = {m0 + 10, 30, 30};
  run(w2);
  CHECK(clears_ok(0, m0, 10), "set_lights: board 0's clearing slots (lights 2-4) from round 0 again");
  for (size_t r = 0; r < 10; r++)
    line("set_lights", 0, m0 + r, r);
  for (uint16_t b = 1; b < 3; b++)
    for (size_t r = 20; r < 30; r++)
      line("set_lights", b, r, r);
  // resync: every board from round 0 again
  s.replay(0);
  size_t m[3];
  for (uint16_t b = 0; b < 3; b++) {
    m[b] = rounds[b].size();
    restart(b);
  }
  const size_t w3[3] = {m[0] + 10, m[1] + 10, m[2] + 10};
  run(w3);
  bool again = true;
  for (uint16_t b = 0; b < 3; b++) {
    again &= clears_ok(b, m[b], 10);
    for (size_t r = 0; r < 10; r++)
      line("resync", b, m[b] + r, r);
  }
  CHECK(again, "resync: every board's clearing slots from round 0 again");
  // the cross-check trace (review-r11/rotation-trace/esphome.txt; compare.py there), when that folder exists
  if (FILE *f = std::fopen("D:/Claude-Folders/Claude-WS2915_dig2analog/review-r11/rotation-trace/esphome.txt", "w")) {
    for (const auto &l : trace)
      std::fprintf(f, "%s\n", l.c_str());
    std::fclose(f);
  }
}

static void test_fifty_boards() {
  const uint16_t N = 50;
  std::printf("50 boards on one pin\n");
  SchedPeek s;
  s.init(N, 4711, 3);
  std::vector<uint8_t> f(s.frame_bytes());
  CHECK(s.frame_bytes() == 32u * 50u + 8u, "frame bytes %u", static_cast<unsigned>(s.frame_bytes()));
  const int64_t t_idle = s.build_idle_frame(0, f.data());
  CHECK(frame_undefined(f.data(), N) == nullptr, "idle frame defined");
  CHECK(t_idle > 16500 && t_idle < 17500, "a 50-board frame is %lld us on the wire (12,864 bits)",
        static_cast<long long>(t_idle));
  CHECK(preamble_frames(N) == 202 && power_on_preamble_frames(N) == 10 && power_on_preamble_frames(1) == 6 &&
            power_on_preamble_frames(2) == 10,
        "preambles: full %u, after a supply switch-on %u", preamble_frames(N), power_on_preamble_frames(N));
  // RAM: per-light state only for lights in use; the queue is freed once it is empty
  CHECK(s.lights_allocated() == 0, "no per-light state before the first command");
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  for (uint16_t b = 0; b < N; b++) {
    s.light_def(b, 0, MONO1, 1000, 0, z, one);
    s.light_set(b, 0, 0, 30000, one, one, 1000, 0);
  }
  CHECK(s.lights_allocated() == N, "one light per board: %u light states", static_cast<unsigned>(s.lights_allocated()));
  int frames = 0;
  for (; s.pending() && frames < 20; frames++)
    s.build_frame(100000 + frames * 20000, f.data());
  CHECK(!s.pending() && frames == 6 && s.queue_capacity() == 0, "2 commands x 3 copies in %d frames, queues freed (%u)",
        frames, static_cast<unsigned>(s.queue_capacity()));
  // the hub's boot: per board its CONFIG pages, one light (DEF, OPT, EFFECT none, SCENE, SCENE_EFFECT, SET) and the
  // clearing of the 4 unused light indices, queued at once
  size_t boot_bytes = 0;
  {
    SchedPeek boot;
    boot.init(N, 5, 3);
    bool all = true;
    for (uint16_t b = 0; b < N; b++) {
      for (uint8_t id = 1; id < ULTRA_CFG_COUNT; id++)
        boot.set_config(b, id, 1);
    }
    boot.flush_config();
    for (uint16_t b = 0; b < N; b++) {
      boot.clear_unused_lights(b, 1);
      boot.light_def(b, 0, MONO1, 1000, 0, z, one);
      boot.light_opt(b, 0, 0, 0, 0, 0, 0);
      boot.light_effect(b, 0, ULTRA_FX_NONE, nullptr, 0, 0);
      boot.light_scene(b, 0, 0, 0, 0, z, one);
      boot.light_scene_effect(b, 0, ULTRA_FX_NONE, nullptr, 0);
      boot.light_set(b, 0, 0, 30000, one, one, 0, 0);
      all &= boot.queued(b) == 3 + 4 * 3 + 6;
    }
    boot_bytes = boot.queue_capacity() * SchedPeek::cmd_bytes();
    CHECK(all, "boot: 21 commands per board queued (%u)", static_cast<unsigned>(boot.queued(0)));
  }
  std::printf("  frame %lld us (%lld frames/s), preamble 202 frames (~%.1f s), after a supply switch-on 10 (~%.2f s); hub RAM "
              "per board %u B + %u B per light in use (this host's pointer size)\n",
              static_cast<long long>(t_idle), 1000000LL / (t_idle + 300), 202 * (t_idle + 500) / 1e6,
              10 * (t_idle + 500) / 1e6, static_cast<unsigned>(SchedPeek::board_bytes()),
              static_cast<unsigned>(SchedPeek::light_bytes()));
  CHECK(SchedPeek::board_bytes() <= 512, "per-board state stays small (%u B)", static_cast<unsigned>(SchedPeek::board_bytes()));
  std::printf("  boot: 21 commands per board, the queues take %.1f KB until they are sent\n", boot_bytes / 1024.0);
  // a pin calibration of all 50: dark, one board after the other in chain order, the dark hold covers it
  {
    CalRun r = run_cal(N, {}, -1, 0, true);
    bool order = true, all = true;
    for (int b = 0; b < N; b++) {
      const CalTrace &t = r.tr[static_cast<size_t>(b)];
      all &= t.dark_first >= 0 && t.cal_first >= 0 && t.rel_first >= 0 && t.cal_copies == 3;
      if (b > 0 && t.cal_first < r.tr[static_cast<size_t>(b - 1)].cal_first + CalSequencer::TEST_MS)
        order = false;
    }
    const double hold_ms = CalSequencer::hold_s(N, 0) * 1000.0;
    CHECK(all && order, "every board tested once, in chain order, one at a time");
    CHECK(r.done_ms > 0 && r.done_ms + 10000 <= hold_ms, "done after %.1f s, the dark hold lasts %.0f s", r.done_ms / 1000.0,
          hold_ms / 1000.0);
    std::printf("  pin calibration of 50 boards: %.1f s (%.2f s per board), dark hold %.0f s\n", r.done_ms / 1000.0,
                r.done_ms / 1000.0 / N, hold_ms / 1000.0);
  }
}

/// All of a board's commands queued at once (5 lights with an 8-step strobe each, every setting, IDENTIFY: 74
/// commands, on each of 50 boards): none is dropped, every one goes out with its copies.
static void test_queue_keeps_all() {
  const uint16_t N = 50;
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  std::vector<Step> steps(ULTRA_MAX_STEPS, Step{100, 0, 30000, {65535, 65535, 65535, 65535, 65535},
                                                {65535, 65535, 65535, 65535, 65535}});
  SchedPeek s;
  s.init(N, 31, 3);
  for (uint16_t b = 0; b < N; b++) {
    for (uint8_t id = 1; id < ULTRA_CFG_COUNT; id++)
      s.set_config(b, id, 1);
    for (uint8_t l = 0; l < ULTRA_LIGHTS; l++) {
      uint8_t roles[5] = {0, 0, 0, 0, 0};
      roles[l] = ULTRA_ROLE_BRIGHTNESS;
      const uint8_t strobe[1] = {ULTRA_MAX_STEPS};
      s.light_def(b, l, roles, 1000, 0, z, one);
      s.light_opt(b, l, 0, 0, 0, 0, 0);
      s.light_set(b, l, 0, 30000, one, one, 100, 0);
      s.light_scene(b, l, 0, 0, 0, z, one);
      s.light_scene_effect(b, l, ULTRA_FX_NONE, nullptr, 0);
      s.light_effect(b, l, ULTRA_FX_STROBE, strobe, 1, 0, &steps);
    }
    s.identify(b, true);
  }
  s.flush_config();
  bool all = true;
  for (uint16_t b = 0; b < N; b++)
    all &= s.queued(b) == 74;
  CHECK(all, "74 commands per board queued, none dropped (board 0: %u)", static_cast<unsigned>(s.queued(0)));
  const size_t queue_bytes = s.queue_capacity() * SchedPeek::cmd_bytes();
  // board 0's slots until the queues are empty (the refresh takes every 16th: counts are at least the copies)
  std::vector<uint8_t> f(s.frame_bytes());
  int light[ULTRA_LIGHTS][ULTRA_TYPES] = {}, board[ULTRA_TYPES] = {}, frames = 0;
  for (; s.pending() && frames < 1000; frames++) {
    s.build_frame(frames * 20000, f.data());
    const uint8_t *sl = slot_of(f, 0), l = sl[1] >> 4;
    if (sl[0] < ULTRA_TYPES)
      (l < ULTRA_LIGHTS ? light[l][sl[0]] : board[sl[0]])++;
  }
  bool copies = board[ULTRA_CONFIG] >= 9 && board[ULTRA_IDENTIFY] == 3;
  for (uint8_t l = 0; l < ULTRA_LIGHTS; l++) {
    for (uint8_t t : {ULTRA_LIGHT_DEF, ULTRA_LIGHT_OPT, ULTRA_SET, ULTRA_EFFECT, ULTRA_SCENE, ULTRA_SCENE_EFFECT})
      copies &= light[l][t] >= 3;
    copies &= light[l][ULTRA_STEP] >= 3 * static_cast<int>(ULTRA_MAX_STEPS);
  }
  CHECK(!s.pending() && copies, "every command went out with its 3 copies (%d frames)", frames);
  std::printf("  74 commands on each of 50 boards: all sent in %d frames; queues %.1f KB at most (%u B per command)\n",
              frames, queue_bytes / 1024.0, static_cast<unsigned>(SchedPeek::cmd_bytes()));
}

/// Five lights of one board streamed faster than the line can send (a strip, ESPHome effects): they take turns, also
/// when the queue's order counter wraps.
static void test_stream_turns() {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
  for (uint32_t first : {0u, 0xFFFFFFFDu}) {
    SchedPeek s;
    s.init(50, 22, 1);
    s.set_order(first);
    std::vector<uint8_t> f(s.frame_bytes());
    int sent[ULTRA_LIGHTS] = {};
    for (int frame = 0; frame < 200; frame++) {
      for (uint8_t l = 0; l < ULTRA_LIGHTS; l++)
        s.light_stream(0, l, static_cast<uint16_t>(10000 + frame), one, one, 20, frame * 20000);
      s.build_frame(frame * 20000, f.data());
      if (f[0] == ULTRA_SET)
        sent[f[1] >> 4]++;
    }
    const int least = *std::min_element(sent, sent + ULTRA_LIGHTS);
    CHECK(least >= 30, "5 streamed lights take turns (order from %08x): the least served got %d of 200 slots",
          static_cast<unsigned>(first), least);
  }
}

/// One data line in test_group_start: its scheduler, frame time, when its next frame starts, and per board and light
/// the start a command got and the end (TIME) of the frame that carried its first copy.
struct GroupLine {
  SchedPeek s;
  uint16_t boards{0}, lights{0};
  uint32_t frame_us{0};   // a frame, its reset and the gap to the next
  int64_t next{0};
  std::vector<uint8_t> f;
  std::vector<int64_t> start, arrived;
  void init(uint16_t n, uint16_t per_board, uint32_t seed, int64_t t0) {
    const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
    this->boards = n;
    this->lights = per_board;
    this->s.init(n, seed, 3);
    this->f.assign(this->s.frame_bytes(), 0);
    for (uint16_t b = 0; b < n; b++) {
      for (uint8_t l = 0; l < per_board; l++) {
        uint8_t roles[5] = {0, 0, 0, 0, 0};
        roles[l] = ULTRA_ROLE_BRIGHTNESS;
        this->s.light_def(b, l, roles, 1000, 0, z, one);
      }
    }
    while (this->s.pending())
      this->s.build_frame(0, this->f.data());
    this->frame_us = static_cast<uint32_t>(this->s.build_idle_frame(0, this->f.data())) + 300 + 500;
    this->next = t0 - 5000 + this->frame_us;   // a refresh frame went out 5 ms before the change
    this->start.assign(static_cast<size_t>(n) * per_board, NO_START);
    this->arrived.assign(static_cast<size_t>(n) * per_board, NO_START);
  }
};

/// One Home Assistant change to every light of two lines (50 boards with 5 lights each, 10 boards with one), its calls
/// arriving over 8 ms: with start_delay auto (start_lead_us, GroupStart, as UltraHub::start_time_us) they all share one
/// start and every first copy is on the wire before it. A fixed 30 ms start delay is shown for comparison.
static int run_group(bool automatic, int64_t &worst_late_us) {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
  const int64_t t0 = 10000000;
  GroupLine a, b;
  a.init(50, 5, 61, t0);
  b.init(10, 1, 62, t0);
  GroupStart group;
  std::set<int64_t> starts;
  // the calls: lights 0 and 1 (and the other line), 4 ms later lights 2 and 3, 4 ms later light 4
  const struct {
    int64_t at;
    uint8_t first, last;
    bool other;
  } calls[] = {{t0, 0, 1, true}, {t0 + 4000, 2, 3, false}, {t0 + 8000, 4, 4, false}};
  size_t call = 0;
  while (call < 3 || a.s.pending() || b.s.pending()) {
    const int64_t ta = a.s.pending() ? a.next : INT64_MAX, tb = b.s.pending() ? b.next : INT64_MAX;
    if (call < 3 && calls[call].at <= std::min(ta, tb)) {
      const auto &c = calls[call++];
      for (GroupLine *line : {&a, &b}) {
        if (line == &b && !c.other)
          continue;
        for (uint16_t bd = 0; bd < line->boards; bd++) {
          for (uint8_t l = c.first; l <= c.last && l < line->lights; l++) {
            int64_t start;
            if (automatic) {
              const int64_t lead = std::max(start_lead_us(a.frame_us, a.lights, a.s.waiting()),
                                            start_lead_us(b.frame_us, b.lights, b.s.waiting()));
              start = group.start(c.at, lead, start_lead_us(line->frame_us, 1, line->s.waiting(bd)) - START_SLACK_US);
            } else {
              start = c.at + 30000;
            }
            starts.insert(start);
            line->start[bd * line->lights + l] = start;
            line->s.light_set(bd, l, 0, 40000, one, one, 1000, start);
          }
        }
        line->next = std::max(line->next, c.at);   // an idle line sends at once
      }
      continue;
    }
    GroupLine &line = ta <= tb ? a : b;
    const int64_t end = line.s.build_frame(line.next, line.f.data());
    for (uint16_t bd = 0; bd < line.boards; bd++) {
      const uint8_t *sl = slot_of(line.f, bd);
      const size_t i = bd * line.lights + (sl[1] >> 4);
      if (sl[0] == ULTRA_SET && line.arrived[i] == NO_START)
        line.arrived[i] = end;
    }
    line.next += line.frame_us;
  }
  int late = 0;
  worst_late_us = INT64_MIN;
  for (GroupLine *line : {&a, &b}) {
    for (size_t i = 0; i < line->start.size(); i++) {
      const int64_t by = line->arrived[i] - line->start[i];
      late += by > 0;
      worst_late_us = std::max(worst_late_us, by);
    }
  }
  return automatic ? (starts.size() == 1 ? late : -1) : late;
}

static void test_group_start() {
  GroupStart g;
  CHECK(g.start(1000, 50000, 20000) == 51000 && g.start(21000, 50000, 20000) == 51000 &&
            g.start(31001, 50000, 20000) == 81001,
        "group start: joined while reachable, a new group after");
  CHECK(start_lead_us(18000, 5, 2) == 18000 * 9 + START_SLACK_US && start_lead_us(18000, 10, 6) == 18000 * 19 + START_SLACK_US,
        "lead: the frame on the wire, the commands and those waiting, a refresh slot per 15");
  int64_t worst_auto = 0, worst_fixed = 0;
  const int late_auto = run_group(true, worst_auto), late_fixed = run_group(false, worst_fixed);
  CHECK(late_auto == 0, "start_delay auto: one start for the whole change, every first copy before it (%d late, by up to "
        "%lld us)", late_auto, static_cast<long long>(worst_auto));
  std::printf("  group change, 50 boards x 5 lights + 10 boards x 1, calls over 8 ms: auto: one start, the last first "
              "copy %.1f ms before it; fixed 30 ms: %d of 260 first copies late (up to %.1f ms)\n",
              -worst_auto / 1000.0, late_fixed, worst_fixed / 1000.0);
}

/// Diagnostics: first copies waiting (start_lead_us), the queue's peak, the line's memory, RMT underruns.
static void test_diagnostics_counts() {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
  SchedPeek s;
  s.init(2, 9, 3);
  std::vector<uint8_t> f(s.frame_bytes());
  for (uint8_t l = 0; l < 3; l++)
    s.light_set(0, l, 0, 1000, one, one, 0, 0);
  s.light_set(1, 0, 0, 1000, one, one, 0, 0);
  s.light_set(0, 0, 0, 2000, one, one, 0, 0);   // replaces an unsent one: still 3 waiting
  CHECK(s.waiting() == 3, "3 first copies waiting on board 0 (%u)", static_cast<unsigned>(s.waiting()));
  s.build_frame(0, f.data());
  CHECK(s.waiting() == 2, "one frame later 2 (%u)", static_cast<unsigned>(s.waiting()));
  CHECK(s.take_queue_peak() == 3 && s.take_queue_peak() == 0, "queue peak 3, then reset");
  const size_t mem = s.take_memory_peak();
  CHECK(mem >= 2 * SchedPeek::board_bytes() + 4 * SchedPeek::light_bytes() + 4 * SchedPeek::cmd_bytes(),
        "memory peak: boards, lights in use and the queue (%u B)", static_cast<unsigned>(mem));
  while (s.pending())
    s.build_frame(0, f.data());
  s.take_memory_peak();   // the queues' peak since the last call
  CHECK(s.waiting() == 0 && s.take_memory_peak() == 2 * SchedPeek::board_bytes() + 4 * SchedPeek::light_bytes(),
        "queues empty and freed: memory is the state only");

  UnderrunWatch w;
  w.drain_us = 250;   // 192 symbols at 1.3 us
  const int64_t fills[] = {0, 120, 245, 370, 495, 900, 1020};   // a 405 us gap after 495
  for (size_t i = 0; i < 7; i++)
    w.fill(fills[i], i == 0);
  w.fill(50000, true);   // the next frame's first fill, after a pause: not an underrun
  w.fill(50125, false);
  CHECK(w.count == 1, "underruns: one refill came too late (%u)", static_cast<unsigned>(w.count));
  UnderrunWatch v;
  v.drain_us = 250;
  v.fill(0, true);
  v.fill(400, false);
  v.fill(800, false);   // two late refills in one frame: one broken frame
  v.fill(100000, true);
  v.fill(100400, false);
  CHECK(v.count == 2, "underruns count broken frames, not late refills (%u)", static_cast<unsigned>(v.count));
}

/// The Diff-Power port's power-off notice (protocol 0.2.6): each copy (its port times them: 0 / 50 / 100 ms) is
/// IDENTIFY 7 POWER_OFF, argument 0, in the board's slot of the next frame, before its queued commands (they follow,
/// nothing lost); the first copy takes a new SEQ, the others repeat it; an IDENTIFY after it has a SEQ of its own.
static const char *const VEC_SLOT_IDENTIFY_POWER_OFF =
    "06f059070000000000000000000000000000000000000000000000000000e97b";

static void test_power_off_notice() {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
  {
    // the protocol's test vector (SEQ 0x59): the scheduler's notice slot, byte for byte
    Scheduler v;
    v.init(1, 1, 3);
    std::vector<uint8_t> fv(v.frame_bytes());
    for (int i = 0; v.pending() && i < 100; i++)
      v.build_frame(i * 20000, fv.data());
    int guard = 0;
    for (; guard < 256; guard++) {   // (the next IDENTIFY SEQ from its seed: step to 0x59)
      v.power_off(0, true);
      v.build_frame(5000000 + guard * 20000, fv.data());
      if (slot_of(fv, 0)[2] == 0x59)
        break;
    }
    CHECK(guard < 256 && from_hex(VEC_SLOT_IDENTIFY_POWER_OFF) ==
                             std::vector<uint8_t>(slot_of(fv, 0), slot_of(fv, 0) + ULTRA_SLOT_BYTES),
          "the notice slot (SEQ 0x59) is the protocol's slot_identify_power_off vector");
  }
  Scheduler s;
  s.init(3, 21, 3);
  std::vector<uint8_t> f(s.frame_bytes());
  for (int i = 0; s.pending() && i < 1000; i++)
    s.build_frame(i * 20000, f.data());
  s.light_set(0, 0, 0, 1234, one, one, 0, 0);   // queued for board 0 before the notice
  s.identify(1, true);                          // board 1: no notice, its IDENTIFY as usual
  const size_t waited = s.waiting(0);
  uint8_t seq[3] = {0, 0, 0};
  for (int k = 0; k < 3; k++) {
    s.power_off(0, k == 0);
    s.power_off(2, k == 0);
    s.power_off(3, k == 0);   // (no such board)
    CHECK(s.pending() && s.power_off_left() == 2 && (k > 0 || s.waiting(0) == waited + 1),
          "copy %d: a frame waiting for it (board 0: %u)", k, static_cast<unsigned>(s.waiting(0)));
    s.build_frame(1000000 + k * 50000, f.data());
    CHECK(frame_ok(f, 3) && s.power_off_left() == 0 && s.take_power_off_sent() && !s.take_power_off_sent(),
          "frame CRCs, the copy out, its frame flagged once (the hub reports it out)");
    for (int b : {0, 2}) {
      const uint8_t *sl = slot_of(f, b);
      CHECK(sl[0] == ULTRA_IDENTIFY && (sl[1] >> 4) == ULTRA_LIGHT_BOARD && (sl[1] & 0x0F) == 0 &&
                sl[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF && sl[ULTRA_PAYLOAD + 1] == 0 && ultra_slot_check(sl) == 0 &&
                (k == 0 || sl[2] == seq[b]),
            "copy %d, board %d: IDENTIFY 7, argument 0, one SEQ (type %u, value %u, seq %u)", k, b, sl[0],
            sl[ULTRA_PAYLOAD], sl[2]);
      seq[b] = sl[2];
    }
    if (k == 0) {
      const uint8_t *b1 = slot_of(f, 1);
      CHECK(b1[0] == ULTRA_IDENTIFY && b1[ULTRA_PAYLOAD] == ULTRA_IDENT_ON, "board 1: its own IDENTIFY, not delayed");
      s.build_frame(1010000, f.data());
      CHECK(slot_of(f, 0)[0] == ULTRA_SET && ultra_get16(slot_of(f, 0) + ULTRA_PAYLOAD) == 1234 &&
                !(slot_of(f, 2)[0] == ULTRA_IDENTIFY && slot_of(f, 2)[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF),
            "between the copies: board 0's queued SET (not lost), no extra copy");
    }
  }
  // a copy queued and not sent before the port woke again: dropped; no frame flagged (negative control)
  s.power_off(0, false);
  s.cancel_power_off();
  s.build_frame(1100000, f.data());
  CHECK(s.power_off_left() == 0 && !s.take_power_off_sent() &&
            !(slot_of(f, 0)[0] == ULTRA_IDENTIFY && slot_of(f, 0)[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF),
        "a cancelled copy: not sent");
  s.identify(2, true);
  s.build_frame(1200000, f.data());
  CHECK(slot_of(f, 2)[0] == ULTRA_IDENTIFY && slot_of(f, 2)[ULTRA_PAYLOAD] == ULTRA_IDENT_ON &&
            slot_of(f, 2)[2] != seq[2],
        "an IDENTIFY after the notice: its own SEQ");
}

#ifdef WITH_ENGINE
/// One frame to the board on slot `slot`, whose clock ticks with the host's (50 us per tick, + tick_offset).
static void feed(ultra_t &u, const std::vector<uint8_t> &f, int slot, int64_t frame_end_us, uint32_t tick_offset = 0) {
  uint32_t hus;
  ultra_parse_header(hdr_of(f), &hus);
  const uint32_t tick = static_cast<uint32_t>(frame_end_us / 50) + tick_offset;
  ultra_on_header(&u, tick, hus);
  ultra_on_slot(&u, slot_of(f, slot), tick);
}

/// Every queued command sent (to boards that miss them).
static void drain(Scheduler &s, std::vector<uint8_t> &f, int64_t start_us) {
  for (int i = 0; s.pending() && i < 1000; i++)
    s.build_frame(start_us + i * 20000, f.data());
}

/// A board replaced (or rebooted) while its light streams every frame: the stream leaves every 16th slot to the
/// refresh, so the new board gets its CONFIG and light definition and shows the stream.
static void test_refresh_under_stream() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  Scheduler s;
  s.init(50, 2, 3);
  s.set_config(0, ULTRA_CFG_PWM_PRESET, 8);
  s.flush_config();
  s.light_def(0, 0, MONO1, 1000, 0, z, one);
  s.light_set(0, 0, 0, 30000, one, one, 0, 0);
  std::vector<uint8_t> f(s.frame_bytes());
  drain(s, f, 0);
  ultra_t u;
  ultra_init(&u, 3);   // the new board
  int64_t end = 0;
  for (int i = 0; i < 256; i++) {
    const int64_t now = 10000000 + i * 20000;
    s.light_stream(0, 0, static_cast<uint16_t>(30000 + i), one, one, 20, now);
    end = s.build_frame(now, f.data());
    feed(u, f, 0, end);
  }
  uint16_t out[5];
  ultra_render(&u, static_cast<uint32_t>(end / 50) + 40, out);
  CHECK(u.light[0].defined && (u.cfg_valid & (1u << ULTRA_CFG_PWM_PRESET)) && out[4] > 29000,
        "a board new during a stream gets its CONFIG and definition (defined %u, output %u)", u.light[0].defined, out[4]);
}

/// A long transition refreshed to a board that missed it (a 120 s fade; the board joins after 40 s, after 300 s):
/// past the SET's 32.8 s start offset it still ends on time, and an expired one shows its target at once.
static void test_old_transition() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  for (int late_s : {40, 300}) {
    Scheduler s;
    s.init(1, 4, 1);
    s.light_def(0, 0, MONO1, 1000, 0, z, one);
    s.light_set(0, 0, 0, 40000, one, one, 120000, 0);   // from host time 0 to 120 s
    std::vector<uint8_t> f(s.frame_bytes());
    drain(s, f, 0);
    ultra_t u;
    ultra_init(&u, 5);
    int64_t end = 0;
    for (int i = 0; i < 6; i++) {   // the refresh: DEF, SET, ...
      end = s.build_frame(late_s * 1000000LL + i * 1000, f.data());
      feed(u, f, 0, end);
    }
    uint16_t before[5], after[5];
    if (late_s == 40) {
      ultra_render(&u, 119000000 / 50, before);
      ultra_render(&u, 121000000 / 50, after);
      CHECK(before[4] < 40000 && after[4] == 40000, "joined after 40 s: the fade ends at 120 s (%u at 119 s, %u at 121 s)",
            before[4], after[4]);
    } else {
      ultra_render(&u, static_cast<uint32_t>(end / 50) + 20, after);
      CHECK(after[4] == 40000, "joined after 300 s: the target at once (%u)", after[4]);
    }
  }
}

/// An effect without a period (a candle) left on for 25 days, a board that powers up then: the refresh's EFFECT start
/// offset (int32 ms) stays far inside its range and the board runs the candle. (Clipped at -2^31 ms, the board took
/// the start as one still to come and never started the effect: the plain state showed.)
/// Turning an effect light off (a breathe held at 25 % over its 100 % state): its EFFECT none and its fade-out at one
/// start fade from what the effect shows; an EFFECT none that starts 90 ms before its fade-out shows the light's own
/// state, full, in between (what the hub's group budget, max_commands_, prevents: lifecycle test_group_budget).
static void test_effect_stop_fade() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  uint16_t peak[2] = {};
  const int64_t gaps[2] = {0, 90000};
  for (int i = 0; i < 2; i++) {
    Scheduler s;
    s.init(1, 23, 1);
    s.light_def(0, 0, MONO1, 1000, 0, z, one);
    s.light_set(0, 0, 0, 65535, one, one, 0, 0);
    uint8_t breathe[7] = {};
    ultra_put24(breathe, 4000);
    ultra_put16(breathe + 3, 16384);   // min = max: a steady 25 %
    ultra_put16(breathe + 5, 16384);
    s.light_effect(0, 0, ULTRA_FX_BREATHE, breathe, 7, 0);
    ultra_t u;
    ultra_init(&u, 24);
    std::vector<uint8_t> f(s.frame_bytes());
    for (int k = 0; k < 6; k++)
      feed(u, f, 0, s.build_frame(100000 + k * 20000, f.data()));
    const int64_t at = 1100000;   // the change's start, 100 ms after the call
    s.light_effect(0, 0, ULTRA_FX_NONE, nullptr, 0, at);
    s.light_set(0, 0, ULTRA_SET_SMOOTH, 0, one, one, 1000, at + gaps[i]);
    // a frame every 20 ms, taken by the board at its end; rendered every ms
    int64_t start = 1000000, end = s.build_frame(start, f.data());
    for (int64_t t = 1000000; t < 2500000; t += 1000) {
      if (t >= end) {
        feed(u, f, 0, end);
        start += 20000;
        end = s.build_frame(start, f.data());
      }
      uint16_t out[5];
      ultra_render(&u, static_cast<uint32_t>(t / 50), out);
      peak[i] = std::max(peak[i], out[4]);
    }
  }
  CHECK(peak[0] <= 17000, "an effect light turned off: from 25 %% down, no flash (peak %u)", peak[0]);
  std::printf("  effect light off: peak %u with its EFFECT none and fade-out at one start, %u with the fade 90 ms "
              "later\n", peak[0], peak[1]);
}

static void test_old_effect() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  Scheduler s;
  s.init(1, 4, 1);
  s.light_def(0, 0, MONO1, 1000, 0, z, one);
  s.light_set(0, 0, 0, 40000, one, one, 0, 0);
  uint8_t candle[4];
  ultra_put16(candle, 30000);
  ultra_put16(candle + 2, 100);
  s.light_effect(0, 0, ULTRA_FX_CANDLE, candle, 4, 0);
  std::vector<uint8_t> f(s.frame_bytes());
  drain(s, f, 0);
  ultra_t u;
  ultra_init(&u, 6);
  const int64_t day25 = 25LL * 86400 * 1000000;
  int64_t end = 0, offset = 0;
  for (int i = 0; i < 6; i++) {   // the refresh: DEF, SET, EFFECT, ...
    end = s.build_frame(day25 + i * 1000, f.data());
    if (f[0] == ULTRA_EFFECT)
      offset = static_cast<int32_t>(ultra_get32(f.data() + ULTRA_PAYLOAD + 1));
    feed(u, f, 0, end);
  }
  uint16_t out[5], lo = 65535, hi = 0;
  for (uint32_t ms = 0; ms < 2000; ms++) {
    ultra_render(&u, static_cast<uint32_t>(end / 50) + 20 * ms, out);
    lo = std::min(lo, out[4]);
    hi = std::max(hi, out[4]);
  }
  CHECK(offset > -2LL * 86400000 && u.light[0].fx == ULTRA_FX_CANDLE && hi > lo,
        "a candle left on for 25 days: a board powered up then runs it (start offset %lld ms, effect %u, %u..%u)",
        static_cast<long long>(offset), u.light[0].fx, lo, hi);
}

/// A light removed from the YAML (index 4 of a board that stayed powered while the ESP rebooted): the new hub clears
/// the index, also by the refresh alone (the board missed the queued copies): no output, no scene, no effect.
static void test_removed_light() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  const uint8_t CH1[5] = {ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0};
  ultra_t u;
  ultra_init(&u, 10);
  {
    Scheduler old;   // the earlier configuration: light 4 on CH1, on, in the power-on scene with a breathe
    old.init(1, 6, 1);
    old.light_def(0, 4, CH1, 1000, 0, z, one);
    old.light_set(0, 4, 0, 40000, one, one, 0, 0);
    old.light_scene(0, 4, 1, 0, 40000, one, one);
    uint8_t breathe[7] = {};
    ultra_put24(breathe, 4000);
    old.light_scene_effect(0, 4, ULTRA_FX_BREATHE, breathe, 3);
    std::vector<uint8_t> f(old.frame_bytes());
    for (int i = 0; i < 8; i++)
      feed(u, f, 0, old.build_frame(100000 + i * 1000, f.data()));
  }
  CHECK(u.light[4].mask == 1 && u.light[4].scene_mode == 1 && u.light[4].scene_fx == ULTRA_FX_BREATHE,
        "light 4 set up by the earlier configuration");
  Scheduler next;   // the new configuration: one light (index 0) on this board
  next.init(1, 7, 1);
  next.clear_unused_lights(0, 1);
  std::vector<uint8_t> f(next.frame_bytes());
  drain(next, f, 500000);
  int64_t end = 0;
  for (int i = 0; i < 50; i++) {
    end = next.build_frame(1000000 + i * 1000, f.data());
    feed(u, f, 0, end, 200000);   // the board's clock ran on, the host's restarted
  }
  uint16_t out[5];
  ultra_render(&u, static_cast<uint32_t>(end / 50) + 200000 + 20, out);
  CHECK(out[0] == 0 && u.light[4].mask == 0 && u.light[4].scene_mode == 0 && u.light[4].scene_fx == ULTRA_FX_NONE,
        "removed light 4 cleared by the refresh (output %u, mask %u, scene %u, effect %u)", out[0], u.light[4].mask,
        u.light[4].scene_mode, u.light[4].scene_fx);
}

/// signal_loss off / fade_off: a board switches its lights off by itself (ultra_all_off, as its main loop does) and
/// keeps its state otherwise; once the frames are back, the unchanged refresh brings back the light and its effect.
/// Board firmware without the fix keeps the SEQs it applied and ignores that refresh: reported, not failed.
static void test_signal_loss_recovery() {
  const uint16_t z[5] = {0, 0, 0, 0, 0}, one[5] = {65535, 65535, 65535, 65535, 65535};
  for (uint32_t fade_ms : {0u, 500u}) {
    Scheduler s;
    s.init(1, 18, 1);
    s.light_def(0, 0, MONO1, 1000, 0, z, one);
    s.light_set(0, 0, 0, 40000, one, one, 0, 0);
    uint8_t breathe[7] = {};
    ultra_put24(breathe, 4000);
    ultra_put16(breathe + 3, 10000);
    ultra_put16(breathe + 5, 65535);
    s.light_effect(0, 0, ULTRA_FX_BREATHE, breathe, 7, 0);
    ultra_t u;
    ultra_init(&u, 19);
    std::vector<uint8_t> f(s.frame_bytes());
    for (int i = 0; i < 12; i++)
      feed(u, f, 0, s.build_frame(100000 + i * 1000, f.data()));
    const uint32_t lost = 2400;   // tick: the data line went quiet for signal_loss_timeout
    uint16_t out[5];
    ultra_all_off(&u, lost, fade_ms);
    ultra_render(&u, lost + fade_ms * 20 + 100, out);
    CHECK(out[4] == 0, "signal loss (fade %u ms): the light goes off (%u)", fade_ms, out[4]);
    if (u.last_seq[0][ULTRA_SET] != 0x100u) {
      if (fade_ms == 0)
        std::printf("  signal loss: this board firmware keeps its SEQs after switching off; the unchanged refresh "
                    "can't bring the light back (it stays off until the next change)\n");
      continue;
    }
    for (int i = 0; i < 12; i++)
      feed(u, f, 0, s.build_frame(1000000 + i * 1000, f.data()));
    ultra_render(&u, 20300, out);
    CHECK(out[4] > 0 && u.light[0].fx == ULTRA_FX_BREATHE,
          "signal loss (fade %u ms): the refresh brings back the light and its effect (%u, effect %u)", fade_ms,
          out[4], u.light[0].fx);
    if (fade_ms == 0)
      std::printf("  signal loss off / fade off: the unchanged refresh brings the light and its effect back\n");
  }
}
#endif

/// wire_ns counts the 1 bits four bytes at a time: the same as one bit at a time (the reference), for every length up
/// to a 50-board frame (1,608 bytes) at densities from all-zero to all-one, with two bit timings; and its cost (host).
static void test_wire_ns() {
  auto reference = [](const uint8_t *b, size_t n, uint32_t t0h, uint32_t t0l, uint32_t t1h, uint32_t t1l) -> uint64_t {
    if (n == 0)
      return 0;
    uint64_t ones = 0;
    for (size_t i = 0; i < n; i++)
      for (int k = 0; k < 8; k++)
        ones += (b[i] >> k) & 1;
    const uint64_t all = ones * (t1h + t1l) + (n * 8 - ones) * (t0h + t0l);
    return all - ((b[n - 1] & 1U) ? t1l : t0l);
  };
  std::mt19937 rng(7);
  std::vector<uint8_t> buf(1608);
  const uint32_t timings[2][4] = {{250, 1050, 700, 650}, {300, 950, 800, 450}};
  int wrong = 0, cases = 0;
  for (const auto &tm : timings) {
    Scheduler s;
    s.set_bit_timing(tm[0], tm[1], tm[2], tm[3]);
    for (size_t n = 0; n <= buf.size(); n += (n < 64 ? 1 : 37)) {
      for (int density : {0, 1, 4, 8, 12, 15, 16}) {   // (sixteenths of the bits set)
        for (auto &v : buf) {
          uint8_t x = 0;
          for (int k = 0; k < 8; k++)
            x |= static_cast<uint8_t>((static_cast<int>(rng() % 16) < density) << k);
          v = x;
        }
        const size_t off = n % 4;   // (unaligned starts too)
        const size_t len = std::min(n, buf.size() - off);
        cases++;
        wrong += s.wire_ns(buf.data() + off, len) != reference(buf.data() + off, len, tm[0], tm[1], tm[2], tm[3]);
      }
    }
  }
  CHECK(wrong == 0, "wire_ns = the bit-by-bit count in %d of %d cases", cases - wrong, cases);
  // the cost of a 50-board frame (host): typical slot data
  for (auto &v : buf)
    v = static_cast<uint8_t>(rng());
  Scheduler s;
  volatile uint64_t sink = 0;
  auto ns = [&](bool swar) {
    const auto a = std::chrono::steady_clock::now();
    for (int i = 0; i < 20000; i++) {
      if (swar) {
        sink = sink + s.wire_ns(buf.data(), buf.size());
      } else {
        uint32_t ones = 0;   // (the loop before: one bit at a time)
        for (size_t j = 0; j < buf.size(); j++)
          for (uint8_t v = buf[j]; v != 0; v = static_cast<uint8_t>(v & (v - 1)))
            ones++;
        sink = sink + ones;
      }
    }
    return std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - a).count() / 20000;
  };
  const double before = ns(false), now = ns(true);
  std::printf("  wire_ns of a 50-board frame (host): %.0f ns, the bit loop before %.0f ns\n", now, before);
}

int main() {
  test_wire_ns();
  test_refresh_rotation();
  test_mapping();
  test_scheduler();
  test_defined_values();
  test_encoder();
  test_switch_frames();
  test_cal_sequencer();
  test_fifty_boards();
  test_scene_effect();
  test_queue_keeps_all();
  test_stream_turns();
  test_group_start();
  test_diagnostics_counts();
  test_power_off_notice();
#ifdef WITH_ENGINE
  test_config_frames();
  test_scene_path();
  test_dtw_path();
  test_dtw_points_path();
  test_chain();
  test_refresh_under_stream();
  test_old_transition();
  test_old_effect();
  test_effect_stop_fade();
  test_removed_light();
  test_signal_loss_recovery();
#else
  std::printf("  (board firmware not found: chain test with the board engine skipped)\n");
#endif
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA HOST TESTS PASSED", failures);
  return failures ? 1 : 0;
}
