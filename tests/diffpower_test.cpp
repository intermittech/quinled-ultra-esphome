// Host test of the Diff-Power port power manager's core (components/diffpower/diffpower_core.h) with simulated time,
// after review-r8/fable-standby §6 (ESPHome): the standby cycle (the notice's copies at 0 / 50 / 100 ms, the enable low
// at 200, OFF_HOLD, STANDBY); a request during the notice (no take-back, the wake after min_off_time, the latency);
// detect flapping and lost (FAULT, its return); the dark window (midnight, noon, 78 degrees north in June and December,
// the offsets exact, no valid time); Always on at boot and a mode change (its turn-off period); pre-wakes and the
// the min_on_time floor; the turn-off period per light; a port without a board; the counters; a controller shutdown; the
// boot; millis() wrapping. Then review-r8/portmgr-gpt's failures as regressions: a plug across the millis() wrap, a
// detect dropout shorter than the debounce (waking and on), the notice's copies timed from their delivery (a slow
// loop, a slow or dead transmitter), a pre-wake masked for 25 days, dark offsets up to 12 h (against a brute-force
// search of the elevation), a shutdown during a notice, a deliberate power cycle. Negative controls alongside: what
// must NOT wake or drop the port.
//
// Build and run: python tests/run_diffpower_test.py

#include "../components/diffpower/diffpower_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace esphome::diffpower;

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      failures++;                                         \
      std::printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

static constexpr uint32_t MIN = 60000;
static constexpr uint32_t NEVER = UINT32_MAX;

/// The port with its lights and its detect input, the time in dt steps (10 ms); the events seen.
struct Sim {
  PortCore p;
  uint32_t now{1000}, dt{10};
  bool detect{true};
  std::vector<bool> lit;
  uint32_t ready{0}, lost{0}, states{0};
  uint32_t ready_at{0};
  std::vector<uint32_t> copy_at;      // each notice copy's time
  std::vector<uint8_t> copy_index;    // ... and its index
  std::vector<uint32_t> enable_at;    // each enable change's time
  std::vector<bool> enable_to;        // ... and its level
  uint32_t deliver_after{0};          // the hub: a copy goes out this long after it was asked for (NEVER: never)
  bool asked{false};
  uint32_t asked_at{0};
  std::vector<uint32_t> out_at;       // each copy's delivery
  explicit Sim(int lights, uint32_t hold0 = 0) {
    PortConfig c;   // (the defaults: hotplug 1.5 s, boot margin 200 ms, notice 200 ms, min off 1 s, min on 2 min)
    c.standby_after_ms = 10 * MIN;
    p.configure(c);
    for (int i = 0; i < lights; i++) {
      p.add_load(i == 0 ? hold0 : 0);
      lit.push_back(false);
    }
  }
  void step() {
    p.set_detect(detect, now);
    for (size_t i = 0; i < lit.size(); i++)
      p.set_lit(static_cast<uint16_t>(i), lit[i], now);
    if (asked && deliver_after != NEVER && now - asked_at >= deliver_after) {
      p.notice_sent(now);
      out_at.push_back(now);
      asked = false;
    }
    const uint8_t ev = p.update(now);
    if (ev & EV_NOTICE) {   // (the hub sends it: at once, or deliver_after later)
      copy_at.push_back(now);
      copy_index.push_back(p.notice_copy());
      asked = true;
      asked_at = now;
      if (deliver_after == 0) {
        p.notice_sent(now);
        out_at.push_back(now);
        asked = false;
      }
    }
    if (ev & EV_READY) {
      ready++;
      ready_at = now;
    }
    if (ev & EV_LOST)
      lost++;
    if (ev & EV_STATE)
      states++;
    if (ev & EV_ENABLE) {
      enable_at.push_back(now);
      enable_to.push_back(p.enable());
    }
    now += dt;
  }
  void run(uint32_t ms) {
    for (uint32_t end = now + ms; static_cast<int32_t>(now - end) < 0;)
      step();
  }
  template<typename F> bool run_until(uint32_t ms, F cond) {
    for (uint32_t end = now + ms; static_cast<int32_t>(now - end) < 0;) {
      step();
      if (cond())
        return true;
    }
    return false;
  }
  uint32_t last_enable_at() const { return enable_at.empty() ? 0 : enable_at.back(); }
};

static std::string text(const PortCore &p) {
  char b[32];
  return port_state_text(p.state(), p.drops(), b, sizeof(b));
}

/// UTC seconds since 1970 of a civil date and time
static double unix_at(int y, int m, int d, int hh, int mm) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = y - era * 400;
  const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const double days = era * 146097.0 + doe - 719468;
  return days * 86400.0 + hh * 3600.0 + mm * 60.0;
}

// ---------------------------------------------------------------------------------------------------------------
static void test_cycle() {
  std::printf("the standby cycle: wake, on, the notice at 0 / 50 / 100 ms, the enable low at 200, off-hold, standby\n");
  Sim s(1);
  s.run(1000);
  CHECK(s.p.state() == PS_STANDBY && !s.p.enable() && text(s.p) == "Standby", "at rest: Standby, enable low");
  const uint32_t t0 = s.now;
  s.lit[0] = true;
  s.step();
  CHECK(s.p.state() == PS_WAKING && s.p.enable() && text(s.p) == "Waking", "a light on: Waking, enable high");
  CHECK(s.run_until(5000, [&] { return s.p.ready(); }) && s.ready_at - t0 >= 1700 && s.ready_at - t0 <= 1710,
        "ready (the hub's frames) after hotplug 1.5 s + boot margin 200 ms (%u ms)", s.ready_at - t0);
  s.run(3000);
  CHECK(s.ready == 1 && text(s.p) == "On" && s.p.wakes() == 1, "on, one ready");
  s.lit[0] = false;
  const uint32_t off = s.now;
  CHECK(s.run_until(11 * MIN, [&] { return !s.copy_at.empty(); }), "the notice");
  const uint32_t n0 = s.copy_at[0];
  CHECK(n0 - off >= 10 * MIN && n0 - off <= 10 * MIN + 20 && s.p.state() == PS_NOTICE && s.p.enable() &&
            text(s.p) == "Powering off",
        "the notice when the turn-off period ran out (%u ms), the enable still high", n0 - off);
  s.run(2000);
  CHECK(s.copy_at.size() == 3 && s.copy_at[1] - n0 == 50 && s.copy_at[2] - n0 == 100 && s.copy_index[0] == 0 &&
            s.copy_index[1] == 1 && s.copy_index[2] == 2,
        "3 copies at 0 / 50 / 100 ms (%u)", static_cast<unsigned>(s.copy_at.size()));
  CHECK(s.enable_at.size() == 2 && !s.enable_to[1] && s.enable_at[1] - n0 == 200, "the enable low at 200 ms (%u)",
        s.enable_at[1] - n0);
  CHECK(s.p.state() == PS_STANDBY && s.p.standby_cycles() == 1, "after min_off_time: Standby, a standby cycle");
  s.run(60000);
  CHECK(s.p.standby_s() >= 61 && s.p.standby_s() <= 63, "seconds in standby counted (%u)",
        static_cast<unsigned>(s.p.standby_s()));
  // negative control: a light switched within its turn-off period never drops the power
  Sim n(1);
  n.lit[0] = true;
  n.run(3000);
  for (int i = 0; i < 5; i++) {
    n.lit[0] = false;
    n.run(5 * MIN);
    n.lit[0] = true;
    n.run(1000);
  }
  CHECK(n.copy_at.empty() && n.p.state() == PS_ON && n.enable_at.size() == 1, "off and on within the period: on");
}

static void test_request_during_notice() {
  std::printf("a request during the notice: no take-back, the wake after min_off_time\n");
  Sim s(2);
  s.lit[0] = true;
  s.run(3000);
  s.lit[0] = false;
  CHECK(s.run_until(11 * MIN, [&] { return !s.copy_at.empty(); }), "the notice");
  const uint32_t n0 = s.copy_at[0];
  s.run(40);
  const uint32_t req = s.now;   // NOTICE + 50
  s.lit[1] = true;
  bool high_in_hold = false;
  uint32_t low_at = 0;
  while (!s.p.ready() && s.now - n0 < 10000) {
    s.step();
    if (!s.p.enable() && low_at == 0)
      low_at = s.now - s.dt;
    if (low_at != 0 && s.now - s.dt - low_at < 1000 && s.p.enable())
      high_in_hold = true;
  }
  CHECK(s.copy_at.size() == 3 && low_at - n0 == 200, "no take-back: all 3 copies, the enable low at 200 ms (%u)",
        low_at - n0);
  CHECK(!high_in_hold, "the enable never high within min_off_time (negative control)");
  CHECK(s.p.ready() && s.ready_at - req <= 3300 && s.ready_at - n0 == 200 + 1000 + 1700,
        "then the wake: ready %u ms after the request (<= 3.3 s)", s.ready_at - req);
  CHECK(s.p.standby_cycles() == 1, "the planned power-off counted");
}

static void test_detect() {
  std::printf("detect: flapping in standby, lost while on (FAULT), its return\n");
  Sim s(1);
  s.dt = 1;
  s.run(500);
  const uint32_t states = s.states;
  for (int i = 0; i < 40; i++) {   // 5 ms pulses every 50 ms
    s.detect = false;
    s.run(5);
    s.detect = true;
    s.run(45);
  }
  CHECK(s.p.state() == PS_STANDBY && s.states == states && !s.p.enable(), "5 ms pulses in standby: no state change");
  s.dt = 10;
  s.lit[0] = true;
  s.run_until(5000, [&] { return s.p.ready(); });
  s.run(1000);
  // the connection lost while on
  s.detect = false;
  s.run(500);
  CHECK(s.p.state() == PS_FAULT && s.p.problem() && s.lost == 1 && s.p.drops() == 1 && s.p.enable() &&
            text(s.p) == "Connection lost (1 drop)",
        "detect lost while on: FAULT, a problem, a drop counted, '%s'", text(s.p).c_str());
  s.run(30000);
  CHECK(s.p.state() == PS_FAULT && s.lost == 1, "stays FAULT, counted once");
  s.detect = true;
  const uint32_t back = s.now;
  s.run(200);
  CHECK(s.p.state() == PS_WAKING && !s.p.problem(), "its return: Waking, the problem over");
  s.run_until(5000, [&] { return s.p.ready(); });
  CHECK(s.ready == 2 && s.ready_at - back >= 1700 && s.ready_at - back <= 1710,
        "ready again 1.7 s after its detect (the power gate: not the debounce): everything sent again (%u ms)",
        s.ready_at - back);
  // lost while on, then nothing wants it: the enable low; its return: Standby
  s.detect = false;
  s.run(500);
  s.lit[0] = false;
  s.run(11 * MIN);
  CHECK(s.p.state() == PS_FAULT && !s.p.enable() && s.copy_at.empty() && s.p.drops() == 2 &&
            text(s.p) == "Connection lost (2 drops)",
        "lost and then not wanted: the enable low, no notice (nothing powered to tell)");
  s.detect = true;
  s.run(500);
  CHECK(s.p.state() == PS_STANDBY && !s.p.problem() && !s.p.enable(), "its return: Standby");
}

static void test_no_board() {
  std::printf("a port without a board\n");
  Sim s(1);
  s.detect = false;
  s.run(1000);
  CHECK(s.p.state() == PS_NO_BOARD && !s.p.enable() && !s.p.problem() && text(s.p) == "No board",
        "no board, nothing wanted: 'No board', enable low, no problem");
  s.lit[0] = true;
  s.run(5000);
  CHECK(s.p.state() == PS_NO_BOARD && s.p.enable() && s.p.problem() && s.ready == 0,
        "a light on: the enable high (a board plugged in gets its power), a problem, never ready");
  s.detect = true;
  const uint32_t plug = s.now;
  s.run_until(5000, [&] { return s.p.ready(); });
  CHECK(s.p.ready() && !s.p.problem() && s.ready_at - plug >= 1700 && s.ready_at - plug <= 1710,
        "plugged in: ready hotplug + boot margin after its detect (%u ms)", s.ready_at - plug);
  Sim n(1);
  n.detect = false;
  n.lit[0] = true;
  n.run(3000);
  n.lit[0] = false;
  n.run(11 * MIN);
  CHECK(n.p.state() == PS_NO_BOARD && !n.p.enable() && n.copy_at.empty() && n.p.standby_cycles() == 0 &&
            !n.p.problem(),
        "no board, the light off: the enable low, no notice, no cycle, no problem");
}

static void test_dark_window() {
  std::printf("During dark: the sun's elevation at two shifted times\n");
  const double lat = 52.1, lon = 5.1;   // (Utrecht)
  const uint32_t hour = 3600;
  // 2026-10-15 (CEST = UTC+2): 23:30 and 00:30 dark, noon light
  CHECK(dark_window(unix_at(2026, 10, 15, 21, 30), lat, lon, hour, hour), "23:30 dark");
  CHECK(dark_window(unix_at(2026, 10, 15, 22, 30), lat, lon, hour, hour), "00:30 dark");
  CHECK(!dark_window(unix_at(2026, 10, 15, 10, 0), lat, lon, hour, hour), "noon light");
  // the sunset (the -0.83 degree crossing), a known value: ~16:47 UTC in Utrecht that day
  double lo = unix_at(2026, 10, 15, 14, 0), hi = unix_at(2026, 10, 15, 20, 0);
  for (int i = 0; i < 40; i++) {
    const double mid = (lo + hi) / 2;
    (solar_elevation(mid, lat, lon) > SUN_HORIZON ? lo : hi) = mid;
  }
  const double sunset = lo;
  const double sunset_min = (sunset - unix_at(2026, 10, 15, 0, 0)) / 60.0;
  CHECK(sunset_min > 16 * 60 + 35 && sunset_min < 16 * 60 + 59, "sunset at %.0f:%02.0f UTC", sunset_min / 60 - 0.5,
        sunset_min - 60 * static_cast<int>(sunset_min / 60));
  // the offsets exact: dark from 45 min before sunset (a minute either side)
  CHECK(!dark_window(sunset - 45 * 60 - 60, lat, lon, 45 * 60, 30 * 60) &&
            dark_window(sunset - 45 * 60 + 60, lat, lon, 45 * 60, 30 * 60),
        "dark from 45 min before sunset");
  lo = unix_at(2026, 10, 16, 3, 0);
  hi = unix_at(2026, 10, 16, 9, 0);
  for (int i = 0; i < 40; i++) {
    const double mid = (lo + hi) / 2;
    (solar_elevation(mid, lat, lon) > SUN_HORIZON ? hi : lo) = mid;
  }
  const double sunrise = hi;
  CHECK(dark_window(sunrise + 30 * 60 - 60, lat, lon, 45 * 60, 30 * 60) &&
            !dark_window(sunrise + 30 * 60 + 60, lat, lon, 45 * 60, 30 * 60),
        "... to 30 min after sunrise");
  // 78 degrees north (Longyearbyen): June never dark, December always dark
  bool june = false, december = true;
  for (int h = 0; h < 24; h++) {
    june |= dark_window(unix_at(2026, 6, 21, h, 0), 78.2, 15.6, hour, hour);
    december &= dark_window(unix_at(2026, 12, 21, h, 0), 78.2, 15.6, hour, hour);
  }
  CHECK(!june && december, "78 N: June never dark, December always dark");
  // the port: During dark follows it; no valid time: the fallback (Always on, or Standby)
  Sim s(1);
  s.p.set_mode(MODE_DURING_DARK);
  s.p.set_dark(false, false);
  s.run(3000);
  CHECK(s.p.ready(), "During dark, no valid time: on (fallback Always on)");
  Sim t(1);
  PortConfig c = t.p.config();
  c.unknown_on = false;
  t.p.configure(c);
  t.p.set_mode(MODE_DURING_DARK);
  t.p.set_dark(false, false);
  t.run(3000);
  CHECK(t.p.state() == PS_STANDBY && t.enable_at.empty(), "... fallback Standby: stays in standby");
  t.p.set_dark(true);
  t.run(3000);
  CHECK(t.p.ready(), "in the dark window: on");
}

static void test_modes() {
  std::printf("Always on at boot; a mode change (its turn-off period)\n");
  Sim s(1);
  s.p.set_mode(MODE_ALWAYS_ON);
  s.step();
  CHECK(s.p.state() == PS_WAKING && s.p.enable(), "Always on at boot: Waking at once, no light lit");
  s.run(2 * MIN);
  s.p.set_mode(MODE_STANDBY);
  const uint32_t change = s.now;
  CHECK(s.run_until(11 * MIN, [&] { return !s.copy_at.empty(); }), "the notice");
  CHECK(s.copy_at[0] - change >= 10 * MIN && s.copy_at[0] - change <= 10 * MIN + 20,
        "Standby at noon, the lights off: the notice after standby_after (%u s)", (s.copy_at[0] - change) / 1000);
  // the dark window ends: the same
  Sim d(1);
  d.p.set_mode(MODE_DURING_DARK);
  d.p.set_dark(true);
  d.run(3 * MIN);
  d.p.set_dark(false);
  const uint32_t end = d.now;
  d.run_until(11 * MIN, [&] { return !d.copy_at.empty(); });
  CHECK(!d.copy_at.empty() && d.copy_at[0] - end >= 10 * MIN && d.copy_at[0] - end <= 10 * MIN + 20,
        "the dark window over: the notice after standby_after");
}

static void test_prewake() {
  std::printf("pre-wake; the min_on_time floor\n");
  Sim s(1);
  s.run(1000);
  const uint32_t t0 = s.now;
  s.p.prewake(s.now, 30000);   // 30 s: floored to min_on_time (2 min)
  s.run_until(5000, [&] { return s.p.ready(); });
  const uint32_t on = s.ready_at;
  CHECK(s.run_until(5 * MIN, [&] { return !s.copy_at.empty(); }), "the notice");
  CHECK(s.copy_at[0] - on >= 2 * MIN && s.copy_at[0] - on <= 2 * MIN + 20 && s.copy_at[0] - t0 >= 2 * MIN,
        "a 30 s pre-wake with no light: on for min_on_time, then the notice (%u s)", (s.copy_at[0] - t0) / 1000);
  // a pre-wake, then a light on for 10 s: its turn-off period
  Sim l(1);
  l.p.prewake(l.now, 2 * MIN);
  l.run(20000);
  l.lit[0] = true;
  l.run(10000);
  l.lit[0] = false;
  const uint32_t off = l.now;
  l.run_until(15 * MIN, [&] { return !l.copy_at.empty(); });
  CHECK(!l.copy_at.empty() && l.copy_at[0] - off >= 10 * MIN && l.copy_at[0] - off <= 10 * MIN + 20,
        "a pre-wake then a light: on through the light's turn-off period");
  // a light with a 20 s turn-off period: floored to min_on_time
  Sim f(1, 20000);
  f.lit[0] = true;
  f.run(3 * MIN);
  f.lit[0] = false;
  const uint32_t foff = f.now;
  f.run_until(5 * MIN, [&] { return !f.copy_at.empty(); });
  CHECK(!f.copy_at.empty() && f.copy_at[0] - foff >= 2 * MIN && f.copy_at[0] - foff <= 2 * MIN + 20,
        "a 20 s turn-off period: 2 min (%u s)", (f.copy_at.empty() ? 0 : f.copy_at[0] - foff) / 1000);
}

static void test_per_light() {
  std::printf("the turn-off period per light: the port sleeps at the longest\n");
  Sim s(2, 30 * MIN);
  s.lit[0] = s.lit[1] = true;
  s.run(3000);
  s.lit[0] = s.lit[1] = false;
  const uint32_t off = s.now;
  s.run(10 * MIN + 1000);
  CHECK(s.p.state() == PS_ON && s.copy_at.empty(), "light 1's 10 min over, light 0's 30 min not: on");
  s.run_until(25 * MIN, [&] { return !s.copy_at.empty(); });
  CHECK(!s.copy_at.empty() && s.copy_at[0] - off >= 30 * MIN && s.copy_at[0] - off <= 30 * MIN + 20,
        "the notice after the longest (%u s)", (s.copy_at.empty() ? 0 : s.copy_at[0] - off) / 1000);
}

static void test_counters() {
  std::printf("the counters: restored, counted on\n");
  Sim s(1);
  s.p.set_counters(41, 3600, 2);
  s.run(1000);
  s.lit[0] = true;
  s.run(3000);
  s.lit[0] = false;
  s.run(10 * MIN + 2000);
  CHECK(s.p.standby_cycles() == 42 && s.p.drops() == 2 && s.p.standby_s() >= 3600, "42 cycles, 2 drops kept");
}

static void test_shutdown_and_boot() {
  std::printf("a controller shutdown (OTA); the boot\n");
  Sim s(1);
  s.lit[0] = true;
  s.run(3000);
  const uint32_t t0 = s.now;
  const uint8_t ev = s.p.shutdown(t0);
  CHECK((ev & EV_NOTICE) && s.p.notice_copy() == 0 && s.p.state() == PS_NOTICE && s.p.enable(),
        "on: NOTICE, the first copy, the enable high");
  s.p.notice_sent(t0);
  int copies = 1;
  for (uint32_t t = t0 + 1; t < t0 + 200; t++) {
    if (s.p.update(t) & EV_NOTICE) {
      copies++;
      s.p.notice_sent(t);
    }
  }
  CHECK(copies == 3 && s.p.enable(), "the copies at 50 and 100 ms, the enable still high");
  const uint8_t evd = s.p.power_down(t0 + 199);
  CHECK((evd & EV_ENABLE) && !s.p.enable() && s.p.state() == PS_OFF && s.p.standby_cycles() == 1,
        "power_down: the enable low, Off, a planned power-off counted");
  s.p.set_lit(0, true, t0 + 500);
  CHECK(s.p.update(t0 + 500) == 0 && !s.p.enable(), "shut down: a light doesn't wake it");
  Sim t(1);
  t.run(1000);
  const uint8_t ev2 = t.p.shutdown(t.now);
  CHECK(ev2 == 0 && !(t.p.power_down(t.now) & EV_ENABLE), "in standby: no notice, nothing to drop");
  Sim w(1);
  w.lit[0] = true;
  w.run(500);
  CHECK(w.p.shutdown(w.now) == 0 && (w.p.power_down(w.now) & EV_ENABLE), "waking: no notice, the enable low");
  // the boot: a light restored on wakes the port; one restored off doesn't
  Sim on(1), off(1);
  on.lit[0] = true;
  on.run(3000);
  off.run(3000);
  CHECK(on.p.ready() && on.ready == 1, "a light restored on: the port wakes at the boot");
  CHECK(off.p.state() == PS_STANDBY && off.enable_at.empty() && off.copy_at.empty(),
        "restored off: the port stays in standby, nothing switched");
}

static void test_wrap() {
  std::printf("millis() wrapping\n");
  Sim s(1);
  s.now = 0xFFFFF000u;
  s.lit[0] = true;
  s.run(3000);
  CHECK(s.p.ready(), "woken across the wrap");
  s.lit[0] = false;
  s.run(10 * MIN + 2000);
  CHECK(s.p.state() == PS_STANDBY && s.copy_at.size() == 3, "the turn-off period and the notice across the wrap");
}

// ------------------------------------------------------------------------------- review-r8/portmgr-gpt regressions
static void test_wrap_plug() {
  std::printf("a board plugged in across the millis() wrap: the hot-plug delay from its detect\n");
  Sim s(1);
  s.now = 0xFFFFE000u;
  s.detect = false;
  s.lit[0] = true;
  s.run(4000);   // (enabled without a board before the wrap)
  CHECK(s.p.state() == PS_NO_BOARD && s.p.enable(), "no board, the enable high");
  s.detect = true;
  const uint32_t plug = s.now;   // after the wrap
  s.run_until(5000, [&] { return s.p.ready(); });
  CHECK(s.p.ready() && s.ready_at - plug >= 1700 && s.ready_at - plug <= 1710,
        "ready %u ms after its detect (>= 1.7 s)", s.ready_at - plug);
  // and FAULT's return across the wrap
  Sim f(1);
  f.now = 0xFFFFD000u;
  f.lit[0] = true;
  f.run(3000);
  f.detect = false;
  f.run(600);
  CHECK(f.p.state() == PS_FAULT, "lost before the wrap: FAULT");
  f.now = 200;   // (the wrap)
  f.detect = true;
  const uint32_t back = f.now;
  f.run_until(5000, [&] { return f.p.ready(); });
  CHECK(f.p.ready() && f.ready_at - back >= 1700, "back after the wrap: ready %u ms after its detect",
        f.ready_at - back);
}

static void test_dropout() {
  std::printf("a detect dropout shorter than the debounce: the 55 V was cut\n");
  // waking: the hot-plug delay again from its return
  Sim s(1);
  s.dt = 1;
  const uint32_t t0 = s.now;
  s.lit[0] = true;
  s.run(1400);
  s.detect = false;
  s.run(90);
  s.detect = true;
  const uint32_t back = s.now;
  s.run_until(5000, [&] { return s.p.ready(); });
  CHECK(s.ready_at - back >= 1700 && s.ready_at - t0 > 3000 && s.p.board(),
        "waking, a 90 ms dropout: ready %u ms after its return (not at 1.7 s)", s.ready_at - back);
  // on: a 30 ms dropout: a drop counted, Waking again, ready (everything sent again) after the delay
  s.run(5000);
  const uint32_t drops = s.p.drops(), ready0 = s.ready;
  s.detect = false;
  s.run(30);
  CHECK(s.p.state() == PS_WAKING && !s.p.ready() && s.p.drops() == drops + 1 && s.lost == 1 && !s.p.problem(),
        "on, a 30 ms dropout: Waking, a drop counted, no problem (it's back)");
  s.detect = true;
  const uint32_t back2 = s.now;
  s.run_until(5000, [&] { return s.ready > ready0; });
  CHECK(s.ready == ready0 + 1 && s.ready_at - back2 >= 1700, "ready again %u ms after its return",
        s.ready_at - back2);
  // a longer loss: FAULT, counted once
  s.detect = false;
  s.run(500);
  CHECK(s.p.state() == PS_FAULT && s.p.problem() && s.p.drops() == drops + 2, "lost: FAULT, one more drop");
  // negative control: a dropout in standby changes nothing
  Sim n(1);
  n.dt = 1;
  n.run(500);
  n.detect = false;
  n.run(30);
  n.detect = true;
  n.run(500);
  CHECK(n.p.state() == PS_STANDBY && n.p.drops() == 0 && n.enable_at.empty(), "standby: nothing");
}

static void test_notice_delivery() {
  std::printf("the notice's copies, from their delivery: a slow loop, a slow or dead transmitter\n");
  // a 100 ms loop: all three copies before the enable drops, 100 ms after the third
  Sim s(1);
  s.dt = 100;
  s.p.prewake(s.now, 1);
  s.run(3 * MIN);
  CHECK(s.copy_at.size() == 3 && s.enable_at.size() == 2 && s.enable_at[1] - s.copy_at[2] >= 100,
        "100 ms loop: %u copies, the enable low %u ms after the third", static_cast<unsigned>(s.copy_at.size()),
        s.enable_at.size() == 2 && s.copy_at.size() == 3 ? s.enable_at[1] - s.copy_at[2] : 0);
  // the hub sends each copy 80 ms after it was asked for: 50 ms apart from their delivery, the drop after the third
  Sim d(1);
  d.deliver_after = 80;
  d.p.prewake(d.now, 1);
  d.run(3 * MIN);
  bool spaced = d.out_at.size() == 3;
  for (size_t i = 1; spaced && i < 3; i++)
    spaced = d.copy_at[i] - d.out_at[i - 1] >= 50;
  CHECK(spaced && d.enable_at.size() == 2 && d.enable_at[1] - d.out_at[2] >= 100,
        "a slow transmitter: 3 copies out, each asked 50 ms after the last one went out, the enable low 100 ms "
        "after the third");
  // a dead transmitter: the first copy never goes out: one asked, the enable low after notice_max (1 s)
  Sim x(1);
  x.deliver_after = NEVER;
  x.p.prewake(x.now, 1);
  x.run(3 * MIN);
  CHECK(x.copy_at.size() == 1 && x.out_at.empty() && x.enable_at.size() == 2 &&
            x.enable_at[1] - x.copy_at[0] >= 1000 && x.enable_at[1] - x.copy_at[0] <= 1010 &&
            x.p.state() == PS_STANDBY,
        "a dead transmitter: one copy asked (not again), the enable low after 1 s");
  // a stray delivery report while on doesn't count for the next notice (negative control)
  Sim n(1);
  n.p.prewake(n.now, 1);
  n.run(3000);
  n.p.notice_sent(n.now);
  n.run(3 * MIN);
  CHECK(n.copy_at.size() == 3 && n.enable_at[1] - n.copy_at[0] == 200, "a stray report: ignored");
}

static void test_masked_prewake() {
  std::printf("a pre-wake left under Always on for 25 days\n");
  Sim s(1);
  s.dt = 1000;
  s.p.prewake(s.now, 2 * MIN);
  s.p.set_mode(MODE_ALWAYS_ON);
  s.run(25u * 24 * 3600 * 1000);
  s.p.set_mode(MODE_STANDBY);
  s.run(11 * MIN);
  CHECK(!s.copy_at.empty() && s.p.state() != PS_ON, "Standby again: the notice after the mode's turn-off period (%u)",
        static_cast<unsigned>(s.p.state()));
  // the same with updates 25 days apart (review-r8/portmgr-gpt's sequence): the timers compare elapsed time
  PortCore q;
  q.set_detect(true, 0);
  q.prewake(0, 2 * MIN);
  q.set_mode(MODE_ALWAYS_ON);
  q.update(0);
  q.update(1700);
  const uint32_t days25 = 25u * 24 * 3600 * 1000;
  q.update(days25);
  q.set_mode(MODE_STANDBY);
  q.update(days25);
  q.update(days25 + 600001);
  CHECK(q.state() == PS_NOTICE, "updates 25 days apart: the notice after the mode's turn-off period (%u)",
        static_cast<unsigned>(q.state()));
  // a light's turn-off period left alone across the 49.7-day wrap (negative control for the retirement)
  Sim l(1);
  l.dt = 1000;
  l.lit[0] = true;
  l.run(3000);
  l.lit[0] = false;
  l.p.set_mode(MODE_ALWAYS_ON);
  l.run(50u * 24 * 3600 * 1000);
  l.p.set_mode(MODE_STANDBY);
  l.run(11 * MIN);
  CHECK(l.p.state() != PS_ON, "a light's old hold doesn't come back after 50 days");
}

/// dark at t, by brute force: the sun below the horizon at any minute of [t - after, t + before]
static bool dark_brute(double t, double lat, double lon, uint32_t before, uint32_t after, double *margin) {
  double lowest = 90;
  for (double s = t - after; s <= t + before + 1; s += 60)
    lowest = std::min(lowest, solar_elevation(std::min(s, t + before), lat, lon));
  *margin = std::fabs(lowest - SUN_HORIZON);
  return lowest < SUN_HORIZON;
}

static void test_dark_offsets() {
  std::printf("dark offsets up to 12 h: the interval, not two samples\n");
  CHECK(dark_window(unix_at(2026, 6, 21, 0, 0), 52.1, 5.1, 6 * 3600, 6 * 3600),
        "Utrecht, summer midnight, 6 h + 6 h: dark (the sun at -14 degrees)");
  CHECK(dark_window(unix_at(2026, 3, 20, 0, 0), 0, 0, 12 * 3600, 12 * 3600), "equatorial midnight, 12 h + 12 h: dark");
  // against the brute-force search: latitudes from the equator to 78 N and 45 S, a year, offsets 0 to 12 h
  const double lats[] = {0, 30, 52.1, 66.5, 78.2, -45};
  const uint32_t offs[] = {0, 1800, 3600, 6 * 3600, 12 * 3600};
  int cases = 0, wrong = 0;
  for (double lat : lats) {
    for (int day = 0; day < 365; day += 23) {
      for (int minute = 0; minute < 1440; minute += 97) {
        const double t = unix_at(2026, 1, 1, 0, 0) + day * 86400.0 + minute * 60.0;
        for (uint32_t b : offs) {
          for (uint32_t a : offs) {
            double margin;
            const bool truth = dark_brute(t, lat, 5.1, b, a, &margin);
            if (margin < 0.05)
              continue;   // (at the horizon itself the one-minute search isn't the truth)
            cases++;
            if (dark_window(t, lat, 5.1, b, a) != truth && wrong++ < 3)
              std::printf("  lat %.1f day %d min %d before %u after %u: %d, brute %d\n", lat, day, minute, b, a,
                          !truth, truth);
          }
        }
      }
    }
  }
  CHECK(wrong == 0, "%d of %d cases differ from the brute-force search", wrong, cases);
}

static void test_shutdown_during_notice() {
  std::printf("a shutdown during a notice: it goes on with its own\n");
  Sim s(1);
  s.p.prewake(s.now, 1);
  s.run_until(3 * MIN, [&] { return !s.copy_at.empty(); });
  const uint32_t n0 = s.copy_at[0];
  s.run(20);
  CHECK(s.p.shutdown(s.now) == 0 && s.p.state() == PS_NOTICE && s.p.enable(), "NOTICE + 25: still its notice");
  s.run(400);
  CHECK(s.copy_at.size() == 3 && s.enable_at.back() - n0 == 200 && !s.p.enable(),
        "its copies at 50 / 100, the enable low at 200 (%u)", s.enable_at.back() - n0);
}

static void test_power_cycle() {
  std::printf("a deliberate power cycle (firmware recovery)\n");
  Sim s(1);
  s.lit[0] = true;
  s.run(3000);
  const uint32_t t0 = s.now, cycles = s.p.standby_cycles();
  const uint8_t ev = s.p.power_cycle(t0);
  CHECK((ev & EV_ENABLE) && !s.p.enable() && s.p.state() == PS_OFF_HOLD, "off at once, a light lit or not");
  s.run_until(10000, [&] { return s.ready > 1; });
  CHECK(s.ready == 2 && s.ready_at - t0 >= 1500 + 1700 && s.ready_at - t0 <= 1500 + 1700 + 20 &&
            s.p.standby_cycles() == cycles && s.copy_at.empty() && s.p.power_cycle_ms() == 3200,
        "off 1.5 s, then a wake: ready %u ms later (power_cycle_ms 3200); no notice, not a standby cycle",
        s.ready_at - t0);
  // from standby, nothing lit: the boards boot once (a wake), then the turn-off period as usual
  Sim t(1);
  t.run(1000);
  t.p.power_cycle(t.now);
  t.run(5000);
  CHECK(t.ready == 1, "from standby: a wake after the off time");
}

int main() {
  std::printf("diffpower test (the port power manager's core)\n");
  test_cycle();
  test_request_during_notice();
  test_detect();
  test_no_board();
  test_dark_window();
  test_modes();
  test_prewake();
  test_per_light();
  test_counters();
  test_shutdown_and_boot();
  test_wrap();
  test_wrap_plug();
  test_dropout();
  test_notice_delivery();
  test_masked_prewake();
  test_dark_offsets();
  test_shutdown_during_notice();
  test_power_cycle();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL DIFFPOWER TESTS PASSED", failures);
  return failures ? 1 : 0;
}
