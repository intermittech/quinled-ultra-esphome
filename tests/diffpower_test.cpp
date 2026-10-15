// Host test of the Diff-Power port power manager's core (components/diffpower/diffpower_core.h) with simulated time,
// after design review, round 8 (ESPHome): the standby cycle (the notice's copies at 0 / 50 / 100 ms, the enable low
// at 200, OFF_HOLD, STANDBY); a request during the notice (no take-back, the wake after min_off_time, the latency);
// detect flapping and lost (FAULT, its return); the dark window (midnight, noon, 78 degrees north in June and December,
// the offsets exact, no valid time); Always on at boot and a mode change (its turn-off period); pre-wakes and the
// the min_on_time floor; the turn-off period per light; a port without a board; the counters; a controller shutdown; the
// boot; millis() wrapping. Then the review's failures as regressions: a plug across the millis() wrap, a
// detect dropout shorter than the debounce (waking and on), the notice's copies timed from their delivery (a slow
// loop, a slow or dead transmitter), a pre-wake masked for 25 days, dark offsets up to 12 h (against a brute-force
// search of the elevation), a shutdown during a notice, a deliberate power cycle. "During dark" from Home Assistant:
// its ISO-8601 times, the stateless rule against the elevation (a year at 0 / 52 / 65 N, polar day and night), the
// source (fresh, stale, missing, disconnected; the sun as the backup). The power mode Off (round 12): off now from ON,
// WAKING and FAULT, never powered whatever is lit, nothing left over once it ends (randomized too). Negative controls
// alongside: what must NOT wake or drop the port.
//
// Build and run: python tests/run_diffpower_test.py

#include "../components/diffpower/diffpower_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
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

// ---------------------------------------------------------------------------------------- the review's regressions
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
  // the same with updates 25 days apart (the review's sequence): the timers compare elapsed time
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

/// Home Assistant's sun times (ISO-8601, as its sun entity sends them): the fraction dropped, Z / +00:00 / other zones;
/// not a time (unknown, no zone, out of range, trailing text): refused
static void test_iso8601() {
  std::printf("Home Assistant's sun times: ISO-8601 with a zone\n");
  struct Case {
    const char *s;
    double want;   // (< 0: refused)
  };
  const double t0 = unix_at(2026, 10, 16, 5, 58) + 41;
  const Case cases[] = {
      {"2026-10-16T05:58:41.237114+00:00", t0},
      // Home Assistant 2026.10's own: sun.sun (microseconds), sensor.sun_next_rising (seconds only)
      {"2026-10-16T06:07:36.038102+00:00", unix_at(2026, 10, 16, 6, 7) + 36},
      {"2026-10-16T16:43:30.106389+00:00", unix_at(2026, 10, 16, 16, 43) + 30},
      {"2026-10-16T06:07:36+00:00", unix_at(2026, 10, 16, 6, 7) + 36},
      {"2026-10-16T05:58:41+00:00", t0},
      {"2026-10-16T05:58:41Z", t0},
      {"2026-10-16T05:58:41.999999Z", t0},
      {"2026-10-16 05:58:41+00:00", t0},
      {"2026-10-16T07:58:41.5+02:00", t0},
      {"2026-10-16T07:58:41+0200", t0},
      {"2026-10-16T00:28:41-05:30", t0},
      {"2026-10-16T07:58:41+02", t0},
      {"2026-12-31T23:59:59Z", unix_at(2027, 1, 1, 0, 0) - 1},
      {"2000-03-01T00:00:00Z", unix_at(2000, 3, 1, 0, 0)},
      {"2099-12-31T00:00:00Z", unix_at(2099, 12, 31, 0, 0)},
      {"1970-01-01T00:00:00Z", 0},
      {"2024-02-29T00:00:00+00:00", unix_at(2024, 2, 29, 0, 0)},
      {"2026-01-01T00:30:00+01:00", unix_at(2025, 12, 31, 23, 30)},
      {"unknown", -1},
      {"None", -1},
      {"", -1},
      {"2026-10-16T05:58:41", -1},
      {"2026-13-16T05:58:41Z", -1},
      {"2026-10-16T25:58:41Z", -1},
      {"2026-10-16T05:58:41.Z", -1},
      {"2026-10-16T05:58:41+00:00 ", -1},
      {"2026-10-16T05:58:41+00:0", -1},
      {"2026-10-16T05:58:41+24:00", -1},
      {"2026-10-16T05:58Z", -1},
      {"2026-10-16T05:58:41Zz", -1},
      {"2026-10-16T05:58:41+02:0x", -1},
      {"2026-10-16T05:58:41+02:000", -1},
      {"2026-1a-16T05:58:41Z", -1},
      {"1969-12-31T23:59:59Z", -1},
      {"2100-01-01T00:00:00Z", -1},
  };
  for (const Case &c : cases) {
    int64_t t = -7;
    const bool ok = parse_iso8601(c.s, std::strlen(c.s), &t);
    CHECK(c.want < 0 ? !ok : (ok && t == static_cast<int64_t>(c.want)), "'%s': %s (%lld)", c.s,
          ok ? "parsed" : "refused", static_cast<long long>(t));
  }
  // every day from 1970 to 2099 (its leap years, the months' lengths) against the civil calendar
  static const int MONTH_DAYS[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int wrong = 0, days = 0;
  for (int y = 1970; y <= 2099; y++) {
    for (int m = 1; m <= 12; m++) {
      for (int d = 1; d <= MONTH_DAYS[m - 1] + (m == 2 && y % 4 == 0); d++, days++) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT12:34:56+00:00", y, m, d);
        int64_t t = 0;
        wrong += !parse_iso8601(buf, std::strlen(buf), &t) || t != static_cast<int64_t>(unix_at(y, m, d, 12, 34)) + 56;
      }
    }
  }
  CHECK(wrong == 0, "every day 1970-2099: %d of %d wrong", wrong, days);
}

/// The sunrises and sunsets (the horizon crossings, to the second) from t0 to t1: what Home Assistant's sun entity
/// gives as next_rising / next_setting, with the same sun as dark_window
static void sun_events(double lat, double lon, double t0, double t1, std::vector<int64_t> *rises,
                       std::vector<int64_t> *sets) {
  const double step = 300;
  bool below = solar_elevation(t0, lat, lon) < SUN_HORIZON;
  for (double t = t0 + step; t <= t1; t += step) {
    const bool b = solar_elevation(t, lat, lon) < SUN_HORIZON;
    if (b != below) {
      double lo = t - step, hi = t;
      for (int k = 0; k < 24; k++) {
        const double mid = (lo + hi) / 2;
        ((solar_elevation(mid, lat, lon) < SUN_HORIZON) == below ? lo : hi) = mid;
      }
      (below ? rises : sets)->push_back(static_cast<int64_t>(std::lround(hi)));
      below = b;
    }
  }
}

/// Home Assistant's next sunrise / sunset at t (moved on at each event): the first of each after t
struct HaFeed {
  std::vector<int64_t> rises, sets;
  size_t r{0}, s{0};
  void at(int64_t t) {
    while (this->r < this->rises.size() && this->rises[this->r] <= t)
      this->r++;
    while (this->s < this->sets.size() && this->sets[this->s] <= t)
      this->s++;
  }
  int64_t rise() const { return this->rises[this->r]; }
  int64_t set() const { return this->sets[this->s]; }
};

/// The stateless rule against the sun's elevation (dark_window), every minute of a year at 0, 52 and 65 degrees north:
/// the longest run of minutes they differ at a window's edge, for the default offsets (60 / 60 min), 30 min / 2 h and
/// none. Then 78 N: polar day never dark, polar night always.
static void test_ha_dark() {
  std::printf("During dark from Home Assistant's next sunrise / sunset: against the sun's elevation, a year\n");
  struct Place {
    double lat, lon;
    int max_min;   // the longest run of differing minutes allowed
  };
  const Place places[] = {{0, 0, 2}, {52.1, 5.1, 4}, {65, 25.5, 6}};
  const uint32_t offs[][2] = {{3600, 3600}, {1800, 7200}, {0, 0}};
  const double y0 = unix_at(2026, 1, 1, 0, 0), y1 = y0 + 365 * 86400.0;
  for (const Place &pl : places) {
    HaFeed f;
    sun_events(pl.lat, pl.lon, y0 - 3 * 86400.0, y1 + 3 * 86400.0, &f.rises, &f.sets);
    for (const auto &o : offs) {
      int run = 0, longest = 0, minutes = 0;
      f.r = f.s = 0;
      for (double t = y0; t < y1; t += 60) {
        const int64_t now = static_cast<int64_t>(t);
        f.at(now);
        const bool differ = ha_dark(now, f.rise(), f.set(), o[0], o[1]) != dark_window(t, pl.lat, pl.lon, o[0], o[1]);
        run = differ ? run + 1 : 0;
        longest = std::max(longest, run);
        minutes += differ;
      }
      std::printf("  %4.1f N, %3u / %3u min: %5d minutes differ in the year, at most %d in a row\n", pl.lat,
                  o[0] / 60, o[1] / 60, minutes, longest);
      CHECK(longest <= pl.max_min, "%.1f N, %u / %u min: %d minutes in a row differ (at most %d)", pl.lat, o[0] / 60,
            o[1] / 60, longest, pl.max_min);
    }
  }
  // 78 N (Longyearbyen): Home Assistant's times weeks ahead in polar day and night
  HaFeed f;
  sun_events(78.2, 15.6, y0 - 3 * 86400.0, y1 + 120 * 86400.0, &f.rises, &f.sets);
  bool june = false, december = true, agree = true;
  for (int h = 0; h < 24; h++) {
    const int64_t jt = static_cast<int64_t>(unix_at(2026, 6, 21, h, 0)), dt = static_cast<int64_t>(unix_at(2026, 12, 21, h, 0));
    f.r = f.s = 0;
    f.at(jt);
    june |= ha_dark(jt, f.rise(), f.set(), 3600, 3600);
    agree &= f.set() - jt > 30 * 86400 && f.rise() > f.set();
    f.at(dt);
    december &= ha_dark(dt, f.rise(), f.set(), 3600, 3600);
    agree &= f.rise() - dt > 30 * 86400 && f.rise() < f.set();
  }
  CHECK(agree && !june && december, "78 N: polar day never dark, polar night always (times weeks ahead: %s)",
        agree ? "yes" : "no");
}

/// Which source decides: Home Assistant's while both times are there and neither is stale (more than 10 min past;
/// kept through a disconnect), else ESPHome's sun, else unknown; no valid time: unknown. Then a whole disconnect at
/// noon: Home Assistant's frozen times until the sunset is 10 min past, then the backup (none without one).
static void test_dark_source() {
  std::printf("During dark's source: Home Assistant's while fresh, else ESPHome's sun, else unknown\n");
  const int64_t now = static_cast<int64_t>(unix_at(2026, 10, 16, 12, 0));
  const int64_t rise = now + 19 * 3600, set = now + 5 * 3600;
  CHECK(dark_source(true, 3, now, rise, set, true) == DARK_HA && dark_source(true, 3, now, rise, set, false) == DARK_HA,
        "both there, ahead: Home Assistant's (with or without the backup)");
  CHECK(dark_source(false, 3, now, rise, set, true) == DARK_NONE && dark_source(false, 0, now, 0, 0, true) == DARK_NONE,
        "no valid time: unknown");
  for (uint8_t have : {0, 1, 2})
    CHECK(dark_source(true, have, now, rise, set, true) == DARK_SUN &&
              dark_source(true, have, now, rise, set, false) == DARK_NONE,
          "received %u of 3: the sun (with one), else unknown", have);
  CHECK(dark_source(true, 3, set + 600, rise, set, false) == DARK_HA &&
            dark_source(true, 3, set + 601, rise, set, true) == DARK_SUN &&
            dark_source(true, 3, set + 601, rise, set, false) == DARK_NONE,
        "the sunset 10 min past, not moved on: still; a second later stale: the sun, or unknown");
  CHECK(dark_source(true, 3, rise + 601, rise, rise + 3600, true) == DARK_SUN, "a stale sunrise too");
  // disconnected at noon (52 N): the frozen times decide until the sunset is 10 min past, then the sun's
  HaFeed f;
  const double day = unix_at(2026, 10, 16, 0, 0);
  sun_events(52.1, 5.1, day - 86400, day + 3 * 86400, &f.rises, &f.sets);
  f.at(now);
  const int64_t fr = f.rise(), fs = f.set();
  int64_t ha_until = 0;
  int wrong = 0;
  for (int64_t t = now; t < now + 36 * 3600; t += 60) {
    const DarkSource src = dark_source(true, 3, t, fr, fs, true);
    if (src == DARK_HA)
      ha_until = t;
    const bool dark = src == DARK_HA ? ha_dark(t, fr, fs, 3600, 3600) : dark_window(t, 52.1, 5.1, 3600, 3600);
    wrong += dark != dark_window(t, 52.1, 5.1, 3600, 3600);
    CHECK(dark_source(true, 3, t, fr, fs, false) == (src == DARK_HA ? DARK_HA : DARK_NONE), "without the backup");
  }
  CHECK(ha_until >= fs + 540 && ha_until <= fs + 600 && wrong == 0,
        "disconnected at noon: Home Assistant's until %+lld s after the sunset, then the sun's; %d minutes wrong",
        static_cast<long long>(ha_until - fs), wrong);
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

// ---------------------------------------------------------------------------- the review recheck's regressions
static void test_prewake_at_expiry() {
  std::printf("a pre-wake right after the one before ran out (before the next update): kept\n");
  PortCore p;
  p.set_detect(true, 0);
  p.prewake(0, 120000);
  p.update(0);
  p.update(119999);
  p.prewake(120001, 120000);   // (the old one ran out at 120000; no update since)
  p.update(120001);
  CHECK(p.want_power(120001) && p.want_power(240000) && !p.want_power(240002),
        "the new pre-wake holds until 240001 (wanted at 120001 %d, 240000 %d, 240002 %d)", p.want_power(120001),
        p.want_power(240000), p.want_power(240002));
  // negative control: a pre-wake still running is extended only by a longer one
  PortCore q;
  q.set_detect(true, 0);
  q.prewake(0, 300000);
  q.update(0);
  q.prewake(100000, 120000);   // (ends at 220000: shorter than what's left)
  q.update(250000);
  CHECK(q.want_power(299999) && !q.want_power(300001), "a shorter pre-wake meanwhile: the first one's end");
}

static void test_long_notice() {
  std::printf("a long notice: never cut short by the cap\n");
  PortCore p;
  PortConfig c;
  c.notice_ms = 1500;   // (the YAML allows 1 s at most; the core keeps its cap at least the notice anyway)
  p.configure(c);
  CHECK(p.config().notice_max_ms == 1500, "the cap raised to the notice (%u)", p.config().notice_max_ms);
  p.set_detect(true, 0);
  p.prewake(0, 1);
  uint32_t n0 = 0, low = 0;
  for (uint32_t t = 0; t < 200000 && low == 0; t += 10) {
    const uint8_t ev = p.update(t);
    if (ev & EV_NOTICE) {
      n0 = n0 == 0 ? t : n0;
      p.notice_sent(t);
    }
    if ((ev & EV_ENABLE) && !p.enable())
      low = t;
  }
  CHECK(n0 != 0 && low - n0 == 1500, "copies on time: the enable low at 1500 ms (%u)", low - n0);
}

// ------------------------------------------------------------------------------------- the power mode Off (round 12)
static std::string off_text(const PortCore &p) {
  char b[32];
  return port_state_text(p.state(), p.drops(), b, sizeof(b), p.off());
}

/// Every enable change high after `from` (a power-up)
static size_t rises_after(const Sim &s, uint32_t from) {
  size_t n = 0;
  for (size_t i = 0; i < s.enable_at.size(); i++)
    n += s.enable_to[i] && static_cast<int32_t>(s.enable_at[i] - from) >= 0;
  return n;
}

static void test_off_mode() {
  std::printf("the power mode Off: off now, never powered whatever is lit, nothing left over once it ends\n");
  // ON, a light lit: the planned power-off at once (no min_on_time), the notice, the enable low, "Off"
  Sim s(1);
  s.lit[0] = true;
  s.run(3000);
  CHECK(s.p.state() == PS_ON, "on");
  const uint32_t t0 = s.now;
  s.p.set_mode(MODE_OFF);
  s.run(2000);
  CHECK(s.copy_at.size() == 3 && s.copy_at[0] - t0 <= 10 && s.enable_at.back() - s.copy_at[0] == 200 &&
            !s.p.enable() && s.p.standby_cycles() == 1,
        "Off while on and lit: the notice at once (%u ms), the enable low 200 ms after it, a standby cycle",
        s.copy_at.empty() ? 0 : s.copy_at[0] - t0);
  CHECK(s.p.state() == PS_STANDBY && off_text(s.p) == "Off" && !s.p.problem(), "at rest: 'Off' (%s)",
        off_text(s.p).c_str());
  // an hour of everything that would power it: lights on and off, pre-wakes, Always on's leftovers, power cycles,
  // detect flaps, a board unplugged and back
  const uint32_t rest = s.now;
  for (int i = 0; i < 60; i++) {
    s.lit[0] = (i % 3) != 0;
    s.p.prewake(s.now, 10 * MIN);
    CHECK(s.p.power_cycle(s.now) == 0, "a power cycle while Off: nothing");
    s.detect = (i % 7) != 3;
    s.run(MIN);
  }
  s.detect = true;
  s.run(MIN);
  CHECK(rises_after(s, rest) == 0 && !s.p.enable() && s.ready == 1 && s.copy_at.size() == 3,
        "an hour of lights, pre-wakes, power cycles and detect flaps while Off: never powered (%zu rises)",
        rises_after(s, rest));
  // Off ends with a light lit: powered (it follows its lights from then on), as a light turned on in Standby
  s.lit[0] = true;
  s.run(1000);
  const uint32_t t1 = s.now;
  s.p.set_mode(MODE_STANDBY);
  CHECK(s.run_until(5000, [&] { return s.p.ready(); }) && s.ready_at - t1 <= 1720,
        "Standby again, the light lit: on (%u ms)", s.ready_at - t1);
  // Off ends with nothing lit: nothing from before powers it (a light turned off while Off, a pre-wake, the
  // turn-off period of a light turned off just before Off, Always on's turn-off period)
  Sim q(2);
  q.p.set_mode(MODE_ALWAYS_ON);
  q.lit[0] = true;
  q.run(3 * MIN);
  q.lit[0] = false;   // (its 10 min turn-off period starts)
  q.run(1000);
  q.p.set_mode(MODE_OFF);
  q.run(3000);
  q.lit[1] = true;    // on and off again while Off
  q.run(MIN);
  q.lit[1] = false;
  q.run(1000);
  q.p.prewake(q.now, 20 * MIN);
  q.run(1000);
  const uint32_t t2 = q.now;
  q.p.set_mode(MODE_STANDBY);
  q.run(30 * MIN);
  CHECK(rises_after(q, t2) == 0 && q.p.state() == PS_STANDBY && off_text(q.p) == "Standby",
        "Standby again, nothing lit: stays in standby (%zu rises, '%s')", rises_after(q, t2), off_text(q.p).c_str());
  // negative control: the same without Off powers the port (the light's turn-off period, the pre-wake)
  Sim c(2);
  c.p.set_mode(MODE_ALWAYS_ON);
  c.lit[0] = true;
  c.run(3 * MIN);
  c.lit[0] = false;
  c.run(1000);
  c.p.set_mode(MODE_STANDBY);
  c.run(5 * MIN);
  CHECK(c.p.state() == PS_ON, "control: Standby keeps the turn-off period (on)");
  // Off while waking: the enable low at once, no notice, never ready
  Sim w(1);
  w.lit[0] = true;
  w.run(800);
  CHECK(w.p.state() == PS_WAKING && w.p.enable(), "waking");
  const uint32_t t3 = w.now;
  w.p.set_mode(MODE_OFF);
  w.step();
  CHECK(!w.p.enable() && w.enable_at.back() == t3 && w.copy_at.empty(), "Off while waking: the enable low now");
  w.run(10 * MIN);
  CHECK(w.ready == 0 && w.copy_at.empty() && off_text(w.p) == "Off" && rises_after(w, t3) == 0,
        "... never ready, no notice, 'Off'");
  // Off with a connection lost (FAULT): at rest, no problem; the board's return wakes nothing
  Sim f(1);
  f.lit[0] = true;
  f.run(3000);
  f.detect = false;
  f.run(1000);
  CHECK(f.p.state() == PS_FAULT && f.p.problem(), "FAULT");
  f.p.set_mode(MODE_OFF);
  f.run(1000);
  CHECK(f.p.state() == PS_NO_BOARD && !f.p.problem() && !f.p.enable() && off_text(f.p) == "Off (no board)",
        "Off: 'Off (no board)', no problem (%s)", off_text(f.p).c_str());
  const uint32_t t4 = f.now;
  f.detect = true;
  f.run(5 * MIN);
  CHECK(rises_after(f, t4) == 0 && off_text(f.p) == "Off", "the board back: 'Off', not powered");
  // no board, a light lit, Off: the enable stays low (Standby raises it so a board plugged in gets power)
  Sim n(1);
  n.detect = false;
  n.p.set_mode(MODE_OFF);
  n.lit[0] = true;
  n.run(5000);
  CHECK(!n.p.enable() && !n.p.problem() && n.enable_at.empty(), "no board, a light, Off: enable low, no problem");
  n.detect = true;
  n.run(5000);
  CHECK(n.enable_at.empty() && n.p.state() == PS_STANDBY, "a board plugged in: not powered");
  // a request during the notice, then Off: no wake after the off-hold
  Sim r(1);
  r.lit[0] = true;
  r.run(3000);
  r.lit[0] = false;
  r.run_until(11 * MIN, [&] { return !r.copy_at.empty(); });
  r.lit[0] = true;   // (during the notice: a wake after min_off_time)
  r.step();
  r.p.set_mode(MODE_OFF);
  const uint32_t t5 = r.now;
  r.run(5 * MIN);
  CHECK(rises_after(r, t5) == 0 && r.ready == 1, "a wake asked for during the notice, then Off: no wake");
  // boot in Off (restored before the first update): never powered
  Sim b(1);
  b.p.set_mode(MODE_OFF);
  b.lit[0] = true;
  b.p.prewake(b.now, 5 * MIN);
  b.run(10 * MIN);
  CHECK(b.enable_at.empty() && off_text(b.p) == "Off", "boot in Off, a light lit: never powered");
  // the mode's values (stored by the select)
  CHECK(MODE_OFF == 3 && MODE_COUNT == 4, "Off is mode 3 (stored values 0-2 unchanged)");
}

/// A random hour per seed: lights, pre-wakes, detect flaps, power cycles, mode changes; while Off the enable never
/// goes high, and at the moment Off is chosen it goes low within the notice's time (ON) or at once (otherwise)
static void test_off_random() {
  std::printf("the power mode Off, randomized: never powered while Off\n");
  uint32_t seed = 12345, bad = 0, offs = 0;
  auto rnd = [&seed](uint32_t n) {
    seed = seed * 1664525u + 1013904223u;
    return (seed >> 8) % n;
  };
  for (int run = 0; run < 200; run++) {
    Sim s(2);
    bool off = false;
    uint32_t off_since = 0;
    for (int i = 0; i < 400; i++) {
      switch (rnd(8)) {
        case 0:
          s.lit[rnd(2)] = rnd(2) != 0;
          break;
        case 1:
          s.p.prewake(s.now, (1 + rnd(20)) * MIN);
          break;
        case 2:
          s.detect = rnd(6) != 0;
          break;
        case 3:
          s.p.power_cycle(s.now);
          break;
        case 4: {
          const PowerMode m = static_cast<PowerMode>(rnd(MODE_COUNT));
          s.p.set_mode(m);
          s.p.set_dark(rnd(2) != 0);
          if ((m == MODE_OFF) != off) {
            off = m == MODE_OFF;
            off_since = s.now;
            offs += off;
          }
          break;
        }
        default:
          break;
      }
      const uint32_t step = 10 * (1 + rnd(500));
      for (uint32_t end = s.now + step; static_cast<int32_t>(s.now - end) < 0;) {
        s.step();
        // (Off since: the notice of a port that was ON may still run: 1 s at most, then the enable is low)
        if (off && s.p.enable() && s.now - off_since > s.p.config().notice_max_ms + 20)
          bad++;
        if (off && s.p.enable() && s.p.state() != PS_NOTICE && s.p.state() != PS_ON)
          bad++;
      }
    }
  }
  CHECK(bad == 0 && offs > 1000, "200 random hours, %u Off periods: %u steps powered while Off", offs, bad);
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
  test_iso8601();
  test_ha_dark();
  test_dark_source();
  test_shutdown_during_notice();
  test_power_cycle();
  test_prewake_at_expiry();
  test_long_notice();
  test_off_mode();
  test_off_random();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL DIFFPOWER TESTS PASSED", failures);
  return failures ? 1 : 0;
}
