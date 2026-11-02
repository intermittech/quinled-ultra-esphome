#pragma once

// A QuinLED Diff-Power sender port's power manager, without ESPHome (tests/diffpower_test.cpp runs it with simulated
// time). Wiki: Diff-Power ports. The design: design review, round 8 (with its fixes).
//
// The hardware: per port an enable output and a detect input. The port carries ~4.5 V / 100 mA (standby, the board
// detection) all the time; its 55 V needs enable AND detect (the sender's AND gate) and arrives ~1.5 s after both
// (the sender's hot-plug delay); any dropout of either cuts it and restarts that delay. The boards behind it then boot;
// a Diff-Power receiver stays dark until the hub's first frame (or shows its power-on scene after 1.5 s).
//
// The states (each change is an event for the wrapper):
//   NO_BOARD  no board detected: the enable follows `wanted` (a board plugged in gets its power); a problem if wanted
//   STANDBY   enable low, a board there (4.5 V)
//   WAKING    enable high: powered hotplug_ms + boot_margin_ms after enable AND raw detect last became true (EV_READY:
//             the hub's frames start)
//   ON        powered: lights get their data; at least min_on_ms, then until nothing wants it. A raw detect dropout
//             (even shorter than the debounce) cut the 55 V: a drop counted, WAKING again from its return
//   NOTICE    the planned power-off notice: a copy at a time (EV_NOTICE), the next notice_spacing_ms after the last one
//             went out (notice_sent()); the enable low notice_ms - 2 x spacing after the third, or notice_max_ms after
//             the notice at the latest. Requests meanwhile wake it after OFF_HOLD (never back to ON)
//   OFF_HOLD  enable low for min_off_ms (the bus falls below the receivers' UVLO; a power cycle: cycle_off_ms), then
//             WAKING or STANDBY
//   FAULT     detect went away while ON (unplugged, a cable fault, the sender's supply gone): a problem; its return
//             wakes the port again
//   OFF       before the first update, and after power_down() (the controller's shutdown)
//
// `wanted`: a light lit, or off for less than its turn-off period (standby_after, per light, at least min_on_ms); a
// pre-wake; the mode Always on, or During dark inside the dark window (no valid time: unknown_on decides), and the
// port's turn-off period after either ends. Every timer is retired at every update, whatever else wants power.
//
// The power mode Off (working on the strips): never wanted, whatever is lit. Choosing it powers the port off now (ON:
// the planned power-off without min_on_ms; WAKING: the enable low at once, no notice: the boards had no frame yet;
// FAULT: at rest, no problem any more) and forgets every turn-off period, pre-wake and wake request; pre-wakes and
// power cycles meanwhile are dropped. The lights keep their state: once another mode is chosen, the port follows
// that mode and the lights lit then (as a light turned on in Standby would), and nothing from before Off.
//
// Always on, the dark re-check: the sender qualifies a port's load (its standby check) only while the enable is low,
// so a port that never powers off would never be checked again. Once every light on it is dark and their turn-off
// periods are over (when Standby would power it off), after a light was lit since its last wake, the port takes a
// planned power-off (the notice: the receivers stay dark, nothing blinks; a standby cycle), stays off recheck_off_ms
// (>= 2 s) and wakes again: once per dark period. A light turned on meanwhile waits for that wake. Standby and During
// dark re-qualify on each wake already.
//
// Counters for the wrapper to persist (EV_COUNTERS): standby cycles (planned power-offs), detect drops (power
// interruptions while ON), seconds in standby (STANDBY and OFF_HOLD).
//
// The wake timeline (WakeLog, EV_WAKE): for every wake, in ms from its start: the sender's DETECT rise, the wake over
// (ready), the first receiver reply, the first frame with a lit light. Observation only: it never changes what the port
// does (the readiness above is the same with or without it).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace esphome::diffpower {

enum PowerMode : uint8_t { MODE_STANDBY = 0, MODE_DURING_DARK, MODE_ALWAYS_ON, MODE_OFF, MODE_COUNT };
enum PortState : uint8_t {
  PS_NO_BOARD = 0,
  PS_STANDBY,
  PS_WAKING,
  PS_ON,
  PS_NOTICE,
  PS_OFF_HOLD,
  PS_FAULT,
  PS_OFF,
  PS_COUNT
};

/// update()'s events (bits)
enum PortEvent : uint8_t {
  EV_STATE = 1,      // the state (or the problem) changed
  EV_ENABLE = 2,     // the enable output changed (enable())
  EV_NOTICE = 4,     // send a copy of the planned power-off notice now (notice_copy(): 0, 1, 2); notice_sent() once out
  EV_READY = 8,      // the wake is over: send everything (the boards just booted; data sent before is lost)
  EV_LOST = 16,      // the power cut while ON (detect lost, or a dropout)
  EV_COUNTERS = 32,  // standby cycles / detect drops changed (persist)
  EV_PREPARE = 64,   // a planned power-off is next: its hubs' last reads first (set_notice_ready(), prepare_max_ms)
  EV_WAKE = 128,     // a wake timeline finished (wake().result(), then wake_taken())
};

static constexpr uint32_t MAX_HOLD_MS = 86400000;   // turn-off periods and pre-wakes: 24 h at most

struct PortConfig {
  uint32_t hotplug_ms{1500};          // the sender's delay from enable AND detect to 55 V
  uint32_t boot_margin_ms{200};       // 55 V -> the boards booted: the hub's frames start
  uint32_t notice_ms{200};            // NOTICE -> the enable drops (on time: copies at 0 / 50 / 100, low at 200)
  uint32_t notice_spacing_ms{50};     // between the notice's copies (from the last one out)
  uint8_t notice_copies{3};
  uint32_t prepare_max_ms{0};         // before the notice: its hubs' last reads, at most this long (0: none)
  uint32_t notice_max_ms{1000};       // NOTICE at most this long (copies the hub can't send are given up; never
                                      // shorter than notice_ms; the receiver forgets a notice after 2 s)
  uint32_t min_off_ms{1000};          // the enable stays low this long after a planned power-off
  uint32_t cycle_off_ms{1500};        // ... after a power cycle (firmware recovery)
  uint32_t min_on_ms{120000};         // ON at least this long; the floor of turn-off periods and pre-wakes
  uint32_t standby_after_ms{600000};  // the turn-off period of a light without its own
  uint32_t debounce_ms{100};          // detect must hold a new level this long (presence; not the power gate)
  uint32_t recheck_off_ms{2000};      // Always on: the dark re-check's power-off (the sender re-qualifies the port)
  bool unknown_on{true};              // During dark without a valid time: powered (else Standby)
};

/// The sun's position at a UTC time (s since 1970) for a longitude (degrees, east positive): its declination and its
/// local hour angle (radians); the low-precision solar coordinates (Astronomical Almanac), within ~0.1 degree.
inline void solar_position(double unix_s, double lon_deg, double *dec, double *hour_angle) {
  constexpr double RAD = 3.14159265358979323846 / 180.0;
  const double d = unix_s / 86400.0 - 10957.5;   // days since J2000.0 (2000-01-01 12:00)
  const double g = (357.529 + 0.98560028 * d) * RAD;
  const double q = 280.459 + 0.98564736 * d;
  const double l = (q + 1.915 * std::sin(g) + 0.020 * std::sin(2 * g)) * RAD;   // the ecliptic longitude
  const double e = (23.439 - 0.00000036 * d) * RAD;                            // the obliquity
  const double ra = std::atan2(std::cos(e) * std::sin(l), std::cos(l));
  *dec = std::asin(std::sin(e) * std::sin(l));
  const double gmst = std::fmod(280.46061837 + 360.98564736629 * d, 360.0) * RAD;
  *hour_angle = gmst + lon_deg * RAD - ra;
}

/// The sun's elevation in degrees at a UTC time for a place (degrees, east positive)
inline double solar_elevation(double unix_s, double lat_deg, double lon_deg) {
  constexpr double RAD = 3.14159265358979323846 / 180.0;
  double dec, h;
  solar_position(unix_s, lon_deg, &dec, &h);
  const double lat = lat_deg * RAD;
  return std::asin(std::sin(lat) * std::sin(dec) + std::cos(lat) * std::cos(dec) * std::cos(h)) / RAD;
}

/// The horizon of sunrise / sunset (refraction and the sun's radius)
static constexpr double SUN_HORIZON = -0.83333;

/// "During dark": from before_s before sunset to after_s after sunrise. That holds at t exactly when the sun is below
/// the horizon somewhere in [t - after_s, t + before_s]: its lowest point there is an end, or the solar midnight in
/// between (the hour angle at 180 degrees). No day boundary; at the poles always or never dark.
inline bool dark_window(double unix_s, double lat_deg, double lon_deg, uint32_t before_s, uint32_t after_s) {
  constexpr double TWO_PI = 2 * 3.14159265358979323846;
  const double a = unix_s - after_s, b = unix_s + before_s;
  if (solar_elevation(a, lat_deg, lon_deg) < SUN_HORIZON || solar_elevation(b, lat_deg, lon_deg) < SUN_HORIZON)
    return true;
  double dec, h;
  solar_position(a, lon_deg, &dec, &h);
  // the hour angle turns once a day: the next solar midnight after a
  double to_midnight = std::fmod(TWO_PI / 2 - h, TWO_PI);
  if (to_midnight < 0)
    to_midnight += TWO_PI;
  for (double m = a + to_midnight / TWO_PI * 86400.0; m <= b; m += 86400.0) {
    if (solar_elevation(m, lat_deg, lon_deg) < SUN_HORIZON)
      return true;
  }
  return false;
}

/// The number in len digits at s (digits checked by the caller)
inline int iso_num(const char *s, int len) {
  int v = 0;
  while (len-- > 0)
    v = v * 10 + (*s++ - '0');
  return v;
}

/// An ISO-8601 time with its zone as Home Assistant sends it ("2026-10-16T05:58:41.237114+00:00", "...Z", "+02:00",
/// "+0200", "+02"; the fraction dropped): seconds since 1970. The years 1970 to 2099 (every fourth a leap year). False
/// if it isn't one (also "unknown", no seconds, no zone).
inline bool parse_iso8601(const char *s, size_t n, int64_t *out) {
  static const char FORM[] = "dddd-dd-ddTdd:dd:dd";   // (a space for the T too)
  if (n < 20)
    return false;
  for (size_t k = 0; k < 19; k++) {
    const char c = s[k];
    if (FORM[k] == 'd' ? c < '0' || c > '9' : c != FORM[k] && !(k == 10 && c == ' '))
      return false;
  }
  size_t i = 19;
  if (s[i] == '.' || s[i] == ',') {
    const size_t f = ++i;
    while (i < n && s[i] >= '0' && s[i] <= '9')
      i++;
    if (i == f)
      return false;
  }
  int zone = 0;   // the offset east of UTC, in minutes
  if (i + 1 != n || (s[i] != 'Z' && s[i] != 'z')) {
    if (i >= n || (s[i] != '+' && s[i] != '-'))
      return false;
    const char *z = s + i + 1;
    const size_t len = n - i - 1;   // HH, HHMM or HH:MM
    const size_t m = len == 5 && z[2] == ':' ? 3 : 2;
    if (len != 2 && len != 4 && m != 3)
      return false;
    for (size_t k = 0; k < len; k++) {
      if (k != 2 || m != 3) {
        if (z[k] < '0' || z[k] > '9')
          return false;
      }
    }
    const int zh = iso_num(z, 2), zm = len > 2 ? iso_num(z + m, 2) : 0;
    if (zh > 23 || zm > 59)
      return false;
    zone = (s[i] == '-' ? -1 : 1) * (zh * 60 + zm);
  }
  const int y = iso_num(s, 4), mo = iso_num(s + 5, 2), d = iso_num(s + 8, 2), h = iso_num(s + 11, 2),
            mi = iso_num(s + 14, 2), sec = iso_num(s + 17, 2);
  if (y < 1970 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || sec > 60)
    return false;
  // the day of the year (1-based) by the month's formula, then the days since 1970 (2000 a leap year as well)
  const int yday = 275 * mo / 9 - ((y & 3) == 0 ? 1 : 2) * ((mo + 9) / 12) + d - 30;
  const int days = (y - 1970) * 365 + (y - 1969) / 4 + yday - 1;
  *out = static_cast<int64_t>(days) * 86400 + h * 3600 + mi * 60 + sec - zone * 60;
  return true;
}

/// Home Assistant's sun times unusable once either is this far in the past (it moves them on at each event)
static constexpr int64_t HA_STALE_S = 600;

/// "During dark" from Home Assistant's next sunrise and next sunset (rise, set; s since 1970), without any state: from
/// before_s before the next sunset (also one that passed and wasn't moved on yet), and up to after_s after the last
/// sunrise. That is the next one while it comes before the next sunset (night: until it, and after_s on), else the
/// next one less a day: off by the sunrise's daily drift (a minute or two at mid latitudes, more toward the polar
/// circles, most around the equinoxes); none if that is still ahead (polar day: no sunrise for days).
inline bool ha_dark(int64_t now, int64_t rise, int64_t set, uint32_t before_s, uint32_t after_s) {
  if (now >= set - static_cast<int64_t>(before_s))
    return true;
  const int64_t last = rise < set ? rise : rise - 86400;
  return now < last + static_cast<int64_t>(after_s) && (rise < set || last <= now);
}

/// Where "During dark" comes from (the first that has an answer); NONE: unknown (time_unknown decides)
enum DarkSource : uint8_t { DARK_NONE = 0, DARK_HA, DARK_SUN };

/// The source now: Home Assistant's sun times while both were received and neither is stale (kept through an API
/// disconnect: they stay right until a sunrise or sunset passes unmoved), else ESPHome's sun if there is one; nothing
/// without a valid time. have: bit 0 the sunrise received, bit 1 the sunset.
inline DarkSource dark_source(bool time_valid, uint8_t have, int64_t now, int64_t rise, int64_t set, bool sun) {
  if (!time_valid)
    return DARK_NONE;
  if (have == 3 && now <= rise + HA_STALE_S && now <= set + HA_STALE_S)
    return DARK_HA;
  return sun ? DARK_SUN : DARK_NONE;
}

/// The timeline of one wake: from its start (the enable's rise, or a board showing up behind an enable already high) to
/// the first frame that carries a lit light; every figure in ms after the start. A wake the port goes through again
/// after a cut while powered (WAKING from ON) is not a new wake. The sender's DETECT is optional: without it, or while
/// it stays high (today's senders), there is no detect figure. A fall of DETECT within the grace after the start is the
/// port's own response to the enable (a sender that holds DETECT low while its port comes up), not a drop; a later fall
/// is one. The hub's lit frames start at ready (they never wait for a reply); its first poll follows >= 1 s after the
/// preamble, so on a line with a return path the first receiver reply comes after the first lit frame: both are
/// recorded, the wake's length is the first lit frame's. Nothing here changes the port's behaviour.
class WakeLog {
 public:
  static constexpr uint32_t SLOW_MS = 2500;       // a wake over this is slow (a warning)
  static constexpr uint32_t GIVE_UP_MS = 10000;   // a wake never ready by this ends here (a warning)
  static constexpr uint32_t SETTLE_MS = 5000;     // ready, but a lit frame / a reply still missing: ends this long after
                                                  // (the hub's own reply timeout)
  static constexpr int32_t NONE = -1;
  enum Outcome : uint8_t {
    OUT_LIGHT = 0,   // the first lit frame went out
    OUT_DARK,        // the wake finished with no lit frame (none wanted, or none came in time)
    OUT_CUT,         // the enable fell (the port was powered off) before it finished
    OUT_STUCK,       // never ready within GIVE_UP_MS
  };
  struct Result {
    uint32_t total{0};   // the wake's length: to the first lit frame, else to its last stage; CUT: to the cut
    int32_t detect{NONE}, lost{NONE}, ready{NONE}, reply{NONE}, light{NONE};
    uint8_t drops{0};    // DETECT falls after the grace
    Outcome outcome{OUT_LIGHT};
    bool lit{false};     // a light was wanted when the wake started
    bool silent{false};  // replies were expected and none came
    bool slow{false}, warn{false};
  };

  /// DETECT falls within this long after the start are the port's own response to the enable
  void set_grace(uint32_t ms) { this->grace_ms_ = ms; }
  /// the hub reads receiver replies (a return path with receivers): the wake waits for the first one (SETTLE_MS at most)
  void expect_reply(bool on) { this->expect_reply_ = on; }

  /// a wake starts: start_ms its zero (the enable AND detect timing starts there), lit: a light is wanted now
  void begin(uint32_t start_ms, bool lit) {
    this->r_ = Result{};
    this->r_.lit = lit;
    this->active_ = true;
    this->done_ = this->announced_ = false;
    this->t0_ = start_ms;
  }
  void detect_edge(bool high, uint32_t now_ms) {
    if (!this->active_)
      return;
    const int32_t dt = this->at_(now_ms);
    if (high) {
      if (this->r_.detect == NONE)
        this->r_.detect = dt;
    } else if (this->r_.detect != NONE || dt > static_cast<int32_t>(this->grace_ms_)) {
      if (this->r_.lost == NONE)
        this->r_.lost = dt;
      if (this->r_.drops < 255)
        this->r_.drops++;
    }
  }
  void ready(uint32_t now_ms) {
    if (this->active_ && this->r_.ready == NONE) {
      this->r_.ready = this->at_(now_ms);
      this->settle_(now_ms);
    }
  }
  /// the first receiver answer of this power-on
  void reply(uint32_t now_ms) {
    if (this->active_ && this->r_.reply == NONE) {
      this->r_.reply = this->at_(now_ms);
      this->settle_(now_ms);
    }
  }
  /// a frame with a lit light went out (counts after the wake is over, and only if a light was wanted at the start)
  void light(uint32_t now_ms) {
    if (this->light_pending()) {
      this->r_.light = this->at_(now_ms);
      this->settle_(now_ms);
    }
  }
  /// the enable fell: an unfinished wake ends here
  void power_off(uint32_t now_ms) {
    if (this->active_)
      this->finish_(now_ms, OUT_CUT);
  }
  void tick(uint32_t now_ms) {
    if (!this->active_)
      return;
    if (this->r_.ready == NONE) {
      if (this->at_(now_ms) >= static_cast<int32_t>(GIVE_UP_MS))
        this->finish_(now_ms, OUT_STUCK);
    } else {
      this->settle_(now_ms);
    }
  }

  bool active() const { return this->active_; }
  /// a lit frame is still awaited (the hub checks only then)
  bool light_pending() const { return this->active_ && this->r_.lit && this->r_.ready != NONE && this->r_.light == NONE; }
  /// a finished timeline waits to be taken
  bool done() const { return this->done_; }
  /// true once for a finished timeline (the event for the wrapper)
  bool announce() {
    if (!this->done_ || this->announced_)
      return false;
    this->announced_ = true;
    return true;
  }
  const Result &result() const { return this->r_; }
  void taken() { this->done_ = false; }

 protected:
  int32_t at_(uint32_t now_ms) const {
    const int32_t dt = static_cast<int32_t>(now_ms - this->t0_);
    return dt < 0 ? 0 : dt;
  }
  /// the wake ends once everything it waits for is in (a lit frame if one is wanted, a reply if the hub reads them), or
  /// SETTLE_MS after ready
  void settle_(uint32_t now_ms) {
    if (!this->active_ || this->r_.ready == NONE)
      return;
    const bool need_light = this->r_.lit && this->r_.light == NONE;
    const bool need_reply = this->expect_reply_ && this->r_.reply == NONE;
    if ((!need_light && !need_reply) || this->at_(now_ms) >= this->r_.ready + static_cast<int32_t>(SETTLE_MS))
      this->finish_(now_ms, this->r_.light != NONE ? OUT_LIGHT : OUT_DARK);
  }
  void finish_(uint32_t now_ms, Outcome o) {
    Result &r = this->r_;
    const int32_t dt = this->at_(now_ms);
    r.outcome = o;
    r.silent = this->expect_reply_ && r.reply == NONE && o != OUT_CUT && o != OUT_STUCK;
    switch (o) {
      case OUT_LIGHT:
        r.total = static_cast<uint32_t>(r.light);
        break;
      case OUT_DARK:
        r.total = static_cast<uint32_t>(r.reply > r.ready ? r.reply : r.ready);
        break;
      default:
        r.total = static_cast<uint32_t>(dt);
        break;
    }
    r.slow = o != OUT_CUT && r.total > SLOW_MS;
    r.warn = r.slow || r.drops > 0 || o == OUT_STUCK || (r.silent && r.lit);
    this->active_ = false;
    this->done_ = true;
  }

  Result r_{};
  bool active_{false}, done_{false}, announced_{false}, expect_reply_{false};
  uint32_t t0_{0}, grace_ms_{100};
};

/// The wake timeline as one line (buf; "No wake yet." for none): "Wake 1905 ms: detect 1240 ms, ready 1700 ms, first reply
/// 1820 ms, first light 1905 ms." The figures in time order; "(slow, over 2500 ms)" after the length of a slow wake,
/// "detect N ms lost" for a DETECT drop, "no light turned on" for a wake that ended with no lit frame, "no receiver
/// answered" where replies were expected and none came, "the port was powered off" for a wake cut short ("Wake
/// interrupted at N ms"), "the port never became ready" for one that never did.
inline const char *wake_text(const WakeLog::Result *r, char *buf, size_t n) {
  if (r == nullptr) {
    snprintf(buf, n, "No wake yet.");
    return buf;
  }
  struct Item {
    int32_t t;
    const char *fmt;
  };
  Item items[5];
  size_t k = 0;
  auto add = [&](int32_t t, const char *fmt) {
    if (t == WakeLog::NONE)
      return;
    size_t i = k++;
    while (i > 0 && items[i - 1].t > t) {
      items[i] = items[i - 1];
      i--;
    }
    items[i] = Item{t, fmt};
  };
  add(r->lost, "detect %d ms lost");
  add(r->detect, "detect %d ms");
  add(r->ready, "ready %d ms");
  add(r->reply, "first reply %d ms");
  add(r->light, "first light %d ms");
  size_t len = 0;
  auto put = [&](const char *fmt, int v) {
    if (len + 1 < n)
      len += static_cast<size_t>(snprintf(buf + len, n - len, fmt, v));
  };
  if (r->outcome == WakeLog::OUT_CUT)
    put("Wake interrupted at %d ms", static_cast<int>(r->total));
  else
    put("Wake %d ms", static_cast<int>(r->total));
  if (r->slow)
    put(" (slow, over %d ms)", static_cast<int>(WakeLog::SLOW_MS));
  const char *tails[2];
  size_t nt = 0;
  if (r->outcome == WakeLog::OUT_CUT)
    tails[nt++] = "the port was powered off";
  else if (r->outcome == WakeLog::OUT_STUCK)
    tails[nt++] = "the port never became ready";
  else {
    if (r->light == WakeLog::NONE)
      tails[nt++] = "no light turned on";
    if (r->silent)
      tails[nt++] = "no receiver answered";
  }
  size_t shown = 0;
  for (size_t i = 0; i < k; i++, shown++) {
    put(shown == 0 ? ": " : ", ", 0);
    put(items[i].fmt, items[i].t);
  }
  for (size_t i = 0; i < nt; i++, shown++) {
    put(shown == 0 ? ": " : ", ", 0);
    if (len + 1 < n)
      len += static_cast<size_t>(snprintf(buf + len, n - len, "%s", tails[i]));
  }
  put(".", 0);
  return buf;
}

class PortCore {
 public:
  void configure(const PortConfig &c) {
    this->cfg_ = c;
    if (this->cfg_.notice_max_ms < this->cfg_.notice_ms)
      this->cfg_.notice_max_ms = this->cfg_.notice_ms;   // (the cap never cuts a notice short)
    this->wake_.set_grace(this->cfg_.debounce_ms);
  }
  const PortConfig &config() const { return this->cfg_; }

  /// A light (or another load) on the port; hold_ms: its turn-off period (0: the port's). Returns its index.
  uint16_t add_load(uint32_t hold_ms) {
    this->loads_.push_back(Load{hold_ms > MAX_HOLD_MS ? MAX_HOLD_MS : hold_ms, false, false, 0});
    return static_cast<uint16_t>(this->loads_.size() - 1);
  }
  uint16_t loads() const { return static_cast<uint16_t>(this->loads_.size()); }
  /// whether load i is on now (polled by the wrapper)
  void set_lit(uint16_t i, bool lit, uint32_t now_ms) {
    if (i >= this->loads_.size())
      return;
    Load &l = this->loads_[i];
    if (l.lit && !lit) {
      l.off_ms = now_ms;
      l.held = this->mode_ != MODE_OFF;   // its turn-off period runs from now (Off: none)
    }
    l.lit = lit;
  }
  /// The power mode (Off: never powered, and nothing from before powers it once another mode is chosen)
  void set_mode(PowerMode m) {
    this->mode_ = m < MODE_COUNT ? m : MODE_STANDBY;
    if (this->mode_ == MODE_OFF)
      this->forget_();
  }
  PowerMode mode() const { return this->mode_; }
  bool off() const { return this->mode_ == MODE_OFF; }
  /// The dark window now (known: the controller has a valid time)
  void set_dark(bool dark, bool known = true) {
    this->dark_ = dark;
    this->dark_known_ = known;
  }
  bool dark() const { return this->dark_; }
  /// Pre-wake (motion): wanted for timeout_ms (min_on_ms to 24 h; longer if a light turns on meanwhile)
  void prewake(uint32_t now_ms, uint32_t timeout_ms) {
    if (this->mode_ == MODE_OFF)
      return;   // (dropped, not kept for later)
    timeout_ms = timeout_ms < this->cfg_.min_on_ms ? this->cfg_.min_on_ms : timeout_ms;
    timeout_ms = timeout_ms > MAX_HOLD_MS ? MAX_HOLD_MS : timeout_ms;
    // (a start and a length, compared as elapsed time: right even with updates days apart; one that ran out before
    // this update came is over, whatever its length)
    const uint32_t elapsed = now_ms - this->prewake_from_;
    if (!this->prewake_ || elapsed >= this->prewake_ms_ || timeout_ms > this->prewake_ms_ - elapsed) {
      this->prewake_from_ = now_ms;
      this->prewake_ms_ = timeout_ms;
    }
    this->prewake_ = true;
  }
  /// The detect input's raw level: the power gate as it is (any dropout cuts the 55 V), and debounced for the board's
  /// presence (the first reading counts at once)
  void set_detect(bool raw, uint32_t now_ms) {
    if (!this->detect_seen_) {
      this->detect_seen_ = true;
      this->raw_ = this->board_ = raw;
      this->detect_change_ms_ = now_ms;
      if (raw && this->enable_)
        this->conj_ms_ = now_ms;
      this->line_note_(now_ms);
      return;
    }
    if (raw != this->raw_) {
      this->raw_ = raw;
      this->detect_change_ms_ = now_ms;
      if (!raw)
        this->raw_lost_ = true;
      else if (this->enable_)
        this->conj_ms_ = now_ms;   // the hot-plug delay from here
      this->wake_.detect_edge(raw, now_ms);
      this->line_note_(now_ms);
    }
    if (raw != this->board_ && now_ms - this->detect_change_ms_ >= this->cfg_.debounce_ms)
      this->board_ = raw;
  }

  /// Whether the port should be powered now; retires every finished timer first (whatever else wants power: a
  /// timer left alone for 24.8 days would come back to life)
  bool want_power(uint32_t now_ms) {
    if (this->mode_ == MODE_OFF) {
      this->forget_();
      return false;
    }
    const bool mode_on = this->mode_ == MODE_ALWAYS_ON ||
                         (this->mode_ == MODE_DURING_DARK && (this->dark_known_ ? this->dark_ : this->cfg_.unknown_on));
    if (mode_on) {
      this->mode_on_ = true;
    } else if (this->mode_on_) {   // the mode: like a light (its turn-off period once Always on or the dark ends)
      this->mode_on_ = false;
      this->mode_off_ms_ = now_ms;
      this->mode_held_ = true;
    }
    if (this->mode_held_ && now_ms - this->mode_off_ms_ >= this->hold_(0))
      this->mode_held_ = false;
    if (this->prewake_ && now_ms - this->prewake_from_ >= this->prewake_ms_)
      this->prewake_ = false;
    bool want = mode_on || this->mode_held_ || this->prewake_;
    for (Load &l : this->loads_) {
      if (l.held && !l.lit && now_ms - l.off_ms >= this->hold_(l.hold_ms))
        l.held = false;
      want |= l.lit || l.held;
    }
    return want && !this->shutting_down_;
  }

  /// Advance the state machine; returns PortEvent bits.
  uint8_t update(uint32_t now_ms) {
    const bool want = this->want_power(now_ms);
    const PortState was = this->state_;
    const bool enable_was = this->enable_, problem_was = this->problem_;
    this->account_(now_ms);
    uint8_t ev = 0;
    this->wake_.tick(now_ms);
    if (this->state_ == PS_OFF && !this->shutting_down_)   // the first update: where it rests now
      this->rest_(now_ms);
    switch (this->state_) {
      case PS_NO_BOARD:
        this->enable_on_(want, now_ms);   // (a board plugged in now gets its power)
        this->problem_ = want;
        if (this->board_) {
          if (want)
            this->waking_();
          else
            this->rest_(now_ms);
        }
        break;
      case PS_STANDBY:
        if (!this->board_) {
          this->rest_(now_ms);
        } else if (want) {
          this->enable_on_(true, now_ms);
          this->waking_();
        }
        break;
      case PS_WAKING:
        if (!this->board_) {
          this->state_ = this->rewake_ ? PS_FAULT : PS_NO_BOARD;   // (lost after a cut while ON: still that problem)
          this->problem_ = this->rewake_ || want;
          this->rewake_ = false;
          this->enable_on_(want, now_ms);
        } else if (this->mode_ == MODE_OFF) {
          // Off: the enable low now (no notice: the boards had no frame yet; ones that booted see a power cut)
          this->enable_ = false;
          this->rewake_ = false;
          this->hold_off_(now_ms, this->cfg_.min_off_ms);
        } else if (this->powered_(now_ms)) {
          this->state_ = PS_ON;
          this->on_ms_ = now_ms;
          this->problem_ = false;
          this->wakes_++;
          this->wake_.ready(now_ms);
          ev |= EV_READY;
        }
        break;
      case PS_ON: {
        const bool recheck = this->recheck_due_();
        if (!this->raw_ || this->raw_lost_) {   // the 55 V cut (a dropout, or the board gone): the boards lost power
          this->preparing_ = false;
          this->drops_++;
          ev |= EV_LOST | EV_COUNTERS;
          if (!this->board_) {
            this->state_ = PS_FAULT;
            this->problem_ = true;
            this->enable_on_(want, now_ms);
          } else {
            this->state_ = PS_WAKING;   // powered again hotplug + boot after its return
            this->rewake_ = true;
          }
        } else if ((!want && (this->mode_ == MODE_OFF || now_ms - this->on_ms_ >= this->cfg_.min_on_ms)) || recheck) {
          // (its hubs read the receivers' counters once more first: prepare_max_ms at most)
          if (this->cfg_.prepare_max_ms == 0 ||
              (this->preparing_ && (this->notice_ready_ || now_ms - this->prep_ms_ >= this->cfg_.prepare_max_ms))) {
            this->preparing_ = false;
            this->notice_(now_ms);
            this->recheck_ = recheck;   // (Always on: back on after recheck_off_ms)
            this->lit_since_wake_ = false;
            ev |= this->copy_(now_ms);
          } else if (!this->preparing_) {
            this->preparing_ = true;
            this->prep_ms_ = now_ms;
            this->notice_ready_ = false;
            ev |= EV_PREPARE;
          }
        } else {
          this->preparing_ = false;   // (wanted again: no power-off)
          for (const Load &l : this->loads_)
            this->lit_since_wake_ |= l.lit;
        }
        break;
      }
      case PS_NOTICE: {
        this->wake_pending_ |= want;
        const uint32_t spread = (this->cfg_.notice_copies - 1u) * this->cfg_.notice_spacing_ms;
        const uint32_t tail = this->cfg_.notice_ms > spread ? this->cfg_.notice_ms - spread : 0;
        if ((this->delivered_ >= this->cfg_.notice_copies && now_ms - this->last_out_ms_ >= tail) ||
            now_ms - this->notice_ms_ >= this->cfg_.notice_max_ms) {
          this->enable_ = false;
          this->pending_ = false;
          const uint32_t off = this->recheck_ && this->cfg_.recheck_off_ms > this->cfg_.min_off_ms ? this->cfg_.recheck_off_ms
                                                                                                 : this->cfg_.min_off_ms;
          this->hold_off_(now_ms, off);
          this->standby_cycles_++;
          ev |= EV_COUNTERS;
        } else {
          ev |= this->copy_(now_ms);
        }
        break;
      }
      case PS_OFF_HOLD:
        this->wake_pending_ |= want;
        if (now_ms - this->off_ms_ >= this->off_hold_ms_) {
          if (this->board_ && (this->wake_pending_ || want)) {
            this->enable_on_(true, now_ms);
            this->waking_();
          } else {
            this->rest_(now_ms);
          }
        }
        break;
      case PS_FAULT:
        if (this->mode_ == MODE_OFF) {   // (Off: at rest, not a problem any more; its return wakes nothing)
          this->rest_(now_ms);
          break;
        }
        this->enable_on_(want, now_ms);
        if (this->board_) {
          if (want)
            this->waking_();
          else
            this->rest_(now_ms);
        }
        break;
      default:
        break;
    }
    this->raw_lost_ = false;
    this->line_note_(now_ms);
    // the wake timeline: a wake starts when the port begins waking from a rest (not again after a cut while ON, nor
    // while one is running: a board that flaps behind a high enable is the same wake), and ends when the enable falls
    if (this->state_ == PS_WAKING && was != PS_WAKING && was != PS_ON && !this->wake_.active())
      this->wake_begin_(now_ms);
    if (!this->enable_ && enable_was)
      this->wake_.power_off(now_ms);
    if (this->state_ != was || this->problem_ != problem_was)
      ev |= EV_STATE;
    if (this->enable_ != enable_was)
      ev |= EV_ENABLE;
    if (this->wake_.announce())
      ev |= EV_WAKE;
    return ev;
  }

  /// Before a planned power-off (EV_PREPARE): its hubs are ready for the notice (their last reads done)
  void set_notice_ready(bool ready) { this->notice_ready_ = ready; }
  bool preparing() const { return this->preparing_; }

  /// The notice's copy EV_NOTICE asked for went out (the Ultra hub: the frame with it handed to the transmitter; a
  /// port without one: at once). The next one is due notice_spacing_ms from now.
  void notice_sent(uint32_t now_ms) {
    if (this->state_ != PS_NOTICE || !this->pending_)
      return;
    this->pending_ = false;
    this->delivered_++;
    this->last_out_ms_ = now_ms;
  }

  /// A deliberate power cycle (firmware recovery): the enable low now, whatever wants power (no notice: the update
  /// session owns the line), cycle_off_ms off, then a wake; not a standby cycle.
  uint8_t power_cycle(uint32_t now_ms) {
    const PortState was = this->state_;
    const bool enable_was = this->enable_;
    if (this->state_ == PS_OFF || this->mode_ == MODE_OFF)
      return 0;   // (Off: the port stays without power)
    this->enable_ = false;
    this->pending_ = false;
    this->problem_ = false;
    this->rewake_ = false;
    this->hold_off_(now_ms, this->cfg_.cycle_off_ms > this->cfg_.min_off_ms ? this->cfg_.cycle_off_ms
                                                                            : this->cfg_.min_off_ms);
    this->wake_pending_ = true;
    this->line_note_(now_ms);
    this->wake_.power_off(now_ms);
    return (this->state_ != was ? EV_STATE : 0) | (enable_was ? EV_ENABLE : 0) | (this->wake_.announce() ? EV_WAKE : 0);
  }
  /// How long a power cycle takes until the boards have power again
  uint32_t power_cycle_ms() const {
    const uint32_t off = this->cfg_.cycle_off_ms > this->cfg_.min_off_ms ? this->cfg_.cycle_off_ms : this->cfg_.min_off_ms;
    return off + this->cfg_.hotplug_ms + this->cfg_.boot_margin_ms;
  }

  /// The controller shuts down (OTA, a reboot): nothing wants power from now on. A port in ON goes to NOTICE (its
  /// first copy: EV_NOTICE); one already in NOTICE goes on with its own. update() sends the others and drops the
  /// enable as usual; power_down() at the end.
  uint8_t shutdown(uint32_t now_ms) {
    this->shutting_down_ = true;
    if (this->state_ != PS_ON)
      return 0;
    this->notice_(now_ms);
    return EV_STATE | this->copy_(now_ms);
  }
  /// The end of a shutdown: the enable low now, Off (a port still in NOTICE: its planned power-off counted).
  uint8_t power_down(uint32_t now_ms) {
    this->account_(now_ms);
    uint8_t ev = 0;
    if (this->state_ == PS_NOTICE) {
      this->standby_cycles_++;
      ev |= EV_COUNTERS;
    }
    if (this->enable_)
      ev |= EV_ENABLE;
    this->enable_ = false;
    if (this->state_ != PS_OFF)
      ev |= EV_STATE;
    this->state_ = PS_OFF;
    this->line_note_(now_ms);
    this->wake_.power_off(now_ms);
    return ev | (this->wake_.announce() ? EV_WAKE : 0);
  }

  /// The wake timeline (WakeLog): the hub tells the first receiver reply of this power-on and each frame it sends with a
  /// lit light; wake().result() once EV_WAKE came (then wake_taken())
  void wake_reply(uint32_t now_ms) { this->wake_.reply(now_ms); }
  void wake_light(uint32_t now_ms) { this->wake_.light(now_ms); }
  /// the hub reads receiver replies: a wake waits for the first one (before a lit frame counts)
  void wake_expect_reply(bool on) { this->wake_.expect_reply(on); }
  bool wake_light_pending() const { return this->wake_.light_pending(); }
  const WakeLog &wake() const { return this->wake_; }
  void wake_taken() { this->wake_.taken(); }

  PortState state() const { return this->state_; }
  bool enable() const { return this->enable_; }
  /// powered and the wake over: the lights' data reaches the boards
  bool ready() const { return this->state_ == PS_ON; }
  /// the line's 55 V (hotplug_ms after enable AND detect, until either drops: update() sees it come) and since when
  /// (ms: its power-on, its cut): the hubs' lifetime ledger counts the boards' powered time by it
  bool line_on() const { return this->line_; }
  uint32_t line_ms() const { return this->line_ms_; }
  /// a board detected (debounced)
  bool board() const { return this->board_; }
  /// FAULT (detect lost while ON, until it is back), or no board while a light wants the port
  bool problem() const { return this->problem_; }
  /// the copy EV_NOTICE asked for (0: the first, a new SEQ)
  uint8_t notice_copy() const { return this->delivered_; }
  uint32_t standby_cycles() const { return this->standby_cycles_; }
  uint32_t drops() const { return this->drops_; }
  /// seconds in standby (STANDBY, OFF_HOLD) in total, up to the last update
  uint32_t standby_s() const { return this->standby_s_; }
  uint32_t wakes() const { return this->wakes_; }
  /// the persisted counters, at boot
  void set_counters(uint32_t cycles, uint32_t standby_s, uint32_t drops) {
    this->standby_cycles_ = cycles;
    this->standby_s_ = standby_s;
    this->drops_ = drops;
  }

 protected:
  struct Load {
    uint32_t hold_ms;
    bool lit, held;
    uint32_t off_ms;
  };
  /// Off: no turn-off period, pre-wake or wake request left (none powers the port once another mode is chosen)
  void forget_() {
    this->mode_on_ = this->mode_held_ = false;
    this->prewake_ = false;
    this->wake_pending_ = false;
    for (Load &l : this->loads_)
      l.held = false;
  }
  /// a turn-off period (0: the port's), min_on_ms to 24 h
  uint32_t hold_(uint32_t hold_ms) const {
    uint32_t h = hold_ms != 0 ? hold_ms : this->cfg_.standby_after_ms;
    h = h < this->cfg_.min_on_ms ? this->cfg_.min_on_ms : h;
    return h > MAX_HOLD_MS ? MAX_HOLD_MS : h;
  }
  void enable_on_(bool on, uint32_t now_ms) {
    if (on && !this->enable_ && this->raw_)
      this->conj_ms_ = now_ms;   // enable AND detect from now: the hot-plug delay from here
    this->enable_ = on;
  }
  /// enable AND detect for hotplug + boot margin, without a dropout
  bool powered_(uint32_t now_ms) const {
    return this->enable_ && this->raw_ && !this->raw_lost_ &&
           now_ms - this->conj_ms_ >= this->cfg_.hotplug_ms + this->cfg_.boot_margin_ms;
  }
  /// WAKING (a board is there: no problem); powered_() times it. The wake re-qualifies the port (the sender's check)
  void waking_() {
    this->state_ = PS_WAKING;
    this->problem_ = false;
    this->lit_since_wake_ = false;
  }
  /// Always on: every light dark and its turn-off period over, after one was lit since the wake, no pre-wake
  bool recheck_due_() const {
    if (this->mode_ != MODE_ALWAYS_ON || !this->lit_since_wake_ || this->prewake_ || this->shutting_down_)
      return false;
    for (const Load &l : this->loads_)
      if (l.lit || l.held)
        return false;
    return true;
  }
  /// A wake starts: from the enable AND detect timing (conj_ms_: the enable's rise, or the board's detect behind an
  /// enable already high), a light wanted now or not
  void wake_begin_(uint32_t now_ms) {
    bool lit = false;
    for (const Load &l : this->loads_)
      lit |= l.lit;
    const uint32_t start = static_cast<int32_t>(now_ms - this->conj_ms_) >= 0 ? this->conj_ms_ : now_ms;
    this->wake_.begin(start, lit);
  }
  /// STANDBY or NO_BOARD, the enable low
  void rest_(uint32_t now_ms) {
    this->enable_ = false;
    this->problem_ = false;
    this->rewake_ = false;
    this->state_ = this->board_ ? PS_STANDBY : PS_NO_BOARD;
    this->acc_ms_ = now_ms;
  }
  void notice_(uint32_t now_ms) {
    this->state_ = PS_NOTICE;
    this->recheck_ = false;
    this->notice_ms_ = now_ms;
    this->delivered_ = 0;
    this->pending_ = false;
    this->wake_pending_ = false;
  }
  /// OFF_HOLD for off_ms
  void hold_off_(uint32_t now_ms, uint32_t off_ms) {
    this->state_ = PS_OFF_HOLD;
    this->off_ms_ = now_ms;
    this->off_hold_ms_ = off_ms;
    this->acc_ms_ = now_ms;
  }
  /// the next notice copy, when due: none waiting to go out, spacing after the last one out
  uint8_t copy_(uint32_t now_ms) {
    if (this->pending_ || this->delivered_ >= this->cfg_.notice_copies ||
        (this->delivered_ != 0 && now_ms - this->last_out_ms_ < this->cfg_.notice_spacing_ms))
      return 0;
    this->pending_ = true;
    return EV_NOTICE;
  }
  /// the line's 55 V changed: timed (on: the hot-plug delay after enable AND detect)
  void line_note_(uint32_t now_ms) {
    const bool on = this->enable_ && this->raw_ && now_ms - this->conj_ms_ >= this->cfg_.hotplug_ms;
    if (on != this->line_) {
      this->line_ = on;
      this->line_ms_ = on ? this->conj_ms_ + this->cfg_.hotplug_ms : now_ms;
    }
  }
  /// time in standby
  void account_(uint32_t now_ms) {
    if (this->state_ != PS_STANDBY && this->state_ != PS_OFF_HOLD)
      return;
    this->standby_ms_ += now_ms - this->acc_ms_;
    this->acc_ms_ = now_ms;
    this->standby_s_ += this->standby_ms_ / 1000;
    this->standby_ms_ %= 1000;
  }

  PortConfig cfg_{};
  std::vector<Load> loads_;
  PowerMode mode_{MODE_STANDBY};
  bool dark_{false}, dark_known_{false};
  bool mode_on_{false}, mode_held_{false};
  uint32_t mode_off_ms_{0};
  bool prewake_{false};
  uint32_t prewake_from_{0}, prewake_ms_{0};
  bool detect_seen_{false}, raw_{false}, raw_lost_{false}, board_{false};
  uint32_t detect_change_ms_{0}, conj_ms_{0};
  PortState state_{PS_OFF};
  bool enable_{false};
  bool line_{false};
  uint32_t line_ms_{0};
  uint32_t on_ms_{0}, notice_ms_{0}, last_out_ms_{0}, off_ms_{0}, off_hold_ms_{0};
  uint8_t delivered_{0};
  bool pending_{false}, wake_pending_{false}, shutting_down_{false}, rewake_{false};
  bool preparing_{false}, notice_ready_{false};
  bool lit_since_wake_{false}, recheck_{false};   // Always on: a light lit since the last wake; this power-off is a re-check
  uint32_t prep_ms_{0};
  bool problem_{false};
  uint32_t standby_cycles_{0}, drops_{0}, standby_s_{0}, standby_ms_{0}, acc_ms_{0}, wakes_{0};
  WakeLog wake_;
};

/// The port's status text: "No board", "Standby", "Waking", "On", "Powering off", "Connection lost (N drops)", "Off"
/// (buf: for the drops); in the power mode Off at rest: "Off", "Off (no board)"
inline const char *port_state_text(PortState s, uint32_t drops, char *buf, size_t n, bool off = false) {
  if (off && (s == PS_STANDBY || s == PS_NO_BOARD))
    return s == PS_STANDBY ? "Off" : "Off (no board)";
  switch (s) {
    case PS_NO_BOARD:
      return "No board";
    case PS_STANDBY:
      return "Standby";
    case PS_WAKING:
      return "Waking";
    case PS_ON:
      return "On";
    case PS_NOTICE:
    case PS_OFF_HOLD:
      return "Powering off";
    case PS_FAULT:
      snprintf(buf, n, "Connection lost (%u drop%s)", static_cast<unsigned>(drops), drops == 1 ? "" : "s");
      return buf;
    default:
      return "Off";
  }
}

}  // namespace esphome::diffpower
