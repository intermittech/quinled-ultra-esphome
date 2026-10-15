#pragma once

// A QuinLED Diff-Power sender port's power manager, without ESPHome (tests/diffpower_test.cpp runs it with simulated
// time). Wiki: Diff-Power ports. The design: review-r8/fable-standby §5, with review-r8/portmgr-gpt's fixes.
//
// The hardware: per port an enable output and a detect input. The port carries ~4.5 V / 100 mA (standby, the board
// detection) all the time; its 55 V needs enable AND detect (the sender's AND gate) and arrives ~1.5 s after both
// (the sender's hot-plug delay); any dropout of either cuts it and restarts that delay. The boards behind it then boot;
// a Diff-Power receiver (0.6.10) stays dark until the hub's first frame (or shows its power-on scene after 1.5 s).
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
// Counters for the wrapper to persist (EV_COUNTERS): standby cycles (planned power-offs), detect drops (power
// interruptions while ON), seconds in standby (STANDBY and OFF_HOLD).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace esphome::diffpower {

enum PowerMode : uint8_t { MODE_STANDBY = 0, MODE_DURING_DARK, MODE_ALWAYS_ON, MODE_COUNT };
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

class PortCore {
 public:
  void configure(const PortConfig &c) {
    this->cfg_ = c;
    if (this->cfg_.notice_max_ms < this->cfg_.notice_ms)
      this->cfg_.notice_max_ms = this->cfg_.notice_ms;   // (the cap never cuts a notice short)
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
      l.held = true;   // its turn-off period runs from now
    }
    l.lit = lit;
  }
  void set_mode(PowerMode m) { this->mode_ = m < MODE_COUNT ? m : MODE_STANDBY; }
  PowerMode mode() const { return this->mode_; }
  /// The dark window now (known: the controller has a valid time)
  void set_dark(bool dark, bool known = true) {
    this->dark_ = dark;
    this->dark_known_ = known;
  }
  bool dark() const { return this->dark_; }
  /// Pre-wake (motion): wanted for timeout_ms (min_on_ms to 24 h; longer if a light turns on meanwhile)
  void prewake(uint32_t now_ms, uint32_t timeout_ms) {
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
      return;
    }
    if (raw != this->raw_) {
      this->raw_ = raw;
      this->detect_change_ms_ = now_ms;
      if (!raw)
        this->raw_lost_ = true;
      else if (this->enable_)
        this->conj_ms_ = now_ms;   // the hot-plug delay from here
    }
    if (raw != this->board_ && now_ms - this->detect_change_ms_ >= this->cfg_.debounce_ms)
      this->board_ = raw;
  }

  /// Whether the port should be powered now; retires every finished timer first (whatever else wants power: a
  /// timer left alone for 24.8 days would come back to life)
  bool want_power(uint32_t now_ms) {
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
        } else if (this->powered_(now_ms)) {
          this->state_ = PS_ON;
          this->on_ms_ = now_ms;
          this->problem_ = false;
          this->wakes_++;
          ev |= EV_READY;
        }
        break;
      case PS_ON:
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
        } else if (!want && now_ms - this->on_ms_ >= this->cfg_.min_on_ms) {
          // (its hubs read the receivers' counters once more first: prepare_max_ms at most)
          if (this->cfg_.prepare_max_ms == 0 ||
              (this->preparing_ && (this->notice_ready_ || now_ms - this->prep_ms_ >= this->cfg_.prepare_max_ms))) {
            this->preparing_ = false;
            this->notice_(now_ms);
            ev |= this->copy_(now_ms);
          } else if (!this->preparing_) {
            this->preparing_ = true;
            this->prep_ms_ = now_ms;
            this->notice_ready_ = false;
            ev |= EV_PREPARE;
          }
        } else {
          this->preparing_ = false;   // (wanted again: no power-off)
        }
        break;
      case PS_NOTICE: {
        this->wake_pending_ |= want;
        const uint32_t spread = (this->cfg_.notice_copies - 1u) * this->cfg_.notice_spacing_ms;
        const uint32_t tail = this->cfg_.notice_ms > spread ? this->cfg_.notice_ms - spread : 0;
        if ((this->delivered_ >= this->cfg_.notice_copies && now_ms - this->last_out_ms_ >= tail) ||
            now_ms - this->notice_ms_ >= this->cfg_.notice_max_ms) {
          this->enable_ = false;
          this->pending_ = false;
          this->hold_off_(now_ms, this->cfg_.min_off_ms);
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
    if (this->state_ != was || this->problem_ != problem_was)
      ev |= EV_STATE;
    if (this->enable_ != enable_was)
      ev |= EV_ENABLE;
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
    if (this->state_ == PS_OFF)
      return 0;
    this->enable_ = false;
    this->pending_ = false;
    this->problem_ = false;
    this->rewake_ = false;
    this->hold_off_(now_ms, this->cfg_.cycle_off_ms > this->cfg_.min_off_ms ? this->cfg_.cycle_off_ms
                                                                            : this->cfg_.min_off_ms);
    this->wake_pending_ = true;
    return (this->state_ != was ? EV_STATE : 0) | (enable_was ? EV_ENABLE : 0);
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
    return ev;
  }

  PortState state() const { return this->state_; }
  bool enable() const { return this->enable_; }
  /// powered and the wake over: the lights' data reaches the boards
  bool ready() const { return this->state_ == PS_ON; }
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
  /// WAKING (a board is there: no problem); powered_() times it
  void waking_() {
    this->state_ = PS_WAKING;
    this->problem_ = false;
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
  uint32_t on_ms_{0}, notice_ms_{0}, last_out_ms_{0}, off_ms_{0}, off_hold_ms_{0};
  uint8_t delivered_{0};
  bool pending_{false}, wake_pending_{false}, shutting_down_{false}, rewake_{false};
  bool preparing_{false}, notice_ready_{false};
  uint32_t prep_ms_{0};
  bool problem_{false};
  uint32_t standby_cycles_{0}, drops_{0}, standby_s_{0}, standby_ms_{0}, acc_ms_{0}, wakes_{0};
};

/// The port's status text: "No board", "Standby", "Waking", "On", "Powering off", "Connection lost (N drops)", "Off"
/// (buf: for the drops)
inline const char *port_state_text(PortState s, uint32_t drops, char *buf, size_t n) {
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
