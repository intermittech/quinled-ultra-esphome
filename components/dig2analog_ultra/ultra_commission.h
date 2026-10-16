#pragma once

// The commissioning of a QuinLED Diff-Power port's Ultra line with telemetry (wiki: Commissioning). Commission: the
// board detected, the 55 V, every receiver answering (its UID16 and firmware), the cable under a test load. Finalize:
// the first three, the current calibration of every board, a full read of every receiver and the final checks; its
// result is stored. No ESPHome here (tests/ultra_telemetry_test.cpp runs it on a simulated line): the hub calls poll()
// every loop with its port's state and does what it asks for (keep the port powered, a test level on the line's
// lights, the calibration, store the record, publish the texts).

#include "ultra_telemetry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace esphome::dig2analog_ultra {

/// The port as the commissioning sees it now
struct CommissionPort {
  bool present{false};   // a Diff-Power port
  bool detect{false};    // ... with a detect input
  bool off{false};       // ... its power mode Off
  bool board{false};     // ... a board detected
  bool fault{false};     // ... the connection lost while powered
  bool ready{false};     // the boards powered, the wake over (the hub's frames go out)
};

/// The last Finalize's result, kept on the ESP32 (one per line)
struct CommissionRecord {
  static constexpr uint32_t MAGIC = 0x31534D43;   // "CMS1"
  uint32_t magic{0};
  uint32_t date{0};                // s since 1970 (0: no valid time then)
  uint32_t fw_min{0}, fw_max{0};   // the receivers' firmware (STATUS: major << 16 | minor << 8 | patch)
  uint8_t passed{0}, receivers{0};
  uint16_t uid[TelemetryPoller::MAX_RECEIVERS]{};
};

/// "2026-10-16" from s since 1970
inline std::string commission_date(uint32_t unix_s) {
  if (unix_s == 0)
    return "date unknown";
  // the civil date of a day number (H. Hinnant's days_from_civil, inverted)
  const int32_t z = static_cast<int32_t>(unix_s / 86400) + 719468;
  const int32_t era = z / 146097, doe = z - era * 146097;
  const int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100), mp = (5 * doy + 2) / 153;
  const int32_t d = doy - (153 * mp + 2) / 5 + 1, m = mp < 10 ? mp + 3 : mp - 9;
  char b[40];   // (room for any int the compiler can see: no -Wformat-truncation)
  snprintf(b, sizeof(b), "%04d-%02d-%02d", static_cast<int>(yoe + era * 400 + (m <= 2)), static_cast<int>(m),
           static_cast<int>(d));
  return b;
}

/// "0.6.10" (STATUS's major << 16 | minor << 8 | patch; 0: a development build)
inline std::string commission_fw(uint32_t v) {
  if (v == 0)
    return "dev";
  char b[16];
  snprintf(b, sizeof(b), "%u.%u.%u", static_cast<unsigned>(v >> 16), static_cast<unsigned>((v >> 8) & 0xFF),
           static_cast<unsigned>(v & 0xFF));
  return b;
}

class Commissioning {
 public:
  enum Kind : uint8_t { COMMISSION = 0, FINALIZE };
  enum Phase : uint8_t { IDLE = 0, QUEUED, BOARD, POWER, ANSWER, LOAD, RESTORE, CALIBRATE, READ };
  /// poll()'s requests to the hub (bits)
  enum Act : uint8_t {
    ACT_TEXT = 1,        // the texts or the commissioned state changed: publish them
    ACT_LEVEL = 2,       // the line's lights at level() (0: back to their own state)
    ACT_CALIBRATE = 4,   // start the current calibration of every board
    ACT_STORE = 8,       // store record()
    ACT_DONE = 16,       // the run is over (the next line's turn)
  };
  static constexpr uint32_t BOARD_MS = 3000;     // a board detected within this (a port with a detect input)
  static constexpr uint32_t POWER_MS = 10000;    // ... powered, the wake over, within this
  static constexpr uint32_t SETTLE_MS = 300;     // a new test level: the STATUS reads count from this after it
  static constexpr uint32_t CAL_START_MS = 5000; // the calibration starts within this
  static constexpr float TARGET_LOAD = 40.0f;    // the load test stops at this % of port_limit
  static constexpr float MIN_DELTA_A = 0.2f;     // a segment's resistance from this change of its current on
                                                 // (the 10 mV voltage steps: ~0.1 ohm of error at 0.2 A)
  static constexpr uint8_t LEVELS = 5;           // the test levels (brightness; ~3 / 14 / 37 / 63 / 100 % duty)
  static constexpr size_t REPORT_MAX = 250;      // (Home Assistant keeps 255 characters of a state)

  /// status_ms: the line's STATUS interval (the waits grow with it); min_voltage: the share of the sender's voltage
  /// every receiver must keep at the port's full load (the load test's extrapolation)
  void configure(uint32_t status_ms, float min_voltage) {
    this->answer_ms_ = std::max<uint32_t>(20000, 3 * status_ms + 5000);
    this->step_ms_ = std::max<uint32_t>(10000, 3 * status_ms + 2000);
    this->min_voltage_ = min_voltage;
  }
  void set_record(const CommissionRecord &r) {
    if (r.magic == CommissionRecord::MAGIC)
      this->record_ = r;
    this->idle_due_ = true;
  }
  const CommissionRecord &record() const { return this->record_; }

  /// A run asked for (a button, an action): it waits for its turn (one line at a time on the controller)
  bool request(Kind kind) {
    if (this->phase_ != IDLE)
      return false;
    this->kind_ = kind;
    this->phase_ = QUEUED;
    this->status_ = std::string(kind == FINALIZE ? "Finalize" : "Commission") + " queued";
    this->changed_ = true;
    return true;
  }
  /// Its turn: the run starts
  void start(uint32_t now_ms) {
    this->phase_ = BOARD;
    this->t_ = now_ms;
    this->report_.clear();
    this->verdict_.clear();
    this->level_ = 0;
    this->step_ = 0;
    this->text_("Running: the board");
  }
  Phase phase() const { return this->phase_; }
  bool queued() const { return this->phase_ == QUEUED; }
  bool running() const { return this->phase_ > QUEUED; }
  /// the port stays powered while it runs
  bool hold_power() const { return this->phase_ > BOARD; }
  /// the line's lights' test level now (brightness, 0..1; 0: their own state)
  float level() const { return this->level_; }
  const std::string &status() const { return this->status_; }
  const std::string &report() const { return this->report_; }
  /// the stored Finalize passed and its receivers are still the ones at their positions
  bool commissioned(const TelemetryPoller &p) const {
    return this->record_.magic == CommissionRecord::MAGIC && this->record_.passed && this->unchanged_(p);
  }

  /// One step; unix_s: the time now (0: none valid); calibrating: the hub's current calibration runs; updating: a
  /// firmware update session runs. Returns Act bits.
  uint8_t poll(uint32_t now_ms, uint32_t unix_s, const CommissionPort &port, TelemetryPoller &p, const PortMonitor &m,
               bool calibrating, bool updating) {
    uint8_t act = 0;
    this->last_p_ = &p;
    this->unix_ = unix_s;
    if (this->phase_ <= QUEUED) {
      const bool c = this->commissioned(p);   // (a receiver replaced since: the state changes)
      if (c != this->was_commissioned_ || this->idle_due_) {
        this->was_commissioned_ = c;
        this->idle_due_ = false;
        if (this->phase_ == IDLE && this->last_.empty())
          this->idle_text_(p);
        act |= ACT_TEXT;
      }
      if (this->changed_) {
        this->changed_ = false;
        act |= ACT_TEXT;
      }
      return act;
    }
    const uint16_t n = p.receivers();
    if (port.present && port.off)
      return this->fail_(now_ms, "The port's power mode is Off");
    if (updating)
      return this->fail_(now_ms, "A firmware update started");
    switch (this->phase_) {
      case BOARD:
        if (!port.present || !port.detect || port.board) {
          this->next_(POWER, now_ms, "Running: the 55 V");
        } else if (now_ms - this->t_ > BOARD_MS) {
          return this->fail_(now_ms, "No board on the port");
        }
        break;
      case POWER:
        if (port.ready) {
          p.mark_fresh(now_ms, true);   // (every page read now: the trip / dip counts before the run start right)
          this->next_(ANSWER, now_ms, "Running: 0/" + std::to_string(n) + " answering");
        } else if (port.present && port.fault) {
          return this->fail_(now_ms, "The port lost its board (connection lost)");
        } else if (now_ms - this->t_ > POWER_MS) {
          return this->fail_(now_ms, "The port didn't power up");
        }
        break;
      case ANSWER: {
        uint16_t ok = 0;
        while (ok < n && p.answering(ok) && p.all_fresh(ok))
          ok++;
        if (ok < n) {
          const std::string s =
              "Running: " + std::to_string(this->answered_(p)) + "/" + std::to_string(n) + " answering";
          if (s != this->status_)
            this->text_(s);
          if (now_ms - this->t_ > this->answer_ms_)
            return this->fail_(now_ms, this->silent_(ok));
          break;
        }
        this->answer_report_(p);
        this->events0_.assign(n, 0);
        for (uint16_t i = 0; i < n; i++)
          this->events0_[i] = events_(p.data(i));
        if (this->kind_ == FINALIZE) {
          this->next_(CALIBRATE, now_ms, "Running: the current calibration");
          act |= ACT_CALIBRATE;
          this->cal_seen_ = false;
          break;
        }
        // the load test: the readings now (no load) are the baseline
        this->snap_(p, m, this->v0_, this->i0_, &this->vs0_);
        this->set_level_(now_ms, 1);
        act |= ACT_LEVEL;
        this->next_(LOAD, now_ms, "Running: the load test");
        break;
      }
      case LOAD:
        act |= this->load_(now_ms, p, m);
        break;
      case RESTORE:
        if (now_ms - this->t_ >= SETTLE_MS)
          act |= this->finish_(now_ms, unix_s, p, this->verdict_.empty(), this->verdict_);
        break;
      case CALIBRATE:
        if (calibrating) {
          this->cal_seen_ = true;
        } else if (this->cal_seen_) {
          p.mark_fresh(now_ms, true);   // every page of every receiver read again
          this->next_(READ, now_ms, "Running: the final read");
        } else if (now_ms - this->t_ > CAL_START_MS) {
          return this->fail_(now_ms, "The current calibration didn't start");
        }
        break;
      case READ: {
        uint16_t ok = 0;
        while (ok < n && p.answering(ok) && p.all_fresh(ok))
          ok++;
        if (ok < n) {
          if (now_ms - this->t_ > this->answer_ms_ + 60000)
            return this->fail_(now_ms, this->rx_(ok) + " didn't answer the final read");
          break;
        }
        std::string why;
        for (uint16_t i = 0; i < n && why.empty(); i++) {
          const ReceiverStatus s = receiver_status(p.data(i), true);
          if (receiver_status_problem(s))
            why = this->rx_(i) + ": " + receiver_status_name(s);
          else if (events_(p.data(i)) != this->events0_[i])
            why = this->rx_(i) + " tripped or dipped during the run";
        }
        act |= this->finish_(now_ms, unix_s, p, why.empty(), why);
        break;
      }
      default:
        break;
    }
    if (this->changed_) {
      this->changed_ = false;
      act |= ACT_TEXT;
    }
    return act;
  }

 protected:
  static constexpr float LEVEL[LEVELS] = {0.3f, 0.5f, 0.7f, 0.85f, 1.0f};

  std::string rx_(uint16_t i) const { return "R" + std::to_string(i + 1); }
  /// receiver k is the first that doesn't answer: where to look
  std::string silent_(uint16_t k) const {
    if (k == 0)
      return "No receiver answers: check the RS-485 wiring";
    return this->rx_(k) + " doesn't answer (" + (k == 1 ? std::string("R1 does") : "R1-" + this->rx_(k - 1) + " do") +
           "): check the cable or power between " + this->rx_(k - 1) + " and " + this->rx_(k);
  }
  uint16_t answered_(const TelemetryPoller &p) const {
    uint16_t k = 0;
    for (uint16_t i = 0; i < p.receivers(); i++)
      k += p.answering(i) && p.fresh(i, TP_STATUS);
    return k;
  }
  /// a receiver's trips and supply dips so far (a change during a run: it tripped or dipped)
  static uint32_t events_(const ReceiverData &d) {
    return static_cast<uint32_t>(d.overload_trips) + d.short_trips + d.dips;
  }
  void text_(const std::string &s) {
    this->status_ = s;
    this->changed_ = true;
  }
  void next_(Phase ph, uint32_t now_ms, const std::string &s) {
    this->phase_ = ph;
    this->t_ = now_ms;
    this->text_(s);
  }
  /// The run failed: the lights back first (a load test), then the verdict
  uint8_t fail_(uint32_t now_ms, const std::string &why) {
    if (this->level_ > 0.0f) {
      this->level_ = 0.0f;
      this->verdict_ = why;
      this->next_(RESTORE, now_ms, "Running: the lights back");
      return ACT_LEVEL | ACT_TEXT;
    }
    return this->finish_(now_ms, this->unix_, *this->last_p_, false, why);
  }
  uint8_t finish_(uint32_t now_ms, uint32_t unix_s, const TelemetryPoller &p, bool passed, const std::string &why) {
    (void) now_ms;
    uint8_t act = ACT_TEXT | ACT_DONE;
    const bool fin = this->kind_ == FINALIZE;
    if (fin && this->phase_ >= ANSWER) {   // (the result is stored once it got as far as the receivers)
      CommissionRecord r;
      r.magic = CommissionRecord::MAGIC;
      r.date = unix_s;
      r.passed = passed;
      r.receivers = static_cast<uint8_t>(p.receivers());
      r.fw_min = p.receivers() != 0 ? UINT32_MAX : 0;
      for (uint16_t i = 0; i < p.receivers(); i++) {
        r.uid[i] = p.uid16(i);
        const uint32_t v = p.data(i).fw_version;
        r.fw_min = v < r.fw_min ? v : r.fw_min;
        r.fw_max = v > r.fw_max ? v : r.fw_max;
      }
      this->record_ = r;
      act |= ACT_STORE;
    }
    if (fin && this->phase_ >= ANSWER) {
      this->last_.clear();
      this->idle_text_(p);
    } else {
      this->last_ = passed ? std::string("Commission passed")
                           : std::string(fin ? "Finalize" : "Commission") + " failed: " + why;
      this->status_ = this->last_;
    }
    if (!passed)
      this->report_ = why + (this->report_.empty() ? "" : ". " + this->report_);
    else if (fin)
      this->report_ += ". Calibrated, the final checks passed";
    this->cap_report_();
    this->phase_ = IDLE;
    this->level_ = 0.0f;
    this->was_commissioned_ = this->commissioned(p);
    this->v0_.clear();
    this->v0_.shrink_to_fit();
    this->i0_.clear();
    this->i0_.shrink_to_fit();
    this->events0_.clear();
    this->events0_.shrink_to_fit();
    return act;
  }
  /// the text while nothing runs and nothing ran since the boot: the stored result
  void idle_text_(const TelemetryPoller &p) {
    const CommissionRecord &r = this->record_;
    if (r.magic != CommissionRecord::MAGIC) {
      this->status_ = "Not commissioned";
    } else if (!r.passed) {
      this->status_ = "Finalize failed " + commission_date(r.date);
    } else {
      this->status_ = "Commissioned " + commission_date(r.date) + (this->unchanged_(p) ? "" : ", changed since");
    }
  }
  bool unchanged_(const TelemetryPoller &p) const {
    if (this->record_.receivers != p.receivers())
      return false;
    for (uint16_t i = 0; i < p.receivers(); i++) {
      if (p.has_uid(i) && p.uid16(i) != this->record_.uid[i])
        return false;
    }
    return true;
  }
  /// "4/4 answering: R1 9C02, R2 3A11, ...; firmware 0.6.10" (several versions: each one's)
  void answer_report_(const TelemetryPoller &p) {
    const uint16_t n = p.receivers();
    bool same = true;
    for (uint16_t i = 1; i < n; i++)
      same &= p.data(i).fw_version == p.data(0).fw_version;
    std::string s = std::to_string(n) + "/" + std::to_string(n) + " answering:";
    for (uint16_t i = 0; i < n; i++) {
      char b[24];
      snprintf(b, sizeof(b), " R%u %04X", static_cast<unsigned>(i + 1), static_cast<unsigned>(p.uid16(i)));
      s += b;
      if (!same)
        s += " " + commission_fw(p.data(i).fw_version);
      if (i + 1 < n)
        s += ",";
    }
    if (same && n > 0)
      s += "; firmware " + commission_fw(p.data(0).fw_version);
    this->report_ = s;
  }
  void snap_(const TelemetryPoller &p, const PortMonitor &m, std::vector<float> &v, std::vector<float> &i, float *vs) {
    const uint16_t n = p.receivers();
    v.assign(n, NAN);
    i.assign(n, NAN);
    for (uint16_t k = 0; k < n; k++) {
      v[k] = telemetry_value(p.data(k), TV_SUPPLY_VOLTAGE);
      i[k] = telemetry_value(p.data(k), TV_INPUT_CURRENT);
    }
    *vs = m.supply_v();
  }
  void set_level_(uint32_t now_ms, uint8_t step) {
    this->step_ = step;
    this->level_ = LEVEL[step - 1];
    this->t_ = now_ms;
    this->marked_ = false;
  }
  /// The load test: each level held until every receiver answered after it; on at the next until the port carries
  /// TARGET_LOAD % of its limit (or the last level), then the measurement and the lights back
  uint8_t load_(uint32_t now_ms, TelemetryPoller &p, const PortMonitor &m) {
    const uint16_t n = p.receivers();
    if (now_ms - this->t_ < SETTLE_MS)
      return 0;
    if (!this->marked_) {
      p.mark_fresh(now_ms, false);
      this->marked_ = true;
      return 0;
    }
    uint16_t ok = 0;
    while (ok < n && p.answering(ok) && p.fresh(ok, TP_STATUS))
      ok++;
    if (ok < n) {
      if (now_ms - this->t_ > this->step_ms_)
        return this->fail_(now_ms, this->rx_(ok) + " stopped answering during the load test");
      return 0;
    }
    for (uint16_t i = 0; i < n; i++) {
      const ReceiverStatus s = receiver_status(p.data(i), true);
      if (s == RS_TRIPPED || events_(p.data(i)) != this->events0_[i])
        return this->fail_(now_ms, this->rx_(i) + " tripped or dipped during the load test");
    }
    const float load = m.load_pct();
    if (this->step_ < LEVELS && !(load >= TARGET_LOAD)) {
      this->set_level_(now_ms, this->step_ + 1);
      return ACT_LEVEL;
    }
    // the measurement: each segment's resistance from the change of its voltage drop and of its current; each
    // receiver's drop scaled to the port's full load
    std::vector<float> v1, i1;
    float vs1 = NAN;
    this->snap_(p, m, v1, i1, &vs1);
    const float vs = !std::isnan(vs1) ? vs1 : this->vs0_;
    const float limit = m.config().limit_a;
    float t0 = 0, t1 = 0;   // the currents through the first segment: the whole port's
    for (uint16_t k = 0; k < n; k++) {
      t0 += std::isnan(this->i0_[k]) ? 0.0f : this->i0_[k];
      t1 += std::isnan(i1[k]) ? 0.0f : i1[k];
    }
    char b[96];
    snprintf(b, sizeof(b), ". Cable at %.2f A:", t1);
    std::string s = b;
    float worst_v = NAN, through0 = t0, through1 = t1, prev0 = this->vs0_, prev1 = vs;
    int worst = -1;
    std::string verdict;
    for (uint16_t k = 0; k < n; k++) {
      const float dd = (prev1 - v1[k]) - (prev0 - this->v0_[k]), dt = through1 - through0;
      if (dt >= MIN_DELTA_A && !std::isnan(dd)) {
        snprintf(b, sizeof(b), " R%u %.0f", static_cast<unsigned>(k + 1), (dd > 0 ? dd : 0.0f) / dt * 1000.0f);
      } else {
        snprintf(b, sizeof(b), " R%u n/a", static_cast<unsigned>(k + 1));
      }
      s += b;
      s += k + 1 < n ? "," : " mOhm";
      // this receiver's voltage at the port's full load: its drop's change per ampere of the port, scaled up
      const float d0 = this->vs0_ - this->v0_[k], d1 = vs - v1[k];
      if (t1 - t0 >= MIN_DELTA_A && !std::isnan(d0) && !std::isnan(d1)) {
        const float full = vs - (d0 + (d1 - d0) * (limit - t0) / (t1 - t0));
        if (std::isnan(worst_v) || full < worst_v) {
          worst_v = full;
          worst = k;
        }
      }
      prev0 = this->v0_[k];
      prev1 = v1[k];
      through0 -= std::isnan(this->i0_[k]) ? 0.0f : this->i0_[k];
      through1 -= std::isnan(i1[k]) ? 0.0f : i1[k];
    }
    if (worst < 0 || std::isnan(vs)) {
      s += "; cable not measured (too little load)";
    } else {
      snprintf(b, sizeof(b), "; at full load (%.2f A) %s gets %.1f V (%.0f %%)", limit, this->rx_(worst).c_str(),
               worst_v, worst_v / vs * 100.0f);
      s += b;
      if (worst_v < this->min_voltage_ * vs) {
        snprintf(b, sizeof(b), "%s: %.1f V at full load (%.0f %%): cable too long or thin", this->rx_(worst).c_str(),
                 worst_v, worst_v / vs * 100.0f);
        verdict = b;
      }
    }
    this->report_ += s;
    this->verdict_ = verdict;
    this->level_ = 0.0f;
    this->next_(RESTORE, now_ms, "Running: the lights back");
    return ACT_LEVEL;
  }
  /// the report within REPORT_MAX bytes, cut at a character (UTF-8)
  void cap_report_() {
    if (this->report_.size() <= REPORT_MAX)
      return;
    size_t cut = REPORT_MAX - 3;
    while (cut > 0 && (static_cast<uint8_t>(this->report_[cut]) & 0xC0) == 0x80)
      cut--;
    this->report_.resize(cut);
    this->report_ += "...";
  }

  CommissionRecord record_{};
  Kind kind_{COMMISSION};
  Phase phase_{IDLE};
  uint32_t t_{0}, answer_ms_{20000}, step_ms_{10000}, unix_{0};
  float min_voltage_{0.9f}, level_{0.0f}, vs0_{NAN};
  uint8_t step_{0};
  bool marked_{false}, cal_seen_{false}, changed_{false}, was_commissioned_{false}, idle_due_{true};
  std::string status_{"Not commissioned"}, report_, verdict_, last_;
  std::vector<float> v0_, i0_;      // the load test's baseline (only while it runs)
  std::vector<uint32_t> events0_;   // each receiver's trips and dips at the start
  const TelemetryPoller *last_p_{nullptr};
};

}  // namespace esphome::dig2analog_ultra
