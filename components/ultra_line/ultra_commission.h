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

namespace esphome::ultra_line {

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

/// "0.7.0" (STATUS's major << 16 | minor << 8 | patch; 0: a development build)
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
    this->why_.clear();
    this->advice_.clear();
    this->ids_.clear();
    this->digital_lines_.clear();
    this->seg_.clear();
    this->cable_a_ = NAN;
    this->worst_ = -1;
    this->cal_ok_ = 0;
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
      return this->fail_(now_ms, "the port's power mode is Off", "Choose another power mode, then run it again.");
    if (updating)
      return this->fail_(now_ms, "a firmware update started", "Run it again once the update is done.");
    switch (this->phase_) {
      case BOARD:
        if (!port.present || !port.detect || port.board) {
          this->next_(POWER, now_ms, "Running: the 55 V");
        } else if (now_ms - this->t_ > BOARD_MS) {
          return this->fail_(now_ms, "no board on the port", "Check the board's connection to the port.");
        }
        break;
      case POWER:
        if (port.ready) {
          p.mark_fresh(now_ms, true);   // (every page read now: the trip / dip counts before the run start right)
          this->next_(ANSWER, now_ms, "Running: 0/" + std::to_string(n) + " answering");
        } else if (port.present && port.fault) {
          return this->fail_(now_ms, "the port lost its board (connection lost)",
                             "Check the board's connection to the port.");
        } else if (now_ms - this->t_ > POWER_MS) {
          return this->fail_(now_ms, "the port didn't power up", "");
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
          if (now_ms - this->t_ > this->answer_ms_) {
            this->answers_(p);   // (the ones that do: the report's facts)
            if (ok == 0)
              return this->fail_(now_ms, "no receiver answers", "Check the RS-485 wiring at the sender.");
            return this->fail_(now_ms, this->rx_(ok) + " doesn't answer",
                               "Check the cable or power between " + this->rx_(ok - 1) + " and " + this->rx_(ok) + ".");
          }
          break;
        }
        this->answers_(p);
        this->events0_.assign(n, 0);
        for (uint16_t i = 0; i < n; i++)
          this->events0_[i] = events_(p.data(i));
        if (this->kind_ == FINALIZE) {
          this->next_(CALIBRATE, now_ms, "Running: the current calibration");
          act |= ACT_CALIBRATE;
          this->cal_seen_ = true;
          for (uint16_t i = 0; i < n; i++)
            if (!(p.data(i).hw_caps & RX_CAP_PIXEL)) this->cal_seen_ = false; // digital tests are explicit receiver presses only
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
          act |= this->finish_(now_ms, unix_s, p, this->why_.empty());
        break;
      case CALIBRATE:
        if (calibrating) {
          this->cal_seen_ = true;
        } else if (this->cal_seen_) {
          p.mark_fresh(now_ms, true);   // every page of every receiver read again
          this->next_(READ, now_ms, "Running: the final read");
        } else if (now_ms - this->t_ > CAL_START_MS) {
          return this->fail_(now_ms, "the current calibration didn't start", "");
        }
        break;
      case READ: {
        uint16_t ok = 0;
        while (ok < n && p.answering(ok) && p.all_fresh(ok))
          ok++;
        if (ok < n) {
          if (now_ms - this->t_ > this->answer_ms_ + 60000)
            return this->fail_(now_ms, this->rx_(ok) + " didn't answer the final read", "Check its cable or power.");
          break;
        }
        // the final checks: no problem, no trip or dip since the start, each calibration test complete
        for (uint16_t i = 0; i < n && this->why_.empty(); i++) {
          const ReceiverData &d = p.data(i);
          const ReceiverStatus s = receiver_status(d, true, 0, p.dip_warning());
          if (d.output_voltage_low.problem()) {
            this->why_ = this->rx_(i) + ": Output voltage low: fuse blown, output shorted / overloaded, or the supply module is current-limiting";
            this->advice_ = "Check the output fuse and wiring.";
          } else if (receiver_status_problem(s)) {
            this->why_ = this->rx_(i) + " reports " + receiver_status_name(s);
            this->advice_ = "See its Status detail.";
          } else if (events_(d) != this->events0_[i]) {
            this->why_ = this->rx_(i) + " tripped or had a supply dip during the calibration";
            this->advice_ = "Check its outputs' wiring and the supply.";
          } else if (!(d.hw_caps & RX_CAP_PIXEL) && d.have_calibration && d.cal_end != ULTRA_CA_END_COMPLETE) {
            this->why_ = this->rx_(i) + (d.cal_end >= ULTRA_CA_END_NOLOAD ? "'s calibration test was not taken ("
                                                                           : "'s calibration test stopped (") +
                         cal_end_why_(d.cal_end) + ")";
            this->advice_ = "Check its outputs' wiring and the supply, then run Finalize again.";
          }
          this->cal_ok_ += d.have_calibration && d.cal_end == ULTRA_CA_END_COMPLETE && d.cal_age_s < CA_AGE_NONE;
        }
        act |= this->finish_(now_ms, unix_s, p, this->why_.empty());
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
  /// how a calibration test stopped or why it was not taken (ULTRA_CA_END_*), for a sentence
  static const char *cal_end_why_(uint8_t end) {
    return end == ULTRA_CA_END_TRIP     ? "a trip"
           : end == ULTRA_CA_END_LIMIT  ? "the current limit"
           : end == ULTRA_CA_END_SUPPLY ? "a supply dip"
           : end == ULTRA_CA_END_NOLOAD ? "no output drew 20 mA"
           : end == ULTRA_CA_END_FLAT   ? "the current did not change"
                                        : "stopped early";
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
  /// The run failed (why: the cause, after "failed: "; advice: what to do, a sentence or ""): the lights back first (a
  /// load test), then the verdict
  uint8_t fail_(uint32_t now_ms, const std::string &why, const std::string &advice) {
    this->why_ = why;
    this->advice_ = advice;
    if (this->level_ > 0.0f) {
      this->level_ = 0.0f;
      this->next_(RESTORE, now_ms, "Running: the lights back");
      return ACT_LEVEL | ACT_TEXT;
    }
    return this->finish_(now_ms, this->unix_, *this->last_p_, false);
  }
  uint8_t finish_(uint32_t now_ms, uint32_t unix_s, const TelemetryPoller &p, bool passed) {
    (void) now_ms;
    this->answers_(p);
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
                           : std::string(fin ? "Finalize" : "Commission") + " failed: " + this->why_;
      this->status_ = this->last_;
    }
    this->report_ = this->compose_(passed, fin, p.receivers());
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
  /// the receivers that answer now (their UID16 and firmware): the report's facts
  void answers_(const TelemetryPoller &p) {
    this->ids_.clear();
    this->digital_lines_.clear();
    for (uint16_t i = 0; i < p.receivers(); i++) {
      if (!(p.data(i).hw_caps & RX_CAP_PIXEL)) continue;
      const auto &c = p.pixel_calibration(i);
      this->digital_lines_ += (this->digital_lines_.empty() ? "" : "\n") + this->rx_(i) + " digital: " +
          c.result_text() + ". " + c.count_text() + ".";
      const auto &f = p.data(i).output_voltage_low;
      this->digital_lines_ += " " + std::string(!p.answering(i) || !f.known ? "Output voltage: unavailable" :
          f.fault ? "Output voltage low: fuse blown, output shorted / overloaded, or the supply module is current-limiting" : "Output voltage: OK") + ".";
    }
    for (uint16_t i = 0; i < p.receivers(); i++) {
      if (p.answering(i) && p.data(i).have_status)
        this->ids_.push_back(Answer{p.data(i).fw_version, i, p.uid16(i)});
    }
  }

  // ------------------------------------------------------------------------------------------------- the report
  /// "A, B and C"
  static std::string and_list_(const std::vector<std::string> &v) {
    std::string s;
    for (size_t i = 0; i < v.size(); i++)
      s += (i == 0 ? "" : i + 1 == v.size() ? " and " : ", ") + v[i];
    return s;
  }
  /// The receivers that answer, full: "3 of 3 receivers answer: R1 9C01, R2 9C02, R3 9C03 (firmware 0.7.0)." (each
  /// one's version when they differ); short: "All 16 receivers answer (firmware 0.6.12 and 0.7.0).", "R1-R8 answer
  /// (firmware 0.7.0)."
  std::string answers_line_(uint16_t n, bool full) const {
    const size_t k = this->ids_.size();
    if (k == 0)
      return "";
    std::vector<uint32_t> fws;
    for (const Answer &a : this->ids_) {
      if (std::find(fws.begin(), fws.end(), a.fw) == fws.end())
        fws.push_back(a.fw);
    }
    std::sort(fws.begin(), fws.end());
    std::vector<std::string> vs;
    for (uint32_t v : fws)
      vs.push_back(commission_fw(v));
    const std::string firmware = "(firmware " + and_list_(vs) + ")";
    if (full) {
      std::string s = std::to_string(k) + " of " + std::to_string(n) + " receivers answer:";
      for (size_t i = 0; i < k; i++) {
        char b[16];
        snprintf(b, sizeof(b), " R%u %04X", static_cast<unsigned>(this->ids_[i].pos + 1),
                 static_cast<unsigned>(this->ids_[i].uid));
        s += b;
        if (fws.size() > 1)
          s += " (" + commission_fw(this->ids_[i].fw) + ")";
        if (i + 1 < k)
          s += ",";
      }
      return s + (fws.size() > 1 ? "." : " " + firmware + ".");
    }
    if (k == n)
      return "All " + std::to_string(n) + " receivers answer " + firmware + ".";
    if (this->ids_.back().pos + 1u == k)   // (a run from R1 on)
      return (k == 1 ? std::string("R1 answers ") : "R1-" + this->rx_(this->ids_.back().pos) + " answer ") + firmware + ".";
    return std::to_string(k) + " of " + std::to_string(n) + " receivers answer " + firmware + ".";
  }
  /// The cable test, each segment (form 0), their range (1); "" without a load test
  std::string cable_line_(int form) const {
    if (std::isnan(this->cable_a_))
      return "";
    char b[64];
    snprintf(b, sizeof(b), "Cable test at %.2f A: ", this->cable_a_);
    std::string s = b;
    std::vector<std::string> vals;
    float lo = NAN, hi = NAN;
    for (float r : this->seg_) {
      if (std::isnan(r)) {
        vals.push_back("n/a");
        continue;
      }
      vals.push_back(std::to_string(static_cast<int>(std::lround(r))));
      lo = std::isnan(lo) || r < lo ? r : lo;
      hi = std::isnan(hi) || r > hi ? r : hi;
    }
    if (std::isnan(lo))
      return s + "too little load to measure the cable.";
    if (form == 0)
      return s + and_list_(vals) + " m\xCE\xA9 per segment.";
    if (std::lround(lo) == std::lround(hi))
      return s + std::to_string(std::lround(lo)) + " m\xCE\xA9 per segment.";
    return s + std::to_string(std::lround(lo)) + " to " + std::to_string(std::lround(hi)) + " m\xCE\xA9 per segment.";
  }
  /// The lowest voltage at the port's full load (a passed load test; a failed one says it in its verdict)
  std::string load_line_() const {
    if (this->worst_ < 0 || std::isnan(this->vs_))
      return "";
    char b[128];
    const bool last = this->worst_ + 1 == static_cast<int>(this->seg_.size());
    snprintf(b, sizeof(b), "At full load (%.2f A) %s%s%s %.1f V (%.0f %%).", this->limit_a_,
             last ? "the furthest receiver (" : "", this->rx_(this->worst_).c_str(),
             last ? ") still gets" : " gets the least:", this->worst_v_, this->worst_v_ / this->vs_ * 100.0f);
    return b;
  }
  /// The whole report (wiki: Commissioning): the verdict, then one fact per line ("\n": Home Assistant keeps them,
  /// the dashboards show lines), within REPORT_MAX bytes: the segments as a range, the receivers without their UIDs,
  /// then no segments; the verdict and the lowest voltage always stay (cut at a character if still too long).
  std::string compose_(bool passed, bool finalize, uint16_t n) const {
    const std::string head = passed ? std::string("Passed.")
                                    : "Failed: " + this->why_ + "." + (this->advice_.empty() ? "" : " " + this->advice_);
    std::string calib;
    if (passed && finalize) {
      calib = this->cal_ok_ == n && n != 0
                  ? "Current calibration: OK on " + std::string(n == 1 ? "its receiver" : "all " + std::to_string(n)) +
                        "; no problems, trips or dips."
                  : std::string("Current calibration done; no problems, trips or dips.");
    }
    if (passed && finalize && !this->digital_lines_.empty())
      calib = "Digital calibration: explicit receiver button only.";
    const std::string load = passed ? this->load_line_() : "";
    std::string s;
    // (answers full / short, cable each / range / none)
    static const uint8_t LADDER[4][2] = {{1, 0}, {1, 1}, {0, 1}, {0, 2}};
    for (const auto &step : LADDER) {
      std::vector<std::string> lines{head, this->answers_line_(n, step[0] != 0),
                                     step[1] < 2 ? this->cable_line_(step[1]) : "", load, calib, step[0] ? this->digital_lines_ : ""};
      s.clear();
      for (const std::string &l : lines) {
        if (!l.empty())
          s += (s.empty() ? "" : "\n") + l;
      }
      if (s.size() <= REPORT_MAX)
        return s;
    }
    size_t cut = REPORT_MAX - 3;
    while (cut > 0 && (static_cast<uint8_t>(s[cut]) & 0xC0) == 0x80)
      cut--;
    return s.substr(0, cut) + "...";
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
        return this->fail_(now_ms, this->rx_(ok) + " stopped answering during the load test",
                           "Check its cable or power.");
      return 0;
    }
    for (uint16_t i = 0; i < n; i++) {
      const ReceiverStatus s = receiver_status(p.data(i), true, 0, p.dip_warning());
      if (p.data(i).output_voltage_low.problem())
        return this->fail_(now_ms, this->rx_(i) + ": Output voltage low: fuse blown, output shorted / overloaded, or the supply module is current-limiting",
                           "Check the output fuse and wiring.");
      if (s == RS_TRIPPED || events_(p.data(i)) != this->events0_[i])
        return this->fail_(now_ms, this->rx_(i) + " tripped or had a supply dip during the load test",
                           "Check its outputs' wiring and the supply.");
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
    this->cable_a_ = t1;
    this->seg_.assign(n, NAN);
    float worst_v = NAN, through0 = t0, through1 = t1, prev0 = this->vs0_, prev1 = vs;
    int worst = -1;
    for (uint16_t k = 0; k < n; k++) {
      const float dd = (prev1 - v1[k]) - (prev0 - this->v0_[k]), dt = through1 - through0;
      if (dt >= MIN_DELTA_A && !std::isnan(dd))
        this->seg_[k] = (dd > 0 ? dd : 0.0f) / dt * 1000.0f;   // mOhm
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
    this->worst_ = std::isnan(vs) ? -1 : worst;
    this->worst_v_ = worst_v;
    this->vs_ = vs;
    this->limit_a_ = limit;
    if (this->worst_ >= 0 && worst_v < this->min_voltage_ * vs) {   // (the verdict: the lights back first)
      char b[128];
      snprintf(b, sizeof(b), "at full load (%.2f A) %s would get %.1f V (%.0f %%, needs %.0f %%)", limit,
               this->rx_(worst).c_str(), worst_v, worst_v / vs * 100.0f, this->min_voltage_ * 100.0f);
      this->why_ = b;
      this->advice_ = "The cable is too long or too thin.";
    }
    this->level_ = 0.0f;
    this->next_(RESTORE, now_ms, "Running: the lights back");
    return ACT_LEVEL;
  }

  struct Answer {   // a receiver that answered: its position, UID16, firmware
    uint32_t fw;
    uint16_t pos, uid;
  };
  CommissionRecord record_{};
  Kind kind_{COMMISSION};
  Phase phase_{IDLE};
  uint32_t t_{0}, answer_ms_{20000}, step_ms_{10000}, unix_{0};
  float min_voltage_{0.8f}, level_{0.0f}, vs0_{NAN};
  uint8_t step_{0};
  bool marked_{false}, cal_seen_{false}, changed_{false}, was_commissioned_{false}, idle_due_{true};
  std::string status_{"Not commissioned"}, report_, last_;
  std::vector<float> v0_, i0_;      // the load test's baseline (only while it runs)
  std::vector<uint32_t> events0_;   // each receiver's trips and dips at the start
  const TelemetryPoller *last_p_{nullptr};
  // the report's facts (the last run): why it failed and what to do, the receivers that answered, the cable test
  // (the port's current then, each segment's mOhm: NAN n/a), the lowest voltage at full load (worst_ -1: not
  // measured), the receivers whose calibration test completed
  std::string why_, advice_, digital_lines_;
  std::vector<Answer> ids_;
  std::vector<float> seg_;
  float cable_a_{NAN}, worst_v_{NAN}, vs_{NAN}, limit_a_{NAN};
  int worst_{-1};
  uint16_t cal_ok_{0};
};

}  // namespace esphome::ultra_line
