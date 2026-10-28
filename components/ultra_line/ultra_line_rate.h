#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "ultra_transport.h"

namespace esphome::ultra_line {

/// The physical port: the boards it may chain and the line speed its board count allows differ (they forward without
/// redriving; protocol docs/timing.md). The values are the sender job's ULTRA_PORT_* (port_job.h checks them).
enum PortType : uint8_t { PT_UNKNOWN = 0, PT_DIFF_POWER = 1, PT_D2A_PIN = 2 };
static constexpr uint16_t PORT_DIFF_POWER_MAX_BOARDS = 8;   // a Diff-Power RS-485 port: eight receivers
static constexpr uint16_t PORT_D2A_PIN_MAX_BOARDS = 50;     // a Dig2Analog-Plus-5ch single-wire data pin
inline uint16_t port_max_boards(uint8_t type) {
  return type == PT_DIFF_POWER ? PORT_DIFF_POWER_MAX_BOARDS : type == PT_D2A_PIN ? PORT_D2A_PIN_MAX_BOARDS : 0;
}

/// The line profiles' bit rates (profile 0..3)
static constexpr const char *LINE_RATE_TEXT[4] = {"800 kHz", "600 kHz", "400 kHz", "200 kHz"};

/// Why a line starts at its profile: the sender helper's reason (ultra_port_line_profile, ULTRA_PORT_PROFILE_*;
/// port_job_test.cpp checks the values), or line_rate initial_profile asking for a slower one
enum LineChoiceWhy : uint8_t {
  LCW_DEFAULT = 0,       // 800 kHz: a Diff-Power port (its receivers' margins step it down), a D2A pin of 1..20 boards
  LCW_BOARD_COUNT = 1,   // a D2A pin's speed table: the profile for its board count
  LCW_TOO_MANY = 2,      // refused: more boards than the port type chains
  LCW_NO_BOARDS = 3,     // refused: no boards
  LCW_UNKNOWN_TYPE = 4,  // refused: the port's type is not known
  LCW_CONFIGURED = 5,    // line_rate initial_profile: slower than the table
};
struct LineChoice {
  uint8_t profile{0}, why{LCW_DEFAULT};
};

/// The profile a line starts at, from the sender helper's answer for its port type and configured board count
/// (ultra_port_line_profile: its reason and profile; above 3 refused, the line keeps 800 kHz). line_rate initial_profile
/// only asks for a slower one. A D2A pin never steps on its own (no return path); admission still decides whether a
/// line may leave 800 kHz.
inline LineChoice choose_line_profile(uint8_t why, uint8_t table, uint8_t configured) {
  if (table > 3)
    return {0, why};
  if (configured > table && configured <= 3)
    return {configured, LCW_CONFIGURED};
  return {table, why};
}
inline bool line_choice_refused(const LineChoice &c) { return c.why >= LCW_TOO_MANY && c.why <= LCW_UNKNOWN_TYPE; }
inline std::string line_choice_text(uint8_t port_type, uint16_t boards, const LineChoice &c) {
  std::string s = port_type == PT_DIFF_POWER ? "Diff-Power port, " : port_type == PT_D2A_PIN ? "Dig2Analog-Plus-5ch pin, "
                                                                                               : "port type unknown, ";
  s += std::to_string(boards) + (boards == 1 ? " board: " : " boards: ");
  switch (c.why) {
    case LCW_BOARD_COUNT: return s + "the speed table for its board count";
    case LCW_TOO_MANY: return s + "refused: more boards than the port type chains";
    case LCW_NO_BOARDS: return s + "refused: no boards";
    case LCW_UNKNOWN_TYPE: return s + "refused: the port's type is not known";
    case LCW_CONFIGURED: return s + "line_rate initial_profile";
    default: return s + "the default";
  }
}

// One frozen LINK_STATS pair. Counts are intervals (snapshot-and-clear), not
// wrapping odometers. Epoch/boot coherence belongs to the poller.
struct LineCounters {
  uint32_t sampled_ms{0};
  uint16_t epoch{0}, valid{0}, bad{0};
  uint8_t profile{255}, supported{0}, flags{0};
  bool coherent{false};
};

inline bool receiver_requests_lower_rate(const LineCounters &s) { return (s.flags & ULTRA_LS_RATE_REQUEST) != 0; }

class LineRatePolicy {
 public:
  struct Options {
    uint32_t window_ms{5000};
    uint16_t minimum_frames{32}, error_permille{10};
    uint8_t bad_windows{3};
    bool automatic{true};
  };
  void configure(const Options &options) { this->options_ = options; }
  void reset(size_t receivers) { this->receivers_.assign(receivers, Receiver{}); }
  void forget(size_t receiver) { if (receiver < this->receivers_.size()) this->receivers_[receiver] = Receiver{}; }
  void applied(uint8_t profile) {
    this->current_ = this->pending_ = profile;
    for (auto &r : this->receivers_) r = Receiver{};
  }
  uint8_t current() const { return this->current_; }
  uint8_t requested() const { return this->pending_; }
  // User/config requests can step either way, but still require admission.
  bool request(uint8_t profile) {
    if (profile > 3) return false;
    this->pending_ = profile;
    return true;
  }
  bool observe(size_t receiver, const LineCounters &s, uint32_t now) {
    if (receiver >= this->receivers_.size()) return false;
    auto &r = this->receivers_[receiver];
    if (!s.coherent || now - s.sampled_ms > 2000 || s.profile != this->current_ ||
        !(s.flags & ULTRA_LS_VALID) || !(s.flags & ULTRA_LS_LOCKED)) {
      r = Receiver{};
      return false;
    }
    // The receiver already qualified consecutive bad windows. Its latched
    // request needs no minimum frame count or additional host-side window.
    if (receiver_requests_lower_rate(s)) return this->step_down_(s);
    // Older receivers have only interval counters; saturated counts are unknown.
    if (s.flags & ULTRA_LS_OVERFLOW) { r = Receiver{}; return false; }
    if (r.seen && r.epoch == s.epoch) return false;
    if (!r.seen || now - r.last_ms > 2000) {
      r = Receiver{};
      r.start_ms = now;
    }
    r.seen = true; r.epoch = s.epoch; r.last_ms = now;
    r.valid += s.valid; r.bad += s.bad;
    if (now - r.start_ms < this->options_.window_ms) return false;
    const uint32_t total = r.valid + r.bad;
    const bool enough = total >= this->options_.minimum_frames;
    const bool bad = enough &&
        static_cast<uint64_t>(r.bad) * 1000 > static_cast<uint64_t>(total) * this->options_.error_permille;
    r.bad_windows = bad ? static_cast<uint8_t>(r.bad_windows < 255 ? r.bad_windows + 1 : 255) : 0;
    r.valid = r.bad = 0; r.start_ms = now;
    if (r.bad_windows < this->options_.bad_windows)
      return false;
    return this->step_down_(s);
  }
 private:
  bool step_down_(const LineCounters &s) {
    if (!this->options_.automatic || this->current_ == 3 || this->pending_ != this->current_ ||
        !(s.supported & (1u << (this->current_ + 1)))) return false;
    this->pending_ = this->current_ + 1;
    return true;
  }
  struct Receiver {
    uint32_t start_ms{0}, last_ms{0}, valid{0}, bad{0};
    uint16_t epoch{0};
    uint8_t bad_windows{0};
    bool seen{false};
  };
  Options options_{};
  uint8_t current_{0}, pending_{0};
  std::vector<Receiver> receivers_;
};
}  // namespace esphome::ultra_line
