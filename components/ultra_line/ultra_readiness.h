#pragma once

#include <cstdint>

namespace esphome::ultra_line {

// A cold receiver supply may start later than the sender port. Allow the measured startup plus margin on a line
// without replies; otherwise prefer a valid answer in this power-on, with a bounded fallback for silent recovery.
// These waits belong to management operations; light and scene frames keep their existing schedule.
static constexpr uint32_t RECEIVER_MODULE_START_MS = 500;
static constexpr uint32_t RECEIVER_REPLY_TIMEOUT_MS = 5000;

class ReceiverReadiness {
 public:
  bool ready(uint32_t now, bool port_ready, bool line_on, uint32_t since, bool returns, uint32_t boot_ms = 0) {
    this->observe_(line_on, since);
    this->timed_out_ = returns && !this->answered_ && port_ready && line_on &&
        uint32_t(now - since) >= boot_ms + RECEIVER_REPLY_TIMEOUT_MS;
    return port_ready && line_on && (returns ? this->answered_ || this->timed_out_ :
        uint32_t(now - since) >= RECEIVER_MODULE_START_MS);
  }
  /// a reply of this power-on came in
  bool answered() const { return this->answered_; }
  bool take_warning() {
    if (!this->timed_out_ || this->warned_) return false;
    this->warned_ = true;
    return true;
  }
  void answer(bool line_on, uint32_t since, uint32_t poll_ms) {
    this->observe_(line_on, since);
    if (line_on && int32_t(poll_ms - since) >= 0)
      this->answered_ = true;
  }

 private:
  void observe_(bool on, uint32_t since) {
    if (!on || !this->on_ || since != this->since_) {
      this->answered_ = false;
      this->timed_out_ = this->warned_ = false;
    }
    this->on_ = on;
    this->since_ = since;
  }
  bool on_{false}, answered_{false};
  bool timed_out_{false}, warned_{false};
  uint32_t since_{0};
};

}  // namespace esphome::ultra_line
