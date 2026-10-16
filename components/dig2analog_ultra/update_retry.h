#pragma once

// Automatic re-install after a firmware update session (the update entity's auto_retry): whether to send another
// normal session for boards that missed one. The frames and pauses stay UpdateSession's; this only decides whether a
// session is sent again. A board already on the announced version doesn't take a session: its app stays lit (firmware
// 0.6.9 shows "already active" on its status LED only), so a repeat costs it nothing.
//  - Lines whose every board answers (QuinLED Diff-Power ultra receivers, telemetry): after the session and the PING
//    burst that confirms the new firmware, each receiver's IDENTITY / STATUS must show the new version. Another session
//    only while one doesn't, or doesn't answer (a receiver that missed it waits in its bootloader, which doesn't
//    answer), up to `retries` times. A receiver that can't take the image (other hardware or revision, an older
//    bootloader, a newer version) is left out.
//  - One-way lines: the hub can't see the result: `retries` more sessions, blind.
// Never with FORCE (recover) and never after an ABORT: the update entity starts no retry then.
// Plain C++: tests/ultra_update_retry_test.cpp runs it with the firmware's bootloader and app.

#include <cstdint>
#include <string>

#include "fw_version.h"
#include "ultra_telemetry.h"

namespace esphome::dig2analog_ultra {

/// The release being installed, as a receiver's pages show it.
struct UpdateTarget {
  uint32_t version{0};   // version word (fw_version.h)
  uint8_t hw_id{0};
  uint16_t hw_rev_mask{0};
  uint8_t min_bootloader{0};
};

enum RxUpdate : uint8_t {
  RXU_ON_VERSION = 0,   // runs the release
  RXU_NO_ANSWER,        // doesn't answer (e.g. waiting in its bootloader)
  RXU_OLD,              // answers, another version (or its IDENTITY not read yet)
  RXU_CANT,             // can't take it: other hardware or revision, an older bootloader, a newer version
};

/// A receiver against the release (IDENTITY: version, hardware and its revision, bootloader).
inline RxUpdate receiver_update_state(const ReceiverData &d, bool answering, const UpdateTarget &t) {
  if (!answering)
    return RXU_NO_ANSWER;
  if (!d.have_identity)
    return RXU_OLD;
  if (d.hw_id != t.hw_id || d.bl_version < t.min_bootloader || d.hw_rev > 15 ||
      ((t.hw_rev_mask >> d.hw_rev) & 1u) == 0)
    return RXU_CANT;
  if (d.fw_word == t.version)
    return RXU_ON_VERSION;
  return d.fw_word > t.version ? RXU_CANT : RXU_OLD;   // (a newer one rejects it without FORCE)
}

class AutoRetry {
 public:
  /// after a session's result: the restarted receivers are read again well within this (IDENTITY after the restart)
  static constexpr uint32_t CHECK_MS = 15000;
  enum Step : uint8_t { FINISH = 0, RETRY, CHECK };

  /// One install: retries (0: off); verify: every board of the line answers (telemetry), else blind.
  void begin(uint8_t retries, bool verify) {
    *this = AutoRetry{};
    this->retries_ = retries;
    this->verify_ = verify;
  }
  /// A session ended (its result, after the PING burst): CHECK the receivers (verify), RETRY (blind, one left), or
  /// FINISH.
  Step session_done(uint32_t now_ms) {
    if (this->retries_ == 0)
      return FINISH;
    if (this->verify_) {
      this->since_ms_ = now_ms;
      return CHECK;
    }
    if (this->used_ < this->retries_) {
      this->used_++;
      return RETRY;
    }
    return FINISH;
  }
  /// Verify, after session_done(): the receivers now. FINISH once each runs the release or can't take it, or when the
  /// check time is over with the retries used up; RETRY when it is over and one is missing; CHECK meanwhile.
  Step check(uint32_t now_ms, const TelemetryPoller &p, const UpdateTarget &t) {
    this->begin_count_(p.receivers());
    for (uint16_t k = 0; k < p.receivers(); k++)
      this->count_(k, receiver_update_state(p.data(k), p.answering(k), t));
    return this->decide_(now_ms);
  }
  /// The same from each receiver's state (n receivers, in chain order).
  Step check(uint32_t now_ms, const RxUpdate *states, uint16_t n) {
    this->begin_count_(n);
    for (uint16_t k = 0; k < n; k++)
      this->count_(k, states[k]);
    return this->decide_(now_ms);
  }

  uint8_t retries() const { return this->retries_; }
  uint8_t used() const { return this->used_; }
  bool verify() const { return this->verify_; }
  /// the counts of the last check()
  uint16_t receivers() const { return this->n_; }
  uint16_t on_version() const { return this->count_of_[RXU_ON_VERSION]; }
  uint16_t pending() const { return this->count_of_[RXU_NO_ANSWER] + this->count_of_[RXU_OLD]; }
  uint16_t cant() const { return this->count_of_[RXU_CANT]; }
  /// verify: every receiver runs the release, or can't take it (and at least one runs it)
  bool installed() const { return this->pending() == 0 && this->on_version() > 0; }

  /// "retry 1 of 2: " while a retry runs ("" before)
  std::string prefix() const {
    return this->used_ == 0 ? std::string() : "retry " + std::to_string(this->used_) + " of " +
                                                  std::to_string(this->retries_) + ": ";
  }
  /// "installed; checking the 3 receivers"
  std::string checking_text(uint16_t receivers) const {
    return this->prefix() + "installed; checking the " + std::to_string(receivers) +
           (receivers == 1 ? " receiver" : " receivers");
  }
  /// Why the retry that just started: "retry 1 of 2: receiver 3 doesn't answer", "retry 1 of 2: 2 receivers not on
  /// 0.6.9 yet", blind: "retry 1 of 2 for boards that may have missed the update".
  std::string retry_text(uint32_t version) const {
    const std::string head = "retry " + std::to_string(this->used_) + " of " + std::to_string(this->retries_);
    if (!this->verify_)
      return head + " for boards that may have missed the update";
    return head + ": " + this->missing_(version);
  }
  /// The end of a verified install: "all 3 receivers on 0.6.9", "2 of 3 receivers on 0.6.9 after 2 retries; receiver 3
  /// doesn't answer (it waits in its bootloader, red 1 Hz, for the next install)", "...; receiver 2 can't take it
  /// (hardware, bootloader or a newer version)".
  std::string result_text(uint32_t version) const {
    const std::string v = format_fw_version(version);
    std::string s = this->n_ == 1 && this->on_version() == 1 ? "the receiver on " + v
                    : this->on_version() == this->n_          ? "all " + std::to_string(this->n_) + " receivers on " + v
                                                              : std::to_string(this->on_version()) + " of " +
                                                           std::to_string(this->n_) + " receivers on " + v;
    if (this->pending() != 0)
      s += " after " + std::to_string(this->used_) + (this->used_ == 1 ? " retry; " : " retries; ") +
           this->missing_(version, true);
    if (this->cant() != 0)
      s += "; " + this->name_(this->first_[RXU_CANT], this->cant()) +
           " can't take it (hardware, bootloader or a newer version)";
    return s;
  }

 protected:
  void begin_count_(uint16_t n) {
    this->n_ = n;
    for (int i = 0; i < 4; i++) {
      this->count_of_[i] = 0;
      this->first_[i] = -1;
    }
  }
  void count_(uint16_t k, RxUpdate s) {
    this->count_of_[s]++;
    if (this->first_[s] < 0)
      this->first_[s] = k;
  }
  Step decide_(uint32_t now_ms) {
    if (this->pending() == 0)
      return FINISH;
    if (now_ms - this->since_ms_ < CHECK_MS)
      return CHECK;
    if (this->used_ < this->retries_) {
      this->used_++;
      return RETRY;
    }
    return FINISH;
  }
  /// "receiver 3" / "2 receivers"
  std::string name_(int first, uint16_t count) const {
    return count == 1 ? "receiver " + std::to_string(first + 1) : std::to_string(count) + " receivers";
  }
  /// what keeps a retry going: those that don't answer first (at the end: they wait in their bootloader), then the
  /// old ones
  std::string missing_(uint32_t version, bool end = false) const {
    const uint16_t none = this->count_of_[RXU_NO_ANSWER], old = this->count_of_[RXU_OLD];
    std::string s;
    if (none != 0)
      s = this->name_(this->first_[RXU_NO_ANSWER], none) + (none == 1 ? " doesn't answer" : " don't answer");
    if (none != 0 && end)
      s += none == 1 ? " (it waits in its bootloader, red 1 Hz, for the next install)"
                     : " (they wait in their bootloader, red 1 Hz, for the next install)";
    if (old != 0)
      s += (s.empty() ? "" : ", ") + this->name_(this->first_[RXU_OLD], old) + " not on " + format_fw_version(version) +
           " yet";
    return s;
  }

  uint8_t retries_{0}, used_{0};
  bool verify_{false};
  uint32_t since_ms_{0};
  uint16_t n_{0};
  uint16_t count_of_[4]{};
  int first_[4]{-1, -1, -1, -1};
};

}  // namespace esphome::dig2analog_ultra
