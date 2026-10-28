#pragma once

// Running a port job (the sender library's ultra_sender_job.h) on a hub's line. The driver is what the hub's transport
// task does, step by step: it takes the line (lease) before the job's frames, waits until the line's last frame is out,
// puts each frame the job returns on the wire with its transport instructions (CHAIN, or a digital port's marker
// envelope), hands the line back while the job verifies (the receivers answer to the hub's own polls then), takes it
// again for the next subtransfer and gives it back when the job ends. The verifier is the main loop's part during
// verification: every node whose identity and FW_UPDATE page were read again since the job asked confirms with that
// fresh evidence (never cached pages). The job's calls are serialized by the driver's lock, and nothing waits while it
// is held: the line's handover is asked and answered without waiting (PortLease), a frame goes on the wire outside
// it, so the main loop never waits for the task longer than one job call. No ESP-IDF / ESPHome dependencies:
// tests/port_job_test.cpp runs both against the real job, also as two threads with the main loop's own methods.

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "port_inventory.h"
#include "ultra_sender_job.h"
#include "ultra_telemetry.h"

namespace esphome::ultra_line {

/// How long the job waits for the receivers' fresh evidence after a subtransfer: the hub resyncs the line, waits for
/// its lock and reads every receiver's identity (parts 0-2 and the descriptor) and FW_UPDATE page again, up to eight
/// receivers on a Diff-Power port. Not a YAML option.
static constexpr uint32_t PORT_JOB_VERIFY_MS = 30000;

class PortJobDriver {
 public:
  /// The hub's side: its clock, its line, its transmitter
  struct Transport {
    virtual ~Transport() = default;
    virtual int64_t now_us() = 0;
    /// when a frame given now starts on the wire: the job stamps its TIME from it (a lane: where its stream has room)
    virtual int64_t frame_start_us() { return this->now_us(); }
    /// the line's last frame and its LOW are out (also the hub's own frames before a lease)
    virtual bool line_free() = 0;
    /// one frame on the wire as the job says (t: CHAIN, or a marker envelope); false: the transmitter refused it
    virtual bool send(const uint8_t *frame, size_t n, const ultra_update_transport_t &t) = 0;
    /// take the line (normal frames, polls and rate changes stop) or hand it back (the boards' state again), never
    /// waiting for it. Taking it: true once it is the job's (an open response window over and read, the hub driving
    /// the line again); false: not yet, asked again at the next step
    virtual bool lease(bool on) = 0;
  };
  enum Step : uint8_t {
    WAIT = 0,   // nothing due yet (sleep until due_us())
    SENT,       // a frame went out
    VERIFYING,  // the line is the hub's: the receivers answer its polls (the verifier confirms)
    DONE,       // the job ended (done or aborted); the line is handed back
    LEASING,    // the line asked for and not the job's yet (the main loop hands it over): sleep a tick
  };
  static constexpr int REFUSALS_MAX = 20;   // the transmitter refusing this often in a row: the job aborts

  void begin(ultra_update_job_t *job, Transport *t, uint8_t *buffer, uint32_t capacity) {
    this->job_ = job;
    this->t_ = t;
    this->buf_ = buffer;
    this->cap_ = capacity;
    this->asked_ = false;
    this->leased_ = false;
    this->refusals_ = 0;
    this->transport_failed_ = false;
  }

  /// One step of the transport task
  Step step() {
    int n = 0;
    ultra_update_transport_t how{};
    {
      std::lock_guard<std::mutex> lock(this->mu_);
      const Step s = this->next_(&n, &how);
      if (s != SENT)
        return s;
    }
    // the frame on the wire outside the lock: the main loop's calls (confirm, progress, abort) never wait for it
    const bool sent = this->t_->send(this->buf_, static_cast<size_t>(n), how);
    std::lock_guard<std::mutex> lock(this->mu_);
    if (sent) {
      this->refusals_ = 0;
      return SENT;
    }
    // a refused frame: the job's schedule has moved on; after REFUSALS_MAX in a row its abort (and its own deadlines)
    if (++this->refusals_ >= REFUSALS_MAX && !this->job_->abort_requested) {
      this->transport_failed_ = true;
      ultra_update_job_abort(this->job_);
    }
    return WAIT;
  }
  /// when the next step is due (the job's due time; now when unknown)
  int64_t due_us() {
    std::lock_guard<std::mutex> lock(this->mu_);
    return this->job_ != nullptr ? this->job_->due_us : 0;
  }
  /// main loop: the job's calls under the driver's lock (confirm, progress, abort)
  template<typename F> auto with_job(F f) -> decltype(f(static_cast<ultra_update_job_t *>(nullptr))) {
    std::lock_guard<std::mutex> lock(this->mu_);
    return f(this->job_);
  }
  bool leased() const { return this->leased_; }
  bool transport_failed() const { return this->transport_failed_; }

 protected:
  /// The step under the lock: SENT with the frame to send (n bytes in buf_, how), else what the step is
  Step next_(int *n_out, ultra_update_transport_t *how) {
    ultra_update_job_t *j = this->job_;
    if (j == nullptr)
      return DONE;
    if (j->state == ULTRA_JOB_DONE) {
      this->release_();
      return DONE;
    }
    const int64_t now = this->t_->now_us();
    if (j->state == ULTRA_JOB_VERIFY) {
      this->release_();
      ultra_update_job_next_frame(j, now, this->buf_, this->cap_);   // (its timeout, or the next subtransfer)
      return j->state == ULTRA_JOB_VERIFY ? VERIFYING : WAIT;
    }
    if (now < j->due_us)
      return WAIT;
    // Nothing has started on this subtransfer: cancellation needs no wire or power-ready lease. In particular,
    // choosing power mode Off while waiting for wake must let the job finish with the port still dark.
    if (!this->leased_ && j->state == ULTRA_JOB_READY && j->abort_requested) {
      ultra_update_job_next_frame(j, now, this->buf_, this->cap_);
      this->release_();
      return DONE;
    }
    if (!this->leased_) {
      this->asked_ = true;
      if (!this->t_->lease(true))
        return LEASING;   // (never a frame before the line is the job's)
      this->leased_ = true;
    }
    if (!this->t_->line_free())
      return WAIT;
    const int64_t start = this->t_->frame_start_us();
    const int n = ultra_update_job_next_frame(j, start, this->buf_, this->cap_);
    if (n <= 0)
      return j->state == ULTRA_JOB_DONE ? (this->release_(), DONE) : WAIT;
    *n_out = n;
    *how = j->transfer.transport;
    return SENT;
  }
  /// the line handed back (also one asked for and not handed over yet)
  void release_() {
    if (this->asked_) {
      this->asked_ = false;
      this->leased_ = false;
      this->t_->lease(false);
    }
  }
  std::mutex mu_;
  ultra_update_job_t *job_{nullptr};
  Transport *t_{nullptr};
  uint8_t *buf_{nullptr};
  uint32_t cap_{0};
  bool asked_{false}, leased_{false};
  bool transport_failed_{false};
  int refusals_{0};
};

/// The line's handover between the job's task and the main loop, neither waiting for the other: the task asks (take
/// the line, or hand it back), the main loop applies the ask once the hub can (an open response window over and read,
/// the line driven again) and answers that ask. Every ask has its own number, so an answer is for that ask only: one
/// that changes while the main loop applies the one before is applied at its next pass.
class PortLease {
 public:
  /// task: ask for the line (on) or hand it back; true once the main loop gave what is asked
  bool ask(bool on) {
    uint32_t a = this->ask_.load();
    if ((a & 1u) != static_cast<uint32_t>(on)) {
      a = (((a >> 1) + 1u) << 1) | static_cast<uint32_t>(on);
      this->ask_.store(a);
    }
    return this->given_.load() == a;
  }
  /// main loop: the ask not answered yet, applied by apply(on) (false: not yet, asked again at the next pass).
  /// 1: the line is the job's now; 0: handed back now; -1: nothing applied
  template<typename Apply> int serve(Apply apply) {
    const uint32_t a = this->ask_.load();
    if (a == this->given_.load() || !apply((a & 1u) != 0))
      return -1;
    this->given_.store(a);
    return static_cast<int>(a & 1u);
  }

 protected:
  std::atomic<uint32_t> ask_{0}, given_{0};   // (its number << 1) | on; given_: the last ask applied
};

/// The sender's identity of a receiver from what the poller read (all three IDENTITY parts and the descriptor)
inline void port_identity_from(ultra_identity_t &id, const ReceiverData &d) {
  std::memset(&id, 0, sizeof(id));
  std::memcpy(id.uid, d.uid, 12);
  id.boot_version = d.bl_version;
  id.board = d.hw_id;
  id.revision = d.hw_rev;
  id.wire = d.wire;
  id.firmware = d.fw_word;
  for (int k = 0; k < 6; k++)
    id.features[k] = d.feature_words[k];
  id.descriptor_length = d.descriptor_length;
  id.descriptor_crc = d.descriptor_crc;
  std::memcpy(id.target.bytes, d.target, UPD_TARGET_BYTES);
  std::memcpy(id.geometry, d.geometry, UPD_GEOMETRY_BYTES);
  id.record_format = d.record_format;
  id.descriptor_format = d.descriptor_format;
  id.entry_10ms = d.entry_10ms;
  id.parts = 7;
}

/// The verification step (main loop, while the driver says VERIFYING): every node still pending on the active image
/// whose receiver identified itself and answered FW_UPDATE again since verify_ms (millis() when the job asked) gives
/// the job that evidence. Returns the confirmations given.
inline int port_job_verify(PortJobDriver &driver, const TelemetryPoller &p, uint32_t verify_ms, int64_t now_us,
                           uint32_t now_ms) {
  return driver.with_job([&](ultra_update_job_t *j) {
    int given = 0;
    if (j == nullptr || j->state != ULTRA_JOB_VERIFY)
      return given;
    for (uint16_t k = 0; k < j->node_count && k < p.receivers(); k++) {
      const ultra_update_result_t &r = j->results[k];
      if (r.result != ULTRA_JOB_PENDING || !r.transfer_complete || !p.answering(k))
        continue;
      const ReceiverData &d = p.data(k);
      if (!d.have_identity || !d.have_update || int32_t(d.identity_ms - verify_ms) < 0 ||
          int32_t(d.update_ms - verify_ms) < 0)
        continue;
      ultra_identity_t id;
      port_identity_from(id, d);
      const uint32_t newest = int32_t(d.update_ms - d.identity_ms) > 0 ? d.update_ms : d.identity_ms;
      const int64_t evidence = now_us - int64_t(uint32_t(now_ms - newest)) * 1000;
      // the answer as it came, at the slot the poll asked it in (its INDEX)
      if (ultra_update_job_confirm(j, k, d.update_index, &id, d.update_answer, evidence) == 0)
        given++;
    }
    return given;
  });
}

}  // namespace esphome::ultra_line
