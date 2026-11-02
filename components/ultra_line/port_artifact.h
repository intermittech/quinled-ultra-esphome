#pragma once

// One artifact of a port job as the sender job streams it (port_job.h: prepare / read / release): its image is
// downloaded only when its subtransfer starts, checked against the signed manifest (size, SHA-256, descriptor, body
// CRC), read from RAM while it goes out, and freed afterwards, so only one image takes the heap at a time. The
// download is the platform's (fw_update.cpp: an HTTP task calls loaded() or failed()); the job calls the hooks from
// the transport task. No ESP-IDF / ESPHome dependencies: tests/port_update_test.cpp.

#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>

#include "fw_manifest.h"

namespace esphome::ultra_line {

class PortArtifactSource {
 public:
  enum State : uint8_t { IDLE = 0, LOADING, READY, FAILED, RELEASED };
  /// starts the download of this artifact (false: it can't); the platform calls loaded() / failed() when it ends
  using StartFn = bool (*)(PortArtifactSource *src);
  using FreeFn = void (*)(uint8_t *img, size_t size);

  void init(const FwImageInfo &image, const std::string &url, StartFn start, FreeFn free_fn) {
    this->image_ = image;
    this->url_ = url;
    this->start_ = start;
    this->free_ = free_fn;
    this->state_.store(IDLE);
  }
  const FwImageInfo &image() const { return this->image_; }
  const std::string &url() const { return this->url_; }
  State state() const { return static_cast<State>(this->state_.load()); }
  const std::string &error() const { return this->error_; }

  /// The download's end (its task): the complete image, whose SHA-256 it computed. Checked, then READY; anything
  /// else FAILED. Ownership of img passes here (freed at once if the job let go meanwhile).
  void loaded(uint8_t *img, size_t size, const uint8_t *sha256) {
    const char *err = fw_check_image(img, size, sha256, this->image_);
    if (err != nullptr) {
      this->drop_(img, size);
      this->failed(err);
      return;
    }
    this->img_ = img;
    this->size_ = size;
    uint8_t want = LOADING;
    if (!this->state_.compare_exchange_strong(want, READY)) {   // released while it downloaded
      this->img_ = nullptr;
      this->drop_(img, size);
    }
  }
  void failed(const char *why) {
    this->error_ = why ? why : "download failed";
    uint8_t want = LOADING;
    this->state_.compare_exchange_strong(want, FAILED);
  }

  // ---- the sender job's hooks
  /// 0 ready, >0 still downloading, <0 failed
  static int prepare(void *ctx) {
    auto *s = static_cast<PortArtifactSource *>(ctx);
    uint8_t st = s->state_.load();
    if (st == IDLE) {
      s->state_.store(LOADING);
      if (s->start_ == nullptr || !s->start_(s)) {
        s->failed("cannot start the download");
        return -1;
      }
      st = s->state_.load();
    }
    return st == READY ? 0 : st == LOADING ? 1 : -1;
  }
  /// the body (offset 0 = the first byte after the descriptor)
  static int read(void *ctx, uint32_t offset, uint8_t *data, uint32_t length) {
    auto *s = static_cast<PortArtifactSource *>(ctx);
    if (s->state_.load() != READY || s->img_ == nullptr || uint64_t(offset) + length > s->size_ - UPD_DESC_BYTES)
      return -1;
    std::memcpy(data, s->img_ + UPD_DESC_BYTES + offset, length);
    return 0;
  }
  /// the job is done with it (also after an abort or a failed preparation)
  static void release(void *ctx) {
    auto *s = static_cast<PortArtifactSource *>(ctx);
    const uint8_t was = s->state_.exchange(RELEASED);
    if (was == READY && s->img_ != nullptr) {
      s->drop_(s->img_, s->size_);
      s->img_ = nullptr;
    }
  }
  ~PortArtifactSource() {
    if (this->img_ != nullptr && this->state_.load() == READY)
      this->drop_(this->img_, this->size_);
  }

 protected:
  void drop_(uint8_t *img, size_t size) {
    if (img != nullptr && this->free_ != nullptr)
      this->free_(img, size);
  }
  FwImageInfo image_;
  std::string url_;
  StartFn start_{nullptr};
  FreeFn free_{nullptr};
  std::atomic<uint8_t> state_{IDLE};
  uint8_t *img_{nullptr};
  size_t size_{0};
  std::string error_;
};

}  // namespace esphome::ultra_line
