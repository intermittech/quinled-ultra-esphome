#include "ultra_diagnostics.h"

#if defined(USE_ESP32) && defined(USE_SENSOR)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra.diagnostics";

void UltraDiagnostics::setup() {
  if (this->underruns_ != nullptr)
    this->hub_->watch_underruns();
  this->last_ms_ = millis();
}

void UltraDiagnostics::update() {
  const UltraHub::Diagnostics d = this->hub_->take_diagnostics();
  const uint32_t now = millis();
  if (this->queue_peak_ != nullptr)
    this->queue_peak_->publish_state(static_cast<float>(d.queue_peak));
  if (this->memory_peak_ != nullptr)
    this->memory_peak_->publish_state(static_cast<float>(d.memory_peak));
  if (this->build_time_ != nullptr)
    this->build_time_->publish_state(static_cast<float>(d.build_us));
  if (this->underruns_ != nullptr) {
    this->underruns_->publish_state(static_cast<float>(d.underruns));
    if (d.underruns != this->last_underruns_ && this->hub_->on_parlio())
      ESP_LOGW(TAG,
               "GPIO%u: %" PRIu32 " frame(s) lost to a slip of the PARLIO stream since the last update (its render "
               "task was late)",
               this->hub_->get_pin(), d.underruns - this->last_underruns_);
    else if (d.underruns != this->last_underruns_)
      ESP_LOGW(TAG,
               "GPIO%u: %" PRIu32 " RMT underrun(s) since the last update: those frames were broken (interrupts "
               "held up; use_dma or a larger rmt_symbols give the RMT more time)",
               this->hub_->get_pin(), d.underruns - this->last_underruns_);
    this->last_underruns_ = d.underruns;
  }
  if (this->frame_rate_ != nullptr && now != this->last_ms_)
    this->frame_rate_->publish_state(static_cast<float>(d.frames - this->last_frames_) * 1000.0f /
                                     static_cast<float>(now - this->last_ms_));
  this->last_ms_ = now;
  this->last_frames_ = d.frames;
}

void UltraDiagnostics::dump_config() {
  ESP_LOGCONFIG(TAG, "QuinLED Ultra diagnostics (GPIO%u):", this->hub_->get_pin());
  LOG_SENSOR("  ", "Queue peak", this->queue_peak_);
  LOG_SENSOR("  ", "Memory peak", this->memory_peak_);
  LOG_SENSOR("  ", "Frame build time", this->build_time_);
  LOG_SENSOR("  ", "RMT underruns", this->underruns_);
  LOG_SENSOR("  ", "Frame rate", this->frame_rate_);
}

}  // namespace esphome::dig2analog_ultra

#endif
