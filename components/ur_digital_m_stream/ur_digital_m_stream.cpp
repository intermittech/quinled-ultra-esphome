#include "ur_digital_m_stream.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstring>

namespace esphome::ur_digital_m_stream {

static const char *const TAG = "ur_digital_m_stream";

void URDigitalMStreamOutput::setup() {
#ifndef USE_DIG2ANALOG_UR_DIGITAL_M
  ESP_LOGE(TAG, "The data line is not a digital port (digital: on the line (ultra_line))");
  this->mark_failed();
  return;
#else
  if (this->hub_ == nullptr || !this->hub_->is_digital()) {
    ESP_LOGE(TAG, "The data line is not a digital port (digital: on the line (ultra_line))");
    this->mark_failed();
    return;
  }
  const size_t bytes = this->get_buffer_size_();
  RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->buf_ = allocator.allocate(bytes);
  this->effect_data_ = allocator.allocate(this->num_leds_);
  if (this->buf_ == nullptr || this->effect_data_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate the LED buffer (%u B)", static_cast<unsigned>(bytes));
    this->mark_failed();
    return;
  }
  std::memset(this->buf_, 0, bytes);
  std::memset(this->effect_data_, 0, this->num_leds_);
#endif
}

void URDigitalMStreamOutput::write_state(light::LightState *state) {
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->buf_ == nullptr)
    return;
  this->mark_shown_();
  if (state->remote_values.is_on()) this->hub_->scheduler().pixel_source(0, true);
  this->hub_->digital_pixels(this->buf_);   // (copied: the hub sends it between its frames, and never during an effect)
#endif
}

light::ESPColorView URDigitalMStreamOutput::get_view_internal(int32_t index) const {
  const light::ChannelColors &colors = this->channel_colors_;
  uint8_t *led = this->buf_ + (index * colors.bytes_per_led());
  return {led + colors.r,
          led + colors.g,
          led + colors.b,
          colors.has_white() ? led + colors.w : nullptr,
          &this->effect_data_[index],
          &this->correction_};
}

void URDigitalMStreamOutput::dump_config() {
  char channel_colors[5];
  ESP_LOGCONFIG(TAG,
                "Diff-Power Ultra Receiver Digital-Medium stream light:\n"
                "  Channel colors: %s\n"
                "  Number of LEDs: %u",
                this->channel_colors_.to_string(channel_colors), this->num_leds_);
}

}  // namespace esphome::ur_digital_m_stream

#endif  // USE_ESP32
