#include "ultra_digital_stream.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstring>

namespace esphome::ultra_digital_stream {

static const char *const TAG = "ultra_digital_stream";

void UltraDigitalStreamOutput::setup() {
#ifndef USE_DIG2ANALOG_ULTRA_DIGITAL
  ESP_LOGE(TAG, "The data line is not a digital port (digital: on the dig2analog_ultra line)");
  this->mark_failed();
  return;
#else
  if (this->hub_ == nullptr || !this->hub_->is_digital()) {
    ESP_LOGE(TAG, "The data line is not a digital port (digital: on the dig2analog_ultra line)");
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

void UltraDigitalStreamOutput::write_state(light::LightState *state) {
#ifdef USE_DIG2ANALOG_ULTRA_DIGITAL
  if (this->buf_ == nullptr)
    return;
  this->mark_shown_();
  this->hub_->digital_pixels(this->buf_);   // (copied: the hub sends it between its frames, and never during an effect)
#endif
}

light::ESPColorView UltraDigitalStreamOutput::get_view_internal(int32_t index) const {
  const light::ChannelColors &colors = this->channel_colors_;
  uint8_t *led = this->buf_ + (index * colors.bytes_per_led());
  return {led + colors.r,
          led + colors.g,
          led + colors.b,
          colors.has_white() ? led + colors.w : nullptr,
          &this->effect_data_[index],
          &this->correction_};
}

void UltraDigitalStreamOutput::dump_config() {
  char channel_colors[5];
  ESP_LOGCONFIG(TAG,
                "Ultra-Digital stream light:\n"
                "  Channel colors: %s\n"
                "  Number of LEDs: %u",
                this->channel_colors_.to_string(channel_colors), this->num_leds_);
}

}  // namespace esphome::ultra_digital_stream

#endif  // USE_ESP32
