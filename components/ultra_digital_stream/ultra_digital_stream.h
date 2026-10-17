#pragma once

#ifdef USE_ESP32

#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/channel_colors.h"
#include "esphome/components/light/light_output.h"
#include "esphome/core/component.h"

#include "esphome/components/dig2analog_ultra/dig2analog_ultra.h"

namespace esphome::ultra_digital_stream {

using dig2analog_ultra::UltraHub;

/// An addressable (ESPHome) light on the strip behind a QuinLED Diff-Power Ultra-Digital receiver: its frames are not sent
/// by an RMT channel of its own but handed to the digital port's hub (dig2analog_ultra with digital:), which sends them
/// between its marker frames, and not at all while the receiver shows a local effect (ultra_digital). All ESPHome
/// addressable effects work as on any other strip.
class UltraDigitalStreamOutput final : public light::AddressableLight {
 public:
  void setup() override;
  void write_state(light::LightState *state) override;
  void dump_config() override;
  /// after the hub (its buffers are allocated in its setup)
  float get_setup_priority() const override { return setup_priority::LATE; }

  int32_t size() const override { return this->num_leds_; }
  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    if (this->channel_colors_.has_white()) {
      traits.set_supported_color_modes({light::ColorMode::RGB_WHITE, light::ColorMode::WHITE});
    } else {
      traits.set_supported_color_modes({light::ColorMode::RGB});
    }
    return traits;
  }

  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_num_leds(uint16_t num_leds) { this->num_leds_ = num_leds; }
  void set_channel_colors(light::ChannelColors channel_colors) { this->channel_colors_ = channel_colors; }

  void clear_effect_data() override {
    for (int i = 0; i < this->size(); i++)
      this->effect_data_[i] = 0;
  }

 protected:
  light::ESPColorView get_view_internal(int32_t index) const override;
  size_t get_buffer_size_() const { return static_cast<size_t>(this->num_leds_) * this->channel_colors_.bytes_per_led(); }

  UltraHub *hub_{nullptr};
  uint8_t *buf_{nullptr};
  uint8_t *effect_data_{nullptr};
  uint16_t num_leds_{0};
  light::ChannelColors channel_colors_{0, 1, 2, light::ChannelColors::NO_WHITE};
};

}  // namespace esphome::ultra_digital_stream

#endif  // USE_ESP32
