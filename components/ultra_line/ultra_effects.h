#pragma once
#ifdef USE_ESP32
#include "esphome/components/light/light_state.h"
#include "esphome/components/light/base_light_effects.h"
#include "esphome/core/hal.h"
#include "ultra_light.h"
#include "ultra_host_effects.h"
#include <cmath>

namespace esphome::ultra_line {

// Use ESPHome's implementations where their behaviour matches. UltraLight limits/coalesces their output SETs.
using UltraPulseEffect = light::PulseLightEffect;
using UltraStrobeEffect = light::StrobeLightEffect;
using UltraRandomEffect = light::RandomLightEffect;
class UltraFlickerEffect : public light::FlickerLightEffect {
 public:
  explicit UltraFlickerEffect(const char *name) : FlickerLightEffect(name) {}
  void set_update_interval(uint32_t ms) { interval_ = ms; }
  void start() override { light_ = UltraLight::find(state_->get_output()); cadence_.reset(); }
  void apply() override {
    if (light_ == nullptr || !cadence_.ready(millis(), std::max(interval_, light_->effect_interval()))) return;
    cadence_.sent(millis());
    FlickerLightEffect::apply();
  }
 protected:
  EffectCadence cadence_;
  uint32_t interval_{16};
  UltraLight *light_{nullptr};
};

// Component-only effects generate the next sample ahead of time: the board interpolates to it in 16 bits.
class UltraEffect : public light::LightEffect {
 public:
  explicit UltraEffect(const char *name) : LightEffect(name) {}
  void start() override {
    light_ = UltraLight::find(state_->get_output());
    start_ms_ = millis();
    cadence_.reset();
  }
  void apply() override {
    if (light_ == nullptr) return;
    const uint32_t now = millis(), interval = light_->effect_interval();
    if (!cadence_.ready(now, interval)) return;
    cadence_.sent(now);
    auto values = state_->remote_values;
    sample_(values, now - start_ms_ + interval);
    light_->send_effect_values(values, interval);
  }
 protected:
  virtual void sample_(light::LightColorValues &values, uint32_t ahead) = 0;
  UltraLight *light_{nullptr};
  EffectCadence cadence_;
  uint32_t start_ms_{0};
};

class UltraCandleEffect final : public UltraEffect {
 public:
  explicit UltraCandleEffect(const char *name) : UltraEffect(name) {}
  void set_intensity(float v) { intensity_ = v; }
  void set_speed(float v) { speed_ = v; }
 protected:
  void sample_(light::LightColorValues &v, uint32_t) override {
    // Flame-shaped random dips, filtered according to the elapsed bus cadence and requested flame speed.
    const float r = random_float();
    const float target = 1.0f - intensity_ * r * r;
    const float alpha = 1.0f - std::exp(-float(light_->effect_interval()) * speed_ / 180.0f);
    level_ += alpha * (target - level_);
    v.set_brightness(v.get_brightness() * level_);
  }
  float intensity_{0.4f}, speed_{1.0f}, level_{1.0f};
};

class UltraBreatheEffect final : public UltraEffect {
 public:
  explicit UltraBreatheEffect(const char *name) : UltraEffect(name) {}
  void set_period(uint32_t ms) { period_ms_ = ms; }
  void set_min_max_brightness(float low, float high) { min_ = low; max_ = high; }
 protected:
  void sample_(light::LightColorValues &v, uint32_t ahead) override {
    v.set_brightness(v.get_brightness() * breathe_level(start_ms_ + ahead, period_ms_, min_, max_));
  }
  uint32_t period_ms_{4000};
  float min_{0}, max_{1};
};

class UltraColorLoopEffect final : public UltraEffect {
 public:
  explicit UltraColorLoopEffect(const char *name) : UltraEffect(name) {}
  void set_period(uint32_t ms) { period_ms_ = ms; }
  void set_saturation(float v) { saturation_ = v; }
 protected:
  void sample_(light::LightColorValues &v, uint32_t ahead) override {
    const float h = float((start_ms_ + ahead) % period_ms_) * 6.0f / period_ms_;
    const float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f), m = 1.0f - saturation_;
    const int sector = int(h);
    v.set_red(m + saturation_ * (sector == 0 || sector == 5 ? 1 : sector == 1 || sector == 4 ? x : 0));
    v.set_green(m + saturation_ * (sector == 1 || sector == 2 ? 1 : sector == 0 || sector == 3 ? x : 0));
    v.set_blue(m + saturation_ * (sector == 3 || sector == 4 ? 1 : sector == 2 || sector == 5 ? x : 0));
    v.set_white(0); v.set_cold_white(0); v.set_warm_white(0);
  }
  uint32_t period_ms_{30000};
  float saturation_{1};
};

class UltraSunEffect final : public UltraEffect {
 public:
  UltraSunEffect(const char *name, bool rise) : UltraEffect(name), rise_(rise) {}
  void set_duration(uint32_t ms) { duration_ms_ = ms; }
  void start() override { done_ = false; UltraEffect::start(); }
  void apply() override {
    if (done_) return;
    if (uint32_t(millis() - start_ms_) >= duration_ms_) {
      done_ = true;
      auto call = rise_ ? state_->make_call() : state_->turn_off();
      call.set_effect(uint32_t(0));
      call.set_transition_length(0);
      call.perform();
      return;
    }
    UltraEffect::apply();
  }
 protected:
  void sample_(light::LightColorValues &v, uint32_t ahead) override {
    const float p = sun_progress(ahead, duration_ms_, rise_);
    v.set_brightness(v.get_brightness() * p);
    v.set_green(v.get_green() * (0.12f + 0.88f * p));
    v.set_blue(v.get_blue() * p * p);
    v.set_cold_white(v.get_cold_white() * p);
  }
  bool rise_, done_{false};
  uint32_t duration_ms_{900000};
};
}  // namespace esphome::ultra_line
#endif
