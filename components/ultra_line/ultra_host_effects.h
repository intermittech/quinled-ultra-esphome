#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace esphome::ultra_line {

// Streamed effect SETs have one copy. Reserve at most 30% of the physical
// slot capacity; requested light rates share the same capacity scaling.
inline uint32_t effect_interval_for_demand(uint32_t demand, uint16_t rate, uint16_t boards, uint32_t frame_us) {
  rate = std::clamp<uint16_t>(rate, 1, 20);
  const uint32_t requested = (1000u + rate - 1) / rate;
  if (!boards) return requested;
  const uint64_t numerator = static_cast<uint64_t>(demand) * frame_us;
  const uint64_t denominator = static_cast<uint64_t>(rate) * boards * 300u;
  return std::max<uint32_t>(requested, (numerator + denominator - 1) / denominator);
}
inline uint32_t analog_effect_interval(size_t lights, uint16_t boards, uint32_t frame_us, uint16_t rate = 20) {
  return effect_interval_for_demand(lights * rate, rate, boards, frame_us);
}

class EffectCadence {
 public:
  bool ready(uint32_t now, uint32_t interval) const { return !sent_ || uint32_t(now - last_) >= interval; }
  void sent(uint32_t now) { last_ = now; sent_ = true; }
  void reset() { sent_ = false; }
 private:
  uint32_t last_{0};
  bool sent_{false};
};

inline float breathe_level(uint32_t at, uint32_t period, float low, float high) {
  const float phase = float(at % period) / period;
  return low + (high - low) * (0.5f - 0.5f * std::cos(phase * 6.28318530718f));
}

inline float sun_progress(uint32_t elapsed, uint32_t duration, bool rise) {
  const float p = std::min(1.0f, float(elapsed) / duration);
  return rise ? p : 1.0f - p;
}

}  // namespace esphome::ultra_line
