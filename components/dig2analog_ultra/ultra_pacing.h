#pragma once

// Home Assistant's database pacing (quinled-ultra-homeassistant: docs/decisions/esphome-requirements.md R12, R25): a
// value goes out when it moved by its deadband and its minimum interval has passed since the last one. A fade's end
// isn't dropped: the samples keep coming (every STATUS, every window), so the first one after the interval sends the
// settled value. A recorded power is the time-weighted mean of each interval (its live twin goes out at the update
// rate), so its statistics stay exact. The first value, NAN (unavailable) and a forced one (standby, a restart) go out
// at once. A user's own `filters:` replace the pacing (paced() off). Pure logic: tests/ultra_telemetry_test.cpp.

#include <cmath>
#include <cstdint>

namespace esphome::dig2analog_ultra {

/// A value's pacing: deadband (the larger of abs and rel x |the value sent|; 0: any change) and minimum interval
struct Pace {
  float abs;
  float rel;
  uint32_t min_ms;
};

/// The defaults (R12): power-like at the update rate (the live twins: 0.2 W or 1 %, the larger), recorded powers as
/// 10 s means
static constexpr Pace PACE_LIVE_W{0.2f, 0.01f, 0};           // 0.2 W or 1 % (min_ms: the update rate, per line)
static constexpr Pace PACE_POWER{0.5f, 0.02f, 10000};        // a 10 s mean, 0.5 W or 2 %
static constexpr Pace PACE_CURRENT{0.02f, 0.0f, 10000};
static constexpr Pace PACE_LOAD{1.0f, 0.0f, 10000};          // (%, integer)
static constexpr Pace PACE_VOLTAGE{0.2f, 0.0f, 10000};       // bus voltage, LED supply
static constexpr Pace PACE_TEMPERATURE{0.5f, 0.0f, 60000};
static constexpr Pace PACE_LOSS_W{0.2f, 0.0f, 10000};
static constexpr Pace PACE_LOSS_PCT{0.5f, 0.0f, 10000};
static constexpr Pace PACE_DROP_V{0.1f, 0.0f, 30000};
static constexpr Pace PACE_RESISTANCE{5.0f, 0.0f, 30000};    // (mOhm)
static constexpr Pace PACE_INPUT_A{0.02f, 0.0f, 30000};      // input current, its peak
static constexpr Pace PACE_ENERGY{0.0f, 0.0f, 300000};       // any growth, every 5 min (and at the shutdown)
static constexpr Pace PACE_MISSED{0.5f, 0.0f, 600000};
static constexpr Pace PACE_UPTIME{0.0f, 0.0f, 3600000};      // hourly (R15)
static constexpr Pace PACE_CHANGE{0.0f, 0.0f, 0};            // every change (counters, user filters)

/// v against what was sent (last; have: something was): due now?
inline bool pace_moved(const Pace &p, float v, float last, bool have) {
  if (!have || std::isnan(v) != std::isnan(last))
    return true;
  if (std::isnan(v))
    return false;
  const float db = std::fmax(p.abs, p.rel * std::fabs(last));
  return db > 0.0f ? std::fabs(v - last) >= db : v != last;
}

/// A paced value: when it was last sent (4 B; what was sent: the sensor's own raw state)
struct Paced {
  uint32_t sent_ms{0};
  /// v (last / have: what was sent) due now; force: at once (a state change: standby, a restart)
  bool due(const Pace &p, float v, float last, bool have, uint32_t now_ms, bool force = false) {
    if (!pace_moved(p, v, last, have))
      return false;
    if (!force && have && !std::isnan(v) && !std::isnan(last) && now_ms - this->sent_ms < p.min_ms)
      return false;
    this->sent_ms = now_ms;
    return true;
  }
};

/// A recorded power: the time-weighted mean of each min_ms window of its samples (each held until the next), sent at
/// the window's end when it moved by the deadband (16 B)
struct MeanPaced {
  uint32_t start_ms{0}, prev_ms{0};
  float acc{0.0f};   // W x ms / 1000 (W x s): enough for 10 s windows
  float prev{NAN};
  /// a sample; true: *out (the window's mean, or v at once) is due
  bool sample(const Pace &p, float v, float last, bool have, uint32_t now_ms, bool force, float *out) {
    if (std::isnan(v) || force || std::isnan(this->prev)) {   // a new run: unavailable, forced, after a gap
      this->prev = v;
      this->start_ms = this->prev_ms = now_ms;
      this->acc = 0.0f;
      *out = v;
      if (force)
        return !(have && (v == last || (std::isnan(v) && std::isnan(last))));
      return !have || std::isnan(v) != std::isnan(last);   // (the first value, or unavailable: at once)
    }
    this->acc += this->prev * static_cast<float>(now_ms - this->prev_ms) * 0.001f;
    this->prev = v;
    this->prev_ms = now_ms;
    const uint32_t span = now_ms - this->start_ms;
    if (span < p.min_ms || span == 0)
      return false;
    const float mean = this->acc / (static_cast<float>(span) * 0.001f);
    this->start_ms = now_ms;
    this->acc = 0.0f;
    *out = mean;
    return pace_moved(p, mean, last, have);
  }
};

}  // namespace esphome::dig2analog_ultra
