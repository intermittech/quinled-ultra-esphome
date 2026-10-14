#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_SENSOR)

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "dig2analog_ultra.h"

namespace esphome::dig2analog_ultra {

/// Diagnostics of one line (sensor platform dig2analog_ultra), each optional, published every update_interval: the
/// most commands queued for one board and the line's heap at its highest since the last update, the longest frame
/// build, RMT underruns since boot (counted only with that sensor) and frames per second.
class UltraDiagnostics final : public PollingComponent {
 public:
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_queue_peak(sensor::Sensor *s) { this->queue_peak_ = s; }
  void set_memory_peak(sensor::Sensor *s) { this->memory_peak_ = s; }
  void set_frame_build_time(sensor::Sensor *s) { this->build_time_ = s; }
  void set_rmt_underruns(sensor::Sensor *s) { this->underruns_ = s; }
  void set_frame_rate(sensor::Sensor *s) { this->frame_rate_ = s; }

  void setup() override;
  void update() override;
  void dump_config() override;

 protected:
  UltraHub *hub_{nullptr};
  sensor::Sensor *queue_peak_{nullptr};
  sensor::Sensor *memory_peak_{nullptr};
  sensor::Sensor *build_time_{nullptr};
  sensor::Sensor *underruns_{nullptr};
  sensor::Sensor *frame_rate_{nullptr};
  uint32_t last_ms_{0}, last_frames_{0}, last_underruns_{0};
};

}  // namespace esphome::dig2analog_ultra

#endif
