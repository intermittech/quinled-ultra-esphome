#pragma once

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/dig2analog_ultra/ultra_parlio.h"

#include <driver/gptimer.h>
#include <driver/parlio_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::dig2analog_ultra {

/// The ESP32-P4's PARLIO TX unit driving up to 16 dig2analog_ultra lines (lanes; ultra_parlio.h). Its stream never
/// stops: two DMA buffers of one chunk each take turns (the driver's loop transmission switches to the newer one when
/// the older ends), and a task renders the next chunk into the buffer just freed. A GPTimer drops each lane's telemetry
/// direction pin at its RW frame's last bit, from the stream's timing.
class UltraParlioBus final : public Component {
 public:
  UltraParlioBus() {
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++)
      this->lanes_[k].attach(&this->clock_, k);
  }
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE + 1.0f; }   // before its lanes' hubs

  // Configuration (codegen; the layout from dig2analog_ultra/parlio_timing.py)
  void set_pin(uint8_t lane, uint8_t gpio, bool inverted) {
    if (lane >= PARLIO_MAX_LANES)
      return;
    this->pins_[lane] = static_cast<int8_t>(gpio);
    if (inverted)
      this->invert_ |= static_cast<uint16_t>(1u << lane);
    if (lane >= this->lanes_used_)
      this->lanes_used_ = lane + 1;
  }
  void set_lane_timing(uint8_t lane, uint16_t h0, uint16_t h1) {
    if (lane < PARLIO_MAX_LANES)
      this->timing_[lane] = ParlioLaneTiming{h0, h1};
  }
  /// sample rate, bits per sample (8 / 16), samples per bit, groups (8 bits) per chunk; the buffers in PSRAM
  void set_layout(uint32_t rate_hz, uint8_t width, uint16_t period, uint32_t chunk_groups, bool psram) {
    this->rate_hz_ = rate_hz;
    this->width_ = width;
    this->period_ = period;
    this->psram_ = psram;
    this->clock_.configure(1000000000u / rate_hz, period, chunk_groups);
  }
  ParlioLane *lane(uint8_t k) { return &this->lanes_[k < PARLIO_MAX_LANES ? k : 0]; }

 protected:
  static bool on_switch_(parlio_tx_unit_handle_t unit, const parlio_tx_buffer_switched_event_data_t *edata,
                         void *arg);
  static bool on_alarm_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
  static void render_task_(void *arg);
  void render_next_();
  void schedule_release_(uint8_t lane, int64_t at_us);
  void arm_();   // (under release_lock_)

  ParlioLane lanes_[PARLIO_MAX_LANES];
  ParlioClock clock_;
  ParlioRenderer renderer_;
  ParlioClock::State state_{};   // the render task's copy of the clock (it is the only writer)
  int8_t pins_[PARLIO_MAX_LANES]{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  ParlioLaneTiming timing_[PARLIO_MAX_LANES]{};
  uint8_t lanes_used_{0};
  uint16_t invert_{0};
  uint32_t rate_hz_{20000000};
  uint8_t width_{16};
  uint16_t period_{27};
  bool psram_{false};
  bool in_psram_{false};   // where the buffers went (internal memory first unless psram)

  uint8_t *buf_[2]{nullptr, nullptr};
  parlio_tx_unit_handle_t unit_{nullptr};
  parlio_transmit_config_t tx_cfg_{};
  TaskHandle_t task_{nullptr};
  volatile int64_t switch_us_{0};      // the newest buffer started (the switch interrupt)

  // the direction pins' releases: a GPTimer at 1 MHz, free running from gp_base_us_ on the host clock
  gptimer_handle_t timer_{nullptr};
  int64_t gp_base_us_{0};
  int64_t release_at_[PARLIO_MAX_LANES]{};   // host us, 0 = none
  portMUX_TYPE release_lock_ = portMUX_INITIALIZER_UNLOCKED;

  // diagnostics
  uint32_t render_us_max_{0}, render_us_sum_{0}, renders_{0};
  int64_t render_ns_est_{0};          // the render's time, smoothed: a late one is foreseen
  volatile uint32_t slips_{0};        // chunks played again (the task was late) or foreseen as late
  uint32_t tx_errors_{0};             // buffers the driver refused
  volatile uint32_t switches_{0};     // buffer switches (the interrupt), and as last seen by loop()
  uint32_t switches_seen_{0};
  uint32_t slips_logged_{0};
  uint32_t log_ms_{0};
};

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
