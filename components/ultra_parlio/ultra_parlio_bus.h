#pragma once

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/ultra_line/ultra_parlio.h"

#include <atomic>
#include "esphome/components/ultra_line/ultra_transport.h"
#include <driver/gptimer.h>
#include <driver/parlio_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::ultra_line {

/// The ESP32-P4's PARLIO TX unit driving up to 16 lines of the QuinLED Ultra protocol (ultra_line hubs on its
/// lanes; ultra_parlio.h ParlioStream). Two
/// DMA buffers of one chunk each take turns (the driver's loop transmission switches to the newer one when the older
/// ends); a task renders the next chunk into the buffer just freed. One GPTimer, whose interrupt also runs with the
/// cache off, does two things from the stream's timing: it drops each lane's telemetry direction pin at its RW frame's
/// last bit, and it kills the stream when a chunk is late for its deadline, before the DMA could play a buffer twice
/// (the pins go to their idle level; the task restarts the stream).
class UltraParlioBus final : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE + 1.0f; }   // before its lanes' hubs

  // Configuration (codegen; the layout from ultra_line/parlio_timing.py)
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
  /// sample rate, bits per sample (8 / 16), samples per bit, groups (8 bits) per chunk, the handover margin (ns: the
  /// DMA's look-ahead), the buffers in PSRAM
  void set_layout(uint32_t rate_hz, uint8_t width, uint16_t period, uint32_t chunk_groups, uint32_t margin_ns,
                  bool psram) {
    this->rate_hz_ = rate_hz;
    this->width_ = width;
    this->period_ = period;
    this->chunk_groups_ = chunk_groups;
    this->margin_ns_ = margin_ns;
    this->psram_ = psram;
    // the lanes exist from here on (the hubs take them in their codegen; their setup() begins them)
    this->stream_.clock().configure(1000000000u / rate_hz, period, chunk_groups);
  }
  bool request_profile(uint8_t profile) {
    if (profile > 3 || this->rate_hz_ != 40000000 || this->profile_request_.load() != 255) return false;
    this->profile_request_.store(profile);
    xTaskNotifyGive(this->task_);
    return true;
  }
  uint8_t profile() const { return this->profile_.load(); }
  bool profile_pending() const { return this->profile_request_.load() != 255; }
  ParlioLane *lane(uint8_t k) { return this->stream_.lane(k); }

 protected:
  static bool on_switch_(parlio_tx_unit_handle_t unit, const parlio_tx_buffer_switched_event_data_t *edata,
                         void *arg);
  static bool on_alarm_(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
  static void render_task_(void *arg);
  void render_next_();
  bool start_stream_();
  bool change_profile_(uint8_t profile);
  std::atomic<uint8_t> profile_{0}, profile_request_{255};
  size_t buffer_capacity_{0};
  void restart_();
  void wait_releases_();
  void arm_();             // (under lock_) the timer at the stream's next alarm
  uint16_t stop_();        // (under lock_) the stream stops: the pins idle, the releases of the frames out by then
  void release_(uint16_t mask);   // (under lock_) the lanes' direction pins
  void pins_to_idle_();    // (in the interrupt) the data pins off the PARLIO, at their idle level
  void pins_to_parlio_();  // and back

  ParlioStream stream_;
  int8_t pins_[PARLIO_MAX_LANES]{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  ParlioLaneTiming timing_[PARLIO_MAX_LANES]{};
  uint8_t lanes_used_{0};
  uint16_t invert_{0};
  uint32_t rate_hz_{20000000};
  uint8_t width_{16};
  uint16_t period_{27};
  uint32_t chunk_groups_{96};
  uint32_t margin_ns_{255000};
  bool psram_{false};
  bool in_psram_{false};   // where the buffers went (internal memory first unless psram)

  uint8_t *buf_[2]{nullptr, nullptr};
  parlio_tx_unit_handle_t unit_{nullptr};
  parlio_transmit_config_t tx_cfg_{};
  TaskHandle_t task_{nullptr};
  void *volatile switched_to_{nullptr};   // the buffer the driver switched to (its interrupt)

  gptimer_handle_t timer_{nullptr};
  int64_t gp_base_us_{0};                 // the timer counts host us from here (1 MHz, free running)
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

  // diagnostics
  uint32_t render_us_max_{0}, render_us_sum_{0}, renders_{0};
  uint32_t restarts_{0}, restarts_logged_{0}, tx_errors_{0}, start_retries_{0};
  volatile uint32_t switches_{0};
  uint32_t switches_seen_{0};
  uint32_t log_ms_{0};
};

}  // namespace esphome::ultra_line

#endif  // USE_ESP32
