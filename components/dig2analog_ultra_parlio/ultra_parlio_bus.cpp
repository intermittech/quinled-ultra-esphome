#include "ultra_parlio_bus.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_memory_utils.h>
#include <esp_timer.h>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra_parlio";

static constexpr size_t BUFFER_ALIGN = 128;      // DMA and cache line alignment (internal memory and PSRAM)
static constexpr uint32_t RENDER_PRIORITY = 20;   // above ESPHome's loop and the network tasks
static constexpr BaseType_t RENDER_CORE = 1;      // off the core of ESPHome's main loop
static constexpr int64_t RENDER_MARGIN_NS = 30000;   // a chunk handed over this close to the switch may be late

// The driver switched to the newer buffer (the older one ended and is free): the task renders the next chunk into it.
bool IRAM_ATTR UltraParlioBus::on_switch_(parlio_tx_unit_handle_t, const parlio_tx_buffer_switched_event_data_t *,
                                          void *arg) {
  auto *bus = static_cast<UltraParlioBus *>(arg);
  bus->switch_us_ = esp_timer_get_time();
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(bus->task_, &woken);
  return woken == pdTRUE;
}

// The timer: every lane whose RW frame's last bit is out lets go of its line; then the next release.
bool IRAM_ATTR UltraParlioBus::on_alarm_(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *arg) {
  auto *bus = static_cast<UltraParlioBus *>(arg);
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&bus->release_lock_);
  for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
    if (bus->release_at_[k] != 0 && bus->release_at_[k] <= now + 1) {
      bus->release_at_[k] = 0;
      bus->lanes_[k].release_now();
    }
  }
  bus->arm_();
  portEXIT_CRITICAL_ISR(&bus->release_lock_);
  return false;
}

// (under release_lock_) the alarm at the earliest release
void IRAM_ATTR UltraParlioBus::arm_() {
  int64_t first = 0;
  for (int64_t at : this->release_at_) {
    if (at != 0 && (first == 0 || at < first))
      first = at;
  }
  if (first == 0)
    return;
  // an alarm the counter already passed would wait for its wrap: then 2 us from now
  uint64_t raw = 0;
  gptimer_get_raw_count(this->timer_, &raw);
  gptimer_alarm_config_t alarm{};
  alarm.alarm_count = std::max<uint64_t>(static_cast<uint64_t>(first - this->gp_base_us_), raw + 2);
  gptimer_set_alarm_action(this->timer_, &alarm);
}

void UltraParlioBus::schedule_release_(uint8_t lane, int64_t at_us) {
  if (esp_timer_get_time() + 2 >= at_us) {   // (only when the render was late) at once
    this->lanes_[lane].release_now();
    return;
  }
  portENTER_CRITICAL(&this->release_lock_);
  this->release_at_[lane] = at_us;
  this->arm_();
  portEXIT_CRITICAL(&this->release_lock_);
}

void UltraParlioBus::render_task_(void *arg) {
  auto *bus = static_cast<UltraParlioBus *>(arg);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    bus->render_next_();
  }
}

// The next chunk into the buffer just freed, handed to the driver (it plays after the one that just started).
void UltraParlioBus::render_next_() {
  const int64_t t_start = esp_timer_get_time();
  ParlioClock::State &st = this->state_;
  const int64_t ready_ns = (t_start * 1000) + this->render_ns_est_ + RENDER_MARGIN_NS;
  const int64_t moved = parlio_slip(st, this->clock_.chunk_ns(), this->switch_us_ * 1000, ready_ns);
  if (moved != 0) {
    this->slips_ += static_cast<uint32_t>(moved > 0 ? moved : -moved);
    // the releases scheduled before: their frames move with the stream
    const int64_t shift_us = moved * static_cast<int64_t>(this->clock_.chunk_ns() / 1000);
    portENTER_CRITICAL(&this->release_lock_);
    for (int64_t &at : this->release_at_) {
      if (at != 0)
        at += shift_us;
    }
    this->arm_();
    portEXIT_CRITICAL(&this->release_lock_);
  }
  const int64_t chunk = st.next_chunk;
  st.next_chunk = chunk + 1;
  this->clock_.write(st);   // hubs reserve from the chunk after it on
  ParlioLane *lanes[PARLIO_MAX_LANES];
  for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++)
    lanes[k] = &this->lanes_[k];
  uint8_t *buf = this->buf_[chunk & 1];
  this->renderer_.render(chunk, static_cast<uint8_t>(chunk & 1), buf, lanes, &this->clock_, st);
  for (uint8_t i = 0; i < this->renderer_.releases(); i++) {
    const ParlioRenderer::Release &r = this->renderer_.release(i);
    this->schedule_release_(r.lane, (r.at_ns + 999) / 1000);
  }
  parlio_tx_unit_transmit(this->unit_, buf, this->renderer_.chunk_bytes() * 8, &this->tx_cfg_);
  const int64_t took = esp_timer_get_time() - t_start;
  this->render_ns_est_ += (took * 1000 - this->render_ns_est_) / 8;
  this->render_us_max_ = std::max(this->render_us_max_, static_cast<uint32_t>(took));
  this->render_us_sum_ += static_cast<uint32_t>(took);
  this->renders_++;
}

void UltraParlioBus::setup() {
  if (!this->renderer_.init(this->width_, this->period_, this->clock_.chunk_groups(), this->timing_,
                            this->lanes_used_, this->invert_)) {
    ESP_LOGE(TAG, "Timing does not fit the sample grid");
    this->mark_failed();
    return;
  }
  const size_t bytes = this->renderer_.chunk_bytes();
  for (int i = 0; i < 2; i++) {
    const uint32_t internal = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
    const uint32_t spiram = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    this->buf_[i] = static_cast<uint8_t *>(heap_caps_aligned_calloc(BUFFER_ALIGN, 1, bytes, this->psram_ ? spiram : internal));
    if (this->buf_[i] == nullptr)   // the other memory
      this->buf_[i] = static_cast<uint8_t *>(heap_caps_aligned_calloc(BUFFER_ALIGN, 1, bytes, this->psram_ ? internal : spiram));
    if (this->buf_[i] == nullptr) {
      ESP_LOGE(TAG, "Cannot allocate the stream's buffers (2 x %u B)", static_cast<unsigned>(bytes));
      this->mark_failed();
      return;
    }
    this->in_psram_ |= esp_ptr_external_ram(this->buf_[i]);
    this->renderer_.clear(this->buf_[i], static_cast<uint8_t>(i));
  }

  // the releases' timer: 1 MHz, free running, on the host clock
  gptimer_config_t tc{};
  tc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  tc.direction = GPTIMER_COUNT_UP;
  tc.resolution_hz = 1000000;
  gptimer_event_callbacks_t tcb{};
  tcb.on_alarm = UltraParlioBus::on_alarm_;
  if (gptimer_new_timer(&tc, &this->timer_) != ESP_OK ||
      gptimer_register_event_callbacks(this->timer_, &tcb, this) != ESP_OK || gptimer_enable(this->timer_) != ESP_OK ||
      gptimer_start(this->timer_) != ESP_OK) {
    ESP_LOGE(TAG, "GPTimer setup failed");
    this->mark_failed();
    return;
  }
  {
    uint64_t raw = 0;
    portENTER_CRITICAL(&this->release_lock_);
    const int64_t now = esp_timer_get_time();
    gptimer_get_raw_count(this->timer_, &raw);
    this->gp_base_us_ = now - static_cast<int64_t>(raw);
    portEXIT_CRITICAL(&this->release_lock_);
  }

  parlio_tx_unit_config_t cfg{};
  cfg.clk_src = PARLIO_CLK_SRC_DEFAULT;
  cfg.clk_in_gpio_num = GPIO_NUM_NC;
  cfg.output_clk_freq_hz = this->rate_hz_;
  cfg.data_width = this->width_;
  for (int i = 0; i < PARLIO_TX_UNIT_MAX_DATA_WIDTH; i++)
    cfg.data_gpio_nums[i] = i < PARLIO_MAX_LANES && this->pins_[i] >= 0 ? static_cast<gpio_num_t>(this->pins_[i]) : GPIO_NUM_NC;
  cfg.clk_out_gpio_num = GPIO_NUM_NC;
  cfg.valid_gpio_num = GPIO_NUM_NC;
  cfg.trans_queue_depth = 2;
  cfg.max_transfer_size = bytes;
  cfg.shift_edge = PARLIO_SHIFT_EDGE_POS;
  cfg.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
  if (parlio_new_tx_unit(&cfg, &this->unit_) != ESP_OK) {
    ESP_LOGE(TAG, "PARLIO TX unit creation failed");
    this->mark_failed();
    return;
  }
  parlio_tx_event_callbacks_t cbs{};
  cbs.on_buffer_switched = UltraParlioBus::on_switch_;
  if (parlio_tx_unit_register_event_callbacks(this->unit_, &cbs, this) != ESP_OK ||
      parlio_tx_unit_enable(this->unit_) != ESP_OK) {
    ESP_LOGE(TAG, "PARLIO TX unit enable failed");
    this->mark_failed();
    return;
  }
  if (xTaskCreatePinnedToCore(UltraParlioBus::render_task_, "d2a_parlio", 4096, this, RENDER_PRIORITY, &this->task_,
                              RENDER_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Cannot start the render task");
    this->mark_failed();
    return;
  }

  // the stream: chunk 0 (idle) plays and loops, chunk 1 (idle) waits; the switch to it renders chunk 2
  this->tx_cfg_.idle_value = this->invert_;
  this->tx_cfg_.flags.loop_transmission = 1;
  if (parlio_tx_unit_transmit(this->unit_, this->buf_[0], bytes * 8, &this->tx_cfg_) != ESP_OK) {
    ESP_LOGE(TAG, "PARLIO transmit failed");
    this->mark_failed();
    return;
  }
  const int64_t t0 = esp_timer_get_time();   // (the first sample went out just before)
  this->switch_us_ = t0 + static_cast<int64_t>(this->clock_.chunk_ns() / 1000);
  this->state_.t0_ns = t0 * 1000;
  this->state_.next_chunk = 2;
  this->state_.running = true;
  parlio_tx_unit_transmit(this->unit_, this->buf_[1], bytes * 8, &this->tx_cfg_);
  this->clock_.write(this->state_);
  this->log_ms_ = millis();
}

void UltraParlioBus::loop() {
  // the stream slipped (the render task was late: a chunk played twice): frames in flight were lost
  const uint32_t slips = this->slips_;
  if (slips != this->slips_logged_ && millis() - this->log_ms_ > 10000) {
    ESP_LOGW(TAG, "The stream slipped %" PRIu32 " chunk(s) so far (the render task was late; frames lost)", slips);
    this->slips_logged_ = slips;
    this->log_ms_ = millis();
  }
}

void UltraParlioBus::dump_config() {
  const uint32_t ns = this->clock_.sample_ns();
  ESP_LOGCONFIG(TAG,
                "QuinLED dig2analog Ultra PARLIO bus:\n"
                "  Lanes: %u, %u-bit samples at %" PRIu32 " Hz (%" PRIu32 " ns)\n"
                "  Bit: %u samples (%" PRIu32 " ns), a group of 8 bits %" PRIu32 " ns\n"
                "  Buffers: 2 x %u B in %s, a chunk of %" PRIu32 " groups (%" PRIu32 " us)\n"
                "  Render: %" PRIu32 " us per chunk on average, %" PRIu32 " us at most; %" PRIu32 " slip(s)",
                this->lanes_used_, this->width_, this->rate_hz_, ns, this->period_, this->period_ * ns,
                this->clock_.group_ns(), static_cast<unsigned>(this->renderer_.chunk_bytes()),
                this->in_psram_ ? "PSRAM" : "internal RAM", this->clock_.chunk_groups(),
                static_cast<uint32_t>(this->clock_.chunk_ns() / 1000),
                this->renders_ ? this->render_us_sum_ / this->renders_ : 0, this->render_us_max_,
                static_cast<uint32_t>(this->slips_));
  for (uint8_t k = 0; k < this->lanes_used_; k++) {
    if (this->pins_[k] < 0)
      continue;
    ESP_LOGCONFIG(TAG, "  Lane %u: GPIO%d%s, '0' %u / '1' %u samples high", k, this->pins_[k],
                  (this->invert_ >> k) & 1 ? " (inverted)" : "", this->timing_[k].h0, this->timing_[k].h1);
  }
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup FAILED");
}

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
