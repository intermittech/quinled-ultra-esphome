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
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/parlio_periph.h>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra_parlio";

static constexpr size_t BUFFER_ALIGN = 128;   // DMA and cache line alignment (internal memory and PSRAM)
// The render task: priority 20 on core 1, ESPHome's loop core (its loop task runs at priority 1 and is preempted for
// the render's ~0.1 ms per chunk); core 0 carries the network stack, IPC and esp_timer tasks at priorities around and
// above 20, which would delay the render. Its lateness is bounded either way: a chunk late for its deadline kills the
// stream safely (ParlioStream).
static constexpr uint32_t RENDER_PRIORITY = 20;
static constexpr BaseType_t RENDER_CORE = 1;
static constexpr int START_TRIES = 4;   // a (re)start whose time is too uncertain is tried again; the last takes any
static constexpr uint32_t RESTART_WAIT_MS = 10;   // the driver refused a start: again after this

// The driver switched to the newer buffer (the older one ended and is free): the task renders the next chunk into it.
bool IRAM_ATTR UltraParlioBus::on_switch_(parlio_tx_unit_handle_t, const parlio_tx_buffer_switched_event_data_t *edata,
                                          void *arg) {
  auto *bus = static_cast<UltraParlioBus *>(arg);
  bus->switched_to_ = edata->new_buffer_addr;
  bus->switches_ = bus->switches_ + 1;
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(bus->task_, &woken);
  return woken == pdTRUE;
}

// The timer, also with the cache off (CONFIG_GPTIMER_ISR_CACHE_SAFE, its control functions in IRAM; everything it
// touches is in internal RAM): the direction pins of the RW frames that ended, and the render deadline. (The replay
// of a buffer can't start before the playing chunk ends, a margin after the deadline: this interrupt has that long.)
bool IRAM_ATTR UltraParlioBus::on_alarm_(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *arg) {
  auto *bus = static_cast<UltraParlioBus *>(arg);
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&bus->lock_);
  const bool kill = bus->stream_.late(now);
  const uint16_t mask = kill ? bus->stop_() : bus->stream_.releases_due(now);
  bus->release_(mask);
  bus->arm_();
  portEXIT_CRITICAL_ISR(&bus->lock_);
  BaseType_t woken = pdFALSE;
  if (kill)
    vTaskNotifyGiveFromISR(bus->task_, &woken);   // the task restarts the stream
  return woken == pdTRUE;
}

// (under lock_) the timer at the stream's next alarm; an alarm already passed fires at once
void IRAM_ATTR UltraParlioBus::arm_() {
  const int64_t at = this->stream_.next_alarm_us();
  if (at == 0)
    return;
  gptimer_alarm_config_t alarm{};
  alarm.alarm_count = static_cast<uint64_t>(at - this->gp_base_us_);
  gptimer_set_alarm_action(this->timer_, &alarm);
}

// (under lock_) the stream stops here: the pins idle first (every frame on the wire is cut there), then the stream
// keeps the releases of the frames that were out by then
uint16_t IRAM_ATTR UltraParlioBus::stop_() {
  this->pins_to_idle_();
  return this->stream_.stop(esp_timer_get_time());
}

void IRAM_ATTR UltraParlioBus::release_(uint16_t mask) {
  for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
    if (mask & (1u << k))
      this->stream_.lane(k)->release_now();
  }
}

void IRAM_ATTR UltraParlioBus::pins_to_idle_() {
  for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
    const int8_t pin = this->pins_[k];
    if (pin < 0)
      continue;
    gpio_ll_set_level(&GPIO, pin, (this->invert_ >> k) & 1u);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
  }
}

void UltraParlioBus::pins_to_parlio_() {
  for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
    if (this->pins_[k] >= 0)
      esp_rom_gpio_connect_out_signal(this->pins_[k], parlio_periph_signals.groups[0].tx_units[0].data_sigs[k], false,
                                      false);
  }
}

void UltraParlioBus::render_task_(void *arg) {
  auto *bus = static_cast<UltraParlioBus *>(arg);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (bus->stream_.killed())
      bus->restart_();
    else
      bus->render_next_();
  }
}

// The next chunk into the buffer just freed, handed to the driver (it plays after the one that just started).
void UltraParlioBus::render_next_() {
  const int64_t t_start = esp_timer_get_time();
  const int64_t chunk = this->stream_.next_chunk();
  // the buffer the driver switched to must be the chunk before this one: else something went wrong, start over
  if (this->switched_to_ != this->buf_[(chunk - 1) & 1]) {
    portENTER_CRITICAL(&this->lock_);
    if (!this->stream_.killed())
      this->release_(this->stop_());
    portEXIT_CRITICAL(&this->lock_);
    this->restart_();
    return;
  }
  void *bufs[2] = {this->buf_[0], this->buf_[1]};
  this->stream_.render(bufs);
  if (parlio_tx_unit_transmit(this->unit_, this->buf_[chunk & 1], this->stream_.chunk_bytes() * 8, &this->tx_cfg_) !=
      ESP_OK) {
    this->tx_errors_++;   // (the deadline kills the stream)
    return;
  }
  // with the DMA in time only if the transmit call returned before the deadline (the timer may not have run yet)
  bool late = false;
  portENTER_CRITICAL(&this->lock_);
  if (!this->stream_.killed()) {   // (else the timer stopped it: its notification restarts it)
    if (this->stream_.handed_over(chunk, esp_timer_get_time())) {
      this->arm_();
    } else {
      this->release_(this->stop_());
      late = true;
    }
  }
  portEXIT_CRITICAL(&this->lock_);
  if (late) {
    this->restart_();
    return;
  }
  const uint32_t took = static_cast<uint32_t>(esp_timer_get_time() - t_start);
  this->render_us_max_ = std::max(this->render_us_max_, took);
  this->render_us_sum_ += took;
  this->renders_++;
}

// Two idle chunks, the first starting within the first transmit call: the stream's (re)start. Its time is known between
// the host clock before that call and after it (the driver starts the unit in it); a call that took too long (the task
// preempted) gives too wide a window: the DMA stops (it played idle samples only) and it is tried again, the last try
// taking any (the deadlines count from the window's start, so it is safe, only the releases come later). The second
// chunk must be with the DMA by its deadline too. false: the driver refused.
bool UltraParlioBus::start_stream_() {
  for (int attempt = 1;; attempt++) {
    const int64_t first = this->stream_.next_chunk();
    for (int i = 0; i < 2; i++)
      this->stream_.clear(this->buf_[i], static_cast<uint8_t>(i));
    if (parlio_tx_unit_enable(this->unit_) != ESP_OK)
      return false;
    const int64_t lo = esp_timer_get_time();
    if (parlio_tx_unit_transmit(this->unit_, this->buf_[first & 1], this->stream_.chunk_bytes() * 8, &this->tx_cfg_) !=
        ESP_OK)
      return false;
    const int64_t hi = esp_timer_get_time();
    this->switched_to_ = this->buf_[first & 1];
    if (parlio_tx_unit_transmit(this->unit_, this->buf_[(first + 1) & 1], this->stream_.chunk_bytes() * 8,
                                &this->tx_cfg_) != ESP_OK)
      return false;
    portENTER_CRITICAL(&this->lock_);
    const bool started = this->stream_.start(first, lo * 1000, hi * 1000, esp_timer_get_time(), attempt >= START_TRIES);
    if (started)
      this->arm_();
    portEXIT_CRITICAL(&this->lock_);
    if (started)
      return true;
    this->start_retries_++;
    parlio_tx_unit_disable(this->unit_);
    if (attempt >= START_TRIES + 2)
      return false;   // (the second chunk late every time: the caller tries again later)
  }
}

// After a stop: the DMA stopped, every frame on the wire dropped, the stream started again with idle chunks; then the
// pins back on the PARLIO (it plays idle chunks by then). A start the driver refused is tried again.
void UltraParlioBus::restart_() {
  parlio_tx_unit_disable(this->unit_);
  this->stream_.drop_all();
  this->restarts_++;
  while (!this->start_stream_()) {
    this->tx_errors_++;   // (the pins stay idle meanwhile; loop() reports it)
    parlio_tx_unit_disable(this->unit_);
    vTaskDelay(pdMS_TO_TICKS(RESTART_WAIT_MS));
  }
  this->pins_to_parlio_();
}

void UltraParlioBus::setup() {
  if (!this->stream_.init(this->timing_, this->lanes_used_, this->invert_, 1000000000u / this->rate_hz_, this->width_,
                          this->period_, this->chunk_groups_, this->margin_ns_)) {
    ESP_LOGE(TAG, "Timing does not fit the sample grid");
    this->mark_failed();
    return;
  }
  const size_t bytes = this->stream_.chunk_bytes();
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
  }

  // the timer: 1 MHz, free running, on the host clock
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
    portENTER_CRITICAL(&this->lock_);
    const int64_t now = esp_timer_get_time();
    gptimer_get_raw_count(this->timer_, &raw);
    this->gp_base_us_ = now - static_cast<int64_t>(raw);
    portEXIT_CRITICAL(&this->lock_);
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
  if (parlio_tx_unit_register_event_callbacks(this->unit_, &cbs, this) != ESP_OK) {
    ESP_LOGE(TAG, "PARLIO TX callback failed");
    this->mark_failed();
    return;
  }
  if (xTaskCreatePinnedToCore(UltraParlioBus::render_task_, "d2a_parlio", 4096, this, RENDER_PRIORITY, &this->task_,
                              RENDER_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Cannot start the render task");
    this->mark_failed();
    return;
  }
  // the stream: chunk 0 (idle) plays, chunk 1 (idle) waits; the switch to it renders chunk 2
  this->tx_cfg_.idle_value = this->invert_;
  this->tx_cfg_.flags.loop_transmission = 1;
  if (!this->start_stream_()) {
    ESP_LOGE(TAG, "PARLIO transmit failed");
    this->mark_failed();
    return;
  }
  this->log_ms_ = millis();
}

void UltraParlioBus::loop() {
  if (millis() - this->log_ms_ < 10000)
    return;
  this->log_ms_ = millis();
  // a chunk was late for its deadline (the render task held up, e.g. by flash writes with the cache off): the stream
  // was stopped before any buffer could play twice and started again; the frames on the wire then were lost
  const uint32_t restarts = this->restarts_;
  if (restarts != this->restarts_logged_) {
    ESP_LOGW(TAG, "The stream restarted %" PRIu32 " time(s) so far (the render task was late; frames lost)", restarts);
    this->restarts_logged_ = restarts;
  }
  // no buffer switch in 10 s: the stream stopped (the driver refused a buffer)
  const uint32_t switches = this->switches_;
  if (switches == this->switches_seen_ || this->tx_errors_ != 0) {
    ESP_LOGE(TAG, "The stream stopped (%" PRIu32 " buffer(s) refused by the driver)", this->tx_errors_);
    this->status_set_error();
  }
  this->switches_seen_ = switches;
}

void UltraParlioBus::dump_config() {
  const uint32_t ns = this->stream_.clock().sample_ns();
  ESP_LOGCONFIG(TAG,
                "QuinLED dig2analog Ultra PARLIO bus:\n"
                "  Lanes: %u, %u-bit samples at %" PRIu32 " Hz (%" PRIu32 " ns)\n"
                "  Bit: %u samples (%" PRIu32 " ns), a group of 8 bits %" PRIu32 " ns\n"
                "  Buffers: 2 x %u B in %s, a chunk of %" PRIu32 " groups (%" PRIu32 " us), handed over %" PRIu32
                " us before the one playing ends\n"
                "  Render: %" PRIu32 " us per chunk on average, %" PRIu32 " us at most; %" PRIu32 " restart(s), %" PRIu32
                " start(s) tried again",
                this->lanes_used_, this->width_, this->rate_hz_, ns, this->period_, this->period_ * ns,
                this->stream_.clock().group_ns(), static_cast<unsigned>(this->stream_.chunk_bytes()),
                this->in_psram_ ? "PSRAM" : "internal RAM", this->chunk_groups_,
                static_cast<uint32_t>(this->stream_.chunk_ns() / 1000), this->margin_ns_ / 1000,
                this->renders_ ? this->render_us_sum_ / this->renders_ : 0, this->render_us_max_, this->restarts_,
                this->start_retries_);
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
