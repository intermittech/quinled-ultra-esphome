#include "dig2analog_ultra.h"
#include "ultra_light.h"
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
#include "fw_manifest.h"
#endif

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <esp_attr.h>
#include <esp_clk_tree.h>
#include <esp_random.h>
#include <esp_timer.h>
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
#include <esp_memory_utils.h>
#include <hal/gpio_ll.h>
#endif

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra";

static constexpr uint32_t RMT_DURATION_MAX = 0x7FFF;  // 15-bit duration field
// Keep the main loop in high-frequency mode this long after the last command, so the copies and
// commands that follow (effect steps, a scene of several lights) go out back to back.
static constexpr uint32_t BUSY_HOLD_MS = 250;

static uint32_t rmt_resolution_hz() {
  uint32_t freq = 0;
  esp_clk_tree_src_get_freq_hz((soc_module_clk_t) RMT_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &freq);
  return freq;
}

static uint32_t ns_to_ticks(uint32_t ns, uint32_t res_hz) {
  return static_cast<uint32_t>((static_cast<uint64_t>(ns) * res_hz + 500000000ULL) / 1000000000ULL);
}

static rmt_symbol_word_t make_symbol(uint32_t ticks0, uint32_t level0, uint32_t ticks1, uint32_t level1) {
  rmt_symbol_word_t s{};
  s.duration0 = ticks0;
  s.level0 = level0;
  s.duration1 = ticks1;
  s.level1 = level1;
  return s;
}

static size_t IRAM_ATTR HOT encoder_callback(const void *data, size_t size, size_t symbols_written,
                                             size_t symbols_free, rmt_symbol_word_t *symbols, bool *done,
                                             void *arg) {
  auto *e = static_cast<EncoderArg *>(arg);
  if (e->watching)
    e->watch.fill(esp_timer_get_time(), symbols_written == 0);
  return encode_symbols(e->symbols, static_cast<const uint8_t *>(data), size, symbols_written, symbols_free, symbols,
                        done);
}

int64_t UltraHub::now_us_() { return esp_timer_get_time(); }

void UltraHub::set_board_config(int board, uint8_t id, uint16_t value) {
  if (!this->started_) {
    this->early_config_.push_back({board, id, value});
    return;
  }
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    if (board < 0 || board == b)
      this->sched_.set_config(b, id, value);
  }
  this->sched_.flush_config();
  this->wake_();
}

void UltraHub::set_pwm_frequency(int board, float hz) {
  uint16_t preset, period;
  pwm_for_frequency(hz, preset, period);
  if (!this->started_) {
    this->early_config_.push_back({board, ULTRA_CFG_PWM_PRESET, preset});
    this->early_config_.push_back({board, ULTRA_CFG_PWM_PERIOD, period});
    return;
  }
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    if (board < 0 || board == b) {
      this->sched_.set_config(b, ULTRA_CFG_PWM_PRESET, preset);
      this->sched_.set_config(b, ULTRA_CFG_PWM_PERIOD, period);
    }
  }
  this->sched_.flush_config();
  this->wake_();
}

// A board forwards Ultra frames only once it is locked: a preamble of DARK frames (preamble_frames()) lets the
// whole chain lock (boards that were locked to a WS protocol show dark meanwhile) before the commands come.
void UltraHub::resync(bool power_on) {
  this->tx_prepared_ = false;  // a frame the RMT refused: dropped, the replay sends everything again
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->tx_rw_ = false;
  this->rw_drop_turn_();       // (a poll frame among them: its UART's turn goes to the next line)
#endif
  this->preamble_full_ = !(power_on && this->synced_once_);
  this->preamble_ = this->preamble_full_ ? preamble_frames(this->num_boards_) : power_on_preamble_frames(this->num_boards_);
  int64_t lead_us = static_cast<int64_t>(this->preamble_) * (this->frame_us_ + 200);
  if (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES)
    lead_us += static_cast<int64_t>(ULTRA_SWITCH_WS_FRAMES - this->switch_frame_) * (this->switch_frame_us_ + 200);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  this->arm_ping_burst_();
  lead_us += static_cast<int64_t>(this->ping_left_) * (this->update_frame_us() + 200) +
             static_cast<int64_t>(this->dark_tail_) * (this->frame_us_ + 200);
#endif
  this->sched_.replay(this->now_us_() + lead_us + this->stream_lead_us_());
  this->wake_();
}

UltraHub::Diagnostics UltraHub::take_diagnostics() {
  uint32_t underruns = this->enc_.watch.count;
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  if (this->lane_ != nullptr)
    underruns = this->lane_->refused() + this->lane_->lost();   // frames a restart of the stream lost
#endif
  const Diagnostics d{this->sched_.take_queue_peak(), this->sched_.take_memory_peak(), this->build_peak_us_,
                      underruns, this->frames_};
  this->build_peak_us_ = 0;
  return d;
}

std::vector<UltraHub *> UltraHub::hubs_;
GroupStart UltraHub::group_;

int64_t UltraHub::lead_us(size_t commands, size_t waiting) const {
  uint32_t frame = this->frame_us_ + 500;   // a frame and the gap to the next
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  // a lane's mailbox takes a frame at most every few chunks (a frame waits for the one before it to be taken):
  // short frames on long chunks come that far apart
  frame = std::max(frame, this->lane_lead_us_);
#endif
  if (this->start_delay_us_ >= 0)
    return this->start_delay_us_ + static_cast<int64_t>(commands > 1 ? commands - 1 : 0) * frame;
  int64_t lead = start_lead_us(frame, commands, waiting);
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  lead += this->rw_extra_us_;   // a command queued as a response window opens waits for its end
#endif
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  lead += this->lane_lead_us_;  // a lane's frame starts a few ms ahead of now
#endif
  return lead;
}

// A group change: every command it can give the busiest board (a SET per light, an effect's EFFECT and STEPs, a
// power-on scene's SCENE: max_commands_), after what waits already.
int64_t UltraHub::command_lead_us_() const {
  if (this->start_delay_us_ >= 0)
    return this->start_delay_us_;
  return this->lead_us(this->max_commands_, this->sched_.waiting());
}

int64_t UltraHub::start_time_us(uint16_t board) {
  int64_t lead = 0;
  for (auto *hub : hubs_)
    lead = std::max(lead, hub->command_lead_us_());
  // a command joins the open group while its first copy gets out by its start: the frame on the wire, what waits
  // on its board, its own frame
  uint32_t frame = this->frame_us_ + 500;
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  frame = std::max(frame, this->lane_lead_us_);
#endif
  int64_t reach = start_lead_us(frame, 1, this->sched_.waiting(board)) - START_SLACK_US;
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  reach += this->lane_lead_us_;
#endif
  return group_.start(this->now_us_(), lead, reach);
}

// The boards have power as far as the hub knows: no supply configured, or it is on and its delay is over.
bool UltraHub::boards_powered_() const {
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr)
    return this->port_->is_ready();
#endif
#ifdef USE_POWER_SUPPLY
  if (this->psu_ != nullptr)
    return this->psu_->is_enabled() && !this->power_waiting_;
#endif
  return true;
}

#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
// After the DARK preamble (the boards have seen normal Ultra frames): PING frames, then a few DARK ones (fw_manifest.h).
void UltraHub::arm_ping_burst_() {
  if (!this->ping_burst_)
    return;
  this->ping_left_ = static_cast<uint16_t>(fw_ping_burst_frames(this->num_boards_));
  this->dark_tail_ = static_cast<uint8_t>(FW_PING_DARK_TAIL);
}

uint32_t UltraHub::update_frame_us() const {
  const uint32_t worst = std::max(this->bit0_high_ns_ + this->bit0_low_ns_, this->bit1_high_ns_ + this->bit1_low_ns_);
  return (UPD_FRAME_BITS * worst) / 1000 + this->reset_ns_ / 1000;
}

uint32_t UltraHub::update_frame_min_us() const {
  const uint32_t best = std::min(this->bit0_high_ns_ + this->bit0_low_ns_, this->bit1_high_ns_ + this->bit1_low_ns_);
  return (UPD_FRAME_BITS * best) / 1000 + this->reset_ns_ / 1000;
}

bool UltraHub::start_update(const uint8_t *image, size_t size, const UpdateSession::Options &opt) {
  if (!this->started_ || this->is_failed() || this->update_running())
    return false;
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->finish_window_(true);   // a response window ends first (<= 26 ms; the session's frames have the full reset)
#endif
  UpdateSession::Options o = opt;
  o.boards = this->num_boards_;
  // the lead-in and END are counted in the shortest frame (the task's own pacing only adds to it), so they last at
  // least as long as asked (the boards erase during the lead-in)
  if (!this->upd_session_.begin(image, static_cast<uint32_t>(size), static_cast<uint8_t>(esp_random() | 1u), o,
                                this->update_frame_min_us()))
    return false;
  this->tx_prepared_ = false;  // the session takes the buffer; post-session replay repairs normal state
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->tx_rw_ = false;        // a poll frame that never went out: its UART's turn goes to the next line
  this->rw_drop_turn_();
#endif
  this->upd_total_ = this->upd_session_.total_frames();
  this->upd_sent_.store(0);
  this->upd_pass_.store(0);
  this->upd_abort_.store(false);
  this->upd_result_.store(UPD_NONE);
  this->upd_state_.store(UPD_RUNNING);
  if (xTaskCreate(UltraHub::update_task_, "d2a_update", 4096, this, 5, &this->upd_task_) != pdPASS) {
    this->upd_state_.store(UPD_IDLE);
    ESP_LOGE(TAG, "Cannot start the update task");
    return false;
  }
  ESP_LOGI(TAG, "Firmware update session: %" PRIu32 " frames to %u boards", this->upd_total_, this->num_boards_);
  return true;
}

float UltraHub::update_progress() const {
  return this->upd_total_ ? static_cast<float>(this->upd_sent_.load()) / static_cast<float>(this->upd_total_) : 0.0f;
}

UltraHub::UpdateResult UltraHub::take_update_result() {
  const UpdateResult r = this->upd_done_;
  this->upd_done_ = UPD_NONE;
  return r;
}

bool UltraHub::has_power_supply() const {
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr)
    return true;
#endif
#ifdef USE_POWER_SUPPLY
  return this->psu_ != nullptr;
#else
  return false;
#endif
}

void UltraHub::power_cycle() {
  if (!this->has_power_supply())
    return;
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr) {   // the port's enable low now (no notice: the session owns the line), then a wake
    this->port_->power_cycle();
    this->cycle_ = CYCLE_OFF;
    this->cycle_ms_ = millis();
    return;
  }
#endif
  this->cycle_ = CYCLE_WAIT_OFF;
  this->cycle_ms_ = millis();
}

// The session owns the RMT channel (the main loop sends nothing meanwhile). Frame by frame: wait until the last one
// (and its reset) is on the wire, the page pause if it asked for one, then the next.
void UltraHub::update_task_(void *arg) {
  auto *hub = static_cast<UltraHub *>(arg);
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  if (hub->lane_ != nullptr) {
    hub->upd_result_.store(hub->update_lane_());
    hub->upd_state_.store(UPD_FINISHED);
    vTaskDelete(nullptr);
    return;
  }
#endif
  rmt_transmit_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  uint8_t result = UPD_DONE;
  uint32_t pause_us = 0;
  const uint32_t tick_us = portTICK_PERIOD_MS * 1000;
  for (;;) {
    if (rmt_tx_wait_all_done(hub->channel_, 200) != ESP_OK) {
      result = UPD_FAILED;
      break;
    }
    if (pause_us != 0)
      vTaskDelay((pause_us + tick_us - 1) / tick_us + 1);   // >= pause_us (the boards program a page)
    if (hub->upd_abort_.load()) {
      result = UPD_ABORTED;
      break;
    }
    if (!hub->upd_session_.next(hub->tx_buf_, static_cast<uint32_t>(now_us_()), &pause_us))
      break;
    if (rmt_transmit(hub->channel_, hub->encoder_, hub->tx_buf_, UPD_FRAME_BYTES, &cfg) != ESP_OK) {
      result = UPD_FAILED;
      break;
    }
    hub->upd_sent_.store(hub->upd_session_.frames_sent());
    hub->upd_pass_.store(hub->upd_session_.pass());
  }
  if (result == UPD_ABORTED) {
    for (int i = 0; i < 12 && rmt_tx_wait_all_done(hub->channel_, 200) == ESP_OK; i++) {
      hub->upd_session_.abort_frame(hub->tx_buf_, static_cast<uint32_t>(now_us_()));
      if (rmt_transmit(hub->channel_, hub->encoder_, hub->tx_buf_, UPD_FRAME_BYTES, &cfg) != ESP_OK)
        break;
    }
  }
  rmt_tx_wait_all_done(hub->channel_, 200);
  hub->upd_result_.store(result);
  hub->upd_state_.store(UPD_FINISHED);
  vTaskDelete(nullptr);
}

#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
// The session on a PARLIO lane: each frame posted with its page pause as the lane's quiet after it (so the line stays
// low that long after its reset, whatever the stream's latency), the next built once the renderer took it. A frame
// whose start passed before it was posted, or that the stream refused (it restarted before the frame went out), goes
// again at a new start, its time stamped again. Every wait is bounded (~2 s without the stream taking a frame: failed;
// after that or an abort ~100 ms) and ends on an abort; the session is over once its last frame is out on the wire.
static void upd_stamp(uint8_t *frame, uint32_t time_us) {
  upd_put32(frame + UPD_BLOCK_BYTES + 3u, time_us);
  frame[UPD_BLOCK_BYTES + 7u] = upd_crc8(frame + UPD_BLOCK_BYTES, 7);
}

uint8_t UltraHub::update_lane_() {
  enum : uint8_t { SENT = 0, AGAIN, ABORT, STOPPED };
  const uint32_t reset_us = this->reset_ns_ / 1000;
  uint32_t limit = 2000 / portTICK_PERIOD_MS + 2;   // ticks without the stream taking a frame: it stopped
  // until the lane took the posted frame (SENT) or refused it (AGAIN)
  auto settle = [&](bool abortable) -> uint8_t {
    for (uint32_t t = 0; this->lane_->busy(); t++) {
      if (abortable && this->upd_abort_.load())
        return ABORT;
      if (t > limit)
        return STOPPED;
      vTaskDelay(1);
    }
    return this->lane_->take_refused() ? AGAIN : SENT;
  };
  // tx_buf_'s frame on the wire, at a fresh start; abortable: an abort ends the wait (not for the ABORT frames)
  auto send = [&](uint32_t quiet_us, bool abortable) -> uint8_t {
    for (uint32_t t = 0;; t++) {
      if (abortable && this->upd_abort_.load())
        return ABORT;
      if (t > limit)
        return STOPPED;
      if (!this->lane_->ready()) {   // (the stream restarting)
        vTaskDelay(1);
        continue;
      }
      upd_stamp(this->tx_buf_, static_cast<uint32_t>(this->lane_->reserve_us()));
      if (!this->lane_->post(this->tx_buf_, UPD_FRAME_BYTES, quiet_us, false)) {
        vTaskDelay(1);   // its start passed (or the stream restarted): a new one
        continue;
      }
      const uint8_t r = settle(abortable);
      if (r != AGAIN)
        return r;
    }
  };
  uint8_t result = UPD_DONE;
  uint32_t pause_us = 0;
  for (;;) {
    if (this->upd_abort_.load()) {
      result = UPD_ABORTED;
      break;
    }
    if (!this->upd_session_.next(this->tx_buf_, 0, &pause_us))
      break;
    const uint8_t s = send(reset_us + pause_us, true);   // the boards program a page in the pause
    if (s != SENT) {
      result = s == ABORT ? UPD_ABORTED : UPD_FAILED;
      break;
    }
    this->upd_sent_.store(this->upd_session_.frames_sent());
    this->upd_pass_.store(this->upd_session_.pass());
  }
  // after an abort or a failure, the ABORT frames and the last frame's end are best effort: a stream that takes no
  // frame for ~100 ms (it takes one within 3 buffers) holds them up no longer
  if (result != UPD_DONE)
    limit = 100 / portTICK_PERIOD_MS + 2;
  if (result == UPD_ABORTED) {
    for (int i = 0; i < 12; i++) {
      this->upd_session_.abort_frame(this->tx_buf_, 0);
      if (send(reset_us, false) != SENT)
        break;
    }
  }
  // the last frame out on the wire (and its quiet), before the line goes back
  for (uint32_t t = 0; now_us_() < this->lane_->end_us() && t <= limit; t++)
    vTaskDelay(1);
  return result;
}
#endif
#endif

#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
// An RW frame may go out now: telemetry configured, the boards powered, nothing urgent (commands, a preamble, a PING
// burst, a WS switch go first), no window open, and the poller has a page due (it keeps the lock and the spacing).
bool UltraHub::rw_due_(bool urgent) {
  if (this->rw_uart_ == nullptr)
    return false;
  // a line that held its UART for a poll frame that was dropped since: it lets go
  if (this->rw_turn_ && !this->rw_open_ && !(this->tx_prepared_ && this->tx_rw_))
    this->rw_drop_turn_();
  if (!this->started_)
    return false;
  // the receivers have no supply: unavailable until they answer again; on a Diff-Power port in standby they show
  // Standby, and once it is powered their status follows their answers again: every receiver's sensors both times
  if (!this->boards_powered_()) {
    this->rw_waiting_ = false;   // (not waiting for the UART)
    // a poll frame prepared but not sent (its transmit refused): dropped, and its UART turn (the boards have no
    // supply; once they have, the replay sends everything); a window already open keeps its turn until it is read
    if (this->tx_prepared_ && this->tx_rw_ && !this->rw_open_) {
      this->tx_prepared_ = false;
      this->tx_rw_ = false;
      this->rw_release_ = false;
      this->rw_drop_turn_();
    }
    if (this->rw_powered_) {
      this->rw_powered_ = false;
      this->poller_.unpowered();
      this->poller_.touch_all();
      this->rw_publish_();
    }
    return false;
  }
  if (!this->rw_powered_) {   // (every receiver Waking until it answers, or misses enough to be gone)
    this->rw_powered_ = true;
    this->rx_note_.resize(this->poller_.receivers(), 0);
    for (auto &n : this->rx_note_)
      n |= RX_SETTLING;
    this->poller_.touch_all();
    this->rw_publish_();
  }
  if (urgent || this->rw_open_) {
    this->rw_waiting_ = false;   // (not waiting for the UART)
    return false;
  }
  // the lock interval from the last lock frame's end (it was set when the frame was built)
  if (this->rw_lock_end_) {
    if (!this->line_out_())
      return false;
    this->poller_.locked(millis());
    this->rw_lock_end_ = false;
  }
  if (!this->poller_.due(millis())) {
    this->rw_waiting_ = false;
    return false;
  }
  return this->rw_uart_turn_();
}

// Just woken and not evaluated yet: not heard, not missed enough to be gone, not behind one that is gone (it can't be
// polled then: "can't be heard")
bool UltraHub::rx_settling(uint16_t receiver) const {
  if (receiver >= this->rx_note_.size() || !(this->rx_note_[receiver] & RX_SETTLING) ||
      this->poller_.answering(receiver) || this->poller_.misses(receiver) >= TelemetryPoller::UNAVAILABLE_MISSES)
    return false;
  const int cut = this->poller_.cut_at(millis());
  return !(cut >= 0 && cut < receiver);
}

// Lines sharing a UART (each with a receiver_enable_pin): one window at a time, and when one ends the line that has
// waited longest goes next, so a line with polls always due can't keep the UART. A line that stopped waiting (its
// commands go first now, or nothing is due) is no longer in the queue; one whose loop hasn't run for a while neither.
uint32_t UltraHub::rw_wait_seq_ = 0;
static constexpr uint32_t RW_WAIT_STALE_MS = 100;

bool UltraHub::rw_uart_turn_() {
  const uint32_t now = millis();
  if (!this->rw_waiting_) {
    this->rw_waiting_ = true;
    this->rw_wait_order_ = ++rw_wait_seq_;
  }
  this->rw_wait_seen_ms_ = now;
  for (const UltraHub *hub : hubs_) {
    if (hub == this || hub->rw_uart_ != this->rw_uart_)
      continue;
    if (hub->rw_turn_)
      return false;   // its window is open
    if (hub->rw_waiting_ && now - hub->rw_wait_seen_ms_ < RW_WAIT_STALE_MS &&
        static_cast<int32_t>(hub->rw_wait_order_ - this->rw_wait_order_) < 0)
      return false;   // it has waited longer
  }
  return true;
}

void UltraHub::rw_drop_turn_() {
  if (!this->rw_turn_ || this->rw_open_)
    return;
  this->rw_turn_ = false;
  if (this->rw_rx_enable_pin_ != nullptr)
    this->rw_rx_enable_pin_->digital_write(false);
}

// The RMT's interrupt after an RW frame's last symbol: let go of the line (ULTRA_RW_RELEASE_US after its last bit).
bool IRAM_ATTR UltraHub::rw_done_isr_(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *arg) {
  auto *hub = static_cast<UltraHub *>(arg);
  if (hub->rw_release_) {
    hub->rw_dir_isr_.digital_write(false);
    hub->rw_release_ = false;
    hub->rw_released_us_ = esp_timer_get_time();
  }
  return false;
}

#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
// A lane: the bus' timer alarm at the RW frame's last bit (PARLIO_RELEASE_NS after it), from the stream's timing. It
// runs with the cache off too: it writes the pin itself, from the hub (in internal RAM: setup() checks), not through
// the ISR pin (its argument is an allocation of its own, which may be in PSRAM).
void IRAM_ATTR UltraHub::rw_lane_release_(void *arg) {
  auto *hub = static_cast<UltraHub *>(arg);
  if (hub->rw_release_) {
    gpio_ll_set_level(&GPIO, hub->rw_dir_gpio_, hub->rw_dir_idle_);
    hub->rw_release_ = false;
    hub->rw_released_us_ = esp_timer_get_time();
  }
}
#endif

// The window is over (wait: until its bytes can be read; <= 26 ms): the UART's bytes to the poller, the sensors.
void UltraHub::finish_window_(bool wait) {
  if (!this->rw_open_)
    return;
  if (wait) {
    while (now_us_() < this->rw_read_us_)
      delayMicroseconds(100);
  }
  this->rw_open_ = false;
  this->rw_release_ = false;
  this->enc_.symbols.reset = this->reset_symbol_;
  if (this->rw_dir_pin_ != nullptr) {
    const int64_t released = this->rw_released_us_;
    if (released == 0 || released - this->rw_t0_us_ > ULTRA_RW_RELEASE_US) {
      if (this->rw_late_++ % 100 == 0)   // the receivers found the line still driven: no answers this window
        ESP_LOGW(TAG, "Telemetry on GPIO%u: the line let go of %s (%" PRIu32 " windows so far)", this->pin_,
                 released == 0 ? "late (no RMT interrupt)" : "more than 150 us after the frame", this->rw_late_);
    }
    this->rw_released_us_ = 0;
  }
  this->rw_rx_.clear();
  uint8_t buf[64];
  size_t n;
  while ((n = this->rw_uart_->available()) > 0 && this->rw_rx_.size() < 4096) {
    n = std::min(n, sizeof(buf));
    if (!this->rw_uart_->read_array(buf, n))
      break;
    this->rw_rx_.insert(this->rw_rx_.end(), buf, buf + n);
  }
  // the window's bytes are read: the port's receiver off, the UART free for the next line on it (DE and /RE tied:
  // the line driven again, which turns its receiver off)
  if (this->rw_rx_enable_pin_ != nullptr)
    this->rw_rx_enable_pin_->digital_write(false);
  if (this->rw_rx_tied_ && this->rw_dir_pin_ != nullptr && !this->rw_driving_) {
    this->rw_dir_pin_->digital_write(true);
    this->rw_driving_ = true;
  }
  this->rw_turn_ = false;
  this->poller_.finish(millis(), this->rw_rx_.data(), this->rw_rx_.size());
  this->rw_publish_();
}

// What changed per receiver: the port's load first (the receivers show their part), the log, then the sensors.
void UltraHub::rw_publish_() {
  this->port_monitor_.update(millis(), this->poller_);
  // the port's energy at the sender (standby: 0 W), kept in the ledger (never down; across reboots)
  this->port_energy_.add(this->port_power_w(), millis());
  LifetimeLedger &led = ultra_ledger();
  if (led.ready()) {
    if (!this->port_seeded_) {
      this->port_energy_.wh = std::max(this->port_energy_.wh, led.counter_mwh(this->keeper_.port_key()) / 1000.0);
      this->port_seeded_ = true;
    }
    led.set_counter(this->keeper_.port_key(), static_cast<uint64_t>(this->port_energy_.wh * 1000.0));
  }
  for (uint16_t i = 0; i < this->poller_.receivers(); i++) {
    uint32_t u = this->poller_.take_updates(i);
    uint8_t note = i < this->rx_note_.size() ? this->rx_note_[i] : 0;
    if ((note & RX_SETTLING) && !this->rx_settling(i)) {   // after a wake: heard, or missed enough: its status now
      note &= ~RX_SETTLING;
      u |= TU_AVAILABILITY;
    }
    if (u == 0)
      continue;
    const ReceiverData &d = this->poller_.data(i);
    if ((u & TU_AVAILABILITY) && (this->port_standby() || (note & RX_SETTLING))) {
      // (its line's power switched on purpose: off, or just back and not evaluated yet)
      ESP_LOGD(TAG, "Receiver %u on GPIO%u: %s", i, this->pin_, this->port_standby() ? "standby" : "waking");
    } else if (u & TU_AVAILABILITY) {
      if (this->poller_.answering(i)) {
        ESP_LOGI(TAG, "Receiver %u on GPIO%u answers", i, this->pin_);
      } else {
        const int cut = this->poller_.cut_at(millis());
        if (cut >= 0 && cut < i)
          ESP_LOGW(TAG, "Receiver %u on GPIO%u can't be heard: receiver %d in front of it doesn't answer", i,
                   this->pin_, cut);
        else
          ESP_LOGW(TAG, "Receiver %u on GPIO%u doesn't answer", i, this->pin_);
      }
    }
    if (u & TU_RESTART) {
      if ((note & RX_WOKE) && millis() - this->woke_ms_ < 60000) {   // (its port's wake: expected)
        ESP_LOGD(TAG, "Receiver %u on GPIO%u restarted (its port woke)", i, this->pin_);
      } else {
        ESP_LOGI(TAG, "Receiver %u on GPIO%u restarted or was replaced", i, this->pin_);
      }
      note &= ~RX_WOKE;
    }
#ifdef USE_DIFFPOWER
    if (this->port_ != nullptr && d.have_identity && !(d.features & ULTRA_FEAT_STANDBY) && !(note & RX_WARNED)) {
      note |= RX_WARNED;
      ESP_LOGW(TAG, "Receiver %u on GPIO%u has no standby support (firmware before 0.6.10): it shows its power-on "
               "scene at each wake of its port; update it", i, this->pin_);
    }
#endif
    if (i < this->rx_note_.size())
      this->rx_note_[i] = note;
    if ((u & (1u << TP_POWER)) && d.pw_latched != 0)
      ESP_LOGW(TAG, "Receiver %u on GPIO%u power events: %s", i, this->pin_, telemetry_power_events_text(d).c_str());
    // the lifetime ledger: its counters merged, a sync when it is ahead, the position's Energy
    const uint8_t lev = this->keeper_.on_updates(this->poller_, i, u, millis());
    if (lev & (LedgerKeeper::EV_NEW_BOARD | LedgerKeeper::EV_RESTORE)) {
      char uid[25];
      for (int k = 0; k < 12; k++)
        snprintf(uid + 2 * k, 3, "%02X", d.uid[k]);
      if (lev & LedgerKeeper::EV_NEW_BOARD) {
        ESP_LOGI(TAG, "New board at %s \xC2\xB7 Receiver %u (UID %s): its Energy goes on from the position's", this->tel_name_,
                 i + 1, uid);
      }
      if (lev & LedgerKeeper::EV_RESTORE) {
        ESP_LOGI(TAG, "%s \xC2\xB7 Receiver %u (UID %s): its lifetime record went back (erased?): restored from the hub "
                 "over its next power-ups", this->tel_name_, i + 1, uid);
      }
    }
    if (lev & LedgerKeeper::EV_ATTENTION) {
      ESP_LOGW(TAG, "%s \xC2\xB7 Receiver %u: its lifetime sync was refused twice: needs attention (the hub shows its "
               "own counts)", this->tel_name_, i + 1);
    }
    if (lev & LedgerKeeper::EV_SYNCED) {
      ESP_LOGD(TAG, "%s \xC2\xB7 Receiver %u: its lifetime counters synced from the hub", this->tel_name_, i + 1);
    }
    this->rw_callback_.call(i, u);
  }
  this->rw_window_callback_.call();
}
#endif

// WS config frames first (boards forced to WS take only those), then the preamble and the replay (resync), with the
// boot protocol queued as CONFIG: the boards run Ultra by then. Boards already in Ultra mode ignore the WS frames
// (their lengths never make a board count 3 frames of one WS protocol, see ultra_switch_frame).
void UltraHub::switch_boards_to_ultra(uint8_t boot_protocol) {
  if (!this->started_ || this->is_failed())
    return;
  const uint8_t p = boot_protocol == 3 ? 3 : 0;
  ESP_LOGI(TAG, "Switching boards to Ultra: WS config frames, then the start-up preamble (boot protocol %s)",
           p == 3 ? "ultra" : "auto");
  this->switch_frame_ = 0;
  this->set_board_config(-1, ULTRA_CFG_BOOT_PROTOCOL, p);
  this->resync();
}

// Something to send: frames back to back until the queues are empty.
void UltraHub::wake_() {
  this->last_busy_ms_ = millis();
  if (!this->high_freq_active_) {
    this->high_freq_.start();
    this->high_freq_active_ = true;
  }
  this->enable_loop();
}

// Request the supply while any light is on (or still fading out). After it really switched on, the boards just
// booted: wait power_on_delay, then send everything again with transitions and effects restarted.
static constexpr uint32_t CAL_BOOT_DELAY_MS = 10000;
static constexpr uint32_t CAL_POWER_WAIT_MS = 30000;

void UltraHub::calibrate_boards(const std::vector<uint16_t> &boards, uint8_t duty_pct) {
  if (this->calibration_running()) {
    ESP_LOGW(TAG, "Current calibration on GPIO%u: already running, request ignored", this->pin_);
    return;
  }
  this->cal_req_boards_ = boards;
  this->cal_req_duty_ = duty_pct;
  this->cal_pending_ = true;  // starts in service_calibration_() once the boards have power and Ultra
  this->cal_started_ = false;
  this->cal_wait_ms_ = millis();
}

void UltraHub::cancel_calibration() {
  if (this->cal_pending_) {
    this->cal_pending_ = false;
    ESP_LOGI(TAG, "Current calibration on GPIO%u: cancelled before it started", this->pin_);
  }
  if (this->cal_seq_.running()) {
    this->cal_seq_.cancel(this->sched_);
    this->wake_();
    ESP_LOGI(TAG, "Current calibration on GPIO%u: cancelled after %u of %u board(s), the pin fades back", this->pin_,
             static_cast<unsigned>(this->cal_started_ ? this->cal_seq_.index() : 0),
             static_cast<unsigned>(this->cal_seq_.count()));
  }
}

bool UltraHub::any_lit_(uint32_t now) const {
  for (auto *l : this->lights_) {
    if (l->is_lit(now))
      return true;
  }
  return false;
}

void UltraHub::service_calibration_() {
  const uint32_t now = millis();
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  if (this->update_running()) {
    if (this->calibration_running())
      this->cancel_calibration();  // the session owns the line; the release goes out after it
    this->cal_power_hold_ = false;
    return;
  }
#endif
  // automatic runs (off unless configured)
  const bool boot_due = this->cal_on_boot_ && !this->cal_boot_done_ && now >= CAL_BOOT_DELAY_MS;
  const bool interval_due = this->cal_interval_ms_ != 0 && now - this->cal_last_ms_ >= this->cal_interval_ms_;
  if ((boot_due || interval_due) && !this->calibration_running() && !(this->cal_only_off_ && this->any_lit_(now))) {
    this->cal_last_ms_ = now;
    this->cal_boot_done_ = true;
    this->calibrate_boards(this->cal_boards_, 0);
  }
  // a request waits until the boards have power and took the preamble
  if (this->cal_pending_) {
    bool ready = this->started_ && this->preamble_ == 0 && !this->port_standby();
#ifdef USE_POWER_SUPPLY
    if (this->psu_ != nullptr)
      ready &= this->psu_->is_enabled() && !this->power_waiting_;
#endif
    if (ready) {
      this->cal_pending_ = false;
      if (this->cal_seq_.start(this->sched_, this->num_boards_, this->cal_req_boards_, this->cal_req_duty_, now)) {
        const size_t n = this->cal_seq_.count();
        ESP_LOGI(TAG,
                 "Current calibration on GPIO%u: %u of %u board(s), %s, ~%u s; the pin goes dark, other pins run on",
                 this->pin_, static_cast<unsigned>(n), static_cast<unsigned>(this->num_boards_),
                 this->cal_req_duty_ == 0 ? "25/50/75/100 %" : "one level",
                 static_cast<unsigned>((CalSequencer::FADE_MS * 2 + n * (CalSequencer::board_ms(this->cal_req_duty_) + 60) +
                                        999) / 1000));
        this->wake_();
      }
    } else if (now - this->cal_wait_ms_ > CAL_POWER_WAIT_MS) {
      this->cal_pending_ = false;
      ESP_LOGW(TAG, "Current calibration on GPIO%u skipped: the boards did not get power / Ultra in 30 s", this->pin_);
    }
  }
  if (this->cal_seq_.running()) {
    switch (this->cal_seq_.poll(this->sched_, now)) {
      case CalSequencer::EV_BOARD:
        this->cal_started_ = true;
        ESP_LOGI(TAG, "Current calibration on GPIO%u: test sent to board %u (%u of %u)", this->pin_,
                 static_cast<unsigned>(this->cal_seq_.board()), static_cast<unsigned>(this->cal_seq_.index() + 1),
                 static_cast<unsigned>(this->cal_seq_.count()));
        this->wake_();
        break;
      case CalSequencer::EV_DONE:
        // one-way: the hub knows the commands went out, not what the boards measured
        ESP_LOGI(TAG,
                 "Current calibration on GPIO%u: commands sent to %u board(s) in %.1f s (not confirmed: the boards "
                 "keep their results); the pin fades back",
                 this->pin_, static_cast<unsigned>(this->cal_seq_.count()), this->cal_seq_.elapsed_ms(now) / 1000.0f);
        this->wake_();
        break;
      default:
        break;
    }
  }
  this->cal_power_hold_ = this->calibration_running();
}

void UltraHub::service_power_() {
#ifdef USE_POWER_SUPPLY
  if (this->psu_ == nullptr)
    return;
  const uint32_t now = millis();
  bool lit = this->cal_power_hold_;
  for (auto *l : this->lights_)
    lit |= l->is_lit(now);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  lit |= this->upd_power_hold_;
  // power_cycle(): released until the supply is really off, then off for 1.5 s, then the normal rule again.
  // PowerSupply::is_enabled() counts the requests, not the pin: after the last request the pin stays high for the
  // supply's keep_on_time, so the boards lose power only then. Other requesters may keep it on: give up after that
  // time + 5 s.
  if (this->cycle_ == CYCLE_WAIT_OFF) {
    const bool off = !this->psu_->is_enabled() &&
                     (!this->psu_released_ || now - this->psu_released_ms_ >= this->psu_keep_on_ms_);
    if (off) {
      this->cycle_ = CYCLE_OFF;
      this->cycle_ms_ = now;
    } else if (now - this->cycle_ms_ > this->psu_keep_on_ms_ + 5000) {
      this->cycle_ = CYCLE_FAILED;
      ESP_LOGW(TAG, "Power cycle: the supply stayed on (another component keeps it on?)");
    }
  } else if (this->cycle_ == CYCLE_OFF && now - this->cycle_ms_ >= 1500) {
    this->cycle_ = CYCLE_DONE;
  }
  if (this->cycle_ == CYCLE_WAIT_OFF || this->cycle_ == CYCLE_OFF)
    lit = false;
#endif
  if (lit && !this->psu_requested_) {
    const bool was_on =
        this->psu_->is_enabled() || (this->psu_released_ && millis() - this->psu_released_ms_ < this->psu_keep_on_ms_);
    this->psu_req_.request();
    this->psu_requested_ = true;
    if (!was_on) {
      this->power_waiting_ = true;
      this->power_on_ms_ = millis();
      ESP_LOGD(TAG, "Power supply on: replaying the boards' state in %" PRIu32 " ms", this->power_on_delay_ms_);
    }
  } else if (!lit && this->psu_requested_) {
    this->psu_req_.unrequest();
    this->psu_requested_ = false;
    this->psu_released_ = true;
    this->psu_released_ms_ = millis();
  }
#endif
}

void UltraHub::setup() {
  this->sched_.init(this->num_boards_, esp_random(), this->repeats_);
  this->started_ = true;
  for (const auto &c : this->early_config_) {
    for (uint16_t b = 0; b < this->num_boards_; b++) {
      if (c.board < 0 || c.board == b)
        this->sched_.set_config(b, c.id, c.value);
    }
  }
  this->early_config_.clear();
  this->early_config_.shrink_to_fit();
  this->sched_.flush_config();
  // light indices without a light in the YAML: cleared on the boards (an earlier configuration may have used them)
  for (uint16_t b = 0; b < this->num_boards_; b++)
    this->sched_.clear_unused_lights(b, b < this->light_counts_.size() ? this->light_counts_[b] : 0);
  for (uint8_t n : this->light_counts_)
    this->max_lights_ = std::max(this->max_lights_, n);
  this->light_counts_.clear();
  this->light_counts_.shrink_to_fit();
  hubs_.push_back(this);

  RAMAllocator<uint8_t> alloc(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  size_t buf_bytes = std::max<size_t>(this->sched_.frame_bytes(), ultra_switch_frame_bytes(this->num_boards_));
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  buf_bytes = std::max<size_t>(buf_bytes, UPD_FRAME_BYTES);   // update sessions and PING frames
#endif
  this->tx_buf_ = alloc.allocate(buf_bytes);
  if (this->tx_buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate the frame buffer (%u B)", static_cast<unsigned>(buf_bytes));
    this->mark_failed();
    return;
  }

#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  if (this->lane_ != nullptr) {
    // a PARLIO lane: the bus renders the frames; the timing (set_timing) is already on its sample grid
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
    const ParlioLane::ReleaseFn release = UltraHub::rw_lane_release_;   // the direction pin (telemetry)
    // the bus' timer lets go of the direction pin also while flash writes have the cache off: what it touches here
    // must be in internal RAM (malloc puts objects this small there; PSRAM only when internal RAM is exhausted)
    if (this->rw_dir_pin_ != nullptr && !esp_ptr_internal(this)) {
      ESP_LOGE(TAG, "PARLIO lane %u: the line's state is in PSRAM (internal RAM full)", this->lane_->index());
      this->mark_failed();
      return;
    }
#else
    const ParlioLane::ReleaseFn release = nullptr;
#endif
    if (!this->lane_->begin(buf_bytes, release, this)) {
      ESP_LOGE(TAG, "PARLIO lane %u: no bus or no memory", this->lane_->index());
      this->mark_failed();
      return;
    }
    this->lane_lead_us_ = this->lane_->lead_us();
    this->sched_.set_bit_timing(this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_, this->bit1_low_ns_);
  } else if (!this->setup_rmt_()) {
    return;
  }
#else
  if (!this->setup_rmt_())
    return;
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->rw_uart_ != nullptr) {
    this->keeper_.attach(&ultra_ledger(), this->tel_key_, this->poller_.receivers());
    ledger_receivers() += this->poller_.receivers() + 1u;   // (its receivers' positions, its port's energy)
    this->rw_slots_.assign(static_cast<size_t>(ULTRA_SLOT_BYTES) * this->num_boards_, 0);
    this->rw_extra_us_ = this->poller_.max_window_us() + ULTRA_RW_HOLDOFF_US;
    if (this->rw_dir_pin_ != nullptr) {
      this->rw_dir_pin_->setup();         // the hub drives the line
      this->rw_dir_pin_->digital_write(true);
      this->rw_dir_isr_ = this->rw_dir_pin_->to_isr();
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
      this->rw_dir_gpio_ = this->rw_dir_pin_->get_pin();   // (a lane's release writes it itself)
      this->rw_dir_idle_ = this->rw_dir_pin_->is_inverted() ? 1 : 0;
#endif
    }
    if (this->rw_rx_enable_pin_ != nullptr) {
      this->rw_rx_enable_pin_->setup();   // off until this line's first window (its UART may be shared)
      this->rw_rx_enable_pin_->digital_write(false);
    }
  }
#endif
  const uint32_t worst = std::max(this->bit0_high_ns_ + this->bit0_low_ns_, this->bit1_high_ns_ + this->bit1_low_ns_);
  this->frame_us_ = static_cast<uint32_t>((static_cast<uint64_t>(this->sched_.frame_bytes()) * 8 * worst) / 1000) +
                    this->reset_ns_ / 1000;
  this->switch_frame_us_ =
      static_cast<uint32_t>((static_cast<uint64_t>(ultra_switch_frame_bytes(this->num_boards_)) * 8 * worst) / 1000) +
      this->reset_ns_ / 1000;
  this->preamble_ = preamble_frames(this->num_boards_);
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  this->arm_ping_burst_();
#endif
  // Line is idle low now; guarantee one full reset period before the first frame.
  delayMicroseconds(this->reset_ns_ / 1000 + 1);
  this->wake_();
}

// The line's RMT channel and encoder (and with a telemetry direction pin, the RMT's done interrupt that lets go of
// the line after an RW frame). false: failed (logged, marked).
bool UltraHub::setup_rmt_() {
  const uint32_t res = rmt_resolution_hz();
  const uint32_t t0h = ns_to_ticks(this->bit0_high_ns_, res), t0l = ns_to_ticks(this->bit0_low_ns_, res);
  const uint32_t t1h = ns_to_ticks(this->bit1_high_ns_, res), t1l = ns_to_ticks(this->bit1_low_ns_, res);
  const uint32_t rst = ns_to_ticks(this->reset_ns_, res);
  const uint32_t rst0 = rst / 2, rst1 = rst - rst0;
  if (res == 0 || t0h == 0 || t0l == 0 || t1h == 0 || t1l == 0 || t0h > RMT_DURATION_MAX ||
      t0l > RMT_DURATION_MAX || t1h > RMT_DURATION_MAX || t1l > RMT_DURATION_MAX || rst1 > RMT_DURATION_MAX) {
    ESP_LOGE(TAG, "Timing does not fit an RMT symbol at %" PRIu32 " Hz", res);
    this->mark_failed();
    return false;
  }
  this->sched_.set_bit_timing(this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_, this->bit1_low_ns_);
  this->enc_.symbols.bit0 = make_symbol(t0h, 1, t0l, 0);
  this->enc_.symbols.bit1 = make_symbol(t1h, 1, t1l, 0);
  this->enc_.symbols.reset = make_symbol(rst0, 0, rst1, 0);
  // the RMT memory on the wire at the shorter bit: refills further apart ran it empty (an underrun)
  const uint32_t shorter = std::min(this->bit0_high_ns_ + this->bit0_low_ns_, this->bit1_high_ns_ + this->bit1_low_ns_);
  this->enc_.watch.drain_us = static_cast<int64_t>(this->rmt_symbols_) * shorter / 1000;

  rmt_tx_channel_config_t ch;
  memset(&ch, 0, sizeof(ch));
  ch.clk_src = RMT_CLK_SRC_DEFAULT;
  ch.resolution_hz = res;
  ch.gpio_num = static_cast<gpio_num_t>(this->pin_);
  ch.mem_block_symbols = this->rmt_symbols_;
  ch.trans_queue_depth = 1;
  ch.flags.invert_out = this->inverted_;
  ch.flags.with_dma = this->use_dma_;
  ch.intr_priority = 0;
  if (rmt_new_tx_channel(&ch, &this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX channel creation failed");
    this->mark_failed();
    return false;
  }
  rmt_simple_encoder_config_t enc;
  memset(&enc, 0, sizeof(enc));
  enc.callback = encoder_callback;
  enc.arg = &this->enc_;
  enc.min_chunk_size = 1;
  if (rmt_new_simple_encoder(&enc, &this->encoder_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT encoder creation failed");
    this->mark_failed();
    return false;
  }
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->reset_symbol_ = this->enc_.symbols.reset;
  this->rw_reset_symbol_ = this->reset_symbol_;
  if (this->rw_uart_ != nullptr && this->rw_dir_pin_ != nullptr) {
    // an RW frame ends with a 2 us low, then the RMT's interrupt lets go of the line: the window is its reset
    const uint32_t us1 = ns_to_ticks(1000, res);
    this->rw_reset_symbol_ = make_symbol(us1, 0, us1, 0);
    rmt_tx_event_callbacks_t cbs;
    memset(&cbs, 0, sizeof(cbs));
    cbs.on_trans_done = UltraHub::rw_done_isr_;
    if (rmt_tx_register_event_callbacks(this->channel_, &cbs, this) != ESP_OK) {
      ESP_LOGE(TAG, "RMT event callback failed (telemetry direction pin)");
      this->mark_failed();
      return false;
    }
  }
#endif
  if (rmt_enable(this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT enable failed");
    this->mark_failed();
    return false;
  }
  return true;
}

#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
// The ESP32's one lifetime ledger (every line's receivers: a board moved between lines keeps its entry). Two stored
// copies alternate: the newer valid one loads (a cut while one is written keeps the other). Each has its size stored
// beside it (written after it): a copy stored with more or fewer receivers loads too. Stored at most every 15 min
// while it changes by a wire unit, at a planned power-off (the notice) and at a reboot.
namespace {
const char *const LEDGER_KEYS[2] = {"ultra_ledger_a", "ultra_ledger_b"};
const char *const LEDGER_SIZE_KEYS[2] = {"ultra_ledger_a_size", "ultra_ledger_b_size"};
ESPPreferenceObject g_ledger_pref[2], g_ledger_size[2];
std::vector<uint8_t> g_ledger_blob;
uint32_t g_ledger_generation = 0, g_ledger_saved_ms = 0, g_ledger_day_ms = 0;
bool g_ledger_loaded = false;
}  // namespace

void UltraHub::ledger_service_() {
  LifetimeLedger &l = ultra_ledger();
  const uint32_t now = millis();
  if (!g_ledger_loaded) {   // the first loop: every line has its receivers counted
    g_ledger_loaded = true;
    // the newer valid copy (by its stored size: whatever the receivers were then), loaded alone (what the newer one
    // dropped stays dropped)
    uint32_t newest = 0;
    size_t used_e = 0, used_p = 0;
    bool any = false;
    std::vector<uint8_t> best;
    for (int k = 0; k < 2; k++) {
      g_ledger_size[k] = global_preferences->make_preference(sizeof(uint32_t), fnv1_hash(LEDGER_SIZE_KEYS[k]), true);
      uint32_t size = 0;
      if (!g_ledger_size[k].load(reinterpret_cast<uint8_t *>(&size), sizeof(size)) || size < 18 || size > LEDGER_MAX_BLOB)
        continue;
      ESPPreferenceObject stored = global_preferences->make_preference(size, fnv1_hash(LEDGER_KEYS[k]), true);
      std::vector<uint8_t> b(size);
      uint32_t gen = 0;
      size_t ne = 0, np = 0;
      if (stored.load(b.data(), b.size()) && LifetimeLedger::peek(b.data(), b.size(), &gen, &ne, &np) &&
          (!any || static_cast<int32_t>(gen - newest) > 0)) {
        best.swap(b);
        newest = gen;
        used_e = ne;
        used_p = np;
        any = true;
      }
    }
    // (blocks of 32: a receiver more or less keeps the size; never fewer than the stored copy uses)
    const size_t n = ledger_receivers();
    const size_t pos = std::max((n + 31) / 32 * 32, (used_p + 31) / 32 * 32);
    const size_t room = std::max((n + std::max<size_t>(4, n / 2) + 31) / 32 * 32, (used_e + 31) / 32 * 32);
    l.init(room, pos);
    if (any)
      l.load(best.data(), best.size(), &newest);
    g_ledger_blob.assign(l.blob_size(), 0);
    for (int k = 0; k < 2; k++)
      g_ledger_pref[k] = global_preferences->make_preference(g_ledger_blob.size(), fnv1_hash(LEDGER_KEYS[k]), true);
    g_ledger_generation = newest;
    g_ledger_saved_ms = g_ledger_day_ms = now;
    ESP_LOGD(TAG, "Lifetime ledger: %u board(s) room, %s", static_cast<unsigned>(l.capacity()),
             any ? "loaded" : "new");
    return;
  }
  if (now - g_ledger_day_ms >= LEDGER_DAY_MS) {   // the ledger's days: the hub's time on
    g_ledger_day_ms += LEDGER_DAY_MS;
    l.set_day(l.day() + 1);
  }
  if (l.dirty() && now - g_ledger_saved_ms >= LEDGER_SAVE_MS)
    ledger_save_(false);
}

void UltraHub::ledger_save_(bool now) {
  LifetimeLedger &l = ultra_ledger();
  if (!g_ledger_loaded || (!l.dirty() && !now))
    return;
  if (l.dirty()) {
    // the next generation into the other copy, its size after it, on the flash now; clean only once it is there (a
    // failed write stays dirty: the next save, periodic or forced, writes it again)
    g_ledger_generation++;
    l.save(g_ledger_blob.data(), g_ledger_generation);
    const uint32_t size = static_cast<uint32_t>(g_ledger_blob.size());
    bool ok = g_ledger_pref[g_ledger_generation & 1].save(g_ledger_blob.data(), g_ledger_blob.size());
    ok = g_ledger_size[g_ledger_generation & 1].save(reinterpret_cast<const uint8_t *>(&size), sizeof(size)) && ok;
    ok = global_preferences->sync() && ok;
    if (ok) {
      l.clear_dirty();
    } else {
      ESP_LOGW(TAG, "Lifetime ledger: storing it failed; again at the next save");
    }
  } else {
    global_preferences->sync();   // (a planned power-off, a reboot: the rest on the flash now)
  }
  g_ledger_saved_ms = millis();
}

// The ledger counts this line's receivers itself (ultra_ledger.h; design review, round 9): a Diff-Power port's power-on
// and cut (its enable AND detect, as the port timed them: every board's session from there, its powered time to
// there), and each receiver's lit time (any of its lights lit while the boards are ready).
void UltraHub::ledger_line_() {
  if (this->rw_uart_ == nullptr)
    return;
  const uint32_t now = millis();
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr && this->port_->core().line_on() != this->line_on_) {
    this->line_on_ = !this->line_on_;
    if (this->line_on_)
      this->keeper_.power_on(this->port_->core().line_ms());
    else
      this->keeper_.cut(this->port_->core().line_ms());
  }
#endif
  this->lit_rx_.assign(this->poller_.receivers(), 0);
  if (this->boards_powered_()) {
    for (auto *l : this->lights_) {
      if (l->get_board() < this->lit_rx_.size() && l->is_lit(now))
        this->lit_rx_[l->get_board()] = 1;
    }
  }
  for (uint16_t i = 0; i < this->lit_rx_.size(); i++)
    this->keeper_.lit_tick(i, this->lit_rx_[i] != 0, now);
}
#endif

void UltraHub::on_shutdown() {
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->shutdown_callback_.call();   // (the energies one last time)
#ifdef USE_DIFFPOWER
  if (this->line_on_) {   // (the port powers the line down now: its boards' sessions end)
    this->line_on_ = false;
    this->keeper_.cut(millis());
  }
#endif
  ledger_save_(true);
#endif
}

#ifdef USE_DIFFPOWER
// The port powers this line while a light here is lit (or within its turn-off period) and while a calibration or an
// update holds it; once powered, everything again (the boards booted dark); before it drops, the notice.
void UltraHub::attach_port_() {
  this->port_attached_ = true;
  for (auto *l : this->lights_)
    this->port_->add_load(l->get_standby_after(), [l](uint32_t now) { return l->is_lit(now); }, l->get_state());
  this->port_->add_load(1, [this](uint32_t) {
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
    return this->cal_power_hold_ || this->upd_power_hold_;
#else
    return this->cal_power_hold_;
#endif
  });
  this->port_->add_on_ready_callback([this]() {
    ESP_LOGD(TAG, "GPIO%u: the port is powered, replaying the boards' state", this->pin_);
    this->sched_.cancel_power_off();   // (a notice copy not sent before the power-off: stale now)
    this->notice_id_ = 0;
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
    if (this->cycle_ == CYCLE_OFF)
      this->cycle_ = CYCLE_DONE;   // (a recovery's power cycle: powered again)
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
    this->rx_note_.resize(this->poller_.receivers(), 0);
    for (auto &n : this->rx_note_)
      n |= RX_WOKE;
    this->woke_ms_ = millis();
#endif
    this->resync(true);
  });
  this->port_->add_on_notice_callback([this](uint8_t copy) { this->send_power_off_notice(copy); });
  this->port_->add_on_pump_callback([this]() { this->pump_notice_(); });
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->rw_uart_ != nullptr)   // before a planned power-off: the receivers' counters read once more, a sync
    this->port_->add_on_prepare_callback(
        [this](bool first) { return this->keeper_.prepare(this->poller_, millis(), first); });
#endif
}

void UltraHub::send_power_off_notice(uint8_t copy) {
  for (uint16_t i = 0; i < this->num_boards_; i++)
    this->sched_.power_off(i, copy == 0);
  if (copy == 0) {
    this->notices_++;
    ESP_LOGD(TAG, "GPIO%u: the planned power-off notice to %u board(s)", this->pin_, this->num_boards_);
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
    ledger_save_(true);   // (the planned cut: the ledger as it is now)
#endif
  }
  this->wake_();
}

// The controller's shutdown (the loop doesn't run any more): a queued notice copy out now if the line is free, else
// at the next call (no waiting here: the port serves every line in turn). A response window ends first; an update
// session owns the line: nothing.
void UltraHub::pump_notice_() {
  this->service_notice_();
  if (this->is_failed() || this->sched_.power_off_left() == 0)
    return;
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  if (this->upd_state_.load() != UPD_IDLE)
    return;
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->rw_open_) {
    if (now_us_() < this->rw_end_us_)
      return;
    this->rw_open_ = false;
  }
  this->rw_next_ = false;
#endif
  if (this->line_free_())
    this->transmit_();
}

// A lane's frame goes out a few ms after it was posted: its notice copy counts as sent (the port times the next copy
// and the power-off from it) once it is out on the wire, whole. A copy the stream refused or cut (it restarted) never
// reached the boards: it goes again, the same SEQ, in the next frame.
void UltraHub::service_notice_() {
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  if (this->notice_id_ == 0)
    return;
  const uint8_t outcome =
      this->lane_->outcome(this->notice_id_, this->notice_epoch_, this->notice_end_us_, now_us_());
  if (outcome == ParlioLane::PENDING)
    return;
  this->notice_id_ = 0;
  if (outcome == ParlioLane::ON_WIRE) {
    this->port_->notice_sent();
  } else {
    for (uint16_t i = 0; i < this->num_boards_; i++)
      this->sched_.power_off(i, false);
    this->wake_();
  }
#endif
}
#endif

void UltraHub::loop() {
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr && !this->port_attached_)
    this->attach_port_();
  this->service_notice_();
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->ledger_service_();
  this->ledger_line_();
#endif
  this->service_calibration_();
#ifdef USE_POWER_SUPPLY
  this->service_power_();
  if (this->power_waiting_ && millis() - this->power_on_ms_ >= this->power_on_delay_ms_) {
    this->power_waiting_ = false;
    ESP_LOGD(TAG, "Power ready: replaying the boards' state");
    this->resync(true);
  }
#endif
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  // An update session owns the line; afterwards it stays low for a second (the boards commit and restart), then
  // the normal frames come back with a resync (preamble, PING burst, everything again). The result is ready once the
  // PING burst is out: the boards confirm their new firmware with it, so the update holds the supply until then.
  if (this->upd_state_.load() != UPD_IDLE) {
    if (this->upd_state_.load() == UPD_FINISHED) {
      this->upd_task_ = nullptr;
      this->upd_state_.store(UPD_IDLE);
      this->upd_quiet_ = true;
      this->upd_quiet_ms_ = millis();
      static const char *const RESULTS[] = {"", "done", "aborted", "failed (RMT)"};
      ESP_LOGI(TAG, "Firmware update session %s", RESULTS[this->upd_result_.load() & 3]);
    }
    return;
  }
  if (this->upd_quiet_) {
    if (millis() - this->upd_quiet_ms_ < 1000)
      return;
    this->upd_quiet_ = false;
    this->upd_resync_pending_ = true;
    this->upd_confirm_ms_ = millis();
    this->resync();
  }
  if (this->upd_resync_pending_ && millis() - this->upd_confirm_ms_ >= 60000) {
    this->upd_resync_pending_ = false;   // the transmitter can't send: failed, rather than holding the supply forever
    this->upd_done_ = UPD_FAILED;
  }
  const bool ping_pending = this->ping_left_ > 0 || this->dark_tail_ > 0;
#else
  const bool ping_pending = false;
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  // A response window: the receivers answer, nothing goes out until it ends; its bytes are read a little later
  // (the UART hands over its last bytes after a few byte times of silence).
  if (this->rw_open_) {
    const int64_t now = now_us_();
    if (now < this->rw_end_us_)
      return;
    if (now >= this->rw_read_us_)
      this->finish_window_(false);
  }
#endif
  const bool urgent = this->tx_prepared_ || this->preamble_ > 0 || ping_pending ||
                      this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES || this->sched_.pending();
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  const bool rw = this->rw_due_(urgent);   // commands go first
  const bool rw_busy = this->rw_open_;
#else
  const bool rw = false, rw_busy = false;
#endif
  if (urgent)
    this->last_busy_ms_ = millis();
  if (this->high_freq_active_ && !urgent && !rw_busy && millis() - this->last_busy_ms_ > BUSY_HOLD_MS) {
    this->high_freq_.stop();
    this->high_freq_active_ = false;
  }
  if (!urgent && !rw && this->has_sent_ && millis() - this->last_frame_ms_ < this->refresh_interval_ms_)
    return;
  // Non-blocking: if the previous frame (incl. its reset) is still on the wire, retry next loop. A lane: the renderer
  // hasn't taken the posted frame yet (it is a few ms ahead of the wire).
  if (!this->line_free_())
    return;
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  if (this->upd_resync_pending_ && !this->tx_prepared_ && this->preamble_ == 0 && !ping_pending) {
    this->upd_resync_pending_ = false;   // the PING burst and its dark tail are on the wire
    this->upd_done_ = static_cast<UpdateResult>(this->upd_result_.load());
  }
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  this->rw_next_ = rw;
#endif
  this->transmit_();
}

void UltraHub::transmit_() {
  // The header (the frame's trailer) carries the host clock at the frame's last bit: the start, taken right before
  // the frame goes out, plus its bits on the wire. A frame the RMT refused is sent again (with a new TIME), not built
  // anew: its commands were taken from the queues already. A lane's frame starts where the stream has room for it.
  const int64_t now = now_us_();
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  const int64_t start = this->lane_ != nullptr ? this->lane_->reserve_us() : now;
#else
  const int64_t start = now;
#endif
  [[maybe_unused]] int64_t time_us = 0;   // the frame's TIME: the host clock at its last bit (telemetry: t0)
  if (!this->tx_prepared_) {
    this->tx_normal_ = true;
    this->tx_preamble_end_ = false;
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
    this->tx_rw_ = false;
    uint16_t polls = 0;
    // the frames that (re)lock the chain: no RW frame before ULTRA_RW_LOCK_MS after the last of them
    if (this->rw_uart_ != nullptr && (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES || this->preamble_ > 0
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
                                      || this->ping_left_ > 0 || this->dark_tail_ > 0
#endif
                                      )) {
      this->poller_.locked(millis());
      this->rw_lock_end_ = true;
    } else if (this->rw_next_) {
      polls = this->poller_.build(millis(), this->rw_slots_.data(), &this->rw_window_us_);
    }
#endif
    size_t bytes = this->sched_.frame_bytes();
    if (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES) {
      // switch_boards_to_ultra: a WS config frame (no Ultra header), with the same bit timing
      this->tx_normal_ = false;
      bytes = ultra_switch_frame(this->tx_buf_, this->switch_frame_++, this->num_boards_);
    } else if (this->preamble_ > 0 && this->sched_.power_off_left() == 0) {   // (a notice copy first)
      this->preamble_--;
      this->sched_.build_dark_frame(start, this->tx_buf_);
      this->tx_preamble_end_ = this->preamble_ == 0 && this->preamble_full_;
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
    } else if (this->ping_left_ > 0) {
      this->tx_normal_ = false;
      this->ping_left_--;
      UpdateSession::ping_frame(this->tx_buf_, static_cast<uint32_t>(start));
      bytes = UPD_FRAME_BYTES;
    } else if (this->dark_tail_ > 0) {
      this->dark_tail_--;
      this->sched_.build_dark_frame(start, this->tx_buf_);
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
    } else if (polls > 0) {
      // an RW frame: POLL slots for the receivers, the boards behind them as usual
      time_us = this->sched_.build_rw_frame(start, this->tx_buf_, this->rw_slots_.data(), polls);
      this->tx_rw_ = true;
      // the UART is this line's until the window is read: its receiver on before the frame's last bit
      this->rw_turn_ = true;
      this->rw_waiting_ = false;
      if (this->rw_rx_enable_pin_ != nullptr)
        this->rw_rx_enable_pin_->digital_write(true);
#endif
    } else {
      this->sched_.build_frame(start, this->tx_buf_);
      this->build_peak_us_ = std::max(this->build_peak_us_, static_cast<uint32_t>(now_us_() - now));
    }
    this->tx_bytes_ = bytes;
    this->tx_prepared_ = true;
  } else if (this->tx_normal_) {
    time_us = this->sched_.retime_frame(start, this->tx_buf_);
  }
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->rw_dir_pin_ != nullptr && !this->rw_driving_) {
    this->rw_dir_pin_->digital_write(true);   // the hub drives the line again (the window is over)
    this->rw_driving_ = true;
  }
  // only an RW frame ends early (its window is the reset): every other frame has the full reset
  this->enc_.symbols.reset = this->tx_rw_ ? this->rw_reset_symbol_ : this->reset_symbol_;
  if (this->tx_rw_) {
    uint8_t junk[64];
    size_t n;
    while ((n = this->rw_uart_->available()) > 0 && this->rw_uart_->read_array(junk, std::min(n, sizeof(junk))))
      ;   // the window's bytes start here (the hub's own frames, breaks)
    this->rw_release_ = this->rw_dir_pin_ != nullptr;
  }
#endif
  rmt_transmit_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  const esp_err_t err = this->lane_ != nullptr
                            ? this->lane_post_()
                            : rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, this->tx_bytes_, &cfg);
#else
  const esp_err_t err = rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, this->tx_bytes_, &cfg);
#endif
  if (err != ESP_OK) {
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
    this->rw_release_ = false;
#endif
    if (!this->status_has_warning())   // the frame is retried every loop: log the first failure only
      ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    this->status_set_warning();
    return;
  }
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->tx_rw_) {
    // nothing goes out until the window ends (W + ULTRA_RW_HOLDOFF_US after the frame's last bit)
    this->rw_open_ = true;
    this->rw_t0_us_ = time_us;
    this->rw_end_us_ = time_us + this->rw_window_us_ + ULTRA_RW_HOLDOFF_US;
    this->rw_read_us_ = this->rw_end_us_ + 2000;
    if (this->rw_dir_pin_ != nullptr)
      this->rw_driving_ = false;   // (the RMT's interrupt lets go of the line)
    this->tx_rw_ = false;
    this->wake_();
  }
#endif
  this->tx_prepared_ = false;
#ifdef USE_DIFFPOWER
  if (this->sched_.take_power_off_sent() && this->port_ != nullptr) {
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
    if (this->lane_ != nullptr) {
      this->notice_id_ = this->lane_->posted_id();   // (once out on the wire the port is told: service_notice_())
      this->notice_epoch_ = this->lane_->posted_epoch();
      this->notice_end_us_ = this->lane_->frame_end_us();
    } else
#endif
      this->port_->notice_sent();   // (its next copy 50 ms from now)
  }
#endif
  if (this->tx_preamble_end_ && this->boards_powered_())
    this->synced_once_ = true;
  this->status_clear_warning();
  this->last_frame_ms_ = millis();
  this->has_sent_ = true;
  this->frames_++;
}

bool UltraHub::line_free_() {
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  if (this->lane_ != nullptr)
    return this->lane_->ready() && !this->lane_->busy();
#endif
  return rmt_tx_wait_all_done(this->channel_, 0) == ESP_OK;
}

bool UltraHub::line_out_() {
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
  if (this->lane_ != nullptr)
    return now_us_() >= this->lane_->end_us();   // (its frame and its reset: the lane's quiet after it)
#endif
  return rmt_tx_wait_all_done(this->channel_, 0) == ESP_OK;
}

#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
// The frame to the lane: its quiet after it is the reset, after an RW frame the window (the receivers answer) and the
// hold-off; with a direction pin the bus' timer lets go of the line at its last bit. ESP_ERR_TIMEOUT: its start
// passed while it was built (sent again with a new start next loop, like a frame the RMT refused).
esp_err_t UltraHub::lane_post_() {
  uint32_t quiet = this->reset_ns_ / 1000;
  bool release = false;
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->tx_rw_) {
    quiet = std::max<uint32_t>(quiet, this->rw_window_us_ + ULTRA_RW_HOLDOFF_US);
    release = this->rw_dir_pin_ != nullptr;
  }
#endif
  return this->lane_->post(this->tx_buf_, this->tx_bytes_, quiet, release) ? ESP_OK : ESP_ERR_TIMEOUT;
}
#endif

void UltraHub::dump_config() {
  const uint32_t period0 = this->bit0_high_ns_ + this->bit0_low_ns_;
  const uint32_t period1 = this->bit1_high_ns_ + this->bit1_low_ns_;
  const uint32_t worst = period0 > period1 ? period0 : period1;
  const uint32_t bits = static_cast<uint32_t>(ULTRA_HDR_BITS + ULTRA_SLOT_BITS * this->num_boards_);
  const uint32_t frame_us = static_cast<uint32_t>((static_cast<uint64_t>(bits) * worst) / 1000) + this->reset_ns_ / 1000;
  ESP_LOGCONFIG(TAG,
                "QuinLED Ultra protocol line:\n"
                "  Boards: %u\n"
                "  Pin: GPIO%u%s\n"
                "  %s: %" PRIu32 "%s\n"
                "  Bit0: %" PRIu32 "/%" PRIu32 " ns, Bit1: %" PRIu32 "/%" PRIu32 " ns, Reset: %" PRIu32 " us\n"
                "  Frame: %" PRIu32 " us (%" PRIu32 " frames/s max)\n"
                "  Refresh: every %" PRIu32 " ms, start delay %" PRIu32 " ms%s, %u copies per command\n"
                "  Preamble: %u frames of DARK slots (after a supply switch-on: %u)",
                this->num_boards_, this->pin_, this->inverted_ ? " (inverted)" : "",
#ifdef USE_DIG2ANALOG_ULTRA_PARLIO
                this->lane_ != nullptr ? "PARLIO lane" : "RMT symbols",
                this->lane_ != nullptr ? static_cast<uint32_t>(this->lane_->index()) : this->rmt_symbols_,
#else
                "RMT symbols", this->rmt_symbols_,
#endif
                this->use_dma_ ? " (DMA)" : "", this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_,
                this->bit1_low_ns_, this->reset_ns_ / 1000, frame_us, frame_us ? 1000000 / frame_us : 0,
                this->refresh_interval_ms_, this->start_delay_ms(),
                this->start_delay_us_ < 0 ? " (auto: + a frame per command waiting, group changes share a start)" : "",
                this->repeats_,
                preamble_frames(this->num_boards_), power_on_preamble_frames(this->num_boards_));
  if (this->num_boards_ > 0 && this->sched_.has_config(0, ULTRA_CFG_BOOT_PROTOCOL)) {
    static const char *const MODES[] = {"auto", "WS2915", "WS2805", "Ultra"};
    const uint16_t m = this->sched_.get_config(0, ULTRA_CFG_BOOT_PROTOCOL);
    ESP_LOGCONFIG(TAG, "  Boot protocol (stored on the boards): %s", MODES[m < 4 ? m : 0]);
  }
  for (uint16_t b = 0; b < this->num_boards_ && b < 4; b++) {
    const uint16_t preset = this->sched_.get_config(b, ULTRA_CFG_PWM_PRESET);
    const uint16_t period = this->sched_.get_config(b, ULTRA_CFG_PWM_PERIOD);
    const uint32_t steps = period != 0 ? period : (preset < 15 ? PWM_PRESET_STEPS[preset] : 0);
    ESP_LOGCONFIG(TAG,
                  "  Board %u: PWM %.1f Hz (%" PRIu32 " steps), dithering %u, threshold %.1f %%, dim floor %u, "
                  "min pulse %u ns, stagger %u, signal loss %u after %.1f s, board limit %.2f A",
                  b, steps ? PWM_CLOCK_HZ / static_cast<float>(steps) : 0.0f, steps,
                  this->sched_.get_config(b, ULTRA_CFG_DITHER), this->sched_.get_config(b, ULTRA_CFG_DITHER_THRESH) / 10.0f,
                  this->sched_.get_config(b, ULTRA_CFG_DIM_FLOOR), this->sched_.get_config(b, ULTRA_CFG_MIN_PULSE) * 10u,
                  this->sched_.get_config(b, ULTRA_CFG_STAGGER), this->sched_.get_config(b, ULTRA_CFG_LOSS),
                  this->sched_.get_config(b, ULTRA_CFG_LOSS_TIMEOUT) / 10.0f,
                  this->sched_.get_config(b, ULTRA_CFG_BOARD_MAX) / 1000.0f);
  }
#ifdef USE_POWER_SUPPLY
  if (this->psu_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Power supply: replay after %" PRIu32 " ms", this->power_on_delay_ms_);
#endif
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->rw_uart_ != nullptr) {
    static const char *const RATES[] = {"1 Mbaud", "500 kbaud", "250 kbaud"};
    ESP_LOGCONFIG(TAG, "  Telemetry: %u receiver(s) (boards 0-%u), answers at %s, windows up to %" PRIu32 " us%s%s",
                  this->poller_.receivers(), this->poller_.receivers() - 1, RATES[this->poller_.rate() % 3],
                  this->poller_.max_window_us(), this->rw_dir_pin_ != nullptr ? ", direction pin" : "",
                  this->rw_rx_enable_pin_ != nullptr ? ", receiver enable pin (its UART may be shared)" : "");
  }
#endif
#ifdef USE_DIG2ANALOG_ULTRA_UPDATE
  if (this->ping_burst_)
    ESP_LOGCONFIG(TAG, "  Firmware update: %" PRIu32 " PING frames after each preamble (UPD frame %" PRIu32 " us)",
                  fw_ping_burst_frames(this->num_boards_), this->update_frame_us());
#endif
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup FAILED");
}

}  // namespace esphome::dig2analog_ultra

#endif  // USE_ESP32
