#include "ultra_line.h"
#include "ultra_light.h"
#include "ultra_sender_update.h"   // (ultra_port_line_profile: the line's start profile)

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <memory>
#include <new>
#include <esp_attr.h>
#include <esp_clk_tree.h>
#include <esp_random.h>
#include <esp_timer.h>
#ifdef USE_ULTRA_LINE_UPDATE
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif
#ifdef USE_ULTRA_PARLIO
#include <esp_memory_utils.h>
#include <hal/gpio_ll.h>
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
#include "esphome/components/uart/uart_component_esp_idf.h"
#include <driver/uart.h>
#endif
#ifdef USE_API_CUSTOM_SERVICES
#include "esphome/components/api/api_server.h"
#include "esphome/components/api/user_services.h"
#endif
#include <strings.h>

namespace esphome::ultra_line {

static const char *const TAG = "ultra_line";

static constexpr uint32_t RMT_DURATION_MAX = 0x7FFF;  // 15-bit duration field
// Keep the main loop in high-frequency mode this long after the last command, so the copies and
// commands that follow (effect steps, a scene of several lights) go out back to back.
static constexpr uint32_t BUSY_HOLD_MS = 250;

static uint32_t rmt_resolution_hz() { return 40000000u; }

/// An RW frame ends with this much LOW before the hub lets go of the line (the protocol: ULTRA_RW_RELEASE_MIN_US .. ULTRA_RW_RELEASE_US after its
/// last edge; the aim is the lower bound + 20 us). A receiver that closes a frame by its LOW run (the Diff-Power Ultra Receiver Digital-Medium on a normal line) needs it:
/// the line stays HIGH after release, and a 2 us LOW before that HIGH is too short to end its frame.
static constexpr uint32_t RW_RELEASE_HOLD_NS = (ULTRA_RW_RELEASE_MIN_US + 20u) * 1000u;
static_assert(RW_RELEASE_HOLD_NS >= ULTRA_RW_RELEASE_MIN_US * 1000u && RW_RELEASE_HOLD_NS + 50000u <= ULTRA_RW_RELEASE_US * 1000u,
              "the hold leaves the RMT interrupt's latency inside the protocol's release window");
#ifdef USE_ULTRA_PARLIO
static_assert(PARLIO_RELEASE_NS == RW_RELEASE_HOLD_NS, "a lane and an RMT line hold the pair LOW the same time");
#endif

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

#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
// A digital port's encoder: lead LOW, marker, the bits, tail LOW (ultra_dport.h), the symbol table set per transmission
static size_t IRAM_ATTR HOT dp_encoder_callback(const void *data, size_t size, size_t symbols_written,
                                                size_t symbols_free, rmt_symbol_word_t *symbols, bool *done,
                                                void *arg) {
  auto *e = static_cast<DpEncoder *>(arg);
  int finished = 0;
  const size_t n = ultra_dp_rmt_encode(&e->tab, static_cast<const uint8_t *>(data), size, symbols_written,
                                       reinterpret_cast<uint32_t *>(symbols), symbols_free, &finished);
  *done = finished != 0;
  return n;
}
#endif

int64_t UltraHub::now_us_() { return esp_timer_get_time(); }

int UltraHub::board_slot_(int board, const char *what) const {
  if (board < 0)
    return -1;
  if (board >= 1 && board <= this->num_boards_)
    return board - 1;
  if (board == 0)
    ESP_LOGW(TAG, "%s on %s: board 0: boards count from 1 (the first board after the controller is board 1)", what,
             this->line_name_());
  else
    ESP_LOGW(TAG, "%s on %s: board %d isn't on it (1-%u)", what, this->line_name_(), board, this->num_boards_);
  return NO_BOARD;
}

void UltraHub::set_board_config(int board, uint8_t id, uint16_t value) {
  board = this->board_slot_(board, "set_board_config");
  if (board == NO_BOARD)
    return;
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

void UltraHub::set_channel_config(int board, uint8_t channel, uint8_t id, uint16_t value) {
  board = this->board_slot_(board, "set_channel_config");
  if (board == NO_BOARD)
    return;
  if (!this->started_) {
    this->early_config_.push_back({board, id, value, channel});
    return;
  }
  for (uint16_t b = 0; b < this->num_boards_; b++)
    if (board < 0 || board == b) this->sched_.set_channel_config(b, channel, id, value);
  this->sched_.flush_config();
  this->wake_();
}

void UltraHub::set_pwm_frequency(int board, float hz) {
  board = this->board_slot_(board, "set_pwm_frequency");
  if (board == NO_BOARD)
    return;
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

// ------------------------------------------------------------------- the receivers' settings from Home Assistant
// (wiki: Controller-Settings) Per board on top of the YAML's values: kept on the ESP32, sent with the board's settings
// (host-managed: at boot and with every refresh, so a board that restarts or is replaced gets them again).

void UltraHub::rx_settings_setup_() {
  const size_t n = this->num_boards_;
  for (uint8_t k = 0; k < 8; k++) {   // the YAML's (it sets every board alike: board 0's)
    if (n != 0 && this->sched_.has_config(0, RX_CFG[k])) {
      this->rx_yaml_[k] = this->sched_.get_config(0, RX_CFG[k]);
      this->rx_yaml_have_ |= static_cast<uint8_t>(1u << k);
    }
  }
  this->rx_ovr_.assign(n, RxOverride{});
  this->rx_pref_ = global_preferences->make_preference(
      n * sizeof(RxOverride), fnv1_hash(std::string("ultra_line_rx:") + this->line_id_) ^ n, true);
  if (n == 0 || !this->rx_pref_.load(reinterpret_cast<uint8_t *>(this->rx_ovr_.data()), n * sizeof(RxOverride))) {
    this->rx_ovr_.assign(n, RxOverride{});
    return;
  }
  for (uint16_t b = 0; b < n; b++) {
    const RxOverride &o = this->rx_ovr_[b];
    for (uint8_t k = 0; k < 8; k++) {
      if (o.mask & (1u << k))
        this->sched_.set_config(b, RX_CFG[k], o.cfg[k]);
    }
  }
}

void UltraHub::rx_settings_lights_() {
  this->rx_lights_done_ = true;
  for (UltraLight *l : this->lights_) {
    const uint16_t b = l->get_board();
    const uint8_t bit = static_cast<uint8_t>(1u << l->get_light_index());
    if (b >= this->rx_ovr_.size())
      continue;
    RxOverride &o = this->rx_ovr_[b];
    if ((o.dtw_set & bit) && l->dim_to_warm_available())
      l->set_dim_to_warm((o.dtw_on & bit) != 0);
  }
  this->settings_cb_.call(-1);
}

void UltraHub::rx_settings_save_() {
  if (!this->rx_ovr_.empty())
    this->rx_pref_.save(reinterpret_cast<const uint8_t *>(this->rx_ovr_.data()),
                        this->rx_ovr_.size() * sizeof(RxOverride));
}

// (the line in the log: its telemetry name, else its id)
const char *UltraHub::line_name_() const {
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->has_telemetry())
    return this->tel_name_;
#endif
  return this->line_id_;
}

// a key and its value; a bad one: a warning that says what it takes, -1
int UltraHub::rx_setting_key_(const char *line, const std::string &key, const std::string &value) {
  const int k = rx_key(key);
  RxOverride t;
  bool on = false;
  if (k < 0)
    ESP_LOGW(TAG, "set_receiver_setting on %s: no key '%s' (pwm, dither, stagger, loss, loss_timeout, limit, "
             "retries, retry_interval, dtw or reset)", line, key.c_str());
  else if ((k == RK_DTW && !parse_on_off(value, &on)) || (k < RK_DTW && !rx_override_cfg(t, k, value, 0)))
    ESP_LOGW(TAG, "set_receiver_setting on %s: %s '%s' isn't valid: %s", line, RX_KEYS[k], value.c_str(),
             RX_VALUES[k]);
  else
    return k;
  return -1;
}

bool UltraHub::set_receiver_setting(int board, const std::string &key, const std::string &value) {
  const int k = rx_setting_key_(this->line_name_(), key, value);
  if (k < 0)
    return false;
  if (board == 0) {   // (board: 1 = the first, as in YAML; -1 every board)
    ESP_LOGW(TAG, "set_receiver_setting on %s: receiver 0: boards count from 1 (the first board after the controller "
             "is board 1)", this->line_name_());
    return false;
  }
  if (board > this->num_boards_) {
    ESP_LOGW(TAG, "set_receiver_setting on %s: receiver %d isn't on it (1-%u)", this->line_name_(), board,
             this->num_boards_);
    return false;
  }
  const int slot = board < 0 ? -1 : board - 1;
  bool on = false;
  parse_on_off(value, &on);
  bool ok = false;
  for (uint16_t b = 0; b < this->num_boards_ && b < this->rx_ovr_.size(); b++) {
    if (slot >= 0 && slot != b)
      continue;
    RxOverride &o = this->rx_ovr_[b];
    if (k == RK_RESET) {   // the YAML's again (an id it doesn't set: no longer sent, the board keeps its own)
      o = RxOverride{};
      for (uint8_t i = 0; i < 8; i++) {
        if (this->rx_yaml_have_ & (1u << i))
          this->sched_.set_config(b, RX_CFG[i], this->rx_yaml_[i]);
        else
          this->sched_.unset_config(b, RX_CFG[i]);
      }
      for (UltraLight *l : this->lights_) {
        if (l->get_board() != b)
          continue;
        if (l->dim_to_warm_available())
          l->set_dim_to_warm(true);
      }
      ok = true;
    } else if (k == RK_DTW) {   // per light of the board
      for (UltraLight *l : this->lights_) {
        if (l->get_board() != b)
          continue;
        const uint8_t idx = l->get_light_index(), bit = static_cast<uint8_t>(1u << idx);
        if (!l->dim_to_warm_available()) continue;
        l->set_dim_to_warm(on);
        o.dtw_set |= bit;
        o.dtw_on = static_cast<uint8_t>(on ? o.dtw_on | bit : o.dtw_on & ~bit);
        ok = true;
      }
    } else {
      const uint16_t retry = this->sched_.has_config(b, ULTRA_CFG_RETRY) ? this->sched_.get_config(b, ULTRA_CFG_RETRY)
                                                                         : static_cast<uint16_t>(10 | 1 << 8);
      if (!rx_override_cfg(o, k, value, retry))   // (checked above)
        return false;
      for (uint8_t i = 0; i < 8; i++) {
        if (o.mask & (1u << i))
          this->sched_.set_config(b, RX_CFG[i], o.cfg[i]);
      }
      ok = true;
    }
  }
  const std::string where = board < 0 ? std::string("every receiver") : "receiver " + std::to_string(board);
  if (!ok) {   // (only dtw: no light of the board takes it)
    if (k == RK_DTW)
      ESP_LOGW(TAG, "set_receiver_setting on %s, %s: dtw: no light there has dim_to_warm: in its YAML",
               this->line_name_(), where.c_str());
    return false;
  }
  ESP_LOGI(TAG, "set_receiver_setting on %s, %s: %s %s", this->line_name_(), where.c_str(), RX_KEYS[k], value.c_str());
  this->sched_.flush_config();
  this->wake_();
  this->rx_settings_save_();
  this->settings_cb_.call(slot);
  return true;
}

std::string UltraHub::receiver_settings_text(uint16_t board) const {
  RxSettingsView v;
  for (uint8_t id = 0; id < HOST_CONFIG_COUNT; id++) {
    v.cfg[id] = this->sched_.get_config(board, id);
    if (this->sched_.has_config(board, id))
      v.have |= 1u << id;
  }
  std::vector<const UltraLight *> ls;   // the board's lights in their order (light index)
  for (const UltraLight *l : this->lights_) {
    if (l->get_board() == board)
      ls.push_back(l);
  }
  std::sort(ls.begin(), ls.end(),
            [](const UltraLight *a, const UltraLight *b) { return a->get_light_index() < b->get_light_index(); });
  for (size_t i = 0; i < ls.size(); i++) {
    v.dtw += (i == 0 ? "" : "|") + std::string(!ls[i]->dim_to_warm_available() ? "n/a"
                                               : ls[i]->get_dim_to_warm()       ? "on"
                                                                                : "off");
  }
  return rx_settings_text(v);
}

bool UltraHub::is_line(const std::string &name) const {
  if (strcasecmp(name.c_str(), this->line_id_) == 0)
    return true;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->has_telemetry() && strcasecmp(name.c_str(), this->tel_name_) == 0)
    return true;
#endif
  return false;
}

void UltraHub::receiver_setting_action(const std::string &port, const std::vector<int32_t> &receivers,
                                       const std::string &key, const std::string &value) {
  const bool all = strcasecmp(port.c_str(), "all") == 0;
  bool any = false;
  for (UltraHub *hub : hubs_)
    any |= all || hub->is_line(port);
  if (!any) {
    ESP_LOGW(TAG, "set_receiver_setting: no line '%s' (a telemetry name, a line's id, or all)", port.c_str());
    return;
  }
  if (rx_setting_key_(port.c_str(), key, value) < 0)   // (one warning, not one per receiver)
    return;
  for (UltraHub *hub : hubs_) {
    if (!all && !hub->is_line(port))
      continue;
    if (receivers.empty()) {
      hub->set_receiver_setting(-1, key, value);
      continue;
    }
    for (int32_t r : receivers) {
      if (r >= 0 && r <= hub->num_boards_)   // (0: its own warning, receivers count from 1)
        hub->set_receiver_setting(r, key, value);
      else
        ESP_LOGW(TAG, "set_receiver_setting on %s: receiver %d isn't on it (1-%u)", hub->line_name_(),
                 static_cast<int>(r), hub->num_boards_);
    }
  }
}

#ifdef USE_API_CUSTOM_SERVICES
/// The action set_receiver_setting(port, receivers, key, value): Home Assistant's esphome.<device>_set_receiver_setting
class ReceiverSettingService final
    : public api::UserServiceDynamic<std::string, std::vector<int32_t>, std::string, std::string> {
 public:
  ReceiverSettingService() : UserServiceDynamic("set_receiver_setting", {"port", "receivers", "key", "value"}) {}

 protected:
  void execute(uint32_t, bool, std::string port, std::vector<int32_t> receivers, std::string key,
               std::string value) override {
    UltraHub::receiver_setting_action(port, receivers, key, value);
  }
};
#endif

// A board forwards Ultra frames only once it is locked: a preamble of DARK frames (preamble_frames()) lets the
// whole chain lock (boards that were locked to a WS protocol show dark meanwhile) before the commands come.
void UltraHub::resync(bool power_on) {
  this->tx_prepared_ = false;  // a frame the RMT refused: dropped, the replay sends everything again
#ifdef USE_ULTRA_LINE_TELEMETRY
  this->tx_rw_ = false;
  this->rw_drop_turn_();       // (a poll frame among them: its UART's turn goes to the next line)
#endif
  this->preamble_full_ = !(power_on && this->synced_once_);
  this->preamble_ = this->preamble_full_ ? preamble_frames(this->num_boards_) : power_on_preamble_frames(this->num_boards_);
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) {
    this->preamble_ = 0;   // (the marker is the "for you"; nothing to lock)
    this->preamble_full_ = true;
    if (power_on) {   // the receiver just booted with its supply: no marker for ULTRA_DP_POWERUP_MS from then
      ultra_dp_powered(&this->dp_, this->now_us_() - static_cast<int64_t>(this->power_on_delay_ms_) * 1000);
    }
#ifdef USE_ULTRA_LINE_TELEMETRY
    this->poller_.locked(millis());
#endif
  }
#endif
  int64_t lead_us = static_cast<int64_t>(this->preamble_) * (this->frame_us_ + 200);
  if (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES)
    lead_us += static_cast<int64_t>(ULTRA_SWITCH_WS_FRAMES - this->switch_frame_) * (this->switch_frame_us_ + 200);
  const int64_t replay_us = this->now_us_() + lead_us + this->stream_lead_us_();
  this->sched_.replay(replay_us);
  this->change_log_.restart(static_cast<uint32_t>((replay_us + 999) / 1000));
  this->wake_();
}

UltraHub::Diagnostics UltraHub::take_diagnostics() {
  uint32_t underruns = this->enc_.watch.count;
#ifdef USE_ULTRA_PARLIO
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
#ifdef USE_ULTRA_PARLIO
  // a lane's mailbox takes a frame at most every few chunks (a frame waits for the one before it to be taken):
  // short frames on long chunks come that far apart
  frame = std::max(frame, this->lane_lead_us_);
#endif
  if (this->start_delay_us_ >= 0)
    return this->start_delay_us_ + static_cast<int64_t>(commands > 1 ? commands - 1 : 0) * frame;
  int64_t lead = start_lead_us(frame, commands, waiting);
#ifdef USE_ULTRA_LINE_TELEMETRY
  lead += this->rw_extra_us_;   // a command queued as a response window opens waits for its end
#endif
#ifdef USE_ULTRA_PARLIO
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
#ifdef USE_ULTRA_PARLIO
  frame = std::max(frame, this->lane_lead_us_);
#endif
  int64_t reach = start_lead_us(frame, 1, this->sched_.waiting(board)) - START_SLACK_US;
#ifdef USE_ULTRA_PARLIO
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

// Shared by the update lease, commissioning and telemetry page selection. Port-up alone does not establish that
// a cold receiver has booted. The power epoch also invalidates an answer on a detect dropout or standby wake.
bool UltraHub::receivers_ready_() const {
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr) {
    bool returns = false;
#ifdef USE_ULTRA_LINE_TELEMETRY
    returns = this->rw_uart_ != nullptr;
#endif
    const auto &core = this->port_->core();
    const bool ready = this->receiver_ready_.ready(millis(), this->boards_powered_(), core.line_on(), core.line_ms(),
                                                 returns, core.config().boot_margin_ms);
    if (this->receiver_ready_.take_warning())
      ESP_LOGW(TAG, "GPIO%u: no receiver answered within %u ms after port-up; continuing with startup allowance",
               this->pin_, RECEIVER_REPLY_TIMEOUT_MS);
    return ready;
  }
#endif
  return this->boards_powered_();
}

#ifdef USE_ULTRA_LINE_TELEMETRY
void UltraHub::receiver_reply_() {
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr) {
    const auto &core = this->port_->core();
    this->receiver_ready_.answer(core.line_on(), core.line_ms(), uint32_t(this->rw_t0_us_ / 1000));
    if (this->receiver_ready_.answered())
      this->port_->wake_reply();   // (the wake timeline: the first reply of this power-on)
  }
#endif
}
#endif

#ifdef USE_ULTRA_LINE_UPDATE
bool UltraHub::job_timing(ultra_timing_t *out) const {
  if (ultra_timing_profile(out, this->line_profile(), 40000000u, this->line_gap_us()))
    return false;
  return uint32_t(out->zero_ticks) * 25u == this->bit0_high_ns_ &&
         uint32_t(out->bit_ticks - out->zero_ticks) * 25u == this->bit0_low_ns_ &&
         uint32_t(out->one_ticks) * 25u == this->bit1_high_ns_ &&
         uint32_t(out->bit_ticks - out->one_ticks) * 25u == this->bit1_low_ns_;
}

int64_t UltraHub::job_frame_start_us() {
#ifdef USE_ULTRA_PARLIO
  if (this->lane_ != nullptr)
    return this->lane_->reserve_us();
#endif
  return now_us_();
}

// The task's frame: the hub's buffer (the main loop sends nothing while the line is leased), its encoder; a lane's
// post, waited for until it is on the wire; a digital port's frame inside the job's marker envelope.
bool UltraHub::job_send(const uint8_t *frame, size_t n, const ultra_update_transport_t &t) {
  if (frame == nullptr || n == 0 || n > this->tx_buf_bytes_ || !this->started_ || this->is_failed() || !this->upd_lease_)
    return false;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_open_ || (this->rw_dir_pin_ != nullptr && !this->rw_driving_))
    return false;   // (lease_line hasn't handed it over yet: never a frame on a line the hub doesn't drive)
#endif
  std::memcpy(this->tx_buf_, frame, n);
#ifdef USE_ULTRA_PARLIO
  if (this->lane_ != nullptr) {
    if (!this->lane_->post(this->tx_buf_, n, this->line_gap_us(), false))
      return false;
    const uint32_t id = this->lane_->posted_id(), epoch = this->lane_->posted_epoch();
    const int64_t end = this->lane_->frame_end_us(), until = now_us_() + 200000;
    uint8_t outcome = ParlioLane::PENDING;
    while (outcome == ParlioLane::PENDING && now_us_() < until) {
      outcome = this->lane_->outcome(id, epoch, end, now_us_());
      if (outcome == ParlioLane::PENDING)
        vTaskDelay(1);
    }
    return outcome == ParlioLane::ON_WIRE;
  }
#endif
  rmt_transmit_config_t cfg;
  std::memset(&cfg, 0, sizeof(cfg));
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) {
    ultra_dp_rmt_t &tab = this->dp_enc_.tab;
    std::memset(&tab, 0, sizeof(tab));
    tab.lead = ultra_dp_rmt_low(t.lead_low_us, this->rmt_res_);
    tab.tail = ultra_dp_rmt_low(t.tail_low_us, this->rmt_res_);
    if (t.marker)
      tab.marker = ultra_dp_rmt_marker(this->rmt_res_);
    tab.bit0 = this->enc_.symbols.bit0.val;
    tab.bit1 = this->enc_.symbols.bit1.val;
  }
#endif
  (void) t;
  return rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, n, &cfg) == ESP_OK &&
         rmt_tx_wait_all_done(this->channel_, 200) == ESP_OK;
}

// The hub's own frames stop at once; the job gets the line only as transmit_() would send its next frame: a response
// window that is open is over and read first, then the hub drives the line again (direction pin) with the full reset.
// Power readiness is shared with commissioning and polls: the full update PING/training must start after wake.
// false: not yet (the main loop asks again; the job's task waits). Also when the job takes it back after verifying.
bool UltraHub::lease_line(bool on) {
  if (!on) {
    if (this->upd_lease_) {
      this->upd_lease_ = false;
      this->resync();   // handed back: the boards' state again, after a fresh preamble
    }
    return true;
  }
  // Keep the hub's startup frames and STATUS probes running until a cold receiver answers. Taking the line here
  // would suppress the very polls that establish readiness. An unstarted job still cancels without UPDATE traffic.
  if (!this->receivers_ready_())
    return false;
  this->upd_lease_ = true;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_open_) {
    const int64_t now = now_us_();
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
    if (this->digital_) {
      this->digital_window_track_();
      if (now < this->rw_end_us_)
        return false;
      this->digital_window_end_(now);
    } else
#endif
    {
      if (now < this->rw_read_us_)
        return false;   // (its end, then its bytes)
      this->finish_window_(false);
    }
  }
  if (this->tx_prepared_ && this->tx_rw_) {   // a poll frame not sent: asked again once the line is back
    this->tx_prepared_ = false;
    this->tx_rw_ = false;
    this->rw_drop_turn_();
  }
  this->rw_release_ = false;
  if (this->rw_dir_pin_ != nullptr && !this->rw_driving_) {
    this->rw_dir_pin_->digital_write(true);
    this->rw_driving_ = true;
  }
  this->enc_.symbols.reset = this->reset_symbol_;
#endif
  return this->boards_powered_();
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
#endif

#ifdef USE_ULTRA_LINE_TELEMETRY
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

// The RMT's interrupt after an RW frame's last symbol (its LOW of RW_RELEASE_HOLD_NS): let go of the line, within ULTRA_RW_RELEASE_US of its last bit.
bool IRAM_ATTR UltraHub::rw_done_isr_(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *arg) {
  auto *hub = static_cast<UltraHub *>(arg);
  if (hub->rw_release_) {
    hub->rw_dir_isr_.digital_write(false);
    hub->rw_release_ = false;
    hub->rw_released_us_ = esp_timer_get_time();
  }
  return false;
}

#ifdef USE_ULTRA_PARLIO
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
  const TelemetryPoller::Stats &st = this->poller_.stats();
  const uint32_t answers = st.answers, broken = st.bad_crc;
  this->poller_.finish(millis(), this->rw_rx_.data(), this->rw_rx_.size());
  if (st.answers != answers)
    this->receiver_reply_();
  // the reply rate: a rate slower once broken answers keep coming, faster again after a quiet hour (ReplyRate)
  const uint8_t was = this->reply_.rate();
  if (this->reply_.window(millis(), st.answers - answers, st.bad_crc - broken)) {
    static const char *const RATES[] = {"1 Mbaud", "500 kbaud", "250 kbaud"};
    this->poller_.set_rate(this->reply_.rate());
    this->rw_extra_us_ = this->poller_.max_window_us() + ULTRA_RW_END_US + this->line_gap_us();   // (its windows' length)
    if (this->reply_.rate() > was) {
      ESP_LOGW(TAG, "Telemetry on GPIO%u: broken answers in %u of the last 32 windows: the receivers answer at %s now "
               "(faster again after %" PRIu32 " min without)", this->pin_, static_cast<unsigned>(ReplyRate::BAD),
               RATES[this->reply_.rate()], this->reply_.quiet_needed_ms() / 60000);
    } else {
      ESP_LOGI(TAG, "Telemetry on GPIO%u: no broken answers for %" PRIu32 " min: the receivers answer at %s again",
               this->pin_, this->reply_.quiet_needed_ms() / 60000, RATES[this->reply_.rate()]);
    }
  }
  this->rw_publish_();
}

// ESPHome's UART keeps the rate it was given (its baud_rate member), the driver's is set directly: a register write
// (load_settings() would reinstall the driver). Only between windows: the line is quiet then.
void UltraHub::rw_uart_rate_() {
  static const uint32_t BAUD[] = {1000000, 500000, 250000};
  const uint32_t baud = BAUD[this->poller_.rate() % ULTRA_RW_RATES];
  if (this->rw_uart_->get_baud_rate() == baud)
    return;
  this->rw_uart_->set_baud_rate(baud);
  auto *idf = static_cast<uart::IDFUARTComponent *>(this->rw_uart_);
  if (uart_set_baudrate(static_cast<uart_port_t>(idf->get_hw_serial_number()), baud) != ESP_OK)
    ESP_LOGW(TAG, "Telemetry on GPIO%u: the UART's rate can't be set to %" PRIu32, this->pin_, baud);
}

// What changed per receiver: the port's load first (the receivers show their part), the log, then the sensors.
void UltraHub::rw_publish_() {
  bool output_voltage_low_hold = this->port_standby() || this->port_off() || this->calibration_running();
#ifdef USE_ULTRA_LINE_UPDATE
  output_voltage_low_hold |= this->update_running();
#endif
  this->poller_.service_output_voltage_low(millis(), output_voltage_low_hold);
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
    if (u & TU_RESTART) {this->sched_.forget_descriptor(i);this->line_rate_.forget(i);}
    if (u & (1u << TP_LINK_CONTEXT)) {
      if (d.line_counters.coherent)
        ESP_LOGD(TAG, "GPIO%u receiver %u: RATE_REQUEST=%s, profile %u, supported 0x%02x, epoch %u",
                 this->pin_, i + 1, receiver_requests_lower_rate(d.line_counters) ? "on" : "off",
                 d.line_counters.profile, d.line_counters.supported, d.line_counters.epoch);
      if (d.line_counters.coherent && d.line_counters.profile == this->line_profile() &&
          (d.line_counters.flags & ULTRA_LS_LOCKED) && !this->line_rate_reason_.empty()) {
        this->line_rate_reason_.clear();
        this->publish_line_speed_();
      }
      if (this->line_rate_.observe(i, d.line_counters, millis()))
        ESP_LOGW(TAG, "GPIO%u receiver %u: %s requests profile %u", this->pin_, i + 1,
                 receiver_requests_lower_rate(d.line_counters) ? "RATE_REQUEST" : "CRC error window",
                 this->line_rate_.requested());
    }
    if (d.have_descriptor) this->sched_.descriptor(i, d.descriptor_bytes);
    if (u & ((1u << TP_DESCRIPTOR) | TU_RESTART)) this->refresh_line_gap_();
    if ((u & TU_AVAILABILITY) && (this->port_standby() || (note & RX_SETTLING))) {
      // (its line's power switched on purpose: off, or just back and not evaluated yet)
      ESP_LOGV(TAG, "Receiver %u on GPIO%u: %s", i + 1, this->pin_, this->port_standby() ? "standby" : "waking");
    } else if (u & TU_AVAILABILITY) {
      if (this->poller_.answering(i)) {
        ESP_LOGI(TAG, "Receiver %u on GPIO%u answers", i + 1, this->pin_);
      } else {
        const int cut = this->poller_.cut_at(millis());
        if (cut >= 0 && cut < i)
          ESP_LOGW(TAG, "Receiver %u on GPIO%u can't be heard: receiver %d in front of it doesn't answer", i + 1,
                   this->pin_, cut + 1);
        else
          ESP_LOGW(TAG, "Receiver %u on GPIO%u doesn't answer", i + 1, this->pin_);
      }
    }
    if (u & TU_RESTART) {
      if ((note & RX_WOKE) && millis() - this->woke_ms_ < 60000) {   // (its port's wake: expected)
        ESP_LOGV(TAG, "Receiver %u on GPIO%u restarted (its port woke)", i + 1, this->pin_);
      } else {
        ESP_LOGI(TAG, "Receiver %u on GPIO%u restarted or was replaced", i + 1, this->pin_);
      }
      note &= ~RX_WOKE;
    }
#ifdef USE_DIFFPOWER
    if (this->port_ != nullptr && d.have_identity && i < this->rx_note_.size()) {
      const bool none = !(d.features & ULTRA_FEAT_STANDBY);
      if (none != ((note & RX_NO_STANDBY) != 0)) {   // (counted once each way: an update, a replaced board)
        note ^= RX_NO_STANDBY;
        this->no_standby_ = static_cast<uint8_t>(this->no_standby_ + (none ? 1 : -1));
      }
      if (none && !(note & RX_WARNED)) {
        note |= RX_WARNED;
        ESP_LOGW(TAG, "Receiver %u on GPIO%u does not advertise standby: it shows its power-on "
                 "scene and writes its flash at each wake of its port, so the port powers off at most %u times a "
                 "day; provision standby support", i + 1, this->pin_, PowerOffCap::CAP);
      }
    }
#endif
    if (i < this->rx_note_.size())
      this->rx_note_[i] = note;
    if ((u & (1u << TP_POWER)) && d.pw_latched != 0)
      ESP_LOGW(TAG, "Receiver %u on GPIO%u power events: %s", i + 1, this->pin_, telemetry_power_events_text(d).c_str());
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
      ESP_LOGV(TAG, "%s \xC2\xB7 Receiver %u: its lifetime counters synced from the hub", this->tel_name_, i + 1);
    }
    // its current calibration: a new test (or its end) into the line's records, kept on flash (rare: after a test)
    if (!(d.hw_caps & RX_CAP_PIXEL) && (u & (1u << TP_CALIBRATION)) && this->cal_log_.observe(i, d, this->poller_.uid16(i), millis(), this->tel_unix_())) {
      ESP_LOGI(TAG, "%s \xC2\xB7 Receiver %u: current calibration %s (%s model)", this->tel_name_, i + 1,
               this->cal_log_.result(i, d).c_str(), calibration_model_text(d.cal_model));
      this->cal_pref_.save(reinterpret_cast<const uint8_t *>(this->cal_log_.records()),
                           this->cal_log_.size() * sizeof(CalRecord));
    }
    this->rx_listeners_.call(i, u);
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
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) {
    ESP_LOGW(TAG, "GPIO%u: a digital port has no boards in WS mode to switch", this->pin_);
    return;
  }
#endif
  const uint8_t p = boot_protocol == 3 ? 3 : 0;
  ESP_LOGI(TAG, "Switching boards to Ultra: WS config frames, then the start-up preamble (boot protocol %s)",
           p == 3 ? "ultra" : "auto");
  this->switch_frame_ = 0;
  this->set_board_config(-1, ULTRA_CFG_BOOT_PROTOCOL, p);
  this->resync();
}

// The log of light changes (ultra_change_log.h; wiki: Ultra-Line-Options, light changes in the log), at its levels:
static void change_log_sink(void * /*ctx*/, uint8_t level, const char *text) {
  if (level == CHANGE_LOG_WARN)
    ESP_LOGW(TAG, "%s", text);
  else if (level == CHANGE_LOG_INFO)
    ESP_LOGI(TAG, "%s", text);
  else
    ESP_LOGD(TAG, "%s", text);
}

void UltraHub::note_change(uint16_t board, const char *platform, uint8_t mask, const uint16_t *level,
                           uint32_t transition_ms, int64_t start_us) {
  if (this->started_)
    this->change_log_.change(millis(), this->frames_, board, platform, mask, level, transition_ms,
                             static_cast<uint32_t>((start_us + 999) / 1000), this->sched_.power_limited(board));
}

void UltraHub::note_stream(uint16_t board, uint8_t mask) {
  if (this->started_)
    this->change_log_.stream(millis(), this->frames_, board, mask);
}

void UltraHub::note_kept(uint16_t board) {
  if (this->started_)
    this->change_log_.kept(board);
}

// Why the line's changes can't go out now, the first that applies (ChangeHold's order)
uint8_t UltraHub::change_hold_() {
#ifdef USE_ULTRA_LINE_UPDATE
  if (this->update_running())
    return HOLD_UPDATE;
#endif
  if (this->port_off())
    return HOLD_PORT_OFF;
  if (this->calibration_running())
    return HOLD_CALIBRATION;
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_ && this->digital_blocked())
    return HOLD_RECEIVER_SILENT;
#endif
  if (this->port_standby())
    return HOLD_PORT_WAKING;
  if (!this->boards_powered_())
    return HOLD_POWER_SUPPLY;
  if (!this->receivers_ready_())
    return HOLD_RECEIVERS;
  if (this->line_rate_hold_)
    return HOLD_LINE_SPEED;
  return HOLD_NONE;
}

// A board's commands: all out (their first copies), still queued, or no descriptor (its slots stay empty)
uint8_t UltraHub::change_board_state_(uint16_t board) const {
  if (this->sched_.descriptor_missing(board))
    return HOLD_NO_DESCRIPTOR;
  return this->sched_.waiting(board) > 0 ? ChangeLog::BOARD_QUEUED : ChangeLog::BOARD_SENT;
}

// Every loop: the hub's state is asked only while a change is watched; nothing here runs per frame
void UltraHub::service_change_log_() {
  this->change_log_.service(
      millis(), this->frames_, this->last_frame_ms_, this->has_sent_, LINE_RATE_TEXT[this->line_profile()],
      [this]() { return this->change_hold_(); }, [this](uint16_t board) { return this->change_board_state_(board); });
}

// Something to send: frames back to back until the queues are empty (and BUSY_HOLD_MS after the last).
void UltraHub::wake_() {
  this->last_busy_ms_ = millis();
  this->loop_fast_();
}

// The loop at full speed until the next idle check, without the busy hold: a response window (its read is timed),
// so the controller isn't held fast for 250 ms after every poll.
void UltraHub::loop_fast_() {
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
  if (this->port_off()) {
    ESP_LOGW(TAG, "Current calibration on GPIO%u skipped: its Diff-Power port's power mode is Off", this->pin_);
    return;
  }
  this->cal_req_boards_ = boards;
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) return;  // UR_DIGITAL_M requires its explicit receiver button and capability discovery
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
  this->cal_req_boards_.clear();
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    if (!boards.empty() && std::find(boards.begin(), boards.end(), b) == boards.end()) continue;
    if (b >= this->poller_.receivers()) continue;
    const auto &d = this->poller_.data(b);
    if (d.have_identity && !(d.hw_caps & RX_CAP_PIXEL) && (d.features & ULTRA_FEAT_CALIBRATION)) this->cal_req_boards_.push_back(b);
  }
  if (this->cal_req_boards_.empty()) return;
#endif
  this->cal_req_duty_ = duty_pct;
  this->cal_pending_ = true;  // starts in service_calibration_() once the boards have power and Ultra
  this->cal_started_ = false;
  this->cal_wait_ms_ = millis();
}

void UltraHub::cancel_calibration() {
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->pixel_cal_job_.active()) {
    if (this->pixel_cal_job_.waiting) {
      const uint16_t b = this->pixel_cal_job_.board;
      this->pixel_cal_job_.board = -1;
      this->poller_.pixel_calibration_result(b, ULTRA_UCE_CANCELLED);
      this->cal_power_hold_ = false;
      this->rw_publish_();
    } else {
      this->pixel_cal_job_.cancel();
    }
    this->wake_();
  }
#endif
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

// The port's wake timeline: a frame that carries light went out (ultra_frame_lit / ultra_pixels_lit; checked only
// while the port waits for one)
void UltraHub::note_wake_light_() {
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr && this->port_->wake_light_pending())
    this->port_->wake_light();
#endif
}

bool UltraHub::any_lit_(uint32_t now) const {
  for (auto *l : this->lights_) {
    if (l->is_lit(now))
      return true;
  }
  for (auto *l : this->pixel_lights_) {
    if (l->pixel_lit(now))
      return true;
  }
  return false;
}

void UltraHub::service_calibration_() {
  const uint32_t now = millis();
#ifdef USE_ULTRA_LINE_UPDATE
  if (this->update_running()) {
    if (this->calibration_running())
      this->cancel_calibration();  // the port job owns the line; the release goes out after it
    this->cal_power_hold_ = false;
    return;
  }
#endif
  if (this->port_off() && this->calibration_running()) {
    ESP_LOGW(TAG, "Current calibration on GPIO%u: its Diff-Power port's power mode is now Off", this->pin_);
    this->cancel_calibration();   // (the port powers off; nothing would reach the boards)
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->pixel_cal_job_.active()) {
    const uint16_t b = this->pixel_cal_job_.board;
    if (this->pixel_cal_job_.waiting) {
      const auto &d = this->poller_.data(b);
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
      const bool stream_port = this->digital_;
#else
      const bool stream_port = false;
#endif
      if (this->started_ && !this->preamble_ && !this->port_standby() && !this->tx_prepared_ &&
          !this->sched_.pending() && this->poller_.answering(b) && TelemetryPoller::digital_calibration(d) &&
          (stream_port || (d.hw_caps & RX_CAP_CHAIN))) {
        // Refresh the comparison baseline after wake/discovery; a retained old DONE cannot finish this job.
        this->pixel_cal_job_.board = -1;
        this->pixel_cal_job_.start(b, now, this->poller_.pixel_calibration(b), d.pixel_cal_wait_ms);
        this->poller_.calibration_quiet(b, now);
        this->wake_();
      }
    }
    if (this->pixel_cal_job_.tick(now, this->poller_.pixel_calibration(b))) {
      this->poller_.calibration_quiet(-1, now);
      if (this->pixel_cal_job_.error == ULTRA_UCE_TIMEOUT) this->poller_.invalidate_pixel_calibration(b);
      this->poller_.pixel_calibration_result(b, this->pixel_cal_job_.error == ULTRA_UCE_TIMEOUT ? ULTRA_UCE_TIMEOUT : 0);
      this->poller_.touch(b, TU_HUB);
      this->rw_publish_();
      this->wake_();
    }
    this->cal_power_hold_ = this->pixel_cal_job_.active();
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
                 static_cast<unsigned>(this->cal_seq_.board() + 1), static_cast<unsigned>(this->cal_seq_.index() + 1),
                 static_cast<unsigned>(this->cal_seq_.count()));
        this->wake_();
        break;
      case CalSequencer::EV_DONE: {
        // the commands are one-way: the receivers' CALIBRATION pages say how each test ended (read now)
#ifdef USE_ULTRA_LINE_TELEMETRY
        for (uint16_t b : this->cal_seq_.jobs())
          this->poller_.read_now(b, TP_CALIBRATION, now);
        const bool results = this->has_telemetry();
#else
        const bool results = false;
#endif
        ESP_LOGI(TAG, "Current calibration on GPIO%u: commands sent to %u board(s) in %.1f s (%s); the pin fades back",
                 this->pin_, static_cast<unsigned>(this->cal_seq_.count()), this->cal_seq_.elapsed_ms(now) / 1000.0f,
                 results ? "the receivers' results: read next" : "not confirmed: the boards keep their results");
        this->wake_();
        break;
      }
      default:
        break;
    }
  }
  this->cal_power_hold_ = this->calibration_running();
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->cal_power_hold_ != this->cal_was_) {   // a run asked for, or over: its receivers' Status (Calibrating)
    this->cal_was_ = this->cal_power_hold_;
    for (uint16_t i = 0; i < this->poller_.receivers(); i++)
      this->poller_.touch(i, TU_HUB);
  }
#endif
}

void UltraHub::calibrate_receiver(uint16_t board) {
  if (board >= this->num_boards_)
    return;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (board < this->poller_.receivers()) {
    const ReceiverData &d = this->poller_.data(board);
    if (d.have_identity && (d.hw_caps & RX_CAP_PIXEL) != 0) {
      if (!TelemetryPoller::digital_calibration(d) || !d.pixel_cal_wait_ms || this->calibration_running() || this->port_off())
        return;
#ifdef USE_ULTRA_LINE_UPDATE
      if (this->update_running()) return;
#endif
      const uint32_t now = millis();
      this->pixel_cal_job_.start(board, now, this->poller_.pixel_calibration(board), d.pixel_cal_wait_ms, true);
      this->poller_.refresh_pixel_identity(board, now);
      this->poller_.pixel_calibration_result(board, 0);
      this->poller_.touch(board, TU_HUB);
      this->cal_power_hold_ = true;
      this->rw_publish_();
      this->wake_();
      return;
    }
    if (d.have_identity && !(d.features & ULTRA_FEAT_CALIBRATION)) {
      ESP_LOGW(TAG, "Current calibration of receiver %u on GPIO%u skipped: it has no current calibration", board + 1,
               this->pin_);
      return;
    }
  }
#endif
  bool joined = false;
  if (this->cal_pending_) {   // a run asked for, waiting for the boards' power: one more board in it
    joined = this->cal_req_duty_ == 0;
    if (joined && !this->cal_req_boards_.empty() &&
        std::find(this->cal_req_boards_.begin(), this->cal_req_boards_.end(), board) == this->cal_req_boards_.end())
      this->cal_req_boards_.push_back(board);
  } else if (this->cal_seq_.running()) {
    joined = this->cal_seq_.add(board);
  } else {
    this->calibrate_boards({board}, 0);
    return;
  }
  if (!joined) {
    ESP_LOGW(TAG, "Current calibration of receiver %u on GPIO%u: the run on its pin can't take it now, request ignored",
             board + 1, this->pin_);
    return;
  }
  ESP_LOGI(TAG, "Current calibration on GPIO%u: receiver %u in the run", this->pin_, board + 1);
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (board < this->poller_.receivers())
    this->poller_.touch(board, TU_HUB);
#endif
}

void UltraHub::service_power_() {
#ifdef USE_POWER_SUPPLY
  if (this->psu_ == nullptr)
    return;
  const uint32_t now = millis();
  bool lit = this->cal_power_hold_;
  for (auto *l : this->lights_)
    lit |= l->is_lit(now);
  for (auto *l : this->pixel_lights_)
    lit |= l->pixel_lit(now);
#ifdef USE_ULTRA_LINE_TELEMETRY
  lit |= this->comm_.hold_power();   // (a commissioning: the supply on while it runs)
#endif
#ifdef USE_ULTRA_LINE_UPDATE
  lit |= this->upd_power_hold_;
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
  this->line_rate_.reset(this->num_boards_);
  uint8_t table = 255;
  const uint8_t why = ultra_port_line_profile(this->port_type_, this->num_boards_, &table);
  this->line_choice_ = choose_line_profile(why, table, this->line_initial_);
  this->line_rate_.request(this->line_choice_.profile);
  const std::string choice = line_choice_text(this->port_type_, this->num_boards_, this->line_choice_);
  if (line_choice_refused(this->line_choice_))
    ESP_LOGE(TAG, "GPIO%u: line stays at 800 kHz: %s", this->pin_, choice.c_str());
  else
    ESP_LOGI(TAG, "GPIO%u: line starts at %s: %s", this->pin_, LINE_RATE_TEXT[this->line_choice_.profile],
             choice.c_str());
  this->publish_line_speed_();
  this->sched_.init(this->num_boards_, esp_random(), this->repeats_);
  this->change_log_.init(this->num_boards_, this->pin_, change_log_sink, this);
  uint16_t returns = 0;   // the boards that answer (the first ones: the receivers of the telemetry)
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_uart_ != nullptr)
    returns = this->poller_.receivers();
#endif
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_)
    returns = this->num_boards_;   // (a digital port's receiver answers its polls on the line)
#endif
  if (returns < this->num_boards_)
    this->change_log_.set_no_return_from(returns);
  this->sched_.require_descriptors(returns);
  this->started_ = true;
  for (const auto &c : this->early_config_) {
    for (uint16_t b = 0; b < this->num_boards_; b++) {
      if (c.board < 0 || c.board == b) {
        if (c.target == ULTRA_TARGET_BOARD) this->sched_.set_config(b, c.id, c.value);
        else this->sched_.set_channel_config(b, c.target, c.id, c.value);
      }
    }
  }
  this->early_config_.clear();
  this->early_config_.shrink_to_fit();
  this->rx_settings_setup_();   // (the receivers' settings from Home Assistant over the YAML's)
  if (!this->every_board_descriptor_.empty()) {   // (first: a board's own entry below replaces it)
    for (uint16_t b = 0; b < this->sched_.num_boards(); b++)
      if (!this->sched_.descriptor(b, this->every_board_descriptor_)) ESP_LOGE(TAG, "Invalid descriptor for board %u", b + 1);
  }
  for (const auto &entry : this->descriptor_inventory_)
    if (!this->sched_.descriptor(entry.board, entry.bytes)) ESP_LOGE(TAG, "Invalid descriptor for board %u", entry.board + 1);
  this->sched_.flush_config();
#ifdef USE_API_CUSTOM_SERVICES
  static bool service_added = false;   // (one action for every line)
  if (!service_added && api::global_api_server != nullptr) {
    service_added = true;
    api::global_api_server->register_user_service(new ReceiverSettingService());   // NOLINT (lives forever)
  }
#endif
  // light indices without a light in the YAML: cleared on the boards (an earlier configuration may have used them)
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    if (std::find(this->pixel_boards_.begin(), this->pixel_boards_.end(), b) != this->pixel_boards_.end())
      continue;   // a Diff-Power-UR-Digital-M: no light indices (its segments are PIXEL_FX, not lights)
    this->sched_.clear_unused_lights(b, b < this->light_counts_.size() ? this->light_counts_[b] : 0);
  }
  for (uint8_t n : this->light_counts_)
    this->max_lights_ = std::max(this->max_lights_, n);
  this->light_counts_.clear();
  this->light_counts_.shrink_to_fit();
  hubs_.push_back(this);

  RAMAllocator<uint8_t> alloc(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  size_t buf_bytes = std::max<size_t>(this->sched_.frame_bytes(), ultra_switch_frame_bytes(this->num_boards_));
#ifdef USE_ULTRA_LINE_UPDATE
  buf_bytes = std::max<size_t>(buf_bytes, UPD_FRAME_BYTES);   // a port job's update frames
#endif
  this->tx_buf_ = alloc.allocate(buf_bytes);
  this->tx_buf_bytes_ = buf_bytes;
  if (this->tx_buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate the frame buffer (%u B)", static_cast<unsigned>(buf_bytes));
    this->mark_failed();
    return;
  }

#ifdef USE_ULTRA_PARLIO
  if (this->lane_ != nullptr) {
    // a PARLIO lane: the bus renders the frames; the timing (set_timing) is already on its sample grid
#ifdef USE_ULTRA_LINE_TELEMETRY
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
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_uart_ != nullptr) {
    this->keeper_.attach(&ultra_ledger(), this->tel_key_, this->poller_.receivers());
    // a Diff-Power-UR-Digital-M on a normal Ultra line (chain mode) is asked for its line too (page 0x11 part 3); on a
    // digital port it is not (a chain board does not answer in the digital window)
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
    this->poller_.set_line_pages(!this->digital_);
#else
    this->poller_.set_line_pages(true);
#endif
    if (this->comm_on_) {   // the commissioning: its last Finalize's result from flash
      this->comm_.configure(this->rate_ms_, this->comm_min_v_);
      this->comm_layer_.hub = this;
      this->comm_pref_ = global_preferences->make_preference<CommissionRecord>(this->tel_key_ ^ 0x434F4D31u);
      CommissionRecord rec;
      if (this->comm_pref_.load(&rec))
        this->comm_.set_record(rec);
    }
    // the receivers' current calibration records (one blob per line: its receivers' count is part of the key)
    const size_t n = this->poller_.receivers();
    this->cal_log_.init(n);
    this->cal_pref_ = global_preferences->make_preference(n * sizeof(CalRecord), this->tel_key_ ^ 0x43414C00u ^ n, true);
    std::vector<CalRecord> recs(n);
    if (this->cal_pref_.load(reinterpret_cast<uint8_t *>(recs.data()), n * sizeof(CalRecord)))
      this->cal_log_.load(recs.data(), n);
    ledger_receivers() += this->poller_.receivers() + 1u;   // (its receivers' positions, its port's energy)
    // the POLL slots: the receivers only (the boards behind them take normal slots); the answers' buffer at once
    this->rw_slots_.assign(static_cast<size_t>(ULTRA_SLOT_BYTES) * this->poller_.receivers(), 0);
    this->rw_rx_.reserve(1024);
    this->rw_extra_us_ = this->poller_.max_window_us() + ULTRA_RW_END_US + this->line_gap_us();
    if (this->rw_dir_pin_ != nullptr) {
      this->rw_dir_pin_->setup();         // the hub drives the line
      this->rw_dir_pin_->digital_write(true);
      this->rw_dir_isr_ = this->rw_dir_pin_->to_isr();
#ifdef USE_ULTRA_PARLIO
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
  this->refresh_line_gap_();
  this->preamble_ = preamble_frames(this->num_boards_);
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_ && !this->digital_setup_())
    return;
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
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  this->rmt_res_ = res;
  if (this->digital_) {   // the strip chip's bit words (the pixel frames), as the RMT's
    const uint32_t p0h = ns_to_ticks(this->px_t0h_, res), p0l = ns_to_ticks(this->px_t0l_, res);
    const uint32_t p1h = ns_to_ticks(this->px_t1h_, res), p1l = ns_to_ticks(this->px_t1l_, res);
    if (this->px_bytes_ != 0 && (p0h == 0 || p0l == 0 || p1h == 0 || p1l == 0 || p0h > RMT_DURATION_MAX ||
                                 p0l > RMT_DURATION_MAX || p1h > RMT_DURATION_MAX || p1l > RMT_DURATION_MAX)) {
      ESP_LOGE(TAG, "The strip's timing does not fit an RMT symbol at %" PRIu32 " Hz", res);
      this->mark_failed();
      return false;
    }
    this->px_word0_ = make_symbol(p0h, 1, p0l, 0).val;
    this->px_word1_ = make_symbol(p1h, 1, p1l, 0).val;
  }
#endif
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
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) {
    enc.callback = dp_encoder_callback;
    enc.arg = &this->dp_enc_;
  }
#endif
  enc.min_chunk_size = 1;
  if (rmt_new_simple_encoder(&enc, &this->encoder_) != ESP_OK) {
    ESP_LOGE(TAG, "RMT encoder creation failed");
    this->mark_failed();
    return false;
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  this->reset_symbol_ = this->enc_.symbols.reset;
  this->rw_reset_symbol_ = this->reset_symbol_;
  if (this->rw_uart_ != nullptr && this->rw_dir_pin_ != nullptr) {
    // an RW frame ends with a LOW of RW_RELEASE_HOLD_NS (80 us, two halves of one symbol), then the RMT's interrupt lets go of the line: the window
    // is its reset (a digital port: the tail LOW of the poll's transmission, ultra_dp_next)
    const uint32_t half = ns_to_ticks(RW_RELEASE_HOLD_NS / 2, res);
    this->rw_reset_symbol_ = make_symbol(half, 0, half, 0);
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

#ifdef USE_ULTRA_LINE_TELEMETRY
// The ESP32's one lifetime ledger (every line's receivers: a board moved between lines keeps its entry). Two stored
// copies alternate: the newer valid one loads (a cut while one is written keeps the other). Each has its size stored
// beside it (written after it): a copy stored with more or fewer receivers loads too. Stored at most every 15 min
// while it changes by a wire unit, and at a reboot (not at every planned power-off: a port waking 100 times a day
// would write it 100 times; a power loss of the ESP32 loses at most those minutes, counted less, never more); its
// stored form is built only for a save (no copy of it kept in RAM).
namespace {
const char *const LEDGER_KEYS[2] = {"ultra_ledger_a", "ultra_ledger_b"};
const char *const LEDGER_SIZE_KEYS[2] = {"ultra_ledger_a_size", "ultra_ledger_b_size"};
ESPPreferenceObject g_ledger_pref[2], g_ledger_size[2];
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
    // (runtime room in blocks of 32, independent of the compact blob; never fewer than the stored copy uses)
    const size_t n = ledger_receivers();
    const size_t pos = std::max((n + 31) / 32 * 32, (used_p + 31) / 32 * 32);
    const size_t room = std::max((n + std::max<size_t>(4, n / 2) + 31) / 32 * 32, (used_e + 31) / 32 * 32);
    l.init(room, pos);
    if (any)
      l.load(best.data(), best.size(), &newest);
    for (int k = 0; k < 2; k++)
      g_ledger_pref[k] = global_preferences->make_preference(l.blob_size(), fnv1_hash(LEDGER_KEYS[k]), true);
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
  if (now - g_ledger_saved_ms >= LEDGER_SAVE_MS && l.dirty())
    ledger_save_(false);
}

void UltraHub::ledger_save_(bool now) {
  LifetimeLedger &l = ultra_ledger();
  if (!g_ledger_loaded || (!l.dirty() && !now))
    return;
  if (l.dirty()) {
    // the next generation into the copy not in use: its blob on the flash first, its size only then (a size never
    // names a blob that isn't there), the copy in use whole throughout. Clean, and that generation the one in use,
    // once both are there; a failure stays dirty and the next save, periodic or forced, writes the same copy again
    // (so does one without the memory for its stored form)
    const size_t n = l.blob_size();
    std::unique_ptr<uint8_t[]> blob(new (std::nothrow) uint8_t[n]);
    if (!blob) {
      ESP_LOGW(TAG, "Lifetime ledger: no memory to store it; again at the next save");
      g_ledger_saved_ms = millis();
      return;
    }
    const uint32_t gen = g_ledger_generation + 1;
    l.save(blob.get(), gen);
    const uint32_t size = static_cast<uint32_t>(n);
    // The ESP32 flash preference copies the supplied length on save (its key is independent of length), so the
    // same preference accepts a compact blob growing/shrinking between saves. Release ours before sync's read-back.
    bool ok = g_ledger_pref[gen & 1].save(blob.get(), n);
    blob.reset();
    ok = ok && global_preferences->sync();
    ok = ok && g_ledger_size[gen & 1].save(reinterpret_cast<const uint8_t *>(&size), sizeof(size)) &&
         global_preferences->sync();
    if (ok) {
      g_ledger_generation = gen;
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
  const bool tick = this->keeper_.lit_due(now);
  this->lit_rx_.resize(this->poller_.receivers(), 0);
  // Keep last loop's state in bit 1 while reducing this loop's lights into bit 0. Transition detection stays at
  // loop cadence (including short pulses and fade deadlines); unchanged clock accounting runs every 50 ms.
  for (auto &lit : this->lit_rx_)
    lit = (lit & 1u) << 1;
  if (this->boards_powered_()) {
    for (auto *l : this->lights_) {
      if (l->get_board() < this->lit_rx_.size() && l->is_lit(now))
        this->lit_rx_[l->get_board()] |= 1;
    }
    for (auto *l : this->pixel_lights_) {
      if (l->pixel_board() < this->lit_rx_.size() && l->pixel_lit(now))
        this->lit_rx_[l->pixel_board()] |= 1;
    }
  }
  for (uint16_t i = 0; i < this->lit_rx_.size(); i++) {
    const uint8_t lit = this->lit_rx_[i];
    if (tick || (lit & 1u) != (lit >> 1))
      this->keeper_.lit_tick(i, (lit & 1u) != 0, now);
  }
}
#endif

void UltraHub::on_shutdown() {
#ifdef USE_ULTRA_LINE_TELEMETRY
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
#ifdef USE_ULTRA_LINE_TELEMETRY
// Receivers without standby support on the port: once PowerOffCap::CAP planned power-offs fall within 24 h, the port
// stays powered until the oldest is a day old (said once each way).
bool UltraHub::standby_hold_(uint32_t now) {
  const bool hold = this->no_standby_ != 0 && this->off_cap_.hold(now);
  if (hold != this->standby_held_) {
    this->standby_held_ = hold;
    if (hold) {
      ESP_LOGW(TAG, "GPIO%u: %u planned power-offs in 24 h with receivers without standby support on the port: it "
               "stays powered until the oldest is a day old (update the receivers)", this->pin_, PowerOffCap::CAP);
    } else {
      ESP_LOGI(TAG, "GPIO%u: the port may power off again", this->pin_);
    }
  }
  return hold;
}
#endif

// The port powers this line while a light here is lit (or within its turn-off period) and while a calibration or an
// update holds it; once powered, everything again (the boards booted dark); before it drops, the notice.
void UltraHub::attach_port_() {
  this->port_attached_ = true;
#ifdef USE_ULTRA_LINE_TELEMETRY
  this->port_->wake_expect_reply(this->rw_uart_ != nullptr && this->poller_.receivers() > 0);   // (the wake timeline waits for the first reply)
#endif
  for (auto *l : this->lights_)
    this->port_->add_load(l->get_standby_after(), [l](uint32_t now) { return l->is_lit(now); }, l->get_state());
  for (auto *l : this->pixel_lights_)
    this->port_->add_load(l->pixel_standby_after(), [l](uint32_t now) { return l->pixel_lit(now); }, l->pixel_state());
  this->port_->add_load(1, [this](uint32_t now) {
#ifdef USE_ULTRA_LINE_TELEMETRY
    if (this->standby_hold_(now) || this->comm_.hold_power())
      return true;
#endif
#ifdef USE_ULTRA_LINE_UPDATE
    return this->cal_power_hold_ || this->upd_power_hold_;
#else
    return this->cal_power_hold_;
#endif
  });
  this->port_->add_on_ready_callback([this]() {
    ESP_LOGD(TAG, "GPIO%u: the port is powered, replaying the boards' state", this->pin_);
    this->sched_.cancel_power_off();   // (a notice copy not sent before the power-off: stale now)
    this->notice_id_ = 0;
#ifdef USE_ULTRA_LINE_TELEMETRY
    this->rx_note_.resize(this->poller_.receivers(), 0);
    for (auto &n : this->rx_note_)
      n |= RX_WOKE;
    this->woke_ms_ = millis();
#endif
    this->resync(true);
  });
  this->port_->add_on_notice_callback([this](uint8_t copy) { this->send_power_off_notice(copy); });
  this->port_->add_on_pump_callback([this]() { this->pump_notice_(); });
#ifdef USE_ULTRA_LINE_TELEMETRY
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
#ifdef USE_ULTRA_LINE_TELEMETRY
    if (this->no_standby_ != 0)
      this->off_cap_.note(millis());   // (receivers without standby support: counted against the cap)
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
#ifdef USE_ULTRA_LINE_UPDATE
  if (this->update_running())
    return;
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
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
#ifdef USE_ULTRA_PARLIO
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

#ifdef USE_ULTRA_LINE_TELEMETRY
UltraHub *UltraHub::comm_active_ = nullptr;

// The line's clock (time_id, else the only time component): the commissioning's date, Last calibrated
uint32_t UltraHub::tel_unix_() const {
#ifdef USE_TIME
  if (this->tel_clock_ != nullptr) {
    const ESPTime t = this->tel_clock_->now();
    if (t.is_valid())
      return static_cast<uint32_t>(t.timestamp);
  }
#endif
  return 0;
}

// ------------------------------------------------------------------------------------------- the commissioning
void UltraHub::commission(bool finalize) {
  if (!this->comm_on_)
    return;
  if (!this->comm_.request(finalize ? Commissioning::FINALIZE : Commissioning::COMMISSION)) {
    ESP_LOGW(TAG, "Telemetry on GPIO%u: a commissioning already runs or waits", this->pin_);
    return;
  }
  ESP_LOGI(TAG, "Telemetry on GPIO%u: %s asked for", this->pin_, finalize ? "Finalize" : "Commission");
  this->comm_cb_.call();
}

void UltraHub::commission_all(bool finalize) {
  for (UltraHub *hub : hubs_)
    hub->commission(finalize);
}

// One run on the controller at a time (two load tests at once could overload the sender's supply); the port held
// powered, the test level on the line's lights, the calibration, the record (wiki: Commissioning)
void UltraHub::service_commission_() {
  const uint32_t now = millis();
  if (this->comm_.queued() && (comm_active_ == nullptr || comm_active_ == this)) {
    comm_active_ = this;
    ESP_LOGI(TAG, "Telemetry on GPIO%u: the commissioning starts", this->pin_);
    this->comm_.start(now);
  }
  CommissionPort cp;
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr) {
    cp.present = true;
    cp.detect = this->port_->has_detect();
    cp.off = this->port_->is_off();
    cp.board = this->port_->core().board();
    cp.fault = this->port_->state() == diffpower::PS_FAULT;
  }
#endif
  cp.ready = this->receivers_ready_();
  const uint32_t unix_s = this->tel_unix_();
#ifdef USE_ULTRA_LINE_UPDATE
  const bool updating = this->update_running();
#else
  const bool updating = false;
#endif
  const uint8_t act = this->comm_.poll(now, unix_s, cp, this->poller_, this->port_monitor_,
                                       this->calibration_running(), updating);
  if (act & Commissioning::ACT_LEVEL)
    this->commission_level_(this->comm_.level());
  if (act & Commissioning::ACT_CALIBRATE)
    this->calibrate_boards({}, 0);
  if (act & Commissioning::ACT_STORE) {
    this->comm_pref_.save(&this->comm_.record());
    global_preferences->sync();   // (a user's action, rare: kept at once)
  }
  if (act & Commissioning::ACT_DONE) {
    ESP_LOGI(TAG, "Telemetry on GPIO%u: %s. %s", this->pin_, this->comm_.status().c_str(),
             this->comm_.report().c_str());
    if (comm_active_ == this)
      comm_active_ = nullptr;
  }
  if (act & Commissioning::ACT_TEXT)
    this->comm_cb_.call();
}

// The load test: the commissioning is a layer over every light of the line (its own state stays underneath, a strip
// shown meanwhile goes on top); level 0: the layer gone, each light back to its own state at once
void UltraHub::commission_level_(float level) {
  for (UltraLight *l : this->lights_) {
    if (level > 0.0f) {
      if (!l->layer_top(&this->comm_layer_))
        l->layer_begin(&this->comm_layer_);
      l->send_test_level(level);
    } else {
      l->layer_end(&this->comm_layer_, 0);
    }
  }
}

void UltraHub::CommissionLayer::layer_shown(UltraLight *light) {
  light->send_test_level(this->hub->comm_.level());
}
#endif

uint32_t UltraHub::effect_interval(uint16_t rate) const {
  uint32_t demand=0;
  for (const auto *light : this->lights_) demand += light->effect_rate();
  return effect_interval_for_demand(demand,rate ? rate : this->effect_rate_,this->num_boards_,this->frame_us_);
}

void UltraHub::refresh_line_gap_() {
  uint32_t gap=500;
  for(uint16_t b=0;b<this->num_boards_;b++) {
    const auto *d=this->sched_.board_descriptor(b);if(d==nullptr) continue;
    ultra_desc_link_t link{};
    for(uint8_t ingress=0;ingress<16;ingress++)
      if(!ultra_descriptor_link(d,ingress,&link) && link.phy==ULTRA_PHY_ADAPTIVE)
        gap=std::max<uint32_t>(gap,link.gap_us);
  }
  this->descriptor_gap_us_=gap;
  const uint32_t bit=std::max(this->bit0_high_ns_+this->bit0_low_ns_,this->bit1_high_ns_+this->bit1_low_ns_);
  this->frame_us_=(uint64_t(this->sched_.frame_bytes())*8*bit+999)/1000+this->line_gap_us();
}

const char *UltraHub::line_profile_refusal_(uint8_t profile, int16_t *board) const {
  *board = -1;
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_ && profile != 0) return "a digital port keeps the strip's waveform";
#endif
  if (profile == 0) return nullptr;
  if (profile > 3) return "no such profile";
  // leaving 800 kHz: a complete all-Ultra inventory whose applications and bootloaders all support the profile
  if (!this->line_inventory_complete_) return "line_rate inventory_complete is not set";
  const bool d2a = this->port_type_ == PT_D2A_PIN;
  if (!d2a && this->num_boards_ > PORT_DIFF_POWER_MAX_BOARDS) return "more than 8 boards";
  if (!(this->line_boot_profiles_ & (1u << profile))) return "line_rate bootloader_profile_mask lacks it";
  uint8_t common = this->line_boot_profiles_;
  for (uint16_t b = 0; b < this->num_boards_; b++) {
    *board = static_cast<int16_t>(b);
    const auto *d = this->sched_.board_descriptor(b);
    if (d == nullptr) return "has no descriptor";
    ultra_desc_link_t link{};
    bool pulse = false;
    for (uint8_t ingress = 0; ingress < 16; ingress++) {
      if (!ultra_descriptor_link(d, ingress, &link) && link.phy == ULTRA_PHY_ADAPTIVE) {
        common &= link.profiles; pulse = true; break;
      }
    }
    if (!pulse) return "has no Ultra input";
    if (!(link.profiles & (1u << profile))) return "firmware lacks it";
  }
  *board = -1;
  // the path: a Dig2Analog-Plus-5ch pin's forwarding gates (its link model, ideal wiring), else the Diff-Power path the
  // YAML qualified
  if (d2a) {
    const uint8_t n = static_cast<uint8_t>(std::min<uint16_t>(this->num_boards_, 255));
    if (!ultra_line_allowed(profile, true, true, common, ultra_d2a_path_ns(n, 0)) || !ultra_d2a_admits(profile, n, 0))
      return "outside the pin's timing budget";
    return nullptr;
  }
  return ultra_line_allowed(profile, true, true, common, this->line_path_ns_) ? nullptr
                                                                              : "path_distortion_ns over its budget";
}

void UltraHub::refuse_line_profile_(uint8_t profile, const UltraHub *who, const char *what, int16_t board) {
  char text[128];
  size_t n = snprintf(text, sizeof text, "refused %s: ", LINE_RATE_TEXT[profile & 3]);
  if (who != this && n < sizeof text) n += snprintf(text + n, sizeof text - n, "lane GPIO%u ", who->pin_);
  if (board >= 0 && n < sizeof text) n += snprintf(text + n, sizeof text - n, "board %d ", board + 1);
  if (n < sizeof text) snprintf(text + n, sizeof text - n, "%s", what);
  if (this->line_refusal_ == text) return;
  this->line_refusal_ = text;
  // a Dig2Analog-Plus-5ch pin that can't leave 800 kHz runs its chain outside the speed table's guarantee
  if (this->port_type_ == PT_D2A_PIN)
    ESP_LOGE(TAG, "GPIO%u: line stays at %s, %s", this->pin_, LINE_RATE_TEXT[this->line_profile()], text);
  else
    ESP_LOGW(TAG, "GPIO%u: line stays at %s, %s", this->pin_, LINE_RATE_TEXT[this->line_profile()], text);
  this->publish_line_speed_();
}

void UltraHub::publish_line_speed_() {
#ifdef USE_TEXT_SENSOR
  if (this->line_speed_sensor_ == nullptr) return;
  const std::string text = this->line_profile_text();
  if (!this->line_speed_sensor_->has_state() || this->line_speed_sensor_->state != text)
    this->line_speed_sensor_->publish_state(text);
#endif
}

bool UltraHub::line_profile_idle_() {
  if (this->is_failed() || !this->started_ || this->calibration_running()) return false;
#ifdef USE_ULTRA_LINE_UPDATE
  if (this->update_running()) return false;
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_open_) return false;
  if (this->tx_prepared_ && this->tx_rw_) {
    this->tx_prepared_ = this->tx_rw_ = false;
    this->rw_drop_turn_();
  }
#endif
  return this->line_free_() && this->line_out_();
}

void UltraHub::apply_line_profile_(uint8_t profile) {
  ultra_line_rmt_t t{};
  if (ultra_line_rmt(profile, 40000000, &t)) return;
  this->bit0_high_ns_ = t.zero_ticks * 25u;
  this->bit0_low_ns_ = (t.bit_ticks - t.zero_ticks) * 25u;
  this->bit1_high_ns_ = t.one_ticks * 25u;
  this->bit1_low_ns_ = (t.bit_ticks - t.one_ticks) * 25u;
#ifdef USE_ULTRA_PARLIO
  if (this->lane_ != nullptr) this->lane_lead_us_ = this->lane_->lead_us();
  else
#endif
  {
    this->enc_.symbols.bit0 = make_symbol(t.zero_ticks, 1, t.bit_ticks - t.zero_ticks, 0);
    this->enc_.symbols.bit1 = make_symbol(t.one_ticks, 1, t.bit_ticks - t.one_ticks, 0);
    this->enc_.watch.drain_us = static_cast<int64_t>(this->rmt_symbols_) * t.bit_ticks / 40;
  }
  this->sched_.set_bit_timing(this->bit0_high_ns_, this->bit0_low_ns_, this->bit1_high_ns_, this->bit1_low_ns_);
  this->frame_us_ = (static_cast<uint64_t>(this->sched_.frame_bytes()) * 8 * t.bit_ticks + 39) / 40 + this->line_gap_us();
  this->switch_frame_us_ = (static_cast<uint64_t>(ultra_switch_frame_bytes(this->num_boards_)) * 8 * t.bit_ticks + 39) / 40 + this->line_gap_us();
  this->line_rate_.applied(profile);
  this->line_rate_hold_ = false;
  this->line_refusal_.clear();
  // (a pin without a return path has no receivers to report their reacquisition)
  this->line_rate_reason_ = this->port_type_ == PT_D2A_PIN ? "" : "reacquiring after profile change";
#ifdef USE_ULTRA_LINE_TELEMETRY
  this->poller_.invalidate_line_stats();
#endif
  this->resync();
  ESP_LOGW(TAG, "GPIO%u: line profile %u, effect interval %" PRIu32 " ms", this->pin_, profile, this->effect_interval());
}

void UltraHub::service_line_rate_() {
  if (this->calibration_running()) {this->line_rate_hold_=false;return;}
  uint8_t requested = this->line_rate_.requested();
#ifdef USE_ULTRA_PARLIO
  if (this->parlio_bus_ != nullptr) {
    // A shared-clock PARLIO group changes together. Every lane must authorize
    // the candidate; a missing inventory on any lane keeps the whole group at 800.
    if (this->parlio_bus_->profile() != this->line_profile()) {
      this->apply_line_profile_(this->parlio_bus_->profile());
      return;
    }
    for (auto *hub : hubs_) if (hub->parlio_bus_ == this->parlio_bus_)
      requested = std::max(requested, hub->line_rate_.requested());
    if (requested == this->line_profile()) {
      if (!this->line_refusal_.empty()) { this->line_refusal_.clear(); this->publish_line_speed_(); }
      return;
    }
    for (auto *hub : hubs_) if (hub->parlio_bus_ == this->parlio_bus_) {
      if (hub->calibration_running()) {
        for (auto *member : hubs_) if (member->parlio_bus_ == this->parlio_bus_) member->line_rate_hold_=false;
        return;
      }
      int16_t board = -1;
      if (const char *refusal = hub->line_profile_refusal_(requested, &board)) {
        for (auto *member : hubs_) if (member->parlio_bus_ == this->parlio_bus_) member->line_rate_hold_=false;
        this->refuse_line_profile_(requested, hub, refusal, board);
        return;
      }
    }
    bool idle = true;
    for (auto *hub : hubs_) if (hub->parlio_bus_ == this->parlio_bus_) {
      hub->line_rate_hold_ = true;
      idle &= hub->line_profile_idle_();
    }
    if (idle && !this->parlio_bus_->profile_pending() && !this->parlio_bus_->request_profile(requested)) {
      this->line_rate_reason_ = "group profile requires a qualified 40 MHz clock";
      for (auto *hub : hubs_) if (hub->parlio_bus_ == this->parlio_bus_) hub->line_rate_hold_ = false;
    }
    return;
  }
#endif
  if (requested == this->line_profile()) {
    if (!this->line_refusal_.empty()) { this->line_refusal_.clear(); this->publish_line_speed_(); }
    return;
  }
  int16_t board = -1;
  if (const char *refusal = this->line_profile_refusal_(requested, &board)) {
    this->refuse_line_profile_(requested, this, refusal, board);
    return;
  }
  this->line_rate_hold_ = true;
  if (this->line_profile_idle_()) this->apply_line_profile_(requested);
}

void UltraHub::loop() {
  if (!this->rx_lights_done_)
    this->rx_settings_lights_();
  this->service_change_log_();
#ifdef USE_DIFFPOWER
  if (this->port_ != nullptr && !this->port_attached_)
    this->attach_port_();
  this->service_notice_();
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
  bool output_voltage_low_hold = this->port_standby() || this->port_off() || this->calibration_running();
#ifdef USE_ULTRA_LINE_UPDATE
  output_voltage_low_hold |= this->update_running();
#endif
  if (this->poller_.service_output_voltage_low(millis(), output_voltage_low_hold))
    this->rw_publish_();
  this->ledger_service_();
  this->ledger_line_();
  if (this->comm_on_)
    this->service_commission_();
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
#ifdef USE_ULTRA_LINE_UPDATE
  // a port job owns the line (lease_line): no normal frames, polls or rate changes meanwhile
  if (this->update_running())
    return;
#endif
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) {
    this->loop_digital_();
    return;
  }
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
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
#ifdef USE_ULTRA_LINE_TELEMETRY
  const bool cal_quiet = this->pixel_cal_job_.quiet();
#else
  const bool cal_quiet = false;
#endif
  this->service_line_rate_();
  if (this->line_rate_hold_) return;
  const bool urgent = this->tx_prepared_ || (!cal_quiet && (this->preamble_ > 0 ||
                      this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES || this->sched_.pending()));
#ifdef USE_ULTRA_LINE_TELEMETRY
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
  if (!urgent && !rw && this->has_sent_ && millis() - this->last_frame_ms_ < (cal_quiet ? 100u : this->refresh_interval_ms_))
    return;
  // Non-blocking: if the previous frame (incl. its reset) is still on the wire, retry next loop. A lane: the renderer
  // hasn't taken the posted frame yet (it is a few ms ahead of the wire).
  if (!this->line_free_())
    return;
#ifdef USE_ULTRA_LINE_TELEMETRY
  this->rw_next_ = rw;
#endif
  this->transmit_();
}

void UltraHub::transmit_() {
  // The header (the frame's trailer) carries the host clock at the frame's last bit: the start, taken right before
  // the frame goes out, plus its bits on the wire. A frame the RMT refused is sent again (with a new TIME), not built
  // anew: its commands were taken from the queues already. A lane's frame starts where the stream has room for it.
  const int64_t now = now_us_();
#ifdef USE_ULTRA_PARLIO
  const int64_t start = this->lane_ != nullptr ? this->lane_->reserve_us() : now;
#else
  const int64_t start = now;
#endif
  [[maybe_unused]] int64_t time_us = 0;   // the frame's TIME: the host clock at its last bit (telemetry: t0)
  if (!this->tx_prepared_) {
    this->tx_normal_ = true;
    this->tx_preamble_end_ = false;
    this->tx_data_ = false;
#ifdef USE_ULTRA_LINE_TELEMETRY
    this->tx_rw_ = false;
    uint16_t polls = 0;
    // the frames that (re)lock the chain: no RW frame before ULTRA_RW_LOCK_MS after the last of them
    if (this->rw_uart_ != nullptr && (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES || this->preamble_ > 0
                                      )) {
      this->poller_.locked(millis());
      this->rw_lock_end_ = true;
    } else if (this->rw_next_) {
      polls = this->poller_.build(millis(), this->rw_slots_.data(), &this->rw_window_us_, !this->receivers_ready_());
    }
#endif
    size_t bytes = this->sched_.frame_bytes();
#ifdef USE_ULTRA_LINE_TELEMETRY
    if (this->pixel_cal_job_.quiet()) {
      auto &job = this->pixel_cal_job_;
      time_us = this->sched_.build_calibration_frame(start, this->tx_buf_, job.board, job.action,
                                                    this->rw_slots_.data(), polls);
      if (!polls) job.action = 0;
      this->tx_rw_ = polls > 0;
      if (this->tx_rw_) {
        this->rw_turn_ = true; this->rw_waiting_ = false;
        if (this->rw_rx_enable_pin_ != nullptr) this->rw_rx_enable_pin_->digital_write(true);
      }
    } else
#endif
    if (this->switch_frame_ < ULTRA_SWITCH_WS_FRAMES) {
      // switch_boards_to_ultra: a WS config frame (no Ultra header), with the same bit timing
      this->tx_normal_ = false;
      bytes = ultra_switch_frame(this->tx_buf_, this->switch_frame_++, this->num_boards_);
    } else if (this->preamble_ > 0 && this->sched_.power_off_left() == 0) {   // (a notice copy first)
      this->preamble_--;
      this->sched_.build_dark_frame(start, this->tx_buf_);
      this->tx_preamble_end_ = this->preamble_ == 0 && this->preamble_full_;
#ifdef USE_ULTRA_LINE_TELEMETRY
    } else if (polls > 0) {
      // an RW frame: POLL slots for the receivers, the boards behind them as usual
      time_us = this->sched_.build_rw_frame(start, this->tx_buf_, this->rw_slots_.data(), polls);
      this->tx_rw_ = true;
      this->tx_data_ = true;
      // the UART is this line's until the window is read: its receiver on before the frame's last bit
      this->rw_turn_ = true;
      this->rw_waiting_ = false;
      if (this->rw_rx_enable_pin_ != nullptr)
        this->rw_rx_enable_pin_->digital_write(true);
#endif
    } else {
      this->sched_.build_frame(start, this->tx_buf_);
      this->tx_data_ = true;
      this->build_peak_us_ = std::max(this->build_peak_us_, static_cast<uint32_t>(now_us_() - now));
    }
    this->tx_bytes_ = bytes;
    this->tx_prepared_ = true;
  } else if (this->tx_normal_) {
    time_us = this->sched_.retime_frame(start, this->tx_buf_);
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_dir_pin_ != nullptr && !this->rw_driving_) {
    this->rw_dir_pin_->digital_write(true);   // the hub drives the line again (the window is over)
    this->rw_driving_ = true;
  }
  // only an RW frame ends early (its window is the reset): every other frame has the full reset
  this->enc_.symbols.reset = this->tx_rw_ ? this->rw_reset_symbol_ : this->reset_symbol_;
  if (this->tx_rw_) {
    this->rw_uart_rate_();   // (this line's reply rate)
    uint8_t junk[64];
    size_t n;
    while ((n = this->rw_uart_->available()) > 0 && this->rw_uart_->read_array(junk, std::min(n, sizeof(junk))))
      ;   // the window's bytes start here (the hub's own frames, breaks)
    this->rw_release_ = this->rw_dir_pin_ != nullptr;
  }
#endif
  rmt_transmit_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
#ifdef USE_ULTRA_PARLIO
  const esp_err_t err = this->lane_ != nullptr
                            ? this->lane_post_()
                            : rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, this->tx_bytes_, &cfg);
#else
  const esp_err_t err = rmt_transmit(this->channel_, this->encoder_, this->tx_buf_, this->tx_bytes_, &cfg);
#endif
  if (err != ESP_OK) {
#ifdef USE_ULTRA_LINE_TELEMETRY
    this->rw_release_ = false;
#endif
    if (!this->status_has_warning())   // the frame is retried every loop: log the first failure only
      ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    this->status_set_warning();
    return;
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->pixel_cal_job_.quiet() && !this->tx_rw_ &&
      this->tx_buf_[this->pixel_cal_job_.board * ULTRA_SLOT_BYTES] == ULTRA_IDENTIFY)
    this->pixel_cal_job_.on_sent(millis());
  if (this->tx_rw_) {
    // nothing goes out until the window ends (W + ULTRA_RW_HOLDOFF_US after the frame's last bit)
    this->rw_open_ = true;
    this->rw_t0_us_ = time_us;
    this->rw_end_us_ = time_us + this->rw_window_us_ + ULTRA_RW_END_US + this->line_gap_us();
    this->rw_read_us_ = this->rw_end_us_ + 2000;
    if (this->rw_dir_pin_ != nullptr)
      this->rw_driving_ = false;   // (the RMT's interrupt lets go of the line)
    this->tx_rw_ = false;
    this->loop_fast_();   // (fast while the window is open: not the busy hold)
  }
#endif
  this->line_quiet_until_us_ = start + static_cast<int64_t>((this->sched_.wire_ns(this->tx_buf_, this->tx_bytes_) + 999) / 1000) + this->line_gap_us();
  this->tx_prepared_ = false;
#ifdef USE_DIFFPOWER
  if (this->sched_.take_power_off_sent() && this->port_ != nullptr) {
#ifdef USE_ULTRA_PARLIO
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
  if (this->tx_data_ && ultra_frame_lit(this->tx_buf_, this->tx_bytes_))
    this->note_wake_light_();   // (only a frame that carries light: a lit SET or PIXEL_FX)
  this->status_clear_warning();
  this->last_frame_ms_ = millis();
  this->has_sent_ = true;
  this->frames_++;
}

bool UltraHub::line_free_() {
#ifdef USE_ULTRA_PARLIO
  if (this->lane_ != nullptr)
    return this->lane_->ready() && !this->lane_->busy();
#endif
  return now_us_() >= this->line_quiet_until_us_ && rmt_tx_wait_all_done(this->channel_, 0) == ESP_OK;
}

bool UltraHub::line_out_() {
#ifdef USE_ULTRA_PARLIO
  if (this->lane_ != nullptr)
    return now_us_() >= this->lane_->end_us();   // (its frame and its reset: the lane's quiet after it)
#endif
  return now_us_() >= this->line_quiet_until_us_ && rmt_tx_wait_all_done(this->channel_, 0) == ESP_OK;
}

#ifdef USE_ULTRA_PARLIO
// The frame to the lane: its quiet after it is the reset, after an RW frame the window (the receivers answer) and the
// hold-off; with a direction pin the bus' timer lets go of the line at its last bit. ESP_ERR_TIMEOUT: its start
// passed while it was built (sent again with a new start next loop, like a frame the RMT refused).
esp_err_t UltraHub::lane_post_() {
  uint32_t quiet = this->line_gap_us();
  bool release = false;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->tx_rw_) {
    quiet = std::max<uint32_t>(quiet, this->rw_window_us_ + ULTRA_RW_END_US + this->line_gap_us());
    release = this->rw_dir_pin_ != nullptr;
  }
#endif
  return this->lane_->post(this->tx_buf_, this->tx_bytes_, quiet, release) ? ESP_OK : ESP_ERR_TIMEOUT;
}
#endif

#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
// ------------------------------------------------------------------------------------------------ a digital port
// The planner (ultra_dport.h, shared with the sender library, tested against the receiver's own delay line there)
// decides when a transmission may start and what LOW goes before and after it; this sends it: pixel frames from the
// stream light, Ultra frames (commands, refreshes, polls) behind a marker, the window of a poll. One RMT channel and
// one encoder whose symbol table (lead LOW, marker, the bits, tail LOW) is set before each transmission, while the line
// is free.

// The stream light hands over a frame: copied (the light may draw the next while this one waits for the line or is
// on the wire); one waits at most, the newest.
// setup(): the planner, the pixel buffers, the back-off a restart keeps. false: failed (logged, marked).
bool UltraHub::digital_setup_() {
  this->preamble_ = 0;
  ultra_dp_init(&this->dp_, this->dp_reset_us_);
  ultra_dp_powered(&this->dp_, this->now_us_());   // (the receiver may have booted with the ESP32: no marker yet)
  if (this->px_bytes_ != 0) {
    RAMAllocator<uint8_t> px_alloc(RAMAllocator<uint8_t>::ALLOC_INTERNAL);
    this->px_pending_ = px_alloc.allocate(this->px_bytes_);
    this->px_tx_ = px_alloc.allocate(this->px_bytes_);
    if (this->px_pending_ == nullptr || this->px_tx_ == nullptr) {
      ESP_LOGE(TAG, "Cannot allocate the pixel buffers (2 x %u B)", static_cast<unsigned>(this->px_bytes_));
      this->mark_failed();
      return false;
    }
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_uart_ != nullptr) {
    this->poller_.locked(millis());
    this->dp_pref_ = global_preferences->make_preference<uint8_t>(this->tel_key_ ^ 0x44504F31u);
    uint8_t flag = 0;
    if (this->dp_pref_.load(&flag) && flag == 1) {
      this->dp_.bo_phase = ULTRA_DP_BO_GAVE_UP;
      this->dp_stored_ = true;
      ESP_LOGW(TAG, "GPIO%u: the receiver stopped answering before the last restart: no markers on the strip "
               "until it is retried (ultra_line.retry_receiver)", this->pin_);
    }
  }
#endif
  return true;
}

void UltraHub::digital_pixels(const uint8_t *data) {
  if (!this->digital_ || this->px_pending_ == nullptr || this->px_bytes_ == 0)
    return;
  std::memcpy(this->px_pending_, data, this->px_bytes_);
  this->px_ready_ = true;
  this->px_have_ = true;
  this->wake_();
}

// A PIXEL_FX of a segment was queued: a local effect (the receiver makes the pixels, the hub sends none and a header at
// least once a second), or effect 0: the stream again once the receiver has finished its frame (a hand-back).
void UltraHub::digital_note_fx(uint8_t segment, uint8_t effect, int64_t start_us, uint32_t local_frame_us) {
  if (!this->digital_ || segment >= 8)
    return;
  const uint8_t bit = static_cast<uint8_t>(1u << segment);
  const bool was = this->fx_active_ != 0;
  if (effect != 0)
    this->fx_active_ = static_cast<uint8_t>(this->fx_active_ | bit);
  else
    this->fx_active_ = static_cast<uint8_t>(this->fx_active_ & ~bit);
  const bool now = this->fx_active_ != 0;
  if (now) {
    ultra_dp_local(&this->dp_, 1);
  } else if (was) {
    ultra_dp_handback(&this->dp_, start_us, local_frame_us);   // (clears the local flag, holds the pixels back)
    this->px_ready_ = this->px_have_;   // the stream's newest picture goes out again first (the light only draws on a change)
  }
  this->wake_();
}

void UltraHub::digital_retry() {
  if (!this->digital_)
    return;
  ultra_dp_retry(&this->dp_);
  this->digital_save_();
  ESP_LOGI(TAG, "GPIO%u: the receiver is polled again", this->pin_);
  this->wake_();
}

// The back-off's end state is kept across restarts: a receiver that stopped answering is not tried again after every
// reboot (each try is a glitch on the strip), only after the user's retry. The first two phases are not: they are
// short and restart with a fresh count.
void UltraHub::digital_save_() {
#ifdef USE_ULTRA_LINE_TELEMETRY
  const bool gave_up = this->dp_.bo_phase == ULTRA_DP_BO_GAVE_UP;
  if (gave_up == this->dp_stored_)
    return;
  this->dp_stored_ = gave_up;
  const uint8_t flag = gave_up ? 1 : 0;
  this->dp_pref_.save(&flag);
#endif
}

void UltraHub::digital_window_end_(int64_t now) {
#ifdef USE_ULTRA_LINE_TELEMETRY
  const uint32_t answers = this->poller_.stats().answers;
  this->finish_window_(false);
  const bool answered = this->poller_.stats().answers != answers;
  const uint8_t was = this->dp_.bo_phase;
  ultra_dp_poll_result(&this->dp_, now, answered);
  if (this->dp_.bo_phase != was) {
    if (this->dp_.bo_phase == ULTRA_DP_BO_GAVE_UP) {
      ESP_LOGW(TAG, "GPIO%u: the receiver does not answer: no more markers on the strip until it is retried "
               "(ultra_line.retry_receiver, or its Retry button)", this->pin_);
    } else if (this->dp_.bo_phase == ULTRA_DP_BO_WAIT) {
      ESP_LOGW(TAG, "GPIO%u: the receiver does not answer: %u more tries in %u s", this->pin_,
               static_cast<unsigned>(ULTRA_DP_TRIES), static_cast<unsigned>(ULTRA_DP_RETRY_S));
    }
    this->digital_save_();
  }
#endif
}

// An open window ends at W_D after the frame's last bit: from the RMT's interrupt (the line was let go of) when the
// hub has it, else from the frame's TIME. The first frame after it retakes the line.
void UltraHub::digital_window_track_() {
#ifdef USE_ULTRA_LINE_TELEMETRY
  int64_t released = this->rw_released_us_;   // (written by the RMT's interrupt: 64 bits, read until it is stable)
  if (released != this->rw_released_us_)
    released = this->rw_released_us_;
  if (released != 0) {
    const int64_t end = released - 2 + static_cast<int64_t>(ultra_dp_window_us(this->poller_.rate()));
    if (end > this->rw_end_us_) {
      this->rw_end_us_ = end;
      this->dp_.free_us = std::max(this->dp_.free_us, end);
    }
  }
#endif
}

void UltraHub::loop_digital_() {
  const int64_t now = now_us_();
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_open_) {
    this->digital_window_track_();
    if (now < this->rw_end_us_)
      return;
    this->digital_window_end_(now);
  }
#endif
  const bool blocked = ultra_dp_blocked(&this->dp_, now);
  const bool quiet = now < this->dp_.quiet_until_us;   // (no marker for a while after the power-up)
#ifdef USE_ULTRA_LINE_TELEMETRY
  const bool cal_quiet = this->pixel_cal_job_.quiet();
#else
  const bool cal_quiet = false;
#endif
  const bool refresh_due = !this->has_sent_ || millis() - this->last_frame_ms_ >= (cal_quiet ? 100u : this->refresh_interval_ms_);
  const bool cmds = !blocked && !quiet && (this->tx_prepared_ || (!cal_quiet && this->sched_.pending()));
  unsigned want = 0;
  if (cmds || (refresh_due && !blocked && !quiet))
    want |= ULTRA_DP_W_CMD;
  bool poll = false;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (!blocked && !quiet && !this->tx_prepared_ && this->rw_due_(false)) {
    poll = true;
    want |= ULTRA_DP_W_POLL;
  }
#endif
  const bool pixels = !cal_quiet && this->px_ready_ && !this->dp_.local;
  if (pixels)
    want |= ULTRA_DP_W_PIXELS;
  if (cmds || poll || pixels)
    this->last_busy_ms_ = millis();
  if (this->high_freq_active_ && !cmds && !poll && !pixels && millis() - this->last_busy_ms_ > BUSY_HOLD_MS) {
    this->high_freq_.stop();
    this->high_freq_active_ = false;
  }
  if (want == 0 || !this->line_free_())
    return;
  ultra_dp_tx_t tx;
  if (!ultra_dp_next(&this->dp_, now, want, &tx))
    return;
  this->transmit_digital_(tx, now);
}

void UltraHub::transmit_digital_(const ultra_dp_tx_t &tx, int64_t now) {
  const uint32_t res = this->rmt_res_;
  ultra_dp_rmt_t &tab = this->dp_enc_.tab;
  std::memset(&tab, 0, sizeof(tab));
  tab.lead = ultra_dp_rmt_low(tx.lead_low_us, res);
  tab.tail = ultra_dp_rmt_low(tx.tail_low_us, res);
  const uint8_t *data;
  size_t size;
  uint32_t wire_us;
  int64_t time_us = 0;
  const bool poll = tx.kind == ULTRA_DP_POLL;
  if (tx.kind == ULTRA_DP_PIXELS) {
    // the frame being sent is a copy: the light draws on (the newest waits in px_pending_)
    std::memcpy(this->px_tx_, this->px_pending_, this->px_bytes_);
    this->px_ready_ = false;
    tab.bit0 = this->px_word0_;
    tab.bit1 = this->px_word1_;
    data = this->px_tx_;
    size = this->px_bytes_;
    uint32_t ones = 0;
    for (size_t i = 0; i < size; i++)
      ones += static_cast<uint32_t>(__builtin_popcount(data[i]));
    const uint64_t ns = static_cast<uint64_t>(ones) * (this->px_t1h_ + this->px_t1l_) +
                        static_cast<uint64_t>(size * 8 - ones) * (this->px_t0h_ + this->px_t0l_);
    wire_us = tx.lead_low_us + static_cast<uint32_t>((ns + 500) / 1000) + tx.tail_low_us;
  } else {
    tab.marker = ultra_dp_rmt_marker(res);
    tab.bit0 = this->enc_.symbols.bit0.val;
    tab.bit1 = this->enc_.symbols.bit1.val;
    // the frame's first bit follows the marker and its gap: its TIME (and the commands' start offsets) from there
    const int64_t frame_start = now + tx.lead_low_us + ULTRA_DP_MARKER_US + ULTRA_DP_GAP_HUB_US;
    if (!this->tx_prepared_) {
      this->tx_normal_ = true;
#ifdef USE_ULTRA_LINE_TELEMETRY
      this->tx_rw_ = false;
      uint16_t polls = 0;
      if (poll)
        polls = this->poller_.build(millis(), this->rw_slots_.data(), &this->rw_window_us_, !this->receivers_ready_());
      if (poll && polls == 0)
        return;   // (nothing to ask after all)
      if (this->pixel_cal_job_.quiet()) {
        auto &job = this->pixel_cal_job_;
        time_us = this->sched_.build_calibration_frame(frame_start, this->tx_buf_, job.board, job.action,
                                                      this->rw_slots_.data(), polls);
        if (!polls) job.action = 0;
        this->tx_rw_ = poll;
      } else if (poll) {
        time_us = this->sched_.build_rw_frame(frame_start, this->tx_buf_, this->rw_slots_.data(), polls);
        this->tx_rw_ = true;
      } else
#endif
      {
        time_us = this->sched_.build_frame(frame_start, this->tx_buf_);
      }
      this->tx_bytes_ = this->sched_.frame_bytes();
      this->tx_prepared_ = true;
    } else {
      time_us = this->sched_.retime_frame(frame_start, this->tx_buf_);   // (a frame the RMT refused: sent again)
    }
    data = this->tx_buf_;
    size = this->tx_bytes_;
    wire_us = tx.lead_low_us + ULTRA_DP_MARKER_US + ULTRA_DP_GAP_HUB_US +
              static_cast<uint32_t>((this->sched_.wire_ns(this->tx_buf_, this->tx_bytes_) + 500) / 1000) +
              tx.tail_low_us;
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_dir_pin_ != nullptr && !this->rw_driving_) {
    this->rw_dir_pin_->digital_write(true);   // the hub drives the line again (the window is over); its LOW lead follows
    this->rw_driving_ = true;
  }
  if (poll) {
    // the UART is this line's until the window is read: the receiver on before the frame's last bit
    this->rw_turn_ = true;
    this->rw_waiting_ = false;
    if (this->rw_rx_enable_pin_ != nullptr)
      this->rw_rx_enable_pin_->digital_write(true);
    this->rw_uart_rate_();
    uint8_t junk[64];
    size_t n;
    while ((n = this->rw_uart_->available()) > 0 && this->rw_uart_->read_array(junk, std::min(n, sizeof(junk))))
      ;   // the window's bytes start here
    this->rw_release_ = this->rw_dir_pin_ != nullptr;
    this->rw_released_us_ = 0;
  }
#endif
  rmt_transmit_config_t cfg;
  std::memset(&cfg, 0, sizeof(cfg));
  const esp_err_t err = rmt_transmit(this->channel_, this->encoder_, data, size, &cfg);
  if (err != ESP_OK) {
#ifdef USE_ULTRA_LINE_TELEMETRY
    this->rw_release_ = false;
    if (poll) {
      this->tx_prepared_ = false;   // (a poll is not worth keeping: asked again)
      this->tx_rw_ = false;
      this->rw_drop_turn_();
    }
#endif
    if (!this->status_has_warning())
      ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
    this->status_set_warning();
    return;
  }
  const uint8_t rate =
#ifdef USE_ULTRA_LINE_TELEMETRY
      this->poller_.rate();
#else
      ULTRA_RW_RATE_1M;
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->pixel_cal_job_.quiet() && tx.kind != ULTRA_DP_PIXELS && !poll &&
      this->tx_buf_[this->pixel_cal_job_.board * ULTRA_SLOT_BYTES] == ULTRA_IDENTIFY)
    this->pixel_cal_job_.on_sent(millis());
#endif
  ultra_dp_started(&this->dp_, &tx, now, wire_us, time_us, rate);
  if (!poll && (tx.kind == ULTRA_DP_PIXELS ? ultra_pixels_lit(data, size) : ultra_frame_lit(data, size)))
    this->note_wake_light_();   // (a pixel frame with a lit byte, or a command frame with a lit SET / PIXEL_FX)
  if (tx.kind == ULTRA_DP_PIXELS) {
    this->px_frames_++;
    this->status_clear_warning();
    return;
  }
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (poll) {
    // nothing goes out until the window ends (W_D after the frame's last bit); the line is let go of by the RMT's
    // interrupt, the receiver answers, the UART is read when the window is over (digital_window_end_)
    this->rw_open_ = true;
    this->rw_t0_us_ = time_us;
    this->rw_end_us_ = this->dp_.free_us;
    this->rw_read_us_ = this->rw_end_us_;
    if (this->rw_dir_pin_ != nullptr)
      this->rw_driving_ = false;
    this->tx_rw_ = false;
    this->loop_fast_();
  }
#endif
  this->tx_prepared_ = false;
#ifdef USE_DIFFPOWER
  if (this->sched_.take_power_off_sent() && this->port_ != nullptr)
    this->port_->notice_sent();   // (its next copy 50 ms from now)
#endif
  this->status_clear_warning();
  this->last_frame_ms_ = millis();
  this->has_sent_ = true;
  this->frames_++;
}
#endif  // USE_DIG2ANALOG_UR_DIGITAL_M

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
#ifdef USE_ULTRA_PARLIO
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
  ESP_LOGCONFIG(TAG, "  Line speed: %s, started at %s (%s)", this->line_profile_text().c_str(),
                LINE_RATE_TEXT[this->line_choice_.profile],
                line_choice_text(this->port_type_, this->num_boards_, this->line_choice_).c_str());
  if (this->num_boards_ > 0 && this->sched_.has_config(0, ULTRA_CFG_BOOT_PROTOCOL)) {
    static const char *const MODES[] = {"auto", "WS2915", "WS2805", "Ultra"};
    const uint16_t m = this->sched_.get_config(0, ULTRA_CFG_BOOT_PROTOCOL);
    ESP_LOGCONFIG(TAG, "  Boot protocol (stored on the boards): %s", MODES[m < 4 ? m : 0]);
  }
  for (uint16_t b = 0; b < this->num_boards_ && b < 4; b++) {
    const uint16_t preset = this->sched_.get_config(b, ULTRA_CFG_PWM_PRESET);
    const uint16_t period = this->sched_.get_config(b, ULTRA_CFG_PWM_PERIOD);
    // (CONFIG 2 is in 10 ns units: the board's timer counts as its firmware rounds them)
    const uint32_t steps = period != 0 ? pwm_period_steps(period) : (preset < 15 ? PWM_PRESET_STEPS[preset] : 0);
    const float pwm_hz = steps ? PWM_CLOCK_HZ / static_cast<float>(steps) : 0.0f;
    char min_pulse[24], skip[48];
    if (this->sched_.has_config(b, ULTRA_CFG_MIN_PULSE))
      snprintf(min_pulse, sizeof(min_pulse), "%u ns", this->sched_.get_config(b, ULTRA_CFG_MIN_PULSE) * 10u);
    else
      snprintf(min_pulse, sizeof(min_pulse), "the board's own");
    if (this->sched_.has_config(b, ULTRA_CFG_SKIP_RATE)) {
      const uint16_t rate = this->sched_.get_config(b, ULTRA_CFG_SKIP_RATE), st = this->sched_.get_config(b, ULTRA_CFG_STAGGER);
      snprintf(skip, sizeof(skip), "%u Hz (slowest %" PRIu32 " Hz)", rate, pwm_skip_slowest_hz(steps, rate, st == 2 || st == 4));
    } else {
      snprintf(skip, sizeof(skip), "2000 Hz (the board's only rate)");
    }
    ESP_LOGCONFIG(TAG,
                  "  Board %u: PWM %.1f Hz (%" PRIu32 " steps), dithering %u, threshold %.1f %%, dim floor %u, "
                  "min pulse %s, pulse skipping %s, stagger %u, signal loss %u after %.1f s, board limit %.2f A",
                  b + 1, pwm_hz, steps,
                  this->sched_.get_config(b, ULTRA_CFG_DITHER), this->sched_.get_config(b, ULTRA_CFG_DITHER_THRESH) / 10.0f,
                  this->sched_.get_config(b, ULTRA_CFG_DIM_FLOOR), min_pulse, skip,
                  this->sched_.get_config(b, ULTRA_CFG_STAGGER), this->sched_.get_config(b, ULTRA_CFG_LOSS),
                  this->sched_.get_config(b, ULTRA_CFG_LOSS_TIMEOUT) / 10.0f,
                  this->sched_.get_config(b, ULTRA_CFG_BOARD_MAX) / 1000.0f);
  }
#ifdef USE_POWER_SUPPLY
  if (this->psu_ != nullptr)
    ESP_LOGCONFIG(TAG, "  Power supply: replay after %" PRIu32 " ms", this->power_on_delay_ms_);
#endif
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->rw_uart_ != nullptr) {
    static const char *const RATES[] = {"1 Mbaud", "500 kbaud", "250 kbaud"};
    ESP_LOGCONFIG(TAG, "  Telemetry: %u receiver(s) (boards 1-%u), answers at %s%s, windows up to %" PRIu32 " us%s%s",
                  this->poller_.receivers(), this->poller_.receivers(), RATES[this->poller_.rate() % 3],
                  this->reply_.adaptive() ? " (slower on broken answers)" : " (pinned)",
                  this->poller_.max_window_us(), this->rw_dir_pin_ != nullptr ? ", direction pin" : "",
                  this->rw_rx_enable_pin_ != nullptr ? ", receiver enable pin (its UART may be shared)" : "");
    if (this->comm_on_)
      ESP_LOGCONFIG(TAG, "  Commissioning: %s", this->comm_.status().c_str());
  }
#endif
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  if (this->digital_) {
    ESP_LOGCONFIG(TAG,
                  "  Digital port: one Diff-Power-UR-Digital-M in the strip's data path, frames behind markers, no "
                  "preamble; strip reset %" PRIu32 " us, pixel frames %" PRIu32 " B (bit0 %" PRIu32 "/%" PRIu32
                  " ns, bit1 %" PRIu32 "/%" PRIu32 " ns)",
                  this->dp_reset_us_, this->px_bytes_, this->px_t0h_, this->px_t0l_, this->px_t1h_, this->px_t1l_);
  }
#endif
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup FAILED");
}

}  // namespace esphome::ultra_line

#endif  // USE_ESP32
