#pragma once

#ifdef USE_ESP32

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/light/light_state.h"

#include "ultra_core.h"
#include "ultra_settings.h"
#include "ultra_line_rate.h"
#include "ultra_host_effects.h"
#include "ultra_readiness.h"
#include "ultra_change_log.h"

#ifdef USE_DIFFPOWER
#include "esphome/components/diffpower/diffpower.h"
#endif
#ifdef USE_POWER_SUPPLY
#include "esphome/components/power_supply/power_supply.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

#include <algorithm>
#include <functional>
#include <vector>

#ifdef USE_ULTRA_LINE_UPDATE
#include "ultra_sender_update.h"
#endif

#ifdef USE_ULTRA_LINE_TELEMETRY
#include "esphome/components/uart/uart_component.h"
#include "esphome/core/preferences.h"
#include "ultra_ledger.h"
#include "ultra_telemetry.h"
#include "ultra_commission.h"
#include "ultra_stream.h"
#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif
#endif

#ifdef USE_ULTRA_PARLIO
#include "ultra_parlio.h"
#include "esphome/components/ultra_parlio/ultra_parlio_bus.h"
#endif

#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
#include "ultra_dport.h"
#endif

namespace esphome::ultra_line {

/// The lambdas' and actions' board numbers count from 1, as in YAML (1 = the first board after the controller);
/// this one is every board of the line
static constexpr int ALL_BOARDS = -1;

class UltraLight;

/// A light of a Diff-Power Ultra Receiver Digital-Medium (ur_digital_m, local pixel effects) on a board of this line: what the hub
/// needs of it for the power (a Diff-Power port, the supply), the receivers' lit-time and the resync.
class PixelSource {
 public:
  virtual uint16_t pixel_board() const = 0;
  /// on, or still fading out on the receiver (power supply request)
  virtual bool pixel_lit(uint32_t now_ms) = 0;
  /// its own turn-off period on a Diff-Power port (0: the port's)
  virtual uint32_t pixel_standby_after() const = 0;
  virtual light::LightState *pixel_state() const = 0;
};

#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
/// A digital port's encoder argument: the symbol table of the transmission (lead LOW, marker, bits, tail LOW)
struct DpEncoder {
  ultra_dp_rmt_t tab;
};
#endif

using EncoderParams = FrameSymbols<rmt_symbol_word_t>;
/// The encoder's argument: the bit symbols, and the underrun watch (diagnostics, only when watching).
struct EncoderArg {
  EncoderParams symbols;
  UnderrunWatch watch;
  bool watching{false};
};

/// One data line with num_boards QuinLED Dig2Analog-Plus-5ch boards in Ultra mode. The boards run transitions, gamma and
/// host effects through faded SET targets; this hub sends commands (one 32-byte slot per board per frame, then a header with
/// the host time at the frame end that keeps all boards on one clock). Board settings are host-managed: the YAML
/// values are sent at boot and repeated, and the runtime setters below change them (in the boards' RAM, never their
/// flash; the boot protocol is the one exception, stored by the boards when it differs).
class UltraHub final : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  /// OTA / a reboot: the receivers' lifetime ledger stored
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // Configuration (codegen)
  void set_pin(uint8_t pin) { this->pin_ = pin; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  void set_num_boards(uint16_t n) { this->num_boards_ = n; }
  void set_light_counts(std::vector<uint8_t> counts) { this->light_counts_ = std::move(counts); }
  void set_max_commands(uint8_t n) { this->max_commands_ = n; }
  void set_rmt_symbols(uint32_t n) { this->rmt_symbols_ = n; }
  void set_use_dma(bool use_dma) { this->use_dma_ = use_dma; }
#ifdef USE_ULTRA_PARLIO
  /// A lane of a ultra_parlio bus (ESP32-P4) instead of an RMT channel; set_timing() then carries the
  /// lane's timing on the bus' sample grid.
  void set_parlio_lane(ParlioLane *lane) { this->lane_ = lane; }
  void set_parlio_bus(UltraParlioBus *bus) { this->parlio_bus_ = bus; }
#endif
  void set_timing(uint32_t bit0_high_ns, uint32_t bit0_low_ns, uint32_t bit1_high_ns, uint32_t bit1_low_ns,
                  uint32_t reset_ns) {
    this->bit0_high_ns_ = bit0_high_ns;
    this->bit0_low_ns_ = bit0_low_ns;
    this->bit1_high_ns_ = bit1_high_ns;
    this->bit1_low_ns_ = bit1_low_ns;
    this->reset_ns_ = reset_ns;
  }
  /// A frame at least this often (clock sync, state refresh), also when nothing changes.
  void set_refresh_interval(uint32_t ms) { this->refresh_interval_ms_ = ms; }
  /// Commands take effect this long after they are queued (instead of the automatic lead, lead_us()).
  void set_start_delay(uint32_t ms) { this->start_delay_us_ = static_cast<int64_t>(ms) * 1000; }
  void set_effect_rate(uint16_t rate) { this->effect_rate_ = std::clamp<uint16_t>(rate,1,20); }
  uint16_t effect_rate() const { return this->effect_rate_; }
  uint32_t effect_interval(uint16_t rate = 0) const;
  void set_line_rate_policy(bool automatic, uint32_t window_ms, uint16_t minimum_frames,
                            uint16_t error_permille, uint8_t bad_windows, bool inventory_complete,
                            uint8_t boot_profiles, uint16_t path_ns, uint8_t initial) {
    this->line_rate_.configure({window_ms, minimum_frames, error_permille, bad_windows, automatic});
    this->line_inventory_complete_ = inventory_complete; this->line_boot_profiles_ = boot_profiles;
    this->line_path_ns_ = path_ns; this->line_initial_ = initial;
  }
  /// PT_DIFF_POWER or PT_D2A_PIN: the boards the port chains and the line speed its board count starts at
  void set_port_type(uint8_t type) { this->port_type_ = type; }
  uint8_t port_type() const { return this->port_type_; }
  bool request_line_profile(uint8_t profile) { return this->line_rate_.request(profile); }
  uint32_t line_gap_us() const { return std::max<uint32_t>(500, std::max(this->reset_ns_ / 1000, this->descriptor_gap_us_)); }
  uint8_t line_profile() const { return this->line_rate_.current(); }
  /// The line's profile, a refused request (why) and what the line is doing about its profile
  std::string line_profile_text() const {
    return std::string(LINE_RATE_TEXT[this->line_profile()]) +
           (this->line_refusal_.empty() ? "" : "; " + this->line_refusal_) +
           (this->line_rate_reason_.empty() ? "" : "; " + this->line_rate_reason_);
  }
#ifdef USE_TEXT_SENSOR
  /// line_profile_text() as a Home Assistant diagnostic (a pin without a return path has no receiver to show it)
  void set_line_speed_sensor(text_sensor::TextSensor *s) { this->line_speed_sensor_ = s; }
#endif
  void set_repeats(uint8_t n) { this->repeats_ = n; }
  void set_power_on_delay(uint32_t ms) { this->power_on_delay_ms_ = ms; }
#ifdef USE_POWER_SUPPLY
  /// Supply of the boards (and LEDs): requested while any light is on; after it really switched on, everything
  /// is sent again after power_on_delay, so fades that ran while the boards had no power are replayed.
  void set_power_supply(power_supply::PowerSupply *psu, uint32_t keep_on_time_ms) {
    this->psu_ = psu;
    this->psu_req_.set_parent(psu);
    this->psu_keep_on_ms_ = keep_on_time_ms;
  }
#endif
#ifdef USE_DIFFPOWER
  /// The QuinLED Diff-Power port this line is on (diffpower): it wakes while a light here is on (or within its
  /// turn-off period), and once it is powered everything is sent again; the receivers show Standby while it isn't.
  void set_port(diffpower::DiffPowerPort *port) { this->port_ = port; }
  /// A copy of the planned power-off notice (protocol 0.3: IDENTIFY 7 POWER_OFF to every board in the next frame,
  /// ahead of a preamble; its port asks for copy 0, 1, 2, each 50 ms after the last one went out, which the hub
  /// reports: notice_sent())
  void send_power_off_notice(uint8_t copy);
  uint32_t power_off_notices() const { return this->notices_; }
#endif
  /// on a Diff-Power port that isn't powered now (standby or waking)
  bool port_standby() const {
#ifdef USE_DIFFPOWER
    return this->port_ != nullptr && !this->port_->is_ready();
#else
    return false;
#endif
  }
  /// on a Diff-Power port in the power mode Off (never powered: calibrations and updates don't run)
  bool port_off() const {
#ifdef USE_DIFFPOWER
    return this->port_ != nullptr && this->port_->is_off();
#else
    return false;
#endif
  }

  // Board settings (ULTRA_CFG_* ids). Before setup they are the YAML defaults. The board as in YAML (lambdas, wiki:
  // Ultra-Board-Settings): 1 = the first board after the controller, ALL_BOARDS (-1) = every board; 0 or a board
  // beyond num_boards changes nothing (logged).
  void set_board_config(int board, uint8_t id, uint16_t value);
  void set_channel_config(int board, uint8_t channel, uint8_t id, uint16_t value);
  void set_board_descriptor(uint16_t board, const std::vector<uint8_t> &bytes) {
    this->descriptor_inventory_.push_back({board, bytes});
  }
  /// every board of the line (YAML every_board_descriptor); a board's own set_board_descriptor wins
  void set_every_board_descriptor(const std::vector<uint8_t> &bytes) { this->every_board_descriptor_ = bytes; }
  /// a board's setting (board 1 = the first; 0 for a board not on this line)
  uint16_t get_board_config(int board, uint8_t id) const {
    return board >= 1 && board <= this->num_boards_ ? this->sched_.get_config(board - 1, id) : 0;
  }
  /// PWM frequency in Hz (2197..100000): a board preset when one is within 0.5 %, else the exact period.
  void set_pwm_frequency(int board, float hz);
  void set_dithering(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_DITHER, mode); }
  void set_dither_threshold(int board, float fraction) {
    this->set_board_config(board, ULTRA_CFG_DITHER_THRESH, clamp_u16_(fraction * 1000.0f + 0.5f, 1, 255));
  }
  void set_dim_floor(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_DIM_FLOOR, mode); }
  void set_min_pulse(int board, uint32_t ns) { this->set_board_config(board, ULTRA_CFG_MIN_PULSE, clamp_u16_(ns / 10.0f + 0.5f, 5, 100)); }
  /// 0 off, 1 fixed (thirds of a period), 2 fixed + spread-spectrum PWM, 3 adaptive (least LED_+ ripple for the
  /// current colour mix), 4 adaptive + spread spectrum
  void set_stagger(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_STAGGER, mode > 4 ? 4 : mode); }
  /// 0 hold, 1 off, 2 fade off, 3 fade to the stored signal-loss level (unset: dark)
  void set_signal_loss(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_LOSS, mode > 3 ? 3 : mode); }
  void set_signal_loss_timeout(int board, uint32_t ms) {
    this->set_board_config(board, ULTRA_CFG_LOSS_TIMEOUT, clamp_u16_(ms / 100.0f + 0.5f, 5, 600));
  }
  void set_feedback_channel(int board, uint8_t mode) { this->set_board_config(board, ULTRA_CFG_FEEDBACK, mode); }
  /// Stored on the board: the trips' retry policy: count 0 never, 1-254, 255 forever; 1-255 s apart. After the
  /// last retry the outputs stay off until a new command or a power cycle; 60 s without a trip starts the count again.
  void set_trip_retries(int board, uint8_t count, uint8_t interval_s) {
    this->set_board_config(board, ULTRA_CFG_RETRY,
                           static_cast<uint16_t>(count | (interval_s == 0 ? 1u : interval_s) << 8));
  }
  /// Protocol mode after power-up, stored on the board (written only when it differs): 0 auto, 1 WS2915, 2 WS2805,
  /// 3 Ultra. Auto / Ultra apply at once; WS2915 / WS2805 from the board's next power-up on (it then ignores Ultra).
  void set_boot_protocol(int board, uint8_t mode) {
    this->set_board_config(board, ULTRA_CFG_BOOT_PROTOCOL, mode > 3 ? 3 : mode);
  }
  /// Power limit: LED current of a channel at 100 % (A), limit per channel and per board (A, 0 = none).
  void set_channel_current(int board, uint8_t channel, float amps) {
    if (channel < CHANNELS)
      this->set_channel_config(board, channel, ULTRA_CFG_CH_CURRENT, clamp_u16_(amps * 1000.0f + 0.5f, 0, 65534));
  }
  void set_max_channel_current(int board, uint8_t channel, float amps) {
    if (channel < CHANNELS)
      this->set_channel_config(board, channel, ULTRA_CFG_CH_MAX, clamp_u16_(amps * 1000.0f + 0.5f, 0, 65534));
  }
  void set_max_board_current(int board, float amps) {
    this->set_board_config(board, ULTRA_CFG_BOARD_MAX, clamp_u16_(amps * 1000.0f + 0.5f, 0, 65535));
  }

  // The receivers' settings from Home Assistant (wiki: Controller-Settings; the action set_receiver_setting)
  /// key / value as in the Settings text (ultra_settings.h: pwm, dither, stagger, loss, loss_timeout, limit, retries,
  /// retry_interval, dtw; reset: the YAML's again); board 1 = the first, ALL_BOARDS (-1): every board. Kept on the ESP32 (one record per
  /// board), sent at once and with every refresh (a board that restarts or is replaced gets them again). false: a bad
  /// key or value, or no light of the board takes it (logged)
  bool set_receiver_setting(int board, const std::string &key, const std::string &value);
  /// a board's settings as its Settings text shows them (its slot: 0 = the first board)
  std::string receiver_settings_text(uint16_t board) const;
  /// the line's id: the action's port is its telemetry name or this
  void set_line_id(const char *id) { this->line_id_ = id; }
  bool is_line(const std::string &name) const;
  /// after a board's settings changed (its slot, 0 = the first board; -1: every board's)
  void add_on_settings_callback(std::function<void(int)> &&cb) { this->settings_cb_.add(std::move(cb)); }
  /// the action set_receiver_setting: the lines named port ("all": every line), receivers 1-based (empty: every board)
  static void receiver_setting_action(const std::string &port, const std::vector<int32_t> &receivers,
                                      const std::string &key, const std::string &value);

  /// Blink the identify pattern on a board (on), or stop it (board 1 = the first board after the controller).
  void identify(int board, bool on = true) {
    const int slot = this->board_slot_(board, "identify");
    if (slot < 0)   // (one board only)
      return;
    this->sched_.identify(slot, on);
    this->wake_();
  }
  /// The board (1 = the first; ALL_BOARDS = every board on this line) blinks its firmware version once on its feedback channel: 3 fast, then
  /// major, minor, patch as value codes, then alpha / beta / rc as 1 / 2 / 3 short (protocol 0.3, IDENTIFY 5). A board
  void show_version(int board = ALL_BOARDS) {
    const int slot = this->board_slot_(board, "show_version");
    if (slot == NO_BOARD)
      return;
    for (uint16_t b = 0; b < this->num_boards_; b++) {
      if (slot < 0 || slot == b)
        this->sched_.show_version(b);
    }
    this->wake_();
  }
  /// Boards with the input current sense, the current calibration of this pin (firmware wiki: Current Sense):
  /// every board on the pin fades dark (0.3 s), then one board after the other in chain order measures each channel
  /// alone (duty_pct 0: at 25 / 50 / 75 / 100 %, ~1.4 s per board; 5..100: at that duty only, ~0.5 s), then the pin
  /// fades back (states, transitions and effects continue: the boards' engines kept running). boards: which ones to
  /// test (empty = all); the whole pin goes dark either way. Other pins keep running. Ignored (logged) while this
  /// pin's run is in progress; with a power_supply it is switched on for the run. Boards without current sense
  /// ignore the test (they just go dark and back).
  void calibrate_boards(const std::vector<uint16_t> &boards, uint8_t duty_pct = 0);
  /// One receiver's test (its button): the standard test of that board, the whole pin dark meanwhile. Joins a run
  /// asked for or in progress on this pin (tested after its boards; CalSequencer::add), else starts one.
  void calibrate_receiver(uint16_t board);
  /// Stop the run: the pin fades back at once (a test already running finishes on its board, <= 1 s).
  void cancel_calibration();
  bool calibration_running() const { return this->cal_pending_ || this->cal_seq_.running()
#ifdef USE_ULTRA_LINE_TELEMETRY
        || this->pixel_cal_job_.active()
#endif
        ; }
  /// The board is in this pin's run: asked for (waiting for the boards' power), or running (to be tested, being
  /// tested, or held dark until the run is over). Its receiver's Status shows Calibrating (with_hub_calibration).
  bool calibrating(uint16_t board) const {
#ifdef USE_ULTRA_LINE_TELEMETRY
    if (this->pixel_cal_job_.active()) return this->pixel_cal_job_.board == board;
#endif
    if (this->cal_pending_)
      return this->cal_req_boards_.empty() ||
             std::find(this->cal_req_boards_.begin(), this->cal_req_boards_.end(), board) != this->cal_req_boards_.end();
    return this->cal_seq_.includes(board);
  }
  /// progress: the board being tested (1 = the first board, as in YAML; 0 while nothing runs), its position in the
  /// run (1-based; 0 while the pin goes dark or nothing runs), the count
  uint16_t calibration_board() const { return this->cal_seq_.running() ? this->cal_seq_.board() + 1 : 0; }
  size_t calibration_step() const { return this->cal_seq_.running() && this->cal_started_ ? this->cal_seq_.index() + 1 : 0; }
  size_t calibration_count() const { return this->cal_seq_.count(); }
  /// One board alone (1 = the first), without the dark pin or any sequencing (lambdas; the action uses calibrate_boards).
  void calibrate_current(int board, uint8_t duty_pct = 0) {
    const int slot = this->board_slot_(board, "calibrate_current");
    if (slot < 0)   // (one board only)
      return;
    calibrate_slot_(static_cast<uint16_t>(slot), duty_pct);
  }
  void calibrate_slot_(uint16_t board, uint8_t duty_pct) {
#ifdef USE_ULTRA_LINE_TELEMETRY
    if (board >= this->poller_.receivers() || !this->poller_.data(board).have_identity ||
        (this->poller_.data(board).hw_caps & RX_CAP_PIXEL)) return;
#endif
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
    if (this->digital_) return;
#endif
    this->sched_.calibrate_current(board, duty_pct);
    this->wake_();
  }
  /// Automatic runs (YAML current_calibration): once after boot and / or every interval_ms (0 = never), optionally
  /// only while every light on this line is off. Off unless configured: calibration is a manual step.
  void set_calibration_schedule(bool on_boot, uint32_t interval_ms, bool only_when_off,
                                const std::vector<uint16_t> &boards) {
    this->cal_on_boot_ = on_boot;
    this->cal_interval_ms_ = interval_ms;
    this->cal_only_off_ = only_when_off;
    this->cal_boards_ = boards;
  }
  /// Send everything again now (e.g. after replacing a board); running transitions and effects restart.
  /// power_on: the boards' supply just switched on: once a full preamble reached them with power, a short one does
  /// (power_on_preamble_frames()).
  void resync(bool power_on = false);
  /// Boards forced to WS2805 / WS2915 (boot_protocol ws2805 / ws2915) ignore Ultra frames. This sends their own WS
  /// config frames "protocol = Ultra" in both formats (ultra_protocol.h ultra_switch_frame), then the start-up
  /// preamble and everything again, and stores boot_protocol (0 auto, 3 Ultra) on the boards. Only on request: every
  /// board that takes it blinks its acknowledge pattern, and boards of the second format flash for a few ms.
  void switch_boards_to_ultra(uint8_t boot_protocol);
  uint16_t get_num_boards() const { return this->num_boards_; }
  bool is_power_ready() const { return !this->power_waiting_ && !this->port_standby(); }

#ifdef USE_ULTRA_LINE_UPDATE
  // "Update all receivers on this port" (fw_update.h). A port job leases the line: normal frames, polls and rate
  // changes pause meanwhile (light commands keep queueing).
  bool update_running() const { return this->upd_lease_; }
  /// The port job takes the line, or hands it back (then the boards' state again). Taking it: false until an open
  /// response window is over and read and the hub drives the line again (asked again until true)
  bool lease_line(bool on);
  /// Keep the power supply requested (a port job)
  void hold_power(bool on) { this->upd_power_hold_ = on; }
  /// A receiver's Recover firmware button: the line's port update entity recovers it (fw_update.cpp recover)
  void set_recover_handler(std::function<void(uint16_t)> &&handler) { this->recover_handler_ = std::move(handler); }
  void recover_receiver(uint16_t board) {
    if (this->recover_handler_)
      this->recover_handler_(board);
  }
  bool has_power_supply() const;
  // The port job's transport (fw_update.cpp, from its task while the line is leased): one frame on the wire as the job
  // says (CHAIN, or a digital port's marker envelope) and on it before this returns; false: refused, or too long.
  bool job_send(const uint8_t *frame, size_t n, const ultra_update_transport_t &t);
  /// the line's last frame and its LOW are out
  bool job_line_free() { return this->line_free_(); }
  /// when a frame given now starts on the wire (a lane: where its stream has room)
  int64_t job_frame_start_us();
  int64_t job_now_us() const { return now_us_(); }
  /// The sender line timing of this line: its profile on the 40 MHz grid and its processing gap. false: the line's
  /// bit timing is not that profile's (the job stamps every frame's TIME from it)
  bool job_timing(ultra_timing_t *out) const;
  bool job_digital() const {
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
    return this->digital_;
#else
    return false;
#endif
  }
  /// the longest frame job_send() takes
  size_t job_frame_max() const { return this->tx_buf_bytes_; }
#endif

  // For UltraLight / effects / strips
  /// The start of a command queued now (SET, EFFECT, ...). Commands queued close together share it, on every line
  /// (GroupStart): a Home Assistant change to a light group or a scene starts at the same moment on every board.
  int64_t start_time_us(uint16_t board);
  /// The start of a streamed step (ESPHome's own effects): the same lead for every step, no groups.
  int64_t stream_start_us() const { return this->now_us_() + this->stream_lead_us_(); }
  /// How far ahead commands start when a board gets `commands` at once and `waiting` first copies are queued before
  /// them (start_delay auto: start_lead_us()); a fixed start_delay, + a frame per further command.
  int64_t lead_us(size_t commands, size_t waiting = 0) const;
  /// The lead of a streamed step in ms (dump_config).
  uint32_t start_delay_ms() const { return static_cast<uint32_t>(this->stream_lead_us_() / 1000); }
  /// One frame on the wire, worst case (us; 0 before setup)
  uint32_t get_frame_us() const { return this->frame_us_; }
  Scheduler &scheduler() { return this->sched_; }
  /// The log of light changes (ultra_change_log.h; wiki: Ultra-Line-Options, light changes in the log): a light's SET
  /// went to the line (its channels, their levels as the board weighs them, its transition and start), a host effect's
  /// step, a change kept under a strip
  void note_change(uint16_t board, const char *platform, uint8_t mask, const uint16_t *level, uint32_t transition_ms,
                   int64_t start_us);
  void note_stream(uint16_t board, uint8_t mask);
  void note_kept(uint16_t board);
  uint8_t get_pin() const { return this->pin_; }
  /// the line is a lane of a ultra_parlio bus
#ifdef USE_ULTRA_PARLIO
  bool on_parlio() const { return this->lane_ != nullptr; }
#else
  bool on_parlio() const { return false; }
#endif
  void wake_();
  void loop_fast_();
  void add_light(UltraLight *light) { this->lights_.push_back(light); }
  size_t analog_light_count() const { return this->lights_.size(); }
  /// A pixel light of a Diff-Power-UR-Digital-M (ur_digital_m): counted as a lit light for the power and the lit-time
  void add_pixel_light(PixelSource *light) { this->pixel_lights_.push_back(light); }
  /// A setting of a Diff-Power-UR-Digital-M (CONFIG 64-77, ULTRA_CFG_PX_*; the receiver keeps it): several, then
  /// pixel_config_flush(): sent at once (one SEQ per CONFIG slot) and with every refresh
  void set_pixel_config(uint16_t board, uint8_t id, uint16_t value) {
#ifdef USE_ULTRA_LINE_TELEMETRY
    if ((id == ULTRA_CFG_PX_CHIP || id == ULTRA_CFG_PX_ORDER || id == ULTRA_CFG_PX_PIXELS) &&
        this->sched_.get_hw_config(board, id) != value)
      this->poller_.invalidate_pixel_calibration(board);
#endif
    this->sched_.set_hw_config(board, id, value);
  }
  void pixel_config_flush() {
    this->sched_.flush_config();
    this->wake_();
  }
  /// A board that is a Diff-Power-UR-Digital-M: it has no light indices to clear (clear_unused_lights)
  void set_pixel_board(uint16_t board) {
    if (std::find(this->pixel_boards_.begin(), this->pixel_boards_.end(), board) == this->pixel_boards_.end())
      this->pixel_boards_.push_back(board);
  }

#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  // A DIGITAL PORT (protocol 0.3, ultra_dport.h): one Diff-Power-UR-Digital-M (num_boards 1) in the
  // data path of a pixel strip. Its Ultra frames go out behind a marker in a latch gap (the strip never sees them), the
  // pixel stream of a light (ur_digital_m_stream) goes between them, the polls leave the line to the receiver's answer,
  // and a receiver running a local effect (PIXEL_FX) gets no pixels and a header at least once a second. No preamble,
  // no chain lock, no firmware update over this line (the receiver's own bootloader path is the Ultra line's).
  /// codegen: the line is a digital port; reset_us: the strip's reset (the latch: the hub's pixel frames' tail, and what
  /// a hand-back from a local effect waits for)
  void set_digital(uint32_t reset_us) {
    this->digital_ = true;
    this->dp_reset_us_ = reset_us;
  }
  bool is_digital() const { return this->digital_; }
  /// codegen of a stream light (ur_digital_m_stream): the pixel frame's size and the strip chip's timing. Several
  /// lights on the strip's one output are not possible: the last one wins (the schema allows one).
  void set_digital_stream(uint32_t bytes, uint32_t bit0_high_ns, uint32_t bit0_low_ns, uint32_t bit1_high_ns,
                          uint32_t bit1_low_ns, uint32_t reset_us) {
    this->px_bytes_ = bytes;
    this->px_t0h_ = bit0_high_ns;
    this->px_t0l_ = bit0_low_ns;
    this->px_t1h_ = bit1_high_ns;
    this->px_t1l_ = bit1_low_ns;
    if (reset_us > this->dp_reset_us_)
      this->dp_reset_us_ = reset_us;
  }
  /// A pixel frame is ready (copied; the newest wins while one waits for the line). Not while a local effect runs.
  void digital_pixels(const uint8_t *data);
  /// The receiver's local effect on a segment changed (PIXEL_FX queued with `effect`; 0: the stream again, at
  /// start_us on the host clock; local_frame_us: the receiver's frame on the strip, which it finishes first)
  void digital_note_fx(uint8_t segment, uint8_t effect, int64_t start_us, uint32_t local_frame_us);
  /// the user intervened (a button, the action ultra_line.retry_receiver): the back-off starts over
  void digital_retry();
  /// the receiver stopped answering: 0 fine, 1 retrying (first series), 2 waiting for the second, 3 gave up
  uint8_t digital_backoff() const { return this->dp_.bo_phase; }
  /// ... no marker frame goes out in this state (commands wait for the retry)
  bool digital_blocked() const { return ultra_dp_blocked(&this->dp_, now_us_()); }
  bool digital_local() const { return this->dp_.local != 0; }
  uint32_t digital_polls() const { return this->dp_.polls; }
  uint32_t digital_answered() const { return this->dp_.answered; }
  uint32_t digital_pixel_frames() const { return this->px_frames_; }
#endif

#ifdef USE_ULTRA_LINE_TELEMETRY
  // Diff-Power receiver telemetry (the return channel, ultra_telemetry.h; wiki: Diff-Power telemetry).
  /// uart: the UART with the hub's RS-485 receiver output on its RX (8N1 at the rate: ULTRA_RW_RATE_*); the first
  /// `receivers` boards of the line are receivers; how often each page is read.
  void set_telemetry(uart::UARTComponent *uart, uint16_t receivers, uint8_t rate, uint32_t status_ms,
                     uint32_t power_ms, uint32_t energy_ms, uint32_t supply_ms, uint32_t health_ms,
                     uint32_t lifetime_ms, uint32_t diag_ms) {
    this->rw_uart_ = uart;
    this->rate_ms_ = status_ms;
    this->poller_.init(receivers, rate,
                       TelemetryIntervals{status_ms, power_ms, energy_ms, supply_ms, health_ms, lifetime_ms, diag_ms});
    this->reply_.configure(rate, true);
  }
  /// adaptive_rate: false pins the receivers' reply rate at the UART's baud_rate (else slower on broken answers:
  /// ReplyRate)
  void set_telemetry_adaptive_rate(bool on) { this->reply_.configure(this->reply_.top(), on); }
  /// dip_warning: a receiver's supply dip is a problem (Status "Supply dips", the port's Problem) below this % of its
  /// reference; shallower ones only show in its Supply note
  void set_telemetry_dip_warning(uint8_t pct) { this->poller_.set_dip_warning(pct); }
  /// At runtime (the controller's settings, wiki: Controller-Settings): the update rate (STATUS and the live values,
  /// ms), the port's load warning / problem (%), how long a problem load lasts (ms), its limit (A), the
  /// commissioning's minimum voltage (a share of the sender's)
  void set_update_rate_ms(uint32_t ms) {
    this->rate_ms_ = ms;
    this->poller_.set_status_interval(ms);
    if (this->comm_on_)
      this->comm_.configure(ms, this->comm_min_v_);
  }
  void set_port_levels(float warning_pct, float problem_pct, uint32_t problem_ms, float limit_a) {
    PortMonitor::Config c = this->port_monitor_.config();
    c.warning_pct = std::isnan(warning_pct) ? c.warning_pct : warning_pct;
    c.problem_pct = std::isnan(problem_pct) ? c.problem_pct : problem_pct;
    c.problem_ms = problem_ms == UINT32_MAX ? c.problem_ms : problem_ms;
    c.limit_a = std::isnan(limit_a) ? c.limit_a : limit_a;
    this->port_monitor_.configure(c);
  }
  void set_commission_min_voltage(float share) {
    this->comm_min_v_ = share;
    if (this->comm_on_)
      this->comm_.configure(this->rate_ms_, share);
  }
  const ReplyRate &reply_rate() const { return this->reply_; }
  /// windows where the line was let go of late (the receivers found it still driven: no answers)
  uint32_t late_releases() const { return this->rw_late_; }
  /// the update rate (STATUS, ms): the live values' pace (update_rate: fast 1 s, normal 2 s, slow 5 s)
  uint32_t update_rate_ms() const { return this->rate_ms_; }
  /// the line's key in the lifetime ledger (its id's hash): its receivers' positions and its port's energy stay with
  /// it when its pin, names or devices change
  void set_telemetry_key(uint32_t key) { this->tel_key_ = key; }
  /// The hub's RS-485 driver enable (high: it drives the line). Low within ~10 us of an RW frame's last bit (the
  /// RMT's interrupt; on a PARLIO lane a timer alarm from the stream's timing), high again before the next frame.
  /// Without it the hardware must let go of the line itself.
  void set_telemetry_direction_pin(InternalGPIOPin *pin) { this->rw_dir_pin_ = pin; }
  /// The port's RS-485 receiver enable (high: its output drives the UART's RX): on from this line's RW frame until its
  /// window is read, off otherwise, so several lines can share one UART (one window open per UART at a time).
  void set_telemetry_receiver_enable_pin(GPIOPin *pin) { this->rw_rx_enable_pin_ = pin; }
  /// DE and /RE tied (receiver_enable_pin: direction_pin): the direction pin is the receiver enable too. It drives the
  /// line again as soon as the window is read (its receiver off), so lines can share a UART that way.
  void set_telemetry_receiver_tied(bool tied) { this->rw_rx_tied_ = tied; }
  /// receiver k has several lights: read its OUTPUTS too (their estimated power, why they're dim)
  void set_telemetry_outputs(uint16_t k, bool on) { this->poller_.set_outputs(k, on); }
  /// the sender port: its current limit (A), its voltage (V, NAN: learned), the load's warning / problem (%) and how
  /// long the problem load must last (ms; 100 % at once)
  void set_telemetry_port(float limit_a, float supply_v, float warning_pct, float problem_pct, uint32_t problem_ms) {
    this->port_monitor_.configure(PortMonitor::Config{limit_a, supply_v, warning_pct, problem_pct, problem_ms});
  }
  const PortMonitor &port_monitor() const { return this->port_monitor_; }
  bool has_telemetry() const { return this->rw_uart_ != nullptr; }
  const TelemetryPoller &telemetry() const { return this->poller_; }
  /// Every receiver's identity and FW_UPDATE page read again (a port job verifies from that fresh evidence)
  void refresh_identities() { this->poller_.invalidate_update_inventory(millis()); }
  /// the port's name (its telemetry `name:`): the logs say which receiver
  void set_telemetry_name(const char *name) { this->tel_name_ = name; }

  // The commissioning (ultra_commission.h; wiki: Commissioning)
  /// Commission (false) or Finalize (true) this line: queued, one line of the controller at a time
  void commission(bool finalize);
  /// every line with commissioning (the action without an id)
  static void commission_all(bool finalize);
  const Commissioning &commissioning() const { return this->comm_; }
  bool has_commissioning() const { return this->comm_on_; }
  /// on with commissioning: true; min_voltage: the share of the sender's voltage each receiver keeps at full load
  void set_commissioning(float min_voltage) {
    this->comm_on_ = true;
    this->comm_min_v_ = min_voltage;
  }
#ifdef USE_TIME
  /// the time for the stored dates: the commissioning's (without: "date unknown"), the receivers' Last calibrated
  void set_telemetry_clock(time::RealTimeClock *clock) { this->tel_clock_ = clock; }
#endif
  /// Each receiver's last current calibration (Calibration, Current model, Last calibrated; kept on flash)
  const CalibrationLog &calibration_log() const { return this->cal_log_; }
  /// the texts or the commissioned state changed (the line's entities)
  void add_on_commission_callback(std::function<void()> &&cb) { this->comm_cb_.add(std::move(cb)); }
  const char *telemetry_name() const { return this->tel_name_; }
  /// The port's power at the sender now (W; 0 in standby, NAN unknown) and its energy (Wh, kept in the ledger, never
  /// down; < 0 until the ledger is loaded)
  float port_power_w() const { return this->port_standby() ? 0.0f : this->port_monitor_.power_w(); }
  double port_energy_wh() const { return this->port_seeded_ ? this->port_energy_.wh : -1.0; }
  /// The ESP32's lifetime ledger (a light's estimated energy is a counter there, by its id's hash)
  static LifetimeLedger &ledger() { return ultra_ledger(); }
  /// At the shutdown (OTA, a reboot): the energies one last time
  void add_on_shutdown_callback(std::function<void()> &&callback) { this->shutdown_callback_.add(std::move(callback)); }
  /// Every line (the controller's totals)
  static const std::vector<UltraHub *> &hubs() { return hubs_; }
  /// The receiver (its slot: 0 = the first board) blinks its identify pattern for ms, then stops (a button: find it
  /// on the ceiling)
  void identify_for(uint16_t board, uint32_t ms) {
    this->sched_.identify(board, true);
    this->wake_();
    this->set_timeout(0x1D000000u | board, ms, [this, board]() {
      this->sched_.identify(board, false);
      this->wake_();
    });
  }
  /// A position's Energy (Wh: its boards' lifetime energy through the ledger, never down; < 0: not known yet)
  double position_energy_wh(uint16_t receiver) const { return this->keeper_.energy_wh(receiver); }
  /// The lifetime counters the ledger keeps for the board at a position (nullptr: not known yet)
  const LifeCounters *ledger_counters(uint16_t receiver) const { return this->keeper_.counters(receiver); }
  /// Its line's power just came back (a port's wake): not heard yet, not missed enough to count as gone (its status:
  /// Waking, no problem)
  bool rx_settling(uint16_t receiver) const;
  /// After each response window in which receiver's data changed: what changed (ultra_telemetry.h take_updates() bits);
  /// only that receiver's callbacks are called.
  void add_on_receiver_callback(uint16_t receiver, std::function<void(uint32_t)> &&callback) {
    this->rx_listeners_.add(receiver, std::move(callback));
  }
  /// once after each response window (and when the receivers lose their supply): the port's summary
  void add_on_window_callback(std::function<void()> &&callback) { this->rw_window_callback_.add(std::move(callback)); }
#endif

  // Diagnostics (sensor platform ultra_line)
  struct Diagnostics {
    size_t queue_peak;    // the most commands queued for one board, since the last call
    size_t memory_peak;   // the line's heap at its highest (state + queues), since the last call
    uint32_t build_us;    // the longest frame build (us), since the last call
    uint32_t underruns;   // RMT underruns since boot (watch_underruns)
    uint32_t frames;      // frames sent since boot
  };
  Diagnostics take_diagnostics();
  /// Count RMT underruns: the encoder notes the time of every RMT refill (~1 us each, in its interrupt).
  void watch_underruns() { this->enc_.watching = true; }

 protected:
  static uint16_t clamp_u16_(float v, uint16_t lo, uint16_t hi) {
    return v < lo ? lo : v > hi ? hi : static_cast<uint16_t>(v);
  }
  static int64_t now_us_();
  void transmit_();
  int64_t command_lead_us_() const;
  int64_t stream_lead_us_() const {
    return this->start_delay_us_ >= 0 ? this->start_delay_us_ : this->lead_us(this->max_lights_);
  }
  void service_power_();
  void service_calibration_();
  // the log of light changes: why the line's changes can't go out (ChangeHold), what a board's are doing
  ChangeLog change_log_;
  uint8_t change_hold_();
  uint8_t change_board_state_(uint16_t board) const;
  void service_change_log_();
  // the receivers' settings from Home Assistant: per board, kept on the ESP32 (RxOverride), over the YAML's values
  void rx_settings_setup_();    // in setup(): loaded, the YAML's values kept, the CONFIG ones applied
  void rx_settings_lights_();   // at the first loop (the lights are there): dim to warm
  void rx_settings_save_();
  const char *line_name_() const;   // (the log: its telemetry name, else its id)
  /// a board number from a lambda or an action (1 = the first board after the controller, as in YAML) as its slot
  /// (0-based); ALL_BOARDS (any negative) -> -1; 0 or beyond num_boards: NO_BOARD (logged, nothing is changed)
  static constexpr int NO_BOARD = -2;
  int board_slot_(int board, const char *what) const;
  static int rx_setting_key_(const char *line, const std::string &key, const std::string &value);
  std::vector<RxOverride> rx_ovr_;
  ESPPreferenceObject rx_pref_;
  uint16_t rx_yaml_[8]{};
  uint8_t rx_yaml_have_{0};
  bool rx_lights_done_{false};
  const char *line_id_{""};
  CallbackManager<void(int)> settings_cb_;
  bool any_lit_(uint32_t now) const;
  /// the port's wake timeline: a frame with a lit light went out
  void note_wake_light_();
#ifdef USE_ULTRA_LINE_TELEMETRY
  bool rw_due_(bool urgent);
  void finish_window_(bool wait);
  void rw_publish_();
  static bool rw_done_isr_(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *arg);
  /// This line's turn on its UART (lines sharing one take turns: one window at a time, the longest waiting first).
  bool rw_uart_turn_();
  /// The turn of a poll frame that won't go out (a resync, an update): let go of it (an open window keeps it).
  void rw_drop_turn_();
  /// The UART at this line's reply rate, before its window (lines sharing a UART may answer at different rates)
  void rw_uart_rate_();
  uart::UARTComponent *rw_uart_{nullptr};
  InternalGPIOPin *rw_dir_pin_{nullptr};
  ISRInternalGPIOPin rw_dir_isr_;
  GPIOPin *rw_rx_enable_pin_{nullptr};
  bool rw_turn_{false};                   // this line holds its UART: from its RW frame until its window is read
  bool rw_waiting_{false};                // ... a poll is due, the UART is another line's: it waits its turn
  uint32_t rw_wait_order_{0};             // when it started waiting (the order among the lines)
  uint32_t rw_wait_seen_ms_{0};           // its loop saw it still waiting
  static uint32_t rw_wait_seq_;
  bool rw_driving_{true};                 // the direction pin is high (the hub drives the line)
  volatile bool rw_release_{false};       // the RMT's done interrupt lets go of the line (an RW frame is on the wire)
  volatile int64_t rw_released_us_{0};
  TelemetryPoller poller_;
  ReplyRate reply_;                       // the receivers' reply rate: slower on a cable that breaks their answers
  // the commissioning: its run, the line's lights at its test level (a layer over them), its record on flash
  struct CommissionLayer final : public UltraLayer {
    UltraHub *hub{nullptr};
    void layer_shown(UltraLight *light) override;
  };
  void service_commission_();
  void commission_level_(float level);
  Commissioning comm_;
  CommissionLayer comm_layer_;
  bool comm_on_{false};
  float comm_min_v_{0.8f};
  ESPPreferenceObject comm_pref_;
  CallbackManager<void()> comm_cb_;
#ifdef USE_TIME
  time::RealTimeClock *tel_clock_{nullptr};
#endif
  uint32_t tel_unix_() const;             // the clock's unix time (0: none, or not set yet)
  // the receivers' current calibration: their records (one blob on flash), the run they saw last
  CalibrationLog cal_log_;
  ESPPreferenceObject cal_pref_;
  bool cal_was_{false};
  static UltraHub *comm_active_;          // the line whose run goes now (one at a time: the sender's supply)
  PortMonitor port_monitor_;              // the port's load and cable, after each window
  LedgerKeeper keeper_;                   // this line's receivers in the lifetime ledger (ultra_ledger.h)
  const char *tel_name_{"Port"};
  uint32_t tel_key_{0}, rate_ms_{3000};
  EnergyCounter port_energy_;             // the port's energy at the sender (its ledger counter)
  bool port_seeded_{false}, rw_rx_tied_{false};
  CallbackManager<void()> shutdown_callback_;
  void ledger_service_();                 // (the ESP32's one ledger: loaded at the first loop, stored as it changes)
  static void ledger_save_(bool now);
  void ledger_line_();                    // the ledger's own counts: the port's power-on / cut, each receiver lit
  bool line_on_{false};                   // ... the port's line powered, as last told to the keeper
  std::vector<uint8_t> lit_rx_;           // ... per receiver: any of its lights lit
  std::vector<uint8_t> rw_slots_;         // the POLL slots of the next RW frame
  std::vector<uint8_t> rw_rx_;            // the window's UART bytes
  bool rw_next_{false};                   // transmit_(): this frame is an RW frame
  bool tx_rw_{false};                     // tx_buf_ holds an RW frame
  uint32_t rw_window_us_{0};              // its window W
  bool rw_open_{false};                   // a window: nothing goes out before rw_end_us_, the UART is read at rw_read_us_
  int64_t rw_t0_us_{0}, rw_end_us_{0}, rw_read_us_{0};
  bool rw_powered_{true};
  bool rw_lock_end_{false};               // a lock frame is out: the lock interval from its end (rw_due_)
  // per receiver: RX_WOKE (its port just woke: its restart is expected), RX_WARNED (no standby support: said once)
  // RX_SETTLING (its line's power just came back: not heard, not missed enough to be gone), RX_NO_STANDBY (counted
  // in no_standby_)
  static constexpr uint8_t RX_WOKE = 1, RX_WARNED = 2, RX_SETTLING = 4, RX_NO_STANDBY = 8;
  std::vector<uint8_t> rx_note_;
#ifdef USE_DIFFPOWER
  uint8_t no_standby_{0};   // receivers on the port without standby support: its planned power-offs capped
  bool standby_held_{false};
  PowerOffCap off_cap_;
  bool standby_hold_(uint32_t now);
#endif
  uint32_t woke_ms_{0};
  uint32_t rw_late_{0};                   // windows where the line was let go of later than ULTRA_RW_RELEASE_US
  uint32_t rw_extra_us_{0};               // a command queued at a window's start waits this much longer (lead_us)
  rmt_symbol_word_t reset_symbol_{}, rw_reset_symbol_{};
  ReceiverListeners rx_listeners_;        // per receiver: its entities (add_on_receiver_callback)
  CallbackManager<void()> rw_window_callback_;
#endif

  uint8_t pin_{0};
  bool inverted_{false};
  bool use_dma_{false};
  uint16_t num_boards_{1};
  uint32_t rmt_symbols_{64};
  uint32_t descriptor_gap_us_{500};
  int64_t line_quiet_until_us_{0};
  void refresh_line_gap_();
  uint16_t effect_rate_{20};
  LineRatePolicy line_rate_;
  bool line_inventory_complete_{false}, line_rate_hold_{false};
  uint8_t port_type_{PT_UNKNOWN}, line_initial_{0};
  LineChoice line_choice_;   // the profile the line started at and why (setup)
  uint8_t line_boot_profiles_{1};
  uint16_t line_path_ns_{65535};
  std::string line_rate_reason_, line_refusal_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *line_speed_sensor_{nullptr};
#endif
  void publish_line_speed_();
  /// Why the line may not run this profile (nullptr: it may) and the board that is why (-1: none)
  const char *line_profile_refusal_(uint8_t profile, int16_t *board) const;
  bool line_profile_allowed_(uint8_t profile) const {
    int16_t board;
    return this->line_profile_refusal_(profile, &board) == nullptr;
  }
  void refuse_line_profile_(uint8_t profile, const UltraHub *who, const char *what, int16_t board);
  bool line_profile_idle_();
  void service_line_rate_();
  void apply_line_profile_(uint8_t profile);
  uint32_t bit0_high_ns_{300};
  uint32_t bit0_low_ns_{950};
  uint32_t bit1_high_ns_{950};
  uint32_t bit1_low_ns_{300};
  uint32_t reset_ns_{500000};
  uint32_t refresh_interval_ms_{200};
  int64_t start_delay_us_{-1};   // -1: auto (lead_us)
  uint8_t max_lights_{1};        // the most lights on one board of this line
  uint8_t max_commands_{1};      // the commands a group change can give the busiest board (codegen)
  static std::vector<UltraHub *> hubs_;   // every line: a group start must reach all of them
  static GroupStart group_;
  uint8_t repeats_{3};

  Scheduler sched_;
  struct DescriptorEntry { uint16_t board; std::vector<uint8_t> bytes; };
  std::vector<DescriptorEntry> descriptor_inventory_;
  std::vector<uint8_t> every_board_descriptor_;
  std::vector<uint8_t> light_counts_;   // lights per board (setup: the other indices are cleared), then freed
  struct PendingConfig {
    int board;
    uint8_t id;
    uint16_t value;
    uint8_t target{ULTRA_TARGET_BOARD};
  };
  std::vector<PendingConfig> early_config_;  // set before setup (codegen), applied in setup()
  bool started_{false};

  uint32_t last_frame_ms_{0};
  uint32_t last_busy_ms_{0};
  uint32_t frame_us_{0};    // one frame on the wire, worst case
  uint16_t preamble_{0};    // DARK frames still to send before the commands (the boards lock to Ultra)
  bool preamble_full_{true};   // the preamble running is the full 4N + 2
  bool synced_once_{false};    // a full preamble reached the boards with their supply on (they all stored Ultra)
  bool boards_powered_() const;
  bool receivers_ready_() const;
  mutable ReceiverReadiness receiver_ready_;
#ifdef USE_ULTRA_LINE_TELEMETRY
  void receiver_reply_();
#endif
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};  // next WS config frame of switch_boards_to_ultra (none pending)
  uint32_t switch_frame_us_{0};                   // its longest frame on the wire
  bool has_sent_{false};
  HighFrequencyLoopRequester high_freq_;
  bool high_freq_active_{false};
  uint32_t frames_{0};

#ifdef USE_ULTRA_LINE_TELEMETRY
  PixelCalibrationJob pixel_cal_job_;
#endif
  // current calibration: the run on this pin, its request (waiting for the boards' power), automatic runs
  CalSequencer cal_seq_;
  bool cal_pending_{false};
  bool cal_started_{false};             // the first board's test was sent (progress)
  std::vector<uint16_t> cal_req_boards_;
  uint8_t cal_req_duty_{0};
  uint32_t cal_wait_ms_{0};             // the request waits for the supply since
  bool cal_power_hold_{false};          // the supply stays requested for the run
#ifdef USE_DIFFPOWER
  diffpower::DiffPowerPort *port_{nullptr};
  bool port_attached_{false};           // its loads registered (the first loop: the lights have their states)
  uint32_t notices_{0};
  uint32_t notice_id_{0};               // a lane's notice copy: its post (counted once out on the wire; 0: none)
  uint32_t notice_epoch_{0};            // ... the stream's epoch it was posted in
  int64_t notice_end_us_{0};            // ... its last bit
  void attach_port_();
  void pump_notice_();
  void service_notice_();
#endif
  bool cal_on_boot_{false};
  bool cal_boot_done_{false};
  uint32_t cal_interval_ms_{0};
  uint32_t cal_last_ms_{0};
  bool cal_only_off_{true};
  std::vector<uint16_t> cal_boards_;

  uint32_t power_on_delay_ms_{0};
  std::vector<UltraLight *> lights_;
  std::vector<PixelSource *> pixel_lights_;   // the Diff-Power-UR-Digital-M receivers' lights (ur_digital_m)
  std::vector<uint16_t> pixel_boards_;        // ... and their boards
  bool power_waiting_{false};
  uint32_t power_on_ms_{0};
#ifdef USE_POWER_SUPPLY
  power_supply::PowerSupply *psu_{nullptr};
  power_supply::PowerSupplyRequester psu_req_;
  uint32_t psu_keep_on_ms_{0};
  bool psu_requested_{false};
  bool psu_released_{false};
  uint32_t psu_released_ms_{0};
#endif

  bool setup_rmt_();
#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
  // the digital port: the planner (ultra_dport.h) decides when a transmission may start, these send it
  void loop_digital_();
  void transmit_digital_(const ultra_dp_tx_t &tx, int64_t now);
  void digital_window_end_(int64_t now);
  void digital_window_track_();
  void digital_save_();
  bool digital_setup_();
  bool digital_{false};
  ultra_dp_t dp_{};
  uint32_t dp_reset_us_{300};
  /// the encoder's symbol table: set before a transmission, while the line is free (the encoder reads it in its
  /// interrupt, until the transmission is out)
  DpEncoder dp_enc_{};
  uint32_t rmt_res_{0};
  uint32_t px_t0h_{400}, px_t0l_{850}, px_t1h_{800}, px_t1l_{450};   // the strip chip's bit timing (ns)
  uint32_t px_word0_{0}, px_word1_{0};                               // ... as RMT words
  uint32_t px_bytes_{0};
  uint8_t *px_pending_{nullptr}, *px_tx_{nullptr};   // the newest frame, and the one being sent
  bool px_ready_{false};
  bool px_have_{false};                              // the stream light handed over a frame at least once
  uint32_t px_frames_{0};
  uint8_t fx_active_{0};                             // per segment: a local effect (the stream passes without)
  ESPPreferenceObject dp_pref_;
  bool dp_stored_{false};                            // the "gave up" flag is on flash
#endif
  uint8_t *tx_buf_{nullptr};
  bool tx_prepared_{false};      // tx_buf_ holds a frame the RMT refused: sent again first
  bool tx_normal_{true};         // it is a normal / DARK frame (retime_frame gives it a new TIME)
  bool tx_preamble_end_{false};  // it is the last frame of a full preamble
  bool tx_data_{false};          // it carries the lights' commands (not a preamble, calibration or switch frame)
  size_t tx_bytes_{0};
  EncoderArg enc_{};
  uint32_t build_peak_us_{0};   // the longest frame build since take_diagnostics()
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  /// The line takes the next frame now (the RMT is done, or the lane's mailbox is empty and its stream runs).
  bool line_free_();
  /// The line's last frame and its reset are out on the wire.
  bool line_out_();
#ifdef USE_ULTRA_PARLIO
  // a PARLIO lane instead of the RMT: frames are posted to the bus' stream a few ms ahead (ultra_parlio.h)
  UltraParlioBus *parlio_bus_{nullptr};
  ParlioLane *lane_{nullptr};
  uint32_t lane_lead_us_{0};    // how far ahead of now its frames start at most
  esp_err_t lane_post_();
#ifdef USE_ULTRA_LINE_TELEMETRY
  static void rw_lane_release_(void *arg);
  uint8_t rw_dir_gpio_{0}, rw_dir_idle_{0};   // the direction pin's GPIO and released level (the lane's release)
#endif
#endif

#ifdef USE_ULTRA_LINE_UPDATE
  bool upd_lease_{false};
  std::function<void(uint16_t)> recover_handler_;
  bool upd_power_hold_{false};
#endif
  size_t tx_buf_bytes_{0};
};

template<typename... Ts> class CalibrateCurrentAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  void set_boards(const std::vector<uint16_t> &boards) { this->boards_ = boards; }
  void set_level(uint8_t duty_pct) { this->duty_pct_ = duty_pct; }

  void play(const Ts &...x) override { this->parent_->calibrate_boards(this->boards_, this->duty_pct_); }

 protected:
  std::vector<uint16_t> boards_;
  uint8_t duty_pct_{0};
};

#ifdef USE_DIG2ANALOG_UR_DIGITAL_M
/// ultra_line.retry_receiver: a digital port polls its receiver again after it stopped answering
template<typename... Ts> class RetryReceiverAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  void play(const Ts &...x) override { this->parent_->digital_retry(); }
};
#else
template<typename... Ts> class RetryReceiverAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  void play(const Ts &...x) override {}
};
#endif

template<typename... Ts> class ShowVersionAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  TEMPLATABLE_VALUE(int, board)

  /// board: 1 = the first board (as in YAML); left out: every board
  void play(const Ts &...x) override {
    this->parent_->show_version(this->board_.has_value() ? this->board_.value(x...) : ALL_BOARDS);
  }
};

template<typename... Ts> class CancelCalibrationAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  void play(const Ts &...x) override { this->parent_->cancel_calibration(); }
};

template<typename... Ts> class SwitchBoardsToUltraAction final : public Action<Ts...>, public Parented<UltraHub> {
 public:
  TEMPLATABLE_VALUE(uint8_t, boot_protocol)

  void play(const Ts &...x) override { this->parent_->switch_boards_to_ultra(this->boot_protocol_.value(x...)); }
};

#ifdef USE_ULTRA_LINE_TELEMETRY
/// ultra_line.commission / .finalize: a line, or (no id) every line with commissioning, one after the other
template<typename... Ts> class CommissionAction final : public Action<Ts...> {
 public:
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_finalize(bool finalize) { this->finalize_ = finalize; }

  void play(const Ts &...x) override {
    if (this->hub_ != nullptr)
      this->hub_->commission(this->finalize_);
    else
      UltraHub::commission_all(this->finalize_);
  }

 protected:
  UltraHub *hub_{nullptr};
  bool finalize_{false};
};
#endif

}  // namespace esphome::ultra_line

#endif  // USE_ESP32
