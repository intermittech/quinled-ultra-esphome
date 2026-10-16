// The hub-level part of tests/ultra_parlio_test.cpp: the hub's loop(), transmit_(), lane_post_(), window, start and
// update code (build/parlio/<variant>/hub_*.inc, cut unchanged from dig2analog_ultra.cpp) with telemetry, PARLIO and
// updates on, around the real Scheduler, TelemetryPoller and the lanes of a modelled bus; RMT lines on a modelled RMT.

namespace esphome::dig2analog_ultra {
namespace hub_lanes {

#define USE_DIG2ANALOG_ULTRA_TELEMETRY
#define USE_DIG2ANALOG_ULTRA_PARLIO
#define USE_DIG2ANALOG_ULTRA_UPDATE
#define USE_DIFFPOWER
using namespace ::esphome::diffpower;
static constexpr int ESP_OK = 0;
using esp_err_t = int;
static constexpr esp_err_t ESP_ERR_TIMEOUT = 0x107;
static constexpr uint32_t BUSY_HOLD_MS = 250;
static constexpr int pdPASS = 1;
typedef struct rmt_channel_t *rmt_channel_handle_t;
struct rmt_tx_done_event_data_t {
  size_t num_symbols;
};
struct rmt_transmit_config_t {
  uint32_t loop_count;
};
struct rmt_symbol_word_t {
  uint32_t duration0 : 15;
  uint32_t level0 : 1;
  uint32_t duration1 : 15;
  uint32_t level1 : 1;
};
static void delayMicroseconds(uint32_t us) { set_clock(clock_us + us); }
static const char *esp_err_to_name(esp_err_t) { return "refused"; }
static uint32_t esp_random() { return 0x2468ACE1u; }
static int xTaskCreate(void (*)(void *), const char *, int, void *, int, void **) { return pdPASS; }   // (not run)

/// a direction or receiver enable pin, with its history
struct Pin {
  bool level{false};
  std::vector<std::pair<int64_t, bool>> log;
  void setup() {}
  void digital_write(bool v) {
    this->level = v;
    this->log.emplace_back(clock_us, v);
  }
  bool digital_read() const { return this->level; }
  void reset(bool v) {
    this->level = v;
    this->log.assign(1, {INT64_MIN, v});
  }
  /// its level at host us t
  bool at(int64_t t) const {
    bool l = false;
    for (const auto &e : this->log) {
      if (e.first > t)
        break;
      l = e.second;
    }
    return l;
  }
};
/// ESPHome's ISR pin: its argument is an allocation of its own (it may be in PSRAM: external)
struct IsrPin {
  Pin *pin{nullptr};
  bool external{false};
  void digital_write(bool v) {
    if (cache_off && this->external)
      psram_in_isr++;
    this->pin->digital_write(v);
  }
};
/// the GPIO matrix, for a raw write (gpio_ll_set_level): the pins by number
struct GpioDev {};
static GpioDev GPIO;
static Pin *gpio_pin[64];
static uint8_t gpio_next = 20;
static void gpio_ll_set_level(GpioDev *, uint32_t num, uint32_t level) {
  if (num < 64 && gpio_pin[num] != nullptr)
    gpio_pin[num]->digital_write(level != 0);
}
struct Uart {
  std::vector<uint8_t> rx;
  size_t available() const { return this->rx.size(); }
  bool read_array(uint8_t *d, size_t n) {
    if (n > this->rx.size())
      return false;
    std::memcpy(d, this->rx.data(), n);
    this->rx.erase(this->rx.begin(), this->rx.begin() + static_cast<std::ptrdiff_t>(n));
    return true;
  }
  // the reply rate (round 12): ESPHome's member and the driver's (uart_set_baudrate), per UART number
  uint32_t baud{1000000};
  uint8_t num{0};
  uint32_t get_baud_rate() const { return this->baud; }
  void set_baud_rate(uint32_t b) { this->baud = b; }
  uint8_t get_hw_serial_number() { return this->num; }
};
namespace uart {
using IDFUARTComponent = Uart;
}
using uart_port_t = int;
static uint32_t uart_hw_baud[4] = {1000000, 1000000, 1000000, 1000000};
static int uart_hw_sets = 0;
static esp_err_t uart_set_baudrate(uart_port_t n, uint32_t b) {
  uart_hw_baud[n & 3] = b;
  uart_hw_sets++;
  return ESP_OK;
}

/// RMT lines: one frame at a time at bit_ns per bit, done after its reset symbol (80 MHz ticks), then its interrupt.
struct UltraHub;
struct Rmt {
  bool busy{false};
  int64_t done_us{0};
  std::vector<uint8_t> frame;   // the last one
  int64_t t0_us{0};             // its last bit
  bool rw_pending{false};
  uint32_t bit_ns{1300};
  bool refuse_rw{false};        // the next poll frame is refused (once)
  bool refuse_rw_all{false};    // ... every one
  std::vector<std::pair<int64_t, std::vector<uint8_t>>> sent;   // (last bit us, frame)
  UltraHub *hub{nullptr};
};
static Rmt rmts[8];
static esp_err_t rmt_tx_wait_all_done(int ch, int) { return rmts[ch].busy ? -1 : ESP_OK; }
static esp_err_t rmt_transmit(int ch, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *);

// ---- a QuinLED Diff-Power port (components/diffpower): its loop(), apply_() and shutdown_all() (build/parlio/<variant>/port.inc,
// cut unchanged) on the real PortCore; its enable and detect pins, the hub's callbacks; no entities, no storage.
/// the test's time inside the port's shutdown (its delay()): the stream plays on, the wire is decoded
static std::function<void(int64_t)> advance_us;
static void delay(uint32_t ms) { advance_us(static_cast<int64_t>(ms) * 1000); }
struct AppStub {
  void feed_wdt() {}
};
static AppStub App;
struct PrefsStub {
  bool sync() { return true; }
};
static PrefsStub prefs_stub;
static PrefsStub *const global_preferences = &prefs_stub;
static constexpr uint32_t MINUTE_MS = 60000;
static constexpr uint32_t SAVE_STANDBY_S = 600;
static constexpr uint32_t SHUTDOWN_MARGIN_MS = 100;   // (diffpower.cpp: the longest notice_max_ms + this)
static constexpr uint32_t DETECT_READ_MS = 5;         // (diffpower.cpp: the detect input read this often)
static const uint32_t RESEND_MS[] = {300, 1000};

/// a light of the line, as its port sees it (UltraLight: its turn-off period, lit)
struct TestLight {
  bool lit{false};
  uint32_t get_standby_after() const { return 1000; }
  bool is_lit(uint32_t) const { return this->lit; }
  void *get_state() const { return nullptr; }
};

struct DiffPowerPort {
  struct LoadRef {
    std::function<bool(uint32_t)> lit;
    void *light;
    bool resend;
  };
  template<typename... A> struct Callbacks {
    std::vector<std::function<void(A...)>> f;
    void call(A... a) {
      for (auto &x : this->f)
        x(a...);
    }
    size_t size() const { return this->f.size(); }
  };
  // the component's (port.inc)
  void loop();
  void apply_(uint8_t ev, uint32_t now);
  static void shutdown_all();
  // as diffpower.h
  void add_load(uint32_t hold_ms, std::function<bool(uint32_t)> &&lit, void *light = nullptr, bool resend = false) {
    this->core_.add_load(hold_ms);
    this->loads_.push_back(LoadRef{std::move(lit), light, resend});
  }
  bool is_ready() const { return this->core_.ready(); }
  void add_on_ready_callback(std::function<void()> &&cb) { this->ready_cb_.f.push_back(std::move(cb)); }
  void add_on_notice_callback(std::function<void(uint8_t)> &&cb) { this->notice_cb_.f.push_back(std::move(cb)); }
  void add_on_pump_callback(std::function<void()> &&cb) { this->pump_cb_.f.push_back(std::move(cb)); }
  void add_on_prepare_callback(std::function<bool(bool)> &&cb) {
    this->prepare_cb_.push_back(std::move(cb));
    PortConfig c = this->core_.config();
    c.prepare_max_ms = PREPARE_MAX_MS;
    this->core_.configure(c);
  }
  void notice_sent() { this->core_.notice_sent(millis()); }
  void power_cycle() { this->apply_(this->core_.power_cycle(millis()), millis()); }
  uint32_t power_cycle_ms() const { return this->core_.power_cycle_ms(); }
  // stand-ins
  void update_dark_(uint32_t) {}
  void save_stats_() {}
  void publish_() {}
  void resend_() {}
  void set_timeout(uint32_t, std::function<void()> &&) {}
  PortCore core_;
  Pin *enable_pin_{nullptr}, *detect_pin_{nullptr};
  const char *name_{"Port 1"};
  std::vector<LoadRef> loads_;
  Callbacks<> ready_cb_, pump_cb_;
  Callbacks<uint8_t> notice_cb_;
  std::vector<std::function<bool(bool)>> prepare_cb_;
  bool prepare_first_{false};
  static constexpr uint32_t PREPARE_MAX_MS = 3000;
  uint32_t minute_ms_{0}, saved_standby_s_{0}, saved_cycles_{0}, saved_drops_{0};
  uint32_t detect_ms_{0};
  bool detect_raw_{true}, detect_read_{false};
  bool off_seen_{false};
  static std::vector<DiffPowerPort *> ports_;
  static bool shut_down_;
};
std::vector<DiffPowerPort *> DiffPowerPort::ports_;
bool DiffPowerPort::shut_down_{false};
#include "port.inc"

struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  enum UpdateState : uint8_t { UPD_IDLE = 0, UPD_RUNNING, UPD_FINISHED };
  void loop();                        // the component's (hub_loop.inc)
  void transmit_();                   // (hub_transmit.inc)
  esp_err_t lane_post_();
  bool rw_due_(bool urgent);          // (hub_rw.inc)
  bool rw_uart_turn_();
  void rw_drop_turn_();
  void finish_window_(bool wait);
  void rw_uart_rate_();
  void rw_publish_();
  static bool rw_done_isr_(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *arg);
  static void rw_lane_release_(void *arg);
  int64_t lead_us(size_t commands, size_t waiting = 0) const;   // (hub_start.inc)
  int64_t command_lead_us_() const;
  int64_t start_time_us(uint16_t board);
  bool start_update(const uint8_t *image, size_t size, const UpdateSession::Options &opt);   // (hub_update_start.inc)
  void resync(bool power_on = false);   // (hub_port.inc)
  bool boards_powered_() const;
  bool line_free_();
  bool line_out_();
  bool rx_settling(uint16_t receiver) const;
  void attach_port_();
  void send_power_off_notice(uint8_t copy);
  void pump_notice_();
  void service_notice_();
  bool port_standby() const { return this->port_ != nullptr && !this->port_->is_ready(); }
  static int64_t now_us_() { return clock_us; }
  // stand-ins
  struct {
    void stop() {}
  } high_freq_;
  void service_calibration_() {}
  void wake_() {}
  void loop_fast_() {}
  void arm_ping_burst_() {}
  uint32_t update_frame_us() const { return 500; }
  int64_t stream_lead_us_() const { return this->start_delay_us_ >= 0 ? this->start_delay_us_ : this->lead_us(1); }
  bool status_has_warning() const { return this->warning_; }
  void status_set_warning() { this->warning_ = true; }
  void status_clear_warning() { this->warning_ = false; }
  bool is_failed() const { return false; }
  bool update_running() const { return this->upd_state_.load() != UPD_IDLE; }
  uint32_t update_frame_min_us() const { return 500; }
  static void update_task_(void *) {}

  // the line
  uint16_t preamble_{0};
  bool preamble_full_{true}, tx_preamble_end_{false}, synced_once_{false};
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};
  uint32_t switch_frame_us_{0};
  Scheduler sched_;
  uint16_t num_boards_{2};
  std::vector<uint8_t> buf_;
  uint8_t *tx_buf_{nullptr};
  size_t tx_bytes_{0};
  bool tx_prepared_{false}, tx_normal_{true};
  uint32_t build_peak_us_{0}, frames_{0}, last_busy_ms_{0}, last_frame_ms_{0}, refresh_interval_ms_{200};
  bool high_freq_active_{false}, has_sent_{false}, started_{true}, warning_{false};
  int channel_{0}, encoder_{0};
  struct {
    FrameSymbols<rmt_symbol_word_t> symbols;
  } enc_;
  uint8_t pin_{16};
  uint32_t reset_ns_{300000};
  uint32_t frame_us_{0};
  int64_t start_delay_us_{-1};
  uint8_t max_commands_{1};
  ParlioLane *lane_{nullptr};
  uint32_t lane_lead_us_{0};
  static GroupStart group_;
  // updates
  std::atomic<uint8_t> upd_state_{UPD_IDLE};
  std::atomic<uint8_t> upd_result_{UPD_NONE};
  std::atomic<bool> upd_abort_{false};
  std::atomic<uint32_t> upd_sent_{0};
  std::atomic<uint8_t> upd_pass_{0};
  uint32_t upd_total_{0};
  void *upd_task_{nullptr};
  bool upd_quiet_{false}, upd_resync_pending_{false};
  uint32_t upd_quiet_ms_{0}, upd_confirm_ms_{0};
  UpdateResult upd_done_{UPD_NONE};
  UpdateSession upd_session_;
  uint16_t ping_left_{0};
  uint8_t dark_tail_{0};
  // the return channel
  Uart *rw_uart_{nullptr};
  Pin *rw_dir_pin_{nullptr};
  IsrPin rw_dir_isr_;
  Pin *rw_rx_enable_pin_{nullptr};
  bool rw_turn_{false}, rw_waiting_{false};
  uint32_t rw_wait_order_{0}, rw_wait_seen_ms_{0};
  static uint32_t rw_wait_seq_;
  bool rw_driving_{true};
  volatile bool rw_release_{false};
  volatile int64_t rw_released_us_{0};
  TelemetryPoller poller_;
  ReplyRate reply_;
  PortMonitor port_monitor_;
  LedgerKeeper keeper_;   // (not attached: nothing to prepare; tests/ultra_telemetry_test.cpp runs it)
  EnergyCounter port_energy_;
  bool port_seeded_{false}, rw_rx_tied_{false};
  float port_power_w() const { return this->port_standby() ? 0.0f : this->port_monitor_.power_w(); }
  const char *tel_name_{"Port"};
  void ledger_service_() {}
  void ledger_line_() {}
  static void ledger_save_(bool) {}
  std::vector<uint8_t> rw_slots_, rw_rx_;
  bool rw_next_{false}, tx_rw_{false};
  uint32_t rw_window_us_{0};
  bool rw_open_{false};
  int64_t rw_t0_us_{0}, rw_end_us_{0}, rw_read_us_{0};
  bool rw_powered_{true};
  bool rw_lock_end_{false};
  static constexpr uint8_t RX_WOKE = 1, RX_WARNED = 2, RX_SETTLING = 4, RX_NO_STANDBY = 8;
  std::vector<uint8_t> rx_note_;
  uint8_t no_standby_{0};   // (receivers without standby support: the port's planned power-offs capped)
  PowerOffCap off_cap_;
  bool standby_hold_(uint32_t now) { return this->no_standby_ != 0 && this->off_cap_.hold(now); }
  uint32_t woke_ms_{0};
  uint32_t rw_late_{0}, rw_extra_us_{0};
  // the Diff-Power port
  DiffPowerPort *port_{nullptr};
  bool port_attached_{false};
  uint32_t notices_{0};
  uint32_t notice_id_{0}, notice_epoch_{0};
  int64_t notice_end_us_{0};
  uint8_t rw_dir_gpio_{0}, rw_dir_idle_{0};
  std::vector<TestLight *> lights_;
  bool cal_power_hold_{false}, upd_power_hold_{false};
  enum PowerCycle : uint8_t { CYCLE_NONE = 0, CYCLE_WAIT_OFF, CYCLE_OFF, CYCLE_DONE, CYCLE_FAILED };
  PowerCycle cycle_{CYCLE_NONE};
  rmt_symbol_word_t reset_symbol_{}, rw_reset_symbol_{};
  ReceiverListeners rx_listeners_;   // (ultra_telemetry.h's: per receiver)
  struct {
    int calls{0};
    void call() { this->calls++; }
  } rw_window_callback_;
  static std::vector<UltraHub *> hubs_;
  uint32_t windows{0};       // (the test's) windows read
  int64_t last_window_us{0}, longest_wait_us{0};

  /// as setup(): on a lane of the bus with the timing on its grid, or on an RMT channel; telemetry with `receivers`
  /// receivers when uart is given (a direction pin, a receiver enable pin)
  void start(ParlioLane *lane, uint32_t b0h, uint32_t b0l, uint32_t b1h, uint32_t b1l, uint32_t reset_ns, Uart *uart,
             Pin *dir, Pin *re, uint32_t seed, uint16_t receivers = 2, uint8_t rate = ULTRA_RW_RATE_1M,
             TelemetryIntervals iv = TelemetryIntervals{}) {
    this->sched_.init(this->num_boards_, seed, 3);
    this->sched_.set_bit_timing(b0h, b0l, b1h, b1l);
    this->reset_ns_ = reset_ns;
    this->buf_.assign(std::max<size_t>(this->sched_.frame_bytes(), UPD_FRAME_BYTES), 0);
    this->tx_buf_ = this->buf_.data();
    this->frame_us_ = static_cast<uint32_t>(this->sched_.frame_bytes() * 8 * std::max(b0h + b0l, b1h + b1l) / 1000 +
                                            reset_ns / 1000);
    rmt_symbol_word_t rst{}, rw{};
    rst.duration0 = rst.duration1 = 12000;   // 300 us at 80 MHz
    rw.duration0 = rw.duration1 = 80;        // 2 us
    this->enc_.symbols.reset = rst;
    this->reset_symbol_ = rst;
    this->rw_reset_symbol_ = dir != nullptr ? rw : rst;
    this->lane_ = lane;
    if (lane != nullptr) {
      lane->begin(this->buf_.size(), UltraHub::rw_lane_release_, this);
      this->lane_lead_us_ = lane->lead_us();
    }
    this->rw_uart_ = uart;
    this->rw_dir_pin_ = dir;
    if (dir != nullptr) {
      dir->reset(true);
      this->rw_dir_isr_.pin = dir;
      this->rw_dir_isr_.external = true;   // (as if in PSRAM: the cache-off interrupt must not touch it)
      if (gpio_next >= 64)
        gpio_next = 20;
      this->rw_dir_gpio_ = gpio_next++;
      gpio_pin[this->rw_dir_gpio_] = dir;
      this->rw_dir_idle_ = 0;
    }
    this->rw_rx_enable_pin_ = re;
    if (re != nullptr)
      re->reset(false);
    if (uart != nullptr) {
      this->poller_.init(receivers, rate, iv);
      this->rw_slots_.assign(ULTRA_SLOT_BYTES * this->num_boards_, 0);
      this->rw_extra_us_ = this->poller_.max_window_us() + ULTRA_RW_HOLDOFF_US;
    }
    this->preamble_ = 3;   // the start-up preamble
    for (uint16_t r = 0; r < this->poller_.receivers(); r++)
      this->rx_listeners_.add(r, [](uint32_t) {});
    hubs_.push_back(this);
  }
  // the commissioning (ultra_commission.h: tests/ultra_telemetry_test.cpp runs it)
  bool comm_on_{false};
  void service_commission_() {}
  Commissioning comm_;
};
std::vector<UltraHub *> UltraHub::hubs_;
uint32_t UltraHub::rw_wait_seq_ = 0;
GroupStart UltraHub::group_;
static constexpr uint32_t RW_WAIT_STALE_MS = 100;

#include "hub_loop.inc"
#include "hub_transmit.inc"
#include "hub_rw.inc"
#include "hub_start.inc"
#include "hub_update_start.inc"
#include "hub_port.inc"

/// The receivers of a line answer an RW frame's POLLs (a STATUS whose voltage and chip ids are the line's own,
/// IDENTITY without optional pages, the rest unsupported; power: with power events, POWER and OUTPUTS answered): the
/// bytes a window brings to the UART.
static std::vector<uint8_t> answers(const uint8_t *frame, uint16_t boards, int line, bool power) {
  std::vector<uint8_t> out{0x00};   // the turnaround checks' junk
  for (uint16_t b = 0; b < boards; b++) {
    const uint8_t *s = frame + ULTRA_SLOT_BYTES * b;
    if (ultra_poll_valid(s) != 0 || s[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY)
      continue;
    uint8_t a[32]{}, d[24]{};
    uint8_t page = s[ULTRA_POLL_PAGE];
    if (page == ULTRA_PAGE_STATUS) {
      ultra_put24(d + ULTRA_ST_FW_VERSION, 0x000608);
      ultra_put24(d + ULTRA_ST_UPTIME, static_cast<uint32_t>(clock_us / 1000000));
      ultra_put16(d + ULTRA_ST_VBUS, static_cast<uint16_t>(2400 + 50 * line));   // line k: 24 V + 0.5 V x k
      ultra_put16(d + ULTRA_ST_MCU_TEMP, ULTRA_ST_NA);
      d[ULTRA_ST_MEASURED] = ULTRA_ST_M_VBUS;
    } else if (page == ULTRA_PAGE_IDENTITY) {
      d[0] = static_cast<uint8_t>(b + 1);
      if (power)
        ultra_put32(d + ULTRA_ID_FEATURES, ULTRA_FEAT_POWER_EVENTS);
    } else if (power && (page == ULTRA_PAGE_POWER || page == ULTRA_PAGE_OUTPUTS)) {
      // (nothing to report)
    } else {
      page = ULTRA_PAGE_UNSUPPORTED;
    }
    a[0] = ULTRA_RSP_SYNC;
    a[1] = page;
    a[2] = static_cast<uint8_t>(s[ULTRA_POLL_INDEX] << 4);
    a[3] = s[2];
    ultra_put16(a + 4, static_cast<uint16_t>(0x1000 + 0x10 * line + b));
    std::memcpy(a + ULTRA_RSP_DATA, d, 24);
    ultra_put16(a + 30, ultra_crc16(a, 30));
    out.push_back(ULTRA_RSP_LEAD);
    out.insert(out.end(), a, a + 32);
  }
  return out;
}
/// R and the rate of an RW frame's POLLs
static void poll_plan(const uint8_t *frame, uint16_t boards, uint8_t *r, uint8_t *rate) {
  *r = 0;
  *rate = ULTRA_RW_RATE_1M;
  for (uint16_t b = 0; b < boards; b++) {
    const uint8_t *s = frame + ULTRA_SLOT_BYTES * b;
    if (ultra_poll_valid(s) == 0) {
      *r = s[ULTRA_POLL_SLOTS];
      *rate = s[ULTRA_POLL_RATE];
      return;
    }
  }
}

/// The return channel of every line: an RW frame's receivers turn around only when the hub let go of the line by
/// ULTRA_RW_RELEASE_US after its last bit (the pin's history); their answers reach the line's UART while its receiver
/// is enabled (no receiver_enable_pin: always); two ports of one UART enabled at once garble each other's bytes.
struct Telemetry {
  struct Window {
    int line;
    int64_t t0_us, end_us;
    std::vector<uint8_t> frame;
    bool turned{false};   // the receivers turned around (the hub had let go of the line)
  };
  std::vector<Window> windows;       // every RW frame seen on a wire
  std::vector<Window> pending;       // its answers still to come
  int uart_of[16]{};
  Uart *uarts[8]{};
  Pin *re[16]{}, *dir[16]{};
  uint16_t boards_of[16]{};
  bool on[16];
  int collisions{0}, late{0}, answered{0}, max_enabled{0};
  bool power{false};   // the receivers report power events (POWER and OUTPUTS pages)
  Telemetry() {
    std::fill(std::begin(this->on), std::end(this->on), true);
    std::fill(std::begin(this->boards_of), std::end(this->boards_of), 2);
  }

  /// an RW frame's last bit on line `line` at t0_us (seen on the wire, maybe a little later)
  void rw_frame(int line, int64_t t0_us, const std::vector<uint8_t> &frame) {
    uint8_t r, rate;
    poll_plan(frame.data(), this->boards_of[line], &r, &rate);
    const int64_t end = t0_us + ultra_rw_window_us(r, rate);
    this->windows.push_back(Window{line, t0_us, end, frame});
    this->pending.push_back(Window{line, t0_us, end, frame});
  }
  /// now: the receivers enabled per UART; the answers of windows at their last burst
  void tick() {
    int enabled[8]{};
    for (int k = 0; k < 16; k++) {
      if (this->re[k] != nullptr && this->re[k]->level)
        enabled[this->uart_of[k]]++;
    }
    for (int u = 0; u < 8; u++)
      this->max_enabled = std::max(this->max_enabled, enabled[u]);
    for (size_t i = 0; i < this->pending.size();) {
      Window &w = this->pending[i];
      // the turnaround check: the hub let go of the line by ULTRA_RW_RELEASE_US after the frame (its pin's history)
      if (!w.turned && clock_us >= w.t0_us + ULTRA_RW_RELEASE_US) {
        if (this->dir[w.line] != nullptr && this->dir[w.line]->at(w.t0_us + ULTRA_RW_RELEASE_US)) {
          this->late++;
          this->pending.erase(this->pending.begin() + static_cast<std::ptrdiff_t>(i));
          continue;
        }
        w.turned = true;
      }
      if (!w.turned || clock_us < w.end_us - 50) {
        i++;
        continue;
      }
      const int u = this->uart_of[w.line];
      std::vector<uint8_t> bytes = answers(w.frame.data(), this->boards_of[w.line], w.line, this->power);
      const bool heard = this->re[w.line] == nullptr || this->re[w.line]->level;
      if (heard && this->on[w.line]) {
        if (enabled[u] > 1) {   // another port drives the same RX
          for (auto &b : bytes)
            b ^= 0x5A;
          this->collisions++;
        }
        this->uarts[u]->rx.insert(this->uarts[u]->rx.end(), bytes.begin(), bytes.end());
        this->answered++;
      }
      this->pending.erase(this->pending.begin() + static_cast<std::ptrdiff_t>(i));
    }
  }
  /// windows of lines on one UART that overlap, and of lines on different UARTs
  void overlaps(int *same, int *across) const {
    *same = *across = 0;
    for (size_t i = 0; i < this->windows.size(); i++) {
      for (size_t j = i + 1; j < this->windows.size(); j++) {
        const Window &a = this->windows[i], &b = this->windows[j];
        if (a.line == b.line || a.t0_us >= b.end_us || b.t0_us >= a.end_us)
          continue;
        (this->uart_of[a.line] == this->uart_of[b.line] ? *same : *across) += 1;
      }
    }
  }
};
static Telemetry tel;

/// 16 lanes, 4 UARTs with 4 lanes each (each lane with its direction and receiver enable pin, 2 receivers): the
/// hubs' own code polls; every RW frame is decoded from the rendered stream and answered through the modelled
/// transceivers. Windows on one UART never overlap (and never 2 receivers enabled on one), windows of different UARTs
/// do; every receiver answers with its own line's values; the direction pin drops within 10 us of each RW frame's
/// last bit; each lane is quiet for its windows while others send; every frame's TIME is its last edge; a lane whose
/// receivers go silent leaves the others answering. A late render now and then kills the stream: the lines go on.
static void test_sixteen_lanes() {
  set_clock(1000000);
  psram_in_isr = 0;
  UltraHub::hubs_.clear();
  tel = Telemetry();
  static Stream bus;
  const Grid g = default_grid(16);
  renew(bus);
  bus.init(g, 16, 96, 16, 0, clock_us * 1000 + 321);
  bus.late = [](int64_t k) -> int64_t { return k % 1500 == 777 ? 1200000 : 0; };   // a kill now and then
  const uint32_t sample_ns = g.sample_ns, P = g.period;
  const uint32_t group_ns = 8 * P * sample_ns;
  const uint32_t reset_ns = (300000 + group_ns - 1) / group_ns * group_ns;   // the reset in whole groups
  static UltraHub hubs[16];
  static Uart uarts[4];
  static Pin dir[16], re[16];
  for (int k = 0; k < 16; k++) {
    renew(hubs[k]);
    hubs[k].pin_ = static_cast<uint8_t>(20 + k);
    uarts[k / 4] = Uart();
    hubs[k].start(bus.s.lane(static_cast<uint8_t>(k)), g.lane[k].h0 * sample_ns, (P - g.lane[k].h0) * sample_ns,
                  g.lane[k].h1 * sample_ns, (P - g.lane[k].h1) * sample_ns, reset_ns, &uarts[k / 4], &dir[k], &re[k],
                  100 + k);
    tel.uart_of[k] = k / 4;
    tel.re[k] = &re[k];
    tel.dir[k] = &dir[k];
  }
  for (int u = 0; u < 4; u++)
    tel.uarts[u] = &uarts[u];
  Decoder dec[16];
  size_t decoded = 0;
  int64_t time_worst = 0;
  size_t ultra_frames = 0;
  bool frame_ok = true;
  struct Quiet {
    int lane;
    int64_t from_s, to_s;
  };
  std::vector<Quiet> quiet;   // each RW frame's window on its lane (samples)

  auto run = [&](int64_t us) {
    const int64_t end = clock_us + us;
    while (clock_us < end) {
      const int64_t next = clock_us + 50;
      bus.run_until(next * 1000);
      set_clock(next);
      // the wire so far, as the boards see it
      for (size_t s = std::max<size_t>(decoded, 1); s < bus.wire.size(); s++) {
        const uint16_t diff = static_cast<uint16_t>(bus.wire[s] ^ bus.wire[s - 1]);
        for (int l = 0; diff != 0 && l < 16; l++) {
          if (diff & (1u << l))
            dec[l].edge(bus.level(static_cast<uint8_t>(l), s), static_cast<int64_t>(s), sample_ns);
        }
      }
      decoded = bus.wire.size();
      for (int l = 0; l < 16; l++) {
        while (dec[l].end(static_cast<int64_t>(decoded), sample_ns)) {
          const Decoder::Frame &f = dec[l].frames.back();
          if (!valid_ultra(f.bytes) || f.bytes.size() != hubs[l].sched_.frame_bytes())
            continue;   // (a frame a kill cut: the boards drop it)
          const int64_t t0_ns = bus.time_of(f.t0);
          uint32_t t = 0;
          const uint8_t *h = f.bytes.data() + hubs[l].sched_.header_offset();
          frame_ok &= ultra_parse_header(h, &t) == 0;
          const int64_t d = static_cast<int64_t>(static_cast<int32_t>(t - static_cast<uint32_t>(t0_ns / 1000))) * 1000 -
                            (t0_ns % 1000);
          time_worst = std::max<int64_t>(time_worst, d < 0 ? -d : d);
          ultra_frames++;
          if (ultra_header_flags(h) & ULTRA_HDR_RW) {
            tel.rw_frame(l, t0_ns / 1000, f.bytes);
            uint8_t r, rate;
            poll_plan(f.bytes.data(), 2, &r, &rate);
            quiet.push_back(Quiet{l, f.t0, f.t0 + static_cast<int64_t>((ultra_rw_window_us(r, rate) + ULTRA_RW_HOLDOFF_US) *
                                                                         1000ULL / sample_ns)});
          }
        }
      }
      tel.tick();
      for (auto &h : hubs)
        h.loop();
    }
  };

  run(8000000);
  bool all = true, own = true;
  for (int k = 0; k < 16; k++) {
    const TelemetryPoller &p = hubs[k].poller_;
    all &= p.answering(0) && p.answering(1);
    own &= std::fabs(telemetry_value(p.data(0), TV_SUPPLY_VOLTAGE) - (24.0f + 0.5f * k)) < 1e-3f &&
           p.uid16(0) == 0x1000 + 0x10 * k && p.uid16(1) == 0x1001 + 0x10 * k && p.stats().stale == 0;
  }
  int same = 0, across = 0;
  tel.overlaps(&same, &across);
  CHECK(all, "16 lanes: every lane's 2 receivers answer (%d windows answered)", tel.answered);
  CHECK(own, "each lane's poller holds only its own receivers (their voltages, chip ids), nothing stale");
  CHECK(same == 0 && tel.max_enabled <= 1 && tel.collisions == 0, "windows of lanes sharing a UART never overlap "
        "(%d), never two receivers enabled on one UART (at most %d), no garbled answers (%d)", same, tel.max_enabled,
        tel.collisions);
  CHECK(across > 0, "windows of lanes on different UARTs overlap (%d pairs): the 4 groups answer at the same time",
        across);
  // the direction pins: each RW frame on the wire released 2-10 us after its last bit; no release without one
  int64_t worst = 0, least = INT64_MAX;
  size_t matched = 0, stray = 0;
  for (const Stream::Fired &f : bus.fired) {
    bool hit = false;
    for (const Telemetry::Window &w : tel.windows) {
      const int64_t d = f.at - w.t0_us * 1000;
      if (w.line == f.lane && d >= 0 && d <= 11000) {
        hit = true;
        worst = std::max(worst, d);
        least = std::min(least, d);
        break;
      }
    }
    matched += hit;
    stray += !hit;
  }
  CHECK(tel.late == 0 && stray == 0 && matched + 2 >= tel.windows.size() && least >= 1000 && worst <= 11000,
        "every RW frame's direction pin dropped %.1f-%.1f us after its last bit (%u of %u), none late, none for a "
        "frame that didn't complete (%u)", least / 1000.0, worst / 1000.0, static_cast<unsigned>(matched),
        static_cast<unsigned>(tel.windows.size()), static_cast<unsigned>(stray));
  // each lane quiet for its windows, the others sending
  bool lane_quiet = true;
  size_t others = 0;
  for (const Quiet &q : quiet) {
    for (int64_t s = q.from_s; s < q.to_s && s < static_cast<int64_t>(bus.wire.size()); s++) {
      lane_quiet &= bus.level(static_cast<uint8_t>(q.lane), static_cast<size_t>(s)) == 0;
      others += s > q.from_s && (bus.wire[s] ^ bus.wire[s - 1]) != 0;
    }
  }
  CHECK(lane_quiet && others > 1000, "%u windows: their lane low throughout, the other lanes sending (%u edges)",
        static_cast<unsigned>(quiet.size()), static_cast<unsigned>(others));
  CHECK(frame_ok && time_worst <= 1500 + bus.start_hi_ns && ultra_frames > 500, "%u Ultra frames decoded from the stream: valid headers, "
        "TIME their last edge within %lld ns", static_cast<unsigned>(ultra_frames), static_cast<long long>(time_worst));
  // lane 5's receivers go silent: only they are unavailable
  tel.on[5] = false;
  run(6000000);
  bool others_ok = true;
  for (int k = 0; k < 16; k++) {
    if (k != 5)
      others_ok &= hubs[k].poller_.answering(0) && hubs[k].poller_.answering(1);
  }
  CHECK(!hubs[5].poller_.answering(0) && !hubs[5].poller_.answering(1) && others_ok,
        "lane 5's receivers silent: only they are unavailable, its UART's 3 other lanes and the rest answer");
  tel.on[5] = true;
  run(6000000);
  CHECK(hubs[5].poller_.answering(0) && hubs[5].poller_.answering(1), "and back");
  tel.overlaps(&same, &across);
  CHECK(same == 0 && tel.max_enabled <= 1, "still no overlap on a UART after it (%d)", same);
  CHECK(psram_in_isr == 0, "the timer's interrupt (cache off) let go of the direction pins without touching the "
        "ISR pins' own allocations (%d times)", psram_in_isr);
  CHECK(bus.restarts >= 10 && bus.replays == 0, "through %lld kills of the stream, no buffer played twice",
        static_cast<long long>(bus.restarts));
  std::printf("  16 lanes, 4 UARTs x 4: %u RW windows (%d pairs overlapping across UARTs, none on one), %u Ultra frames, "
              "%lld kills, direction pins %.1f-%.1f us after the last bit\n", static_cast<unsigned>(tel.windows.size()),
              across, static_cast<unsigned>(ultra_frames), static_cast<long long>(bus.restarts), least / 1000.0,
              worst / 1000.0);
  UltraHub::hubs_.clear();
}

static esp_err_t rmt_transmit(int ch, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  Rmt &rmt = rmts[ch];
  UltraHub *h = rmt.hub;
  const uint8_t *hd = data + n - ULTRA_HDR_BYTES;
  const bool rw = n == h->sched_.frame_bytes() && (ultra_header_flags(hd) & ULTRA_HDR_RW);
  if (rw && (rmt.refuse_rw || rmt.refuse_rw_all)) {
    rmt.refuse_rw = false;
    return -1;
  }
  const uint32_t ticks = h->enc_.symbols.reset.duration0 + h->enc_.symbols.reset.duration1;
  const int64_t bits_us = static_cast<int64_t>(n) * 8 * rmt.bit_ns / 1000;
  rmt.busy = true;
  rmt.t0_us = clock_us + bits_us;
  rmt.done_us = rmt.t0_us + ticks / 80;
  rmt.frame.assign(data, data + n);
  rmt.rw_pending = rw;
  rmt.sent.emplace_back(rmt.t0_us, rmt.frame);
  return ESP_OK;
}

/// RMT lines, their frames and done interrupts, every 20 us
static void run_rmt(UltraHub *hubs, int n, int64_t us, DiffPowerPort *port = nullptr) {
  const int64_t end = clock_us + us;
  while (clock_us < end) {
    set_clock(clock_us + 20);
    for (int k = 0; k < n; k++) {
      Rmt &r = rmts[k];
      if (r.rw_pending && clock_us >= r.t0_us) {
        r.rw_pending = false;
        tel.rw_frame(k, r.t0_us, r.frame);
      }
      if (r.busy && clock_us >= r.done_us) {
        r.busy = false;
        UltraHub::rw_done_isr_(nullptr, nullptr, &hubs[k]);   // the RMT's done interrupt
      }
    }
    tel.tick();
    for (int k = 0; k < n; k++) {
      const uint32_t built = hubs[k].poller_.stats().windows;
      hubs[k].loop();
      if (k == 0 && port != nullptr)
        port->loop();
      if (hubs[k].poller_.stats().windows != built) {   // an RW frame (its window) went out
        hubs[k].windows++;
        hubs[k].last_window_us = clock_us;
      }
    }
  }
}

/// RMT lines with the same turn-taking: 3 lines share one UART (each with a receiver enable pin), a 4th has its own
/// (no receiver enable pin): never 2 receivers enabled on the shared UART, no overlapping windows there, every line's
/// receivers answer with their own values.
static void test_rmt_shared_uart() {
  set_clock(50000000);
  UltraHub::hubs_.clear();
  tel = Telemetry();
  static UltraHub hubs[4];
  static Uart uarts[2];
  static Pin dir[4], re[4];
  for (int k = 0; k < 4; k++) {
    renew(hubs[k]);
    hubs[k].channel_ = k;
    hubs[k].pin_ = static_cast<uint8_t>(16 + k);
    rmts[k] = Rmt();
    rmts[k].hub = &hubs[k];
    const bool shared = k < 3;
    hubs[k].start(nullptr, 250, 1050, 700, 650, 300000, &uarts[shared ? 0 : 1], &dir[k], shared ? &re[k] : nullptr,
                  200 + k);
    tel.uart_of[k] = shared ? 0 : 1;
    tel.re[k] = shared ? &re[k] : nullptr;
    tel.dir[k] = &dir[k];
  }
  tel.uarts[0] = &uarts[0];
  tel.uarts[1] = &uarts[1];
  run_rmt(hubs, 4, 10000000);
  bool all = true, own = true;
  for (int k = 0; k < 4; k++) {
    const TelemetryPoller &p = hubs[k].poller_;
    all &= p.answering(0) && p.answering(1);
    own &= std::fabs(telemetry_value(p.data(0), TV_SUPPLY_VOLTAGE) - (24.0f + 0.5f * k)) < 1e-3f &&
           p.uid16(1) == 0x1001 + 0x10 * k && p.stats().stale == 0;
  }
  int same = 0, across = 0;
  tel.overlaps(&same, &across);
  CHECK(all && own, "RMT lines: 3 sharing a UART and one on its own, every receiver answers with its own values");
  CHECK(same == 0 && tel.max_enabled <= 1 && tel.collisions == 0 && tel.late == 0, "the shared UART: one window at a "
        "time (%d overlaps), never 2 receivers enabled (%d), nothing garbled (%d)", same, tel.max_enabled,
        tel.collisions);
  CHECK(across > 0, "the line with its own UART answers alongside the shared ones (%d overlaps)", across);
  std::printf("  RMT lines: 3 on one UART (receiver enable pins) + 1 on its own: %u windows, none overlapping on the "
              "shared UART\n", static_cast<unsigned>(tel.windows.size()));
  UltraHub::hubs_.clear();
}

/// Starvation: line 0 on a shared UART with 64 receivers, STATUS and POWER every second, OUTPUTS,
/// answers at 250 kbaud and 10.25 us bits: its polls are always overdue (a window takes ~200 ms, 16 answers; it needs
/// 12 a second). Line 1 (2 receivers, STATUS and POWER every second) shares its UART: it gets its turn when line 0's window
/// ends, its receivers answer, it never waits much more than its own interval and one of line 0's windows.
static void test_fair_uart() {
  set_clock(90000000);
  UltraHub::hubs_.clear();
  tel = Telemetry();
  static UltraHub hubs[2];
  static Uart uart;
  static Pin dir[2], re[2];
  uart = Uart();
  for (int k = 0; k < 2; k++) {
    renew(hubs[k]);
    hubs[k].channel_ = k;
    rmts[k] = Rmt();
    rmts[k].hub = &hubs[k];
    tel.uart_of[k] = 0;
    tel.re[k] = &re[k];
    tel.dir[k] = &dir[k];
  }
  tel.uarts[0] = &uart;
  tel.power = true;
  hubs[0].num_boards_ = 64;
  hubs[0].refresh_interval_ms_ = 600000;   // (no refresh frames between its polls: back to back, the worst case)
  tel.boards_of[0] = 64;
  rmts[0].bit_ns = 10250;
  TelemetryIntervals busy;
  busy.status_ms = 1000;
  busy.power_ms = 1000;
  hubs[0].start(nullptr, 250, 10000, 700, 9550, 300000, &uart, &dir[0], &re[0], 300, 64, ULTRA_RW_RATE_250K, busy);
  for (uint16_t r = 0; r < 64; r++)
    hubs[0].poller_.set_outputs(r, true);
  hubs[1].start(nullptr, 250, 1050, 700, 650, 300000, &uart, &dir[1], &re[1], 301, 2, ULTRA_RW_RATE_1M, busy);
  int64_t longest = 0, last = clock_us + 1000000;
  uint32_t w1 = 0;
  for (int t = 0; t < 600; t++) {   // 60 s
    run_rmt(hubs, 2, 100000);
    if (hubs[1].windows != w1) {
      longest = std::max(longest, hubs[1].last_window_us - last);
      last = hubs[1].last_window_us;
      w1 = hubs[1].windows;
    }
  }
  longest = std::max(longest, clock_us - last);
  int same = 0, across = 0;
  tel.overlaps(&same, &across);
  CHECK(hubs[0].windows > 200 && hubs[1].windows >= 60 && longest < 1500000 && hubs[1].poller_.answering(0) &&
            hubs[1].poller_.answering(1) && same == 0,
        "a busy line sharing the UART (%u windows in 60 s) leaves the other its turns: %u windows, at most %.2f s "
        "between them, its receivers answering", hubs[0].windows, hubs[1].windows, longest / 1e6);
  std::printf("  a busy line on a shared UART: %u windows in 60 s, the other line %u (at most %.2f s apart)\n",
              hubs[0].windows, hubs[1].windows, longest / 1e6);
  UltraHub::hubs_.clear();
}

/// A poll frame that the transmitter refused keeps its line's UART turn (it goes again next loop); an update session
/// starting then drops that frame, and the turn with it: the other line on the UART keeps polling through the session,
/// the first line's receiver is off.
static void test_failed_poll_then_update() {
  set_clock(200000000);
  UltraHub::hubs_.clear();
  tel = Telemetry();
  static UltraHub hubs[2];
  static Uart uart;
  static Pin dir[2], re[2];
  uart = Uart();
  for (int k = 0; k < 2; k++) {
    renew(hubs[k]);
    hubs[k].channel_ = k;
    rmts[k] = Rmt();
    rmts[k].hub = &hubs[k];
    TelemetryIntervals iv;
    iv.status_ms = 1000;
    hubs[k].start(nullptr, 250, 1050, 700, 650, 300000, &uart, &dir[k], &re[k], 400 + k, 2, ULTRA_RW_RATE_1M, iv);
    tel.uart_of[k] = 0;
    tel.re[k] = &re[k];
    tel.dir[k] = &dir[k];
  }
  tel.uarts[0] = &uart;
  run_rmt(hubs, 2, 3000000);
  // line 0's next poll frame is refused: it keeps the turn, its receiver on
  rmts[0].refuse_rw = true;
  for (int t = 0; t < 400000 && !(hubs[0].rw_turn_ && hubs[0].tx_prepared_ && !hubs[0].rw_open_); t++)
    run_rmt(hubs, 2, 20);
  CHECK(hubs[0].rw_turn_ && hubs[0].tx_prepared_ && hubs[0].tx_rw_ && re[0].level, "a refused poll frame keeps its "
        "line's turn and receiver for the retry");
  static uint8_t img[UPD_DESC_BYTES + 512];
  for (size_t i = 0; i < sizeof(img); i++)
    img[i] = static_cast<uint8_t>(i * 5 + 1);
  const bool started = hubs[0].start_update(img, sizeof(img), UpdateSession::Options());
  CHECK(started && !hubs[0].rw_turn_ && !re[0].level, "the update session drops the poll frame and its turn (%d %d %d)",
        started, hubs[0].rw_turn_, re[0].level);
  const uint32_t before = hubs[1].windows;
  run_rmt(hubs, 2, 5000000);   // (line 0's session runs meanwhile: its loop returns at once)
  CHECK(hubs[1].windows >= before + 3 && hubs[1].poller_.answering(0), "the other line on the UART polls on through "
        "the session (%u windows in 5 s)", hubs[1].windows - before);
  hubs[0].upd_state_.store(UltraHub::UPD_IDLE);
  UltraHub::hubs_.clear();
}

/// A group change on two lines at once, an RMT line and a lane whose bus has 10 ms buffers, each line with 5 lights on
/// one board (5 commands per change): the automatic start is late enough for the lane's cadence (its mailbox takes a
/// frame at most every few chunks), so every command's first copy reaches its board before it starts, on both lines,
/// and both start at the same moment.
static void test_group_cadence() {
  set_clock(300000000);
  UltraHub::hubs_.clear();
  UltraHub::group_ = GroupStart();
  static Stream bus;
  renew(bus);
  const Grid g = default_grid(16);
  bus.init(g, 16, 928, 16, 0, clock_us * 1000);   // ~10 ms buffers
  static UltraHub hubs[2];
  for (int k = 0; k < 2; k++) {
    renew(hubs[k]);
    hubs[k].num_boards_ = 1;
    hubs[k].max_commands_ = 5;
    hubs[k].channel_ = k;
    rmts[k] = Rmt();
    rmts[k].hub = &hubs[k];
  }
  hubs[0].start(nullptr, 250, 1050, 700, 650, 300000, nullptr, nullptr, nullptr, 500);
  const uint32_t group_ns = 8 * g.period * g.sample_ns;
  hubs[1].start(bus.s.lane(0), 250, (g.period - 5) * 50, 700, (g.period - 14) * 50,
                (300000 + group_ns - 1) / group_ns * group_ns, nullptr, nullptr, nullptr, 501);
  auto run = [&](int64_t us) {
    const int64_t end = clock_us + us;
    while (clock_us < end) {
      const int64_t next = clock_us + 20;
      bus.run_until(next * 1000);
      set_clock(next);
      Rmt &r = rmts[0];
      if (r.busy && clock_us >= r.done_us)
        r.busy = false;
      for (auto &h : hubs)
        h.loop();
    }
  };
  run(2000000);   // the preambles are out
  int64_t starts[2][5];
  const uint16_t a[CHANNELS] = {100, 200, 300, 400, 500}, b[CHANNELS] = {65535, 65535, 65535, 65535, 65535};
  for (int light = 0; light < 5; light++) {
    for (int k = 0; k < 2; k++) {
      starts[k][light] = hubs[k].start_time_us(0);
      hubs[k].sched_.light_set(0, static_cast<uint8_t>(light), 0, 20000, a, b, 1000, starts[k][light]);
    }
    run(1000);   // one call per light, a ms apart
  }
  run(300000);
  // every SET's first copy: its offset to its frame's TIME (ms) >= 0 (it arrived before its start)
  auto first_offsets = [](const std::vector<std::vector<uint8_t>> &frames, int *late, int *seen) {
    bool first[5] = {true, true, true, true, true};
    *late = *seen = 0;
    for (const auto &f : frames) {
      if (!valid_ultra(f) || f.size() != ultra_frame_bytes(1))
        continue;
      const uint8_t *s = f.data();
      if (s[0] != ULTRA_SET || (s[1] >> 4) >= 5 || !first[s[1] >> 4])
        continue;
      first[s[1] >> 4] = false;
      const int16_t off = static_cast<int16_t>(ultra_get16(s + ULTRA_PAYLOAD + 25));
      *late += off < 0;
      (*seen)++;
    }
  };
  std::vector<std::vector<uint8_t>> rmt_frames, lane_frames;
  for (const auto &p : rmts[0].sent)
    rmt_frames.push_back(p.second);
  const Decoder d = Decoder::of(bus, 0);
  for (const auto &f : d.frames)
    lane_frames.push_back(f.bytes);
  int late[2], seen[2];
  first_offsets(rmt_frames, &late[0], &seen[0]);
  first_offsets(lane_frames, &late[1], &seen[1]);
  bool together = true;
  for (int light = 0; light < 5; light++)
    together &= starts[0][light] == starts[1][light];
  CHECK(seen[0] == 5 && seen[1] == 5 && late[0] == 0 && late[1] == 0 && together, "a group change of 5 lights on an RMT "
        "line and a lane with 10 ms buffers: every first copy before its start (late: RMT %d, lane %d of %d / %d), one "
        "start for both", late[0], late[1], seen[0], seen[1]);
  std::printf("  group start: 5 commands on an RMT line and a lane with 10 ms buffers, every first copy in time (lead "
              "%.1f ms)\n", (starts[1][0] - 300000000 - 2000000) / 1000.0);
  UltraHub::hubs_.clear();
}


/// A line on a Diff-Power port, on a lane: the port's logic (components/diffpower) and the hub's side of it, unchanged,
/// on the modelled stream; the receivers answer only while the port is powered. A lane has no RMT channel: asking it
/// never says done. Through a wake, a planned power-off, a wake again and the controller's shutdown:
///  - after each wake the preamble comes first (the first frame posted after it is DARK), and the first poll frame
///    starts on the wire no sooner than ULTRA_RW_LOCK_MS after the preamble's last frame ended; the receivers answer;
///  - no poll frame from the notice's first copy until the port is powered again;
///  - each power-off: 3 notice copies (IDENTIFY POWER_OFF to every board), at least 50 ms apart from their starts on
///    the wire, every copy out before the enable drops, the enable at least 100 ms after the third copy's start (as
///    on an RMT line, where a copy starts as it is handed over);
///  - the shutdown hook: the same 3 copies through the stream while the loop no longer runs, the enable low after.
static void test_port_on_lane(uint32_t chunk_groups, const char *what, bool inject = false, uint16_t boards = 2) {
  set_clock(400000000);
  UltraHub::hubs_.clear();
  DiffPowerPort::ports_.clear();
  DiffPowerPort::shut_down_ = false;
  tel = Telemetry();
  static Stream bus;
  renew(bus);
  const Grid g = default_grid(16);
  bus.init(g, 16, chunk_groups, 16, 0, clock_us * 1000 + 777);
  const uint32_t sample_ns = g.sample_ns, P = g.period, group_ns = 8 * P * sample_ns;
  const uint32_t reset_ns = (300000 + group_ns - 1) / group_ns * group_ns;
  static UltraHub hub;
  static Uart uart;
  static Pin dir, enable, detect;
  static TestLight light;
  static DiffPowerPort port;
  renew(hub);
  renew(port);
  uart = Uart();
  light = TestLight();
  hub.num_boards_ = boards;
  tel.boards_of[0] = boards;
  enable.reset(false);
  detect.reset(true);
  PortConfig pc = port.core_.config();
  pc.min_on_ms = 2000;
  port.core_.configure(pc);
  port.enable_pin_ = &enable;
  port.detect_pin_ = &detect;
  DiffPowerPort::ports_.push_back(&port);
  TelemetryIntervals iv;
  iv.status_ms = 1000;
  hub.start(bus.s.lane(0), g.lane[0].h0 * sample_ns, (P - g.lane[0].h0) * sample_ns, g.lane[0].h1 * sample_ns,
            (P - g.lane[0].h1) * sample_ns, reset_ns, &uart, &dir, nullptr, 600, 2, ULTRA_RW_RATE_1M, iv);
  hub.port_ = &port;
  hub.lights_.push_back(&light);
  hub.channel_ = 7;          // (a lane has no RMT channel: it never says done)
  rmts[7] = Rmt();
  rmts[7].busy = true;
  tel.uarts[0] = &uart;
  tel.uart_of[0] = 0;
  tel.dir[0] = &dir;

  struct Seen {
    int64_t start_us, end_us;
    bool rw, dark, notice;
  };
  std::vector<Seen> seen;
  Decoder dec;
  size_t decoded = 0;
  std::vector<std::pair<int64_t, uint8_t>> states{{clock_us, port.core_.state()}};
  // inject: the stream stops (a late render) at notice copies: right after the planned power-off's first post (the
  // copy is refused: it never went out), within its third post's last bytes and the shutdown's first (cut); with
  // frames longer than a buffer, also within the second post's last bytes once the lane's next frame (a command
  // queued with it) was taken: the notice is cut while a later frame is in the pipeline
  int phase = 0, posts = 0, injected = 0, pipelined = 0;
  uint32_t last_id = 0, watch_id = 0, watch_next = 0;
  int64_t kill_at = 0, watch_end = 0;
  const uint16_t a[CHANNELS] = {100, 200, 300, 400, 500}, b[CHANNELS] = {65535, 65535, 65535, 65535, 65535};
  auto step = [&](int64_t until_us, bool loops) {
    while (clock_us < until_us) {
      const int64_t next = std::min(until_us, clock_us + 50);
      if (kill_at != 0 && kill_at <= next) {
        bus.run_until(kill_at * 1000);
        if (!bus.dead) {
          bus.stop_at(kill_at * 1000);
          injected++;
        }
        kill_at = 0;
      }
      bus.run_until(next * 1000);
      set_clock(next);
      for (size_t s = std::max<size_t>(decoded, 1); s < bus.wire.size(); s++) {
        if ((bus.wire[s] ^ bus.wire[s - 1]) & 1u)
          dec.edge(bus.level(0, s), static_cast<int64_t>(s), sample_ns);
      }
      decoded = bus.wire.size();
      while (dec.end(static_cast<int64_t>(decoded), sample_ns)) {
        const Decoder::Frame &f = dec.frames.back();
        if (!valid_ultra(f.bytes) || f.bytes.size() != hub.sched_.frame_bytes())
          continue;
        Seen x{bus.time_of(f.start) / 1000, bus.time_of(f.t0) / 1000, false, true, false};
        x.rw = (ultra_header_flags(f.bytes.data() + hub.sched_.header_offset()) & ULTRA_HDR_RW) != 0;
        for (uint16_t b = 0; b < 2; b++) {
          const uint8_t *sl = f.bytes.data() + ULTRA_SLOT_BYTES * b;
          bool zero = true;
          for (uint32_t i = 0; i < ULTRA_SLOT_BYTES; i++)
            zero &= sl[i] == 0;
          x.dark &= zero;
          x.notice |= sl[0] == ULTRA_IDENTIFY && sl[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF;
        }
        seen.push_back(x);
        if (x.rw)
          tel.rw_frame(0, x.end_us, f.bytes);
      }
      tel.on[0] = port.core_.state() == PS_ON || port.core_.state() == PS_NOTICE;
      tel.tick();
      if (loops) {
        hub.loop();
        if (clock_us % 1000 < 50)
          port.loop();   // (ESPHome's loop, once a ms here)
      }
      if (port.core_.state() != states.back().second)
        states.emplace_back(clock_us, port.core_.state());
      if (hub.notice_id_ != 0 && hub.notice_id_ != last_id) {   // a notice copy posted
        last_id = hub.notice_id_;
        posts++;
        if (inject && phase == 1 && posts == 1) {
          kill_at = clock_us + 1;
        } else if (inject && ((phase == 1 && posts == 3) || (phase == 2 && posts == 1))) {
          kill_at = hub.notice_end_us_ - 30;
        } else if (inject && boards > 2 && phase == 1 && posts == 2) {
          watch_id = hub.notice_id_;
          watch_end = hub.notice_end_us_;
          watch_next = 0;
          hub.sched_.light_set(0, 0, 0, 1000, a, b, 100, clock_us + 500000);   // the lane's next frame follows
        }
      }
      if (watch_id != 0) {   // the next frame taken while the notice is on the wire: cut the notice before its end
        ParlioLane *lane = bus.s.lane(0);
        if (watch_next == 0 && lane->posted_id() != watch_id)
          watch_next = lane->posted_id();
        if (watch_next != 0 && lane->taken_id() == watch_next && clock_us < watch_end - 30) {
          kill_at = watch_end - 30;
          pipelined++;
          watch_id = 0;
        } else if (clock_us >= watch_end) {
          watch_id = 0;
        }
      }
    }
  };
  advance_us = [&](int64_t us) { step(clock_us + us, false); };

  std::vector<int64_t> on_at, notice_at, drop_at;   // the port powered; the episodes' first copy; the enable low
  light.lit = true;
  step(clock_us + 6000000, true);
  const bool on1 = port.is_ready() && hub.poller_.answering(0) && hub.poller_.answering(1);
  light.lit = false;
  phase = 1;
  posts = 0;
  step(clock_us + 4000000, true);
  const bool standby = port.core_.state() == PS_STANDBY && !enable.level;
  phase = 0;
  light.lit = true;
  step(clock_us + 6000000, true);
  const bool on2 = port.is_ready() && hub.poller_.answering(0) && hub.poller_.answering(1);
  const int64_t t_sd = clock_us;
  phase = 2;
  posts = 0;
  DiffPowerPort::shutdown_all();
  const int64_t sd_ms = (clock_us - t_sd) / 1000;
  step(clock_us + 200000, false);   // (what was posted: out on the wire)
  CHECK(on1 && standby && on2 && !enable.level && sd_ms < 1100, "%s: a line on a port, on a lane: powered, answering "
        "(%d), standby (%d), powered again, answering (%d), the shutdown's enable low (%d) after %lld ms (%u windows, "
        "%u answers, %u stale, %d let go late)", what, on1, standby, on2, !enable.level, static_cast<long long>(sd_ms),
        hub.poller_.stats().windows, hub.poller_.stats().answers, hub.poller_.stats().stale, tel.late);

  for (const auto &e : states) {
    if (e.second == PS_ON)
      on_at.push_back(e.first);
  }
  for (const auto &e : enable.log) {
    if (e.first != INT64_MIN && !e.second)
      drop_at.push_back(e.first);
  }
  // the wakes: preamble first, then the lock interval from its last frame's end before a poll frame
  bool preamble_first = on_at.size() == 2, lock_ok = true, rw_after = true;
  for (int64_t on : on_at) {
    const Seen *first = nullptr;
    int64_t last_dark_end = 0;
    for (const Seen &x : seen) {
      if (x.start_us < on + hub.lane_lead_us_)
        continue;
      if (first == nullptr)
        first = &x;
      if (!x.dark)
        break;
      last_dark_end = x.end_us;
    }
    preamble_first &= first != nullptr && first->dark;
    const Seen *rw = nullptr;
    for (const Seen &x : seen) {
      if (x.rw && x.start_us >= on) {
        rw = &x;
        break;
      }
    }
    rw_after &= rw != nullptr;
    lock_ok &= rw != nullptr && rw->start_us >= last_dark_end + static_cast<int64_t>(ULTRA_RW_LOCK_MS) * 1000;
  }
  CHECK(preamble_first && rw_after && lock_ok, "%s: after each wake the preamble first, poll frames again, the first "
        "no sooner than the lock interval after the preamble's end", what);
  // the power-offs: the planned one and the shutdown's
  bool copies_ok = drop_at.size() == 2, quiet = true;
  int64_t least_gap = INT64_MAX, least_tail = INT64_MAX;
  for (size_t k = 0; k < drop_at.size(); k++) {
    const int64_t from = k == 0 ? on_at[0] : on_at.size() > 1 ? on_at[1] : 0;
    std::vector<const Seen *> c;
    for (const Seen &x : seen) {
      if (x.notice && x.start_us >= from)
        c.push_back(&x);
      if (c.size() == 3)
        break;
    }
    if (c.size() != 3) {
      copies_ok = false;
      continue;
    }
    least_gap = std::min({least_gap, c[1]->start_us - c[0]->start_us, c[2]->start_us - c[1]->start_us});
    least_tail = std::min(least_tail, drop_at[k] - c[2]->start_us);
    copies_ok &= c[2]->end_us < drop_at[k];
    const int64_t until = k == 0 && on_at.size() > 1 ? on_at[1] + static_cast<int64_t>(ULTRA_RW_LOCK_MS) * 1000 : INT64_MAX;
    for (const Seen &x : seen)
      quiet &= !(x.rw && x.start_us >= c[0]->start_us && x.start_us < until);
  }
  CHECK(copies_ok && least_gap >= 50000 && least_tail >= 99000 && quiet, "%s: each power-off (planned, shutdown): 3 "
        "notice copies at least %.1f ms apart on the wire, all out before the enable drops, it %.1f ms after the third "
        "one's start at least; no poll frame from the first copy until powered again", what, least_gap / 1000.0,
        least_tail / 1000.0);
  CHECK(!inject || injected == 3 + (boards > 2), "%s: the stream stopped at %d notice copies (%d)", what,
        3 + (boards > 2), injected);
  CHECK(!inject || boards <= 2 || pipelined == 1, "%s: a notice copy cut while the lane's next frame was already "
        "taken (%d)", what, pipelined);
  CHECK(psram_in_isr == 0, "%s: the timer's interrupt touched no ISR pin's own allocation (%d)", what, psram_in_isr);
  std::printf("  a line on a Diff-Power port, on a lane (%s): preamble after each wake, polls after the lock interval, "
              "notice copies >= %.1f ms apart, the enable %.1f ms after the third, the shutdown in %lld ms", what,
              least_gap / 1000.0, least_tail / 1000.0, static_cast<long long>(sd_ms));
  if (inject)
    std::printf("; %d copies refused or cut by stops of the stream (%d with the next frame taken), sent again", injected,
                pipelined);
  std::printf("\n");
  UltraHub::hubs_.clear();
  DiffPowerPort::ports_.clear();
}


/// A poll frame the transmitter keeps refusing holds its line's UART turn while it is retried; when the line's
/// Diff-Power port goes down, the frame is dropped and the turn with it (its receiver off): the other line on the UART
/// polls on.
static void test_port_down_unsent_poll() {
  set_clock(600000000);
  UltraHub::hubs_.clear();
  DiffPowerPort::ports_.clear();
  DiffPowerPort::shut_down_ = false;
  tel = Telemetry();
  static UltraHub hubs[2];
  static Uart uart;
  static Pin dir[2], re[2], enable, detect;
  static TestLight light;
  static DiffPowerPort port;
  uart = Uart();
  renew(port);
  light = TestLight();
  light.lit = true;
  enable.reset(false);
  detect.reset(true);
  PortConfig pc = port.core_.config();
  pc.min_on_ms = 2000;
  port.core_.configure(pc);
  port.enable_pin_ = &enable;
  port.detect_pin_ = &detect;
  DiffPowerPort::ports_.push_back(&port);
  for (int k = 0; k < 2; k++) {
    renew(hubs[k]);
    hubs[k].channel_ = k;
    rmts[k] = Rmt();
    rmts[k].hub = &hubs[k];
    TelemetryIntervals iv;
    iv.status_ms = 1000;
    hubs[k].start(nullptr, 250, 1050, 700, 650, 300000, &uart, &dir[k], &re[k], 700 + k, 2, ULTRA_RW_RATE_1M, iv);
    tel.uart_of[k] = 0;
    tel.re[k] = &re[k];
    tel.dir[k] = &dir[k];
  }
  tel.uarts[0] = &uart;
  hubs[0].port_ = &port;
  hubs[0].lights_.push_back(&light);
  run_rmt(hubs, 2, 6000000, &port);   // powered, both lines polling
  const bool both = port.is_ready() && hubs[0].poller_.answering(0) && hubs[1].poller_.answering(0);
  rmts[0].refuse_rw_all = true;       // line 0's poll frames refused from now on: it keeps its turn, retrying
  for (int t = 0; t < 300000 && !(hubs[0].rw_turn_ && hubs[0].tx_prepared_ && !hubs[0].rw_open_); t++)
    run_rmt(hubs, 2, 20, &port);
  const bool held = hubs[0].rw_turn_ && hubs[0].tx_prepared_ && re[0].level;
  light.lit = false;                  // its port goes down (the light's turn-off period, the notice)
  run_rmt(hubs, 2, 3000000, &port);
  const uint32_t before = hubs[1].windows;
  run_rmt(hubs, 2, 5000000, &port);
  CHECK(both && held && !port.is_ready() && !hubs[0].rw_turn_ && !re[0].level && hubs[1].windows >= before + 3 &&
            hubs[1].poller_.answering(0),
        "a refused poll frame holds its line's UART turn (%d); its port down, the frame and the turn are dropped (turn "
        "%d, receiver %d): the other line polls on (%u windows in 5 s)", held, hubs[0].rw_turn_, re[0].level,
        hubs[1].windows - before);
  std::printf("  a refused poll frame, then its port down: the frame and its UART turn dropped, the other line polls "
              "on\n");
  UltraHub::hubs_.clear();
  DiffPowerPort::ports_.clear();
}

#undef USE_DIG2ANALOG_ULTRA_TELEMETRY
#undef USE_DIG2ANALOG_ULTRA_PARLIO
#undef USE_DIG2ANALOG_ULTRA_UPDATE
#undef USE_DIFFPOWER

}  // namespace hub_lanes

// ================================================================================================ the update on a lane
// The update task's session on a lane (build/parlio/<variant>/hub_update_lane.inc): every frame as the session built
// it, after every page the line low at least its reset + the page pause on the stream, whatever the stream's latency;
// an abort ends with 12 ABORT frames; expired reservations and refused frames go again; a stalled stream ends the
// session (aborted or failed) in bounded time; the session ends once its last frame is out.
namespace hub_update {

static Stream *bus = nullptr;
static uint32_t abort_at = UINT32_MAX;   // the abort request once this many frames went out
static uint32_t portTICK_PERIOD_MS = 1;
static uint32_t ticks = 0, tick_limit = UINT32_MAX;

/// The lane as the task sees it, with faults: posts that fail as if their start passed (expire), frames refused after
/// they were posted (refuse: the stream restarted before they went out), a renderer that stops taking frames (stall).
struct TestLane {
  ParlioLane *lane{nullptr};
  int expire{0}, refuse{0};
  bool stall{false}, fake_refused{false}, fake_posted{false};
  int reserves{0}, posts{0};
  bool ready() const { return this->lane->ready(); }
  bool busy() {
    if (this->stall)
      return true;
    if (this->fake_posted) {
      this->fake_posted = false;
      this->fake_refused = true;
      return false;
    }
    return this->lane->busy();
  }
  bool take_refused() {
    if (this->fake_refused) {
      this->fake_refused = false;
      return true;
    }
    return this->lane->take_refused();
  }
  int64_t reserve_us() {
    this->reserves++;
    return this->lane->reserve_us();
  }
  bool post(const uint8_t *f, size_t n, uint32_t quiet, bool release) {
    this->posts++;
    if (this->expire > 0) {
      this->expire--;
      return false;
    }
    if (this->refuse > 0) {   // accepted, then refused: it never goes out
      this->refuse--;
      this->fake_posted = true;
      return true;
    }
    return this->lane->post(f, n, quiet, release);
  }
  int64_t end_us() const { return this->lane->end_us(); }
};

struct UltraHub;
static UltraHub *hub = nullptr;
static void vTaskDelay(uint32_t n);

struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  uint8_t update_lane_();   // the component's
  static int64_t now_us_() { return clock_us; }
  TestLane *lane_{nullptr};
  uint8_t buf_[64]{};
  uint8_t *tx_buf_{buf_};
  uint32_t reset_ns_{302400};
  UpdateSession upd_session_;
  std::atomic<bool> upd_abort_{false};
  std::atomic<uint32_t> upd_sent_{0};
  std::atomic<uint8_t> upd_pass_{0};
};

/// the task sleeps: the stream plays on
static void vTaskDelay(uint32_t n) {
  if (++ticks > tick_limit)
    throw std::runtime_error("the update task never returned");
  const int64_t end = clock_us + static_cast<int64_t>(n) * portTICK_PERIOD_MS * 1000;
  bus->run_until(end * 1000);
  set_clock(end);
  if (hub->upd_sent_.load() >= abort_at)
    hub->upd_abort_.store(true);
}

#define USE_DIG2ANALOG_ULTRA_UPDATE
#define USE_DIG2ANALOG_ULTRA_PARLIO
#include "hub_update_lane.inc"
#undef USE_DIG2ANALOG_ULTRA_UPDATE
#undef USE_DIG2ANALOG_ULTRA_PARLIO

static uint8_t img[UPD_DESC_BYTES + 2000];
static UpdateSession::Options options() {
  for (size_t i = 0; i < sizeof(img); i++)
    img[i] = static_cast<uint8_t>(i * 7 + 3);
  UpdateSession::Options o;
  o.lead_in_us = 20000;
  o.end_us = 10000;
  return o;
}
static const uint32_t FRAME_MIN_US = (UPD_FRAME_BITS * 1300) / 1000 + 302;

/// a session on a fresh stream: its result; the frames on the wire checked against the session's own
struct Run {
  uint8_t result{0};
  size_t session{0}, pauses{0}, aborts{0};
  int64_t least{INT64_MAX};
  bool same{true}, paused{true}, threw{false};
  int64_t returned_us{0}, last_end_us{0};
};
static Run run_session(TestLane &tl, std::function<void(TestLane &)> setup = nullptr) {
  static Stream s;
  renew(s);
  set_clock(9000000);
  const Grid g = default_grid(1);
  s.init(g, 8, 96, 1, 0, clock_us * 1000);
  bus = &s;
  UltraHub h;
  hub = &h;
  tl.lane = s.s.lane(0);
  tl.lane->begin(UPD_FRAME_BYTES, nullptr, nullptr);
  if (setup)
    setup(tl);
  h.lane_ = &tl;
  const UpdateSession::Options o = options();
  h.upd_session_.begin(img, sizeof(img), 0x42, o, FRAME_MIN_US);
  UpdateSession ref;
  ref.begin(img, sizeof(img), 0x42, o, FRAME_MIN_US);
  ticks = 0;
  Run r;
  try {
    r.result = h.update_lane_();
  } catch (const std::runtime_error &) {
    r.threw = true;
  }
  r.returned_us = clock_us;
  tl.stall = false;
  for (int k = 0; k < 6; k++)
    s.step();   // the rest on the wire
  const Decoder dec = Decoder::of(s, 0);
  for (size_t i = 0; i < dec.frames.size(); i++) {
    const std::vector<uint8_t> &f = dec.frames[i].bytes;
    r.same &= f.size() == UPD_FRAME_BYTES && upd_check_frame(f.data()) == 0;
    r.last_end_us = s.time_of(dec.frames[i].t0) / 1000;
    if (f.size() == UPD_FRAME_BYTES && f[UPD_B_TYPE] == UPD_ABORT) {
      r.aborts++;
      continue;
    }
    uint8_t rf[UPD_FRAME_BYTES];
    uint32_t pause_us = 0;
    r.same &= ref.next(rf, 0, &pause_us) && f.size() == UPD_FRAME_BYTES && std::memcmp(rf, f.data(), UPD_BLOCK_BYTES) == 0;
    if (pause_us != 0 && i + 1 < dec.frames.size()) {
      // the low time from its last falling edge to the next frame's first rising edge
      const int64_t low_us = (dec.frames[i + 1].start - dec.frames[i].t0) * g.sample_ns / 1000;
      r.paused &= low_us >= 300 + static_cast<int64_t>(pause_us);   // its reset and the page pause
      r.least = std::min(r.least, low_us);
      r.pauses++;
    }
    r.session++;
  }
  return r;
}

static void test_update_on_lane() {
  options();
  for (uint32_t tick_ms : {1u, 10u}) {
    portTICK_PERIOD_MS = tick_ms;
    for (int run = 0; run < 2; run++) {
      abort_at = run == 1 ? 100 : UINT32_MAX;
      tick_limit = 1000000;
      TestLane tl;
      const Run r = run_session(tl);
      UpdateSession ref;
      ref.begin(img, sizeof(img), 0x42, options(), FRAME_MIN_US);
      CHECK(r.same && r.paused && !r.threw, "update on a lane, %u ms tick, run %d: %u frames as the session built "
            "them, after %u pages the line low >= reset + page pause (least %lld us)", tick_ms, run,
            static_cast<unsigned>(r.session), static_cast<unsigned>(r.pauses), static_cast<long long>(r.least));
      CHECK(r.returned_us >= r.last_end_us, "it returns once its last frame is out (%lld >= %lld)",
            static_cast<long long>(r.returned_us), static_cast<long long>(r.last_end_us));
      if (run == 0) {
        CHECK(r.result == UltraHub::UPD_DONE && r.session == ref.total_frames() && r.pauses == 3 * 8 && r.aborts == 0,
              "a whole session on a lane: done, %u frames, %u page pauses", static_cast<unsigned>(r.session),
              static_cast<unsigned>(r.pauses));
      } else {
        CHECK(r.result == UltraHub::UPD_ABORTED && r.session >= 100 && r.session <= 101 && r.aborts == 12,
              "an abort after frame 100: the frames up to it, then 12 ABORT frames (%u, %u)",
              static_cast<unsigned>(r.session), static_cast<unsigned>(r.aborts));
      }
    }
  }
  portTICK_PERIOD_MS = 1;
  abort_at = UINT32_MAX;
  std::printf("  update on a lane: every frame as the session built it, after every page the line low >= its reset + "
              "the page pause (1 kHz and 100 Hz tick), the last frame out before it returns\n");
}

static void test_update_failures() {
  options();
  abort_at = UINT32_MAX;
  tick_limit = 100000;
  UpdateSession ref;
  ref.begin(img, sizeof(img), 0x42, options(), FRAME_MIN_US);
  {   // expired reservations: posted again at new starts
    TestLane tl;
    tl.expire = 30;
    const Run r = run_session(tl);
    CHECK(r.result == UltraHub::UPD_DONE && r.same && r.session == ref.total_frames() && tl.reserves >= 31 + 1,
          "30 expired reservations: each reserved again (%d reservations, %d posts), the whole session on the wire",
          tl.reserves, tl.posts);
  }
  {   // frames refused after they were posted: sent again
    TestLane tl;
    tl.refuse = 5;
    const Run r = run_session(tl);
    CHECK(r.result == UltraHub::UPD_DONE && r.same && r.session == ref.total_frames(), "5 frames refused after their "
          "post: sent again, the whole session on the wire, in order (%u of %u)", static_cast<unsigned>(r.session),
          static_cast<unsigned>(ref.total_frames()));
  }
  {   // the stream stalls (it never takes a frame): the session fails after ~2 s of the task's ticks
    TestLane tl;
    const Run r = run_session(tl, [](TestLane &t) { t.stall = true; });
    CHECK(!r.threw && r.result == UltraHub::UPD_FAILED && ticks < 2002 + 110, "a stalled stream: the session fails "
          "in bounded time (%u ticks of 1 ms)", ticks);
  }
  {   // stalled, then an abort: aborted at once, its ABORT frames tried for ~100 ms (the stream takes none)
    TestLane tl;
    abort_at = 0;   // the abort comes with the task's first sleep
    const Run r = run_session(tl, [](TestLane &t) { t.stall = true; });
    abort_at = UINT32_MAX;
    CHECK(!r.threw && r.result == UltraHub::UPD_ABORTED && ticks < 250, "stalled, then an abort: aborted within "
          "%u ticks of 1 ms", ticks);
  }
  tick_limit = UINT32_MAX;
  std::printf("  update failures: expired reservations and refused frames sent again, a stalled stream ends the session "
              "(failed, or aborted), in bounded time\n");
}

}  // namespace hub_update
}  // namespace esphome::dig2analog_ultra
