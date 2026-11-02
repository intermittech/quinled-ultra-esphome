// Host test of the hub's digital port (a Diff-Power Ultra Receiver Digital-Medium in the data path of a pixel strip):
// tests/run_ur_digital_m_transport_test.py cuts the methods unchanged from components/ultra_line/
// ultra_line.cpp into build/transport/*.inc (the encoder callback, loop_digital_, transmit_digital_, the window's
// end, the pixel buffers, the local effect and the hand-back, the back-off's stored flag), and this file runs them with
// stand-ins for ESPHome and the RMT driver: the RMT stand-in runs the encoder over the transmission and keeps its
// symbols, the checks read the wire back from them: the LOW before every marker and after a frame, the marker, the
// window of a poll, the retake, the line let go of and taken back (the direction pin), the pixel frames whole. The
// planner (ultra_dport.h) is the sender library's own, byte for byte, tested there against the receiver's delay line.
// Build: python tests/run_ur_digital_m_transport_test.py   (uses the ziglang pip package as C/C++ compiler)

#include "../components/ultra_line/ultra_core.h"
#include "../components/ultra_line/ultra_dport.h"
#include "board_pixel_fixture.h"
#include "../components/ultra_line/ultra_sender_update.h"   // (a port job's transport instructions)

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#define IRAM_ATTR
#define HOT
#define USE_DIG2ANALOG_UR_DIGITAL_M
#define USE_ULTRA_LINE_TELEMETRY
#define USE_ULTRA_LINE_UPDATE
#define ESP_LOGE(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGV(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGW(...) logw_count++

static int logw_count = 0;
static int failures = 0, checks = 0;
#define CHECK(cond, ...) \
  do { \
    checks++; \
    if (!(cond)) { \
      failures++; \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
    } \
  } while (0)

// ------------------------------------------------------------------------------------------------ the clock
static int64_t g_now_us = 5000000;
static uint32_t millis() { return static_cast<uint32_t>(g_now_us / 1000); }

// ------------------------------------------------------------------------------------------------ ESP-IDF's RMT
typedef int esp_err_t;
static constexpr esp_err_t ESP_OK = 0, ESP_FAIL = -1;
static const char *esp_err_to_name(esp_err_t) { return "ERR"; }
union rmt_symbol_word_t {
  struct {
    uint32_t duration0 : 15;
    uint32_t level0 : 1;
    uint32_t duration1 : 15;
    uint32_t level1 : 1;
  };
  uint32_t val;
};
struct rmt_transmit_config_t {
  int loop_count;
};
typedef void *rmt_channel_handle_t;
typedef void *rmt_encoder_handle_t;

// a transmission as the RMT stand-in saw it
struct Tx {
  int64_t start_us{0};
  std::vector<uint32_t> words;
  std::vector<uint8_t> bytes;
  bool marker{false};
  uint32_t bit0{0}, bit1{0};
  struct Seg {
    double t, d;
    int level;
  };
  std::vector<Seg> segs;
  double end_rel{0};
  double first_rise{-1}, last_fall{-1};   // relative to the start (us)
  double marker_high{-1}, marker_low{-1};
  bool poll_frame{false};
  bool de_high{true};   // the direction pin as it started
};
static std::vector<Tx> g_tx;
static int g_rmt_fail = 0;             // the next rmt_transmit fails
static int64_t g_busy_until_us = 0;    // the RMT is out of the last transmission from here
static int64_t g_isr_us = -1;          // the last poll's done interrupt (the line is let go of)

esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void *, size_t, const rmt_transmit_config_t *);
/// (a port job's frame: the RMT done once its last symbol is out; the stand-in's time doesn't move while it waits)
static esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int) { return ESP_OK; }

namespace esphome {

template<typename T> class RAMAllocator {
 public:
  enum Flags { ALLOC_INTERNAL = 1 };
  explicit RAMAllocator(int) {}
  T *allocate(size_t n) { return static_cast<T *>(std::malloc(n * sizeof(T))); }
};

class GPIOPin {
 public:
  struct Event {
    int64_t t;
    bool level;
  };
  void digital_write(bool v) {
    level = v;
    events.push_back({g_now_us, v});
  }
  bool level{true};
  std::vector<Event> events;
};

class ESPPreferenceObject {
 public:
  ESPPreferenceObject() = default;
  ESPPreferenceObject(uint32_t key, std::map<uint32_t, std::vector<uint8_t>> *store) : key_(key), store_(store) {}
  bool save(const uint8_t *p) {
    saves++;
    (*this->store_)[this->key_] = std::vector<uint8_t>(p, p + 1);
    return true;
  }
  bool load(uint8_t *p) {
    auto it = this->store_->find(this->key_);
    if (it == this->store_->end())
      return false;
    *p = it->second[0];
    return true;
  }
  static int saves;

 protected:
  uint32_t key_{0};
  std::map<uint32_t, std::vector<uint8_t>> *store_{nullptr};
};
int ESPPreferenceObject::saves = 0;
struct Preferences {
  std::map<uint32_t, std::vector<uint8_t>> store;
  template<typename T> ESPPreferenceObject make_preference(uint32_t key) { return ESPPreferenceObject(key, &this->store); }
};
static Preferences prefs_store;
static Preferences *global_preferences = &prefs_store;

class Component {
 public:
  bool status_has_warning() const { return warn_; }
  void status_set_warning() { warn_ = true; }
  void status_clear_warning() { warn_ = false; }
  void mark_failed() { failed_ = true; }
  bool warn_{false}, failed_{false};
};

namespace ultra_line {

static constexpr uint32_t BUSY_HOLD_MS = 250;

struct HighFrequencyLoopRequester {
  void start() { running = true; }
  void stop() { running = false; }
  bool running{false};
};

using EncoderParams = FrameSymbols<rmt_symbol_word_t>;
struct EncoderArg {
  EncoderParams symbols;
};
struct DpEncoder {
  ultra_dp_rmt_t tab;
};

// the telemetry poller of the line: what the digital port's code asks of it
struct MockPoller {
  struct Stats {
    uint32_t windows{0}, answers{0};
  };
  const Stats &stats() const { return stats_; }
  uint8_t rate() const { return ULTRA_RW_RATE_1M; }
  void locked(uint32_t) {}
  uint16_t build(uint32_t, uint8_t *slots, uint32_t *window_us, bool startup_probe = false) {
    CHECK(!startup_probe, "unmanaged digital transport adds no module startup wait");
    static uint8_t seq = 0;
    const uint8_t arg[4] = {0, 0, 0, 0};
    ultra_slot_poll(slots, ++seq, 0, 1, ULTRA_PAGE_STATUS, ULTRA_RW_RATE_1M, arg, 4);
    *window_us = ultra_rw_window_us(1, ULTRA_RW_RATE_1M);
    due = false;
    polls_built++;
    return 1;
  }
  Stats stats_;
  bool due{false};
  int polls_built{0};
};
struct MockUart {
  size_t available() { return 0; }
  bool read_array(uint8_t *, size_t) { return false; }
};

class UltraHub final : public Component {
 public:
  PixelCalibrationJob pixel_cal_job_;
  // ---- state the cut methods use (the same names as the component's)
  bool digital_{true};
  ultra_dp_t dp_{};
  uint32_t dp_reset_us_{300};
  DpEncoder dp_enc_{};
  uint32_t rmt_res_{80000000};
  uint32_t px_t0h_{400}, px_t0l_{850}, px_t1h_{800}, px_t1l_{450};
  uint32_t px_word0_{0}, px_word1_{0};
  uint32_t px_bytes_{0};
  uint8_t *px_pending_{nullptr}, *px_tx_{nullptr};
  bool px_ready_{false};
  bool px_have_{false};
  uint32_t px_frames_{0};
  uint8_t fx_active_{0};
  ESPPreferenceObject dp_pref_;
  bool dp_stored_{false};
  uint32_t tel_key_{0x1234};
  uint8_t pin_{16};
  uint16_t preamble_{5};
  void note_wake_light_() {}   // (the port's wake timeline: not under test here)
  Scheduler sched_;
  uint8_t *tx_buf_{nullptr};
  size_t tx_bytes_{0};
  bool tx_prepared_{false};
  bool tx_normal_{true};
  bool tx_rw_{false};
  EncoderArg enc_{};
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  bool rw_open_{false}, rw_turn_{false}, rw_waiting_{false}, rw_driving_{true}, rw_release_{false};
  int64_t rw_t0_us_{0}, rw_end_us_{0}, rw_read_us_{0};
  volatile int64_t rw_released_us_{0};
  GPIOPin *rw_dir_pin_{nullptr};
  GPIOPin *rw_rx_enable_pin_{nullptr};
  MockUart *rw_uart_{nullptr};
  MockPoller poller_;
  std::vector<uint8_t> rw_slots_;
  uint32_t rw_window_us_{0};
  uint32_t refresh_interval_ms_{200};
  bool has_sent_{false};
  uint32_t last_frame_ms_{0}, last_busy_ms_{0}, frames_{0};
  HighFrequencyLoopRequester high_freq_;
  bool high_freq_active_{false};
  bool answer_{true};   // the test: the window brings a valid answer
  // a port job's lease of the line
  bool upd_lease_{false}, started_{true};
  size_t tx_buf_bytes_{64};
  rmt_symbol_word_t reset_symbol_{};
  bool is_failed() const { return false; }
  bool boards_powered_() const { return true; }   // this transport fixture has no managed supply
  bool receivers_ready_() const { return this->boards_powered_(); }
  void receiver_reply_() {}
  void resync() { preamble_ = 3; }

  static int64_t now_us_() { return g_now_us; }
  void wake_() { last_busy_ms_ = millis(); }
  void loop_fast_() {
    high_freq_.start();
    high_freq_active_ = true;
  }
  bool line_free_() { return g_now_us >= g_busy_until_us; }
  bool rw_due_(bool) { return poller_.due && !rw_open_; }
  void rw_drop_turn_() {
    if (rw_turn_ && rw_rx_enable_pin_ != nullptr)
      rw_rx_enable_pin_->digital_write(false);
    rw_turn_ = false;
  }
  void rw_uart_rate_() {}
  void finish_window_(bool) {
    rw_open_ = false;
    rw_release_ = false;
    rw_released_us_ = 0;
    poller_.stats_.windows++;
    if (answer_)
      poller_.stats_.answers++;
    if (rw_rx_enable_pin_ != nullptr)
      rw_rx_enable_pin_->digital_write(false);
    rw_turn_ = false;
  }

  // ---- cut from the component
  bool digital_setup_();
  void digital_pixels(const uint8_t *data);
  void digital_note_fx(uint8_t segment, uint8_t effect, int64_t start_us, uint32_t local_frame_us);
  void digital_retry();
  void digital_save_();
  void digital_window_end_(int64_t now);
  void digital_window_track_();
  void loop_digital_();
  void transmit_digital_(const ultra_dp_tx_t &tx, int64_t now);
  bool lease_line(bool on);
  bool job_send(const uint8_t *frame, size_t n, const ultra_update_transport_t &t);
};

#include "build/transport/dp_encoder.inc"
#include "build/transport/dp_methods.inc"

}  // namespace ultra_line
}  // namespace esphome

using namespace esphome;
using namespace esphome::ultra_line;

// the RMT stand-in: the encoder runs over the transmission in chunks of random size (a real refill), the symbols are
// kept; a poll's done interrupt is the hub's release of the line
static std::mt19937 rng(7);
static UltraHub *g_hub = nullptr;

esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void *data, size_t size,
                       const rmt_transmit_config_t *) {
  if (g_rmt_fail > 0) {
    g_rmt_fail--;
    return ESP_FAIL;
  }
  Tx t;
  t.start_us = g_now_us;
  t.de_high = g_hub->rw_dir_pin_ == nullptr || g_hub->rw_dir_pin_->level;
  std::vector<rmt_symbol_word_t> sym(size * 8 + 8);
  size_t written = 0;
  bool done = false;
  while (!done) {
    const size_t chunk = 1 + rng() % 70;
    const size_t n =
        dp_encoder_callback(data, size, written, std::min(chunk, sym.size() - written), sym.data() + written, &done, &g_hub->dp_enc_);
    written += n;
    if (n == 0 && !done) {
      std::printf("encoder stuck\n");
      std::exit(2);
    }
  }
  for (size_t i = 0; i < written; i++)
    t.words.push_back(sym[i].val);
  t.bytes.assign(static_cast<const uint8_t *>(data), static_cast<const uint8_t *>(data) + size);
  t.marker = g_hub->dp_enc_.tab.marker != 0;
  t.bit0 = g_hub->dp_enc_.tab.bit0;
  t.bit1 = g_hub->dp_enc_.tab.bit1;
  double us = 0;
  for (uint32_t w : t.words) {
    const double d0 = (w & 0x7FFFu) / 80.0, d1 = ((w >> 16) & 0x7FFFu) / 80.0;
    t.segs.push_back({us, d0, static_cast<int>((w >> 15) & 1u)});
    t.segs.push_back({us + d0, d1, static_cast<int>((w >> 31) & 1u)});
    us += d0 + d1;
  }
  t.end_rel = us;
  bool first_bit_seen = false;
  for (const auto &s : t.segs) {
    if (s.level == 1 && s.d > 0) {
      if (t.first_rise < 0)
        t.first_rise = s.t;
      t.last_fall = s.t + s.d;
      if (t.marker && t.marker_high < 0)
        t.marker_high = s.d;
      first_bit_seen = true;
    }
  }
  (void) first_bit_seen;
  t.poll_frame = t.marker && t.bytes.size() >= ULTRA_SLOT_BYTES + ULTRA_HDR_BYTES &&
                 (ultra_header_flags(t.bytes.data() + t.bytes.size() - ULTRA_HDR_BYTES) & ULTRA_HDR_RW) != 0;
  g_busy_until_us = g_now_us + static_cast<int64_t>(std::ceil(us));   // (the RMT is done when its last symbol is out)
  if (t.poll_frame)
    g_isr_us = g_now_us + static_cast<int64_t>(t.end_rel + 3.0);   // (the tail's 2 us, the interrupt's latency)
  g_tx.push_back(std::move(t));
  return ESP_OK;
}

// ------------------------------------------------------------------------------------------------ the rig
struct Rig {
  UltraHub hub;
  GPIOPin de, rxen;
  MockUart uart;
  int64_t pix_period{0}, next_pix{0}, poll_period{0}, next_poll{0}, cmd_period{0}, next_cmd{0};
  uint32_t pix_n{0};
  uint8_t cmd_effect{0};
  int64_t t_setup{0};
  std::vector<uint8_t> pixbuf;

  explicit Rig(uint32_t pixel_bytes = 90, bool telemetry = true) {
    g_hub = &hub;
    g_tx.clear();
    g_busy_until_us = 0;
    g_isr_us = -1;
    ESPPreferenceObject::saves = 0;
    logw_count = 0;
    hub.sched_.init(1, 99, 3);
    hub.sched_.set_bit_timing(250, 1050, 700, 650);
    hub.tx_buf_ = static_cast<uint8_t *>(std::malloc(64));
    hub.tx_bytes_ = hub.sched_.frame_bytes();
    auto sym = [](uint32_t ns_h, uint32_t ns_l) {
      rmt_symbol_word_t s{};
      s.duration0 = static_cast<uint32_t>(ns_h * 0.08 + 0.5);
      s.level0 = 1;
      s.duration1 = static_cast<uint32_t>(ns_l * 0.08 + 0.5);
      s.level1 = 0;
      return s;
    };
    hub.enc_.symbols.bit0 = sym(250, 1050);
    hub.enc_.symbols.bit1 = sym(700, 650);
    hub.px_word0_ = sym(hub.px_t0h_, hub.px_t0l_).val;
    hub.px_word1_ = sym(hub.px_t1h_, hub.px_t1l_).val;
    hub.px_bytes_ = pixel_bytes;
    hub.rw_slots_.assign(ULTRA_SLOT_BYTES, 0);
    hub.rw_dir_pin_ = &de;
    hub.rw_rx_enable_pin_ = &rxen;
    hub.rw_uart_ = telemetry ? &uart : nullptr;
    hub.encoder_ = &hub.dp_enc_;
    pixbuf.assign(pixel_bytes, 0);
    t_setup = g_now_us;
    CHECK(hub.digital_setup_(), "setup");
  }
  ~Rig() {
    std::free(hub.tx_buf_);
    std::free(hub.px_pending_);
    std::free(hub.px_tx_);
  }

  // time forward to until_us; the pixel and poll timers, the commands, the done interrupt of a poll
  void run(int64_t until_us, int64_t step = 25) {
    while (g_now_us < until_us) {
      int64_t next = std::min<int64_t>(until_us, g_now_us + step);
      if (g_isr_us >= 0 && g_isr_us < next)
        next = std::max<int64_t>(g_isr_us, g_now_us);
      g_now_us = next;
      if (g_isr_us >= 0 && g_now_us >= g_isr_us) {
        g_isr_us = -1;
        if (hub.rw_release_) {   // the RMT's interrupt: let go of the line
          de.digital_write(false);
          hub.rw_release_ = false;
          hub.rw_released_us_ = g_now_us;
        }
      }
      if (pix_period > 0 && g_now_us >= next_pix) {
        pix_n++;
        for (size_t i = 0; i < pixbuf.size(); i++)
          pixbuf[i] = static_cast<uint8_t>(pix_n * 7 + i * 13);
        pixbuf[0] = static_cast<uint8_t>(pix_n);
        if (pixbuf.size() > 1)
          pixbuf[1] = static_cast<uint8_t>(pix_n >> 8);
        hub.digital_pixels(pixbuf.data());
        next_pix += pix_period;
      }
      if (poll_period > 0 && g_now_us >= next_poll) {
        hub.poller_.due = true;
        next_poll += poll_period;
      }
      if (cmd_period > 0 && g_now_us >= next_cmd) {
        ultra_pixel_fx_t fx;
        std::memset(&fx, 0, sizeof fx);
        fx.effect = cmd_effect;
        fx.brightness = static_cast<uint8_t>(1 + (next_cmd / 1000) % 200);
        hub.sched_.light_pixel_fx(0, 0, fx, g_now_us);
        next_cmd += cmd_period;
      }
      hub.loop_digital_();
    }
  }
  int count(bool marker, bool poll = false) const {
    int n = 0;
    for (const auto &t : g_tx)
      n += (t.marker == marker) && (!poll || t.poll_frame);
    return n;
  }
};

// ------------------------------------------------------------------------------------------------ the wire checks
static int wire_problems(const GPIOPin &de, int from, double reset_us, bool print) {
  int bad = 0;
  auto report = [&](int i, const char *what, double v) {
    bad++;
    if (print && bad <= 8)
      std::printf("    tx %d: %s (%.1f)\n", i, what, v);
  };
  for (int i = from; i < static_cast<int>(g_tx.size()); i++) {
    const Tx &t = g_tx[i];
    const double abs_first = static_cast<double>(t.start_us) + t.first_rise;
    if (t.marker) {
      // HIGH 20 us, LOW 10 us, then 320 bits that are the frame
      bool ok = false;
      size_t k = 0;
      while (k < t.words.size() && (t.words[k] & 0xFFFFu) != 0 && ((t.words[k] >> 15) & 1u) == 0)
        k++;   // (the lead: level-0 words)
      if (k < t.words.size()) {
        const uint32_t w = t.words[k];
        const double hi = (w & 0x7FFFu) / 80.0, lo = ((w >> 16) & 0x7FFFu) / 80.0;
        ok = std::fabs(hi - ULTRA_DP_MARKER_US) < 0.5 && std::fabs(lo - ULTRA_DP_GAP_HUB_US) < 0.5;
      }
      if (!ok)
        report(i, "marker is not 20 us HIGH + 10 us LOW", 0);
      if (t.bytes.size() != 40)
        report(i, "a marker frame is one slot and the header", static_cast<double>(t.bytes.size()));
      // the bits are the frame's
      std::vector<uint8_t> dec;
      for (size_t j = k + 1; j + 1 < t.words.size(); j++) {
        const size_t bit = j - (k + 1);
        if (bit >= t.bytes.size() * 8)
          break;
        const bool one = t.words[j] == t.bit1;
        if ((t.words[j] != t.bit0 && t.words[j] != t.bit1) || one != ((t.bytes[bit >> 3] >> (7 - (bit & 7))) & 1))
          report(i, "a bit of the frame is not its byte's", static_cast<double>(bit));
      }
    } else {
      // pixel frame: the bits are the frame's
      size_t k = 0;
      while (k < t.words.size() && ((t.words[k] >> 15) & 1u) == 0 && ((t.words[k] >> 31) & 1u) == 0)
        k++;
      for (size_t bit = 0; bit < t.bytes.size() * 8; bit++) {
        const bool one = ((t.bytes[bit >> 3] >> (7 - (bit & 7))) & 1) != 0;
        if (k + bit >= t.words.size() || t.words[k + bit] != (one ? t.bit1 : t.bit0)) {
          report(i, "a pixel bit is not its byte's", static_cast<double>(bit));
          break;
        }
      }
    }
    if (i == 0)
      continue;
    const Tx &p = g_tx[i - 1];
    const double prev_end = static_cast<double>(p.start_us) + p.end_rel;
    if (static_cast<double>(t.start_us) < prev_end - 1.0)
      report(i, "starts before the last transmission is out", static_cast<double>(t.start_us) - prev_end);
    if (p.poll_frame) {
      const double t_end = static_cast<double>(p.start_us) + p.last_fall;
      const double win_end = t_end + ultra_dp_window_us(ULTRA_RW_RATE_1M);
      if (static_cast<double>(t.start_us) < win_end - 2.0)
        report(i, "retakes the line inside the window", static_cast<double>(t.start_us) - win_end);
      const double low = abs_first - static_cast<double>(t.start_us);
      if (low < (t.marker ? ULTRA_DP_QUIET_US : ULTRA_DP_RETAKE_US) - 1.0)
        report(i, "LOW after the retake too short", low);
      // the direction pin: released within 150 us of the frame's last bit, high again by the retake
      bool released = false, retaken = false;
      for (const auto &e : de.events) {
        if (!e.level && e.t >= p.start_us + p.last_fall - 1 && e.t <= t_end + 150)
          released = true;
        if (e.level && e.t >= win_end - 5 && e.t <= t.start_us)
          retaken = true;
      }
      if (!released)
        report(i, "the line was not let go of within 150 us of the poll", 0);
      if (!retaken)
        report(i, "the line was not retaken by the next transmission", 0);
    } else {
      const double low = abs_first - (static_cast<double>(p.start_us) + p.last_fall);
      const double need = t.marker ? ULTRA_DP_QUIET_US : (p.marker ? ULTRA_DP_AFTER_US : reset_us);
      if (low < need - 1.0)
        report(i, t.marker ? "LOW before the marker too short" : "LOW before the pixels too short", low);
    }
  }
  return bad;
}

static void check_wire(const Rig &r, const char *what, int from = 0, double reset_us = 300) {
  const int bad = wire_problems(r.de, from, reset_us, true);
  CHECK(bad == 0, "%s: %d wire problem(s)", what, bad);
}

// the hub's output for the firmware's model of the receiver (tests/run_ultra_dline_replay_test.py), when asked for:
// ULTRA_TRANSPORT_TRACE=<folder>
static void dump_trace(const char *name, const Rig &r) {
  const char *dir = std::getenv("ULTRA_TRANSPORT_TRACE");
  if (dir == nullptr)
    return;
  const std::string path = std::string(dir) + "/" + name + ".trace";
  FILE *f = std::fopen(path.c_str(), "w");
  if (f == nullptr) {
    std::printf("cannot write %s\n", path.c_str());
    std::exit(2);
  }
  for (const auto &t : g_tx) {
    std::fprintf(f, "T %lld %c %zu", static_cast<long long>(t.start_us), t.poll_frame ? 'Q' : (t.marker ? 'F' : 'P'),
                 t.bytes.size());
    for (uint8_t b : t.bytes)
      std::fprintf(f, " %02x", b);
    std::fprintf(f, " %zu", t.words.size());
    for (uint32_t w : t.words)
      std::fprintf(f, " %x", w);
    std::fprintf(f, "\n");
  }
  for (const auto &e : r.de.events)
    std::fprintf(f, "D %lld %d\n", static_cast<long long>(e.t), e.level ? 1 : 0);
  std::fclose(f);
}

// ------------------------------------------------------------------------------------------------ the tests
static void test_stream_and_polls() {
  Rig r(90);
  r.pix_period = 16667;
  r.next_pix = g_now_us;
  r.poll_period = 100000;
  r.next_poll = g_now_us + 100000;
  r.run(r.t_setup + 3000000);
  const int px = r.count(false), markers = r.count(true), polls = r.count(true, true);
  CHECK(px >= 170, "pixel frames in 3 s: %d", px);
  CHECK(polls >= 15 && polls <= 31, "polls in 3 s: %d", polls);
  CHECK(r.hub.dp_.answered == r.hub.dp_.polls && r.hub.dp_.bo_phase == ULTRA_DP_BO_OK, "every poll answered (%u of %u)",
        r.hub.dp_.answered, r.hub.dp_.polls);
  // rule 3: no marker for a second after the setup
  bool early = false;
  for (const auto &t : g_tx)
    early |= t.marker && t.start_us < r.t_setup + 1000000;
  CHECK(!early, "no marker frame in the first second");
  CHECK(markers >= polls, "marker frames %d", markers);
  CHECK(r.hub.px_frames_ == static_cast<uint32_t>(px), "the hub counts its pixel frames");
  check_wire(r, "stream + polls");
  dump_trace("stream_polls", r);
  std::printf("ok   a stream at 60 fps with a poll every 100 ms (%d pixel frames, %d polls), nothing in the first second\n", px,
              polls);
}

static void test_pixels_copied_and_newest_wins() {
  Rig r(30);
  r.hub.dp_.quiet_until_us = 0;
  std::vector<uint8_t> a(30, 0xAA), b(30, 0x55);
  r.hub.digital_pixels(a.data());
  r.hub.digital_pixels(b.data());   // before the first went out: the newest only
  std::fill(b.begin(), b.end(), 0x11);   // the light draws on in its own buffer
  r.run(g_now_us + 5000);
  std::vector<const Tx *> frames;
  for (const auto &t : g_tx) {
    if (!t.marker)
      frames.push_back(&t);
  }
  CHECK(frames.size() == 1, "one pixel frame (%zu)", frames.size());
  if (!frames.empty())
    CHECK(frames[0]->bytes == std::vector<uint8_t>(30, 0x55), "the newest frame, as it was handed over");
  check_wire(r, "copy");
  std::printf("ok   the pixel frame is a copy, the newest one wins\n");
}

static void test_commands_fairness() {
  Rig r(90);
  r.hub.dp_.quiet_until_us = 0;
  r.pix_period = 2500;
  r.next_pix = g_now_us;
  r.poll_period = 7000;
  r.next_poll = g_now_us + 7000;
  r.cmd_period = 1500;   // a command queue that never empties
  r.next_cmd = g_now_us;
  r.cmd_effect = 0;
  const int64_t t0 = g_now_us;
  r.run(t0 + 400000);
  const int px = r.count(false), polls = r.count(true, true), frames = r.count(true) - polls;
  CHECK(px >= 100, "the stream gets its frames in a command storm (%d of ~160)", px);
  CHECK(polls >= 35, "the polls get their turns (%d of ~57)", polls);
  CHECK(frames >= 60, "and the commands go out (%d)", frames);
  check_wire(r, "command storm");
  dump_trace("storm", r);
  std::printf("ok   a command storm starves neither the stream (%d frames) nor the polls (%d)\n", px, polls);
}

static void test_local_effect_and_handback() {
  Rig r(90);
  r.hub.dp_.quiet_until_us = 0;
  r.pix_period = 16667;
  r.next_pix = g_now_us;
  r.poll_period = 200000;
  r.next_poll = g_now_us + 200000;
  r.run(g_now_us + 300000);
  CHECK(r.count(false) > 10, "streaming first");
  // a local effect on segment 0: no more pixels, a command or two, a header at least once a second
  ultra_pixel_fx_t fx;
  std::memset(&fx, 0, sizeof fx);
  fx.effect = 7;
  fx.brightness = 200;
  r.hub.sched_.light_pixel_fx(0, 0, fx, g_now_us);
  r.hub.digital_note_fx(0, 7, g_now_us, 18000);
  const size_t n0 = g_tx.size();
  const int64_t t_local = g_now_us;
  r.run(t_local + 5000000, 50);
  int px_during = 0;
  std::vector<int64_t> hdr;
  for (size_t i = n0; i < g_tx.size(); i++) {
    if (!g_tx[i].marker)
      px_during++;
    else
      hdr.push_back(g_tx[i].start_us);
  }
  CHECK(px_during <= 1, "no pixel frame during a local effect (%d; one may have been waiting)", px_during);
  int64_t worst = 0;
  for (size_t i = 1; i < hdr.size(); i++)
    worst = std::max(worst, hdr[i] - hdr[i - 1]);
  CHECK(!hdr.empty() && worst <= 1000000 + 25000, "a header at least once a second (worst gap %lld us)", static_cast<long long>(worst));
  CHECK(r.hub.dp_.local != 0, "the planner knows");
  check_wire(r, "local effect", static_cast<int>(n0));
  // the hand-back: effect 0 applied in 40 ms; the stream starts again after the receiver's frame and a reset
  r.pix_period = 0;   // the light has nothing new: the last picture it drew is the one to show again
  const int64_t t_back = g_now_us + 40000;
  r.hub.digital_note_fx(0, 0, t_back, 18000);
  const size_t n1 = g_tx.size();
  r.run(t_back + 300000);
  int first_px = -1;
  for (size_t i = n1; i < g_tx.size(); i++) {
    if (!g_tx[i].marker) {
      first_px = static_cast<int>(i);
      break;
    }
  }
  CHECK(first_px >= 0, "the stream is back");
  {
    // the receiver goes back to the stream only after a LOW as long as its chip's latch: one marker frame first
    int nops = 0, before_px = 0;
    for (size_t i = n1; i < g_tx.size(); i++) {
      if (g_tx[i].start_us >= t_back + 18000 + 300 && g_tx[i].marker && (first_px < 0 || static_cast<int>(i) < first_px))
        before_px++;
      nops += g_tx[i].marker && g_tx[i].start_us >= t_back + 18000;
    }
    CHECK(before_px == 1 && nops >= 1, "a marker frame (a NOP) after the receiver's frame and before the first pixel frame (%d)", before_px);
  }
  if (first_px >= 0)
    CHECK(g_tx[first_px].bytes == r.pixbuf, "the stream's newest picture goes out first (the light draws only on a change)");
  if (first_px >= 0)
    CHECK(g_tx[first_px].start_us >= t_back + 18000 + 300, "the first pixel frame waits for the receiver's frame and a reset (%lld, from %lld)",
          static_cast<long long>(g_tx[first_px].start_us), static_cast<long long>(t_back + 18300));
  check_wire(r, "hand-back", static_cast<int>(n1));
  dump_trace("local", r);
  std::printf("ok   a local effect: no pixels, a header at least every second; the hand-back waits for the receiver's frame\n");
}

static void test_backoff() {
  Rig r(90);
  r.hub.dp_.quiet_until_us = 0;
  r.hub.answer_ = false;
  r.pix_period = 16667;
  r.next_pix = g_now_us;
  r.poll_period = 100000;
  r.next_poll = g_now_us + 100000;
  const int64_t t0 = g_now_us;
  r.run(t0 + 2000000, 100);
  CHECK(r.hub.poller_.stats_.windows == 3, "3 polls in quick succession, then none (%u)", r.hub.poller_.stats_.windows);
  CHECK(r.hub.dp_.bo_phase == ULTRA_DP_BO_WAIT, "waiting for the second series (%u)", r.hub.dp_.bo_phase);
  int markers = r.count(true);
  // a command queued while it waits stays queued: no marker for 60 s
  ultra_pixel_fx_t fx;
  std::memset(&fx, 0, sizeof fx);
  fx.effect = 0;
  r.hub.sched_.light_pixel_fx(0, 0, fx, g_now_us);
  const size_t n_wait = g_tx.size();
  r.run(t0 + 50000000, 500);
  CHECK(r.count(true) == markers, "no marker frame in the wait (%d more)", r.count(true) - markers);
  int px_wait = 0;
  for (size_t i = n_wait; i < g_tx.size(); i++)
    px_wait += !g_tx[i].marker;
  CHECK(px_wait > 1000, "the stream goes on during the wait (%d frames)", px_wait);
  r.run(t0 + 70000000, 500);
  CHECK(r.hub.poller_.stats_.windows == 6, "3 more after 60 s (%u)", r.hub.poller_.stats_.windows);
  CHECK(r.hub.dp_.bo_phase == ULTRA_DP_BO_GAVE_UP, "then it gave up (%u)", r.hub.dp_.bo_phase);
  CHECK(ESPPreferenceObject::saves == 1 && prefs_store.store[r.hub.tel_key_ ^ 0x44504F31u][0] == 1, "kept across restarts (%d saves)",
        ESPPreferenceObject::saves);
  markers = r.count(true);
  const int64_t t_gave_up = g_now_us;
  fx.brightness = 77;
  r.hub.sched_.light_pixel_fx(0, 0, fx, g_now_us);   // a new command: it waits
  r.run(t_gave_up + 30000000, 500);
  CHECK(r.count(true) == markers, "no marker frame after it gave up (%d more)", r.count(true) - markers);
  CHECK(r.hub.sched_.pending(), "the command waits");
  check_wire(r, "back-off");
  // a restart: the flag is read, no poll, the commands wait; then the user's retry
  {
    Rig r2(90);
    CHECK(r2.hub.dp_.bo_phase == ULTRA_DP_BO_GAVE_UP && r2.hub.dp_stored_, "after a restart: still given up");
    r2.hub.dp_.quiet_until_us = 0;
    r2.hub.answer_ = true;
    r2.poll_period = 100000;
    r2.next_poll = g_now_us + 100000;
    r2.pix_period = 16667;
    r2.next_pix = g_now_us;
    r2.run(g_now_us + 3000000, 100);
    CHECK(r2.count(true) == 0 && r2.count(false) > 100, "restarted given up: pixels, no marker (%d / %d)", r2.count(true), r2.count(false));
    r2.hub.digital_retry();
    CHECK(r2.hub.dp_.bo_phase == ULTRA_DP_BO_OK && !r2.hub.dp_stored_, "retry: polled again, the flag cleared");
    CHECK(prefs_store.store[r2.hub.tel_key_ ^ 0x44504F31u][0] == 0, "the cleared flag is stored");
    r2.run(g_now_us + 1000000, 100);
    CHECK(r2.hub.dp_.answered >= 5 && r2.count(true, true) >= 5, "polled and answered again (%u)", r2.hub.dp_.answered);
    check_wire(r2, "after the retry");
  }
  std::printf("ok   the back-off: 3 polls, 60 s without a marker, 3 more, then none; kept across a restart; the retry\n");
  prefs_store.store.clear();
}

static void test_rmt_failures() {
  Rig r(30);
  r.hub.dp_.quiet_until_us = 0;
  ultra_pixel_fx_t fx;
  std::memset(&fx, 0, sizeof fx);
  fx.effect = 0;
  fx.brightness = 99;
  r.hub.sched_.light_pixel_fx(0, 0, fx, g_now_us);
  g_rmt_fail = 2;   // the first two attempts fail
  r.run(g_now_us + 20000);
  CHECK(r.count(true) > 0, "the command frame goes out after the refusals");
  CHECK(r.hub.status_has_warning() == false, "and the warning is cleared");
  // a poll that fails is dropped (asked again), the UART turn given back
  r.hub.poller_.due = true;
  g_rmt_fail = 1;
  const int polls0 = r.count(true, true);
  r.run(g_now_us + 5000);
  CHECK(r.hub.rw_open_ == false || r.count(true, true) > polls0, "a refused poll opens no window");
  CHECK(!r.hub.tx_prepared_, "nothing stays prepared");
  r.run(g_now_us + 20000);
  check_wire(r, "refusals");
  std::printf("ok   a transmission the RMT refuses: the frame is sent again, a poll dropped\n");
}

static void test_checks_catch() {
  Rig r(90);
  r.hub.dp_.quiet_until_us = 0;
  r.pix_period = 5000;
  r.next_pix = g_now_us;
  r.poll_period = 20000;
  r.next_poll = g_now_us + 20000;
  r.run(g_now_us + 200000);
  CHECK(wire_problems(r.de, 0, 300, false) == 0, "the clean run is clean");
  const auto saved = g_tx;
  int found = 0;
  for (size_t i = 2; i < g_tx.size() && found < 30; i++) {
    // the marker too early (the LOW before it too short)
    if (g_tx[i].marker && !g_tx[i - 1].poll_frame) {
      g_tx[i].start_us -= 150;
      found += wire_problems(r.de, static_cast<int>(i), 300, false) > 0;
      g_tx[i] = saved[i];
      break;
    }
  }
  CHECK(found > 0, "negative control: a marker 150 us early is found");
  g_tx = saved;
  found = 0;
  for (size_t i = 2; i < g_tx.size(); i++) {
    if (g_tx[i].marker) {   // a marker of 8 us
      uint32_t &w = g_tx[i].words[g_tx[i].words[0] & 0x8000u ? 0 : 1];
      w = (w & ~0x7FFFu) | (8 * 80);
      found += wire_problems(r.de, static_cast<int>(i), 300, false) > 0;
      break;
    }
  }
  CHECK(found > 0, "negative control: a short marker is found");
  g_tx = saved;
  // pixels 100 us after a marker frame
  for (size_t i = 2; i < g_tx.size(); i++) {
    if (!g_tx[i].marker && g_tx[i - 1].marker && !g_tx[i - 1].poll_frame) {
      g_tx[i].start_us -= 250;
      CHECK(wire_problems(r.de, static_cast<int>(i), 300, false) > 0, "negative control: pixels 250 us early after a frame are found");
      break;
    }
  }
  g_tx = saved;
  std::printf("ok   the wire checks see a marker too early, a short marker and pixels too soon (negative controls)\n");
}

static void test_calibration_quiet_transport() {
  Rig r(180);
  r.hub.dp_.quiet_until_us = 0;
  r.run(g_now_us + 10000);
  r.pix_period = 1000; r.next_pix = g_now_us;
  PixelCalibration c;
  r.hub.pixel_cal_job_.start(0, millis(), c, fixture_cal_wait_ms("ur_digital_m_v203"));
  ultra_pixel_fx_t fx{}; fx.effect = 1; fx.brightness = 77;
  r.hub.sched_.light_pixel_fx(0,0,fx,g_now_us);
  const size_t first = g_tx.size();
  g_rmt_fail = 1;
  r.run(g_now_us + 450000);
  unsigned requests=0;
  int64_t last=0;
  for(size_t i=first;i<g_tx.size();i++) {
    const auto &t=g_tx[i];
    CHECK(t.marker && t.bytes[0] != ULTRA_PIXEL_FX, "quiet job pauses stream and scene commands");
    if(t.bytes[0]==ULTRA_IDENTIFY) {
      requests++;
      CHECK(t.bytes[ULTRA_PAYLOAD]==2 && t.bytes[ULTRA_PAYLOAD+1]==0 && ultra_crc16(t.bytes.data(),30)==ultra_get16(t.bytes.data()+30),
            "CRC sealed request has no installer rating");
    }
    if(last) CHECK(t.start_us-last >= 99000, "sparse clock traffic preserves quiet gaps");
    last=t.start_us;
  }
  CHECK(requests==1 && r.hub.pixel_cal_job_.sent, "one start on wire after refused transmit");
  CHECK(r.hub.sched_.pending() && r.hub.px_ready_, "newest scene and stream retained for release");
  r.hub.pixel_cal_job_.cancel();
  r.run(g_now_us+120000);
  CHECK(g_tx.back().bytes[0]==ULTRA_IDENTIFY && g_tx.back().bytes[ULTRA_PAYLOAD]==4, "cancel uses release action");
  r.hub.pixel_cal_job_.board=-1;
  r.pix_period=0;
  const size_t resume=g_tx.size();
  r.run(g_now_us+30000);
  CHECK(g_tx.size()>resume && !r.hub.px_ready_, "latest buffered stream resumes after release");
  std::printf("ok   calibration pauses streaming and queued commands, sparse clocks, request retry, cancel and resume\n");
}


// A port job takes the digital port's line while the receiver's response window is open: the RMT's
// interrupt has let go of the line (direction pin low). The job's first frame (a marker envelope) waits until the
// window is over and read, and starts with the hub driving the line again; one tried before is refused. The same when
// the job takes the line back after verifying (the hub polled meanwhile).
static void test_job_handover() {
  Rig r(90);
  r.pix_period = 16667;
  r.next_pix = g_now_us;
  r.poll_period = 40000;
  r.next_poll = g_now_us + 100000;
  uint8_t frame[40] = {};
  frame[0] = 0xA5;
  ultra_update_transport_t t{};
  t.lead_low_us = ULTRA_DP_QUIET_US;
  t.tail_low_us = 2;
  t.marker = 1;
  auto tick = [&r]() {   // 25 us; the RMT's done interrupt lets go of the line (the hub's loop: leased, nothing)
    g_now_us += 25;
    if (g_isr_us >= 0 && g_now_us >= g_isr_us) {
      g_isr_us = -1;
      if (r.hub.rw_release_) {
        r.de.digital_write(false);
        r.hub.rw_release_ = false;
        r.hub.rw_released_us_ = g_now_us;
      }
    }
  };
  for (int round = 0; round < 2; round++) {
    const int64_t limit = g_now_us + 3000000;
    while (g_now_us < limit && !(r.hub.rw_open_ && !r.de.level))
      r.run(g_now_us + 25);
    const bool in_window = r.hub.rw_open_ && !r.de.level;
    const uint32_t windows = r.hub.poller_.stats().windows;
    const size_t before = g_tx.size();
    r.hub.lease_line(true);   // (the main loop: service_job_)
    const bool refused = !r.hub.job_send(frame, sizeof(frame), t) && g_tx.size() == before;
    for (int k = 0; k < 4000 && g_tx.size() == before; k++) {
      tick();
      r.hub.lease_line(true);
      r.hub.job_send(frame, sizeof(frame), t);
    }
    const bool out = g_tx.size() == before + 1 && g_tx.back().bytes.size() == sizeof(frame) && g_tx.back().bytes[0] == 0xA5;
    const Tx first = out ? g_tx.back() : Tx{};
    CHECK(in_window && refused && out && first.de_high && first.marker && first.start_us >= r.hub.rw_end_us_ &&
              !r.hub.rw_open_ && r.hub.poller_.stats().windows == windows + 1,
          "%s inside the receiver's window: refused there (%d), on the wire after it was read (%d windows -> %u) with "
          "the direction pin high (%d)", round == 0 ? "a job takes the line" : "the job takes it back after verifying",
          refused, static_cast<int>(windows), r.hub.poller_.stats().windows, first.de_high);
    g_now_us += 2000;
    g_busy_until_us = g_now_us;
    r.hub.lease_line(false);   // (verifying: the hub's own frames and polls again)
    r.run(g_now_us + 500000);
  }
  std::printf("ok   a port job takes the digital port's line inside the receiver's window (and back): its frames wait "
              "for the window, then go out with the direction pin high\n");
}

int main() {
  test_calibration_quiet_transport();
  test_stream_and_polls();
  test_pixels_copied_and_newest_wins();
  test_commands_fairness();
  test_local_effect_and_handback();
  test_backoff();
  test_rmt_failures();
  test_checks_catch();
  test_job_handover();
  std::printf("%s: %d checks, %d failures\n", failures ? "FAILED" : "ALL DIGITAL TRANSPORT TESTS PASSED", checks, failures);
  return failures ? 1 : 0;
}
