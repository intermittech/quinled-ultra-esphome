// Host test of the component's methods that need ESPHome / ESP-IDF, so tests/ultra_host_test.cpp can't compile them:
// tests/run_ultra_lifecycle_test.py cuts their bodies unchanged from components/dig2analog_ultra into build/lifecycle/
// *.inc, and this file compiles them with small stand-ins for ESPHome, the RMT driver and the HTTP client. Covers the
// light's setup, a transition cut short, its power request after weeks; the update's install queue, chunked downloads
// and the confirmation frames after a session; a frame the RMT refuses; the recovery's power cycle on ESPHome's power
// supply (its keep-on time). Logic only: no ESP-IDF, no timing.
// Build: python tests/run_ultra_lifecycle_test.py   (uses the ziglang pip package as C/C++ compiler)

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/fw_manifest.h"
#include "../components/dig2analog_ultra/update_session.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <vector>

#define USE_DIG2ANALOG_ULTRA_UPDATE
#define ESP_LOGE(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGW(...) ((void) 0)

namespace esphome::dig2analog_ultra {

static int failures = 0;
#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      failures++; \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
    } \
  } while (0)

static const char *const TAG = "test";
static uint32_t now_ms = 0;
static uint32_t millis() { return now_ms; }

// ================================================================================================ light
namespace light {
struct LightColorValues {   // what setup_state uses of it (power_on_effect)
  float get_color_temperature() const { return 0.0f; }
  void set_cold_white(float) {}
  void set_warm_white(float) {}
};
struct LightState {
  float gamma{1.0f};
  float get_gamma_correct() const { return this->gamma; }
  uint32_t get_default_transition_length() const { return 1000; }
  template<typename T> void add_remote_values_listener(T *) {}
};
}  // namespace light

namespace lights {

/// The hub as a light sees it.
struct Hub {
  Scheduler sched;
  Hub() { this->sched.init(1, 123, 1); }
  Scheduler &scheduler() { return this->sched; }
  void wake_() {}
  void add_light(void *) {}
  uint32_t start_delay_ms() const { return 30; }
  int64_t start_time_us() const { return now_ms * 1000LL + 30000; }
  int64_t start_time_us(uint16_t) const { return this->start_time_us(); }
};

struct UltraTransition {
  bool finished{false};
  bool is_finished() const { return this->finished; }
};

enum PowerOnScene : uint8_t { SCENE_NONE = 0, SCENE_FIXED };

struct UltraLight {
  explicit UltraLight(Hub &hub) : hub_(&hub) {}
  // the component's (build/lifecycle/light_methods.inc, is_lit.inc)
  void setup_state(light::LightState *state);
  void send_opt_();
  void transition_done_(UltraTransition *t);
  void lit_(uint16_t m, uint32_t length_ms, int64_t start_us);
  void send_pixel(uint16_t m, const uint16_t *a, const uint16_t *b, uint32_t length_ms, uint8_t flags, bool stream,
                  int64_t start_us);
  void start_effect(uint8_t fx, const uint8_t *params, size_t n, const std::vector<Step> *steps, int64_t start_us);
#include "is_lit.inc"
  // stand-ins
  void map(const light::LightColorValues &, uint16_t &m, uint16_t *a, uint16_t *b) const {
    m = 65535;
    for (int i = 0; i < 5; i++)
      a[i] = b[i] = 65535;
  }
  void send_scene_effect_() {
    this->hub_->scheduler().light_scene_effect(this->board_, this->index_, ULTRA_FX_NONE, nullptr, 0);
  }
  void stop_effect() {
    this->hub_->scheduler().light_effect(this->board_, this->index_, ULTRA_FX_NONE, nullptr, 0,
                                         this->hub_->start_time_us());
  }

  static std::vector<UltraLight *> registry_;
  Hub *hub_;
  light::LightState *state_{nullptr};
  UltraTransition *transition_{nullptr};
  void *power_on_effect_{nullptr};
  light::LightColorValues power_on_values_;
  uint8_t scene_mode_{SCENE_NONE};
  bool sent_{false}, on_{false}, fading_out_{false}, zero_means_zero_{false}, constant_brightness_{false};
  bool dtw_available_{false}, dtw_on_{false};
  uint32_t off_until_ms_{0};
  uint16_t board_{0};
  uint8_t index_{0}, dtw_n_{0};
  uint8_t roles_[5] = {ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0};
  uint16_t min_[5] = {0, 0, 0, 0, 0}, max_[5] = {65535, 65535, 65535, 65535, 65535};
  float cold_white_temperature_{0}, warm_white_temperature_{0}, dtw_assist_{0}, dtw_warm_{0};
  float dtw_pb_[ULTRA_OPT_POINTS_MAX] = {}, dtw_pt_[ULTRA_OPT_POINTS_MAX] = {};
};
std::vector<UltraLight *> UltraLight::registry_;

#include "light_methods.inc"

/// setup_state: LIGHT_OPT also without dim_to_warm (no options) and EFFECT none go out, so a board that stayed powered
/// while the ESP rebooted drops an earlier configuration's options and effect. transition_done_: a transition cut
/// short re-arms sending (the board still fades to its end). is_lit: a light never on, or off for weeks, does not
/// request the power supply (millis() wraps every 49.7 days, a signed compare flips after 24.8).
static void test_light() {
  Hub hub;
  UltraLight l(hub);
  light::LightState state;
  l.setup_state(&state);
  std::vector<uint8_t> f(hub.sched.frame_bytes());
  bool opt = false, fx = false;
  for (int i = 0; i < 20; i++) {
    hub.sched.build_frame(i * 1000, f.data());
    opt |= f[0] == ULTRA_LIGHT_OPT && f[ULTRA_PAYLOAD] == 0;
    fx |= f[0] == ULTRA_EFFECT && f[ULTRA_PAYLOAD] == ULTRA_FX_NONE;
  }
  CHECK(opt && fx, "setup: LIGHT_OPT without options (%d) and EFFECT none (%d)", opt, fx);

  // gamma_correct as LIGHT_DEF's gamma x 1000; ESPHome's 0 is linear (its gamma table i / 255): 1000 on the wire
  for (float g : {2.8f, 1.0f, 0.0f}) {
    Hub h2;
    UltraLight l2(h2);
    light::LightState s2;
    s2.gamma = g;
    l2.setup_state(&s2);
    uint32_t sent = 0xFFFFFFFFu;
    for (int i = 0; i < 20 && h2.sched.pending(); i++) {
      h2.sched.build_frame(i * 1000, f.data());
      if (f[0] == ULTRA_LIGHT_DEF)
        sent = ultra_get16(f.data() + ULTRA_PAYLOAD + 3);
    }
    const uint32_t want = g > 0.0f ? static_cast<uint32_t>(g * 1000.0f + 0.5f) : 1000u;
    CHECK(sent == want, "setup: gamma_correct %.1f goes out as gamma x 1000 = %u (%u)", g, want, sent);
  }

  UltraTransition t;
  l.transition_ = &t;
  l.sent_ = true;
  l.transition_done_(&t);
  CHECK(!l.sent_ && l.transition_ == nullptr, "a transition cut short: the next values go out even if equal");
  t.finished = true;
  l.transition_ = &t;
  l.sent_ = true;
  l.transition_done_(&t);
  CHECK(l.sent_, "a finished transition: equal values are not sent again");

  CHECK(!l.is_lit(0x80000001u), "never on: not lit after 24.8 days");
  now_ms = 0xFFFFFFF0u;   // a fade-out across the wrap: 100 ms + the start delay (30 ms)
  l.lit_(65535, 0, hub.start_time_us());
  l.lit_(0, 100, hub.start_time_us());
  CHECK(l.is_lit(now_ms + 50) && !l.is_lit(now_ms + 131) && !l.is_lit(now_ms + 0x80000100u),
        "fading out: lit until its end, across the wrap, and not again weeks later");
  now_ms = 0;
}

/// The supply stays requested until a fade-out has ended on the board: from the SET's own start (a group start or
/// commands waiting before it can put it well after the call), also when a second fade-out moves the end, and across
/// the millis() wrap.
static void test_power_deadline() {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
  Hub hub;
  UltraLight l(hub);
  now_ms = 1000;
  l.on_ = true;
  l.send_pixel(0, one, one, 1000, 0, false, 1600000);   // a 1 s fade-out, starting 600 ms after the call
  CHECK(l.is_lit(2500) && !l.is_lit(2602), "a fade-out starting later: lit until it ends at 2600 ms (%u)",
        static_cast<unsigned>(l.off_until_ms_));
  now_ms = 4000;
  l.on_ = true;
  l.send_pixel(0, one, one, 1000, 0, false, 4030000);
  now_ms = 4500;
  l.send_pixel(0, one, one, 5000, 0, false, 4530000);   // a second, longer fade-out
  CHECK(l.is_lit(9000) && !l.is_lit(9532), "a second fade-out moves the end to 9530 ms (%u)",
        static_cast<unsigned>(l.off_until_ms_));
  now_ms = 0xFFFFFFF0u;
  l.on_ = true;
  l.send_pixel(0, one, one, 100, 0, false, (static_cast<int64_t>(now_ms) + 600) * 1000);
  CHECK(l.is_lit(now_ms + 650) && !l.is_lit(now_ms + 702), "a later fade-out across the millis() wrap");
  now_ms = 0;
}

/// start_effect: 0 is a start (a periodic effect's phase epoch during its first period after boot), NO_START asks
/// the hub for one.
static void test_effect_epoch() {
  uint8_t breathe[3] = {};
  ultra_put24(breathe, 4000);
  const struct {
    int64_t start, want;
  } cases[] = {{0, 0}, {4000000, 4000000}, {NO_START, 130000}};   // the hub's: now (100 ms) + 30 ms
  for (const auto &c : cases) {
    Hub hub;
    UltraLight l(hub);
    now_ms = 100;
    l.start_effect(ULTRA_FX_BREATHE, breathe, 3, nullptr, c.start);
    std::vector<uint8_t> f(hub.sched.frame_bytes());
    const int64_t end = hub.sched.build_frame(100000, f.data());
    const int64_t at = end + static_cast<int32_t>(ultra_get32(f.data() + ULTRA_PAYLOAD + 1)) * 1000LL;
    CHECK(f[0] == ULTRA_EFFECT && std::llabs(at - c.want) <= 1000, "effect start %lld us on the wire (want %lld)",
          static_cast<long long>(at), static_cast<long long>(c.want));
  }
  now_ms = 0;
}

}  // namespace lights

// ================================================================================================ sunrise / sunset
namespace sun {

static int performed = 0;   // light calls performed: the effect's end
namespace light {
struct LightCall {
  void set_effect(uint32_t) {}
  void set_transition_length(uint32_t) {}
  void perform() { performed++; }
};
struct LightState {
  void *get_output() { return nullptr; }
  std::string get_name() const { return "sun"; }
  LightCall make_call() { return {}; }
  LightCall turn_off() { return {}; }
};
struct LightEffect {
  explicit LightEffect(const char *name) : name_(name) {}
  virtual ~LightEffect() = default;
  virtual void start() {}
  virtual void stop() {}
  virtual void apply() {}
  const char *name_;
  LightState *state_{nullptr};
};
}  // namespace light

/// The light and hub as the effects see them: the hub's start for the effect is `at`.
struct Hub {
  int64_t at{0};
  int64_t start_time_us(uint16_t) const { return this->at; }
};
struct UltraLight {
  Hub hub;
  static UltraLight *find(void *);
  Hub *get_hub() { return &this->hub; }
  uint16_t get_board() const { return 0; }
  void start_effect(uint8_t, const uint8_t *, size_t, const std::vector<Step> *, int64_t) {}
  void stop_effect() {}
};
static UltraLight the_light;
UltraLight *UltraLight::find(void *) { return &the_light; }
static const char *const EFFECT_TAG = "test";

#include "sun_effect.inc"

/// Sunrise / sunset end (their light call) from the board's start of the effect, not the call: a start 600 ms ahead
/// (a group start, commands waiting) ends 600 ms later, once, also across the millis() wrap.
static void test_sun_end() {
  for (bool rise : {true, false}) {
    for (uint32_t begin : {1000u, 0xFFFFFF00u}) {
      performed = 0;
      now_ms = begin;
      light::LightState state;
      UltraSunEffect fx("sun", rise);
      fx.state_ = &state;
      fx.set_duration(1000);
      the_light.hub.at = (static_cast<int64_t>(begin) + 600) * 1000;
      fx.start();
      bool early = false;
      for (uint32_t t : {0u, 1100u, 1600u}) {   // the end: 1600 ms + 100 ms grace
        now_ms = begin + t;
        fx.apply();
        early |= performed != 0;
      }
      now_ms = begin + 1700;
      fx.apply();
      fx.apply();
      CHECK(!early && performed == 1, "%s from %u: ends 1.7 s after the call, once (%d)", rise ? "sunrise" : "sunset",
            static_cast<unsigned>(begin), performed);
    }
  }
  now_ms = 0;
}

}  // namespace sun

// ================================================================================================ update entity
namespace update {
enum UpdateState : uint8_t { UPDATE_STATE_INSTALLING = 1 };
}

namespace http_request {
static constexpr int HTTP_STATUS_OK = 200;
enum class HttpReadLoopResult { DATA, RETRY, COMPLETE, ERROR };
enum class HttpReadStatus { OK, ERROR };
struct HttpReadResult {
  HttpReadStatus status;
};
/// A response: its body and Content-Length (0: chunked, as ESPHome reports it).
struct Container {
  int status_code{200};
  size_t content_length{0}, pos{0};
  std::vector<uint8_t> body;
  bool ended{false};
  int read(uint8_t *out, size_t n) {
    n = std::min(n, this->body.size() - this->pos);
    std::memcpy(out, this->body.data() + this->pos, n);
    this->pos += n;
    return static_cast<int>(n);
  }
  bool is_read_complete() const { return this->pos == this->body.size(); }
  size_t get_bytes_read() const { return this->pos; }
  void end() { this->ended = true; }
};
/// ESPHome's: 0 bytes and not complete is a RETRY until the timeout; here at once an error (a connection cut short).
static HttpReadLoopResult http_read_loop_result(int got, uint32_t &, uint32_t, bool complete) {
  return got > 0 ? HttpReadLoopResult::DATA : complete ? HttpReadLoopResult::COMPLETE : HttpReadLoopResult::ERROR;
}
static HttpReadResult http_read_fully(Container *c, uint8_t *buf, size_t n, size_t, uint32_t) {
  return {c->read(buf, n) == static_cast<int>(n) ? HttpReadStatus::OK : HttpReadStatus::ERROR};
}
struct Client {
  std::shared_ptr<Container> response;
  std::shared_ptr<Container> get(const std::string &) { return this->response; }
  uint32_t get_timeout() const { return 1000; }
};
}  // namespace http_request

namespace updates {

static constexpr float DOWNLOAD_SHARE = 15.0f;
typedef void *TaskHandle_t;
static constexpr int pdPASS = 1;
static void *task_arg = nullptr;   // the last task's argument
static int xTaskCreate(void (*)(void *), const char *, int, void *arg, int, TaskHandle_t *) {
  task_arg = arg;
  return pdPASS;
}
/// No other line has the image: the install downloads it. Remembers what it was asked for.
struct ImageCache {
  std::string sha256;
  size_t size{0};
  uint8_t *acquire(const std::string &sha, size_t n) {
    this->sha256 = sha;
    this->size = n;
    return nullptr;
  }
  void trim() {}
} image_cache;

struct UltraFirmwareUpdate {
  enum Phase : uint8_t { IDLE, QUEUED, DOWNLOADING, WAIT_POWER, SENDING };
  struct DownloadResult {
    UltraFirmwareUpdate *self;
    FwImageInfo image;
    std::string url;
    uint32_t version;
    uint8_t hw_rev;
  };
  // the component's (build/lifecycle/update_methods.inc)
  bool fetch_(const std::string &url, std::vector<uint8_t> &out, size_t max_len);
  void start_install_();
  bool take_shared_image_();
  /// loop()'s QUEUED step: the install's turn (build/lifecycle/update_queued.inc)
  void queued_step() {
    switch (this->phase_) {
#include "update_queued.inc"
      default:
        break;
    }
  }
  // stand-ins
  static void download_task_(void *) {}
  std::string get_name() const { return "test"; }
  void image_ready_() { this->phase_ = WAIT_POWER; }
  void set_status_(const std::string &) {}
  void publish_progress_(float, bool) {}
  void finish_(const std::string &, bool) { this->phase_ = IDLE; }

  static UltraFirmwareUpdate *active_;
  http_request::Client *http_{nullptr};
  Phase phase_{IDLE};
  uint8_t state_{0};
  uint32_t phase_ms_{0}, latest_{0}, job_version_{0};
  std::atomic<bool> dl_abort_{false};
  std::atomic<uint32_t> dl_bytes_{0};
  FwImageInfo image_, job_image_;
  std::string image_url_, job_url_;
  uint8_t hw_rev_{3};
  uint8_t *img_{nullptr};
  size_t img_size_{0};
};
UltraFirmwareUpdate *UltraFirmwareUpdate::active_ = nullptr;

#include "update_methods.inc"

/// An install waiting for another line keeps the release it was started for when a check finishes or the channel
/// changes meanwhile: the image, its URL and version (also for the other line's shared image).
static void test_install_queue() {
  UltraFirmwareUpdate u;
  u.image_.size = 1000;
  u.image_.sha256 = std::string(64, 'a');
  u.image_url_ = "chosen.bin";
  u.latest_ = 0x0602FF;
  u.start_install_();
  u.latest_ = 0;   // a check (another channel) meanwhile
  u.image_.size = 2000;
  u.image_.sha256 = std::string(64, 'b');
  u.image_url_ = "other.bin";
  u.queued_step();
  auto *r = static_cast<UltraFirmwareUpdate::DownloadResult *>(task_arg);
  CHECK(r != nullptr && r->version == 0x0602FF && r->image.size == 1000 && r->url == "chosen.bin",
        "a queued install downloads the release it was started for");
  CHECK(image_cache.sha256 == std::string(64, 'a') && image_cache.size == 1000,
        "and looks for that one in the other lines' image");
  delete r;
  UltraFirmwareUpdate::active_ = nullptr;
}

/// Manifest / signature downloads, with Content-Length and chunked (none): read up to the limit, refused when longer
/// or cut short, the connection closed every time.
static void test_fetch() {
  struct Case {
    size_t body, content_length;
    bool ok;
    const char *what;
  };
  const Case cases[] = {{72, 0, true, "chunked"},
                        {72, 72, true, "with Content-Length"},
                        {161, 0, false, "chunked, longer than the limit"},
                        {72, 73, false, "cut short"}};
  UltraFirmwareUpdate u;
  http_request::Client http;
  u.http_ = &http;
  for (const Case &c : cases) {
    http.response = std::make_shared<http_request::Container>();
    http.response->body.assign(c.body, 0x35);
    http.response->content_length = c.content_length;
    std::vector<uint8_t> out;
    const bool ok = u.fetch_("manifest.json.sig", out, 160);
    CHECK(ok == c.ok && (!ok || out.size() == c.body), "fetch, %s: %s", c.what, ok ? "taken" : "refused");
    CHECK(http.response->ended, "fetch, %s: the connection is closed", c.what);
  }
}

}  // namespace updates

// ================================================================================================ hub
static constexpr int ESP_OK = 0;
using esp_err_t = int;
static constexpr uint32_t BUSY_HOLD_MS = 250;
static bool rmt_idle = true;   // the last frame (and its reset) is out
static esp_err_t rmt_tx_wait_all_done(int, int) { return rmt_idle ? ESP_OK : -1; }

namespace hub_loop {

/// The hub around its loop(): an update session just finished; the frames after it as counters.
struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  enum UpdateState : uint8_t { UPD_IDLE = 0, UPD_RUNNING, UPD_FINISHED };
  void loop();   // the component's (build/lifecycle/hub_loop.inc)
  // stand-ins
  struct {
    void stop() {}
  } high_freq_;
  void service_calibration_() {}
  void resync() {   // preamble, PING burst, dark tail
    this->preamble_ = 2;
    this->ping_left_ = 2;
    this->dark_tail_ = 1;
  }
  void transmit_() {
    if (this->preamble_ > 0)
      this->preamble_--;
    else if (this->ping_left_ > 0)
      this->ping_left_--;
    else if (this->dark_tail_ > 0)
      this->dark_tail_--;
    this->has_sent_ = true;
    this->last_frame_ms_ = millis();
    rmt_idle = false;
  }

  std::atomic<uint8_t> upd_state_{UPD_FINISHED};
  std::atomic<uint8_t> upd_result_{UPD_DONE};
  void *upd_task_{nullptr};
  bool upd_quiet_{false}, upd_resync_pending_{false}, tx_prepared_{false};
  uint32_t upd_quiet_ms_{0}, upd_confirm_ms_{0};
  UpdateResult upd_done_{UPD_NONE};
  uint16_t preamble_{0}, ping_left_{0};
  uint8_t dark_tail_{0};
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};
  Scheduler sched_;
  uint32_t last_busy_ms_{0}, last_frame_ms_{0}, refresh_interval_ms_{200};
  bool high_freq_active_{false}, has_sent_{false};
  int channel_{0};
};

#include "hub_loop.inc"

/// After a session and its quiet second, the result waits until the resync's PING burst and dark tail are out on the
/// wire: the boards confirm their new firmware with it, and the update holds the power supply until its result. A
/// transmitter that can't send: failed after 60 s.
static void test_update_confirmation() {
  UltraHub h;
  rmt_idle = true;
  now_ms = 0;
  h.loop();   // the session finished: the line stays quiet
  now_ms = 1001;
  h.loop();   // the resync and its first frame
  CHECK(h.upd_done_ == UltraHub::UPD_NONE, "the quiet second alone gives no result");
  for (int i = 0; i < 5; i++) {
    rmt_idle = true;
    now_ms += 20;
    h.loop();
    if (h.preamble_ || h.ping_left_ || h.dark_tail_)
      CHECK(h.upd_done_ == UltraHub::UPD_NONE, "no result while the preamble / PING burst goes out");
  }
  rmt_idle = false;
  now_ms += 201;
  h.loop();
  CHECK(h.upd_done_ == UltraHub::UPD_NONE, "no result while the last frame is still on the wire");
  rmt_idle = true;
  h.loop();
  CHECK(h.upd_done_ == UltraHub::UPD_DONE, "the result once the dark tail is out");

  UltraHub broken;
  rmt_idle = true;
  now_ms = 0;
  broken.loop();
  now_ms = 1001;
  broken.loop();
  rmt_idle = false;   // never done again
  now_ms = 61001;
  broken.loop();
  CHECK(broken.upd_done_ == UltraHub::UPD_FAILED, "a transmitter that can't send: failed after 60 s");
  rmt_idle = true;
  now_ms = 0;
}

}  // namespace hub_loop

namespace hub_transmit {

struct rmt_transmit_config_t {
  uint32_t loop_count;
};
static int refusals = 0;            // rmt_transmit refuses this many, then sends
static std::vector<uint8_t> sent;   // the last frame it sent
static esp_err_t rmt_transmit(int, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  if (refusals > 0) {
    refusals--;
    return -1;
  }
  sent.assign(data, data + n);
  return ESP_OK;
}

/// The hub around its transmit_(), one board.
struct UltraHub {
  UltraHub() { this->sched_.init(1, 44, 1); }
  void transmit_();   // the component's (build/lifecycle/hub_transmit.inc)
  // stand-ins
  static int64_t now_us_() { return now_ms * 1000LL; }
  bool boards_powered_() const { return true; }
  bool status_has_warning() const { return this->warning_; }
  void status_set_warning() { this->warning_ = true; }
  void status_clear_warning() { this->warning_ = false; }

  Scheduler sched_;
  uint16_t num_boards_{1};
  uint8_t buf_[64]{};
  uint8_t *tx_buf_{buf_};
  bool tx_prepared_{false}, tx_normal_{true}, tx_preamble_end_{false};
  size_t tx_bytes_{0};
  uint16_t preamble_{0}, ping_left_{0};
  uint8_t dark_tail_{0};
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};
  bool preamble_full_{true}, synced_once_{false}, has_sent_{false}, warning_{false};
  uint32_t last_frame_ms_{0}, frames_{0}, build_peak_us_{0};
  int channel_{0}, encoder_{0};
};

#include "hub_transmit.inc"

/// A frame the RMT refuses is sent again, not built anew: the only copy of a command (repeats 1) still goes out, with
/// a new TIME. The last frame of a full preamble counts once it is out.
static void test_transmit_retry() {
  UltraHub h;
  h.sched_.identify(0, true);
  refusals = 2;
  sent.clear();
  now_ms = 10;
  h.transmit_();
  now_ms = 20;
  h.transmit_();
  CHECK(sent.empty() && h.tx_prepared_ && h.status_has_warning(), "refused twice: the frame waits");
  now_ms = 30;
  h.transmit_();
  uint32_t time = 0;
  CHECK(sent.size() == ultra_frame_bytes(1) && sent[0] == ULTRA_IDENTIFY &&
            ultra_parse_header(sent.data() + ULTRA_SLOT_BYTES, &time) == 0 && time >= 30000 &&
            !h.status_has_warning(),
        "then it goes out with the IDENTIFY and a new TIME (%u us)", static_cast<unsigned>(time));

  UltraHub boot;
  boot.preamble_ = 1;
  refusals = 1;
  boot.transmit_();
  CHECK(!boot.synced_once_, "the last preamble frame refused: not synced yet");
  boot.transmit_();
  CHECK(boot.synced_once_, "once it is out: synced");
  now_ms = 0;
}

}  // namespace hub_transmit

// ================================================================================================ hub: power supply
namespace hub_power {

namespace power_supply {
/// ESPHome's PowerSupply as it behaves (power_supply.cpp): the requests are counted; the pin goes high at the first
/// request, and low keep_on_time after the last one (a timer the next request cancels). is_enabled() counts the
/// requests, not the pin.
struct PowerSupply {
  uint32_t keep_on_ms{10000};
  int requests{0};
  bool pin{false};
  bool off_timer{false};
  uint32_t off_at{0};
  bool is_enabled() const { return this->requests != 0; }
  void request_high_power() {
    if (this->requests == 0) {
      this->off_timer = false;
      this->pin = true;
    }
    this->requests++;
  }
  void unrequest_high_power() {
    if (this->requests == 0)
      return;
    if (--this->requests == 0) {
      this->off_timer = true;
      this->off_at = now_ms + this->keep_on_ms;
    }
  }
  /// the scheduler's turn: the keep-on timer
  void tick() {
    if (this->off_timer && static_cast<int32_t>(now_ms - this->off_at) >= 0) {
      this->off_timer = false;
      this->pin = false;
    }
  }
};
struct PowerSupplyRequester {
  PowerSupply *parent_{nullptr};
  bool requested_{false};
  void set_parent(PowerSupply *p) { this->parent_ = p; }
  void request() {
    if (!this->requested_) {
      this->parent_->request_high_power();
      this->requested_ = true;
    }
  }
  void unrequest() {
    if (this->requested_) {
      this->parent_->unrequest_high_power();
      this->requested_ = false;
    }
  }
};
}  // namespace power_supply

struct UltraLight {
  bool lit{false};
  bool is_lit(uint32_t) const { return this->lit; }
};

/// The hub around service_power_() and power_cycle() (build/lifecycle/hub_power.inc).
struct UltraHub {
  enum PowerCycle : uint8_t { CYCLE_NONE = 0, CYCLE_WAIT_OFF, CYCLE_OFF, CYCLE_DONE, CYCLE_FAILED };
  void service_power_();   // the component's
  void power_cycle();      // the component's
  // stand-ins
  bool has_power_supply() const { return this->psu_ != nullptr; }

  power_supply::PowerSupply *psu_{nullptr};
  power_supply::PowerSupplyRequester psu_req_;
  uint32_t psu_keep_on_ms_{0};
  bool psu_requested_{false}, psu_released_{false};
  uint32_t psu_released_ms_{0};
  bool power_waiting_{false};
  uint32_t power_on_ms_{0}, power_on_delay_ms_{100};
  bool cal_power_hold_{false}, upd_power_hold_{false};
  std::vector<UltraLight *> lights_;
  PowerCycle cycle_{CYCLE_NONE};
  uint32_t cycle_ms_{0};
  uint8_t pin_{16};
};

#define USE_POWER_SUPPLY
#include "hub_power.inc"
#undef USE_POWER_SUPPLY

/// recover's power cycle (dig2analog_ultra.recover with power_cycle: true): the boards must really lose power.
/// ESPHome's PowerSupply keeps its pin high for keep_on_time (10 s by default) after the hub's release and
/// is_enabled() only counts the requests, so the cycle has to wait for that time; a re-request before it cancels the
/// switch-off and the boards never reboot. Afterwards the supply is on again and the boards' state is replayed
/// (power_waiting_).
static void test_power_cycle() {
  power_supply::PowerSupply psu;
  psu.keep_on_ms = 10000;
  UltraHub h;
  h.psu_ = &psu;
  h.psu_req_.set_parent(&psu);
  h.psu_keep_on_ms_ = psu.keep_on_ms;
  now_ms = 1000;
  h.upd_power_hold_ = true;   // an update session holds the supply (fw_update.cpp image_ready_)
  h.service_power_();
  psu.tick();
  CHECK(psu.pin && h.psu_requested_ && h.power_waiting_, "the update's hold switches the supply on");
  h.power_waiting_ = false;
  // the recovery's power cycle, serviced every 10 ms
  h.power_cycle();
  const uint32_t t0 = now_ms;
  uint32_t low_ms = 0, low_from = 0;
  for (int i = 0; i < 4000 && h.cycle_ != UltraHub::CYCLE_DONE && h.cycle_ != UltraHub::CYCLE_FAILED; i++) {
    now_ms += 10;
    h.service_power_();
    psu.tick();
    if (!psu.pin) {
      if (low_ms == 0)
        low_from = now_ms - t0;
      low_ms += 10;
    }
  }
  CHECK(h.cycle_ == UltraHub::CYCLE_DONE, "the cycle completes (state %u)", h.cycle_);
  CHECK(low_ms >= 1500 && low_ms <= 1600, "the boards really lost power for 1.5 s (pin low %u ms, from +%u ms)", low_ms,
        low_from);
  CHECK(low_from >= psu.keep_on_ms && low_from <= psu.keep_on_ms + 50,
        "... once the supply's keep-on time was over (+%u ms)",
        low_from);
  CHECK(psu.pin && h.psu_requested_, "the supply is on again afterwards");
  CHECK(h.power_waiting_, "and the boards' state is replayed (power_waiting)");

  // another component keeps the supply requested: the cycle gives up (the status tells the user to power-cycle by hand)
  power_supply::PowerSupply psu2;
  UltraHub h2;
  h2.psu_ = &psu2;
  h2.psu_req_.set_parent(&psu2);
  h2.psu_keep_on_ms_ = psu2.keep_on_ms;
  h2.upd_power_hold_ = true;
  h2.service_power_();
  psu2.request_high_power();   // e.g. a ws2915 line on the same supply
  h2.power_cycle();
  for (int i = 0; i < 4000 && h2.cycle_ == UltraHub::CYCLE_WAIT_OFF; i++) {
    now_ms += 10;
    h2.service_power_();
    psu2.tick();
  }
  CHECK(h2.cycle_ == UltraHub::CYCLE_FAILED && psu2.pin,
        "kept on by another requester: the cycle fails, the supply stays on");
  now_ms = 0;
}

}  // namespace hub_power

// ================================================================================================ hub: start times
namespace hub_start {

/// The hub around its start functions (build/lifecycle/hub_start.inc): a 50-board line, start_delay auto.
struct UltraHub {
  int64_t lead_us(size_t commands, size_t waiting = 0) const;   // the component's
  int64_t command_lead_us_() const;                              // the component's
  int64_t start_time_us(uint16_t board);                         // the component's
  static int64_t now_us_() { return 10000000; }

  Scheduler sched_;
  uint32_t frame_us_{17670};   // 50 boards, the default timing
  int64_t start_delay_us_{-1};
  uint8_t max_lights_{5}, max_commands_{10};
  static std::vector<UltraHub *> hubs_;
  static GroupStart group_;
};
std::vector<UltraHub *> UltraHub::hubs_;
GroupStart UltraHub::group_;

#include "hub_start.inc"

/// One Home Assistant call turning off five effect lights of one board: each light's EFFECT none and its fade-out go
/// out at one start, budgeted by max_commands_ (codegen: two commands per effect light). A budget of one command per
/// light splits the change between a light's EFFECT and its SET (test_effect_stop_fade: the light flashes to full).
static void test_group_budget() {
  const uint16_t one[5] = {65535, 65535, 65535, 65535, 65535};
  size_t groups[2] = {};
  const uint8_t budgets[2] = {10, 5};
  for (int i = 0; i < 2; i++) {
    UltraHub h;
    h.sched_.init(50, 1, 3);
    h.max_commands_ = budgets[i];
    UltraHub::hubs_ = {&h};
    UltraHub::group_ = GroupStart();
    std::set<int64_t> starts;
    for (uint8_t l = 0; l < 5; l++) {
      const int64_t stop = h.start_time_us(0);
      h.sched_.light_effect(0, l, ULTRA_FX_NONE, nullptr, 0, stop);
      const int64_t fade = h.start_time_us(0);
      h.sched_.light_set(0, l, ULTRA_SET_SMOOTH, 0, one, one, 1000, fade);
      starts.insert(stop);
      starts.insert(fade);
    }
    groups[i] = starts.size();
  }
  CHECK(groups[0] == 1, "five effect lights off in one call: one start for every EFFECT none and fade-out (%u)",
        static_cast<unsigned>(groups[0]));
  std::printf("  group budget: 5 effect lights off: %u start(s) with two commands per light, %u with one\n",
              static_cast<unsigned>(groups[0]), static_cast<unsigned>(groups[1]));
  UltraHub::hubs_.clear();
}

}  // namespace hub_start

// ================================================================================================ hub: update sender
namespace hub_update {

struct rmt_transmit_config_t {
  uint32_t loop_count;
};
static uint32_t portTICK_PERIOD_MS = 1;   // ESPHome's 1 kHz FreeRTOS tick; also run at 100 Hz
static void vTaskDelete(void *) {}

/// The hub around its update task and frame times (build/lifecycle/hub_update.inc).
struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  enum UpdateState : uint8_t { UPD_IDLE = 0, UPD_RUNNING, UPD_FINISHED };
  static void update_task_(void *arg);    // the component's
  uint32_t update_frame_us() const;       // the component's
  uint32_t update_frame_min_us() const;   // the component's
  static int64_t now_us_() { return 5000000; }

  int channel_{0}, encoder_{1};
  uint8_t buf_[64]{};
  uint8_t *tx_buf_{buf_};
  uint32_t bit0_high_ns_{250}, bit0_low_ns_{1050}, bit1_high_ns_{700}, bit1_low_ns_{650}, reset_ns_{300000};
  UpdateSession upd_session_;
  std::atomic<bool> upd_abort_{false};
  std::atomic<uint32_t> upd_sent_{0};
  std::atomic<uint8_t> upd_pass_{0}, upd_result_{UPD_NONE}, upd_state_{UPD_RUNNING};
};

/// The line as the task sees it: rmt_transmit hands one frame over, rmt_tx_wait_all_done returns once it and its
/// reset are out, vTaskDelay(n) keeps the line low for n - 1 to n ticks (it ends at the n-th tick from a random phase).
struct Line {
  UltraHub *hub{nullptr};
  std::vector<std::vector<uint8_t>> wire;   // the frames, in order
  std::vector<uint32_t> quiet_us;           // per frame: the least low time after its reset (the delays' worst phase)
  bool on_wire{false};
  int overlapped{0}, refused{0};
  size_t abort_at{SIZE_MAX};                // the abort request comes with this frame
};
static Line line;
static esp_err_t rmt_transmit(int, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  if (n != UPD_FRAME_BYTES) {
    line.refused++;
    return -1;
  }
  line.overlapped += line.on_wire;   // a frame handed over while the last one is still out
  line.wire.emplace_back(data, data + n);
  line.quiet_us.push_back(0);
  line.on_wire = true;
  if (line.wire.size() == line.abort_at)
    line.hub->upd_abort_.store(true);
  return ESP_OK;
}
static esp_err_t rmt_tx_wait_all_done(int, int) {
  line.on_wire = false;
  return ESP_OK;
}
static void vTaskDelay(uint32_t ticks) {
  line.overlapped += line.on_wire;   // a pause that starts before its frame and reset are out
  if (!line.quiet_us.empty() && ticks > 0)
    line.quiet_us.back() += (ticks - 1) * portTICK_PERIOD_MS * 1000;
}

#include "hub_update.inc"

/// The bootloader's page pause after a frame's reset (its README: "the host's 2.7 ms pause" covers a page program at
/// HCLK / 2 and the decoder's resync): the update task keeps the line low at least that long after every page's last
/// chunk, at any tick phase, at a 1 kHz and a 100 Hz FreeRTOS tick; the default asks for no less. An abort request
/// still ends with 12 ABORT frames.
static void test_update_task() {
  const uint32_t bl_page_pause_us = 2700;
  CHECK(UpdateSession::Options().page_pause_us >= bl_page_pause_us, "the default page pause is the bootloader's "
        "2.7 ms or more (%u us)", static_cast<unsigned>(UpdateSession::Options().page_pause_us));
  static uint8_t img[UPD_DESC_BYTES + 2000];
  for (size_t i = 0; i < sizeof(img); i++)
    img[i] = static_cast<uint8_t>(i * 7 + 3);
  UpdateSession::Options o;
  o.lead_in_us = 20000;
  o.end_us = 10000;
  for (uint32_t tick_ms : {1u, 10u}) {
    portTICK_PERIOD_MS = tick_ms;
    for (int run = 0; run < 2; run++) {   // a whole session, an abort
      UltraHub h;
      line = Line();
      line.hub = &h;
      h.upd_session_.begin(img, sizeof(img), 0x42, o, h.update_frame_min_us());
      UpdateSession ref;   // the same session, frame by frame
      ref.begin(img, sizeof(img), 0x42, o, h.update_frame_min_us());
      if (run == 1)
        line.abort_at = 100;
      UltraHub::update_task_(&h);
      size_t session = 0, pauses = 0;
      uint32_t least = UINT32_MAX;
      bool same = true, paused = true;
      for (size_t i = 0; i < line.wire.size(); i++) {
        uint8_t f[UPD_FRAME_BYTES];
        uint32_t pause_us = 0;
        if (line.wire[i][UPD_B_TYPE] == UPD_ABORT)
          break;
        same &= ref.next(f, static_cast<uint32_t>(UltraHub::now_us_()), &pause_us) &&
                std::memcmp(f, line.wire[i].data(), UPD_FRAME_BYTES) == 0;
        if (pause_us != 0 && i + 1 < line.wire.size()) {   // the next frame waits for the page program
          paused &= pause_us >= bl_page_pause_us && line.quiet_us[i] >= pause_us;
          least = std::min(least, line.quiet_us[i]);
          pauses++;
        }
        session++;
      }
      const size_t aborts = line.wire.size() - session;
      CHECK(same && paused && line.overlapped == 0 && line.refused == 0, "update task, %u ms tick, run %d: %u frames "
            "as the session built them, after %u pages the line low >= the page pause (least %u us)", tick_ms, run,
            static_cast<unsigned>(session), static_cast<unsigned>(pauses), least);
      if (run == 0) {
        CHECK(h.upd_result_.load() == UltraHub::UPD_DONE && session == ref.total_frames() && pauses == 3 * 8,
              "a whole session: done, %u frames, %u page pauses (3 passes x 8 pages)", static_cast<unsigned>(session),
              static_cast<unsigned>(pauses));
      } else {
        CHECK(h.upd_result_.load() == UltraHub::UPD_ABORTED && session == 100 && aborts == 12,
              "an abort with frame 100: the frames up to it, then 12 ABORT frames (%u, %u)",
              static_cast<unsigned>(session), static_cast<unsigned>(aborts));
      }
    }
  }
  portTICK_PERIOD_MS = 1;
  std::printf("  update task: after every page the line stays low >= %u us (1 kHz and 100 Hz tick)\n",
              static_cast<unsigned>(UpdateSession::Options().page_pause_us));
}

/// The lead-in (the boards erase meanwhile) and END, counted in update_frame_min_us(): on the wire at least as long as
/// asked even with no gap at all between frames, at 1 and 50 boards, for the default bit timing and one where the '1'
/// is the shorter bit. Before, they were counted at the longer bit + 50 us.
static void test_update_lead_in() {
  static uint8_t img[UPD_DESC_BYTES + 2000];
  for (size_t i = 0; i < sizeof(img); i++)
    img[i] = static_cast<uint8_t>(i * 13 + 5);
  struct Timing {
    uint32_t b0h, b0l, b1h, b1l, reset;
  };
  const Timing timings[] = {{250, 1050, 700, 650, 300000}, {700, 800, 300, 600, 280000}};
  for (const Timing &t : timings) {
    UltraHub h;
    h.bit0_high_ns_ = t.b0h;
    h.bit0_low_ns_ = t.b0l;
    h.bit1_high_ns_ = t.b1h;
    h.bit1_low_ns_ = t.b1l;
    h.reset_ns_ = t.reset;
    for (uint16_t boards : {1, 50}) {
      UpdateSession::Options o;
      o.boards = boards;
      uint64_t lead_ns[2] = {0, 0}, end_ns[2] = {0, 0};
      for (int k = 0; k < 2; k++) {   // now, before
        UpdateSession s;
        s.begin(img, sizeof(img), 0x42, o, k == 0 ? h.update_frame_min_us() : h.update_frame_us() + 50);
        uint8_t f[UPD_FRAME_BYTES];
        uint32_t pause_us;
        bool lead = true;
        while (s.next(f, 0, &pause_us)) {
          uint64_t ns = t.reset;   // the frame on the wire: its bits at their own length, then the reset
          for (size_t b = 0; b < UPD_FRAME_BITS; b++)
            ns += (f[b >> 3] & (0x80u >> (b & 7u))) ? t.b1h + t.b1l : t.b0h + t.b0l;
          lead &= f[UPD_B_TYPE] == UPD_ANNOUNCE;
          lead_ns[k] += lead ? ns : 0;
          end_ns[k] += f[UPD_B_TYPE] == UPD_END ? ns : 0;
        }
      }
      const uint64_t lead_asked = (o.lead_in_us + static_cast<uint64_t>(o.lead_in_per_board_us) * boards) * 1000;
      CHECK(lead_ns[0] >= lead_asked && end_ns[0] >= o.end_us * 1000ULL, "lead-in %.1f ms (%.1f asked), END %.1f ms "
            "(%.1f asked) with no gap between frames", lead_ns[0] / 1e6, lead_asked / 1e6, end_ns[0] / 1e6,
            o.end_us / 1e3);
      if (t.b0h == 250 && boards == 50)
        std::printf("  lead-in at 50 boards with no gap between frames: %.0f ms for %.0f asked (before: %.0f ms, %.1f %% "
                    "short); END %.0f ms for %.0f (before: %.0f)\n", lead_ns[0] / 1e6, lead_asked / 1e6,
                    lead_ns[1] / 1e6, 100.0 * (1.0 - static_cast<double>(lead_ns[1]) / lead_asked), end_ns[0] / 1e6,
                    o.end_us / 1e3, end_ns[1] / 1e6);
    }
  }
}

}  // namespace hub_update

}  // namespace esphome::dig2analog_ultra

using namespace esphome::dig2analog_ultra;

int main() {
  lights::test_light();
  updates::test_install_queue();
  updates::test_fetch();
  hub_loop::test_update_confirmation();
  hub_transmit::test_transmit_retry();
  hub_power::test_power_cycle();
  lights::test_power_deadline();
  lights::test_effect_epoch();
  sun::test_sun_end();
  hub_start::test_group_budget();
  hub_update::test_update_task();
  hub_update::test_update_lead_in();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA LIFECYCLE TESTS PASSED", failures);
  return failures ? 1 : 0;
}
