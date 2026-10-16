// Host test of the component's methods that need ESPHome / ESP-IDF, so tests/ultra_host_test.cpp can't compile them:
// tests/run_ultra_lifecycle_test.py cuts their bodies unchanged from components/dig2analog_ultra into build/lifecycle/
// *.inc, and this file compiles them with small stand-ins for ESPHome, the RMT driver and the HTTP client. Covers the
// light's setup, a transition cut short, its power request after weeks; the update's install queue, chunked downloads
// and the confirmation frames after a session; a frame the RMT refuses; the recovery's power cycle on ESPHome's power
// supply (its keep-on time); auto_retry: the sessions after an install (blind, and from the receivers' versions), its
// status texts, never with force, never after an abort; a Diff-Power port (its pins, a plain light's state sent again
// once powered, the notice before the enable drops, OTA, the power mode select) and an Ultra line on one (its lights
// and holds as the port's loads, its replay when powered, Standby in its receivers' entities). Logic only: no ESP-IDF,
// no timing.
// Build: python tests/run_ultra_lifecycle_test.py   (uses the ziglang pip package as C/C++ compiler)

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/fw_manifest.h"
#include "../components/dig2analog_ultra/update_retry.h"
#include "../components/dig2analog_ultra/update_session.h"
#include "../components/dig2analog_ultra/ultra_ledger.h"
#include "../components/dig2analog_ultra/ultra_pacing.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"
#include "../components/dig2analog_ultra/ultra_commission.h"
#include "../components/diffpower/diffpower_core.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <new>
#include <set>
#include <string>
#include <vector>

// a nothrow array new that fails on demand (the ledger's stored form, built only for a save, without the memory)
static bool fail_nothrow_new = false;
void *operator new[](std::size_t n, const std::nothrow_t &) noexcept {
  return fail_nothrow_new ? nullptr : std::malloc(n);
}
void operator delete[](void *p) noexcept { std::free(p); }

#define USE_DIG2ANALOG_ULTRA_UPDATE
#define ESP_LOGE(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGV(...) ((void) 0)
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
  bool off_stop_{false};
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
  bool line_free_();   // (hub_line.inc)
  bool line_out_();
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
  // the commissioning (ultra_commission.h: tests/ultra_telemetry_test.cpp runs it)
  bool comm_on_{false};
  void service_commission_() {}
  Commissioning comm_;
};

#include "hub_loop.inc"
#include "hub_line.inc"

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
  void abort_update_for_shutdown_();      // the component's
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
  bool slow{false};                         // (the shutdown test: a pause per frame in real time)
};
static Line line;
static std::atomic<size_t> line_sent{0};   // (frames handed over, for the shutdown test's other thread)
static esp_err_t rmt_transmit(int, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  if (n != UPD_FRAME_BYTES) {
    line.refused++;
    return -1;
  }
  line.overlapped += line.on_wire;   // a frame handed over while the last one is still out
  line.wire.emplace_back(data, data + n);
  line.quiet_us.push_back(0);
  line.on_wire = true;
  line_sent++;
  if (line.wire.size() == line.abort_at)
    line.hub->upd_abort_.store(true);
  return ESP_OK;
}
static esp_err_t rmt_tx_wait_all_done(int, int) {
  line.on_wire = false;
  if (line.slow)
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  return ESP_OK;
}
static void vTaskDelay(uint32_t ticks) {
  line.overlapped += line.on_wire;   // a pause that starts before its frame and reset are out
  if (!line.quiet_us.empty() && ticks > 0)
    line.quiet_us.back() += (ticks - 1) * portTICK_PERIOD_MS * 1000;
}
// (the shutdown's wait: real time, the task running on its own thread)
static uint32_t millis() {
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}
static void delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
struct AppStub {
  int feeds{0};
  void feed_wdt() { this->feeds++; }
};
static AppStub App;

#include "hub_update.inc"

/// An ESPHome OTA / reboot during a session (on_shutdown): the task, on its own thread, is told to abort; the
/// shutdown waits until its ABORT frames are out (bounded), then goes on. Not running: nothing happens.
static void test_update_shutdown() {
  static uint8_t img[UPD_DESC_BYTES + 2000];
  for (size_t i = 0; i < sizeof(img); i++)
    img[i] = static_cast<uint8_t>(i * 7 + 3);
  UpdateSession::Options o;
  o.lead_in_us = 20000;
  o.end_us = 10000;
  UltraHub idle;
  idle.upd_state_.store(UltraHub::UPD_IDLE);
  idle.abort_update_for_shutdown_();
  CHECK(!idle.upd_abort_.load(), "no session: the shutdown changes nothing");
  UltraHub h;
  line = Line();
  line.hub = &h;
  line.slow = true;   // (a frame every ~0.2 ms: the task is still sending when the shutdown comes)
  line_sent = 0;
  h.upd_session_.begin(img, sizeof(img), 0x42, o, h.update_frame_min_us());
  std::thread task([&h]() { UltraHub::update_task_(&h); });
  while (line_sent.load() < 50)
    std::this_thread::yield();
  const auto t0 = std::chrono::steady_clock::now();
  h.abort_update_for_shutdown_();
  const double waited = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  const bool finished = h.upd_state_.load() == UltraHub::UPD_FINISHED;
  task.join();
  size_t aborts = 0, last_session = 0;
  for (size_t i = 0; i < line.wire.size(); i++) {
    if (line.wire[i][UPD_B_TYPE] == UPD_ABORT)
      aborts++;
    else
      last_session = i + 1;
  }
  CHECK(finished && h.upd_result_.load() == UltraHub::UPD_ABORTED && aborts == 12 && last_session + 12 == line.wire.size(),
        "a shutdown during the session: aborted, its 12 ABORT frames last on the wire, the task done before the "
        "shutdown went on (%.1f ms, %u frames first)", waited, static_cast<unsigned>(last_session));
  line.slow = false;
}

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

// ================================================================================================ hub: telemetry
// The hub's loop() and transmit_() again, now with the return channel (USE_DIG2ANALOG_ULTRA_TELEMETRY), and its window
// functions (build/lifecycle/hub_rw.inc), on the real Scheduler and TelemetryPoller: a line of 4 boards, the first 2
// Diff-Power receivers, a direction pin, a UART; the RMT and the receivers modelled with a clock in us. Then several
// lines at once, each with its own RMT channel, UART, direction pin and receivers.
namespace hub_rw {

#define USE_DIG2ANALOG_ULTRA_TELEMETRY
#define USE_DIFFPOWER   // (rw_publish_'s standby support warning; the port itself: a stand-in)
#define IRAM_ATTR
typedef int rmt_channel_handle_t;
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
static int64_t clock_us = 0;
static void advance(int64_t us) {
  clock_us += us;
  now_ms = static_cast<uint32_t>(clock_us / 1000);
}
static int64_t esp_timer_get_time() { return clock_us; }
static void delayMicroseconds(uint32_t us) { advance(us); }
static const char *esp_err_to_name(esp_err_t) { return "refused"; }

struct Pin {   // the direction pin (and its ISR twin)
  bool level{false};
  void setup() {}
  void digital_write(bool v) { this->level = v; }
};
struct IsrPin {
  Pin *pin{nullptr};
  void digital_write(bool v) { this->pin->level = v; }
};
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
  // the reply rate (round 12): ESPHome's member and the driver's (uart_set_baudrate)
  uint32_t baud{1000000};
  uint32_t get_baud_rate() const { return this->baud; }
  void set_baud_rate(uint32_t b) { this->baud = b; }
  uint8_t get_hw_serial_number() { return 0; }
};
namespace uart {
using IDFUARTComponent = Uart;
}
using uart_port_t = int;
static uint32_t uart_hw_baud = 1000000;
static esp_err_t uart_set_baudrate(uart_port_t, uint32_t b) {
  uart_hw_baud = b;
  return ESP_OK;
}

struct UltraHub;
/// The RMT: one frame at a time, done after its bits and its reset symbol (80 MHz ticks); then its interrupt.
struct Sent {
  std::vector<uint8_t> frame;
  int64_t at_us;
  uint32_t reset_ticks;
  bool pin_high;   // the direction pin as the frame started
};
struct Rmt {
  bool busy{false};
  int64_t done_us{0};
  int refuse{0};
  bool isr{true};   // its done interrupt runs (false: held up)
  std::vector<Sent> sent;
  UltraHub *hub{nullptr};
};
static Rmt rmts[4];         // per RMT channel (a line's channel_)
static Rmt &rmt = rmts[0];
static esp_err_t rmt_tx_wait_all_done(int ch, int) { return rmts[ch].busy ? -1 : ESP_OK; }
static esp_err_t rmt_transmit(int, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *);

struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  enum UpdateState : uint8_t { UPD_IDLE = 0, UPD_RUNNING, UPD_FINISHED };
  void loop();                        // the component's (hub_loop.inc)
  void transmit_();                   // (hub_transmit.inc)
  bool rw_due_(bool urgent);          // (hub_rw.inc)
  bool rw_uart_turn_();
  void rw_drop_turn_();
  void finish_window_(bool wait);
  void rw_publish_();
  void rw_uart_rate_();
  static bool rw_done_isr_(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *arg);
  static int64_t now_us_() { return clock_us; }
  void wake_();                       // (hub_wake.inc)
  void loop_fast_();
  // stand-ins
  struct {
    int starts{0}, stops{0};
    void start() { this->starts++; }
    void stop() { this->stops++; }
  } high_freq_;
  void enable_loop() {}
  void service_calibration_() {}
  void resync() { this->preamble_ = 3; }
  bool boards_powered_() const { return this->powered; }
  bool status_has_warning() const { return this->warning_; }
  void status_set_warning() { this->warning_ = true; }
  void status_clear_warning() { this->warning_ = false; }

  // the line
  std::atomic<uint8_t> upd_state_{UPD_IDLE};
  std::atomic<uint8_t> upd_result_{UPD_NONE};
  void *upd_task_{nullptr};
  bool upd_quiet_{false}, upd_resync_pending_{false};
  uint32_t upd_quiet_ms_{0}, upd_confirm_ms_{0};
  UpdateResult upd_done_{UPD_NONE};
  uint16_t ping_left_{0};
  uint8_t dark_tail_{0};
  uint16_t preamble_{0};
  bool preamble_full_{true}, tx_preamble_end_{false}, synced_once_{false};
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};
  Scheduler sched_;
  uint16_t num_boards_{4};
  std::vector<uint8_t> buf_;
  uint8_t *tx_buf_{nullptr};
  size_t tx_bytes_{0};
  bool tx_prepared_{false}, tx_normal_{true};
  uint32_t build_peak_us_{0}, frames_{0}, last_busy_ms_{0}, last_frame_ms_{0}, refresh_interval_ms_{200};
  bool high_freq_active_{false}, has_sent_{false}, started_{true}, powered{true}, warning_{false};
  int channel_{0}, encoder_{0}, wakes{0};
  struct {
    FrameSymbols<rmt_symbol_word_t> symbols;
  } enc_;
  uint8_t pin_{16};
  // the return channel
  Uart *rw_uart_{nullptr};
  Pin *rw_dir_pin_{nullptr};
  IsrPin rw_dir_isr_;
  Pin *rw_rx_enable_pin_{nullptr};   // (lines sharing a UART: tests/ultra_parlio_test.cpp)
  bool rw_turn_{false}, rw_waiting_{false};
  uint32_t rw_wait_order_{0}, rw_wait_seen_ms_{0};
  static uint32_t rw_wait_seq_;
  bool rw_driving_{true};
  volatile bool rw_release_{false};
  volatile int64_t rw_released_us_{0};
  TelemetryPoller poller_;
  ReplyRate reply_;
  PortMonitor port_monitor_;
  EnergyCounter port_energy_;
  bool port_seeded_{false}, rw_rx_tied_{false};
  float port_power_w() const { return this->port_standby() ? 0.0f : this->port_monitor_.power_w(); }
  LedgerKeeper keeper_;   // (not attached here: tests/ultra_telemetry_test.cpp runs it)
  const char *tel_name_{"Port"};
  void ledger_service_() {}
  void ledger_line_() {}
  std::vector<uint8_t> rw_slots_, rw_rx_;
  bool rw_next_{false}, tx_rw_{false};
  uint32_t rw_window_us_{0};
  bool rw_open_{false};
  int64_t rw_t0_us_{0}, rw_end_us_{0}, rw_read_us_{0};
  bool rw_powered_{true}, rw_lock_end_{false};
  static constexpr uint8_t RX_WOKE = 1, RX_WARNED = 2, RX_SETTLING = 4, RX_NO_STANDBY = 8;
  std::vector<uint8_t> rx_note_;
  uint8_t no_standby_{0};
  bool rx_settling(uint16_t receiver) const;   // (hub_rw.inc)
  bool port_standby() const { return this->port_ != nullptr && !this->powered; }
  uint32_t woke_ms_{0};
  struct PortStub {   // (a Diff-Power port: whether there is one, its notice copies out)
    int sent{0};
    void notice_sent() { this->sent++; }
  } *port_{nullptr};
  bool port_attached_{true};
  void attach_port_() {}
  void service_notice_() {}   // (a lane's notice copy: tests/ultra_parlio_test.cpp)
  bool line_free_();          // (hub_line.inc)
  bool line_out_();
  uint32_t rw_late_{0}, rw_extra_us_{0};
  rmt_symbol_word_t reset_symbol_{}, rw_reset_symbol_{};
  ReceiverListeners rx_listeners_;   // (ultra_telemetry.h's: per receiver)
  struct {
    int calls{0};
    void call() { this->calls++; }
  } rw_window_callback_;
  static std::vector<UltraHub *> hubs_;

  /// as setup() with telemetry: 2 receivers at 1 Mbaud, the direction pin high, the reset symbols
  void start(Uart *uart, Pin *pin) {
    this->sched_.init(this->num_boards_, 7, 3);
    this->buf_.assign(this->sched_.frame_bytes(), 0);
    this->tx_buf_ = this->buf_.data();
    rmt_symbol_word_t rst{}, rw{};
    rst.duration0 = rst.duration1 = 12000;   // 300 us at 80 MHz
    rw.duration0 = rw.duration1 = 80;        // 2 us
    this->enc_.symbols.reset = rst;
    this->reset_symbol_ = rst;
    this->rw_reset_symbol_ = pin != nullptr ? rw : rst;
    this->rw_uart_ = uart;
    this->rw_dir_pin_ = pin;
    if (pin != nullptr) {
      pin->level = true;
      this->rw_dir_isr_.pin = pin;
    }
    this->poller_.init(2, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    this->rw_slots_.assign(ULTRA_SLOT_BYTES * this->num_boards_, 0);
    this->rw_extra_us_ = this->poller_.max_window_us() + ULTRA_RW_HOLDOFF_US;
    this->preamble_ = 3;   // the start-up preamble
    hubs_.push_back(this);
  }
  // the commissioning (ultra_commission.h: tests/ultra_telemetry_test.cpp runs it)
  bool comm_on_{false};
  void service_commission_() {}
  Commissioning comm_;
};
std::vector<UltraHub *> UltraHub::hubs_;
uint32_t UltraHub::rw_wait_seq_ = 0;
static constexpr uint32_t RW_WAIT_STALE_MS = 100;

#include "hub_loop.inc"
#include "hub_wake.inc"
#include "hub_line.inc"
#include "hub_transmit.inc"
#include "hub_rw.inc"

/// The receivers: they answer an RW frame's POLLs in their slots (a STATUS with 24.00 V, IDENTITY without the optional
/// pages, OUTPUTS unsupported), the bytes in the UART from 1 ms after the frame's last bit.
struct Answers {
  std::vector<uint8_t> bytes;
  int64_t at_us{0};
  bool pending{false};
};
static Answers answers_of[4];   // per line (its RMT channel): its receivers' answers, for its UART
static Answers &answers = answers_of[0];
static bool receivers_on_of[4] = {true, true, true, true};
static bool &receivers_on = receivers_on_of[0];
// the cable (round 12): answers at a rate faster than this (a lower ULTRA_RW_RATE_*) arrive broken (their CRC fails)
static uint8_t cable_rate_of[4] = {0, 0, 0, 0};

static esp_err_t rmt_transmit(int ch, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  Rmt &rmt = rmts[ch];
  Answers &answers = answers_of[ch];
  if (rmt.refuse > 0) {
    rmt.refuse--;
    return -1;
  }
  UltraHub *h = rmt.hub;
  const uint32_t ticks = h->enc_.symbols.reset.duration0 + h->enc_.symbols.reset.duration1;
  rmt.sent.push_back(Sent{std::vector<uint8_t>(data, data + n), clock_us, ticks,
                          h->rw_dir_pin_ == nullptr || h->rw_dir_pin_->level});
  const int64_t bits_us = static_cast<int64_t>(n) * 8 * 1300 / 1000;
  rmt.busy = true;
  rmt.done_us = clock_us + bits_us + ticks / 80;
  const uint8_t *hd = data + n - ULTRA_HDR_BYTES;
  if (n == h->sched_.frame_bytes() && (ultra_header_flags(hd) & ULTRA_HDR_RW) && receivers_on_of[ch]) {
    answers.bytes.clear();
    answers.bytes.push_back(0x00);   // the turnaround checks' junk
    for (uint16_t b = 0; b < h->num_boards_; b++) {
      const uint8_t *s = data + ULTRA_SLOT_BYTES * b;
      if (ultra_poll_valid(s) != 0 || s[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY)
        continue;
      uint8_t a[32]{}, d[24]{};
      uint8_t page = s[ULTRA_POLL_PAGE];
      if (page == ULTRA_PAGE_STATUS) {
        ultra_put24(d + ULTRA_ST_FW_VERSION, 0x000608);
        ultra_put24(d + ULTRA_ST_UPTIME, static_cast<uint32_t>(clock_us / 1000000));
        ultra_put16(d + ULTRA_ST_VBUS, static_cast<uint16_t>(2400 + 1200 * ch));   // line k: 24 V + 12 V x k
        ultra_put16(d + ULTRA_ST_MCU_TEMP, ULTRA_ST_NA);
        d[ULTRA_ST_MEASURED] = ULTRA_ST_M_VBUS;
      } else if (page == ULTRA_PAGE_IDENTITY && s[ULTRA_POLL_ARG] == ULTRA_ID_PART_BOARD) {
        d[ULTRA_ID_UID] = static_cast<uint8_t>(b + 1);
      } else if (page == ULTRA_PAGE_IDENTITY && s[ULTRA_POLL_ARG] == ULTRA_ID_PART_FEATURES) {
        ultra_put32(d + ULTRA_IDF_FEATURES, b == 1 ? ULTRA_FEAT_STANDBY : 0u);   // (receiver 1: 0.2.6 standby support)
      } else {
        page = ULTRA_PAGE_UNSUPPORTED;
      }
      a[0] = ULTRA_RSP_SYNC;
      a[1] = page;
      a[2] = static_cast<uint8_t>(s[ULTRA_POLL_INDEX] << 4);
      a[3] = s[2];
      ultra_put16(a + 4, static_cast<uint16_t>(0x1000 + 0x100 * ch + b));
      std::memcpy(a + ULTRA_RSP_DATA, d, 24);
      ultra_put16(a + 30, ultra_crc16(a, 30));
      if (s[ULTRA_POLL_RATE] < cable_rate_of[ch])
        a[12] ^= 0x5A;   // (too fast for this cable: broken)
      answers.bytes.push_back(ULTRA_RSP_LEAD);
      answers.bytes.insert(answers.bytes.end(), a, a + 32);
    }
    answers.at_us = rmt.done_us + 1000;
    answers.pending = true;
  }
  return ESP_OK;
}

/// time passes in steps of 50 us: frames end (their interrupt), the answers arrive, the hub's loop runs
static void run_us(UltraHub &h, Uart &uart, int64_t us) {
  const int64_t end = clock_us + us;
  while (clock_us < end) {
    advance(50);
    if (rmt.busy && clock_us >= rmt.done_us) {
      rmt.busy = false;
      if (rmt.isr)
        UltraHub::rw_done_isr_(0, nullptr, &h);
    }
    if (answers.pending && clock_us >= answers.at_us) {
      uart.rx.insert(uart.rx.end(), answers.bytes.begin(), answers.bytes.end());
      answers.pending = false;
    }
    h.loop();
  }
}

static bool is_rw(const Sent &s) {
  return s.frame.size() >= ULTRA_HDR_BYTES && (ultra_header_flags(s.frame.data() + s.frame.size() - ULTRA_HDR_BYTES) & ULTRA_HDR_RW);
}

/// The hub with telemetry: RW frames only 1 s after the preamble and while nothing is queued; the frame itself (POLLs
/// for the receivers, the header's RW, the short reset and the direction pin let go of by the RMT's interrupt);
/// nothing else on the wire until the window's end, a command right after it; the UART flushed before the frame and
/// read after the window; the sensors' callback; a refused RW frame sent again as one; a late release counted; the
/// supply off: the receivers unavailable; finish_window_(true) (an update session starting) waits for the window.
/// The loop's high-frequency share with polls only (no commands): fast while a response window is open, not held
/// for BUSY_HOLD_MS after every RW frame. The control: the busy hold at every RW frame (as before), 25 % and more.
static double rw_fast_share(bool hold_at_rw, double *rw_per_s = nullptr) {
  clock_us = 0;
  now_ms = 0;
  rmt = Rmt();
  answers = Answers();
  UltraHub::hubs_.clear();
  Uart uart;
  Pin pin;
  UltraHub h;
  rmt.hub = &h;
  h.start(&uart, &pin);
  run_us(h, uart, 5000000);   // (the preamble, the first answers, the busy hold after them)
  int64_t fast = 0, steps = 0, rws = 0;
  size_t sent = rmt.sent.size();
  const int64_t end = clock_us + 60000000;
  while (clock_us < end) {
    run_us(h, uart, 50);
    if (rmt.sent.size() != sent && is_rw(rmt.sent.back())) {
      rws++;
      if (hold_at_rw)
        h.wake_();   // (the control: the RW frame's wake_() with its busy hold)
    }
    sent = rmt.sent.size();
    fast += h.high_freq_active_;
    steps++;
  }
  if (rw_per_s != nullptr)
    *rw_per_s = static_cast<double>(rws) / 60.0;
  return 100.0 * static_cast<double>(fast) / static_cast<double>(steps);
}

/// A Diff-Power port's receivers without standby support are counted once (receiver 0 here; receiver 1 has it), and
/// warned about once: the port's planned power-offs are capped while one is there.
static void test_rw_no_standby() {
  clock_us = 0;
  now_ms = 0;
  rmt = Rmt();
  answers = Answers();
  UltraHub::hubs_.clear();
  Uart uart;
  Pin pin;
  UltraHub h;
  UltraHub::PortStub port;
  h.port_ = &port;
  rmt.hub = &h;
  h.start(&uart, &pin);
  h.rx_note_.assign(h.poller_.receivers(), 0);
  run_us(h, uart, 20000000);
  CHECK(h.no_standby_ == 1 && (h.rx_note_[0] & UltraHub::RX_NO_STANDBY) && !(h.rx_note_[1] & UltraHub::RX_NO_STANDBY) &&
            (h.rx_note_[0] & UltraHub::RX_WARNED),
        "receivers without standby support: %u counted (receiver 0), warned once", h.no_standby_);
}

/// The reply rate on a line (round 12): what 20 s of telemetry ends with on a cable that breaks answers faster than
/// `cable`, with adaptive_rate or pinned, the receivers answering or not.
struct RateRun {
  uint8_t rate, poll_rate;   // the poller's, and the last RW frame's POLL
  uint32_t baud, hw_baud, extra_before, extra_after, broken, answers;
};
static RateRun reply_rate_run(uint8_t cable, bool adaptive, bool on) {
  clock_us = 0;
  now_ms = 0;
  rmt = Rmt();
  answers = Answers();
  UltraHub::hubs_.clear();
  cable_rate_of[0] = cable;
  receivers_on = on;
  uart_hw_baud = 1000000;
  Uart uart;
  Pin pin;
  UltraHub h;
  rmt.hub = &h;
  h.start(&uart, &pin);
  h.reply_.configure(ULTRA_RW_RATE_1M, adaptive);
  const uint32_t before = h.rw_extra_us_;
  run_us(h, uart, 20000000);
  RateRun r{h.poller_.rate(), 0xFF, uart.baud, uart_hw_baud, before, h.rw_extra_us_, h.poller_.stats().bad_crc,
            h.poller_.stats().answers};
  for (auto it = rmt.sent.rbegin(); it != rmt.sent.rend(); ++it) {
    if (is_rw(*it)) {
      r.poll_rate = it->frame[ULTRA_POLL_RATE];
      break;
    }
  }
  cable_rate_of[0] = 0;
  receivers_on = true;
  return r;
}

/// A cable that breaks the answers at 1 Mbaud (fine at 500 k): the line steps down within a few windows, its POLLs ask
/// for 500 k, the UART is set to 500 kbaud (ESPHome's and the driver's) at the next window, the commands' lead
/// follows the longer windows; at 500 k nothing breaks: it stays there. Controls: a good cable stays at 1 M; pinned,
/// the bad cable stays at 1 M (its answers keep breaking); receivers that don't answer (missing only): no step.
static void test_rw_reply_rate() {
  const RateRun bad = reply_rate_run(ULTRA_RW_RATE_500K, true, true);
  CHECK(bad.rate == ULTRA_RW_RATE_500K && bad.poll_rate == ULTRA_RW_RATE_500K && bad.baud == 500000 &&
            bad.hw_baud == 500000 && bad.extra_after > bad.extra_before && bad.broken >= ReplyRate::BAD &&
            bad.broken <= 3 * ReplyRate::BAD && bad.answers >= 8,
        "a cable broken at 1 M: 500 k (POLL %u, UART %u / %u), lead %u -> %u us, %u broken, %u answers",
        bad.poll_rate, bad.baud, bad.hw_baud, bad.extra_before, bad.extra_after, bad.broken, bad.answers);
  const RateRun good = reply_rate_run(0, true, true);
  const RateRun pinned = reply_rate_run(ULTRA_RW_RATE_500K, false, true);
  const RateRun silent = reply_rate_run(ULTRA_RW_RATE_500K, true, false);
  CHECK(good.rate == ULTRA_RW_RATE_1M && good.baud == 1000000 && good.hw_baud == 1000000 && good.broken == 0 &&
            pinned.rate == ULTRA_RW_RATE_1M && pinned.poll_rate == ULTRA_RW_RATE_1M &&
            pinned.broken > 2 * ReplyRate::BAD &&
            silent.rate == ULTRA_RW_RATE_1M && silent.answers == 0,
        "controls: a good cable 1 M; pinned 1 M (%u broken); no answers at all 1 M", pinned.broken);
}

static void test_rw_fast_share() {
  double rate = 0;
  const double now = rw_fast_share(false, &rate), control = rw_fast_share(true);
  // the control: about 250 ms per RW frame (the windows overlap nothing here)
  CHECK(rate > 0.2 && now < 1.0 && control > 0.8 * 25.0 * rate,
        "the loop fast %.1f %% of the time at %.2f RW frames / s (busy hold at every poll: %.1f %%)", now, rate, control);
  std::printf("  the loop's high-frequency share, polls only: %.2f %% at %.2f RW frames / s (with the busy hold at every "
              "poll: %.1f %%)\n", now, rate, control);
}

static void test_telemetry_window() {
  clock_us = 0;
  now_ms = 0;
  rmt = Rmt();
  answers = Answers();
  UltraHub::hubs_.clear();
  Uart uart;
  Pin pin;
  UltraHub h;
  rmt.hub = &h;
  h.start(&uart, &pin);
  std::vector<std::pair<uint16_t, uint32_t>> calls;
  for (uint16_t r = 0; r < h.poller_.receivers(); r++)
    h.rx_listeners_.add(r, [&calls, r](uint32_t u) { calls.push_back({r, u}); });
  run_us(h, uart, 3000000);
  // the preamble went out first, the first RW frame at least 1 s after its last frame's end (its header and reset)
  const int64_t last_dark =
      rmt.sent.size() > 3 ? rmt.sent[2].at_us + static_cast<int64_t>(rmt.sent[2].frame.size()) * 8 * 1300 / 1000 +
                                rmt.sent[2].reset_ticks / 80
                          : -1;   // the preamble: the first 3 frames
  int64_t first_rw = -1;
  for (const Sent &s : rmt.sent) {
    if (is_rw(s) && first_rw < 0)
      first_rw = s.at_us;
  }
  CHECK(last_dark >= 0 && first_rw >= last_dark + 1000000, "the first RW frame %lld us after the preamble's last",
        static_cast<long long>(first_rw - last_dark));
  // the RW frames: POLLs for boards 0 and 1, the other boards' slots not; the short reset; the pin low after each
  bool frames_ok = true, pin_ok = true, hold_ok = true;
  int rw_frames = 0;
  for (size_t i = 0; i < rmt.sent.size(); i++) {
    const Sent &s = rmt.sent[i];
    if (!s.pin_high)
      pin_ok = false;   // every frame starts with the hub driving
    if (!is_rw(s)) {
      frames_ok &= s.reset_ticks == 24000;
      continue;
    }
    rw_frames++;
    frames_ok &= s.reset_ticks == 160 && ultra_poll_valid(s.frame.data()) == 0 &&
                 ultra_poll_valid(s.frame.data() + 32) == 0 && ultra_poll_valid(s.frame.data() + 64) != 0 &&
                 ultra_poll_valid(s.frame.data() + 96) != 0;
    // the next frame no earlier than the window's end: 1000 + 2 x 460 + 200 us after this frame's last bit
    if (i + 1 < rmt.sent.size()) {
      const int64_t last_bit = s.at_us + static_cast<int64_t>(s.frame.size()) * 8 * 1300 / 1000;
      hold_ok &= rmt.sent[i + 1].at_us >= last_bit + 2120 - 60;   // (the frame's TIME is an estimate: +-60 us)
    }
  }
  CHECK(rw_frames >= 2 && frames_ok, "%d RW frames: POLLs for the receivers only, the RW reset short, others full",
        rw_frames);
  CHECK(pin_ok, "every frame goes out with the direction pin high (raised again after each window)");
  CHECK(hold_ok, "nothing goes out before an RW frame's window ends");
  CHECK(h.poller_.answering(0) && h.poller_.answering(1) && h.poller_.data(1).have_status &&
            std::fabs(telemetry_value(h.poller_.data(0), TV_SUPPLY_VOLTAGE) - 24.0f) < 1e-3f,
        "the answers read from the UART after the window: both receivers decoded");
  bool called0 = false, called1 = false;
  for (auto &c : calls) {
    called0 |= c.first == 0 && (c.second & TU_AVAILABILITY);
    called1 |= c.first == 1 && (c.second & (1u << TP_STATUS));
  }
  CHECK(called0 && called1, "the sensors' callback per receiver (%u calls)", static_cast<unsigned>(calls.size()));

  // a command queued: it goes before any RW frame; one queued while a window is open waits for its end
  size_t n0 = rmt.sent.size();
  while (!h.rw_open_)
    run_us(h, uart, 50);
  const int64_t end = h.rw_end_us_;
  CHECK(pin.level, "the direction pin high while the RW frame is on the wire");
  while (rmt.busy)
    run_us(h, uart, 50);
  CHECK(!pin.level, "the RMT's interrupt let go of the line after the RW frame");
  h.sched_.identify(3, true);
  n0 = rmt.sent.size();
  run_us(h, uart, 20000);
  CHECK(rmt.sent.size() > n0 && rmt.sent[n0].at_us >= end && !is_rw(rmt.sent[n0]),
        "the command waited for the window's end, then went out first");
  h.sched_.identify(3, false);

  // junk in the UART before an RW frame is dropped; a refused RW frame goes out again as one
  uart.rx.assign(40, ULTRA_RSP_SYNC);
  rmt.refuse = 1;
  const uint32_t bad0 = h.poller_.stats().bad_crc;
  n0 = rmt.sent.size();
  while (!h.rw_open_)
    run_us(h, uart, 50);
  CHECK(is_rw(rmt.sent.back()) && h.poller_.stats().bad_crc == bad0, "a refused RW frame sent again as one; the junk "
        "before it flushed");
  run_us(h, uart, 10000);
  CHECK(h.poller_.stats().bad_crc == bad0, "no junk SYNC from before the frame reached the parser (%u)",
        h.poller_.stats().bad_crc - bad0);

  // the RMT's interrupt held up: the release counted late
  rmt.isr = false;
  while (!h.rw_open_)
    run_us(h, uart, 50);
  run_us(h, uart, 10000);
  rmt.isr = true;
  CHECK(h.rw_late_ == 1, "a release that didn't come in time is counted (%u)", h.rw_late_);

  // an update session starting: finish_window_(true) waits for the window and reads it
  while (!h.rw_open_)
    run_us(h, uart, 50);
  const int64_t read_at = h.rw_read_us_;
  run_us(h, uart, 1500);   // (the answers arrive)
  h.finish_window_(true);
  CHECK(!h.rw_open_ && clock_us >= read_at && h.enc_.symbols.reset.duration0 == 12000,
        "finish_window_(true) waits for the window, reads it, the full reset back");

  // the supply off: unavailable, no RW frames
  calls.clear();
  h.powered = false;
  const size_t rw0 = static_cast<size_t>(std::count_if(rmt.sent.begin(), rmt.sent.end(), is_rw));
  run_us(h, uart, 5000000);
  bool gone = false;
  for (auto &c : calls)
    gone |= c.first == 0 && (c.second & TU_AVAILABILITY);
  CHECK(gone && !h.poller_.answering(0) &&
            static_cast<size_t>(std::count_if(rmt.sent.begin(), rmt.sent.end(), is_rw)) == rw0,
        "the supply off: the receivers unavailable, no polls");
  // power back: every receiver's sensors at once (a Diff-Power port's Standby gone; the status follows the answers)
  calls.clear();
  h.powered = true;
  run_us(h, uart, 20000);
  bool touched0 = false, touched1 = false;
  for (auto &c : calls) {
    touched0 |= c.first == 0 && (c.second & TU_AVAILABILITY);
    touched1 |= c.first == 1 && (c.second & TU_AVAILABILITY);
  }
  CHECK(touched0 && touched1, "power back: every receiver's sensors refreshed at once");
  UltraHub::hubs_.clear();
  now_ms = 0;
  std::printf("  telemetry: %d RW frames on a 4-board line, 2 receivers, every window held and read\n", rw_frames);
}

/// Several lines with telemetry at once (a larger sender: one hub per line, each with its own RMT channel, UART,
/// direction pin and receivers): they poll at the same time, their windows overlap (nothing is shared or taken in
/// turns), each reads only its own receivers (their own voltages and chip ids) into its own poller, port and callbacks;
/// one line's receivers going silent leaves the others answering.
static void test_several_lines() {
  constexpr int N = 4;
  clock_us = 0;
  now_ms = 0;
  for (auto &r : rmts)
    r = Rmt();
  for (auto &a : answers_of)
    a = Answers();
  UltraHub::hubs_.clear();
  Uart uarts[N];
  Pin pins[N];
  UltraHub hubs[N];
  int calls[N]{};
  for (int k = 0; k < N; k++) {
    hubs[k].channel_ = k;
    hubs[k].pin_ = static_cast<uint8_t>(16 + k);
    rmts[k].hub = &hubs[k];
    hubs[k].start(&uarts[k], &pins[k]);
    for (uint16_t r = 0; r < hubs[k].poller_.receivers(); r++)
      hubs[k].rx_listeners_.add(r, [&calls, k](uint32_t) { calls[k]++; });
  }
  int overlap = 0, rw_frames[N]{};
  auto run = [&](int64_t us) {
    const int64_t end = clock_us + us;
    while (clock_us < end) {
      advance(50);
      for (int k = 0; k < N; k++) {
        Rmt &r = rmts[k];
        if (r.busy && clock_us >= r.done_us) {
          r.busy = false;
          UltraHub::rw_done_isr_(k, nullptr, &hubs[k]);
        }
        Answers &a = answers_of[k];
        if (a.pending && clock_us >= a.at_us) {
          uarts[k].rx.insert(uarts[k].rx.end(), a.bytes.begin(), a.bytes.end());
          a.pending = false;
        }
        hubs[k].loop();
      }
      int open = 0;
      for (auto &h : hubs)
        open += h.rw_open_;
      overlap += open >= 2;
    }
  };
  run(4000000);
  bool own = true, all = true;
  for (int k = 0; k < N; k++) {
    for (const Sent &s : rmts[k].sent)
      rw_frames[k] += is_rw(s);
    const TelemetryPoller &p = hubs[k].poller_;
    all &= p.answering(0) && p.answering(1) && rw_frames[k] >= 2 && calls[k] > 0 && pins[k].level;
    own &= std::fabs(telemetry_value(p.data(0), TV_SUPPLY_VOLTAGE) - (24.0f + 12.0f * k)) < 1e-3f &&
           p.uid16(0) == 0x1000 + 0x100 * k && p.uid16(1) == 0x1001 + 0x100 * k && p.stats().stale == 0 &&
           p.stats().missing == 0;
  }
  CHECK(all, "%d lines: each polls its own 2 receivers (%d / %d / %d / %d RW frames), its callbacks", N, rw_frames[0],
        rw_frames[1], rw_frames[2], rw_frames[3]);
  CHECK(own, "each line's poller holds only its own receivers (their voltages, chip ids), nothing stale or missing");
  CHECK(overlap > 0, "the lines' windows overlap: each line's on its own (%d samples with 2 or more open)", overlap);
  // line 1's receivers go silent: only its receivers unavailable, the others keep answering
  receivers_on_of[1] = false;
  run(6000000);
  CHECK(!hubs[1].poller_.answering(0) && !hubs[1].poller_.answering(1) && hubs[0].poller_.answering(0) &&
            hubs[2].poller_.answering(1) && hubs[3].poller_.answering(0),
        "line 1's receivers silent: only they are unavailable");
  receivers_on_of[1] = true;
  run(6000000);
  CHECK(hubs[1].poller_.answering(0) && hubs[1].poller_.answering(1), "and back");
  UltraHub::hubs_.clear();
  now_ms = 0;
  std::printf("  telemetry: %d lines polling at once, %d samples with overlapping windows\n", N, overlap);
}

/// Lines sharing one UART with DE and /RE tied (receiver_enable_pin: direction_pin): a line's receiver is on only while
/// its direction pin is low (its window), and the hub drives the line again as soon as its window is read, so no two
/// lines ever have their receivers on at once; every line polls its own receivers. Negative control: without the tie
/// a line keeps its receiver on until its next frame, and the receivers collide.
static void test_tied_shared_uart() {
  constexpr int N = 3;
  for (int tied = 1; tied >= 0; tied--) {
    clock_us = 0;
    now_ms = 0;
    for (auto &r : rmts)
      r = Rmt();
    for (auto &a : answers_of)
      a = Answers();
    UltraHub::hubs_.clear();
    Uart uart;
    Pin pins[N];
    UltraHub hubs[N];
    for (int k = 0; k < N; k++) {
      hubs[k].channel_ = k;
      hubs[k].pin_ = static_cast<uint8_t>(16 + k);
      rmts[k].hub = &hubs[k];
      hubs[k].start(&uart, &pins[k]);
      hubs[k].rw_rx_tied_ = tied != 0;
    }
    int both_on = 0, lost = 0;
    auto run = [&](int64_t us) {
      const int64_t end = clock_us + us;
      while (clock_us < end) {
        advance(50);
        for (int k = 0; k < N; k++) {
          Rmt &r = rmts[k];
          if (r.busy && clock_us >= r.done_us) {
            r.busy = false;
            UltraHub::rw_done_isr_(k, nullptr, &hubs[k]);
          }
          Answers &a = answers_of[k];
          if (a.pending && clock_us >= a.at_us) {   // (its port's receiver hears them only while it is on)
            if (!pins[k].level)
              uart.rx.insert(uart.rx.end(), a.bytes.begin(), a.bytes.end());
            else
              lost++;
            a.pending = false;
          }
          hubs[k].loop();
        }
        int on = 0;
        for (auto &pin : pins)
          on += !pin.level;
        both_on += on >= 2;
      }
    };
    run(4000000);
    bool all = true;
    for (int k = 0; k < N; k++)
      all &= hubs[k].poller_.answering(0) && hubs[k].poller_.answering(1) && hubs[k].poller_.stats().stale == 0;
    if (tied) {
      CHECK(all && both_on == 0 && lost == 0,
            "%d lines on one UART, DE and /RE tied: each polls its own receivers (%d), never two receivers on at once "
            "(%d samples), no answer lost (%d)", N, all, both_on, lost);
    } else {
      CHECK(both_on > 0, "negative control: without the tie two receivers are on at once (%d samples)", both_on);
    }
  }
  UltraHub::hubs_.clear();
  now_ms = 0;
  std::printf("  telemetry: 3 lines on one UART with DE and /RE tied, never two receivers on at once\n");
}

/// A hub on a Diff-Power port (rw_publish_): a receiver without ULTRA_FEAT_STANDBY (0.2.6) is warned about once, one
/// with it not; without a port nobody is.
static void test_standby_support() {
  for (bool on_port : {true, false}) {
    clock_us = 0;
    now_ms = 0;
    rmt = Rmt();
    answers = Answers();
    UltraHub::hubs_.clear();
    Uart uart;
    Pin pin;
    UltraHub h;
    rmt.hub = &h;
    UltraHub::PortStub port;
    h.port_ = on_port ? &port : nullptr;
    h.start(&uart, &pin);
    h.rx_note_.assign(2, UltraHub::RX_WOKE);   // (as the port's ready callback)
    run_us(h, uart, 4000000);
    const bool warned0 = h.rx_note_[0] & UltraHub::RX_WARNED, warned1 = h.rx_note_[1] & UltraHub::RX_WARNED;
    CHECK(h.poller_.data(0).have_identity && h.poller_.data(1).have_identity && warned0 == on_port && !warned1,
          "%s: receiver 0 (no standby support) warned %d, receiver 1 (with it) %d", on_port ? "on a port" : "no port",
          warned0, warned1);
  }
  UltraHub::hubs_.clear();
  now_ms = 0;
  std::printf("  telemetry: a receiver without standby support warned about once (on a Diff-Power port)\n");
}

/// After a wake: every receiver settling (Waking: no problem) until it answers, or misses enough to be
/// gone; then its status at once (a callback), and none of it logged as "doesn't answer".
static void test_wake_settling() {
  for (bool answer : {true, false}) {
    clock_us = 0;
    now_ms = 0;
    rmt = Rmt();
    answers = Answers();
    UltraHub::hubs_.clear();
    Uart uart;
    Pin pin;
    UltraHub h;
    rmt.hub = &h;
    UltraHub::PortStub port;
    h.port_ = &port;
    h.start(&uart, &pin);
    run_us(h, uart, 4000000);
    CHECK(h.poller_.answering(0) && h.poller_.answering(1) && !h.rx_settling(0), "powered: both answer");
    h.powered = false;   // the port sleeps
    run_us(h, uart, 1000000);
    receivers_on = answer;
    std::vector<uint32_t> after;
    for (uint16_t r = 0; r < h.poller_.receivers(); r++) {
      h.rx_listeners_.add(r, [&h, &after, r](uint32_t u) {
        if (h.powered && (u & TU_AVAILABILITY))
          after.push_back(r);
      });
    }
    h.powered = true;   // its wake: the preamble again
    h.resync();
    run_us(h, uart, 50000);
    CHECK(h.rx_settling(0) && h.rx_settling(1) && !h.poller_.answering(0) &&
              !receiver_status_problem(RS_WAKING) && after.size() == 2,
          "just woken: both settling (Waking), their status published as such");
    run_us(h, uart, 12000000);
    CHECK(!h.rx_settling(0) && !h.rx_settling(1) && h.poller_.answering(0) == answer && after.size() >= 4,
          "%s: settled (%u status updates)", answer ? "they answer" : "silent: 3 misses, gone",
          static_cast<unsigned>(after.size()));
    receivers_on = true;
  }
  UltraHub::hubs_.clear();
  now_ms = 0;
  std::printf("  telemetry: after a wake the receivers show Waking until they answer or miss 3 polls\n");
}

/// The lock interval runs from the last lock frame's end: 64 boards; and with its frame refused once
static void test_lock_from_end() {
  for (bool refuse : {false, true}) {
    clock_us = 0;
    now_ms = 0;
    rmt = Rmt();
    answers = Answers();
    UltraHub::hubs_.clear();
    Uart uart;
    Pin pin;
    UltraHub h;
    h.num_boards_ = 64;
    rmt.hub = &h;
    h.start(&uart, &pin);
    if (refuse) {
      while (h.preamble_ > 1)
        run_us(h, uart, 50);
      rmt.refuse = 1;   // the last preamble frame refused once
    }
    run_us(h, uart, 4000000);
    const Sent &last = rmt.sent[2];   // (the stand-in's preamble: 3 frames)
    const int64_t end = last.at_us + static_cast<int64_t>(last.frame.size()) * 8 * 1300 / 1000 + last.reset_ticks / 80;
    int64_t first = -1;
    for (const Sent &s : rmt.sent) {
      if (is_rw(s)) {
        first = s.at_us;
        break;
      }
    }
    CHECK(first >= end + 1000000, "%s: the first RW frame %lld us after the last preamble frame's end",
          refuse ? "its last frame refused once" : "64 boards", static_cast<long long>(first - end));
  }
  UltraHub::hubs_.clear();
  now_ms = 0;
  std::printf("  telemetry: the lock interval from the last preamble frame's end (64 boards, a refusal)\n");
}

#undef USE_DIG2ANALOG_ULTRA_TELEMETRY
#undef USE_DIFFPOWER
#undef IRAM_ATTR

}  // namespace hub_rw

// ================================================================================================ receiver entities
// The receivers' Home Assistant entities (ultra_receiver.cpp: UltraReceiver, UltraTelemetryPort, UltraLightTelemetry,
// cut into build/lifecycle/receiver_*.inc) on stand-in sensors, fed by a real TelemetryPoller.
namespace receiver_entities {

static std::vector<std::string> *pub_log = nullptr;   // (the dispatch test: every publish, in order, by entity id)
static void log_publish(int id, const std::string &v) {
  if (pub_log != nullptr)
    pub_log->push_back(std::to_string(id) + "=" + v + "@" + std::to_string(now_ms));
}

namespace sensor {
struct Sensor {
  bool has{false};
  float state{NAN};
  int publishes{0};
  int id{-1};
  std::vector<std::pair<uint32_t, float>> history;   // (when, what)
  bool has_state() const { return this->has; }
  float get_raw_state() const { return this->state; }
  void publish_state(float v) {
    this->state = v;
    this->has = true;
    this->publishes++;
    this->history.push_back({now_ms, v});
    char b[24];
    std::snprintf(b, sizeof(b), "%.9g", v);
    log_publish(this->id, b);
  }
};
}  // namespace sensor
namespace text_sensor {
struct TextSensor {
  bool has{false};
  std::string state;
  int publishes{0};
  int id{-1};
  bool has_state() const { return this->has; }
  void publish_state(const std::string &v) {
    this->state = v;
    this->has = true;
    this->publishes++;
    log_publish(this->id, v);
  }
};
}  // namespace text_sensor
namespace binary_sensor {
struct BinarySensor {
  bool state{false};
  int id{-1};
  void publish_state(bool v) {
    this->state = v;
    log_publish(this->id, v ? "on" : "off");
  }
};
}  // namespace binary_sensor

struct UltraHub {
  TelemetryPoller p;
  PortMonitor m;
  LifetimeLedger led;
  LedgerKeeper keeper;
  UltraHub() {
    this->led.init(4, 2);
    this->keeper.attach(&this->led, 5u << 16, 2);
  }
  bool standby{false};    // its Diff-Power port in standby
  bool off{false};        // ... its power mode Off
  bool settling{false};   // ... just woken: the receivers not heard yet
  uint32_t rate{1000};    // the update rate
  double port_wh{-1.0};
  uint32_t update_rate_ms() const { return this->rate; }
  const char *telemetry_name() const { return "Port 4"; }
  float port_power_w() const { return this->standby ? 0.0f : this->m.power_w(); }
  double port_energy_wh() const { return this->port_wh; }
  static LifetimeLedger &ledger() {
    static LifetimeLedger l;
    return l;
  }
  double position_energy_wh(uint16_t receiver) const { return this->keeper.energy_wh(receiver); }
  const LifeCounters *ledger_counters(uint16_t receiver) const { return this->keeper.counters(receiver); }
  const TelemetryPoller &telemetry() const { return this->p; }
  const PortMonitor &port_monitor() const { return this->m; }
  bool port_standby() const { return this->standby; }
  bool port_off() const { return this->off; }
  bool rx_settling(uint16_t) const { return this->settling; }
  ReplyRate reply;
  uint32_t late{0};
  const ReplyRate &reply_rate() const { return this->reply; }
  uint32_t late_releases() const { return this->late; }
};

#include "receiver_consts.inc"

struct UltraReceiver {   // as ultra_receiver.h
  enum Value : uint8_t {
    POWER = 0, ENERGY, BUS_VOLTAGE, MCU_TEMPERATURE, INPUT_CURRENT, UPTIME, TRIPS, SUPPLY_DIPS, LIMIT_EVENTS,
    MISSED_POLLS, VOLTAGE_DROP, CABLE_RESISTANCE, LED_SUPPLY, CURRENT_PEAK, SUPPLY_IMPEDANCE, LIT_HOURS, POWER_UPS,
    LIVE_POWER, VALUE_COUNT,
  };
  enum Text : uint8_t {
    STATUS = 0, STATUS_DETAIL, FIRMWARE, LAST_RESET, LAST_TRIP, LAST_DIP, LINK, CHANNEL_HEALTH, PORT, TEXT_COUNT,
  };
  void update_(uint32_t updates);
  void value_(uint8_t which, float v, bool force = false);
  void text_(uint8_t which, const std::string &s);
  UltraHub *hub_{nullptr};
  uint16_t board_{0};
  uint32_t unpaced_{0};
  sensor::Sensor *values_[VALUE_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *problem_{nullptr};
  Paced paced_[VALUE_COUNT]{};
  MeanPaced power_mean_;
};
struct UltraTelemetryPort {   // as ultra_receiver.h
  enum Value : uint8_t {
    POWER = 0, CURRENT, LOAD, CABLE_LOSS, CABLE_LOSS_PCT, PEAK_LOAD, LIVE_POWER, ENERGY, VALUE_COUNT,
  };
  enum Text : uint8_t { RECEIVERS = 0, STATUS, STATUS_DETAIL, TEXT_COUNT };
  enum Health : uint8_t { H_WINDOWS = 0, H_MISSING, H_BROKEN, H_CONTENTION, H_RESTARTS, H_LATE, H_RATE, HEALTH_COUNT };
  static constexpr uint32_t HEALTH_MS = 60000;
  void update_();
  void value_(uint8_t which, float v, bool force);
  void publish_health_();
  sensor::Sensor *health_[HEALTH_COUNT]{};
  bool has_health_{false}, health_seen_{false};
  uint32_t health_ms_{0}, answers_was_{0}, missing_was_{0};
  UltraHub *hub_{nullptr};
  uint32_t unpaced_{0};
  sensor::Sensor *values_[VALUE_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *problem_{nullptr};
  Paced paced_[VALUE_COUNT]{};
  MeanPaced power_mean_;
  bool standby_was_{false};
};
struct UltraLightTelemetry {   // as ultra_receiver.h
  void update_(uint32_t updates);
  void store_energy_(bool force);
  UltraHub *hub_{nullptr};
  uint16_t board_{0};
  uint8_t mask_{0}, unpaced_{0};
  bool seeded_{false};
  uint32_t key_{0};
  sensor::Sensor *power_{nullptr};
  sensor::Sensor *energy_{nullptr};
  text_sensor::TextSensor *status_{nullptr};
  text_sensor::TextSensor *port_{nullptr};
  EnergyCounter energy_count_;
  MeanPaced power_mean_;
  Paced power_paced_, energy_paced_;
};

#define USE_DIG2ANALOG_ULTRA_DIAG_TEXTS   // (diagnostics: full: the fault texts are built)
#include "receiver_methods.inc"

/// a receiver's answer to a POLL slot (index from the slot), for the stand-in line below
static void answer(const uint8_t *s, uint16_t uid, const ReceiverData &model, uint8_t why, std::vector<uint8_t> &out) {
  uint8_t a[32]{}, d[24]{};
  const uint8_t page = s[ULTRA_POLL_PAGE];
  if (page == ULTRA_PAGE_STATUS) {
    ultra_put24(d + ULTRA_ST_FW_VERSION, 0x000608);
    ultra_put24(d + ULTRA_ST_UPTIME, model.uptime_s);
    d[ULTRA_ST_STATE] = model.power_state;
    ultra_put16(d + ULTRA_ST_VBUS, model.vbus_10mv);
    ultra_put16(d + ULTRA_ST_MCU_TEMP, model.mcu_temp);
    ultra_put16(d + ULTRA_ST_CURRENT, model.current_ma);
    d[ULTRA_ST_MEASURED] = ULTRA_ST_M_VBUS | ULTRA_ST_M_CURRENT;
    d[ULTRA_ST_RESET] = ULTRA_ST_RST_POWER;
  } else if (page == ULTRA_PAGE_IDENTITY && s[ULTRA_POLL_ARG] == ULTRA_ID_PART_BOARD) {
    for (int k = 0; k < 12; k++)
      d[ULTRA_ID_UID + k] = static_cast<uint8_t>(uid + 7 * k);
    ultra_put32(d + ULTRA_ID_FW_VERSION, 0x000608FF);
  } else if (page == ULTRA_PAGE_IDENTITY && s[ULTRA_POLL_ARG] == ULTRA_ID_PART_FEATURES) {
    ultra_put32(d + ULTRA_IDF_FEATURES, model.features);
  } else if (page == ULTRA_PAGE_ENERGY) {
    ultra_put32(d + ULTRA_EN_TOTAL_WH, 1234);
    ultra_put32(d + ULTRA_EN_POWER_MW, 34000);
    if (model.features & ULTRA_FEAT_LIFETIME) {   // (0.2.5)
      ultra_put16(d + ULTRA_EN_VB, model.vb_10mv);
      ultra_put16(d + ULTRA_EN_PEAK_MA, model.peak_ma);
    }
  } else if (page == ULTRA_PAGE_LIFETIME) {
    ultra_put24(d + ULTRA_LT_POWERED, model.lt_powered);
    ultra_put24(d + ULTRA_LT_LIT, model.lt_lit);
    ultra_put24(d + ULTRA_LT_POWER_UPS, model.lt_power_ups);
    ultra_put16(d + ULTRA_LT_SHORT_TRIPS, model.lt_short_trips);
    ultra_put24(d + ULTRA_LT_DIPS, model.lt_dips);
  } else if (page == ULTRA_PAGE_DIAG && s[ULTRA_POLL_ARG] == 0) {
    d[ULTRA_DG_BC_CAUSE] = 0xFF;
    ultra_put16(d + ULTRA_DG_SUPPLY_Z, model.supply_z_mohm);
    d[ULTRA_DG_FLASH_PENDING] = model.flash_pending;
  } else if (page == ULTRA_PAGE_DIAG && s[ULTRA_POLL_ARG] == 1) {
    d[ULTRA_DG_TRIP] = model.fm_trip;
    ultra_put16(d + ULTRA_DG_TRIP_LIT_H, model.fm_trip_lit_h);
  } else if (page == ULTRA_PAGE_OUTPUTS) {
    const uint16_t ma[5] = {250, 250, 240, 255, 410};
    for (int i = 0; i < 5; i++)
      ultra_put16(d + ULTRA_OU_CURRENT + 2 * i, ma[i]);
    d[ULTRA_OU_WHY] = why;
  }
  a[0] = ULTRA_RSP_SYNC;
  a[1] = page;
  a[2] = static_cast<uint8_t>(s[ULTRA_POLL_INDEX] << 4);
  a[3] = s[2];
  ultra_put16(a + 4, uid);
  std::memcpy(a + ULTRA_RSP_DATA, d, 24);
  ultra_put16(a + 30, ultra_crc16(a, 30));
  out.push_back(ULTRA_RSP_LEAD);
  out.insert(out.end(), a, a + 32);
}

// ------------------------------------------------------------------------------------------- the dispatch (round 9)
/// The poller's per-receiver state from a test (its data, its answering), without the wire
struct PokePoller : TelemetryPoller {
  static ReceiverData &data(TelemetryPoller &p, uint16_t i) { return (p.*(&PokePoller::rx_))[i].d; }
  static void answering(TelemetryPoller &p, uint16_t i, bool on) { (p.*(&PokePoller::rx_))[i].answering = on; }
};

/// Two copies of a line's entities (every receiver value and text, its Problem, a light's estimate) on one hub's data
struct DispatchWorld {
  explicit DispatchWorld(uint16_t n) : vals(n), texts(n), problems(n), rx(n), lpow(n), lenergy(n), lstatus(n), lport(n),
                                       light(n) {}
  std::vector<std::array<sensor::Sensor, UltraReceiver::VALUE_COUNT>> vals;
  std::vector<std::array<text_sensor::TextSensor, UltraReceiver::TEXT_COUNT>> texts;
  std::vector<binary_sensor::BinarySensor> problems;
  std::vector<UltraReceiver> rx;
  std::vector<sensor::Sensor> lpow, lenergy;
  std::vector<text_sensor::TextSensor> lstatus, lport;
  std::vector<UltraLightTelemetry> light;
  std::vector<std::string> log;
  uint64_t calls{0};
  void attach(UltraHub &hub, uint32_t key_base) {
    for (uint16_t k = 0; k < this->rx.size(); k++) {
      UltraReceiver &r = this->rx[k];
      r.hub_ = &hub;
      r.board_ = k;
      r.problem_ = &this->problems[k];
      this->problems[k].id = 20000 + k;
      for (int i = 0; i < UltraReceiver::VALUE_COUNT; i++) {
        r.values_[i] = &this->vals[k][i];
        this->vals[k][i].id = k * 100 + i;
      }
      for (int i = 0; i < UltraReceiver::TEXT_COUNT; i++) {
        r.texts_[i] = &this->texts[k][i];
        this->texts[k][i].id = 10000 + k * 100 + i;
      }
      UltraLightTelemetry &l = this->light[k];
      l.hub_ = &hub;
      l.board_ = k;
      l.mask_ = 0x1F;
      l.key_ = key_base + k;   // (each copy its own ledger counters: both start from 0)
      l.power_ = &this->lpow[k];
      l.energy_ = &this->lenergy[k];
      l.status_ = &this->lstatus[k];
      l.port_ = &this->lport[k];
      this->lpow[k].id = 30000 + k;
      this->lenergy[k].id = 40000 + k;
      this->lstatus[k].id = 50000 + k;
      this->lport[k].id = 60000 + k;
    }
  }
};

/// Design review, round 9: a receiver's news goes to its own listeners only (ReceiverListeners) instead of to every
/// listener of the line, each filtering by its receiver (as before). Differential: n receivers with every entity and a
/// light each, windows of up to 16 receivers' news (values, availability, restarts, random): both ways give the same
/// publishes, in the same order, at the same moments. Also the listener calls a window and the host time a window,
/// with the entities' work and with the dispatch alone.
static void test_receiver_dispatch(uint16_t n, int windows) {
  now_ms = 0;
  UltraHub::ledger().init(256, 256);
  UltraHub hub;
  hub.p.init(n, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  PortMonitor::Config pcfg;
  pcfg.supply_v = 51.0f;
  hub.m.configure(pcfg);
  auto old_w = std::make_unique<DispatchWorld>(n), new_w = std::make_unique<DispatchWorld>(n);
  old_w->attach(hub, 0x01000000);
  new_w->attach(hub, 0x02000000);
  // before: every entity of the line for every receiver's news, filtering; now: ReceiverListeners. Added as ESPHome
  // sets them up: the receivers, then the lights
  std::vector<std::function<void(uint16_t, uint32_t)>> before;
  ReceiverListeners now;
  DispatchWorld &o = *old_w, &w = *new_w;
  for (uint16_t k = 0; k < n; k++) {
    before.push_back([&o, k](uint16_t receiver, uint32_t updates) {
      o.calls++;
      if (receiver == o.rx[k].board_)
        o.rx[k].update_(updates);
    });
    now.add(k, [&w, k](uint32_t updates) {
      w.calls++;
      w.rx[k].update_(updates);
    });
  }
  for (uint16_t k = 0; k < n; k++) {
    before.push_back([&o, k](uint16_t receiver, uint32_t updates) {
      o.calls++;
      if (receiver == o.light[k].board_)
        o.light[k].update_(updates);
    });
    now.add(k, [&w, k](uint32_t updates) {
      w.calls++;
      w.light[k].update_(updates);
    });
  }
  std::mt19937 rng(77);
  double t_before = 0, t_now = 0;
  std::vector<std::pair<uint16_t, uint32_t>> news;
  for (int win = 0; win < windows; win++) {
    now_ms += 250;
    news.clear();
    const int count = 1 + static_cast<int>(rng() % std::min<uint32_t>(16, n));
    std::vector<uint16_t> order(n);
    for (uint16_t k = 0; k < n; k++)
      order[k] = k;
    std::shuffle(order.begin(), order.end(), rng);
    order.resize(count);
    std::sort(order.begin(), order.end());   // (rw_publish_: the receivers in turn)
    for (uint16_t i : order) {
      ReceiverData &d = PokePoller::data(hub.p, i);
      d.have_identity = d.have_status = d.have_energy = true;
      d.features = ULTRA_FEAT_ENERGY | ULTRA_FEAT_LIFETIME;
      d.uptime_s += 1;
      d.vbus_10mv = static_cast<uint16_t>(4800 + rng() % 400);
      d.current_ma = static_cast<uint16_t>(rng() % 3000);
      d.measured = ULTRA_ST_M_VBUS | ULTRA_ST_M_CURRENT;
      d.mcu_temp = static_cast<uint16_t>(300 + rng() % 200);
      d.power_state = static_cast<uint8_t>(rng() % 8 == 0 ? rng() % 4 : 0);
      d.total_wh += rng() % 3;
      d.boot_mwh += rng() % 500;
      d.power_mw = rng() % 80000;
      d.have_lifetime = true;
      d.lt_power_ups += rng() % 64 == 0;
      uint32_t u = (1u << TP_STATUS) | (rng() % 4 == 0 ? 1u << TP_ENERGY : 0u) |
                   (rng() % 16 == 0 ? 1u << TP_LIFETIME : 0u);
      if (rng() % 40 == 0) {
        PokePoller::answering(hub.p, i, !hub.p.answering(i));
        u |= TU_AVAILABILITY;
      } else if (!hub.p.answering(i)) {
        PokePoller::answering(hub.p, i, true);
        u |= TU_AVAILABILITY;
      }
      if (rng() % 200 == 0) {
        d.uptime_s = 0;
        u |= TU_RESTART;
      }
      news.push_back({i, u});
    }
    pub_log = &o.log;
    auto t0 = std::chrono::steady_clock::now();
    for (const auto &nw : news)
      for (auto &f : before)
        f(nw.first, nw.second);
    auto t1 = std::chrono::steady_clock::now();
    pub_log = &w.log;
    for (const auto &nw : news)
      now.call(nw.first, nw.second);
    auto t2 = std::chrono::steady_clock::now();
    pub_log = nullptr;
    t_before += std::chrono::duration<double, std::micro>(t1 - t0).count();
    t_now += std::chrono::duration<double, std::micro>(t2 - t1).count();
  }
  size_t first_diff = o.log.size() == w.log.size() ? o.log.size() : std::min(o.log.size(), w.log.size());
  for (size_t i = 0; i < std::min(o.log.size(), w.log.size()); i++) {
    if (o.log[i] != w.log[i]) {
      first_diff = i;
      break;
    }
  }
  CHECK(o.log == w.log && !o.log.empty(),
        "dispatch, %u receivers: the same %zu publishes in the same order either way (first difference at %zu)",
        static_cast<unsigned>(n), o.log.size(), first_diff);
  // the dispatch alone (the entities' work left out): a call that only counts
  uint64_t c_before = 0, c_now = 0;
  std::vector<std::function<void(uint16_t, uint32_t)>> before2;
  ReceiverListeners now2;
  for (uint16_t rep = 0; rep < 2; rep++)
    for (uint16_t k = 0; k < n; k++) {
      before2.push_back([&c_before, k](uint16_t r, uint32_t) { c_before += r == k; });
      now2.add(k, [&c_now](uint32_t) { c_now++; });
    }
  const int reps = 20000;
  auto t0 = std::chrono::steady_clock::now();
  for (int r = 0; r < reps; r++)
    for (uint16_t i = 0; i < std::min<uint16_t>(16, n); i++)
      for (auto &f : before2)
        f(i, 1);
  auto t1 = std::chrono::steady_clock::now();
  for (int r = 0; r < reps; r++)
    for (uint16_t i = 0; i < std::min<uint16_t>(16, n); i++)
      now2.call(i, 1);
  auto t2 = std::chrono::steady_clock::now();
  CHECK(c_before == c_now, "dispatch alone: the same receivers reached (%llu / %llu)",
        static_cast<unsigned long long>(c_before), static_cast<unsigned long long>(c_now));
  std::printf("  dispatch, %u receivers + %u lights, %d windows: %.1f / %.1f listener calls a window (before / now), "
              "host %.1f / %.1f us a window with the entities' work; the dispatch alone, 16 receivers' news: %.2f / "
              "%.2f us\n",
              static_cast<unsigned>(n), static_cast<unsigned>(n), windows, double(o.calls) / windows,
              double(w.calls) / windows, t_before / windows, t_now / windows,
              std::chrono::duration<double, std::micro>(t1 - t0).count() / reps,
              std::chrono::duration<double, std::micro>(t2 - t1).count() / reps);
  now_ms = 0;
}

/// Two receivers, the second with several lights: its Status, Status detail and Problem, its values (unavailable when
/// it stops answering, the texts kept), the uptime / missed polls at most once a minute, the port's summary, a light's
/// estimate and status.
static void test_receiver_entities() {
  now_ms = 0;
  UltraHub::ledger().init(4, 8);   // (the ESP32's ledger, loaded: the lights' estimated energy is kept there)
  UltraHub hub;
  hub.p.init(2, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  hub.p.set_outputs(1, true);
  hub.p.locked(0);
  sensor::Sensor vals[2][UltraReceiver::VALUE_COUNT];
  text_sensor::TextSensor texts[2][UltraReceiver::TEXT_COUNT];
  binary_sensor::BinarySensor problems[2];
  UltraReceiver rx[2];
  for (uint16_t k = 0; k < 2; k++) {
    rx[k].hub_ = &hub;
    rx[k].board_ = k;
    rx[k].problem_ = &problems[k];
    for (int i = 0; i < UltraReceiver::VALUE_COUNT; i++)
      rx[k].values_[i] = &vals[k][i];
    for (int i = 0; i < UltraReceiver::TEXT_COUNT; i++)
      rx[k].texts_[i] = &texts[k][i];
  }
  sensor::Sensor port_values[UltraTelemetryPort::VALUE_COUNT], light_power, light_energy;
  sensor::Sensor &port_power = port_values[UltraTelemetryPort::POWER];
  binary_sensor::BinarySensor port_problem;
  text_sensor::TextSensor port_texts[UltraTelemetryPort::TEXT_COUNT], light_status, light_port;
  text_sensor::TextSensor &port_receivers = port_texts[UltraTelemetryPort::RECEIVERS];
  UltraTelemetryPort port;
  port.hub_ = &hub;
  port.problem_ = &port_problem;
  for (int i = 0; i < UltraTelemetryPort::VALUE_COUNT; i++)
    port.values_[i] = &port_values[i];
  for (int i = 0; i < UltraTelemetryPort::TEXT_COUNT; i++)
    port.texts_[i] = &port_texts[i];
  sensor::Sensor port_health[UltraTelemetryPort::HEALTH_COUNT];
  for (int i = 0; i < UltraTelemetryPort::HEALTH_COUNT; i++)
    port.health_[i] = &port_health[i];
  port.has_health_ = true;
  PortMonitor::Config pcfg;
  pcfg.supply_v = 51.0f;
  hub.m.configure(pcfg);
  UltraLightTelemetry light;
  light.hub_ = &hub;
  light.board_ = 1;
  light.mask_ = 0x10;   // CH5
  light.power_ = &light_power;
  light.energy_ = &light_energy;
  light.status_ = &light_status;
  light.port_ = &light_port;
  hub.port_wh = 5.0;   // (the hub's port energy: published every 5 min, at once the first time)

  ReceiverData model[2];
  for (auto &m : model) {
    m.vbus_10mv = 5000;   // 50.00 V x 0.680 A = 34.0 W (Power: STATUS's voltage x current)
    m.mcu_temp = 380;
    m.current_ma = 680;
    m.uptime_s = 100;
    m.features = ULTRA_FEAT_ENERGY;
  }
  // receiver 2 is a 0.2.5 one: LED supply and peak (ENERGY), LIFETIME, DIAG's two parts
  model[1].features |= ULTRA_FEAT_LIFETIME | ULTRA_FEAT_DIAG;
  model[1].vb_10mv = 2400;
  model[1].peak_ma = 900;
  model[1].lt_powered = 23456;
  model[1].lt_lit = 12345;
  model[1].lt_power_ups = 41;
  model[1].lt_short_trips = 3;
  model[1].lt_dips = 2;
  model[1].supply_z_mohm = 95;
  model[1].fm_trip = 1 | 25 << 2;   // a short at 12.5 A, at 1100 lit hours
  model[1].flash_pending = ULTRA_DG_FP_EXHAUSTED;   // (0.2.6) its lifetime record held
  model[1].fm_trip_lit_h = 1100;
  uint8_t why = 0;
  bool silent0 = false;
  auto window = [&]() {
    std::vector<uint8_t> slots(64), uart;
    uint32_t w;
    const uint16_t n = hub.p.build(now_ms, slots.data(), &w);
    for (uint16_t b = 0; b < n; b++) {
      if (b == 0 && silent0)
        break;   // (it silences the one behind it too)
      if (slots[32 * b + ULTRA_POLL_INDEX] != ULTRA_POLL_RELAY_ONLY)
        answer(slots.data() + 32 * b, static_cast<uint16_t>(0x9C00 + b), model[b], why, uart);
    }
    now_ms += 10;
    hub.p.finish(now_ms, uart.data(), uart.size());
    hub.m.update(now_ms, hub.p);
    for (uint16_t k = 0; k < 2; k++) {   // as UltraHub::rw_publish_: the port's load, the ledger, the callbacks
      const uint32_t u = hub.p.take_updates(k);
      if (u == 0)
        continue;
      hub.keeper.on_updates(hub.p, k, u, now_ms);
      rx[k].update_(u);
      if (k == 1)
        light.update_(u);
    }
    port.update_();
  };
  auto run = [&](uint32_t ms) {
    const uint32_t end = now_ms + ms;
    while (now_ms < end) {
      if (hub.p.due(now_ms))
        window();
      else
        now_ms += 50;
    }
  };
  run(20000);
  // (R25 / R12) the live twin at the update rate, the recorded Power the 10 s mean of the same samples (at most every
  // 10 s after its first value); the port's live power and energy; the light's Port (R10)
  bool spaced = true;
  const auto &ph = vals[0][UltraReceiver::POWER].history;
  for (size_t i = 2; i < ph.size(); i++)
    spaced &= ph[i].first - ph[i - 1].first >= 10000;
  CHECK(std::fabs(vals[0][UltraReceiver::LIVE_POWER].state - 34.0f) < 1e-3f && spaced &&
            std::fabs(vals[0][UltraReceiver::POWER].state - 34.0f) < 1e-3f,
        "Live power %.1f W, Power %.1f W (%d sent, 10 s apart: %d)", vals[0][UltraReceiver::LIVE_POWER].state,
        vals[0][UltraReceiver::POWER].state, vals[0][UltraReceiver::POWER].publishes, spaced);
  CHECK(std::fabs(port_values[UltraTelemetryPort::LIVE_POWER].state - 51.0f * 1.36f) < 1e-3f &&
            port_values[UltraTelemetryPort::ENERGY].state == 5.0f && light_port.state == "Port 4 \xC2\xB7 receiver 2",
        "the port's live power %.1f W, its energy %.1f Wh; the light's Port '%s'",
        port_values[UltraTelemetryPort::LIVE_POWER].state, port_values[UltraTelemetryPort::ENERGY].state,
        light_port.state.c_str());
  CHECK(texts[0][UltraReceiver::STATUS].state == "OK" && !problems[0].state &&
            texts[0][UltraReceiver::STATUS_DETAIL].state == "OK." &&
            std::fabs(vals[0][UltraReceiver::POWER].state - 34.0f) < 1e-3f &&
            vals[0][UltraReceiver::ENERGY].state == 1234.0f &&
            std::fabs(vals[0][UltraReceiver::BUS_VOLTAGE].state - 50.0f) < 1e-3f &&
            texts[0][UltraReceiver::FIRMWARE].state == "0.6.8" &&
            texts[1][UltraReceiver::LINK].state == "2 \xC2\xB7 UID 9C01",
        "receiver entities: Status '%s', Power %.1f W, Energy %.0f Wh, bus %.2f V, firmware '%s', link '%s'",
        texts[0][UltraReceiver::STATUS].state.c_str(), vals[0][UltraReceiver::POWER].state,
        vals[0][UltraReceiver::ENERGY].state, vals[0][UltraReceiver::BUS_VOLTAGE].state,
        texts[0][UltraReceiver::FIRMWARE].state.c_str(), texts[1][UltraReceiver::LINK].state.c_str());
  // the port: 1.36 A (60 %) at a 51 V sender: 69.4 W; the cable: (51 - 50) V x 1.36 A + (50 - 50) V x 0.68 A
  CHECK(port_receivers.state == "2 of 2 answering" && std::fabs(port_power.state - 51.0f * 1.36f) < 1e-3f &&
            std::fabs(port_values[UltraTelemetryPort::CURRENT].state - 1.36f) < 1e-4f &&
            std::fabs(port_values[UltraTelemetryPort::LOAD].state - 1.36f / 2.25f * 100.0f) < 1e-3f &&
            std::fabs(port_values[UltraTelemetryPort::CABLE_LOSS].state - 1.36f) < 1e-3f &&
            port_texts[UltraTelemetryPort::STATUS].state == "OK" && !port_problem.state &&
            std::fabs(vals[1][UltraReceiver::VOLTAGE_DROP].state - 1.0f) < 1e-3f,
        "port: '%s', %.2f A, %.1f W, cable loss %.2f W, status '%s'; receiver 2's drop %.2f V",
        port_receivers.state.c_str(), port_values[UltraTelemetryPort::CURRENT].state, port_power.state,
        port_values[UltraTelemetryPort::CABLE_LOSS].state, port_texts[UltraTelemetryPort::STATUS].state.c_str(),
        vals[1][UltraReceiver::VOLTAGE_DROP].state);
  CHECK(std::fabs(light_power.state - 34.0f * 410 / 1405) < 1e-3f && light_status.state == "OK" &&
            light_energy.has_state(),
        "a light's estimate: %.2f W, status '%s'", light_power.state, light_status.state.c_str());
  // (Power: unavailable, then V x I from STATUS, then ENERGY's own; STATUS came ~6 times)
  // (0.2.6) receiver 2's lifetime record held: a note after its Status detail, no problem; receiver 1 none
  CHECK(texts[1][UltraReceiver::STATUS_DETAIL].state.find("Lifetime record held: the receiver saves less often") !=
                std::string::npos &&
            texts[0][UltraReceiver::STATUS_DETAIL].state.find("Lifetime record held") == std::string::npos,
        "the record held: '%s'", texts[1][UltraReceiver::STATUS_DETAIL].state.c_str());
  // (0.2.6) the lifetime counts and Energy through the hub's ledger: a lower LIFETIME (its record held, a reload)
  // doesn't lower them
  model[1].lt_lit = 12000;
  model[1].lt_power_ups = 40;
  run(8000);
  CHECK(std::fabs(vals[1][UltraReceiver::LIT_HOURS].state - 1234.5f) < 1e-2f &&
            vals[1][UltraReceiver::POWER_UPS].state == 41.0f && vals[0][UltraReceiver::ENERGY].state == 1234.0f &&
            hub.keeper.counters(1) != nullptr && hub.keeper.counters(1)->lit == 12345,
        "the ledger's counts shown: lit %.1f h, %.0f power-ups (the board now says 1200.0 / 40), Energy %.1f Wh",
        vals[1][UltraReceiver::LIT_HOURS].state, vals[1][UltraReceiver::POWER_UPS].state,
        vals[0][UltraReceiver::ENERGY].state);
  model[1].lt_lit = 12345;
  model[1].lt_power_ups = 41;
  CHECK(vals[0][UltraReceiver::UPTIME].publishes == 2 && vals[0][UltraReceiver::POWER].publishes <= 3,
        "the uptime: unknown, then its first value, then hourly (R15); unchanged values not again (%d, %d)",
        vals[0][UltraReceiver::UPTIME].publishes, vals[0][UltraReceiver::POWER].publishes);
  // (0.2.5) receiver 2's new entities; receiver 1 (0.2.4): unknown; the port's peak load: 0.68 + 0.90 A
  CHECK(std::fabs(vals[1][UltraReceiver::LED_SUPPLY].state - 24.0f) < 1e-3f &&
            std::fabs(vals[1][UltraReceiver::CURRENT_PEAK].state - 0.9f) < 1e-4f &&
            vals[1][UltraReceiver::SUPPLY_IMPEDANCE].state == 95.0f &&
            std::fabs(vals[1][UltraReceiver::LIT_HOURS].state - 1234.5f) < 1e-2f &&
            vals[1][UltraReceiver::POWER_UPS].state == 41.0f && vals[1][UltraReceiver::TRIPS].state == 3.0f &&
            vals[1][UltraReceiver::SUPPLY_DIPS].state == 2.0f &&
            texts[1][UltraReceiver::LAST_TRIP].state == "short circuit at 12.5 A (at 1100 lit hours, before its last "
                                                        "power-up)" &&
            std::isnan(vals[0][UltraReceiver::LED_SUPPLY].state) && std::isnan(vals[0][UltraReceiver::LIT_HOURS].state) &&
            std::fabs(port_values[UltraTelemetryPort::PEAK_LOAD].state - 1.58f / 2.25f * 100.0f) < 1e-3f,
        "0.2.5 entities: LED %.2f V, peak %.3f A, Z %.0f, lit %.1f h, %.0f ups, %.0f trips, last trip '%s'; port peak "
        "%.1f %%",
        vals[1][UltraReceiver::LED_SUPPLY].state, vals[1][UltraReceiver::CURRENT_PEAK].state,
        vals[1][UltraReceiver::SUPPLY_IMPEDANCE].state, vals[1][UltraReceiver::LIT_HOURS].state,
        vals[1][UltraReceiver::POWER_UPS].state, vals[1][UltraReceiver::TRIPS].state,
        texts[1][UltraReceiver::LAST_TRIP].state.c_str(), port_values[UltraTelemetryPort::PEAK_LOAD].state);

  // limiting on receiver 2: its status, its light's; the port without a problem
  model[1].power_state = ULTRA_ST_LIMITING;
  why = ULTRA_OU_W_LIMIT;
  run(8000);
  CHECK(texts[1][UltraReceiver::STATUS].state == "Limiting" && !problems[1].state && light_status.state == "Limiting" &&
            !port_problem.state && port_texts[UltraTelemetryPort::STATUS].state == "1 limiting",
        "limiting: '%s', the light '%s'", texts[1][UltraReceiver::STATUS].state.c_str(), light_status.state.c_str());

  // (0.2.5) STATUS's reason "too hot" on receiver 2: its Status and Problem; its light isn't dimmed by it (Limiting)
  model[1].power_state = ULTRA_ST_LIMITING | ULTRA_ST_WHY_HOT << ULTRA_ST_WHY_SHIFT;
  run(8000);
  CHECK(texts[1][UltraReceiver::STATUS].state == "Too hot" && problems[1].state && light_status.state == "Limiting" &&
            port_problem.state && port_texts[UltraTelemetryPort::STATUS].state == "Too hot",
        "too hot: '%s' / '%s', the light '%s', port '%s'", texts[1][UltraReceiver::STATUS].state.c_str(),
        texts[1][UltraReceiver::STATUS_DETAIL].state.c_str(), light_status.state.c_str(),
        port_texts[UltraTelemetryPort::STATUS].state.c_str());
  model[1].power_state = ULTRA_ST_LIMITING;
  run(4000);

  // receiver 1 stops answering: No answer, a problem, values unavailable, texts kept; receiver 2 behind it too
  silent0 = true;
  run(30000);
  CHECK(texts[0][UltraReceiver::STATUS].state == "No answer" && problems[0].state &&
            std::isnan(vals[0][UltraReceiver::POWER].state) && texts[0][UltraReceiver::FIRMWARE].state == "0.6.8" &&
            texts[1][UltraReceiver::STATUS_DETAIL].state.find("receiver 1 before it doesn't answer") != std::string::npos &&
            port_problem.state && port_receivers.state == "0 of 2 answering" && std::isnan(port_power.state) &&
            port_texts[UltraTelemetryPort::STATUS].state == "No answer" &&
            light_status.state == "No answer" && std::isnan(light_power.state),
        "no answer: '%s' / '%s', port '%s'", texts[0][UltraReceiver::STATUS_DETAIL].state.c_str(),
        texts[1][UltraReceiver::STATUS_DETAIL].state.c_str(), port_receivers.state.c_str());
  CHECK(vals[0][UltraReceiver::MISSED_POLLS].has_state() && vals[0][UltraReceiver::MISSED_POLLS].state > 0.0f,
        "missed polls: %.1f %%", vals[0][UltraReceiver::MISSED_POLLS].state);
  silent0 = false;
  run(20000);
  CHECK(texts[0][UltraReceiver::STATUS].state == "OK" && port_receivers.state == "2 of 2 answering",
        "back: '%s', '%s'", texts[0][UltraReceiver::STATUS].state.c_str(), port_receivers.state.c_str());

  // the line's Diff-Power port in standby: the receivers can't answer, and that's no problem (the hub's rw_due_
  // touches every receiver when the power changes)
  hub.standby = true;
  silent0 = true;
  hub.p.touch_all();
  run(30000);
  CHECK(texts[0][UltraReceiver::STATUS].state == "Standby" && !problems[0].state && !problems[1].state &&
            texts[1][UltraReceiver::STATUS].state == "Standby" && !port_problem.state &&
            port_texts[UltraTelemetryPort::STATUS].state == "Standby" && light_status.state == "Standby",
        "port in standby: '%s' / '%s', problems %d %d, port '%s' (%d), light '%s'",
        texts[0][UltraReceiver::STATUS].state.c_str(), texts[1][UltraReceiver::STATUS].state.c_str(), problems[0].state,
        problems[1].state, port_texts[UltraTelemetryPort::STATUS].state.c_str(), port_problem.state,
        light_status.state.c_str());
  CHECK(vals[1][UltraReceiver::POWER].state == 0.0f && vals[1][UltraReceiver::INPUT_CURRENT].state == 0.0f &&
            std::isnan(vals[1][UltraReceiver::BUS_VOLTAGE].state) &&
            std::isnan(vals[1][UltraReceiver::MCU_TEMPERATURE].state) &&
            !std::isnan(vals[1][UltraReceiver::ENERGY].state) && port_power.state == 0.0f &&
            port_values[UltraTelemetryPort::CURRENT].state == 0.0f &&
            std::isnan(port_values[UltraTelemetryPort::LOAD].state) && light_power.state == 0.0f,
        "port in standby: Power / Current 0 (receiver %.1f W, port %.1f W, light %.1f W), the rest unknown, Energy "
        "kept (%.1f)",
        vals[1][UltraReceiver::POWER].state, port_power.state, light_power.state, vals[1][UltraReceiver::ENERGY].state);
  // ... its power mode Off (round 12): "Port off" / "Off", as Standby otherwise (no problem, Power 0)
  hub.off = true;
  hub.p.touch_all();
  run(30000);
  CHECK(texts[0][UltraReceiver::STATUS].state == "Port off" && texts[1][UltraReceiver::STATUS].state == "Port off" &&
            texts[0][UltraReceiver::STATUS_DETAIL].state.find("power mode is Off") != std::string::npos &&
            !problems[0].state && !problems[1].state && !port_problem.state &&
            port_texts[UltraTelemetryPort::STATUS].state == "Off" && light_status.state == "Port off" &&
            vals[1][UltraReceiver::POWER].state == 0.0f && port_power.state == 0.0f,
        "port Off: '%s' ('%s'), port '%s', light '%s'", texts[0][UltraReceiver::STATUS].state.c_str(),
        texts[0][UltraReceiver::STATUS_DETAIL].state.c_str(), port_texts[UltraTelemetryPort::STATUS].state.c_str(),
        light_status.state.c_str());
  hub.off = false;
  // just woken, not heard yet: Waking and no problem (receivers, light, port), not No answer (design review, round 8)
  hub.standby = false;
  hub.settling = true;
  hub.p.touch_all();
  run(30000);
  CHECK(texts[0][UltraReceiver::STATUS].state == "Waking" && texts[1][UltraReceiver::STATUS].state == "Waking" &&
            !problems[0].state && !problems[1].state && !port_problem.state &&
            port_texts[UltraTelemetryPort::STATUS].state == "Waking" && light_status.state == "Waking",
        "just woken: '%s' / '%s', problems %d %d, port '%s' (%d), light '%s'",
        texts[0][UltraReceiver::STATUS].state.c_str(), texts[1][UltraReceiver::STATUS].state.c_str(), problems[0].state,
        problems[1].state, port_texts[UltraTelemetryPort::STATUS].state.c_str(), port_problem.state,
        light_status.state.c_str());
  hub.settling = false;
  // negative control: powered again (evaluated), silent: No answer and a problem as before
  hub.p.touch_all();
  run(30000);
  CHECK(texts[0][UltraReceiver::STATUS].state == "No answer" && problems[0].state && port_problem.state,
        "powered and silent: No answer again");
  silent0 = false;
  run(20000);
  // the line's health (round 12; diagnostic, disabled by default): once a minute, the missing answers of that minute
  using H = UltraTelemetryPort;
  run(125000);   // (all answer: a whole minute without missing ones)
  const float clean = port_health[H::H_MISSING].state, w0 = port_health[H::H_WINDOWS].state;
  const int pubs = port_health[H::H_WINDOWS].publishes;
  hub.late = 7;
  hub.reply.configure(ULTRA_RW_RATE_500K, false);
  silent0 = true;
  run(125000);   // (receiver 1 silent, receiver 2 behind it: every answer of a whole minute missing)
  CHECK(clean == 0.0f && port_health[H::H_MISSING].state == 100.0f && port_health[H::H_WINDOWS].state > w0 &&
            port_health[H::H_WINDOWS].publishes - pubs <= 3 && port_health[H::H_BROKEN].state == 0.0f &&
            port_health[H::H_CONTENTION].state == 0.0f && port_health[H::H_LATE].state == 7.0f &&
            port_health[H::H_RATE].state == 500.0f && port_health[H::H_RESTARTS].has_state(),
        "health: missing %.0f %% then %.0f %%, windows %.0f -> %.0f (%d publishes in 2 min), late %.0f, rate %.0f kbit/s",
        clean, port_health[H::H_MISSING].state, w0, port_health[H::H_WINDOWS].state,
        port_health[H::H_WINDOWS].publishes - pubs, port_health[H::H_LATE].state, port_health[H::H_RATE].state);
  silent0 = false;
  run(20000);
  now_ms = 0;
  std::printf("  receiver entities: Status / Problem / detail, values, the port's summary, a light's estimate, the "
              "line's health\n");
}

}  // namespace receiver_entities

// ================================================================================================ Diff-Power port
// components/diffpower/diffpower.cpp's methods (build/lifecycle/diffpower_methods.inc) on the real PortCore: the
// enable / detect pins, a plain light (any ESPHome light) and its state sent again once the port is powered, the
// notice's copies asked one at a time and timed from their delivery (a modelled hub), the status texts, the counters
// (saved, kept apart per port id), the select's restore, and the shutdown hook: 2 ports on and 1 in standby, a port
// already in its notice, 4 slow transmitters, a quiet port's standby time (design review, round 8).
namespace port_power {
#define USE_TEXT_SENSOR
#define USE_BINARY_SENSOR
#define USE_SENSOR
#define USE_SELECT
using namespace ::esphome::diffpower;

struct Pin {
  bool level{false};
  int writes{0};
  uint32_t changed_at{0};
  void setup() {}
  void digital_write(bool v) {
    if (v != this->level)
      this->changed_at = now_ms;
    this->level = v;
    this->writes++;
  }
  bool digital_read() const {
    this->reads++;
    return this->level;
  }
  mutable int reads{0};
};
struct LightState;
struct LightOutput {
  int writes{0};
  void write_state(LightState *) { this->writes++; }
};
struct Values {
  bool on{false};
  bool is_on() const { return this->on; }
};
struct LightState {   // a plain ESPHome light (its output, its target and current values)
  Values current_values, remote_values;
  LightOutput out;
  LightOutput *get_output() { return &this->out; }
};
struct TextSensor {
  std::string state;
  bool has{false};
  std::vector<std::string> history;
  bool has_state() const { return this->has; }
  void publish_state(const std::string &s) {
    if (this->history.empty() || this->history.back() != s)
      this->history.push_back(s);
    this->state = s;
    this->has = true;
  }
};
struct BinarySensor {
  bool state{false};
  void publish_state(bool v) { this->state = v; }
};
struct Sensor {
  float state{NAN};
  bool has{false};
  bool has_state() const { return this->has; }
  void publish_state(float v) {
    this->state = v;
    this->has = true;
  }
};
namespace light {
using LightState = port_power::LightState;
}
namespace text_sensor {
using TextSensor = port_power::TextSensor;
}
namespace binary_sensor {
using BinarySensor = port_power::BinarySensor;
}
namespace sensor {
using Sensor = port_power::Sensor;
}
struct Callbacks {
  std::vector<std::function<void()>> f;
  void call() {
    for (auto &x : this->f)
      x();
  }
  uint16_t size() const { return static_cast<uint16_t>(this->f.size()); }
};
struct NoticeCallbacks {
  std::vector<std::function<void(uint8_t)>> f;
  void call(uint8_t copy) {
    for (auto &x : this->f)
      x(copy);
  }
  uint16_t size() const { return static_cast<uint16_t>(this->f.size()); }
};
static const char *const TAG = "diffpower";
static const uint32_t RESEND_MS[] = {300, 1000};
static constexpr uint32_t MINUTE_MS = 60000;
static constexpr uint32_t SAVE_STANDBY_S = 600;
static constexpr uint32_t DETECT_READ_MS = 5;
static constexpr uint32_t SHUTDOWN_MARGIN_MS = 100;
// the shutdown's stand-ins: delay() lets the time pass, the watchdog; the preferences (a store by key, its sync)
static void delay(uint32_t ms) { now_ms += ms; }
struct AppStub {
  void feed_wdt() {}
};
static AppStub App;
struct PortStats {
  uint32_t cycles, standby_s, drops;
};
static std::vector<std::pair<uint32_t, PortStats>> stored;   // the ESP32's preferences: key -> counters
struct StatsPref {
  uint32_t key{0};
  PortStats v{};
  int saves{0};
  bool load(PortStats *s) {
    for (const auto &e : stored) {
      if (e.first == this->key) {
        *s = e.second;
        return true;
      }
    }
    return false;
  }
  bool save(const PortStats *s) {
    this->v = *s;
    this->saves++;
    for (auto &e : stored) {
      if (e.first == this->key) {
        e.second = *s;
        return true;
      }
    }
    stored.push_back({this->key, *s});
    return true;
  }
};
struct Prefs {
  int syncs{0};
  bool sync() {
    this->syncs++;
    return true;
  }
  template<typename T> StatsPref make_preference(uint32_t key) {
    StatsPref p;
    p.key = key;
    return p;
  }
};
static Prefs prefs;
static Prefs *const global_preferences = &prefs;
static uint32_t fnv1_hash(const std::string &s) {
  uint32_t h = 2166136261u;
  for (char c : s) {
    h *= 16777619u;
    h ^= static_cast<uint8_t>(c);
  }
  return h;
}

struct DiffPowerPort {
  struct LoadRef {
    std::function<bool(uint32_t)> lit;
    LightState *light;
    bool resend;
  };
  using Stats = PortStats;
  // the component's (build/lifecycle/diffpower_methods.inc)
  void setup();
  void loop();
  void apply_(uint8_t ev, uint32_t now);
  void resend_();
  void save_stats_();
  void publish_();
  static void shutdown_all();
  void wake(uint32_t timeout_ms);
  bool has_light(LightState *light) const;
  // as diffpower.h
  void add_light(LightState *light, uint32_t hold_ms) {
    this->core_.add_load(hold_ms);
    this->loads_.push_back(
        LoadRef{[light](uint32_t) { return light->current_values.is_on() || light->remote_values.is_on(); }, light, true});
  }
  void add_on_notice_callback(std::function<void(uint8_t)> &&cb) { this->notice_cb_.f.push_back(std::move(cb)); }
  void add_on_pump_callback(std::function<void()> &&cb) { this->pump_cb_.f.push_back(std::move(cb)); }
  void add_on_prepare_callback(std::function<bool(bool)> &&cb) {
    this->prepare_cb_.push_back(std::move(cb));
    PortConfig c = this->core_.config();
    c.prepare_max_ms = PREPARE_MAX_MS;
    this->core_.configure(c);
  }
  void notice_sent() { this->core_.notice_sent(millis()); }
  bool has_dark_source() const { return this->dark; }
  void set_mode(uint8_t mode) { this->core_.set_mode(static_cast<PowerMode>(mode)); }
  uint8_t get_mode() const { return this->core_.mode(); }
  // stand-ins
  bool dark{false};   // (a dark source: Home Assistant's sun times or ESPHome's sun)
  void update_dark_(uint32_t) {}
  struct Timer {
    uint32_t at;
    std::function<void()> f;
  };
  std::vector<Timer> timers;
  void set_timeout(uint32_t ms, std::function<void()> &&f) { this->timers.push_back(Timer{now_ms + ms, std::move(f)}); }
  void run_timers() {
    for (size_t i = 0; i < this->timers.size();) {
      if (static_cast<int32_t>(now_ms - this->timers[i].at) >= 0) {
        auto f = std::move(this->timers[i].f);
        this->timers.erase(this->timers.begin() + static_cast<std::ptrdiff_t>(i));
        f();
      } else {
        i++;
      }
    }
  }

  StatsPref stats_pref_;
  PortCore core_;
  Pin *enable_pin_{nullptr};
  Pin *detect_pin_{nullptr};
  const char *name_{"Port 1"};
  const char *stats_key_{"port1"};
  std::vector<LoadRef> loads_;
  Callbacks ready_cb_, pump_cb_;
  NoticeCallbacks notice_cb_;
  std::vector<std::function<bool(bool)>> prepare_cb_;
  bool prepare_first_{false};
  static constexpr uint32_t PREPARE_MAX_MS = 3000;
  uint32_t saved_standby_s_{0}, saved_cycles_{0}, saved_drops_{0}, minute_ms_{0};
  uint32_t detect_ms_{0};
  bool detect_raw_{true}, detect_read_{false};
  bool off_seen_{false};
  TextSensor *status_{nullptr};
  BinarySensor *problem_{nullptr};
  Sensor *cycles_{nullptr}, *hours_{nullptr}, *drops_{nullptr};
  static std::vector<DiffPowerPort *> ports_;
  static bool shut_down_;
};
std::vector<DiffPowerPort *> DiffPowerPort::ports_;
bool DiffPowerPort::shut_down_{false};

struct PowerModeSelect {
  void setup();
  uint8_t mode_of_(size_t index) const;
  size_t index_of_(uint8_t mode) const;
  void control(size_t index);
  struct Pref {
    uint32_t v{0};
    bool have{false};
    bool load(uint32_t *p) {
      if (!this->have)
        return false;
      *p = this->v;
      return true;
    }
    bool save(const uint32_t *p) {
      this->v = *p;
      this->have = true;
      return true;
    }
  } pref_;
  template<typename T> Pref make_entity_preference() { return this->pref_; }
  size_t published{99};
  void publish_state(size_t i) { this->published = i; }
  DiffPowerPort *port_{nullptr};
};

#include "diffpower_methods.inc"

/// An Ultra hub as its port sees it: a copy asked for is queued; it goes out at the hub's next loop, or at a pump
/// (the shutdown), each costing `cost` ms there (a slow transmitter), and the hub reports it out.
struct HubModel {
  DiffPowerPort *port{nullptr};
  Pin *enable{nullptr};
  bool queued{false};
  uint32_t cost{0};
  std::vector<uint32_t> asked, out;
  std::vector<uint8_t> index;
  std::vector<bool> enabled, at_shutdown;   // the enable as each copy went out; out from a pump
  bool pumping{false};
  void attach(DiffPowerPort &p, Pin &en) {
    this->port = &p;
    this->enable = &en;
    p.add_on_notice_callback([this](uint8_t c) {
      this->queued = true;
      this->asked.push_back(now_ms);
      this->index.push_back(c);
    });
    p.add_on_pump_callback([this]() {
      this->pumping = true;
      this->service();
      this->pumping = false;
    });
  }
  void service() {
    now_ms += this->cost;
    if (!this->queued)
      return;
    this->queued = false;
    this->out.push_back(now_ms);
    this->enabled.push_back(this->enable->level);
    this->at_shutdown.push_back(this->pumping);
    this->port->notice_sent();
  }
  /// each copy asked 50 ms after the last one went out, all with the enable high, the enable low 100 ms after the third
  bool well_spaced() const {
    bool ok = this->out.size() == 3 && this->asked.size() == 3 && this->index == std::vector<uint8_t>{0, 1, 2};
    for (size_t i = 1; ok && i < 3; i++)
      ok = this->asked[i] - this->out[i - 1] >= 50;
    for (bool e : this->enabled)
      ok = ok && e;
    return ok && !this->enable->level && this->enable->changed_at - this->out[2] >= 100;
  }
};

/// 10 ms steps: the hubs' loops (a queued copy out), then the ports' loops and timers
static void run(std::vector<DiffPowerPort *> ports, uint32_t ms, std::vector<HubModel *> hubs = {}) {
  for (uint32_t end = now_ms + ms; static_cast<int32_t>(now_ms - end) < 0; now_ms += 10) {
    for (auto *h : hubs)
      h->service();
    for (auto *p : ports) {
      p->loop();
      p->run_timers();
    }
  }
}

/// The detect input is read every 5 ms, not every loop (an expander pin: a bus transfer each), whatever the loop's
/// pace: a second of 1 ms loops reads it ~200 times; a 20 ms dropout while powered is still a drop (the 55 V cut).
static void test_detect_rate() {
  now_ms = 1000;
  Pin enable, detect;
  detect.level = true;
  LightState strip;
  DiffPowerPort p;
  p.enable_pin_ = &enable;
  p.detect_pin_ = &detect;
  p.add_light(&strip, 0);
  strip.remote_values.on = strip.current_values.on = true;
  run({&p}, 5000);   // (powered: On)
  const int r0 = detect.reads;
  for (int ms = 0; ms < 1000; ms++, now_ms++) {
    p.loop();
    p.run_timers();
  }
  const int reads = detect.reads - r0;
  const uint32_t drops0 = p.core_.drops();
  detect.level = false;
  for (int ms = 0; ms < 20; ms++, now_ms++)
    p.loop();
  detect.level = true;
  for (int ms = 0; ms < 100; ms++, now_ms++)
    p.loop();
  CHECK(reads >= 195 && reads <= 201 && p.core_.drops() == drops0 + 1,
        "detect read %d times in a second of 1 ms loops (every 5 ms); a 20 ms dropout counted (%u drops)", reads,
        p.core_.drops());
}

static void test_port() {
  now_ms = 1000;
  Pin enable, detect;
  detect.level = true;
  LightState strip, other;
  TextSensor status;
  BinarySensor problem;
  Sensor cycles, hours, drops;
  DiffPowerPort p;
  PortConfig cfg;
  cfg.standby_after_ms = 60000;   // (floored to min_on_time: 2 min)
  p.core_.configure(cfg);
  p.enable_pin_ = &enable;
  p.detect_pin_ = &detect;
  p.status_ = &status;
  p.problem_ = &problem;
  p.cycles_ = &cycles;
  p.hours_ = &hours;
  p.drops_ = &drops;
  p.add_light(&strip, 0);
  HubModel hub;
  hub.attach(p, enable);
  int readies = 0;
  p.ready_cb_.f.push_back([&] { readies++; });
  run({&p}, 1000, {&hub});
  CHECK(status.state == "Standby" && !enable.level && strip.out.writes == 0, "at rest: Standby, enable low");
  // the plain light on (its data goes out now, into the dark): the port wakes; once powered, its state again 3 times
  strip.remote_values.on = strip.current_values.on = true;
  run({&p}, 1000, {&hub});
  CHECK(status.state == "Waking" && enable.level && strip.out.writes == 0, "waking: enable high, nothing sent yet");
  run({&p}, 2000, {&hub});
  CHECK(status.state == "On" && readies == 1 && strip.out.writes == 3,
        "on (1.7 s): the light's state sent again at once, +300, +1000 ms (%d)", strip.out.writes);
  // off: the turn-off period (2 min), the notice's copies (each out at the hub's next loop), the enable low after them
  strip.remote_values.on = strip.current_values.on = false;
  const uint32_t off = now_ms;
  run({&p}, 119000, {&hub});
  CHECK(enable.level && hub.asked.empty(), "the turn-off period (floored to 2 min): still on");
  run({&p}, 3000, {&hub});
  CHECK(hub.well_spaced() && hub.asked[0] - off >= 120000,
        "the notice: 3 copies, each asked 50 ms after the last one went out, the enable high; low 100 ms after the "
        "third (%u)",
        static_cast<unsigned>(hub.out.size()));
  CHECK(status.state == "Standby" && cycles.state == 1.0f && p.stats_pref_.v.cycles == 1,
        "then Standby, a standby cycle published and saved");
  CHECK(status.history.size() >= 5 && status.history[1] == "Waking" && status.history[2] == "On" &&
            status.history[3] == "Powering off",
        "the status: Standby, Waking, On, Powering off, Standby");
  run({&p}, 61000, {&hub});
  CHECK(hours.state > 0.0f && hours.state < 0.05f, "hours in standby published once a minute (%.2f)", hours.state);
  CHECK(p.has_light(&strip) && !p.has_light(&other), "has_light");

  // the shutdown hook: p and q on, r in standby
  Pin en2, det2, en3, det3;
  det2.level = det3.level = true;
  DiffPowerPort q, r;
  q.enable_pin_ = &en2;
  q.detect_pin_ = &det2;
  r.enable_pin_ = &en3;
  r.detect_pin_ = &det3;
  HubModel hp, hq, hr;
  hp.attach(p, enable);   // (p's first hub: asked copies only before)
  hq.attach(q, en2);
  hr.attach(r, en3);
  hub.queued = false;
  DiffPowerPort::ports_ = {&p, &q, &r};
  p.wake(120000);
  q.wake(120000);
  run({&p, &q, &r}, 3000, {&hub, &hp, &hq, &hr});
  CHECK(enable.level && en2.level && !en3.level && p.core_.ready() && q.core_.ready(), "p and q on, r in standby");
  const uint32_t t0 = now_ms, cycles0 = p.core_.standby_cycles();
  DiffPowerPort::shut_down_ = false;
  DiffPowerPort::shutdown_all();
  CHECK(hp.well_spaced() && hq.well_spaced() && hr.asked.empty() && hp.at_shutdown[2] && hq.at_shutdown[2],
        "2 ports on: 3 copies each, out through the pump, well spaced; none for the one in standby");
  CHECK(!enable.level && !en2.level && !en3.level && now_ms - t0 < 400, "every enable low (%u ms)", now_ms - t0);
  CHECK(status.state == "Off" && p.core_.standby_cycles() == cycles0 + 1 && prefs.syncs == 1,
        "Off, the planned power-off counted, the preferences synced");
  DiffPowerPort::shutdown_all();
  CHECK(prefs.syncs == 1, "the hook runs once");

  // the shutdown 25 ms into a port's own notice: it goes on with its copies and its drop
  {
    now_ms = 100000;
    Pin en, det;
    det.level = true;
    DiffPowerPort n;
    n.enable_pin_ = &en;
    n.detect_pin_ = &det;
    HubModel hn;
    hn.attach(n, en);
    n.wake(120000);
    run({&n}, 3000, {&hn});
    while (hn.out.empty())
      run({&n}, 10, {&hn});
    run({&n}, 20, {&hn});   // NOTICE + 25 or so
    DiffPowerPort::ports_ = {&n};
    DiffPowerPort::shut_down_ = false;
    DiffPowerPort::shutdown_all();
    CHECK(hn.well_spaced() && en.changed_at - hn.asked[0] >= 200,
          "a shutdown during a notice: 3 copies, the enable low %u ms after it began",
          en.changed_at - hn.asked[0]);
  }
  // four slow transmitters (30 ms per pump): each port still 3 well spaced copies
  {
    now_ms = 200000;
    Pin en[4], det[4];
    DiffPowerPort ps[4];
    HubModel hs[4];
    std::vector<DiffPowerPort *> all;
    std::vector<HubModel *> hubs;
    for (int i = 0; i < 4; i++) {
      det[i].level = true;
      ps[i].enable_pin_ = &en[i];
      ps[i].detect_pin_ = &det[i];
      hs[i].attach(ps[i], en[i]);
      ps[i].wake(120000);
      all.push_back(&ps[i]);
      hubs.push_back(&hs[i]);
    }
    run(all, 3000, hubs);
    for (auto &h : hs)
      h.cost = 30;
    DiffPowerPort::ports_ = all;
    DiffPowerPort::shut_down_ = false;
    const uint32_t start = now_ms;
    DiffPowerPort::shutdown_all();
    bool ok = true;
    for (auto &h : hs)
      ok = ok && h.well_spaced();
    CHECK(ok && now_ms - start < 1100, "4 slow transmitters: every port 3 well spaced copies (%u ms in all)",
          now_ms - start);
  }
  // a long notice (1.5 s, the core's cap raised with it) through the shutdown: not cut short
  {
    now_ms = 250000;
    Pin en, det;
    det.level = true;
    DiffPowerPort l;
    PortConfig lc;
    lc.notice_ms = 1500;
    l.core_.configure(lc);
    l.enable_pin_ = &en;
    l.detect_pin_ = &det;
    HubModel hl;
    hl.attach(l, en);
    l.wake(120000);
    run({&l}, 3000, {&hl});
    DiffPowerPort::ports_ = {&l};
    DiffPowerPort::shut_down_ = false;
    DiffPowerPort::shutdown_all();
    CHECK(hl.out.size() == 3 && !en.level && en.changed_at - hl.asked[0] >= 1500,
          "a 1.5 s notice at the shutdown: the enable low %u ms after it began", en.changed_at - hl.asked[0]);
  }
  // nine minutes in standby, then a reboot: that time saved
  {
    now_ms = 300000;
    Pin en, det;
    det.level = true;
    DiffPowerPort s;
    s.enable_pin_ = &en;
    s.detect_pin_ = &det;
    run({&s}, 540000);
    DiffPowerPort::ports_ = {&s};
    DiffPowerPort::shut_down_ = false;
    DiffPowerPort::shutdown_all();
    CHECK(s.core_.standby_s() >= 539 && s.stats_pref_.v.standby_s == s.core_.standby_s(),
          "a reboot after 9 min of standby: %u s saved of %u", s.stats_pref_.v.standby_s, s.core_.standby_s());
  }
  DiffPowerPort::ports_.clear();
  // two ports with one name keep their counters apart (the key: their id)
  {
    stored.clear();
    Pin e1, d1, e2, d2;
    DiffPowerPort a, b;
    a.enable_pin_ = &e1;
    b.enable_pin_ = &e2;
    a.detect_pin_ = &d1;
    b.detect_pin_ = &d2;
    a.name_ = b.name_ = "Port";
    a.stats_key_ = "port_a";
    b.stats_key_ = "port_b";
    a.setup();
    b.setup();
    a.core_.set_counters(5, 50, 1);
    b.core_.set_counters(7, 70, 2);
    a.save_stats_();
    b.save_stats_();
    DiffPowerPort a2;
    a2.enable_pin_ = &e1;
    a2.detect_pin_ = &d1;
    a2.name_ = "Renamed";
    a2.stats_key_ = "port_a";
    a2.setup();
    CHECK(stored.size() == 2 && a2.core_.standby_cycles() == 5 && a2.core_.drops() == 1,
          "same name, two ids: two records; a new name keeps its id's counters");
    DiffPowerPort::ports_.clear();
  }
  // During dark stored, its dark source gone from the YAML: the YAML's mode, stored and shown
  {
    DiffPowerPort m;
    m.set_mode(MODE_STANDBY);
    PowerModeSelect sel;
    sel.port_ = &m;
    sel.pref_.have = true;
    sel.pref_.v = MODE_DURING_DARK;
    sel.setup();
    CHECK(m.get_mode() == MODE_STANDBY && sel.pref_.v == MODE_STANDBY && sel.published == 0,
          "During dark without a dark source: the YAML's Standby (mode %u, stored %u, shown %u)", m.get_mode(), sel.pref_.v,
          static_cast<unsigned>(sel.published));
    DiffPowerPort s2;
    s2.dark = true;
    PowerModeSelect sel2;
    sel2.port_ = &s2;
    sel2.pref_.have = true;
    sel2.pref_.v = MODE_DURING_DARK;
    sel2.setup();
    CHECK(s2.get_mode() == MODE_DURING_DARK && sel2.published == 1, "with a dark source: During dark restored");
  }

  // a port without a board: 'No board'; a light on: the enable high (a board plugged in gets power), a problem
  Pin en4, det4;
  LightState l4;
  TextSensor st4;
  BinarySensor pr4;
  DiffPowerPort n;
  n.enable_pin_ = &en4;
  n.detect_pin_ = &det4;
  n.status_ = &st4;
  n.problem_ = &pr4;
  n.add_light(&l4, 0);
  run({&n}, 500);
  CHECK(st4.state == "No board" && !pr4.state, "'%s', no problem", st4.state.c_str());
  l4.remote_values.on = true;
  run({&n}, 5000);
  CHECK(st4.state == "No board" && en4.level && pr4.state && l4.out.writes == 0,
        "a light on, no board: enabled, a problem, nothing sent");
  // without a detect pin (a sender without its Detect output): a board assumed, never 'No board'; a light on powers
  // the port, off: back to standby after its turn-off period
  Pin en5;
  LightState l5;
  TextSensor st5;
  BinarySensor pr5;
  DiffPowerPort w;
  w.enable_pin_ = &en5;
  w.status_ = &st5;
  w.problem_ = &pr5;
  w.add_light(&l5, 0);
  run({&w}, 500);
  const bool w_standby = st5.state == "Standby" && !en5.level;
  l5.remote_values.on = true;
  run({&w}, 5000);
  const bool w_on = st5.state == "On" && en5.level && l5.out.writes > 0;
  l5.remote_values.on = false;
  run({&w}, 11 * 60000);
  bool no_board = false;
  for (const auto &s : st5.history)
    no_board |= s == "No board";
  CHECK(w_standby && w_on && st5.state == "Standby" && !en5.level && !pr5.state && !no_board,
        "without a detect pin: Standby, a light on: On, off: Standby again; never 'No board' (%zu states, last '%s')",
        st5.history.size(), st5.state.c_str());
  // negative control: a light not on a port is never touched
  CHECK(other.out.writes == 0, "a light not on the port: untouched");
  // the power mode select without a dark source: Standby, Always on
  PowerModeSelect sel;
  sel.port_ = &p;
  sel.control(1);
  CHECK(p.get_mode() == MODE_ALWAYS_ON && sel.pref_.v == MODE_ALWAYS_ON && sel.published == 1 &&
            sel.index_of_(MODE_STANDBY) == 0 && sel.index_of_(MODE_ALWAYS_ON) == 1,
        "the select without a dark source: option 1 is Always on");
  // the power mode Off (round 12): option 2 without a dark source, on flash at once (a sync; other modes: the usual
  // flash_write_interval), restored at boot before the port's first update
  {
    Pin en6, det6;
    det6.level = true;
    LightState l6;
    TextSensor st6;
    DiffPowerPort o;
    o.enable_pin_ = &en6;
    o.detect_pin_ = &det6;
    o.status_ = &st6;
    o.add_light(&l6, 0);
    PowerModeSelect os;
    os.port_ = &o;
    l6.remote_values.on = true;
    run({&o}, 5000);
    const bool lit_on = en6.level && st6.state == "On";
    const int syncs = prefs.syncs;
    os.control(2);
    run({&o}, 3000);
    CHECK(lit_on && o.get_mode() == MODE_OFF && os.pref_.v == MODE_OFF && prefs.syncs == syncs + 1 &&
              os.index_of_(MODE_OFF) == 2 && !en6.level && st6.state == "Off",
          "Off chosen with a light lit: the enable low, 'Off' (%s), on flash now (%d syncs)", st6.state.c_str(),
          prefs.syncs - syncs);
    run({&o}, 30 * 60000);
    const bool stayed = !en6.level && st6.state == "Off";
    os.control(0);
    run({&o}, 5000);
    CHECK(stayed && prefs.syncs == syncs + 1 && en6.level && st6.state == "On",
          "Off for 30 min with the light on: never powered; Standby again (no sync): on ('%s')", st6.state.c_str());
    DiffPowerPort b;
    b.set_mode(MODE_ALWAYS_ON);
    PowerModeSelect bs;
    bs.port_ = &b;
    bs.pref_.have = true;
    bs.pref_.v = MODE_OFF;
    bs.setup();
    CHECK(b.get_mode() == MODE_OFF && bs.published == 2, "Off restored at boot over the YAML's Always on");
  }
  now_ms = 0;
  std::printf("  Diff-Power port: wake, a plain light sent again, the notice, the shutdown hook (2 + 1 ports, during a "
              "notice, slow transmitters, standby time), the counters per id, the select\n");
}

/// Before a planned power-off its hubs' last reads (an Ultra hub: the receivers' lifetime counters, a sync):
/// the port stays on (polls allowed) until every hub is ready, 3 s at most; then the notice, and
/// from its first copy to the cut the port isn't ready (no POLL). Wanted again meanwhile: no power-off.
static void test_port_prepare() {
  now_ms = 1000;
  Pin enable, detect;
  detect.level = true;
  LightState strip;
  DiffPowerPort p;
  PortConfig cfg;
  cfg.standby_after_ms = 60000;
  p.core_.configure(cfg);
  p.enable_pin_ = &enable;
  p.detect_pin_ = &detect;
  p.add_light(&strip, 0);
  HubModel hub;
  hub.attach(p, enable);
  struct Call {
    uint32_t ms;
    bool first, ready;
  };
  std::vector<Call> calls;
  uint32_t ready_at = 0;   // (0: never)
  p.add_on_prepare_callback([&](bool first) {
    calls.push_back(Call{now_ms, first, p.core_.ready()});
    return ready_at != 0 && now_ms >= ready_at;
  });
  strip.remote_values.on = strip.current_values.on = true;
  run({&p}, 3000, {&hub});
  // off: after the turn-off period its hub reads for 400 ms, then the notice
  strip.remote_values.on = strip.current_values.on = false;
  const uint32_t off = now_ms;
  run({&p}, 119000, {&hub});
  CHECK(calls.empty() && enable.level, "the turn-off period: no prepare yet");
  for (uint32_t end = now_ms + 5000; calls.empty() && static_cast<int32_t>(now_ms - end) < 0;)
    run({&p}, 10, {&hub});
  ready_at = calls.empty() ? 0 : calls[0].ms + 400;
  // (no POLL from the notice to the cut: the port not ready at any step from the first copy to the enable low)
  bool quiet = true;
  for (uint32_t end = now_ms + 3000; static_cast<int32_t>(now_ms - end) < 0;) {
    run({&p}, 10, {&hub});
    if (!hub.asked.empty() && enable.level)
      quiet = quiet && !p.core_.ready();
  }
  bool all_ready = true;
  for (const auto &c : calls)
    all_ready = all_ready && c.ready;
  CHECK(!calls.empty() && calls[0].first && calls[0].ms - off >= 120000 && calls.size() > 1 && !calls[1].first &&
            all_ready && hub.asked.size() == 3 && hub.asked[0] >= ready_at && hub.asked[0] - ready_at <= 20 &&
            hub.well_spaced(),
        "prepare: first once, the port ready (polls) throughout (%d), the notice when the hub is ready (%u ms after "
        "it, %u copies)",
        all_ready, hub.asked.empty() ? 0u : hub.asked[0] - ready_at, static_cast<unsigned>(hub.asked.size()));
  CHECK(quiet && !hub.asked.empty() && calls.back().ms <= hub.asked[0],
        "from the notice to the cut: not ready (no POLL: %d), no prepare call after it", quiet);
  // a hub never ready: the notice 3 s after the prepare began
  strip.remote_values.on = strip.current_values.on = true;
  run({&p}, 3000, {&hub});
  strip.remote_values.on = strip.current_values.on = false;
  calls.clear();
  hub.asked.clear();
  hub.out.clear();
  hub.index.clear();
  hub.enabled.clear();
  hub.at_shutdown.clear();
  ready_at = 0;
  run({&p}, 126000, {&hub});
  CHECK(!calls.empty() && calls[0].first && hub.asked.size() == 3 && hub.asked[0] - calls[0].ms >= 2980 &&
            hub.asked[0] - calls[0].ms <= 3020 && hub.well_spaced(),
        "a hub never ready: the notice %u ms after the prepare began", hub.asked.empty() ? 0u : hub.asked[0] - calls[0].ms);
  // wanted again during the prepare: no power-off; the next one prepares again (first again)
  strip.remote_values.on = strip.current_values.on = true;
  run({&p}, 3000, {&hub});
  strip.remote_values.on = strip.current_values.on = false;
  calls.clear();
  hub.asked.clear();
  hub.out.clear();
  hub.index.clear();
  hub.enabled.clear();
  hub.at_shutdown.clear();
  run({&p}, 120200, {&hub});
  const size_t before = calls.size();
  strip.remote_values.on = strip.current_values.on = true;
  run({&p}, 5000, {&hub});
  const bool on = enable.level && hub.asked.empty() && p.core_.ready();
  strip.remote_values.on = strip.current_values.on = false;
  ready_at = now_ms + 120100;
  run({&p}, 123000, {&hub});
  size_t firsts = 0;
  for (const auto &c : calls)
    firsts += c.first ? 1 : 0;
  CHECK(before > 0 && on && firsts == 2 && hub.asked.size() == 3 && hub.well_spaced(),
        "wanted again during the prepare: stays on (%d); the next power-off prepares again (%u firsts, %u copies)", on,
        static_cast<unsigned>(firsts), static_cast<unsigned>(hub.asked.size()));
  now_ms = 0;
  std::printf("  Diff-Power port: the hubs' last reads before the notice (ready / 3 s / wanted again), no POLL after it\n");
}

#undef USE_TEXT_SENSOR
#undef USE_BINARY_SENSOR
#undef USE_SENSOR
#undef USE_SELECT
}  // namespace port_power

// The hub on a Diff-Power port (dig2analog_ultra.cpp attach_port_, send_power_off_notice, pump_notice_,
// boards_powered_, has_power_supply, power_cycle, and transmit_ from hub_transmit.inc, on the real Scheduler): its
// lights (with their own turn-off periods) and its calibration / update holds as the port's loads; powered:
// everything again; each notice copy in the next frame for every board (IDENTIFY 7, one SEQ per notice, before what
// is queued and before a preamble), reported once out; the shutdown's pump never waits; a stale copy dropped at the
// next wake; a recovery's power cycle on the port (design review, round 8).
namespace hub_port {
#define USE_DIFFPOWER   // (USE_DIG2ANALOG_ULTRA_UPDATE: defined at the top)
struct rmt_transmit_config_t {
  uint32_t loop_count;
};
static std::vector<std::vector<uint8_t>> frames;   // what the RMT sent
static esp_err_t rmt_transmit(int, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  frames.emplace_back(data, data + n);
  return ESP_OK;
}
/// the RMT busy: the wait's timeout passes
static esp_err_t rmt_tx_wait_all_done(int, int timeout_ms) {
  if (rmt_idle)
    return ESP_OK;
  now_ms += static_cast<uint32_t>(timeout_ms);
  return -1;
}
struct LightState {};
struct Light {
  bool lit{false};
  uint32_t hold{0};
  LightState state;
  bool is_lit(uint32_t) const { return this->lit; }
  uint32_t get_standby_after() const { return this->hold; }
  LightState *get_state() { return &this->state; }
};
namespace diffpower {
struct DiffPowerPort {   // what the hub uses of it
  struct Load {
    uint32_t hold;
    std::function<bool(uint32_t)> lit;
    LightState *light;
  };
  std::vector<Load> loads;
  std::vector<std::function<void()>> ready, pump;
  std::vector<std::function<void(uint8_t)>> notice;
  bool is_ready_{false};
  int sent{0}, cycles{0};
  void add_load(uint32_t hold, std::function<bool(uint32_t)> &&lit, LightState *light = nullptr) {
    this->loads.push_back(Load{hold, std::move(lit), light});
  }
  void add_on_ready_callback(std::function<void()> &&f) { this->ready.push_back(std::move(f)); }
  void add_on_notice_callback(std::function<void(uint8_t)> &&f) { this->notice.push_back(std::move(f)); }
  void add_on_pump_callback(std::function<void()> &&f) { this->pump.push_back(std::move(f)); }
  bool is_ready() const { return this->is_ready_; }
  void notice_sent() { this->sent++; }
  void power_cycle() { this->cycles++; }
};
}  // namespace diffpower
struct UltraHub {
  enum UpdateState : uint8_t { UPD_IDLE = 0, UPD_RUNNING, UPD_FINISHED };
  enum PowerCycle : uint8_t { CYCLE_NONE = 0, CYCLE_WAIT_OFF, CYCLE_OFF, CYCLE_DONE, CYCLE_FAILED };
  UltraHub() {
    this->sched_.init(this->num_boards_, 5, 3);
    this->buf_.assign(this->sched_.frame_bytes(), 0);
    this->tx_buf_ = this->buf_.data();
  }
  void attach_port_();
  void send_power_off_notice(uint8_t copy);
  void pump_notice_();
  void service_notice_();
  bool boards_powered_() const;
  bool has_power_supply() const;
  void power_cycle();
  void transmit_();   // (hub_transmit.inc)
  bool line_free_();  // (hub_line.inc)
  bool line_out_();
  // stand-ins
  void resync(bool) { this->resyncs++; }
  void wake_() { this->wakes++; }
  bool is_failed() const { return false; }
  static int64_t now_us_() { return now_ms * 1000LL; }
  bool status_has_warning() const { return false; }
  void status_set_warning() {}
  void status_clear_warning() {}
  std::vector<Light *> lights_;
  diffpower::DiffPowerPort *port_{nullptr};
  bool port_attached_{false}, cal_power_hold_{false}, upd_power_hold_{false};
  uint32_t notices_{0};
  uint32_t notice_id_{0}, notice_epoch_{0};
  int64_t notice_end_us_{0};
  uint8_t pin_{16};
  int resyncs{0}, wakes{0};
  std::atomic<uint8_t> upd_state_{UPD_IDLE};
  PowerCycle cycle_{CYCLE_NONE};
  uint32_t cycle_ms_{0};
  // the line (transmit_)
  Scheduler sched_;
  uint16_t num_boards_{3};
  std::vector<uint8_t> buf_;
  uint8_t *tx_buf_{nullptr};
  bool tx_prepared_{false}, tx_normal_{true}, tx_preamble_end_{false};
  size_t tx_bytes_{0};
  uint16_t preamble_{0}, ping_left_{0};
  uint8_t dark_tail_{0};
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};
  bool preamble_full_{true}, synced_once_{false}, has_sent_{false};
  uint32_t last_frame_ms_{0}, frames_{0}, build_peak_us_{0};
  int channel_{0}, encoder_{0};
  // the commissioning (ultra_commission.h: tests/ultra_telemetry_test.cpp runs it)
  bool comm_on_{false};
  void service_commission_() {}
  Commissioning comm_;
};
#include "hub_port.inc"
#include "hub_line.inc"
#include "hub_transmit.inc"

/// The boards whose slot in frame f has IDENTIFY 7 (argument 0, CRC ok); seq: their SEQs
static int notice_slots(const std::vector<uint8_t> &f, uint16_t boards, std::vector<uint8_t> *seq = nullptr) {
  int n = 0;
  for (uint16_t b = 0; b < boards; b++) {
    const uint8_t *s = f.data() + ULTRA_SLOT_BYTES * b;
    if (s[0] == ULTRA_IDENTIFY && (s[1] >> 4) == ULTRA_LIGHT_BOARD && s[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF &&
        s[ULTRA_PAYLOAD + 1] == 0 && ultra_slot_check(s) == 0) {
      n++;
      if (seq != nullptr)
        seq->push_back(s[2]);
    }
  }
  return n;
}
/// a preamble frame: DARK slots (all zero)
static bool is_dark(const std::vector<uint8_t> &f) {
  return std::all_of(f.begin(), f.begin() + ULTRA_SLOT_BYTES, [](uint8_t v) { return v == 0; });
}

static void test_hub_port() {
  Light a, b;
  b.hold = 1800000;
  diffpower::DiffPowerPort port;
  UltraHub h;
  h.lights_ = {&a, &b};
  h.port_ = &port;
  h.attach_port_();
  CHECK(port.loads.size() == 3 && port.loads[0].hold == 0 && port.loads[1].hold == 1800000 &&
            port.loads[1].light == b.get_state() && port.loads[2].hold == 1,
        "the hub's lights (their own turn-off periods) and its holds: the port's loads");
  CHECK(!port.loads[0].lit(0) && !port.loads[2].lit(0), "nothing lit, nothing held");
  a.lit = true;
  h.upd_power_hold_ = true;
  CHECK(port.loads[0].lit(0) && !port.loads[1].lit(0) && port.loads[2].lit(0), "a light on, an update session held");
  CHECK(!h.boards_powered_(), "the port not ready: no polls, no calibration");
  port.is_ready_ = true;
  for (auto &f : port.ready)
    f();
  CHECK(h.boards_powered_() && h.resyncs == 1, "powered: everything sent again");
  // the notice: each copy asked (copy 0, 1, 2) in the next frame for every board, before board 1's queued IDENTIFY;
  // one SEQ per notice; each copy's frame reported out once
  h.sched_.identify(1, true);
  now_ms = 5000;
  frames.clear();
  std::vector<uint8_t> seqs;
  int with_notice = 0;
  for (uint8_t c = 0; c < 3; c++) {
    for (auto &f : port.notice)
      f(c);
    h.transmit_();
    with_notice += notice_slots(frames.back(), 3, &seqs);
  }
  h.transmit_();
  bool one_seq = seqs.size() == 9;
  for (size_t i = 0; one_seq && i < 9; i++)
    one_seq = seqs[i] == seqs[i % 3];
  const uint8_t *b1 = frames[3].data() + ULTRA_SLOT_BYTES;
  CHECK(h.notices_ == 1 && frames.size() == 4 && with_notice == 9 && one_seq && notice_slots(frames[3], 3) == 0 &&
            port.sent == 3,
        "3 copies: IDENTIFY 7 in every slot, one SEQ per board, counted once, each reported out once (%d)", port.sent);
  CHECK(b1[0] == ULTRA_IDENTIFY && b1[ULTRA_PAYLOAD] == ULTRA_IDENT_ON, "board 1's queued IDENTIFY after them");
  std::vector<uint8_t> seqs2;
  for (auto &f : port.notice)
    f(0);
  h.transmit_();
  notice_slots(frames.back(), 3, &seqs2);
  CHECK(seqs2.size() == 3 && seqs2[0] != seqs[0] && h.notices_ == 2, "the next notice: a new SEQ");
  // a copy goes ahead of a preamble (a resync's): the preamble waits one frame
  h.preamble_ = 2;
  for (auto &f : port.notice)
    f(1);
  frames.clear();
  h.transmit_();
  CHECK(notice_slots(frames[0], 3) == 3 && !is_dark(frames[0]) && h.preamble_ == 2, "a copy ahead of the preamble");
  h.transmit_();
  CHECK(is_dark(frames[1]) && h.preamble_ == 1, "then the preamble");
  h.preamble_ = 0;
  // the shutdown's pump: the RMT busy: nothing, and no waiting; free: the copy out and reported
  for (auto &f : port.notice)
    f(2);
  frames.clear();
  rmt_idle = false;
  const uint32_t t1 = now_ms;
  const int sent0 = port.sent;
  for (auto &f : port.pump)
    f();
  CHECK(frames.empty() && now_ms == t1 && port.sent == sent0, "the RMT busy: the pump returns at once, nothing sent");
  rmt_idle = true;
  for (auto &f : port.pump)
    f();
  CHECK(frames.size() == 1 && notice_slots(frames[0], 3) == 3 && port.sent == sent0 + 1, "free: the copy out, reported");
  for (auto &f : port.pump)
    f();
  CHECK(frames.size() == 1, "nothing queued: the pump sends nothing");
  // negative control: an update session owns the line
  for (auto &f : port.notice)
    f(0);
  h.upd_state_ = UltraHub::UPD_RUNNING;
  frames.clear();
  for (auto &f : port.pump)
    f();
  CHECK(frames.empty(), "an update session running: the pump leaves the line alone");
  h.upd_state_ = UltraHub::UPD_IDLE;
  // a copy not sent before the port woke again: dropped at the wake (a 0.2.6 board would hold dark)
  CHECK(h.sched_.power_off_left() == 3, "a copy queued");
  for (auto &f : port.ready)
    f();
  CHECK(h.sched_.power_off_left() == 0, "the wake: the stale copy dropped");
  // a recovery's power cycle on the port (design review, round 8)
  CHECK(h.has_power_supply(), "a port counts as a supply the hub switches");
  h.power_cycle();
  CHECK(port.cycles == 1 && h.cycle_ == UltraHub::CYCLE_OFF, "power_cycle(): the port's cycle");
  for (auto &f : port.ready)
    f();
  CHECK(h.cycle_ == UltraHub::CYCLE_DONE, "powered again: done");
  UltraHub plain;
  CHECK(!plain.has_power_supply(), "no port, no supply: none (negative control)");
  now_ms = 0;
  std::printf("  an Ultra line on a Diff-Power port: its loads, the replay, the notice's copies (ahead of a preamble, "
              "the shutdown's pump, stale ones dropped), a recovery's power cycle\n");
}

#undef USE_DIFFPOWER
}  // namespace hub_port

// ================================================================================================ ledger store
// The ESP32's lifetime ledger on its preferences (dig2analog_ultra.cpp: ledger_service_, ledger_save_, cut into
// build/lifecycle/hub_ledger.inc) on a stand-in for ESPHome's ESP32 backend (components/esp32/preferences.cpp): by
// key, a load only with the stored length, saves queued until sync(), which writes them in order and forgets the
// queue even when a write fails; a reboot loses the queue.
namespace ledger_store {
static std::map<uint32_t, std::vector<uint8_t>> stored, pending;
static std::vector<uint32_t> order;
static int saves = 0;
static bool fail_sync = false;
static int cut_after = -1;   // this many more writes (across syncs), then the power goes
static bool fail_blobs = false;   // a blob's write fails (no room); the 4-byte size records still go
struct ESPPreferenceObject {
  uint32_t key{0};
  bool load(uint8_t *out, size_t size) {
    auto &src = pending.count(this->key) ? pending : stored;
    auto it = src.find(this->key);
    if (it == src.end() || it->second.size() != size)
      return false;
    std::copy(it->second.begin(), it->second.end(), out);
    return true;
  }
  bool save(const uint8_t *in, size_t size) {
    if (!pending.count(this->key))
      order.push_back(this->key);
    pending[this->key] = std::vector<uint8_t>(in, in + size);
    saves++;
    return true;
  }
};
struct Preferences {
  ESPPreferenceObject make_preference(size_t, uint32_t key, bool) { return ESPPreferenceObject{key}; }
  bool sync() {   // (ESPHome's: every queued key in turn, on past a failed one; false if any failed)
    bool ok = !fail_sync;
    for (uint32_t k : order) {
      if (cut_after == 0)
        break;
      if (cut_after > 0)
        cut_after--;
      if (fail_blobs && pending[k].size() > 4)
        ok = false;
      else if (!fail_sync)
        stored[k] = pending[k];
    }
    pending.clear();
    order.clear();
    return ok;
  }
};
static Preferences prefs;
static Preferences *const global_preferences = &prefs;
static uint32_t fnv1_hash(const char *s) {
  uint32_t h = 2166136261u;
  for (; *s != 0; s++) {
    h *= 16777619u;
    h ^= static_cast<uint8_t>(*s);
  }
  return h;
}
struct UltraHub {
  void ledger_service_();
  static void ledger_save_(bool now);
};
#include "hub_ledger.inc"

/// the ESP32 restarts (its queue lost) with this many receivers counted
static void reboot(size_t receivers) {
  ultra_ledger() = LifetimeLedger{};
  ledger_receivers() = receivers;
  g_ledger_loaded = false;
  g_ledger_generation = g_ledger_saved_ms = g_ledger_day_ms = 0;
  now_ms = 0;
  pending.clear();
  order.clear();
}

/// Receivers added or removed across an allocation boundary (21 -> 22 receivers: 32 -> 64 entries; 32
/// -> 33: the positions too) keep the stored ledger; a cut between a copy and its size record keeps the other copy.
static void test_ledger_resize() {
  UltraHub h;
  uint8_t uid[12] = {0x42};
  bool made = false, swapped = false;
  stored.clear();
  reboot(21);
  h.ledger_service_();
  LedgerEntry *e = ultra_ledger().touch(uid, &made);
  ultra_ledger().add_energy(*e, 150000);
  ultra_ledger().position_energy(5u << 16, *e, &swapped);
  h.ledger_save_(true);
  const size_t small = ultra_ledger().blob_size();
  const size_t small_capacity = ultra_ledger().capacity();
  reboot(22);
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  const bool grown = e != nullptr && e->c.mwh == 150000 && ultra_ledger().capacity() > small_capacity &&
                     ultra_ledger().blob_size() == small;
  CHECK(grown, "21 -> 22 receivers (%u -> %u B, same occupied rows): the stored ledger kept (%llu mWh)",
        static_cast<unsigned>(small),
        static_cast<unsigned>(ultra_ledger().blob_size()), static_cast<unsigned long long>(e ? e->c.mwh : 0));
  if (e == nullptr)
    return;
  ultra_ledger().add_energy(*e, 10000);
  h.ledger_save_(true);
  reboot(21);   // (back: the newer copy is the bigger one)
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  CHECK(e != nullptr && e->c.mwh == 160000, "back to 21: the newer, bigger copy loads (%llu mWh)",
        static_cast<unsigned long long>(e ? e->c.mwh : 0));
  if (e == nullptr)
    return;
  reboot(33);   // (the positions over their boundary: 32 -> 64)
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  bool sw = false;
  CHECK(e != nullptr && ultra_ledger().position_energy(5u << 16, *e, &sw) == 160000 && !sw,
        "33 receivers: the position kept (no new board)");
  if (e == nullptr)
    return;
  // a cut between a copy (another size now) and its size record: that copy refused, the other one loads
  ultra_ledger().add_energy(*e, 10000);
  h.ledger_save_(true);   // (170 Wh, sizes recorded)
  reboot(70);
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  ultra_ledger().add_energy(*e, 10000);
  uint8_t extra_uid[12] = {0x43};
  ultra_ledger().touch(extra_uid, &made);   // compact size grows with occupancy, not reserved capacity
  cut_after = 1;          // (180 Wh: the copy written, its size record not)
  h.ledger_save_(true);
  cut_after = -1;
  reboot(70);
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  CHECK(e != nullptr && e->c.mwh == 170000, "a cut between a copy and its size: the other copy (%llu mWh)",
        static_cast<unsigned long long>(e ? e->c.mwh : 0));
  // (on the boot loader: the newer copy, without an entry it aged out, wins whole)
  stored.clear();
  reboot(1);
  h.ledger_service_();
  LedgerEntry *a = ultra_ledger().touch(uid, &made);
  ultra_ledger().add_energy(*a, 1000);
  h.ledger_save_(true);
  ultra_ledger().add_energy(*a, 1000);
  h.ledger_save_(true);
  ultra_ledger().set_day(LEDGER_AGE_DAYS);   // (aged out: no position holds it)
  h.ledger_save_(true);
  reboot(1);
  h.ledger_service_();
  CHECK(g_ledger_generation == 3 && ultra_ledger().find(uid) == nullptr,
        "the newer copy (generation %u) without the aged entry: it stays gone", g_ledger_generation);
  std::printf("  ledger store: receivers added and removed across the allocation boundaries, a cut mid-save, the newer "
              "copy whole\n");
}

/// A flush that fails (the NVS write) keeps the ledger dirty: a later forced save (a planned cut) with
/// nothing new writes it again, and so does the periodic save.
static void test_ledger_flush_failure() {
  UltraHub h;
  uint8_t uid[12] = {0x43};
  bool made = false;
  stored.clear();
  reboot(1);
  h.ledger_service_();
  ultra_ledger().touch(uid, &made);
  fail_sync = true;
  h.ledger_save_(true);   // (the flush fails)
  fail_sync = false;
  const bool kept = ultra_ledger().dirty();
  h.ledger_save_(true);   // a planned cut, nothing new: written again
  reboot(1);
  h.ledger_service_();
  CHECK(kept && ultra_ledger().find(uid) != nullptr, "a failed flush stays dirty (%d) and the next forced save "
        "writes it (%d)", kept, ultra_ledger().find(uid) != nullptr);
  // the periodic save retries too
  uint8_t uid2[12] = {0x44};
  ultra_ledger().touch(uid2, &made);
  fail_sync = true;
  now_ms = LEDGER_SAVE_MS + 1;
  h.ledger_service_();
  fail_sync = false;
  now_ms = 2 * LEDGER_SAVE_MS + 2;
  h.ledger_service_();
  reboot(1);
  h.ledger_service_();
  CHECK(ultra_ledger().find(uid2) != nullptr, "the periodic save after a failed one writes it again");
  // no memory for its stored form: nothing written, still dirty; the next save writes it
  uint8_t uid3[12] = {0x45};
  ultra_ledger().touch(uid3, &made);
  const int before = saves;
  fail_nothrow_new = true;
  h.ledger_save_(true);
  fail_nothrow_new = false;
  const bool waited = saves == before && ultra_ledger().dirty();
  h.ledger_save_(true);
  reboot(1);
  h.ledger_service_();
  CHECK(waited && ultra_ledger().find(uid3) != nullptr,
        "no memory for the stored form: nothing written, still dirty (%d); the next save writes it (%d)", waited,
        ultra_ledger().find(uid3) != nullptr);
  std::printf("  ledger store: a failed flush written again (forced and periodic), a save without memory too\n");
}

/// Design review, round 9: a resize whose blob writes fail (no room) while the size records would go: two valid
/// copies, then 21 -> 22 receivers (3,506 -> 5,938 B) and two failed saves. The copy in use stays whole and readable
/// (its generation and its board), the failed one never gets the new size; the next save that works writes it.
static void test_ledger_resize_failures() {
  UltraHub h;
  uint8_t uid[12] = {0x46};
  bool made = false;
  stored.clear();
  reboot(21);
  h.ledger_service_();
  LedgerEntry *e = ultra_ledger().touch(uid, &made);
  ultra_ledger().add_energy(*e, 150000);
  h.ledger_save_(true);
  ultra_ledger().add_power_up(*e);
  h.ledger_save_(true);   // (two valid copies: generation 2 in use)
  reboot(22);
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  if (e == nullptr) {
    CHECK(false, "resize failures: the stored ledger before them loads");
    return;
  }
  ultra_ledger().add_power_up(*e);
  fail_blobs = true;
  h.ledger_save_(true);
  h.ledger_save_(true);   // (both fail: the same copy tried again)
  fail_blobs = false;
  const bool dirty = ultra_ledger().dirty();
  reboot(22);
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  const uint32_t gen = g_ledger_generation;
  CHECK(dirty && e != nullptr && gen == 2,
        "two failed blob writes of a resize: still dirty (%d), the copy in use loads (board %d, generation %u)", dirty,
        e != nullptr, gen);
  if (e == nullptr)
    return;
  ultra_ledger().add_power_up(*e);
  h.ledger_save_(true);
  reboot(22);
  h.ledger_service_();
  e = ultra_ledger().find(uid);
  CHECK(e != nullptr && g_ledger_generation == 3 && e->c.ups == 2,
        "then a save that works: the new size, generation %u (power-ups %u)", g_ledger_generation, e ? e->c.ups : 0);
  std::printf("  ledger store: a resize whose blob writes fail keeps the copy in use whole\n");
}
}  // namespace ledger_store

// ================================================================================================ ledger line
// The hub's side of the ledger's own counts (dig2analog_ultra.cpp ledger_line_, cut into
// build/lifecycle/hub_ledger_line.inc; design review, round 9): a Diff-Power port's power-on and cut as the port timed
// them (PortCore::line_on / line_ms: the 55 V hotplug_ms after enable AND detect, until either drops), each receiver
// lit while the boards are ready, by its own lights only.
namespace ledger_line {
#define USE_DIFFPOWER
using ::esphome::diffpower::PortConfig;
using ::esphome::diffpower::PortCore;
struct Light {
  uint16_t board;
  bool on;
  uint16_t get_board() const { return this->board; }
  bool is_lit(uint32_t) const { return this->on; }
};
struct Keeper {   // what the hub tells the ledger
  bool lit_due(uint32_t) const { return true; }   // the differential ledger suite tests the real cadence
  std::vector<std::string> calls;
  std::vector<int> lit;
  void power_on(uint32_t ms) { this->calls.push_back("on " + std::to_string(ms)); }
  void cut(uint32_t ms) { this->calls.push_back("cut " + std::to_string(ms)); }
  const char *last() const { return this->calls.empty() ? "none" : this->calls.back().c_str(); }
  void lit_tick(uint16_t i, bool on, uint32_t) {
    if (this->lit.size() <= i)
      this->lit.resize(i + 1, 0);
    this->lit[i] = on;
  }
};
struct Port {
  PortCore c;
  const PortCore &core() const { return this->c; }
};
struct UltraHub {
  int *rw_uart_{nullptr};
  Port *port_{nullptr};
  Keeper keeper_;
  struct {
    uint16_t receivers() const { return 3; }
  } poller_;
  std::vector<Light *> lights_;
  bool ready{false};
  bool boards_powered_() const { return this->ready; }
  bool line_on_{false};
  std::vector<uint8_t> lit_rx_;
  void ledger_line_();
};
#include "hub_ledger_line.inc"

static void test_ledger_line() {
  int uart = 0;
  Port port;
  PortConfig cfg;
  port.c.configure(cfg);
  const uint16_t load = port.c.add_load(0);
  Light a{0, false}, b{2, false}, far{7, true};   // (board 7: no receiver there)
  UltraHub h;
  h.port_ = &port;
  h.lights_ = {&a, &b, &far};
  now_ms = 1000;
  port.c.set_detect(true, now_ms);
  port.c.update(now_ms);   // (standby)
  h.ledger_line_();
  CHECK(h.keeper_.calls.empty() && h.keeper_.lit.empty(), "ledger line: nothing without the return channel");
  h.rw_uart_ = &uart;
  // a light wants the port: enable at 2000; the 55 V from 2000 + hotplug_ms (seen at the next update)
  now_ms = 2000;
  a.on = true;
  port.c.set_lit(load, true, now_ms);
  port.c.update(now_ms);
  h.ledger_line_();
  now_ms = 2000 + cfg.hotplug_ms + 37;
  port.c.update(now_ms);
  h.ledger_line_();
  CHECK(h.keeper_.calls.size() == 1 && h.keeper_.calls[0] == "on " + std::to_string(2000 + cfg.hotplug_ms) &&
            h.keeper_.lit == std::vector<int>({0, 0, 0}),
        "ledger line: the power-on at the 55 V's start (%s), not lit before the boards are ready",
        h.keeper_.last());
  h.ready = true;
  b.on = true;
  h.ledger_line_();
  CHECK(h.keeper_.lit == std::vector<int>({1, 0, 1}), "ledger line: receivers 0 and 2 lit by their own lights");
  a.on = false;
  h.ledger_line_();
  CHECK(h.keeper_.lit == std::vector<int>({0, 0, 1}), "ledger line: receiver 0 dark again");
  // a dropout (the detect gone: the 55 V cut at once), then back
  now_ms = 9000;
  port.c.set_detect(false, now_ms);
  h.ready = false;
  now_ms = 9012;
  h.ledger_line_();
  CHECK(h.keeper_.calls.size() == 2 && h.keeper_.calls[1] == "cut 9000" && h.keeper_.lit == std::vector<int>({0, 0, 0}),
        "ledger line: the cut when the detect dropped (%s), nothing lit", h.keeper_.last());
  now_ms = 9500;
  port.c.set_detect(true, now_ms);
  port.c.update(now_ms);
  h.ledger_line_();
  now_ms = 9500 + cfg.hotplug_ms;
  port.c.update(now_ms);
  h.ledger_line_();
  CHECK(h.keeper_.calls.size() == 3 && h.keeper_.calls[2] == "on " + std::to_string(9500 + cfg.hotplug_ms),
        "ledger line: powered again from the detect's return + hotplug_ms (%s)", h.keeper_.last());
  // the power cycle (a recovery): the enable low now
  now_ms = 20000;
  port.c.power_cycle(now_ms);
  now_ms = 20005;
  h.ledger_line_();
  CHECK(h.keeper_.calls.size() == 4 && h.keeper_.calls[3] == "cut 20000", "ledger line: a power cycle's cut (%s)",
        h.keeper_.last());
  now_ms = 0;
  std::printf("  ledger line: the port's power-on and cut as it timed them, each receiver's lit time\n");
}
#undef USE_DIFFPOWER
}  // namespace ledger_line

}  // namespace esphome::dig2analog_ultra

using namespace esphome::dig2analog_ultra;

// ================================================================================================ auto_retry
// The update entity after a session's result (fw_update.cpp perform, start_session_, session_done_, abort and loop()'s
// SENDING / CHECKING): the session again for boards that missed it, blind on a one-way line, from the receivers'
// versions on a line of Diff-Power receivers (telemetry).
namespace update_retry {
#define USE_DIG2ANALOG_ULTRA_TELEMETRY
static const char *const BLINK_LEGEND = "Red blinks: ...";
static constexpr float DOWNLOAD_SHARE = 15.0f;
static constexpr uint32_t PUBLISH_MS = 1000;

/// The hub as the entity sees it: the result of each session (after its PING burst), the receivers' telemetry.
struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  enum PowerCycle : uint8_t { CYCLE_NONE = 0, CYCLE_WAIT_OFF, CYCLE_OFF, CYCLE_DONE, CYCLE_FAILED };
  uint16_t boards{3};
  bool with_telemetry{false};
  TelemetryPoller p;
  int sessions{0};
  bool forced{false}, aborted{false};
  UpdateResult result{UPD_NONE};
  bool start_update(const uint8_t *, size_t, const UpdateSession::Options &o) {
    this->sessions++;
    this->forced = o.force;
    return true;
  }
  UpdateResult take_update_result() {
    const UpdateResult r = this->result;
    this->result = UPD_NONE;
    return r;
  }
  bool update_running() const { return true; }
  float update_progress() const { return 0.5f; }
  uint8_t update_pass() const { return 0; }
  uint16_t get_num_boards() const { return this->boards; }
  bool has_telemetry() const { return this->with_telemetry; }
  const TelemetryPoller &telemetry() const { return this->p; }
  bool has_power_supply() const { return false; }
  bool port_off() const { return this->off; }
  bool off{false};   // (its Diff-Power port's power mode Off)
  void power_cycle() {}
  uint32_t power_cycle_ms() const { return 0; }
  PowerCycle power_cycle_state() const { return CYCLE_NONE; }
  void abort_update() { this->aborted = true; }
  void hold_power(bool) {}
};

struct UltraFirmwareUpdate {
  enum Phase : uint8_t { IDLE, QUEUED, DOWNLOADING, WAIT_POWER, SENDING, CHECKING };
  // the component's (build/lifecycle/update_retry_methods.inc)
  void perform(bool force);
  bool line_answers_() const;
  void start_install_();
  void start_session_();
  void session_done_();
  void abort();
  /// loop()'s SENDING and CHECKING steps (build/lifecycle/update_sending.inc)
  void step() {
    const uint32_t now = millis();
    switch (this->phase_) {
#include "update_sending.inc"
      default:
        break;
    }
  }
  // stand-ins
  std::string get_name() const { return "test"; }
  void set_status_(const std::string &s) {
    if (this->texts.empty() || this->texts.back() != s)
      this->texts.push_back(s);
  }
  void publish_progress_(float, bool) {}
  void finish_(const std::string &s, bool installed) {
    this->phase_ = IDLE;
    this->texts.push_back(s);
    this->installed = installed;
  }
  struct Info {
    std::string latest_version{"0.6.9"};
  } update_info_;
  std::vector<std::string> texts;
  bool installed{false};

  static UltraFirmwareUpdate *active_;
  UltraHub *hub_{nullptr};
  Phase phase_{IDLE};
  uint8_t state_{0};
  bool have_release_{true}, force_{false}, recover_{false}, power_cycle_{false}, off_stop_{false};
  uint8_t passes_{3}, auto_retry_{2};
  AutoRetry retry_;
  uint32_t installed_{0x000608FF}, latest_{0x000609FF}, job_version_{0}, lead_in_ms_{500}, phase_ms_{0};
  uint32_t last_publish_ms_{0};
  std::atomic<bool> dl_abort_{false};
  std::atomic<uint32_t> dl_bytes_{0};
  FwImageInfo image_, job_image_;
  std::string image_url_, job_url_;
  uint8_t *img_{nullptr};
  size_t img_size_{0};
};
UltraFirmwareUpdate *UltraFirmwareUpdate::active_ = nullptr;

#include "update_retry_methods.inc"

/// A receiver's answer to its POLL (the pages the check reads: IDENTITY, STATUS), as a receiver on `version` would.
static void answer(const uint8_t *s, uint16_t k, uint32_t version, std::vector<uint8_t> &out) {
  uint8_t a[32]{}, d[24]{};
  const uint8_t page = s[ULTRA_POLL_PAGE];
  if (page == ULTRA_PAGE_IDENTITY && s[ULTRA_POLL_ARG] == ULTRA_ID_PART_BOARD) {
    ultra_put32(d + ULTRA_ID_FW_VERSION, version);
    d[ULTRA_ID_BL_VERSION] = 2;
    d[ULTRA_ID_HW_ID] = UPD_HW_ID_D2A_ULTRA_5CH;
    d[ULTRA_ID_HW_REV] = UPD_REV_D2A_R04;
  } else if (page == ULTRA_PAGE_STATUS) {
    d[ULTRA_ST_HW_ID] = UPD_HW_ID_D2A_ULTRA_5CH;
    ultra_put24(d + ULTRA_ST_UPTIME, 100 + now_ms / 1000);
    d[ULTRA_ST_RESET] = ULTRA_ST_RST_POWER;
  }
  a[0] = ULTRA_RSP_SYNC;
  a[1] = page;
  a[2] = static_cast<uint8_t>(s[ULTRA_POLL_INDEX] << 4);
  a[3] = s[2];
  ultra_put16(a + 4, static_cast<uint16_t>(0x7700 + k));
  std::memcpy(a + ULTRA_RSP_DATA, d, 24);
  ultra_put16(a + 30, ultra_crc16(a, 30));
  out.push_back(ULTRA_RSP_LEAD);
  out.insert(out.end(), a, a + 32);
}

/// The receivers answer for ms (versions[k]; 0: it doesn't answer, nor the ones behind it), the entity steps.
static void run(UltraFirmwareUpdate &u, UltraHub &h, const uint32_t *versions, uint32_t ms) {
  const uint32_t end = now_ms + ms;
  while (now_ms < end) {
    if (h.with_telemetry && h.p.due(now_ms)) {
      std::vector<uint8_t> slots(32 * h.boards), uart;
      uint32_t w;
      const uint16_t n = h.p.build(now_ms, slots.data(), &w);
      for (uint16_t b = 0; b < n && versions[b] != 0; b++) {
        if (slots[32 * b + ULTRA_POLL_INDEX] != ULTRA_POLL_RELAY_ONLY)
          answer(slots.data() + 32 * b, b, versions[b], uart);
      }
      now_ms += 10;
      h.p.finish(now_ms, uart.data(), uart.size());
    } else {
      now_ms += 50;
    }
    u.step();
  }
}

/// A session's result comes in; the entity steps once.
static void result(UltraFirmwareUpdate &u, UltraHub &h, UltraHub::UpdateResult r) {
  h.result = r;
  now_ms += 3000;
  u.step();
}

static void test_auto_retry() {
  const uint32_t V = 0x000609FF, OLD = 0x000608FF;
  // a one-way line: every retry sent (the hub can't see), texts, then done
  {
    now_ms = 1000;
    UltraHub h;
    UltraFirmwareUpdate u;
    u.hub_ = &h;
    u.perform(false);
    CHECK(u.phase_ == UltraFirmwareUpdate::QUEUED && u.retry_.retries() == 2 && !u.retry_.verify(),
          "a one-way line: 2 retries, blind");
    u.start_session_();
    result(u, h, UltraHub::UPD_DONE);
    CHECK(h.sessions == 2 && u.phase_ == UltraFirmwareUpdate::SENDING &&
              u.texts.back() == "retry 1 of 2 for boards that may have missed the update",
          "blind: the session again (%d), '%s'", h.sessions, u.texts.back().c_str());
    now_ms += 2000;
    u.step();
    CHECK(u.texts.back() == "retry 1 of 2: sending: pass 1 of 3, 50 %", "its progress: '%s'", u.texts.back().c_str());
    result(u, h, UltraHub::UPD_DONE);
    result(u, h, UltraHub::UPD_DONE);
    CHECK(h.sessions == 3 && u.phase_ == UltraFirmwareUpdate::IDLE && u.installed && !h.forced &&
              u.texts.back() == std::string("done: 0.6.9 sent to the 3 boards of this line in 3 sessions. ") +
                                    BLINK_LEGEND,
          "blind: 3 sessions, never forced, then done: '%s'", u.texts.back().c_str());
  }
  // its Diff-Power port Off (round 12): no install, no recovery (the port would never power the boards)
  {
    UltraHub h;
    h.off = true;
    UltraFirmwareUpdate u;
    u.hub_ = &h;
    u.perform(false);
    const bool no_install = u.phase_ == UltraFirmwareUpdate::IDLE && h.sessions == 0 && !u.texts.empty() &&
                            u.texts.back() == "failed: its Diff-Power port's power mode is Off";
    u.perform(true);
    h.off = false;   // negative control: the same install once the port isn't Off
    u.perform(false);
    CHECK(no_install && u.phase_ == UltraFirmwareUpdate::QUEUED, "port Off: no install ('%s'); not Off: queued",
          u.texts.empty() ? "" : u.texts.back().c_str());
  }
  // force_update / recover: no retry
  {
    UltraHub h;
    UltraFirmwareUpdate u;
    u.hub_ = &h;
    u.perform(true);
    u.start_session_();
    result(u, h, UltraHub::UPD_DONE);
    CHECK(h.sessions == 1 && h.forced && u.phase_ == UltraFirmwareUpdate::IDLE &&
              u.texts.back().rfind("done: 0.6.9 sent to the 3 boards of this line. ", 0) == 0,
          "force: one session, no retry ('%s')", u.texts.back().c_str());
  }
  // an abort the user asked for: no retry, also not while the receivers are checked
  {
    UltraHub h;
    UltraFirmwareUpdate u;
    u.hub_ = &h;
    u.perform(false);
    u.start_session_();
    u.abort();
    result(u, h, UltraHub::UPD_ABORTED);
    CHECK(h.aborted && h.sessions == 1 && u.phase_ == UltraFirmwareUpdate::IDLE && !u.installed &&
              u.texts.back().rfind("aborted", 0) == 0,
          "abort while sending: no retry ('%s')", u.texts.back().c_str());
    UltraHub t;
    t.with_telemetry = true;
    t.p.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    UltraFirmwareUpdate v;
    v.hub_ = &t;
    v.perform(false);
    v.start_session_();
    result(v, t, UltraHub::UPD_DONE);
    CHECK(v.phase_ == UltraFirmwareUpdate::CHECKING, "telemetry: the receivers are checked");
    v.abort();
    CHECK(t.sessions == 1 && v.phase_ == UltraFirmwareUpdate::IDLE && v.texts.back().rfind("aborted", 0) == 0,
          "abort while checking: no retry ('%s')", v.texts.back().c_str());
  }
  // a line of 3 receivers: all on the release -> done at once; one in its bootloader -> retries while it doesn't answer
  {
    now_ms = 100000;
    UltraHub h;
    h.with_telemetry = true;
    h.p.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    h.p.locked(now_ms);
    const uint32_t before[3] = {OLD, OLD, OLD}, after[3] = {V, V, V};
    UltraFirmwareUpdate u;
    u.hub_ = &h;
    u.job_image_.hw_id = UPD_HW_ID_D2A_ULTRA_5CH;
    u.image_ = u.job_image_;
    u.image_.hw_rev_mask = 1u << UPD_REV_D2A_R04;
    u.image_.min_bootloader = 2;
    run(u, h, before, 3000);
    u.perform(false);
    CHECK(u.retry_.verify(), "every board answers: the retries check their versions");
    u.start_session_();
    result(u, h, UltraHub::UPD_DONE);
    CHECK(u.phase_ == UltraFirmwareUpdate::CHECKING && u.texts.back() == "installed; checking the 3 receivers",
          "'%s'", u.texts.back().c_str());
    // (each restarted with the release: uptime from 0, IDENTITY again)
    h.p = TelemetryPoller{};
    h.p.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    h.p.locked(now_ms);
    run(u, h, after, 5000);
    CHECK(h.sessions == 1 && u.phase_ == UltraFirmwareUpdate::IDLE && u.installed &&
              u.texts.back() == "done: all 3 receivers on 0.6.9",
          "all on the release: no retry, '%s'", u.texts.back().c_str());

    UltraFirmwareUpdate w;
    w.hub_ = &h;
    w.image_ = u.image_;
    h.p = TelemetryPoller{};   // (a line on the old version again)
    h.p.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    h.p.locked(now_ms);
    run(w, h, before, 3000);
    w.perform(false);
    w.start_session_();
    result(w, h, UltraHub::UPD_DONE);
    // receiver 1 restarted with the release; receiver 2 waits in its bootloader (and silences receiver 3)
    const uint32_t stuck[3] = {V, 0, 0};
    h.p = TelemetryPoller{};
    h.p.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    h.p.locked(now_ms);
    run(w, h, stuck, AutoRetry::CHECK_MS + 2000);
    const bool said = std::find(w.texts.begin(), w.texts.end(), "retry 1 of 2: 2 receivers don't answer") !=
                      w.texts.end();
    CHECK(h.sessions == 3 && w.phase_ == UltraFirmwareUpdate::SENDING && said &&
              w.texts.back() == "retry 1 of 2: sending: pass 1 of 3, 50 %",
          "receiver 2 in its bootloader: retry 1 (%d), why said %d, then '%s'", h.sessions, said,
          w.texts.back().c_str());
    result(w, h, UltraHub::UPD_DONE);
    run(w, h, after, 5000);
    CHECK(h.sessions == 3 && w.phase_ == UltraFirmwareUpdate::IDLE && w.installed && w.retry_.used() == 1 &&
              w.texts.back() == "done: all 3 receivers on 0.6.9",
          "the retry installed it: no second retry, '%s'", w.texts.back().c_str());
  }
  now_ms = 0;
}

#undef USE_DIG2ANALOG_ULTRA_TELEMETRY
}  // namespace update_retry

// The commissioning's hub side (round 12; build/lifecycle/hub_commission.inc): one line of the controller at a time,
// the port held powered, the test level on every light of the line as a layer (and off it again), the calibration,
// the record stored. The run itself: tests/ultra_telemetry_test.cpp.
namespace hub_commission {
namespace diffpower = ::esphome::diffpower;
#define USE_DIG2ANALOG_ULTRA_TELEMETRY
#define USE_DIFFPOWER
struct UltraLight;
struct UltraLayer {
  virtual void layer_shown(UltraLight *light) = 0;
};
struct UltraLight {   // what the commissioning uses of it: a layer on top, its test level
  std::vector<UltraLayer *> layers;
  std::vector<float> levels;   // each test level sent
  int ends{0};
  void layer_begin(UltraLayer *l) { this->layers.push_back(l); }
  void layer_end(UltraLayer *l, uint32_t) {
    for (auto it = this->layers.begin(); it != this->layers.end(); ++it) {
      if (*it == l) {
        this->layers.erase(it);
        this->ends++;
        return;
      }
    }
  }
  bool layer_top(const UltraLayer *l) const { return !this->layers.empty() && this->layers.back() == l; }
  void send_test_level(float level) { this->levels.push_back(level); }
};
struct PortCoreView {
  bool board_{true};
  bool board() const { return this->board_; }
};
struct DiffPowerPort {
  bool detect{true}, off{false};
  PortCoreView c;
  diffpower::PortState st{diffpower::PS_ON};
  bool has_detect() const { return this->detect; }
  bool is_off() const { return this->off; }
  const PortCoreView &core() const { return this->c; }
  diffpower::PortState state() const { return this->st; }
};
struct Prefs {
  int syncs{0};
  bool sync() {
    this->syncs++;
    return true;
  }
};
static Prefs prefs;
static Prefs *const global_preferences = &prefs;
struct UltraHub {
  // the hub's (build/lifecycle/hub_commission.inc)
  void commission(bool finalize);
  static void commission_all(bool finalize);
  void service_commission_();
  void commission_level_(float level);
  struct CommissionLayer final : public UltraLayer {
    UltraHub *hub{nullptr};
    void layer_shown(UltraLight *light) override;
  };
  // stand-ins
  uint8_t pin_{16};
  std::vector<UltraLight *> lights_;
  DiffPowerPort *port_{nullptr};
  TelemetryPoller poller_;
  PortMonitor port_monitor_;
  Commissioning comm_;
  CommissionLayer comm_layer_;
  bool comm_on_{true}, powered{true}, cal_running{false};
  uint32_t cal_until{0};
  int cal_calls{0}, texts{0};
  struct {
    int saves{0};
    CommissionRecord last;
    bool save(const CommissionRecord *r) {
      this->saves++;
      this->last = *r;
      return true;
    }
  } comm_pref_;
  struct {
    UltraHub *hub;
    void call() { this->hub->texts++; }
  } comm_cb_{this};
  static UltraHub *comm_active_;
  static std::vector<UltraHub *> hubs_;
  bool boards_powered_() const { return this->powered; }
  bool calibration_running() const { return this->cal_running; }
  bool update_running() const { return false; }
  void calibrate_boards(const std::vector<uint16_t> &, uint8_t) {
    this->cal_calls++;
    this->cal_running = true;
    this->cal_until = now_ms + 3000;
  }
  UltraHub() {
    this->poller_.init(0, ULTRA_RW_RATE_1M, TelemetryIntervals{});   // (no receivers: the run's steps only)
    this->comm_.configure(1000, 0.9f);
    this->comm_layer_.hub = this;
  }
  void tick() {
    if (this->cal_running && static_cast<int32_t>(now_ms - this->cal_until) >= 0)
      this->cal_running = false;
    this->service_commission_();
  }
};
UltraHub *UltraHub::comm_active_ = nullptr;
std::vector<UltraHub *> UltraHub::hubs_;
#include "hub_commission.inc"

static void test_commission_glue() {
  now_ms = 1000;
  UltraLight a1, a2, b1;
  DiffPowerPort pa, pb;
  UltraHub a, b;
  a.pin_ = 16;
  b.pin_ = 17;
  a.lights_ = {&a1, &a2};
  b.lights_ = {&b1};
  a.port_ = &pa;
  b.port_ = &pb;
  UltraHub::hubs_ = {&a, &b};
  UltraHub::commission_all(false);
  bool both = false, a_first = false, held = false;
  float a_max = 0.0f;
  for (int k = 0; k < 20000 && (a.comm_.phase() != Commissioning::IDLE || b.comm_.phase() != Commissioning::IDLE);
       k++) {
    now_ms += 5;
    a.tick();
    b.tick();
    both |= a.comm_.running() && b.comm_.running();
    a_first |= a.comm_.running() && b.comm_.queued();
    held |= a.comm_.hold_power();
    if (!a1.levels.empty())
      a_max = std::max(a_max, a1.levels.back());
  }
  CHECK(!both && a_first && held && a.comm_.status() == "Commission passed" && b.comm_.status() == "Commission passed" &&
            UltraHub::comm_active_ == nullptr,
        "two lines: one at a time, the port held; both passed ('%s', '%s')", a.comm_.status().c_str(),
        b.comm_.status().c_str());
  CHECK(a1.levels.size() == Commissioning::LEVELS && a2.levels.size() == Commissioning::LEVELS && a_max == 1.0f &&
            b1.levels.size() == Commissioning::LEVELS && a1.layers.empty() && a2.layers.empty() && b1.layers.empty() &&
            a1.ends == 1 && b1.ends == 1 && a.texts > 0,
        "the test levels on every light of the line as a layer (%zu levels, up to %.2f), off again after (%d)",
        a1.levels.size(), a_max, a1.ends);
  // a strip shown over a light during the test, ended: the test level again on top (layer_shown)
  UltraLight s;
  a.lights_ = {&s};
  a.commission(false);
  for (int k = 0; k < 600 && a.comm_.level() == 0.0f; k++) {
    now_ms += 5;
    a.tick();
  }
  const size_t sent = s.levels.size();
  a.comm_layer_.layer_shown(&s);
  CHECK(s.levels.size() == sent + 1 && s.levels.back() == a.comm_.level(), "a layer above it ended: the test level again");
  for (int k = 0; k < 20000 && a.comm_.phase() != Commissioning::IDLE; k++) {
    now_ms += 5;
    a.tick();
  }
  // finalize: the calibration asked for, the record stored (and synced); the port Off: refused at once
  const int syncs = prefs.syncs;
  a.commission(true);
  for (int k = 0; k < 40000 && a.comm_.phase() != Commissioning::IDLE; k++) {
    now_ms += 5;
    a.tick();
  }
  pa.off = true;
  a.commission(false);
  for (int k = 0; k < 10 && a.comm_.phase() != Commissioning::IDLE; k++) {
    now_ms += 5;
    a.tick();
  }
  CHECK(a.cal_calls == 1 && a.comm_pref_.saves == 1 && a.comm_pref_.last.passed && prefs.syncs == syncs + 1 &&
            a.comm_.status() == "Commission failed: The port's power mode is Off",
        "finalize: calibrated %d, stored %d (passed %d), synced; then Off: '%s'", a.cal_calls, a.comm_pref_.saves,
        a.comm_pref_.last.passed, a.comm_.status().c_str());
  UltraHub::hubs_.clear();
  now_ms = 0;
}
#undef USE_DIG2ANALOG_ULTRA_TELEMETRY
#undef USE_DIFFPOWER
}  // namespace hub_commission

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
  hub_update::test_update_shutdown();
  hub_update::test_update_lead_in();
  hub_rw::test_telemetry_window();
  hub_rw::test_rw_fast_share();
  hub_rw::test_rw_no_standby();
  hub_rw::test_rw_reply_rate();
  hub_rw::test_several_lines();
  hub_rw::test_tied_shared_uart();
  hub_rw::test_standby_support();
  hub_rw::test_wake_settling();
  hub_rw::test_lock_from_end();
  receiver_entities::test_receiver_entities();
  receiver_entities::test_receiver_dispatch(50, 2000);
  receiver_entities::test_receiver_dispatch(2, 2000);   // (a P4 port: 2 receivers a line)
  update_retry::test_auto_retry();
  ledger_store::test_ledger_resize();
  ledger_store::test_ledger_flush_failure();
  ledger_store::test_ledger_resize_failures();
  ledger_line::test_ledger_line();
  port_power::test_port();
  port_power::test_detect_rate();
  port_power::test_port_prepare();
  hub_port::test_hub_port();
  hub_commission::test_commission_glue();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA LIFECYCLE TESTS PASSED", failures);
  return failures ? 1 : 0;
}
