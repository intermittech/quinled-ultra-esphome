// Host test of the component's methods that need ESPHome / ESP-IDF, so tests/ultra_host_test.cpp can't compile them:
// tests/run_ultra_lifecycle_test.py cuts their bodies unchanged from components/dig2analog_ultra into build/lifecycle/
// *.inc, and this file compiles them with small stand-ins for ESPHome, the RMT driver and the HTTP client. Covers the
// light's setup, a transition cut short, its power request after weeks; the update's install queue, chunked downloads
// and the confirmation frames after a session; a frame the RMT refuses. Logic only: no ESP-IDF, no timing.
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
#include <string>
#include <vector>

#define USE_DIG2ANALOG_ULTRA_UPDATE
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
  float get_gamma_correct() const { return 1.0f; }
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
  void lit_(uint16_t m, uint32_t length_ms);
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
  l.lit_(65535, 0);
  l.lit_(0, 100);
  CHECK(l.is_lit(now_ms + 50) && !l.is_lit(now_ms + 131) && !l.is_lit(now_ms + 0x80000100u),
        "fading out: lit until its end, across the wrap, and not again weeks later");
  now_ms = 0;
}

}  // namespace lights

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

}  // namespace esphome::dig2analog_ultra

using namespace esphome::dig2analog_ultra;

int main() {
  lights::test_light();
  updates::test_install_queue();
  updates::test_fetch();
  hub_loop::test_update_confirmation();
  hub_transmit::test_transmit_retry();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA LIFECYCLE TESTS PASSED", failures);
  return failures ? 1 : 0;
}
