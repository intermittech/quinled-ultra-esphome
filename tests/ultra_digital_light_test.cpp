// Host test of the ultra_digital light's own code (components/ultra_digital/ultra_digital.cpp): tests/run_ultra_digital_test.py
// cuts the PixelLight / PixelFxEffect methods that need ESPHome unchanged into build/digital/light_methods.inc, and this
// file compiles them with small stand-ins for ESPHome's light, the hub and the page 0x81 poller. Covers the boot state, an
// effect change (a new anchor, the crossfade), a change of speed / palette / colour 2 (the same anchor), off and on with
// the transition, an effect the receiver lacks (Solid), the hand-back to a streaming light and its exclusivity both ways,
// STORE, the 20 day anchor, the lit time for the power, and page 0x81's news. The pure parts are in ultra_digital_test.cpp.

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/ultra_pixel_fx.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#define USE_DIG2ANALOG_ULTRA_TELEMETRY
#define ESP_LOGE(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGV(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
static std::vector<std::string> warnings;
#define ESP_LOGW(tag, ...) log_warning(__VA_ARGS__)
static void log_warning(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  warnings.push_back(buf);
}

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      failures++;                                         \
      std::printf("  FAIL %s:%d: ", __FILE__, __LINE__);  \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

static uint32_t now_ms = 0;
static uint32_t millis() { return now_ms; }

namespace esphome {
namespace dig2analog_ultra {
// the hub as the light sees it
struct FakeTelemetry {
  PixelState ps;
  ReceiverData rd;
  uint16_t receivers() const { return 1; }
  const PixelState &pixel(uint16_t) const { return this->ps; }
  const ReceiverData &data(uint16_t) const { return this->rd; }
};
struct FakeHub {
  Scheduler sched;
  FakeTelemetry tel;
  bool telemetry_on{true};
  std::vector<uint16_t> pixel_boards;
  std::vector<std::pair<uint8_t, uint16_t>> configs;
  std::vector<std::function<void(uint32_t)>> callbacks;
  void *pixel_light{nullptr};
  uint32_t wakes{0};
  FakeHub() { this->sched.init(2, 99, 3); }
  Scheduler &scheduler() { return this->sched; }
  void wake_() { this->wakes++; }
  int64_t start_time_us(uint16_t) const { return now_ms * 1000LL + 40000; }   // the lead: 40 ms
  void set_pixel_config(uint16_t, uint8_t id, uint16_t v) { this->configs.push_back({id, v}); }
  uint32_t flushes{0};
  void pixel_config_flush() { this->flushes++; }
  template<typename T> void add_pixel_light(T *l) { this->pixel_light = l; }
  bool has_telemetry() const { return this->telemetry_on; }
  const FakeTelemetry &telemetry() const { return this->tel; }
  void add_on_receiver_callback(uint16_t, std::function<void(uint32_t)> &&cb) { this->callbacks.push_back(std::move(cb)); }
};
}  // namespace dig2analog_ultra

namespace light {
struct LightOutput {};
struct LightRemoteValuesListener {
  virtual ~LightRemoteValuesListener() = default;
  virtual void on_light_remote_values_update() {}
};
struct LightColorValues {
  bool on{false};
  float brightness{1.0f}, color_brightness{1.0f}, red{1}, green{1}, blue{1};
  bool is_on() const { return this->on; }
  float get_brightness() const { return this->brightness; }
  float get_color_brightness() const { return this->color_brightness; }
  float get_red() const { return this->red; }
  float get_green() const { return this->green; }
  float get_blue() const { return this->blue; }
};
struct LightState;
struct Call {
  LightState *s;
  bool has_state{false}, state{false};
  uint32_t transition{0};
  Call &set_state(bool v) {
    this->has_state = true;
    this->state = v;
    return *this;
  }
  Call &set_transition_length(uint32_t ms) {
    this->transition = ms;
    return *this;
  }
  void perform();
};
struct LightState {
  std::string name;
  LightColorValues remote_values, current_values;
  LightOutput *output{nullptr};
  uint32_t default_transition{1000};
  std::vector<LightRemoteValuesListener *> listeners;
  std::vector<bool> calls;   // every set_state(...) performed on it
  const std::string &get_name() const { return this->name; }
  uint32_t get_default_transition_length() const { return this->default_transition; }
  LightOutput *get_output() const { return this->output; }
  void add_remote_values_listener(LightRemoteValuesListener *l) { this->listeners.push_back(l); }
  Call make_call() { return Call{this}; }
  /// the light's remote values changed (ESPHome calls the listeners after a call is applied)
  void set_remote(bool on, float brightness = 1.0f) {
    this->remote_values.on = on;
    this->remote_values.brightness = brightness;
    this->current_values = this->remote_values;
    for (auto *l : this->listeners)
      l->on_light_remote_values_update();
  }
};
void Call::perform() {
  if (this->has_state) {
    this->s->calls.push_back(this->state);
    this->s->set_remote(this->state);
  }
}
}  // namespace light

namespace ultra_digital {
using dig2analog_ultra::FakeHub;
using dig2analog_ultra::PixelFxTracker;
using dig2analog_ultra::PixelLightState;
using dig2analog_ultra::PixelState;
using UltraHub = dig2analog_ultra::FakeHub;

// as ultra_digital.cpp's constants
static constexpr uint32_t PARAM_RAMP_MS = 250;

enum PixelParam : uint8_t { PARAM_SPEED = 0, PARAM_INTENSITY, PARAM_CUSTOM1, PARAM_CUSTOM2, PARAM_COUNT };
class PixelLight;

class PixelFxEffect {
 public:
  PixelFxEffect(const char *name, uint8_t id) : name_(name), id_(id) {}
  void start();
  void stop();
  uint8_t id() const { return this->id_; }
  const char *name_;
  light::LightState *state_{nullptr};
  uint8_t id_;
  PixelLight *light_() const;
  const std::string get_name() const { return this->name_; }
};

class ExclusiveWatcher final : public light::LightRemoteValuesListener {
 public:
  ExclusiveWatcher(PixelLight *owner, light::LightState *other) : owner_(owner), other_(other) {}
  void on_light_remote_values_update() override;
  PixelLight *owner_;
  light::LightState *other_;
};

class PixelLight final : public light::LightOutput, public light::LightRemoteValuesListener {
 public:
  // the component's (build/digital/light_methods.inc)
  void setup();
  void loop();
  void on_light_remote_values_update() override;
  void write_state(light::LightState *state);
  void set_param(uint8_t which, uint8_t value);
  uint8_t get_param(uint8_t which) const;
  void set_palette_index(size_t index);
  void set_color2(const uint8_t *rgb);
  void set_color3(const uint8_t *rgb);
  void effect_started(PixelFxEffect *effect);
  void effect_stopped(PixelFxEffect *effect);
  void transition_requested(uint32_t length_ms);
  static PixelLight *find(light::LightOutput *output);
  void other_changed_(light::LightState *other);
  void release_();
  void queue_fx_(const PixelFxTracker::Out &o);
  uint32_t strip_frame_us_{0};
  void send_();
  uint8_t shown_effect_();
  void pixel_page_();
  // stand-ins
  void request_send_() {
    this->dirty_ = true;
    this->hub_->wake_();
  }
  bool pixel_lit(uint32_t now_ms) { return this->track_.lit(now_ms); }
  uint16_t palette_id_() const {
    return this->palette_index_ < this->palette_ids_.size() ? this->palette_ids_[this->palette_index_]
                                                            : static_cast<uint16_t>(ULTRA_PX_PAL_DEFAULT);
  }
  static void set_rgb_(uint8_t *to, const uint8_t *from) {
    for (int i = 0; i < 3; i++)
      to[i] = from[i];
  }
  struct Cfg {
    uint8_t id;
    uint16_t value;
  };

  UltraHub *hub_{nullptr};
  light::LightState *state_{nullptr};
  void *transition_{nullptr};
  uint16_t board_{0};
  uint8_t segment_{0};
  uint8_t solid_{1};
  uint8_t effect_{1};
  PixelFxEffect *chosen_{nullptr};
  std::vector<Cfg> cfg_;
  std::vector<uint8_t> palette_ids_;
  size_t palette_index_{0};
  std::vector<PixelFxEffect *> effects_;
  std::vector<light::LightState *> others_;
  PixelLightState state_in_;
  uint32_t standby_after_ms_{0};
  uint32_t store_after_ms_{0};
  PixelFxTracker track_;
  bool dirty_{false};
  bool effect_changed_{false};
  bool restart_{false};
  bool warned_missing_{false}, warned_cap_{false};
  uint32_t transition_ms_{0};
  bool first_loop_{true};
  bool page_hooked_{false};
  std::string status_last_, missing_logged_;
  static std::vector<PixelLight *> registry_;
};
std::vector<PixelLight *> PixelLight::registry_;

#include "light_methods.inc"
}  // namespace ultra_digital
}  // namespace esphome

using namespace esphome;
using ultra_digital::PixelFxEffect;
using ultra_digital::PixelLight;

static const uint8_t *slot_of(const std::vector<uint8_t> &f, int i) { return f.data() + ULTRA_SLOT_BYTES * i; }

/// everything the hub would send for board 0 so far: the PIXEL_FX frames, in order
struct Wire {
  std::vector<std::vector<uint8_t>> fx;   // every PIXEL_FX slot, new SEQ only
  uint8_t last_seq{0};
  bool any{false};
  void drain(dig2analog_ultra::FakeHub &hub, int frames = 6) {
    std::vector<uint8_t> f(hub.sched.frame_bytes());
    for (int i = 0; i < frames; i++) {
      hub.sched.build_frame(now_ms * 1000LL + i * 3000, f.data());
      const uint8_t *s = slot_of(f, 0);
      if (s[0] == ULTRA_PIXEL_FX && (!any || s[2] != last_seq)) {
        fx.push_back(std::vector<uint8_t>(s, s + ULTRA_SLOT_BYTES));
        last_seq = s[2];
        any = true;
      }
    }
  }
};

struct Rig {
  dig2analog_ultra::FakeHub hub;
  light::LightState state, stream;
  PixelLight l;
  PixelFxEffect e_aurora{"Aurora", 13}, e_solid{"Solid", 1}, e_new{"Newer", 40};
  Wire wire;
  explicit Rig(bool with_stream = false) {
    state.name = "Strip";
    stream.name = "Stream";
    state.output = &l;
    for (PixelFxEffect *e : {&e_aurora, &e_solid, &e_new}) {
      e->state_ = &state;
      l.effects_.push_back(e);
    }
    l.hub_ = &hub;
    l.state_ = &state;
    l.palette_ids_ = {ULTRA_PX_PAL_DEFAULT, ULTRA_PX_PAL_COLORS, 0, 1, 2};
    l.solid_ = 1;
    l.effect_ = 1;
    l.state_in_.speed = 128;
    l.state_in_.intensity = 128;
    l.store_after_ms_ = 0;
    l.cfg_.push_back({ULTRA_CFG_PX_PIXELS, 300});
    if (with_stream)
      l.others_.push_back(&stream);
    PixelLight::registry_.push_back(&l);
    state.add_remote_values_listener(&l);
    hub.tel.rd.have_identity = true;
    hub.tel.rd.hw_caps = ULTRA_HWCAP_PIXEL_FX;
  }
  ~Rig() { PixelLight::registry_.clear(); }
  /// our light's state changed (a call performed: the remote values, then LightState::loop writes the output)
  void set(bool on, float brightness = 1.0f) {
    state.set_remote(on, brightness);
    l.write_state(&state);
  }
  /// the loop at now_ms; the wire drained
  void run(uint32_t ms = 20) {
    for (uint32_t i = 0; i < ms; i++) {
      now_ms++;
      l.loop();
    }
    wire.drain(hub);
  }
};

static int32_t start_of(const std::vector<uint8_t> &s) { return static_cast<int32_t>(ultra_get32(s.data() + ULTRA_PX_START)); }

static void test_boot_and_effects() {
  warnings.clear();
  now_ms = 100000;
  Rig r;
  r.l.setup();
  CHECK(r.hub.configs.size() == 1 && r.hub.configs[0].first == ULTRA_CFG_PX_PIXELS &&
            r.hub.flushes == 1 && r.hub.callbacks.size() == 1,
        "setup: its CONFIG is sent (once, flushed), the page hook is in");
  r.state.set_remote(true, 0.5f);
  r.run();
  CHECK(r.wire.fx.size() == 1, "boot: one PIXEL_FX (%zu)", r.wire.fx.size());
  const auto &boot = r.wire.fx[0];
  CHECK(boot[ULTRA_PX_EFFECT] == 1 && boot[ULTRA_PX_BRIGHTNESS] == 128 && boot[ULTRA_PX_TRANSITION] == 0 &&
            boot[ULTRA_PX_TRANSITION + 1] == 0 && boot[ULTRA_PX_PALETTE] == ULTRA_PX_PAL_DEFAULT,
        "boot: Solid, brightness 0.5, no crossfade, the effect's own palette");
  CHECK(r.l.pixel_lit(now_ms), "on: lit for the power");

  // an effect: a new anchor, the crossfade over the light's default transition
  const int64_t anchor0 = r.l.track_.anchor_us();
  now_ms += 500;
  r.e_aurora.start();
  r.run();
  CHECK(r.wire.fx.size() == 2 && r.wire.fx[1][ULTRA_PX_EFFECT] == 13, "Aurora goes out");
  CHECK(ultra_get16(r.wire.fx[1].data() + ULTRA_PX_TRANSITION) == 1000, "... with the default transition as the crossfade");
  CHECK(r.l.track_.anchor_us() > anchor0, "... and a new anchor");
  const int64_t anchor1 = r.l.track_.anchor_us();

  // speed: the same anchor, the colours ramp
  r.l.set_param(ultra_digital::PARAM_SPEED, 200);
  r.run();
  CHECK(r.wire.fx.size() == 3 && r.wire.fx[2][ULTRA_PX_SPEED] == 200 && r.l.track_.anchor_us() == anchor1 &&
            ultra_get16(r.wire.fx[2].data() + ULTRA_PX_TRANSITION) == 250,
        "speed: the effect keeps its anchor, a 250 ms ramp");
  r.l.set_param(ultra_digital::PARAM_SPEED, 200);
  r.run();
  CHECK(r.wire.fx.size() == 3, "the same value again: nothing");
  r.l.set_palette_index(3);
  r.run();
  CHECK(r.wire.fx.size() == 4 && r.wire.fx[3][ULTRA_PX_PALETTE] == 1 && r.l.track_.anchor_us() == anchor1, "palette index 3 = catalogue palette 1");
  r.l.set_palette_index(99);
  r.run();
  CHECK(r.wire.fx.size() == 4, "a palette index out of range: nothing");
  const uint8_t c2[3] = {9, 8, 7};
  r.l.set_color2(c2);
  r.run();
  CHECK(r.wire.fx.size() == 5 && r.wire.fx[4][ULTRA_PX_C2] == 9 && r.wire.fx[4][ULTRA_PX_C2 + 2] == 7, "colour 2 by the action");
  r.l.set_param(ultra_digital::PARAM_CUSTOM2, 77);
  r.run();
  CHECK(r.wire.fx.size() == 6 && r.wire.fx[5][ULTRA_PX_CUSTOM2] == 77 && r.l.get_param(ultra_digital::PARAM_CUSTOM2) == 77, "custom 2");

  // the transition ESPHome asked for goes to the receiver
  r.l.transition_requested(3000);
  r.set(true, 0.25f);
  r.run();
  CHECK(r.wire.fx.size() == 7 && r.wire.fx[6][ULTRA_PX_BRIGHTNESS] == 64 && ultra_get16(r.wire.fx[6].data() + ULTRA_PX_TRANSITION) == 3000,
        "a transition: the target brightness and its length");
  // off: brightness 0 with the transition, lit until it is over
  r.l.transition_requested(2000);
  r.set(false, 0.25f);
  r.run(1);
  CHECK(r.wire.fx.size() == 8 && r.wire.fx[7][ULTRA_PX_BRIGHTNESS] == 0 && r.wire.fx[7][ULTRA_PX_EFFECT] == 13, "off: brightness 0, the effect carries on");
  CHECK(r.l.pixel_lit(now_ms) && r.l.pixel_lit(now_ms + 1900) && !r.l.pixel_lit(now_ms + 2100), "lit while it fades out");
  // an effect is stopped (None): Solid again
  r.set(true, 1.0f);
  r.run();
  r.e_aurora.stop();
  r.run();
  CHECK(r.wire.fx.back()[ULTRA_PX_EFFECT] == 1, "no effect: Solid");
  // a stop of an effect that was replaced already does nothing
  r.e_aurora.start();
  r.e_solid.start();
  r.e_aurora.stop();
  r.run();
  CHECK(r.wire.fx.back()[ULTRA_PX_EFFECT] == 1 && r.l.effect_ == 1, "the old effect's stop after the new one's start: still the new one");
  CHECK(warnings.empty(), "no warnings (%zu)", warnings.size());
  std::printf("  light: boot, effect (anchor, crossfade), speed / palette / colours keep the anchor, transition, off, Solid\n");
}

static void test_missing_effect() {
  warnings.clear();
  now_ms = 200000;
  Rig r;
  r.l.setup();
  r.state.set_remote(true);
  r.run();
  r.e_new.start();
  r.run();
  CHECK(r.wire.fx.back()[ULTRA_PX_EFFECT] == 40, "before the maps are known: sent as chosen");
  // the receiver's maps arrive: effect 40 is not in its firmware
  uint8_t m[24] = {};
  for (int id : {1, 13})
    m[id >> 3] |= static_cast<uint8_t>(1u << (id & 7));
  dig2analog_ultra::decode_pixel_page(r.hub.tel.ps, ULTRA_PXG_PART_EFFECTS, m);
  CHECK(!r.hub.callbacks.empty(), "hooked");
  r.l.pixel_page_();
  r.run();
  CHECK(r.wire.fx.back()[ULTRA_PX_EFFECT] == 1, "page 0x81 says it is missing: Solid is sent");
  CHECK(!warnings.empty(), "... and it is in the log (%zu)", warnings.size());
  const size_t n = r.wire.fx.size();
  r.e_aurora.start();
  r.run();
  CHECK(r.wire.fx.size() == n + 1 && r.wire.fx.back()[ULTRA_PX_EFFECT] == 13, "an effect it has is sent as chosen");
  // no telemetry: sent as chosen, whatever
  Rig q;
  q.hub.telemetry_on = false;
  q.l.setup();
  CHECK(q.hub.callbacks.empty(), "no page hook without telemetry");
  q.state.set_remote(true);
  q.e_new.start();
  q.run();
  CHECK(q.wire.fx.back()[ULTRA_PX_EFFECT] == 40, "without telemetry nothing is known: as chosen");
  std::printf("  light: an effect the receiver lacks shows Solid, logged\n");
}

static void test_exclusive() {
  warnings.clear();
  now_ms = 300000;
  Rig r(true);
  r.l.setup();
  // boot: this light on and the stream light on: this one wins, the stream light is switched off
  r.stream.set_remote(true);
  r.state.set_remote(true);
  r.run();
  CHECK(r.stream.calls.size() == 1 && !r.stream.calls[0], "boot: the streaming light is switched off");
  CHECK(r.wire.fx.size() == 1 && r.wire.fx[0][ULTRA_PX_EFFECT] != 0, "... and ours is sent");
  // our light switched on while the stream is on: the stream goes off
  Rig s(true);
  s.l.setup();
  s.run();                                   // (boot: off, the stream light off)
  s.stream.set_remote(true);                 // the stream light on (while ours is off but local: it hands back)
  s.run();
  CHECK(s.wire.fx.size() >= 1 && s.wire.fx.back()[ULTRA_PX_EFFECT] == 0, "the stream light on: PIXEL_FX 0 hands the strip back");
  const size_t n = s.wire.fx.size();
  s.set(true);                               // ours on: the stream light goes off
  s.run();
  CHECK(!s.stream.calls.empty() && !s.stream.calls.back(), "ours on: the streaming light is switched off");
  CHECK(s.wire.fx.size() == n + 1 && s.wire.fx.back()[ULTRA_PX_EFFECT] != 0, "... and ours takes the strip with a new anchor, no crossfade");
  // the stream light on while ours is running: ours is handed back and goes off
  s.stream.calls.clear();
  s.stream.set_remote(true);
  s.run();
  CHECK(s.wire.fx.back()[ULTRA_PX_EFFECT] == 0, "ours running, the stream light on: PIXEL_FX 0");
  CHECK(!s.state.calls.empty() && !s.state.calls.back() && !s.state.remote_values.on, "... and our light is switched off");
  const size_t m = s.wire.fx.size();
  s.run(100);
  CHECK(s.wire.fx.size() == m, "released and off: nothing more is sent");
  CHECK(!s.l.pixel_lit(now_ms), "released: not lit");
  // a boot with the stream light on and ours off: the receiver streams
  Rig b(true);
  b.l.setup();
  b.stream.set_remote(true);
  b.run();
  CHECK(b.wire.fx.size() == 1 && b.wire.fx[0][ULTRA_PX_EFFECT] == 0, "boot: the stream light on, ours off: effect 0 (a stored effect stops)");
  std::printf("  light: exclusive with a streaming light: boot, both ways, the hand-back\n");
}

static void test_store_and_anchor() {
  now_ms = 400000;
  Rig r;
  r.l.store_after_ms_ = 30000;
  r.l.setup();
  r.state.set_remote(true);
  r.run();
  const size_t n = r.wire.fx.size();
  now_ms += 29000;
  r.run(10);
  CHECK(r.wire.fx.size() == n, "STORE: not before the delay");
  now_ms += 2000;
  r.run(10);
  CHECK(r.wire.fx.size() == n + 1 && (r.wire.fx.back()[ULTRA_PX_FLAGS] & ULTRA_PXF_STORE) &&
            ultra_get16(r.wire.fx.back().data() + ULTRA_PX_TRANSITION) == 0,
        "STORE once, 30 s after the last change");
  now_ms += 60000;
  r.run(10);
  CHECK(r.wire.fx.size() == n + 1, "only once");
  r.l.set_param(ultra_digital::PARAM_SPEED, 10);
  r.run();
  CHECK(!(r.wire.fx.back()[ULTRA_PX_FLAGS] & ULTRA_PXF_STORE), "a change: sent without STORE (the receiver keeps the stored one)");
  // the anchor is renewed after 20 days: a crossfade, a new START
  const int64_t a0 = r.l.track_.anchor_us();
  now_ms += 21u * 86400000u;
  r.run(10);
  CHECK(r.l.track_.anchor_us() > a0 && r.wire.fx.back()[ULTRA_PX_EFFECT] == 1, "20 days: a new anchor");
  const int64_t gap = r.l.track_.anchor_us() - a0;
  CHECK(gap > 20LL * 86400 * 1000000, "... a fresh one (%lld us later)", static_cast<long long>(gap));
  // ... but the same slot with only a new START: the receiver keeps its phase (a running Sunrise goes on)
  {
    const size_t k = r.wire.fx.size();
    CHECK(k >= 2, "wire");
    std::vector<uint8_t> prev = r.wire.fx[k - 2], cur = r.wire.fx[k - 1];
    prev[ULTRA_PX_START] = prev[ULTRA_PX_START + 1] = prev[ULTRA_PX_START + 2] = prev[ULTRA_PX_START + 3] = 0;
    cur[ULTRA_PX_START] = cur[ULTRA_PX_START + 1] = cur[ULTRA_PX_START + 2] = cur[ULTRA_PX_START + 3] = 0;
    CHECK(prev[ULTRA_PX_EFFECT] == cur[ULTRA_PX_EFFECT] && prev[ULTRA_PX_FLAGS] == cur[ULTRA_PX_FLAGS] &&
              !(cur[ULTRA_PX_FLAGS] & ULTRA_PXF_RESTART),
          "the automatic renewal: no RESTART (the slot is the last one's but for its START and SEQ)");
  }
  // the user chooses the effect again (in Home Assistant): a time-driven effect starts over: the slot differs
  {
    const size_t k = r.wire.fx.size();
    const uint8_t fx = r.wire.fx.back()[ULTRA_PX_EFFECT];
    r.e_solid.start();
    r.run();
    CHECK(r.wire.fx.size() == k + 1 && r.wire.fx.back()[ULTRA_PX_EFFECT] == fx &&
              (r.wire.fx.back()[ULTRA_PX_FLAGS] & ULTRA_PXF_RESTART),
          "the same effect chosen again: a new START and RESTART, so the receiver starts it over");
    r.l.set_param(ultra_digital::PARAM_SPEED, 33);
    r.run();
    CHECK(r.wire.fx.size() == k + 2 && !(r.wire.fx.back()[ULTRA_PX_FLAGS] & ULTRA_PXF_RESTART),
          "a parameter change after it does not carry RESTART");
  }
  std::printf("  light: STORE once after the delay, not with a change, the anchor after 20 days\n");
}

int main() {
  test_boot_and_effects();
  test_missing_effect();
  test_exclusive();
  test_store_and_anchor();
  if (failures == 0)
    std::printf("ALL ULTRA DIGITAL LIGHT TESTS PASSED (0 failures)\n");
  else
    std::printf("FAILED (%d failures)\n", failures);
  return failures == 0 ? 0 : 1;
}
