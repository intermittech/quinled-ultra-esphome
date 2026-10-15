#pragma once

// A QuinLED Diff-Power sender port in ESPHome (wiki: Diff-Power ports): its enable output, its detect input, the
// lights on it (any ESPHome light; a dig2analog_ultra hub on it adds its own), its power mode and the dark window. The
// logic is diffpower_core.h; this drives the pins, polls the lights, sends their state again once the port is powered,
// keeps its counters across reboots and publishes the port's status.

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "esphome/components/light/light_state.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_DIFFPOWER_SUN
#include "esphome/components/sun/sun.h"
#endif

#include "diffpower_core.h"

#include <functional>
#include <vector>

namespace esphome::diffpower {

class DiffPowerPort final : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  /// OTA / a reboot: every port in ON gets its notice (one already in NOTICE finishes its own), all ports served
  /// together until each one's enable is low (1 s at most), then every enable low
  void on_shutdown() override { shutdown_all(); }
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // configuration (codegen)
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_detect_pin(GPIOPin *pin) { this->detect_pin_ = pin; }
  void set_name(const char *name) { this->name_ = name; }
  /// the key of its counters on the ESP32 (from its id: unique, and a new name keeps them)
  void set_stats_key(const char *key) { this->stats_key_ = key; }
  void set_timing(uint32_t hotplug_ms, uint32_t boot_margin_ms, uint32_t notice_ms, uint32_t min_off_ms,
                  uint32_t min_on_ms, uint32_t standby_after_ms) {
    PortConfig c = this->core_.config();
    c.hotplug_ms = hotplug_ms;
    c.boot_margin_ms = boot_margin_ms;
    c.notice_ms = notice_ms;
    c.min_off_ms = min_off_ms;
    c.min_on_ms = min_on_ms;
    c.standby_after_ms = standby_after_ms;
    this->core_.configure(c);
  }
  /// During dark without a valid time: powered (else Standby)
  void set_unknown_on(bool on) {
    PortConfig c = this->core_.config();
    c.unknown_on = on;
    this->core_.configure(c);
  }
  void set_mode(uint8_t mode) {
    this->core_.set_mode(static_cast<PowerMode>(mode));
    this->dark_checked_ = false;   // (the dark window again now)
  }
  uint8_t get_mode() const { return this->core_.mode(); }
#ifdef USE_DIFFPOWER_SUN
  /// the sun's time source and place (its latitude / longitude, copied at codegen: sun::Sun doesn't share them)
  void set_sun(sun::Sun *sun, float latitude, float longitude, uint32_t before_s, uint32_t after_s) {
    this->sun_ = sun;
    this->lat_ = latitude;
    this->lon_ = longitude;
    this->before_s_ = before_s;
    this->after_s_ = after_s;
  }
#endif
  bool has_sun() const { return this->has_sun_(); }
  /// any ESPHome light on this port; hold_ms: its own turn-off period (0: the port's). Its state is sent again once
  /// the port is powered.
  void add_light(light::LightState *light, uint32_t hold_ms) {
    this->add_load(hold_ms, [light](uint32_t) { return light->current_values.is_on() || light->remote_values.is_on(); },
                   light, true);
  }
  /// A load: on while lit() says so (lights, a hub's calibration / update); light: for wake actions by light;
  /// resend: write the light's state again when the port is powered
  void add_load(uint32_t hold_ms, std::function<bool(uint32_t)> &&lit, light::LightState *light = nullptr,
                bool resend = false) {
    this->core_.add_load(hold_ms);
    this->loads_.push_back(LoadRef{std::move(lit), light, resend});
  }
#ifdef USE_TEXT_SENSOR
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_ = s; }
#endif
#ifdef USE_BINARY_SENSOR
  void set_problem_sensor(binary_sensor::BinarySensor *s) { this->problem_ = s; }
#endif
#ifdef USE_SENSOR
  void set_cycles_sensor(sensor::Sensor *s) { this->cycles_ = s; }
  void set_hours_sensor(sensor::Sensor *s) { this->hours_ = s; }
  void set_drops_sensor(sensor::Sensor *s) { this->drops_ = s; }
#endif

  /// Pre-wake (motion): powered for timeout_ms (at least min_on_time), or longer once a light turns on
  void wake(uint32_t timeout_ms);
  bool has_light(light::LightState *light) const;
  /// powered and the wake over: the lights' data reaches the boards
  bool is_ready() const { return this->core_.ready(); }
  PortState state() const { return this->core_.state(); }
  const PortCore &core() const { return this->core_; }
  const char *get_port_name() const { return this->name_; }
  /// Each time the wake is over (an Ultra hub: send everything again, its preamble first)
  void add_on_ready_callback(std::function<void()> &&cb) { this->ready_cb_.add(std::move(cb)); }
  /// A copy of the planned power-off notice to send (0, 1, 2; the first a new SEQ): an Ultra hub queues it and calls
  /// notice_sent() once its frame went out; the next copy follows 50 ms after that, the enable 100 ms after the third
  void add_on_notice_callback(std::function<void(uint8_t)> &&cb) { this->notice_cb_.add(std::move(cb)); }
  /// At the shutdown (the loop doesn't run any more): send what is queued now if the line is free (no waiting)
  void add_on_pump_callback(std::function<void()> &&cb) { this->pump_cb_.add(std::move(cb)); }
  /// Before a planned power-off: a hub's last reads (true: ready for the notice; first: the first call of this one).
  /// The notice waits for all of them, 3 s at most.
  void add_on_prepare_callback(std::function<bool(bool)> &&cb) {
    this->prepare_cb_.push_back(std::move(cb));
    PortConfig c = this->core_.config();
    c.prepare_max_ms = PREPARE_MAX_MS;
    this->core_.configure(c);
  }
  void notice_sent() { this->core_.notice_sent(millis()); }
  /// A deliberate power cycle (firmware recovery): off now, then a wake; power_cycle_ms() until powered again
  void power_cycle() { this->apply_(this->core_.power_cycle(millis()), millis()); }
  uint32_t power_cycle_ms() const { return this->core_.power_cycle_ms(); }
  static const std::vector<DiffPowerPort *> &ports() { return ports_; }
  static void shutdown_all();

 protected:
  struct LoadRef {
    std::function<bool(uint32_t)> lit;
    light::LightState *light;
    bool resend;
  };
  /// the counters kept across reboots
  struct Stats {
    uint32_t cycles, standby_s, drops;
  };
  bool has_sun_() const;
  void update_dark_(uint32_t now);
  void apply_(uint8_t ev, uint32_t now);
  void resend_();
  void publish_();
  void save_stats_();

  PortCore core_;
  GPIOPin *enable_pin_{nullptr};
  GPIOPin *detect_pin_{nullptr};
  const char *name_{"port"};
  const char *stats_key_{"port"};
  std::vector<LoadRef> loads_;
  CallbackManager<void()> ready_cb_;
  CallbackManager<void(uint8_t)> notice_cb_;
  CallbackManager<void()> pump_cb_;
  std::vector<std::function<bool(bool)>> prepare_cb_;
  bool prepare_first_{false};
  static constexpr uint32_t PREPARE_MAX_MS = 3000;
#ifdef USE_DIFFPOWER_SUN
  sun::Sun *sun_{nullptr};
#endif
  float lat_{0}, lon_{0};
  uint32_t before_s_{0}, after_s_{0};
  uint32_t dark_ms_{0};
  bool dark_checked_{false};
  ESPPreferenceObject stats_pref_;
  uint32_t saved_standby_s_{0}, saved_cycles_{0}, saved_drops_{0}, minute_ms_{0};
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *status_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *problem_{nullptr};
#endif
#ifdef USE_SENSOR
  sensor::Sensor *cycles_{nullptr}, *hours_{nullptr}, *drops_{nullptr};
#endif
  static std::vector<DiffPowerPort *> ports_;
  static bool shut_down_;   // (shutdown_all: once)
};

#ifdef USE_SELECT
/// The port's power mode (Standby / During dark / Always on), restored on the ESP32 (restore_value); the YAML's
/// power_mode first
class PowerModeSelect final : public select::Select, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_port(DiffPowerPort *port) { this->port_ = port; }

 protected:
  void control(size_t index) override;
  /// option index <-> PowerMode (without a sun: no "During dark")
  uint8_t mode_of_(size_t index) const;
  size_t index_of_(uint8_t mode) const;
  DiffPowerPort *port_{nullptr};
  ESPPreferenceObject pref_;
};
#endif

/// diffpower.wake: pre-wake ports, or the ports of lights, for timeout (back to standby if no light turns on)
template<typename... Ts> class WakeAction final : public Action<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint32_t, timeout)
  void add_port(DiffPowerPort *port) { this->ports_.push_back(port); }
  void add_light(light::LightState *light) { this->lights_.push_back(light); }
  void play(const Ts &...x) override {
    const uint32_t t = this->timeout_.value(x...);
    for (auto *p : this->ports_)
      p->wake(t);
    for (auto *l : this->lights_) {
      for (auto *p : DiffPowerPort::ports()) {
        if (p->has_light(l))
          p->wake(t);
      }
    }
  }

 protected:
  std::vector<DiffPowerPort *> ports_;
  std::vector<light::LightState *> lights_;
};

}  // namespace esphome::diffpower
