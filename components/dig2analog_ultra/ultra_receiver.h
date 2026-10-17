#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_DIG2ANALOG_ULTRA_TELEMETRY)

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#include "dig2analog_ultra.h"
#include "ultra_pacing.h"
#include "ultra_settings.h"

namespace esphome::dig2analog_ultra {

/// One QuinLED Diff-Power Ultra receiver in Home Assistant (receiver_schema.py; its own device): Power (the 10 s mean)
/// and Live power (the update rate), Energy, Status and Problem, its Port; its diagnostics. Published when a response
/// window brought news, paced for Home Assistant's database (ultra_pacing.h); unavailable (NAN) while it doesn't
/// answer or doesn't measure a value.
class UltraReceiver final : public Component {
 public:
  enum Value : uint8_t {
    POWER = 0, ENERGY, BUS_VOLTAGE, MCU_TEMPERATURE, INPUT_CURRENT, UPTIME, TRIPS, SUPPLY_DIPS, LIMIT_EVENTS,
    MISSED_POLLS, VOLTAGE_DROP, CABLE_RESISTANCE, LED_SUPPLY, CURRENT_PEAK, SUPPLY_IMPEDANCE, LIT_HOURS, POWER_UPS,
    LIVE_POWER, LAST_CALIBRATED, CONNECTED_LOAD, VALUE_COUNT,
  };
  enum Text : uint8_t {
    STATUS = 0, STATUS_DETAIL, FIRMWARE, LAST_RESET, LAST_TRIP, LAST_DIP, LINK, CHANNEL_HEALTH, PORT, CALIBRATION,
    CURRENT_MODEL, SUPPLY_NOTE, SETTINGS, TEXT_COUNT,
  };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_value(uint8_t which, sensor::Sensor *s) {
    if (which < VALUE_COUNT)
      this->values_[which] = s;
  }
  /// a value with the user's own filters: every change (they replace the pacing)
  void set_unpaced(uint8_t which) { this->unpaced_ |= 1u << which; }
  void set_text(uint8_t which, text_sensor::TextSensor *s) {
    if (which < TEXT_COUNT)
      this->texts_[which] = s;
  }
  void set_problem(binary_sensor::BinarySensor *s) { this->problem_ = s; }
  void setup() override;
  void dump_config() override;
  /// OTA / a reboot: its Energy one last time
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
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

#ifdef USE_BUTTON
/// A receiver's buttons: Identify (it blinks its identify pattern for 10 s), Calibrate current (its test, the whole
/// line dark meanwhile: UltraHub::calibrate_receiver)
class UltraReceiverButton final : public button::Button {
 public:
  enum Kind : uint8_t { IDENTIFY = 0, CALIBRATE };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_kind(uint8_t kind) { this->kind_ = kind; }

 protected:
  void press_action() override {
    if (this->kind_ == CALIBRATE)
      this->hub_->calibrate_receiver(this->board_);
    else
      this->hub_->identify_for(this->board_, 10000);
  }
  UltraHub *hub_{nullptr};
  uint16_t board_{0};
  uint8_t kind_{0};
};

/// A line's buttons: its current calibration, its boards' versions (disabled until enabled in Home Assistant); its
/// commissioning: Commission, Finalize (wiki: Commissioning)
class UltraLineButton final : public button::Button {
 public:
  enum Kind : uint8_t { CALIBRATE = 0, SHOW_VERSION, COMMISSION, FINALIZE };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_kind(uint8_t kind) { this->kind_ = kind; }

 protected:
  void press_action() override {
    if (this->kind_ == CALIBRATE)
      this->hub_->calibrate_boards({}, 0);
    else if (this->kind_ == SHOW_VERSION)
      this->hub_->show_version(-1);
    else
      this->hub_->commission(this->kind_ == FINALIZE);
  }
  UltraHub *hub_{nullptr};
  uint8_t kind_{0};
};
#endif

/// The port's summary (one Diff-Power sender port: its receivers, PortMonitor): its current against the port's limit,
/// its power at the sender (the 10 s mean, and live), its energy there, the cable loss along it, its peak load, "2 of
/// 3 answering", its status and a problem (the load, or a receiver's).
class UltraTelemetryPort final : public Component {
 public:
  enum Value : uint8_t {
    POWER = 0, CURRENT, LOAD, CABLE_LOSS, CABLE_LOSS_PCT, PEAK_LOAD, LIVE_POWER, ENERGY, VALUE_COUNT,
  };
  enum Text : uint8_t { RECEIVERS = 0, STATUS, STATUS_DETAIL, SETUP_INFO, TEXT_COUNT };
  /// the line's telemetry health (diagnostic, disabled by default; once a minute): poll windows, missing answers (%
  /// of those asked for in that minute), broken answers (bad CRC), answers with contention, receiver restarts, late
  /// releases of the line, the reply rate (kbit/s)
  enum Health : uint8_t { H_WINDOWS = 0, H_MISSING, H_BROKEN, H_CONTENTION, H_RESTARTS, H_LATE, H_RATE, HEALTH_COUNT };
  static constexpr uint32_t HEALTH_MS = 60000;
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_value(uint8_t which, sensor::Sensor *s) {
    if (which < VALUE_COUNT)
      this->values_[which] = s;
  }
  void set_health(uint8_t which, sensor::Sensor *s) {
    if (which < HEALTH_COUNT) {
      this->health_[which] = s;
      this->has_health_ = true;
    }
  }
  void set_unpaced(uint8_t which) { this->unpaced_ |= 1u << which; }
  void set_text(uint8_t which, text_sensor::TextSensor *s) {
    if (which < TEXT_COUNT)
      this->texts_[which] = s;
  }
  void set_problem(binary_sensor::BinarySensor *s) { this->problem_ = s; }
  /// the commissioning's texts (where it is; the last run's report) and whether the line is commissioned
  void set_commission_status(text_sensor::TextSensor *s) { this->comm_status_ = s; }
  void set_commission_report(text_sensor::TextSensor *s) { this->comm_report_ = s; }
  void set_commissioned(binary_sensor::BinarySensor *s) { this->commissioned_ = s; }
  void setup() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void update_();
  void value_(uint8_t which, float v, bool force);
  void publish_health_();
  UltraHub *hub_{nullptr};
  uint32_t unpaced_{0};
  sensor::Sensor *values_[VALUE_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *problem_{nullptr};
  Paced paced_[VALUE_COUNT]{};
  MeanPaced power_mean_;
  bool standby_was_{false};
  uint32_t setup_sig_{0};   // the Setup info's signature at its last build (ultra_telemetry.h)
  bool setup_sig_set_{false};
  sensor::Sensor *health_[HEALTH_COUNT]{};
  bool has_health_{false}, health_seen_{false};
  uint32_t health_ms_{0}, answers_was_{0}, missing_was_{0};
  void publish_commission_();
  text_sensor::TextSensor *comm_status_{nullptr}, *comm_report_{nullptr};
  binary_sensor::BinarySensor *commissioned_{nullptr};
};

/// A light on a receiver with several lights (on its own device): its estimated power (the receiver's measured power
/// split by the channel currents of its model; the 10 s mean), its estimated energy (counted up here, kept in the
/// ledger across restarts), its status (the receiver's, as far as it dims the light) and its receiver's Port. Not a
/// Component (made by the light's codegen): attach() hooks it to the hub's windows.
class UltraLightTelemetry final {
 public:
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_channels(uint8_t mask) { this->mask_ = mask; }
  /// its estimated energy's counter in the ledger (the light's id's hash)
  void set_key(uint32_t key) { this->key_ = key; }
  void set_estimated_power(sensor::Sensor *s) { this->power_ = s; }
  void set_estimated_energy(sensor::Sensor *s) { this->energy_ = s; }
  void set_status(text_sensor::TextSensor *s) { this->status_ = s; }
  void set_port(text_sensor::TextSensor *s) { this->port_ = s; }
  void set_unpaced(bool power, bool energy) { this->unpaced_ = (power ? 1 : 0) | (energy ? 2 : 0); }
  void attach();

 protected:
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

/// The controller's totals (its device): every port's power at the sender together (the 10 s mean, and live), and
/// their energy. Home Assistant follows these few values instead of every port.
class UltraTotals final : public Component {
 public:
  enum Value : uint8_t { POWER = 0, LIVE_POWER, ENERGY, VALUE_COUNT };
  void set_value(uint8_t which, sensor::Sensor *s) {
    if (which < VALUE_COUNT)
      this->values_[which] = s;
  }
  void set_unpaced(uint8_t which) { this->unpaced_ |= 1u << which; }
  void setup() override;
  void dump_config() override;
  void on_shutdown() override { this->update_(true); }
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void update_(bool force);
  void value_(uint8_t which, float v, bool force);
  uint32_t unpaced_{0};
  sensor::Sensor *values_[VALUE_COUNT]{};
  Paced paced_[VALUE_COUNT]{};
  MeanPaced power_mean_;
};

// ------------------------------------------------------------------------------ the controller's settings entities
/// The controller's settings (wiki: Controller-Settings), one per ESP32, each applied to every line with telemetry:
/// set in Home Assistant, kept on the ESP32 (SettingRecord: a changed YAML default wins once), applied at boot after
/// the lines' setup and at once on a change.
enum ControllerSetting : uint8_t {
  CS_UPDATE_RATE = 0,   // select: fast / normal / slow (1 / 2 / 5 s)
  CS_ADAPTIVE_RATE,     // switch
  CS_LOAD_WARNING,      // %
  CS_LOAD_PROBLEM,      // %
  CS_LOAD_PROBLEM_TIME, // s
  CS_PORT_LIMIT,        // A
  CS_DIP_WARNING,       // %
  CS_COMMISSION_MIN_V,  // %
  CS_SUPPLY_RATING,     // W (the dashboards' headroom; nothing in ESPHome)
  CS_COUNT,
};
static constexpr uint32_t UPDATE_RATE_MS[3] = {1000, 2000, 5000};
/// the setting applied to every line with telemetry
void apply_controller_setting(uint8_t kind, float value);
/// the stored choice (SettingRecord) of a setting: value at boot, store a change
class SettingStore {
 public:
  void init(uint8_t kind, float yaml);
  float boot_value() const { return this->boot_; }
  void store(float value);

 protected:
  ESPPreferenceObject pref_;
  float yaml_{0.0f}, boot_{0.0f};
};

#ifdef USE_NUMBER
class UltraSettingNumber final : public number::Number, public Component {
 public:
  void set_kind(uint8_t kind) { this->kind_ = kind; }
  void set_default(float yaml) { this->yaml_ = yaml; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }   // (after the lines' setup)

 protected:
  void control(float value) override;
  SettingStore store_;
  float yaml_{0.0f};
  uint8_t kind_{0};
};
#endif
#ifdef USE_SELECT
/// Telemetry update rate: fast / normal / slow
class UltraSettingSelect final : public select::Select, public Component {
 public:
  void set_kind(uint8_t kind) { this->kind_ = kind; }
  void set_default(float yaml) { this->yaml_ = yaml; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  void control(size_t index) override;
  SettingStore store_;
  float yaml_{0.0f};
  uint8_t kind_{0};
};
#endif
#ifdef USE_SWITCH
/// Adaptive reply rate
class UltraSettingSwitch final : public switch_::Switch, public Component {
 public:
  void set_kind(uint8_t kind) { this->kind_ = kind; }
  void set_default(float yaml) { this->yaml_ = yaml; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  void write_state(bool state) override;
  SettingStore store_;
  float yaml_{0.0f};
  uint8_t kind_{0};
};
#endif
/// Telemetry diagnostics: the lines' compile-time diagnostics level, published once
class UltraSettingText final : public Component {
 public:
  void set_text(text_sensor::TextSensor *t, const char *value) {
    this->text_ = t;
    this->value_ = value;
  }
  void setup() override { this->text_->publish_state(this->value_); }
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  text_sensor::TextSensor *text_{nullptr};
  const char *value_{""};
};

}  // namespace esphome::dig2analog_ultra

#endif
