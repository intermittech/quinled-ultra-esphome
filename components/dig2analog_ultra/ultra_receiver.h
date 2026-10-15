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

#include "dig2analog_ultra.h"
#include "ultra_pacing.h"

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
    LIVE_POWER, VALUE_COUNT,
  };
  enum Text : uint8_t {
    STATUS = 0, STATUS_DETAIL, FIRMWARE, LAST_RESET, LAST_TRIP, LAST_DIP, LINK, CHANNEL_HEALTH, PORT, TEXT_COUNT,
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
/// A receiver's Identify button: it blinks its identify pattern for 10 s
class UltraIdentifyButton final : public button::Button {
 public:
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }

 protected:
  void press_action() override { this->hub_->identify_for(this->board_, 10000); }
  UltraHub *hub_{nullptr};
  uint16_t board_{0};
};

/// A line's buttons (disabled until enabled in Home Assistant): its current calibration, its boards' versions
class UltraLineButton final : public button::Button {
 public:
  enum Kind : uint8_t { CALIBRATE = 0, SHOW_VERSION };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_kind(uint8_t kind) { this->kind_ = kind; }

 protected:
  void press_action() override {
    if (this->kind_ == CALIBRATE)
      this->hub_->calibrate_boards({}, 0);
    else
      this->hub_->show_version(-1);
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
  enum Text : uint8_t { RECEIVERS = 0, STATUS, STATUS_DETAIL, TEXT_COUNT };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_value(uint8_t which, sensor::Sensor *s) {
    if (which < VALUE_COUNT)
      this->values_[which] = s;
  }
  void set_unpaced(uint8_t which) { this->unpaced_ |= 1u << which; }
  void set_text(uint8_t which, text_sensor::TextSensor *s) {
    if (which < TEXT_COUNT)
      this->texts_[which] = s;
  }
  void set_problem(binary_sensor::BinarySensor *s) { this->problem_ = s; }
  void setup() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void update_();
  void value_(uint8_t which, float v, bool force);
  UltraHub *hub_{nullptr};
  uint32_t unpaced_{0};
  sensor::Sensor *values_[VALUE_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *problem_{nullptr};
  Paced paced_[VALUE_COUNT]{};
  MeanPaced power_mean_;
  bool standby_was_{false};
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

}  // namespace esphome::dig2analog_ultra

#endif
