#pragma once

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_DIG2ANALOG_ULTRA_TELEMETRY)

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include "dig2analog_ultra.h"

namespace esphome::dig2analog_ultra {

/// One QuinLED Diff-Power ultra receiver in Home Assistant (receiver_schema.py; its own device): Power, Energy,
/// Status and Problem; its diagnostics. Published when a response window brought news, and only what changed;
/// unavailable (NAN) while it doesn't answer or doesn't measure a value.
class UltraReceiver final : public Component {
 public:
  enum Value : uint8_t {
    POWER = 0, ENERGY, BUS_VOLTAGE, MCU_TEMPERATURE, INPUT_CURRENT, UPTIME, TRIPS, SUPPLY_DIPS, LIMIT_EVENTS,
    MISSED_POLLS, VOLTAGE_DROP, CABLE_RESISTANCE, LED_SUPPLY, CURRENT_PEAK, SUPPLY_IMPEDANCE, LIT_HOURS, POWER_UPS,
    VALUE_COUNT,
  };
  enum Text : uint8_t {
    STATUS = 0, STATUS_DETAIL, FIRMWARE, LAST_RESET, LAST_TRIP, LAST_DIP, LINK, CHANNEL_HEALTH, TEXT_COUNT,
  };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_value(uint8_t which, sensor::Sensor *s) {
    if (which < VALUE_COUNT)
      this->values_[which] = s;
  }
  void set_text(uint8_t which, text_sensor::TextSensor *s) {
    if (which < TEXT_COUNT)
      this->texts_[which] = s;
  }
  void set_problem(binary_sensor::BinarySensor *s) { this->problem_ = s; }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void update_(uint32_t updates);
  void value_(uint8_t which, float v);
  void text_(uint8_t which, const std::string &s);
  UltraHub *hub_{nullptr};
  uint16_t board_{0};
  sensor::Sensor *values_[VALUE_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *problem_{nullptr};
  uint32_t slow_ms_{0};   // uptime and missed polls: at most once a minute
  bool slow_sent_{false};
};

/// The port's summary (one Diff-Power sender port: its receivers, PortMonitor): its current against the port's limit,
/// its power at the sender, the cable loss along it, its peak load, "2 of 3 answering", its status and a problem (the
/// load, or a receiver's).
class UltraTelemetryPort final : public Component {
 public:
  enum Value : uint8_t { POWER = 0, CURRENT, LOAD, CABLE_LOSS, CABLE_LOSS_PCT, PEAK_LOAD, VALUE_COUNT };
  enum Text : uint8_t { RECEIVERS = 0, STATUS, STATUS_DETAIL, TEXT_COUNT };
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_value(uint8_t which, sensor::Sensor *s) {
    if (which < VALUE_COUNT)
      this->values_[which] = s;
  }
  void set_text(uint8_t which, text_sensor::TextSensor *s) {
    if (which < TEXT_COUNT)
      this->texts_[which] = s;
  }
  void set_problem(binary_sensor::BinarySensor *s) { this->problem_ = s; }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void update_();
  UltraHub *hub_{nullptr};
  sensor::Sensor *values_[VALUE_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *problem_{nullptr};
};

/// A light on a receiver with several lights (on its own device): its estimated power (the receiver's measured power
/// split by the channel currents of its model), its estimated energy (counted up here; it starts over when the ESP
/// restarts: total_increasing), and its status (the receiver's, as far as it dims the light). Not a Component (made
/// by the light's codegen): attach() hooks it to the hub's windows.
class UltraLightTelemetry final {
 public:
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_channels(uint8_t mask) { this->mask_ = mask; }
  void set_estimated_power(sensor::Sensor *s) { this->power_ = s; }
  void set_estimated_energy(sensor::Sensor *s) { this->energy_ = s; }
  void set_status(text_sensor::TextSensor *s) { this->status_ = s; }
  void attach();

 protected:
  void update_(uint32_t updates);
  UltraHub *hub_{nullptr};
  uint16_t board_{0};
  uint8_t mask_{0};
  sensor::Sensor *power_{nullptr};
  sensor::Sensor *energy_{nullptr};
  text_sensor::TextSensor *status_{nullptr};
  EnergyCounter energy_count_;
};

}  // namespace esphome::dig2analog_ultra

#endif
