#include "ultra_receiver.h"

#if defined(USE_ESP32) && defined(USE_DIG2ANALOG_ULTRA_TELEMETRY)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra.receiver";

static constexpr uint32_t ALL_NEW = TU_AVAILABILITY | TU_RESTART;
static constexpr uint32_t PG_STATUS = 1u << TP_STATUS, PG_POWER = 1u << TP_POWER, PG_ENERGY = 1u << TP_ENERGY,
                          PG_SUPPLY = 1u << TP_SUPPLY, PG_OUTPUTS = 1u << TP_OUTPUTS, PG_HEALTH = 1u << TP_HEALTH,
                          PG_IDENTITY = 1u << TP_IDENTITY, PG_LIFETIME = 1u << TP_LIFETIME, PG_DIAG = 1u << TP_DIAG,
                          PG_FAULTS = 1u << TP_FAULTS;
static constexpr uint32_t SLOW_MS = 60000;

// ------------------------------------------------------------------------------------------------------- receiver
void UltraReceiver::setup() {
  this->hub_->add_on_telemetry_callback([this](uint16_t receiver, uint32_t updates) {
    if (receiver == this->board_)
      this->update_(updates);
  });
}

void UltraReceiver::value_(uint8_t which, float v) {
  sensor::Sensor *s = this->values_[which];
  if (s == nullptr)
    return;
  if (s->has_state() && (s->state == v || (std::isnan(s->state) && std::isnan(v))))
    return;   // unchanged
  s->publish_state(v);
}

void UltraReceiver::text_(uint8_t which, const std::string &s) {
  text_sensor::TextSensor *t = this->texts_[which];
  if (t != nullptr && !s.empty() && (!t->has_state() || t->state != s))
    t->publish_state(s);
}

void UltraReceiver::update_(uint32_t updates) {
  const TelemetryPoller &p = this->hub_->telemetry();
  const bool answering = p.answering(this->board_);
  const ReceiverData &d = p.data(this->board_);
  const bool all = (updates & ALL_NEW) != 0;
  auto from = [&](uint32_t pages) { return all || (updates & pages) != 0; };
  auto v = [&](TelemetryValue tv) { return answering ? telemetry_value(d, tv) : NAN; };

  // the status: worst first, with its plain line and the Problem
  const ReceiverStatus st = receiver_status(d, answering);
  this->text_(STATUS, receiver_status_name(st));
  this->text_(STATUS_DETAIL, receiver_status_detail(d, st, this->board_, p.cut_at(millis())));
  if (this->problem_ != nullptr)
    this->problem_->publish_state(receiver_status_problem(st));   // (binary_sensor publishes changes only)

  if (from(PG_STATUS | PG_ENERGY))
    this->value_(POWER, v(TV_INPUT_POWER));
  if (from(PG_ENERGY)) {
    this->value_(ENERGY, v(TV_ENERGY));
    this->value_(LED_SUPPLY, v(TV_LED_SUPPLY));       // (0.2.5) after the buck
    this->value_(CURRENT_PEAK, v(TV_CURRENT_PEAK));   // (0.2.5) since the ENERGY read before
  }
  if (from(PG_DIAG))
    this->value_(SUPPLY_IMPEDANCE, v(TV_SUPPLY_IMPEDANCE));
  if (from(PG_LIFETIME)) {
    this->value_(LIT_HOURS, v(TV_LIT_HOURS));
    this->value_(POWER_UPS, v(TV_POWER_UPS));
  }
  if (from(PG_STATUS)) {
    this->value_(BUS_VOLTAGE, v(TV_SUPPLY_VOLTAGE));
    this->value_(MCU_TEMPERATURE, v(TV_MCU_TEMPERATURE));
    this->value_(INPUT_CURRENT, v(TV_INPUT_CURRENT));
    // along the port's cable: the drop from the sender to here, the segment before it (PortMonitor)
    const PortMonitor &m = this->hub_->port_monitor();
    this->value_(VOLTAGE_DROP, answering ? m.drop_v(this->board_) : NAN);
    const float r = m.resistance(this->board_);
    this->value_(CABLE_RESISTANCE, std::isnan(r) ? NAN : r * 1000.0f);
  }
  // trips and dips: the lifetime counts on a board with LIFETIME (0.2.5), else since its power-up
  if (from(PG_POWER | PG_LIFETIME)) {
    this->value_(TRIPS, v(TV_TRIPS));
    this->value_(LIMIT_EVENTS, v(TV_LIMIT_EVENTS));
  }
  if (from(PG_SUPPLY | PG_LIFETIME))
    this->value_(SUPPLY_DIPS, v(TV_DIPS));
  if (all || !this->slow_sent_ || millis() - this->slow_ms_ >= SLOW_MS) {   // at most once a minute
    this->slow_ms_ = millis();
    this->slow_sent_ = true;
    this->value_(UPTIME, v(TV_UPTIME));
    const uint32_t polls = p.polls(this->board_);
    this->value_(MISSED_POLLS, polls ? 100.0f * static_cast<float>(p.missed(this->board_)) / polls : NAN);
  }
  if (!answering)
    return;   // the texts keep what they last said
  if (from(PG_IDENTITY | PG_STATUS))
    this->text_(FIRMWARE, telemetry_firmware_text(d));
  // (0.2.5: DIAG's breadcrumbs and fault memory, across power cuts)
  if (from(PG_STATUS | PG_DIAG | PG_FAULTS))
    this->text_(LAST_RESET, telemetry_last_reset(d));
  if (from(PG_POWER | PG_FAULTS))
    this->text_(LAST_TRIP, telemetry_last_trip(d));
  if (from(PG_SUPPLY | PG_FAULTS))
    this->text_(LAST_DIP, telemetry_last_dip(d));
  if (from(PG_HEALTH | PG_FAULTS | PG_IDENTITY))
    this->text_(CHANNEL_HEALTH, telemetry_health_text(d));
  if (all && p.has_uid(this->board_)) {
    char b[24];
    snprintf(b, sizeof(b), "%u \xC2\xB7 UID %04X", static_cast<unsigned>(this->board_ + 1),
             static_cast<unsigned>(p.uid16(this->board_)));
    this->text_(LINK, b);
  }
}

void UltraReceiver::dump_config() {
  ESP_LOGCONFIG(TAG, "QuinLED Diff-Power ultra receiver %u (GPIO%u):", this->board_ + 1, this->hub_->get_pin());
  LOG_SENSOR("  ", "Power", this->values_[POWER]);
  LOG_SENSOR("  ", "Energy", this->values_[ENERGY]);
  LOG_TEXT_SENSOR("  ", "Status", this->texts_[STATUS]);
  LOG_BINARY_SENSOR("  ", "Problem", this->problem_);
}

// ----------------------------------------------------------------------------------------------------------- port
void UltraTelemetryPort::setup() {
  this->hub_->add_on_window_callback([this]() { this->update_(); });
}

void UltraTelemetryPort::update_() {
  const TelemetryPoller &p = this->hub_->telemetry();
  const PortMonitor &m = this->hub_->port_monitor();
  auto value = [this](uint8_t which, float v) {
    sensor::Sensor *s = this->values_[which];
    if (s != nullptr && (!s->has_state() || !(s->state == v || (std::isnan(s->state) && std::isnan(v)))))
      s->publish_state(v);
  };
  auto text = [this](uint8_t which, const std::string &v) {
    text_sensor::TextSensor *s = this->texts_[which];
    if (s != nullptr && (!s->has_state() || s->state != v))
      s->publish_state(v);
  };
  value(POWER, m.power_w());
  value(CURRENT, m.current_a());
  value(LOAD, m.load_pct());
  value(CABLE_LOSS, m.cable_loss_w());
  value(CABLE_LOSS_PCT, m.cable_loss_pct());
  value(PEAK_LOAD, m.peak_load_pct());
  uint16_t answering = 0;
  bool problem = m.problem();
  for (uint16_t i = 0; i < p.receivers(); i++) {
    answering += p.answering(i);
    problem |= receiver_status_problem(receiver_status(p.data(i), p.answering(i)));
  }
  if (this->problem_ != nullptr)
    this->problem_->publish_state(problem);
  text(RECEIVERS, std::to_string(answering) + " of " + std::to_string(p.receivers()) + " answering");
  text(STATUS, port_status(m, p));
  text(STATUS_DETAIL, port_status_detail(m, millis()));
}

void UltraTelemetryPort::dump_config() {
  const PortMonitor::Config &c = this->hub_->port_monitor().config();
  ESP_LOGCONFIG(TAG, "QuinLED Diff-Power port (GPIO%u): %u receiver(s), limit %.2f A, warning %.0f %%, problem %.0f %% "
                "for %u s, sender %s",
                this->hub_->get_pin(), this->hub_->telemetry().receivers(), c.limit_a, c.warning_pct, c.problem_pct,
                static_cast<unsigned>(c.problem_ms / 1000), std::isnan(c.supply_v) ? "learned" : "configured");
  LOG_SENSOR("  ", "Power", this->values_[POWER]);
  LOG_SENSOR("  ", "Current", this->values_[CURRENT]);
  LOG_SENSOR("  ", "Load", this->values_[LOAD]);
  LOG_BINARY_SENSOR("  ", "Problem", this->problem_);
  LOG_TEXT_SENSOR("  ", "Status", this->texts_[STATUS]);
}

// ---------------------------------------------------------------------------------------------------------- light
void UltraLightTelemetry::attach() {
  this->hub_->add_on_telemetry_callback([this](uint16_t receiver, uint32_t updates) {
    if (receiver == this->board_)
      this->update_(updates);
  });
}

void UltraLightTelemetry::update_(uint32_t updates) {
  const TelemetryPoller &p = this->hub_->telemetry();
  const bool answering = p.answering(this->board_);
  const ReceiverData &d = p.data(this->board_);
  if (this->status_ != nullptr) {
    const std::string s = receiver_status_name(light_status(d, answering, this->mask_));
    if (!this->status_->has_state() || this->status_->state != s)
      this->status_->publish_state(s);
  }
  if (!(updates & (PG_OUTPUTS | PG_ENERGY | PG_STATUS | ALL_NEW)))
    return;
  const float w = answering ? light_power_estimate(d, this->mask_) : NAN;
  this->energy_count_.add(w, millis());
  if (this->power_ != nullptr && (!this->power_->has_state() || this->power_->state != w))
    this->power_->publish_state(w);
  if (this->energy_ != nullptr && (updates & (PG_OUTPUTS | ALL_NEW)))
    this->energy_->publish_state(static_cast<float>(this->energy_count_.wh));
}

}  // namespace esphome::dig2analog_ultra

#endif
