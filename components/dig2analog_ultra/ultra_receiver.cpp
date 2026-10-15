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
                          PG_FAULTS = 1u << TP_FAULTS, PG_SYNC = 1u << TP_SYNC;

/// a live value's pace: its deadband at the update rate (a little less: the STATUS reads come with some jitter)
static Pace live_pace(uint32_t rate_ms) { return Pace{PACE_LIVE_W.abs, PACE_LIVE_W.rel, rate_ms - rate_ms / 4}; }

/// one sensor's publish through its pace (unpaced: its user's own filters, every change); mean: a recorded power
static void publish_paced(sensor::Sensor *s, const Pace &p, Paced &paced, MeanPaced *mean, float v, bool force) {
  if (s == nullptr)
    return;
  const bool have = s->has_state();
  const float last = s->get_raw_state();
  if (mean != nullptr) {
    float out;
    if (mean->sample(p, v, last, have, millis(), force, &out))
      s->publish_state(out);
    return;
  }
  if (paced.due(p, v, last, have, millis(), force))
    s->publish_state(v);
}

// ------------------------------------------------------------------------------------------------------- receiver

static Pace receiver_pace(uint8_t which, uint32_t rate_ms) {
  switch (which) {
    case UltraReceiver::POWER:
      return PACE_POWER;
    case UltraReceiver::LIVE_POWER:
      return live_pace(rate_ms);
    case UltraReceiver::ENERGY:
      return PACE_ENERGY;
    case UltraReceiver::BUS_VOLTAGE:
    case UltraReceiver::LED_SUPPLY:
      return PACE_VOLTAGE;
    case UltraReceiver::MCU_TEMPERATURE:
      return PACE_TEMPERATURE;
    case UltraReceiver::INPUT_CURRENT:
    case UltraReceiver::CURRENT_PEAK:
      return PACE_INPUT_A;
    case UltraReceiver::UPTIME:
      return PACE_UPTIME;
    case UltraReceiver::MISSED_POLLS:
      return PACE_MISSED;
    case UltraReceiver::VOLTAGE_DROP:
      return PACE_DROP_V;
    case UltraReceiver::CABLE_RESISTANCE:
      return PACE_RESISTANCE;
    default:
      return PACE_CHANGE;   // counters, lit hours, the supply impedance: every change
  }
}

void UltraReceiver::setup() {
  this->hub_->add_on_receiver_callback(this->board_, [this](uint32_t updates) { this->update_(updates); });
  // its port and position: "Port 4 · receiver 2" (the system health groups receivers by it)
  this->text_(PORT, std::string(this->hub_->telemetry_name()) + " \xC2\xB7 receiver " + std::to_string(this->board_ + 1));
}

void UltraReceiver::value_(uint8_t which, float v, bool force) {
  const bool unpaced = this->unpaced_ & (1u << which);
  const Pace p = unpaced ? PACE_CHANGE : receiver_pace(which, this->hub_->update_rate_ms());
  publish_paced(this->values_[which], p, this->paced_[which], which == POWER && !unpaced ? &this->power_mean_ : nullptr,
                v, force);
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
  const bool all = (updates & ALL_NEW) != 0;   // (its availability changed, a restart: values at once)
  auto from = [&](uint32_t pages) { return all || (updates & pages) != 0; };
  // its Diff-Power port not on: no power (Power and Current 0), nothing else known; Energy keeps its total.
  // The lifetime counts: the ledger's (the highest seen: also while the board's record is held, DIAG EXHAUSTED)
  const bool standby = this->hub_->port_standby();
  const LifeCounters *lc = d.have_identity && (d.features & ULTRA_FEAT_LIFETIME) ? this->hub_->ledger_counters(this->board_)
                                                                                 : nullptr;
  auto v = [&](TelemetryValue tv) {
    if (standby)
      return tv == TV_INPUT_POWER || tv == TV_INPUT_CURRENT ? 0.0f : NAN;
    if (!answering)
      return NAN;
    if (lc != nullptr && d.have_lifetime) {
      switch (tv) {
        case TV_LIT_HOURS:
          return lc->lit * 0.1f;
        case TV_POWER_UPS:
          return static_cast<float>(lc->ups);
        case TV_TRIPS:
          return static_cast<float>(lc->ovl + lc->shorts);
        case TV_DIPS:
          return static_cast<float>(lc->dips);
        default:
          break;
      }
    }
    return telemetry_value(d, tv);
  };

  // the status: worst first, with its plain line and the Problem (just woken, not heard yet: Waking)
  const ReceiverStatus st = standby                                 ? (this->hub_->port_off() ? RS_PORT_OFF : RS_STANDBY)
                            : this->hub_->rx_settling(this->board_) ? RS_WAKING
                                                                    : receiver_status(d, answering);
  // (a text is built only for its entity: a level the YAML leaves out costs nothing at run time)
  auto has = [this](uint8_t which) { return this->texts_[which] != nullptr; };
  this->text_(STATUS, receiver_status_name(st));
  if (has(STATUS_DETAIL)) {
    this->text_(STATUS_DETAIL, receiver_status_detail(d, st, this->board_, p.cut_at(millis())) +
                                   (st >= RS_STANDBY ? std::string() : receiver_status_notes(d)));
  }
  if (this->problem_ != nullptr)
    this->problem_->publish_state(receiver_status_problem(st));   // (binary_sensor publishes changes only)

  if (from(PG_STATUS | PG_ENERGY)) {   // (the same samples: the recorded Power is their 10 s mean)
    const float w = v(TV_INPUT_POWER);
    this->value_(POWER, w, all);
    this->value_(LIVE_POWER, w, all);
  }
  if (from(PG_ENERGY | PG_SYNC | PG_STATUS)) {
    // the position's Energy: through the ledger, never down (a replaced board, a restart, the ESP32's restart); its
    // ENERGY readings go in with the STATUS after them (it dates a restart). Unavailable while it doesn't answer
    // (kept in standby), as its other values
    const double wh = this->hub_->position_energy_wh(this->board_);
    this->value_(ENERGY, wh >= 0 && (answering || standby) ? static_cast<float>(wh) : NAN, all);
  }
  if (from(PG_ENERGY)) {
    this->value_(LED_SUPPLY, v(TV_LED_SUPPLY), all);       // (0.2.5) after the buck
    this->value_(CURRENT_PEAK, v(TV_CURRENT_PEAK), all);   // (0.2.5) since the ENERGY read before
  }
  if (from(PG_DIAG))
    this->value_(SUPPLY_IMPEDANCE, v(TV_SUPPLY_IMPEDANCE), all);
  if (from(PG_LIFETIME | PG_SYNC)) {
    this->value_(LIT_HOURS, v(TV_LIT_HOURS), all);
    this->value_(POWER_UPS, v(TV_POWER_UPS), all);
  }
  if (from(PG_STATUS)) {
    this->value_(BUS_VOLTAGE, v(TV_SUPPLY_VOLTAGE), all);
    this->value_(MCU_TEMPERATURE, v(TV_MCU_TEMPERATURE), all);
    this->value_(INPUT_CURRENT, v(TV_INPUT_CURRENT), all);
    this->value_(UPTIME, v(TV_UPTIME), all);   // (hourly, and at a restart)
    // along the port's cable: the drop from the sender to here, the segment before it (PortMonitor)
    const PortMonitor &m = this->hub_->port_monitor();
    this->value_(VOLTAGE_DROP, answering && !standby ? m.drop_v(this->board_) : NAN, all);
    const float r = standby ? NAN : m.resistance(this->board_);
    this->value_(CABLE_RESISTANCE, std::isnan(r) ? NAN : r * 1000.0f, all);
  }
  // trips and dips: the lifetime counts on a board with LIFETIME (0.2.5), else since its power-up
  if (from(PG_POWER | PG_LIFETIME | PG_SYNC)) {
    this->value_(TRIPS, v(TV_TRIPS), all);
    this->value_(LIMIT_EVENTS, v(TV_LIMIT_EVENTS), all);
  }
  if (from(PG_SUPPLY | PG_LIFETIME | PG_SYNC))
    this->value_(SUPPLY_DIPS, v(TV_DIPS), all);
  const uint32_t polls = p.polls(this->board_);   // (every 10 min at most: the hub's own statistic)
  this->value_(MISSED_POLLS, polls && !standby ? 100.0f * static_cast<float>(p.missed(this->board_)) / polls : NAN, all);
  if (!answering)
    return;   // the texts keep what they last said
  if (has(FIRMWARE) && from(PG_IDENTITY | PG_STATUS))
    this->text_(FIRMWARE, telemetry_firmware_text(d));
#ifdef USE_DIG2ANALOG_ULTRA_DIAG_TEXTS
  // (0.2.5: DIAG's breadcrumbs and fault memory, across power cuts; their builders are compiled only when a receiver
  // has one of these texts: diagnostics: full, or the entity given)
  if (has(LAST_RESET) && from(PG_STATUS | PG_DIAG | PG_FAULTS))
    this->text_(LAST_RESET, telemetry_last_reset(d));
  if (has(LAST_TRIP) && from(PG_POWER | PG_FAULTS))
    this->text_(LAST_TRIP, telemetry_last_trip(d));
  if (has(LAST_DIP) && from(PG_SUPPLY | PG_FAULTS))
    this->text_(LAST_DIP, telemetry_last_dip(d));
  if (has(CHANNEL_HEALTH) && from(PG_HEALTH | PG_FAULTS | PG_IDENTITY))
    this->text_(CHANNEL_HEALTH, telemetry_health_text(d));
#endif
  if (has(LINK) && all && p.has_uid(this->board_)) {
    char b[24];
    snprintf(b, sizeof(b), "%u \xC2\xB7 UID %04X", static_cast<unsigned>(this->board_ + 1),
             static_cast<unsigned>(p.uid16(this->board_)));
    this->text_(LINK, b);
  }
}

void UltraReceiver::on_shutdown() {
  const double wh = this->hub_->position_energy_wh(this->board_);
  if (wh >= 0)
    this->value_(ENERGY, static_cast<float>(wh), true);
}

void UltraReceiver::dump_config() {
  ESP_LOGCONFIG(TAG, "QuinLED Diff-Power Ultra receiver %u (GPIO%u):", this->board_ + 1, this->hub_->get_pin());
  LOG_SENSOR("  ", "Power", this->values_[POWER]);
  LOG_SENSOR("  ", "Live power", this->values_[LIVE_POWER]);
  LOG_SENSOR("  ", "Energy", this->values_[ENERGY]);
  LOG_TEXT_SENSOR("  ", "Status", this->texts_[STATUS]);
  LOG_BINARY_SENSOR("  ", "Problem", this->problem_);
}

// ----------------------------------------------------------------------------------------------------------- port
static Pace port_pace(uint8_t which, uint32_t rate_ms) {
  switch (which) {
    case UltraTelemetryPort::POWER:
      return PACE_POWER;
    case UltraTelemetryPort::LIVE_POWER:
      return live_pace(rate_ms);
    case UltraTelemetryPort::CURRENT:
      return PACE_CURRENT;
    case UltraTelemetryPort::LOAD:
    case UltraTelemetryPort::PEAK_LOAD:
      return PACE_LOAD;
    case UltraTelemetryPort::CABLE_LOSS:
      return PACE_LOSS_W;
    case UltraTelemetryPort::CABLE_LOSS_PCT:
      return PACE_LOSS_PCT;
    case UltraTelemetryPort::ENERGY:
      return PACE_ENERGY;
    default:
      return PACE_CHANGE;
  }
}

void UltraTelemetryPort::setup() {
  this->hub_->add_on_window_callback([this]() { this->update_(); });
}

void UltraTelemetryPort::value_(uint8_t which, float v, bool force) {
  const bool unpaced = this->unpaced_ & (1u << which);
  const Pace p = unpaced ? PACE_CHANGE : port_pace(which, this->hub_->update_rate_ms());
  publish_paced(this->values_[which], p, this->paced_[which], which == POWER && !unpaced ? &this->power_mean_ : nullptr,
                v, force);
}

void UltraTelemetryPort::update_() {
  const TelemetryPoller &p = this->hub_->telemetry();
  const PortMonitor &m = this->hub_->port_monitor();
  auto text = [this](uint8_t which, const std::string &v) {
    text_sensor::TextSensor *s = this->texts_[which];
    if (s != nullptr && (!s->has_state() || s->state != v))
      s->publish_state(v);
  };
  // its Diff-Power port not on: no power (Power and Current 0), nothing else known, nothing is a problem
  const bool standby = this->hub_->port_standby();
  const bool force = standby != this->standby_was_;   // (standby and back: at once)
  this->standby_was_ = standby;
  const float w = this->hub_->port_power_w();
  this->value_(POWER, w, force);
  this->value_(LIVE_POWER, w, force);
  this->value_(CURRENT, standby ? 0.0f : m.current_a(), force);
  this->value_(LOAD, standby ? NAN : m.load_pct(), force);
  this->value_(CABLE_LOSS, standby ? NAN : m.cable_loss_w(), force);
  this->value_(CABLE_LOSS_PCT, standby ? NAN : m.cable_loss_pct(), force);
  this->value_(PEAK_LOAD, standby ? NAN : m.peak_load_pct(), force);
  const double wh = this->hub_->port_energy_wh();
  if (wh >= 0)
    this->value_(ENERGY, static_cast<float>(wh), false);
  uint16_t answering = 0;
  bool problem = m.problem() && !standby;
  bool waking = false;   // (receivers not heard yet after a wake: no problem, the port Waking)
  for (uint16_t i = 0; i < p.receivers(); i++) {
    answering += p.answering(i);
    const bool settling = this->hub_->rx_settling(i);
    waking |= settling;
    problem |= !standby && !settling && receiver_status_problem(receiver_status(p.data(i), p.answering(i)));
  }
  if (this->problem_ != nullptr)
    this->problem_->publish_state(problem);
  text(RECEIVERS, std::to_string(answering) + " of " + std::to_string(p.receivers()) + " answering");
  const bool off = standby && this->hub_->port_off();
  text(STATUS, off ? std::string("Off") : standby ? std::string("Standby") : waking ? std::string("Waking")
                                                                                     : port_status(m, p));
  text(STATUS_DETAIL, off       ? std::string("The port's power mode is Off: no 55 V until another mode is chosen.")
                      : standby ? std::string("The port is in standby (4.5 V).")
                      : waking  ? std::string("The port just woke: waiting for its receivers' first answers.")
                                : port_status_detail(m));
}

void UltraTelemetryPort::on_shutdown() {
  const double wh = this->hub_->port_energy_wh();
  if (wh >= 0)
    this->value_(ENERGY, static_cast<float>(wh), true);
}

void UltraTelemetryPort::dump_config() {
  const PortMonitor::Config &c = this->hub_->port_monitor().config();
  ESP_LOGCONFIG(TAG, "QuinLED Diff-Power port (GPIO%u): %u receiver(s), limit %.2f A, warning %.0f %%, problem %.0f %% "
                "for %u s, sender %s",
                this->hub_->get_pin(), this->hub_->telemetry().receivers(), c.limit_a, c.warning_pct, c.problem_pct,
                static_cast<unsigned>(c.problem_ms / 1000), std::isnan(c.supply_v) ? "learned" : "configured");
  LOG_SENSOR("  ", "Power", this->values_[POWER]);
  LOG_SENSOR("  ", "Live power", this->values_[LIVE_POWER]);
  LOG_SENSOR("  ", "Energy", this->values_[ENERGY]);
  LOG_SENSOR("  ", "Current", this->values_[CURRENT]);
  LOG_SENSOR("  ", "Load", this->values_[LOAD]);
  LOG_BINARY_SENSOR("  ", "Problem", this->problem_);
  LOG_TEXT_SENSOR("  ", "Status", this->texts_[STATUS]);
}

// ---------------------------------------------------------------------------------------------------------- light
void UltraLightTelemetry::attach() {
  ledger_receivers() += 1;   // (its estimated energy's counter)
  this->hub_->add_on_receiver_callback(this->board_, [this](uint32_t updates) { this->update_(updates); });
  this->hub_->add_on_shutdown_callback([this]() { this->store_energy_(true); });
}

void UltraLightTelemetry::store_energy_(bool force) {
  LifetimeLedger &l = UltraHub::ledger();
  if (!l.ready())
    return;   // (the ledger loads at the first loop)
  if (!this->seeded_) {   // its stored energy: it goes on from there
    this->energy_count_.wh = std::max(this->energy_count_.wh, l.counter_mwh(this->key_) / 1000.0);
    this->seeded_ = true;
  }
  l.set_counter(this->key_, static_cast<uint64_t>(this->energy_count_.wh * 1000.0));
  publish_paced(this->energy_, (this->unpaced_ & 2) ? PACE_CHANGE : PACE_ENERGY, this->energy_paced_, nullptr,
                static_cast<float>(this->energy_count_.wh), force);
}

void UltraLightTelemetry::update_(uint32_t updates) {
  const TelemetryPoller &p = this->hub_->telemetry();
  const bool answering = p.answering(this->board_);
  const ReceiverData &d = p.data(this->board_);
  if (this->port_ != nullptr && !this->port_->has_state())   // its receiver: "Port 4 · receiver 2"
    this->port_->publish_state(std::string(this->hub_->telemetry_name()) + " \xC2\xB7 receiver " +
                               std::to_string(this->board_ + 1));
  if (this->status_ != nullptr) {
    const std::string s = receiver_status_name(
        this->hub_->port_standby()             ? (this->hub_->port_off() ? RS_PORT_OFF : RS_STANDBY)
        : this->hub_->rx_settling(this->board_) ? RS_WAKING
                                                : light_status(d, answering, this->mask_));
    if (!this->status_->has_state() || this->status_->state != s)
      this->status_->publish_state(s);
  }
  if (!(updates & (PG_OUTPUTS | PG_ENERGY | PG_STATUS | ALL_NEW)))
    return;
  const float w = this->hub_->port_standby() ? 0.0f : answering ? light_power_estimate(d, this->mask_) : NAN;
  this->energy_count_.add(w, millis());
  const bool force = (updates & ALL_NEW) != 0;
  publish_paced(this->power_, (this->unpaced_ & 1) ? PACE_CHANGE : PACE_POWER, this->power_paced_,
                (this->unpaced_ & 1) ? nullptr : &this->power_mean_, w, force);
  this->store_energy_(false);
}

// ------------------------------------------------------------------------------------------------------ the totals
void UltraTotals::setup() {
  for (UltraHub *hub : UltraHub::hubs()) {
    if (hub->has_telemetry())
      hub->add_on_window_callback([this]() { this->update_(false); });
  }
}

void UltraTotals::value_(uint8_t which, float v, bool force) {
  uint32_t rate = 0xFFFFFFFFu;
  for (UltraHub *hub : UltraHub::hubs()) {
    if (hub->has_telemetry())
      rate = std::min(rate, hub->update_rate_ms());
  }
  const bool unpaced = this->unpaced_ & (1u << which);
  const Pace p = unpaced ? PACE_CHANGE : which == POWER ? PACE_POWER : which == LIVE_POWER ? live_pace(rate) : PACE_ENERGY;
  publish_paced(this->values_[which], p, this->paced_[which], which == POWER && !unpaced ? &this->power_mean_ : nullptr,
                v, force);
}

void UltraTotals::update_(bool force) {
  float w = 0.0f;
  double wh = 0.0;
  bool known = true, energy = true;
  for (UltraHub *hub : UltraHub::hubs()) {
    if (!hub->has_telemetry())
      continue;
    const float pw = hub->port_power_w();
    known &= !std::isnan(pw);   // (a port not known yet: no total; Home Assistant falls back to the ports)
    w += std::isnan(pw) ? 0.0f : pw;
    const double e = hub->port_energy_wh();
    energy &= e >= 0;
    wh += e;
  }
  this->value_(POWER, known ? w : NAN, force);
  this->value_(LIVE_POWER, known ? w : NAN, force);
  if (energy)
    this->value_(ENERGY, static_cast<float>(wh), force);
}

void UltraTotals::dump_config() {
  ESP_LOGCONFIG(TAG, "QuinLED Ultra totals (every port at its sender):");
  LOG_SENSOR("  ", "Total power", this->values_[POWER]);
  LOG_SENSOR("  ", "Total live power", this->values_[LIVE_POWER]);
  LOG_SENSOR("  ", "Total energy", this->values_[ENERGY]);
}

}  // namespace esphome::dig2analog_ultra

#endif
