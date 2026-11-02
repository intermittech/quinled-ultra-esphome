// Telemetry for Diff-Power Ultra Receiver Analog-Medium and Diff-Power Ultra Receiver Digital-Medium.
#include "ultra_receiver.h"

#if defined(USE_ESP32) && defined(USE_ULTRA_LINE_TELEMETRY)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::ultra_line {

static const char *const TAG = "ultra_line.receiver";

static constexpr uint32_t ALL_NEW = TU_AVAILABILITY | TU_RESTART;
static constexpr uint32_t PG_STATUS = 1u << TP_STATUS, PG_POWER = 1u << TP_POWER, PG_ENERGY = 1u << TP_ENERGY,
                          PG_SUPPLY = 1u << TP_SUPPLY, PG_OUTPUTS = 1u << TP_OUTPUTS, PG_HEALTH = 1u << TP_HEALTH,
                          PG_IDENTITY = 1u << TP_IDENTITY | 1u << TP_FEATURES, PG_LIFETIME = 1u << TP_LIFETIME,
                          PG_DIAG = 1u << TP_DIAG, PG_FAULTS = 1u << TP_FAULTS, PG_CALIBRATION = 1u << TP_CALIBRATION,
                          PG_SYNC = 1u << TP_SYNC;

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
    case UltraReceiver::FULL_LOAD_VOLTAGE:
      return PACE_VOLTAGE;
    case UltraReceiver::FULL_LOAD_DROP:
      return PACE_DROP_PCT;
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
  // its settings (the hub's first loop has applied the ones from Home Assistant; each change since)
  if (this->texts_[SETTINGS] != nullptr) {
    this->hub_->add_on_settings_callback([this](int board) {
      if (board < 0 || board == this->board_)
        this->text_(SETTINGS, this->hub_->receiver_settings_text(this->board_));
    });
  }
}

void UltraReceiver::value_(uint8_t which, float v, bool force) {
  const bool unpaced = this->unpaced_ & (1u << which);
  const Pace p = unpaced ? PACE_CHANGE : receiver_pace(which, this->hub_->update_rate_ms());
  publish_paced(this->values_[which], p, this->paced_[which], which == POWER && !unpaced ? &this->power_mean_ : nullptr,
                v, force);
}

void UltraReceiver::text_(uint8_t which, const std::string &v) {
  text_sensor::TextSensor *t = this->texts_[which];
  const std::string s = text_state(v);   // (Home Assistant's 255 characters)
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
  if (this->rate_request_ != nullptr) {
    const auto &s = d.line_counters;
    if (answering && !standby && s.coherent && millis() - s.sampled_ms <= 2000 &&
        (s.flags & ULTRA_LS_VALID))
      this->rate_request_->publish_state(receiver_requests_lower_rate(s));
    else
      this->rate_request_->invalidate_state();
  }
  if (this->output_voltage_low_ != nullptr) {
    if (answering && !standby && d.output_voltage_low.known)
      this->output_voltage_low_->publish_state(d.output_voltage_low.fault);
    else
      this->output_voltage_low_->invalidate_state();
  }
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

  // the status: worst first, with its plain line and the Problem (just woken, not heard yet: Waking; in the hub's
  // calibration run: Calibrating)
  const ReceiverStatus st = with_hub_calibration(
      standby                                 ? (this->hub_->port_off() ? RS_PORT_OFF : RS_STANDBY)
      : this->hub_->rx_settling(this->board_) ? RS_WAKING
                                              : receiver_status(d, answering, 0, p.dip_warning()),
      this->hub_->calibrating(this->board_));
  // (a text is built only for its entity: a level the YAML leaves out costs nothing at run time)
  auto has = [this](uint8_t which) { return this->texts_[which] != nullptr; };
  this->text_(STATUS, receiver_status_name(st));
  if (has(STATUS_DETAIL)) {
    this->text_(STATUS_DETAIL, receiver_status_detail(d, st, this->board_, p.cut_at(millis())) +
                                   (st >= RS_STANDBY ? std::string() : receiver_status_notes(d)));
  }
  const auto &count_cal = p.pixel_calibration(this->board_);
  const bool count_problem = answering && !standby && count_cal.count_problem();
  if (has(STATUS_DETAIL) && count_problem && !receiver_status_problem(st))
    this->text_(STATUS_DETAIL, count_cal.count_text());
  if (this->problem_ != nullptr)
    this->problem_->publish_state(receiver_status_problem(st) || count_problem);   // (binary_sensor publishes changes only)

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
    this->value_(LED_SUPPLY, v(TV_LED_SUPPLY), all);       //  at the LED output
    this->value_(CURRENT_PEAK, v(TV_CURRENT_PEAK), all);   //  since the ENERGY read before
    this->value_(SUPPLY_IMPEDANCE, v(TV_SUPPLY_IMPEDANCE), all);
  }
  // the cable drop at full load (Z x its full-load current): what its Z, bus voltage and current, limit and model give
  const bool cable_inputs = from(PG_STATUS | PG_ENERGY | PG_POWER | PG_CALIBRATION);
  if (cable_inputs) {
    const CableDrop cd = cable_drop(d);
    const bool known = answering && !standby && cd.known;
    this->value_(FULL_LOAD_VOLTAGE, known ? cd.full_v : NAN, all);
    this->value_(FULL_LOAD_DROP, known ? cd.drop_pct : NAN, all);
  }
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
  // trips and dips: the lifetime counts on a board with LIFETIME , else since its power-up
  if (from(PG_POWER | PG_LIFETIME | PG_SYNC)) {
    this->value_(TRIPS, v(TV_TRIPS), all);
    this->value_(LIMIT_EVENTS, v(TV_LIMIT_EVENTS), all);
  }
  if (from(PG_SUPPLY | PG_LIFETIME | PG_SYNC))
    this->value_(SUPPLY_DIPS, v(TV_DIPS), all);
  const uint32_t polls = p.polls(this->board_);   // (every 10 min at most: the hub's own statistic)
  this->value_(MISSED_POLLS, polls && !standby ? 100.0f * static_cast<float>(p.missed(this->board_)) / polls : NAN, all);
  // CALIBRATION changes and availability only; invalidate before the no-answer early return.
  const bool digital = (d.hw_caps & RX_CAP_PIXEL) != 0;
  const PixelCalibration &uc = p.pixel_calibration(this->board_);
  const bool digital_update = digital && (all || (updates & (PG_CALIBRATION | PG_STATUS | PG_ENERGY |
                            (1u << TP_UD_META) | (1u << TP_UD_QUALITY) | TU_HUB)));
  if (!digital && (all || (updates & (PG_CALIBRATION | TU_HUB)))) {
    this->value_(CONFIGURED_PIXELS, NAN);
    this->value_(DETECTED_PIXELS, NAN);
    this->value_(USED_PIXELS, NAN);
    this->text_(PIXEL_COUNT, "Pixel count unknown");
  }
  if (digital_update) {
    const bool counts = answering && !standby && uc.have;
    this->value_(CONFIGURED_PIXELS, counts ? uc.get(6) : NAN);
    this->value_(DETECTED_PIXELS, counts && uc.get(8) ? uc.get(8) : NAN);
    this->value_(USED_PIXELS, counts && uc.get(10) ? uc.get(10) : NAN);
    if (has(PIXEL_COUNT)) this->text_(PIXEL_COUNT, counts ? uc.count_text() : "Pixel count unknown");
    this->text_(CALIBRATION, this->hub_->calibrating(this->board_) && (!uc.have || uc.meta[2] >= ULTRA_UCS_DONE) ? "Queued" : uc.result_text());
    this->text_(CURRENT_MODEL, answering && !standby && uc.usable() ? "Calibrated" : "Default");
    if (has(POWER_MODEL)) this->text_(POWER_MODEL, uc.json(answering && !standby));
    const bool known = answering && !standby && uc.usable();
    for (uint8_t i = 0; i < 5; i++)
      this->value_(CH1_FULL_SCALE + i, known ? uc.channel_ma(i) / 1000.0f : NAN);
    this->value_(IDLE_CURRENT, known ? uc.idle_ma() / 1000.0f : NAN);
  }
  if (!digital && from(PG_CALIBRATION)) {
    const bool known = answering && !standby && d.have_calibration && (d.cal_model == 1 || d.cal_model == 2);
    for (uint8_t i = 0; i < 5; i++)
      this->value_(CH1_FULL_SCALE + i, known ? d.cal_channel_ma[i] / 1000.0f : NAN);
    this->value_(IDLE_CURRENT, known ? d.cal_idle_ma / 1000.0f : NAN);
    if (has(POWER_MODEL)) {
      char json[84];   // max 83 characters plus NUL, including signed idle and five u16 currents
      snprintf(json, sizeof(json),
               "{\"v\":1,\"src\":\"%s\",\"idle_ma\":%d,\"ch_ma\":[%u,%u,%u,%u,%u]}",
               known ? (d.cal_model == 2 ? "calibrated" : "learned") : "none",
               known ? static_cast<int>(d.cal_idle_ma) : 0,
               known ? static_cast<unsigned>(d.cal_channel_ma[0]) : 0u,
               known ? static_cast<unsigned>(d.cal_channel_ma[1]) : 0u,
               known ? static_cast<unsigned>(d.cal_channel_ma[2]) : 0u,
               known ? static_cast<unsigned>(d.cal_channel_ma[3]) : 0u,
               known ? static_cast<unsigned>(d.cal_channel_ma[4]) : 0u);
      this->text_(POWER_MODEL, json);
    }
  }
  if (!answering)
    return;   // the texts keep what they last said
  if (has(FIRMWARE) && from(PG_IDENTITY | PG_STATUS))
    this->text_(FIRMWARE, telemetry_firmware_text(d));
  // its supply dips in the last 24 h, against the line's dip_warning (the receiver page's note); a board without the
  // SUPPLY page: whether STATUS says a dip in the last 30 s
  if (has(SUPPLY_NOTE) && from(PG_SUPPLY | PG_IDENTITY | PG_STATUS))
    this->text_(SUPPLY_NOTE, telemetry_supply_note(d, p.dip_warning()));
  // the cable drop advisor's line (its inputs: the cable_inputs pages, and its identity: the board's kind)
  if (has(CABLE_DROP) && (cable_inputs || from(PG_IDENTITY)))
    this->text_(CABLE_DROP, telemetry_cable_drop(d, p.dip_warning()));
  // its current calibration (CALIBRATION, with the hub's record: observed before this call): how its last test ended,
  // the model in use, when its last complete test ended, the connected load (the channels together at 100 %)
  if (!digital && d.have_calibration && from(PG_CALIBRATION)) {
    const CalibrationLog &cl = this->hub_->calibration_log();
    this->text_(CALIBRATION, cl.result(this->board_, d));
    this->text_(CURRENT_MODEL, calibration_model_text(d.cal_model));
    const uint32_t ok = cl.last_unix(this->board_);
    this->value_(LAST_CALIBRATED, ok != 0 ? static_cast<float>(ok) : NAN, all);
    this->value_(CONNECTED_LOAD, d.cal_connected_ma != 0 ? d.cal_connected_ma / 1000.0f : NAN, all);
  }
#ifdef USE_ULTRA_LINE_DIAG_TEXTS
  // (DIAG's breadcrumbs and fault memory, across power cuts; their builders are compiled only when a receiver
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
  ESP_LOGCONFIG(TAG, "Diff-Power-UR-Analog-M / Diff-Power-UR-Digital-M %u (GPIO%u):", this->board_ + 1, this->hub_->get_pin());
  LOG_SENSOR("  ", "Power", this->values_[POWER]);
  LOG_SENSOR("  ", "Live power", this->values_[LIVE_POWER]);
  LOG_SENSOR("  ", "Energy", this->values_[ENERGY]);
  LOG_TEXT_SENSOR("  ", "Status", this->texts_[STATUS]);
  LOG_BINARY_SENSOR("  ", "Problem", this->problem_);
  LOG_BINARY_SENSOR("  ", "Output voltage low / fuse", this->output_voltage_low_);
  LOG_BINARY_SENSOR("  ", "Lower line rate requested", this->rate_request_);
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
  if (this->hub_->has_commissioning())
    this->hub_->add_on_commission_callback([this]() { this->publish_commission_(); });
}

void UltraTelemetryPort::publish_commission_() {
  const Commissioning &c = this->hub_->commissioning();
  const std::string status = text_state(c.status()), report = text_state(c.report());
  if (this->comm_status_ != nullptr && (!this->comm_status_->has_state() || this->comm_status_->state != status))
    this->comm_status_->publish_state(status);
  if (this->comm_report_ != nullptr && (!this->comm_report_->has_state() || this->comm_report_->state != report))
    this->comm_report_->publish_state(report);
  if (this->commissioned_ != nullptr)
    this->commissioned_->publish_state(c.commissioned(this->hub_->telemetry()));
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
  auto text = [this](uint8_t which, const std::string &value) {
    text_sensor::TextSensor *s = this->texts_[which];
    const std::string v = text_state(value);   // (Home Assistant's 255 characters)
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
    const ReceiverStatus st = receiver_status(p.data(i), p.answering(i), 0, p.dip_warning());
    problem |= !standby && !settling && (receiver_status_problem(with_hub_calibration(st, this->hub_->calibrating(i))) ||
                                         p.pixel_calibration(i).count_problem());
  }
  if (this->problem_ != nullptr)
    this->problem_->publish_state(problem);
  // the 36 V rules of the port (a text and its own problem: the port's Problem keeps its meaning)
  this->rules_.update(p, m, standby);
  text(LOAD_RULES, this->rules_.text());
  if (this->rules_problem_ != nullptr)
    this->rules_problem_->publish_state(this->rules_.problem());
  text(LINE_PROFILE, this->hub_->line_profile_text());
  text(RECEIVERS, std::to_string(answering) + " of " + std::to_string(p.receivers()) + " answering");
  const bool off = standby && this->hub_->port_off();
  text(STATUS, off ? std::string("Off") : standby ? std::string("Standby") : waking ? std::string("Waking")
                                                                                     : port_status(m, p));
  text(STATUS_DETAIL, off       ? std::string("The port's power mode is Off: no 55 V until another mode is chosen.")
                      : standby ? std::string("The port is in standby (4.5 V).")
                      : waking  ? std::string("The port just woke: waiting for its receivers' first answers.")
                                : port_status_detail(m));
  // the receivers in cable order for the setup generator (changes with a receiver found, replaced or tested)
  if (this->texts_[SETUP_INFO] != nullptr) {
    const uint32_t sig = telemetry_setup_sig(p);   // (integer work per window; the text only when it changed)
    if (!this->setup_sig_set_ || sig != this->setup_sig_) {
      this->setup_sig_ = sig;
      this->setup_sig_set_ = true;
      text(SETUP_INFO, telemetry_setup_info(p));
    }
  }
  this->publish_health_();
}

// The line's telemetry health, once a minute (after a window: none while the port is in standby, nothing changes then)
void UltraTelemetryPort::publish_health_() {
  const uint32_t now = millis();
  if (!this->has_health_ || (this->health_seen_ && now - this->health_ms_ < HEALTH_MS))
    return;
  this->health_seen_ = true;
  this->health_ms_ = now;
  const TelemetryPoller::Stats &st = this->hub_->telemetry().stats();
  auto put = [this](uint8_t which, float v) {
    sensor::Sensor *s = this->health_[which];
    if (s != nullptr && (!s->has_state() || s->state != v))
      s->publish_state(v);
  };
  put(H_WINDOWS, static_cast<float>(st.windows));
  const uint32_t missing = st.missing - this->missing_was_, asked = st.answers - this->answers_was_ + missing;
  if (asked != 0)   // (none asked for in that minute: unchanged)
    put(H_MISSING, 100.0f * static_cast<float>(missing) / static_cast<float>(asked));
  this->answers_was_ = st.answers;
  this->missing_was_ = st.missing;
  put(H_BROKEN, static_cast<float>(st.bad_crc));
  put(H_CONTENTION, static_cast<float>(st.contention));
  put(H_RESTARTS, static_cast<float>(st.restarts));
  put(H_LATE, static_cast<float>(this->hub_->late_releases()));
  static const float KBIT[] = {1000.0f, 500.0f, 250.0f};
  put(H_RATE, KBIT[this->hub_->reply_rate().rate() % ULTRA_RW_RATES]);
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
  LOG_BINARY_SENSOR("  ", "Load rules problem", this->rules_problem_);
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
    const std::string s = receiver_status_name(with_hub_calibration(
        this->hub_->port_standby()             ? (this->hub_->port_off() ? RS_PORT_OFF : RS_STANDBY)
        : this->hub_->rx_settling(this->board_) ? RS_WAKING
                                                : light_status(d, answering, this->mask_),
        this->hub_->calibrating(this->board_)));
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

// ------------------------------------------------------------------------------ the controller's settings entities
void apply_controller_setting(uint8_t kind, float v) {
  for (UltraHub *hub : UltraHub::hubs()) {
    if (!hub->has_telemetry())
      continue;
    switch (kind) {
      case CS_UPDATE_RATE:
        hub->set_update_rate_ms(UPDATE_RATE_MS[static_cast<size_t>(v) < 3 ? static_cast<size_t>(v) : 0]);
        break;
      case CS_ADAPTIVE_RATE:
        hub->set_telemetry_adaptive_rate(v != 0.0f);
        break;
      case CS_LOAD_WARNING:
        hub->set_port_levels(v, NAN, UINT32_MAX, NAN);
        break;
      case CS_LOAD_PROBLEM:
        hub->set_port_levels(NAN, v, UINT32_MAX, NAN);
        break;
      case CS_LOAD_PROBLEM_TIME:
        hub->set_port_levels(NAN, NAN, static_cast<uint32_t>(v * 1000.0f + 0.5f), NAN);
        break;
      case CS_PORT_LIMIT:
        hub->set_port_levels(NAN, NAN, UINT32_MAX, v);
        break;
      case CS_DIP_WARNING:
        hub->set_telemetry_dip_warning(static_cast<uint8_t>(v + 0.5f));
        break;
      case CS_COMMISSION_MIN_V:
        hub->set_commission_min_voltage(v / 100.0f);
        break;
      default:   // (Supply rating: for the dashboards only)
        break;
    }
  }
}

void SettingStore::init(uint8_t kind, float yaml) {
  this->yaml_ = yaml;
  this->pref_ = global_preferences->make_preference<SettingRecord>(fnv1_hash("ultra_line_setting") ^ kind);
  SettingRecord r;
  const bool loaded = this->pref_.load(&r);
  this->boot_ = setting_boot_value(r, loaded, yaml);
}

void SettingStore::store(float value) {
  SettingRecord r;
  r.magic = SettingRecord::MAGIC;
  r.yaml = this->yaml_;
  r.value = value;
  this->pref_.save(&r);
}

#ifdef USE_NUMBER
void UltraSettingNumber::setup() {
  this->store_.init(this->kind_, this->yaml_);
  const float v = this->store_.boot_value();
  if (v != this->yaml_)   // (the YAML's value is in the lines already)
    apply_controller_setting(this->kind_, v);
  this->publish_state(v);
}
void UltraSettingNumber::control(float value) {
  apply_controller_setting(this->kind_, value);
  this->store_.store(value);
  this->publish_state(value);
}
#endif
#ifdef USE_SELECT
void UltraSettingSelect::setup() {
  this->store_.init(this->kind_, this->yaml_);
  const float v = this->store_.boot_value();
  if (v != this->yaml_)
    apply_controller_setting(this->kind_, v);
  this->publish_state(static_cast<size_t>(v));
}
void UltraSettingSelect::control(size_t index) {
  apply_controller_setting(this->kind_, static_cast<float>(index));
  this->store_.store(static_cast<float>(index));
  this->publish_state(index);
}
#endif
#ifdef USE_SWITCH
void UltraSettingSwitch::setup() {
  this->store_.init(this->kind_, this->yaml_);
  const float v = this->store_.boot_value();
  if (v != this->yaml_)
    apply_controller_setting(this->kind_, v);
  this->publish_state(v != 0.0f);
}
void UltraSettingSwitch::write_state(bool state) {
  apply_controller_setting(this->kind_, state ? 1.0f : 0.0f);
  this->store_.store(state ? 1.0f : 0.0f);
  this->publish_state(state);
}
#endif

void UltraTotals::dump_config() {
  ESP_LOGCONFIG(TAG, "QuinLED Ultra totals (every port at its sender):");
  LOG_SENSOR("  ", "Total power", this->values_[POWER]);
  LOG_SENSOR("  ", "Total live power", this->values_[LIVE_POWER]);
  LOG_SENSOR("  ", "Total energy", this->values_[ENERGY]);
}

}  // namespace esphome::ultra_line

#endif
