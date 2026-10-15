#include "diffpower.h"

#include "esphome/components/light/light_output.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <string>

namespace esphome::diffpower {

static const char *const TAG = "diffpower";
static constexpr uint32_t DARK_CHECK_MS = 60000;   // the dark window, once a minute (and after a mode change)
static constexpr uint32_t MINUTE_MS = 60000;       // the hours in standby: published once a minute
static constexpr uint32_t SAVE_STANDBY_S = 600;    // ... stored every 10 min of standby (cycles, drops: each change)
static constexpr uint32_t SHUTDOWN_MARGIN_MS = 100;   // the shutdown's notices: the longest notice_max_ms + this
// the lights' state again after the port is powered: at once, then twice more (boards still booting)
static const uint32_t RESEND_MS[] = {300, 1000};

std::vector<DiffPowerPort *> DiffPowerPort::ports_;
bool DiffPowerPort::shut_down_{false};

void DiffPowerPort::setup() {
  ports_.push_back(this);
  this->enable_pin_->setup();
  this->enable_pin_->digital_write(false);
  if (this->detect_pin_ != nullptr)
    this->detect_pin_->setup();
  this->stats_pref_ =
      global_preferences->make_preference<Stats>(fnv1_hash(std::string("diffpower:") + this->stats_key_));
  Stats s{};
  if (this->stats_pref_.load(&s))
    this->core_.set_counters(s.cycles, s.standby_s, s.drops);
  this->saved_standby_s_ = this->core_.standby_s();
  this->saved_cycles_ = this->core_.standby_cycles();
  this->saved_drops_ = this->core_.drops();
  this->publish_();
}

void DiffPowerPort::dump_config() {
  const PortConfig &c = this->core_.config();
  static const char *const MODES[MODE_COUNT] = {"Standby", "During dark", "Always on"};
  ESP_LOGCONFIG(TAG,
                "Diff-Power port '%s':\n"
                "  Power mode: %s, turn-off period %" PRIu32 " s, on at least %" PRIu32 " s\n"
                "  Hot-plug %" PRIu32 " ms + boot %" PRIu32 " ms, notice %" PRIu32 " ms, off at least %" PRIu32 " ms\n"
                "  %u load(s); %" PRIu32 " standby cycles, %" PRIu32 " detect drops so far",
                this->name_, MODES[this->core_.mode()], c.standby_after_ms / 1000, c.min_on_ms / 1000, c.hotplug_ms,
                c.boot_margin_ms, c.notice_ms, c.min_off_ms, static_cast<unsigned>(this->loads_.size()),
                this->core_.standby_cycles(), this->core_.drops());
  LOG_PIN("  Enable: ", this->enable_pin_);
  LOG_PIN("  Detect: ", this->detect_pin_);
#ifdef USE_DIFFPOWER_SUN
  if (this->sun_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  During dark: %" PRIu32 " min before sunset to %" PRIu32 " min after sunrise; no time: %s",
                  this->before_s_ / 60, this->after_s_ / 60, c.unknown_on ? "Always on" : "Standby");
  }
#endif
}

bool DiffPowerPort::has_sun_() const {
#ifdef USE_DIFFPOWER_SUN
  return this->sun_ != nullptr;
#else
  return false;
#endif
}

// "During dark": the sun's elevation before_s ahead and after_s back (diffpower_core.h); no valid time: unknown
void DiffPowerPort::update_dark_(uint32_t now) {
#ifdef USE_DIFFPOWER_SUN
  if (this->sun_ == nullptr || (this->dark_checked_ && now - this->dark_ms_ < DARK_CHECK_MS))
    return;
  this->dark_checked_ = true;
  this->dark_ms_ = now;
  const ESPTime t = this->sun_->get_time()->now();
  const bool known = t.is_valid();
  const bool dark = known && dark_window(static_cast<double>(t.timestamp), this->lat_, this->lon_, this->before_s_,
                                         this->after_s_);
  if (dark != this->core_.dark()) {
    ESP_LOGD(TAG, "Port '%s': %s the dark window", this->name_, dark ? "in" : "out of");
  }
  this->core_.set_dark(dark, known);
#endif
}

void DiffPowerPort::loop() {
  const uint32_t now = millis();
  this->core_.set_detect(this->detect_pin_ == nullptr || this->detect_pin_->digital_read(), now);
  for (uint16_t i = 0; i < this->loads_.size(); i++)
    this->core_.set_lit(i, this->loads_[i].lit(now), now);
  this->update_dark_(now);
  if (this->core_.preparing()) {   // a planned power-off next: its hubs' last reads first
    bool ready = true;
    for (auto &cb : this->prepare_cb_)
      ready &= cb(this->prepare_first_);
    this->prepare_first_ = false;
    this->core_.set_notice_ready(ready);
  }
  this->apply_(this->core_.update(now), now);
  if (now - this->minute_ms_ >= MINUTE_MS) {   // the hours in standby
    this->minute_ms_ = now;
    if (this->core_.standby_s() - this->saved_standby_s_ >= SAVE_STANDBY_S)
      this->save_stats_();
    this->publish_();
  }
}

void DiffPowerPort::apply_(uint8_t ev, uint32_t now) {
  if (ev & EV_NOTICE) {
    const uint8_t copy = this->core_.notice_copy();
    if (copy == 0) {
      ESP_LOGD(TAG, "Port '%s': the planned power-off notice", this->name_);
    }
    this->notice_cb_.call(copy);
    if (this->notice_cb_.size() == 0)
      this->core_.notice_sent(now);   // (nobody to tell: plain lights)
  }
  if (ev & EV_PREPARE)
    this->prepare_first_ = true;
  if (ev & EV_ENABLE)
    this->enable_pin_->digital_write(this->core_.enable());
  if (ev & EV_LOST) {
    ESP_LOGW(TAG, "Port '%s': the board's detect went away while powered (unplugged, a cable fault?)", this->name_);
  }
  if (ev & EV_COUNTERS)
    this->save_stats_();
  if (ev & EV_READY) {
    ESP_LOGD(TAG, "Port '%s': powered, the lights' state sent again", this->name_);
    this->resend_();
    for (uint32_t ms : RESEND_MS)
      this->set_timeout(ms, [this]() {
        if (this->core_.ready())
          this->resend_();
      });
    this->ready_cb_.call();
  }
  if (ev & (EV_STATE | EV_COUNTERS)) {
    if (ev & EV_STATE) {
      char buf[32];
      ESP_LOGD(TAG, "Port '%s': %s", this->name_,
               port_state_text(this->core_.state(), this->core_.drops(), buf, sizeof(buf)));
    }
    this->publish_();
  }
}

// The plain lights' data sent before the 55 V came is lost: their current state again (an Ultra hub sends its own)
void DiffPowerPort::resend_() {
  for (const auto &l : this->loads_) {
    if (l.resend && l.light != nullptr && l.light->get_output() != nullptr)
      l.light->get_output()->write_state(l.light);
  }
}

void DiffPowerPort::save_stats_() {
  Stats s{this->core_.standby_cycles(), this->core_.standby_s(), this->core_.drops()};
  this->stats_pref_.save(&s);
  this->saved_standby_s_ = s.standby_s;
  this->saved_cycles_ = s.cycles;
  this->saved_drops_ = s.drops;
}

void DiffPowerPort::publish_() {
#ifdef USE_TEXT_SENSOR
  if (this->status_ != nullptr) {
    char buf[32];
    const std::string s = port_state_text(this->core_.state(), this->core_.drops(), buf, sizeof(buf));
    if (!this->status_->has_state() || this->status_->state != s)
      this->status_->publish_state(s);
  }
#endif
#ifdef USE_BINARY_SENSOR
  if (this->problem_ != nullptr)
    this->problem_->publish_state(this->core_.problem());
#endif
#ifdef USE_SENSOR
  auto value = [](sensor::Sensor *s, float v) {
    if (s != nullptr && (!s->has_state() || s->state != v))
      s->publish_state(v);
  };
  value(this->cycles_, static_cast<float>(this->core_.standby_cycles()));
  value(this->hours_, static_cast<float>(this->core_.standby_s() / 36) / 100.0f);
  value(this->drops_, static_cast<float>(this->core_.drops()));
#endif
}

// OTA / a reboot (synchronous: the loop doesn't run any more): every port in ON starts its notice, one already in
// NOTICE goes on with its own. All ports are served together, without waiting on any one: the hubs send what is
// queued when their line is free, each port asks for its next copy 50 ms after the last one went out and drops its
// enable 100 ms after the third (or 1 s after its notice at the latest). Then every enable low, the counters saved.
void DiffPowerPort::shutdown_all() {
  if (shut_down_)
    return;
  shut_down_ = true;
  const uint32_t t0 = millis();
  uint32_t limit = 0;
  for (auto *p : ports_) {
    p->apply_(p->core_.shutdown(t0), t0);
    limit = std::max(limit, p->core_.config().notice_max_ms + SHUTDOWN_MARGIN_MS);
  }
  for (bool busy = true; busy && millis() - t0 < limit;) {
    busy = false;
    for (auto *p : ports_) {
      if (p->core_.state() != PS_NOTICE)
        continue;
      p->pump_cb_.call();
      const uint32_t now = millis();
      p->apply_(p->core_.update(now), now);
      busy |= p->core_.state() == PS_NOTICE;
    }
    if (busy) {
      delay(1);
      App.feed_wdt();
    }
  }
  for (auto *p : ports_) {
    p->apply_(p->core_.power_down(millis()), millis());
    if (p->core_.standby_s() != p->saved_standby_s_ || p->core_.standby_cycles() != p->saved_cycles_ ||
        p->core_.drops() != p->saved_drops_)
      p->save_stats_();   // (the standby time since the last 10-min save)
  }
  global_preferences->sync();
}

void DiffPowerPort::wake(uint32_t timeout_ms) {
  ESP_LOGD(TAG, "Port '%s': pre-wake for %" PRIu32 " s", this->name_, timeout_ms / 1000);
  this->core_.prewake(millis(), timeout_ms);
}

bool DiffPowerPort::has_light(light::LightState *light) const {
  for (const auto &l : this->loads_) {
    if (l.light == light)
      return true;
  }
  return false;
}

#ifdef USE_SELECT
uint8_t PowerModeSelect::mode_of_(size_t index) const {
  if (!this->port_->has_sun() && index >= MODE_DURING_DARK)
    return static_cast<uint8_t>(index + 1);   // (Standby, Always on)
  return static_cast<uint8_t>(index);
}

size_t PowerModeSelect::index_of_(uint8_t mode) const {
  if (!this->port_->has_sun() && mode >= MODE_DURING_DARK)
    return mode == MODE_DURING_DARK ? 0 : mode - 1;
  return mode;
}

void PowerModeSelect::setup() {
  this->pref_ = this->make_entity_preference<uint32_t>();
  uint32_t stored;
  // the stored mode if this select still offers it (During dark without a sun any more: the YAML's mode, stored)
  if (this->pref_.load(&stored) && stored < MODE_COUNT) {
    if (stored == MODE_DURING_DARK && !this->port_->has_sun()) {
      stored = this->port_->get_mode();
      this->pref_.save(&stored);
    } else {
      this->port_->set_mode(static_cast<uint8_t>(stored));
    }
  }
  this->publish_state(this->index_of_(this->port_->get_mode()));
}

void PowerModeSelect::control(size_t index) {
  const uint8_t mode = this->mode_of_(index);
  uint32_t stored = mode;
  this->pref_.save(&stored);
  this->port_->set_mode(mode);
  this->publish_state(index);
}
#endif

}  // namespace esphome::diffpower
