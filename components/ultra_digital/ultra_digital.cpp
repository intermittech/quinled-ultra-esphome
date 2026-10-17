#include "ultra_digital.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace esphome::ultra_digital {

static const char *const TAG = "ultra_digital";

using dig2analog_ultra::pixel_fx_of;
using dig2analog_ultra::pixel_fx_same;
using dig2analog_ultra::PixelState;

std::vector<PixelLight *> PixelLight::registry_;

/// the effect keeps its phase this long; then it gets a new anchor (START is an int32 ms from the header TIME: 24.8 days)
static constexpr int64_t REANCHOR_US = 20LL * 86400 * 1000000;
/// a change of speed, colours, ... ramps over this long on the receiver
static constexpr uint32_t PARAM_RAMP_MS = 250;

PixelLight *PixelLight::find(light::LightOutput *output) {
  for (auto *l : registry_) {
    if (static_cast<light::LightOutput *>(l) == output)
      return l;
  }
  return nullptr;
}

// ------------------------------------------------------------------------------------------------- the transition
PixelTransition::~PixelTransition() { this->light_->transition_done(this); }

void PixelTransition::start() {
  // ESPHome's own start (off -> on keeps the target colours, on -> off the old ones); the receiver gets the end values
  // and the length, and ramps itself
  light::LightTransitionTransformer::start();
  this->light_->transition_requested(this->length_);
  this->light_->request_send_();
}

void PixelTransition::stop() { this->light_->transition_done(this); }

void PixelLight::transition_requested(uint32_t length_ms) {
  this->transition_ms_ = length_ms > 0xFFFF ? 0xFFFF : length_ms;
}

void PixelLight::transition_done(PixelTransition *t) {
  if (this->transition_ == t)
    this->transition_ = nullptr;
}

std::unique_ptr<light::LightTransformer> PixelLight::create_default_transition() {
  auto t = std::make_unique<PixelTransition>(this);
  this->transition_ = t.get();
  return t;
}

// --------------------------------------------------------------------------------------------------- the effects
PixelLight *PixelFxEffect::light_() const { return PixelLight::find(this->state_->get_output()); }

void PixelFxEffect::start() {
  PixelLight *l = this->light_();
  if (l == nullptr) {
    ESP_LOGE(TAG, "'%s': %s only runs on an ultra_digital light", this->state_->get_name().c_str(), this->name_);
    return;
  }
  l->effect_started(this);
}

void PixelFxEffect::stop() {
  PixelLight *l = this->light_();
  if (l != nullptr)
    l->effect_stopped(this);
}

void PixelLight::effect_started(PixelFxEffect *effect) {
  this->chosen_ = effect;
  this->effect_ = effect->id();
  this->effect_changed_ = true;
  this->request_send_();
}

void PixelLight::effect_stopped(PixelFxEffect *effect) {
  if (this->chosen_ != effect)
    return;   // another one started already
  this->chosen_ = nullptr;
  this->effect_ = this->solid_;
  this->effect_changed_ = true;
  this->request_send_();
}

// ---------------------------------------------------------------------------------------------------- the light
light::LightTraits PixelLight::get_traits() {
  auto traits = light::LightTraits();
  // the colour is c1; the white of RGBW and RGBCCT strips comes from the auto-white mode on the receiver
  traits.set_supported_color_modes({light::ColorMode::RGB});
  return traits;
}

void PixelLight::setup_state(light::LightState *state) {
  this->state_ = state;
  registry_.push_back(this);
  state->add_remote_values_listener(this);   // (before LightState::setup() restores: the first update is the boot state)
}

void PixelLight::setup() {
  this->hub_->set_pixel_board(this->board_);
  for (const Cfg &c : this->cfg_)
    this->hub_->set_pixel_config(this->board_, c.id, c.value);
  this->hub_->add_pixel_light(this);
  for (light::LightState *other : this->others_)
    other->add_remote_values_listener(new ExclusiveWatcher(this, other));   // NOLINT (lives as long as the program)
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->hub_->has_telemetry() && !this->page_hooked_) {
    this->page_hooked_ = true;
    this->hub_->add_on_receiver_callback(this->board_, [this](uint32_t updates) {
      constexpr uint32_t PX = (1u << dig2analog_ultra::TP_PX_STATE) | (1u << dig2analog_ultra::TP_PX_EFFECTS) |
                              (1u << dig2analog_ultra::TP_PX_PALETTES) | (1u << dig2analog_ultra::TP_FEATURES) |
                              dig2analog_ultra::TU_AVAILABILITY;
      if (updates & PX)
        this->pixel_page_();
    });
  }
#endif
}

void PixelLight::dump_config() {
  ESP_LOGCONFIG(TAG, "Pixel light '%s': board %u, segment %u (pixels %u + %u), solid effect %u, %u effects",
                this->state_ != nullptr ? this->state_->get_name().c_str() : "?", this->board_, this->segment_,
                this->state_in_.seg_start, this->state_in_.seg_len, this->solid_,
                static_cast<unsigned>(this->effects_.size()));
  for (const Cfg &c : this->cfg_)
    ESP_LOGCONFIG(TAG, "  CONFIG %u = %u", c.id, c.value);
  for (light::LightState *o : this->others_)
    ESP_LOGCONFIG(TAG, "  exclusive with '%s'", o->get_name().c_str());
}

void PixelLight::on_light_remote_values_update() {
  // this light switched on: the streaming lights of the strip go off (one picture at a time)
  const bool on = this->state_->remote_values.is_on();
  if (on && !this->first_loop_) {
    for (light::LightState *o : this->others_) {
      if (o->remote_values.is_on() || o->current_values.is_on())
        o->make_call().set_state(false).set_transition_length(0).perform();
    }
  }
  this->last_change_ms_ = millis();
  this->store_pending_ = this->store_after_ms_ != 0;
}

void ExclusiveWatcher::on_light_remote_values_update() { this->owner_->other_changed_(this->other_); }

void PixelLight::other_changed_(light::LightState *other) {
  if (!other->remote_values.is_on() || this->released_)
    return;
  if (!this->sent_)
    return;   // (the receiver isn't in local mode: nothing to hand back)
  // the stream light switched on: the strip is handed back to it (the hub's pixels pass), ours goes off
  ESP_LOGD(TAG, "'%s' switched on: '%s' releases the strip to the stream", other->get_name().c_str(),
           this->state_->get_name().c_str());
  this->release_();
  if (this->state_->remote_values.is_on())
    this->state_->make_call().set_state(false).set_transition_length(0).perform();
}

void PixelLight::release_() {
  ultra_pixel_fx_t f;
  std::memset(&f, 0, sizeof f);
  f.effect = ULTRA_PX_FX_STREAM;
  this->hub_->scheduler().light_pixel_fx(this->board_, this->segment_, f, this->hub_->start_time_us(this->board_));
  this->hub_->wake_();
  this->released_ = true;
  this->sent_ = false;
  this->dirty_ = false;
  this->on_ = false;
  this->fading_out_ = false;
}

// Instant changes (calls without a transition, the end of one) arrive here; the transition sends from loop()
void PixelLight::write_state(light::LightState *state) {
  if (this->transition_ != nullptr)
    return;
  this->request_send_();
}

uint8_t PixelLight::shown_effect_() {
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (this->hub_->has_telemetry() && this->board_ < this->hub_->telemetry().receivers()) {
    const PixelState &ps = this->hub_->telemetry().pixel(this->board_);
    if (!ps.has_effect(this->effect_)) {
      if (!this->warned_missing_) {
        this->warned_missing_ = true;
        ESP_LOGW(TAG, "'%s': effect %u is not in the receiver's firmware: showing Solid", this->state_->get_name().c_str(),
                 this->effect_);
      }
      return this->solid_;
    }
    this->warned_missing_ = false;
  }
#endif
  return this->effect_;
}

void PixelLight::loop() {
  if (this->first_loop_) {
    // the state at boot: arbitrate the exclusive lights (this one wins: its effect is on the receiver), send it
    this->first_loop_ = false;
    bool other_on = false;
    for (light::LightState *o : this->others_)
      other_on |= o->remote_values.is_on();
    if (this->state_->remote_values.is_on()) {
      for (light::LightState *o : this->others_) {
        if (o->remote_values.is_on())
          o->make_call().set_state(false).set_transition_length(0).perform();
      }
    }
    if (other_on && !this->state_->remote_values.is_on()) {
      this->release_();   // a streaming light is on and this one is off: the receiver streams (a stored effect stops)
    } else {
      this->effect_changed_ = true;
      this->dirty_ = true;
    }
  }
  const uint32_t now = millis();
  if (this->dirty_)
    this->send_();
  if (this->sent_ && !this->released_ && this->hub_->start_time_us(this->board_) - this->anchor_us_ > REANCHOR_US) {
    this->effect_changed_ = true;   // the anchor is old: a new one (one crossfade, once in 20 days)
    this->transition_ms_ = 1000;
    this->send_();
  }
  if (this->store_pending_ && this->sent_ && !this->released_ && !this->dirty_ &&
      now - this->last_change_ms_ >= this->store_after_ms_) {
    this->store_pending_ = false;
    ultra_pixel_fx_t f = this->last_;
    f.flags = static_cast<uint8_t>(f.flags | ULTRA_PXF_STORE);
    f.transition_ms = 0;
    this->hub_->scheduler().light_pixel_fx(this->board_, this->segment_, f, this->anchor_us_);
    this->hub_->wake_();
    ESP_LOGD(TAG, "'%s': stored on the receiver as its power-on effect", this->state_->get_name().c_str());
  }
}

void PixelLight::send_() {
  this->dirty_ = false;
  const light::LightColorValues &v = this->state_->remote_values;
  const bool on = v.is_on();
  if (this->released_) {
    if (!on)
      return;   // handed back to the stream and still off: nothing to say
    this->released_ = false;
    this->effect_changed_ = true;
  }
  PixelLightState s = this->state_in_;
  s.on = on;
  s.brightness = v.get_brightness();
  const float cb = v.get_color_brightness();
  s.red = v.get_red() * cb;
  s.green = v.get_green() * cb;
  s.blue = v.get_blue() * cb;
  s.effect = this->shown_effect_();
  s.palette = static_cast<uint8_t>(this->palette_id_());
  uint32_t transition = this->transition_ms_;
  if (this->effect_changed_ && this->sent_ && transition == 0)
    transition = std::min<uint32_t>(this->state_->get_default_transition_length(), 0xFFFF);   // the effect crossfade
  s.transition_ms = static_cast<uint16_t>(transition);
  ultra_pixel_fx_t f = pixel_fx_of(s);
  if (this->sent_ && !this->effect_changed_ && pixel_fx_same(f, this->last_)) {
    this->transition_ms_ = 0;
    return;   // the receiver is there already
  }
  const int64_t start_us = this->hub_->start_time_us(this->board_);
  if (this->effect_changed_ || !this->sent_)
    this->anchor_us_ = start_us;
  this->hub_->scheduler().light_pixel_fx(this->board_, this->segment_, f, this->anchor_us_);
  this->hub_->wake_();
  this->last_ = f;
  this->sent_ = true;
  this->effect_changed_ = false;
  this->transition_ms_ = 0;
  // lit: on, or still fading out on the receiver (the power supply, a Diff-Power port's wake)
  if (this->on_ && !on) {
    this->off_until_ms_ = static_cast<uint32_t>((start_us + 999) / 1000) + f.transition_ms;
    this->fading_out_ = true;
  } else if (on) {
    this->fading_out_ = false;
  }
  this->on_ = on;
}

// ----------------------------------------------------------------------------------------------------- parameters
void PixelLight::set_param(uint8_t which, uint8_t value) {
  switch (which) {
    case PARAM_SPEED: this->state_in_.speed = value; break;
    case PARAM_INTENSITY: this->state_in_.intensity = value; break;
    case PARAM_CUSTOM1: this->state_in_.custom1 = value; break;
    case PARAM_CUSTOM2: this->state_in_.custom2 = value; break;
    default: return;
  }
  this->transition_ms_ = std::max<uint32_t>(this->transition_ms_, PARAM_RAMP_MS);
  this->last_change_ms_ = millis();
  this->store_pending_ = this->store_after_ms_ != 0;
  this->request_send_();
}

uint8_t PixelLight::get_param(uint8_t which) const {
  switch (which) {
    case PARAM_SPEED: return this->state_in_.speed;
    case PARAM_INTENSITY: return this->state_in_.intensity;
    case PARAM_CUSTOM1: return this->state_in_.custom1;
    case PARAM_CUSTOM2: return this->state_in_.custom2;
    default: return 0;
  }
}

void PixelLight::set_palette_index(size_t index) {
  if (index >= this->palette_ids_.size())
    return;
  this->palette_index_ = index;
  this->transition_ms_ = std::max<uint32_t>(this->transition_ms_, PARAM_RAMP_MS);
  this->last_change_ms_ = millis();
  this->store_pending_ = this->store_after_ms_ != 0;
  this->request_send_();
}

void PixelLight::set_color2(const uint8_t *rgb) {
  set_rgb_(this->state_in_.c2, rgb);
  this->transition_ms_ = std::max<uint32_t>(this->transition_ms_, PARAM_RAMP_MS);
  this->request_send_();
}

void PixelLight::set_color3(const uint8_t *rgb) {
  set_rgb_(this->state_in_.c3, rgb);
  this->transition_ms_ = std::max<uint32_t>(this->transition_ms_, PARAM_RAMP_MS);
  this->request_send_();
}

// ------------------------------------------------------------------------------------------------- page 0x81
std::string PixelLight::status_text_() const {
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  if (!this->hub_->has_telemetry() || this->board_ >= this->hub_->telemetry().receivers())
    return "No telemetry";
  const dig2analog_ultra::ReceiverData &d = this->hub_->telemetry().data(this->board_);
  const PixelState &ps = this->hub_->telemetry().pixel(this->board_);
  if (!d.have_identity)
    return "Waiting for the receiver";
  if (!(d.hw_caps & ULTRA_HWCAP_PIXEL_FX))
    return "No local effects (firmware too old)";
  if (!ps.have_state)
    return "Waiting for the receiver";
  const char *mode = ps.mode == ULTRA_PXM_LOCAL ? "Local" : ps.mode == ULTRA_PXM_STANDALONE ? "Standalone" : "Stream";
  std::string t = mode;
  const uint8_t id = this->segment_ < ULTRA_PX_SEGMENTS ? ps.running[this->segment_] : 0;
  if (ps.mode != ULTRA_PXM_STREAM && id != 0) {
    t += ": ";
    const char *name = nullptr;
    for (const PixelFxEffect *e : this->effects_) {
      if (e->id() == id)
        name = e->get_name().c_str();
    }
    if (name != nullptr)
      t += name;
    else
      t += "effect " + std::to_string(id);
  }
  char buf[64];
  if (ps.mode != ULTRA_PXM_STREAM) {
    std::snprintf(buf, sizeof buf, ", %u.%u fps, load %u %%", ps.fps10 / 10, ps.fps10 % 10,
                  static_cast<unsigned>(ps.load / 2));
    t += buf;
    if (ps.limit < 255) {
      std::snprintf(buf, sizeof buf, ", limited to %u %%", static_cast<unsigned>(ps.limit * 100u / 255u));
      t += buf;
    }
    if (ps.late != 0) {
      std::snprintf(buf, sizeof buf, ", %u late frames", ps.late);
      t += buf;
    }
  }
  return t;
#else
  return "No telemetry";
#endif
}

void PixelLight::pixel_page_() {
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  const dig2analog_ultra::ReceiverData &d = this->hub_->telemetry().data(this->board_);
  const PixelState &ps = this->hub_->telemetry().pixel(this->board_);
  if (d.have_identity && !(d.hw_caps & ULTRA_HWCAP_PIXEL_FX) && !this->warned_cap_) {
    this->warned_cap_ = true;
    ESP_LOGW(TAG, "'%s': the receiver on board %u has no local effects (ULTRA_HWCAP_PIXEL_FX): update its firmware",
             this->state_->get_name().c_str(), this->board_);
  }
  if (ps.have_effects) {
    // the catalogue's effects this receiver's firmware lacks (the YAML's list is the catalogue's; a newer ESPHome or an
    // older receiver): marked in the log, Solid shows when one is chosen
    std::string missing;
    for (const PixelFxEffect *e : this->effects_) {
      if (!ps.has_effect(e->id())) {
        missing += (missing.empty() ? "" : ", ");
        missing += e->get_name().c_str();
      }
    }
    if (!missing.empty() && missing != this->missing_logged_) {
      this->missing_logged_ = missing;
      ESP_LOGW(TAG, "'%s': not in the receiver's firmware (catalogue %u): %s", this->state_->get_name().c_str(),
               ps.catalogue, missing.c_str());
    }
    if (!ps.has_effect(this->effect_) && !this->released_)
      this->request_send_();   // (shown_effect_: Solid)
  }
#ifdef USE_TEXT_SENSOR
  if (this->status_ != nullptr) {
    const std::string t = this->status_text_();
    if (t != this->status_last_) {
      this->status_last_ = t;
      this->status_->publish_state(t);
    }
  }
#endif
#endif
}

// ------------------------------------------------------------------------------------------------------ entities
#ifdef USE_NUMBER
void PixelNumber::setup() {
  this->pref_ = global_preferences->make_preference<float>(this->get_preference_hash());
  float v;
  if (!this->pref_.load(&v))
    v = this->light_->get_param(this->which_);
  v = std::min(255.0f, std::max(0.0f, std::round(v)));
  this->light_->set_param(this->which_, static_cast<uint8_t>(v));
  this->publish_state(v);
}

void PixelNumber::control(float value) {
  value = std::min(255.0f, std::max(0.0f, std::round(value)));
  this->light_->set_param(this->which_, static_cast<uint8_t>(value));
  this->pref_.save(&value);
  this->publish_state(value);
}
#endif

#ifdef USE_SELECT
void PixelPalette::setup() {
  this->pref_ = global_preferences->make_preference<float>(this->get_preference_hash());
  float v;
  size_t index = this->light_->get_palette_index();
  if (this->pref_.load(&v) && v >= 0.0f && static_cast<size_t>(v) < this->size())
    index = static_cast<size_t>(v);
  this->light_->set_palette_index(index);
  this->publish_state(index);
}

void PixelPalette::control(size_t index) {
  this->light_->set_palette_index(index);
  const float v = static_cast<float>(index);
  this->pref_.save(&v);
  this->publish_state(index);
}
#endif

}  // namespace esphome::ultra_digital

#endif  // USE_ESP32
