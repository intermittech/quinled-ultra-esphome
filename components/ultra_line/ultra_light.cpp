#include "ultra_light.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "ultra_effects.h"

#include <algorithm>
#include <cmath>
#include <strings.h>

namespace esphome::ultra_line {

static const char *const TAG = "ultra_line.light";

std::vector<UltraLight *> UltraLight::registry_;

UltraTransition::~UltraTransition() { this->light_->transition_done_(this); }

void UltraTransition::start() {
  // ESPHome's own start: off -> on keeps the target colors, on -> off keeps the old ones, and a color mode change
  // goes through off. The board gets the end values, the length and whether to go through off.
  light::LightTransitionTransformer::start();
  this->light_->transition_start_(this, this->end_values_, this->length_, this->changing_color_mode_);
}

void UltraTransition::stop() { this->light_->transition_done_(this); }

UltraLight *UltraLight::find(light::LightOutput *output) {
  for (auto *l : registry_) {
    if (static_cast<light::LightOutput *>(l) == output)
      return l;
  }
  return nullptr;
}

bool UltraLight::has_rgb() const {
  for (uint8_t r : this->roles_) {
    if (r == ULTRA_ROLE_RED)
      return true;
  }
  return false;
}

// Stored fade parameter, capped at one second (fade into the stored level).
static uint8_t scene_fade(const light::LightState *state) {
  const uint32_t fade = (state->get_default_transition_length() + 50) / 100;
  return static_cast<uint8_t>(fade > 10 ? 10 : fade);
}

light::LightTraits UltraLight::get_traits() {
  bool rgb = false, white = false, cww = false;
  for (uint8_t r : this->roles_) {
    rgb |= r == ULTRA_ROLE_RED;
    white |= r == ULTRA_ROLE_WHITE;
    cww |= r == ULTRA_ROLE_COLD_WHITE;
  }
  auto traits = light::LightTraits();
  if (rgb && cww) {
    if (this->color_interlock_) {
      traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLD_WARM_WHITE});
    } else {
      traits.set_supported_color_modes({light::ColorMode::RGB_COLD_WARM_WHITE});
    }
  } else if (rgb && white) {
    if (this->color_interlock_) {
      traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::WHITE});
    } else {
      traits.set_supported_color_modes({light::ColorMode::RGB_WHITE});
    }
  } else if (rgb) {
    traits.set_supported_color_modes({light::ColorMode::RGB});
  } else if (cww) {
    traits.set_supported_color_modes({light::ColorMode::COLD_WARM_WHITE});
  } else {
    traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  }
  if (cww) {
    traits.set_min_mireds(this->cold_white_temperature_);
    traits.set_max_mireds(this->warm_white_temperature_);
  }
  return traits;
}

std::unique_ptr<light::LightTransformer> UltraLight::create_default_transition() {
  return std::make_unique<UltraTransition>(this);
}

void UltraLight::setup_state(light::LightState *state) {
  this->state_ = state;
  registry_.push_back(this);
  uint8_t flags = 0;
  if (this->zero_means_zero_)
    flags |= ULTRA_DEF_ZERO_MEANS_ZERO;
  if (this->constant_brightness_)
    flags |= ULTRA_DEF_CONSTANT_BRIGHTNESS;
  // ESPHome's gamma_correct 0 is linear (its gamma table: i / 255), so gamma 1.0 on the board, whose x^0 would show
  // every level that is on at 100 %
  const float gamma = state->get_gamma_correct();
  const uint16_t g = !(gamma > 0.0f) ? 1000
                     : static_cast<uint16_t>(std::max(1L, std::lround(gamma > 65.0f ? 65000.0f : gamma * 1000.0f)));
  this->hub_->scheduler().light_def(this->board_, this->index_, this->roles_, g, flags, this->min_, this->max_);
  this->send_opt_();
  for (uint8_t slot = 0; slot < 2; slot++) {
    if (this->stored_have_ & (1u << slot)) {
      this->send_stored_level_(slot);
    } else if (slot == 0 && this->scene_mode_ != SCENE_NONE) {
      state->add_remote_values_listener(this);
    } else {
      static const uint16_t z[CHANNELS] = {};
      this->hub_->scheduler().light_scene(this->board_, this->index_, slot, 3, 0, z, z);
    }
  }
  this->hub_->add_light(this);
  this->hub_->wake_();
}

static uint16_t mired10(float mireds) {
  const float v = mireds * 10.0f + 0.5f;
  return static_cast<uint16_t>(v < 0.0f ? 0.0f : v > 65535.0f ? 65535.0f : v);
}

// Dim to warm runs on the board: every 1 ms it moves the CW / WW mix with the brightness (state x brightness, so
// also during transitions and effects), from the light's colour temperature at full brightness to the warm end, or
// along the set points.
void UltraLight::send_opt_() {
  const float assist = this->dtw_assist_ * 65535.0f + 0.5f;
  uint16_t pm[ULTRA_OPT_POINTS_MAX], pt[ULTRA_OPT_POINTS_MAX];
  for (uint8_t i = 0; i < this->dtw_n_; i++) {
    const float m = this->dtw_pb_[i] * 65535.0f + 0.5f;
    pm[i] = static_cast<uint16_t>(m < 1.0f ? 1.0f : m > 65535.0f ? 65535.0f : m);
    pt[i] = mired10(this->dtw_pt_[i]);
  }
  uint8_t flags = this->dtw_on_ ? ULTRA_OPT_DIM_TO_WARM : 0;
  if (this->dtw_n_ > 0)
    flags |= ULTRA_OPT_POINTS;
  this->hub_->scheduler().light_opt(this->board_, this->index_, flags, mired10(this->cold_white_temperature_),
                                    mired10(this->warm_white_temperature_), mired10(this->dtw_warm_),
                                    static_cast<uint16_t>(assist > 65535.0f ? 65535.0f : assist), this->dtw_n_, pm,
                                    pt);
}

void UltraLight::set_dim_to_warm(bool on) {
  if (!this->dtw_available_) {
    ESP_LOGW(TAG, "dim to warm needs dim_to_warm: in the light's configuration");
    return;
  }
  if (on == this->dtw_on_)
    return;
  this->dtw_on_ = on;
  this->send_opt_();
  this->hub_->wake_();
}

// The power-on scene is what ESPHome will restore at its next boot (restore_mode / initial_state); the board stores
// it (writes debounced, small changes at most every 10 min) and shows it from its own power-up until the host is back.
void UltraLight::on_light_remote_values_update() {
  if (this->scene_mode_ == SCENE_NONE || (this->scene_mode_ == SCENE_FIXED && this->scene_sent_) ||
      (this->stored_have_ & 1))   // an explicit stored power-on level takes precedence
    return;
  light::LightColorValues v = this->state_->remote_values;
  if (this->scene_mode_ == SCENE_FOLLOW_ON)
    v.set_state(true);
  else if (this->scene_mode_ == SCENE_FOLLOW_INVERTED)
    v.set_state(!v.is_on());
  if (v.is_on() && v.get_brightness() == 0.0f)
    v.set_brightness(1.0f);  // as LightState::setup() does on boot
  uint16_t m, a[CHANNELS], b[CHANNELS];
  this->map(v, m, a, b);
  const uint8_t f = scene_fade(this->state_);
  if (this->scene_sent_ && m == this->scene_m_ && f == this->scene_fade_ && memcmp(a, this->scene_a_, sizeof a) == 0 &&
      memcmp(b, this->scene_b_, sizeof b) == 0)
    return;
  this->scene_sent_ = true;
  this->scene_m_ = m;
  this->scene_fade_ = f;
  memcpy(this->scene_a_, a, sizeof a);
  memcpy(this->scene_b_, b, sizeof b);
  this->hub_->scheduler().light_scene(this->board_, this->index_, 0, f, m, a, b);
  this->hub_->wake_();
}

// Fixed stored levels: no effect state or effect parameters are sent to a receiver.
void UltraLight::send_stored_level_(uint8_t slot) {
  light::LightColorValues v = this->stored_values_[slot];
  const float ct = v.get_color_temperature(), lo = this->cold_white_temperature_, hi = this->warm_white_temperature_;
  if (ct > 0.0f && hi > lo) {
    const float ww = (std::min(std::max(ct, lo), hi) - lo) / (hi - lo), cw = 1.0f - ww, top = std::max(ww, cw);
    const float g = this->state_->get_gamma_correct(), inv = g > 0.0f ? 1.0f / g : 1.0f;
    v.set_cold_white(std::pow(cw / top, inv));
    v.set_warm_white(std::pow(ww / top, inv));
  }
  uint16_t m, a[CHANNELS], b[CHANNELS];
  this->map(v, m, a, b);
  this->hub_->scheduler().light_scene(this->board_, this->index_, slot, this->stored_fade_[slot], m, a, b);
}

uint32_t UltraLight::effect_interval() const {
  return this->hub_->effect_interval(this->effect_rate());
}

void UltraLight::loop() {
  if (this->pixel_pending_ && this->effect_cadence_.ready(millis(), this->effect_interval())) {
    this->pixel_pending_ = false;
    this->send_pixel(this->pixel_m_, this->pixel_a_, this->pixel_b_, this->pixel_length_, this->pixel_flags_,
                     this->pixel_pending_stream_, this->hub_->stream_start_us(), true);
    return;
  }
  if (!this->effect_pending_ || !this->effect_cadence_.ready(millis(), this->effect_interval())) return;
  if (this->state_->get_current_effect_index() == 0) { this->effect_pending_ = false; return; }
  auto values = this->effect_pending_values_;
  this->effect_pending_ = false;
  this->send_(values, this->effect_pending_length_, this->effect_pending_flags_, true);
}

void UltraLight::map(const light::LightColorValues &v, uint16_t &m, uint16_t *a, uint16_t *b) const {
  uint8_t caps = static_cast<uint8_t>(v.get_color_mode());
  if (caps == 0 && this->state_ != nullptr)
    caps = static_cast<uint8_t>(this->state_->remote_values.get_color_mode());
  const ColorValues cv{v.get_state(),      v.get_brightness(),  v.get_color_brightness(),
                       v.get_red(),        v.get_green(),       v.get_blue(),
                       v.get_white(),      v.get_cold_white(),  v.get_warm_white(),
                       caps};
  map_values(this->roles_, cv, m, a, b);
}

// stream: a step of an ESPHome (host-side) effect. It is sent once (the next step replaces a lost one) and glides
// (effect_glide): over the measured step interval, or with auto: small steps over the interval, a jump up at once, a
// jump down over GLIDE_TAIL_MS, so a chase stays crisp and two lights crossing over do not dip.
void UltraLight::send_(const light::LightColorValues &v, uint32_t length, uint8_t flags, bool stream) {
  // Keep the newest underlying host-effect sample while a strip is the visible layer.
  this->effect_pending_values_ = v;
  if (!this->layers_.empty()) {
    if (!stream)
      this->hub_->note_kept(this->board_);   // (the log: kept while a strip is shown)
    return;
  }
  if (stream) {
    if (!this->effect_cadence_.ready(millis(), this->effect_interval())) {
      this->effect_pending_length_ = length;
      this->effect_pending_flags_ = flags;
      this->effect_pending_ = true;
      return;
    }
    this->effect_cadence_.sent(millis());
    this->effect_pending_ = false;
  } else {
    this->effect_pending_ = false;
  }
  uint16_t m, a[CHANNELS], b[CHANNELS];
  this->map(v, m, a, b);
  if (length == 0 && this->sent_ && m == this->last_m_ && memcmp(a, this->last_a_, sizeof a) == 0 &&
      memcmp(b, this->last_b_, sizeof b) == 0)
    return;  // the board is there already
  if (stream && length == 0 && this->glide_mode_ != GLIDE_OFF &&
      this->effect_kind_(this->state_->get_current_effect_index()) != EFFECT_HARD) {
    const uint32_t interval = this->glide_.step(millis());
    if (this->glide_mode_ == GLIDE_ON || !this->sent_) {
      length = interval;
    } else {
      uint16_t was[CHANNELS], now[CHANNELS];  // A x B: the colour as the board weighs it
      for (uint8_t i = 0; i < CHANNELS; i++) {
        was[i] = static_cast<uint16_t>(static_cast<uint32_t>(this->last_a_[i]) * this->last_b_[i] / 65535);
        now[i] = static_cast<uint16_t>(static_cast<uint32_t>(a[i]) * b[i] / 65535);
      }
      length = glide_for(interval, change_of(this->last_m_, was, m, now),
                         value_of(m, now) >= value_of(this->last_m_, was));
    }
  }
  this->sent_ = true;
  this->last_m_ = m;
  memcpy(this->last_a_, a, sizeof a);
  memcpy(this->last_b_, b, sizeof b);
  const int64_t start_us = stream ? this->hub_->stream_start_us() : this->hub_->start_time_us(this->board_);
  if (stream) {
    this->hub_->scheduler().light_stream(this->board_, this->index_, m, a, b, length, start_us, flags);
    this->hub_->note_stream(this->board_, this->channel_mask_());
  } else {
    this->hub_->scheduler().light_set(this->board_, this->index_, flags, m, a, b, length,
                                      start_us);
    uint16_t level[CHANNELS];   // the log: the levels as the board weighs them (M x A x B)
    for (uint8_t c = 0; c < CHANNELS; c++)
      level[c] = ChangeLog::weigh(m, a[c], b[c]);
    this->hub_->note_change(this->board_, this->platform_, this->channel_mask_(), level, length, start_us);
  }
  this->lit_(m, length, start_us);
  this->hub_->wake_();
}

void UltraLight::lit_(uint16_t m, uint32_t length_ms, int64_t start_us) {
  if ((this->on_ || this->fading_out_) && m == 0) {
    // fading out on the board: from the SET's start (a group start on another line, commands waiting before it), in
    // millis() (on the ESP32 the esp_timer us / 1000); a second fade-out moves the end
    this->off_until_ms_ = static_cast<uint32_t>((start_us + 999) / 1000) + length_ms;
    this->fading_out_ = true;
  } else if (m != 0) {
    this->fading_out_ = false;
  }
  this->on_ = m != 0;
}

void UltraLight::send_pixel(uint16_t m, const uint16_t *a, const uint16_t *b, uint32_t length_ms, uint8_t flags,
                            bool stream, int64_t start_us, bool effect_step) {
  if (effect_step) {
    if (!this->effect_cadence_.ready(millis(), this->effect_interval())) {
      this->pixel_pending_ = true;
      this->pixel_m_ = m;
      memcpy(this->pixel_a_, a, sizeof this->pixel_a_);
      memcpy(this->pixel_b_, b, sizeof this->pixel_b_);
      this->pixel_length_ = length_ms;
      this->pixel_flags_ = flags;
      this->pixel_pending_stream_ = stream;
      return;
    }
    this->effect_cadence_.sent(millis());
    this->pixel_pending_ = false;
    // Addressable effects, including hard strobe, are host steps (one SET, no command repeats).
    stream = true;
    if (length_ms != 0) length_ms = std::max(length_ms, this->effect_interval());
  }
  if (start_us == 0)
    start_us = this->hub_->start_time_us(this->board_);
  if (stream) {
    this->hub_->scheduler().light_stream(this->board_, this->index_, m, a, b, length_ms, start_us, flags);
  } else {
    this->hub_->scheduler().light_set(this->board_, this->index_, flags, m, a, b, length_ms, start_us);
  }
  this->lit_(m, length_ms, start_us);
  this->hub_->wake_();
}

void UltraLight::layer_begin(UltraLayer *layer) {
  this->effect_pending_ = false;
  this->pixel_pending_ = false;
  for (auto it = this->layers_.begin(); it != this->layers_.end(); ++it) {
    if (*it == layer) {
      this->layers_.erase(it);
      break;
    }
  }
  this->layers_.push_back(layer);
  this->hub_->wake_();
}

void UltraLight::layer_end(UltraLayer *layer, uint32_t fade_ms) {
  const bool top = this->layer_top(layer);
  bool found = false;
  for (auto it = this->layers_.begin(); it != this->layers_.end(); ++it) {
    if (*it == layer) {
      this->layers_.erase(it);
      found = true;
      break;
    }
  }
  if (!found || !top)
    return;
  this->pixel_pending_ = false;
  if (!this->layers_.empty()) {
    this->layers_.back()->layer_shown(this);
    return;
  }
  // Back to the light's own state: its target (or the current step of an ESPHome effect running on it), fading,
  // and its host effect supplies its next sample.
  this->hub_->wake_();
  if (this->state_ == nullptr)
    return;
  const light::LightColorValues v =
      this->state_->get_current_effect_index() != 0 ? this->effect_pending_values_ : this->state_->remote_values;
  this->sent_ = false;
  this->send_(v, fade_ms, ULTRA_SET_SMOOTH);
}

void UltraLight::transition_start_(UltraTransition *t, const light::LightColorValues &end, uint32_t length,
                                   bool via_off) {
  this->transition_ = t;
  this->send_(end, length, ULTRA_SET_SMOOTH | (via_off ? ULTRA_SET_VIA_OFF : 0),
              this->state_->get_current_effect_index() != 0);
}

void UltraLight::transition_done_(UltraTransition *t) {
  if (this->transition_ == t) {
    if (!t->is_finished())
      this->sent_ = false;   // cut short: the board still fades to its end, so send the next values even if equal
    this->transition_ = nullptr;
  }
}

// Instant changes (calls without transition, the end of a transition): the board jumps. Steps of ESPHome's own
// effects are coalesced into rate-limited SETs; strobe has no glide. During a board-run transition ESPHome's
// intermediate values are not sent.
void UltraLight::write_state(light::LightState *state) {
  if (this->transition_ != nullptr)
    return;
  const uint32_t fx = state->get_current_effect_index();
  if (fx != this->glide_effect_) {
    this->glide_effect_ = fx;
    this->glide_.reset();
    this->effect_pending_ = false;
  }
  this->send_(state->current_values, 0, 0, fx != 0);
}

}  // namespace esphome::ultra_line

#endif  // USE_ESP32
