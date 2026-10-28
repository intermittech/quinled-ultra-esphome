#pragma once

#ifdef USE_ESP32

#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/light/light_effect.h"
#include "esphome/components/light/transformers.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_TEXT
#include "esphome/components/text/text.h"
#endif

#include "esphome/components/ultra_line/ultra_line.h"
#include "esphome/components/ultra_line/ultra_pixel_fx.h"

#include <memory>
#include <string>
#include <vector>

namespace esphome::ur_digital_m {

using ultra_line::PixelFxTracker;
using ultra_line::PixelLightState;
using ultra_line::PixelSource;
using ultra_line::UltraHub;

class PixelLight;
class PixelColour;

/// ESPHome's transition, run by the receiver: the light sends the target and the length once (PIXEL_FX TRANSITION);
/// the receiver ramps brightness and colours per frame, whatever the hub's loop does. Nothing is written to the line
/// meanwhile.
class PixelTransition final : public light::LightTransitionTransformer {
 public:
  explicit PixelTransition(PixelLight *light) : light_(light) {}
  ~PixelTransition() override;
  void start() override;
  void stop() override;

 protected:
  PixelLight *light_;
};

/// The light's sliders and the extra values (the entities, the action)
enum PixelParam : uint8_t { PARAM_SPEED = 0, PARAM_INTENSITY, PARAM_CUSTOM1, PARAM_CUSTOM2, PARAM_COUNT };

/// An effect of the catalogue (ESPHome's effect list is generated from it): choosing it starts it on the receiver
class PixelFxEffect final : public light::LightEffect {
 public:
  PixelFxEffect(const char *name, uint8_t id) : LightEffect(name), id_(id) {}
  void start() override;
  void stop() override;
  void apply() override {}
  uint8_t id() const { return this->id_; }

 protected:
  PixelLight *light_() const;
  uint8_t id_;
};

/// A light on one segment of a Diff-Power Ultra Receiver Digital-Medium's strip (the QuinLED Diff-Power system's digital receiver): the
/// receiver makes the pixels itself (PIXEL_FX), ESPHome says what: the effect (the catalogue), its colours, speed,
/// intensity and palette, brightness and the transition. On a digital port exclusive with a streaming light on the same
/// strip; on a normal Ultra line (chain mode: the receiver at any position of the chain) its PIXEL_FX and CONFIG are slots
/// of that line like any board's, nothing is streamed and nothing is handed back.
class PixelLight final : public light::LightOutput,
                         public Component,
                         public PixelSource,
                         public light::LightRemoteValuesListener {
 public:
  // Configuration (codegen)
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_board(uint16_t board) { this->board_ = board; }
  void set_segment(uint8_t segment) { this->segment_ = segment; }
  /// the segment's pixels (first, count; 0 = to the end of the strip) and its flags (ULTRA_PXF_REVERSE / MIRROR / LOCK /
  /// the auto-white mode)
  void set_range(uint16_t start, uint16_t length) {
    this->state_in_.seg_start = start;
    this->state_in_.seg_len = length;
  }
  void set_flags(uint8_t flags) { this->state_in_.flags = flags; }
  /// the strip's frame on the wire (us): what a hand-back to the stream waits for (the receiver finishes its frame first)
  void set_strip_frame_us(uint32_t us) { this->strip_frame_us_ = us; }
  /// the receiver's settings (CONFIG 129-139 and 141: one set for the strip, every light of the board must agree)
  void set_receiver_config(uint8_t id, uint16_t value) { this->cfg_.push_back({id, value}); }
  /// CONFIG 75, the line mode (ULTRA_PXLM_*) the YAML says, and the mode the line needs (STREAM on a digital port, CHAIN on
  /// a normal Ultra line). With telemetry the hub sends it once the receiver's IDENTITY shows RX_CAP_CHAIN.
  void set_line_mode(uint8_t mode, uint8_t expected) {
    this->line_mode_ = mode;
    this->line_expected_ = expected;
  }
  void set_solid_id(uint8_t id) { this->solid_ = id; }
  void set_board_type(const std::string &type) { this->board_type_ = type; }
  void set_build_capabilities(const std::string &value) { this->build_capabilities_ = value; }
  void set_palette_ids(const std::vector<uint8_t> &ids) { this->palette_ids_ = ids; }
  void set_defaults(uint8_t speed, uint8_t intensity, uint8_t palette_index, uint8_t custom1, uint8_t custom2) {
    this->state_in_.speed = speed;
    this->state_in_.intensity = intensity;
    this->palette_index_ = palette_index;
    this->state_in_.custom1 = custom1;
    this->state_in_.custom2 = custom2;
  }
  /// c2, c3 as 0xRRGGBB
  void set_colors(uint32_t c2, uint32_t c3) {
    for (int i = 0; i < 3; i++) {
      this->state_in_.c2[i] = static_cast<uint8_t>(c2 >> (16 - 8 * i));
      this->state_in_.c3[i] = static_cast<uint8_t>(c3 >> (16 - 8 * i));
    }
  }
  void set_standby_after(uint32_t ms) { this->standby_after_ms_ = ms; }
  /// store the effect on the receiver as its power-on / standalone effect (PIXEL_FX STORE), this long after the last
  /// change (the receiver writes it once, never while the strip shows a visible pause)
  void set_store_after(uint32_t ms) { this->store_after_ms_ = ms; }
  void add_exclusive(light::LightState *other) { this->others_.push_back(other); }
  void add_effect(PixelFxEffect *effect) { this->effects_.push_back(effect); }
#ifdef USE_TEXT_SENSOR
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_ = s; }
  void set_capabilities_sensor(text_sensor::TextSensor *s) { this->capabilities_ = s; }
  /// the receiver's line (once per receiver, on a normal Ultra line)
  void set_line_sensor(text_sensor::TextSensor *s) { this->line_ = s; }
#endif

  // Component
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // LightOutput
  light::LightTraits get_traits() override;
  std::unique_ptr<light::LightTransformer> create_default_transition() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;
  // LightRemoteValuesListener: this light switched on / off (exclusive lights)
  void on_light_remote_values_update() override;

  // PixelSource (the hub)
  uint16_t pixel_board() const override { return this->board_; }
  bool pixel_lit(uint32_t now_ms) override { return this->track_.lit(now_ms); }
  uint32_t pixel_standby_after() const override { return this->standby_after_ms_; }
  light::LightState *pixel_state() const override { return this->state_; }

  // Parameters (the entities, the action); a change is sent at once, the effect keeps running (same phase)
  void set_param(uint8_t which, uint8_t value);
  uint8_t get_param(uint8_t which) const;
  /// the Palette select's option (0 = Effect default, 1 = c1 -> c2 -> c3, then the catalogue's)
  void set_palette_index(size_t index);
  size_t get_palette_index() const { return this->palette_index_; }
  void set_color2(const uint8_t *rgb);
  void set_color3(const uint8_t *rgb);
  const uint8_t *get_color2() const { return this->state_in_.c2; }
  const uint8_t *get_color3() const { return this->state_in_.c3; }
#ifdef USE_TEXT
  void set_color2_text(PixelColour *text) { this->color2_text_ = text; }
  void set_color3_text(PixelColour *text) { this->color3_text_ = text; }
#endif

  // Called by the effects and the transition
  void effect_started(PixelFxEffect *effect);
  void effect_stopped(PixelFxEffect *effect);
  void transition_requested(uint32_t length_ms);
  void transition_done(PixelTransition *t);

  /// an exclusive (streaming) light switched on or changed: hand the strip back to its stream
  void other_changed_(light::LightState *other);

  /// the light behind a light output, nullptr for other platforms (no RTTI in ESPHome)
  static PixelLight *find(light::LightOutput *output);
  light::LightState *get_state() const { return this->state_; }
  uint8_t effect_id() const { return this->effect_; }

 protected:
  friend class PixelTransition;
  struct Cfg {
    uint8_t id;
    uint16_t value;
  };
  static void set_rgb_(uint8_t *to, const uint8_t *from) {
    for (int i = 0; i < 3; i++)
      to[i] = from[i];
  }
  void request_send_() {
    this->dirty_ = true;
    this->hub_->wake_();
  }
  void send_();
  void release_();                               // the hub's pixels pass (PIXEL_FX effect 0)
  void queue_fx_(const PixelFxTracker::Out &o);  // a PIXEL_FX to the receiver (and the hub's digital port)
  void pixel_page_();                            // page 0x11 news
  void send_line_mode_();                        // CONFIG 75 to the receiver
  void line_identity_();                         // the receiver's IDENTITY is known (or it is not heard): CONFIG 75
  std::string status_text_() const;
  std::string line_text_() const;
  uint8_t shown_effect_();                       // the effect to send: the chosen one, Solid when the receiver lacks it
  uint16_t palette_id_() const {
    return this->palette_index_ < this->palette_ids_.size() ? this->palette_ids_[this->palette_index_]
                                                            : static_cast<uint16_t>(ULTRA_PX_PAL_DEFAULT);
  }

  UltraHub *hub_{nullptr};
  light::LightState *state_{nullptr};
  PixelTransition *transition_{nullptr};
  uint16_t board_{0};
  uint8_t segment_{0};
  uint8_t solid_{1};
  uint8_t effect_{1};                  // the catalogue id chosen (solid_ without an effect)
  PixelFxEffect *chosen_{nullptr};
  std::vector<Cfg> cfg_;
  std::vector<uint8_t> palette_ids_;
  size_t palette_index_{0};
  std::vector<PixelFxEffect *> effects_;
  std::vector<light::LightState *> others_;
  PixelLightState state_in_;           // c2, c3, speed, ... (the parts the light's state doesn't carry)
  uint32_t standby_after_ms_{0};
  uint32_t store_after_ms_{0};
  uint32_t strip_frame_us_{0};
  uint8_t line_mode_{ULTRA_PXLM_AUTO}, line_expected_{ULTRA_PXLM_STREAM};
  bool line_sent_{false};              // CONFIG 75 is with the hub
  bool line_waiting_{false};           // ... held back until IDENTITY says whether the receiver has RX_CAP_CHAIN
  std::string line_last_, line_issue_logged_;

  PixelFxTracker track_;               // what the segment was told, the phase anchor, lit
  bool dirty_{false};                  // something to send in the next loop
  bool effect_changed_{false};         // a new effect: a new phase anchor, the crossfade
  bool restart_{false};                // the user chose the effect (again): time-driven effects start over
  bool warned_missing_{false}, warned_cap_{false};
  uint32_t transition_ms_{0};          // the length asked for since the last send
  bool first_loop_{true};
  bool page_hooked_{false};
  std::string status_last_, missing_logged_;
  std::string board_type_, build_capabilities_, capabilities_last_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *capabilities_{nullptr};
  text_sensor::TextSensor *status_{nullptr};
  text_sensor::TextSensor *line_{nullptr};
#endif

  static std::vector<PixelLight *> registry_;
#ifdef USE_TEXT
  PixelColour *color2_text_{nullptr};
  PixelColour *color3_text_{nullptr};
#endif
};

/// Another light of the strip's output (an exclusive streaming light): tells the PixelLight when it switches on
class ExclusiveWatcher final : public light::LightRemoteValuesListener {
 public:
  ExclusiveWatcher(PixelLight *owner, light::LightState *other) : owner_(owner), other_(other) {}
  void on_light_remote_values_update() override;

 protected:
  PixelLight *owner_;
  light::LightState *other_;
};

#ifdef USE_NUMBER
/// Speed / Intensity / Custom 1 / Custom 2 of a pixel light (WLED's sliders); kept across restarts
class PixelNumber final : public number::Number, public Component {
 public:
  void set_light(PixelLight *light) { this->light_ = light; }
  void set_param(uint8_t which) { this->which_ = which; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  void control(float value) override;
  PixelLight *light_{nullptr};
  ESPPreferenceObject pref_;
  uint8_t which_{0};
};
#endif

#ifdef USE_SELECT
/// The palette of a pixel light (kept across effect changes): Effect default, c1 -> c2 -> c3, then the catalogue's palettes
class PixelPalette final : public select::Select, public Component {
 public:
  void set_light(PixelLight *light) { this->light_ = light; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  void control(size_t index) override;
  PixelLight *light_{nullptr};
  ESPPreferenceObject pref_;
};
#endif

#ifdef USE_TEXT
/// Colour 2 / 3 of a pixel light, stored as RGB in the entity's preferences.
class PixelColour final : public text::Text, public Component {
 public:
  void set_light(PixelLight *light, bool third) { this->light_ = light; this->third_ = third; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  /// Called by the light's setters, including changes from effect_params.
  void publish_color(const uint8_t *rgb);
  static bool parse_color(const std::string &value, uint8_t *rgb);
  static std::string format_color(const uint8_t *rgb);

 protected:
  void control(const std::string &value) override;
  PixelLight *light_{nullptr};
  ESPPreferenceObject pref_;
  bool third_{false};
};
#endif

/// ur_digital_m.effect_params: c2, c3 and the extra values of a pixel light (automations)
template<typename... Ts> class EffectParamsAction : public Action<Ts...> {
 public:
  void set_light(light::LightState *state) { this->state_ = state; }
  TEMPLATABLE_VALUE(uint32_t, color_2)   // 0xRRGGBB
  TEMPLATABLE_VALUE(uint32_t, color_3)
  TEMPLATABLE_VALUE(uint8_t, speed)
  TEMPLATABLE_VALUE(uint8_t, intensity)
  TEMPLATABLE_VALUE(uint8_t, custom1)
  TEMPLATABLE_VALUE(uint8_t, custom2)
  TEMPLATABLE_VALUE(uint8_t, palette)    // the Palette select's option index

  void play(const Ts &...x) override {
    PixelLight *parent_ = PixelLight::find(this->state_->get_output());
    if (parent_ == nullptr)
      return;
    auto rgb = [](uint32_t v, uint8_t *o) {
      o[0] = static_cast<uint8_t>(v >> 16);
      o[1] = static_cast<uint8_t>(v >> 8);
      o[2] = static_cast<uint8_t>(v);
    };
    uint8_t c[3];
    if (this->color_2_.has_value()) {
      rgb(this->color_2_.value(x...), c);
      parent_->set_color2(c);
    }
    if (this->color_3_.has_value()) {
      rgb(this->color_3_.value(x...), c);
      parent_->set_color3(c);
    }
    if (this->speed_.has_value())
      parent_->set_param(PARAM_SPEED, this->speed_.value(x...));
    if (this->intensity_.has_value())
      parent_->set_param(PARAM_INTENSITY, this->intensity_.value(x...));
    if (this->custom1_.has_value())
      parent_->set_param(PARAM_CUSTOM1, this->custom1_.value(x...));
    if (this->custom2_.has_value())
      parent_->set_param(PARAM_CUSTOM2, this->custom2_.value(x...));
    if (this->palette_.has_value())
      parent_->set_palette_index(this->palette_.value(x...));
  }

 protected:
  light::LightState *state_{nullptr};
};

}  // namespace esphome::ur_digital_m

#endif  // USE_ESP32
