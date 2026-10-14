#pragma once

// Ultra mode host side without ESP-IDF / ESPHome dependencies (tests/ultra_host_test.cpp runs it on a PC,
// together with the board firmware's engine): ESPHome light values -> M / A / B, the frame scheduler and
// the bit encoder. Protocol: ultra_protocol.h (a copy of the board firmware's master).

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "ultra_protocol.h"
#include "ultra_stream.h"

#if defined(__GNUC__)
#define ULTRA_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define ULTRA_ALWAYS_INLINE inline
#endif

namespace esphome::dig2analog_ultra {

static constexpr uint8_t CHANNELS = 5;
static constexpr int64_t NO_START = INT64_MIN;

// ESPHome ColorCapability bits (a ColorMode is the OR of its capabilities)
static constexpr uint8_t CAP_WHITE = 1 << 2;
static constexpr uint8_t CAP_COLD_WARM_WHITE = 1 << 4;
static constexpr uint8_t CAP_RGB = 1 << 5;

/// The fields of an ESPHome LightColorValues that the board needs.
struct ColorValues {
  float state, brightness, color_brightness, red, green, blue, white, cold_white, warm_white;
  uint8_t caps;  // color mode
};

inline uint16_t to16(float v) {
  if (!(v > 0.0f))
    return 0;
  if (v >= 1.0f)
    return 65535;
  return static_cast<uint16_t>(v * 65535.0f + 0.5f);
}

/// M / A / B for SET and STEP (see ultra_protocol.h). ESPHome interpolates state, brightness, color_brightness and
/// each color separately and gamma-corrects their product; the board does the same with M = state x brightness,
/// A = the channel's color and B = color_brightness (RGB channels).
inline void map_values(const uint8_t *roles, const ColorValues &v, uint16_t &m, uint16_t *a, uint16_t *b) {
  m = to16(v.state * v.brightness);
  for (uint8_t i = 0; i < CHANNELS; i++) {
    a[i] = 0;
    b[i] = 65535;
    switch (roles[i]) {
      case ULTRA_ROLE_BRIGHTNESS:
        a[i] = 65535;
        break;
      case ULTRA_ROLE_RED:
      case ULTRA_ROLE_GREEN:
      case ULTRA_ROLE_BLUE:
        if (v.caps & CAP_RGB) {
          a[i] = to16(roles[i] == ULTRA_ROLE_RED ? v.red : roles[i] == ULTRA_ROLE_GREEN ? v.green : v.blue);
          b[i] = to16(v.color_brightness);
        }
        break;
      case ULTRA_ROLE_WHITE:
        if (v.caps & CAP_WHITE)
          a[i] = to16(v.white);
        break;
      case ULTRA_ROLE_COLD_WHITE:
        if (v.caps & CAP_COLD_WARM_WHITE)
          a[i] = to16(v.cold_white);
        break;
      case ULTRA_ROLE_WARM_WHITE:
        if (v.caps & CAP_COLD_WARM_WHITE)
          a[i] = to16(v.warm_white);
        break;
      default:
        break;
    }
  }
}

/// One strobe step (ESPHome strobe color) as the board takes it.
struct Step {
  uint16_t duration_ms, transition_ms;
  uint16_t m, a[CHANNELS], b[CHANNELS];
};

/// The repeat period of a board effect in ms (as the board computes it), 0 = not periodic.
inline uint32_t effect_period_ms(uint8_t fx, const uint8_t *params, size_t n, const std::vector<Step> *steps) {
  uint32_t v = 0;
  switch (fx) {
    case ULTRA_FX_PULSE:
      v = n >= 9 ? ultra_get24(params + 6) : 0;
      return 2 * (v ? v : 1);
    case ULTRA_FX_BREATHE:
      v = n >= 3 ? ultra_get24(params) : 0;
      return v ? v : 4000;
    case ULTRA_FX_COLOR_LOOP:
      v = n >= 3 ? ultra_get24(params) : 0;
      return v ? v : 30000;
    case ULTRA_FX_STROBE:
      if (steps != nullptr) {
        for (size_t i = 0; i < steps->size() && i < ULTRA_MAX_STEPS; i++)
          v += (*steps)[i].duration_ms ? (*steps)[i].duration_ms : 1;
      }
      return v;
    default:
      return 0;
  }
}

/// The start of a periodic effect on the host clock's phase: the latest start <= start_us that is a whole number of
/// periods after the clock's zero. Lights started at different moments (an HA group or scene sends one call per
/// light) then run exactly in step, across all data lines of the controller.
inline int64_t phase_start_us(int64_t start_us, uint32_t period_ms) {
  if (period_ms == 0 || start_us < 0)
    return start_us;
  const int64_t p = static_cast<int64_t>(period_ms) * 1000;
  return start_us - start_us % p;
}

/// PWM presets of the board firmware (period in 144 MHz counts), index = CONFIG value.
static constexpr uint16_t PWM_PRESET_STEPS[15] = {65535, 46341, 32768, 23170, 16384, 11585, 8192, 7200,
                                                  5760,  4800,  4096,  3600,  2880,  2048,  1440};
static constexpr uint32_t PWM_CLOCK_HZ = 144000000;

/// Frequency -> (preset, period): a preset when one is within 0.5 %, else the raw period (period 0 = preset).
inline void pwm_for_frequency(float hz, uint16_t &preset, uint16_t &period) {
  preset = 8;
  period = 0;
  if (!(hz > 0.0f))
    return;
  float best = 1e9f;
  for (uint16_t i = 0; i < 15; i++) {
    const float f = static_cast<float>(PWM_CLOCK_HZ) / PWM_PRESET_STEPS[i];
    const float err = f > hz ? (f - hz) / hz : (hz - f) / hz;
    if (err < best) {
      best = err;
      preset = i;
    }
  }
  if (best > 0.005f) {
    float steps = static_cast<float>(PWM_CLOCK_HZ) / hz + 0.5f;
    period = static_cast<uint16_t>(steps < 1440.0f ? 1440.0f : steps > 65535.0f ? 65535.0f : steps);
  }
}

/// Frames the hub sends before its commands (at boot, after resync / power-on): full frames of DARK slots (all
/// zero, never valid) + the header. A board locks onto Ultra after 3 headers and forwards one frame later (~4
/// frames per board, one board after the other); a board that remembers Ultra confirms it with the first one. A
/// board still locked to WS2805 / WS2915 shows its WS window of the zeros (dark, never a colour) until it locks.
/// A header-only frame would not do: the first board takes the first 256 bits of every frame as its slot.
inline uint16_t preamble_frames(uint16_t boards) { return static_cast<uint16_t>(ultra_preamble_frames(boards)); }
/// After the boards' supply switched on, once a full preamble has reached them with power: they all stored Ultra and
/// confirm it with the first frame, so a short preamble does (a board new since then locks in ~4 frames; 10 cover 2
/// of them, the refresh brings the rest; wiki: Ultra Mode). 50 boards: 10 frames (0.18 s) instead of 202
/// (3.6 s).
static constexpr uint16_t POWER_ON_PREAMBLE_FRAMES = 10;
inline uint16_t power_on_preamble_frames(uint16_t boards) {
  const uint16_t full = preamble_frames(boards);
  return full < POWER_ON_PREAMBLE_FRAMES ? full : POWER_ON_PREAMBLE_FRAMES;
}

/// How far ahead a command starts (start_delay: auto), so it is on the wire before it starts: the frame that may be on
/// the wire now, then one frame per command for the busiest board (a board gets one slot per frame, first copies
/// first): `commands` new ones and `waiting` first copies queued before them, and every 16th slot may be a refresh
/// (Scheduler::build_frame); + slack for ESPHome's loop. frame_us: one frame and the gap after it. One Home Assistant
/// change to a light group or a scene gives every light a command: with commands = the most lights on one board,
/// they all start together.
static constexpr int64_t START_SLACK_US = 10000;
inline int64_t start_lead_us(uint32_t frame_us, size_t commands, size_t waiting) {
  const size_t n = commands + waiting;
  return static_cast<int64_t>(frame_us) * static_cast<int64_t>(1 + n + (n + 14) / 15) + START_SLACK_US;
}

/// Commands queued close together (one Home Assistant call to a light group or a scene, an automation: a call per
/// light, a few ms apart) share one start, also across data lines. The first opens a group, lead_us ahead; the next
/// ones join while their own first copy is still out by then (reach_us), else they open the next group.
class GroupStart {
 public:
  int64_t start(int64_t now_us, int64_t lead_us, int64_t reach_us) {
    if (this->start_ == NO_START || now_us + reach_us > this->start_)
      this->start_ = now_us + lead_us;
    return this->start_;
  }

 protected:
  int64_t start_{NO_START};
};

/// RMT underruns (diagnostics): the encoder refills the RMT memory from its interrupt. Two refills further apart than
/// the whole memory takes to go out at the shortest bit mean the line ran out of symbols: that frame was broken.
struct UnderrunWatch {
  int64_t drain_us{0};   // the RMT memory on the wire at the shortest bit
  int64_t last_us{0};
  volatile uint32_t count{0};
  /// a refill at now_us; first: the frame's first fill (before it starts)
  ULTRA_ALWAYS_INLINE void fill(int64_t now_us, bool first) {
    if (!first && now_us - this->last_us > this->drain_us)
      this->count = this->count + 1;
    this->last_us = now_us;
  }
};

/// Builds the frames: one slot per board, then the header (a trailer: its TIME is the host clock at the frame's
/// last bit, the moment every board sees the line go quiet). Each board has an urgent queue (commands, each sent
/// `repeats` times, first copies first) and a refresh list (its current CONFIG, light definitions, states and
/// effects) that fills the slots of boards without urgent work, so boards that missed a command or were
/// power-cycled catch up. SET / EFFECT carry an absolute host start time; the offset to each frame's TIME
/// is filled in when the frame is built, so every copy (and every board) starts at the same moment.
class Scheduler {
 public:
  void init(uint16_t num_boards, uint32_t seed, uint8_t repeats = 3) {
    this->repeats_ = repeats == 0 ? 1 : repeats;
    this->boards_.clear();
    this->queue_peak_ = this->queue_bytes_ = this->queue_bytes_peak_ = 0;
    this->boards_.resize(num_boards);
    this->starts_.assign(num_boards, NO_START);
    this->floors_.assign(num_boards, 0);
    this->durations_.assign(num_boards, 0);
    for (auto &b : this->boards_) {
      for (auto &row : b.seq) {
        for (auto &s : row) {
          seed = seed * 1664525u + 1013904223u;
          s = static_cast<uint8_t>(seed >> 24);
        }
      }
    }
  }
  uint16_t num_boards() const { return static_cast<uint16_t>(this->boards_.size()); }
  size_t frame_bytes() const { return ultra_frame_bytes(static_cast<uint32_t>(this->boards_.size())); }
  /// The header's offset in a frame (after the slots).
  size_t header_offset() const { return ultra_header_offset(static_cast<uint32_t>(this->boards_.size())); }

  /// Bit timing on the wire (ns): the header TIME is computed from the bits of each frame.
  void set_bit_timing(uint32_t bit0_high_ns, uint32_t bit0_low_ns, uint32_t bit1_high_ns, uint32_t bit1_low_ns) {
    this->t0h_ = bit0_high_ns;
    this->t0l_ = bit0_low_ns;
    this->t1h_ = bit1_high_ns;
    this->t1l_ = bit1_low_ns;
  }
  /// From the first bit's rising edge to the last bit's falling edge of `bytes` on the wire (ns).
  uint64_t wire_ns(const uint8_t *bytes, size_t size) const {
    if (size == 0)
      return 0;
    uint32_t ones = 0;
    for (size_t i = 0; i < size; i++) {
      for (uint8_t v = bytes[i]; v != 0; v = static_cast<uint8_t>(v & (v - 1)))
        ones++;
    }
    const uint64_t bits = static_cast<uint64_t>(size) * 8;
    const uint64_t all = ones * static_cast<uint64_t>(this->t1h_ + this->t1l_) +
                         (bits - ones) * static_cast<uint64_t>(this->t0h_ + this->t0l_);
    return all - ((bytes[size - 1] & 1U) ? this->t1l_ : this->t0l_);   // the last bit ends with its falling edge
  }

  /// Board setting (CONFIG id). Sent with flush_config(); identical values are not re-sent.
  void set_config(uint16_t board, uint8_t id, uint16_t value) {
    if (board >= this->boards_.size() || id == ULTRA_CFG_NONE || id >= ULTRA_CFG_COUNT)
      return;
    Board &b = this->boards_[board];
    if (b.cfg_have[id] && b.cfg[id] == value)
      return;
    b.cfg[id] = value;
    b.cfg_have[id] = true;
    b.cfg_dirty = true;
  }
  uint16_t get_config(uint16_t board, uint8_t id) const {
    return board < this->boards_.size() && id < ULTRA_CFG_COUNT ? this->boards_[board].cfg[id] : 0;
  }
  bool has_config(uint16_t board, uint8_t id) const {
    return board < this->boards_.size() && id < ULTRA_CFG_COUNT && this->boards_[board].cfg_have[id];
  }
  /// Queue the changed settings (all of them in 3 CONFIG slots of up to 9 values).
  void flush_config() {
    for (auto &b : this->boards_) {
      if (!b.cfg_dirty)
        continue;
      b.cfg_dirty = false;
      for (uint8_t page = 0; page < CONFIG_PAGES; page++) {
        uint8_t ids[9];
        uint16_t vals[9];
        uint32_t n = 0;
        for (uint8_t id = 1 + page * 9; id < ULTRA_CFG_COUNT && id < 1 + (page + 1) * 9; id++) {
          if (b.cfg_have[id]) {
            ids[n] = id;
            vals[n++] = b.cfg[id];
          }
        }
        Item &it = b.config[page];
        it.valid = n != 0;
        if (!it.valid)
          continue;
        ultra_slot_config(it.slot, this->next_seq_(b, ULTRA_LIGHT_BOARD, ULTRA_CONFIG), ids, vals, n);
        it.start_us = NO_START;
        this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_CONFIG, page));
      }
    }
  }

  void light_def(uint16_t board, uint8_t light, const uint8_t *roles, uint16_t gamma_x1000, uint8_t flags,
                 const uint16_t *min_p, const uint16_t *max_p) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).def;
    ultra_slot_light_def(it.slot, light, this->next_seq_(b, light, ULTRA_LIGHT_DEF), roles, gamma_x1000, flags, min_p,
                         max_p);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(light, ULTRA_LIGHT_DEF, 0));
  }

  void light_set(uint16_t board, uint8_t light, uint8_t flags, uint16_t m, const uint16_t *a, const uint16_t *bb,
                 uint32_t dur_ms, int64_t start_us) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).set;
    ultra_slot_set(it.slot, light, this->next_seq_(b, light, ULTRA_SET), flags, m, a, bb, dur_ms, 0);
    it.valid = true;
    it.start_us = start_us;
    it.stream = false;
    this->queue_(b, key_(light, ULTRA_SET, 0));
  }

  /// A streamed step (a host-side effect step, a strip pixel): a linear SET over glide_ms, sent once (a lost one is
  /// replaced by the next step) and replacing a queued one. It never starts before it arrives, so a late one is not
  /// cut short; when the board's queue was behind, the glide stretches to the time since the light's last step, so the
  /// light keeps moving instead of waiting.
  void light_stream(uint16_t board, uint8_t light, uint16_t m, const uint16_t *a, const uint16_t *bb,
                    uint32_t glide_ms, int64_t start_us) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).set;
    ultra_slot_set(it.slot, light, this->next_seq_(b, light, ULTRA_SET), 0, m, a, bb, glide_ms, 0);
    it.valid = true;
    it.start_us = start_us;
    it.stream = true;
    it.glide_ms = glide_ms;
    this->queue_(b, key_(light, ULTRA_SET, 0), 1);
  }

  /// Light options (LIGHT_OPT): dim to warm with the LEDs' and the warm end's mireds x 10, red assist (Q16), and
  /// optional set points (n pairs of M and mireds x 10; flag ULTRA_OPT_POINTS).
  void light_opt(uint16_t board, uint8_t light, uint8_t flags, uint16_t cw_mired10, uint16_t ww_mired10,
                 uint16_t warm_mired10, uint16_t red_assist, uint8_t n = 0, const uint16_t *pt_m = nullptr,
                 const uint16_t *pt_mired10 = nullptr) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).opt;
    ultra_slot_light_opt_points(it.slot, light, this->next_seq_(b, light, ULTRA_LIGHT_OPT), flags, cw_mired10,
                                ww_mired10, warm_mired10, red_assist, n, pt_m, pt_mired10);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(light, ULTRA_LIGHT_OPT, 0));
  }

  /// Power-on scene of a light (stored by the board when its outputs change): mode 0 = not in the scene, 1 = show
  /// m / a / b (like SET) from the board's power-up until the host takes over, cross-fading over fade_100ms.
  void light_scene(uint16_t board, uint8_t light, uint8_t mode, uint8_t fade_100ms, uint16_t m, const uint16_t *a,
                   const uint16_t *bb) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).scene;
    ultra_slot_scene(it.slot, light, this->next_seq_(b, light, ULTRA_SCENE), mode, fade_100ms, m, a, bb);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(light, ULTRA_SCENE, 0));
  }

  /// Power-on effect of a light (SCENE_EFFECT, protocol 0.2.3): fx (FX_NONE = none) and its parameters (as EFFECT),
  /// run by the board on the light's SCENE values from its power-up until the host takes over. The board stores it
  /// with the light's definition and scene (flash, written only when it changes). Like the SCENE it is sent at boot,
  /// repeated by replay() and the refresh; a new SEQ only when it changes (an unchanged call queues nothing).
  void light_scene_effect(uint16_t board, uint8_t light, uint8_t fx, const uint8_t *params, size_t n) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).scene_fx;
    uint8_t s[ULTRA_SLOT_BYTES];
    ultra_slot_scene_effect(s, light, it.slot[2], fx, params, static_cast<uint32_t>(n));
    if (it.valid && std::memcmp(s, it.slot, ULTRA_SLOT_BYTES) == 0)
      return;
    ultra_slot_scene_effect(it.slot, light, this->next_seq_(b, light, ULTRA_SCENE_EFFECT), fx, params,
                            static_cast<uint32_t>(n));
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(light, ULTRA_SCENE_EFFECT, 0));
  }

  /// Effect (FX_NONE stops it) at start_us; strobe steps are sent along (count in params[0]). While the light's
  /// effect is held (hold_effect: a strip shows on the light) the effect is only kept, for release_effect().
  void light_effect(uint16_t board, uint8_t light, uint8_t fx, const uint8_t *params, size_t n, int64_t start_us,
                    const std::vector<Step> *steps = nullptr) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    LightItems &li = li_(b, light);
    if (li.held) {
      ultra_slot_effect(li.held_effect.slot, light, 0, fx, 0, params, static_cast<uint32_t>(n));
      li.held_effect.valid = true;
      li.held_effect.start_us = start_us;
      li.held_effect.period_ms = effect_period_ms(fx, params, n, steps);
      li.held_steps.clear();
      if (steps != nullptr)
        this->make_steps_(li.held_steps, light, 0, *steps);
      return;
    }
    const uint8_t seq = this->next_seq_(b, light, ULTRA_EFFECT);
    ultra_slot_effect(li.effect.slot, light, seq, fx, 0, params, static_cast<uint32_t>(n));
    li.effect.valid = true;
    li.effect.start_us = start_us;
    li.effect.period_ms = effect_period_ms(fx, params, n, steps);
    // a new effect drops the old one's steps, also from the queue
    li.steps.clear();
    this->unqueue_(b, key_(light, ULTRA_STEP, 0), true);
    this->queue_(b, key_(light, ULTRA_EFFECT, 0));
    if (steps != nullptr) {
      this->make_steps_(li.steps, light, seq, *steps);
      for (size_t i = 0; i < li.steps.size(); i++)
        this->queue_(b, key_(light, ULTRA_STEP, static_cast<uint8_t>(i)));
    }
  }

  /// A strip shows on the light (dig2analog_ultra_strip layer): its board effect stops at start_us and is kept;
  /// effects set meanwhile are kept too.
  void hold_effect(uint16_t board, uint8_t light, int64_t start_us) {
    if (!this->valid_(board, light) || this->effect_held(board, light))
      return;
    Board &b = this->boards_[board];
    LightItems &li = li_(b, light);
    li.held_effect = li.effect;
    li.held_steps = li.steps;
    if (!li.effect.valid || li.effect.slot[ULTRA_PAYLOAD] == ULTRA_FX_NONE) {
      li.held = true;  // nothing runs: nothing to stop
      return;
    }
    this->light_effect(board, light, ULTRA_FX_NONE, nullptr, 0, start_us);
    li.held = true;
  }

  /// The strip is gone: the kept effect runs again, at its own start (so in phase), with new sequence numbers.
  void release_effect(uint16_t board, uint8_t light) {
    if (!this->effect_held(board, light))
      return;
    Board &b = this->boards_[board];
    LightItems &li = *b.light[light];
    li.held = false;
    if (!li.held_effect.valid)
      return;
    const uint8_t seq = this->next_seq_(b, light, ULTRA_EFFECT);
    li.effect = li.held_effect;
    li.effect.slot[2] = seq;
    ultra_slot_seal(li.effect.slot);
    li.steps = li.held_steps;
    for (auto &st : li.steps) {
      st.slot[2] = seq;  // a STEP carries its EFFECT's sequence number
      ultra_slot_seal(st.slot);
    }
    this->unqueue_(b, key_(light, ULTRA_STEP, 0), true);
    this->queue_(b, key_(light, ULTRA_EFFECT, 0));
    for (size_t i = 0; i < li.steps.size(); i++)
      this->queue_(b, key_(light, ULTRA_STEP, static_cast<uint8_t>(i)));
  }
  bool effect_held(uint16_t board, uint8_t light) const {
    return this->valid_(board, light) && this->boards_[board].light[light] && this->boards_[board].light[light]->held;
  }

  void identify(uint16_t board, bool on) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_identify(it.slot, this->next_seq_(b, ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY), on ? 1 : 0);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY, 0));
  }

  /// The board blinks its firmware version once on its feedback channel (IDENTIFY 5, protocol 0.2.1). Boards with
  /// firmware from before 0.2.1 ignore it: no blink means older firmware.
  void show_version(uint16_t board) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_identify(it.slot, this->next_seq_(b, ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY), ULTRA_IDENT_VERSION);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY, 0));
  }

  /// Dark hold: the board fades dark (0.3 s) and stays dark for hold_s seconds (0 = 60) or until release_dark();
  /// its engine keeps running underneath (CalSequencer).
  void hold_dark(uint16_t board, uint8_t hold_s) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_hold_dark(it.slot, this->next_seq_(b, ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY), hold_s);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY, 0));
  }
  void release_dark(uint16_t board) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_identify(it.slot, this->next_seq_(b, ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY), ULTRA_IDENT_RELEASE);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY, 0));
  }
  /// The board's last IDENTIFY (identify, calibrate, dark, release) still has copies to send.
  bool identify_pending(uint16_t board) const {
    if (board >= this->boards_.size())
      return false;
    const uint16_t k = key_(ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY, 0);
    for (const auto &c : this->boards_[board].queue) {
      if (c.key == k)
        return true;
    }
    return false;
  }

  /// Current-sense boards: measure each channel alone (duty_pct 0: at 25 / 50 / 75 / 100 %; 5..100: at that duty);
  /// others ignore it.
  void calibrate_current(uint16_t board, uint8_t duty_pct) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_calibrate(it.slot, this->next_seq_(b, ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY), duty_pct);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_IDENTIFY, 0));
  }

  /// Everything again as urgent commands, transitions restarted at start_us (boards that were powered up meanwhile
  /// replay them; boards that kept their state ignore the repeats). Effects keep their own start: a board that comes
  /// back joins in phase with the others (a sunrise continues where it is).
  void replay(int64_t start_us) {
    for (auto &b : this->boards_) {
      for (uint8_t p = 0; p < CONFIG_PAGES; p++) {
        if (b.config[p].valid)
          this->queue_(b, key_(ULTRA_LIGHT_BOARD, ULTRA_CONFIG, p));
      }
      for (uint8_t l = 0; l < ULTRA_LIGHTS; l++) {
        if (!b.light[l])
          continue;
        LightItems &li = *b.light[l];
        if (li.def.valid)
          this->queue_(b, key_(l, ULTRA_LIGHT_DEF, 0));
        if (li.opt.valid)
          this->queue_(b, key_(l, ULTRA_LIGHT_OPT, 0));
        if (li.set.valid) {
          li.set.start_us = start_us;
          this->queue_(b, key_(l, ULTRA_SET, 0));
        }
        if (li.effect.valid)
          this->queue_(b, key_(l, ULTRA_EFFECT, 0));
        if (li.scene.valid)
          this->queue_(b, key_(l, ULTRA_SCENE, 0));
        if (li.scene_fx.valid)
          this->queue_(b, key_(l, ULTRA_SCENE_EFFECT, 0));
        for (size_t i = 0; i < li.steps.size(); i++)
          this->queue_(b, key_(l, ULTRA_STEP, static_cast<uint8_t>(i)));
      }
    }
  }

  /// The board's lights from first_unused on are not in the YAML (any more): an empty LIGHT_DEF (no channels), SCENE
  /// mode 0 and SCENE_EFFECT none clear what an earlier configuration left on a board that stayed powered or stored
  /// it. Built on the fly (no per-light state), queued now and part of the refresh.
  void clear_unused_lights(uint16_t board, uint8_t first_unused) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    b.first_unused = first_unused > ULTRA_LIGHTS ? ULTRA_LIGHTS : first_unused;
    for (uint8_t l = b.first_unused; l < ULTRA_LIGHTS; l++) {
      for (uint8_t type : {ULTRA_LIGHT_DEF, ULTRA_SCENE, ULTRA_SCENE_EFFECT}) {
        this->next_seq_(b, l, type);
        this->queue_(b, key_(l, type, 0));
      }
    }
  }

  /// Commands waiting to be sent.
  bool pending() const {
    for (const auto &b : this->boards_) {
      if (!b.queue.empty())
        return true;
    }
    return false;
  }
  size_t queued(uint16_t board) const { return board < this->boards_.size() ? this->boards_[board].queue.size() : 0; }
  /// First copies waiting on a board: frames before a new command's first copy goes out.
  size_t waiting(uint16_t board) const { return board < this->boards_.size() ? this->boards_[board].unsent : 0; }
  /// ... on the busiest board.
  size_t waiting() const {
    size_t n = 0;
    for (const auto &b : this->boards_)
      n = b.unsent > n ? b.unsent : n;
    return n;
  }
  /// The most commands queued for one board since the last call (diagnostics).
  size_t take_queue_peak() {
    const size_t n = this->queue_peak_;
    this->queue_peak_ = 0;
    return n;
  }
  /// The heap this line took at most since the last call (diagnostics): its state (boards, lights in use, effect
  /// steps) and its queues at their peak.
  size_t take_memory_peak() {
    size_t n = this->boards_.capacity() * sizeof(Board) + this->queue_bytes_peak_;
    for (const auto &b : this->boards_) {
      for (const auto &l : b.light) {
        if (l)
          n += sizeof(LightItems) + (l->steps.capacity() + l->held_steps.capacity()) * sizeof(Item);
      }
    }
    this->queue_bytes_peak_ = this->queue_bytes_;
    return n;
  }

  /// Builds one frame into out (frame_bytes()) that starts on the wire at start_us (host clock). Returns its TIME:
  /// the host clock at the last bit (the header carries the low 32 bits).
  int64_t build_frame(int64_t start_us, uint8_t *out) {
    uint8_t *s = out;
    for (size_t i = 0; i < this->boards_.size(); i++, s += ULTRA_SLOT_BYTES) {
      this->starts_[i] = NO_START;
      this->floors_[i] = 0;
      Board &b = this->boards_[i];
      // every 16th slot of a board with a continuous stream is a refresh: a board that rebooted meanwhile still gets
      // its CONFIG and light definitions
      if (b.urgent_run >= 15 && this->take_refresh_(b, s, i, start_us)) {
        b.urgent_run = 0;
      } else if (this->take_urgent_(b, s, i, start_us)) {
        b.urgent_run++;
      } else {
        b.urgent_run = 0;
        if (!this->take_refresh_(b, s, i, start_us))
          ultra_slot_nop(s);
      }
      this->durations_[i] = s[0] == ULTRA_SET ? ultra_get24(s + ULTRA_PAYLOAD + 22) : 0;
    }
    return this->finish_(start_us, out);
  }

  /// The frame build_frame() / build_dark_frame() last built, again (its transmit failed): a new TIME and start
  /// offsets, the same slots, nothing more taken from the queues.
  int64_t retime_frame(int64_t start_us, uint8_t *out) { return this->finish_(start_us, out); }

  /// A frame with only NOP slots (nothing to do).
  int64_t build_idle_frame(int64_t start_us, uint8_t *out) {
    for (size_t i = 0; i < this->boards_.size(); i++) {
      ultra_slot_nop(out + ULTRA_SLOT_BYTES * i);
      this->starts_[i] = NO_START;
      this->floors_[i] = 0;
    }
    return this->finish_(start_us, out);
  }
  /// A preamble frame: DARK slots (all zero) + header, see preamble_frames().
  int64_t build_dark_frame(int64_t start_us, uint8_t *out) {
    for (size_t i = 0; i < this->boards_.size(); i++) {
      ultra_slot_dark(out + ULTRA_SLOT_BYTES * i);
      this->starts_[i] = NO_START;
      this->floors_[i] = 0;
    }
    return this->finish_(start_us, out);
  }

 protected:
  static constexpr uint8_t CONFIG_PAGES = 3;

  struct Item {
    uint8_t slot[ULTRA_SLOT_BYTES]{};
    int64_t start_us{NO_START};
    uint32_t period_ms{0};  // EFFECT: its repeat period (0 = not periodic)
    uint32_t glide_ms{0};   // streamed SET: the glide asked for
    bool stream{false};     // streamed SET: sent once, never starts before it arrives
    bool valid{false};
  };
  struct Cmd {
    uint16_t key;
    uint8_t sends;
    uint8_t copies;  // sends before it leaves the queue
    bool behind;     // it replaced a command that had not been sent yet
    uint32_t order;
  };
  struct LightItems {
    Item def, opt, set, effect, scene, scene_fx;
    std::vector<Item> steps;
    bool held{false};  // a strip shows on the light: held_effect waits for release_effect()
    Item held_effect;
    std::vector<Item> held_steps;
    int64_t last_stream_us{NO_START};  // when its last streamed SET went out
  };
  /// Per board ~410 B, + ~430 B per light in use (allocated with its first command): 50 boards with one light each
  /// take ~42 KB of the ESP32's heap (wiki: Ultra Mode). The queue holds keys (12 B per command) and is freed whenever
  /// it empties.
  struct Board {
    std::vector<Cmd> queue;
    Item config[CONFIG_PAGES];
    Item identify;  // the last IDENTIFY (identify, version, dark, release, calibrate)
    std::unique_ptr<LightItems> light[ULTRA_LIGHTS];
    uint16_t cfg[ULTRA_CFG_COUNT]{};
    bool cfg_have[ULTRA_CFG_COUNT]{};
    bool cfg_dirty{false};
    uint8_t seq[ULTRA_LIGHTS + 1][ULTRA_TYPES]{};
    uint16_t rr{0};
    uint8_t urgent_run{0};   // queued commands sent in a row (build_frame: every 16th slot a refresh)
    uint8_t first_unused{ULTRA_LIGHTS};   // light indices from here on are not in the YAML
    uint8_t unsent{0};                    // queued commands of which no copy went out yet
  };

  /// The commands that clear a light index not in the YAML (clear_unused_lights), with its current sequence number.
  static Item unused_item_(const Board &b, uint8_t light, uint8_t type) {
    Item it;
    const uint16_t zero[CHANNELS] = {};
    const uint8_t roles[CHANNELS] = {};
    const uint8_t seq = b.seq[light][type];
    if (type == ULTRA_LIGHT_DEF)
      ultra_slot_light_def(it.slot, light, seq, roles, 1000, 0, zero, zero);
    else if (type == ULTRA_SCENE)
      ultra_slot_scene(it.slot, light, seq, 0, 0, 0, zero, zero);
    else
      ultra_slot_scene_effect(it.slot, light, seq, ULTRA_FX_NONE, nullptr, 0);
    it.valid = true;
    return it;
  }

  static void make_steps_(std::vector<Item> &out, uint8_t light, uint8_t seq, const std::vector<Step> &steps) {
    const uint8_t count = static_cast<uint8_t>(steps.size() > ULTRA_MAX_STEPS ? ULTRA_MAX_STEPS : steps.size());
    out.clear();
    for (uint8_t i = 0; i < count; i++) {
      const Step &s = steps[i];
      Item it;
      ultra_slot_step(it.slot, light, seq, i, count, s.duration_ms, s.transition_ms, s.m, s.a, s.b);
      it.valid = true;
      it.start_us = NO_START;
      out.push_back(it);
    }
  }

  /// A periodic effect's start, moved forward by whole periods once it is more than a day old: the offset to a
  /// frame stays far inside the EFFECT slot's int32 ms (24.8 days), so a board that comes back after weeks still
  /// joins in phase.
  static int64_t recent_start_(const Item &it, int64_t now_us) {
    if (it.start_us == NO_START || it.period_ms == 0 || now_us - it.start_us < 86400LL * 1000000)
      return it.start_us;
    const int64_t p = static_cast<int64_t>(it.period_ms) * 1000;
    return it.start_us + (now_us - it.start_us) / p * p;
  }

  static uint16_t key_(uint8_t light, uint8_t type, uint8_t sub) {
    return static_cast<uint16_t>(((light & 0x0F) << 12) | ((type & 0x0F) << 8) | sub);
  }
  bool valid_(uint16_t board, uint8_t light) const { return board < this->boards_.size() && light < ULTRA_LIGHTS; }
  static LightItems &li_(Board &b, uint8_t light) {
    if (!b.light[light])
      b.light[light].reset(new LightItems());
    return *b.light[light];
  }
  static uint8_t next_seq_(Board &b, uint8_t light, uint8_t type) {
    return ++b.seq[light < ULTRA_LIGHTS ? light : ULTRA_LIGHTS][type];
  }

  /// The item a queued key stands for: the queue holds keys, the items live in the board's state, so a copy that goes
  /// out is always the latest command of its key.
  static Item queued_item_(const Board &b, uint16_t key) {
    const uint8_t light = key >> 12, type = (key >> 8) & 15, sub = key & 255;
    if (type == ULTRA_CONFIG)
      return b.config[sub];
    if (type == ULTRA_IDENTIFY)
      return b.identify;
    if (!b.light[light])
      return unused_item_(b, light, type);
    const LightItems &li = *b.light[light];
    switch (type) {
      case ULTRA_LIGHT_DEF:
        return li.def;
      case ULTRA_LIGHT_OPT:
        return li.opt;
      case ULTRA_SET:
        return li.set;
      case ULTRA_EFFECT:
        return li.effect;
      case ULTRA_SCENE:
        return li.scene;
      case ULTRA_SCENE_EFFECT:
        return li.scene_fx;
      default:  // STEP (a new effect drops the old steps from the queue; an empty slot is never valid)
        return sub < li.steps.size() ? li.steps[sub] : Item{};
    }
  }

  /// A newer command replaces a queued one with the same key (same light and type, same page / step) and keeps its
  /// place while no copy of it went out (so lights that change every frame take turns). copies: how often it is sent
  /// (0 = repeats). One entry per key: at most 3 CONFIG pages, IDENTIFY and per light 6 items + 8 STEPs (74), so
  /// nothing is ever dropped.
  void queue_(Board &b, uint16_t key, uint8_t copies = 0) {
    bool behind = false;
    uint32_t order = this->order_++;
    for (const auto &c : b.queue) {
      if (c.key == key && c.sends == 0) {
        behind = true;
        order = c.order;
      }
    }
    this->unqueue_(b, key, false);
    const size_t cap = b.queue.capacity();
    b.queue.push_back(Cmd{key, 0, copies ? copies : this->repeats_, behind, order});
    b.unsent++;
    this->queue_bytes_ += (b.queue.capacity() - cap) * sizeof(Cmd);
    this->queue_peak_ = b.queue.size() > this->queue_peak_ ? b.queue.size() : this->queue_peak_;
    if (this->queue_bytes_ > this->queue_bytes_peak_)
      this->queue_bytes_peak_ = this->queue_bytes_;
  }
  /// steps: drop every STEP of that light, whatever its index
  static void unqueue_(Board &b, uint16_t key, bool steps) {
    for (size_t i = 0; i < b.queue.size();) {
      const bool same = steps ? (b.queue[i].key & 0xFF00) == (key & 0xFF00) : b.queue[i].key == key;
      if (same) {
        b.unsent -= b.queue[i].sends == 0;
        b.queue.erase(b.queue.begin() + static_cast<std::ptrdiff_t>(i));
      } else {
        i++;
      }
    }
  }

  /// Start offset of SET (int16 ms) / EFFECT (int32 ms) relative to this frame's TIME, then the CRC. floor: not
  /// before TIME (a streamed SET starts when it arrives, at the latest). A SET that started more than 32.8 s ago (a
  /// long transition refreshed to a board that missed it) starts at that limit with its duration (the SET's own)
  /// shortened by the rest: it still ends on time, and one that is over shows its target at once.
  static void stamp_(uint8_t *s, int64_t start_us, int64_t hdr_us, bool floor, uint32_t duration) {
    if (start_us == NO_START)
      return;
    const int64_t d = floor && start_us < hdr_us ? 0 : start_us - hdr_us;
    int64_t ms = d >= 0 ? (d + 500) / 1000 : -((-d + 499) / 1000);
    if (s[0] == ULTRA_SET) {
      if (ms < -32768) {
        const uint64_t lost = static_cast<uint64_t>(-32768 - ms);
        duration = lost >= duration ? 0 : duration - static_cast<uint32_t>(lost);
      }
      ultra_put24(s + ULTRA_PAYLOAD + 22, duration);
      ms = ms < -32768 ? -32768 : ms > 32767 ? 32767 : ms;
      ultra_put16(s + ULTRA_PAYLOAD + 25, static_cast<uint16_t>(static_cast<int16_t>(ms)));
    } else if (s[0] == ULTRA_EFFECT) {
      ms = ms < INT32_MIN ? INT32_MIN : ms > INT32_MAX ? INT32_MAX : ms;
      ultra_put32(s + ULTRA_PAYLOAD + 1, static_cast<uint32_t>(static_cast<int32_t>(ms)));
    }
    ultra_slot_seal(s);
  }

  /// The command with the fewest copies sent (oldest first): first copies of everything go out first. A streamed
  /// SET that waited behind others glides at least as long as it has been since the light's last one went out.
  bool take_urgent_(Board &b, uint8_t *s, size_t slot, int64_t now_us) {
    if (b.queue.empty())
      return false;
    size_t best = 0;
    for (size_t i = 1; i < b.queue.size(); i++) {
      const Cmd &c = b.queue[i], &o = b.queue[best];
      if (c.sends < o.sends || (c.sends == o.sends && static_cast<int32_t>(c.order - o.order) < 0))
        best = i;
    }
    Cmd &c = b.queue[best];
    const Item item = queued_item_(b, c.key);
    std::memcpy(s, item.slot, ULTRA_SLOT_BYTES);
    this->starts_[slot] = recent_start_(item, now_us);
    if (item.stream) {
      LightItems &li = li_(b, static_cast<uint8_t>((s[1] >> 4) % ULTRA_LIGHTS));
      uint64_t glide = item.glide_ms;
      if (c.behind && li.last_stream_us != NO_START) {
        const uint64_t since = static_cast<uint64_t>(now_us - li.last_stream_us) / 1000;
        glide = since > glide ? since : glide;
        glide = glide > GLIDE_MAX_MS ? GLIDE_MAX_MS : glide;
        ultra_put24(s + ULTRA_PAYLOAD + 22, static_cast<uint32_t>(glide));
      }
      li.last_stream_us = now_us;
      this->floors_[slot] = 1;
    }
    b.unsent -= c.sends == 0;
    if (++c.sends >= c.copies) {
      b.queue.erase(b.queue.begin() + static_cast<std::ptrdiff_t>(best));
      if (b.queue.empty()) {
        this->queue_bytes_ -= b.queue.capacity() * sizeof(Cmd);
        std::vector<Cmd>().swap(b.queue);   // nothing stays allocated between bursts (boot: every board's items)
      }
    }
    return true;
  }

  /// Next item of the board's refresh list: CONFIG pages, then per light DEF, OPT, SET, EFFECT, SCENE, SCENE_EFFECT,
  /// STEPs; for a light index not in the YAML its clearing DEF, SCENE and SCENE_EFFECT (clear_unused_lights).
  bool take_refresh_(Board &b, uint8_t *s, size_t slot, int64_t now_us) {
    const uint16_t per_light = 6 + ULTRA_MAX_STEPS;
    const uint16_t total = CONFIG_PAGES + ULTRA_LIGHTS * per_light;
    for (uint16_t tries = 0; tries < total; tries++) {
      const uint16_t pos = b.rr;
      b.rr = static_cast<uint16_t>((b.rr + 1) % total);
      const Item *it = nullptr;
      if (pos < CONFIG_PAGES) {
        it = &b.config[pos];
      } else {
        const uint16_t l = (pos - CONFIG_PAGES) / per_light, k = (pos - CONFIG_PAGES) % per_light;
        if (!b.light[l]) {
          if (l < b.first_unused || (k != 0 && k != 4 && k != 5))
            continue;
          const uint8_t type = k == 0 ? ULTRA_LIGHT_DEF : k == 4 ? ULTRA_SCENE : ULTRA_SCENE_EFFECT;
          std::memcpy(s, unused_item_(b, static_cast<uint8_t>(l), type).slot, ULTRA_SLOT_BYTES);
          return true;
        }
        const LightItems &li = *b.light[l];
        it = k == 0   ? &li.def
             : k == 1 ? &li.opt
             : k == 2 ? &li.set
             : k == 3 ? &li.effect
             : k == 4 ? &li.scene
             : k == 5 ? &li.scene_fx
                      : (k - 6u < li.steps.size() ? &li.steps[k - 6] : nullptr);
      }
      if (it == nullptr || !it->valid)
        continue;
      std::memcpy(s, it->slot, ULTRA_SLOT_BYTES);
      this->starts_[slot] = recent_start_(*it, now_us);
      this->floors_[slot] = it->stream ? 1 : 0;
      return true;
    }
    return false;
  }

  /// TIME = start + the frame's bits on the wire; the start offsets of the slots refer to it, and they are bits of
  /// the frame themselves: stamp against an estimate, then once more against the result (an offset only changes
  /// when its ms rounding flips; the header bits are counted with a TIME that may be off by those few bits).
  int64_t finish_(int64_t start_us, uint8_t *out) {
    const size_t n = this->boards_.size(), slots = ULTRA_SLOT_BYTES * n;
    uint8_t *h = out + slots;
    const uint64_t avg = (this->t0h_ + this->t0l_ + this->t1h_ + this->t1l_) / 2;
    int64_t t = start_us + static_cast<int64_t>((this->frame_bytes() * 8 * avg) / 1000);
    for (int pass = 0; pass < 2; pass++) {
      for (size_t i = 0; i < n; i++)
        stamp_(out + ULTRA_SLOT_BYTES * i, this->starts_[i], t, this->floors_[i] != 0, this->durations_[i]);
      ultra_build_header(h, static_cast<uint32_t>(t));
      t = start_us + static_cast<int64_t>((this->wire_ns(out, slots + ULTRA_HDR_BYTES) + 500) / 1000);
    }
    ultra_build_header(h, static_cast<uint32_t>(t));
    return t;
  }

  std::vector<Board> boards_;
  std::vector<int64_t> starts_;      // per slot of the frame being built: its absolute start (SET / EFFECT)
  std::vector<uint32_t> durations_;  // per slot: a SET's own duration (stamp_ may shorten it in the frame)
  std::vector<uint8_t> floors_;      // per slot: a streamed SET (start not before the frame's TIME)
  uint32_t t0h_{250}, t0l_{1050}, t1h_{700}, t1l_{650};
  uint8_t repeats_{3};
  uint32_t order_{0};
  size_t queue_peak_{0};        // the most commands queued for one board (take_queue_peak)
  size_t queue_bytes_{0};       // the queues' heap now, and its peak (take_memory_peak)
  size_t queue_bytes_peak_{0};
};

/// The current calibration of one pin (firmware wiki: Current Sense):
///  1. every board on the pin goes dark (IDENTIFY 3: 0.3 s fade; the board's engine keeps running underneath);
///  2. one board after the other, in chain order, runs its test (IDENTIFY 2), so only one test channel is ever on;
///  3. every board is released (IDENTIFY 4: fades back; states, transitions and effects continue).
/// The link is one-way: a step waits until its command's copies are out (or SEND_TIMEOUT_MS, when the line can't
/// send), then the test's own time. A board that missed its command never blocks the run: the next one starts on
/// time. The dark hold carries the whole run's length plus slack, so boards release themselves if the hub stops.
/// Pure logic, host-tested (tests/ultra_host_test.cpp); UltraHub drives it.
class CalSequencer {
 public:
  static constexpr uint32_t FADE_MS = 300 + 50;          // the boards' dark fade, + margin
  static constexpr uint32_t TEST_MS = 950;               // the standard test (board isense.h: 50 + 20 x 45 ms)
  static constexpr uint32_t TEST_ONE_LEVEL_MS = 180;     // one level only
  static constexpr uint32_t MARGIN_MS = 150;             // after a test, before the next board's command
  static constexpr uint32_t SEND_TIMEOUT_MS = 2000;      // a command that can't get out: move on
  enum Event : uint8_t { EV_NONE = 0, EV_BOARD, EV_DONE };

  /// false: already running (ignored)
  bool start(Scheduler &s, uint16_t num_boards, const std::vector<uint16_t> &boards, uint8_t duty_pct,
             uint32_t now_ms) {
    if (this->phase_ != IDLE || num_boards == 0)
      return false;
    this->jobs_.clear();
    for (uint16_t b : boards) {
      if (b < num_boards)
        this->jobs_.push_back(b);
    }
    if (boards.empty()) {
      for (uint16_t b = 0; b < num_boards; b++)
        this->jobs_.push_back(b);
    }
    this->num_boards_ = num_boards;
    this->duty_ = duty_pct;
    this->idx_ = 0;
    this->t0_ = this->t_ = now_ms;
    this->sent_ = false;
    const uint8_t hold = hold_s(this->jobs_.size(), duty_pct);
    for (uint16_t b = 0; b < num_boards; b++)
      s.hold_dark(b, hold);                              // the whole pin goes dark, not just the boards tested
    this->phase_ = DARKENING;
    return true;
  }
  /// Stop: every board released at once (a test already running finishes on its board, <= 1 s)
  void cancel(Scheduler &s) {
    if (this->phase_ == IDLE)
      return;
    for (uint16_t b = 0; b < this->num_boards_; b++)
      s.release_dark(b);                                 // replaces a calibrate command not sent yet
    this->phase_ = IDLE;
  }
  Event poll(Scheduler &s, uint32_t now_ms) {
    switch (this->phase_) {
      case DARKENING: {
        bool out = true;
        for (uint16_t b = 0; b < this->num_boards_; b++)
          out &= !s.identify_pending(b);
        if (!this->sent_ && (out || now_ms - this->t_ >= SEND_TIMEOUT_MS)) {
          this->sent_ = true;                            // the fade runs from the last copy at the latest
          this->t_ = now_ms;
        }
        if (!this->sent_ || now_ms - this->t_ < FADE_MS)
          return EV_NONE;
        if (this->jobs_.empty())
          return this->finish_(s);
        return this->next_(s, now_ms);
      }
      case SENDING:
        if (!s.identify_pending(this->jobs_[this->idx_]) || now_ms - this->t_ >= SEND_TIMEOUT_MS) {
          this->phase_ = TESTING;                        // a board that took the last copy starts now at the latest
          this->t_ = now_ms;
        }
        return EV_NONE;
      case TESTING:
        if (now_ms - this->t_ < board_ms(this->duty_))
          return EV_NONE;
        if (++this->idx_ >= this->jobs_.size())
          return this->finish_(s);
        return this->next_(s, now_ms);
      default:
        return EV_NONE;
    }
  }
  bool running() const { return this->phase_ != IDLE; }
  /// the board being tested (after EV_BOARD), its position (0-based) and the count
  uint16_t board() const { return this->idx_ < this->jobs_.size() ? this->jobs_[this->idx_] : 0; }
  size_t index() const { return this->idx_; }
  size_t count() const { return this->jobs_.size(); }
  uint32_t elapsed_ms(uint32_t now_ms) const { return now_ms - this->t0_; }
  /// one board's share of the run: its test, then the margin (the command's copies add a few frames)
  static uint32_t board_ms(uint8_t duty_pct) { return (duty_pct ? TEST_ONE_LEVEL_MS : TEST_MS) + MARGIN_MS; }
  /// the dark hold sent to the boards: the whole run (2 frames per board for the copies) + 10 s, at most 255 s
  static uint8_t hold_s(size_t n, uint8_t duty_pct) {
    const uint32_t ms = FADE_MS + static_cast<uint32_t>(n) * (board_ms(duty_pct) + 100) + 10000;
    const uint32_t sec = (ms + 999) / 1000;
    return static_cast<uint8_t>(sec > 255 ? 255 : sec);
  }

 protected:
  enum Phase : uint8_t { IDLE = 0, DARKENING, SENDING, TESTING };
  Event next_(Scheduler &s, uint32_t now_ms) {
    s.calibrate_current(this->jobs_[this->idx_], this->duty_);
    this->phase_ = SENDING;
    this->t_ = now_ms;
    return EV_BOARD;
  }
  Event finish_(Scheduler &s) {
    for (uint16_t b = 0; b < this->num_boards_; b++)
      s.release_dark(b);
    this->phase_ = IDLE;
    return EV_DONE;
  }
  std::vector<uint16_t> jobs_;
  uint16_t num_boards_{0};
  uint8_t duty_{0};
  size_t idx_{0};
  Phase phase_{IDLE};
  uint32_t t0_{0}, t_{0};
  bool sent_{false};
};

/// RMT symbol stream of one frame: bytes MSB first, then one reset symbol (see ws2915_frame.h).
template<typename Symbol> struct FrameSymbols {
  Symbol bit0, bit1, reset;
};

template<typename Symbol>
ULTRA_ALWAYS_INLINE size_t encode_symbols(const FrameSymbols<Symbol> &p, const uint8_t *bytes, size_t size,
                                          size_t symbols_written, size_t symbols_free, Symbol *symbols, bool *done) {
  const size_t end = size * 8;
  size_t pos = symbols_written, out = 0;
  while (out < symbols_free) {
    if (pos < end) {
      symbols[out++] = (bytes[pos >> 3] & (0x80U >> (pos & 7U))) ? p.bit1 : p.bit0;
    } else {
      symbols[out++] = p.reset;
      *done = true;
      break;
    }
    pos++;
  }
  return out;
}

}  // namespace esphome::dig2analog_ultra
