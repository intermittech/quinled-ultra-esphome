#pragma once

// The hub's side of the QuinLED Diff-Power Ultra-Digital receiver's local pixel effects (PIXEL_FX, CONFIG 129-140,
// page 0x81; ultra_protocol.h ULTRA_PX_* / ULTRA_PXG_* / ULTRA_PXL_*). Pure logic, no ESPHome or ESP-IDF dependency
// (host-tested: tests/ultra_digital_test.cpp): what page 0x81 says (PixelState, its part 3 the line: PixelLineState and
// what it means for the line the receiver is on), the chips and colour orders, and how the light's state becomes a
// PIXEL_FX.

#include "ultra_protocol.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

namespace esphome::dig2analog_ultra {

// ---------------------------------------------------------------------------------------------------- page 0x81
/// Part 3, the line the receiver sits on (protocol 0.2.9, ULTRA_PXL_*): its mode (CONFIG 140 as stored, the mode in
/// effect), whether it sees Ultra frames, and how many windows it took part in
struct PixelLineState {
  bool have{false};
  uint8_t config{0}, mode{0}, flags{0}, slots{0};   // mode: 0 undecided, ULTRA_PXLM_STREAM, ULTRA_PXLM_CHAIN
  uint16_t frames{0}, non_ultra{0}, long_frames{0}, windows{0}, answers{0}, missed{0}, markers{0};
  uint32_t bits{0};
};

/// What a receiver's page 0x81 (ULTRA_PAGE_HW_FX) says, part by part (the parts are read on their own: state every
/// few seconds, the effect and palette maps once after the receiver starts, the line every few seconds on a normal
/// Ultra line)
struct PixelState {
  // part 0, the state
  bool have_state{false};
  uint16_t catalogue{0}, max_pixels{0}, fps10{0}, config_crc{0};
  uint8_t effects{0}, palettes{0}, chips{0}, segments{0};
  uint8_t mode{ULTRA_PXM_STREAM};
  uint8_t running[ULTRA_PX_SEGMENTS]{}, seq[ULTRA_PX_SEGMENTS]{};
  uint8_t load{0}, limit{255}, late{0};
  // part 3, the line
  PixelLineState line;
  // part 1, 2: bit (id & 7) of byte id >> 3 (ids 0-191)
  bool have_effects{false}, have_palettes{false};
  uint8_t effect_map[24]{}, palette_map[24]{};

  /// the firmware has this effect (true while the map isn't known: nothing is marked missing before it is read)
  bool has_effect(uint8_t id) const { return !this->have_effects || (id < 192 && (this->effect_map[id >> 3] >> (id & 7)) & 1); }
  bool has_palette(uint8_t id) const {
    return !this->have_palettes || id == ULTRA_PX_PAL_COLORS || id == ULTRA_PX_PAL_DEFAULT ||
           (id < 192 && (this->palette_map[id >> 3] >> (id & 7)) & 1);
  }
};

/// One answer's DATA (24 bytes) of page 0x81 into s (part: the POLL's argument 0, ULTRA_PXG_PART_*)
inline void decode_pixel_page(PixelState &s, uint8_t part, const uint8_t *p) {
  switch (part) {
    case ULTRA_PXG_PART_EFFECTS:
      std::memcpy(s.effect_map, p, sizeof s.effect_map);
      s.have_effects = true;
      break;
    case ULTRA_PXG_PART_PALETTES:
      std::memcpy(s.palette_map, p, sizeof s.palette_map);
      s.have_palettes = true;
      break;
    case ULTRA_PXG_PART_LINE: {
      PixelLineState &l = s.line;
      l.config = p[ULTRA_PXL_CONFIG];
      l.mode = p[ULTRA_PXL_MODE];
      l.flags = p[ULTRA_PXL_FLAGS];
      l.slots = p[ULTRA_PXL_SLOTS];
      l.frames = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_FRAMES));
      l.non_ultra = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_NONULTRA));
      l.long_frames = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_LONG));
      l.windows = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_WINDOWS));
      l.answers = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_ANSWERS));
      l.missed = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_MISSED));
      l.bits = ultra_get24(p + ULTRA_PXL_BITS);
      l.markers = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXL_MARKERS));
      l.have = true;
      break;
    }
    default:
      s.catalogue = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXG_CATALOGUE));
      s.effects = p[ULTRA_PXG_EFFECTS];
      s.palettes = p[ULTRA_PXG_PALETTES];
      s.chips = p[ULTRA_PXG_CHIPS];
      s.segments = p[ULTRA_PXG_SEGMENTS];
      s.max_pixels = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXG_MAX_PIXELS));
      s.mode = p[ULTRA_PXG_MODE];
      std::memcpy(s.running, p + ULTRA_PXG_RUNNING, ULTRA_PX_SEGMENTS);
      std::memcpy(s.seq, p + ULTRA_PXG_SEQ, ULTRA_PX_SEGMENTS);
      s.fps10 = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXG_FPS10));
      s.load = p[ULTRA_PXG_LOAD];
      s.limit = p[ULTRA_PXG_LIMIT];
      s.late = p[ULTRA_PXG_LATE];
      s.config_crc = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXG_CONFIG_CRC));
      s.have_state = true;
      break;
  }
}

// ----------------------------------------------------------------------------------------------- the line (chain mode)
/// What the YAML says about the receiver's line mode (the `mode:` option), and what the line needs
///  - mode: auto | stream | chain, as CONFIG 140 (ULTRA_PXLM_*); the hub sends it with the other CONFIG ids
///  - the line: a digital port needs a receiver in STREAM mode, a normal Ultra line one in CHAIN mode
inline const char *pixel_mode_name(uint8_t mode) {
  switch (mode) {
    case ULTRA_PXLM_STREAM: return "stream";
    case ULTRA_PXLM_CHAIN: return "chain";
    default: return "undecided";
  }
}

/// What is wrong with the receiver's line, as plain text ("" = nothing): the mode in effect is not the one the line needs,
/// it saw a frame of more than one slot (it is not the last board: phase 1 forwards nothing), frames that are no Ultra
/// frames or marker frames on a chain line, a share of the windows it missed. expected: the mode the line needs (ULTRA_PXLM_CHAIN on a
/// normal Ultra line). Undecided (AUTO deciding after a power-up) is not a fault. Parts joined with "; ".
inline std::string pixel_line_issues(const PixelLineState &l, uint8_t expected) {
  std::string t;
  auto add = [&t](const std::string &what) { t += (t.empty() ? "" : "; ") + what; };
  if (!l.have)
    return t;
  if (l.mode != 0 && l.mode != expected) {
    add(std::string("Wrong mode: the receiver is in ") + pixel_mode_name(l.mode) + " mode, this line needs " +
        pixel_mode_name(expected) + " mode");
  }
  if (expected == ULTRA_PXLM_CHAIN) {
    if (l.flags & ULTRA_PXL_F_LONG)
      add("Not the last board of its line (it saw frames of more than one slot)");
    if ((l.flags & ULTRA_PXL_F_NONULTRA) && l.mode == ULTRA_PXLM_CHAIN)
      add(std::to_string(l.non_ultra) + " frames that are no Ultra frames");
    if ((l.flags & ULTRA_PXL_F_MARKERS) && l.mode == ULTRA_PXLM_CHAIN)
      add("Marker frames on the line (a digital port's hub is talking to it)");
    // a window now and then is lost for good reasons (a flash write on the receiver while it is dark); a share of them is a fault
    if (l.missed >= 3 && static_cast<uint32_t>(l.missed) * 50u > l.windows)
      add("Missed " + std::to_string(l.missed) + " of " + std::to_string(l.windows) + " windows");
  }
  return t;
}

/// The receiver's line in one text (the Line status entity of a receiver on a normal Ultra line): the mode in effect and
/// whether it is locked to the line's Ultra frames, then what is wrong with it. have_identity / hw_caps: what IDENTITY said.
inline std::string pixel_line_status(bool have_identity, uint32_t hw_caps, const PixelState &ps, uint8_t expected) {
  if (!have_identity)
    return "Waiting for the receiver";
  if (!(hw_caps & ULTRA_HWCAP_CHAIN))
    return "No chain mode (firmware too old)";
  const PixelLineState &l = ps.line;
  if (!l.have)
    return "Waiting for the receiver";
  std::string t;
  if (l.mode == 0)
    t = "Deciding";
  else if (l.mode == ULTRA_PXLM_CHAIN)
    t = (l.flags & ULTRA_PXL_F_LOCKED) ? "Chain, locked" : "Chain, not locked";
  else
    t = "Stream";
  const std::string issues = pixel_line_issues(l, expected);
  if (!issues.empty())
    t += " - " + issues;
  return t;
}

// ------------------------------------------------------------------------------------------------------ the chips
/// The chips of the YAML (`chip:`), in the order of PixelChipInfo; the wire preset is ULTRA_PXC_*
enum PixelChip : uint8_t {
  PIXEL_CHIP_WS2812B = 0,   // V5 / 2020 and later, also WS2811 V2.2 and later
  PIXEL_CHIP_WS2812B_CLASSIC,
  PIXEL_CHIP_SK6812,
  PIXEL_CHIP_SK6812_RGBW,
  PIXEL_CHIP_WS2814,
  PIXEL_CHIP_WS2805,
  PIXEL_CHIP_WS2815,
  PIXEL_CHIP_WS2811_400K,
  PIXEL_CHIP_COUNT,
};
struct PixelChipInfo {
  uint8_t preset;          // ULTRA_PXC_*
  uint8_t bytes;           // per pixel: 3, 4 or 5
  const char *own_order;   // the chip's own wire order (CONFIG 130 = 0)
};
static constexpr PixelChipInfo PIXEL_CHIPS[PIXEL_CHIP_COUNT] = {
    {ULTRA_PXC_WS2812B_V5, 3, "GRB"},      {ULTRA_PXC_WS2812B_CLASSIC, 3, "GRB"}, {ULTRA_PXC_SK6812_RGB, 3, "GRB"},
    {ULTRA_PXC_SK6812_RGBW, 4, "GRBW"},    {ULTRA_PXC_WS2814_RGBW, 4, "RGBW"},    {ULTRA_PXC_WS2805_RGBCCT, 5, "RGBWC"},
    {ULTRA_PXC_WS2815, 3, "GRB"},          {ULTRA_PXC_WS2811_400K, 3, "RGB"},
};

/// A colour order text ("GRB", "RGBW", "RGBWC": W the white, WS2805: the warm one, C its cold white) as CONFIG 130's
/// value (ULTRA_PX_ORDER). Only the chip's number of letters, each channel once, from its own channels (R G B; W for 4
/// and 5 bytes; C for 5); anything else: false. Case-insensitive.
inline bool pixel_order_value(PixelChip chip, const char *text, uint16_t *value) {
  const uint8_t n = PIXEL_CHIPS[chip].bytes;
  if (std::strlen(text) != n)
    return false;
  uint16_t v = 0;
  uint8_t seen = 0;
  for (uint8_t k = 0; k < n; k++) {
    uint8_t ch;
    switch (text[k] | 0x20) {
      case 'r': ch = ULTRA_PXCH_R; break;
      case 'g': ch = ULTRA_PXCH_G; break;
      case 'b': ch = ULTRA_PXCH_B; break;
      case 'w': ch = ULTRA_PXCH_W; break;
      case 'c': ch = ULTRA_PXCH_CW; break;
      default: return false;
    }
    if ((ch == ULTRA_PXCH_W && n < 4) || (ch == ULTRA_PXCH_CW && n < 5) || (seen & (1u << ch)))
      return false;
    seen |= static_cast<uint8_t>(1u << ch);
    v |= static_cast<uint16_t>(ch << (3 * k));
  }
  *value = v;
  return true;
}

// ------------------------------------------------------------------------------------------------- the light's state
/// What a pixel light asks of its segment (before the scheduler stamps START): ESPHome's light state + the entities
struct PixelLightState {
  bool on{false};
  float brightness{1.0f};        // 0..1 (the light's brightness)
  float red{1}, green{1}, blue{1};   // c1: the colour at full brightness (colour_brightness folded in)
  uint8_t c2[3]{0, 0, 0}, c3[3]{0, 0, 0};
  uint8_t effect{1};             // the catalogue id shown (Solid when no effect is chosen)
  uint8_t speed{128}, intensity{128}, palette{ULTRA_PX_PAL_DEFAULT};
  uint8_t custom1{0}, custom2{0};
  uint16_t transition_ms{0};
  uint16_t seg_start{0}, seg_len{0};
  uint8_t flags{0};
};

inline uint8_t pixel_byte(float v) {
  const float x = v * 255.0f + 0.5f;
  return x <= 0.0f ? 0 : x >= 255.0f ? 255 : static_cast<uint8_t>(x);
}

/// The light's state as the PIXEL_FX it sends (START still 0): off is brightness 0 with the transition; the
/// transition goes to the receiver, which ramps brightness and colours per frame
inline ultra_pixel_fx_t pixel_fx_of(const PixelLightState &s) {
  ultra_pixel_fx_t f;
  std::memset(&f, 0, sizeof f);
  f.effect = s.effect;
  f.speed = s.speed;
  f.intensity = s.intensity;
  f.palette = s.palette;
  f.c[0][0] = pixel_byte(s.red);
  f.c[0][1] = pixel_byte(s.green);
  f.c[0][2] = pixel_byte(s.blue);
  std::memcpy(f.c[1], s.c2, 3);
  std::memcpy(f.c[2], s.c3, 3);
  f.brightness = s.on ? pixel_byte(s.brightness) : 0;
  f.transition_ms = s.transition_ms;
  f.seg_start = s.seg_start;
  f.seg_len = s.seg_len;
  f.flags = static_cast<uint8_t>(s.flags & 0x3Fu);
  f.custom1 = s.custom1;
  f.custom2 = s.custom2;
  return f;
}

/// The part of two PIXEL_FX that matters for "the receiver already shows this" (everything but the transition)
inline bool pixel_fx_same(const ultra_pixel_fx_t &a, const ultra_pixel_fx_t &b) {
  ultra_pixel_fx_t x = a, y = b;
  x.transition_ms = y.transition_ms = 0;
  uint8_t sx[ULTRA_SLOT_BYTES], sy[ULTRA_SLOT_BYTES];   // (the wire bytes: no padding to compare)
  ultra_slot_pixel_fx(sx, 0, 0, &x);
  ultra_slot_pixel_fx(sy, 0, 0, &y);
  return std::memcmp(sx, sy, ULTRA_SLOT_BYTES) == 0;
}

// ----------------------------------------------------------------------------------------------------- the sender
/// What a pixel light has told its segment, and when something new is due (pure; the light feeds it ESPHome's state,
/// tests/ultra_digital_test.cpp feeds it a script). One PIXEL_FX is the whole truth about the segment: the effect with
/// its parameters, the colours, the brightness (0 = off) and the transition. START is the effect's time zero: a new
/// effect takes a new one, a change of its parameters, colours or brightness keeps it (the effect carries on in phase on
/// every receiver), and after REANCHOR_US it gets a fresh one (START is an int32 of ms: 24.8 days).
class PixelFxTracker {
 public:
  static constexpr int64_t REANCHOR_US = 20LL * 86400 * 1000000;

  struct Out {
    ultra_pixel_fx_t fx;
    int64_t anchor_us;   // START, host time
  };

  /// The state wanted now. effect_changed: a new effect (or the first send, or a new anchor asked for): START now
  /// (start_us: where a command queued now starts), the crossfade over default_transition_ms when the wanted transition
  /// is 0. false: nothing to tell the receiver (it is there already); else out is the PIXEL_FX to send.
  /// restart: the user chose the effect (again): the PIXEL_FX carries ULTRA_PXF_RESTART (with the new START effect_changed
  /// gives): the random effects are re-seeded, a time-driven effect (Sunrise, Sunset) begins again. A receiver keeps its
  /// phase when the same slot comes with only a new START (that is what the automatic renewal after 20 days relies on). The
  /// bit is an action: it is in out->fx only, not in the state kept (the STORE copy and the comparison).
  bool update(const PixelLightState &want, bool effect_changed, uint32_t default_transition_ms, int64_t start_us,
              uint32_t now_ms, Out *out, bool restart = false) {
    if (this->released_) {
      if (!want.on)
        return false;   // handed back to the stream and still off: nothing to say
      this->released_ = false;
      effect_changed = true;
    }
    PixelLightState s = want;
    if (effect_changed && this->sent_ && s.transition_ms == 0)
      s.transition_ms = static_cast<uint16_t>(std::min<uint32_t>(default_transition_ms, 0xFFFF));   // the crossfade
    const ultra_pixel_fx_t f = pixel_fx_of(s);
    if (this->sent_ && !effect_changed && pixel_fx_same(f, this->last_))
      return false;
    if (effect_changed || !this->sent_)
      this->anchor_us_ = start_us;
    this->last_ = f;
    this->sent_ = true;
    // lit: on, or still fading out on the receiver (the power supply, a Diff-Power port's wake)
    if (this->on_ && !want.on) {
      this->off_until_ms_ = static_cast<uint32_t>((start_us + 999) / 1000) + f.transition_ms;
      this->fading_out_ = true;
    } else if (want.on) {
      this->fading_out_ = false;
    }
    this->on_ = want.on;
    this->changed_ms_ = now_ms;
    this->store_due_ = true;
    out->fx = f;
    if (restart)
      out->fx.flags = static_cast<uint8_t>(out->fx.flags | ULTRA_PXF_RESTART);
    out->anchor_us = this->anchor_us_;
    return true;
  }

  /// The strip goes back to the hub's stream: PIXEL_FX effect 0 (to send); the state starts over
  Out release(int64_t start_us) {
    Out o;
    std::memset(&o.fx, 0, sizeof o.fx);
    o.fx.effect = ULTRA_PX_FX_STREAM;
    o.anchor_us = start_us;
    this->released_ = true;
    this->sent_ = false;
    this->on_ = false;
    this->fading_out_ = false;
    this->store_due_ = false;
    return o;
  }

  bool released() const { return this->released_; }
  bool sent() const { return this->sent_; }
  /// the receiver shows a local effect that this light started (so there is something to hand back)
  bool local() const { return this->sent_ && !this->released_; }
  bool lit(uint32_t now_ms) {
    if (this->fading_out_ && static_cast<int32_t>(this->off_until_ms_ - now_ms) <= 0)
      this->fading_out_ = false;
    return this->on_ || this->fading_out_;
  }
  /// the anchor is old: ask for a fresh one (update(..., effect_changed = true) with a crossfade)
  bool reanchor_due(int64_t start_us) const { return this->local() && start_us - this->anchor_us_ > REANCHOR_US; }
  /// `store_after_ms` after the last change: the PIXEL_FX to send once more with STORE (the receiver's power-on effect)
  bool store(uint32_t now_ms, uint32_t store_after_ms, Out *out) {
    if (!this->store_due_ || store_after_ms == 0 || !this->local() || now_ms - this->changed_ms_ < store_after_ms)
      return false;
    this->store_due_ = false;
    out->fx = this->last_;
    out->fx.flags = static_cast<uint8_t>(out->fx.flags | ULTRA_PXF_STORE);
    out->fx.transition_ms = 0;
    out->anchor_us = this->anchor_us_;
    return true;
  }
  /// a change of the light that isn't a send of its own (the colours c2, c3; the light's state): the store timer restarts
  void touched(uint32_t now_ms) {
    this->changed_ms_ = now_ms;
    this->store_due_ = this->sent_ && !this->released_;
  }
  const ultra_pixel_fx_t &last() const { return this->last_; }
  int64_t anchor_us() const { return this->anchor_us_; }

 protected:
  ultra_pixel_fx_t last_{};
  int64_t anchor_us_{0};
  uint32_t off_until_ms_{0}, changed_ms_{0};
  bool sent_{false}, released_{false}, on_{false}, fading_out_{false}, store_due_{false};
};

}  // namespace esphome::dig2analog_ultra
