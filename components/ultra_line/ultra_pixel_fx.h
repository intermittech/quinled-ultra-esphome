#pragma once

// Local pixel effects and their PIXEL_FX state page (0x11), scoped CONFIG
// 64..77, and the independent LINK_STATS page (0x13). Pure host logic.

#include "ultra_protocol.h"
#include "ultra_platform.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

namespace esphome::ultra_line {

// ---------------------------------------------------------------------------------------------------- page 0x11
/// LINK_STATS part 0: physical ingress widths and snapshot epoch.
struct PixelLineState {
  bool have{false};
  uint8_t port{0}, phy{0}, flags{0}, quantum_ns{0};
  uint16_t zero_min{0}, zero_max{0}, one_min{0}, one_max{0}, invalid{0}, epoch{0};
};
inline void decode_pixel_link(PixelLineState &s, const uint8_t *p) {
  s.port=p[ULTRA_LS_PORT]; s.phy=p[ULTRA_LS_PHY]; s.flags=p[ULTRA_LS_FLAGS]; s.quantum_ns=p[ULTRA_LS_QUANTUM_NS];
  s.zero_min=ultra_get16(p+ULTRA_LS_ZERO_MIN); s.zero_max=ultra_get16(p+ULTRA_LS_ZERO_MAX);
  s.one_min=ultra_get16(p+ULTRA_LS_ONE_MIN); s.one_max=ultra_get16(p+ULTRA_LS_ONE_MAX);
  s.invalid=ultra_get16(p+ULTRA_LS_INVALID); s.epoch=ultra_get16(p+ULTRA_LS_EPOCH); s.have=true;
}

/// What a receiver's page 0x11 (ULTRA_PAGE_PIXEL_FX) says, part by part (the parts are read on their own: state every
/// few seconds, the effect and palette maps once after the receiver starts, the line every few seconds on a normal
/// Ultra line)
struct PixelState {
  // part 0, the state
  bool have_state{false};
  uint16_t catalogue{0}, max_pixels{0}, fps10{0}, config_crc{0};
  uint8_t effects{0}, palettes{0}, chips{0}, segments{0};
  uint8_t mode{ULTRA_PXM_STREAM};
  uint8_t running[HOST_PIXEL_SEGMENTS]{}, seq[HOST_PIXEL_SEGMENTS]{};
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

/// HA diagnostic state (under 255 chars). Publish runtime only when both maps are complete;
/// a restart/replacement invalidates the maps and returns to the configured board's fallback.
inline std::string pixel_capabilities(const PixelState &s, const std::string &board, const std::string &fallback) {
  if (!s.have_effects || !s.have_palettes) return fallback;
  auto hex = [](const uint8_t *data) {
    static const char digits[] = "0123456789abcdef";
    std::string out;
    for (unsigned i = 0; i < 24; i++) { out += digits[data[i] >> 4]; out += digits[data[i] & 15]; }
    return out;
  };
  return "{\"v\":1,\"source\":\"runtime\",\"board\":\"" + board + "\",\"effects\":\"" + hex(s.effect_map) +
         "\",\"palettes\":\"" + hex(s.palette_map) + "\"}";
}

/// One answer's DATA (24 bytes) of page 0x11 into s (part: the POLL's argument 0, ULTRA_PXG_PART_*)
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
    default:
      s.catalogue = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXG_CATALOGUE));
      s.effects = p[ULTRA_PXG_EFFECTS];
      s.palettes = p[ULTRA_PXG_PALETTES];
      s.chips = p[ULTRA_PXG_CHIPS];
      s.segments = p[ULTRA_PXG_SEGMENTS];
      s.max_pixels = static_cast<uint16_t>(ultra_get16(p + ULTRA_PXG_MAX_PIXELS));
      s.mode = p[ULTRA_PXG_MODE];
      std::memcpy(s.running, p + ULTRA_PXG_RUNNING, HOST_PIXEL_SEGMENTS);
      std::memcpy(s.seq, p + ULTRA_PXG_SEQ, HOST_PIXEL_SEGMENTS);
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
///  - mode: auto | stream | chain, as CONFIG 75 (ULTRA_PXLM_*); the hub sends it with the other CONFIG ids
///  - the line: a digital port needs a receiver in STREAM mode, a normal Ultra line one in CHAIN mode
inline const char *pixel_mode_name(uint8_t mode) {
  return mode==ULTRA_PXLM_CHAIN ? "chain" : mode==ULTRA_PXLM_STREAM ? "stream" : "auto";
}
inline std::string pixel_line_issues(const PixelLineState &s, uint8_t) {
  if (!s.have) return "";
  if (s.flags & ULTRA_LS_OVERFLOW) return "Link statistics overflow";
  if (s.flags & ULTRA_LS_WARNING) return "Link timing warning";
  return "";
}
inline std::string pixel_line_status(bool identity, uint32_t caps, const PixelState &ps, uint8_t) {
  if (!identity) return "Waiting for the receiver descriptor";
  if (!(caps & RX_CAP_CHAIN)) return "Chain mode unavailable";
  if (!ps.line.have) return "Waiting for link statistics";
  return (ps.line.flags & ULTRA_LS_LOCKED) ? "Link locked" : "Link not locked";
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
    {ULTRA_PXC_WS2815, 3, "GRB"},
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
  ultra_slot_pixel_fx(sx, 0, 0, 0, &x);
  ultra_slot_pixel_fx(sy, 0, 0, 0, &y);
  return std::memcmp(sx, sy, ULTRA_SLOT_BYTES) == 0;
}

// ----------------------------------------------------------------------------------------------------- the sender
/// What a pixel light has told its segment, and when something new is due (pure; the light feeds it ESPHome's state,
/// tests/ur_digital_m_test.cpp feeds it a script). One PIXEL_FX is the whole truth about the segment: the effect with
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
    o.fx.effect = ULTRA_PX_FX_STOP;
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

}  // namespace esphome::ultra_line
