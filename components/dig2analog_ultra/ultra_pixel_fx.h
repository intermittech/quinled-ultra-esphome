#pragma once

// The hub's side of the QuinLED Diff-Power Ultra-Digital receiver's local pixel effects (PIXEL_FX, CONFIG 129-139,
// page 0x81; ultra_protocol.h ULTRA_PX_* / ULTRA_PXG_*). Pure logic, no ESPHome or ESP-IDF dependency (host-tested:
// tests/ultra_digital_test.cpp): what page 0x81 says (PixelState), the chips and colour orders, and how the light's
// state becomes a PIXEL_FX.

#include "ultra_protocol.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace esphome::dig2analog_ultra {

// ---------------------------------------------------------------------------------------------------- page 0x81
/// What a receiver's page 0x81 (ULTRA_PAGE_HW_FX) says, part by part (the parts are read on their own: state every
/// few seconds, the effect and palette maps once after the receiver starts)
struct PixelState {
  // part 0, the state
  bool have_state{false};
  uint16_t catalogue{0}, max_pixels{0}, fps10{0}, config_crc{0};
  uint8_t effects{0}, palettes{0}, chips{0}, segments{0};
  uint8_t mode{ULTRA_PXM_STREAM};
  uint8_t running[ULTRA_PX_SEGMENTS]{}, seq[ULTRA_PX_SEGMENTS]{};
  uint8_t load{0}, limit{255}, late{0};
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

}  // namespace esphome::dig2analog_ultra
