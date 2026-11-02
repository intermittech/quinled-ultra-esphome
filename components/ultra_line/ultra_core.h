#pragma once

// Ultra mode host side without ESP-IDF / ESPHome dependencies (tests/ultra_host_test.cpp runs it on a PC,
// together with the board firmware's engine): ESPHome light values -> M / A / B, the frame scheduler and
// the bit encoder. Protocol: ultra_protocol.h (a copy of the board firmware's master).

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "ultra_protocol.h"
#include "ultra_platform.h"
#include "ultra_descriptor.h"
#include "ultra_pixel_calibration.h"
#include "ultra_stream.h"

#if defined(__GNUC__)
#define ULTRA_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define ULTRA_ALWAYS_INLINE inline
#endif

namespace esphome::ultra_line {

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
static constexpr uint32_t PWM_PERIOD_UNITS_HZ = 100000000;  // CONFIG 2: 10 ns

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
    float steps = static_cast<float>(PWM_PERIOD_UNITS_HZ) / hz + 0.5f;
    period = static_cast<uint16_t>(steps < 1.0f ? 1.0f : steps > 65535.0f ? 65535.0f : steps);
  }
}

/// CONFIG 2 (a period in 10 ns units) -> the board's timer period in 144 MHz counts, as its firmware rounds it: up, so a
/// period is never shortened, and kept to 1,440 .. 65,535 counts (100 kHz .. 2.197 kHz). 0 = the preset (0 returned).
inline uint32_t pwm_period_steps(uint32_t units) {
  if (units == 0)
    return 0;
  const uint32_t ticks = (units * (PWM_CLOCK_HZ / 1000000) + 99) / 100;
  return ticks < 1440 ? 1440 : ticks > 65535 ? 65535 : ticks;
}

/// The slowest rate a board plays its pulse-skipping patterns at (its firmware's rule, display only), Hz rounded as the
/// board's console reports it. The pattern is (144 MHz / rate) / steps periods, at most ULTRA_SKIP_TABLE_MAX, and at
/// least the 16-bit dither's ceil(65535 / steps) (2 periods below 4.39 kHz: there 2 kHz plays f / 2 below 4 kHz); with
/// spread spectrum the table is at least 32 periods (at 1 kHz a multiple of the pattern), its pulses evenly spaced.
inline uint32_t pwm_skip_slowest_hz(uint32_t steps, uint32_t rate, bool spread) {
  if (steps == 0 || steps > 65535 || rate == 0)
    return 0;
  uint32_t ml = (PWM_CLOCK_HZ / rate) / steps;
  if (ml > ULTRA_SKIP_TABLE_MAX)
    ml = ULTRA_SKIP_TABLE_MAX;
  const uint32_t dither = (65535 + steps - 1) / steps;
  if (ml < dither)
    ml = dither;
  uint32_t len = ml;
  if (spread && len < 32)
    len = rate == ULTRA_SKIP_HZ_DEFAULT ? 32 : ml * ((32 + ml - 1) / ml);
  const uint32_t pmin = (len + ml - 1) / ml, gap = (len + pmin - 1) / pmin;
  return (PWM_CLOCK_HZ / gap + steps / 2) / steps;
}

/// Frames the hub sends before its commands (at boot, after resync / power-on): full frames of DARK slots (all
/// zero, never valid) + the header. A board locks onto Ultra after 3 headers and forwards one frame later (~4
/// frames per board, one board after the other); a board that remembers Ultra confirms it with the first one. A
/// board still locked to WS2805 / WS2915 shows its WS window of the zeros (dark, never a colour) until it locks.
/// A header-only frame would not do: the first board takes the first 256 bits of every frame as its slot.
/// The port's wake timeline (diffpower): does an Ultra frame (its slots, then the 8-byte header) light anything? A SET with
/// M > 0 and a channel whose A and B are both above 0, or a PIXEL_FX running an effect (not STOP) above brightness 0.
/// LIGHT_DEF, CONFIG, IDENTIFY, NOP, POLL and a dark SET light nothing.
inline bool ultra_frame_lit(const uint8_t *frame, size_t bytes) {
  for (size_t at = 0; at + ULTRA_SLOT_BYTES + ULTRA_HDR_BYTES <= bytes; at += ULTRA_SLOT_BYTES) {
    const uint8_t *s = frame + at, *p = s + ULTRA_PAYLOAD, type = static_cast<uint8_t>(s[0] & 0x9Fu);
    if (type == ULTRA_SET && ultra_get16(p) > 0) {
      for (uint8_t c = 0; c < CHANNELS; c++)
        if (ultra_get16(p + 2 + 2 * c) > 0 && ultra_get16(p + 12 + 2 * c) > 0)
          return true;
    } else if (type == ULTRA_PIXEL_FX && s[ULTRA_PX_EFFECT] != 0 && s[ULTRA_PX_BRIGHTNESS] != 0) {
      return true;
    }
  }
  return false;
}
/// ... and a digital port's pixel frame: a lit byte.
inline bool ultra_pixels_lit(const uint8_t *data, size_t bytes) {
  for (size_t i = 0; i < bytes; i++)
    if (data[i] != 0)
      return true;
  return false;
}

inline uint16_t preamble_frames(uint16_t boards) { return static_cast<uint16_t>(ultra_preamble_frames(boards)); }
/// After the boards' supply switched on, once a full preamble has reached them with power: they all stored Ultra and
/// confirm it with the first frame, so a short preamble does (a board new since then locks in ~4 frames; 10 cover 2
/// of them, the refresh brings the rest; wiki: Ultra how it works). 50 boards: 10 frames (0.18 s) instead of 202
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
  bool frame_bad{false};  // several late refills still describe only one broken frame
  /// a refill at now_us; first: the frame's first fill (before it starts)
  ULTRA_ALWAYS_INLINE void fill(int64_t now_us, bool first) {
    if (first)
      this->frame_bad = false;
    if (!first && !this->frame_bad && now_us - this->last_us > this->drain_us) {
      this->count = this->count + 1;
      this->frame_bad = true;
    }
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
  /// The boards that report their own descriptor (the first n: they answer on a return channel) send nothing before
  /// it. A board without a return channel takes the YAML's descriptor (descriptors: / every_board_descriptor:) if one is
  /// given, else its commands go out unchecked: the descriptor is optional there up to 20 boards on a Dig2Analog-Plus-5ch
  /// pin (the configuration check demands it above). A no-return board waiting for one would get NOP slots forever
  /// (bench 2026-10-29: the light on in ESPHome, the board dark).
  void require_descriptors(uint16_t n) {
    for (uint16_t b = 0; b < this->boards_.size() && b < n; b++) this->boards_[b].require_descriptor = true;
  }
  const ultra_descriptor_t *board_descriptor(uint16_t board) const {
    return board < this->boards_.size() && !this->boards_[board].descriptor.empty()
               ? &this->boards_[board].descriptor_view : nullptr;
  }
  void forget_descriptor(uint16_t board) {
    if (board < boards_.size()) {boards_[board].descriptor.clear();boards_[board].descriptor_view={};}
  }
  bool descriptor(uint16_t board, const std::vector<uint8_t> &bytes) {
    if (board >= boards_.size()) return false;
    auto &b = boards_[board];
    if (b.descriptor == bytes) return true;
    ultra_descriptor_t d{};
    if (bytes.size() > ULTRA_DESC_MAX_BYTES || ultra_descriptor_decode(&d, bytes.data(), bytes.size())) return false;
    b.descriptor = bytes;
    d.bytes=b.descriptor.data(); b.descriptor_view=d;
    if(b.domains) for(uint8_t k=0;k<14;k++) if(b.domains->have & (1u<<k)) {
      uint16_t applied;
      if(ultra_descriptor_config(&d,ULTRA_TARGET_BOARD,33+k,b.domains->values[k],&applied)) b.domains->have &= ~(1u<<k);
      else b.domains->values[k]=applied;
    }
    for (uint8_t id = 1; id < HOST_CONFIG_COUNT; id++) if (b.cfg_have & (1u << id)) {
      uint16_t applied;
      if (ultra_descriptor_config(&d, ULTRA_TARGET_BOARD, id, b.cfg[id], &applied)) b.cfg_have &= ~(1u << id);
      else b.cfg[id] = applied;
    }
    for (uint8_t ch = 0; ch < CHANNELS; ch++) for (uint8_t k = 0; k < 2; k++) {
      const uint16_t bit = 1u << (2 * ch + k);
      uint16_t applied;
      if (!(b.channel_have & bit)) continue;
      if (ultra_descriptor_config(&d, ch, 11 + k, b.channel_cfg[ch][k], &applied)) b.channel_have &= ~bit;
      else b.channel_cfg[ch][k] = applied;
    }
    if (b.hw) {
      ultra_desc_pixel_t px{};
      const bool have = ultra_descriptor_pixel(&d, 0, &px) == 0;
      for (uint8_t k = 0; k < ULTRA_CFG_PX_COUNT; k++) if (b.hw->have & (1u << k)) {
        uint16_t applied;
        if (!have || ultra_descriptor_config(&d, 0, ULTRA_CFG_PX_FIRST + k, b.hw->cfg[k], &applied)) b.hw->have &= ~(1u << k);
        else b.hw->cfg[k] = applied;
      }
      if (have && (b.hw->have & (1u << (ULTRA_CFG_PX_PIXELS - ULTRA_CFG_PX_FIRST)))) {
        const uint16_t chip = b.hw->cfg[ULTRA_CFG_PX_CHIP - ULTRA_CFG_PX_FIRST];
        const uint8_t bpp = chip == ULTRA_PXC_WS2805_RGBCCT ? 5 : chip == ULTRA_PXC_SK6812_RGBW || chip == ULTRA_PXC_WS2814_RGBW ? 4 : 3;
        auto &count = b.hw->cfg[ULTRA_CFG_PX_PIXELS - ULTRA_CFG_PX_FIRST];
        count = std::min<uint16_t>(count, std::min<uint16_t>(px.max_pixels[bpp - 3], px.frame_bytes / bpp));
      }
    }
    b.cfg_dirty = true;
    flush_config();
    if (d.outputs) pixel_source(board, b.stream_source);
    replay(0);
    return true;
  }
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
    // the 1 bits, four bytes at a time (SWAR: no table, no bit loop; the ESP32 has no popcount instruction)
    auto ones32 = [](uint32_t x) {
      x = x - ((x >> 1) & 0x55555555u);
      x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
      return (((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
    };
    uint32_t ones = 0;
    size_t i = 0;
    for (; i + 4 <= size; i += 4) {
      uint32_t w;
      std::memcpy(&w, bytes + i, 4);
      ones += ones32(w);
    }
    for (; i < size; i++)
      ones += ones32(bytes[i]);
    const uint64_t bits = static_cast<uint64_t>(size) * 8;
    const uint64_t all = ones * static_cast<uint64_t>(this->t1h_ + this->t1l_) +
                         (bits - ones) * static_cast<uint64_t>(this->t0h_ + this->t0l_);
    return all - ((bytes[size - 1] & 1U) ? this->t1l_ : this->t0l_);   // the last bit ends with its falling edge
  }

  /// Board setting (CONFIG id). Sent with flush_config(); identical values are not re-sent.
  void set_config(uint16_t board, uint8_t id, uint16_t value) {
    if (board < boards_.size() && id >= 33 && id <= 46) {
      auto &b = boards_[board];
      if (!resolve_config_(b, ULTRA_TARGET_BOARD, id, value)) return;
      if (!b.domains) b.domains.reset(new DomainConfig());
      const uint8_t k=id-33;
      if ((b.domains->have & (1u<<k)) && b.domains->values[k]==value) return;
      b.domains->values[k]=value; b.domains->have|=1u<<k; b.cfg_dirty=true; return;
    }
    if (board >= this->boards_.size() || !ultra_config_scope_ok(ULTRA_TARGET_BOARD, id) || id >= HOST_CONFIG_COUNT)
      return;
    Board &b = this->boards_[board];
    if (!resolve_config_(b, ULTRA_TARGET_BOARD, id, value)) return;
    if ((b.cfg_have & (1u << id)) && b.cfg[id] == value)
      return;
    b.cfg[id] = value;
    b.cfg_have |= 1u << id;
    b.cfg_dirty = true;
  }
  /// A setting no longer sent (the board keeps what it has; the next flush_config sends the others)
  void unset_config(uint16_t board, uint8_t id) {
    if (board < boards_.size() && id >= 33 && id <= 46) {
      auto &b=boards_[board]; if(b.domains) {b.domains->have &= ~(1u<<(id-33)); b.cfg_dirty=true;} return;
    }
    if (board >= this->boards_.size() || id >= HOST_CONFIG_COUNT)
      return;
    Board &b = this->boards_[board];
    if (b.cfg_have & (1u << id)) {
      b.cfg_have &= ~(1u << id);
      b.cfg_dirty = true;
    }
  }
  uint16_t get_config(uint16_t board, uint8_t id) const {
    if(board < boards_.size() && id>=33 && id<=46)
      return boards_[board].domains ? boards_[board].domains->values[id-33] : 0;
    return board < this->boards_.size() && id < HOST_CONFIG_COUNT ? this->boards_[board].cfg[id] : 0;
  }
  bool has_config(uint16_t board, uint8_t id) const {
    if(board < boards_.size() && id>=33 && id<=46)
      return boards_[board].domains && (boards_[board].domains->have & (1u<<(id-33)));
    return board < this->boards_.size() && id < HOST_CONFIG_COUNT &&
           (this->boards_[board].cfg_have & (1u << id)) != 0;
  }
  // Channel model and ceiling are scoped CONFIG 11 / 12.
  void set_channel_config(uint16_t board, uint8_t channel, uint8_t id, uint16_t value) {
    if (board >= boards_.size() || channel >= CHANNELS || (id != 11 && id != 12)) return;
    auto &b = boards_[board];
    const uint8_t k = id - 11;
    if (!resolve_config_(b, channel, id, value)) return;
    if ((b.channel_have & (1u << (2 * channel + k))) && b.channel_cfg[channel][k] == value) return;
    b.channel_cfg[channel][k] = value;
    b.channel_have |= 1u << (2 * channel + k);
    b.cfg_dirty = true;
  }
  uint16_t config_crc(uint16_t board, bool pixel_only = false) const {
    if (board >= boards_.size()) return 0;
    const auto &b = boards_[board];
    std::vector<uint8_t> records;
    auto add = [&](uint8_t target, uint8_t id, uint16_t value) {
      records.push_back(target); records.push_back(id);
      records.push_back(value >> 8); records.push_back(value & 255);
    };
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      if (!pixel_only) for (uint8_t k = 0; k < 2; k++)
        if (b.channel_have & (1u << (2 * ch + k))) add(ch, 11 + k, b.channel_cfg[ch][k]);
      if (ch == 0 && b.hw) for (uint8_t k = 0; k < ULTRA_CFG_PX_COUNT; k++)
        if (b.hw->have & (1u << k)) add(0, ULTRA_CFG_PX_FIRST + k, b.hw->cfg[k]);
    }
    if (!pixel_only) for (uint8_t id = 1; id < HOST_CONFIG_COUNT; id++)
      if (b.cfg_have & (1u << id)) add(ULTRA_TARGET_BOARD, id, b.cfg[id]);
    if (!pixel_only && b.domains) for(uint8_t k=0;k<14;k++)
      if(b.domains->have & (1u<<k)) add(ULTRA_TARGET_BOARD,33+k,b.domains->values[k]);
    return ultra_config_crc(records.data(), records.size() / 4);
  }
  /// A hardware-specific setting (CONFIG ids 64-77: the Diff-Power Ultra Receiver Digital-Medium's local pixel effects, ULTRA_CFG_PX_*).
  /// Like set_config(): sent with flush_config(), identical values are not re-sent, repeated by the refresh.
  void set_hw_config(uint16_t board, uint8_t id, uint16_t value) {
    if (board >= this->boards_.size() || id < ULTRA_CFG_PX_FIRST || id >= ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT)
      return;
    Board &b = this->boards_[board];
    if (id == ULTRA_CFG_PX_CHIP && value >= ULTRA_PXC_COUNT) return;
    if (!b.descriptor.empty() && ultra_descriptor_config(&b.descriptor_view, 0, id, value, &value)) return;
    const uint8_t k = id - ULTRA_CFG_PX_FIRST;
    if (!b.hw) {
      b.hw.reset(new HwConfig());
      restart_rotation_(b);   // (its list has more items now)
    }
    if ((b.hw->have & (1u << k)) && b.hw->cfg[k] == value)
      return;
    b.hw->cfg[k] = value;
    b.hw->have |= static_cast<uint16_t>(1u << k);
    b.cfg_dirty = true;
  }
  uint16_t get_hw_config(uint16_t board, uint8_t id) const {
    return board < this->boards_.size() && this->boards_[board].hw && id >= ULTRA_CFG_PX_FIRST &&
                   id < ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT
               ? this->boards_[board].hw->cfg[id - ULTRA_CFG_PX_FIRST]
               : 0;
  }
  bool has_hw_config(uint16_t board, uint8_t id) const {
    return board < this->boards_.size() && this->boards_[board].hw && id >= ULTRA_CFG_PX_FIRST &&
           id < ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT &&
           (this->boards_[board].hw->have & (1u << (id - ULTRA_CFG_PX_FIRST))) != 0;
  }
  /// Queue the changed settings (all of them in 3 CONFIG slots of up to 9 values, then the hardware-specific ones in
  /// HW_PAGES).
  void flush_config() {
    for (auto &b : this->boards_) {
      if (!b.cfg_dirty)
        continue;
      b.cfg_dirty = false;
      for(uint8_t page=0;b.domains && page<2;page++) {
        uint8_t ids[9];uint16_t vals[9];uint8_t n=0;
        for(uint8_t k=9*page;k<14 && k<9*(page+1);k++) if(b.domains->have & (1u<<k)) {
          ids[n]=33+k;vals[n++]=b.domains->values[k];
        }
        auto &it=b.domains->page[page];it.valid=n!=0;
        if(!it.valid) ultra_slot_nop(it.slot);
        if(n){ultra_slot_config(it.slot,ULTRA_TARGET_BOARD,next_seq_(b,ULTRA_TARGET_BOARD,ULTRA_CONFIG),ids,vals,n);
          queue_(b,key_(ULTRA_TARGET_BOARD,ULTRA_CONFIG,CONFIG_PAGES+HW_PAGES+page));}
      }
      for (uint8_t page = 0; b.hw && page < HW_PAGES; page++) {
        uint8_t ids[9];
        uint16_t vals[9];
        uint32_t n = 0;
        for (uint8_t k = page * 9; k < ULTRA_CFG_PX_COUNT && k < (page + 1) * 9; k++) {
          if (b.hw->have & (1u << k)) {
            ids[n] = static_cast<uint8_t>(ULTRA_CFG_PX_FIRST + k);
            vals[n++] = b.hw->cfg[k];
          }
        }
        Item &it = b.hw->page[page];
        it.valid = n != 0;
        if (!it.valid) {
          ultra_slot_nop(it.slot);
          continue;
        }
        const uint8_t seq = this->next_seq_(b, 0, ULTRA_CONFIG);
        ultra_slot_config(it.slot, 0, seq, ids, vals, n);
        it.start_us = NO_START;
        this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_CONFIG, static_cast<uint8_t>(CONFIG_PAGES + page)));
      }
      for (uint8_t ch = 0; ch < CHANNELS; ch++) {
        uint8_t ids[2]; uint16_t vals[2]; uint8_t n = 0;
        for (uint8_t k = 0; k < 2; k++) if (b.channel_have & (1u << (2 * ch + k))) {
          ids[n] = 11 + k; vals[n++] = b.channel_cfg[ch][k];
        }
        Item &it = b.config[BOARD_CONFIG_PAGES + ch];
        it.valid = n != 0;
        if(!it.valid) ultra_slot_nop(it.slot);
        if (n) {
          ultra_slot_config(it.slot, ch, next_seq_(b, ch, ULTRA_CONFIG), ids, vals, n);
          queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_CONFIG, BOARD_CONFIG_PAGES + ch));
        }
      }
      for (uint8_t page = 0; page < BOARD_CONFIG_PAGES; page++) {
        uint8_t ids[9];
        uint16_t vals[9];
        uint32_t n = 0;
        for (uint8_t id = 1 + page * 9; id < HOST_CONFIG_COUNT && id < 1 + (page + 1) * 9; id++) {
          if (b.cfg_have & (1u << id)) {
            ids[n] = id;
            vals[n++] = b.cfg[id];
          }
        }
        Item &it = b.config[page];
        it.valid = n != 0;
        if (!it.valid) {
          ultra_slot_nop(it.slot);
          continue;
        }
        ultra_slot_config(it.slot, ULTRA_TARGET_BOARD, this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_CONFIG), ids, vals, n);
        it.start_us = NO_START;
        this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_CONFIG, page));
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
                         max_p, 0);
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
                    uint32_t glide_ms, int64_t start_us, uint8_t flags = 0) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    Item &it = li_(b, light).set;
    ultra_slot_set(it.slot, light, this->next_seq_(b, light, ULTRA_SET), flags, m, a, bb, glide_ms, 0);
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

  /// Stored level: mode 0 = power-on, 1 = signal loss; both fade in.
  void light_scene(uint16_t board, uint8_t light, uint8_t mode, uint8_t fade_100ms, uint16_t m, const uint16_t *a,
                   const uint16_t *bb) {
    if (!this->valid_(board, light))
      return;
    Board &b = this->boards_[board];
    if (mode > 1) return;
    Item &it = mode == 0 ? li_(b, light).scene : li_(b, light).loss_scene;
    const Item &other = mode == 0 ? li_(b, light).loss_scene : li_(b, light).scene;
    uint8_t seq = this->next_seq_(b, light, ULTRA_SCENE);
    // Both selectors share the receiver's (light, type) key, including after SEQ wrap.
    if (other.valid && seq == other.slot[2])
      seq = this->next_seq_(b, light, ULTRA_SCENE);
    const uint8_t wire_mode = mode;
    ultra_slot_scene(it.slot, light, seq, wire_mode, fade_100ms, m, a, bb);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(light, ULTRA_SCENE, mode));
  }


  /// A local pixel effect of a Diff-Power-UR-Digital-M's segment (PIXEL_FX; light = the segment, 0 .. 3), starting at
  /// start_us: the effect's time zero on the host clock (the receivers' phase anchor; a change of its parameters keeps
  /// it, a new effect takes a new one). fx.effect 0 hands the strip back to the hub's stream. Sent with repeats like
  /// SET and part of the refresh (the receiver applies a SEQ once).
  void light_pixel_fx(uint16_t board, uint8_t segment, const ultra_pixel_fx_t &fx, int64_t start_us) {
    if (!this->valid_(board, segment) || segment >= HOST_PIXEL_SEGMENTS)
      return;
    Board &b = this->boards_[board];
    this->pixel_source(board, fx.effect == ULTRA_PX_FX_STOP);
    LightItems &li = li_(b, segment);
    if (!li.pixfx)
      li.pixfx.reset(new Item());
    Item &it = *li.pixfx;
    // RESTART is an action, not a state: the command's copies carry it, the stored slot (the refresh, a replay) does not,
    // or every refresh would start the effect over
    ultra_pixel_fx_t stored = fx;
    stored.flags = static_cast<uint8_t>(stored.flags & ~ULTRA_PXF_RESTART);
    it.restart = (fx.flags & ULTRA_PXF_RESTART) != 0;
    ultra_slot_pixel_fx(it.slot, 0, segment, this->next_seq_(b, segment, ULTRA_PIXEL_FX), &stored);
    it.valid = true;
    it.start_us = start_us;
    it.stream = false;
    this->queue_(b, key_(segment, ULTRA_PIXEL_FX, 0));
  }
  /// The segment's last PIXEL_FX is queued or sent (the light keeps its own copy of what it asked for)
  bool pixel_fx_valid(uint16_t board, uint8_t segment) const {
    return this->valid_(board, segment) && this->boards_[board].light[segment] &&
           this->boards_[board].light[segment]->pixfx && this->boards_[board].light[segment]->pixfx->valid;
  }

  void pixel_source(uint16_t board, bool stream) {
    if (board >= boards_.size()) return;
    auto &b = boards_[board];
    b.stream_source = stream;
    if (b.descriptor.empty()) return;
    uint8_t source = 0;
    if (stream) {
      ultra_desc_pixel_t px{};
      if (ultra_descriptor_pixel(&b.descriptor_view, 0, &px) || !(px.sources & ULTRA_SOURCE_MASK_STREAM)) return;
      for (uint8_t ingress = 0; ingress < 16; ingress++) if (px.ingress_mask & (1u << ingress)) {
        source = ULTRA_SOURCE_INGRESS(ingress); break;
      }
    }
    set_hw_config(board, ULTRA_CFG_PX_SOURCE, source);
    flush_config();
  }


  void identify(uint16_t board, bool on) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_identify(it.slot, this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_IDENTIFY), on ? 1 : 0);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_IDENTIFY, 0));
  }

  /// The board blinks its firmware version once on its feedback channel (IDENTIFY 5).
  void show_version(uint16_t board) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_identify(it.slot, this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_IDENTIFY), ULTRA_IDENT_VERSION);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_IDENTIFY, 0));
  }

  /// Dark hold: the board fades dark (0.3 s) and stays dark for hold_s seconds (0 = 60) or until release_dark();
  /// its engine keeps running underneath (CalSequencer).
  void hold_dark(uint16_t board, uint8_t hold_s) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_hold_dark(it.slot, this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_IDENTIFY), hold_s);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_IDENTIFY, 0));
  }
  void release_dark(uint16_t board) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    Item &it = b.identify;
    ultra_slot_identify(it.slot, this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_IDENTIFY), ULTRA_IDENT_RELEASE);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_IDENTIFY, 0));
  }
  /// A copy of the planned power-off notice (IDENTIFY 7 POWER_OFF, argument 0; its Diff-Power port times them: 0 /
  /// 50 / 100 ms): the board's supply drops soon. In its slot of the next frame, before anything queued; the first
  /// copy takes a new SEQ, the others repeat it (the board acts once).
  void power_off(uint16_t board, bool first) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    if (first)
      b.off_seq = this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_IDENTIFY);
    b.off_left = 1;
  }
  /// Boards whose notice copy hasn't gone out yet.
  uint16_t power_off_left() const {
    uint16_t n = 0;
    for (const auto &b : this->boards_)
      n += b.off_left != 0;
    return n;
  }
  /// The frame just built carried a notice copy (once: the hub reports it once the frame went out).
  bool take_power_off_sent() {
    const bool sent = this->off_sent_;
    this->off_sent_ = false;
    return sent;
  }
  /// A notice copy not sent by now is stale (its port woke again): dropped.
  void cancel_power_off() {
    for (auto &b : this->boards_)
      b.off_left = 0;
    this->off_sent_ = false;
  }

  /// The board's last IDENTIFY (identify, calibrate, dark, release) still has copies to send.
  bool identify_pending(uint16_t board) const {
    if (board >= this->boards_.size())
      return false;
    const uint16_t k = key_(ULTRA_TARGET_BOARD, ULTRA_IDENTIFY, 0);
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
    ultra_slot_calibrate(it.slot, this->next_seq_(b, ULTRA_TARGET_BOARD, ULTRA_IDENTIFY), duty_pct);
    it.valid = true;
    it.start_us = NO_START;
    this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_IDENTIFY, 0));
  }

  /// Everything again as urgent commands, transitions restarted at start_us (boards that were powered up meanwhile
  /// replay them; boards that kept their state ignore the repeats). Effects keep their own start: a board that comes
  /// back joins in phase with the others (a sunrise continues where it is).
  void replay(int64_t start_us) {
    for (auto &b : this->boards_) {
      restart_rotation_(b);
      for (uint8_t p = 0; p < CONFIG_PAGES; p++) {
        if (b.config[p].valid)
          this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_CONFIG, p));
      }
      for (uint8_t p = 0; b.hw && p < HW_PAGES; p++) {
        if (b.hw->page[p].valid)
          this->queue_(b, key_(ULTRA_TARGET_BOARD, ULTRA_CONFIG, static_cast<uint8_t>(CONFIG_PAGES + p)));
      }
      for(uint8_t p=0;b.domains && p<2;p++) if(b.domains->page[p].valid)
        queue_(b,key_(ULTRA_TARGET_BOARD,ULTRA_CONFIG,CONFIG_PAGES+HW_PAGES+p));
      for (uint8_t l = 0; l < HOST_LIGHTS; l++) {
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
        if (li.scene.valid)
          this->queue_(b, key_(l, ULTRA_SCENE, 0));
        if (li.loss_scene.valid)
          this->queue_(b, key_(l, ULTRA_SCENE, 1));
        if (li.pixfx && li.pixfx->valid) {
          li.pixfx->restart = false;   // (a replay is a state: the RESTART was an action, done)
          this->queue_(b, key_(l, ULTRA_PIXEL_FX, 0));   // (its START is kept: the effect continues in phase)
        }
      }
    }
  }

  /// The board's lights from first_unused on are not in the YAML (any more): an empty LIGHT_DEF (no channels), SCENE
  /// zero levels clear both stored scenes from an earlier configuration. Built on the fly (no per-light state), queued now and part of the refresh.
  void clear_unused_lights(uint16_t board, uint8_t first_unused) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    b.first_unused = first_unused > HOST_LIGHTS ? HOST_LIGHTS : first_unused;
    restart_rotation_(b);
    for (uint8_t l = b.first_unused; l < HOST_LIGHTS; l++) {
      for (uint8_t type : {ULTRA_LIGHT_DEF, ULTRA_SCENE}) {
        this->next_seq_(b, l, type);
        this->queue_(b, key_(l, type, 0));
        if (type == ULTRA_SCENE) {
          this->next_seq_(b, l, type);  // reserve one SEQ for each stored level
          this->queue_(b, key_(l, type, 1));
        }
      }
    }
  }

  /// Commands waiting to be sent.
  bool pending() const {
    for (const auto &b : this->boards_) {
      if (b.require_descriptor && b.descriptor.empty()) continue;
      if (!b.queue.empty() || b.off_left > 0)
        return true;
    }
    return false;
  }
  size_t queued(uint16_t board) const { return board < this->boards_.size() ? this->boards_[board].queue.size() : 0; }
  /// The board needs a descriptor and has none: its slots stay empty (the log says why its changes don't go out)
  bool descriptor_missing(uint16_t board) const {
    return board < this->boards_.size() && this->boards_[board].require_descriptor && this->boards_[board].descriptor.empty();
  }
  /// A power_limit on the board (a board or channel maximum current): it may lower the levels it is sent
  bool power_limited(uint16_t board) const {
    if (board >= this->boards_.size()) return false;
    const auto &b = this->boards_[board];
    if ((b.cfg_have & (1u << ULTRA_CFG_BOARD_MAX)) && b.cfg[ULTRA_CFG_BOARD_MAX] != 0)
      return true;
    for (uint8_t ch = 0; ch < CHANNELS; ch++)
      if ((b.channel_have & (1u << (2 * ch + 1))) && b.channel_cfg[ch][1] != 0) return true;
    return false;
  }
  /// First copies waiting on a board: frames before a new command's first copy goes out.
  size_t waiting(uint16_t board) const {
    return board < this->boards_.size() ? this->boards_[board].unsent + this->boards_[board].off_left : 0;
  }
  /// ... on the busiest board.
  size_t waiting() const {
    size_t n = 0;
    for (const auto &b : this->boards_)
      n = b.unsent + b.off_left > n ? b.unsent + b.off_left : n;
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
      n += b.descriptor.capacity() + (b.hw ? sizeof(HwConfig) : 0) + (b.domains ? sizeof(DomainConfig) : 0);
      for (const auto &l : b.light) {
        if (l)
          n += sizeof(LightItems) + (l->pixfx ? 1 : 0) * sizeof(Item);
      }
    }
    this->queue_bytes_peak_ = this->queue_bytes_;
    return n;
  }

  /// Builds one frame into out (frame_bytes()) that starts on the wire at start_us (host clock). Returns its TIME:
  /// the host clock at the last bit (the header carries the low 32 bits).
  int64_t build_frame(int64_t start_us, uint8_t *out) {
    uint8_t *s = out;
    for (size_t i = 0; i < this->boards_.size(); i++, s += ULTRA_SLOT_BYTES)
      this->fill_slot_(i, s, start_us);
    this->hdr_flags_ = 0;
    return this->finish_(start_us, out);
  }

  /// An RW frame (the return channel, ultra_telemetry.h): its first `polls` slots are the POLLs from poll_slots, the
  /// others as build_frame() fills them; the header has ULTRA_HDR_RW. (The hub sends one only while nothing is queued:
  /// commands go first.)
  int64_t build_rw_frame(int64_t start_us, uint8_t *out, const uint8_t *poll_slots, size_t polls) {
    uint8_t *s = out;
    for (size_t i = 0; i < this->boards_.size(); i++, s += ULTRA_SLOT_BYTES) {
      if (i < polls) {
        std::memcpy(s, poll_slots + ULTRA_SLOT_BYTES * i, ULTRA_SLOT_BYTES);
        this->starts_[i] = NO_START;
        this->floors_[i] = 0;
        this->durations_[i] = 0;
      } else {
        this->fill_slot_(i, s, start_us);
      }
    }
    this->hdr_flags_ = ULTRA_HDR_RW;
    return this->finish_(start_us, out);
  }

  // A digital calibration owns the entire chain. Queued state is kept for release, never consumed here.
  int64_t build_calibration_frame(int64_t start_us, uint8_t *out, uint16_t board, uint8_t action,
                                  const uint8_t *poll_slots = nullptr, size_t polls = 0) {
    this->build_idle_frame(start_us, out);
    if (polls) {
      std::memcpy(out, poll_slots, polls * ULTRA_SLOT_BYTES);
      this->hdr_flags_ = ULTRA_HDR_RW;
    } else if (action && board < this->boards_.size()) {
      ultra_slot_pixel_calibration(out + board * ULTRA_SLOT_BYTES, 0,
          this->next_seq_(this->boards_[board], 0, ULTRA_IDENTIFY), action);
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
    this->hdr_flags_ = 0;
    return this->finish_(start_us, out);
  }
  /// A preamble frame: DARK slots (all zero) + header, see preamble_frames().
  int64_t build_dark_frame(int64_t start_us, uint8_t *out) {
    for (size_t i = 0; i < this->boards_.size(); i++) {
      ultra_slot_dark(out + ULTRA_SLOT_BYTES * i);
      this->starts_[i] = NO_START;
      this->floors_[i] = 0;
    }
    this->hdr_flags_ = 0;
    return this->finish_(start_us, out);
  }

 protected:
  static constexpr uint8_t BOARD_CONFIG_PAGES = 3;
  static constexpr uint8_t CONFIG_PAGES = BOARD_CONFIG_PAGES + CHANNELS;
  static constexpr uint8_t HW_PAGES = (ULTRA_CFG_PX_COUNT + 8) / 9;   // CONFIG 64-77 (ULTRA_CFG_PX_*): 2 slots
  static_assert(1 + CONFIG_PAGES * 9 >= HOST_CONFIG_COUNT, "every official CONFIG id in a CONFIG slot");

  struct Item {
    uint8_t slot[ULTRA_SLOT_BYTES]{};
    int64_t start_us{NO_START};
    uint32_t glide_ms{0};   // streamed SET: the glide asked for
    bool stream{false};     // streamed SET: sent once, never starts before it arrives
    bool restart{false};    // PIXEL_FX: its copies carry ULTRA_PXF_RESTART (the stored slot does not)
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
    Item def, opt, set, scene, loss_scene;
    std::unique_ptr<Item> pixfx;   // a Diff-Power-UR-Digital-M's segment (light = segment): its local pixel effect
    int64_t last_stream_us{NO_START};  // when its last streamed SET went out
  };
  /// Per board ~410 B, + ~430 B per light in use (allocated with its first command): 50 boards with one light each
  /// take ~42 KB of the ESP32's heap (wiki: Ultra line options). The queue holds keys (12 B per command) and is freed whenever
  /// it empties.
  static_assert(HOST_CONFIG_COUNT <= 32, "Board::cfg_have holds a bit per CONFIG id");
  /// Refresh: CONFIG, then each light's DEF, OPT, SET, two stored levels, PIXEL_FX; hardware CONFIG last.
  static constexpr uint16_t REFRESH_PER_LIGHT = 6;
  static constexpr uint16_t REFRESH_MAIN = CONFIG_PAGES + HOST_LIGHTS * REFRESH_PER_LIGHT;
  static constexpr uint16_t REFRESH_ITEMS = REFRESH_MAIN + HW_PAGES + 2;
  static_assert(REFRESH_ITEMS <= 255, "Board::rr holds a refresh position");
  /// The hardware-specific CONFIG ids (64-77, ULTRA_CFG_PX_*) of a board that has them, in HW_PAGES slots
  struct HwConfig {
    Item page[HW_PAGES];
    uint16_t cfg[ULTRA_CFG_PX_COUNT]{};
    uint16_t have{0};   // the ids set (bit per id - 64)
  };
  struct DomainConfig { Item page[2];uint16_t values[14]{};uint16_t have{0}; };
  struct Board {   // (the slots first: no padding)
    std::vector<uint8_t> descriptor;
    ultra_descriptor_t descriptor_view{};
    bool require_descriptor{false};
    bool stream_source{false};
    Item config[CONFIG_PAGES];
    Item identify;  // the last IDENTIFY (identify, version, dark, release, calibrate)
    std::vector<Cmd> queue;
    std::unique_ptr<LightItems> light[HOST_LIGHTS];
    uint16_t cfg[HOST_CONFIG_COUNT]{};
    uint16_t channel_cfg[CHANNELS][2]{};
    uint16_t channel_have{0};
    std::unique_ptr<HwConfig> hw;   // CONFIG 64-77 (only a board that has them: a Diff-Power-UR-Digital-M)
    std::unique_ptr<DomainConfig> domains;
    uint8_t rr{0};    // the refresh list's next position (REFRESH_ITEMS of them)
    uint8_t rot{0};   // complete refresh rounds since its init / a resync / a change of its lights: 0..6, then 3..6
    uint32_t cfg_have{0};   // the CONFIG ids set (bit per id)
    bool cfg_dirty{false};
    uint8_t seq[HOST_LIGHTS + 1][ULTRA_PIXEL_FX + 1]{};   // (ULTRA_PIXEL_FX is the last type; ULTRA_TYPES stays below it)
    uint8_t urgent_run{0};   // queued commands sent in a row (build_frame: every 16th slot a refresh)
    uint8_t first_unused{HOST_LIGHTS};   // light indices from here on are not in the YAML
    uint8_t unsent{0};                    // queued commands of which no copy went out yet
    uint8_t off_left{0}, off_seq{0};      // the power-off notice: a copy to send, its SEQ
  };

  /// The commands that clear a light index not in the YAML (clear_unused_lights), with its current sequence number.
  static Item unused_item_(const Board &b, uint8_t light, uint8_t type, uint8_t sub = 0) {
    Item it;
    const uint16_t zero[CHANNELS] = {};
    const uint8_t roles[CHANNELS] = {};
    const uint8_t seq = static_cast<uint8_t>(b.seq[light][type] - (type == ULTRA_SCENE && sub == 0));
    if (type == ULTRA_LIGHT_DEF)
      ultra_slot_light_def(it.slot, light, seq, roles, 1000, 0, zero, zero, 0);
    else if (type == ULTRA_SCENE)
      ultra_slot_scene(it.slot, light, seq, sub, 3, 0, zero, zero);
    it.valid = true;
    return it;
  }

  // Pixel-effect anchors are renewed by the digital light; other starts are SET transition starts.
  static int64_t recent_start_(const Item &it, int64_t) { return it.start_us; }

  static uint16_t key_(uint8_t light, uint8_t type, uint8_t sub) {
    return static_cast<uint16_t>(((light & 0x0F) << 12) | ((type & 0x0F) << 8) | sub);
  }
  bool valid_(uint16_t board, uint8_t light) const { return board < this->boards_.size() && light < HOST_LIGHTS; }
  static LightItems &li_(Board &b, uint8_t light) {
    if (!b.light[light]) {
      b.light[light].reset(new LightItems());
      restart_rotation_(b);   // (its lights changed)
    }
    return *b.light[light];
  }
  /// The refresh rotation from its start: the list from its first item, the clearing slots in every round again
  static void restart_rotation_(Board &b) {
    b.rr = 0;
    b.rot = 0;
  }
  static uint8_t next_seq_(Board &b, uint8_t light, uint8_t type) {
    return ++b.seq[light < HOST_LIGHTS ? light : HOST_LIGHTS][type];
  }

  /// The item a queued key stands for: the queue holds keys, the items live in the board's state, so a copy that goes
  /// out is always the latest command of its key.
  static Item queued_item_(const Board &b, uint16_t key) {
    const uint8_t light = key >> 12, type = (key >> 8) & 15, sub = key & 255;
    if (type == ULTRA_CONFIG) {
      if(sub>=CONFIG_PAGES+HW_PAGES) return b.domains && sub<CONFIG_PAGES+HW_PAGES+2 ? b.domains->page[sub-CONFIG_PAGES-HW_PAGES] : Item{};
      if (sub >= CONFIG_PAGES)
        return b.hw && sub - CONFIG_PAGES < HW_PAGES ? b.hw->page[sub - CONFIG_PAGES] : Item{};
      return b.config[sub];
    }
    if (type == ULTRA_IDENTIFY)
      return b.identify;
    if (!b.light[light])
      return unused_item_(b, light, type, sub);
    const LightItems &li = *b.light[light];
    switch (type) {
      case ULTRA_LIGHT_DEF:
        return li.def;
      case ULTRA_LIGHT_OPT:
        return li.opt;
      case ULTRA_SET:
        return li.set;
      case ULTRA_SCENE:
        return sub == 0 ? li.scene : li.loss_scene;
      case ULTRA_PIXEL_FX:
        return li.pixfx ? *li.pixfx : Item{};
      default:
        return Item{};
    }
  }

  static bool resolve_config_(const Board &b,uint8_t target,uint8_t id,uint16_t &value) {
    if (!ultra_config_scope_ok(target,id)) return false;
    if (b.descriptor.empty()) return true;
    return !ultra_descriptor_config(&b.descriptor_view,target,id,value,&value);
  }
  static bool slot_allowed_(const Board &b,const uint8_t *s) {
    if (b.descriptor.empty()) return !b.require_descriptor;
    const auto &d=b.descriptor_view;
    const uint8_t type=s[0]&ULTRA_TYPE_MASK,target=s[1];
    if (s[0]&ULTRA_TYPE_BCAST || !ultra_command_known(s[0])) return false;
    if(type==ULTRA_NOP || type==ULTRA_POLL) return true;
    if(type==ULTRA_IDENTIFY) return target==ULTRA_TARGET_BOARD || target<d.outputs;
    if(type==ULTRA_CONFIG) {
      for(uint8_t k=0;k<9;k++){const auto id=s[3+3*k];uint16_t applied;
        if(id && ultra_descriptor_config(&d,target,id,ultra_get16(s+4+3*k),&applied))return false;}
      return true;
    }
    if(type==ULTRA_PIXEL_FX){ultra_desc_pixel_t p{};return !ultra_descriptor_pixel(&d,target>>4,&p) &&
      (p.flags&ULTRA_PIXEL_LOCAL) && ((target&15)==15 || (target&15)<p.segments);}
    if(target>=d.lights)return false;
    if(type==ULTRA_LIGHT_DEF) for(uint8_t k=0;k<5;k++)
      if(((s[3+k/2]>>((k&1)?0:4))&15) && s[29]+k>=d.channels)return false;
    return true;
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

  /// Start offset of SET (int16 ms) / PIXEL_FX (int32 ms) relative to this frame's host ms floor(TIME / 1000), then
  /// the CRC. The absolute start is rounded to its ms once, so every frame carrying an item of that start names the
  /// same host ms, and every board adds it to its own floor(TIME / 1000), on the host's ms grid (the TIME wrap is whole
  /// ms). floor: not before the frame's ms (a streamed SET starts when it arrives, at the latest). A SET that started
  /// more than 32.8 s ago (a long transition refreshed to a board that missed it) starts at that limit with its
  /// duration (the SET's own) shortened by the rest: it still ends on time, and one that is over shows its target at once.
  static void stamp_(uint8_t *s, int64_t start_us, int64_t hdr_us, bool floor, uint32_t duration) {
    if (start_us == NO_START)
      return;
    int64_t ms = (start_us >= 0 ? (start_us + 500) / 1000 : -((-start_us + 499) / 1000)) - hdr_us / 1000;   // (hdr_us >= 0)
    if (floor && ms < 0)
      ms = 0;
    if ((s[0] & ULTRA_TYPE_MASK) == ULTRA_SET) {
      if (ms < -32768) {
        const uint64_t lost = static_cast<uint64_t>(-32768 - ms);
        duration = lost >= duration ? 0 : duration - static_cast<uint32_t>(lost);
      }
      ultra_put24(s + ULTRA_PAYLOAD + 22, duration);
      ms = ms < -32768 ? -32768 : ms > 32767 ? 32767 : ms;
      ultra_put16(s + ULTRA_PAYLOAD + 25, static_cast<uint16_t>(static_cast<int16_t>(ms)));
    } else if (s[0] == ULTRA_PIXEL_FX) {   // (START is at slot byte 4 in both)
      ms = ms < INT32_MIN ? INT32_MIN : ms > INT32_MAX ? INT32_MAX : ms;
      ultra_put32(s + ULTRA_PAYLOAD + 1, static_cast<uint32_t>(static_cast<int32_t>(ms)));
    }
    ultra_slot_seal(s);
  }

  /// Slot i of a frame: a queued command, else the board's next refresh item, else NOP.
  void fill_slot_(size_t i, uint8_t *s, int64_t start_us) {
    this->starts_[i] = NO_START;
    this->floors_[i] = 0;
    Board &b = this->boards_[i];
    if (b.require_descriptor && b.descriptor.empty()) { ultra_slot_nop(s); return; }
    // the power-off notice first (consecutive frames); every 16th slot of a board with a continuous stream is a
    // refresh: a board that rebooted meanwhile still gets its CONFIG and light definitions
    if (b.off_left > 0) {
      ultra_slot_power_off(s, b.off_seq);
      b.off_left = 0;
      this->off_sent_ = true;
    } else if (b.urgent_run >= 15 && this->take_refresh_(b, s, i, start_us)) {
      b.urgent_run = 0;
    } else if (this->take_urgent_(b, s, i, start_us)) {
      b.urgent_run++;
    } else {
      b.urgent_run = 0;
      if (!this->take_refresh_(b, s, i, start_us))
        ultra_slot_nop(s);
    }
    if (!slot_allowed_(b,s)) {ultra_slot_nop(s);this->starts_[i]=NO_START;this->floors_[i]=0;}
    this->durations_[i] = (s[0] & ULTRA_TYPE_MASK) == ULTRA_SET ? ultra_get24(s + ULTRA_PAYLOAD + 22) : 0;
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
    if (item.restart && s[0] == ULTRA_PIXEL_FX) {
      s[ULTRA_PX_FLAGS] = static_cast<uint8_t>(s[ULTRA_PX_FLAGS] | ULTRA_PXF_RESTART);
      ultra_slot_seal(s);
    }
    this->starts_[slot] = recent_start_(item, now_us);
    if (item.stream) {
      LightItems &li = li_(b, static_cast<uint8_t>(s[1] % HOST_LIGHTS));
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

  /// Next refresh item, including both zero levels for removed lights.
  /// The rotation (the same in the QuinLED Ultra sender library): the clearing slots are in every round for the first
  /// 3 complete rounds after the board's init, a resync or a change of its lights, then in every 4th round only
  /// (rounds 0, 1, 2, 6, 10, ...); the used lights' items in every round. A round ends where the list starts again.
  bool take_refresh_(Board &b, uint8_t *s, size_t slot, int64_t now_us) {
    const uint16_t per_light = REFRESH_PER_LIGHT;
    const uint16_t total = REFRESH_ITEMS;
    for (uint16_t tries = 0; tries < total; tries++) {
      const uint16_t pos = b.rr;
      const uint8_t rot = b.rot;   // (the round this position is in)
      b.rr = static_cast<uint8_t>((b.rr + 1) % total);
      if (b.rr == 0)
        b.rot = b.rot < 6 ? static_cast<uint8_t>(b.rot + 1) : 3;   // a round done: 0..6, then 3, 4, 5, 6, 3, ...
      const Item *it = nullptr;
      if(pos>=REFRESH_MAIN+HW_PAGES) {
        it=b.domains?&b.domains->page[pos-REFRESH_MAIN-HW_PAGES]:nullptr;
      } else if (pos >= REFRESH_MAIN) {   // the tail: the hardware-specific CONFIG pages
        it = b.hw ? &b.hw->page[pos - REFRESH_MAIN] : nullptr;
      } else if (pos < CONFIG_PAGES) {
        it = &b.config[pos];
      } else {
        const uint16_t l = (pos - CONFIG_PAGES) / per_light, k = (pos - CONFIG_PAGES) % per_light;
                if (!b.light[l]) {
          if (l < b.first_unused || (k != 0 && k != 3 && k != 4) || !(rot < 3 || rot % 4 == 2))
            continue;
          const uint8_t type = k == 0 ? ULTRA_LIGHT_DEF : ULTRA_SCENE;
          std::memcpy(s, unused_item_(b, static_cast<uint8_t>(l), type, k == 4 ? 1 : 0).slot, ULTRA_SLOT_BYTES);
          return true;
        }
        const LightItems &li = *b.light[l];
        it = k == 0 ? &li.def : k == 1 ? &li.opt : k == 2 ? &li.set
             : k == 3 ? &li.scene : k == 4 ? &li.loss_scene : li.pixfx.get();
      }
      if (it == nullptr || !it->valid)
        continue;
      // Host effects supply their next SET themselves. Refreshing an old sample would consume the bus budget,
      // and can put two copies of a sample on consecutive frames at the periodic refresh turn.
      if (it->stream && (it->slot[0] & ULTRA_TYPE_MASK) == ULTRA_SET)
        continue;
      std::memcpy(s, it->slot, ULTRA_SLOT_BYTES);
      this->starts_[slot] = recent_start_(*it, now_us);
      this->floors_[slot] = it->stream ? 1 : 0;
      return true;
    }
    return false;
  }

  /// TIME = start + the frame's bits on the wire; the start offsets of the slots refer to its ms, and they are bits of
  /// the frame themselves: stamp against an estimate, then once more against the result (an offset only changes
  /// when the frame's ms flips; the header bits are counted with a TIME that may be off by those few bits), and a
  /// last time against the TIME written: the offsets always count from the header's own ms.
  int64_t finish_(int64_t start_us, uint8_t *out) {
    const size_t n = this->boards_.size(), slots = ULTRA_SLOT_BYTES * n;
    uint8_t *h = out + slots;
    const uint64_t avg = (this->t0h_ + this->t0l_ + this->t1h_ + this->t1l_) / 2;
    int64_t t = start_us + static_cast<int64_t>((this->frame_bytes() * 8 * avg) / 1000);
    for (int pass = 0; pass < 2; pass++) {
      for (size_t i = 0; i < n; i++)
        stamp_(out + ULTRA_SLOT_BYTES * i, this->starts_[i], t, this->floors_[i] != 0, this->durations_[i]);
      ultra_build_header_flags(h, ultra_time_wire(static_cast<uint64_t>(t)), this->hdr_flags_);
      t = start_us + static_cast<int64_t>((this->wire_ns(out, slots + ULTRA_HDR_BYTES) + 500) / 1000);
    }
    for (size_t i = 0; i < n; i++)
      stamp_(out + ULTRA_SLOT_BYTES * i, this->starts_[i], t, this->floors_[i] != 0, this->durations_[i]);
    ultra_build_header_flags(h, ultra_time_wire(static_cast<uint64_t>(t)), this->hdr_flags_);
    return t;
  }

  std::vector<Board> boards_;
  std::vector<int64_t> starts_;      // per slot of the frame being built: its absolute start (SET / EFFECT)
  std::vector<uint32_t> durations_;  // per slot: a SET's own duration (stamp_ may shorten it in the frame)
  std::vector<uint8_t> floors_;      // per slot: a streamed SET (start not before the frame's TIME)
  uint8_t hdr_flags_{0};             // the frame being built: ULTRA_HDR_RW for an RW frame (retime_frame keeps it)
  uint32_t t0h_{300}, t0l_{950}, t1h_{950}, t1l_{300};
  uint8_t repeats_{3};
  bool off_sent_{false};        // the frame just built carried a power-off notice copy
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
  static constexpr uint32_t TEST_MS = 1270;              // the standard test (board isense.h: 70 + 20 x 60 ms)
  static constexpr uint32_t TEST_ONE_LEVEL_MS = 360;     // one level only (6 x 60 ms)
  static constexpr uint32_t MARGIN_MS = 150;             // after a test, before the next board's command
  static constexpr uint32_t SEND_TIMEOUT_MS = 2000;      // a command that can't get out: move on
  static constexpr uint8_t MAX_ADDED = 4;                // boards added to a run (add): 4 x 1.4 s within the hold's 10 s
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
    this->added_ = 0;
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
  /// One more board into the run (a receiver's button while its line runs): tested after the others, when it isn't in
  /// the run yet, the run is the standard test and the dark hold's slack still covers it (MAX_ADDED). false: not added.
  bool add(uint16_t board) {
    if (this->phase_ == IDLE || this->includes(board))
      return this->phase_ != IDLE;
    if (board >= this->num_boards_ || this->duty_ != 0 || this->added_ >= MAX_ADDED)
      return false;
    this->jobs_.push_back(board);
    this->added_++;
    return true;
  }
  /// the board is in the run (to be tested, being tested, or tested and held dark until the run is over)
  bool includes(uint16_t board) const {
    return this->phase_ != IDLE && std::find(this->jobs_.begin(), this->jobs_.end(), board) != this->jobs_.end();
  }
  /// the boards of the run (after EV_DONE: the last run's)
  const std::vector<uint16_t> &jobs() const { return this->jobs_; }
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
  uint8_t added_{0};
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

}  // namespace esphome::ultra_line
