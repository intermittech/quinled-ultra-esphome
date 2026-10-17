// Host test for the hub's side of the Ultra-Digital receiver's local pixel effects (protocol 0.2.8, PIXEL_FX) and its
// chain mode (0.2.9, CONFIG 140, page 0x81 part 3):
//  - components/dig2analog_ultra/ultra_pixel_fx.h: the chips and colour orders, the PIXEL_FX a light's state becomes,
//    page 0x81's decoding, and the tracker that decides when something new is due (a new effect = a new phase anchor, a
//    change of its parameters keeps it, the hand-back to the stream, the 20 day anchor, the STORE copy, lit);
//  - ultra_core.h's Scheduler: PIXEL_FX slots (the START stamp, copies, replacement, the refresh, a replay), CONFIG 129-140
//    in their own slots, a board without them unchanged; a normal Ultra line whose last board is the receiver (its slot,
//    the boards in front untouched);
//  - ultra_telemetry.h: page 0x81 read from a modelled Ultra-Digital receiver (HWCAP PIXEL_FX only; the maps once; UNSUPPORTED
//    not asked again; a receiver without the capability never sees the page; part 3, the line, only with HWCAP_CHAIN on a
//    normal Ultra line);
//  - ultra_pixel_fx.h: part 3's decoding (the protocol's test vector answers) and the line's status text.
// The light platform itself (ultra_digital.cpp) runs inside ESPHome: tests/validate_configs.py checks its schema and the
// examples compile.
//
// Build and run: python tests/run_ultra_digital_test.py

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/ultra_pixel_fx.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"

#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

using namespace esphome::dig2analog_ultra;

static int failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      failures++;                                         \
      std::printf("  FAIL %s:%d: ", __FILE__, __LINE__);  \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

static std::string hex(const uint8_t *p, size_t n) {
  static const char *d = "0123456789abcdef";
  std::string s;
  for (size_t i = 0; i < n; i++) {
    s += d[p[i] >> 4];
    s += d[p[i] & 15];
  }
  return s;
}

// ------------------------------------------------------------------------------------------------------ the chips
static void test_chips() {
  for (int c = 0; c < PIXEL_CHIP_COUNT; c++)   // (tests/run_ultra_digital_test.py compares these with light.py's table)
    std::printf("CHIP %d %u %s %u\n", c, PIXEL_CHIPS[c].bytes, PIXEL_CHIPS[c].own_order, PIXEL_CHIPS[c].preset);
  uint16_t v = 0;
  CHECK(pixel_order_value(PIXEL_CHIP_WS2812B, "GRB", &v) && v == ULTRA_PX_ORDER(1, 0, 2, 0, 0), "GRB = G R B (%04x)", v);
  CHECK(pixel_order_value(PIXEL_CHIP_WS2812B, "brg", &v) && v == ULTRA_PX_ORDER(2, 0, 1, 0, 0), "case-insensitive");
  CHECK(pixel_order_value(PIXEL_CHIP_WS2814, "WRGB", &v) && v == ULTRA_PX_ORDER(3, 0, 1, 2, 0), "WS2814 WRGB (%04x)", v);
  CHECK(pixel_order_value(PIXEL_CHIP_SK6812_RGBW, "GRBW", &v) && v == ULTRA_PX_ORDER(1, 0, 2, 3, 0), "SK6812 GRBW");
  CHECK(pixel_order_value(PIXEL_CHIP_WS2805, "RGBWC", &v) && v == ULTRA_PX_ORDER(0, 1, 2, 3, 4), "WS2805 RGBWC (%04x)", v);
  CHECK(pixel_order_value(PIXEL_CHIP_WS2805, "RGBCW", &v) && v == ULTRA_PX_ORDER(0, 1, 2, 4, 3), "WS2805 RGBCW");
  CHECK(!pixel_order_value(PIXEL_CHIP_WS2812B, "RGBW", &v), "3 bytes: no W");
  CHECK(!pixel_order_value(PIXEL_CHIP_WS2814, "RGB", &v), "4 bytes: 4 letters");
  CHECK(!pixel_order_value(PIXEL_CHIP_WS2812B, "RGG", &v), "each channel once");
  CHECK(!pixel_order_value(PIXEL_CHIP_WS2812B, "RGX", &v), "unknown letter");
  CHECK(!pixel_order_value(PIXEL_CHIP_WS2814, "RGBC", &v), "C is the WS2805's cold white only");
  // the chips' own orders: the header's wording
  for (int c = 0; c < PIXEL_CHIP_COUNT; c++)
    CHECK(pixel_order_value(static_cast<PixelChip>(c), PIXEL_CHIPS[c].own_order, &v), "own order of chip %d is valid", c);
  CHECK(PIXEL_CHIPS[PIXEL_CHIP_WS2805].bytes == 5 && PIXEL_CHIPS[PIXEL_CHIP_WS2814].bytes == 4, "bytes per pixel");
  CHECK(PIXEL_CHIP_COUNT == ULTRA_PXC_COUNT, "a table row for every preset");
  std::printf("  chips: %d presets, orders\n", PIXEL_CHIP_COUNT);
}

// ----------------------------------------------------------------------------------------- the slot a state becomes
static PixelLightState sample() {
  PixelLightState s;
  s.on = true;
  s.brightness = 0.5f;
  s.red = 1.0f;
  s.green = 0.5f;
  s.blue = 0.0f;
  s.c2[0] = 10, s.c2[1] = 20, s.c2[2] = 30;
  s.c3[0] = 40, s.c3[1] = 50, s.c3[2] = 60;
  s.effect = 7;
  s.speed = 100;
  s.intensity = 150;
  s.palette = ULTRA_PX_PAL_COLORS;
  s.custom1 = 1;
  s.custom2 = 2;
  s.transition_ms = 1500;
  s.seg_start = 5;
  s.seg_len = 90;
  s.flags = ULTRA_PXF_REVERSE | (ULTRA_PXW_ACCURATE << ULTRA_PXF_WHITE_SHIFT);
  return s;
}

static void test_slot() {
  uint8_t slot[ULTRA_SLOT_BYTES];
  ultra_pixel_fx_t f = pixel_fx_of(sample());
  f.start_ms = -1234;
  ultra_slot_pixel_fx(slot, 2, 9, &f);
  CHECK(ultra_slot_check(slot) == 0, "CRC");
  CHECK(slot[0] == ULTRA_PIXEL_FX && (slot[1] >> 4) == 2 && slot[2] == 9, "type, segment, SEQ");
  CHECK(slot[ULTRA_PX_EFFECT] == 7 && slot[ULTRA_PX_SPEED] == 100 && slot[ULTRA_PX_INTENSITY] == 150 &&
            slot[ULTRA_PX_PALETTE] == ULTRA_PX_PAL_COLORS,
        "effect, speed, intensity, palette");
  CHECK(slot[ULTRA_PX_C1] == 255 && slot[ULTRA_PX_C1 + 1] == 128 && slot[ULTRA_PX_C1 + 2] == 0, "c1 (%u %u %u)",
        slot[ULTRA_PX_C1], slot[ULTRA_PX_C1 + 1], slot[ULTRA_PX_C1 + 2]);
  CHECK(slot[ULTRA_PX_C2] == 10 && slot[ULTRA_PX_C2 + 2] == 30 && slot[ULTRA_PX_C3] == 40 && slot[ULTRA_PX_C3 + 2] == 60,
        "c2, c3");
  CHECK(slot[ULTRA_PX_BRIGHTNESS] == 128, "brightness 0.5 = 128");
  CHECK(ultra_get16(slot + ULTRA_PX_TRANSITION) == 1500, "transition");
  CHECK(ultra_get16(slot + ULTRA_PX_SEG_START) == 5 && ultra_get16(slot + ULTRA_PX_SEG_LEN) == 90, "segment range");
  CHECK(slot[ULTRA_PX_FLAGS] == (ULTRA_PXF_REVERSE | (ULTRA_PXW_ACCURATE << ULTRA_PXF_WHITE_SHIFT)), "flags");
  CHECK(slot[ULTRA_PX_CUSTOM1] == 1 && slot[ULTRA_PX_CUSTOM2] == 2, "custom1, custom2");
  CHECK(static_cast<int32_t>(ultra_get32(slot + ULTRA_PX_START)) == -1234, "START");
  // the golden (the protocol repository's test vector slot_pixel_fx is the same bytes: tests/run_ultra_digital_test.py)
  std::printf("VEC_SLOT_PIXEL_FX = \"%s\"\n", hex(slot, ULTRA_SLOT_BYTES).c_str());
  // the receiver firmware's own self-test vector (ud-fx-core, HANDOFF-protocol.md): an independent encoder agrees
  {
    ultra_pixel_fx_t h;
    std::memset(&h, 0, sizeof h);
    h.effect = 9;
    h.start_ms = -1500;
    h.speed = 1;
    h.intensity = 2;
    h.palette = ULTRA_PX_PAL_COLORS;
    for (int i = 0; i < 9; i++)
      h.c[i / 3][i % 3] = static_cast<uint8_t>(10 + i);
    h.brightness = 200;
    h.transition_ms = 700;
    h.seg_start = 12;
    h.seg_len = 300;
    h.flags = ULTRA_PXF_STORE | ULTRA_PXF_MIRROR | (ULTRA_PXW_BRIGHT << ULTRA_PXF_WHITE_SHIFT);
    h.custom1 = 5;
    h.custom2 = 6;
    uint8_t hs[ULTRA_SLOT_BYTES];
    ultra_slot_pixel_fx(hs, 2, 77, &h);
    CHECK(hex(hs, ULTRA_SLOT_BYTES) == "0b204d09fffffa240102fe0a0b0c0d0e0f101112c802bc000c012c1e050660d2",
          "the firmware's self-test vector: %s", hex(hs, ULTRA_SLOT_BYTES).c_str());
    const uint16_t zeros[ULTRA_CFG_PX_COUNT] = {};
    CHECK(ultra_px_config_crc(zeros) == 0xf810, "ultra_px_config_crc of twelve zeros (%04x; an independent CRC-16/CCITT-FALSE)",
          ultra_px_config_crc(zeros));
    // CONFIG 129-140 as the protocol's vectors set them (slot_config_pixel + slot_config_pixel_line_mode): the CRC the
    // answer_pixel_state vector carries
    const uint16_t chain_cfg[ULTRA_CFG_PX_COUNT] = {ULTRA_PXC_SK6812_RGBW, ULTRA_PX_ORDER(ULTRA_PXCH_G, ULTRA_PXCH_R, ULTRA_PXCH_B, ULTRA_PXCH_W, 0),
                                                    300, 60, 200, 0, ULTRA_PXW_ACCURATE, 0, 0,
                                                    ULTRA_PXP_POWER_ON | ULTRA_PXP_ON_LOSS, 30, ULTRA_PXLM_CHAIN};
    CHECK(ultra_px_config_crc(chain_cfg) == 0xba35, "the vectors' twelve values: CRC 0xBA35 (%04x)", ultra_px_config_crc(chain_cfg));
    CHECK(ultra_px_config_crc(chain_cfg) != ultra_px_config_crc(zeros), "(not all zeros)");
    const uint8_t lm_ids[3] = {ULTRA_CFG_PX_POLICY, ULTRA_CFG_PX_LOSS_TIMEOUT, ULTRA_CFG_PX_LINE_MODE};
    const uint16_t lm_vals[3] = {ULTRA_PXP_POWER_ON | ULTRA_PXP_ON_LOSS, 30, ULTRA_PXLM_CHAIN};
    uint8_t lm[ULTRA_SLOT_BYTES];
    ultra_slot_config_px(lm, 0x48, lm_ids, lm_vals, 3);
    CHECK(ultra_slot_check(lm) == 0 && lm[ULTRA_PAYLOAD] == 138 && lm[ULTRA_PAYLOAD + 6] == 140 &&
              ultra_get16(lm + ULTRA_PAYLOAD + 7) == ULTRA_PXLM_CHAIN,
          "CONFIG 138-140 in one slot");
    std::printf("VEC_SLOT_CONFIG_PIXEL_LINE_MODE = \"%s\"\n", hex(lm, ULTRA_SLOT_BYTES).c_str());
  }
  // the other side reads it back (the receiver's parser)
  ultra_pixel_fx_t g;
  CHECK(ultra_pixel_fx_parse(slot, &g) == 2 && g.effect == 7 && g.start_ms == -1234 && g.brightness == 128 &&
            g.seg_len == 90 && pixel_fx_same(f, g),
        "the receiver's parser reads it back");
  // off = brightness 0 with the transition
  PixelLightState off = sample();
  off.on = false;
  const ultra_pixel_fx_t fo = pixel_fx_of(off);
  CHECK(fo.brightness == 0 && fo.transition_ms == 1500 && fo.effect == 7 && fo.c[0][0] == 255, "off: brightness 0, the rest kept");
  // a stream request (effect 0)
  PixelLightState st = sample();
  st.effect = 0;
  CHECK(pixel_fx_of(st).effect == ULTRA_PX_FX_STREAM, "effect 0 = stream");
  CHECK(pixel_byte(-1.0f) == 0 && pixel_byte(2.0f) == 255 && pixel_byte(1.0f) == 255 && pixel_byte(0.0f) == 0, "clamped");
  // the transition is not part of "the receiver shows this already"
  PixelLightState t2 = sample();
  t2.transition_ms = 0;
  CHECK(pixel_fx_same(pixel_fx_of(sample()), pixel_fx_of(t2)), "same but for the transition");
  t2.speed = 101;
  CHECK(!pixel_fx_same(pixel_fx_of(sample()), pixel_fx_of(t2)), "a speed change is a change");
}

// -------------------------------------------------------------------------------------------------------- page 0x81
static void test_page() {
  PixelState ps;
  CHECK(ps.has_effect(5) && ps.has_palette(3), "nothing is marked missing before the maps are read");
  uint8_t d[24] = {};
  ultra_put16(d + ULTRA_PXG_CATALOGUE, 3);
  d[ULTRA_PXG_EFFECTS] = 20;
  d[ULTRA_PXG_PALETTES] = 10;
  d[ULTRA_PXG_CHIPS] = 8;
  d[ULTRA_PXG_SEGMENTS] = 4;
  ultra_put16(d + ULTRA_PXG_MAX_PIXELS, 1032);
  d[ULTRA_PXG_MODE] = ULTRA_PXM_LOCAL;
  d[ULTRA_PXG_RUNNING + 1] = 14;
  d[ULTRA_PXG_SEQ + 1] = 77;
  ultra_put16(d + ULTRA_PXG_FPS10, 305);
  d[ULTRA_PXG_LOAD] = 24;
  d[ULTRA_PXG_LIMIT] = 200;
  d[ULTRA_PXG_LATE] = 2;
  ultra_put16(d + ULTRA_PXG_CONFIG_CRC, 0xBEEF);
  decode_pixel_page(ps, ULTRA_PXG_PART_STATE, d);
  CHECK(ps.have_state && ps.catalogue == 3 && ps.effects == 20 && ps.palettes == 10 && ps.chips == 8 && ps.segments == 4 &&
            ps.max_pixels == 1032 && ps.mode == ULTRA_PXM_LOCAL && ps.running[1] == 14 && ps.seq[1] == 77 &&
            ps.fps10 == 305 && ps.load == 24 && ps.limit == 200 && ps.late == 2 && ps.config_crc == 0xBEEF,
        "part 0");
  CHECK(!ps.have_effects && ps.has_effect(100), "the maps are separate parts");
  uint8_t m[24] = {};
  for (int id : {1, 2, 3, 14, 19})
    m[id >> 3] |= static_cast<uint8_t>(1u << (id & 7));
  decode_pixel_page(ps, ULTRA_PXG_PART_EFFECTS, m);
  CHECK(ps.have_effects && ps.has_effect(14) && ps.has_effect(19) && !ps.has_effect(4) && !ps.has_effect(100) &&
            !ps.has_effect(200),
        "effect map");
  uint8_t pm[24] = {};
  pm[0] = 0x07;
  decode_pixel_page(ps, ULTRA_PXG_PART_PALETTES, pm);
  CHECK(ps.have_palettes && ps.has_palette(2) && !ps.has_palette(3) && ps.has_palette(ULTRA_PX_PAL_COLORS) &&
            ps.has_palette(ULTRA_PX_PAL_DEFAULT),
        "palette map: c1-c2-c3 and the effect's own are always there");
  std::printf("  page 0x81: the three parts, the maps\n");
}

// The protocol's test vector answers (test-vectors/vectors.json answer_pixel_state, answer_pixel_line): tests/run_ultra_digital_test.py
// compares these two strings with the repository's. An Ultra-Digital receiver, last board (index 2) of a normal Ultra line.
static const char *const ANSWER_PIXEL_STATE = "a5812069567800011a1408040366010700000021000000025818ff00ba351100";
static const char *const ANSWER_PIXEL_LINE = "a5812068567802021103a11200020000020001ff000100034000000000003d51";

static std::vector<uint8_t> unhex(const char *h) {
  std::vector<uint8_t> out;
  for (size_t i = 0; h[i] != 0 && h[i + 1] != 0; i += 2) {
    auto v = [](char c) { return c >= 'a' ? c - 'a' + 10 : c - '0'; };
    out.push_back(static_cast<uint8_t>(v(h[i]) * 16 + v(h[i + 1])));
  }
  return out;
}

static void test_line() {
  // the vectors, decoded as the poller decodes an answer (DATA from byte 6)
  const std::vector<uint8_t> st = unhex(ANSWER_PIXEL_STATE), ln = unhex(ANSWER_PIXEL_LINE);
  CHECK(st.size() == 32 && ln.size() == 32 && ultra_answer_check(st.data()) == 0 && ultra_answer_check(ln.data()) == 0,
        "the two answers are valid (%zu, %zu bytes)", st.size(), ln.size());
  PixelState ps;
  decode_pixel_page(ps, ULTRA_PXG_PART_STATE, st.data() + ULTRA_RSP_DATA);
  uint16_t cfg[ULTRA_CFG_PX_COUNT] = {ULTRA_PXC_SK6812_RGBW, ULTRA_PX_ORDER(ULTRA_PXCH_G, ULTRA_PXCH_R, ULTRA_PXCH_B, ULTRA_PXCH_W, 0),
                                      300, 60, 200, 0, ULTRA_PXW_ACCURATE, 0, 0, 3, 30, ULTRA_PXLM_CHAIN};
  CHECK(ps.have_state && ps.mode == ULTRA_PXM_LOCAL && ps.running[0] == 7 && ps.fps10 == 600 && ps.config_crc == ultra_px_config_crc(cfg),
        "answer_pixel_state: the state, and the CONFIG CRC is the one over our twelve values (%04x)", ps.config_crc);
  decode_pixel_page(ps, ULTRA_PXG_PART_LINE, ln.data() + ULTRA_RSP_DATA);
  const PixelLineState &l = ps.line;
  CHECK(l.have && l.config == ULTRA_PXLM_CHAIN && l.mode == ULTRA_PXLM_CHAIN && l.flags == (ULTRA_PXL_F_LOCKED | ULTRA_PXL_F_LIT) &&
            l.slots == 3 && l.frames == 41234 && l.non_ultra == 2 && l.long_frames == 0 && l.windows == 512 && l.answers == 511 &&
            l.missed == 1 && l.bits == 832 && l.markers == 0,
        "answer_pixel_line decoded field by field");
  CHECK(pixel_line_issues(l, ULTRA_PXLM_CHAIN).empty(), "... a locked last board with one missed window: nothing wrong ('%s')",
        pixel_line_issues(l, ULTRA_PXLM_CHAIN).c_str());
  CHECK(pixel_line_status(true, ULTRA_HWCAP_PIXEL_FX | ULTRA_HWCAP_CHAIN, ps, ULTRA_PXLM_CHAIN) == "Chain, locked", "its status: %s",
        pixel_line_status(true, ULTRA_HWCAP_PIXEL_FX | ULTRA_HWCAP_CHAIN, ps, ULTRA_PXLM_CHAIN).c_str());
  // every field at its own offset
  uint8_t d[24];
  for (int i = 0; i < 24; i++)
    d[i] = static_cast<uint8_t>(0x10 + i);
  PixelState q;
  decode_pixel_page(q, ULTRA_PXG_PART_LINE, d);
  CHECK(q.line.config == 0x10 && q.line.mode == 0x11 && q.line.flags == 0x12 && q.line.slots == 0x13 && q.line.frames == 0x1415 &&
            q.line.non_ultra == 0x1617 && q.line.long_frames == 0x1819 && q.line.windows == 0x1A1B && q.line.answers == 0x1C1D &&
            q.line.missed == 0x1E1F && q.line.bits == 0x202122u && q.line.markers == 0x2324,
        "the offsets of ULTRA_PXL_*");
  CHECK(!q.have_state && !q.have_effects, "(part 3 touches neither the state nor the maps)");

  // what is wrong with a line
  PixelLineState w = l;
  CHECK(pixel_line_issues(PixelLineState{}, ULTRA_PXLM_CHAIN).empty(), "nothing read: nothing wrong");
  w.mode = ULTRA_PXLM_STREAM;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN).find("Wrong mode: the receiver is in stream mode, this line needs chain mode") == 0,
        "a stream receiver on a chain line: %s", pixel_line_issues(w, ULTRA_PXLM_CHAIN).c_str());
  w.mode = ULTRA_PXLM_CHAIN;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_STREAM).find("Wrong mode: the receiver is in chain mode, this line needs stream mode") == 0,
        "a chain receiver on a digital port: %s", pixel_line_issues(w, ULTRA_PXLM_STREAM).c_str());
  w.mode = 0;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN).empty() && pixel_line_status(true, ULTRA_HWCAP_CHAIN, [&] {
          PixelState t;
          t.line = w;
          return t;
        }(), ULTRA_PXLM_CHAIN) == "Deciding",
        "undecided (AUTO deciding) is not a fault");
  w = l;
  w.flags |= ULTRA_PXL_F_LONG;
  w.slots = 4;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN) == "Not the last board of its line (it saw frames of more than one slot)",
        "frames of more than one slot: not the last board (%s)", pixel_line_issues(w, ULTRA_PXLM_CHAIN).c_str());
  w = l;
  w.flags |= ULTRA_PXL_F_NONULTRA;
  w.non_ultra = 17;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN) == "17 frames that are no Ultra frames", "no-Ultra frames (%s)",
        pixel_line_issues(w, ULTRA_PXLM_CHAIN).c_str());
  w = l;
  w.flags |= ULTRA_PXL_F_MARKERS;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN).find("Marker frames") == 0, "marker frames on a chain line");
  w = l;
  w.windows = 1000;
  w.missed = 19;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN).empty(), "19 of 1000 windows: fine (a flash write now and then)");
  w.missed = 21;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN) == "Missed 21 of 1000 windows", "21 of 1000 (2.1 %%): a fault (%s)",
        pixel_line_issues(w, ULTRA_PXLM_CHAIN).c_str());
  w.windows = 20;
  w.missed = 2;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_CHAIN).empty(), "2 windows are never a fault");
  w = l;
  w.flags = ULTRA_PXL_F_LONG | ULTRA_PXL_F_NONULTRA | ULTRA_PXL_F_MARKERS;
  w.mode = ULTRA_PXLM_STREAM;
  CHECK(pixel_line_issues(w, ULTRA_PXLM_STREAM).empty(), "a stream receiver on a digital port: the chain's checks don't apply");
  // the status text
  PixelState t;
  CHECK(pixel_line_status(false, 0, t, ULTRA_PXLM_CHAIN) == "Waiting for the receiver", "no IDENTITY yet");
  CHECK(pixel_line_status(true, ULTRA_HWCAP_PIXEL_FX, t, ULTRA_PXLM_CHAIN) == "No chain mode (firmware too old)", "no HWCAP_CHAIN");
  CHECK(pixel_line_status(true, ULTRA_HWCAP_PIXEL_FX | ULTRA_HWCAP_CHAIN, t, ULTRA_PXLM_CHAIN) == "Waiting for the receiver",
        "the line not read yet");
  t.line = l;
  t.line.flags = 0;
  CHECK(pixel_line_status(true, ULTRA_HWCAP_CHAIN, t, ULTRA_PXLM_CHAIN) == "Chain, not locked", "chain, no Ultra frame for 2 s");
  t.line.mode = ULTRA_PXLM_STREAM;
  CHECK(pixel_line_status(true, ULTRA_HWCAP_CHAIN, t, ULTRA_PXLM_CHAIN) ==
            "Stream - Wrong mode: the receiver is in stream mode, this line needs chain mode",
        "stream on a chain line: %s", pixel_line_status(true, ULTRA_HWCAP_CHAIN, t, ULTRA_PXLM_CHAIN).c_str());
  CHECK(std::string(pixel_mode_name(ULTRA_PXLM_STREAM)) == "stream" && std::string(pixel_mode_name(ULTRA_PXLM_CHAIN)) == "chain",
        "mode names");
  std::printf("VEC_ANSWER_PIXEL_STATE = \"%s\"\n", ANSWER_PIXEL_STATE);
  std::printf("VEC_ANSWER_PIXEL_LINE = \"%s\"\n", ANSWER_PIXEL_LINE);
  std::printf("  page 0x81 part 3: the vectors' answers decoded, the offsets, what is wrong with a line, its status text\n");
}

// -------------------------------------------------------------------------------------------------------- the tracker
static void test_tracker() {
  PixelFxTracker t;
  PixelFxTracker::Out o;
  PixelLightState s = sample();
  s.transition_ms = 0;
  const int64_t T0 = 1000000;
  CHECK(t.update(s, true, 800, T0, 100, &o) && o.anchor_us == T0 && o.fx.effect == 7 && o.fx.transition_ms == 0,
        "the first send takes an anchor, no crossfade");
  CHECK(t.local() && t.sent() && t.lit(100), "lit while on");
  CHECK(!t.update(s, false, 800, T0 + 5000, 105, &o), "the same again: the receiver is there already");
  s.speed = 90;
  CHECK(t.update(s, false, 800, T0 + 9000, 110, &o) && o.anchor_us == T0 && o.fx.speed == 90,
        "a change of speed keeps the phase anchor");
  s.red = 0.2f;
  s.transition_ms = 250;
  CHECK(t.update(s, false, 800, T0 + 20000, 120, &o) && o.anchor_us == T0 && o.fx.transition_ms == 250 &&
            o.fx.c[0][0] == 51,
        "a colour change keeps it too, with the transition");
  s.effect = 9;
  s.transition_ms = 0;
  CHECK(t.update(s, true, 800, T0 + 30000, 130, &o) && o.anchor_us == T0 + 30000 && o.fx.effect == 9 &&
            o.fx.transition_ms == 800,
        "a new effect: a new anchor, the crossfade over the default transition");
  s.transition_ms = 2000;
  s.effect = 10;
  CHECK(t.update(s, true, 800, T0 + 40000, 140, &o) && o.fx.transition_ms == 2000, "an asked-for transition wins");
  // off: brightness 0 with the transition; lit until it has faded
  s.on = false;
  s.transition_ms = 1000;
  CHECK(t.update(s, false, 800, T0 + 50000, 1050, &o) && o.fx.brightness == 0 && o.anchor_us == T0 + 40000,
        "off: brightness 0 with the transition, the effect carries on");
  CHECK(t.lit(1050) && t.lit(2000) && !t.lit(2100), "still lit while it fades out (until start + transition)");
  s.on = true;
  CHECK(t.update(s, false, 800, T0 + 60000, 1200, &o) && o.fx.brightness == 128 && t.lit(1200), "on again");
  // the STORE copy: this long after the last change, once, with the same anchor and no transition
  PixelFxTracker::Out so;
  CHECK(!t.store(1200 + 29000, 30000, &so), "not yet");
  CHECK(t.store(1200 + 30000, 30000, &so) && (so.fx.flags & ULTRA_PXF_STORE) && so.fx.transition_ms == 0 &&
            so.anchor_us == t.anchor_us() && so.fx.effect == 10,
        "STORE after the delay");
  CHECK(!t.store(1200 + 99000, 30000, &so), "only once");
  CHECK(!t.store(1200 + 99000, 0, &so), "never without a delay");
  t.touched(5000);
  CHECK(t.store(5000 + 30000, 30000, &so), "a touch (a light change) starts the timer again");
  // the same slot with only a new START is what the receiver takes as a seamless renewal: the 20-day renewal sends exactly
  // that; a user's choice of the effect (again) carries RESTART, in that one PIXEL_FX only
  {
    PixelFxTracker r;
    PixelFxTracker::Out a, b, c, d;
    PixelLightState q = sample();
    q.transition_ms = 0;
    CHECK(r.update(q, true, 800, T0, 100, &a) && !(a.fx.flags & ULTRA_PXF_RESTART), "restart: the first send, no RESTART");
    CHECK(r.update(q, true, 800, T0 + 1000000, 200, &b) && b.anchor_us == T0 + 1000000 && pixel_fx_same(a.fx, b.fx) &&
              !(b.fx.flags & ULTRA_PXF_RESTART),
          "an automatic renewal: the same slot with a new START, no RESTART");
    CHECK(r.update(q, true, 800, T0 + 2000000, 300, &c, true) && c.anchor_us == T0 + 2000000 &&
              (c.fx.flags & ULTRA_PXF_RESTART) && c.fx.effect == b.fx.effect && c.fx.speed == b.fx.speed,
          "the user chose the effect again: a new START and RESTART");
    q.speed = 77;
    PixelFxTracker::Out e;
    CHECK(r.update(q, false, 800, T0 + 3500000, 450, &e) && e.anchor_us == T0 + 2000000 && !(e.fx.flags & ULTRA_PXF_RESTART),
          "a parameter change after it keeps the START and does not restart again");
    PixelFxTracker::Out st;
    CHECK(r.store(450 + 30000, 30000, &st) && !(st.fx.flags & ULTRA_PXF_RESTART) && (st.fx.flags & ULTRA_PXF_STORE),
          "the STORE copy never carries it");
    (void) d;
  }
  // 20 days: a new anchor
  CHECK(!t.reanchor_due(T0 + 19LL * 86400 * 1000000) && t.reanchor_due(T0 + 61000 + PixelFxTracker::REANCHOR_US),
        "the anchor is renewed after 20 days");
  // the hand-back to the stream
  o = t.release(T0 + 70000);
  CHECK(o.fx.effect == ULTRA_PX_FX_STREAM && t.released() && !t.local() && !t.lit(2000), "release: effect 0, nothing of ours shown");
  CHECK(!t.reanchor_due(T0 + 99LL * 86400 * 1000000) && !t.store(1e9, 1, &so), "nothing renews or stores a released strip");
  s.on = false;
  CHECK(!t.update(s, false, 800, T0 + 80000, 2100, &o), "released and off: nothing to say");
  s.on = true;
  s.transition_ms = 0;
  CHECK(t.update(s, false, 800, T0 + 90000, 2200, &o) && o.anchor_us == T0 + 90000 && !t.released() && o.fx.transition_ms == 0,
        "switched on again: a new anchor, no crossfade (the first send)");
  std::printf("  tracker: anchors, transitions, off, lit, STORE, 20 days, release\n");
}

// ------------------------------------------------------------------------------------------------------- the scheduler
static const uint8_t *slot_of(const std::vector<uint8_t> &f, int i) { return f.data() + ULTRA_SLOT_BYTES * i; }
static const uint8_t *hdr_of(const std::vector<uint8_t> &f) { return f.data() + f.size() - ULTRA_HDR_BYTES; }
static bool frame_ok(const std::vector<uint8_t> &f, uint16_t boards) {
  uint32_t t;
  if (f.size() != ultra_frame_bytes(boards) || ultra_parse_header(hdr_of(f), &t) != 0)
    return false;
  for (uint16_t i = 0; i < boards; i++) {
    if (ultra_slot_check(slot_of(f, i)) != 0)
      return false;
  }
  return true;
}

static void test_scheduler() {
  Scheduler s;
  s.init(3, 777, 3);
  std::vector<uint8_t> f(s.frame_bytes());
  const int64_t ANCHOR = 5000000;   // 5 s (host us)
  PixelLightState st = sample();
  st.transition_ms = 0;
  s.light_pixel_fx(1, 0, pixel_fx_of(st), ANCHOR);
  CHECK(s.pending() && s.queued(1) == 1 && s.pixel_fx_valid(1, 0) && !s.pixel_fx_valid(0, 0), "queued for board 1 only");
  std::vector<uint8_t> seqs;
  for (int i = 0; i < 4; i++) {
    const int64_t start = 4000000 + i * 3000;
    const int64_t hdr = s.build_frame(start, f.data());
    CHECK(frame_ok(f, 3), "frame CRCs");
    const uint8_t *slot = slot_of(f, 1);
    if (i < 3) {
      CHECK(slot[0] == ULTRA_PIXEL_FX && (slot[1] >> 4) == 0, "board 1 gets PIXEL_FX for segment 0 (copy %d)", i);
      seqs.push_back(slot[2]);
      const int32_t off = static_cast<int32_t>(ultra_get32(slot + ULTRA_PX_START));
      CHECK(std::llabs(hdr + static_cast<int64_t>(off) * 1000 - ANCHOR) <= 500, "START %d ms at TIME %lld: the anchor", off,
            static_cast<long long>(hdr));
      CHECK(slot[ULTRA_PX_EFFECT] == 7 && slot[ULTRA_PX_SPEED] == 100, "the parameters");
    }
    CHECK(slot_of(f, 0)[0] == ULTRA_NOP && slot_of(f, 2)[0] == ULTRA_NOP, "the other boards have nothing to do");
  }
  CHECK(seqs.size() == 3 && seqs[0] == seqs[1] && seqs[1] == seqs[2], "three copies of one SEQ");
  CHECK(!s.pending(), "queue empty after 3 copies");
  const uint8_t seq1 = seqs[0];

  // a change of a parameter: the same anchor, a new SEQ, replacing a queued one
  st.speed = 50;
  s.light_pixel_fx(1, 0, pixel_fx_of(st), ANCHOR);
  st.speed = 60;
  s.light_pixel_fx(1, 0, pixel_fx_of(st), ANCHOR);
  CHECK(s.queued(1) == 1, "a newer PIXEL_FX replaces the queued one");
  const int64_t hdr = s.build_frame(6000000, f.data());
  const uint8_t *slot = slot_of(f, 1);
  CHECK(slot[0] == ULTRA_PIXEL_FX && slot[2] == static_cast<uint8_t>(seq1 + 2) && slot[ULTRA_PX_SPEED] == 60, "the latest, SEQ +2");
  CHECK(std::llabs(hdr + static_cast<int64_t>(static_cast<int32_t>(ultra_get32(slot + ULTRA_PX_START))) * 1000 - ANCHOR) <= 500,
        "the anchor kept");
  // a segment of its own is a separate command
  PixelLightState seg1 = sample();
  seg1.effect = 3;
  s.light_pixel_fx(1, 1, pixel_fx_of(seg1), ANCHOR + 1000000);
  CHECK(s.queued(1) == 2, "segment 1 is another command");
  // segments 4 and up are not a thing (the receiver ignores LIGHT 4-14)
  s.light_pixel_fx(1, 4, pixel_fx_of(seg1), ANCHOR);
  CHECK(s.queued(1) == 2 && !s.pixel_fx_valid(1, 4), "segment 4 refused");
  for (int i = 0; i < 8; i++)
    s.build_frame(6100000 + 3000 * i, f.data());
  CHECK(!s.pending(), "sent");

  // the refresh: at rest the PIXEL_FX of both segments come round again with the same SEQ and anchor
  std::set<int> seen;
  uint8_t fx_seq[2] = {0, 0};
  for (int i = 0; i < 200; i++) {
    const int64_t h = s.build_frame(7000000 + 3000 * i, f.data());
    const uint8_t *sl = slot_of(f, 1);
    if (sl[0] == ULTRA_PIXEL_FX) {
      const int seg = sl[1] >> 4;
      seen.insert(seg);
      CHECK(std::llabs(h + static_cast<int64_t>(static_cast<int32_t>(ultra_get32(sl + ULTRA_PX_START))) * 1000 -
                       (seg == 0 ? ANCHOR : ANCHOR + 1000000)) <= 500,
            "the refresh carries the anchor");
      if (fx_seq[seg] != 0)
        CHECK(fx_seq[seg] == sl[2], "the refresh repeats the SEQ");
      fx_seq[seg] = sl[2];
    }
    for (int b : {0, 2})
      CHECK(slot_of(f, b)[0] == ULTRA_NOP, "other boards: nothing");
  }
  CHECK(seen == (std::set<int>{0, 1}), "the refresh covers both segments");

  // a replay (the port woke): both queued again, with the anchors as they were
  s.replay(9000000);
  CHECK(s.queued(1) == 2, "replay queues both");
  s.build_frame(9000000, f.data());
  CHECK(slot_of(f, 1)[0] == ULTRA_PIXEL_FX, "replayed");
  std::printf("  scheduler: PIXEL_FX copies, START stamp, replacement, refresh, replay\n");

  // the stream: PIXEL_FX effect 0 for the segment
  Scheduler u;
  u.init(1, 3, 3);
  std::vector<uint8_t> g(u.frame_bytes());
  ultra_pixel_fx_t rel;
  std::memset(&rel, 0, sizeof rel);
  u.light_pixel_fx(0, 0, rel, 1000000);
  u.build_frame(900000, g.data());
  CHECK(slot_of(g, 0)[0] == ULTRA_PIXEL_FX && slot_of(g, 0)[ULTRA_PX_EFFECT] == 0, "effect 0 goes out like any other");

  // RESTART is an action: the command's copies carry it, the refresh and a replay do not
  Scheduler r;
  r.init(1, 9, 3);
  std::vector<uint8_t> h(r.frame_bytes());
  PixelLightState rs = sample();
  rs.transition_ms = 0;
  ultra_pixel_fx_t rfx = pixel_fx_of(rs);
  rfx.flags = static_cast<uint8_t>(rfx.flags | ULTRA_PXF_RESTART | ULTRA_PXF_LOCK);
  r.light_pixel_fx(0, 0, rfx, 4000000);
  int with = 0, without = 0;
  bool replayed = false;
  for (int i = 0; i < 300; i++) {
    r.build_frame(3000000 + 3000 * i, h.data());
    const uint8_t *sl = slot_of(h, 0);
    if (sl[0] != ULTRA_PIXEL_FX)
      continue;
    CHECK(ultra_slot_check(sl) == 0 && (sl[ULTRA_PX_FLAGS] & ULTRA_PXF_LOCK), "sealed, the other flags stay");
    (sl[ULTRA_PX_FLAGS] & ULTRA_PXF_RESTART ? with : without)++;
    if (with == 3 && without >= 2 && !replayed) {
      replayed = true;
      r.replay(5000000);
    }
  }
  CHECK(with == 3 && without >= 4, "3 copies carry RESTART, the refresh and the replay do not (%d / %d)", with, without);
  // the golden (the protocol repository's test vector slot_pixel_fx_restart is the same bytes)
  {
    ultra_pixel_fx_t v;
    std::memset(&v, 0, sizeof v);
    v.effect = 20;
    v.speed = 128;
    v.intensity = 128;
    v.palette = 0xFFu;
    v.c[0][0] = v.c[0][1] = v.c[0][2] = 255;
    v.brightness = 255;
    v.flags = ULTRA_PXF_RESTART | ULTRA_PXF_LOCK;
    uint8_t vs[ULTRA_SLOT_BYTES];
    ultra_slot_pixel_fx(vs, 1, 0x0B, &v);
    CHECK(vs[ULTRA_PX_FLAGS] == 0x60, "FLAGS RESTART | LOCK = 0x60");
    std::printf("VEC_SLOT_PIXEL_FX_RESTART = \"%s\"\n", hex(vs, ULTRA_SLOT_BYTES).c_str());
  }
}

static void test_config() {
  Scheduler s;
  s.init(2, 5, 3);
  std::vector<uint8_t> f(s.frame_bytes());
  // an analog board's CONFIG is untouched; the hardware-specific ids 129-140 sit in their own slots
  s.set_config(0, ULTRA_CFG_PWM_PRESET, 8);
  for (uint8_t k = 0; k < ULTRA_CFG_PX_COUNT; k++)
    s.set_hw_config(1, ULTRA_CFG_PX_FIRST + k, static_cast<uint16_t>(1000 + k));
  s.set_hw_config(1, 128, 5);                          // not a pixel id
  s.set_hw_config(1, ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT, 5);   // beyond
  s.set_hw_config(5, ULTRA_CFG_PX_FIRST, 5);           // no such board
  CHECK(s.has_hw_config(1, 131) && s.get_hw_config(1, 131) == 1002 && !s.has_hw_config(0, 131) && !s.has_hw_config(1, 128),
        "stored per board");
  s.flush_config();
  CHECK(s.queued(0) == 1 && s.queued(1) == 2, "board 0: one CONFIG; board 1: two slots for 12 values (%u)",
        static_cast<unsigned>(s.queued(1)));
  std::set<uint8_t> ids;
  uint8_t seqs[2] = {0, 0};
  for (int i = 0; i < 3; i++) {
    s.build_frame(1000000 + 3000 * i, f.data());
    const uint8_t *sl = slot_of(f, 1);
    CHECK(sl[0] == ULTRA_CONFIG && ultra_slot_check(sl) == 0, "CONFIG slot");
    for (int j = 0; j < 9; j++) {
      const uint8_t id = sl[ULTRA_PAYLOAD + 3 * j];
      if (id == 0)
        continue;
      ids.insert(id);
      CHECK(ultra_get16(sl + ULTRA_PAYLOAD + 3 * j + 1) == 1000 + id - ULTRA_CFG_PX_FIRST, "value of %u", id);
    }
    seqs[i % 2] = sl[2];
  }
  CHECK(ids.size() == ULTRA_CFG_PX_COUNT && *ids.begin() == 129 && *ids.rbegin() == 140, "all of 129-140 on the wire (%u)",
        static_cast<unsigned>(ids.size()));
  CHECK(ids.count(140) == 1 && ULTRA_CFG_PX_COUNT == 12u, "CONFIG 140 (the line mode) is one of them, twelve in all");
  // the same values again: nothing queued; one changed: its slot only
  for (int i = 0; i < 12; i++)
    s.build_frame(2000000 + 3000 * i, f.data());
  CHECK(!s.pending(), "sent");
  for (uint8_t k = 0; k < ULTRA_CFG_PX_COUNT; k++)
    s.set_hw_config(1, ULTRA_CFG_PX_FIRST + k, static_cast<uint16_t>(1000 + k));
  s.flush_config();
  CHECK(!s.pending(), "identical values are not sent again");
  s.set_hw_config(1, 139, 7);
  s.flush_config();
  CHECK(s.queued(1) >= 1 && s.queued(1) <= 2, "one changed id: its page (the pages are rebuilt together, as the official ones) (%u)",
        static_cast<unsigned>(s.queued(1)));
  // CONFIG 140 alone: the second page changes, and its value is on the wire
  for (int i = 0; i < 12; i++)
    s.build_frame(2500000 + 3000 * i, f.data());
  s.set_hw_config(1, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_CHAIN);
  s.flush_config();
  CHECK(s.pending(), "CONFIG 140 changed: queued");
  bool saw140 = false;
  for (int i = 0; i < 6; i++) {
    s.build_frame(2800000 + 3000 * i, f.data());
    const uint8_t *sl = slot_of(f, 1);
    for (int j = 0; j < 9 && sl[0] == ULTRA_CONFIG; j++) {
      if (sl[ULTRA_PAYLOAD + 3 * j] == ULTRA_CFG_PX_LINE_MODE)
        saw140 |= ultra_get16(sl + ULTRA_PAYLOAD + 3 * j + 1) == ULTRA_PXLM_CHAIN;
    }
  }
  CHECK(saw140, "... CONFIG 140 = CHAIN is on the wire");
  // the refresh sends them too, a board without them never gets one
  int cfg_hw = 0, cfg_other_board = 0;
  for (int i = 0; i < 400; i++) {
    s.build_frame(3000000 + 3000 * i, f.data());
    const uint8_t *sl = slot_of(f, 1);
    if (sl[0] == ULTRA_CONFIG && sl[ULTRA_PAYLOAD] >= ULTRA_CFG_PX_FIRST)
      cfg_hw++;
    const uint8_t *a = slot_of(f, 0);
    if (a[0] == ULTRA_CONFIG && a[ULTRA_PAYLOAD] >= ULTRA_CFG_PX_FIRST)
      cfg_other_board++;
  }
  CHECK(cfg_hw >= 2 && cfg_other_board == 0, "the refresh repeats the pixel CONFIG (%d); board 0 never has it (%d)", cfg_hw,
        cfg_other_board);
  s.replay(0);
  CHECK(s.queued(1) == 2 && s.queued(0) == 1, "replay: both pages of board 1, the one of board 0");
  // the builders: the official one leaves the hardware-specific ids out, the pixel one takes only 129-139
  uint8_t slot[ULTRA_SLOT_BYTES];
  const uint8_t ids2[5] = {26, 129, 139, 200, 140};
  const uint16_t vals[5] = {1, 2, 3, 4, 2};
  ultra_slot_config(slot, 1, ids2, vals, 5);
  CHECK(slot[ULTRA_PAYLOAD] == 0 && slot[ULTRA_PAYLOAD + 3] == 0, "ultra_slot_config: official ids only (26, 129, 139, 200, 140 all out)");
  ultra_slot_config_px(slot, 1, ids2, vals, 5);
  CHECK(slot[ULTRA_PAYLOAD] == 129 && slot[ULTRA_PAYLOAD + 3] == 139 && slot[ULTRA_PAYLOAD + 6] == 140 && slot[ULTRA_PAYLOAD + 9] == 0,
        "ultra_slot_config_px: 129, 139 and 140 in, 26 and 200 out");
  std::printf("  CONFIG 129-140: their own slots, copies, refresh, replay; other boards unchanged\n");
}

// A normal Ultra line whose last board is the receiver (chain mode): boards 0 and 1 analog (a light each), board 2 the
// receiver. Its PIXEL_FX and CONFIG are slots of the line like any board's, in its own position and nowhere else; the boards in
// front send what they send without it; nothing marker-like or pixel-like is on the line.
static void test_chain_line() {
  const int64_t ANCHOR = 7000000;
  std::vector<uint8_t> trace[2][60];
  for (int run = 0; run < 2; run++) {
    Scheduler s;
    s.init(3, 4242, 3);
    for (uint16_t b = 0; b < 2; b++) {   // the analog boards: a SET on light 0, and a CONFIG
      s.clear_unused_lights(b, 5);
      s.set_config(b, ULTRA_CFG_PWM_PRESET, 8);
    }
    s.flush_config();
    if (run == 1) {
      PixelLightState st = sample();
      st.transition_ms = 0;
      s.set_hw_config(2, ULTRA_CFG_PX_PIXELS, 300);
      s.set_hw_config(2, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_CHAIN);
      s.flush_config();
      s.light_pixel_fx(2, 0, pixel_fx_of(st), ANCHOR);
    }
    std::vector<uint8_t> f(s.frame_bytes());
    std::set<uint8_t> types2;
    int fx = 0, cfg140 = 0;
    for (int i = 0; i < 60; i++) {
      s.build_frame(4000000 + 3000 * i, f.data());
      CHECK(frame_ok(f, 3), "frame %d: 3 slots + header, all CRCs", i);
      CHECK(f.size() == ultra_frame_bytes(3), "frame %d: the normal length, no marker, no pixel bytes (%zu)", i, f.size());
      trace[run][i].assign(f.begin(), f.begin() + 2 * ULTRA_SLOT_BYTES);
      const uint8_t *sl = slot_of(f, 2);
      if (run == 1) {
        types2.insert(sl[0]);
        if (sl[0] == ULTRA_PIXEL_FX)
          fx++;
        for (int j = 0; j < 9 && sl[0] == ULTRA_CONFIG; j++)
          cfg140 += sl[ULTRA_PAYLOAD + 3 * j] == ULTRA_CFG_PX_LINE_MODE && ultra_get16(sl + ULTRA_PAYLOAD + 3 * j + 1) == ULTRA_PXLM_CHAIN;
      } else {
        CHECK(sl[0] == ULTRA_NOP, "without the receiver: board 2 is idle");
      }
    }
    if (run == 1) {
      CHECK(fx >= 3 && cfg140 >= 3, "the receiver's slot: PIXEL_FX (%d) and CONFIG 140 = CHAIN (%d) came by, copies and refresh", fx, cfg140);
      CHECK(types2.count(ULTRA_PIXEL_FX) && types2.count(ULTRA_CONFIG) && !types2.count(ULTRA_SET) && !types2.count(ULTRA_LIGHT_DEF) &&
                !types2.count(ULTRA_EFFECT),
            "... and no analog slot type in it");
    }
  }
  for (int i = 0; i < 60; i++)
    CHECK(trace[0][i] == trace[1][i], "frame %d: the slots of boards 0 and 1 are the same with and without the receiver", i);
  std::printf("  a chain line: the receiver's slot only, the boards in front unchanged, normal frames\n");
}

// ------------------------------------------------------------------------------------ page 0x81 from a modelled receiver
struct UdModel {
  uint8_t uid[12]{9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 11, 12};
  uint32_t caps{ULTRA_HWCAP_RETURN | ULTRA_HWCAP_PIXEL | ULTRA_HWCAP_PIXEL_FX};
  bool unsupported_fx{false};
  bool unsupported_line{false};
  uint8_t line_mode{ULTRA_PXLM_CHAIN}, line_flags{ULTRA_PXL_F_LOCKED};
  uint32_t asked[4]{};      // page 0x81 parts
  uint32_t other_asked{0};
  uint32_t uptime{50};
  uint8_t mode{ULTRA_PXM_LOCAL};
  uint8_t effect_ids[4]{3, 14, 19};
  uint16_t uid16() const { return ultra_crc16(this->uid, 12); }
  void answer(const uint8_t *poll, uint8_t *a) {
    const uint8_t page = poll[ULTRA_POLL_PAGE];
    const uint8_t *arg = poll + ULTRA_POLL_ARG;
    uint8_t d[24]{};
    uint8_t out = page;
    if (page == ULTRA_PAGE_HW_FX) {
      if (this->unsupported_fx || arg[0] > 3 || (arg[0] == 3 && (this->unsupported_line || !(this->caps & ULTRA_HWCAP_CHAIN)))) {
        out = ULTRA_PAGE_UNSUPPORTED;
      } else {
        this->asked[arg[0]]++;
        if (arg[0] == ULTRA_PXG_PART_STATE) {
          ultra_put16(d + ULTRA_PXG_CATALOGUE, 1);
          d[ULTRA_PXG_EFFECTS] = 3;
          d[ULTRA_PXG_SEGMENTS] = 4;
          d[ULTRA_PXG_MODE] = this->mode;
          d[ULTRA_PXG_RUNNING] = 14;
          ultra_put16(d + ULTRA_PXG_FPS10, 300);
        } else if (arg[0] == ULTRA_PXG_PART_EFFECTS) {
          for (uint8_t id : this->effect_ids) {
            if (id != 0)
              d[id >> 3] |= static_cast<uint8_t>(1u << (id & 7));
          }
        } else if (arg[0] == ULTRA_PXG_PART_LINE) {
          d[ULTRA_PXL_CONFIG] = this->line_mode;
          d[ULTRA_PXL_MODE] = this->line_mode;
          d[ULTRA_PXL_FLAGS] = this->line_flags;
          d[ULTRA_PXL_SLOTS] = 1;
          ultra_put16(d + ULTRA_PXL_FRAMES, static_cast<uint16_t>(this->asked[3] * 10));
        } else {
          d[0] = 0x03;
        }
      }
    } else {
      switch (page) {
        case ULTRA_PAGE_IDENTITY:
          if (arg[0] == ULTRA_ID_PART_FEATURES) {
            ultra_put32(d + ULTRA_IDF_FEATURES, 0);
          } else {
            std::memcpy(d + ULTRA_ID_UID, this->uid, 12);
            d[ULTRA_ID_HW_ID] = 3;
            ultra_put32(d + ULTRA_ID_CAPS, this->caps);
          }
          break;
        case ULTRA_PAGE_STATUS:
          d[ULTRA_ST_HW_ID] = 3;
          ultra_put24(d + ULTRA_ST_UPTIME, this->uptime);
          ultra_put16(d + ULTRA_ST_MCU_TEMP, 400);
          ultra_put16(d + ULTRA_ST_TEMP_BOARD, ULTRA_ST_NA);
          ultra_put16(d + ULTRA_ST_VBUS, 2400);
          break;
        default:
          this->other_asked++;
          out = ULTRA_PAGE_UNSUPPORTED;
          break;
      }
    }
    a[0] = ULTRA_RSP_SYNC;
    a[1] = out;
    a[2] = static_cast<uint8_t>(poll[ULTRA_POLL_INDEX] << 4);
    a[3] = poll[2];
    ultra_put16(a + 4, this->uid16());
    std::memcpy(a + ULTRA_RSP_DATA, d, 24);
    ultra_put16(a + 30, ultra_crc16(a, 30));
  }
};

/// run the poller against one receiver for ms
static void run(TelemetryPoller &p, UdModel &m, uint32_t &now, uint32_t ms) {
  const uint32_t end = now + ms;
  uint32_t last_s = now / 1000;
  while (static_cast<int32_t>(now - end) < 0) {
    if (p.due(now)) {
      std::vector<uint8_t> slots(ULTRA_SLOT_BYTES * 2, 0);
      uint32_t w = 0;
      const uint16_t n = p.build(now, slots.data(), &w);
      std::vector<uint8_t> uart;
      for (uint16_t k = 0; k < n; k++) {
        const uint8_t *sl = slots.data() + ULTRA_SLOT_BYTES * k;
        if (sl[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY)
          continue;
        uint8_t a[32];
        m.answer(sl, a);
        uart.push_back(ULTRA_RSP_LEAD);
        uart.insert(uart.end(), a, a + 32);
      }
      now += 1 + w / 1000;
      p.finish(now, uart.data(), uart.size());
    } else {
      now += 5;
    }
    for (; last_s < now / 1000; last_s++)
      m.uptime++;
  }
}

static void test_poller() {
  UdModel m;
  uint32_t now = 100000;
  TelemetryPoller p;
  p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  p.locked(now);
  run(p, m, now, 20000);
  CHECK(p.data(0).have_identity && (p.data(0).hw_caps & ULTRA_HWCAP_PIXEL_FX), "identity: the capability is known");
  CHECK(m.asked[0] >= 3 && m.asked[1] == 1 && m.asked[2] == 1, "state read repeatedly (%u), the maps once (%u, %u)", m.asked[0],
        m.asked[1], m.asked[2]);
  const PixelState &ps = p.pixel(0);
  CHECK(ps.have_state && ps.mode == ULTRA_PXM_LOCAL && ps.running[0] == 14 && ps.fps10 == 300 && ps.catalogue == 1 &&
            ps.have_effects && ps.has_effect(14) && !ps.has_effect(5) && ps.have_palettes && ps.has_palette(1) && !ps.has_palette(2),
        "decoded into PixelState");
  CHECK(m.other_asked <= 20, "no unknown pages asked in a loop (%u)", m.other_asked);
  // the maps are read again after a restart (the firmware may have changed)
  m.uptime = 0;
  m.effect_ids[3] = 5;
  const uint32_t maps_before = m.asked[1];
  run(p, m, now, 30000);
  CHECK(m.asked[1] > maps_before && p.pixel(0).has_effect(5), "after a restart the maps are read again (%u)", m.asked[1]);
  // a receiver that says UNSUPPORTED is not asked again
  UdModel u;
  u.unsupported_fx = true;
  TelemetryPoller q;
  uint32_t t2 = 5000;
  q.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  q.locked(t2);
  run(q, u, t2, 60000);
  CHECK(u.asked[0] + u.asked[1] + u.asked[2] == 0 && !q.pixel(0).have_state && q.pixel(0).has_effect(5),
        "UNSUPPORTED: nothing decoded, nothing marked missing");
  // a receiver without the capability never sees page 0x81
  UdModel plain;
  plain.caps = ULTRA_HWCAP_RETURN | ULTRA_HWCAP_RELAY;
  TelemetryPoller r;
  uint32_t t3 = 9000;
  r.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  r.locked(t3);
  run(r, plain, t3, 60000);
  CHECK(plain.asked[0] + plain.asked[1] + plain.asked[2] == 0 && r.data(0).have_identity, "no page 0x81 without HWCAP_PIXEL_FX");
  std::printf("  poller: page 0x81 parts, the maps once and after a restart, UNSUPPORTED, no capability\n");
}

static void test_poller_line() {
  // a receiver with ULTRA_HWCAP_CHAIN on a normal Ultra line: the line (part 3) is read like the state, the maps once
  {
    UdModel m;
    m.caps |= ULTRA_HWCAP_CHAIN;
    uint32_t now = 100000;
    TelemetryPoller p;
    p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    p.set_line_pages(true);
    p.locked(now);
    run(p, m, now, 20000);
    CHECK(m.asked[3] >= 3 && m.asked[0] >= 3 && m.asked[1] == 1, "part 3 read repeatedly (%u), the state too (%u), the maps once (%u)", m.asked[3],
          m.asked[0], m.asked[1]);
    const PixelLineState &l = p.pixel(0).line;
    CHECK(l.have && l.mode == ULTRA_PXLM_CHAIN && (l.flags & ULTRA_PXL_F_LOCKED) && l.frames == static_cast<uint16_t>(m.asked[3] * 10),
          "decoded into PixelState::line (mode %u, frames %u)", l.mode, l.frames);
    CHECK(pixel_line_status(p.data(0).have_identity, p.data(0).hw_caps, p.pixel(0), ULTRA_PXLM_CHAIN) == "Chain, locked", "its status text");
  }
  // a digital port (line pages off): the capability is there, the page is never asked for
  {
    UdModel m;
    m.caps |= ULTRA_HWCAP_CHAIN;
    uint32_t now = 100000;
    TelemetryPoller p;
    p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    p.locked(now);
    run(p, m, now, 20000);
    CHECK(m.asked[3] == 0 && m.asked[0] >= 3 && !p.pixel(0).line.have, "a digital port: part 3 is never asked (%u)", m.asked[3]);
  }
  // an older receiver (no ULTRA_HWCAP_CHAIN): never asked either
  {
    UdModel m;
    uint32_t now = 100000;
    TelemetryPoller p;
    p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    p.set_line_pages(true);
    p.locked(now);
    run(p, m, now, 20000);
    CHECK(m.asked[3] == 0 && m.asked[0] >= 3, "no HWCAP_CHAIN: part 3 is not asked (%u)", m.asked[3]);
  }
  // UNSUPPORTED on part 3: not asked again, the rest goes on
  {
    UdModel m;
    m.caps |= ULTRA_HWCAP_CHAIN;
    m.unsupported_line = true;
    uint32_t now = 100000;
    TelemetryPoller p;
    p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    p.set_line_pages(true);
    p.locked(now);
    run(p, m, now, 60000);
    CHECK(m.asked[3] == 0 && m.asked[0] >= 10 && !p.pixel(0).line.have, "UNSUPPORTED part 3: nothing decoded, the state still read (%u)", m.asked[0]);
  }
  // twelve pages' bits fit the masks: UNSUPPORTED on the last page doesn't lose the others (bit 16 and up)
  CHECK(TP_COUNT == 17 && TP_PX_LINE == 16, "17 poll pages (TP_PX_LINE is bit 16 of the masks)");
  std::printf("  poller: page 0x81 part 3 only with HWCAP_CHAIN on a normal Ultra line, UNSUPPORTED not asked again\n");
}

int main() {
  test_chips();
  test_slot();
  test_page();
  test_line();
  test_tracker();
  test_scheduler();
  test_config();
  test_chain_line();
  test_poller();
  test_poller_line();
  if (failures == 0)
    std::printf("ALL ULTRA DIGITAL TESTS PASSED (0 failures)\n");
  else
    std::printf("FAILED (%d failures)\n", failures);
  return failures == 0 ? 0 : 1;
}
