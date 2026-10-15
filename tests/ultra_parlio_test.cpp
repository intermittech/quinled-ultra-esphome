// Host test of the PARLIO lanes (dig2analog_ultra_parlio on an ESP32-P4: ultra_parlio.h) and of the telemetry
// coordinator for lines sharing a UART. The bus' DMA, render task and timer are modelled the way
// components/dig2analog_ultra_parlio/ultra_parlio_bus.cpp drives them (chunk after chunk; at each buffer switch the
// chunk after the one that just started is rendered; a timer drops the direction pins); the hub's own loop(),
// transmit_() and window code are cut unchanged from dig2analog_ultra.cpp by tests/run_ultra_parlio_test.py.
//  1. waveform: every lane, sample for sample, is what the RMT encoder (encode_symbols) sends with the lane's timing
//     on the sample grid (16 and 8-bit samples, 20 and 10 MHz, an inverted lane, lanes with their own high times);
//     the lanes independent of each other; the rounding within the Physical Layer;
//  2. TIME: every frame's header TIME is its last falling edge on the stream, on every lane;
//  3. windows: an RW frame's lane stays quiet for its window while the others send; the direction pin drops within
//     10 us after the RW frame's last bit, never before it;
//  4. slips: the render late, a chunk played twice: the clock moves with it, the frames reserved before refused, the
//     frames after it stamped right;
//  5. 16 lanes on the hub's own code, 4 UARTs shared by 4 lanes each: windows of one UART never overlap, windows of
//     different UARTs do, every receiver answers with its own data, a silent lane leaves the others answering;
//  6. RMT lines sharing a UART (the same coordinator);
//  7. a firmware update session on a lane: every page pause on the wire.
// Build: python tests/run_ultra_parlio_test.py (also its negative controls and the render benchmark)

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/ultra_parlio.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"
#include "../components/dig2analog_ultra/update_session.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>

#define ESP_LOGE(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGW(...) ((void) 0)
#define IRAM_ATTR

namespace esphome::dig2analog_ultra {

static int failures = 0;
#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      failures++; \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
    } \
  } while (0)

static const char *const TAG = "test";
static uint32_t now_ms = 0;
static int64_t clock_us = 0;   // the host clock (esp_timer)
static uint32_t millis() { return now_ms; }
static int64_t esp_timer_get_time() { return clock_us; }
static void set_clock(int64_t us) {
  clock_us = us;
  now_ms = static_cast<uint32_t>(us / 1000);
}

// ================================================================================================ the bus, modelled
/// A lane's timing in ns (the hub's YAML) and on the grid.
struct Grid {
  uint32_t sample_ns;
  uint16_t period;
  ParlioLaneTiming lane[PARLIO_MAX_LANES];
};
static uint16_t to_samples(uint32_t ns, uint32_t sample_ns) { return static_cast<uint16_t>((ns + sample_ns / 2) / sample_ns); }
/// as parlio_timing.py: highs rounded, every bit the longest bit of the lanes
static Grid grid(uint32_t sample_ns, const std::vector<std::array<uint32_t, 4>> &ns) {
  Grid g{};
  g.sample_ns = sample_ns;
  for (size_t l = 0; l < ns.size(); l++) {
    const uint16_t h0 = to_samples(ns[l][0], sample_ns), l0 = to_samples(ns[l][1], sample_ns);
    const uint16_t h1 = to_samples(ns[l][2], sample_ns), l1 = to_samples(ns[l][3], sample_ns);
    g.lane[l] = ParlioLaneTiming{h0, h1};
    g.period = std::max<uint16_t>(g.period, std::max<uint16_t>(h0 + l0, h1 + l1));
  }
  return g;
}

/// The bus as ultra_parlio_bus.cpp drives it: chunk c plays from t0 + c x chunk; at each switch (chunk k starts) the
/// chunk after it is rendered into the buffer just freed. wire: every sample played, in order.
struct Stream {
  ParlioClock clock;
  ParlioLane lanes[PARLIO_MAX_LANES];
  ParlioRenderer r;
  ParlioClock::State st;
  uint8_t width{16};
  uint16_t period{27};
  uint16_t invert{0};
  uint32_t sample_ns{50};
  std::vector<uint8_t> buf[2];
  std::vector<uint16_t> wire;
  std::vector<ParlioRenderer::Release> releases;
  int64_t t0_ns{0};      // the stream's real start (the wire's sample 0)
  uint64_t rendered{0};  // chunks rendered

  bool init(const Grid &g, uint8_t w, uint32_t chunk_groups, uint8_t lanes_used, uint16_t inv, int64_t start_ns) {
    this->sample_ns = g.sample_ns;
    this->period = g.period;
    this->width = w;
    this->invert = inv;
    this->clock.configure(g.sample_ns, g.period, chunk_groups);
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++)
      this->lanes[k].attach(&this->clock, k);
    if (!this->r.init(w, g.period, chunk_groups, g.lane, lanes_used, inv))
      return false;
    for (int i = 0; i < 2; i++) {
      this->buf[i].assign(this->r.chunk_bytes(), 0xEE);
      this->r.clear(this->buf[i].data(), static_cast<uint8_t>(i));
    }
    this->wire.clear();
    this->releases.clear();
    this->t0_ns = start_ns;
    this->st = ParlioClock::State{};
    this->st.t0_ns = start_ns;
    this->st.next_chunk = 2;
    this->st.running = true;
    this->clock.write(this->st);
    this->play(0);   // chunk 0 plays (idle), chunk 1 waits (idle)
    return true;
  }
  uint64_t chunk_ns() const { return this->clock.chunk_ns(); }
  /// the host time a chunk starts (no slips)
  int64_t chunk_start_ns(int64_t c) const { return this->t0_ns + c * static_cast<int64_t>(this->chunk_ns()); }
  void play(int64_t c) {
    const uint8_t *b = this->buf[c & 1].data();
    const size_t n = this->r.chunk_samples();
    for (size_t i = 0; i < n; i++)
      this->wire.push_back(this->width == 8 ? b[i] : static_cast<uint16_t>(b[2 * i] | b[2 * i + 1] << 8));
  }
  /// the render task at a switch: chunk next_chunk - 1 started; ready_ns: when this render hands its chunk over
  int64_t render(int64_t switch_ns, int64_t ready_ns) {
    const int64_t moved = parlio_slip(this->st, this->chunk_ns(), switch_ns, ready_ns);
    const int64_t c = this->st.next_chunk;
    this->st.next_chunk = c + 1;
    this->clock.write(this->st);
    ParlioLane *ls[PARLIO_MAX_LANES];
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++)
      ls[k] = &this->lanes[k];
    this->r.render(c, static_cast<uint8_t>(c & 1), this->buf[c & 1].data(), ls, &this->clock, this->st);
    for (uint8_t i = 0; i < this->r.releases(); i++)
      this->releases.push_back(this->r.release(i));
    this->rendered++;
    return moved;
  }
  /// the next switch on time: the chunk that waited starts, the one after it is rendered
  void step() {
    const int64_t k = this->st.next_chunk - 1;
    this->play(k);
    const int64_t sw = this->st.t0_ns + k * static_cast<int64_t>(this->chunk_ns());
    this->render(sw, sw + 50000);
  }
  /// a lane's level at a sample (its idle level 0)
  uint8_t level(uint8_t lane, size_t s) const {
    return static_cast<uint8_t>(((this->wire[s] ^ this->invert) >> lane) & 1u);
  }
};

/// A test symbol for the RMT encoder (the same fields as rmt_symbol_word_t).
struct Sym {
  uint32_t duration0, level0, duration1, level1;
};

/// What the RMT encoder sends for a frame at the lane's timing (in samples), from sample `at`, with `quiet` samples of
/// reset after it, written into exp.
static void rmt_waveform(std::vector<uint8_t> &exp, size_t at, const uint8_t *bytes, size_t n, const ParlioLaneTiming &t,
                         uint16_t period, uint32_t quiet) {
  FrameSymbols<Sym> fs;
  fs.bit0 = Sym{t.h0, 1, static_cast<uint32_t>(period - t.h0), 0};
  fs.bit1 = Sym{t.h1, 1, static_cast<uint32_t>(period - t.h1), 0};
  fs.reset = Sym{quiet / 2, 0, quiet - quiet / 2, 0};
  std::vector<Sym> syms(n * 8 + 1);
  bool done = false;
  const size_t k = encode_symbols(fs, bytes, n, 0, syms.size(), syms.data(), &done);
  size_t pos = at;
  for (size_t i = 0; i < k; i++) {
    for (int half = 0; half < 2; half++) {
      const uint32_t d = half ? syms[i].duration1 : syms[i].duration0;
      const uint8_t lv = static_cast<uint8_t>(half ? syms[i].level1 : syms[i].level0);
      for (uint32_t j = 0; j < d && pos < exp.size(); j++)
        exp[pos++] = lv;
    }
  }
}

struct Posted {
  int64_t start;   // group
  std::vector<uint8_t> bytes;
  uint32_t quiet;  // groups
  uint32_t time_us;     // its header TIME (Ultra frames), else 0
  int64_t time_full;    // as build_frame returned it
};

// ================================================================================================ 1. waveform
/// Random frames on every lane, posted as a hub posts them (reserve, build, post once the last one was taken), then
/// each lane's samples against the RMT encoder's waveform of the frames it took, at their reserved starts.
static void test_waveform() {
  struct Case {
    const char *name;
    uint32_t sample_ns;
    uint8_t width, lanes;
    uint16_t invert;
    bool own_highs;
    uint8_t grid;   // the renderer's path: the unrolled default grid (1: 20 MHz, 2: 10 MHz) or the runs (0)
  };
  const Case cases[] = {
      {"16 lanes, 20 MHz", 50, 16, 16, 0, false, 1},
      {"16 lanes, 20 MHz, lane 11 inverted", 50, 16, 16, 0x0800, false, 1},
      {"16 lanes, 20 MHz, lanes 3 and 12 inverted, lanes with their own high times", 50, 16, 16, 0x1008, true, 0},
      {"6 lanes, 8-bit samples, 20 MHz, lane 1 inverted", 50, 8, 6, 0x0002, false, 1},
      {"8 lanes, 8-bit samples, 20 MHz, lanes with their own high times", 50, 8, 8, 0, true, 0},
      {"12 lanes, 10 MHz", 100, 16, 12, 0, false, 2},
      {"5 lanes, 8-bit samples, 10 MHz", 100, 8, 5, 0x0010, false, 2},
  };
  for (const Case &c : cases) {
    std::vector<std::array<uint32_t, 4>> ns(c.lanes, std::array<uint32_t, 4>{250, 1050, 700, 650});
    if (c.own_highs) {
      ns[5] = {300, 1000, 800, 600};
      ns[c.lanes > 9 ? 9 : 2] = {350, 900, 600, 700};
    }
    const Grid g = grid(c.sample_ns, ns);
    Stream s;
    const uint32_t chunk_groups = c.sample_ns == 50 ? 96 : 64;
    CHECK(s.init(g, c.width, chunk_groups, c.lanes, c.invert, 1000000000) && s.r.grid() == c.grid,
          "%s: the renderer takes the timing (path %u)", c.name, s.r.grid());
    std::mt19937 rng(7 + c.lanes);
    std::vector<std::vector<Posted>> posted(c.lanes);
    std::vector<std::vector<uint8_t>> frame(c.lanes);
    for (int sw = 0; sw < 300; sw++) {
      for (uint8_t l = 0; l < c.lanes; l++) {
        ParlioLane &lane = s.lanes[l];
        if (!lane.ready())
          lane.begin(400, nullptr, nullptr);
        if (lane.busy() || rng() % 3 == 0)
          continue;
        lane.reserve_us();
        frame[l].resize(1 + rng() % 300);   // the hub's buffer: the renderer copies it when it takes it
        for (auto &b : frame[l])
          b = static_cast<uint8_t>(rng());
        const uint32_t quiet_us = 300 + rng() % 400;
        if (lane.post(frame[l].data(), frame[l].size(), quiet_us, false)) {
          const uint32_t q = static_cast<uint32_t>((quiet_us * 1000ULL + s.clock.group_ns() - 1) / s.clock.group_ns());
          posted[l].push_back(Posted{lane.reserved_group(), frame[l], q, 0, 0});
        }
      }
      s.step();
    }
    for (int sw = 0; sw < 6; sw++)
      s.step();   // every frame posted is on the wire
    const uint32_t gs = 8u * g.period;
    bool all = true, apart = true;
    size_t frames = 0;
    for (uint8_t l = 0; l < c.lanes; l++) {
      std::vector<uint8_t> exp(s.wire.size(), 0);
      int64_t free = 0;
      for (const Posted &p : posted[l]) {
        apart &= p.start >= free;   // after the last frame and its quiet
        free = p.start + static_cast<int64_t>(p.bytes.size()) + p.quiet;
        rmt_waveform(exp, static_cast<size_t>(p.start) * gs, p.bytes.data(), p.bytes.size(), g.lane[l], g.period, p.quiet * gs);
        frames++;
      }
      size_t bad = SIZE_MAX;
      for (size_t i = 0; i < s.wire.size() && bad == SIZE_MAX; i++) {
        if (s.level(l, i) != exp[i])
          bad = i;
      }
      if (bad != SIZE_MAX) {
        all = false;
        std::printf("  lane %u differs at sample %u (group %u): %u, the RMT encoder %u\n", l, static_cast<unsigned>(bad),
                    static_cast<unsigned>(bad / gs), s.level(l, bad), exp[bad]);
      }
      CHECK(s.lanes[l].refused() == 0, "%s: lane %u refused nothing (%u)", c.name, l, s.lanes[l].refused());
    }
    CHECK(all && apart && frames > 100, "%s: %u frames, every lane sample for sample the RMT encoder's waveform at the "
          "lane's grid timing, frames apart by their quiet", c.name, static_cast<unsigned>(frames));
  }

  // one lane's waveform doesn't depend on the others: the same frames on lane 2 among silent and among busy lanes
  const Grid g = grid(50, std::vector<std::array<uint32_t, 4>>(16, std::array<uint32_t, 4>{250, 1050, 700, 650}));
  std::vector<uint8_t> lane2[2];
  for (int busy = 0; busy < 2; busy++) {
    Stream s;
    s.init(g, 16, 96, 16, 0, 0);
    std::mt19937 rng(99), noise(5);
    std::vector<uint8_t> f(200), other[16];
    for (int sw = 0; sw < 60; sw++) {
      for (uint8_t l = 0; l < 16; l++) {
        ParlioLane &lane = s.lanes[l];
        if (!lane.ready())
          lane.begin(400, nullptr, nullptr);
        if (lane.busy())
          continue;
        if (l == 2) {
          lane.reserve_us();
          for (auto &b : f)
            b = static_cast<uint8_t>(rng());
          lane.post(f.data(), f.size(), 300, false);
        } else if (busy) {
          lane.reserve_us();
          other[l].assign(1 + noise() % 250, 0);
          for (auto &b : other[l])
            b = static_cast<uint8_t>(noise());
          lane.post(other[l].data(), other[l].size(), 300, false);
        }
      }
      s.step();
    }
    for (size_t i = 0; i < s.wire.size(); i++)
      lane2[busy].push_back(s.level(2, i));
  }
  CHECK(lane2[0] == lane2[1], "lane 2's waveform is the same among silent and among busy lanes");

  // the rounding (parlio_timing.py): at 20 MHz the default timing is exact but the '0' low, stretched to the '1''s
  // period (1100 instead of 1050 ns); at 10 MHz '0' high 300 ns. Every value within the Physical Layer, every high
  // time within half a sample of the YAML's
  struct Rounding {
    uint32_t sample_ns;
    uint32_t ns[4];
  };
  for (const Rounding &r : {Rounding{50, {250, 1050, 700, 650}}, Rounding{100, {250, 1050, 700, 650}},
                            Rounding{125, {250, 1050, 700, 650}}, Rounding{25, {250, 1050, 700, 650}}}) {
    const Grid gr = grid(r.sample_ns, {std::array<uint32_t, 4>{r.ns[0], r.ns[1], r.ns[2], r.ns[3]}});
    const uint32_t t0h = gr.lane[0].h0 * r.sample_ns, t1h = gr.lane[0].h1 * r.sample_ns;
    const uint32_t t0l = (gr.period - gr.lane[0].h0) * r.sample_ns, t1l = (gr.period - gr.lane[0].h1) * r.sample_ns;
    const bool limits = t0h >= 220 && t0h <= 380 && t1h >= 580 && t1h <= 2000 && t0l >= 450 && t0l <= 10000 &&
                        t1l >= 450 && t1l <= 10000;
    const bool near = std::abs(static_cast<int>(t0h) - static_cast<int>(r.ns[0])) <= static_cast<int>(r.sample_ns / 2) &&
                      std::abs(static_cast<int>(t1h) - static_cast<int>(r.ns[2])) <= static_cast<int>(r.sample_ns / 2);
    const bool lows = t0l + r.sample_ns / 2 >= r.ns[1] && t1l + r.sample_ns / 2 >= r.ns[3];
    CHECK(limits && near && lows, "%u ns samples: '0' %u / %u ns, '1' %u / %u ns: within the Physical Layer, highs within "
          "half a sample, lows never shorter", r.sample_ns, t0h, t0l, t1h, t1l);
    if (r.sample_ns == 50 || r.sample_ns == 100)
      std::printf("  rounding at %u MHz: '0' %u / %u ns, '1' %u / %u ns (the YAML: %u / %u, %u / %u)\n",
                  1000 / r.sample_ns, t0h, t0l, t1h, t1l, r.ns[0], r.ns[1], r.ns[2], r.ns[3]);
  }
  std::printf("  waveform: every lane is the RMT encoder's, sample for sample (16 / 8-bit, 20 / 10 MHz, inverted, own "
              "high times)\n");
}

// ================================================================================================ 2. TIME
/// Ultra frames of real schedulers (1 to 50 boards) on 16 lanes: each header's TIME (and what build_frame returned) is
/// its last falling edge on the stream, within 1.5 us: the us rounding (0.5 us each for the start and the bits), and
/// the frame's last bit, the header's CRC, which the scheduler counts before TIME is in it (a '0' or a '1' low: 450 ns
/// apart at 20 MHz); RMT lines have the same.
static void test_time() {
  const Grid g = grid(50, std::vector<std::array<uint32_t, 4>>(16, std::array<uint32_t, 4>{250, 1050, 700, 650}));
  Stream s;
  s.init(g, 16, 96, 16, 0, 777777123);   // an odd start: the stream's t0 isn't a whole us
  const uint32_t sample_ns = g.sample_ns, gs = 8u * g.period;
  Scheduler sched[16];
  std::vector<uint8_t> buf[16];
  std::vector<std::vector<Posted>> posted(16);
  for (uint8_t l = 0; l < 16; l++) {
    const uint16_t boards = static_cast<uint16_t>(l == 0 ? 1 : l == 1 ? 50 : 1 + l * 3);
    sched[l].init(boards, 11 + l, 3);
    sched[l].set_bit_timing(g.lane[l].h0 * sample_ns, (g.period - g.lane[l].h0) * sample_ns, g.lane[l].h1 * sample_ns,
                            (g.period - g.lane[l].h1) * sample_ns);
    buf[l].assign(sched[l].frame_bytes(), 0);
    s.lanes[l].begin(buf[l].size(), nullptr, nullptr);
  }
  for (int sw = 0; sw < 400; sw++) {
    set_clock(s.chunk_start_ns(s.st.next_chunk - 1) / 1000 + 100);
    for (uint8_t l = 0; l < 16; l++) {
      ParlioLane &lane = s.lanes[l];
      if (lane.busy())
        continue;
      if (sw % 7 == static_cast<int>(l % 7)) {   // commands with start times now and then
        const uint16_t m = 30000, a[CHANNELS] = {1000, 2000, 3000, 4000, 5000}, b[CHANNELS] = {65535, 65535, 65535, 65535, 65535};
        for (uint16_t board = 0; board < sched[l].num_boards(); board++)
          sched[l].light_set(board, 0, 0, m, a, b, 500, clock_us + 30000);
      }
      const int64_t start = lane.reserve_us();
      const int64_t t = sched[l].build_frame(start, buf[l].data());
      if (lane.post(buf[l].data(), buf[l].size(), 300, false)) {
        uint32_t hdr = 0;
        ultra_parse_header(buf[l].data() + sched[l].header_offset(), &hdr);
        posted[l].push_back(Posted{lane.reserved_group(), buf[l], 28, hdr, t});
      }
    }
    s.step();
  }
  for (int sw = 0; sw < 8; sw++)
    s.step();
  int64_t worst = 0;
  size_t frames = 0;
  bool edges = true, header = true;
  for (uint8_t l = 0; l < 16; l++) {
    for (const Posted &p : posted[l]) {
      const size_t n = p.bytes.size();
      const size_t last = static_cast<size_t>((p.start + static_cast<int64_t>(n)) * gs - g.period +
                                              ((p.bytes[n - 1] & 1) ? g.lane[l].h1 : g.lane[l].h0));
      if (last >= s.wire.size())
        continue;
      edges &= s.level(l, last - 1) == 1 && s.level(l, last) == 0;   // it is the frame's last falling edge
      const int64_t edge_ns = s.t0_ns + static_cast<int64_t>(last) * sample_ns;
      const int64_t err = p.time_full * 1000 - edge_ns;
      worst = std::max<int64_t>(worst, err < 0 ? -err : err);
      header &= p.time_us == static_cast<uint32_t>(p.time_full);
      frames++;
    }
  }
  CHECK(edges && header && worst <= 1500 && frames > 500, "TIME of %u frames on 16 lanes (1 to 50 boards): the last "
        "falling edge on the stream within %lld ns (1.5 us), the header carries it", static_cast<unsigned>(frames),
        static_cast<long long>(worst));
  std::printf("  TIME: %u frames on 16 lanes, the header's TIME within %lld ns of the frame's last edge\n",
              static_cast<unsigned>(frames), static_cast<long long>(worst));
}

// ================================================================================================ 3. windows
/// Lane 6 sends RW frames (its window and hold-off as its quiet, a direction pin), the other lanes keep sending: the
/// lane is low for its whole window while the others have edges in it; the timer (1 us, an interrupt 1-3 us late)
/// drops the direction pin within 10 us after the RW frame's last falling edge, never before it.
static int64_t release_isr_latency_ns(size_t i) { return 1000 + static_cast<int64_t>(i % 3) * 1000; }

static void test_windows() {
  const Grid g = grid(50, std::vector<std::array<uint32_t, 4>>(16, std::array<uint32_t, 4>{250, 1050, 700, 650}));
  Stream s;
  s.init(g, 16, 96, 16, 0, 5000000123);
  const uint32_t gs = 8u * g.period, sample_ns = g.sample_ns;
  const uint8_t rw_lane = 6;
  const uint32_t window = ultra_rw_window_us(4, ULTRA_RW_RATE_1M) + ULTRA_RW_HOLDOFF_US;
  std::vector<uint8_t> f[16];
  std::vector<Posted> rw;
  std::mt19937 rng(3);
  int released = 0;
  for (uint8_t l = 0; l < 16; l++)
    s.lanes[l].begin(300, [](void *arg) { (*static_cast<int *>(arg))++; }, &released);
  for (int sw = 0; sw < 300; sw++) {
    for (uint8_t l = 0; l < 16; l++) {
      ParlioLane &lane = s.lanes[l];
      if (lane.busy())
        continue;
      lane.reserve_us();
      f[l].assign(80 + rng() % 150, 0);
      for (auto &b : f[l])
        b = static_cast<uint8_t>(rng());
      const bool is_rw = l == rw_lane;
      if (lane.post(f[l].data(), f[l].size(), is_rw ? window : 300, is_rw) && is_rw)
        rw.push_back(Posted{lane.reserved_group(), f[l], 0, 0, 0});
    }
    s.step();
  }
  for (int sw = 0; sw < 6; sw++)
    s.step();
  bool quiet = true, others = true, order = true;
  int64_t worst = 0, least = INT64_MAX;
  size_t windows = 0, other_edges = 0;
  CHECK(s.releases.size() == rw.size(), "a release per RW frame taken (%u / %u)", static_cast<unsigned>(s.releases.size()),
        static_cast<unsigned>(rw.size()));
  for (size_t i = 0; i < rw.size() && i < s.releases.size(); i++) {
    const Posted &p = rw[i];
    const size_t n = p.bytes.size();
    const size_t last = static_cast<size_t>((p.start + static_cast<int64_t>(n)) * gs - g.period +
                                            ((p.bytes[n - 1] & 1) ? g.lane[rw_lane].h1 : g.lane[rw_lane].h0));
    const size_t end = last + static_cast<size_t>(window * 1000ULL / sample_ns);
    if (end >= s.wire.size())
      continue;
    for (size_t k = last; k < end; k++)
      quiet &= s.level(rw_lane, k) == 0;
    size_t edges = 0;
    for (size_t k = last + 1; k < end; k++)
      edges += (s.wire[k] ^ s.wire[k - 1]) != 0;
    others &= edges > 1000;
    other_edges += edges;
    // the timer: an alarm at the release's us, its interrupt 1-3 us later
    const ParlioRenderer::Release &r = s.releases[i];
    order &= r.lane == rw_lane;
    const int64_t fired_ns = (r.at_ns + 999) / 1000 * 1000 + release_isr_latency_ns(i);
    const int64_t edge_ns = s.t0_ns + static_cast<int64_t>(last) * sample_ns;
    worst = std::max(worst, fired_ns - edge_ns);
    least = std::min(least, fired_ns - edge_ns);
    windows++;
  }
  CHECK(quiet && others && windows > 20, "%u RW windows: lane 6 low for W + hold-off (%u us) after each, the other 15 "
        "lanes send meanwhile (%u edges)", static_cast<unsigned>(windows), window, static_cast<unsigned>(other_edges));
  CHECK(order && least >= static_cast<int64_t>(PARLIO_RELEASE_NS) && worst <= 10000, "the direction pin drops %.1f-%.1f "
        "us after the RW frame's last falling edge (within 10 us, never before it)", least / 1000.0, worst / 1000.0);
  std::printf("  windows: lane 6 quiet through %u windows while 15 lanes send; direction pin %.1f-%.1f us after the last "
              "bit\n", static_cast<unsigned>(windows), least / 1000.0, worst / 1000.0);
}

// ================================================================================================ 4. slips
/// The render task late: the DMA plays the chunk it has again. parlio_slip foresees it (or measures it at the next
/// switch): the clock moves by the repeat, frames reserved before are refused (counted, like a frame lost on the
/// wire), and the frames after it carry the TIME of their real last edge.
static void test_slips() {
  // the rule alone
  {
    const uint64_t c = 1000000;
    ParlioClock::State st{};
    st.t0_ns = 0;
    st.next_chunk = 5;   // chunk 4 started at 4 ms
    CHECK(parlio_slip(st, c, 4000000, 4600000) == 0 && st.t0_ns == 0 && st.epoch == 0, "on time: no slip");
    CHECK(parlio_slip(st, c, 4000000, 5100000) == 1 && st.t0_ns == 1000000 && st.epoch == 1,
          "handed over after chunk 4's end: foreseen, one chunk later");
    st.next_chunk = 6;   // the switch to chunk 5 came on time after all (the render was quicker): back
    CHECK(parlio_slip(st, c, 5000500, 5300000) == -1 && st.t0_ns == 0 && st.epoch == 2, "the slip didn't happen: back");
    st.next_chunk = 7;   // a repeat nobody foresaw: chunk 6 started 2 chunks late
    CHECK(parlio_slip(st, c, 8000300, 8200000) == 2 && st.t0_ns == 2000000 && st.epoch == 3, "measured: 2 chunks later");
  }
  // end to end: frames on 4 lanes; at switch 120 the task is 1.3 chunks late
  const Grid g = grid(50, std::vector<std::array<uint32_t, 4>>(4, std::array<uint32_t, 4>{250, 1050, 700, 650}));
  Stream s;
  s.init(g, 8, 96, 4, 0, 1000000000);
  const uint32_t gs = 8u * g.period, groups = 96;
  const size_t chunk_samples = s.r.chunk_samples();
  Scheduler sched[4];
  std::vector<uint8_t> buf[4];
  struct Out {
    int64_t start;
    size_t bytes;
    int64_t time;
    uint8_t last;
    bool refused;
  };
  std::vector<Out> out[4];
  for (uint8_t l = 0; l < 4; l++) {
    sched[l].init(static_cast<uint16_t>(4 + l * 5), 3 + l, 3);
    sched[l].set_bit_timing(250, (g.period - 5) * 50, 700, (g.period - 14) * 50);
    buf[l].assign(sched[l].frame_bytes(), 0);
    s.lanes[l].begin(buf[l].size(), nullptr, nullptr);
  }
  int64_t repeat = -1;   // the chunk played twice
  for (int sw = 0; sw < 240; sw++) {
    for (uint8_t l = 0; l < 4; l++) {
      ParlioLane &lane = s.lanes[l];
      const uint32_t r0 = lane.refused();
      const bool busy = lane.busy();
      if (lane.refused() != r0)
        out[l].back().refused = true;   // the stream slipped before it was taken: it never went out
      if (busy)
        continue;
      const int64_t start = lane.reserve_us();
      const int64_t t = sched[l].build_frame(start, buf[l].data());
      if (lane.post(buf[l].data(), buf[l].size(), 300, false))
        out[l].push_back(Out{lane.reserved_group(), buf[l].size(), t, buf[l].back(), false});
    }
    const int64_t k = s.st.next_chunk - 1;
    const int64_t sw_ns = s.t0_ns + static_cast<int64_t>(s.wire.size()) * 50;   // chunk k starts (real time)
    s.play(k);
    if (sw == 120) {
      s.play(k);   // late: chunk k plays again before the next one is handed over
      repeat = k;
      const int64_t moved = s.render(sw_ns, sw_ns + static_cast<int64_t>(s.chunk_ns()) * 13 / 10);
      CHECK(moved == 1, "the late render foresees one chunk played twice (%lld)", static_cast<long long>(moved));
    } else {
      const int64_t moved = s.render(sw_ns, sw_ns + 40000);
      CHECK(moved == 0, "switch %d: no slip measured (%lld)", sw, static_cast<long long>(moved));
    }
  }
  // every frame's TIME against its last edge on the wire: those after the repeat a chunk later on the wire; those it
  // cut through are lost (the boards drop them: their slots' CRCs), those refused never went out
  int64_t worst = 0;
  size_t checked = 0, after = 0, refused = 0, cut = 0;
  bool edges = true;
  for (uint8_t l = 0; l < 4; l++) {
    for (const Out &o : out[l]) {
      if (o.refused) {
        refused++;
        continue;
      }
      const int64_t cs = o.start / groups, ce = (o.start + static_cast<int64_t>(o.bytes) - 1) / groups;
      if (cs <= repeat && repeat < ce) {
        cut++;
        continue;
      }
      const size_t shift = cs > repeat ? chunk_samples : 0;
      const int64_t last_bit = (o.start + static_cast<int64_t>(o.bytes)) * gs - g.period +
                               ((o.last & 1) ? g.lane[l].h1 : g.lane[l].h0);
      const size_t last = static_cast<size_t>(last_bit) + shift;
      if (last >= s.wire.size())
        continue;   // not played yet
      edges &= s.level(l, last - 1) == 1 && s.level(l, last) == 0;
      const int64_t err = o.time * 1000 - (s.t0_ns + static_cast<int64_t>(last) * 50);
      worst = std::max<int64_t>(worst, err < 0 ? -err : err);
      checked++;
      after += shift != 0;
    }
  }
  CHECK(edges && worst <= 1500 && after > 50 && checked > after + 50, "after a slip: %u frames (%u after it) carry their "
        "real last edge as TIME (within %lld ns)", static_cast<unsigned>(checked), static_cast<unsigned>(after),
        static_cast<long long>(worst));
  CHECK(s.st.epoch == 1 && refused >= 1 && refused <= 4, "one slip, one epoch (%u); the frames reserved before it and "
        "not taken yet refused (%u)", s.st.epoch, static_cast<unsigned>(refused));
  std::printf("  slips: a render 1.3 chunks late foreseen, the clock moved a chunk; %u frames refused (reserved before "
              "it), %u cut by the repeat, TIME right after it\n", static_cast<unsigned>(refused), static_cast<unsigned>(cut));
}

// ================================================================================================ 5./6. the hub's code
// The hub's loop(), transmit_(), lane_post_() and window code (build/parlio/hub_*.inc, cut unchanged from
// dig2analog_ultra.cpp) with telemetry and PARLIO on, around the real Scheduler, TelemetryPoller and the lanes of a
// modelled bus; RMT lines on a modelled RMT.
namespace hub_lanes {

#define USE_DIG2ANALOG_ULTRA_TELEMETRY
#define USE_DIG2ANALOG_ULTRA_PARLIO
static constexpr int ESP_OK = 0;
using esp_err_t = int;
static constexpr esp_err_t ESP_ERR_TIMEOUT = 0x107;
static constexpr uint32_t BUSY_HOLD_MS = 250;
typedef struct rmt_channel_t *rmt_channel_handle_t;
struct rmt_tx_done_event_data_t {
  size_t num_symbols;
};
struct rmt_transmit_config_t {
  uint32_t loop_count;
};
struct rmt_symbol_word_t {
  uint32_t duration0 : 15;
  uint32_t level0 : 1;
  uint32_t duration1 : 15;
  uint32_t level1 : 1;
};
static void delayMicroseconds(uint32_t us) { set_clock(clock_us + us); }
static const char *esp_err_to_name(esp_err_t) { return "refused"; }

struct Pin {   // a direction or receiver enable pin
  bool level{false};
  void setup() {}
  void digital_write(bool v) { this->level = v; }
};
struct IsrPin {
  Pin *pin{nullptr};
  void digital_write(bool v) { this->pin->level = v; }
};
struct Uart {
  std::vector<uint8_t> rx;
  size_t available() const { return this->rx.size(); }
  bool read_array(uint8_t *d, size_t n) {
    if (n > this->rx.size())
      return false;
    std::memcpy(d, this->rx.data(), n);
    this->rx.erase(this->rx.begin(), this->rx.begin() + static_cast<std::ptrdiff_t>(n));
    return true;
  }
};

/// RMT lines: one frame at a time, done after its bits and its reset symbol (80 MHz ticks), then its interrupt.
struct UltraHub;
struct Rmt {
  bool busy{false};
  int64_t done_us{0};
  std::vector<uint8_t> frame;   // the last one
  int64_t t0_us{0};             // its last bit
  bool rw_pending{false};
  UltraHub *hub{nullptr};
};
static Rmt rmts[8];
static esp_err_t rmt_tx_wait_all_done(int ch, int) { return rmts[ch].busy ? -1 : ESP_OK; }
static esp_err_t rmt_transmit(int ch, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *);

struct UltraHub {
  void loop();                        // the component's (hub_loop.inc)
  void transmit_();                   // (hub_transmit.inc)
  esp_err_t lane_post_();
  bool rw_due_(bool urgent);          // (hub_rw.inc)
  bool rw_uart_free_() const;
  void finish_window_(bool wait);
  void rw_publish_();
  static bool rw_done_isr_(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *arg);
  static void rw_lane_release_(void *arg);
  static int64_t now_us_() { return clock_us; }
  // stand-ins
  struct {
    void stop() {}
  } high_freq_;
  void service_calibration_() {}
  void wake_() {}
  bool boards_powered_() const { return true; }
  bool status_has_warning() const { return this->warning_; }
  void status_set_warning() { this->warning_ = true; }
  void status_clear_warning() { this->warning_ = false; }

  // the line
  uint16_t preamble_{0};
  bool preamble_full_{true}, tx_preamble_end_{false}, synced_once_{false};
  uint8_t switch_frame_{ULTRA_SWITCH_WS_FRAMES};
  Scheduler sched_;
  uint16_t num_boards_{2};
  std::vector<uint8_t> buf_;
  uint8_t *tx_buf_{nullptr};
  size_t tx_bytes_{0};
  bool tx_prepared_{false}, tx_normal_{true};
  uint32_t build_peak_us_{0}, frames_{0}, last_busy_ms_{0}, last_frame_ms_{0}, refresh_interval_ms_{200};
  bool high_freq_active_{false}, has_sent_{false}, started_{true}, warning_{false};
  int channel_{0}, encoder_{0};
  struct {
    FrameSymbols<rmt_symbol_word_t> symbols;
  } enc_;
  uint8_t pin_{16};
  uint32_t reset_ns_{300000};
  ParlioLane *lane_{nullptr};
  uint32_t lane_lead_us_{0};
  // the return channel
  Uart *rw_uart_{nullptr};
  Pin *rw_dir_pin_{nullptr};
  IsrPin rw_dir_isr_;
  Pin *rw_rx_enable_pin_{nullptr};
  bool rw_turn_{false};
  bool rw_driving_{true};
  volatile bool rw_release_{false};
  volatile int64_t rw_released_us_{0};
  TelemetryPoller poller_;
  PortMonitor port_monitor_;
  std::vector<uint8_t> rw_slots_, rw_rx_;
  bool rw_next_{false}, tx_rw_{false};
  uint32_t rw_window_us_{0};
  bool rw_open_{false};
  int64_t rw_t0_us_{0}, rw_end_us_{0}, rw_read_us_{0};
  bool rw_powered_{true};
  uint32_t rw_late_{0}, rw_extra_us_{0};
  rmt_symbol_word_t reset_symbol_{}, rw_reset_symbol_{};
  struct {
    std::vector<std::function<void(uint16_t, uint32_t)>> f;
    void call(uint16_t a, uint32_t b) {
      for (auto &x : this->f)
        x(a, b);
    }
  } rw_callback_;
  struct {
    int calls{0};
    void call() { this->calls++; }
  } rw_window_callback_;
  static std::vector<UltraHub *> hubs_;

  /// as setup() with telemetry (2 receivers at 1 Mbaud, a direction pin, a receiver enable pin): on a lane of the bus
  /// with the timing on its grid, or on an RMT channel
  void start(ParlioLane *lane, uint32_t b0h, uint32_t b0l, uint32_t b1h, uint32_t b1l, uint32_t reset_ns, Uart *uart,
             Pin *dir, Pin *re, uint32_t seed) {
    this->sched_.init(this->num_boards_, seed, 3);
    this->sched_.set_bit_timing(b0h, b0l, b1h, b1l);
    this->reset_ns_ = reset_ns;
    this->buf_.assign(this->sched_.frame_bytes(), 0);
    this->tx_buf_ = this->buf_.data();
    rmt_symbol_word_t rst{}, rw{};
    rst.duration0 = rst.duration1 = 12000;   // 300 us at 80 MHz
    rw.duration0 = rw.duration1 = 80;        // 2 us
    this->enc_.symbols.reset = rst;
    this->reset_symbol_ = rst;
    this->rw_reset_symbol_ = dir != nullptr ? rw : rst;
    this->lane_ = lane;
    if (lane != nullptr) {
      lane->begin(this->buf_.size(), UltraHub::rw_lane_release_, this);
      this->lane_lead_us_ = lane->lead_us();
    }
    this->rw_uart_ = uart;
    this->rw_dir_pin_ = dir;
    if (dir != nullptr) {
      dir->level = true;
      this->rw_dir_isr_.pin = dir;
    }
    this->rw_rx_enable_pin_ = re;
    if (re != nullptr)
      re->level = false;
    this->poller_.init(2, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    this->rw_slots_.assign(ULTRA_SLOT_BYTES * this->num_boards_, 0);
    this->rw_extra_us_ = this->poller_.max_window_us() + ULTRA_RW_HOLDOFF_US;
    this->preamble_ = 3;   // the start-up preamble
    hubs_.push_back(this);
  }
};
std::vector<UltraHub *> UltraHub::hubs_;

#include "hub_loop.inc"
#include "hub_transmit.inc"
#include "hub_rw.inc"

/// The receivers of a line answer an RW frame's POLLs (a STATUS whose voltage and chip ids are the line's own,
/// IDENTITY without optional pages, the rest unsupported): the bytes a window brings to the UART.
static std::vector<uint8_t> answers(const uint8_t *frame, uint16_t boards, int line) {
  std::vector<uint8_t> out{0x00};   // the turnaround checks' junk
  for (uint16_t b = 0; b < boards; b++) {
    const uint8_t *s = frame + ULTRA_SLOT_BYTES * b;
    if (ultra_poll_valid(s) != 0 || s[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY)
      continue;
    uint8_t a[32]{}, d[24]{};
    uint8_t page = s[ULTRA_POLL_PAGE];
    if (page == ULTRA_PAGE_STATUS) {
      ultra_put24(d + ULTRA_ST_FW_VERSION, 0x000608);
      ultra_put24(d + ULTRA_ST_UPTIME, static_cast<uint32_t>(clock_us / 1000000));
      ultra_put16(d + ULTRA_ST_VBUS, static_cast<uint16_t>(2400 + 50 * line));   // line k: 24 V + 0.5 V x k
      ultra_put16(d + ULTRA_ST_MCU_TEMP, ULTRA_ST_NA);
      d[ULTRA_ST_MEASURED] = ULTRA_ST_M_VBUS;
    } else if (page == ULTRA_PAGE_IDENTITY) {
      d[0] = static_cast<uint8_t>(b + 1);
    } else {
      page = ULTRA_PAGE_UNSUPPORTED;
    }
    a[0] = ULTRA_RSP_SYNC;
    a[1] = page;
    a[2] = static_cast<uint8_t>(s[ULTRA_POLL_INDEX] << 4);
    a[3] = s[2];
    ultra_put16(a + 4, static_cast<uint16_t>(0x1000 + 0x10 * line + b));
    std::memcpy(a + ULTRA_RSP_DATA, d, 24);
    ultra_put16(a + 30, ultra_crc16(a, 30));
    out.push_back(ULTRA_RSP_LEAD);
    out.insert(out.end(), a, a + 32);
  }
  return out;
}
static uint8_t poll_slots(const uint8_t *frame, uint16_t boards) {
  for (uint16_t b = 0; b < boards; b++) {
    const uint8_t *s = frame + ULTRA_SLOT_BYTES * b;
    if (ultra_poll_valid(s) == 0)
      return s[ULTRA_POLL_SLOTS];
  }
  return 0;
}

/// The return channel of every line: an RW frame's receivers turn around only when the hub let go of the line by
/// ULTRA_RW_RELEASE_US after its last bit; their answers reach the line's UART while its receiver is enabled (no
/// receiver_enable_pin: always); two ports of one UART enabled at once garble each other's bytes.
struct Telemetry {
  struct Window {
    int line;
    int64_t t0_us, end_us;
    std::vector<uint8_t> frame;
  };
  std::vector<Window> windows;       // every RW frame seen on a wire
  std::vector<Window> pending;       // its answers still to come
  int uart_of[16]{};
  Uart *uarts[8]{};
  Pin *re[16]{}, *dir[16]{};
  bool on[16];
  uint16_t boards{2};
  int collisions{0}, late{0}, answered{0}, max_enabled{0};
  Telemetry() { std::fill(std::begin(this->on), std::end(this->on), true); }

  /// an RW frame's last bit on line `line` at t0_us
  void rw_frame(int line, int64_t t0_us, const std::vector<uint8_t> &frame) {
    const uint8_t r = poll_slots(frame.data(), this->boards);
    const int64_t end = t0_us + ultra_rw_window_us(r, ULTRA_RW_RATE_1M);
    this->windows.push_back(Window{line, t0_us, end, frame});
    this->pending.push_back(Window{line, t0_us, end, frame});
  }
  /// at the window's last burst: the answers to the UART (or not)
  void tick() {
    int enabled[8]{};
    for (int k = 0; k < 16; k++) {
      if (this->re[k] != nullptr && this->re[k]->level)
        enabled[this->uart_of[k]]++;
    }
    for (int u = 0; u < 8; u++)
      this->max_enabled = std::max(this->max_enabled, enabled[u]);
    for (size_t i = 0; i < this->pending.size();) {
      Window &w = this->pending[i];
      // the turnaround check: the hub let go by ULTRA_RW_RELEASE_US (checked once, then the window runs)
      if (this->dir[w.line] != nullptr && w.t0_us + ULTRA_RW_RELEASE_US <= clock_us && w.t0_us >= 0) {
        if (this->dir[w.line]->level) {
          this->late++;
          this->pending.erase(this->pending.begin() + static_cast<std::ptrdiff_t>(i));
          continue;
        }
        w.t0_us = -w.t0_us - 1;   // checked
      }
      if (clock_us < w.end_us - 50) {
        i++;
        continue;
      }
      const int u = this->uart_of[w.line];
      std::vector<uint8_t> bytes = this->on[w.line] ? answers(w.frame.data(), this->boards, w.line) : std::vector<uint8_t>{0};
      const bool heard = this->re[w.line] == nullptr || this->re[w.line]->level;
      if (heard && this->on[w.line]) {
        if (enabled[u] > 1) {   // another port drives the same RX
          for (auto &b : bytes)
            b ^= 0x5A;
          this->collisions++;
        }
        this->uarts[u]->rx.insert(this->uarts[u]->rx.end(), bytes.begin(), bytes.end());
        this->answered++;
      }
      this->pending.erase(this->pending.begin() + static_cast<std::ptrdiff_t>(i));
    }
  }
  /// windows of lines on one UART that overlap, and of lines on different UARTs
  void overlaps(int *same, int *across) const {
    *same = *across = 0;
    for (size_t i = 0; i < this->windows.size(); i++) {
      for (size_t j = i + 1; j < this->windows.size(); j++) {
        const Window &a = this->windows[i], &b = this->windows[j];
        const int64_t a0 = a.t0_us < 0 ? -a.t0_us - 1 : a.t0_us, b0 = b.t0_us < 0 ? -b.t0_us - 1 : b.t0_us;
        if (a.line == b.line || a0 >= b.end_us || b0 >= a.end_us)
          continue;
        (this->uart_of[a.line] == this->uart_of[b.line] ? *same : *across) += 1;
      }
    }
  }
};
static Telemetry tel;

/// The decoder of one lane's wire, as a board's input sees it: high pulses are bits (>= 472 ns a '1'), a low of 50 us
/// ends the frame (its last falling edge: t0). Checks every Ultra frame's TIME against its t0.
struct Decoder {
  struct Frame {
    int64_t start, t0;   // its first rising and last falling edge (samples)
    std::vector<uint8_t> bytes;
  };
  int64_t rise{-1}, fall{-1}, start{-1};
  std::vector<uint8_t> bits;
  std::vector<Frame> frames;
  void edge(bool up, int64_t s, uint32_t sample_ns) {
    if (up) {
      this->rise = s;
      if (this->bits.empty())
        this->start = s;
    } else if (this->rise >= 0) {
      this->bits.push_back((s - this->rise) * sample_ns >= 472 ? 1 : 0);
      this->fall = s;
    }
  }
  /// the frame is over once the line stayed low 50 us
  bool end(int64_t s, uint32_t sample_ns) {
    if (this->bits.empty() || this->fall < 0 || (s - this->fall) * sample_ns < 50000)
      return false;
    std::vector<uint8_t> bytes(this->bits.size() / 8, 0);
    for (size_t i = 0; i + 8 <= this->bits.size(); i += 8) {
      for (int j = 0; j < 8; j++)
        bytes[i / 8] = static_cast<uint8_t>(bytes[i / 8] << 1 | this->bits[i + j]);
    }
    this->frames.push_back(Frame{this->start, this->fall, bytes});
    this->bits.clear();
    return true;
  }
};

static int64_t ceil_us(int64_t ns) { return (ns + 999) / 1000; }

/// 16 lanes, 4 UARTs with 4 lanes each (each lane with its direction and receiver enable pin, 2 receivers): the
/// hubs' own code polls; every RW frame is decoded from the rendered stream and answered through the modelled
/// transceivers. Windows on one UART never overlap (and never 2 receivers enabled on one), windows of different UARTs
/// do; every receiver answers with its own line's values; the direction pin drops within 10 us of each RW frame's
/// last bit; each lane is quiet for its windows while others send; every frame's TIME is its last edge; a lane whose
/// receivers go silent leaves the others answering.
static void test_sixteen_lanes() {
  set_clock(1000000);
  UltraHub::hubs_.clear();
  tel = Telemetry();
  static Stream bus;
  const Grid g = grid(50, std::vector<std::array<uint32_t, 4>>(16, std::array<uint32_t, 4>{250, 1050, 700, 650}));
  bus.init(g, 16, 96, 16, 0, clock_us * 1000 + 321);
  const uint32_t sample_ns = g.sample_ns, P = g.period;
  const uint32_t group_ns = 8 * P * sample_ns;
  const uint32_t reset_ns = (300000 + group_ns - 1) / group_ns * group_ns;   // the reset in whole groups
  static UltraHub hubs[16];
  static Uart uarts[4];
  static Pin dir[16], re[16];
  for (int k = 0; k < 16; k++) {
    hubs[k] = UltraHub();
    hubs[k].pin_ = static_cast<uint8_t>(20 + k);
    hubs[k].start(&bus.lanes[k], g.lane[k].h0 * sample_ns, (P - g.lane[k].h0) * sample_ns, g.lane[k].h1 * sample_ns,
                  (P - g.lane[k].h1) * sample_ns, reset_ns, &uarts[k / 4], &dir[k], &re[k], 100 + k);
    tel.uart_of[k] = k / 4;
    tel.re[k] = &re[k];
    tel.dir[k] = &dir[k];
  }
  for (int u = 0; u < 4; u++)
    tel.uarts[u] = &uarts[u];
  Decoder dec[16];
  struct Release {
    int lane;
    int64_t at_us;
  };
  std::vector<Release> due, fired;   // the timer's alarms, and when they dropped a pin
  size_t decoded_samples = 0;
  int64_t time_worst = 0;
  size_t ultra_frames = 0, rw_seen = 0;
  bool frame_ok = true;
  struct Quiet {
    int lane;
    int64_t from_s, to_s;
  };
  std::vector<Quiet> quiet;   // each RW frame's window on its lane (samples)

  auto run = [&](int64_t us) {
    const int64_t end = clock_us + us;
    while (clock_us < end) {
      const int64_t k = bus.st.next_chunk - 1;   // the chunk that starts next
      const int64_t switch_us = ceil_us(bus.t0_ns + k * static_cast<int64_t>(bus.chunk_ns()));
      int64_t next = std::min(clock_us + 50, switch_us);
      for (const Release &r : due)
        next = std::min(next, r.at_us);
      for (const Telemetry::Window &w : tel.pending)
        next = std::min(next, w.t0_us >= 0 ? w.t0_us + ULTRA_RW_RELEASE_US : w.end_us - 50);
      set_clock(std::max(next, clock_us));
      if (clock_us >= switch_us) {
        // chunk k starts: it plays (decoded as the boards see it), the one after it is rendered
        bus.play(k);
        for (size_t s = decoded_samples; s < bus.wire.size(); s++) {
          const uint16_t diff = static_cast<uint16_t>(bus.wire[s] ^ (s ? bus.wire[s - 1] : 0));
          for (int l = 0; l < 16; l++) {
            if (diff & (1u << l))
              dec[l].edge(bus.level(static_cast<uint8_t>(l), s), static_cast<int64_t>(s), sample_ns);
          }
        }
        decoded_samples = bus.wire.size();
        for (int l = 0; l < 16; l++) {
          while (dec[l].end(static_cast<int64_t>(decoded_samples), sample_ns)) {
            const Decoder::Frame &f = dec[l].frames.back();
            const int64_t t0_ns = bus.t0_ns + f.t0 * sample_ns;
            if (f.bytes.size() != hubs[l].sched_.frame_bytes())
              continue;
            uint32_t t = 0;
            const uint8_t *h = f.bytes.data() + hubs[l].sched_.header_offset();
            frame_ok &= ultra_parse_header(h, &t) == 0;
            // TIME (low 32 bits of the host us) against the last edge (ns)
            const int64_t d = static_cast<int64_t>(static_cast<int32_t>(t - static_cast<uint32_t>(t0_ns / 1000))) * 1000 -
                              (t0_ns % 1000);
            time_worst = std::max<int64_t>(time_worst, d < 0 ? -d : d);
            ultra_frames++;
            if (ultra_header_flags(h) & ULTRA_HDR_RW) {
              rw_seen++;
              const int64_t t0_us = t0_ns / 1000;
              tel.rw_frame(l, t0_us, f.bytes);
              const uint8_t r = poll_slots(f.bytes.data(), 2);
              quiet.push_back(Quiet{l, f.t0, f.t0 + static_cast<int64_t>((ultra_rw_window_us(r, ULTRA_RW_RATE_1M) +
                                                                                  ULTRA_RW_HOLDOFF_US) * 1000ULL / sample_ns)});
            }
          }
        }
        bus.render(switch_us * 1000, switch_us * 1000 + 40000);
        for (uint8_t i = 0; i < bus.r.releases(); i++) {
          const ParlioRenderer::Release &r = bus.r.release(i);
          due.push_back(Release{r.lane, ceil_us(r.at_ns) + 1 + static_cast<int64_t>(due.size() % 3)});   // + the ISR
        }
      }
      for (size_t i = 0; i < due.size();) {
        if (due[i].at_us <= clock_us) {
          bus.lanes[due[i].lane].release_now();   // the timer's interrupt
          fired.push_back(Release{due[i].lane, clock_us});
          due.erase(due.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
          i++;
        }
      }
      tel.tick();
      for (auto &h : hubs)
        h.loop();
    }
  };

  run(8000000);
  bool all = true, own = true;
  for (int k = 0; k < 16; k++) {
    const TelemetryPoller &p = hubs[k].poller_;
    all &= p.answering(0) && p.answering(1);
    own &= std::fabs(telemetry_value(p.data(0), TV_SUPPLY_VOLTAGE) - (24.0f + 0.5f * k)) < 1e-3f &&
           p.uid16(0) == 0x1000 + 0x10 * k && p.uid16(1) == 0x1001 + 0x10 * k && p.stats().stale == 0;
  }
  int same = 0, across = 0;
  tel.overlaps(&same, &across);
  CHECK(all, "16 lanes: every lane's 2 receivers answer (%d windows answered)", tel.answered);
  CHECK(own, "each lane's poller holds only its own receivers (their voltages, chip ids), nothing stale");
  CHECK(same == 0 && tel.max_enabled <= 1 && tel.collisions == 0, "windows of lanes sharing a UART never overlap "
        "(%d), never two receivers enabled on one UART (at most %d), no garbled answers (%d)", same, tel.max_enabled,
        tel.collisions);
  CHECK(across > 0, "windows of lanes on different UARTs overlap (%d pairs): the 4 groups answer at the same time",
        across);
  // the direction pins: each RW frame's release within 10 us after its last bit
  int64_t worst = 0, least = INT64_MAX;
  size_t matched = 0;
  for (const Telemetry::Window &w : tel.windows) {
    const int64_t t0 = w.t0_us < 0 ? -w.t0_us - 1 : w.t0_us;
    for (const Release &r : fired) {
      if (r.lane == w.line && r.at_us >= t0 - 20 && r.at_us <= t0 + 200) {
        worst = std::max(worst, r.at_us - t0);
        least = std::min(least, r.at_us - t0);
        matched++;
        break;
      }
    }
  }
  CHECK(tel.late == 0 && matched == tel.windows.size() && least >= 1 && worst <= 10, "the direction pin of every RW "
        "frame dropped %lld-%lld us after its last bit (%u of %u), none late", static_cast<long long>(least),
        static_cast<long long>(worst), static_cast<unsigned>(matched), static_cast<unsigned>(tel.windows.size()));
  // each lane quiet for its windows, the others sending
  bool lane_quiet = true;
  size_t others = 0;
  for (const Quiet &q : quiet) {
    for (int64_t s = q.from_s; s < q.to_s && s < static_cast<int64_t>(bus.wire.size()); s++) {
      lane_quiet &= bus.level(static_cast<uint8_t>(q.lane), static_cast<size_t>(s)) == 0;
      others += s > q.from_s && (bus.wire[s] ^ bus.wire[s - 1]) != 0;
    }
  }
  CHECK(lane_quiet && others > 1000, "%u windows: their lane low throughout, the other lanes sending (%u edges)",
        static_cast<unsigned>(quiet.size()), static_cast<unsigned>(others));
  CHECK(frame_ok && time_worst <= 1500 && ultra_frames > 500, "%u Ultra frames decoded from the stream: valid headers, "
        "TIME their last edge within %lld ns", static_cast<unsigned>(ultra_frames), static_cast<long long>(time_worst));
  // lane 5's receivers go silent: only they are unavailable
  tel.on[5] = false;
  run(6000000);
  bool others_ok = true;
  for (int k = 0; k < 16; k++) {
    if (k != 5)
      others_ok &= hubs[k].poller_.answering(0) && hubs[k].poller_.answering(1);
  }
  CHECK(!hubs[5].poller_.answering(0) && !hubs[5].poller_.answering(1) && others_ok,
        "lane 5's receivers silent: only they are unavailable, its UART's 3 other lanes and the rest answer");
  tel.on[5] = true;
  run(6000000);
  CHECK(hubs[5].poller_.answering(0) && hubs[5].poller_.answering(1), "and back");
  tel.overlaps(&same, &across);
  CHECK(same == 0 && tel.max_enabled <= 1, "still no overlap on a UART after it (%d)", same);
  uint32_t refused = 0;
  for (int k = 0; k < 16; k++)
    refused += bus.lanes[k].refused();
  CHECK(refused == 0, "no frame refused (%u)", refused);
  std::printf("  16 lanes, 4 UARTs x 4: %u RW windows (%d pairs overlapping across UARTs, none on one), %u Ultra frames, "
              "direction pins %lld-%lld us after the last bit\n", static_cast<unsigned>(tel.windows.size()), across,
              static_cast<unsigned>(ultra_frames), static_cast<long long>(least), static_cast<long long>(worst));
  UltraHub::hubs_.clear();
}

static esp_err_t rmt_transmit(int ch, int, const uint8_t *data, size_t n, const rmt_transmit_config_t *) {
  Rmt &rmt = rmts[ch];
  UltraHub *h = rmt.hub;
  const uint32_t ticks = h->enc_.symbols.reset.duration0 + h->enc_.symbols.reset.duration1;
  const int64_t bits_us = static_cast<int64_t>(n) * 8 * 1300 / 1000;
  rmt.busy = true;
  rmt.t0_us = clock_us + bits_us;
  rmt.done_us = rmt.t0_us + ticks / 80;
  rmt.frame.assign(data, data + n);
  const uint8_t *hd = data + n - ULTRA_HDR_BYTES;
  rmt.rw_pending = n == h->sched_.frame_bytes() && (ultra_header_flags(hd) & ULTRA_HDR_RW);
  return ESP_OK;
}

/// RMT lines with the same coordinator: 3 lines share one UART (each with a receiver enable pin), a 4th has its own
/// (no receiver enable pin): never 2 receivers enabled on the shared UART, no overlapping windows there, every line's
/// receivers answer with their own values.
static void test_rmt_shared_uart() {
  set_clock(50000000);
  UltraHub::hubs_.clear();
  tel = Telemetry();
  static UltraHub hubs[4];
  static Uart uarts[2];
  static Pin dir[4], re[4];
  for (int k = 0; k < 4; k++) {
    hubs[k] = UltraHub();
    hubs[k].channel_ = k;
    hubs[k].pin_ = static_cast<uint8_t>(16 + k);
    rmts[k] = Rmt();
    rmts[k].hub = &hubs[k];
    const bool shared = k < 3;
    hubs[k].start(nullptr, 250, 1050, 700, 650, 300000, &uarts[shared ? 0 : 1], &dir[k], shared ? &re[k] : nullptr,
                  200 + k);
    tel.uart_of[k] = shared ? 0 : 1;
    tel.re[k] = shared ? &re[k] : nullptr;
    tel.dir[k] = &dir[k];
  }
  tel.uarts[0] = &uarts[0];
  tel.uarts[1] = &uarts[1];
  auto run = [&](int64_t us) {
    const int64_t end = clock_us + us;
    while (clock_us < end) {
      set_clock(clock_us + 20);
      for (int k = 0; k < 4; k++) {
        Rmt &r = rmts[k];
        if (r.rw_pending && clock_us >= r.t0_us) {
          r.rw_pending = false;
          tel.rw_frame(k, r.t0_us, r.frame);
        }
        if (r.busy && clock_us >= r.done_us) {
          r.busy = false;
          UltraHub::rw_done_isr_(nullptr, nullptr, &hubs[k]);   // the RMT's done interrupt
        }
      }
      tel.tick();
      for (auto &h : hubs)
        h.loop();
    }
  };
  run(10000000);
  bool all = true, own = true;
  for (int k = 0; k < 4; k++) {
    const TelemetryPoller &p = hubs[k].poller_;
    all &= p.answering(0) && p.answering(1);
    own &= std::fabs(telemetry_value(p.data(0), TV_SUPPLY_VOLTAGE) - (24.0f + 0.5f * k)) < 1e-3f &&
           p.uid16(1) == 0x1001 + 0x10 * k && p.stats().stale == 0;
  }
  int same = 0, across = 0;
  tel.overlaps(&same, &across);
  CHECK(all && own, "RMT lines: 3 sharing a UART and one on its own, every receiver answers with its own values");
  CHECK(same == 0 && tel.max_enabled <= 1 && tel.collisions == 0 && tel.late == 0, "the shared UART: one window at a "
        "time (%d overlaps), never 2 receivers enabled (%d), nothing garbled (%d)", same, tel.max_enabled,
        tel.collisions);
  CHECK(across > 0, "the line with its own UART answers alongside the shared ones (%d overlaps)", across);
  std::printf("  RMT lines: 3 on one UART (receiver enable pins) + 1 on its own: %u windows, none overlapping on the "
              "shared UART\n", static_cast<unsigned>(tel.windows.size()));
  UltraHub::hubs_.clear();
}

#undef USE_DIG2ANALOG_ULTRA_TELEMETRY
#undef USE_DIG2ANALOG_ULTRA_PARLIO

}  // namespace hub_lanes

// ================================================================================================ 7. update on a lane
// The update task's session on a lane (build/parlio/hub_update_lane.inc): every frame as the session builds it, after
// every page the line low at least its reset + the page pause on the stream, whatever the stream's latency; an abort
// still ends with 12 ABORT frames.
namespace hub_update {

struct UltraHub;
static Stream *bus = nullptr;
static UltraHub *hub = nullptr;
static uint32_t abort_at = UINT32_MAX;   // the abort request once this many frames went out
static uint32_t portTICK_PERIOD_MS = 1;
static void vTaskDelay(uint32_t ticks);

struct UltraHub {
  enum UpdateResult : uint8_t { UPD_NONE = 0, UPD_DONE, UPD_ABORTED, UPD_FAILED };
  uint8_t update_lane_();   // the component's
  ParlioLane *lane_{nullptr};
  uint8_t buf_[64]{};
  uint8_t *tx_buf_{buf_};
  uint32_t reset_ns_{302400};
  UpdateSession upd_session_;
  std::atomic<bool> upd_abort_{false};
  std::atomic<uint32_t> upd_sent_{0};
  std::atomic<uint8_t> upd_pass_{0};
};

/// the task sleeps: the stream plays on (its switches, the renders)
static void vTaskDelay(uint32_t ticks) {
  const int64_t end = clock_us + static_cast<int64_t>(ticks) * portTICK_PERIOD_MS * 1000;
  for (;;) {
    const int64_t k = bus->st.next_chunk - 1;
    const int64_t sw = (bus->t0_ns + k * static_cast<int64_t>(bus->chunk_ns()) + 999) / 1000;
    if (sw > end)
      break;
    set_clock(sw);
    bus->step();
  }
  set_clock(end);
  if (hub->upd_sent_.load() >= abort_at)
    hub->upd_abort_.store(true);
}

#define USE_DIG2ANALOG_ULTRA_UPDATE
#define USE_DIG2ANALOG_ULTRA_PARLIO
#include "hub_update_lane.inc"
#undef USE_DIG2ANALOG_ULTRA_UPDATE
#undef USE_DIG2ANALOG_ULTRA_PARLIO

static void test_update_on_lane() {
  static uint8_t img[UPD_DESC_BYTES + 2000];
  for (size_t i = 0; i < sizeof(img); i++)
    img[i] = static_cast<uint8_t>(i * 7 + 3);
  UpdateSession::Options o;
  o.lead_in_us = 20000;
  o.end_us = 10000;
  const Grid g = grid(50, {std::array<uint32_t, 4>{250, 1050, 700, 650}});
  const uint32_t frame_min_us = (UPD_FRAME_BITS * 1300) / 1000 + 302;
  for (uint32_t tick_ms : {1u, 10u}) {
    portTICK_PERIOD_MS = tick_ms;
    for (int run = 0; run < 2; run++) {
      static Stream s;
      set_clock(9000000);
      s.init(g, 8, 96, 1, 0, clock_us * 1000);
      bus = &s;
      UltraHub h;
      hub = &h;
      abort_at = run == 1 ? 100 : UINT32_MAX;
      h.lane_ = &s.lanes[0];
      h.lane_->begin(UPD_FRAME_BYTES, nullptr, nullptr);
      h.upd_session_.begin(img, sizeof(img), 0x42, o, frame_min_us);
      UpdateSession ref;
      ref.begin(img, sizeof(img), 0x42, o, frame_min_us);
      const uint8_t result = h.update_lane_();
      for (int k = 0; k < 6; k++)
        s.step();   // the last frames on the wire
      // the lane's frames as a board's input sees them
      hub_lanes::Decoder dec;
      for (size_t i = 1; i < s.wire.size(); i++) {
        if ((s.wire[i] ^ s.wire[i - 1]) & 1u)
          dec.edge(s.level(0, i), static_cast<int64_t>(i), g.sample_ns);
        dec.end(static_cast<int64_t>(i), g.sample_ns);
      }
      size_t session = 0, pauses = 0, aborts = 0;
      int64_t least = INT64_MAX;
      bool same = true, paused = true;
      for (size_t i = 0; i < dec.frames.size(); i++) {
        const std::vector<uint8_t> &f = dec.frames[i].bytes;
        same &= f.size() == UPD_FRAME_BYTES && upd_check_frame(f.data()) == 0;
        if (f.size() == UPD_FRAME_BYTES && f[UPD_B_TYPE] == UPD_ABORT) {
          aborts++;
          continue;
        }
        uint8_t r[UPD_FRAME_BYTES];
        uint32_t pause_us = 0;
        same &= ref.next(r, 0, &pause_us) && f.size() == UPD_FRAME_BYTES &&
                std::memcmp(r, f.data(), UPD_BLOCK_BYTES) == 0;
        if (pause_us != 0 && i + 1 < dec.frames.size()) {
          // the low time from its last falling edge to the next frame's first rising edge
          const int64_t low_us = (dec.frames[i + 1].start - dec.frames[i].t0) * g.sample_ns / 1000;
          paused &= low_us >= 300 + static_cast<int64_t>(pause_us);   // its reset and the page pause
          least = std::min(least, low_us);
          pauses++;
        }
        session++;
      }
      CHECK(same && paused, "update on a lane, %u ms tick, run %d: %u frames "
            "as the session built them, after %u pages the line low >= reset + page pause (least %lld us)", tick_ms, run,
            static_cast<unsigned>(session), static_cast<unsigned>(pauses), static_cast<long long>(least));
      if (run == 0) {
        CHECK(result == UltraHub::UPD_DONE && session == ref.total_frames() && pauses == 3 * 8 && aborts == 0,
              "a whole session on a lane: done, %u frames, %u page pauses", static_cast<unsigned>(session),
              static_cast<unsigned>(pauses));
      } else {
        CHECK(result == UltraHub::UPD_ABORTED && session >= 100 && session <= 101 && aborts == 12,
              "an abort after frame 100: the frames up to it, then 12 ABORT frames (%u, %u)",
              static_cast<unsigned>(session), static_cast<unsigned>(aborts));
      }
    }
  }
  portTICK_PERIOD_MS = 1;
  std::printf("  update on a lane: every frame as the session built it, after every page the line low >= its reset + "
              "the page pause (1 kHz and 100 Hz tick)\n");
}

}  // namespace hub_update

// ================================================================================================ the render's cost
/// The render per chunk, built with -O2 and no sanitizers by tests/run_ultra_parlio_test.py: every lane busy with
/// 8-board frames back to back (the worst case), and the same bus idle. Prints ns per chunk and the share of a core.
static int bench() {
  struct Case {
    const char *name;
    uint8_t lanes, width;
    uint32_t sample_ns;
    bool busy;
  };
  const Case cases[] = {
      {"16 lanes, 16-bit, 20 MHz, all busy", 16, 16, 50, true},
      {"8 lanes, 8-bit, 20 MHz, all busy", 8, 8, 50, true},
      {"16 lanes, 16-bit, 10 MHz, all busy", 16, 16, 100, true},
      {"16 lanes, 16-bit, 20 MHz, idle", 16, 16, 50, false},
  };
  for (const Case &c : cases) {
    const Grid g = grid(c.sample_ns, std::vector<std::array<uint32_t, 4>>(c.lanes, std::array<uint32_t, 4>{250, 1050, 700, 650}));
    const uint32_t groups = c.sample_ns == 50 ? 96 : 96;   // ~1 ms at 20 MHz, ~0.54 ms at 10 MHz... (whole groups)
    ParlioClock clock;
    clock.configure(c.sample_ns, g.period, groups);
    ParlioLane lanes[PARLIO_MAX_LANES];
    ParlioLane *ls[PARLIO_MAX_LANES];
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
      lanes[k].attach(&clock, k);
      ls[k] = &lanes[k];
    }
    ParlioRenderer r;
    r.init(c.width, g.period, groups, g.lane, c.lanes, 0);
    std::vector<uint8_t> buf[2];
    for (int i = 0; i < 2; i++) {
      buf[i].assign(r.chunk_bytes(), 0);
      r.clear(buf[i].data(), static_cast<uint8_t>(i));
    }
    ParlioClock::State st{};
    st.next_chunk = 2;
    st.running = true;
    clock.write(st);
    std::vector<uint8_t> frame(ultra_frame_bytes(8));   // 8 boards
    std::mt19937 rng(1);
    for (auto &b : frame)
      b = static_cast<uint8_t>(rng());
    for (uint8_t k = 0; k < c.lanes; k++)
      lanes[k].begin(frame.size(), nullptr, nullptr);
    const int chunks = 20000;
    int64_t ns = 0;
    uint64_t sink = 0;
    for (int i = 0; i < chunks; i++) {
      if (c.busy) {
        for (uint8_t k = 0; k < c.lanes; k++) {
          if (!lanes[k].busy()) {
            lanes[k].reserve_us();
            lanes[k].post(frame.data(), frame.size(), 300, false);
          }
        }
      }
      const int64_t ch = st.next_chunk;
      st.next_chunk = ch + 1;
      clock.write(st);
      const auto a = std::chrono::steady_clock::now();
      r.render(ch, static_cast<uint8_t>(ch & 1), buf[ch & 1].data(), ls, &clock, st);
      ns += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - a).count();
      sink += buf[ch & 1][static_cast<size_t>(i) % buf[0].size()];
    }
    const double per_chunk = static_cast<double>(ns) / chunks;
    const double chunk_ns = static_cast<double>(clock.chunk_ns());
    const double frame_ns = (ultra_frame_bytes(8) * 8.0 * g.period * c.sample_ns) + 302400.0;   // a frame and its reset
    std::printf("bench %-38s: %7.0f ns per chunk of %4.0f us (%u B): %5.2f %% of a host core; per 8-board frame cycle "
                "(%.2f ms) %.1f us%s\n", c.name, per_chunk, chunk_ns / 1000.0, static_cast<unsigned>(r.chunk_bytes()),
                100.0 * per_chunk / chunk_ns, frame_ns / 1e6, per_chunk * frame_ns / chunk_ns / 1000.0,
                sink == 1 ? " " : "");
  }
  return 0;
}
}  // namespace esphome::dig2analog_ultra

int main(int argc, char **argv) {
  using namespace esphome::dig2analog_ultra;
  if (argc > 1 && std::strcmp(argv[1], "--bench") == 0)
    return bench();
  test_waveform();
  test_time();
  test_windows();
  test_slips();
  hub_lanes::test_sixteen_lanes();
  hub_lanes::test_rmt_shared_uart();
  hub_update::test_update_on_lane();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA PARLIO TESTS PASSED", failures);
  return failures ? 1 : 0;
}
