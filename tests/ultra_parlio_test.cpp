// Host test of the PARLIO lanes (dig2analog_ultra_parlio on an ESP32-P4: ultra_parlio.h) and of the telemetry
// turn-taking for lines sharing a UART. The bus' DMA, render task and timer are modelled the way
// components/dig2analog_ultra_parlio/ultra_parlio_bus.cpp drives ParlioStream (the DMA plays the newest buffer handed
// over in time, the same one again when none came, nothing once the timer killed the stream); the hub's own loop(),
// transmit_(), window, start and update code are cut unchanged from dig2analog_ultra.cpp by
// tests/run_ultra_parlio_test.py.
//  1. waveform: every lane, sample for sample, is what the RMT encoder (encode_symbols) sends with the lane's timing
//     on the sample grid (16 and 8-bit samples, 20 and 10 MHz, inverted lanes, lanes with their own high times); the
//     lanes independent of each other; the rounding within the Physical Layer;
//  2. TIME: every frame's header TIME is its last falling edge on the stream, on every lane;
//  3. windows: an RW frame's lane stays quiet for its window while the others send; the direction pin drops within
//     10 us after the RW frame's last bit, never before it;
//  4. a late render: the stream is killed before the DMA could play a buffer twice; every frame with a valid header on
//     the wire is exactly a frame a hub posted, once, with its real TIME (a repeat would splice valid slots into the
//     wrong boards' positions); the stream restarts and frames flow again;
//  5. a late switch interrupt changes nothing; a kill cancels the releases of the frames it cuts, never moves one;
//  6. 16 lanes on the hub's own code, 4 UARTs shared by 4 lanes each (also through a kill): windows of one UART never
//     overlap, windows of different UARTs do, every receiver answers with its own data, a silent lane leaves the
//     others answering; RMT lines sharing a UART; a busy line can't starve the others; a poll that failed to go out
//     doesn't keep the UART through an update; a group start on a lane with long buffers covers its command cadence;
//  7. a firmware update session on a lane: every page pause on the wire, expired reservations, refused frames, a
//     stalled stream with and without an abort, the last frame out before it returns.
// Build: python tests/run_ultra_parlio_test.py (also its negative controls and the render benchmark)

#include "../components/dig2analog_ultra/ultra_core.h"
#include "../components/dig2analog_ultra/ultra_parlio.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"
#include "../components/dig2analog_ultra/update_session.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <new>
#include <random>
#include <stdexcept>
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
/// a fresh object in place (the stream and the hubs hold atomics: not assignable)
template<typename T> static void renew(T &x) {
  x.~T();
  new (&x) T();
}

// ================================================================================================ the grid
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
static Grid default_grid(uint8_t lanes, uint32_t sample_ns = 50) {
  return grid(sample_ns, std::vector<std::array<uint32_t, 4>>(lanes, std::array<uint32_t, 4>{250, 1050, 700, 650}));
}
/// as parlio_timing.py margin_ns(): two DMA descriptors' worth of samples and the slack
static uint32_t margin_ns(uint32_t sample_ns, uint8_t width) {
  return static_cast<uint32_t>(2ull * 4095 * 8 * sample_ns / width + 50000);
}

// ================================================================================================ the bus, modelled
/// The bus as ultra_parlio_bus.cpp drives ParlioStream (times in host ns):
///  - the DMA plays chunk after chunk; at each boundary it takes the newest buffer handed over by the stream's margin
///    before it (its last descriptors), else it plays the same buffer again (loop transmission) - unless the timer
///    killed the stream: then the pins are idle;
///  - at each buffer switch the switch interrupt (isr_ns late) wakes the task, which renders the next chunk and hands
///    it over task_ns later; late(k) delays it further at the switch to chunk k;
///  - the timer: at the stream's next alarm (+ timer_ns): the releases, the deadline; after a kill the task restarts
///    the stream restart_ns later.
/// wire: the pins, sample by sample (sample s at wire_t0 + s x sample).
struct Stream {
  ParlioStream s;
  uint8_t width{16};
  uint16_t invert{0};
  uint32_t sample_ns{50};
  uint32_t margin{0};
  std::vector<uint8_t> buf[2];
  std::vector<uint16_t> wire;
  int64_t wire_t0{0};
  int64_t isr_ns{2000}, task_ns{50000}, timer_ns{2000}, restart_ns{300000};
  std::function<int64_t(int64_t)> late;
  struct Fired {
    uint8_t lane;
    int64_t at;
  };
  std::vector<Fired> fired;   // the releases, when the timer dropped a pin
  int64_t replays{0}, restarts{0};
  // state
  int64_t T{0};            // the chunk on the wire started
  int64_t on_wire{0};      // its index (its buffer: on_wire & 1)
  int64_t handed{0}, handed_at{0};
  bool need_render{false}, need_hand{false}, dead{false};
  int64_t render_at{0}, hand_at{0}, rendered{0}, cut{0}, last_event{0};

  bool init(const Grid &g, uint8_t w, uint32_t chunk_groups, uint8_t lanes_used, uint16_t inv, int64_t start_ns) {
    this->sample_ns = g.sample_ns;
    this->width = w;
    this->invert = inv;
    this->margin = margin_ns(g.sample_ns, w);
    if (!this->s.init(g.lane, lanes_used, inv, g.sample_ns, w, g.period, chunk_groups, this->margin))
      return false;
    for (auto &b : this->buf)
      b.assign(this->s.chunk_bytes(), 0xEE);
    this->wire.clear();
    this->fired.clear();
    this->wire_t0 = start_ns;
    this->last_event = start_ns;
    this->start_at(start_ns);
    return true;
  }
  int64_t C() const { return static_cast<int64_t>(this->s.chunk_ns()); }
  size_t chunk_samples() const { return this->s.renderer().chunk_samples(); }
  int64_t wire_end() const { return this->wire_t0 + static_cast<int64_t>(this->wire.size()) * this->sample_ns; }
  void start_at(int64_t t) {
    const int64_t first = this->s.next_chunk();
    for (int i = 0; i < 2; i++)
      this->s.clear(this->buf[i].data(), static_cast<uint8_t>(i));
    this->s.start(first, t);
    this->T = t;
    this->on_wire = first;
    this->handed = first + 1;
    this->handed_at = t;
    this->need_render = this->need_hand = this->dead = false;
  }
  void append(int64_t c, size_t n) {
    const uint8_t *b = this->buf[c & 1].data();
    for (size_t i = 0; i < n; i++)
      this->wire.push_back(this->width == 8 ? b[i] : static_cast<uint16_t>(b[2 * i] | b[2 * i + 1] << 8));
  }
  void idle_until(int64_t t) {
    while (this->wire_end() < t)
      this->wire.push_back(this->invert);
  }
  /// every event up to t_ns
  void run_until(int64_t t_ns) {
    const int64_t INF = INT64_MAX;
    for (;;) {
      const int64_t e_render = !this->dead && this->need_render ? this->render_at : INF;
      const int64_t e_hand = !this->dead && this->need_hand ? this->hand_at : INF;
      const int64_t alarm = this->s.next_alarm_us();
      const int64_t e_alarm = alarm != 0 ? std::max(alarm * 1000 + this->timer_ns, this->last_event) : INF;
      const int64_t e_end = this->dead ? INF : this->T + this->C();
      const int64_t e_restart = this->dead ? this->cut + this->restart_ns : INF;
      const int64_t e = std::min({e_render, e_hand, e_alarm, e_end, e_restart});
      if (e == INF || e > t_ns)
        break;
      this->last_event = e;
      set_clock(e / 1000);   // (the hubs' release callbacks read it)
      if (e == e_render) {
        void *bufs[2] = {this->buf[0].data(), this->buf[1].data()};
        this->rendered = this->s.render(bufs);
        this->need_render = false;
        this->need_hand = true;
        this->hand_at = e + this->task_ns;
      } else if (e == e_hand) {
        this->need_hand = false;
        if (!this->s.killed()) {   // (the bus: under its lock, after the transmit)
          this->s.handed_over(this->rendered);
          this->handed = this->rendered;
          this->handed_at = e;
        }
      } else if (e == e_alarm) {
        bool kill = false;
        const uint16_t mask = this->s.alarm(e / 1000, &kill);
        for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
          if (mask & (1u << k)) {
            this->s.lane(k)->release_now();
            this->fired.push_back(Fired{k, e});
          }
        }
        if (kill && !this->dead) {   // the pins to their idle level now: the chunk on the wire is cut here
          this->dead = true;
          this->cut = e;
          this->append(this->on_wire, static_cast<size_t>((e - this->T) / this->sample_ns));
        }
      } else if (e == e_end) {
        this->append(this->on_wire, this->chunk_samples());
        // the DMA moves on to the newest buffer, if it came in time; else it plays the same one again
        if (this->handed > this->on_wire && this->handed_at <= this->T + this->C() - this->margin) {
          this->on_wire = this->handed;
          this->need_render = true;
          const int64_t extra = this->late ? this->late(this->on_wire) : 0;
          this->render_at = this->T + this->C() + this->isr_ns + extra;
        } else {
          this->replays++;
        }
        this->T += this->C();
      } else {   // the task restarts the stream
        this->idle_until(e);
        this->s.drop_all();
        this->restarts++;
        this->start_at(this->wire_end());
      }
    }
  }
  /// up to the end of the chunk on the wire (or the restart after a kill)
  void step() { this->run_until(this->dead ? this->cut + this->restart_ns : this->T + this->C()); }
  /// a lane's level at a sample (its idle level 0)
  uint8_t level(uint8_t lane, size_t s) const {
    return static_cast<uint8_t>(((this->wire[s] ^ this->invert) >> lane) & 1u);
  }
  int64_t time_of(int64_t sample) const { return this->wire_t0 + sample * this->sample_ns; }
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
  bool release{false};
  bool refused{false};
};

/// The decoder of one lane's wire, as a board's input sees it: high pulses are bits (>= 472 ns a '1'), a low of 50 us
/// ends the frame (its last falling edge: t0).
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
  /// every frame of a lane of the whole wire
  static Decoder of(const Stream &s, uint8_t lane) {
    Decoder d;
    for (size_t i = 1; i < s.wire.size(); i++) {
      if (((s.wire[i] ^ s.wire[i - 1]) >> lane) & 1u)
        d.edge(s.level(lane, i), static_cast<int64_t>(i), s.sample_ns);
      if (!d.bits.empty())
        d.end(static_cast<int64_t>(i), s.sample_ns);
    }
    d.end(static_cast<int64_t>(s.wire.size()) + 2000000, s.sample_ns);
    return d;
  }
};

/// a frame with a valid Ultra header (what the boards act on)
static bool valid_ultra(const std::vector<uint8_t> &f) {
  uint32_t t = 0;
  return f.size() >= ULTRA_HDR_BYTES + ULTRA_SLOT_BYTES && (f.size() - ULTRA_HDR_BYTES) % ULTRA_SLOT_BYTES == 0 &&
         ultra_parse_header(f.data() + f.size() - ULTRA_HDR_BYTES, &t) == 0;
}

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
    const uint32_t chunk_groups = c.sample_ns == 50 ? 96 : (c.width == 8 ? 120 : 92);
    CHECK(s.init(g, c.width, chunk_groups, c.lanes, c.invert, 1000000000) && s.s.renderer().grid() == c.grid,
          "%s: the renderer takes the timing (path %u)", c.name, s.s.renderer().grid());
    std::mt19937 rng(7 + c.lanes);
    std::vector<std::vector<Posted>> posted(c.lanes);
    std::vector<std::vector<uint8_t>> frame(c.lanes);
    for (int sw = 0; sw < 300; sw++) {
      for (uint8_t l = 0; l < c.lanes; l++) {
        ParlioLane &lane = *s.s.lane(l);
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
          const uint32_t q = static_cast<uint32_t>((quiet_us * 1000ULL + s.s.clock().group_ns() - 1) / s.s.clock().group_ns());
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
      CHECK(s.s.lane(l)->refused() == 0, "%s: lane %u refused nothing (%u)", c.name, l, s.s.lane(l)->refused());
    }
    CHECK(all && apart && frames > 100 && s.replays == 0 && s.restarts == 0, "%s: %u frames, every lane sample for "
          "sample the RMT encoder's waveform at the lane's grid timing, frames apart by their quiet", c.name,
          static_cast<unsigned>(frames));
  }

  // one lane's waveform doesn't depend on the others: the same frames on lane 2 among silent and among busy lanes
  const Grid g = default_grid(16);
  std::vector<uint8_t> lane2[2];
  for (int busy = 0; busy < 2; busy++) {
    Stream s;
    s.init(g, 16, 96, 16, 0, 0);
    std::mt19937 rng(99), noise(5);
    std::vector<uint8_t> f(200), other[16];
    for (int sw = 0; sw < 60; sw++) {
      for (uint8_t l = 0; l < 16; l++) {
        ParlioLane &lane = *s.s.lane(l);
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
  const Grid g = default_grid(16);
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
    s.s.lane(l)->begin(buf[l].size(), nullptr, nullptr);
  }
  for (int sw = 0; sw < 400; sw++) {
    set_clock(s.T / 1000 + 100);
    for (uint8_t l = 0; l < 16; l++) {
      ParlioLane &lane = *s.s.lane(l);
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
      const int64_t err = p.time_full * 1000 - s.time_of(static_cast<int64_t>(last));
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
/// lane is low for its whole window while the others have edges in it; the timer (an interrupt 2 us late) drops the
/// direction pin within 10 us after the RW frame's last falling edge, never before it.
static void test_windows() {
  const Grid g = default_grid(16);
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
    s.s.lane(l)->begin(300, [](void *arg) { (*static_cast<int *>(arg))++; }, &released);
  for (int sw = 0; sw < 300; sw++) {
    for (uint8_t l = 0; l < 16; l++) {
      ParlioLane &lane = *s.s.lane(l);
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
  CHECK(s.fired.size() == static_cast<size_t>(released) && s.fired.size() + 1 >= rw.size() && s.fired.size() <= rw.size(),
        "a release per RW frame on the wire (%u / %u)", static_cast<unsigned>(s.fired.size()), static_cast<unsigned>(rw.size()));
  for (size_t i = 0; i < rw.size() && i < s.fired.size(); i++) {
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
    order &= s.fired[i].lane == rw_lane;
    const int64_t d = s.fired[i].at - s.time_of(static_cast<int64_t>(last));
    worst = std::max(worst, d);
    least = std::min(least, d);
    windows++;
  }
  CHECK(quiet && others && windows > 20, "%u RW windows: lane 6 low for W + hold-off (%u us) after each, the other 15 "
        "lanes send meanwhile (%u edges)", static_cast<unsigned>(windows), window, static_cast<unsigned>(other_edges));
  CHECK(order && least >= static_cast<int64_t>(PARLIO_RELEASE_NS) && worst <= 10000, "the direction pin drops %.1f-%.1f "
        "us after the RW frame's last falling edge (within 10 us, never before it)", least / 1000.0, worst / 1000.0);
  std::printf("  windows: lane 6 quiet through %u windows while 15 lanes send; direction pin %.1f-%.1f us after the last "
              "bit\n", static_cast<unsigned>(windows), least / 1000.0, worst / 1000.0);
}

// ================================================================================================ 4. a late render
/// Real Ultra frames on 16 lanes posted as hubs post them (8-board frames that start on chunk boundaries and span three
/// chunks on 4 lanes; 1-30 boards on the others, RW frames with releases on 4), the render task
/// late now and then: by less than its margin (nothing happens), by more (the timer kills the stream at the deadline,
/// before the DMA could play its buffer twice). Every frame on the wire with a valid header must be exactly a frame a
/// hub posted, once, with its TIME its real last edge: a buffer played twice would splice whole, valid slots into the
/// frames on the wire, for the wrong boards, or replay a frame with a stale TIME. The stream restarts, frames flow.
/// Returns the frames decoded valid.
struct LaneRun {
  Scheduler sched;
  std::vector<uint8_t> buf;
  std::vector<Posted> posted;
};
static size_t run_lateness(Stream &s, LaneRun *lr, const Grid &g, int switches, std::function<bool(uint8_t)> rw_lane,
                           const char *what) {
  const uint32_t window = ultra_rw_window_us(2, ULTRA_RW_RATE_1M) + ULTRA_RW_HOLDOFF_US;
  for (int sw = 0; sw < switches; sw++) {
    set_clock(std::max<int64_t>(clock_us, s.T / 1000 + 100));
    for (uint8_t l = 0; l < 16; l++) {
      ParlioLane &lane = *s.s.lane(l);
      const bool busy = lane.busy();
      if (!busy && lane.take_refused() && !lr[l].posted.empty())
        lr[l].posted.back().refused = true;   // it never went out (the stream restarted before)
      if (busy || !lane.ready())
        continue;
      const uint16_t m = static_cast<uint16_t>(1000 * (sw % 60)), a[CHANNELS] = {1, 2, 3, 4, 5};
      const uint16_t b[CHANNELS] = {65535, 65535, 65535, 65535, 65535};
      for (uint16_t board = 0; board < lr[l].sched.num_boards(); board++)
        lr[l].sched.light_set(board, 0, 0, static_cast<uint16_t>(m + board), a, b, 300, clock_us + 30000);
      const int64_t start = lane.reserve_us();
      const int64_t t = lr[l].sched.build_frame(start, lr[l].buf.data());
      const bool rw = rw_lane(l);
      if (lane.post(lr[l].buf.data(), lr[l].buf.size(), rw ? window : 303, rw))
        lr[l].posted.push_back(Posted{lane.reserved_group(), lr[l].buf, 0, 0, t, rw});
    }
    s.step();
  }
  for (int sw = 0; sw < 8; sw++)
    s.step();
  // every decoded frame with a valid header: exactly one posted frame, once, its TIME its last edge
  size_t valid = 0, matched_twice = 0, unmatched = 0, cut = 0;
  int64_t worst = 0;
  for (uint8_t l = 0; l < 16; l++) {
    const Decoder d = Decoder::of(s, l);
    std::vector<int> seen(lr[l].posted.size(), 0);
    for (const Decoder::Frame &f : d.frames) {
      if (!valid_ultra(f.bytes)) {
        cut++;
        continue;
      }
      valid++;
      int hit = -1;
      for (size_t i = 0; i < lr[l].posted.size() && hit < 0; i++) {
        if (!lr[l].posted[i].refused && lr[l].posted[i].bytes == f.bytes)
          hit = static_cast<int>(i);
      }
      if (hit < 0) {
        unmatched++;
        if (unmatched <= 3)
          std::printf("  %s: lane %u: a valid frame of %u bytes no hub posted (at %.3f ms)\n", what, l,
                      static_cast<unsigned>(f.bytes.size()), (s.time_of(f.t0) - s.wire_t0) / 1e6);
        continue;
      }
      matched_twice += seen[hit]++ > 0;
      const int64_t err = lr[l].posted[hit].time_full * 1000 - s.time_of(f.t0);
      worst = std::max<int64_t>(worst, err < 0 ? -err : err);
    }
  }
  CHECK(unmatched == 0 && matched_twice == 0 && worst <= 1500, "%s: %u valid frames on the wire, each exactly a posted "
        "frame, once (%u not, %u twice), TIME its last edge within %lld ns; %u cut frames (the boards drop them)", what,
        static_cast<unsigned>(valid), static_cast<unsigned>(unmatched), static_cast<unsigned>(matched_twice),
        static_cast<long long>(worst), static_cast<unsigned>(cut));
  return valid;
}

static void lanes_for_lateness(Stream &s, LaneRun *lr, const Grid &g) {
  for (uint8_t l = 0; l < 16; l++) {
    const uint16_t boards = static_cast<uint16_t>(l < 4 ? 8 : 1 + (l * 7) % 30);
    lr[l].sched.init(boards, 40 + l, 3);
    lr[l].sched.set_bit_timing(250, (g.period - 5) * 50, 700, (g.period - 14) * 50);
    lr[l].buf.assign(lr[l].sched.frame_bytes(), 0);
    s.s.lane(l)->begin(lr[l].buf.size(), [](void *) {}, nullptr);
  }
}

static void test_late_render() {
  const Grid g = default_grid(16);
  {
    Stream s;
    s.init(g, 16, 96, 16, 0, 3000000000);
    static LaneRun lr[16];
    for (auto &x : lr)
      x = LaneRun();
    lanes_for_lateness(s, lr, g);
    // late by 0.5 / 0.7 of a chunk (the handover still before the deadline), by 1.3 / 0.9 / 3 (a kill)
    s.late = [](int64_t k) -> int64_t {
      switch (k % 97) {
        case 20: return 500000;
        case 35: return 680000;
        case 50: return 1350000;
        case 70: return 900000;
        case 90: return 3100000;
        default: return 0;
      }
    };
    const size_t valid = run_lateness(s, lr, g, 600, [](uint8_t l) { return l >= 12; }, "late renders");
    uint32_t refused = 0, lost = 0;
    for (uint8_t l = 0; l < 16; l++) {
      refused += s.s.lane(l)->refused();
      lost += s.s.lane(l)->lost();
    }
    CHECK(s.replays == 0 && s.restarts >= 15 && s.s.kills() == static_cast<uint32_t>(s.restarts) && valid > 1500,
          "late renders: the stream killed before any buffer played twice (%lld repeats), %lld restarts, frames flow "
          "after them (%u valid)", static_cast<long long>(s.replays), static_cast<long long>(s.restarts),
          static_cast<unsigned>(valid));
    std::printf("  late render: %lld kills before a repeat, %u valid frames (each a posted one, once), %u refused and "
                "%u cut frames counted as lost\n", static_cast<long long>(s.restarts), static_cast<unsigned>(valid),
                refused, lost);
  }
  // 8-board frames across chunk boundaries, one render late: the middle chunk would play twice
  {
    Stream s;
    s.init(g, 16, 96, 16, 0, 4000000000);
    static LaneRun lr[16];
    for (auto &x : lr)
      x = LaneRun();
    lanes_for_lateness(s, lr, g);
    s.late = [](int64_t k) -> int64_t { return k == 40 ? 1200000 : 0; };
    run_lateness(s, lr, g, 80, [](uint8_t) { return false; }, "one late render mid-frame");
    CHECK(s.replays == 0 && s.restarts == 1, "one late render: one kill, no repeat (%lld, %lld)",
          static_cast<long long>(s.restarts), static_cast<long long>(s.replays));
  }
}

// ================================================================================================ 5. late interrupts
/// The switch interrupt 700 us late (but the task's handover still in time): no kill, no refusal, every frame's TIME
/// right, every release on time: no time is taken from an interrupt's timestamp. Then late renders with RW frames in
/// flight: every release that fires belongs to an RW frame on the wire, 2-10 us after its last edge; the releases of
/// the frames a kill cut never fire (a release is never moved).
static void test_late_interrupts() {
  const Grid g = default_grid(16);
  {
    Stream s;
    s.init(g, 16, 96, 16, 0, 6000000000);
    s.isr_ns = 700000;
    s.task_ns = 30000;
    static LaneRun lr[16];
    for (auto &x : lr)
      x = LaneRun();
    lanes_for_lateness(s, lr, g);
    const size_t valid = run_lateness(s, lr, g, 300, [](uint8_t l) { return l % 3 == 0; }, "a late switch interrupt");
    uint32_t refused = 0;
    for (uint8_t l = 0; l < 16; l++)
      refused += s.s.lane(l)->refused();
    CHECK(s.restarts == 0 && s.replays == 0 && refused == 0 && valid > 1000, "the switch interrupt 700 us late: no "
          "kill, no repeat, nothing refused (%lld, %lld, %u)", static_cast<long long>(s.restarts),
          static_cast<long long>(s.replays), refused);
  }
  {
    Stream s;
    s.init(g, 16, 96, 16, 0, 7000000000);
    s.late = [](int64_t k) -> int64_t { return k % 23 == 11 ? 1100000 : 0; };
    static LaneRun lr[16];
    for (auto &x : lr)
      x = LaneRun();
    lanes_for_lateness(s, lr, g);
    run_lateness(s, lr, g, 400, [](uint8_t) { return true; }, "late renders, RW frames on every lane");
    // each release: an RW frame on the wire on its lane, 2-10 us before
    size_t matched = 0, stray = 0;
    int64_t worst = 0, least = INT64_MAX;
    std::vector<Decoder> dec;
    for (uint8_t l = 0; l < 16; l++)
      dec.push_back(Decoder::of(s, l));
    for (const Stream::Fired &f : s.fired) {
      bool hit = false;
      for (const Decoder::Frame &fr : dec[f.lane].frames) {
        const int64_t d = f.at - s.time_of(fr.t0);
        if (valid_ultra(fr.bytes) && d >= 0 && d <= 10000) {
          hit = true;
          worst = std::max(worst, d);
          least = std::min(least, d);
          break;
        }
      }
      matched += hit;
      stray += !hit;
    }
    CHECK(stray == 0 && matched > 500 && least >= 2000 && worst <= 10000 && s.restarts >= 10,
          "through %lld kills: %u releases, each 2-10 us after an RW frame's last edge on its lane (%.1f-%.1f us), %u "
          "for frames that didn't complete", static_cast<long long>(s.restarts), static_cast<unsigned>(matched),
          least / 1000.0, worst / 1000.0, static_cast<unsigned>(stray));
    std::printf("  late interrupts: a 700 us late switch changes nothing; through %lld kills %u releases, all on time, "
                "none for a cut frame\n", static_cast<long long>(s.restarts), static_cast<unsigned>(matched));
  }
}

//BUS_TESTS_END
}  // namespace esphome::dig2analog_ultra

#include "ultra_parlio_test_hubs.inc"

namespace esphome::dig2analog_ultra {
// ================================================================================================ the render's cost
/// The render per chunk, built with -O2 and no sanitizers by tests/run_ultra_parlio_test.py: every lane busy with
/// 8-board frames back to back (the worst case), and the same bus idle. Prints ns per chunk and the share of a core.
static int bench() {
  std::printf("bench sizes: a lane %u B (+ its frame's copy), the stream %u B, + %u B per group of a chunk "
              "(96 groups: %u B)\n", static_cast<unsigned>(sizeof(ParlioLane)), static_cast<unsigned>(sizeof(ParlioStream)),
              20u + 2u, 96u * 22u);
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
    const Grid g = default_grid(c.lanes, c.sample_ns);
    ParlioStream st;
    st.init(g.lane, c.lanes, 0, c.sample_ns, c.width, g.period, 96, margin_ns(c.sample_ns, c.width));
    std::vector<uint8_t> buf[2];
    for (int i = 0; i < 2; i++) {
      buf[i].assign(st.chunk_bytes(), 0);
      st.clear(buf[i].data(), static_cast<uint8_t>(i));
    }
    st.start(0, 0);
    void *bufs[2] = {buf[0].data(), buf[1].data()};
    std::vector<uint8_t> frame(ultra_frame_bytes(8));   // 8 boards
    std::mt19937 rng(1);
    for (auto &b : frame)
      b = static_cast<uint8_t>(rng());
    for (uint8_t k = 0; k < c.lanes; k++)
      st.lane(k)->begin(frame.size(), nullptr, nullptr);
    const int chunks = 20000;
    int64_t ns = 0;
    uint64_t sink = 0;
    for (int i = 0; i < chunks; i++) {
      if (c.busy) {
        for (uint8_t k = 0; k < c.lanes; k++) {
          if (!st.lane(k)->busy()) {
            st.lane(k)->reserve_us();
            st.lane(k)->post(frame.data(), frame.size(), 300, false);
          }
        }
      }
      const auto a = std::chrono::steady_clock::now();
      const int64_t ch = st.render(bufs);
      st.handed_over(ch);
      ns += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - a).count();
      sink += buf[ch & 1][static_cast<size_t>(i) % buf[0].size()];
    }
    const double per_chunk = static_cast<double>(ns) / chunks;
    const double chunk_ns = static_cast<double>(st.chunk_ns());
    const double frame_ns = (ultra_frame_bytes(8) * 8.0 * g.period * c.sample_ns) + 302400.0;   // a frame and its reset
    std::printf("bench %-38s: %7.0f ns per chunk of %4.0f us (%u B): %5.2f %% of a host core; per 8-board frame cycle "
                "(%.2f ms) %.1f us%s\n", c.name, per_chunk, chunk_ns / 1000.0, static_cast<unsigned>(st.chunk_bytes()),
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
  test_late_render();
  test_late_interrupts();
  hub_lanes::test_sixteen_lanes();
  hub_lanes::test_rmt_shared_uart();
  hub_lanes::test_fair_uart();
  hub_lanes::test_failed_poll_then_update();
  hub_lanes::test_group_cadence();
  hub_update::test_update_on_lane();
  hub_update::test_update_failures();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA PARLIO TESTS PASSED", failures);
  return failures ? 1 : 0;
}
