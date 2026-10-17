#pragma once

// PARLIO lanes (ESP32-P4: one parallel-output DMA unit drives up to 16 data lines of the QuinLED Ultra protocol;
// dig2analog_ultra_parlio): each data line is a lane, its waveform one bit of every sample. Without ESP-IDF / ESPHome dependencies
// (tests/ultra_parlio_test.cpp runs it on a PC):
//  - the bit grid: every bit of every lane takes one period (the bus' longest bit: a shorter bit's low time is
//    stretched to it), a lane's '0' / '1' high time is its own. 8 bits are a group; a frame starts on a group, so a
//    group carries one byte of every lane that is in a frame;
//  - ParlioClock: where the stream is (a group's start on the host clock, the chunk the renderer takes frames for);
//  - ParlioLane: a hub's mailbox (one frame at a time, its start reserved ahead of the renderer) and the renderer's
//    copy of the frame on the wire;
//  - ParlioRenderer: one chunk of samples from the lanes' frames, bit-sliced (a byte of every lane per group);
//  - ParlioStream: what the bus does with them: the chunks in turn, the direction pins' releases, and the render
//    deadline that kills the stream before the DMA could play a buffer twice.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#ifndef ULTRA_ALWAYS_INLINE
#if defined(__GNUC__)
#define ULTRA_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define ULTRA_ALWAYS_INLINE inline
#endif
#endif

namespace esphome::dig2analog_ultra {

static constexpr uint8_t PARLIO_MAX_LANES = 16;
/// The direction pin drops this long after an RW frame's last bit (an alarm from the stream's timing; the RMT's lines:
/// their done interrupt after the same LOW, RW_RELEASE_HOLD_NS): the protocol's ULTRA_RW_RELEASE_MIN_US (60) + 20 us. The pair then floats HIGH;
/// a receiver that closes a frame by its LOW run needs the LOW.
static constexpr uint32_t PARLIO_RELEASE_NS = 80000;
/// A (re)start's time is known within this (the first transmit call's duration): a slower one is tried again.
static constexpr uint32_t PARLIO_START_SKEW_NS = 50000;

/// A lane's bit timing in samples: the high time of a '0' and of a '1' (the low time fills the bus' period).
struct ParlioLaneTiming {
  uint16_t h0{0}, h1{0};
};

/// Byte -> 0 / 1 per bit, MSB (the bit that goes first) in byte 0 of w[0], bit 3 in byte 0 of w[1]: shifted by a
/// lane's index within its 8, each byte of the sum is then one bit time's lanes.
struct ParlioSpread {
  uint32_t w[256][2];
  constexpr ParlioSpread() : w{} {
    for (int v = 0; v < 256; v++) {
      for (int j = 0; j < 8; j++) {
        if (v & (0x80 >> j))
          this->w[v][j >> 2] |= 1u << (8 * (j & 3));
      }
    }
  }
};
inline constexpr ParlioSpread PARLIO_SPREAD{};

/// Where the stream is. Written by the renderer only; the hubs read it (a sequence lock: the main loop and an update
/// task read, the render task writes).
class ParlioClock {
 public:
  struct State {
    int64_t next_chunk{0};  // the renderer takes the lanes' frames for this chunk next (earlier chunks are fixed)
    int64_t t0_ns{0};       // the host time of group 0's first sample (ns): chunk k plays from t0 + k x chunk
    uint32_t epoch{0};      // +1 at every (re)start: frames reserved before it are refused
    bool running{false};
  };
  void configure(uint32_t sample_ns, uint16_t period, uint32_t chunk_groups) {
    this->sample_ns_ = sample_ns;
    this->group_samples_ = 8u * period;
    this->group_ns_ = sample_ns * 8u * period;
    this->chunk_groups_ = chunk_groups;
  }
  uint32_t sample_ns() const { return this->sample_ns_; }
  uint32_t group_ns() const { return this->group_ns_; }
  uint32_t group_samples() const { return this->group_samples_; }
  uint32_t chunk_groups() const { return this->chunk_groups_; }
  uint64_t chunk_ns() const { return static_cast<uint64_t>(this->group_ns_) * this->chunk_groups_; }

  State read() const {
    for (;;) {
      const uint32_t q = this->seq_.load(std::memory_order_acquire);
      if (q & 1u)
        continue;
      const State s = this->state_;
      std::atomic_thread_fence(std::memory_order_acquire);
      if (this->seq_.load(std::memory_order_relaxed) == q)
        return s;
    }
  }
  void write(const State &s) {
    const uint32_t q = this->seq_.load(std::memory_order_relaxed);
    this->seq_.store(q + 1u, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    this->state_ = s;
    this->seq_.store(q + 2u, std::memory_order_release);
  }
  /// The host time of a sample (ns) and of a group's first sample (us, the nearest: a frame's TIME, its start + its
  /// bits rounded, is then within 1 us of its last edge).
  int64_t sample_time_ns(const State &s, int64_t sample) const {
    return s.t0_ns + sample * static_cast<int64_t>(this->sample_ns_);
  }
  int64_t group_us(const State &s, int64_t group) const {
    return floor_div_(s.t0_ns + group * static_cast<int64_t>(this->group_ns_) + 500, 1000);
  }
  static int64_t floor_div_(int64_t a, int64_t b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

  /// (the stream's stop, under the bus' lock; also the timer's interrupt: inline) epoch ended with its pins idle by
  /// cut_us. The last STOPS stops are kept.
  ULTRA_ALWAYS_INLINE void note_stop(uint32_t epoch, uint32_t cut_us) {
    const uint32_t n = this->stops_.load(std::memory_order_relaxed);
    this->stop_epoch_[n % STOPS] = epoch;
    this->stop_cut_[n % STOPS] = cut_us;
    this->stops_.store(n + 1, std::memory_order_release);
  }
  /// Whether a frame of `epoch` whose last bit is out at end_us (host us, low 32 bits) was cut: 1 the stop of its
  /// epoch came before that (or that stop is no longer known: too many since), 0 not (no stop, or one after it).
  int cut_in(uint32_t epoch, uint32_t end_us) const {
    for (;;) {
      const uint32_t n = this->stops_.load(std::memory_order_acquire);
      int cut = this->read().epoch == epoch ? 0 : 1;   // (an earlier epoch ended with a stop: unknown unless kept)
      for (uint32_t i = 0; i < STOPS && i < n; i++) {
        const uint32_t k = (n - 1 - i) % STOPS;
        if (this->stop_epoch_[k] == epoch) {
          cut = static_cast<int32_t>(this->stop_cut_[k] - end_us) < 0 ? 1 : 0;
          break;
        }
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      if (this->stops_.load(std::memory_order_relaxed) == n)
        return cut;
    }
  }

 protected:
  static constexpr uint32_t STOPS = 4;
  std::atomic<uint32_t> seq_{0};
  State state_{};
  uint32_t stop_epoch_[STOPS]{}, stop_cut_[STOPS]{};
  std::atomic<uint32_t> stops_{0};
  uint32_t sample_ns_{50}, group_samples_{216}, group_ns_{10800}, chunk_groups_{96};
};

/// One data line of the bus. The hub posts one frame at a time, at a start it reserved from the stream's clock
/// (after the lane's last frame and its quiet, and a chunk ahead of the renderer); the renderer copies it when it
/// reaches the chunk it starts in, and the hub may build the next one.
class ParlioLane {
 public:
  enum : uint8_t { FREE = 0, POSTED, REFUSED };
  /// what became of a post (outcome())
  enum Outcome : uint8_t { PENDING = 0, ON_WIRE, FAILED };
  using ReleaseFn = void (*)(void *arg);

  // ---- the bus (configuration)
  void attach(ParlioClock *clock, uint8_t index) {
    this->clock_ = clock;
    this->index_ = index;
  }
  uint8_t index() const { return this->index_; }

  // ---- the hub (its main loop, or its update task)
  /// max_bytes: the hub's largest frame (the renderer keeps a copy of the one on the wire); release: drops the hub's
  /// direction pin (an interrupt, after an RW frame's last bit). false: no bus, or no memory.
  bool begin(size_t max_bytes, ReleaseFn release, void *arg) {
    if (this->clock_ == nullptr || max_bytes == 0)
      return false;
    this->copy_.reset(new uint8_t[max_bytes]);
    this->max_bytes_ = max_bytes;
    this->release_fn_ = release;
    this->release_arg_ = arg;
    return true;
  }
  /// the stream runs and the lane has its copy buffer
  bool ready() const { return this->copy_ && this->clock_ != nullptr && this->clock_->read().running; }
  /// A posted frame still waits for the renderer. A frame it refused (the stream restarted, or its start passed) is
  /// counted and forgotten, like a frame lost on the wire; take_refused() tells whether the last one was.
  bool busy() {
    const uint8_t s = this->state_.load(std::memory_order_acquire);
    if (s == REFUSED) {
      this->refused_++;
      this->last_refused_ = true;
      this->state_.store(FREE, std::memory_order_relaxed);
      return false;
    }
    return s == POSTED;
  }
  /// The last posted frame was refused (after busy() returned false): it never went out.
  bool take_refused() {
    const bool r = this->last_refused_;
    this->last_refused_ = false;
    return r;
  }
  /// The host time (us) the next frame starts at: after the lane's last frame and its quiet, and a chunk beyond the
  /// one the renderer takes frames for next (so a post() within a chunk's time is never late).
  int64_t reserve_us() {
    const ParlioClock::State c = this->clock_->read();
    const int64_t horizon = (c.next_chunk + 1) * static_cast<int64_t>(this->clock_->chunk_groups());
    this->start_ = this->free_ > horizon ? this->free_ : horizon;
    this->epoch_ = c.epoch;
    this->clock_state_ = c;
    return this->clock_->group_us(c, this->start_);
  }
  /// The frame built for reserve_us()'s start, then quiet_us of low (at least a group); release: the direction pin
  /// drops after its last bit. false: the start passed meanwhile, or the stream slipped (reserve again).
  bool post(const uint8_t *frame, size_t bytes, uint32_t quiet_us, bool release) {
    const ParlioClock::State c = this->clock_->read();
    if (bytes == 0 || bytes > this->max_bytes_ || c.epoch != this->epoch_ ||
        this->start_ < c.next_chunk * static_cast<int64_t>(this->clock_->chunk_groups()))
      return false;
    const uint64_t g = this->clock_->group_ns();
    this->p_frame_ = frame;
    this->p_bytes_ = static_cast<uint32_t>(bytes);
    this->p_start_ = this->start_;
    this->p_quiet_ = static_cast<uint32_t>((static_cast<uint64_t>(quiet_us) * 1000u + g - 1) / g);
    if (this->p_quiet_ == 0)
      this->p_quiet_ = 1;
    this->p_release_ = release;
    this->p_epoch_ = this->epoch_;
    this->last_refused_ = false;
    this->end_us_ = this->clock_->group_us(this->clock_state_, this->start_ + static_cast<int64_t>(bytes) + this->p_quiet_);
    this->frame_end_us_ = this->clock_->group_us(this->clock_state_, this->start_ + static_cast<int64_t>(bytes));
    this->p_id_ = ++this->post_seq_;
    this->state_.store(POSTED, std::memory_order_release);
    return true;
  }
  /// When the last posted frame and its quiet are over on the wire (host us; a restart ends them sooner).
  int64_t end_us() const { return this->end_us_; }
  /// The last post: its id and epoch (outcome()) and when its last bit is out (host us).
  uint32_t posted_id() const { return this->p_id_; }
  uint32_t posted_epoch() const { return this->p_epoch_; }
  /// the last post the renderer took (tests, diagnostics)
  uint32_t taken_id() const { return this->taken_id_.load(std::memory_order_acquire); }
  int64_t frame_end_us() const { return this->frame_end_us_; }
  /// What became of post `id` (of `epoch`, its last bit out at end_us) by now_us: refused (the stream restarted before
  /// it went out, or its start passed), or cut by the stop of its epoch (before its last bit, whatever the renderer
  /// took after it): FAILED; taken by the renderer and its last bit out, no stop before: ON_WIRE (complete, as the
  /// boards see it); else PENDING. (A refusal is seen at the hub's next loop, before its next post of the lane.)
  uint8_t outcome(uint32_t id, uint32_t epoch, int64_t end_us, int64_t now_us) const {
    if (id == 0)
      return PENDING;
    if (this->refused_id_.load(std::memory_order_acquire) == id)
      return FAILED;
    if (this->clock_->cut_in(epoch, static_cast<uint32_t>(end_us)) != 0)
      return FAILED;
    const uint32_t taken = this->taken_id_.load(std::memory_order_acquire);
    if (static_cast<int32_t>(taken - id) >= 0 && now_us >= end_us)
      return ON_WIRE;
    return PENDING;
  }
  /// How far ahead of now a frame starts at most: the chunk on the wire, the one rendered after it, the margin chunk,
  /// and a group.
  uint32_t lead_us() const {
    return this->clock_ == nullptr ? 0
                                   : static_cast<uint32_t>((3 * this->clock_->chunk_ns() + this->clock_->group_ns()) / 1000 + 1);
  }
  /// the start group reserve_us() chose (tests, diagnostics)
  int64_t reserved_group() const { return this->start_; }
  /// frames refused before they went out, and frames a restart cut on the wire (the boards drop those)
  uint32_t refused() const { return this->refused_; }
  uint32_t lost() const { return this->lost_; }

  // ---- the bus' timer (an interrupt, also with the cache off): the RW frame's last bit is out, the hub lets go
  ULTRA_ALWAYS_INLINE void release_now() const {
    if (this->release_fn_ != nullptr)
      this->release_fn_(this->release_arg_);
  }

 protected:
  friend class ParlioRenderer;
  friend class ParlioStream;
  ParlioClock *clock_{nullptr};
  uint8_t index_{0};
  // the hub's
  int64_t start_{0};
  uint32_t epoch_{0};
  ParlioClock::State clock_state_{};   // as reserve_us() read it
  int64_t end_us_{0};
  int64_t frame_end_us_{0};
  uint32_t post_seq_{0};
  uint32_t refused_{0};
  bool last_refused_{false};
  size_t max_bytes_{0};
  // posted (the hub writes them, then POSTED; the renderer reads them after it)
  std::atomic<uint8_t> state_{FREE};
  const uint8_t *p_frame_{nullptr};
  uint32_t p_bytes_{0};
  int64_t p_start_{0};
  uint32_t p_quiet_{0};
  uint32_t p_epoch_{0};
  bool p_release_{false};
  uint32_t p_id_{0};
  // what became of the posts (outcome(); the stops: ParlioClock::cut_in())
  std::atomic<uint32_t> taken_id_{0}, refused_id_{0};
  // the renderer's (free_: the hub reads it once the frame was taken)
  int64_t free_{0};            // the first group after the last frame taken and its quiet
  std::unique_ptr<uint8_t[]> copy_;
  int64_t cur_start_{0};       // the frame on the wire (copy_): its first group, its length
  uint32_t cur_bytes_{0};
  uint32_t lost_{0};           // frames a restart cut
  ReleaseFn release_fn_{nullptr};
  void *release_arg_{nullptr};
};

#if defined(__GNUC__) && !defined(__clang__)
#define PARLIO_HOT __attribute__((optimize("O2")))   // the render loop, also in a build for size (ESP-IDF's -Os)
#else
#define PARLIO_HOT
#endif

/// A grid every lane shares, unrolled at compile time (the default timing: 20 MHz 5 / 14 / 27 samples, 10 MHz
/// 3 / 7 / 14): of a group's samples only the 32-bit words that hold a high sample are stored, each once, as a
/// combination of the group's A (the lanes in a frame) and its bit time's B (the lanes sending a '1'); the words of
/// the low tails never change. A word never holds the highs of two bit times (P - H1 >= samples per word).
template<typename T, int H0, int H1, int P> struct ParlioGrid {
  static constexpr int SPW = 4 / static_cast<int>(sizeof(T));
  static constexpr int BITS = 8 * static_cast<int>(sizeof(T));
  static_assert(H0 > 0 && H0 < H1 && P - H1 >= SPW, "a word holds the highs of one bit time");
  /// sample `pos` of a bit time: 0 low, 1 the '0' high (every lane in a frame), 2 the '1' high
  static constexpr int code(int pos) { return pos < 0 || pos >= H1 ? 0 : pos < H0 ? 1 : 2; }
  template<int C> static ULTRA_ALWAYS_INLINE uint32_t sel(uint32_t z, uint32_t a, uint32_t b) {
    return C == 0 ? z : C == 1 ? a : b;
  }
  template<int J, int W, int... K>
  static ULTRA_ALWAYS_INLINE void word(uint32_t *o, uint32_t z, uint32_t a, uint32_t b,
                                       std::integer_sequence<int, K...>) {
    o[W] = (0u | ... | (sel<code(W * SPW + K - J * P)>(z, a, b) << (K * BITS)));
  }
  template<int J, int... I>
  static ULTRA_ALWAYS_INLINE void bit(uint32_t *o, uint32_t z, uint32_t a, uint32_t b, std::integer_sequence<int, I...>) {
    (word<J, (J * P) / SPW + I>(o, z, a, b, std::make_integer_sequence<int, SPW>{}), ...);
  }
  /// bit time J of a group: the words from its first sample to its last high one
  template<int J> static ULTRA_ALWAYS_INLINE void bit(uint32_t *o, uint32_t z, uint32_t a, const uint32_t *w) {
    uint32_t ones = (w[J >> 2] >> (8 * (J & 3))) & 0xFFu;
    if constexpr (sizeof(T) == 2)
      ones |= ((w[2 + (J >> 2)] >> (8 * (J & 3))) & 0xFFu) << 8;
    bit<J>(o, z, a, ones ^ z, std::make_integer_sequence<int, (J * P + H1 - 1) / SPW - (J * P) / SPW + 1>{});
  }
  /// one group (o: its first word): active the lanes in a frame, w their bits (the accumulator), inv the idle levels
  static ULTRA_ALWAYS_INLINE void group(uint32_t *o, uint32_t active, const uint32_t *w, uint32_t inv) {
    const uint32_t a = active ^ inv;
    bit<0>(o, inv, a, w);
    bit<1>(o, inv, a, w);
    bit<2>(o, inv, a, w);
    bit<3>(o, inv, a, w);
    bit<4>(o, inv, a, w);
    bit<5>(o, inv, a, w);
    bit<6>(o, inv, a, w);
    bit<7>(o, inv, a, w);
  }
};

/// Renders the stream in chunks of whole groups: the lanes' bytes bit-sliced into a byte of 8 bit times per group,
/// then each bit time written as runs of one sample word (all lanes in a frame high, then those sending a '1', then
/// low). Runs that are low on every lane are written once (clear()) and never again. The default grids take an
/// unrolled path (ParlioGrid), any other the runs.
class ParlioRenderer {
 public:
  struct Release {
    uint8_t lane;
    int64_t at_ns;    // host time
    int64_t end_ns;   // the frame's last falling edge
  };

  /// width: 8 or 16 bits per sample; period: samples per bit; timing: the lanes' high times; invert: lanes whose pin
  /// idles high. false: a timing that doesn't fit (a high time of 0, or one not shorter than the period).
  bool init(uint8_t width, uint16_t period, uint32_t chunk_groups, const ParlioLaneTiming *timing, uint8_t lanes,
            uint16_t invert) {
    this->width_ = width == 8 ? 8 : 16;
    this->period_ = period;
    this->chunk_groups_ = chunk_groups;
    this->lanes_ = lanes > PARLIO_MAX_LANES ? PARLIO_MAX_LANES : lanes;
    this->invert_ = static_cast<uint16_t>(invert & (this->width_ == 8 ? 0xFFu : 0xFFFFu));
    std::memcpy(this->timing_, timing, sizeof(ParlioLaneTiming) * this->lanes_);
    // the runs of one bit time: where any lane's '0' or '1' ends, the lanes still high change
    this->runs_ = 0;
    uint16_t s = 0;
    while (s < period) {
      uint16_t m0 = 0, m1 = 0, e = period;
      for (uint8_t l = 0; l < this->lanes_; l++) {
        const ParlioLaneTiming &t = this->timing_[l];
        if (t.h0 == 0 && t.h1 == 0)
          continue;   // no hub on this lane: it stays idle
        if (t.h0 == 0 || t.h0 >= t.h1 || t.h1 >= period)
          return false;
        if (s < t.h0)
          m0 |= static_cast<uint16_t>(1u << l);
        if (s < t.h1)
          m1 |= static_cast<uint16_t>(1u << l);
        for (uint16_t edge : {t.h0, t.h1}) {
          if (edge > s && edge < e)
            e = edge;
        }
      }
      if (m0 != 0 || m1 != 0) {
        if (this->runs_ >= MAX_RUNS)
          return false;
        this->run_[this->runs_++] = Run{s, static_cast<uint16_t>(e - s), m0, m1};
      }
      s = e;
    }
    // every lane on the default grid: the unrolled path
    this->grid_ = 0;
    int h0 = -1, h1 = -1;
    bool same = true;
    for (uint8_t l = 0; l < this->lanes_; l++) {
      const ParlioLaneTiming &t = this->timing_[l];
      if (t.h0 == 0 && t.h1 == 0)
        continue;
      same &= (h0 < 0 || (h0 == t.h0 && h1 == t.h1));
      h0 = t.h0;
      h1 = t.h1;
    }
    if (same && h0 == 5 && h1 == 14 && period == 27)
      this->grid_ = 1;
    else if (same && h0 == 3 && h1 == 7 && period == 14)
      this->grid_ = 2;
#ifdef PARLIO_ONLY_GRID
    if (this->grid_ != PARLIO_ONLY_GRID)
      return false;   // (the code generation built another path)
#endif
#ifdef PARLIO_ONLY_WIDTH
    if (this->width_ != PARLIO_ONLY_WIDTH)
      return false;
#endif
    this->acc_.reset(new Acc[chunk_groups]);
    this->dirty_[0].reset(new uint8_t[chunk_groups]());
    this->dirty_[1].reset(new uint8_t[chunk_groups]());
    return true;
  }
  uint32_t group_samples() const { return 8u * this->period_; }
  size_t chunk_samples() const { return static_cast<size_t>(this->chunk_groups_) * this->group_samples(); }
  size_t chunk_bytes() const { return this->chunk_samples() * (this->width_ / 8u); }

  /// A DMA buffer before its first use: every lane at its idle level.
  void clear(void *out, uint8_t buffer) {
    if (this->width_ == 8)
      std::memset(out, this->invert_ & 0xFF, this->chunk_samples());
    else
      std::fill_n(static_cast<uint16_t *>(out), this->chunk_samples(), this->invert_);
    std::memset(this->dirty_[buffer & 1].get(), 0, this->chunk_groups_);
  }

  /// Chunk `chunk` into out (DMA buffer `buffer`, 0 / 1): the frames on the wire, and the posted frames that start in
  /// it (copied: the hub may build its next one), or refused (their start passed, or the stream slipped since they
  /// were reserved). releases(): the direction pins to drop for the RW frames taken here.
  PARLIO_HOT void render(int64_t chunk, uint8_t buffer, void *out, ParlioLane *const *lanes, const ParlioClock *clock,
                         const ParlioClock::State &clk) {
    const int64_t g0 = chunk * static_cast<int64_t>(this->chunk_groups_);
    const int64_t g1 = g0 + this->chunk_groups_;
    std::memset(this->acc_.get(), 0, sizeof(Acc) * this->chunk_groups_);
    this->releases_ = 0;
    for (uint8_t l = 0; l < this->lanes_; l++) {
      ParlioLane *lane = lanes[l];
      if (lane == nullptr)
        continue;   // (a lane whose hub hasn't begun yet: nothing on the wire, nothing posted)
      // the frame on the wire: its part in this chunk
      if (lane->cur_bytes_ != 0) {
        const int64_t end = lane->cur_start_ + lane->cur_bytes_;
        this->emit_(l, lane, g0, g1);
        if (end <= g1)
          lane->cur_bytes_ = 0;
      }
      // a posted frame that starts here (the one on the wire ended before it: in lane order its part is done)
      if (lane->state_.load(std::memory_order_acquire) != ParlioLane::POSTED)
        continue;
      if (lane->p_epoch_ != clk.epoch || lane->p_start_ < g0 || lane->p_start_ < lane->free_) {
        lane->refused_id_.store(lane->p_id_, std::memory_order_release);
        lane->state_.store(ParlioLane::REFUSED, std::memory_order_release);
        continue;
      }
      if (lane->p_start_ >= g1)
        continue;   // a later chunk (the frame on the wire may still be in this one: free_ is after it)
      std::memcpy(lane->copy_.get(), lane->p_frame_, lane->p_bytes_);
      lane->cur_start_ = lane->p_start_;
      lane->cur_bytes_ = lane->p_bytes_;
      lane->free_ = lane->p_start_ + lane->p_bytes_ + lane->p_quiet_;
      lane->taken_id_.store(lane->p_id_, std::memory_order_release);
      if (lane->p_release_ && this->releases_ < PARLIO_MAX_LANES) {
        // the last bit's falling edge: its period's start + its high time
        const ParlioLaneTiming &t = this->timing_[l];
        const int64_t bit_start = (lane->cur_start_ + lane->cur_bytes_) * this->group_samples() - this->period_;
        const uint16_t high = (lane->copy_[lane->cur_bytes_ - 1] & 1u) ? t.h1 : t.h0;
        const int64_t end_ns = clock->sample_time_ns(clk, bit_start + high);
        this->release_[this->releases_++] = Release{l, end_ns + PARLIO_RELEASE_NS, end_ns};
      }
      lane->state_.store(ParlioLane::FREE, std::memory_order_release);
      this->emit_(l, lane, g0, g1);
      if (lane->cur_start_ + lane->cur_bytes_ <= g1)
        lane->cur_bytes_ = 0;
    }
    // (the bus' code generation names its width and grid, so only that path is built: PARLIO_ONLY_WIDTH / _GRID)
#if !defined(PARLIO_ONLY_WIDTH) || PARLIO_ONLY_WIDTH == 8
    if (this->width_ == 8)
      this->expand_any_(static_cast<uint8_t *>(out), buffer);
#endif
#if !defined(PARLIO_ONLY_WIDTH) || PARLIO_ONLY_WIDTH == 16
    if (this->width_ == 16)
      this->expand_any_(static_cast<uint16_t *>(out), buffer);
#endif
  }
  /// the unrolled path's grid (0: the runs)
  uint8_t grid() const { return this->grid_; }
  uint8_t releases() const { return this->releases_; }
  const Release &release(uint8_t i) const { return this->release_[i]; }

 protected:
  static constexpr uint8_t MAX_RUNS = 2 * PARLIO_MAX_LANES + 1;
  struct Run {
    uint16_t start, len;
    uint16_t m0, m1;   // lanes high here when sending a '0' / a '1'
  };
  struct Acc {
    uint32_t w[4];     // lanes 0-7: w[0] (bits 0-3 of the group) and w[1] (bits 4-7); lanes 8-15: w[2], w[3]
    uint16_t active;   // lanes in a frame
  };

  /// The lane's frame bytes in groups [g0, g1) of this chunk into the accumulators.
  void emit_(uint8_t l, const ParlioLane *lane, int64_t g0, int64_t g1) {
    const int64_t from = lane->cur_start_ > g0 ? lane->cur_start_ : g0;
    const int64_t end = lane->cur_start_ + lane->cur_bytes_;
    const int64_t to = end < g1 ? end : g1;
    if (from >= to)
      return;
    const uint8_t *src = lane->copy_.get() + (from - lane->cur_start_);
    Acc *a = this->acc_.get() + (from - g0);
    const uint32_t sh = l & 7u, k = (l >> 3) * 2u, n = static_cast<uint32_t>(to - from);
    const uint16_t bit = static_cast<uint16_t>(1u << l);
    for (uint32_t i = 0; i < n; i++) {
      const uint32_t *s = PARLIO_SPREAD.w[src[i]];
      a[i].w[k] |= s[0] << sh;
      a[i].w[k + 1] |= s[1] << sh;
      a[i].active |= bit;
    }
  }

  /// A run of one sample word: word stores where it can (the buffers are word aligned; a run of 5 or 9 samples).
  static void fill_(uint16_t *p, uint16_t v, uint32_t n) {
    if ((reinterpret_cast<uintptr_t>(p) & 2u) != 0 && n != 0) {
      *p++ = v;
      n--;
    }
    auto *q = reinterpret_cast<uint32_t *>(p);
    const uint32_t w = v | static_cast<uint32_t>(v) << 16;
    for (; n >= 2; n -= 2)
      *q++ = w;
    if (n != 0)
      *reinterpret_cast<uint16_t *>(q) = v;
  }
  static void fill_(uint8_t *p, uint8_t v, uint32_t n) {
    for (; (reinterpret_cast<uintptr_t>(p) & 3u) != 0 && n != 0; n--)
      *p++ = v;
    auto *q = reinterpret_cast<uint32_t *>(p);
    const uint32_t w = v * 0x01010101u;
    for (; n >= 4; n -= 4)
      *q++ = w;
    for (p = reinterpret_cast<uint8_t *>(q); n != 0; n--)
      *p++ = v;
  }

  template<typename T> void expand_any_(T *o, uint8_t buffer) {
#if !defined(PARLIO_ONLY_GRID) || PARLIO_ONLY_GRID == 1
    if (this->grid_ == 1)
      this->expand_grid_<T, 5, 14, 27>(o, buffer);
#endif
#if !defined(PARLIO_ONLY_GRID) || PARLIO_ONLY_GRID == 2
    if (this->grid_ == 2)
      this->expand_grid_<T, 3, 7, 14>(o, buffer);
#endif
#if !defined(PARLIO_ONLY_GRID) || PARLIO_ONLY_GRID == 0
    if (this->grid_ == 0)
      this->expand_(o, buffer);
#endif
  }

  /// The accumulators into samples on a default grid (ParlioGrid).
  template<typename T, int H0, int H1, int P> PARLIO_HOT void expand_grid_(T *out, uint8_t buffer) {
    uint8_t *dirty = this->dirty_[buffer & 1].get();
    constexpr uint32_t words = 8 * P * sizeof(T) / 4;   // per group
    auto *o = reinterpret_cast<uint32_t *>(out);
    const uint32_t inv = this->invert_;
    for (uint32_t gi = 0; gi < this->chunk_groups_; gi++, o += words) {
      const Acc &a = this->acc_[gi];
      if (a.active == 0 && !dirty[gi])
        continue;   // idle now and last time: every sample already at the idle level
      dirty[gi] = a.active != 0;
      ParlioGrid<T, H0, H1, P>::group(o, a.active, a.w, inv);
    }
  }

  /// The accumulators into samples: per group 8 bit times, each the runs of its sample words.
  template<typename T> PARLIO_HOT void expand_(T *out, uint8_t buffer) {
    uint8_t *dirty = this->dirty_[buffer & 1].get();
    const uint32_t gs = this->group_samples();
    const T inv = static_cast<T>(this->invert_);
    for (uint32_t gi = 0; gi < this->chunk_groups_; gi++) {
      const Acc &a = this->acc_[gi];
      T *o = out + gi * gs;
      if (a.active == 0) {
        if (!dirty[gi])
          continue;   // idle now and last time: every sample already at the idle level
        dirty[gi] = 0;
        for (uint32_t j = 0; j < 8; j++, o += this->period_) {
          for (uint8_t r = 0; r < this->runs_; r++)
            fill_(o + this->run_[r].start, inv, this->run_[r].len);
        }
        continue;
      }
      dirty[gi] = 1;
      const uint32_t active = a.active;
      for (uint32_t j = 0; j < 8; j++, o += this->period_) {
        const uint32_t sh = 8u * (j & 3u), w = j >> 2;
        const uint32_t ones = ((a.w[w] >> sh) & 0xFFu) | (((a.w[2 + w] >> sh) & 0xFFu) << 8);
        const uint32_t zeros = active & ~ones;
        for (uint8_t r = 0; r < this->runs_; r++) {
          const Run &run = this->run_[r];
          fill_(o + run.start, static_cast<T>(((zeros & run.m0) | (ones & run.m1)) ^ inv), run.len);
        }
      }
    }
  }

  uint8_t width_{16};
  uint16_t period_{27};
  uint32_t chunk_groups_{0};
  uint8_t lanes_{0};
  uint16_t invert_{0};
  ParlioLaneTiming timing_[PARLIO_MAX_LANES]{};
  Run run_[MAX_RUNS]{};
  uint8_t runs_{0};
  uint8_t grid_{0};   // 1: 5 / 14 / 27, 2: 3 / 7 / 14 on every lane (ParlioGrid), 0: the runs
  std::unique_ptr<Acc[]> acc_;
  std::unique_ptr<uint8_t[]> dirty_[2];   // per DMA buffer: the groups that carried a frame when last rendered
  Release release_[PARLIO_MAX_LANES]{};
  uint8_t releases_{0};
};

/// The stream as the bus (ultra_parlio_bus.cpp) drives it, and the tests: the clock, the renderer, the lanes, the
/// direction pins' releases and the render deadline.
///  - The DMA plays the newest buffer handed over, again and again while no newer one is there. A buffer played twice
///    would splice its slots into the frames on the wire: whole slots, valid, for the wrong boards. So the chunk after
///    the one playing must be with the DMA a margin before that one ends (its last descriptors, and slack), and both
///    the task and the timer hold to it: the task's handover counts only if it is done before the deadline (checked
///    under the bus' lock, after the transmit call returned), and at the deadline the timer stops a stream whose next
///    chunk isn't there: the bus drives the pins to their idle level, every frame on the wire is cut (the boards drop
///    a cut frame). The replay can't start before the playing chunk ends, a margin after the deadline: the timer's
///    interrupt has that long. The task then restarts the stream: idle chunks first, every frame on the wire dropped,
///    every reservation refused (a new epoch).
///  - No time is taken from an interrupt's timestamp: chunk k plays from t0 + k x chunk, exactly, from a (re)start
///    until a stop. A start's first sample goes out within the first transmit call: its time is known between the
///    host clock before that call and after it. The deadlines count from the earlier (never late), the frames' TIMEs
///    and releases from the later (a release never before its frame's last bit); a start whose call took longer than
///    PARLIO_START_SKEW_NS is tried again.
///  - A direction pin's release is set when its RW frame's chunk is handed over and never moves. A stop fires the
///    releases of the frames that were out by then and cancels those of the frames it cut.
/// The task's calls and the timer's (an interrupt that also runs with the cache off) share the releases and the
/// deadline: the bus calls handed_over(), stop(), next_alarm_us(), late() and releases_due() under its lock.
class ParlioStream {
 public:
  bool init(const ParlioLaneTiming *timing, uint8_t lanes, uint16_t invert, uint32_t sample_ns, uint8_t width,
            uint16_t period, uint32_t chunk_groups, uint32_t margin_ns) {
    this->clock_.configure(sample_ns, period, chunk_groups);
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++)
      this->lanes_[k].attach(&this->clock_, k);
    this->margin_ns_ = margin_ns;
    return this->r_.init(width, period, chunk_groups, timing, lanes, invert);
  }
  ParlioClock &clock() { return this->clock_; }
  ULTRA_ALWAYS_INLINE ParlioLane *lane(uint8_t k) { return &this->lanes_[k < PARLIO_MAX_LANES ? k : 0]; }
  const ParlioRenderer &renderer() const { return this->r_; }
  size_t chunk_bytes() const { return this->r_.chunk_bytes(); }
  uint64_t chunk_ns() const { return this->clock_.chunk_ns(); }
  /// a DMA buffer at the idle level, before it is first played (also at a restart)
  void clear(void *buf, uint8_t buffer) { this->r_.clear(buf, buffer); }

  // ---- the task
  /// The stream (re)starts: chunk `first` (idle) began between lo_ns and hi_ns (the host clock before and after the
  /// first transmit call); chunk first + 1 (idle) was handed over at second_us. false: not started (the bus stops the
  /// DMA and tries again): the start's time is too uncertain (unless any_skew: the last try), or the second chunk
  /// came too late for its deadline.
  bool start(int64_t first, int64_t lo_ns, int64_t hi_ns, int64_t second_us, bool any_skew = false) {
    const int64_t skew = hi_ns > lo_ns ? hi_ns - lo_ns : 0;
    if (skew > PARLIO_START_SKEW_NS && !any_skew)
      return false;
    // chunk first + 1 must have been with the DMA a margin before chunk first ends (counted from the earliest start)
    const int64_t c = static_cast<int64_t>(this->chunk_ns());
    if (second_us >= ParlioClock::floor_div_(hi_ns - skew + c - static_cast<int64_t>(this->margin_ns_), 1000))
      return false;
    this->skew_ns_ = skew;
    this->st_.t0_ns = hi_ns - first * c;
    this->st_.next_chunk = first + 2;
    this->st_.epoch++;
    this->st_.running = true;
    this->clock_.write(this->st_);
    for (int64_t &at : this->release_us_)
      at = 0;
    this->deadline_us_ = this->deadline_of_(first + 2);
    this->killed_.store(false, std::memory_order_release);
    return true;
  }
  /// the chunk the next render() renders (a restart's first chunk)
  int64_t next_chunk() const { return this->st_.next_chunk; }
  /// At a buffer switch: the chunk after the one that just started, into bufs[chunk & 1]. Returns its index.
  int64_t render(void *const *bufs) {
    const int64_t c = this->st_.next_chunk;
    this->st_.next_chunk = c + 1;
    this->clock_.write(this->st_);   // hubs reserve from the chunk after it on
    ParlioLane *ls[PARLIO_MAX_LANES];
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++)
      ls[k] = &this->lanes_[k];
    this->r_.render(c, static_cast<uint8_t>(c & 1), bufs[c & 1], ls, &this->clock_, this->st_);
    // its releases in us, here (no division in the interrupt)
    this->pending_ = this->r_.releases();
    for (uint8_t i = 0; i < this->pending_; i++) {
      const ParlioRenderer::Release &r = this->r_.release(i);
      // (its end as early as it can be: the stream may have started up to skew before t0; a release for a frame the
      // stop cut is harmless, one missing for a complete frame leaves the line driven through its window)
      this->pending_at_[i] = Pending{r.lane, (r.at_ns + 999) / 1000,
                                     ParlioClock::floor_div_(r.end_ns - this->skew_ns_, 1000)};
    }
    return c;
  }
  /// (under the bus' lock, the transmit call returned at now_us) chunk c is with the DMA: in time only before its
  /// deadline and with the stream alive. Then its releases are armed and the deadline is the next chunk's; false: too
  /// late (the bus stops the stream: stop()) or already stopped.
  bool handed_over(int64_t c, int64_t now_us) {
    if (this->killed() || this->deadline_us_ == 0 || now_us >= this->deadline_us_)
      return false;
    for (uint8_t i = 0; i < this->pending_; i++) {
      this->release_us_[this->pending_at_[i].lane] = this->pending_at_[i].at_us;
      this->release_end_us_[this->pending_at_[i].lane] = this->pending_at_[i].end_us;
    }
    this->pending_ = 0;
    this->deadline_us_ = this->deadline_of_(c + 1);
    return true;
  }
  /// After a stop: the frames on the wire dropped (each lane counts them), the renderer forgets them; the bus then
  /// clears both buffers, restarts the DMA and calls start() (a new epoch: every reservation refused).
  void drop_all() {
    for (ParlioLane &l : this->lanes_) {
      if (l.cur_bytes_ != 0)
        l.lost_++;
      l.cur_bytes_ = 0;
    }
    this->pending_ = 0;
    this->st_.running = false;
    this->clock_.write(this->st_);
  }
  bool killed() const { return this->killed_.load(std::memory_order_acquire); }
  uint32_t kills() const { return this->kills_; }
  int64_t deadline_us() const { return this->deadline_us_; }

  // ---- the timer and the task (under the bus' lock; also in an interrupt with the cache off: inline, no division)
  /// the next alarm (host us): the earliest release, or the deadline; 0 = none
  ULTRA_ALWAYS_INLINE int64_t next_alarm_us() const {
    int64_t first = this->deadline_us_;
    for (int64_t at : this->release_us_) {
      if (at != 0 && (first == 0 || at < first))
        first = at;
    }
    return first;
  }
  /// The deadline passed without the next chunk: the stream must stop (the bus drives the pins idle, then stop()).
  ULTRA_ALWAYS_INLINE bool late(int64_t now_us) const { return this->deadline_us_ != 0 && now_us >= this->deadline_us_; }
  /// At now_us: the lanes whose RW frame ended (the bus releases their direction pins).
  ULTRA_ALWAYS_INLINE uint16_t releases_due(int64_t now_us) {
    uint16_t mask = 0;
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
      const int64_t at = this->release_us_[k];
      if (at != 0 && at <= now_us + 1) {
        mask |= static_cast<uint16_t>(1u << k);
        this->release_us_[k] = 0;
      }
    }
    return mask;
  }
  /// The stream stops: the pins went to their idle level by cut_us. The frames whose last bit can have been out by
  /// then are complete: their releases now (the returned lanes); the others were cut: no release. The stop is noted
  /// with its epoch (ParlioLane::outcome(): every frame of it not out by then was cut). No deadline any more;
  /// killed() until the next start().
  ULTRA_ALWAYS_INLINE uint16_t stop(int64_t cut_us) {
    uint16_t mask = 0;
    for (uint8_t k = 0; k < PARLIO_MAX_LANES; k++) {
      if (this->release_us_[k] != 0 && this->release_end_us_[k] <= cut_us)
        mask |= static_cast<uint16_t>(1u << k);
      this->release_us_[k] = 0;
    }
    this->clock_.note_stop(this->st_.epoch, static_cast<uint32_t>(cut_us));
    this->deadline_us_ = 0;
    this->kills_++;
    this->killed_.store(true, std::memory_order_release);
    return mask;
  }

 protected:
  /// chunk k must be with the DMA a margin before chunk k - 1 ends (host us, rounded down), counted from the earliest
  /// the stream can have started
  int64_t deadline_of_(int64_t k) const {
    return ParlioClock::floor_div_(this->st_.t0_ns - this->skew_ns_ + k * static_cast<int64_t>(this->chunk_ns()) -
                                       static_cast<int64_t>(this->margin_ns_), 1000);
  }
  struct Pending {
    uint8_t lane;
    int64_t at_us, end_us;
  };
  ParlioClock clock_;
  ParlioClock::State st_{};   // the task's copy (it is the only writer)
  ParlioLane lanes_[PARLIO_MAX_LANES];
  ParlioRenderer r_;
  uint32_t margin_ns_{0};
  int64_t skew_ns_{0};        // the start's uncertainty: t0 is its latest time
  Pending pending_at_[PARLIO_MAX_LANES]{};
  uint8_t pending_{0};
  int64_t release_us_[PARLIO_MAX_LANES]{};       // host us, 0 = none
  int64_t release_end_us_[PARLIO_MAX_LANES]{};   // ... its frame's last bit, at the earliest
  int64_t deadline_us_{0};                       // the next chunk not with the DMA by then: stop; 0 = none
  std::atomic<bool> killed_{false};
  uint32_t kills_{0};
};

}  // namespace esphome::dig2analog_ultra
