#pragma once

// Light changes in the log (wiki: Ultra-Line-Options, light changes in the log): what a change sends to a board, when
// it reached its target (or what replaced it first), why changes can't go out yet, and a line every 60 s while nothing
// changes. A board gets at most one line a second: a dragged slider's changes are coalesced, the latest values win.
// Nothing here runs per frame or formats a text per frame. No ESPHome: the hub gives the time, its frame count and its
// state, and takes the lines through a sink (tests/ultra_change_log_test.cpp runs it).

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace esphome::ultra_line {

enum ChangeLogLevel : uint8_t { CHANGE_LOG_DEBUG = 0, CHANGE_LOG_INFO = 1, CHANGE_LOG_WARN = 2 };

/// Why changes can't go out yet. The line's own (UltraHub's change_hold_(), the first that applies in this order),
/// then a board's, then the warnings of a single change.
enum ChangeHold : uint8_t {
  HOLD_NONE = 0,
  HOLD_UPDATE,            // an update session holds the line
  HOLD_PORT_OFF,          // its Diff-Power port's power mode is Off
  HOLD_CALIBRATION,       // a current calibration holds the line dark
  HOLD_RECEIVER_SILENT,   // a digital port's receiver stopped answering
  HOLD_PORT_WAKING,       // its Diff-Power port in standby or waking
  HOLD_POWER_SUPPLY,      // the power supply not on yet, or its power_on_delay running
  HOLD_RECEIVERS,         // the receivers not ready yet (their start, their first answer)
  HOLD_LINE_SPEED,        // the line changes its speed
  HOLD_NO_DESCRIPTOR,     // (a board) no descriptor: its slots stay empty
  HOLD_STRIP,             // (a change) kept while a strip is shown on its light
  HOLD_NO_BOARD,          // (a change) no such board on the line: dropped
  HOLD_KINDS,
};
static_assert(HOLD_KINDS <= 16, "ChangeLog::warned_ holds a bit per hold");

class ChangeLog {
 public:
  static constexpr uint8_t CHANNELS = 5;
  static constexpr uint32_t LINE_GAP_MS = 1000;   // a board's lines at least this far apart
  static constexpr uint32_t SETTLE_MS = 100;      // a group / scene (a change per light, ms apart) in one line
  static constexpr uint32_t GRACE_MS = 3000;      // a hold that is part of normal operation (a port's wake) warns after this
  static constexpr uint32_t IDLE_MS = 60000;      // the DEBUG line while nothing changes
  /// what a board's commands are doing (board_state of service()): all out, or still queued; else a ChangeHold
  static constexpr uint8_t BOARD_SENT = HOLD_NONE, BOARD_QUEUED = 0xFF;
  using Sink = void (*)(void *ctx, uint8_t level, const char *text);

  /// boards: the line's (the per-board state is allocated here, once); pin: its GPIO (the texts)
  void init(uint16_t boards, uint8_t pin, Sink sink, void *ctx) {
    this->boards_.assign(boards, Board{});
    this->pin_ = pin;
    this->sink_ = sink;
    this->ctx_ = ctx;
  }
  /// boards from `first` on have no return channel (none with telemetry: 0); told once, at the first service()
  void set_no_return_from(uint16_t first) { this->no_return_from_ = first; }

  /// A channel's level as the board weighs it: M x A x B (before its gamma, min / max power and power_limit)
  static uint16_t weigh(uint16_t m, uint16_t a, uint16_t b) {
    return static_cast<uint16_t>(static_cast<uint64_t>(m) * a * b / (65535ull * 65535ull));
  }

  /// A light's change went to the line (a SET): its channels (mask) and their levels (weigh(), by channel), its
  /// transition, its start on the host clock (ms, the clock of now). limited: a power_limit is set on the board.
  void change(uint32_t now, uint32_t frames, uint16_t board, const char *platform, uint8_t mask, const uint16_t *level,
              uint32_t transition_ms, uint32_t start_ms, bool limited) {
    if (board >= this->boards_.size()) {
      this->warn_(HOLD_NO_BOARD, board);
      return;
    }
    Board &b = this->boards_[board];
    b.platform = platform;
    b.used |= mask;
    for (uint8_t c = 0; c < CHANNELS; c++)
      if (mask & (1u << c))
        b.level[c] = level[c];
    this->last_change_ms_ = now;
    this->idle_ms_ = now;
    this->idle_frames_ = frames;
    this->busy_ = true;
    const bool replaces = (b.flags & F_SHOWN) && (b.shown_mask & mask) != 0;
    if (!(b.flags & F_PENDING)) {
      b.flags |= F_PENDING;
      b.count = 0;
      b.pend_mask = 0;
      b.pend_first_ms = now;
    }
    if ((b.flags & F_SHOWN) && !(b.flags & F_TOLD) && (replaces || b.out != OUT_NONE)) {
      // the change in the last line: replaced before it reached its target, or its outcome not told yet (told with
      // the next line; another light's change leaves a fade that still runs alone)
      b.prev = b.out == OUT_NONE ? static_cast<uint8_t>(OUT_SUPERSEDED) : b.out;
      b.prev_ms = (b.out == OUT_NONE ? now : b.out_ms) - b.cur_ms;
      b.prev_frames = (b.out == OUT_NONE ? frames : b.out_frames) - b.cur_frames;
      b.flags |= F_TOLD;
    }
    if (b.count < 0xFFFF)
      b.count++;
    b.pend_mask |= mask;
    if (limited)
      b.flags |= F_LIMITED;
    else
      b.flags &= ~F_LIMITED;
    // the board's target: this change's, or still another light's (its fade runs on)
    const bool fading = b.out == OUT_NONE && (b.flags & (F_SHOWN | F_PENDING)) != 0 && b.len_ms != 0;
    const uint32_t target = start_ms + transition_ms;
    if (replaces || !fading || static_cast<int32_t>(target - b.target_ms) > 0) {
      b.target_ms = target;
      b.len_ms = transition_ms;
    }
    b.transition_ms = transition_ms;
    b.out = OUT_NONE;
    b.cur_ms = now;
    b.cur_frames = frames;
  }

  /// A step of a host effect on the light's channels (mask): it takes over from the change in the last line
  void stream(uint32_t now, uint32_t frames, uint16_t board, uint8_t mask) {
    if (board >= this->boards_.size())
      return;
    Board &b = this->boards_[board];
    if ((b.flags & (F_SHOWN | F_PENDING)) && b.out == OUT_NONE && ((b.shown_mask | b.pend_mask) & mask)) {
      b.out = OUT_EFFECT;
      b.out_ms = now;
      b.out_frames = frames;
    }
  }

  /// A light's change kept (a strip is shown on it): it goes out when the strip is off
  void kept(uint16_t board) { this->warn_(HOLD_STRIP, board); }

  /// The line sends everything again (a resync): what still fades starts over at start_ms
  void restart(uint32_t start_ms) {
    for (auto &b : this->boards_)
      if ((b.flags & (F_SHOWN | F_PENDING)) && b.out == OUT_NONE)
        b.target_ms = start_ms + b.len_ms;
  }

  /// Every loop. frames: the line's frames so far; last_frame_ms / have_frame: its last frame; speed: the line's
  /// speed text. hold() -> the line's ChangeHold; board_state(board) -> BOARD_SENT, BOARD_QUEUED or a ChangeHold.
  /// Both are asked only while a change is watched.
  template<typename Hold, typename BoardState>
  void service(uint32_t now, uint32_t frames, uint32_t last_frame_ms, bool have_frame, const char *speed, Hold &&hold,
               BoardState &&board_state) {
    if (!this->started_) {
      this->started_ = true;
      if (!this->busy_) {
        this->idle_ms_ = this->last_change_ms_ = now;
        this->idle_frames_ = frames;
      }
      this->boot_line_();
    }
    if (this->busy_) {
      const uint8_t h = hold();
      bool watched = false, in_flight = false;
      for (uint16_t i = 0; i < this->boards_.size(); i++) {
        Board &b = this->boards_[i];
        if (!(b.flags & (F_SHOWN | F_PENDING)))
          continue;
        if (b.out == OUT_NONE) {
          const uint8_t s = board_state(i);
          if (s != BOARD_SENT && s != BOARD_QUEUED && uint32_t(now - b.cur_ms) >= GRACE_MS)
            this->warn_(s, i);
          if (h == HOLD_NONE && s == BOARD_SENT && static_cast<int32_t>(now - b.target_ms) >= 0) {
            b.out = OUT_REACHED;
            b.out_ms = now;
            b.out_frames = frames;
            this->warned_ = 0;   // a change got through: the next hold is worth a warning again
          } else {
            in_flight = true;
          }
        }
        const bool free = !(b.flags & F_LINE) || uint32_t(now - b.line_ms) >= LINE_GAP_MS;
        if ((b.flags & F_PENDING) && free && uint32_t(now - b.pend_first_ms) >= SETTLE_MS) {
          this->change_line_(i, b, now, speed);
        } else if ((b.flags & F_SHOWN) && !(b.flags & F_PENDING) && b.out != OUT_NONE && !(b.flags & F_TOLD) && free) {
          this->outcome_line_(i, b, now);
        }
        watched |= (b.flags & F_PENDING) || ((b.flags & F_SHOWN) && !(b.flags & F_TOLD));
      }
      if (in_flight && h != HOLD_NONE) {
        if (this->hold_ != h) {
          this->hold_ = h;
          this->hold_ms_ = now;
        }
        if (!grace_(h) || uint32_t(now - this->hold_ms_) >= GRACE_MS)
          this->warn_(h, 0);
      } else {
        this->hold_ = HOLD_NONE;
      }
      this->busy_ = watched;
      return;
    }
    if (uint32_t(now - this->idle_ms_) >= IDLE_MS) {
      const uint32_t window = now - this->idle_ms_, n = frames - this->idle_frames_;
      char age[40];
      if (have_frame)
        snprintf(age, sizeof age, "last frame %.1f s ago", (now - last_frame_ms) / 1000.0f);
      else
        snprintf(age, sizeof age, "no frame sent yet");
      this->emit_(CHANGE_LOG_DEBUG, "GPIO%u: no change for %u s; %u frames in the last %u s (%.1f/s), %s, %s",
                  this->pin_, unsigned((now - this->last_change_ms_) / 1000), unsigned(n), unsigned(window / 1000),
                  n * 1000.0f / window, age, speed);
      this->idle_ms_ = now;
      this->idle_frames_ = frames;
    }
  }

  /// a change is watched (service() asks the hub's state)
  bool busy() const { return this->busy_; }
  size_t memory() const { return this->boards_.capacity() * sizeof(Board); }

 protected:
  enum Outcome : uint8_t { OUT_NONE = 0, OUT_REACHED, OUT_SUPERSEDED, OUT_EFFECT };
  static constexpr uint8_t F_PENDING = 1;   // a change not in a line yet (coalescing)
  static constexpr uint8_t F_SHOWN = 2;     // a line showed a change: its outcome is watched
  static constexpr uint8_t F_TOLD = 4;      // ... and its outcome was told
  static constexpr uint8_t F_LINE = 8;      // the board had a line (line_ms)
  static constexpr uint8_t F_LIMITED = 16;  // a power_limit on the board
  struct Board {
    const char *platform{nullptr};
    uint16_t level[CHANNELS]{};
    uint16_t count{0};                    // changes in the coming line
    uint8_t used{0}, pend_mask{0}, shown_mask{0};
    uint8_t flags{0}, out{OUT_NONE}, prev{OUT_NONE};
    uint32_t pend_first_ms{0};
    uint32_t cur_ms{0}, cur_frames{0};    // the latest change
    uint32_t target_ms{0}, len_ms{0}, transition_ms{0};
    uint32_t out_ms{0}, out_frames{0};    // its outcome
    uint32_t prev_ms{0}, prev_frames{0};  // the outcome of the change in the last line, told with the next one
    uint32_t line_ms{0};
  };

  // transient holds of normal operation warn only after GRACE_MS
  static bool grace_(uint8_t hold) {
    return hold == HOLD_PORT_WAKING || hold == HOLD_POWER_SUPPLY || hold == HOLD_RECEIVERS ||
           hold == HOLD_LINE_SPEED || hold == HOLD_NO_DESCRIPTOR;
  }

#if defined(__GNUC__)
  __attribute__((format(printf, 3, 4)))
#endif
  void emit_(uint8_t level, const char *fmt, ...) {
    char text[320];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof text, fmt, args);
    va_end(args);
    if (this->sink_ != nullptr)
      this->sink_(this->ctx_, level, text);
  }

  static void percent_(char *out, size_t n, uint16_t level) {
    const uint32_t tenths = (static_cast<uint32_t>(level) * 1000u + 32767u) / 65535u;
    if (level == 0)
      snprintf(out, n, "0");
    else if (tenths == 0)
      snprintf(out, n, "<0.1");
    else if (tenths < 100)
      snprintf(out, n, "%u.%u", unsigned(tenths / 10), unsigned(tenths % 10));
    else
      snprintf(out, n, "%u", unsigned((tenths + 5) / 10));
  }

  void change_line_(uint16_t i, Board &b, uint32_t now, const char *speed) {
    static const char *const NAMES[CHANNELS] = {"R", "G", "B", "W1", "W2"};
    char values[80] = "", *p = values;
    for (uint8_t c = 0; c < CHANNELS; c++) {
      if (!(b.used & (1u << c)))
        continue;
      char pct[8];
      percent_(pct, sizeof pct, b.level[c]);
      p += snprintf(p, values + sizeof values - p, "%s%s %s %%", p == values ? "" : " ", NAMES[c], pct);
    }
    char extra[128] = "", *e = extra;
    if (b.flags & F_LIMITED)
      e += snprintf(e, extra + sizeof extra - e, "; the board's power_limit may lower them");
    if (b.count > 1)
      e += snprintf(e, extra + sizeof extra - e, "; %u changes coalesced", unsigned(b.count));
    if (b.prev == OUT_SUPERSEDED)
      snprintf(e, extra + sizeof extra - e, "; the previous change superseded after %.1f s, %u frames",
               b.prev_ms / 1000.0f, unsigned(b.prev_frames));
    else if (b.prev == OUT_REACHED)
      snprintf(e, extra + sizeof extra - e, "; the previous change reached its target, %u frames in %.1f s",
               unsigned(b.prev_frames), b.prev_ms / 1000.0f);
    else if (b.prev == OUT_EFFECT)
      snprintf(e, extra + sizeof extra - e, "; the previous change superseded by an effect after %.1f s, %u frames",
               b.prev_ms / 1000.0f, unsigned(b.prev_frames));
    char who[48];
    if (b.platform != nullptr)
      snprintf(who, sizeof who, "board %u (%s)", unsigned(i) + 1, b.platform);
    else
      snprintf(who, sizeof who, "board %u", unsigned(i) + 1);
    this->emit_(CHANGE_LOG_INFO, "%s: %s, transition %.1f s -> GPIO%u, %s%s", who, values, b.transition_ms / 1000.0f,
                this->pin_, speed, extra);
    b.flags = static_cast<uint8_t>((b.flags & ~(F_PENDING | F_TOLD)) | F_SHOWN | F_LINE);
    b.shown_mask = b.pend_mask;
    b.prev = OUT_NONE;
    b.line_ms = now;
  }

  void outcome_line_(uint16_t i, Board &b, uint32_t now) {
    const float t = (b.out_ms - b.cur_ms) / 1000.0f;
    const unsigned n = unsigned(b.out_frames - b.cur_frames);
    if (b.out == OUT_REACHED)
      this->emit_(CHANGE_LOG_INFO, "board %u on GPIO%u: target reached, %u frames in %.1f s", unsigned(i) + 1, this->pin_, n, t);
    else
      this->emit_(CHANGE_LOG_INFO, "board %u on GPIO%u: superseded by an effect after %.1f s, %u frames", unsigned(i) + 1,
                  this->pin_, t, n);
    b.flags |= F_TOLD | F_LINE;
    b.line_ms = now;
  }

  void boot_line_() {
    const uint16_t n = static_cast<uint16_t>(this->boards_.size());
    if (this->no_return_from_ >= n)
      return;
    const uint16_t first = this->no_return_from_;
    if (n - first == 1)
      this->emit_(CHANGE_LOG_INFO,
                  "GPIO%u: board %u has no return channel: the log shows what is sent; delivery can't be confirmed",
                  this->pin_, unsigned(first) + 1);
    else
      this->emit_(CHANGE_LOG_INFO,
                  "GPIO%u: boards %u-%u have no return channel: the log shows what is sent; delivery can't be confirmed",
                  this->pin_, unsigned(first) + 1, unsigned(n));
  }

  /// once per kind, until a change gets through again
  void warn_(uint8_t hold, uint16_t board) {
    if (hold == HOLD_NONE || hold >= HOLD_KINDS || (this->warned_ & (1u << hold)))
      return;
    this->warned_ |= static_cast<uint16_t>(1u << hold);
    switch (hold) {
      case HOLD_NO_DESCRIPTOR:
        this->emit_(CHANGE_LOG_WARN,
                    "GPIO%u: board %u has no descriptor: its commands can't go out until it reports its own (a "
                    "receiver that answers on the return channel; or give it descriptors:)",
                    this->pin_, unsigned(board) + 1);
        return;
      case HOLD_STRIP:
        this->emit_(CHANGE_LOG_WARN, "GPIO%u: board %u: the change is kept while a strip is shown on its light", this->pin_,
                    unsigned(board) + 1);
        return;
      case HOLD_NO_BOARD:
        this->emit_(CHANGE_LOG_WARN, "GPIO%u: no board %u on this line (%u boards): the change is dropped", this->pin_,
                    unsigned(board) + 1, unsigned(this->boards_.size()));
        return;
      default:
        break;
    }
    static const char *const WHY[] = {
        "",
        "an update session holds the line",
        "its Diff-Power port's power mode is Off",
        "a current calibration holds the line dark (the change shows when it ends)",
        "the receiver stopped answering: nothing goes out until it is retried",
        "its Diff-Power port is in standby or waking (bus-on)",
        "the power supply isn't on yet, or its power_on_delay runs (the module start)",
        "the receivers aren't ready (their start, or no answer since the port came up)",
        "the line is changing its speed",
    };
    this->emit_(CHANGE_LOG_WARN, "GPIO%u: changes can't go out yet: %s", this->pin_, WHY[hold]);
  }

  std::vector<Board> boards_;
  Sink sink_{nullptr};
  void *ctx_{nullptr};
  uint32_t last_change_ms_{0}, idle_ms_{0}, idle_frames_{0}, hold_ms_{0};
  uint16_t no_return_from_{0xFFFF}, warned_{0};
  uint8_t pin_{0}, hold_{HOLD_NONE};
  bool started_{false}, busy_{false};
};

}  // namespace esphome::ultra_line
