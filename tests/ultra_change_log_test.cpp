// The log of light changes (components/ultra_line/ultra_change_log.h; wiki: Logs): its texts, its rate limits and its
// warnings, on a simulated line (a loop every ms, a frame every 5 ms while a change runs, every 200 ms otherwise).
// Writes every line it saw to build/change_log/transcript.txt (the text snapshot) and fails on the first wrong one.
// Usage: tests/run_ultra_change_log_test.py

#include "../components/ultra_line/ultra_change_log.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace esphome::ultra_line;

namespace {

int failures = 0;
FILE *transcript = nullptr;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
      failures++;                                                            \
    }                                                                        \
  } while (0)

struct Line {
  uint32_t ms;
  uint8_t level;
  std::string text;
};

struct Sim {
  ChangeLog log;
  std::vector<Line> lines;
  uint32_t now{1000}, frames{0}, last_frame{0};
  bool have_frame{false};
  uint8_t hold{HOLD_NONE};
  std::vector<uint8_t> board_state;   // per board: ChangeLog::BOARD_SENT, BOARD_QUEUED or a ChangeHold
  bool asked{false};                  // service() asked the hub's state (only while a change is watched)
  uint32_t busy_until{0};             // frames every 5 ms until then (a change runs), else every 200 ms

  Sim(const char *name, uint16_t boards, uint16_t no_return_from = 0xFFFF) : board_state(boards, ChangeLog::BOARD_SENT) {
    log.init(boards, 16, &Sim::sink, this);
    if (no_return_from != 0xFFFF)
      log.set_no_return_from(no_return_from);
    if (transcript != nullptr)
      std::fprintf(transcript, "== %s\n", name);
  }
  static void sink(void *ctx, uint8_t level, const char *text) {
    auto *s = static_cast<Sim *>(ctx);
    s->lines.push_back({s->now, level, text});
    static const char *const LV[] = {"D", "I", "W"};
    if (transcript != nullptr)
      std::fprintf(transcript, "%7.3f %s %s\n", s->now / 1000.0, LV[level], text);
  }
  /// time runs on to `until`: a loop every ms
  void run(uint32_t until) {
    while (now < until) {
      now++;
      const uint32_t period = now < busy_until ? 5 : 200;
      if (!have_frame || now - last_frame >= period) {
        frames++;
        last_frame = now;
        have_frame = true;
      }
      log.service(now, frames, last_frame, have_frame, "800 kHz",
                  [this]() {
                    asked = true;
                    return hold;
                  },
                  [this](uint16_t b) {
                    asked = true;
                    return board_state[b];
                  });
    }
  }
  /// a light's change on board (all five channels, R G B W1 W2 in % of full)
  void change(uint16_t board, std::vector<float> pct, uint32_t transition_ms, bool limited = false,
              const char *platform = "dig2analog_plus", uint8_t mask = 0x1F) {
    uint16_t level[ChangeLog::CHANNELS]{};
    for (size_t c = 0; c < pct.size() && c < ChangeLog::CHANNELS; c++)
      level[c] = static_cast<uint16_t>(pct[c] * 655.35f + 0.5f);
    const uint32_t start = now + 10;   // (the hub's lead)
    log.change(now, frames, board, platform, mask, level, transition_ms, start, limited);
    if (start + transition_ms > busy_until)
      busy_until = start + transition_ms;
  }
  size_t count(uint8_t level) const {
    size_t n = 0;
    for (const auto &l : lines)
      n += l.level == level;
    return n;
  }
  size_t count_with(const char *part) const {
    size_t n = 0;
    for (const auto &l : lines)
      n += l.text.find(part) != std::string::npos;
    return n;
  }
  const Line *find(const char *part) const {
    for (const auto &l : lines)
      if (l.text.find(part) != std::string::npos)
        return &l;
    return nullptr;
  }
  /// a board's lines (slot 0..: its INFO lines start with "board <slot + 1> ", the number a user reads)
  std::vector<const Line *> board_lines(uint16_t board) const {
    std::vector<const Line *> out;
    const std::string tag = "board " + std::to_string(board + 1) + " ";
    for (const auto &l : lines)
      if (l.level == CHANGE_LOG_INFO && l.text.compare(0, tag.size(), tag) == 0)
        out.push_back(&l);
    return out;
  }
};

bool has(const Line *l, const char *part) { return l != nullptr && l->text.find(part) != std::string::npos; }

// One change: what goes out (the addendum's example), then its target reached; nothing more until the idle line
void test_one_change() {
  Sim s("one change", 1);
  s.run(2000);
  CHECK(s.lines.empty());   // (every board has a return channel: no boot line)
  s.change(0, {100, 0, 0, 0, 0}, 1000);
  s.run(2099);
  CHECK(s.lines.empty());   // (a group / scene settles for 100 ms first)
  s.run(2100);
  CHECK(s.lines.size() == 1);
  CHECK(s.lines[0].level == CHANGE_LOG_INFO);
  CHECK(s.lines[0].text == "board 1 (dig2analog_plus): R 100 % G 0 % B 0 % W1 0 % W2 0 %, transition 1.0 s -> GPIO16, 800 kHz");
  s.run(5000);
  CHECK(s.lines.size() == 2);
  CHECK(has(&s.lines[1], "board 1 on GPIO16: target reached, "));
  CHECK(has(&s.lines[1], " frames in 1.0 s"));
  CHECK(s.lines[1].ms >= s.lines[0].ms + ChangeLog::LINE_GAP_MS);
  CHECK(!s.log.busy());
  // the levels' texts: below 10 % with a tenth, a level that is on never shows as 0
  s.change(0, {50, 2.5f, 0.01f, 0, 100}, 0, true);
  s.run(8000);
  const Line *l = s.find("R 50 % G 2.5 % B <0.1 % W1 0 % W2 100 %, transition 0.0 s");
  CHECK(l != nullptr);
  CHECK(has(l, "; the board's power_limit may lower them"));
  // only the channels of the board's lights
  Sim w("two channels", 1);
  w.change(0, {0, 0, 0, 40, 60}, 500, false, "ur_analog_m", 0x18);
  w.run(1200);
  CHECK(w.lines.size() == 1 && w.lines[0].text == "board 1 (ur_analog_m): W1 40 % W2 60 %, transition 0.5 s -> GPIO16, 800 kHz");
}

// A dragged slider (a change every 50 ms for 3 s): at most one line a second, the latest values in each, the count
void test_dragged_slider() {
  Sim s("dragged slider", 2);
  s.run(2000);
  for (int i = 0; i <= 60; i++) {
    s.change(0, {float(i), 0, 0, 0, 0}, 1000);
    s.run(s.now + 50);
  }
  s.run(10000);
  const auto lines = s.board_lines(0);
  CHECK(lines.size() >= 4 && lines.size() <= 5);   // 3 s of changes + their outcome, a line a second at most
  for (size_t i = 1; i < lines.size(); i++)
    CHECK(lines[i]->ms - lines[i - 1]->ms >= ChangeLog::LINE_GAP_MS);
  CHECK(s.count(CHANGE_LOG_WARN) == 0);
  CHECK(s.count_with("changes coalesced") >= 2);
  CHECK(s.count_with("the previous change superseded after") >= 2);
  // the last change line carries the slider's last value, and the outcome line comes after it
  const Line *last_change = nullptr;
  for (const auto *l : lines)
    if (has(l, "transition"))
      last_change = l;
  CHECK(has(last_change, "R 60 % G 0 %"));
  CHECK(has(lines.back(), "target reached"));
  CHECK(s.board_lines(1).empty());   // (the other board had no change)
}

// A transition interrupted by a new change: the next line tells how long the first one ran
void test_interrupted_transition() {
  Sim s("interrupted transition", 1);
  s.run(2000);
  s.change(0, {100, 100, 100, 0, 0}, 5000);
  s.run(3000);
  s.change(0, {0, 0, 0, 0, 0}, 1000);
  s.run(10000);
  const auto lines = s.board_lines(0);
  CHECK(lines.size() == 3);
  if (lines.size() == 3) {
    CHECK(has(lines[0], "R 100 % G 100 % B 100 % W1 0 % W2 0 %, transition 5.0 s"));
    CHECK(has(lines[1], "R 0 % G 0 % B 0 % W1 0 % W2 0 %, transition 1.0 s"));
    CHECK(has(lines[1], "; the previous change superseded after 1.0 s, "));
    CHECK(has(lines[2], "target reached"));
    CHECK(has(lines[2], " frames in 1.0 s"));
  }
  // a fade replaced by a host effect's steps
  Sim e("superseded by an effect", 1);
  e.change(0, {100, 0, 0, 0, 0}, 4000);
  e.run(2000);
  e.log.stream(e.now, e.frames, 0, 0x07);
  e.run(4000);
  CHECK(e.board_lines(0).size() == 2);
  CHECK(e.find("board 1 on GPIO16: superseded by an effect after 1.0 s, ") != nullptr);
}

// The warnings: once per reason, with the reason; a hold of normal operation (a port's wake) only after 3 s
void test_warnings() {
  {
    Sim s("port off", 1);
    s.hold = HOLD_PORT_OFF;
    s.change(0, {100, 0, 0, 0, 0}, 1000);
    s.run(20000);
    CHECK(s.count(CHANGE_LOG_WARN) == 1);
    CHECK(s.find("GPIO16: changes can't go out yet: its Diff-Power port's power mode is Off") != nullptr);
    CHECK(s.count_with("target reached") == 0);   // (held: never reached)
    s.change(0, {50, 0, 0, 0, 0}, 1000);
    s.run(30000);
    CHECK(s.count(CHANGE_LOG_WARN) == 1);   // (said once until a change gets through)
    s.hold = HOLD_NONE;
    s.run(40000);
    CHECK(s.count_with("target reached") == 1);
    s.hold = HOLD_UPDATE;   // a new reason after a change got through: said again
    s.change(0, {10, 0, 0, 0, 0}, 0);
    s.run(42000);
    CHECK(s.find("GPIO16: changes can't go out yet: an update session holds the line") != nullptr);
  }
  {
    Sim s("port waking", 1);
    s.hold = HOLD_PORT_WAKING;   // a wake of 2 s: normal, no warning
    s.change(0, {100, 0, 0, 0, 0}, 500);
    s.run(3000);
    s.hold = HOLD_NONE;
    s.run(6000);
    CHECK(s.count(CHANGE_LOG_WARN) == 0);
    CHECK(s.count_with("target reached") == 1);
    s.hold = HOLD_POWER_SUPPLY;  // the module start taking too long: warned after 3 s
    s.change(0, {0, 0, 0, 0, 0}, 500);
    const uint32_t t0 = s.now;
    s.run(t0 + 2900);
    CHECK(s.count(CHANGE_LOG_WARN) == 0);
    s.run(t0 + 3200);
    CHECK(s.count(CHANGE_LOG_WARN) == 1);
    CHECK(s.find("the power supply isn't on yet, or its power_on_delay runs (the module start)") != nullptr);
  }
  {
    Sim s("board holds", 3);
    s.board_state[1] = HOLD_NO_DESCRIPTOR;
    s.change(1, {100, 0, 0, 0, 0}, 0);
    s.run(10000);
    CHECK(s.count(CHANGE_LOG_WARN) == 1);
    CHECK(has(s.find("board 2 has no descriptor"), "its commands can't go out"));
    s.change(7, {100, 0, 0, 0, 0}, 0);   // a board that isn't on the line
    CHECK(s.find("GPIO16: no board 8 on this line (3 boards): the change is dropped") != nullptr);
    s.log.kept(2);
    s.log.kept(2);
    CHECK(s.count_with("the change is kept while a strip is shown on its light") == 1);
    // queued commands (a busy line) are no reason to warn
    Sim q("queued", 1);
    q.board_state[0] = ChangeLog::BOARD_QUEUED;
    q.change(0, {100, 0, 0, 0, 0}, 0);
    q.run(9000);
    CHECK(q.count(CHANGE_LOG_WARN) == 0 && q.count_with("target reached") == 0);
    q.board_state[0] = ChangeLog::BOARD_SENT;
    q.run(11000);
    CHECK(q.count_with("target reached") == 1);
  }
}

// No return channel: one line at boot, never per change; the DEBUG line every 60 s while nothing changes
void test_boot_and_idle() {
  Sim s("no return, idle", 3, 0);
  s.run(1001);
  CHECK(s.lines.size() == 1 &&
        s.lines[0].text == "GPIO16: boards 1-3 have no return channel: the log shows what is sent; delivery can't be confirmed");
  s.change(0, {100, 0, 0, 0, 0}, 0);
  s.run(5000);
  CHECK(s.count_with("no return channel") == 1);
  const size_t before = s.lines.size();
  s.run(1001 + 59999);                // (the change at 1001: the idle line 60 s after it)
  CHECK(s.lines.size() == before);
  s.run(1001 + 60000);
  CHECK(s.count(CHANGE_LOG_DEBUG) == 1);
  const Line *d = s.find("GPIO16: no change for 60 s; ");
  CHECK(has(d, " frames in the last 60 s ("));
  CHECK(has(d, "/s), last frame "));
  CHECK(has(d, " s ago, 800 kHz"));
  s.run(200000);
  CHECK(s.count(CHANGE_LOG_DEBUG) == 3);   // (every 60 s)
  CHECK(s.find("no change for 180 s; 300 frames in the last 60 s (5.0/s)") != nullptr);
  Sim one("one board without a return channel", 2, 1);
  one.run(1001);
  CHECK(one.lines.size() == 1 && one.lines[0].text ==
        "GPIO16: board 2 has no return channel: the log shows what is sent; delivery can't be confirmed");
}

// Never per frame: a slider dragged at 60 Hz for 30 s on 8 boards: a line a second per board at most, and the hub's
// state asked only while a change is watched
void test_bounded() {
  Sim s("60 Hz on 8 boards", 8);
  s.run(2000);
  for (int i = 0; i < 1800; i++) {
    for (uint16_t b = 0; b < 8; b++)
      s.change(b, {float(i % 100), 0, 0, 0, 0}, 300);
    s.run(s.now + 16);
  }
  s.run(s.now + 5000);
  for (uint16_t b = 0; b < 8; b++) {
    const auto lines = s.board_lines(b);
    CHECK(lines.size() <= 32 + 1);
    for (size_t i = 1; i < lines.size(); i++)
      CHECK(lines[i]->ms - lines[i - 1]->ms >= ChangeLog::LINE_GAP_MS);
  }
  CHECK(!s.log.busy());
  s.asked = false;
  s.run(s.now + 30000);
  CHECK(!s.asked);   // (idle: the hub isn't asked)
  CHECK(s.log.memory() <= 8 * 128);
}

}  // namespace

int main(int argc, char **argv) {
  if (argc > 1)
    transcript = std::fopen(argv[1], "w");
  test_one_change();
  test_dragged_slider();
  test_interrupted_transition();
  test_warnings();
  test_boot_and_idle();
  test_bounded();
  if (transcript != nullptr)
    std::fclose(transcript);
  if (failures) {
    std::printf("ultra_change_log_test: %d FAILED\n", failures);
    return 1;
  }
  std::printf("ultra_change_log_test: all passed\n");
  return 0;
}
