// ------------------------------------------------------------------------------------------- threaded line handover
// The job's task and the main loop as two threads, on the component's own code (cut unchanged into
// tests/build/port_job/thread_*.inc by run_port_job_test.py): UltraFirmwareUpdate::JobRun (the task's transport and its
// lease), job_task_, service_job_ and abort; UltraHub::lease_line, job_send and job_frame_start_us. The hub around them
// is a stand-in: a line that is always in one of its response windows while it is the hub's (back-to-back polls), the
// receivers answering in each, its direction pin, its RMT or PARLIO lane. Every take of the line meets an open window
// (the first, and each one after a verification); the main loop never waits for the task longer than one loop slice
// (the task once waited there for 3 s, holding the driver's lock), the window is read, the direction pin is driven
// and the full reset restored before the job's first frame. Also the race where the task's ask changes after the main
// loop looked at it and before it takes the driver's lock. On an RMT line, a digital port (marker envelopes), a lane.
//
// The clock is real time; while the line is the job's and the job waits for its next frame (no window, no poll then),
// the task's sleep moves the clock to the job's due time instead (its erase and transit waits take no real time).

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "../components/ultra_line/port_artifact.h"
#include "ultra_dport.h"
#include "../components/ultra_line/ultra_readiness.h"
#include "../components/diffpower/diffpower_core.h"

namespace esphome::ultra_line::threaded {

static_assert(RECEIVER_MODULE_START_MS == 500 && RECEIVER_REPLY_TIMEOUT_MS == 5000,
              "bench startup margin and the silent-return safety timeout");

static void test_readiness_epochs() {
  ReceiverReadiness r;
  check(!r.ready(1999, true, true, 1500, false, 200) && r.ready(2000, true, true, 1500, false, 200),
        "no-return startup: exactly 500 ms from bus-on, beyond port-up");
  check(!r.ready(3500, true, true, 1500, true, 200), "a return channel waits beyond the fixed allowance");
  r.answer(true, 1500, 1499);
  check(!r.ready(3500, true, true, 1500, true, 200), "a stale power epoch's window cannot unlock readiness");
  r.answer(true, 1500, 3500);
  check(r.ready(3500, true, true, 1500, true, 200) && !r.take_warning(), "valid reply unlocks immediately");
  check(!r.ready(3501, false, false, 3501, true, 200), "power loss drops a prior reply");
  check(!r.ready(9199, true, true, 4000, true, 200), "silent return: still waiting 4999 ms after port-up");
  check(r.ready(9200, true, true, 4000, true, 200) && r.take_warning() && !r.take_warning(),
        "silent return: exactly 5000 ms after port-up, one warning");
  check(!r.ready(9500, true, true, 9400, true, 200), "new bus-on epoch invalidates fallback and reply");
  ReceiverReadiness wrap;
  const uint32_t since = UINT32_MAX - 300;
  check(!wrap.ready(since + 499, true, true, since, false) && wrap.ready(since + 500, true, true, since, false),
        "the startup allowance survives millis wrap");
}

static const auto T0 = std::chrono::steady_clock::now();
static int64_t real_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - T0).count();
}
static std::atomic<int64_t> warp_us{0};
static int64_t test_us = -1;   // deterministic power-up regression, before the threaded cases
static int64_t esp_timer_get_time() { return test_us >= 0 ? test_us : real_us() + warp_us.load(); }
static int64_t now_us_() { return esp_timer_get_time(); }
static uint32_t millis() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

static constexpr int64_t SLICE_US = 16000;    // one normal main loop slice (ESPHome's loop interval)
static constexpr int64_t WINDOW_US = 40000;   // a response window, from its poll to its read (> a sleep's jitter)
static constexpr int64_t HOLD_US = 50000;     // the race: the task inside one step this long (the main loop waits)
static constexpr int64_t WATCHDOG_US = 5000000;   // a pass this long: the main loop is stuck (the test fails)
static constexpr uint32_t FULL_RESET = 24000, RW_RESET = 777;

static thread_local bool on_task = false;
static PortJobDriver *task_driver = nullptr;
static ultra_update_job_t *race_job = nullptr;
static std::atomic<int> race{0};   // 1 armed, 2 the task holds the lock at the job's verification, 3 let go

using TaskHandle_t = void *;
static void vTaskDelete(TaskHandle_t) {}
static void vTaskDelay(uint32_t ticks) {
  if (on_task && task_driver->leased()) {
    const int64_t due = task_driver->due_us(), now = esp_timer_get_time();
    if (due > now) {
      warp_us += due - now;
      return;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
}
// the task's clock in its steps (JobRun::now_us, under the driver's lock): where the race holds it
static int64_t step_clock() {
  if (on_task && race.load() == 1 && race_job->state == ULTRA_JOB_VERIFY) {
    race.store(2);
    std::this_thread::sleep_for(std::chrono::microseconds(HOLD_US));
    race.store(3);
  }
  return esp_timer_get_time();
}

static const char *const TAG = "port_job_threads";
static void log_(const char *, const char *, ...) {}
static int readiness_warnings = 0;
static void warn_(const char *, const char *message, ...) {
  if (std::strstr(message, "no receiver answered") != nullptr) readiness_warnings++;
}
#define ESP_LOGI(tag, ...) log_(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) warn_(tag, __VA_ARGS__)

using esp_err_t = int;
static constexpr esp_err_t ESP_OK = 0;
using rmt_channel_handle_t = void *;
using rmt_encoder_handle_t = void *;
struct rmt_transmit_config_t {
  int loop_count;
};

/// A job frame as it went on the wire: what the hub's line was then
struct Sent {
  int64_t at_us;
  size_t bytes;
  bool pin_high, window_open, leased;
  uint32_t reset;
  uint8_t type;
};
static std::vector<Sent> sent;

struct Pin {
  bool level{false};
  void digital_write(bool on) { this->level = on; }
};

class UltraHub;
class ParlioLane {
 public:
  enum : uint8_t { PENDING = 0, ON_WIRE, LOST };
  UltraHub *hub{nullptr};
  uint32_t id_{0};
  bool post(const uint8_t *, size_t n, uint32_t, bool);
  uint32_t posted_id() const { return this->id_; }
  uint32_t posted_epoch() const { return 1; }
  int64_t frame_end_us() const { return now_us_(); }
  uint8_t outcome(uint32_t, uint32_t, int64_t, int64_t) const { return ON_WIRE; }
  int64_t reserve_us() const { return now_us_(); }
};

class UltraFirmwareUpdate;

struct PowerPort {
  diffpower::PortCore core_;
  const diffpower::PortCore &core() const { return this->core_; }
  bool is_ready() const { return this->core_.ready(); }
};

/// The hub as the cut methods see it; tick() is its own loop (its polls while the line is its own)
class UltraHub {
 public:
  PowerPort *port_{nullptr};
  uint8_t pin_{16};
  bool boards_powered_() const;
  bool receivers_ready_() const;
  mutable ReceiverReadiness receiver_ready_;
  int *rw_uart_{nullptr};
  bool digital_{false};
  ParlioLane *lane_{nullptr};
  bool started_{true};
  uint8_t tx_buf_[UPD_FRAME_BYTES * 4]{};
  size_t tx_buf_bytes_{sizeof(tx_buf_)};
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  uint32_t rmt_res_{10000000};
  struct {
    ultra_dp_rmt_t tab;
  } dp_enc_{};
  struct Word {
    uint32_t val;
  };
  struct {
    struct {
      Word bit0, bit1;
      uint32_t reset;
    } symbols;
  } enc_{};
  uint32_t reset_symbol_{FULL_RESET};
  bool upd_lease_{false};
  bool rw_open_{false}, rw_driving_{true}, rw_release_{false}, tx_prepared_{false}, tx_rw_{false};
  int64_t rw_read_us_{0}, rw_end_us_{0};
  Pin pin;
  Pin *rw_dir_pin_{&pin};
  VerifyPoller *poller{nullptr};
  const PortInventory *inv{nullptr};   // (the receivers that answer: the job's nodes)
  const std::vector<ultra_update_artifact_t> *artifacts{nullptr};
  UltraFirmwareUpdate *update{nullptr};
  int windows{0}, read_on_take{0}, resyncs{0}, refreshes{0};

  bool lease_line(bool on);
  bool job_send(const uint8_t *frame, size_t n, const ultra_update_transport_t &t);
  int64_t job_frame_start_us();
  bool job_line_free() { return true; }
  bool is_failed() const { return false; }
  uint32_t line_gap_us() const { return 500; }
  bool has_telemetry() const { return true; }
  const TelemetryPoller &telemetry() const { return *this->poller; }
  bool port_off() const { return false; }
  void refresh_identities() { this->refreshes++; }
  void resync() { this->resyncs++; }
  void rw_drop_turn_() {}
  void finish_window_(bool) { this->read_(); }
  void digital_window_track_() {}
  void digital_window_end_(int64_t) { this->read_(); }
  void tick();
  void read_();
};

class UltraFirmwareUpdate {
 public:
  struct JobRun;
  JobRun *run_{nullptr};
  UltraHub *hub_{nullptr};
  uint32_t progress_ms_{0};
  struct {
    bool has_progress{false};
    float progress{0};
  } update_info_;
  bool ended_{false};
  int statuses_{0};
  std::string get_name() const { return "port"; }
  void set_status_(const std::string &) { this->statuses_++; }
  void publish_state() {}
  void end_job_() { this->ended_ = true; }
  static void job_task_(void *arg);
  void service_job_();
  void abort();
};

static UltraHub *wire_hub = nullptr;
static void on_wire(const uint8_t *frame, size_t n) {
  const UltraHub &h = *wire_hub;
  sent.push_back(Sent{now_us_(), n, h.pin.level, h.rw_open_, h.upd_lease_, h.enc_.symbols.reset, frame[0]});
}
static esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void *frame, size_t n,
                              const rmt_transmit_config_t *) {
  on_wire(static_cast<const uint8_t *>(frame), n);
  return ESP_OK;
}
static esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int) { return ESP_OK; }
bool ParlioLane::post(const uint8_t *frame, size_t n, uint32_t, bool) {
  on_wire(frame, n);
  this->id_++;
  return true;
}

#define USE_ULTRA_LINE_TELEMETRY
#define USE_DIG2ANALOG_UR_DIGITAL_M
#define USE_ULTRA_PARLIO
#define USE_DIFFPOWER
#define esp_timer_get_time step_clock   // (JobRun's clock only: its now_us)
#include "build/port_job/thread_job_run.inc"
#undef esp_timer_get_time
#include "build/port_job/thread_update.inc"
#include "build/port_job/thread_hub.inc"
#undef USE_DIFFPOWER
#undef USE_ULTRA_LINE_TELEMETRY
#undef USE_DIG2ANALOG_UR_DIGITAL_M
#undef USE_ULTRA_PARLIO
#undef ESP_LOGI
#undef ESP_LOGW

// a window read: the receivers' answers (those the job has sent their image to: on it now), the line the hub's again
void UltraHub::read_() {
  this->rw_open_ = false;
  if (this->upd_lease_)
    this->read_on_take++;
  const uint32_t now_ms = millis();
  this->update->run_->driver.with_job([&](ultra_update_job_t *j) {
    if (j->state != ULTRA_JOB_VERIFY)
      return 0;
    const uint16_t active = j->active_artifact;
    for (uint16_t n = 0; n < this->inv->nodes.size(); n++) {
      const ultra_update_result_t &r = j->results[n];
      if (r.result == ULTRA_JOB_PENDING && r.transfer_complete && r.artifact == active && this->inv->nodes[n].live)
        this->poller->answer(n, this->inv->nodes[n], (*this->artifacts)[active].descriptor.bytes, now_ms);
    }
    return 0;
  });
}

// the hub's loop while the line is its own: a window read once it is over, the next poll at once (the receivers
// drive the line in it: the direction pin low, the RW reset)
void UltraHub::tick() {
  if (this->upd_lease_)
    return;
  const int64_t now = now_us_();
  if (this->rw_open_ && now >= (this->digital_ ? this->rw_end_us_ : this->rw_read_us_))
    this->read_();
  if (!this->rw_open_) {
    this->rw_open_ = true;
    this->rw_release_ = true;
    this->rw_driving_ = false;
    this->pin.digital_write(false);
    this->enc_.symbols.reset = RW_RESET;
    this->rw_end_us_ = this->rw_read_us_ = now + WINDOW_US;
    this->windows++;
  }
}

struct Run {
  int64_t worst_us{0}, race_us{-1};
  int passes{0}, waited{0};   // waited: takes the hub refused at first (their window still open)
  bool ended{false};
};

static PortNode silent_node(PortNode n) { return silent(n); }   // (test_power_ready's silent is its readiness)

// Production power core -> boards_powered_ -> lease -> driver -> sender, with simulated time. A short raw DETECT
// loss while WAKING must restart the existing readiness wait; no UPDATE frame may consume training before it.
// recover: the Recover firmware of the port's one silent known board (no reply ends the wait: the 5 s fallback).
static void test_power_ready(bool already_up, bool dropout, bool during_dark, bool lane, bool digital,
                             bool cancel = false, bool returns = false, bool silent = false, bool late_cancel = false,
                             bool recover = false) {
  test_us = 1000000;
  PowerPort port;
  port.core_.set_detect(true, millis());
  port.core_.set_mode(during_dark ? diffpower::MODE_DURING_DARK : diffpower::MODE_STANDBY);
  port.core_.set_dark(false);
  port.core_.update(millis());
  check(port.core_.state() == diffpower::PS_STANDBY, "update readiness: port starts dark in standby");
  const uint32_t settle = port.core_.config().hotplug_ms + port.core_.config().boot_margin_ms;
  check(settle == 1700, "update readiness: existing 1500 + 200 ms values");
  port.core_.prewake(millis(), 120000);   // the update's power hold
  port.core_.update(millis());
  if (already_up) {
    test_us += int64_t(port.core_.config().hotplug_ms + RECEIVER_MODULE_START_MS) * 1000;
    check(port.core_.update(millis()) & diffpower::EV_READY, "update readiness: already-up setup reaches EV_READY");
  }
  const int64_t start = test_us;
  const uint32_t module_ms = silent ? port.core_.config().boot_margin_ms + RECEIVER_REPLY_TIMEOUT_MS :
      (returns ? 2000 : RECEIVER_MODULE_START_MS);
  const int64_t expected_ready = already_up ? start : start +
      int64_t((dropout ? 800 : 0) + port.core_.config().hotplug_ms + module_ms) * 1000;
  const int64_t alive = already_up ? start : start +
      int64_t((dropout ? 800 : 0) + port.core_.config().hotplug_ms + (returns ? 2000 : 300)) * 1000;
  PortInventory inv;
  inv.nodes = {recover ? silent_node(node(0, V407_T, true)) : node(0, V407_T, true)};
  const std::vector<Image> images = {image_for(V407_T, 0x000701FF, 512)};
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {info_for(images[0], V407_T, 'a')};
  m.layouts = {layout_for(V407_T)};
  if (recover)
    check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "silent board: the release checks");
  const PortPlan plan = recover ? build_recover_plan(inv, m, PT_DIFF_POWER, 0) : build_port_plan(inv, m, PT_DIFF_POWER);
  if (recover)
    check(!plan.blocked && plan.recover == 0 && plan.nodes[0].action == PA_SEND && plan.nodes[0].expect == PE_RECOVERY,
          "silent board: its Recover firmware plans its image, blind");
  line_for(1, PT_DIFF_POWER);
  if (digital)
    check(ultra_sender_digital(&::line, 1) == ULTRA_OK, "update readiness: digital transport");
  Glue g;
  check(g.begin(inv, plan, images, start) == 0, "update readiness: update begins");
  UltraHub hub;
  ParlioLane l;
  hub.port_ = &port;
  int uart;
  hub.rw_uart_ = returns ? &uart : nullptr;
  if (already_up && returns)
    hub.receiver_ready_.answer(port.core_.line_on(), port.core_.line_ms(), millis());
  hub.digital_ = digital;
  hub.lane_ = lane ? &l : nullptr;
  hub.pin.digital_write(true);
  hub.enc_.symbols.reset = FULL_RESET;
  wire_hub = &hub;
  sent.clear();
  UltraFirmwareUpdate::JobRun run;
  run.hub = &hub;
  uint8_t buffer[1024];
  run.driver.begin(&g.job, &run, buffer, sizeof(buffer));
  if (cancel) {
    if (late_cancel) {
      test_us += int64_t(settle) * 1000;
      port.core_.update(millis());
      check(port.is_ready(), "update readiness: cancellation during module wait, after electrical port-up");
    }
    check(run.driver.step() == PortJobDriver::LEASING && run.serve_lease() == -1,
          "update readiness: cancellation starts with a pending dark-port lease");
    port.core_.set_mode(diffpower::MODE_OFF);
    port.core_.update(millis());
    ultra_update_job_abort(&g.job);
    check(run.driver.step() == PortJobDriver::DONE && g.job.state == ULTRA_JOB_DONE &&
              g.results[0].result == PRR_FAILED && g.results[0].reason == PRQ_ABORT && sent.empty(),
          "update readiness: Off abort finishes without UPDATE traffic");
    if (!late_cancel)
      check(!port.core_.enable(), "update readiness: an early Off abort never powers the cold port");
    check(run.serve_lease() == 0 && !hub.upd_lease_, "update readiness: cancelled wake releases the pending lease");
    wire_hub = nullptr;
    test_us = -1;
    return;
  }
  bool early = false, old_deadline_blocked = !dropout, new_deadline_blocked = already_up;
  const int warnings_before = readiness_warnings;
  int announces = 0, pings[2]{};
  int64_t first_end[2]{}, last_end[2]{}, first_ping = -1;
  for (; test_us <= expected_ready + 5000000 && announces < 3; test_us += 1000) {
    const int64_t elapsed = test_us - start;
    if (dropout && (elapsed == 750000 || elapsed == 800000))
      port.core_.set_detect(elapsed == 800000, millis());
    port.core_.update(millis());
    if (returns && !silent && test_us >= alive)
      hub.receiver_ready_.answer(port.core_.line_on(), port.core_.line_ms(), millis());
    run.serve_lease();
    const size_t before = sent.size();
    run.driver.step();
    // The task asks, the main loop serves, the task can send in the same timestamp when already ready.
    run.serve_lease();
    run.driver.step();
    early |= test_us < expected_ready && !sent.empty();
    if (dropout && elapsed == int64_t(settle) * 1000)
      old_deadline_blocked = !port.is_ready() && sent.empty();
    if (test_us == expected_ready - 1000)
      new_deadline_blocked = !hub.receivers_ready_() && sent.empty();
    if (sent.size() != before) {
      const Sent &s = sent.back();
      if (s.type == UPD_PING) {
        if (first_ping < 0) first_ping = s.at_us;
        const int phase = announces ? 1 : 0;
        // Only PINGs launched after readiness count toward the visible preamble/training duration.
        if (s.at_us >= alive) {
          if (!pings[phase]++) first_end[phase] = g.job.transfer.poll_end_us;
          last_end[phase] = g.job.transfer.poll_end_us;
        }
      } else if (s.type == UPD_ANNOUNCE) {
        announces++;
      }
    }
  }
  check(!early && old_deadline_blocked && new_deadline_blocked,
        "update readiness: no frame before uninterrupted EN/DETECT + readiness (up=%d drop=%d dark=%d lane=%d digital=%d)",
        already_up, dropout, during_dark, lane, digital);
  check(first_ping == expected_ready, "update readiness: first PING at ready, no extra delay (%lld == %lld)",
        static_cast<long long>(first_ping), static_cast<long long>(expected_ready));
  check(announces == 3 && pings[0] > 1 && last_end[0] - first_end[0] >= 1000000 &&
            pings[1] > 1 && last_end[1] - first_end[1] >= 1000000,
        "update readiness: full preamble and training after ready (%lld / %lld us)",
        static_cast<long long>(last_end[0] - first_end[0]), static_cast<long long>(last_end[1] - first_end[1]));
  check(readiness_warnings - warnings_before == int(silent),
        "module readiness: only a silent return path warns, once per power-on");
  hub.lease_line(false);
  wire_hub = nullptr;
  test_us = -1;
}

// The 5 s silent fallback ends only the readiness wait (decision 10-23, option B): after it, "update all" on a port where
// a known board doesn't answer refuses, every silent board named, the sender agrees and nothing goes on the wire; that
// board's Recover firmware proceeds at the fallback (test_power_ready recover).
static void test_silent_after_fallback(uint16_t boards) {
  test_us = 1000000;
  PowerPort port;
  port.core_.set_detect(true, millis());
  port.core_.set_mode(diffpower::MODE_STANDBY);
  port.core_.set_dark(false);
  port.core_.update(millis());
  port.core_.prewake(millis(), 120000);
  port.core_.update(millis());
  UltraHub hub;
  hub.port_ = &port;
  int uart;
  hub.rw_uart_ = &uart;
  wire_hub = &hub;
  sent.clear();
  const int warnings_before = readiness_warnings;
  // one board: no reply at all, readiness only by the fallback; two: B0 answers 1,800 ms after bus-on, B1 never
  const int64_t start = test_us;
  const int64_t answer = start + int64_t(port.core_.config().hotplug_ms + 300) * 1000;
  const int64_t ready = boards == 1 ? start + int64_t(port.core_.config().hotplug_ms + port.core_.config().boot_margin_ms +
                                                      RECEIVER_REPLY_TIMEOUT_MS) * 1000 : answer;
  bool early = false;
  for (; test_us < ready; test_us += 1000) {
    port.core_.update(millis());
    early |= hub.receivers_ready_();
  }
  port.core_.update(millis());
  if (boards > 1)
    hub.receiver_ready_.answer(port.core_.line_on(), port.core_.line_ms(), millis());
  check(!early && hub.receivers_ready_() && readiness_warnings - warnings_before == int(boards == 1),
        "silent board (%u boards): readiness at %lld ms (%s)", boards, static_cast<long long>((ready - start) / 1000),
        boards == 1 ? "no reply: the 5 s fallback, one warning" : "B1's reply, no warning");
  const uint16_t quiet = static_cast<uint16_t>(boards - 1);
  PortInventory inv;
  for (uint16_t k = 0; k < boards; k++)
    inv.nodes.push_back(k == quiet ? silent(node(k, V407_T, true)) : node(k, V407_T, true));
  for (auto &n : inv.nodes)
    n.uid[1] = static_cast<uint8_t>(0x40 + n.position);
  const std::vector<Image> images = {image_for(V407_T, 0x000701FF, 512)};
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {info_for(images[0], V407_T, 'a')};
  m.layouts = {layout_for(V407_T)};
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "silent board: the release checks");
  const PortPlan all = build_port_plan(inv, m, PT_DIFF_POWER);
  const std::string named = "B" + std::to_string(quiet + 1) + " not answering: check power and cable";
  check(all.blocked && all.block_reason == PRS_SILENT && all.block_position == quiet &&
            port_blocked_text(all) == named,
        "silent board (%u boards): \"update all\" after the fallback refuses, the board named (%s)", boards,
        port_blocked_text(all).c_str());
  line_for(boards, PT_DIFF_POWER);
  Glue g;
  const int rc = g.begin(inv, all, images, esp_timer_get_time());
  check(rc == ULTRA_UPDATE_SILENT_BOARDS && g.results[quiet].result == PRR_UNREACHABLE &&
            g.results[quiet].reason == PRQ_NOT_ANSWERING,
        "silent board (%u boards): ... the sender refuses it too (%d)", boards, rc);
  PortReport rep = port_report_from_plan(all, &inv);
  port_job_take_results(rep, g.results.data(), g.results.size());
  port_report_close(rep, PJO_BLOCKED);
  check(port_report_text(rep).find("Not installed 0.7.1: " + named + ", nothing sent") == 0 && sent.empty(),
        "silent board (%u boards): ... its report names it, nothing on the wire (%s)", boards,
        port_report_text(rep).c_str());
  const PortPlan rp = build_recover_plan(inv, m, PT_DIFF_POWER, quiet);
  check(!rp.blocked && rp.recover == quiet && rp.nodes[quiet].action == PA_SEND && rp.nodes[quiet].expect == PE_RECOVERY,
        "silent board (%u boards): its Recover firmware is not blocked", boards);
  wire_hub = nullptr;
  test_us = -1;
}

/// One job on one kind of line: the task (job_task_) and the main loop (the hub's loop, then service_job_), as the
/// component runs them. A Diff-Power port or lane: a mixed port (a V407 and a V203 receiver, a one-way board, a current
/// one), two subtransfers, so the second take comes after a verification; a digital port: its one receiver.
static void run_line(const char *what, bool digital, bool lane, bool arm_race) {
  const uint32_t V071 = 0x000701FF;
  PortInventory inv;
  if (digital)
    inv.nodes = {node(0, V407_T, true)};
  else
    inv.nodes = {node(0, V407_T, true), node(1, V203_T, true), node(2, V203_T, false), node(3, V407_T, true, V071)};
  if (!digital)
    inv.nodes[3].uid[1] = 9;
  const uint16_t count = static_cast<uint16_t>(inv.nodes.size());
  const int takes = digital ? 1 : 2;
  static std::vector<Image> images;
  images = {image_for(V203_T, V071, 512), image_for(V407_T, V071, 768)};
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {info_for(images[0], V203_T, 'a'), info_for(images[1], V407_T, 'b')};
  m.layouts = {layout_for(V203_T), layout_for(V407_T)};
  const PortPlan plan = build_port_plan(inv, m, PT_DIFF_POWER);
  line_for(count, PT_DIFF_POWER);
  if (digital)
    check(ultra_sender_digital(&::line, 1) == ULTRA_OK, "%s: the digital line", what);
  Glue g;
  check(g.begin(inv, plan, images, esp_timer_get_time()) == 0, "%s: the job begins", what);
  g.job.options.verify_timeout_ms = PORT_JOB_VERIFY_MS;

  VerifyPoller p;
  p.init(count, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  UltraHub hub;
  ParlioLane l;
  l.hub = &hub;
  hub.digital_ = digital;
  hub.lane_ = lane ? &l : nullptr;
  hub.poller = &p;
  hub.inv = &inv;
  hub.artifacts = &g.artifacts;
  wire_hub = &hub;
  sent.clear();
  UltraFirmwareUpdate update;
  auto run = std::make_unique<UltraFirmwareUpdate::JobRun>();
  run->hub = &hub;
  run->plan = plan;
  run->inv = inv;
  static uint8_t buffer[1024];
  run->driver.begin(&g.job, run.get(), buffer, sizeof(buffer));
  update.run_ = run.get();
  update.hub_ = &hub;
  hub.update = &update;
  task_driver = &run->driver;
  race_job = &g.job;
  race.store(arm_race ? 1 : 0);
  hub.tick();   // (a poll out: its window open when the job asks for the line)

  Run r;
  const int64_t start = real_us();
  std::thread task([&run] {
    on_task = true;
    UltraFirmwareUpdate::job_task_(run.get());
  });
  // a main loop pass that never returns (the task keeps the driver's lock): a failure, not a hung test
  std::atomic<int64_t> pass_t0{-1};
  std::atomic<bool> done{false};
  std::thread watchdog([&] {
    while (!done.load()) {
      const int64_t t0 = pass_t0.load();
      if (t0 >= 0 && real_us() - t0 > WATCHDOG_US) {
        check(false, "%s%s: the main loop is blocked by the task for over %lld us (one pass; the signal-loss floor is "
              "2,100 ms)", what, arm_race ? " (race)" : "", static_cast<long long>(WATCHDOG_US));
        std::printf("port job glue against the sender job: %d checks, %d failed, %d pending\n", checks, failures,
                    pending);
        std::fflush(stdout);
        std::_Exit(1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  bool was_refused = false;
  while (!update.ended_ && real_us() - start < 120000000) {
    hub.tick();
    const int before = race.load();
    const int64_t t0 = real_us();
    pass_t0.store(t0);
    update.service_job_();
    pass_t0.store(-1);
    const int64_t took = real_us() - t0;
    const int after = race.load();
    const bool refused = hub.upd_lease_ && hub.rw_open_;   // (asked; lease_line said not yet)
    if (refused && !was_refused)
      r.waited++;
    was_refused = refused;
    if (before == 2 || (before == 1 && after >= 2))
      r.race_us = std::max(r.race_us, took);   // (this pass met the task holding the lock)
    else
      r.worst_us = std::max(r.worst_us, took);
    r.passes++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  task.join();
  done.store(true);
  watchdog.join();
  r.ended = update.ended_;
  const bool none_early = !sent.empty() && std::all_of(sent.begin(), sent.end(), [](const Sent &s) {
    return s.pin_high && !s.window_open && s.leased && s.reset == FULL_RESET;
  });
  const bool verified = digital ? g.results[0].result == PRR_VERIFIED
                                : g.results[0].result == PRR_VERIFIED && g.results[1].result == PRR_VERIFIED &&
                                      g.results[2].result == PRR_SENT_UNVERIFIED && g.results[3].result == PRR_CURRENT;
  check(r.ended && g.job.state == ULTRA_JOB_DONE && verified, "%s: the job ends, its receivers verified (%d passes)",
        what, r.passes);
  check(r.worst_us <= SLICE_US,
        "%s: the main loop never waits for the task longer than one loop slice (longest pass %lld us <= %lld us)",
        what, static_cast<long long>(r.worst_us), static_cast<long long>(SLICE_US));
  check(r.waited == takes && hub.read_on_take == takes && hub.resyncs == takes && hub.refreshes == takes,
        "%s: every take met an open response window, waited for it and read it first; every hand-back resynced and "
        "read the receivers again (%d takes waited, %d read on a take, %d resyncs, %d identity reads; %d expected)",
        what, r.waited, hub.read_on_take, hub.resyncs, hub.refreshes, takes);
  check(none_early, "%s: every job frame (%u) on a line the job holds, the window read, the direction pin driven, "
        "the full reset (%u ticks)", what, unsigned(sent.size()), sent.empty() ? 0u : sent.front().reset);
  if (arm_race)
    check(race.load() == 3 && r.race_us >= 0 && r.race_us <= HOLD_US + SLICE_US,
          "%s: the task's ask changes while the main loop waits for the driver's lock (the task held it %lld us): "
          "the main loop waited %lld us (<= the hold + one slice), the hand-back applied at its next pass",
          what, static_cast<long long>(HOLD_US), static_cast<long long>(r.race_us));
  std::printf("  %s%s: longest main loop pass %lld us (the race's pass: %lld us), %d takes waited for an open window, "
              "%u job frames, %d passes\n",
              what, arm_race ? " (race)" : "", static_cast<long long>(r.worst_us), static_cast<long long>(r.race_us),
              r.waited, unsigned(sent.size()), r.passes);
  race.store(0);
  task_driver = nullptr;
  wire_hub = nullptr;
}

/// PortLease alone: an answer is for its ask only
static void test_lease_numbers() {
  PortLease l;
  int applied = 0;
  check(l.ask(false) && l.serve([&](bool) { return ++applied > 0; }) == -1 && applied == 0,
        "lease: the line is the hub's at first, nothing to apply");
  check(!l.ask(true) && l.serve([](bool) { return false; }) == -1 && !l.ask(true),
        "lease: a take the hub cannot hand over yet (its window) stays asked");
  // the task hands back while the main loop applies its take: the take is answered, the hand-back is not yet
  const int took = l.serve([&](bool on) {
    check(on && !l.ask(false), "lease: the ask changes while the main loop applies the one before");
    return true;
  });
  check(took == 1 && !l.ask(false), "lease: the main loop answered the take only (%d)", took);
  check(l.serve([](bool on) { return !on; }) == 0 && l.ask(false), "lease: the hand-back at its next pass");
  // taken, handed back, taken again before the main loop looked: the new take is its own ask
  check(!l.ask(true) && l.serve([](bool) { return true; }) == 1 && l.ask(true), "lease: taken");
  check(!l.ask(false) && !l.ask(true), "lease: an ask after a hand-back the main loop has not applied waits for it");
  check(l.serve([](bool on) { return on; }) == 1 && l.ask(true), "lease: ... the main loop applies the newest ask");
}

static void test_threaded_handover() {
  test_readiness_epochs();
  test_lease_numbers();
  for (int transport = 0; transport < 3; transport++)
    for (int scenario = 0; scenario < 4; scenario++)
      test_power_ready(scenario == 1, scenario == 2, scenario == 3, transport == 1, transport == 2);
  test_power_ready(false, false, false, false, false, true);
  for (int transport = 0; transport < 3; transport++)
    for (int scenario = 0; scenario < 4; scenario++)
      test_power_ready(scenario == 1, scenario == 2, scenario == 3, transport == 1, transport == 2, false, true);
  for (int transport = 0; transport < 3; transport++)
    test_power_ready(false, false, false, transport == 1, transport == 2, false, true, true);
  test_power_ready(false, false, false, false, false, true, true);
  test_power_ready(false, false, false, false, false, true, true, false, true);
  test_power_ready(false, false, false, false, false, true, false, false, true);
  test_silent_after_fallback(1);
  test_silent_after_fallback(2);
  for (int transport = 0; transport < 3; transport++)
    test_power_ready(false, false, false, transport == 1, transport == 2, false, true, true, false, true);
  struct Line {
    const char *what;
    bool digital, lane;
  };
  static const Line LINES[] = {{"threaded RMT", false, false}, {"threaded digital port", true, false},
                               {"threaded PARLIO lane", false, true}};
  for (const Line &l : LINES) {
    run_line(l.what, l.digital, l.lane, false);
    run_line(l.what, l.digital, l.lane, true);
  }
  std::printf("  threaded handover: the task and the main loop on the component's own lease, service and hub code "
              "(RMT, digital port, PARLIO lane): every take inside an open window, no wait over one loop slice\n");
}

}  // namespace esphome::ultra_line::threaded
