// Host test of the return channel's hub side (components/dig2analog_ultra/ultra_telemetry.h) against a modelled
// chain of Diff-Power ultra receivers: the RW frames' POLL plans, the answers in a hub UART's byte stream with junk,
// missing and late answers, a receiver that never answers, restarts and swapped boards, the POWER acknowledge,
// BUSY / UNSUPPORTED pages, more than 16 receivers, the sensor values / texts, and protocol 0.2.5 (LIFETIME, DIAG's
// two parts, STATUS's reason, ENERGY's LED supply and peak, HEALTH's flags). The receiver firmware's own answer path:
// tests/ultra_telemetry_contract_test.cpp.
//
// Build and run: python tests/run_ultra_telemetry_test.py

#include "../components/dig2analog_ultra/ultra_telemetry.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace esphome::dig2analog_ultra;

static int failures = 0;
#define CHECK(cond, ...)               \
  do {                                 \
    if (!(cond)) {                     \
      failures++;                      \
      std::printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__);        \
      std::printf("\n");               \
    }                                  \
  } while (0)

static std::mt19937 rng(1234);

// ------------------------------------------------------------------------------------------------ the receiver model
/// One receiver as the hub sees it over the line (wiki: Return Channel): it takes part when its slot is a valid POLL
/// and the boards in front of it took part; it answers its page, applies a POWER acknowledge first.
struct Model {
  uint8_t uid[12]{};
  bool has_channel{true};    // false: an older firmware / a dig2analog board: never takes part, relays nothing
  bool drop_next{false};     // takes no part in the next window (a flash write just before the frame)
  bool outputs_page{true};   // optional page
  int busy_energy{0};        // answers ENERGY BUSY this many times
  uint32_t uptime{100};
  uint8_t reset{ULTRA_ST_RST_POWER};
  uint8_t state{0};          // STATUS 8 (POWER_EVENT added while a latch is set)
  uint8_t why{0};            // OUTPUTS 23
  uint16_t vbus{2400}, current{1500};
  uint16_t limits{0}, overloads{0}, shorts{0}, dips{0};
  uint8_t latched{0};
  uint8_t trip_cause{0}, retry{0};
  uint32_t total_wh{1234}, boot_mwh{567}, power_mw{36000};
  uint32_t features{ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_HEALTH | ULTRA_FEAT_ENERGY |
                    ULTRA_FEAT_MCU_TEMP | ULTRA_FEAT_CURRENT_SENSE};
  uint32_t asked[16]{};      // POLLs per page id (< 16)
  uint32_t acks{0};          // acknowledges taken
  bool contention{false};
  // protocol 0.2.5 (set025(): its features too)
  uint8_t why025{0};         // STATUS 8 bits 4-7
  uint8_t he_flags{ULTRA_HE_F_TESTED | ULTRA_HE_F_ABNORMAL};
  uint8_t ch2_class{ULTRA_HE_ABNORMAL | ULTRA_HE_R_NOT_RISING};
  uint16_t vb{0xFFFF}, peak{0};   // ENERGY 15 / 17 (0 before 0.2.5)
  uint32_t lit_01h{12345}, power_ups{41}, lt_dips{2};
  uint16_t lt_overloads{1}, lt_shorts{3};
  uint8_t bc_cause{0xFF};
  uint16_t supply_z{95};
  uint8_t fm_trip{0}, fm_rst{0};
  uint16_t fm_trip_h{0};
  uint8_t fm_dip_pct{0};
  uint8_t flash_pending{0};  // DIAG 20 (0.2.6: ULTRA_DG_FP_EXHAUSTED)
  uint32_t diag_parts[3]{};  // DIAG POLLs per part (2: any other)

  void set025() {
    this->features |= ULTRA_FEAT_LIFETIME | ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE;
    this->vb = 2400;
  }

  uint16_t uid16() const { return ultra_crc16(this->uid, 12); }
  void event(uint8_t ev) {
    this->latched |= ev;
    if (ev & ULTRA_PW_EV_SHORT)
      this->shorts++;
    if (ev & ULTRA_PW_EV_OVERLOAD)
      this->overloads++;
    if (ev & ULTRA_PW_EV_LIMITED)
      this->limits++;
    if (ev & ULTRA_PW_EV_SUPPLY_DIP)
      this->dips++;
  }
  void restart() {
    this->uptime = 0;
    this->reset = ULTRA_ST_RST_WATCHDOG;
    this->limits = this->overloads = this->shorts = this->dips = 0;
    this->latched = 0;
    this->boot_mwh = 0;
  }
  /// the answer to a POLL slot with index i
  void answer(const uint8_t *poll, uint8_t *a) {
    const uint8_t page = poll[ULTRA_POLL_PAGE];
    const uint8_t *arg = poll + ULTRA_POLL_ARG;
    uint8_t d[24]{};
    uint8_t out_page = page;
    if (page < 16)
      this->asked[page]++;
    switch (page) {
      case ULTRA_PAGE_STATUS:
        ultra_put24(d + ULTRA_ST_FW_VERSION, 0x000608);
        d[ULTRA_ST_HW] = 0x21;
        ultra_put24(d + ULTRA_ST_UPTIME, this->uptime);
        d[ULTRA_ST_RESET] = this->reset;
        d[ULTRA_ST_STATE] = static_cast<uint8_t>(this->state | (this->latched ? ULTRA_ST_POWER_EVENT : 0) |
                                                 this->why025 << ULTRA_ST_WHY_SHIFT);
        ultra_put16(d + ULTRA_ST_MCU_TEMP, 412);
        ultra_put16(d + ULTRA_ST_TEMP_BOARD, ULTRA_ST_NA);
        ultra_put16(d + ULTRA_ST_VBUS, this->vbus);
        ultra_put16(d + ULTRA_ST_VDDA, 3301);
        ultra_put16(d + ULTRA_ST_CURRENT, this->current);
        d[ULTRA_ST_MEASURED] = ULTRA_ST_M_VBUS | ULTRA_ST_M_CURRENT;
        break;
      case ULTRA_PAGE_IDENTITY:
        std::memcpy(d, this->uid, 12);
        ultra_put32(d + ULTRA_ID_FW_VERSION, 0x000608FF);
        d[ULTRA_ID_BL_VERSION] = 2;
        d[ULTRA_ID_HW_ID] = 2;
        ultra_put16(d + ULTRA_ID_HW_CAPS, ULTRA_HWCAP_RETURN | ULTRA_HWCAP_RELAY | ULTRA_HWCAP_VBUS | ULTRA_HWCAP_CURRENT);
        ultra_put32(d + ULTRA_ID_FEATURES, this->features);
        break;
      case ULTRA_PAGE_POWER:
        if (arg[0] & ULTRA_PW_ARG_ACK) {   // a latch clears when its count equals the acknowledged one
          this->acks++;
          if ((this->latched & ULTRA_PW_EV_LIMITED) && ultra_get16(arg + 1) == this->limits)
            this->latched &= ~ULTRA_PW_EV_LIMITED;
          if ((this->latched & ULTRA_PW_EV_OVERLOAD) && ultra_get16(arg + 3) == this->overloads)
            this->latched &= ~ULTRA_PW_EV_OVERLOAD;
          if ((this->latched & ULTRA_PW_EV_SHORT) && ultra_get16(arg + 5) == this->shorts)
            this->latched &= ~ULTRA_PW_EV_SHORT;
          if ((this->latched & ULTRA_PW_EV_SUPPLY_DIP) && ultra_get16(arg + 7) == this->dips)
            this->latched &= ~ULTRA_PW_EV_SUPPLY_DIP;
        }
        d[ULTRA_PW_NOW] = this->state & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING);
        d[ULTRA_PW_LATCHED] = this->latched;
        ultra_put16(d + ULTRA_PW_LIMIT_EVENTS, this->limits);
        ultra_put16(d + ULTRA_PW_OVL_TRIPS, this->overloads);
        ultra_put16(d + ULTRA_PW_SHORT_TRIPS, this->shorts);
        ultra_put24(d + ULTRA_PW_LIMIT_AGE, this->limits ? 40 : 0xFFFFFF);
        ultra_put24(d + ULTRA_PW_TRIP_AGE, this->shorts + this->overloads ? 12 : 0xFFFFFF);
        ultra_put16(d + ULTRA_PW_LIMIT_MA, 4000);
        d[ULTRA_PW_SRC_CAUSE] = static_cast<uint8_t>((ULTRA_PW_SRC_USER << 4) | this->trip_cause);
        ultra_put16(d + ULTRA_PW_EVENT_MA, 4100);
        ultra_put16(d + ULTRA_PW_TRIP_MA, 12400);
        ultra_put16(d + ULTRA_PW_FACTOR, this->state & ULTRA_ST_TRIPPED ? 0 : 65535);
        d[ULTRA_PW_RETRY_S] = this->retry;
        break;
      case ULTRA_PAGE_SUPPLY:
        ultra_put16(d + ULTRA_SU_DIPS, this->dips);
        ultra_put16(d + ULTRA_SU_REF_MV, 24000);
        ultra_put16(d + ULTRA_SU_NOW_MV, 23950);
        d[ULTRA_SU_LOGGED] = this->dips ? 1 : 0;
        ultra_put24(d + ULTRA_SU_EV_AGE, this->dips ? 3600 : 0xFFFFFF);
        ultra_put16(d + ULTRA_SU_EV_MV, 18200);
        d[ULTRA_SU_EV_PCT] = 76;
        ultra_put16(d + ULTRA_SU_EV_MS, 12);
        break;
      case ULTRA_PAGE_ENERGY:
        if (this->busy_energy > 0) {
          this->busy_energy--;
          out_page = ULTRA_PAGE_BUSY;
          break;
        }
        ultra_put32(d + ULTRA_EN_BOOT_MWH, this->boot_mwh);
        ultra_put32(d + ULTRA_EN_TOTAL_WH, this->total_wh);
        ultra_put32(d + ULTRA_EN_POWER_MW, this->power_mw);
        ultra_put24(d + ULTRA_EN_SAVED_AGE, 100);
        if (this->features & ULTRA_FEAT_LIFETIME) {   // (before 0.2.5: 0)
          ultra_put16(d + ULTRA_EN_VB, this->vb);
          ultra_put16(d + ULTRA_EN_PEAK_MA, this->peak);
        }
        break;
      case ULTRA_PAGE_HEALTH:
        d[ULTRA_HE_FLAGS] = this->he_flags;
        for (int i = 0; i < 5; i++)
          d[ULTRA_HE_CLASS + i] = ULTRA_HE_NORMAL;
        d[ULTRA_HE_CLASS + 1] = this->ch2_class;
        break;
      case ULTRA_PAGE_LIFETIME:
        if (!(this->features & ULTRA_FEAT_LIFETIME)) {
          out_page = ULTRA_PAGE_UNSUPPORTED;
          break;
        }
        ultra_put24(d + ULTRA_LT_POWERED, this->lit_01h * 2);
        ultra_put24(d + ULTRA_LT_LIT, this->lit_01h);
        ultra_put24(d + ULTRA_LT_POWER_UPS, this->power_ups);
        ultra_put16(d + ULTRA_LT_OVERLOAD_TRIPS, this->lt_overloads);
        ultra_put16(d + ULTRA_LT_SHORT_TRIPS, this->lt_shorts);
        ultra_put24(d + ULTRA_LT_DIPS, this->lt_dips);
        break;
      case ULTRA_PAGE_DIAG:
        this->diag_parts[arg[0] < 2 ? arg[0] : 2]++;
        if (!(this->features & ULTRA_FEAT_DIAG) || arg[0] > 1) {
          out_page = ULTRA_PAGE_UNSUPPORTED;
          break;
        }
        if (arg[0] == 0) {
          d[ULTRA_DG_RESET] = this->reset;
          d[ULTRA_DG_BC_CAUSE] = this->bc_cause;
          ultra_put16(d + ULTRA_DG_BC_PC, 0x1A2C);
          ultra_put24(d + ULTRA_DG_BC_UPTIME, 12600);
          ultra_put16(d + ULTRA_DG_SUPPLY_Z, this->supply_z);
          d[ULTRA_DG_FLASH_PENDING] = this->flash_pending;
          d[ULTRA_DG_C10_LOAD] = d[ULTRA_DG_C10_PEAK] = 0xFF;
        } else {
          d[ULTRA_DG_TRIP] = this->fm_trip;
          ultra_put16(d + ULTRA_DG_TRIP_LIT_H, this->fm_trip_h);
          d[ULTRA_DG_DIP_PCT] = this->fm_dip_pct;
          d[ULTRA_DG_DIP_10MS] = 3;
          d[ULTRA_DG_DIP_CURRENT] = 12;
          d[ULTRA_DG_RST] = this->fm_rst;
          d[ULTRA_DG_RST_UP_H] = 12;
          const int8_t trend[5] = {-3, ULTRA_HE_NA, 1, ULTRA_HE_NA, ULTRA_HE_NA};
          for (int i = 0; i < 5; i++)
            d[ULTRA_DG_TREND + i] = static_cast<uint8_t>(trend[i]);
          ultra_put16(d + ULTRA_DG_TREND_LIT_H, 1200);
          d[ULTRA_DG_TREND_TESTS] = 4;
        }
        break;
      case ULTRA_PAGE_OUTPUTS:
        if (!this->outputs_page) {
          out_page = ULTRA_PAGE_UNSUPPORTED;
          break;
        }
        d[ULTRA_OU_WHY] = this->why;
        break;
      default:
        out_page = ULTRA_PAGE_UNSUPPORTED;
        break;
    }
    a[0] = ULTRA_RSP_SYNC;
    a[1] = out_page;
    a[2] = static_cast<uint8_t>((poll[ULTRA_POLL_INDEX] << 4) | (this->latched ? ULTRA_RSP_EVENT : 0) |
                                (this->contention ? ULTRA_RSP_CONTENTION : 0));
    a[3] = poll[2];
    ultra_put16(a + 4, this->uid16());
    std::memcpy(a + ULTRA_RSP_DATA, d, 24);
    ultra_put16(a + 30, ultra_crc16(a, 30));
  }
};

/// The line: the boards in chain order, then boards without a return channel (dig2analog) behind them.
struct Chain {
  std::vector<Model> rx;
  uint32_t now_ms{0};
  uint32_t frames{0};
  std::vector<uint8_t> last_uart;   // (a late copy for the stale test)
  bool junk{true};
  bool corrupt_one{false};          // break one answer's CRC (it counts as missing)
  bool plan_ok{true};
  std::string plan_err;
  std::vector<uint32_t> rw_ms;      // when each RW frame went out

  explicit Chain(size_t n) : rx(n) {
    for (size_t i = 0; i < n; i++) {
      for (int k = 0; k < 12; k++)
        rx[i].uid[k] = static_cast<uint8_t>(0x30 + 7 * i + k);
    }
  }
  void junk_bytes(std::vector<uint8_t> &u, int n) {
    for (int i = 0; i < n; i++) {
      const uint32_t r = rng();
      u.push_back(r % 5 == 0 ? ULTRA_RSP_SYNC : r % 3 == 0 ? 0x00 : static_cast<uint8_t>(r >> 8));
    }
  }
  /// one RW frame from the poller and its window; returns the POLLed boards
  uint16_t window(TelemetryPoller &p) {
    std::vector<uint8_t> slots(ULTRA_SLOT_BYTES * (rx.size() + 1), 0);
    uint32_t w = 0;
    const uint16_t n = p.build(now_ms, slots.data(), &w);
    if (n == 0)
      return 0;
    frames++;
    rw_ms.push_back(now_ms);
    // the plan: every slot a valid POLL, one R and one rate, unique rising indices, W from them
    int r_all = -1, last_index = -1;
    for (uint16_t k = 0; k < n; k++) {
      const uint8_t *s = slots.data() + ULTRA_SLOT_BYTES * k;
      const bool valid = ultra_slot_check(s) == 0 && ultra_poll_valid(s) == 0;
      const int r = s[ULTRA_POLL_SLOTS];
      if (!valid || (r_all >= 0 && r != r_all) || s[ULTRA_POLL_RATE] != p.rate()) {
        plan_ok = false;
        plan_err = "slot " + std::to_string(k) + " not a valid POLL / other R or rate";
      }
      r_all = r;
      const uint8_t idx = s[ULTRA_POLL_INDEX];
      if (idx != ULTRA_POLL_RELAY_ONLY) {
        if (static_cast<int>(idx) != last_index + 1) {
          plan_ok = false;
          plan_err = "indices not unique / rising at slot " + std::to_string(k);
        }
        last_index = idx;
      }
    }
    if (last_index + 1 != r_all || w != ultra_rw_window_us(static_cast<uint8_t>(r_all), p.rate())) {
      plan_ok = false;
      plan_err = "R != the answering count, or W wrong";
    }
    // the answers in index order (chain order); a board that doesn't take part silences everything behind it
    std::vector<uint8_t> uart;
    if (junk)
      junk_bytes(uart, 6);   // the turnaround checks' pulses: 0x00 / junk, stray 0xA5
    bool silent = false;
    std::vector<int> corrupt_at;
    for (uint16_t k = 0; k < n && k < rx.size(); k++) {
      Model &m = rx[k];
      const uint8_t *s = slots.data() + ULTRA_SLOT_BYTES * k;
      if (silent || !m.has_channel || m.drop_next) {
        m.drop_next = false;
        silent = true;
        continue;
      }
      if (s[ULTRA_POLL_INDEX] == ULTRA_POLL_RELAY_ONLY)
        continue;
      uint8_t a[32];
      m.answer(s, a);
      uart.push_back(ULTRA_RSP_LEAD);
      const size_t at = uart.size();
      uart.insert(uart.end(), a, a + 32);
      if (corrupt_one) {
        uart[at + 10] ^= 0x40;
        corrupt_one = false;
      }
      if (junk && rng() % 4 == 0)
        junk_bytes(uart, 1 + rng() % 3);
    }
    if (junk)
      junk_bytes(uart, 3);
    now_ms += 1 + w / 1000;
    p.finish(now_ms, uart.data(), uart.size());
    last_uart = uart;
    return n;
  }
  /// run for ms (or until done() holds after a window): an RW frame whenever one is due, the models' uptime counting
  template<typename F> bool run_until(TelemetryPoller &p, uint32_t ms, F done) {
    const uint32_t end = now_ms + ms;
    while (static_cast<int32_t>(now_ms - end) < 0) {
      const uint32_t before = now_ms / 1000;
      if (p.due(now_ms)) {
        window(p);
        if (done())
          return true;
      } else {
        now_ms += 5;
      }
      for (uint32_t s = before; s < now_ms / 1000; s++) {
        for (auto &m : rx)
          m.uptime++;
      }
    }
    return false;
  }
  void run(TelemetryPoller &p, uint32_t ms) {
    const uint32_t end = now_ms + ms;
    while (static_cast<int32_t>(now_ms - end) < 0) {
      const uint32_t before = now_ms / 1000;
      if (p.due(now_ms))
        window(p);
      else
        now_ms += 5;
      for (uint32_t s = before; s < now_ms / 1000; s++) {
        for (auto &m : rx)
          m.uptime++;
      }
    }
  }
};

static TelemetryPoller started(Chain &c, uint16_t receivers, uint8_t rate = ULTRA_RW_RATE_1M,
                               TelemetryIntervals iv = TelemetryIntervals{}) {
  TelemetryPoller p;
  p.init(receivers, rate, iv);
  p.locked(c.now_ms);
  return p;
}

// ------------------------------------------------------------------------------------------------------------ tests
/// Four receivers for a minute: the lock, the pages each one has, every value decoded, the intervals kept.
static void test_basic() {
  Chain c(4);
  c.rx[2].outputs_page = false;
  TelemetryPoller p = started(c, 4);
  for (uint16_t i = 0; i < 4; i++)
    p.set_outputs(i, true);   // (receivers with several lights)
  CHECK(!p.due(c.now_ms + 1000) && p.due(c.now_ms + 1001), "no RW frame before 1 s of headers after the preamble");
  c.run(p, 60000);
  CHECK(c.plan_ok, "every RW frame's POLL plan valid (%s)", c.plan_err.c_str());
  bool spaced = true;
  for (size_t i = 1; i < c.rw_ms.size(); i++)
    spaced &= c.rw_ms[i] - c.rw_ms[i - 1] >= TelemetryPoller::MIN_SPACING_MS;
  CHECK(spaced && c.rw_ms.front() > 1000, "RW frames >= 10 ms apart, the first after 1 s (%u frames)",
        static_cast<unsigned>(c.rw_ms.size()));
  for (uint16_t i = 0; i < 4; i++) {
    const ReceiverData &d = p.data(i);
    const Model &m = c.rx[i];
    CHECK(p.answering(i) && d.have_identity && d.have_status && d.have_power && d.have_supply && d.have_energy &&
              d.have_health && d.have_outputs == (i != 2),
          "receiver %u: every page it has read (outputs %d)", i, d.have_outputs);
    CHECK(std::fabs(telemetry_value(d, TV_SUPPLY_VOLTAGE) - 24.0f) < 1e-3f &&
              std::fabs(telemetry_value(d, TV_INPUT_CURRENT) - 1.5f) < 1e-4f &&
              std::fabs(telemetry_value(d, TV_INPUT_POWER) - 36.0f) < 1e-3f &&
              std::fabs(telemetry_value(d, TV_MCU_TEMPERATURE) - 41.2f) < 1e-3f &&
              telemetry_value(d, TV_ENERGY) == 1234.0f && telemetry_value(d, TV_UPTIME) <= m.uptime &&
              telemetry_value(d, TV_UPTIME) + 4 >= m.uptime,
          "receiver %u: V %.2f I %.3f P %.2f T %.1f E %.0f up %.0f", i, telemetry_value(d, TV_SUPPLY_VOLTAGE),
          telemetry_value(d, TV_INPUT_CURRENT), telemetry_value(d, TV_INPUT_POWER),
          telemetry_value(d, TV_MCU_TEMPERATURE), telemetry_value(d, TV_ENERGY), telemetry_value(d, TV_UPTIME));
    CHECK(telemetry_firmware_text(d) == "0.6.8" && telemetry_reset_text(d.reset_cause) == "power-on" &&
              telemetry_status_text(d) == "ok" && telemetry_power_events_text(d) == "no events" &&
              telemetry_health_text(d) == "CH2 abnormal (current didn't rise with the level)",
          "receiver %u texts: '%s' '%s' '%s' '%s' '%s'", i, telemetry_firmware_text(d).c_str(),
          telemetry_reset_text(d.reset_cause).c_str(), telemetry_status_text(d).c_str(),
          telemetry_power_events_text(d).c_str(), telemetry_health_text(d).c_str());
    // the intervals: STATUS every 3 s (early answers in frames others need: up to every frame), POWER 5 s, the
    // slow pages about once a minute, IDENTITY once
    CHECK(m.asked[ULTRA_PAGE_IDENTITY] == 1 && m.asked[ULTRA_PAGE_STATUS] >= 15 && m.asked[ULTRA_PAGE_POWER] >= 9 &&
              m.asked[ULTRA_PAGE_POWER] <= 14 && m.asked[ULTRA_PAGE_ENERGY] >= 1 && m.asked[ULTRA_PAGE_ENERGY] <= 3 &&
              m.asked[ULTRA_PAGE_OUTPUTS] == (i == 2 ? 1u : m.asked[ULTRA_PAGE_OUTPUTS]),
          "receiver %u polls in 60 s: identity %u status %u power %u outputs %u energy %u supply %u health %u", i,
          m.asked[ULTRA_PAGE_IDENTITY], m.asked[ULTRA_PAGE_STATUS], m.asked[ULTRA_PAGE_POWER],
          m.asked[ULTRA_PAGE_OUTPUTS], m.asked[ULTRA_PAGE_ENERGY], m.asked[ULTRA_PAGE_SUPPLY], m.asked[ULTRA_PAGE_HEALTH]);
  }
  CHECK(c.frames < 60, "the schedules align: %u RW frames for 4 receivers in a minute", c.frames);
  CHECK(p.stats().missing == 0 && p.stats().stale == 0, "no answer missing or stale (%u, %u)", p.stats().missing,
        p.stats().stale);
  std::printf("  4 receivers, 60 s: %u RW frames, %u answers, junk with stray SYNC bytes: %u rejected\n", c.frames,
              p.stats().answers, p.stats().bad_crc);
}

/// The parser: an answer cut short, a broken CRC, 0xA5 inside DATA, a late copy of an older window, duplicates.
static void test_parser() {
  Chain c(2);
  TelemetryPoller p = started(c, 2);
  c.now_ms = 1000;
  c.window(p);   // IDENTITY from both
  CHECK(p.answering(0) && p.answering(1) && p.data(1).have_identity, "both identified");
  // a late copy of that window's bytes in the next one: wrong SEQ, not taken
  std::vector<uint8_t> slots(64);
  uint32_t w;
  c.now_ms += 100;
  p.build(c.now_ms, slots.data(), &w);
  const uint32_t stale0 = p.stats().stale;
  std::vector<uint8_t> late = c.last_uart;
  // and a SYNC with 31 bytes of junk behind it, the answers duplicated
  late.push_back(ULTRA_RSP_SYNC);
  for (int i = 0; i < 40; i++)
    late.push_back(static_cast<uint8_t>(i * 13));
  p.finish(c.now_ms + 6, late.data(), late.size());
  CHECK(p.stats().stale >= stale0 + 2, "an older window's answers are stale (%u)", p.stats().stale - stale0);
  CHECK(p.misses(0) == 1 && p.misses(1) == 0, "only the first missing answer counts (%u %u)", p.misses(0),
        p.misses(1));
  // an answer whose DATA holds 0xA5 everywhere, a broken one before it: the scan resyncs after the bad one
  c.rx[0].vbus = 0xA5A5;
  c.corrupt_one = true;
  c.now_ms += 300;
  c.window(p);
  CHECK(p.misses(0) == 2 && p.answering(0), "a broken CRC is a missing answer (%u)", p.misses(0));
  c.run(p, 4000);   // STATUS read again
  CHECK(p.misses(0) == 0 && p.data(0).vbus_10mv == 0xA5A5, "0xA5 in DATA is no start: %04X", p.data(0).vbus_10mv);
}

/// Missing answers: retried in a later window with a new SEQ; the boards behind a silent one don't count; one that
/// keeps missing cuts the chain there, and the ones behind come back with it.
static void test_missing() {
  Chain c(4);
  TelemetryPoller p = started(c, 4);
  c.run(p, 10000);
  const uint8_t seq0 = p.seq(1);
  c.rx[1].drop_next = true;   // a flash write before the frame: no part this window, nor behind it
  c.now_ms += 4000;
  c.window(p);
  CHECK(p.misses(1) == 1 && p.misses(2) == 0 && p.misses(3) == 0 && p.answering(2),
        "receiver 1 silent: its miss counts, 2 and 3 behind it don't (%u %u %u)", p.misses(1), p.misses(2),
        p.misses(3));
  c.run(p, 2000);
  CHECK(p.misses(1) == 0 && p.seq(1) != seq0, "asked again in a later window with a new SEQ");

  c.rx[2].has_channel = false;   // e.g. a board with an older firmware swapped in at position 2
  c.run(p, 15000);
  CHECK(!p.answering(2) && !p.answering(3) && p.answering(0) && p.answering(1) && p.cut_at(c.now_ms) == 2,
        "receiver 2 never answers: it and 3 behind it unavailable, the chain cut at 2 (%d)", p.cut_at(c.now_ms));
  std::vector<uint8_t> slots(32 * 5);
  uint32_t w;
  CHECK(p.build(c.now_ms, slots.data(), &w) == 3, "nothing polled past the cut");
  p.finish(c.now_ms + 5, nullptr, 0);
  CHECK(std::isnan(telemetry_value(p.data(3), TV_SUPPLY_VOLTAGE)) == false,
        "(its last values stay in the data; the sensors show unavailable from answering())");
  c.rx[2].has_channel = true;
  c.run(p, 15000);
  CHECK(p.answering(2) && p.answering(3) && p.cut_at(c.now_ms) < 0, "receiver 2 answers again: 2 and 3 back");
}

/// A restart (the uptime falls): IDENTITY again, nothing past it for 1 s, no acknowledge across it. A new board at a
/// position (UID16): its data dropped and read again.
static void test_restart_and_swap() {
  Chain c(3);
  TelemetryPoller p = started(c, 3);
  c.run(p, 8000);
  (void) p.take_updates(0);
  // a short circuit: POWER read, then the board restarts before the acknowledge goes out
  c.rx[0].event(ULTRA_PW_EV_SHORT);
  c.rx[0].state = ULTRA_ST_TRIPPED;
  c.rx[0].trip_cause = ULTRA_PW_CAUSE_SHORT;
  c.rx[0].retry = 8;
  const bool read = c.run_until(p, 6000, [&] { return (p.data(0).pw_latched & ULTRA_PW_EV_SHORT) != 0; });
  CHECK(read && c.rx[0].latched == ULTRA_PW_EV_SHORT, "the short read, not acknowledged yet (latched %02X)",
        p.data(0).pw_latched);
  CHECK(telemetry_status_text(p.data(0)) == "tripped: short circuit, retry in 8 s",
        "status: '%s'", telemetry_status_text(p.data(0)).c_str());
  const uint32_t acks0 = c.rx[0].acks;
  const uint32_t ident0 = c.rx[0].asked[ULTRA_PAGE_IDENTITY];
  c.rx[0].restart();
  c.rx[0].state = 0;
  c.rx[0].event(ULTRA_PW_EV_LIMITED);   // a new latch after the restart
  c.run(p, 20000);
  CHECK(c.rx[0].asked[ULTRA_PAGE_IDENTITY] == ident0 + 1 && (p.take_updates(0) & TU_RESTART),
        "the restart seen: IDENTITY again");
  CHECK(c.rx[0].latched == 0 && c.rx[0].acks > acks0 && p.data(0).short_trips == 0 && p.data(0).limit_events == 1,
        "acknowledged after the restart with its new counts only (latched %02X, %u acks)", c.rx[0].latched,
        c.rx[0].acks - acks0);
  // a swapped board at position 1
  for (auto &b : c.rx[1].uid)
    b ^= 0x5A;
  c.rx[1].features &= ~ULTRA_FEAT_HEALTH;
  const uint32_t ident1 = c.rx[1].asked[ULTRA_PAGE_IDENTITY];
  c.run(p, 3000);
  CHECK(c.rx[1].asked[ULTRA_PAGE_IDENTITY] == ident1 + 1 && !p.data(1).have_health &&
            p.data(1).have_identity && p.data(1).uid[0] == c.rx[1].uid[0],
        "a new board at position 1: its data dropped, IDENTITY read, HEALTH not asked any more");
}

/// POWER: the counts read are shown and acknowledged exactly; an event after the read stays latched; a dip's count
/// comes from SUPPLY first.
static void test_power_ack() {
  Chain c(1);
  TelemetryPoller p = started(c, 1);
  c.run(p, 8000);
  c.rx[0].event(ULTRA_PW_EV_LIMITED);
  c.rx[0].state = ULTRA_ST_LIMITING;
  c.run(p, 6000);   // RFLAGS EVENT / STATUS: POWER read, the ack a second later
  CHECK(telemetry_status_text(p.data(0)) == "limiting to 4.00 A (user limit)", "status '%s'",
        telemetry_status_text(p.data(0)).c_str());
  CHECK(c.rx[0].latched == 0 && telemetry_power_events_text(p.data(0)) == "1 limit event (last 40 s ago, 4.10 A)",
        "the limit event shown and acknowledged ('%s', latched %02X)", telemetry_power_events_text(p.data(0)).c_str(),
        c.rx[0].latched);
  // an event between the read and the acknowledge: its latch stays, read and acknowledged next time
  c.rx[0].state = 0;
  c.rx[0].event(ULTRA_PW_EV_OVERLOAD);
  const bool read = c.run_until(p, 8000, [&] { return (p.data(0).pw_latched & ULTRA_PW_EV_OVERLOAD) != 0; });
  c.rx[0].event(ULTRA_PW_EV_OVERLOAD);   // a second one before the ack
  const uint32_t acks0 = c.rx[0].acks;
  c.run_until(p, 4000, [&] { return c.rx[0].acks > acks0; });   // the ack with 1 overload: the latch stays
  CHECK(read && c.rx[0].latched == ULTRA_PW_EV_OVERLOAD && p.data(0).pw_latched == ULTRA_PW_EV_OVERLOAD,
        "an event after the read stays latched (%02X)", c.rx[0].latched);
  c.run(p, 3000);
  CHECK(c.rx[0].latched == 0 && p.data(0).overload_trips == 2, "then read and acknowledged (%02X, %u)",
        c.rx[0].latched, p.data(0).overload_trips);
  // a supply dip: SUPPLY's count read, then acknowledged
  c.rx[0].event(ULTRA_PW_EV_SUPPLY_DIP);
  c.run(p, 6000);
  CHECK(c.rx[0].latched == 0 && p.data(0).dips == 1 &&
            telemetry_power_events_text(p.data(0)).find("1 supply dip (last to 18.20 V, 76 %, 12 ms, 60 min ago)") !=
                std::string::npos,
        "the dip read from SUPPLY and acknowledged ('%s')", telemetry_power_events_text(p.data(0)).c_str());
}

/// BUSY: asked again soon. UNSUPPORTED: not asked again. OUTPUTS' reasons in the status text.
static void test_busy_unsupported_why() {
  Chain c(1);
  c.rx[0].busy_energy = 2;
  c.rx[0].why = ULTRA_OU_W_SOFT_START | ULTRA_OU_W_SIGNAL_LOSS | ULTRA_OU_W_TEST;
  TelemetryPoller p = started(c, 1);
  p.set_outputs(0, true);
  c.run(p, 5000);
  CHECK(p.data(0).have_energy && c.rx[0].asked[ULTRA_PAGE_ENERGY] == 3, "ENERGY BUSY twice, then read (%u)",
        c.rx[0].asked[ULTRA_PAGE_ENERGY]);
  CHECK(telemetry_status_text(p.data(0)) == "soft start, signal loss, calibration / test jig", "status '%s'",
        telemetry_status_text(p.data(0)).c_str());
  Chain c2(1);
  c2.rx[0].outputs_page = false;
  c2.rx[0].features = 0;   // a receiver without the optional pages
  TelemetryPoller p2 = started(c2, 1);
  p2.set_outputs(0, true);
  c2.run(p2, 30000);
  const ReceiverData &d = p2.data(0);
  CHECK(c2.rx[0].asked[ULTRA_PAGE_OUTPUTS] == 1 && c2.rx[0].asked[ULTRA_PAGE_POWER] == 0 &&
            c2.rx[0].asked[ULTRA_PAGE_ENERGY] == 0 && std::isnan(telemetry_value(d, TV_ENERGY)) &&
            std::isnan(telemetry_value(d, TV_SHORT_TRIPS)) && telemetry_status_text(d) == "ok",
        "UNSUPPORTED asked once; pages without their feature never; missing values NAN");
}

/// More than 16 receivers: at most 16 answer per window, relay only in front of the last one; all of them read.
static void test_long_chain() {
  Chain c(20);
  TelemetryPoller p = started(c, 20, ULTRA_RW_RATE_500K);
  c.run(p, 30000);
  bool all = true;
  for (uint16_t i = 0; i < 20; i++)
    all &= p.answering(i) && p.data(i).have_identity && p.data(i).have_status;
  CHECK(all && c.plan_ok, "20 receivers in windows of up to 16: all read (%s)", c.plan_err.c_str());
  CHECK(p.max_window_us() == 1000 + 16 * 790, "the longest window at 500 kbaud: %u us", p.max_window_us());
  std::printf("  20 receivers at 500 kbaud, 30 s: %u RW frames, %u answers, none missing (%u)\n", c.frames,
              p.stats().answers, p.stats().missing);
}

/// The supply off: everything unavailable; after the preamble and 1 s the polls resume and see the restart.
static void test_unpowered() {
  Chain c(2);
  TelemetryPoller p = started(c, 2);
  c.run(p, 5000);
  p.unpowered();
  CHECK(!p.answering(0) && !p.answering(1) && !p.due(c.now_ms + 5000), "unpowered: unavailable, no polls");
  c.now_ms += 20000;
  for (auto &m : c.rx)
    m.restart();
  p.locked(c.now_ms);
  c.run(p, 4000);
  CHECK(p.answering(0) && p.answering(1) && p.data(0).reset_cause == ULTRA_ST_RST_WATCHDOG &&
            c.rx[0].asked[ULTRA_PAGE_IDENTITY] == 2,
        "power back: answering, the restart seen, IDENTITY again");
}

/// SEQ and millis() wraps over a long run.
/// The Home Assistant side: the Status (worst first), its Problem, the plain-language line, a light's status; OUTPUTS
/// only for the receivers it's switched on for; the missed-poll statistic; a light's estimated power.
static void test_status() {
  // OUTPUTS: only where switched on (a receiver with several lights)
  Chain c(2);
  TelemetryPoller p = started(c, 2);
  p.set_outputs(1, true);
  c.run(p, 12000);
  CHECK(c.rx[0].asked[ULTRA_PAGE_OUTPUTS] == 0 && c.rx[1].asked[ULTRA_PAGE_OUTPUTS] >= 2,
        "OUTPUTS only from the receiver with several lights (%u, %u)", c.rx[0].asked[ULTRA_PAGE_OUTPUTS],
        c.rx[1].asked[ULTRA_PAGE_OUTPUTS]);
  CHECK(p.polls(0) > 3 && p.missed(0) == 0, "missed polls: 0 of %u", p.polls(0));
  c.rx[0].drop_next = true;
  c.now_ms += 4000;
  c.window(p);
  CHECK(p.missed(0) == 1 && p.missed(1) == 1, "a silent receiver: a missed poll for it and the one behind (%u %u)",
        p.missed(0), p.missed(1));

  // the statuses, worst first
  ReceiverData d;
  d.have_status = true;
  d.mcu_temp = 412;
  CHECK(receiver_status(d, false) == RS_NO_ANSWER && receiver_status(d, true) == RS_OK, "no answer / OK");
  CHECK(receiver_status_detail(d, RS_NO_ANSWER, 2, -1) ==
            "No answer. Receivers 1 and 2 before it answer, so check its cable or power." &&
            receiver_status_detail(d, RS_NO_ANSWER, 3, 1) ==
                "No answer: receiver 2 before it doesn't answer, so this one can't be heard. Check that one first." &&
            receiver_status_detail(d, RS_NO_ANSWER, 0, -1).find("RS-485") != std::string::npos,
        "no answer: '%s'", receiver_status_detail(d, RS_NO_ANSWER, 2, -1).c_str());
  d.have_power = true;
  d.power_state = ULTRA_ST_LIMITING;
  d.limit_ma = 2000;
  d.limit_source = ULTRA_PW_SRC_USER;
  d.factor = 57671;   // 88 %
  CHECK(receiver_status(d, true) == RS_LIMITING && !receiver_status_problem(RS_LIMITING) &&
            receiver_status_detail(d, RS_LIMITING, 1, -1) ==
                "The board limit (2.0 A, user limit) is reducing the outputs by about 12 %.",
        "limiting: '%s'", receiver_status_detail(d, RS_LIMITING, 1, -1).c_str());
  d.have_supply = true;
  d.dips = 3;
  d.logged = 3;
  d.dip_age_s = 4000;
  d.dip_mv = 39800;
  d.dip_pct = 86;
  d.dip_ms = 35;
  d.dip_ma = 1000;
  CHECK(receiver_status(d, true) == RS_SUPPLY_DIPS && receiver_status_problem(RS_SUPPLY_DIPS) &&
            receiver_status_detail(d, RS_SUPPLY_DIPS, 1, -1) ==
                "3 supply dips in the last 24 h, the last to 39.8 V (86 %) for 35 ms at 1.0 A. The supply or its "
                "cable may be too weak." &&
            light_status(d, true, 0x01) == RS_LIMITING,
        "supply dips over limiting: '%s'", receiver_status_detail(d, RS_SUPPLY_DIPS, 1, -1).c_str());
  d.dip_age_s = 90000;   // older than a day
  CHECK(receiver_status(d, true) == RS_LIMITING, "dips older than a day: no longer a problem");
  d.mcu_temp = 880;
  CHECK(receiver_status(d, true) == RS_TOO_HOT && receiver_status_detail(d, RS_TOO_HOT, 0, -1).find("88.0") != std::string::npos,
        "too hot: '%s'", receiver_status_detail(d, RS_TOO_HOT, 0, -1).c_str());
  d.mcu_temp = 412;
  d.power_state = ULTRA_ST_TRIPPED;
  d.short_trips = 1;
  d.trip_cause = ULTRA_PW_CAUSE_SHORT;
  d.trip_ma = 12400;
  d.trip_age_s = 3;
  d.retry_s = 8;
  CHECK(receiver_status(d, true) == RS_TRIPPED && std::string(receiver_status_name(RS_TRIPPED)) == "Tripped" &&
            receiver_status_detail(d, RS_TRIPPED, 0, -1) ==
                "Tripped by a short circuit at 12.40 A; retry in 8 s. Check the wiring of its outputs." &&
            telemetry_last_trip(d) == "short circuit at 12.40 A",
        "tripped: '%s'", receiver_status_detail(d, RS_TRIPPED, 0, -1).c_str());
  d.power_state = 0;
  d.retry_s = 0;
  d.trip_age_s = 300;   // on again, a trip 5 min ago: still shown
  CHECK(receiver_status(d, true) == RS_TRIPPED && receiver_status_detail(d, RS_TRIPPED, 0, -1).find("on again") !=
                                                      std::string::npos,
        "a recent trip stays: '%s'", receiver_status_detail(d, RS_TRIPPED, 0, -1).c_str());
  d.trip_age_s = 700;
  d.have_outputs = true;
  d.why = ULTRA_OU_W_SOFT_START;
  CHECK(receiver_status(d, true) == RS_SOFT_START && light_status(d, true, 0x01) == RS_SOFT_START &&
            std::string(receiver_status_name(RS_SOFT_START)) == "Soft start",
        "soft start (OUTPUTS)");
  d.why = ULTRA_OU_W_SIGNAL_LOSS | ULTRA_OU_W_TEST;
  CHECK(receiver_status(d, true) == RS_SIGNAL_LOSS && receiver_status_problem(RS_SIGNAL_LOSS), "signal loss first");
  d.why = ULTRA_OU_W_TEST;
  CHECK(receiver_status(d, true) == RS_CALIBRATING && telemetry_last_dip(d) == "to 39.80 V (86 %) for 35 ms at 1.00 A",
        "calibrating; last dip '%s'", telemetry_last_dip(d).c_str());

  // a light's estimated power: the measured input power, split by the model's channel currents
  ReceiverData o;
  o.have_status = true;
  o.measured = ULTRA_ST_M_VBUS;   // (no current measured: ENERGY's power; with it, STATUS's voltage x current)
  o.vbus_10mv = 4790;
  o.have_energy = true;
  o.power_mw = 34000;
  CHECK(std::isnan(light_power_estimate(o, 0x01)), "no OUTPUTS yet: unknown");
  o.have_outputs = true;
  const uint16_t ma[5] = {250, 250, 240, 255, 410};
  for (int i = 0; i < 5; i++)
    o.ch_ma[i] = ma[i];
  const float p1 = light_power_estimate(o, 0x01), p5 = light_power_estimate(o, 0x10), all = light_power_estimate(o, 0x1F);
  CHECK(std::fabs(p1 - 34.0f * 250 / 1405) < 1e-3f && std::fabs(p5 - 34.0f * 410 / 1405) < 1e-3f &&
            std::fabs(all - 34.0f) < 1e-3f,
        "estimated power: CH1 %.2f W, CH5 %.2f W, all %.2f W of 34 W", p1, p5, all);
  o.measured |= ULTRA_ST_M_CURRENT;
  o.current_ma = 500;
  CHECK(std::fabs(telemetry_value(o, TV_INPUT_POWER) - 23.95f) < 1e-3f, "Power: STATUS's 47.90 V x 0.500 A first (%.2f)",
        telemetry_value(o, TV_INPUT_POWER));
  for (auto &v : o.ch_ma)
    v = 0;
  CHECK(light_power_estimate(o, 0x01) == 0.0f, "every output off: 0 W");

  // its energy: 6 W for an hour in 5 s steps = 6 Wh; a gap of more than a minute isn't counted; unknown: nothing
  EnergyCounter e;
  uint32_t ms = 0xFFFFF000u;   // (millis() wraps meanwhile)
  for (int i = 0; i <= 720; i++, ms += 5000)
    e.add(6.0f, ms);
  const double hour = e.wh;
  e.add(NAN, ms);
  e.add(6.0f, ms + 5000);
  e.add(6.0f, ms + 5000 + 120000);   // a 2 min gap
  CHECK(std::fabs(hour - 6.0) < 1e-6 && e.wh == hour, "estimated energy: %.4f Wh for 6 W over an hour, gaps not "
        "counted (%.4f)", hour, e.wh);
}

/// The port's load (PortMonitor): the receivers' input currents against the limit, the power at the sender, the
/// cable loss along the chain (and per receiver its voltage drop, the segment before it); a learned sender voltage;
/// the warning, the problem at 100 % or held at 95 %; a receiver that doesn't answer: incomplete, no cable loss.
static void test_port() {
  Chain c(3);
  const uint16_t volts[3] = {5400, 5260, 5130};   // 54.0 / 52.6 / 51.3 V
  for (int k = 0; k < 3; k++) {
    c.rx[k].vbus = volts[k];
    c.rx[k].current = 540;   // 0.54 A each: 1.62, 1.08, 0.54 A through the segments
    c.rx[k].he_flags = ULTRA_HE_F_TESTED;   // (healthy channels: no receiver problem)
    c.rx[k].ch2_class = ULTRA_HE_NORMAL;
  }
  TelemetryPoller p = started(c, 3);
  c.run(p, 8000);
  PortMonitor m;
  PortMonitor::Config cfg;
  cfg.supply_v = 55.0f;
  m.configure(cfg);
  m.update(c.now_ms, p);
  const float loss = 1.0f * 1.62f + 1.4f * 1.08f + 1.3f * 0.54f;
  CHECK(m.complete() && std::fabs(m.current_a() - 1.62f) < 1e-4f && std::fabs(m.load_pct() - 72.0f) < 1e-3f &&
            std::fabs(m.power_w() - 89.1f) < 1e-3f && std::fabs(m.cable_loss_w() - loss) < 1e-3f &&
            std::fabs(m.cable_loss_pct() - loss / 89.1f * 100.0f) < 1e-3f && !m.warning() && !m.problem(),
        "port: %.2f A, %.0f %%, %.1f W at the sender, cable loss %.2f W (%.1f %%)", m.current_a(), m.load_pct(),
        m.power_w(), m.cable_loss_w(), m.cable_loss_pct());
  CHECK(std::fabs(m.drop_v(0) - 1.0f) < 1e-3f && std::fabs(m.drop_v(2) - 3.7f) < 1e-3f &&
            std::fabs(m.resistance(0) - 1.0f / 1.62f) < 1e-4f && std::fabs(m.resistance(1) - 1.4f / 1.08f) < 1e-4f &&
            std::fabs(m.resistance(2) - 1.3f / 0.54f) < 1e-4f,
        "drop to receiver 3: %.2f V; segments %.0f / %.0f / %.0f mOhm", m.drop_v(2), m.resistance(0) * 1000,
        m.resistance(1) * 1000, m.resistance(2) * 1000);
  CHECK(port_status(m, p) == "OK" && port_status_detail(m, c.now_ms) == "OK.", "port status: '%s'",
        port_status(m, p).c_str());

  // the sender's voltage learned: receiver 1's highest
  PortMonitor l;
  l.update(c.now_ms, p);
  c.rx[0].vbus = 5480;
  c.run(p, 4000);
  l.update(c.now_ms, p);
  c.rx[0].vbus = 5400;
  c.run(p, 4000);
  l.update(c.now_ms, p);
  CHECK(std::fabs(l.supply_v() - 54.8f) < 1e-3f && std::fabs(l.power_w() - 54.8f * 1.62f) < 1e-3f,
        "learned sender voltage %.2f V", l.supply_v());

  // near the limit: a warning at 91 %; 96 % held for 30 s: a problem; 101 %: at once
  for (auto &r : c.rx)
    r.current = 683;   // 2.05 A: 91 %
  c.run(p, 4000);
  m.update(c.now_ms, p);
  CHECK(m.warning() && !m.problem() && port_status(m, p) == "91 % load" &&
            port_status_detail(m, c.now_ms) == "2.05 A of 2.25 A (91 %): near the port's limit.",
        "91 %%: '%s' / '%s'", port_status(m, p).c_str(), port_status_detail(m, c.now_ms).c_str());
  for (auto &r : c.rx)
    r.current = 720;   // 2.16 A: 96 %
  bool early = false;
  for (int s = 0; s < 40; s++) {
    c.run(p, 1000);
    m.update(c.now_ms, p);
    early |= s < 25 && m.problem();
  }
  CHECK(!early && m.problem() && port_status_detail(m, c.now_ms).find("may trip") != std::string::npos,
        "96 %%: a problem only after 30 s ('%s')", port_status_detail(m, c.now_ms).c_str());
  for (auto &r : c.rx)
    r.current = 760;   // 2.28 A: 101 %
  PortMonitor o;
  o.configure(cfg);
  c.run(p, 4000);
  o.update(c.now_ms, p);
  CHECK(o.over() && o.problem(), "101 %%: a problem at once (%.0f %%)", o.load_pct());

  // a receiver that doesn't answer: incomplete (the others' current), no cable loss, the line names it
  for (auto &r : c.rx)
    r.current = 540;
  c.rx[2].has_channel = false;
  c.run(p, 15000);
  m.update(c.now_ms, p);
  CHECK(!m.complete() && m.missing() == 2 && std::fabs(m.current_a() - 1.08f) < 1e-4f && std::isnan(m.cable_loss_w()) &&
            port_status(m, p) == "No answer" &&
            port_status_detail(m, c.now_ms) == "Receiver 3 doesn't report: the port's current and power are without it.",
        "incomplete: %.2f A, '%s' / '%s'", m.current_a(), port_status(m, p).c_str(),
        port_status_detail(m, c.now_ms).c_str());
}

/// Protocol 0.2.5: LIFETIME and DIAG's two parts read after IDENTITY (and again after a restart), not from a board
/// without them; the lifetime counts in place of the since-power-up ones; ENERGY's LED supply and peak (0 before
/// 0.2.5: unknown); the texts from the fault memory; the Status from STATUS's reason and HEALTH's flags; a light's
/// status from its own channels; the port's peak load.
static void test_025() {
  Chain c(3);
  c.rx[0].set025();
  c.rx[1].set025();   // receiver 2 (index 2): a 0.6.8 board
  c.rx[0].fm_trip = 2 | 13 << 2;   // an overload at 6.5 A at 1100 lit hours, before this power-up
  c.rx[0].fm_trip_h = 1100;
  c.rx[0].fm_dip_pct = 81;
  c.rx[0].fm_rst = 1;   // a watchdog reset, 12 h up
  c.rx[1].reset = ULTRA_ST_RST_SOFTWARE;
  c.rx[1].bc_cause = 2;   // the run before: an illegal instruction
  c.rx[1].he_flags = ULTRA_HE_F_TESTED;
  c.rx[1].ch2_class = ULTRA_HE_NORMAL;
  TelemetryPoller p = started(c, 3);
  c.run(p, 20000);
  for (int k = 0; k < 2; k++) {
    CHECK(c.rx[k].asked[ULTRA_PAGE_LIFETIME] == 1 && c.rx[k].diag_parts[0] == 1 && c.rx[k].diag_parts[1] == 1 &&
              c.rx[k].diag_parts[2] == 0 && p.data(k).have_lifetime && p.data(k).have_diag && p.data(k).have_faults,
          "receiver %d: LIFETIME and DIAG parts 0 / 1 read once after IDENTITY (%u, %u / %u / %u)", k,
          c.rx[k].asked[ULTRA_PAGE_LIFETIME], c.rx[k].diag_parts[0], c.rx[k].diag_parts[1], c.rx[k].diag_parts[2]);
  }
  CHECK(c.rx[2].asked[ULTRA_PAGE_LIFETIME] == 0 && c.rx[2].asked[ULTRA_PAGE_DIAG] == 0 && !p.data(2).have_lifetime,
        "a board without the features: never asked");
  const ReceiverData &d0 = p.data(0), &d2 = p.data(2);
  CHECK(std::fabs(telemetry_value(d0, TV_LED_SUPPLY) - 24.0f) < 1e-3f && telemetry_value(d0, TV_CURRENT_PEAK) == 0.0f &&
            telemetry_value(d0, TV_SUPPLY_IMPEDANCE) == 95.0f &&
            std::fabs(telemetry_value(d0, TV_LIT_HOURS) - 1234.5f) < 1e-2f &&
            telemetry_value(d0, TV_POWER_UPS) == 41.0f && telemetry_value(d0, TV_TRIPS) == 4.0f &&
            telemetry_value(d0, TV_DIPS) == 2.0f,
        "0.2.5 values: LED %.2f V, peak %.3f A, Z %.0f mOhm, lit %.1f h, %.0f power-ups, %.0f trips, %.0f dips",
        telemetry_value(d0, TV_LED_SUPPLY), telemetry_value(d0, TV_CURRENT_PEAK),
        telemetry_value(d0, TV_SUPPLY_IMPEDANCE), telemetry_value(d0, TV_LIT_HOURS), telemetry_value(d0, TV_POWER_UPS),
        telemetry_value(d0, TV_TRIPS), telemetry_value(d0, TV_DIPS));
  CHECK(std::isnan(telemetry_value(d2, TV_LED_SUPPLY)) && std::isnan(telemetry_value(d2, TV_CURRENT_PEAK)) &&
            std::isnan(telemetry_value(d2, TV_LIT_HOURS)) && telemetry_value(d2, TV_TRIPS) == 0.0f &&
            telemetry_value(d2, TV_DIPS) == 0.0f,
        "a 0.6.8 board: ENERGY's zeros unknown, its trips / dips since its power-up");
  CHECK(telemetry_last_trip(d0) == "overload at 6.5 A (at 1100 lit hours, before its last power-up)" &&
            telemetry_last_dip(d0) == "to 81 % for 30 ms at 1.2 A (before its last power-up)" &&
            telemetry_last_reset(d0) == "power-on; the last abnormal one: watchdog, 12 h up" &&
            telemetry_last_reset(p.data(1)) ==
                "software; the run before: up 3.5 h, exception 2 (illegal instruction) at 0x1A2C" &&
            telemetry_last_trip(d2) == "none" && telemetry_last_reset(d2) == "power-on",
        "texts: '%s' / '%s' / '%s' / '%s'", telemetry_last_trip(d0).c_str(), telemetry_last_dip(d0).c_str(),
        telemetry_last_reset(d0).c_str(), telemetry_last_reset(p.data(1)).c_str());
  CHECK(telemetry_health_text(d0) ==
                "CH2 abnormal (current didn't rise with the level); trend over 4 tests and 1200 lit h: CH1 -3 %, CH3 +1 %" &&
            telemetry_health_text(d2) == "CH2 abnormal (current didn't rise with the level)",
        "health: '%s'", telemetry_health_text(d0).c_str());
  // the Status: a channel fault (HEALTH) is a problem; a light on other channels doesn't show it
  CHECK(receiver_status(d0, true) == RS_CHANNEL_FAULT && receiver_status_problem(RS_CHANNEL_FAULT) &&
            receiver_status_detail(d0, RS_CHANNEL_FAULT, 0, -1) ==
                "CH2 is abnormal at the last calibration test (the current didn't rise with the level): check its LEDs "
                "and wiring." &&
            light_status(d0, true, 0x01) == RS_OK && light_status(d0, true, 0x02) == RS_CHANNEL_FAULT &&
            receiver_status(p.data(1), true) == RS_OK,
        "channel fault: '%s'", receiver_status_detail(d0, RS_CHANNEL_FAULT, 0, -1).c_str());
  // STATUS's reason, read every few seconds
  struct Case {
    uint8_t why, state, he;
    ReceiverStatus rs;
    const char *detail;
  };
  const Case cases[] = {
      {ULTRA_ST_WHY_RETRIES_OUT, ULTRA_ST_TRIPPED, 0, RS_TRIPPED, "retries are used up"},
      {ULTRA_ST_WHY_SHORT, ULTRA_ST_TRIPPED, 0, RS_TRIPPED, "Tripped by a short circuit"},
      {ULTRA_ST_WHY_HOT, 0, 0, RS_TOO_HOT, "over its warning level (41.2"},
      {ULTRA_ST_WHY_NONE, 0, ULTRA_HE_F_HOT, RS_TOO_HOT, "over its warning level"},
      {ULTRA_ST_WHY_SUPPLY, 0, 0, RS_SUPPLY_DIPS, "in the last 30 s"},
      {ULTRA_ST_WHY_SIGNAL_LOSS, 0, 0, RS_SIGNAL_LOSS, "signal_loss setting"},
      {ULTRA_ST_WHY_NONE, 0, ULTRA_HE_F_CAL_SUGGESTED, RS_CAL_SUGGESTED, "calibration test is suggested"},
      {ULTRA_ST_WHY_TEST, 0, 0, RS_CALIBRATING, "test jig"},
      {ULTRA_ST_WHY_CALIBRATING, 0, 0, RS_CALIBRATING, "calibration test is running"},
      {ULTRA_ST_WHY_SOFT_START, 0, 0, RS_SOFT_START, "ramp up"},
      {ULTRA_ST_WHY_DARK_HOLD, 0, 0, RS_HELD_DARK, "Held dark"},
      {ULTRA_ST_WHY_PATTERN, 0, 0, RS_BLINKING, "blink pattern"},
  };
  Model &m = c.rx[1];
  for (const Case &k : cases) {
    m.why025 = k.why;
    m.state = k.state;
    m.he_flags = static_cast<uint8_t>(ULTRA_HE_F_TESTED | k.he);
    TelemetryPoller q = started(c, 2);
    c.run(q, 6000);
    const ReceiverData &d = q.data(1);
    const ReceiverStatus rs = receiver_status(d, true);
    const std::string det = receiver_status_detail(d, rs, 1, -1);
    const bool dims = k.rs != RS_TOO_HOT && k.rs != RS_SUPPLY_DIPS && k.rs != RS_CAL_SUGGESTED;
    CHECK(rs == k.rs && det.find(k.detail) != std::string::npos &&
              light_status(d, true, 0x01) == (dims ? k.rs : RS_OK) &&
              receiver_status_problem(rs) == (k.rs <= RS_CAL_SUGGESTED),
          "why %u / health %02X: '%s' (%s), the light's '%s'", k.why, k.he, receiver_status_name(rs), det.c_str(),
          receiver_status_name(light_status(d, true, 0x01)));
  }
  m.why025 = 0;
  m.state = 0;
  m.he_flags = ULTRA_HE_F_TESTED;
  // a watchdog restart: DIAG (the breadcrumbs) read again at once, with IDENTITY
  const uint32_t parts0 = c.rx[0].diag_parts[0];
  c.rx[0].restart();
  c.rx[0].bc_cause = 0;
  c.run(p, 8000);
  CHECK(c.rx[0].diag_parts[0] == parts0 + 1 && p.data(0).bc_cause == 0 &&
            telemetry_last_reset(p.data(0)) == "watchdog; the run before: up 3.5 h, no exception",
        "after a restart: DIAG read again ('%s')", telemetry_last_reset(p.data(0)).c_str());

  // the port's peak load: the receivers' peaks since their ENERGY reads before, added up
  Chain e(2);
  for (auto &r : e.rx) {
    r.set025();
    r.he_flags = ULTRA_HE_F_TESTED;
    r.ch2_class = ULTRA_HE_NORMAL;
    r.current = 540;
    r.peak = 1200;
  }
  TelemetryPoller q = started(e, 2);
  e.run(q, 8000);
  PortMonitor pm;
  pm.update(e.now_ms, q);
  CHECK(std::fabs(pm.peak_a() - 2.4f) < 1e-4f && pm.peaks() && port_status(pm, q) == "Peaks 107 %" &&
            port_status_detail(pm, e.now_ms).find("Peaks up to 2.40 A (107 %) in the last minute, 1.08 A on average") ==
                0,
        "peaks: %.2f A, '%s' / '%s'", pm.peak_a(), port_status(pm, q).c_str(), port_status_detail(pm, e.now_ms).c_str());
  e.rx[1].features &= ~(ULTRA_FEAT_LIFETIME | ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE);
  for (auto &b : e.rx[1].uid)
    b ^= 0x33;   // (a 0.6.8 board swapped in: IDENTITY again)
  e.run(q, 8000);
  pm.update(e.now_ms, q);
  CHECK(std::fabs(pm.peak_a() - 1.74f) < 1e-4f && !pm.peaks() && port_status(pm, q) == "OK",
        "a board without the peak counts with its average: %.2f A", pm.peak_a());
}

/// Energy never goes back (Home Assistant would count a lower total_increasing value as a reset: the whole total again):
/// the hub's total grows by the receiver's mWh since its power-up; a wake that brings back an older stored lifetime, a
/// restart, the ESP32's restart (its stored total), another board at the position: never lower, nothing twice.
static void test_energy() {
  TelemetryIntervals iv;
  iv.energy_ms = 2000;
  Chain c(1);
  TelemetryPoller p = started(c, 1, ULTRA_RW_RATE_1M, iv);
  std::vector<float> seen;
  auto run = [&](TelemetryPoller &q, uint32_t ms) {
    c.run(q, ms);
    seen.push_back(telemetry_value(q.data(0), TV_ENERGY));
  };
  run(p, 8000);
  CHECK(std::fabs(seen.back() - 1234.0f) < 1e-3f, "first answer: the receiver's lifetime (%.3f Wh)", seen.back());
  c.rx[0].boot_mwh += 2500;   // 2.5 Wh used
  c.rx[0].total_wh += 2;
  run(p, 8000);
  CHECK(std::fabs(seen.back() - 1236.5f) < 1e-3f, "+2.5 Wh (its count since power-up, in mWh): %.3f", seen.back());
  // a wake: the board lost what it hadn't stored (lifetime 1230 again), a new power-up
  c.rx[0].restart();
  c.rx[0].total_wh = 1230;
  c.rx[0].boot_mwh = 300;
  run(p, 8000);
  CHECK(std::fabs(seen.back() - 1236.8f) < 1e-3f, "a wake with an older stored lifetime: +0.3 Wh, not back (%.3f)",
        seen.back());
  c.rx[0].boot_mwh = 1300;
  run(p, 8000);
  CHECK(std::fabs(seen.back() - 1237.8f) < 1e-3f, "then +1.0 Wh (%.3f)", seen.back());
  // a power-off that didn't cut it (a supply kept on): its count goes on: only the new mWh, never its session again
  p.unpowered();
  p.locked(c.now_ms);   // (powered again: the preamble locks the chain)
  c.rx[0].boot_mwh = 1400;
  run(p, 8000);
  CHECK(std::fabs(seen.back() - 1237.9f) < 1e-3f, "a power-off without a cut: +0.1 Wh, not its session twice (%.3f)",
        seen.back());
  // the ESP32 restarts: its stored total seeds the new poller; the receiver's lifetime is lower
  const uint64_t stored = p.energy_mwh(0);
  TelemetryPoller q = started(c, 1, ULTRA_RW_RATE_1M, iv);
  q.seed_energy(0, stored);
  run(q, 8000);
  CHECK(std::fabs(seen.back() - 1237.9f) < 1e-3f, "after the ESP32's restart: the stored total (%.3f)", seen.back());
  c.rx[0].boot_mwh += 200;
  run(q, 8000);
  CHECK(std::fabs(seen.back() - 1238.1f) < 1e-3f, "and on from there (%.3f)", seen.back());
  // ... and with the receiver ahead (it stored more while the hub was off): its lifetime
  TelemetryPoller r = started(c, 1, ULTRA_RW_RATE_1M, iv);
  r.seed_energy(0, 1000000);
  c.rx[0].total_wh = 1500;
  run(r, 8000);
  CHECK(std::fabs(seen.back() - 1500.0f) < 1e-3f, "the receiver's lifetime ahead of the stored total (%.3f)", seen.back());
  // another board at the position (a new UID), its own large count: the base, nothing added for it
  for (auto &b : c.rx[0].uid)
    b ^= 0x5A;
  c.rx[0].boot_mwh = 900000;
  c.rx[0].total_wh = 50;
  run(r, 8000);
  const float before = seen.back();
  c.rx[0].boot_mwh += 100;
  run(r, 8000);
  CHECK(std::fabs(before - 1500.0f) < 1e-3f && std::fabs(seen.back() - 1500.1f) < 1e-3f,
        "another board: nothing added for its count (%.3f), then its use (%.3f)", before, seen.back());
  bool never_back = true;
  for (size_t i = 1; i < seen.size(); i++)
    never_back = never_back && (seen[i] >= seen[i - 1] || i == 7);   // (7: a new poller seeded higher, not lower)
  CHECK(never_back, "Energy never went back");
  // negative control: the old rule (the receiver's lifetime, at most up) would have shown 1230 after the restart
  std::printf("  Energy: never back after a wake, a restart, the ESP32's restart, another board\n");
}

/// (0.2.6) DIAG 20 bit 7: the receiver's lifetime record held: a note after the Status detail, no problem
static void test_record_held() {
  Chain c(1);
  c.rx[0].set025();
  c.rx[0].he_flags = ULTRA_HE_F_TESTED;   // (healthy: its status only shows what this note does)
  c.rx[0].ch2_class = ULTRA_HE_NORMAL;
  c.rx[0].flash_pending = ULTRA_DG_FP_EXHAUSTED;
  TelemetryPoller p = started(c, 1);
  c.run(p, 8000);
  const ReceiverData &d = p.data(0);
  CHECK(d.have_diag && receiver_status_notes(d) ==
                           " Lifetime record held: the receiver saves less often to protect its flash." &&
            !receiver_status_problem(receiver_status(d, true)),
        "the record held: '%s', no problem", receiver_status_notes(d).c_str());
  Chain n(1);
  n.rx[0].set025();
  TelemetryPoller q = started(n, 1);
  n.run(q, 8000);
  CHECK(q.data(0).have_diag && receiver_status_notes(q.data(0)).empty(), "not held: no note");
}

static void test_wraps() {
  Chain c(2);
  c.now_ms = 0xFFFF0000u;   // millis() wraps during the run
  TelemetryPoller p = started(c, 2, ULTRA_RW_RATE_250K, TelemetryIntervals{500, 500, 2000, 2000, 2000});
  c.run(p, 200000);
  CHECK(c.frames > 300 && p.stats().missing == 0 && p.answering(0) && p.answering(1) && p.stats().restarts == 0,
        "%u windows (SEQ wrapped), millis() wrapped: nothing missing, no false restart (%u)", c.frames,
        p.stats().restarts);
}

int main() {
  std::printf("ultra telemetry test (the return channel's hub side)\n");
  test_basic();
  test_parser();
  test_missing();
  test_restart_and_swap();
  test_power_ack();
  test_busy_unsupported_why();
  test_long_chain();
  test_unpowered();
  test_wraps();
  test_energy();
  test_record_held();
  test_status();
  test_port();
  test_025();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA TELEMETRY TESTS PASSED", failures);
  return failures ? 1 : 0;
}
