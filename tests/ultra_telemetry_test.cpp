// Host test of the return channel's hub side (components/dig2analog_ultra/ultra_telemetry.h) against a modelled
// chain of Diff-Power ultra receivers: the RW frames' POLL plans, the answers in a hub UART's byte stream with junk,
// missing and late answers, a receiver that never answers, restarts and swapped boards, the POWER acknowledge,
// BUSY / UNSUPPORTED pages, more than 16 receivers, the sensor values / texts, and protocol 0.2.5 (LIFETIME, DIAG's
// two parts, STATUS's reason, ENERGY's LED supply and peak, HEALTH's flags), and (0.2.6) the lifetime ledger and its
// LIFE_SYNC (components/dig2analog_ultra/ultra_ledger.h; design review, round 8, a replaced / moved board, a
// replaced controller). The receiver firmware's own answer path: tests/ultra_telemetry_contract_test.cpp.
//
// Build and run: python tests/run_ultra_telemetry_test.py

#include "../components/dig2analog_ultra/ultra_ledger.h"
#include "../components/dig2analog_ultra/ultra_pacing.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"

#include <cmath>
#include <cstring>
#include <chrono>
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
  uint32_t powered_01h{24690}, records{77};   // LIFETIME 0 / 21
  // LIFE_SYNC (0.2.6, feature bit 22): one applied merge per power-up; the caps (design review, round 8)
  bool merged{false};
  uint32_t syncs{0};
  uint8_t sync_result{0xFF}, sync_mask{0};
  std::vector<uint8_t> results;
  // (the tests: what changes on the board between the hub's read and its sync; a forced result)
  int records_bump_on_sync{0}, wh_bump_on_sync{0}, powered_bump_on_sync{0};
  uint8_t force_result{0xFF};
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
    this->merged = false;
  }
  /// used energy: its count since power-up and (whole Wh) its lifetime
  void use(uint32_t mwh) {
    const uint32_t before = this->boot_mwh / 1000;
    this->boot_mwh += mwh;
    this->total_wh += this->boot_mwh / 1000 - before;
  }
  /// a cut and a power-up: what it hadn't stored lost (lost_wh), a power-up more
  void power_up(uint32_t lost_wh = 0) {
    this->restart();
    this->reset = ULTRA_ST_RST_POWER;
    this->total_wh -= lost_wh;
    this->power_ups++;
  }
  /// LIFE_SYNC: the hub's ledger merged where it is ahead (in wire units), the caps, once per power-up
  void life_sync(const uint8_t *arg, uint8_t *d) {
    this->syncs++;
    if (this->records_bump_on_sync > 0) {   // (its record written since the hub's read)
      this->records_bump_on_sync--;
      this->records++;
    }
    if (this->wh_bump_on_sync > 0) {
      this->wh_bump_on_sync--;
      this->total_wh++;
    }
    this->powered_01h += this->powered_bump_on_sync;   // (its own time on since the read: 0.1 h units)
    this->powered_bump_on_sync = 0;
    uint8_t result = ULTRA_LS_R_APPLIED, mask = 0;
    const uint8_t *l = arg + ULTRA_LS_LIFETIME;
    const uint32_t a_pow = ultra_get24(l + ULTRA_LT_POWERED), a_lit = ultra_get24(l + ULTRA_LT_LIT),
                   a_ups = ultra_get24(l + ULTRA_LT_POWER_UPS), a_dips = ultra_get24(l + ULTRA_LT_DIPS);
    const uint16_t a_ovl = ultra_get16(l + ULTRA_LT_OVERLOAD_TRIPS), a_sh = ultra_get16(l + ULTRA_LT_SHORT_TRIPS);
    const uint32_t a_wh = ultra_get32(arg + ULTRA_LS_TOTAL_WH);
    auto dd = [](uint32_t a, uint32_t own) { return a > own && a - own < 0x800000u ? a - own : 0u; };
    const uint32_t d_pow = dd(a_pow, this->powered_01h), d_lit = dd(a_lit, this->lit_01h), d_ups = dd(a_ups, this->power_ups),
                   d_dips = dd(a_dips, this->lt_dips), d_ovl = dd(a_ovl, this->lt_overloads), d_sh = dd(a_sh, this->lt_shorts),
                   d_wh = dd(a_wh, this->total_wh);
    if (this->force_result != 0xFF) {
      result = this->force_result;
    } else if (ultra_get16(arg + ULTRA_LS_UID16) != this->uid16()) {
      result = ULTRA_LS_R_UID;
    } else if (arg[ULTRA_LS_RECORDS] != (this->records & 0xFF)) {
      result = ULTRA_LS_R_RECORDS;
    } else if (!(d_pow | d_lit | d_ups | d_dips | d_ovl | d_sh | d_wh)) {
      result = ULTRA_LS_R_NOOP;   // (doesn't use up the merge)
    } else if (this->merged) {
      result = ULTRA_LS_R_ONCE;
    } else {
      const uint32_t p_ok = d_pow <= 7200 ? d_pow : 0;   // 30 days
      const uint32_t ev_cap = p_ok * 36 + 64;            // (Δpowered_s / 10 + 64)
      auto take = [&](uint32_t dv, uint32_t cap, uint32_t bit, auto &field) {
        if (dv == 0)
          return;
        if (dv > cap) {
          result = ULTRA_LS_R_CAPPED;
          return;
        }
        field += dv;
        mask |= bit;
      };
      take(d_pow, 7200, ULTRA_LS_F_POWERED, this->powered_01h);
      take(d_lit, p_ok + 1, ULTRA_LS_F_LIT, this->lit_01h);
      take(d_ups, ev_cap, ULTRA_LS_F_UPS, this->power_ups);
      take(d_ovl, ev_cap, ULTRA_LS_F_TRIPS, this->lt_overloads);
      take(d_sh, ev_cap, ULTRA_LS_F_TRIPS, this->lt_shorts);
      take(d_dips, ev_cap, ULTRA_LS_F_DIPS, this->lt_dips);
      take(d_wh, p_ok * 100 + 1000, ULTRA_LS_F_WH, this->total_wh);   // (energy.c: 1 kW over Δpowered + 1 h)
      if (mask != 0)
        this->merged = true;
    }
    this->sync_result = result;
    this->sync_mask = mask;
    this->results.push_back(result);
    this->lifetime(d);
    ultra_put32(d + ULTRA_LS_A_TOTAL_WH, this->total_wh);
    d[ULTRA_LS_A_RESULT] = result;
    d[ULTRA_LS_A_APPLIED] = mask;
    ultra_put16(d + ULTRA_LS_A_RECORDS, static_cast<uint16_t>(this->records));
  }
  void lifetime(uint8_t *d) const {
    ultra_put24(d + ULTRA_LT_POWERED, this->powered_01h);
    ultra_put24(d + ULTRA_LT_LIT, this->lit_01h);
    ultra_put24(d + ULTRA_LT_POWER_UPS, this->power_ups);
    ultra_put16(d + ULTRA_LT_OVERLOAD_TRIPS, this->lt_overloads);
    ultra_put16(d + ULTRA_LT_SHORT_TRIPS, this->lt_shorts);
    ultra_put24(d + ULTRA_LT_DIPS, this->lt_dips);
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
        this->lifetime(d);
        ultra_put24(d + ULTRA_LT_RECORDS, this->records);
        break;
      case ULTRA_PAGE_LIFE_SYNC:
        if (!(this->features & ULTRA_FEAT_LIFE_SYNC)) {
          out_page = ULTRA_PAGE_UNSUPPORTED;
          break;
        }
        this->life_sync(arg, d);
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
  uint64_t frame_hash{1469598103934665603ull};   // (FNV-1a of every build(): its slots, W, the boards)
  uint16_t window(TelemetryPoller &p) {
    std::vector<uint8_t> slots(ULTRA_SLOT_BYTES * (rx.size() + 1), 0);
    uint32_t w = 0;
    const uint16_t n = p.build(now_ms, slots.data(), &w);
    if (n == 0)
      return 0;
    for (size_t b = 0; b < ULTRA_SLOT_BYTES * static_cast<size_t>(n); b++)
      frame_hash = (frame_hash ^ slots[b]) * 1099511628211ull;
    frame_hash = (frame_hash ^ w ^ (static_cast<uint64_t>(n) << 32)) * 1099511628211ull;
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
              std::fabs(telemetry_value(d, TV_ENERGY) - 1234.567f) < 1e-3f && telemetry_value(d, TV_UPTIME) <= m.uptime &&
              telemetry_value(d, TV_UPTIME) + 4 >= m.uptime,
          "receiver %u: V %.2f I %.3f P %.2f T %.1f E %.3f up %.0f", i, telemetry_value(d, TV_SUPPLY_VOLTAGE),
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
                "The board limit (2.0 A, user limit) is reducing the outputs by about 10 %.",
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
  CHECK(receiver_status(d, true) == RS_TOO_HOT && receiver_status_detail(d, RS_TOO_HOT, 0, -1).find("85 \xC2\xB0" "C or more") != std::string::npos &&
            receiver_status_detail(d, RS_TOO_HOT, 0, -1).find("88") == std::string::npos,
        "too hot, without its live temperature (R14): '%s'", receiver_status_detail(d, RS_TOO_HOT, 0, -1).c_str());
  d.mcu_temp = 412;
  d.power_state = ULTRA_ST_TRIPPED;
  d.short_trips = 1;
  d.trip_cause = ULTRA_PW_CAUSE_SHORT;
  d.trip_ma = 12400;
  d.trip_age_s = 3;
  d.retry_s = 8;
  CHECK(receiver_status(d, true) == RS_TRIPPED && std::string(receiver_status_name(RS_TRIPPED)) == "Tripped" &&
            receiver_status_detail(d, RS_TRIPPED, 0, -1) ==
                "Tripped by a short circuit at 12.40 A; it tries again by itself. Check the wiring of its outputs." &&
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
/// the warning from 80 %, the problem at 100 % or held at 90 % (the defaults; and a port's own 90 / 95 %); a receiver
/// that doesn't answer: incomplete, no cable loss.
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
  CHECK(port_status(m, p) == "OK" && port_status_detail(m) == "OK.", "port status: '%s'",
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

  // near the limit, the defaults: a warning at 81 %; 88 % for 40 s: no problem; 91 % held for 30 s: a problem;
  // 101 %: at once. A port with its own (load_warning 90 %, load_problem 95 %): no warning at 81 / 88 %, at 91 % a
  // warning only; 96 % held for 30 s: a problem
  PortMonitor u;
  PortMonitor::Config own = cfg;
  own.warning_pct = 90.0f;
  own.problem_pct = 95.0f;
  u.configure(own);
  const auto load = [&](uint16_t ma) {
    for (auto &r : c.rx)
      r.current = ma;
  };
  load(608);   // 1.82 A: 81 %
  c.run(p, 4000);
  m.update(c.now_ms, p);
  u.update(c.now_ms, p);
  CHECK(m.warning() && !m.problem() && port_status(m, p) == "80 % load" &&
            port_status_detail(m) == "1.8 A of 2.25 A (80 %): near the port's limit." && !u.warning(),
        "81 %%: '%s' / '%s' (own 90 %%: %s)", port_status(m, p).c_str(), port_status_detail(m).c_str(),
        u.warning() ? "a warning" : "none");
  load(660);   // 1.98 A: 88 %
  bool any = false, own_any = false;
  for (int s = 0; s < 40; s++) {
    c.run(p, 1000);
    m.update(c.now_ms, p);
    u.update(c.now_ms, p);
    any |= m.problem();
    own_any |= u.warning() || u.problem();
  }
  CHECK(m.warning() && !any && !own_any, "88 %% for 40 s: a warning, no problem (own: %s)",
        own_any ? "a warning or problem" : "none");
  load(683);   // 2.05 A: 91 %
  bool early = false;
  for (int s = 0; s < 40; s++) {
    c.run(p, 1000);
    m.update(c.now_ms, p);
    u.update(c.now_ms, p);
    early |= s < 25 && m.problem();
    own_any |= u.problem();
  }
  CHECK(!early && m.problem() && port_status_detail(m).find("may trip") != std::string::npos && u.warning() &&
            !own_any,
        "91 %%: a problem only after 30 s ('%s'); own 95 %%: a warning only", port_status_detail(m).c_str());
  load(720);   // 2.16 A: 96 %
  early = false;
  for (int s = 0; s < 40; s++) {
    c.run(p, 1000);
    u.update(c.now_ms, p);
    early |= s < 25 && u.problem();
  }
  CHECK(!early && u.problem(), "96 %%, own 95 %%: a problem only after 30 s");
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
            port_status_detail(m) == "Receiver 3 doesn't report: the port's current and power are without it.",
        "incomplete: %.2f A, '%s' / '%s'", m.current_a(), port_status(m, p).c_str(),
        port_status_detail(m).c_str());
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
      {ULTRA_ST_WHY_HOT, 0, 0, RS_TOO_HOT, "over its warning level. Check"},
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
  CHECK(std::fabs(pm.peak_a() - 2.4f) < 1e-4f && pm.peaks() && port_status(pm, q) == "Peaks 105 %" &&
            port_status_detail(pm).find("Peaks up to 2.4 A (105 %) in the last minute: short peaks are fine") ==
                0,
        "peaks: %.2f A, '%s' / '%s'", pm.peak_a(), port_status(pm, q).c_str(), port_status_detail(pm).c_str());
  e.rx[1].features &= ~(ULTRA_FEAT_LIFETIME | ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE);
  for (auto &b : e.rx[1].uid)
    b ^= 0x33;   // (a 0.6.8 board swapped in: IDENTITY again)
  e.run(q, 8000);
  pm.update(e.now_ms, q);
  CHECK(std::fabs(pm.peak_a() - 1.74f) < 1e-4f && !pm.peaks() && port_status(pm, q) == "OK",
        "a board without the peak counts with its average: %.2f A", pm.peak_a());
}

// ------------------------------------------------------------------------------------ the lifetime ledger (0.2.6)
/// A hub's ledger side of one line: its poller and its keeper on an ESP32's ledger.
struct LedgerHub {
  TelemetryPoller p;
  LedgerKeeper k;
  uint8_t events{0};
  uint32_t new_boards{0}, restores{0}, attentions{0};
};

/// Home Assistant's view: each position's Energy as it got it (the same entities across the controller's restarts)
struct HaView {
  std::vector<double> last;
  double worst_back{0};   // the largest step back (Wh)
};

static TelemetryIntervals ledger_iv() {
  TelemetryIntervals iv;
  iv.status_ms = 1000;
  iv.energy_ms = 2000;
  iv.lifetime_ms = 4000;
  return iv;
}

static void ledger_board(Model &m) {
  m.set025();
  m.features |= ULTRA_FEAT_LIFE_SYNC;
}

/// the hub starts now (the ESP32's boot; its boards keep their uptime)
static void start_hub(LedgerHub &h, Chain &c, LifetimeLedger &l, uint8_t pin = 5) {
  h = LedgerHub{};
  h.p = started(c, static_cast<uint16_t>(c.rx.size()), ULTRA_RW_RATE_1M, ledger_iv());
  h.k.attach(&l, static_cast<uint32_t>(pin) << 16, static_cast<uint16_t>(c.rx.size()), c.now_ms);
}

/// the line runs for ms: each window's news to the keeper, the positions' Energy to Home Assistant
static void drive(Chain &c, LedgerHub &h, HaView &ha, uint32_t ms) {
  const uint32_t end = c.now_ms + ms;
  while (static_cast<int32_t>(c.now_ms - end) < 0) {
    const uint32_t before = c.now_ms / 1000;
    if (h.p.due(c.now_ms)) {
      c.window(h.p);
      for (uint16_t i = 0; i < h.p.receivers(); i++) {
        const uint8_t ev = h.k.on_updates(h.p, i, h.p.take_updates(i), c.now_ms);
        h.events |= ev;
        h.new_boards += (ev & LedgerKeeper::EV_NEW_BOARD) ? 1 : 0;
        h.restores += (ev & LedgerKeeper::EV_RESTORE) ? 1 : 0;
        h.attentions += (ev & LedgerKeeper::EV_ATTENTION) ? 1 : 0;
        const double wh = h.k.energy_wh(i);
        if (wh >= 0) {
          if (ha.last.size() <= i)
            ha.last.resize(i + 1, -1.0);
          if (ha.last[i] >= 0 && ha.last[i] - wh > ha.worst_back)
            ha.worst_back = ha.last[i] - wh;
          ha.last[i] = wh;
        }
      }
    } else {
      c.now_ms += 5;
    }
    for (uint32_t s = before; s < c.now_ms / 1000; s++) {
      for (auto &m : c.rx)
        m.uptime++;
    }
  }
}

static uint64_t ledger_mwh(const LedgerHub &h, uint16_t i) {
  const LifeCounters *lc = h.k.counters(i);
  return lc != nullptr ? lc->mwh : 0;
}
static unsigned long long ull(uint64_t v) { return static_cast<unsigned long long>(v); }
static bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }

/// First contact adopts; the ledger counts on from there, never down; after a power-up the ledger given
/// back once (applied); mWh carried across Wh boundaries and across the cuts that drop a board's fraction (only whole
/// missing Wh go back); the answer paths: a record written since (re-read, applied), no-op, once, refused twice
/// (needs attention, the ledger shown), capped (the rest at the next power-up).
static void test_ledger_sync() {
  Chain c(1);
  Model &m = c.rx[0];
  ledger_board(m);
  LifetimeLedger led;
  led.init(8, 4);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, led);
  drive(c, h, ha, 10000);
  const LifeCounters *lc = h.k.counters(0);
  CHECK(lc != nullptr && lc->lit == 12345 && lc->powered == 24690 && lc->ups == 41 && lc->mwh == 1234567 &&
            m.syncs == 0,
        "first contact: its counters adopted (lit %u, ups %u, %llu mWh), no sync (%u)", lc ? lc->lit : 0,
        lc ? lc->ups : 0, ull(ledger_mwh(h, 0)), m.syncs);
  CHECK(near(h.k.energy_wh(0), 1234.567), "the position's Energy: the board's lifetime (%.3f Wh)", h.k.energy_wh(0));
  // 16: mWh across a Wh boundary
  m.use(300);
  drive(c, h, ha, 5000);
  m.use(300);
  drive(c, h, ha, 5000);
  CHECK(ledger_mwh(h, 0) == 1235167 && m.total_wh == 1235, "+0.6 Wh across a Wh boundary: %llu mWh",
        ull(ledger_mwh(h, 0)));
  // a cut: it lost its fraction (stored 1235 Wh) and 0.3 h lit / 0.4 h powered it hadn't stored; a power-up more
  m.power_up();
  m.lit_01h -= 3;
  m.powered_01h -= 4;
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 1 && m.sync_result == ULTRA_LS_R_APPLIED && m.lit_01h == 12345 && m.powered_01h == 24690 &&
            m.total_wh == 1235 && h.k.counters(0)->ups == 42,
        "after its power-up one sync: lit / powered back (%u / %u), its Wh not (%u: only its fraction was lost), its "
        "own power-up count adopted (%u)",
        m.lit_01h, m.powered_01h, m.total_wh, h.k.counters(0)->ups);
  CHECK(ledger_mwh(h, 0) == 1235167, "the fraction it dropped stays in the ledger (%llu mWh)", ull(ledger_mwh(h, 0)));
  m.use(900);
  drive(c, h, ha, 30000);
  CHECK(ledger_mwh(h, 0) == 1236067 && m.syncs == 1, "on across the boundary in the hub (%llu mWh), one sync (%u)",
        ull(ledger_mwh(h, 0)), m.syncs);
  m.use(2400);
  drive(c, h, ha, 5000);
  m.power_up(2);   // (stored 1236 of its 1238 Wh)
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 2 && m.sync_result == ULTRA_LS_R_APPLIED && m.total_wh == 1238 && ledger_mwh(h, 0) == 1238467,
        "2 Wh lost at a cut: given back (%u Wh), the fraction stays in the ledger (%llu mWh)", m.total_wh,
        ull(ledger_mwh(h, 0)));
  // a record written between the read and the sync: refused (RECORDS), read again, applied
  m.records_bump_on_sync = 1;
  m.power_up(1);
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 4 && m.results.size() == 4 && m.results[2] == ULTRA_LS_R_RECORDS && m.results[3] == ULTRA_LS_R_APPLIED &&
            m.total_wh == 1238 && !h.k.attention(0),
        "its record written since the read: refused, read again, applied (%u syncs, %u Wh)", m.syncs, m.total_wh);
  // no-op: it caught up before the sync came (nothing merged: its one merge not used)
  m.wh_bump_on_sync = 1;
  m.power_up(1);
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 5 && m.sync_result == ULTRA_LS_R_NOOP && !m.merged && !h.k.attention(0) && m.total_wh == 1238,
        "caught up before the sync: no-op, nothing merged, done (%u syncs)", m.syncs);
  // once: it took one this power-up already (another hub): done, the rest at its next power-up
  m.force_result = ULTRA_LS_R_ONCE;
  m.power_up(1);
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 6 && m.sync_result == ULTRA_LS_R_ONCE && !h.k.attention(0) && m.total_wh == 1237,
        "already merged this power-up: done, no retry (%u syncs)", m.syncs);
  m.force_result = 0xFF;
  m.power_up();
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 7 && m.sync_result == ULTRA_LS_R_APPLIED && m.total_wh == 1238, "its next power-up: applied (%u Wh)",
        m.total_wh);
  // refused twice (another board's UID16): needs attention, the ledger shown; the next applied clears it
  m.force_result = ULTRA_LS_R_UID;
  m.power_up(1);
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 9 && h.k.attention(0) && h.attentions == 1 && h.k.counters(0)->mwh > m.total_wh * 1000ull + 999,
        "refused twice: needs attention (%u syncs), the ledger shown (%llu mWh, the board %u Wh)", m.syncs,
        ull(ledger_mwh(h, 0)), m.total_wh);
  m.force_result = 0xFF;
  m.power_up();
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 10 && m.sync_result == ULTRA_LS_R_APPLIED && !h.k.attention(0) && m.total_wh == 1238,
        "the next power-up: applied, attention cleared");
  // the read-to-POLL race (design review, round 9): its powered time passed a 0.1 h between the read
  // and the POLL, so its powered step and with it its lit cap are one less: the ledger's one unit of margin keeps the
  // sync applied (lit +5 of the 6 it lost; the last one at its next power-up)
  m.power_up();
  m.powered_01h -= 5;
  m.lit_01h -= 6;
  m.powered_bump_on_sync = 1;
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 11 && m.sync_result == ULTRA_LS_R_APPLIED && (m.sync_mask & ULTRA_LS_F_LIT) &&
            m.powered_01h == 24690 && m.lit_01h == 12344 && !h.k.attention(0),
        "the read-to-POLL race: applied (result %u, mask %02X): powered %u, lit %u", m.sync_result, m.sync_mask,
        m.powered_01h, m.lit_01h);
  m.power_up();
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 12 && m.sync_result == ULTRA_LS_R_APPLIED && m.lit_01h == 12345,
        "its next power-up: the last lit unit (its cap without powered behind: %u)", m.lit_01h);
  // capped (two units in between: past the margin): powered merged, lit not, done; the rest next power-up
  m.power_up();
  m.powered_01h -= 5;
  m.lit_01h -= 6;
  m.powered_bump_on_sync = 2;
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 13 && m.sync_result == ULTRA_LS_R_CAPPED && (m.sync_mask & ULTRA_LS_F_POWERED) &&
            !(m.sync_mask & ULTRA_LS_F_LIT) && m.powered_01h == 24690 && m.lit_01h == 12339 && !h.k.attention(0),
        "capped (lit over its cap): powered merged, lit not (%u), done (mask %02X)", m.lit_01h, m.sync_mask);
  m.power_up();
  drive(c, h, ha, 15000);
  CHECK(m.syncs == 14 && m.sync_result == ULTRA_LS_R_APPLIED && m.lit_01h == 12340,
        "its next power-up: lit +1 (its cap without powered behind: %u)", m.lit_01h);
  CHECK(ha.worst_back == 0, "Energy never went back (%.3f Wh)", ha.worst_back);
  std::printf("  ledger: adopt, merge, one sync per power-up, the mWh carried, every answer path\n");
}

/// The Wh a sync gives back count the board's own fraction: 1.967 Wh lost at a cut, 0.9 Wh into its new
/// power-up when the hub reads it: 1 whole Wh back (the board never above the ledger; 0.967 stays with the hub).
static void test_ledger_fraction() {
  Chain c(1);
  Model &m = c.rx[0];
  ledger_board(m);
  m.total_wh = 1238;
  m.boot_mwh = 467;
  LifetimeLedger led;
  led.init(4, 2);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, led);
  drive(c, h, ha, 10000);
  m.use(1500);
  drive(c, h, ha, 5000);
  m.power_up(1);   // (stored 1238 of its 1239.967)
  m.use(900);
  drive(c, h, ha, 15000);
  const uint64_t board = m.total_wh * 1000ull + m.boot_mwh % 1000;
  CHECK(m.syncs == 1 && m.sync_result == ULTRA_LS_R_APPLIED && m.total_wh == 1239 && ledger_mwh(h, 0) == 1240867 &&
            board <= ledger_mwh(h, 0) && ha.worst_back == 0,
        "its fraction counted: %u Wh back to %u (the board %llu mWh, the ledger %llu)", m.total_wh - 1238, m.total_wh,
        ull(board), ull(ledger_mwh(h, 0)));
}

/// The replacement cases: a receiver ahead (adopted per counter); a replaced board at a position (a new
/// entry, the position's Energy goes on, the old entry kept); a board moved to another line's position (its entry
/// follows it, the counters go on); a new controller adopts the boards' lifetime.
static void test_ledger_boards() {
  Chain c(2);
  for (auto &m : c.rx)
    ledger_board(m);
  c.rx[1].total_wh = 500;
  c.rx[1].lit_01h = 600;
  c.rx[1].powered_01h = 900;
  LifetimeLedger led;
  led.init(4, 4);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, led);
  drive(c, h, ha, 10000);
  // a receiver ahead (it ran on another controller, back with a power-up): what the hub didn't see taken at its
  // session's start, a unit (a Wh) short (the board's own remainder may ride on a sync), the hub's own remainders
  // kept (its powered time below 0.1 h, its 0.567 Wh); its power-ups; no sync
  c.rx[0].power_up();
  c.rx[0].lit_01h += 100;
  c.rx[0].powered_01h += 150;
  c.rx[0].power_ups += 2;
  c.rx[0].total_wh += 50;
  drive(c, h, ha, 10000);
  const LedgerEntry *ahead = led.find(c.rx[0].uid);
  CHECK(h.k.counters(0)->lit == 12444 && h.k.counters(0)->powered == 24839 && h.k.counters(0)->ups == 44 &&
            ledger_mwh(h, 0) == 1283567 && ahead != nullptr && ahead->pow_rem > 15000 && c.rx[0].syncs == 0,
        "a receiver ahead: taken (lit %u, powered %u, ups %u, %llu mWh), its own %u ms below 0.1 h kept, no sync",
        h.k.counters(0)->lit, h.k.counters(0)->powered, h.k.counters(0)->ups, ull(ledger_mwh(h, 0)),
        ahead != nullptr ? ahead->pow_rem : 0);
  // R1: position 1's board replaced by a used one (50 kWh): a new entry, the position goes on, the old one kept
  uint8_t old_uid[12];
  std::memcpy(old_uid, c.rx[1].uid, 12);
  const double before = h.k.energy_wh(1);
  Model b;
  ledger_board(b);
  for (int k = 0; k < 12; k++)
    b.uid[k] = static_cast<uint8_t>(0xC0 + k);
  b.total_wh = 50000;
  b.boot_mwh = 0;
  b.uptime = 0;
  b.lit_01h = 99999;
  b.powered_01h = 199999;
  c.rx[1] = b;
  drive(c, h, ha, 15000);
  CHECK(h.new_boards == 1 && near(h.k.energy_wh(1), before) && led.find(old_uid) != nullptr &&
            led.find(b.uid) != nullptr && h.k.counters(1)->lit == 99999 && c.rx[1].syncs == 0,
        "a replaced board: logged once (%u), the position goes on (%.3f, was %.3f), the old entry kept, the new one "
        "adopted, no sync",
        h.new_boards, h.k.energy_wh(1), before);
  c.rx[1].use(1500);
  drive(c, h, ha, 5000);
  CHECK(near(h.k.energy_wh(1), before + 1.5), "then its use (%.3f)", h.k.energy_wh(1));
  // R2: board 0 moved to another line's position 0 (a new board in its place): its entry follows it
  Chain c2(1);
  c2.now_ms = c.now_ms;
  c2.rx[0] = c.rx[0];
  c2.rx[0].power_up(1);   // (it lost 1 Wh and 0.2 h at the move)
  c2.rx[0].lit_01h -= 2;
  c2.rx[0].powered_01h -= 2;
  const LifeCounters moved = *h.k.counters(0);
  Model n;
  ledger_board(n);
  for (int k = 0; k < 12; k++)
    n.uid[k] = static_cast<uint8_t>(0xE0 + k);
  n.uptime = 0;
  c.rx[0] = n;
  LedgerHub h2;
  HaView ha2;
  start_hub(h2, c2, led, 6);
  h2.k.attach(&led, 6u << 16, 1, 0);   // (the same ESP32: its hub started at 0)
  drive(c2, h2, ha2, 15000);
  drive(c, h, ha, 15000);
  CHECK(h2.k.counters(0) != nullptr && h2.k.counters(0)->lit == moved.lit && c2.rx[0].lit_01h == moved.lit &&
            c2.rx[0].syncs == 1 && near(h2.k.energy_wh(0), moved.mwh / 1000.0),
        "a moved board: its entry follows it (lit %u, synced back: %u), the new position starts at its ledger "
        "(%.3f)",
        h2.k.counters(0) ? h2.k.counters(0)->lit : 0, c2.rx[0].lit_01h, h2.k.energy_wh(0));
  CHECK(h.new_boards == 2 && h.k.energy_wh(0) >= ha.last[0] - 1e-9, "its old position: a new board, on from there");
  CHECK(ha.worst_back == 0, "Energy never went back on one controller (%.3f Wh)", ha.worst_back);
  // R3: the controller replaced (an empty ledger, the same entities): each board's lifetime at first contact; where
  // the old one had re-based a replaced board, the value jumps (Home Assistant: a rise counts, a drop of 10 % or
  // more is a new meter cycle: the wiki says how to fix the statistic)
  LifetimeLedger fresh;
  fresh.init(4, 4);
  LedgerHub h3;
  HaView ha3 = ha;
  start_hub(h3, c, fresh);
  drive(c, h3, ha3, 15000);
  CHECK(near(h3.k.energy_wh(0), 1234.567) && near(h3.k.energy_wh(1), 50001.5) && c.rx[1].syncs == 0 &&
            c.rx[0].syncs == 0,
        "a new controller: the boards' lifetimes (%.3f / %.3f; the old one showed %.3f / %.3f), no sync",
        h3.k.energy_wh(0), h3.k.energy_wh(1), ha.last[0], ha.last[1]);
  std::printf("  ledger: a receiver ahead, a replaced board, a moved board, a new controller\n");
}

/// A board whose record went back (erased): logged once, the ledger given back over a few power-ups (each
/// counter at most the receiver's cap per merge); its position never back.
static void test_ledger_erased() {
  Chain c(1);
  Model &m = c.rx[0];
  ledger_board(m);
  LifetimeLedger led;
  led.init(4, 2);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, led);
  drive(c, h, ha, 10000);
  m.power_up();
  m.records = 2;
  m.powered_01h = 20;
  m.lit_01h = 10;
  m.power_ups = 1;
  m.lt_dips = m.lt_overloads = m.lt_shorts = 0;
  m.total_wh = 3;
  drive(c, h, ha, 15000);
  CHECK(h.restores == 1 && m.syncs == 1 && m.powered_01h == 7220 && m.lit_01h == 7210 && m.total_wh == 1234 &&
            m.lt_dips == 2 && m.lt_shorts == 3,
        "erased: logged (%u), the first power-up back by the caps' steps, lit one less (the race margin) (powered %u, "
        "lit %u, %u Wh)",
        h.restores, m.powered_01h, m.lit_01h, m.total_wh);
  for (int k = 0; k < 4; k++) {
    m.power_up();
    drive(c, h, ha, 15000);
  }
  CHECK(m.syncs == 4 && m.powered_01h == 24690 && m.lit_01h == 12345 && h.restores == 1 && ha.worst_back == 0,
        "whole again after 4 power-ups (%u syncs: powered %u, lit %u), logged once, Energy never back", m.syncs,
        m.powered_01h, m.lit_01h);
  std::printf("  ledger: an erased board restored over 4 power-ups\n");
}

/// The ESP32 restarts (a stored ledger older than what was shown): the board's lifetime, its
/// power-up's count not again; used while the ESP32 was off: counted; a used board fitted while it was off: the
/// position goes on; a reading of a new power-up before the STATUS that shows it; a power-off without a cut; the
/// 32-bit count wrapping.
static void test_ledger_restart() {
  Chain c(1);
  Model &m = c.rx[0];
  ledger_board(m);
  LifetimeLedger led;
  led.init(8, 4);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, led);
  drive(c, h, ha, 10000);
  m.use(150000);
  drive(c, h, ha, 5000);
  std::vector<uint8_t> blob(led.blob_size());
  led.save(blob.data(), 1);   // (the 15 min save)
  const uint64_t saved = ledger_mwh(h, 0);
  m.use(10000);               // 10 Wh more shown; then the ESP32 crashes before its next save
  drive(c, h, ha, 5000);
  const double shown = h.k.energy_wh(0);
  LifetimeLedger led2;
  led2.init(8, 4);
  uint32_t gen = 0;
  CHECK(led2.load(blob.data(), blob.size(), &gen) && gen == 1, "the stored ledger loads");
  start_hub(h, c, led2);
  drive(c, h, ha, 10000);
  CHECK(near(h.k.energy_wh(0), shown) && near(shown, 1394.567) && ha.worst_back == 0,
        "N1: the ESP32 restarted (stored %.3f): %.3f Wh as shown before (%.3f), its power-up's count not again",
        saved / 1000.0, h.k.energy_wh(0), shown);
  m.use(700);
  drive(c, h, ha, 5000);
  CHECK(near(h.k.energy_wh(0), shown + 0.7), "then on (%.3f)", h.k.energy_wh(0));
  // N2: used while the ESP32 was off (a clean shutdown stored the ledger)
  blob.resize(led2.blob_size());
  led2.save(blob.data(), 2);
  m.use(20000);
  m.uptime += 3600;
  c.now_ms += 3600000;
  LifetimeLedger led3;
  led3.init(8, 4);
  led3.load(blob.data(), blob.size(), &gen);
  start_hub(h, c, led3);
  drive(c, h, ha, 10000);
  CHECK(near(h.k.energy_wh(0), shown + 20.7) && ha.worst_back == 0, "N2: used while the ESP32 was off: counted (%.3f)",
        h.k.energy_wh(0));
  // a reading of a new power-up before the STATUS that shows it: its use once
  const double e0 = h.k.energy_wh(0);
  m.power_up();
  m.use(300);
  h.p.read_now(0, TP_ENERGY, c.now_ms - 20000);
  drive(c, h, ha, 10000);
  CHECK(near(h.k.energy_wh(0), e0 + 0.3), "a new power-up's reading before its STATUS: +0.3 Wh, once (%.3f)",
        h.k.energy_wh(0) - e0);
  m.use(200);
  drive(c, h, ha, 5000);
  CHECK(near(h.k.energy_wh(0), e0 + 0.5), "then +0.2 (%.3f)", h.k.energy_wh(0) - e0);
  // a power-off that didn't cut it (a supply kept on): its count goes on, only the new mWh
  h.p.unpowered();
  h.p.locked(c.now_ms);
  m.use(100);
  drive(c, h, ha, 10000);
  CHECK(near(h.k.energy_wh(0), e0 + 0.6), "a power-off without a cut: +0.1 Wh, not its power-up's count again (%.3f)",
        h.k.energy_wh(0) - e0);
  // N3: a used board fitted while the ESP32 was off (shut down cleanly): the position goes on, not +50 kWh
  blob.resize(led3.blob_size());
  led3.save(blob.data(), 3);
  const double last = ha.last[0];
  Model b;
  ledger_board(b);
  for (int k = 0; k < 12; k++)
    b.uid[k] = static_cast<uint8_t>(0x90 + k);
  b.total_wh = 50000;
  b.boot_mwh = 0;
  b.uptime = 30;
  c.rx[0] = b;
  c.now_ms += 60000;
  LifetimeLedger led4;
  led4.init(8, 4);
  led4.load(blob.data(), blob.size(), &gen);
  start_hub(h, c, led4);
  drive(c, h, ha, 10000);
  CHECK(h.new_boards == 1 && near(h.k.energy_wh(0), last) && ha.worst_back == 0,
        "N3: a used board fitted while the ESP32 was off: the position goes on (%.3f, was %.3f)", h.k.energy_wh(0), last);
  // ... after a crash instead (the last save older): back by what was shown since that save at most
  blob.resize(led4.blob_size());
  led4.save(blob.data(), 4);
  const double at_save = h.k.energy_wh(0);
  c.rx[0].use(5000);
  drive(c, h, ha, 5000);
  Model b2 = b;
  for (int k = 0; k < 12; k++)
    b2.uid[k] = static_cast<uint8_t>(0xA0 + k);
  c.rx[0] = b2;
  LifetimeLedger led5;
  led5.init(8, 4);
  led5.load(blob.data(), blob.size(), &gen);
  start_hub(h, c, led5);
  drive(c, h, ha, 10000);
  CHECK(near(h.k.energy_wh(0), at_save) && near(ha.worst_back, 5.0),
        "a crash, then a board fitted while off: back to the last save (%.3f): the 5 Wh shown since (%.3f): the bound",
        h.k.energy_wh(0), ha.worst_back);
  // the 32-bit count wraps within a power-up (4295 kWh): on, nothing lost
  Chain w(1);
  ledger_board(w.rx[0]);
  w.rx[0].boot_mwh = 0xFFFFF000u;
  w.rx[0].total_wh = 4400000;
  LifetimeLedger lw;
  lw.init(2, 1);
  LedgerHub hw;
  HaView haw;
  start_hub(hw, w, lw);
  drive(w, hw, haw, 10000);
  const uint64_t w0 = ledger_mwh(hw, 0);
  w.rx[0].boot_mwh += 0x2000;   // (wraps: 0x1000)
  w.rx[0].total_wh += 8;
  drive(w, hw, haw, 10000);
  CHECK(ledger_mwh(hw, 0) == w0 + 0x2000 && haw.worst_back == 0, "its count wrapped: +%llu mWh",
        ull(ledger_mwh(hw, 0) - w0));
  std::printf("  ledger: the ESP32's restarts, a reading before its STATUS, no cut, the count's wrap\n");
}

/// A board without LIFE_SYNC (0.2.5) never gets 0x0E (the ledger only, shown); without LIFETIME (0.2.4): its
/// Energy through the ledger still. The stored form: two copies, a broken one refused; an old entry ages out.
static void test_ledger_misc() {
  Chain c(2);
  c.rx[0].set025();                 // LIFETIME, no LIFE_SYNC
  LifetimeLedger led;
  led.init(4, 2);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, led);
  drive(c, h, ha, 10000);
  c.rx[0].power_up(2);
  c.rx[0].lit_01h -= 5;
  c.rx[1].power_up(2);
  drive(c, h, ha, 15000);
  CHECK(c.rx[0].asked[ULTRA_PAGE_LIFE_SYNC] == 0 && c.rx[1].asked[ULTRA_PAGE_LIFE_SYNC] == 0 && !h.k.sync_sent(0) &&
            !h.p.sync_pending(0) && h.k.counters(0)->lit == 12345 &&
            ledger_mwh(h, 0) == 1234567 && ledger_mwh(h, 1) == 1234567 && ha.worst_back == 0,
        "no LIFE_SYNC (bit 22): no 0x0E (%u / %u), the ledger shown (lit %u, %llu mWh)",
        c.rx[0].asked[ULTRA_PAGE_LIFE_SYNC], c.rx[1].asked[ULTRA_PAGE_LIFE_SYNC], h.k.counters(0)->lit, ull(ledger_mwh(h, 0)));
  // the stored form: a copy refused when broken, its generation; another size refused
  std::vector<uint8_t> a(led.blob_size()), b(led.blob_size());
  led.save(a.data(), 7);
  led.save(b.data(), 8);
  b[40] ^= 1;
  LifetimeLedger l2;
  l2.init(4, 2);
  uint32_t gen = 0;
  const bool b_ok = l2.load(b.data(), b.size(), &gen);
  const bool a_ok = l2.load(a.data(), a.size(), &gen);
  a[0] ^= 1;
  LifetimeLedger l3;
  l3.init(4, 2);
  uint32_t g3 = 0;
  CHECK(!b_ok && a_ok && gen == 7 && !l3.load(a.data(), a.size(), &g3) && !l3.load(a.data(), a.size() - 2, &g3) &&
            l2.find(c.rx[0].uid) != nullptr && l2.find(c.rx[0].uid)->c.lit == 12345,
        "the stored copies: a broken one refused, the other loads (generation %u)", gen);
  // age-out: a board not seen for 90 hub days that no position holds goes; one a position holds stays
  Model gone;
  for (int k = 0; k < 12; k++)
    gone.uid[k] = static_cast<uint8_t>(0x10 + k);
  bool created = false;
  led.touch(gone.uid, &created);
  led.set_day(led.day() + LEDGER_AGE_DAYS - 1);
  const bool kept = led.find(gone.uid) != nullptr;
  led.set_day(led.day() + 1);
  CHECK(created && kept && led.find(gone.uid) == nullptr && led.find(c.rx[0].uid) != nullptr,
        "a board not seen for %u days goes, the ones the positions hold stay", LEDGER_AGE_DAYS);
  std::printf("  ledger: no 0x0E without bit 22, the stored copies, the age-out\n");
}

// --------------------------------------------------------------------------------------- Home Assistant's pacing
/// A paced sensor: what it sent, when (ultra_pacing.h)
struct PacedOut {
  bool have{false};
  float last{NAN};
  std::vector<std::pair<uint32_t, float>> sent;
  void send(uint32_t ms, float v) {
    this->have = true;
    this->last = v;
    this->sent.push_back({ms, v});
  }
};

/// R12 / R25: the deadband (the larger of abs and rel), the minimum interval, a fade's end sent with the first sample
/// after the interval, NAN and the first value at once, a forced value at once; a recorded power is the time-weighted
/// mean of each 10 s (its live twin at the update rate from the same samples), so the energy from either is the same.
static void test_pacing() {
  // the deadband and the interval (bus voltage: 0.2 V, 10 s)
  Paced pv;
  PacedOut v;
  auto feed = [&](Paced &p, const Pace &c, PacedOut &o, uint32_t ms, float x, bool force = false) {
    if (p.due(c, x, o.last, o.have, ms, force))
      o.send(ms, x);
  };
  feed(pv, PACE_VOLTAGE, v, 0, 24.00f);
  feed(pv, PACE_VOLTAGE, v, 1000, 24.10f);    // under the deadband
  feed(pv, PACE_VOLTAGE, v, 2000, 24.30f);    // over it, within 10 s: waits
  feed(pv, PACE_VOLTAGE, v, 9000, 24.35f);
  feed(pv, PACE_VOLTAGE, v, 10000, 24.40f);   // 10 s: the value now (the fade's end so far)
  feed(pv, PACE_VOLTAGE, v, 11000, 24.40f);
  feed(pv, PACE_VOLTAGE, v, 30000, 24.40f);   // settled: nothing
  CHECK(v.sent.size() == 2 && v.sent[0].second == 24.00f && v.sent[1].first == 10000 && v.sent[1].second == 24.40f,
        "deadband + interval: %u sent, the second %.2f V at %u ms", static_cast<unsigned>(v.sent.size()),
        v.sent.size() > 1 ? v.sent[1].second : 0.0f, v.sent.size() > 1 ? v.sent[1].first : 0u);
  // unavailable and back: at once, whatever the interval; a forced value at once
  feed(pv, PACE_VOLTAGE, v, 31000, NAN);
  feed(pv, PACE_VOLTAGE, v, 32000, 24.40f);
  feed(pv, PACE_VOLTAGE, v, 33000, 26.0f, true);
  CHECK(v.sent.size() == 5 && std::isnan(v.sent[2].second) && v.sent[3].second == 24.40f && v.sent[4].second == 26.0f,
        "NAN, back, forced: at once (%u)", static_cast<unsigned>(v.sent.size()));
  // the relative deadband: 2 % of 200 W is 4 W (the larger)
  CHECK(!pace_moved(PACE_POWER, 203.0f, 200.0f, true) && pace_moved(PACE_POWER, 204.0f, 200.0f, true) &&
            !pace_moved(PACE_POWER, 10.4f, 10.0f, true) && pace_moved(PACE_POWER, 10.5f, 10.0f, true),
        "power's deadband: 0.5 W or 2 %%, the larger");
  // the live twins (round 11): 0.2 W or 1 %, the larger; and what that saves on a steady load with its noise (a
  // random walk of +-0.3 % a second around 20 / 60 / 120 W, an hour at 1 s)
  CHECK(!pace_moved(PACE_LIVE_W, 100.9f, 100.0f, true) && pace_moved(PACE_LIVE_W, 101.0f, 100.0f, true) &&
            !pace_moved(PACE_LIVE_W, 10.15f, 10.0f, true) && pace_moved(PACE_LIVE_W, 10.25f, 10.0f, true),
        "live power's deadband: 0.2 W or 1 %%, the larger");
  for (float base : {20.0f, 60.0f, 120.0f}) {
    uint32_t seed = 99;
    float x = base, last_old = NAN, last_new = NAN;
    bool have_old = false, have_new = false;
    int sent_old = 0, sent_new = 0;
    const Pace old_live{0.2f, 0.0f, 0};
    for (int s = 0; s < 3600; s++) {
      seed = seed * 1103515245u + 12345u;
      x += base * 0.003f * (static_cast<float>((seed >> 8) % 2001) / 1000.0f - 1.0f);
      x += (base - x) * 0.05f;   // (drawn back to the load)
      if (pace_moved(old_live, x, last_old, have_old)) {
        last_old = x;
        have_old = true;
        sent_old++;
      }
      if (pace_moved(PACE_LIVE_W, x, last_new, have_new)) {
        last_new = x;
        have_new = true;
        sent_new++;
      }
    }
    std::printf("  live power at %.0f W with +-0.3 %% noise, an hour at 1 s: %d states sent (0.2 W alone: %d)\n", base,
                sent_new, sent_old);
    CHECK(sent_new <= sent_old, "the live deadband never sends more (%d, %d)", sent_new, sent_old);
  }
  // a fade: live at 1 s (0.2 W), recorded as the 10 s mean; their energy the same
  MeanPaced mean;
  Paced live;
  PacedOut rec, liv;
  const Pace live_c{0.2f, 0.0f, 750};
  double e_true = 0.0;
  float prev = NAN;
  for (uint32_t ms = 0; ms <= 60000; ms += 1000) {
    const float w = ms < 20000 ? 10.0f : ms < 40000 ? 10.0f + (ms - 20000) / 200.0f : 110.0f;   // 10 W, a fade to 110 W
    if (!std::isnan(prev))
      e_true += prev * 1.0;   // (W x s, held until the next sample)
    prev = w;
    float out;
    if (mean.sample(PACE_POWER, w, rec.last, rec.have, ms, false, &out))
      rec.send(ms, out);
    feed(live, live_c, liv, ms, w);
  }
  // each mean covers the 10 s before it; a window not sent was within the deadband of the value before
  double e_rec = 0.0;
  float cur = rec.sent.front().second;
  for (uint32_t b = 10000; b <= 60000; b += 10000) {
    for (const auto &s : rec.sent) {
      if (s.first == b)
        cur = s.second;
    }
    e_rec += cur * 10.0;
  }
  bool spaced = true;
  for (size_t i = 2; i < rec.sent.size(); i++)
    spaced &= rec.sent[i].first - rec.sent[i - 1].first >= 10000;
  CHECK(rec.sent.size() == 4 && spaced && std::fabs(rec.sent.back().second - 110.0f) < 1e-3f &&
            std::fabs(e_rec - e_true) < 0.5,
        "the recorded power: the first value, then 10 s means while it moves (%u: %.1f .. %.1f W), its energy %.1f Ws "
        "(true %.1f)", static_cast<unsigned>(rec.sent.size()), rec.sent.front().second, rec.sent.back().second, e_rec,
        e_true);
  CHECK(liv.sent.size() >= 20 && liv.sent.size() <= 23 && liv.sent.back().second == 110.0f,
        "the live twin: every second while it moves (%u), steady not again", static_cast<unsigned>(liv.sent.size()));
  // energy: any growth, every 5 min at most
  Paced pe;
  PacedOut e;
  for (uint32_t s = 0; s <= 900; s += 10)
    feed(pe, PACE_ENERGY, e, s * 1000, 1000.0f + s * 0.01f);
  CHECK(e.sent.size() == 4 && e.sent[1].first == 300000 && e.sent[3].first == 900000,
        "energy: every 5 min while it grows (%u)", static_cast<unsigned>(e.sent.size()));
  // the user's own filters: every change
  Paced pc;
  PacedOut c;
  for (uint32_t ms = 0; ms < 5000; ms += 1000)
    feed(pc, PACE_CHANGE, c, ms, static_cast<float>(ms / 2000));
  CHECK(c.sent.size() == 3, "unpaced: every change (%u)", static_cast<unsigned>(c.sent.size()));
}

/// The ledger's counters (a port's energy, a light's) and its keys: kept with the line's id (not its pin), never
/// down, stored and loaded with the receivers' positions.
static void test_ledger_counters() {
  LifetimeLedger led;
  CHECK(!led.ready() && led.counter_mwh(1) == 0, "not loaded: nothing kept");
  led.init(4, 4);
  const uint32_t port = (ledger_key("port4") & 0xFFFF0000u) | 0xFFFFu, light = ledger_key("bookshelf_1");
  led.set_counter(port, 1500);
  led.set_counter(port, 1200);   // (never down)
  led.set_counter(light, 250);
  CHECK(led.counter_mwh(port) == 1500 && led.counter_mwh(light) == 250 && led.dirty(), "counters: up only (%llu)",
        ull(led.counter_mwh(port)));
  std::vector<uint8_t> blob(led.blob_size());
  led.save(blob.data(), 1);
  LifetimeLedger l2;
  l2.init(4, 4);
  uint32_t gen = 0;
  CHECK(l2.load(blob.data(), blob.size(), &gen) && l2.counter_mwh(port) == 1500 && l2.counter_mwh(light) == 250,
        "stored and loaded");
  // a counter and a receiver position with the same key: apart
  LedgerKeeper k;
  k.attach(&l2, ledger_key("port4"), 2);
  CHECK(k.key(1) == ((ledger_key("port4") & 0xFFFF0000u) | 1u) && k.port_key() == port,
        "a line's keys: its id's hash, the receiver's index, 0xFFFF its port");
  CHECK(ledger_key("port4") == 0x9A093BD6u, "the key: FNV-1a as receiver_schema.py's ledger_key (%08X)",
        static_cast<unsigned>(ledger_key("port4")));
}

/// The lock interval after a preamble is waited out once (the first frame after it
/// forgets it); a lock 24.8 days old (past the signed half of millis()) must not stop the polls
/// (with the first frame the hub sends).
static void test_r9_lock_long_run() {
  TelemetryPoller p;
  p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  p.locked(0);
  const bool early = p.due(1000), after = p.due(1001);
  std::vector<uint8_t> slots(ULTRA_SLOT_BYTES * 2);
  uint32_t w = 0;
  p.build(1001, slots.data(), &w);   // (the first frame after the lock)
  p.finish(1010, nullptr, 0);
  p.read_now(0, TP_STATUS, 0x80000000u);
  const bool aged = p.due(0x80000000u), later = p.due(0xFFFFF000u);
  p.locked(0xFFFFFF00u);   // a new preamble: its own lock interval again (across millis()'s wrap)
  const bool relock = p.due(0xFFFFFF00u + 500), relock_after = p.due(0xFFFFFF00u + 1001);
  CHECK(!early && after && aged && later && !relock && relock_after,
        "polls after the lock interval (%d / %d), still at 24.9 and 49 days (%d / %d); a new lock waits again (%d / %d)",
        early, after, aged, later, relock, relock_after);
}

/// The same kind: a receiver's restart hold (1 s) must not cut the chain behind it 24.8 days later, and a
/// line back from a month in standby must poll at once (its pages' last times are older than half of millis()).
static void test_r9_long_runs() {
  auto age = [](Chain &c, uint32_t ms) {   // the line on, nothing polled meanwhile (the receivers count on)
    c.now_ms += ms;
    for (auto &m : c.rx)
      m.uptime = (m.uptime + ms / 1000) & 0xFFFFFF;
  };
  {
    Chain c(2);
    TelemetryPoller p = started(c, 2);
    c.run(p, 8000);
    c.rx[0].restart();   // (a hold of 1 s: the receiver behind it waits)
    c.run(p, 20000);
    age(c, 0x80000000u);
    const uint32_t s1 = c.rx[1].asked[ULTRA_PAGE_STATUS];
    c.run(p, 10000);
    CHECK(c.rx[1].asked[ULTRA_PAGE_STATUS] > s1 && p.answering(1),
          "24.9 days after receiver 1 restarted, receiver 2 behind it still polled (%u STATUS)",
          c.rx[1].asked[ULTRA_PAGE_STATUS] - s1);
  }
  {
    Chain c(1);
    TelemetryPoller p = started(c, 1);
    c.run(p, 8000);
    p.unpowered();   // a month in standby
    age(c, 0x90000000u);
    c.rx[0].restart();
    p.locked(c.now_ms);
    const uint32_t s = c.rx[0].asked[ULTRA_PAGE_STATUS];
    c.run(p, 5000);
    CHECK(c.rx[0].asked[ULTRA_PAGE_STATUS] > s && p.answering(0),
          "back after 28 days in standby: polled at once (%u STATUS)", c.rx[0].asked[ULTRA_PAGE_STATUS] - s);
  }
}

/// A load replaces the ledger: an entry absent from the newer copy (aged out there) doesn't survive from
/// a copy loaded before it.
static void test_r9_load_replaces() {
  uint8_t uid[12] = {1};
  bool created = false;
  LifetimeLedger l;
  l.init(4, 2);
  l.touch(uid, &created)->c.mwh = 123000;
  std::vector<uint8_t> old(l.blob_size()), newer(l.blob_size());
  l.save(old.data(), 2);
  l.set_day(LEDGER_AGE_DAYS);   // (aged out)
  newer.resize(l.blob_size());
  l.save(newer.data(), 3);
  LifetimeLedger out;
  out.init(4, 2);
  uint32_t gen = 0;
  out.load(old.data(), old.size(), &gen);
  out.load(newer.data(), newer.size(), &gen);
  CHECK(gen == 3 && out.find(uid) == nullptr, "the newer copy (generation %u) without the aged entry: gone (%d)",
        gen, out.find(uid) != nullptr);
}

/// A replaced board whose UID16 is the old one's (a CRC collision) is still another board: its full
/// UID decides. The position goes on (no 50 kWh jump), the new entry adopts its own lifetime.
static void test_r9_uid16_collision() {
  Chain c(1);
  ledger_board(c.rx[0]);
  c.rx[0].total_wh = 100;
  c.rx[0].boot_mwh = 0;
  LifetimeLedger l;
  l.init(8, 4);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, l);
  drive(c, h, ha, 10000);
  Model b = c.rx[0];
  b.uid[0] ^= 0x80;
  const uint16_t fingerprint = c.rx[0].uid16();
  for (unsigned v = 0; v < 65536; v++) {
    b.uid[10] = static_cast<uint8_t>(v >> 8);
    b.uid[11] = static_cast<uint8_t>(v);
    if (b.uid16() == fingerprint)
      break;
  }
  CHECK(b.uid16() == fingerprint && std::memcmp(b.uid, c.rx[0].uid, 12) != 0, "a distinct UID with the same UID16");
  b.total_wh = 50000;
  b.boot_mwh = 0;
  b.uptime = 0;
  b.merged = false;
  const double before = h.k.energy_wh(0);
  c.rx[0] = b;
  drive(c, h, ha, 15000);
  CHECK(near(h.k.energy_wh(0), before) && h.new_boards == 1 && l.find(b.uid) != nullptr &&
            l.find(b.uid)->c.mwh == 50000000 && h.p.data(0).uid[0] == b.uid[0],
        "the same UID16, another board: the position goes on (%.3f, was %.3f), its own entry (%llu mWh)",
        h.k.energy_wh(0), before, ull(l.find(b.uid) ? l.find(b.uid)->c.mwh : 0));
}

/// The ESP32 restarts under a board whose ledger leads its own lifetime (50 Wh it lost earlier): the
/// stored checkpoint (the board's count since power-up when the ledger was stored) carries the use since then, so
/// the position goes on from what it showed (160 Wh), not from the stored 150 or the board's 110.
static void test_r9_restart_with_lead() {
  Chain c(1);
  ledger_board(c.rx[0]);
  c.rx[0].total_wh = 100;
  c.rx[0].boot_mwh = 0;
  LifetimeLedger l;
  l.init(8, 4);
  LedgerHub h;
  HaView ha;
  start_hub(h, c, l);
  drive(c, h, ha, 10000);
  l.find(c.rx[0].uid)->c.mwh = 150000;   // (a lead: records the board lost before)
  drive(c, h, ha, 5000);
  std::vector<uint8_t> blob(l.blob_size());
  l.save(blob.data(), 1);
  c.rx[0].use(10000);
  drive(c, h, ha, 5000);
  const double shown = h.k.energy_wh(0);
  LifetimeLedger restored;
  restored.init(8, 4);
  uint32_t gen = 0;
  restored.load(blob.data(), blob.size(), &gen);
  start_hub(h, c, restored);
  drive(c, h, ha, 10000);
  CHECK(near(shown, 160.0) && near(h.k.energy_wh(0), shown) && ha.worst_back == 0,
        "the ESP32 restarted under a board 50 Wh behind its ledger: %.3f Wh (shown before %.3f, back %.3f)",
        h.k.energy_wh(0), shown, ha.worst_back);
  // ... and the board power-cycled while the ESP32 was off (a new session: its whole count new)
  blob.resize(restored.blob_size());
  restored.save(blob.data(), 2);
  c.rx[0].power_up();
  c.rx[0].use(2000);
  LifetimeLedger again;
  again.init(8, 4);
  again.load(blob.data(), blob.size(), &gen);
  start_hub(h, c, again);
  drive(c, h, ha, 10000);
  CHECK(near(h.k.energy_wh(0), shown + 2.0), "a board power-cycled meanwhile: its new session counted (%.3f)",
        h.k.energy_wh(0));
}

/// A powered, known board moved to another slot (or back after another board was there) counts only
/// what it used since it was last read: its count since power-up follows its UID in the ledger, not the slot.
static void test_r9_moved_live_board() {
  {
    Chain c(1);
    ledger_board(c.rx[0]);
    c.rx[0].uptime = 0;
    c.rx[0].boot_mwh = 0;
    c.rx[0].total_wh = 100;
    LifetimeLedger l;
    l.init(8, 4);
    LedgerHub h;
    HaView ha;
    start_hub(h, c, l);
    drive(c, h, ha, 20000);
    c.rx[0].use(10000);
    drive(c, h, ha, 5000);
    const Model a = c.rx[0];
    const uint64_t before = l.find(a.uid)->c.mwh;
    Model b = a;
    b.uid[0] ^= 0x80;
    b.total_wh = 200;
    b.boot_mwh = 0;
    b.uptime = 0;
    c.rx[0] = b;
    drive(c, h, ha, 10000);
    c.rx[0] = a;
    c.rx[0].uptime += 10;   // (still powered; no more use)
    drive(c, h, ha, 15000);
    CHECK(l.find(a.uid)->c.mwh == before && ha.worst_back == 0,
          "a powered board back at its slot: %llu mWh (was %llu), not its session again",
          ull(l.find(a.uid)->c.mwh), ull(before));
  }
  {
    Chain c(2);
    for (auto &m : c.rx) {
      ledger_board(m);
      m.uptime = 0;
      m.boot_mwh = 0;
      m.total_wh = 100;
    }
    LifetimeLedger l;
    l.init(8, 4);
    LedgerHub h;
    HaView ha;
    start_hub(h, c, l);
    drive(c, h, ha, 20000);
    c.rx[0].use(10000);
    c.rx[1].use(20000);
    drive(c, h, ha, 5000);
    std::vector<uint8_t> u0(c.rx[0].uid, c.rx[0].uid + 12), u1(c.rx[1].uid, c.rx[1].uid + 12);
    const uint64_t a = l.find(u0.data())->c.mwh, b = l.find(u1.data())->c.mwh;
    std::swap(c.rx[0], c.rx[1]);
    drive(c, h, ha, 15000);
    c.rx[0].use(1000);   // (then the moved boards' own use counts)
    drive(c, h, ha, 10000);
    CHECK(l.find(u0.data())->c.mwh == a && l.find(u1.data())->c.mwh == b + 1000,
          "two powered boards swapped: %llu / %llu mWh (were %llu / %llu, +1000 after)",
          ull(l.find(u0.data())->c.mwh), ull(l.find(u1.data())->c.mwh), ull(a), ull(b));
  }
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

/// due()'s memo ("nothing due" for the rest of that millisecond) against the scan itself: a long run where the loop asks
/// several times per millisecond, with every kind of change in between (windows, OUTPUTS on / off, a page asked now,
/// a sync, the lock, the supply off): the two always agree. Then what it saves per call (host ns, 50 receivers).
static void test_due_memo() {
  for (uint16_t n : {1, 16, 50}) {
    Chain c(n);
    c.junk = false;
    for (auto &m : c.rx)
      m.set025();
    TelemetryIntervals iv;
    iv.status_ms = 1000;
    TelemetryPoller p = started(c, n, ULTRA_RW_RATE_1M, iv);
    uint32_t seed = 12345u + n;
    auto rnd = [&seed]() {
      seed = seed * 1103515245u + 12345u;
      return seed >> 8;
    };
    int wrong = 0, checks = 0, windows = 0;
    for (int step = 0; step < 300000; step++) {
      const uint32_t r = rnd() % 1000;
      if (r < 600) {
        // (the loop asks again within the same millisecond)
      } else if (r < 930) {
        c.now_ms += 1 + rnd() % 4;
      } else if (r < 970) {
        if (p.due(c.now_ms)) {
          c.window(p);
          windows++;
        }
      } else if (r < 980) {
        p.set_outputs(static_cast<uint16_t>(rnd() % n), (rnd() & 1) != 0);
      } else if (r < 990) {
        p.read_now(static_cast<uint16_t>(rnd() % n), static_cast<uint8_t>(rnd() % TP_COUNT), c.now_ms);
      } else if (r < 994) {
        uint8_t args[32]{};
        p.request_sync(static_cast<uint16_t>(rnd() % n), args, c.now_ms);
      } else if (rnd() % 64 == 0) {   // (rare: the lock interval starts again after each)
        if (r < 997)
          p.locked(c.now_ms);
        else
          p.unpowered();
      } else if (r >= 997) {
        p.locked(c.now_ms);   // (the line locked again after a supply cut: soon)
      }
      const bool memo = p.due(c.now_ms), scan = p.due_scan(c.now_ms);
      wrong += memo != scan;
      checks++;
    }
    CHECK(wrong == 0 && windows > 100, "%u receivers: due() with its memo = the scan in %d of %d checks (%d windows)", n,
          checks - wrong, checks, windows);
  }
  Chain c(50);
  c.junk = false;
  TelemetryIntervals iv;
  iv.status_ms = 1000;
  TelemetryPoller p = started(c, 50, ULTRA_RW_RATE_1M, iv);
  c.run(p, 60000);
  while (!p.due(c.now_ms))
    c.now_ms++;
  c.window(p);
  const uint32_t now = c.now_ms + 1;
  volatile uint32_t sink = 0;
  auto ns = [&](bool memo) {
    const auto a = std::chrono::steady_clock::now();
    for (int i = 0; i < 200000; i++)
      sink = sink + (memo ? p.due(now) : p.due_scan(now));
    return std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - a).count() / 200000;
  };
  const double scan = ns(false), memo = ns(true);
  CHECK(!p.due(now), "nothing due just after a window");
  std::printf("  due() with nothing due, 50 receivers (host): %.1f ns the scan, %.1f ns the memo within a millisecond\n",
              scan, memo);
}

/// Every RW frame's POLL slots over 10 minutes at 20 and 50 receivers (the 16-of-N selection, receivers that don't
/// answer, OUTPUTS, 0.2.5 boards): their hash, the same before and after a change of build()'s implementation.
static void test_window_hash() {
  uint64_t all = 0;
  for (uint16_t n : {20, 50}) {
    Chain c(n);
    for (uint16_t k = 0; k < n; k += 2)
      c.rx[k].set025();
    c.rx[7].has_channel = false;   // (it, and the ones behind it, silent for a while)
    TelemetryIntervals iv;
    iv.status_ms = 1000;
    TelemetryPoller p = started(c, n, ULTRA_RW_RATE_1M, iv);
    for (uint16_t k = 0; k < n; k += 3)
      p.set_outputs(k, true);
    c.run(p, 120000);
    c.rx[7].has_channel = true;
    c.run(p, 480000);
    std::printf("  RW frames at %u receivers: %u, hash %016llx\n", n, c.frames,
                static_cast<unsigned long long>(c.frame_hash));
    all ^= c.frame_hash;
  }
  CHECK(all != 0, "the window hash");
}

/// PowerOffCap (receivers without standby support on a port): 9 planned power-offs never hold; the 10th within 24 h
/// holds until the oldest is a day old; one a day for a month never holds; across the millis() wrap; and with the
/// check every minute for 50 days after a burst, no old entry comes back after the wrap.
static void test_power_off_cap() {
  PowerOffCap c;
  const uint32_t first = 0xFFFF0000u;   // (the burst crosses the millis() wrap)
  uint32_t t = first;
  bool early = false;
  for (int k = 0; k < 9; k++) {
    c.note(t);
    early |= c.hold(t);
    t += 600000;   // every 10 min
  }
  CHECK(!early && !c.hold(t), "9 power-offs in 1.5 h: no hold");
  c.note(t);
  CHECK(c.hold(t) && c.hold(first + PowerOffCap::WINDOW_MS - 1), "the 10th: held until the oldest is a day old");
  CHECK(!c.hold(first + PowerOffCap::WINDOW_MS), "... then the port may power off again");
  PowerOffCap d;
  bool held = false;
  for (uint32_t day = 0, now = 5000; day < 30; day++, now += PowerOffCap::WINDOW_MS) {
    d.note(now);
    held |= d.hold(now);
  }
  CHECK(!held, "one a day for a month: never held");
  PowerOffCap e;
  uint32_t now = 1000;
  for (int k = 0; k < 10; k++, now += 1000)
    e.note(now);
  bool back = false, first_day = true;
  for (uint32_t m = 0; m < 50u * 24 * 60; m++, now += 60000) {
    const bool h = e.hold(now);
    if (m > 24 * 60 + 1)
      back |= h;
    else if (m < 23 * 60)
      first_day &= h;
  }
  CHECK(first_day && !back, "a burst, then 50 days checked every minute: held the first day, never after the wrap");
}

/// The reply rate (round 12): one rate slower at 4 broken of the last 32 counted windows (3: not; old ones slide out;
/// windows without any answer don't count, missing answers alone neither), not below 250 k; one rate faster after an
/// hour of windows without a broken one (a standby gap counts 60 s), never above the configured rate; a failed try
/// doubles the quiet period up to a day, one that holds resets it; pinned: never.
static void test_reply_rate() {
  std::printf("the reply rate: slower on broken answers, faster after a quiet hour\n");
  uint32_t now = 0xFFFFF000u;   // (across the millis() wrap)
  ReplyRate r;
  r.configure(ULTRA_RW_RATE_1M, true);
  auto win = [&](uint32_t answers, uint32_t broken, uint32_t step = 1000) {
    now += step;
    return r.window(now, answers, broken);
  };
  // 3 broken of 32: no step; a 4th after the first slid out: none either
  bool changed = false;
  for (int k = 0; k < 32; k++)
    changed |= win(2, k < 3 ? 1 : 0);
  changed |= win(2, 1);   // (the window 32 back was broken: it slid out, 3 of 32 again)
  CHECK(!changed && r.rate() == ULTRA_RW_RATE_1M, "3 of the last 32 broken: 1 M");
  // windows without anything heard, or only missing answers: don't count (no step, and they don't push broken ones
  // out): 3 broken and 28 good, 200 without an answer, then a broken one is the 4th of the last 32 counted
  ReplyRate e;
  e.configure(ULTRA_RW_RATE_1M, true);
  uint32_t te = 0;
  bool early = false;
  for (int k = 0; k < 31; k++)
    early |= e.window(te += 1000, 2, k < 3 ? 1 : 0);
  for (int k = 0; k < 200; k++)
    early |= e.window(te += 1000, 0, 0);
  CHECK(!early && e.window(te += 1000, 1, 1) && e.rate() == ULTRA_RW_RATE_500K,
        "200 windows without an answer between: they don't count (the 4th of 32 steps down)");
  bool down = false;
  for (int k = 0; k < 4 && !down; k++)   // (with the one before still among the last 32: at the 3rd)
    down = win(2, 1);
  CHECK(down && r.rate() == ULTRA_RW_RATE_500K && r.steps_down() == 1, "broken ones in a row: 500 k");
  for (int k = 0; k < 3; k++)
    win(2, 1);
  CHECK(r.rate() == ULTRA_RW_RATE_500K, "after a step the count starts again: 3 more broken, still 500 k");
  win(2, 1);
  for (int k = 0; k < 40; k++)
    win(2, 1);
  CHECK(r.rate() == ULTRA_RW_RATE_250K && r.steps_down() == 2, "4 more: 250 k, the slowest (not below)");
  // an hour of windows without a broken one: 500 k; a standby of 2 h between two windows counts 60 s
  bool up = false;
  for (int k = 0; k < 3599 && !up; k++)
    up = win(2, 0);
  CHECK(!up, "59:59 quiet: still 250 k");
  win(2, 0, 2 * 3600000u);   // (a 2 h standby gap)
  CHECK(r.rate() == ULTRA_RW_RATE_500K && r.steps_up() == 1, "an hour of windows (the gap counted 60 s): 500 k");
  // the try fails (broken again): 250 k, and the next quiet period is 2 h
  for (int k = 0; k < 4; k++)
    win(2, 1);
  CHECK(r.rate() == ULTRA_RW_RATE_250K && r.quiet_needed_ms() == 2 * ReplyRate::QUIET_MS,
        "the try failed: 250 k, the next quiet period 2 h (%u min)", r.quiet_needed_ms() / 60000);
  // failed tries double it up to a day
  for (int t = 0; t < 8; t++) {
    const uint32_t need = r.quiet_needed_ms();
    for (uint32_t ms = 0; ms < need + 1000 && r.rate() != ULTRA_RW_RATE_500K; ms += 1000)
      win(2, 0);
    for (int k = 0; k < 4; k++)
      win(2, 1);
  }
  CHECK(r.quiet_needed_ms() == ReplyRate::QUIET_MAX_MS, "8 failed tries: a day at most (%u h)",
        r.quiet_needed_ms() / 3600000);
  // a try that holds an hour resets it; at the configured rate (1 M) no more steps up
  for (uint32_t ms = 0; ms <= ReplyRate::QUIET_MAX_MS && r.rate() != ULTRA_RW_RATE_500K; ms += 1000)
    win(2, 0);
  for (int k = 0; k < 3601; k++)
    win(2, 0);
  CHECK(r.quiet_needed_ms() == ReplyRate::QUIET_MS, "the try held an hour: the quiet period 1 h again");
  for (int k = 0; k < 3601 * 3; k++)
    win(2, 0);
  CHECK(r.rate() == ULTRA_RW_RATE_1M && r.steps_up() >= 2, "then 1 M, the configured rate, and no higher");
  // configured 500 k: never 1 M; pinned: never changes
  ReplyRate t;
  t.configure(ULTRA_RW_RATE_500K, true);
  ReplyRate p;
  p.configure(ULTRA_RW_RATE_1M, false);
  bool moved = false;
  for (int k = 0; k < 20000; k++) {
    now += 1000;
    t.window(now, 2, k % 2000 < 10 ? 1 : 0);
    moved |= p.window(now, 2, 1);
  }
  CHECK(t.rate() >= ULTRA_RW_RATE_500K && !moved && p.rate() == ULTRA_RW_RATE_1M,
        "configured 500 k: never faster (%u); pinned at 1 M: never moved", t.rate());
}

int main() {
  std::printf("ultra telemetry test (the return channel's hub side)\n");
  test_power_off_cap();
  test_reply_rate();
  test_window_hash();
  test_due_memo();
  test_basic();
  test_parser();
  test_missing();
  test_restart_and_swap();
  test_power_ack();
  test_busy_unsupported_why();
  test_long_chain();
  test_unpowered();
  test_wraps();
  test_ledger_sync();
  test_ledger_fraction();
  test_ledger_boards();
  test_ledger_erased();
  test_ledger_restart();
  test_ledger_misc();
  test_ledger_counters();
  test_pacing();
  test_r9_lock_long_run();
  test_r9_long_runs();
  test_r9_load_replaces();
  test_r9_uid16_collision();
  test_r9_restart_with_lead();
  test_r9_moved_live_board();
  test_record_held();
  test_status();
  test_port();
  test_025();
  std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL ULTRA TELEMETRY TESTS PASSED", failures);
  return failures ? 1 : 0;
}
