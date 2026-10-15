// Return channel contract (protocol 0.2.5; a 0.2.4 firmware too), end to end on a PC: the component's TelemetryPoller (ultra_telemetry.h)
// polls the Diff-Power ultra receiver firmware itself - BOARD=2 of the board firmware, src/app/main.c on the firmware's
// own hardware model (test/test_soak.c with test_soak_rc.c, through tests/ultra_telemetry_receiver.c). The poller's RW
// frames go over the modelled DIN wire into the firmware's real receive path (the receiver sees its own POLL as its
// first slot, as at its chain position); the burst the firmware sends becomes the hub UART's byte stream (the
// turnaround checks' junk bytes, breaks, the 0xFF lead bytes, the other receivers' answers, stray bytes and 0xA5s), and
// what the poller decodes - ReceiverData, the sensor values, the texts - must match the firmware's own state.
// Needs the board firmware (ULTRA_FIRMWARE). Build and run: python tests/run_ultra_telemetry_contract_test.py
// Every window: the RW frame's plan (valid POLLs, one R and rate, rising indices, RELAY_ONLY between), the receiver's
// window against the poller's W (turn around, check, the burst in its slot, forward again by W + END), the answer's
// framing; every answer the poller takes is checked against the firmware's state when it built it; every acknowledge
// the firmware takes must carry the counts the hub read in that boot; every lost answer must be asked again (new SEQ).
//  - pages: one receiver at chain position 0 (OUTPUTS on), two minutes: no RW frame within ULTRA_RW_LOCK_MS, every
//    page, the texts (firmware, reset cause, status, power events, health before / after a calibration test), the
//    status helpers, the lights' power estimates, the lifetime energy (total_increasing within a boot);
//  - soft start: OUTPUTS' WHY while the first light ramps after power-up -> "soft start" / RS_SOFT_START;
//  - power: a short read while tripped ("tripped: short circuit, retry in 1 s"), its acknowledge (POWER slot byte
//    7 = 0x01 with the counts as read) clearing the firmware's latch, the retry's limit event acknowledged in turn, how
//    long the status text lags the firmware; a supply dip (SUPPLY read before its acknowledge) and a limit event after
//    the hub's read (it stays latched through that acknowledge); "limiting to 1.00 A (user limit)"; the events texts;
//  - chain: the receiver at position 2 of 3, 500 kbaud: its index and slot; the first board drops out: the receiver is
//    silenced (its turnaround check finds the line taken) without a miss, the line cut there, CONTENTION afterwards;
//  - relay: the receiver at position 15 of 18, 250 kbaud (W 24.2 ms): RELAY_ONLY when it isn't among the 16 (it turns
//    around, sends nothing, the boards behind it are heard), index 15 otherwise;
//  - lost: a POLL lost on the wire, contention at the check and at the burst, a broken answer, a late main loop, flash
//    writes right before a frame, a stale answer replayed, a restart while the hub polls every 20 ms (the clock not
//    rated yet): every missing answer asked again in a later window with a new SEQ;
//  - restart: an unnoticed watchdog restart while an acknowledge is due (no acknowledge across it, IDENTITY again, the
//    new boot's own short shown and acknowledged), then the hub's own power cycle (unpowered / locked, the lifetime
//    energy kept through the supply dip's write);
//  - retry (a 0.2.5 firmware): LIFETIME and DIAG's two parts against the firmware's record; the retry policy (CONFIG
//    24: no retries): STATUS's reason "retries used up" and the Status's line, a new command switches it on again; the
//    short in LIFETIME's count and the fault memory;
//  - update repeat (auto_retry): a whole update session from the component's sender for the version the receiver runs,
//    and one for other hardware: the receiver decides (already active / not for it) and its light stays as it was,
//    every output every ms, through the session, the quiet second and the answer on its status LED (feedback channel
//    red and all; firmware 0.6.9).

#include "../components/dig2analog_ultra/ultra_ledger.h"
#include "../components/dig2analog_ultra/ultra_telemetry.h"
#include "../components/dig2analog_ultra/update_session.h"
#include "ultra_telemetry_receiver.h"
extern "C" {
#include "energy.h"   // (the firmware's src/core: its lifetime record and the LIFE_SYNC merge, for the month)
}

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace esphome::dig2analog_ultra;

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

static const uint32_t RESET_POWER_PIN = 0x0C000000u;   // RCC RSTSCKR: power-on + pin
static const uint32_t RESET_WATCHDOG = 0x20000000u;    // IWDGRSTF
static const uint32_t FAKE_FEATURES = ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_HEALTH |
                                      ULTRA_FEAT_ENERGY;

static int tp_of(uint8_t page, uint8_t part) {
  for (int i = 0; i < TP_COUNT; i++) {
    if (TP_PAGE_ID[i] == page && TP_PART[i] == part)
      return i;
  }
  return -1;
}

static bool contains(const std::string &s, const char *part) { return s.find(part) != std::string::npos; }

static const char *const TP_NAME[TP_COUNT] = {"IDENTITY", "POWER",  "STATUS",   "OUTPUTS", "SUPPLY",
                                              "ENERGY",   "HEALTH", "LIFETIME", "DIAG",    "DIAG 1"};

// ------------------------------------------------------------------------------- the firmware's state vs the poller
/// The page just answered (as decoded into d) against the firmware's state when it built the answer.
static void verify_page(int tp, const ReceiverData &d, const ucrx_truth_t &t, const char *w) {
  switch (tp) {
    case TP_IDENTITY:
      CHECK(std::memcmp(d.uid, t.uid, 12) == 0 && d.fw_word == t.fw_version && d.bl_version == t.bl_version &&
                d.hw_id == (t.hw_id != 0 ? t.hw_id : t.board) && d.hw_caps == t.hw_caps && d.features == t.features,
            "%s: IDENTITY firmware %08X (%08X), bootloader %u (%u), hw %u, caps %04X (%04X), features %08X (%08X)", w,
            (unsigned) d.fw_word, (unsigned) t.fw_version, d.bl_version, t.bl_version, d.hw_id, d.hw_caps, t.hw_caps,
            (unsigned) d.features, (unsigned) t.features);
      CHECK(telemetry_firmware_text(d) == t.fw_text, "%s: firmware text '%s', the firmware's '%s'", w,
            telemetry_firmware_text(d).c_str(), t.fw_text);
      break;
    case TP_STATUS: {
      const uint16_t ma = static_cast<uint16_t>(t.i_avg_ma < 0 ? 0 : t.i_avg_ma > 65535 ? 65535 : t.i_avg_ma);
      const uint8_t measured = static_cast<uint8_t>((t.vbus_valid ? ULTRA_ST_M_VBUS : 0) |
                                                    (t.isense_enabled ? ULTRA_ST_M_CURRENT : 0) |
                                                    (t.isense_calibrated ? ULTRA_ST_M_CALIBRATED : 0));
      const uint8_t hw = static_cast<uint8_t>(((t.hw_id != 0 ? t.hw_id : t.board) << 4) | (t.hw_rev & 15));
      CHECK(d.fw_version == t.fw_version >> 8 && d.hw == hw && d.uptime_s + 1 >= t.up_s && d.uptime_s <= t.up_s &&
                d.reset_cause == (t.reset_flags >> 26),
            "%s: STATUS version %06X (%06X), hw %02X, uptime %u (%u), reset %02X (%08X)", w, (unsigned) d.fw_version,
            (unsigned) (t.fw_version >> 8), d.hw, (unsigned) d.uptime_s, (unsigned) t.up_s, d.reset_cause,
            (unsigned) t.reset_flags);
      CHECK(d.power_state == (t.power_state | t.why << ULTRA_ST_WHY_SHIFT) &&
                static_cast<int16_t>(d.mcu_temp) == t.mcu_c10 &&
                d.board_temp == ULTRA_ST_NA && d.vbus_10mv == (t.vbus_valid ? t.vbus_mv / 10 : 0xFFFFu) &&
                d.vdda_mv == t.vdda_mv && std::abs(static_cast<int>(d.current_ma) - ma) <= 30 + ma / 50 &&
                d.measured == measured,
            "%s: STATUS state %02X (%02X), MCU %d (%d), V %u x10 mV (%u mV), VDDA %u (%u), I %u mA (%d), measured %02X "
            "(%02X)",
            w, d.power_state, t.power_state | t.why << ULTRA_ST_WHY_SHIFT, static_cast<int16_t>(d.mcu_temp), t.mcu_c10,
            d.vbus_10mv,
            (unsigned) t.vbus_mv, d.vdda_mv, t.vdda_mv, d.current_ma, (int) t.i_avg_ma, d.measured, measured);
      CHECK(d.pwm_preset == t.pwm_preset && d.config_crc == t.config_crc &&
                d.modes == static_cast<uint8_t>((t.proto_mode << 4) | (t.stagger & 15)),
            "%s: STATUS preset %u (%u), CONFIG crc %04X (%04X), modes %02X", w, d.pwm_preset, t.pwm_preset,
            d.config_crc, t.config_crc, d.modes);
      const float v = telemetry_value(d, TV_SUPPLY_VOLTAGE), i = telemetry_value(d, TV_INPUT_CURRENT);
      const float c = telemetry_value(d, TV_MCU_TEMPERATURE);
      CHECK(t.vbus_valid ? std::fabs(v - t.vbus_mv / 1000.0f) <= 0.011f : std::isnan(v),
            "%s: supply voltage %.3f V, the firmware's %u mV", w, v, (unsigned) t.vbus_mv);
      CHECK(t.isense_enabled ? std::fabs(i - d.current_ma / 1000.0f) < 1e-4f : std::isnan(i),
            "%s: input current %.3f A", w, i);
      CHECK(t.mcu_c10 == static_cast<int16_t>(ULTRA_ST_NA) ? std::isnan(c) : std::fabs(c - t.mcu_c10 / 10.0f) < 1e-3f,
            "%s: MCU temperature %.1f C (%d)", w, c, t.mcu_c10);
      CHECK(telemetry_value(d, TV_UPTIME) == static_cast<float>(d.uptime_s), "%s: uptime value", w);
      break;
    }
    case TP_OUTPUTS: {
      const uint8_t why = static_cast<uint8_t>(
          (t.limiting ? ULTRA_OU_W_LIMIT : 0) | (t.tripped ? ULTRA_OU_W_TRIPPED : 0) |
          (t.ch_limited ? ULTRA_OU_W_CH_LIMIT : 0) | (t.dark_hold ? ULTRA_OU_W_DARK_HOLD : 0) |
          (t.pattern ? ULTRA_OU_W_PATTERN : 0) | (t.soft_start ? ULTRA_OU_W_SOFT_START : 0) |
          (t.signal_lost ? ULTRA_OU_W_SIGNAL_LOSS : 0) | (t.test ? ULTRA_OU_W_TEST : 0));
      int off = 0;
      for (int ch = 0; ch < 5; ch++)
        off += std::abs(static_cast<int>(d.level[ch]) - t.out[ch]) > 600;   // (a ramp / breathe moves ~200 a ms)
      CHECK(off == 0 && d.effects == t.fx_lights && d.defined == t.defined_lights && d.scene == t.scene_hold &&
                d.why == why,
            "%s: OUTPUTS %u %u %u %u %u (%u %u %u %u %u), effects %02X (%02X), defined %02X (%02X), scene %02X (%02X), "
            "why %02X (%02X)",
            w, d.level[0], d.level[1], d.level[2], d.level[3], d.level[4], t.out[0], t.out[1], t.out[2], t.out[3],
            t.out[4], d.effects, t.fx_lights, d.defined, t.defined_lights, d.scene, t.scene_hold, d.why, why);
      break;
    }
    case TP_POWER: {
      const bool tripped = t.trip_cause != ULTRA_PW_CAUSE_NONE;
      uint32_t retry = tripped ? (t.retry_ms + 999) / 1000 : 0;
      retry = tripped && retry < 1 ? 1 : retry > 255 ? 255 : retry;
      auto age_ok = [](uint32_t got, uint32_t fw) { return fw == 0xFFFFFF ? got == fw : got == fw || got + 1 == fw; };
      CHECK(d.pw_now == (t.power_state & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING)) &&
                d.pw_latched == t.latched && d.limit_events == t.limit_events && d.overload_trips == t.ovl_trips &&
                d.short_trips == t.short_trips,
            "%s: POWER now %02X (%02X), latched %02X (%02X), limit events %u (%u), overload %u (%u), short %u (%u)", w,
            d.pw_now, t.power_state, d.pw_latched, t.latched, d.limit_events, t.limit_events, d.overload_trips,
            t.ovl_trips, d.short_trips, t.short_trips);
      CHECK(age_ok(d.limit_age_s, t.limit_age_s) && age_ok(d.trip_age_s, t.trip_age_s) && d.limit_ma == t.limit_ma &&
                d.limit_source == (t.limit_src & 15) && d.trip_cause == (t.last_cause & 15) &&
                d.event_ma == t.event_ma && d.trip_ma == t.trip_ma &&
                std::abs(static_cast<int>(d.factor) - (tripped ? 0 : t.factor)) <= (t.limiting ? 1500 : 0) &&
                (d.retry_s == retry || d.retry_s + 1 == retry),
            "%s: POWER ages %u / %u (%u / %u), limit %u mA (%u) src %u (%u), cause %u (%u), event %u mA (%u), trip %u "
            "mA (%u), factor %u (%u), retry %u s (%u)",
            w, (unsigned) d.limit_age_s, (unsigned) d.trip_age_s, (unsigned) t.limit_age_s, (unsigned) t.trip_age_s,
            d.limit_ma, t.limit_ma, d.limit_source, t.limit_src, d.trip_cause, t.last_cause, d.event_ma, t.event_ma,
            d.trip_ma, t.trip_ma, d.factor, t.factor, d.retry_s, (unsigned) retry);
      CHECK(telemetry_value(d, TV_SHORT_TRIPS) == t.short_trips &&
                telemetry_value(d, TV_LIMIT_EVENTS) == t.limit_events &&
                telemetry_value(d, TV_OVERLOAD_TRIPS) == t.ovl_trips,
            "%s: the POWER counters' sensor values", w);
      break;
    }
    case TP_SUPPLY:
      CHECK(d.dips == t.dips && d.logged == t.logged && telemetry_value(d, TV_SUPPLY_DIPS) == t.dips,
            "%s: SUPPLY dips %u (%u), logged %u (%u)", w, d.dips, t.dips, d.logged, t.logged);
      if (t.logged != 0) {
        const uint32_t age = t.up_s - t.dip_t_s;
        CHECK((d.dip_age_s == age || d.dip_age_s + 1 == age) && d.dip_mv == t.dip_mv && d.dip_pct == t.dip_pct &&
                  d.dip_ms == t.dip_ms && d.dip_ma == t.dip_ma,
              "%s: SUPPLY's newest dip: %u s ago (%u), %u mV (%u), %u %% (%u), %u ms (%u), %u mA (%u)", w,
              (unsigned) d.dip_age_s, (unsigned) age, d.dip_mv, t.dip_mv, d.dip_pct, t.dip_pct, d.dip_ms, t.dip_ms,
              d.dip_ma, t.dip_ma);
      } else {
        CHECK(d.dip_age_s == 0xFFFFFF, "%s: SUPPLY without a logged dip: no age (%06X)", w, (unsigned) d.dip_age_s);
      }
      break;
    case TP_HEALTH: {
      bool same = d.he_flags == t.he_flags && d.dark_ma == t.he_dark_ma && d.temp_peak == t.temp_peak;
      for (int ch = 0; ch < 5; ch++)
        same = same && d.ch_class[ch] == t.he_class[ch] && d.test_dev[ch] == t.he_test_dev[ch] &&
               d.model_dev[ch] == t.he_model_dev[ch];
      CHECK(same, "%s: HEALTH flags %02X (%02X), CH1 class %02X (%02X), dark %d mA (%d), peak %d (%d)", w, d.he_flags,
            t.he_flags, d.ch_class[0], t.he_class[0], d.dark_ma, t.he_dark_ma, d.temp_peak, t.temp_peak);
      break;
    }
    case TP_ENERGY: {
      const uint32_t saved_age = t.energy_saved ? t.up_s - t.saved_s : 0xFFFFFF;
      CHECK((d.total_wh == t.total_wh || d.total_wh + 1 == t.total_wh) && d.boot_mwh <= t.boot_mwh &&
                d.boot_mwh + 60 >= t.boot_mwh && (d.saved_age_s == saved_age || d.saved_age_s + 1 == saved_age),
            "%s: ENERGY lifetime %u Wh (%u), since boot %u mWh (%u), saved %u s ago (%u)", w, (unsigned) d.total_wh,
            (unsigned) t.total_wh, (unsigned) d.boot_mwh, (unsigned) t.boot_mwh, (unsigned) d.saved_age_s,
            (unsigned) saved_age);
      // the Energy sensor never goes below what it showed on this board (a restart loses the Wh since its last
      // save); the Power is STATUS's voltage x current when both are measured (fresher), else ENERGY's
      const bool vi = (d.measured & ULTRA_ST_M_VBUS) && (d.measured & ULTRA_ST_M_CURRENT) && d.vbus_10mv != 0xFFFF;
      const float want_w = vi ? d.vbus_10mv * 0.01f * d.current_ma * 0.001f : d.power_mw / 1000.0f;
      CHECK(telemetry_value(d, TV_ENERGY) == static_cast<float>(d.energy_wh) && d.energy_wh >= d.total_wh &&
                std::fabs(telemetry_value(d, TV_INPUT_POWER) - want_w) < 1e-3f,
            "%s: the energy / power values (%u >= %u Wh, %.3f W)", w, (unsigned) d.energy_wh, (unsigned) d.total_wh,
            telemetry_value(d, TV_INPUT_POWER));
      // (0.2.5) the LED supply after the buck; the peak since the ENERGY answer before (at least the current now)
      if (t.has_025) {
        const float vb = telemetry_value(d, TV_LED_SUPPLY), pk = telemetry_value(d, TV_CURRENT_PEAK);
        CHECK(d.vb_10mv == (t.vb_valid ? t.vb_mv / 10 : 0xFFFFu) &&
                  (t.vb_valid ? std::fabs(vb - t.vb_mv / 10 * 0.01f) < 1e-3f : std::isnan(vb)) &&
                  d.peak_ma + 30 + d.peak_ma / 50 >= (t.i_avg_ma > 0 ? t.i_avg_ma : 0) &&
                  std::fabs(pk - d.peak_ma / 1000.0f) < 1e-4f,
              "%s: ENERGY LED supply %u x10 mV (%u mV), peak %u mA (now %d)", w, d.vb_10mv, (unsigned) t.vb_mv,
              d.peak_ma, (int) t.i_avg_ma);
      } else {
        CHECK(std::isnan(telemetry_value(d, TV_LED_SUPPLY)) && std::isnan(telemetry_value(d, TV_CURRENT_PEAK)),
              "%s: a 0.2.4 firmware's ENERGY: no LED supply / peak", w);
      }
      break;
    }
    case TP_LIFETIME:
      CHECK(t.has_025 && d.lt_powered == ((t.lt_on_s / 360u) & 0xFFFFFFu) &&
                d.lt_lit == ((t.lt_lit_s / 360u) & 0xFFFFFFu) && d.lt_power_ups == (t.lt_ups & 0xFFFFFFu) &&
                d.lt_overload_trips == t.lt_trips[0] && d.lt_short_trips == t.lt_trips[1] &&
                d.lt_dips == (t.lt_dips & 0xFFFFFFu) && d.lt_temp_peak == t.lt_peak_t &&
                d.lt_current_peak == t.lt_peak_a && d.lt_watchdog == t.lt_wd && d.lt_updates == t.lt_updates &&
                d.lt_records == (t.lt_writes & 0xFFFFFFu),
            "%s: LIFETIME on %u (%u s), lit %u (%u s), ups %u (%u), trips %u / %u (%u / %u), dips %u (%u), records %u "
            "(%u)",
            w, (unsigned) d.lt_powered, (unsigned) t.lt_on_s, (unsigned) d.lt_lit, (unsigned) t.lt_lit_s,
            (unsigned) d.lt_power_ups, (unsigned) t.lt_ups, d.lt_overload_trips, d.lt_short_trips, t.lt_trips[0],
            t.lt_trips[1], (unsigned) d.lt_dips, (unsigned) t.lt_dips, (unsigned) d.lt_records,
            (unsigned) t.lt_writes);
      CHECK(std::fabs(telemetry_value(d, TV_LIT_HOURS) - d.lt_lit * 0.1f) < 1e-3f &&
                telemetry_value(d, TV_POWER_UPS) == static_cast<float>(d.lt_power_ups) &&
                telemetry_value(d, TV_TRIPS) == static_cast<float>(d.lt_short_trips + d.lt_overload_trips) &&
                telemetry_value(d, TV_DIPS) == static_cast<float>(d.lt_dips),
            "%s: the lifetime values", w);
      break;
    case TP_DIAG: {
      const uint8_t bc = t.bc_valid ? t.bc_cause : 0xFF;
      CHECK(t.has_025 && d.dg_reset == (t.reset_flags >> 26) && d.bc_cause == bc &&
                (!t.bc_valid || (d.bc_pc == t.bc_pc && d.bc_uptime_s == (t.bc_up_s > 0xFFFFFFu ? 0xFFFFFFu : t.bc_up_s))) &&
                d.supply_z_mohm == (t.supply_z > 0xFFFFu ? 0xFFFFu : t.supply_z) && d.flash_errs == t.flash_err &&
                d.c10_load == 0xFF && d.c10_peak == 0xFF,
            "%s: DIAG reset %02X, breadcrumbs %02X (%02X), Z %u mOhm (%u), flash errors %u (%u), C10 %02X", w,
            d.dg_reset, d.bc_cause, bc, d.supply_z_mohm, (unsigned) t.supply_z, d.flash_errs, t.flash_err, d.c10_load);
      const float z = telemetry_value(d, TV_SUPPLY_IMPEDANCE);
      CHECK(d.supply_z_mohm == 0 ? std::isnan(z) : z == static_cast<float>(d.supply_z_mohm), "%s: the impedance %.0f", w,
            z);
      break;
    }
    case TP_FAULTS: {
      bool trend_na = d.trend_lit_h == 0xFFFF && d.trend_tests == 0;
      for (int ch = 0; ch < 5; ch++)
        trend_na = trend_na && d.trend[ch] == ULTRA_HE_NA;
      CHECK(t.has_025 && d.fm_trip == t.fm_trip && d.fm_trip_lit_h == t.fm_trip_h && d.fm_dip_pct == t.fm_dip_pct &&
                d.fm_dip_10ms == t.fm_dip_cs && d.fm_dip_a10 == t.fm_dip_a && d.fm_rst == t.fm_rst &&
                d.fm_rst_up_h == t.fm_rst_h && trend_na,
            "%s: DIAG 1 trip %02X (%02X) at %u h (%u), dip %u %% (%u), reset %02X (%02X), trend %d / %u h / %u tests", w,
            d.fm_trip, t.fm_trip, d.fm_trip_lit_h, t.fm_trip_h, d.fm_dip_pct, t.fm_dip_pct, d.fm_rst, t.fm_rst,
            d.trend[0], d.trend_lit_h, d.trend_tests);
      break;
    }
    case TP_SYNC:   // (0.2.6) the counters after the merge, its result, RECORDS
      CHECK(d.lt_powered == ((t.lt_on_s / 360u) & 0xFFFFFFu) && d.lt_lit == ((t.lt_lit_s / 360u) & 0xFFFFFFu) &&
                d.lt_power_ups == (t.lt_ups & 0xFFFFFFu) && d.lt_dips == (t.lt_dips & 0xFFFFFFu) &&
                d.lt_overload_trips == t.lt_trips[0] && d.lt_short_trips == t.lt_trips[1] &&
                d.total_wh == t.total_wh && d.ls_result <= ULTRA_LS_R_CAPPED &&
                d.ls_records16 == static_cast<uint16_t>(t.lt_writes),
            "%s: LIFE_SYNC on %u (%u s), lit %u (%u s), %u Wh (%u), result %u, applied %02X, records %u (%u)", w,
            (unsigned) d.lt_powered, (unsigned) t.lt_on_s, (unsigned) d.lt_lit, (unsigned) t.lt_lit_s,
            (unsigned) d.total_wh, (unsigned) t.total_wh, d.ls_result, d.ls_applied, d.ls_records16,
            (unsigned) t.lt_writes);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------------------- the other receivers
/// A receiver of the chain that isn't the firmware: answers its POLLs with plain data (valid framing, its own UID).
struct Fake {
  uint8_t uid[12]{};
  uint16_t uid16{0};
  uint32_t boot_ms{0}, features{0};
  bool present{true};       // takes part in the windows (false: unplugged, or its POLL lost)
  bool contention{false};   // its last check / burst found the line taken: CONTENTION in its next answer

  void init(int pos, uint32_t now_ms, uint32_t feat) {
    for (int i = 0; i < 12; i++)
      this->uid[i] = static_cast<uint8_t>(0x31 * pos + 7 * i + 1);
    this->uid16 = ultra_crc16(this->uid, 12);
    this->boot_ms = now_ms - 40000;   // up for 40 s
    this->features = feat;
  }
  /// Takes part (turns around) in a window with this slot.
  bool takes_part(const uint8_t *s) const {
    return this->present && ultra_slot_check(s) == 0 && ultra_poll_valid(s) == 0;
  }
  /// Its answer (32 bytes) to its POLL.
  void answer(const uint8_t *s, uint32_t now_ms, uint8_t *a) {
    uint8_t page = s[ULTRA_POLL_PAGE], *d = a + ULTRA_RSP_DATA;
    std::memset(a, 0, ULTRA_SLOT_BYTES);
    switch (page) {
      case ULTRA_PAGE_IDENTITY:
        std::memcpy(d, this->uid, 12);
        ultra_put32(d + ULTRA_ID_FW_VERSION, 0x000608FFu);
        d[ULTRA_ID_HW_ID] = 2;
        ultra_put16(d + ULTRA_ID_HW_CAPS, ULTRA_HWCAP_RETURN | ULTRA_HWCAP_RELAY);
        ultra_put32(d + ULTRA_ID_FEATURES, this->features);
        break;
      case ULTRA_PAGE_STATUS:
        ultra_put24(d + ULTRA_ST_FW_VERSION, 0x000608);
        d[ULTRA_ST_HW] = 0x20;
        ultra_put24(d + ULTRA_ST_UPTIME, (now_ms - this->boot_ms) / 1000);
        d[ULTRA_ST_RESET] = ULTRA_ST_RST_POWER;
        ultra_put16(d + ULTRA_ST_MCU_TEMP, ULTRA_ST_NA);
        ultra_put16(d + ULTRA_ST_TEMP_BOARD, ULTRA_ST_NA);
        ultra_put16(d + ULTRA_ST_VBUS, 0xFFFF);
        break;
      case ULTRA_PAGE_POWER:
        ultra_put24(d + ULTRA_PW_LIMIT_AGE, 0xFFFFFF);
        ultra_put24(d + ULTRA_PW_TRIP_AGE, 0xFFFFFF);
        ultra_put16(d + ULTRA_PW_LIMIT_MA, 0xFFFF);
        ultra_put16(d + ULTRA_PW_FACTOR, 65535);
        break;
      case ULTRA_PAGE_SUPPLY:
        ultra_put24(d + ULTRA_SU_DEEP_AGE, 0xFFFFFF);
        ultra_put24(d + ULTRA_SU_EV_AGE, 0xFFFFFF);
        break;
      case ULTRA_PAGE_HEALTH:
        for (int i = 0; i < 5; i++)
          d[ULTRA_HE_TEST_DEV + i] = d[ULTRA_HE_MODEL_DEV + i] = static_cast<uint8_t>(ULTRA_HE_NA);
        ultra_put16(d + ULTRA_HE_TEMP_PEAK, ULTRA_ST_NA);
        ultra_put24(d + ULTRA_HE_TEMP_AGE, 0xFFFFFF);
        break;
      case ULTRA_PAGE_ENERGY:
        ultra_put24(d + ULTRA_EN_SAVED_AGE, 0xFFFFFF);
        break;
      case ULTRA_PAGE_OUTPUTS:
        d[ULTRA_OU_DEFINED] = 0x01;
        break;
      default:
        page = ULTRA_PAGE_UNSUPPORTED;
        break;
    }
    a[0] = ULTRA_RSP_SYNC;
    a[1] = page;
    a[2] = static_cast<uint8_t>((s[ULTRA_POLL_INDEX] << 4) | (this->contention ? ULTRA_RSP_CONTENTION : 0));
    a[3] = s[2];
    ultra_put16(a + 4, this->uid16);
    ultra_put16(a + 30, ultra_crc16(a, 30));
    this->contention = false;
  }
};

// --------------------------------------------------------------------------------------------------- the bench
/// The hub (its TelemetryPoller, the RW frames, the UART), the receiver at chain position k and the fakes around it.
static bool hub_lit = false;   // the hub's lights on (lights_on): the ledger's lit time (LedgerKeeper::lit_tick)

struct Bench {
  const char *name;
  TelemetryPoller p;
  uint16_t n{1}, k{0};
  std::vector<Fake> fakes;
  std::mt19937 rng{12345};
  // the next window's disturbances
  bool lose_poll{false}, break_answer{false}, replay{false}, contend_check{false}, contend_tx{false};
  uint32_t late_us{0};
  // the last window
  ucrx_window_t o{};
  ucrx_truth_t t{};
  uint8_t poll[ULTRA_SLOT_BYTES]{};   // the receiver's slot as the hub built it
  bool polled{false}, asked{false}, heard{false};   // it got a POLL / one with an answer index / its answer arrived
  int tp{-1};                         // the page asked
  uint32_t updates{0}, upd_all{0};
  std::vector<uint8_t> stale;         // an earlier answer (FF + 32 bytes)
  // counts
  uint32_t windows{0}, n_asked{0}, n_heard{0}, n_relay{0}, n_lost{0}, n_retried{0}, n_false{0}, n_eaten{0};
  uint32_t n_restart_upd{0};          // take_updates() with TU_RESTART
  uint32_t reads[TP_COUNT]{};
  struct Lost {
    int tp;
    uint8_t seq;
    uint32_t window;
  };
  std::vector<Lost> pending;          // lost answers waiting for their page's next answer
  // the acknowledge (per boot of the receiver): the counts of the POWER / SUPPLY answers heard since it booted
  int boot{0};
  bool power_read{false}, supply_read{false};
  uint16_t read_cnt[3]{}, read_dips{0};
  uint32_t acks{0}, acks_across{0}, acks_wrong{0};
  float last_energy{NAN};
  // the hub's lifetime ledger (ultra_ledger.h): the position's Energy never back, across every restart
  LifetimeLedger led;
  LedgerKeeper keeper;
  double last_position{-1};

  explicit Bench(const char *nm) : name(nm) { hub_lit = false; }

  void power_up(int keep_flash, uint32_t reset_flags) {
    ucrx_power_up(keep_flash, reset_flags);
    this->boot++;
    this->power_read = this->supply_read = false;
    this->last_energy = NAN;   // (total_increasing within a boot; across a restart: sc_restart)
  }
  void start(uint16_t receivers, uint16_t pos, uint8_t rate, const TelemetryIntervals &iv, bool outputs = false) {
    this->n = receivers;
    this->k = pos;
    this->p.init(receivers, rate, iv);
    this->p.set_outputs(pos, outputs);   // (OUTPUTS: off by default, a receiver with several lights)
    this->fakes.assign(receivers, Fake{});
    for (uint16_t i = 0; i < receivers; i++)
      this->fakes[i].init(i, ucrx_hub_ms(), FAKE_FEATURES);
    this->led.init(receivers + 4u, receivers);
    this->keeper.attach(&this->led, 5u << 16, receivers, ucrx_hub_ms());
    this->p.locked(ucrx_hub_ms());   // (the hub's preamble: the soak's frames from the power-up on)
  }

  /// The hub's loop for ms: an RW frame whenever the poller has one due, else the model runs (normal frames).
  void run(uint32_t ms) {
    const uint32_t end = ucrx_hub_ms() + ms;
    while (static_cast<int32_t>(ucrx_hub_ms() - end) < 0) {
      if (!this->window())
        ucrx_run_ms(1);
    }
  }
  /// Runs until cond() (checked after every window / ms), at most ms.
  template<typename F> bool run_until(uint32_t ms, F cond) {
    const uint32_t end = ucrx_hub_ms() + ms;
    while (static_cast<int32_t>(ucrx_hub_ms() - end) < 0) {
      if (!this->window())
        ucrx_run_ms(1);
      if (cond())
        return true;
    }
    return false;
  }

  void junk(std::vector<uint8_t> &u) {
    const uint32_t len = 1 + this->rng() % 40;
    for (uint32_t i = 0; i < len; i++)
      u.push_back(this->rng() % 4 == 0 ? ULTRA_RSP_SYNC : static_cast<uint8_t>(this->rng()));
  }

  /// One RW frame and its window if the poller has one due; false: nothing due.
  bool window() {
    const uint32_t now = ucrx_hub_ms();
    if (!this->p.due(now))
      return false;
    std::vector<uint8_t> f(ULTRA_SLOT_BYTES * this->n);
    uint32_t w_us = 0;
    const uint16_t m = this->p.build(now, f.data(), &w_us);
    CHECK(m != 0, "%s: due, but build() polls nobody", this->name);
    if (m == 0)
      return false;
    this->windows++;
    for (uint16_t b = m; b < this->n; b++)
      ultra_slot_nop(f.data() + ULTRA_SLOT_BYTES * b);   // (behind them: normal slots)
    // the plan: every POLL valid, one R and one rate, unique rising indices, RELAY_ONLY between
    const uint8_t R = f[ULTRA_POLL_SLOTS], rate = f[ULTRA_POLL_RATE];
    int last = -1;
    bool plan_ok = R >= 1 && R <= ULTRA_RW_MAX_SLOTS && rate == this->p.rate() &&
                   w_us == ultra_rw_window_us(R, rate);
    for (uint16_t b = 0; b < m; b++) {
      const uint8_t *s = f.data() + ULTRA_SLOT_BYTES * b;
      plan_ok = plan_ok && ultra_slot_check(s) == 0 && ultra_poll_valid(s) == 0 && s[ULTRA_POLL_SLOTS] == R &&
                s[ULTRA_POLL_RATE] == rate;
      if (s[ULTRA_POLL_INDEX] != ULTRA_POLL_RELAY_ONLY) {
        plan_ok = plan_ok && s[ULTRA_POLL_INDEX] == last + 1;
        last = s[ULTRA_POLL_INDEX];
      }
    }
    plan_ok = plan_ok && last == R - 1 && f[ULTRA_SLOT_BYTES * (m - 1) + ULTRA_POLL_INDEX] == R - 1;
    CHECK(plan_ok, "%s: window %u: the RW frame's plan (%u POLLs, R %u, rate %u, W %u us)", this->name,
          (unsigned) this->windows, m, R, rate, (unsigned) w_us);

    // the receiver's slot, as the hub built it and as the wire delivers it
    uint8_t *own = f.data() + ULTRA_SLOT_BYTES * this->k;
    this->polled = this->k < m;
    std::memcpy(this->poll, own, ULTRA_SLOT_BYTES);
    this->asked = this->polled && own[ULTRA_POLL_INDEX] != ULTRA_POLL_RELAY_ONLY;
    this->tp = this->asked ? tp_of(own[ULTRA_POLL_PAGE], own[ULTRA_POLL_PAGE] == ULTRA_PAGE_DIAG ? own[ULTRA_POLL_ARG] : 0)
                           : -1;
    const bool ack =
        this->asked && own[ULTRA_POLL_PAGE] == ULTRA_PAGE_POWER && (own[ULTRA_POLL_ARG] & ULTRA_PW_ARG_ACK);
    if (this->asked) {
      this->n_asked++;
      for (const Lost &l : this->pending) {
        CHECK(l.tp != this->tp || l.seq != own[2], "%s: window %u: a lost page asked again with its old SEQ %u",
              this->name, (unsigned) this->windows, own[2]);
      }
    }
    if (this->polled && !this->asked)
      this->n_relay++;
    // the boards in front of it: one that doesn't take part leaves the line driven: its turnaround check finds it
    bool front_ok = true;
    for (uint16_t b = 0; b < this->k && b < m; b++)
      front_ok = front_ok && this->fakes[b].takes_part(f.data() + ULTRA_SLOT_BYTES * b);
    std::vector<uint8_t> view(f.begin() + ULTRA_SLOT_BYTES * this->k, f.end());
    if (this->lose_poll && this->polled)
      view[20] ^= 0x10;   // (a bit error on the wire: the receiver's CRC fails)
    ucrx_contend(this->contend_check || (this->polled && !front_ok), this->contend_tx);
    bool behind_answers = false;
    for (uint16_t b = this->k + 1; b < m; b++)
      behind_answers = behind_answers || f[ULTRA_SLOT_BYTES * b + ULTRA_POLL_INDEX] != ULTRA_POLL_RELAY_ONLY;
    ucrx_window(view.data(), this->n - this->k, ULTRA_HDR_RW, w_us, this->late_us, behind_answers, &this->o, &this->t);
    ucrx_contend(0, 0);

    // the acknowledge: applied by the firmware when it took the POLL (rated; also when it was late afterwards)
    if (ack && (this->o.took_part || (this->o.missed && this->t.clock_rated))) {
      this->acks++;
      if (!this->power_read) {
        this->acks_across++;
      } else {
        const uint8_t *a = own + ULTRA_POLL_ARG;
        if (ultra_get16(a + 1) != this->read_cnt[0] || ultra_get16(a + 3) != this->read_cnt[1] ||
            ultra_get16(a + 5) != this->read_cnt[2] || ultra_get16(a + 7) != (this->supply_read ? this->read_dips : 0))
          this->acks_wrong++;
      }
    }
    // the receiver's window against the poller's: the burst in its slot, forward again by W + END
    if (this->polled && this->o.took_part) {
      const double s_i = ULTRA_RW_FIRST_US + ultra_rw_slot_us(rate) * (this->asked ? own[ULTRA_POLL_INDEX] : 0);
      CHECK(this->o.turn_us >= ULTRA_RW_DIR_MIN_US && this->o.turn_us <= ULTRA_RW_DIR_MAX_US && this->o.checked &&
                this->o.check_us >= ULTRA_RW_DIR_MAX_US && this->o.check_us <= ULTRA_RW_FIRST_US &&
                this->o.fwd_us >= w_us && this->o.fwd_us <= w_us + ULTRA_RW_END_US,
            "%s: window %u: turned around at %.1f us, check %.1f, forward again %.1f (W %u)", this->name,
            (unsigned) this->windows, this->o.turn_us, this->o.check_us, this->o.fwd_us, (unsigned) w_us);
      if (this->o.sent) {
        CHECK(this->asked && this->o.tx_us >= s_i - ULTRA_RW_EARLY_US && this->o.tx_us <= s_i + ULTRA_RW_LATE_US,
              "%s: window %u: the burst at %.1f us (slot %u at %.0f)", this->name, (unsigned) this->windows,
              this->o.tx_us, own[ULTRA_POLL_INDEX], s_i);
        CHECK(this->o.answer[0] == ULTRA_RSP_SYNC && ultra_answer_check(this->o.answer) == 0 &&
                  (this->o.answer[2] >> 4) == own[ULTRA_POLL_INDEX] && this->o.answer[3] == own[2] &&
                  ultra_get16(this->o.answer + 4) == ultra_crc16(this->t.uid, 12),
              "%s: window %u: the answer's framing (index %u, SEQ %u / %u)", this->name, (unsigned) this->windows,
              this->o.answer[2] >> 4, this->o.answer[3], own[2]);
      } else {
        CHECK(!this->asked || this->o.quiet, "%s: window %u: asked (index %u), took part, sent nothing, not quiet",
              this->name, (unsigned) this->windows, own[ULTRA_POLL_INDEX]);
      }
    }

    // the hub's UART: the turnaround checks' pulses, then the bursts in index order (lead byte, answer, break)
    std::vector<std::vector<uint8_t>> by_index(R);
    bool chain_ok = true;
    int pulses = 0;
    for (uint16_t b = 0; b < m; b++) {
      const uint8_t *s = f.data() + ULTRA_SLOT_BYTES * b;
      const uint8_t idx = s[ULTRA_POLL_INDEX];
      bool part;
      if (b == this->k) {
        part = this->o.took_part && !this->o.quiet;
        pulses += this->o.checked && chain_ok;
        if (this->o.sent && idx < R) {
          by_index[idx].assign(this->o.answer, this->o.answer + ULTRA_SLOT_BYTES);
          if (this->break_answer)
            by_index[idx][12] ^= 0x04;   // (a bit error on the way back)
        }
      } else {
        Fake &fk = this->fakes[b];
        part = fk.takes_part(s);
        if (part && !chain_ok) {
          fk.contention = true;   // its check reads the line taken
          part = false;
        } else if (part) {
          pulses++;
          if (idx != ULTRA_POLL_RELAY_ONLY) {
            by_index[idx].resize(ULTRA_SLOT_BYTES);
            fk.answer(s, now, by_index[idx].data());
          }
        }
      }
      chain_ok = chain_ok && part;
    }
    std::vector<uint8_t> u;
    std::vector<size_t> starts;   // where the answers' SYNC bytes are
    for (int i = 0; i < pulses; i++)
      u.push_back(static_cast<uint8_t>(0xFF00 >> (this->rng() % 9)));   // (a pulse: 0x00 .. 0xFF with a framing error)
    const bool replayed = this->replay && !this->stale.empty();
    if (replayed) {
      starts.push_back(u.size() + 1);
      u.insert(u.end(), this->stale.begin(), this->stale.end());
    }
    if (this->rng() % 2)
      this->junk(u);
    for (uint8_t i = 0; i < R; i++) {
      if (by_index[i].empty())
        continue;
      if (this->rng() % 3 == 0)
        u.push_back(0x00);   // (the line LOW before the burst: a break)
      u.push_back(ULTRA_RSP_LEAD);
      starts.push_back(u.size());
      u.insert(u.end(), by_index[i].begin(), by_index[i].end());
      u.push_back(0x00);     // (the line LOW again after it)
      if (this->rng() % 4 == 0)
        this->junk(u);
    }
    if (this->o.sent) {
      this->stale.assign(1, ULTRA_RSP_LEAD);
      this->stale.insert(this->stale.end(), this->o.answer, this->o.answer + ULTRA_SLOT_BYTES);
    }
    this->heard = this->asked && this->o.sent && !this->break_answer && front_ok;
    // a SYNC in the junk whose 32 bytes pass the CRC by chance (1 in 65,536): a false answer
    uint32_t false_answers = 0;
    for (size_t i = 0; i + ULTRA_SLOT_BYTES <= u.size(); i++) {
      bool real = false;
      for (size_t st : starts)
        real = real || st == i;
      false_answers += !real && ultra_answer_check(u.data() + i) == 0;
    }
    this->n_false += false_answers;
    const auto st0 = this->p.stats();
    this->p.finish(ucrx_hub_ms(), u.data(), u.size());
    CHECK(this->p.stats().stale >= st0.stale + (replayed ? 1u : 0u) &&
              this->p.stats().stale <= st0.stale + (replayed ? 1u : 0u) + false_answers,
          "%s: window %u: stale answers %u -> %u (POLL seq %u index %u page %02X; answer sent %d: page %02X "
          "index/flags %02X seq %u)",
          this->name, (unsigned) this->windows, (unsigned) st0.stale, (unsigned) this->p.stats().stale, own[2],
          own[ULTRA_POLL_INDEX], own[ULTRA_POLL_PAGE], this->o.sent, this->o.answer[1], this->o.answer[2],
          this->o.answer[3]);
    this->lose_poll = this->break_answer = this->replay = this->contend_check = this->contend_tx = false;
    this->late_us = 0;

    // what the poller made of it
    this->updates = this->p.take_updates(this->k);
    this->upd_all |= this->updates;
    this->keeper.lit_tick(this->k, hub_lit, ucrx_hub_ms());   // (as UltraHub::ledger_line_)
    for (uint16_t b = 0; b < this->n; b++)   // (as UltraHub::rw_publish_: every receiver's news to the ledger)
      this->keeper.on_updates(this->p, b, b == this->k ? this->updates : this->p.take_updates(b), ucrx_hub_ms());
    const double pos = this->keeper.energy_wh(this->k);
    if (pos >= 0) {
      CHECK(pos >= this->last_position, "%s: window %u: the position's Energy %.3f Wh after %.3f", this->name,
            (unsigned) this->windows, pos, this->last_position);
      this->last_position = pos;
    }
    char w[96];
    std::snprintf(w, sizeof(w), "%s: window %u (%s)", this->name, (unsigned) this->windows,
                  this->tp >= 0 ? TP_NAME[this->tp] : "-");
    if (this->heard && false_answers != 0 && this->tp >= 0 && !(this->updates & (1u << this->tp))) {
      // the false answer overlapped this one: the poller skipped 32 bytes after it (see the report)
      std::printf("  NOTE %s: window %u: a false answer in the junk (CRC by chance) took the receiver's answer with "
                  "it\n",
                  this->name, (unsigned) this->windows);
      this->heard = false;
      this->n_eaten++;
    }
    if (this->updates & TU_RESTART)
      this->n_restart_upd++;
    if (this->heard) {
      this->n_heard++;
      const uint8_t page = this->o.answer[1];
      CHECK(page == TP_PAGE_ID[this->tp] || page == ULTRA_PAGE_UNSUPPORTED, "%s: answered page %02X", w, page);
      if (page == TP_PAGE_ID[this->tp]) {
        this->reads[this->tp]++;
        CHECK(this->updates & (1u << this->tp), "%s: the poller didn't take the answer (updates %08X)", w,
              (unsigned) this->updates);
        verify_page(this->tp, this->p.data(this->k), this->t, w);
        if (this->tp == TP_POWER) {
          this->power_read = true;
          this->read_cnt[0] = this->p.data(this->k).limit_events;
          this->read_cnt[1] = this->p.data(this->k).overload_trips;
          this->read_cnt[2] = this->p.data(this->k).short_trips;
        } else if (this->tp == TP_SUPPLY) {
          this->supply_read = true;
          this->read_dips = this->p.data(this->k).dips;
        } else if (this->tp == TP_ENERGY) {
          const float e = telemetry_value(this->p.data(this->k), TV_ENERGY);
          CHECK(std::isnan(this->last_energy) || e >= this->last_energy, "%s: lifetime energy %.0f Wh after %.0f", w,
                e, this->last_energy);
          this->last_energy = e;
        }
      }
      for (size_t i = 0; i < this->pending.size();) {
        if (this->pending[i].tp == this->tp) {
          this->n_retried++;
          this->pending.erase(this->pending.begin() + i);
        } else {
          i++;
        }
      }
      CHECK(this->p.answering(this->k) && this->p.misses(this->k) == 0 && this->p.has_uid(this->k) &&
                this->p.uid16(this->k) == ultra_crc16(this->t.uid, 12),
            "%s: answering %d, misses %u, UID16 %04X", w, this->p.answering(this->k), this->p.misses(this->k),
            this->p.uid16(this->k));
    } else if (this->asked) {
      this->n_lost++;
      this->pending.push_back(Lost{this->tp, own[2], this->windows});
    }
    return true;
  }

  /// Every lost answer's page was answered again since (asked with a new SEQ: checked as it goes).
  void check_retried() {
    for (const Lost &l : this->pending) {
      CHECK(false, "%s: the answer lost in window %u (page %d, SEQ %u) was never asked again and answered", this->name,
            (unsigned) l.window, l.tp, l.seq);
    }
    this->pending.clear();
  }
  const ReceiverData &d() const { return this->p.data(this->k); }
  void summary() const {
    const auto &s = this->p.stats();
    CHECK(this->p.polls(this->k) == this->n_asked && this->p.missed(this->k) == this->n_lost,
          "%s: the poller's count of the receiver's polls %u (%u), missed %u (%u)", this->name,
          (unsigned) this->p.polls(this->k), (unsigned) this->n_asked, (unsigned) this->p.missed(this->k),
          (unsigned) this->n_lost);
    std::printf("  %s: %u windows, receiver asked %u, answered %u, relay only %u, lost %u (asked again: %u); poller: "
                "answers %u, missing %u, bad CRC / junk SYNC %u, stale %u, contention %u, restarts %u; false answers "
                "in the junk %u\n",
                this->name, (unsigned) this->windows, (unsigned) this->n_asked, (unsigned) this->n_heard,
                (unsigned) this->n_relay, (unsigned) this->n_lost, (unsigned) this->n_retried, (unsigned) s.answers,
                (unsigned) s.missing, (unsigned) s.bad_crc, (unsigned) s.stale, (unsigned) s.contention,
                (unsigned) s.restarts, (unsigned) this->n_false);
  }
};

static TelemetryIntervals intervals(uint32_t status, uint32_t power, uint32_t slow) {
  TelemetryIntervals iv;
  iv.status_ms = status;
  iv.power_ms = power;
  iv.energy_ms = iv.supply_ms = iv.health_ms = slow;
  return iv;
}

static void lights_on(uint16_t m) {
  ucrx_cmd_def(0, 0, 3, 1000);   // RGB on CH1-3: 3 x 1.5 A at full
  ucrx_cmd_set(0, m, 0);
  hub_lit = m != 0;
}

// ---------------------------------------------------------------------------------------------------- scenarios
static void sc_pages() {
  std::printf("pages: one receiver at chain position 0, two minutes\n");
  Bench b("pages");
  b.power_up(0, RESET_POWER_PIN);
  b.start(1, 0, ULTRA_RW_RATE_1M, TelemetryIntervals{}, true);
  lights_on(65535);
  static const uint8_t BREATHE[7] = {0, 0x0F, 0xA0, 0x27, 0x10, 0xFF, 0xFF};   // 4 s, 0.15 .. 1
  ucrx_cmd_def(1, 3, 1, 1000);
  ucrx_cmd_set(1, 40000, 0);
  ucrx_cmd_effect(1, ULTRA_FX_BREATHE, BREATHE, 7);
  // no RW frame in the first ULTRA_RW_LOCK_MS after the preamble
  bool early = false;
  for (int i = 0; i < 990; i++) {
    early = early || b.p.due(ucrx_hub_ms());
    ucrx_run_ms(1);
  }
  CHECK(!early, "pages: an RW frame was due within ULTRA_RW_LOCK_MS of the preamble");
  b.run(4000);
  const ReceiverData &d = b.d();
  CHECK(d.have_identity && d.have_status && d.have_outputs && d.have_power && d.have_supply && d.have_energy &&
            d.have_health && b.p.answering(0),
        "pages: every page read within 4 s of the first window (%d %d %d %d %d %d %d)", d.have_identity, d.have_status,
        d.have_outputs, d.have_power, d.have_supply, d.have_energy, d.have_health);
  CHECK(b.t.has_025 ? d.have_lifetime && d.have_diag && d.have_faults && receiver_025(d)
                    : !d.have_lifetime && !d.have_diag && !d.have_faults && !receiver_025(d),
        "pages: the 0.2.5 pages read when the firmware has them (%d: %d %d %d)", b.t.has_025, d.have_lifetime,
        d.have_diag, d.have_faults);
  CHECK(b.upd_all & TU_AVAILABILITY, "pages: the receiver became available");
  CHECK(telemetry_reset_text(d.reset_cause) == "pin, power-on" && telemetry_last_reset(d) == "pin, power-on",
        "pages: reset '%s' / '%s'", telemetry_reset_text(d.reset_cause).c_str(), telemetry_last_reset(d).c_str());
  CHECK(telemetry_status_text(d) == "ok" && telemetry_power_events_text(d) == "no events",
        "pages: status '%s', power events '%s'", telemetry_status_text(d).c_str(),
        telemetry_power_events_text(d).c_str());
  CHECK(telemetry_health_text(d) == "not tested", "pages: health before a test '%s'",
        telemetry_health_text(d).c_str());
  CHECK(d.effects == 0x02 && d.defined == 0x03, "pages: OUTPUTS effects %02X, defined %02X", d.effects, d.defined);
  CHECK(receiver_status(d, b.p.answering(0)) == RS_OK && telemetry_last_trip(d) == "none" &&
            telemetry_last_dip(d) == "none",
        "pages: status %s, last trip '%s', last dip '%s'", receiver_status_name(receiver_status(d, b.p.answering(0))),
        telemetry_last_trip(d).c_str(), telemetry_last_dip(d).c_str());
  {
    const float rgb = light_power_estimate(d, 0x07), ch4 = light_power_estimate(d, 0x08),
                all = telemetry_value(d, TV_INPUT_POWER);
    CHECK(rgb > 0.5f * all && ch4 > 0.0f && rgb + ch4 <= all * 1.001f + 0.01f,
          "pages: the lights' power estimates %.1f W + %.1f W of %.1f W", rgb, ch4, all);
  }
  std::printf("  firmware %s, reset %s, %.2f V, %.3f A, MCU %.1f C, uptime %.0f s, status '%s', health '%s'\n",
              telemetry_firmware_text(d).c_str(), telemetry_reset_text(d.reset_cause).c_str(),
              telemetry_value(d, TV_SUPPLY_VOLTAGE), telemetry_value(d, TV_INPUT_CURRENT),
              telemetry_value(d, TV_MCU_TEMPERATURE), telemetry_value(d, TV_UPTIME), telemetry_status_text(d).c_str(),
              telemetry_health_text(d).c_str());
  // a calibration test: HEALTH tested (read with its next interval)
  ucrx_cmd_identify(ULTRA_IDENT_CAL, 0);
  // the hub's estimate of the lights' energy (EnergyCounter over light_power_estimate of every channel, fed after each
  // answer) against the firmware's own input energy over the same time; the power sensor against the firmware's power
  EnergyCounter ec;
  ucrx_truth_t e0, now;
  ucrx_truth(&e0);
  float worst_w = 0.0f;
  uint32_t seen_w = b.windows, worst_age = 0, last_energy_read = ucrx_hub_ms();
  b.run_until(125000, [&] {
    if (b.windows == seen_w)
      return false;
    seen_w = b.windows;
    if (b.heard && b.tp == TP_ENERGY)
      last_energy_read = ucrx_hub_ms();
    ec.add(light_power_estimate(d, 0x1F), ucrx_hub_ms());
    if (b.heard && b.tp == TP_STATUS) {
      const float fw_w = b.t.vbus_mv / 1000.0f * b.t.i_avg_ma / 1000.0f;
      const float dw = std::fabs(telemetry_value(d, TV_INPUT_POWER) - fw_w);
      if (dw > worst_w) {
        worst_w = dw;
        worst_age = ucrx_hub_ms() - last_energy_read;
      }
    }
    return false;
  });
  ucrx_truth(&now);
  const double fw_wh = (now.boot_mwh - e0.boot_mwh) / 1000.0;
  std::printf("  energy over %.0f s: the hub's EnergyCounter %.2f Wh, the firmware's %.2f Wh; the power sensor up to "
              "%.1f W off the firmware's V x I (its ENERGY read %u s old)\n",
              125.0, ec.wh, fw_wh, worst_w, (unsigned) (worst_age / 1000));
  CHECK(std::fabs(ec.wh - fw_wh) <= 0.1 * fw_wh, "pages: the lights' energy estimate %.2f Wh, the firmware's %.2f Wh",
        ec.wh, fw_wh);
  CHECK(b.reads[TP_HEALTH] >= 2 && b.reads[TP_ENERGY] >= 2 && b.reads[TP_SUPPLY] >= 2 && b.reads[TP_STATUS] >= 35 &&
            b.reads[TP_POWER] >= 20 && b.reads[TP_OUTPUTS] >= 20 && b.reads[TP_IDENTITY] == 1,
        "pages: reads per page: IDENTITY %u, POWER %u, STATUS %u, OUTPUTS %u, SUPPLY %u, ENERGY %u, HEALTH %u",
        (unsigned) b.reads[0], (unsigned) b.reads[1], (unsigned) b.reads[2], (unsigned) b.reads[3],
        (unsigned) b.reads[4], (unsigned) b.reads[5], (unsigned) b.reads[6]);
  CHECK(telemetry_health_text(d) == "ok" && (d.he_flags & ULTRA_HE_F_TESTED) && d.ch_class[0] == ULTRA_HE_NORMAL,
        "pages: health after the test '%s' (flags %02X, CH1 %02X)", telemetry_health_text(d).c_str(), d.he_flags,
        d.ch_class[0]);
  CHECK(telemetry_value(d, TV_ENERGY) >= 1.0f && d.boot_mwh > 1000, "pages: lifetime energy %.0f Wh, %u mWh this boot",
        telemetry_value(d, TV_ENERGY), (unsigned) d.boot_mwh);
  CHECK(b.p.stats().bad_crc > 0 && b.p.stats().stale == 0 && b.p.stats().restarts == 0,
        "pages: junk SYNC bytes skipped (%u), stale %u, restarts %u", (unsigned) b.p.stats().bad_crc,
        (unsigned) b.p.stats().stale, (unsigned) b.p.stats().restarts);
  CHECK(b.n_lost <= 3, "pages: %u answers lost without a disturbance (the clock not rated at the first windows, a "
        "flash write right before a frame)", (unsigned) b.n_lost);
  CHECK(b.acks == 0, "pages: acknowledges without an event (%u)", (unsigned) b.acks);
  b.check_retried();
  std::printf("  after the test: health '%s', lifetime %.0f Wh, %.1f W\n", telemetry_health_text(d).c_str(),
              telemetry_value(d, TV_ENERGY), telemetry_value(d, TV_INPUT_POWER));
  b.summary();
}

static void sc_soft_start() {
  std::printf("soft start: OUTPUTS while the first light after power-up ramps\n");
  Bench b("soft start");
  b.power_up(0, RESET_POWER_PIN);
  b.start(1, 0, ULTRA_RW_RATE_1M, intervals(3000, 40, 60000), true);
  b.run(2500);
  CHECK(b.d().have_outputs && !(b.d().why & ULTRA_OU_W_SOFT_START), "soft start: dark at first (why %02X)",
        b.d().why);
  lights_on(50000);
  bool seen = b.run_until(1500, [&] { return contains(telemetry_status_text(b.d()), "soft start"); });
  CHECK(seen, "soft start: no OUTPUTS answer showed the ramp (status '%s')", telemetry_status_text(b.d()).c_str());
  CHECK(receiver_status(b.d(), true) == RS_SOFT_START &&
            contains(receiver_status_detail(b.d(), RS_SOFT_START, 0, -1), "Soft start"),
        "soft start: status %s", receiver_status_name(receiver_status(b.d(), true)));
  std::printf("  during the ramp: status '%s' (why %02X)\n", telemetry_status_text(b.d()).c_str(), b.d().why);
  b.run(1500);
  CHECK(telemetry_status_text(b.d()) == "ok" && b.d().why == 0, "soft start: after the ramp '%s'",
        telemetry_status_text(b.d()).c_str());
  b.check_retried();
  b.summary();
}

static void sc_power() {
  std::printf("power: a short, a limit event after the read, a supply dip, the limit\n");
  Bench b("power");
  b.power_up(0, RESET_POWER_PIN);
  b.start(1, 0, ULTRA_RW_RATE_1M, intervals(500, 5000, 60000), true);   // (STATUS twice a second: the trip is seen)
  lights_on(65535);
  b.run(5000);
  CHECK(b.d().have_power && b.d().pw_latched == 0 && telemetry_power_events_text(b.d()) == "no events",
        "power: nothing at first ('%s')", telemetry_power_events_text(b.d()).c_str());

  // a short: EVENT in the next answer, POWER read at once (still tripped), shown, acknowledged with a POWER poll
  ucrx_short();
  const uint32_t t_short = ucrx_hub_ms();
  bool read = b.run_until(2000, [&] { return b.heard && b.tp == TP_POWER && (b.d().pw_latched & ULTRA_PW_EV_SHORT); });
  const uint32_t t_read = ucrx_hub_ms();
  std::string st = telemetry_status_text(b.d()), ev = telemetry_power_events_text(b.d());
  std::printf("  the short read %u ms after it: status '%s', events '%s'\n", (unsigned) (t_read - t_short), st.c_str(),
              ev.c_str());
  CHECK(read && b.t.trip_cause == ULTRA_PW_CAUSE_SHORT && contains(st, "tripped: short circuit, retry in 1 s") &&
            contains(ev, "1 short circuit (last 0 s ago, 16.") && b.d().short_trips == 1 &&
            b.d().trip_cause == ULTRA_PW_CAUSE_SHORT && b.d().retry_s == 1,
        "power: the short read while tripped (%d, firmware cause %u): status '%s', events '%s'", read,
        b.t.trip_cause, st.c_str(), ev.c_str());
  {
    const std::string det = receiver_status_detail(b.d(), receiver_status(b.d(), true), 0, -1);
    CHECK(receiver_status(b.d(), true) == RS_TRIPPED && receiver_status_problem(RS_TRIPPED) &&
              !receiver_status_problem(RS_SOFT_START) && contains(det, "Tripped by a short circuit at 16.") &&
              contains(det, "; it tries again by itself") && contains(telemetry_last_trip(b.d()), "short circuit at 16."),
          "power: status %s, '%s', last trip '%s'", receiver_status_name(receiver_status(b.d(), true)), det.c_str(),
          telemetry_last_trip(b.d()).c_str());
  }
  uint32_t acks0 = b.acks;
  bool acked = b.run_until(3000, [&] { return b.acks > acks0; });
  std::printf("  acknowledged %u ms after the read (ACK_DELAY_MS %u)\n", (unsigned) (ucrx_hub_ms() - t_read),
              (unsigned) TelemetryPoller::ACK_DELAY_MS);
  CHECK(acked && b.heard && b.tp == TP_POWER && (b.poll[ULTRA_POLL_ARG] & ULTRA_PW_ARG_ACK) &&
            ultra_get16(b.poll + ULTRA_POLL_ARG + 5) == 1 && !(b.t.latched & ULTRA_PW_EV_SHORT) &&
            !(b.d().pw_latched & ULTRA_PW_EV_SHORT),
        "power: the acknowledge (POLL byte 7 %02X, short trips %u) cleared the firmware's latch (%02X)",
        b.poll[ULTRA_POLL_ARG], ultra_get16(b.poll + ULTRA_POLL_ARG + 5), b.t.latched);
  // the retry (soft start again), its limit event acknowledged in turn; how long the status text shows a reason the
  // firmware no longer has (OUTPUTS' WHY is read every power_ms, STATUS every status_ms)
  uint32_t t_fw_ok = 0, t_text_ok = 0;
  b.run_until(12000, [&] {
    ucrx_truth_t now;
    ucrx_truth(&now);
    const bool fw_ok = !now.soft_start && !now.limiting && !now.tripped && !(now.power_state & ULTRA_ST_LIMITING);
    if (!fw_ok)
      t_fw_ok = t_text_ok = 0;
    else if (t_fw_ok == 0)
      t_fw_ok = ucrx_hub_ms();
    if (fw_ok && t_text_ok == 0 && telemetry_status_text(b.d()) == "ok")
      t_text_ok = ucrx_hub_ms();
    return t_text_ok != 0 && b.t.latched == 0 && b.d().pw_latched == 0;
  });
  std::printf("  after the retry: the firmware ok, the status text 'ok' %u ms later\n",
              (unsigned) (t_text_ok - t_fw_ok));
  CHECK(t_text_ok != 0 && b.t.latched == 0 && b.d().pw_latched == 0 && !(b.o.answer[2] & ULTRA_RSP_EVENT),
        "power: everything acknowledged after the retry, 'ok' (latched %02X, RFLAGS %02X, status '%s')", b.t.latched,
        b.o.answer[2], telemetry_status_text(b.d()).c_str());
  CHECK(t_text_ok - t_fw_ok <= 5200, "power: the status text showed an old reason %u ms after it ended",
        (unsigned) (t_text_ok - t_fw_ok));
  b.run(2500);   // (2 s without a reduction: the next one is a new limit event)

  // a limit event after the hub's read: its acknowledge (the counts as read) leaves the new one latched
  ucrx_supply_mv(18000);
  ucrx_run_ms(20);
  ucrx_supply_mv(24000);
  read = b.run_until(3000, [&] { return b.heard && b.tp == TP_POWER && (b.d().pw_latched & ULTRA_PW_EV_SUPPLY_DIP); });
  CHECK(read, "power: the dip latched and read");
  const uint16_t limit_read = b.d().limit_events;
  ucrx_cmd_config(ULTRA_CFG_BOARD_MAX, 1000);   // 1 A: the limiter reduces (a limit event)
  acks0 = b.acks;
  acked = b.run_until(3000, [&] { return b.acks > acks0; });
  CHECK(acked && ultra_get16(b.poll + ULTRA_POLL_ARG + 1) == limit_read && b.t.limit_events == limit_read + 1 &&
            (b.t.latched & ULTRA_PW_EV_LIMITED) && !(b.t.latched & ULTRA_PW_EV_SUPPLY_DIP),
        "power: acknowledged %u limit events, %u dips (the firmware: %u, %u): latched %02X", ultra_get16(b.poll + 8),
        ultra_get16(b.poll + 14), b.t.limit_events, b.t.dips, b.t.latched);
  CHECK(b.supply_read && b.read_dips == 1 && ultra_get16(b.poll + ULTRA_POLL_ARG + 7) == 1,
        "power: SUPPLY read before the dip's acknowledge (dips %u)", b.read_dips);
  b.run_until(5000, [&] { return b.t.latched == 0 && b.d().pw_latched == 0; });
  CHECK(b.t.latched == 0 && b.d().pw_latched == 0, "power: the newer limit event acknowledged in turn (%02X)",
        b.t.latched);
  b.run(2000);
  st = telemetry_status_text(b.d());
  ev = telemetry_power_events_text(b.d());
  std::printf("  limited: status '%s', events '%s'\n", st.c_str(), ev.c_str());
  CHECK(st == "limiting to 1.00 A (user limit)", "power: status '%s'", st.c_str());
  CHECK(contains(ev, "1 short circuit") && contains(ev, " limit event") && contains(ev, "1 supply dip (last to 1") &&
            contains(ev, " ms, ") && telemetry_value(b.d(), TV_SUPPLY_DIPS) == 1.0f,
        "power: events '%s'", ev.c_str());
  {
    const ReceiverStatus rs = receiver_status(b.d(), true);
    const std::string det = receiver_status_detail(b.d(), rs, 0, -1);
    std::printf("  status %s: '%s'; last dip '%s'\n", receiver_status_name(rs), det.c_str(),
                telemetry_last_dip(b.d()).c_str());
    CHECK(rs == RS_TRIPPED && contains(det, "; on again") &&
              contains(telemetry_last_dip(b.d()), "to 18.00 V (75 %) for "),
          "power: a trip %u s ago still shows: %s '%s', last dip '%s'", (unsigned) b.d().trip_age_s,
          receiver_status_name(rs), det.c_str(), telemetry_last_dip(b.d()).c_str());
  }
  CHECK(b.acks_across == 0 && b.acks_wrong == 0, "power: %u acknowledges without a read, %u not the counts read",
        (unsigned) b.acks_across, (unsigned) b.acks_wrong);
  b.check_retried();
  b.summary();
}

static void sc_chain() {
  std::printf("chain: the receiver at position 2 of 3, 500 kbaud; the first board drops out\n");
  Bench b("chain");
  b.power_up(0, RESET_POWER_PIN);
  b.start(3, 2, ULTRA_RW_RATE_500K, TelemetryIntervals{});
  lights_on(30000);
  b.run(6000);
  CHECK(b.d().have_identity && b.d().have_health && b.p.answering(0) && b.p.answering(1) && b.p.answering(2) &&
            b.n_heard > 5,
        "chain: all three answer (%d %d %d), the receiver %u times", b.p.answering(0), b.p.answering(1),
        b.p.answering(2), (unsigned) b.n_heard);
  CHECK(b.p.data(0).have_status && std::memcmp(b.p.data(1).uid, b.fakes[1].uid, 12) == 0 &&
            std::memcmp(b.d().uid, b.t.uid, 12) == 0,
        "chain: each board's data at its own position");
  // the first board drops out: the boards behind it can't be heard; only its miss counts
  const uint32_t missing0 = b.p.stats().missing, cont0 = b.p.stats().contention;
  b.fakes[0].present = false;
  b.run(1200);
  CHECK(b.p.misses(0) >= 1 && b.p.misses(2) == 0 && b.p.stats().missing > missing0,
        "chain: board 0 missing (%u), the receiver silenced without a miss (%u)", b.p.misses(0), b.p.misses(2));
  b.run(4000);
  CHECK(!b.p.answering(0) && b.p.cut_at(ucrx_hub_ms()) == 0,
        "chain: board 0 unavailable after %u misses, the line cut there (%d)", TelemetryPoller::CUT_MISSES,
        b.p.cut_at(ucrx_hub_ms()));
  {
    const ReceiverStatus rs = receiver_status(b.d(), b.p.answering(2));
    const std::string det = receiver_status_detail(b.d(), rs, 2, b.p.cut_at(ucrx_hub_ms()));
    CHECK(!b.p.answering(2) && rs == RS_NO_ANSWER && contains(det, "receiver 1 before it doesn't answer"),
          "chain: the receiver behind the cut: %s '%s'", receiver_status_name(rs), det.c_str());
  }
  b.fakes[0].present = true;
  b.run(15000);
  CHECK(b.p.answering(0) && b.p.answering(2) && b.p.stats().contention > cont0,
        "chain: back; the receiver's next answer had CONTENTION (%u)", (unsigned) (b.p.stats().contention - cont0));
  b.check_retried();
  b.summary();
}

static void sc_relay() {
  std::printf("relay: the receiver at position 15 of 18 (16 answer per window), 250 kbaud\n");
  Bench b("relay");
  b.power_up(0, RESET_POWER_PIN);
  b.start(18, 15, ULTRA_RW_RATE_250K, TelemetryIntervals{});
  lights_on(30000);
  uint32_t max_index = 0, relay_turned = 0, relay_behind = 0;
  const uint32_t end = ucrx_hub_ms() + 30000;
  while (static_cast<int32_t>(ucrx_hub_ms() - end) < 0) {
    if (!b.window()) {
      ucrx_run_ms(1);
      continue;
    }
    if (b.asked && b.poll[ULTRA_POLL_INDEX] > max_index)
      max_index = b.poll[ULTRA_POLL_INDEX];
    if (b.polled && !b.asked) {
      relay_turned += b.o.took_part && b.o.checked && !b.o.sent && !b.o.quiet;
      relay_behind += b.p.misses(16) == 0 && b.p.misses(17) == 0;
    }
  }
  CHECK(b.n_relay >= 3 && relay_turned == b.n_relay && relay_behind == b.n_relay,
        "relay: RELAY_ONLY %u times: turned around and relayed %u, the boards behind heard %u", (unsigned) b.n_relay,
        (unsigned) relay_turned, (unsigned) relay_behind);
  CHECK(b.n_heard >= 20 && max_index == 15, "relay: answered %u times, at index up to %u", (unsigned) b.n_heard,
        (unsigned) max_index);
  bool all = true;
  for (uint16_t i = 0; i < 18; i++)
    all = all && b.p.answering(i);
  CHECK(all && b.d().have_energy && b.d().have_health, "relay: all 18 answering, every page of the receiver read");
  b.check_retried();
  b.summary();
}

static void sc_lost() {
  std::printf("lost: a lost POLL, contention, a broken answer, a late main loop, flash writes, a stale answer\n");
  Bench b("lost");
  b.power_up(0, RESET_POWER_PIN);
  b.start(1, 0, ULTRA_RW_RATE_1M, intervals(20, 300, 60000));
  lights_on(40000);
  b.run(3000);
  static const char *const CASES[5] = {"a POLL lost on the wire", "the turnaround check finds the line taken",
                                       "the burst finds the line taken", "a broken answer (CRC)",
                                       "the main loop 900 us late"};
  for (int c = 0; c < 5; c++) {
    b.run(1000);
    const uint32_t cont0 = b.p.stats().contention, bad0 = b.p.stats().bad_crc;
    b.lose_poll = c == 0;
    b.contend_check = c == 1;
    b.contend_tx = c == 2;
    b.break_answer = c == 3;
    b.late_us = c == 4 ? 900 : 0;
    while (!b.window())
      ucrx_run_ms(1);
    const bool lost = b.asked && !b.heard;
    const int tp = b.tp;
    const uint8_t seq = b.poll[2];
    const uint32_t w_lost = b.windows;
    CHECK(lost, "lost: %s: the answer arrived anyway", CASES[c]);
    CHECK(c == 0 ? !b.o.took_part : c <= 2 ? b.o.took_part && b.o.quiet && !b.o.sent : c == 3 ? b.o.sent : b.o.missed,
          "lost: %s: the firmware's window (took part %d, quiet %d, sent %d, missed %d)", CASES[c], b.o.took_part,
          b.o.quiet, b.o.sent, b.o.missed);
    const bool again = b.run_until(2000, [&] { return b.heard && b.tp == tp; });
    CHECK(again && b.poll[2] != seq, "lost: %s: page %s asked again (SEQ %u, lost %u) and answered", CASES[c],
          TP_NAME[tp], b.poll[2], seq);
    std::printf("  %s: %s asked again and answered %u windows later\n", CASES[c], TP_NAME[tp],
                (unsigned) (b.windows - w_lost));
    if (c == 1 || c == 2) {
      b.run(500);
      CHECK(b.p.stats().contention == cont0 + 1, "lost: %s: CONTENTION in the next answer (%u)", CASES[c],
            (unsigned) (b.p.stats().contention - cont0));
    }
    if (c == 3)
      CHECK(b.p.stats().bad_crc > bad0, "lost: %s: dropped for its CRC", CASES[c]);
  }
  // a stale answer (an earlier window's) in the stream: counted, not taken
  b.replay = true;
  while (!b.window())
    ucrx_run_ms(1);
  // flash writes (a persisted setting changed, written ~2 s later while dark): the tracker re-arms 10 ms after one,
  // so a frame right after it is lost (no POLL decoded); asked again in a later window
  ucrx_cmd_set(0, 0, 0);
  b.run(1000);
  uint32_t flash_lost = 0, flash_last = ucrx_flash_writes();
  const uint32_t flash0 = flash_last;
  for (int i = 0; i < 40 && flash_lost < 3; i++) {
    ucrx_cmd_config(ULTRA_CFG_SOFT_START, static_cast<uint16_t>(i & 1));
    const uint32_t end = ucrx_hub_ms() + 2600;
    while (static_cast<int32_t>(ucrx_hub_ms() - end) < 0) {
      if (!b.window()) {
        ucrx_run_ms(1);
        continue;
      }
      flash_lost += b.asked && !b.o.took_part && !b.o.missed && ucrx_flash_writes() != flash_last;
      flash_last = ucrx_flash_writes();
    }
  }
  std::printf("  flash: %u operations, %u windows lost right after one (asked again later)\n",
              (unsigned) (ucrx_flash_writes() - flash0), (unsigned) flash_lost);
  CHECK(ucrx_flash_writes() > flash0 && flash_lost >= 1, "lost: flash writes (%u) losing a frame (%u)",
        (unsigned) (ucrx_flash_writes() - flash0), (unsigned) flash_lost);
  // a restart (watchdog) while the hub polls every 20 ms: its first frames aren't taken (not locked yet, then the
  // clock not rated: no part, counted missed); asked again with back-off until it answers
  b.power_up(1, RESET_WATCHDOG);
  uint32_t unlocked = 0, unrated = 0, w0 = b.windows;
  const uint32_t t_boot = ucrx_hub_ms();
  bool back = b.run_until(5000, [&] {
    if (b.windows == w0)
      return false;
    w0 = b.windows;
    unlocked += b.asked && !b.o.took_part && !b.o.missed;
    unrated += b.asked && b.o.missed && !b.t.clock_rated;
    return b.heard;
  });
  std::printf("  a restart: answered again %u ms after it; before that %u frames not taken (no lock yet), %u POLLs "
              "taken without a part (clock not rated)\n",
              (unsigned) (ucrx_hub_ms() - t_boot), (unsigned) unlocked, (unsigned) unrated);
  CHECK(back && unlocked + unrated >= 2, "lost: the restart's first frames lost (%u, %u), answered again",
        (unsigned) unlocked, (unsigned) unrated);
  b.run(3000);
  CHECK(b.p.stats().restarts == 1, "lost: the restart seen (%u)", (unsigned) b.p.stats().restarts);
  b.check_retried();
  CHECK(b.p.answering(0) && b.p.stats().stale == 1, "lost: answering at the end, the replayed answer stale (%u)",
        (unsigned) b.p.stats().stale);
  b.summary();
}

static void sc_restart() {
  std::printf("restart: an unnoticed watchdog restart with an acknowledge due, then the hub's own power cycle\n");
  Bench b("restart");
  b.power_up(0, RESET_POWER_PIN);
  b.start(1, 0, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  lights_on(65535);
  b.run(70000);   // (lit: lifetime energy; ENERGY read twice)
  const uint32_t id_reads0 = b.reads[TP_IDENTITY];
  // a short, read: an acknowledge is due with a POWER poll; the receiver restarts (watchdog) before it goes out
  ucrx_short();
  bool read = b.run_until(3000, [&] { return b.heard && b.tp == TP_POWER && (b.d().pw_latched & ULTRA_PW_EV_SHORT); });
  CHECK(read, "restart: the short read");
  const uint32_t up_before = b.d().uptime_s;
  const float e_hub = telemetry_value(b.d(), TV_ENERGY);
  const double pos_before = b.keeper.energy_wh(0);
  ucrx_truth_t fw;
  ucrx_truth(&fw);
  const uint32_t e_fw = fw.total_wh, t0 = ucrx_hub_ms();
  b.power_up(1, RESET_WATCHDOG);
  uint32_t t_answer = 0, t_detect = 0, t_identity = 0, acks_before_detect = 0, seen_w = b.windows;
  bool new_short_shown = false;
  const uint32_t restart_upd0 = b.n_restart_upd;
  auto watch = [&] {
    if (b.windows == seen_w)
      return false;
    seen_w = b.windows;
    if (t_answer == 0 && b.o.took_part)
      t_answer = ucrx_hub_ms();
    if (t_detect == 0 && b.n_restart_upd != restart_upd0)
      t_detect = ucrx_hub_ms();
    if (t_detect == 0)
      acks_before_detect = b.acks_across;
    if (t_detect != 0 && t_identity == 0 && b.heard && b.tp == TP_IDENTITY)
      t_identity = ucrx_hub_ms();
    new_short_shown = new_short_shown || (t_detect != 0 && b.heard && b.tp == TP_POWER &&
                                          (b.d().pw_latched & ULTRA_PW_EV_SHORT));
    return false;
  };
  b.run_until(100, watch);
  lights_on(65535);   // (the hub's refresh of the light's state)
  b.run_until(250, watch);
  ucrx_short();       // the new boot's own short: its count 1, as the one read before the restart
  b.run_until(12000, watch);
  const ReceiverData &d = b.d();
  std::printf("  restart: first window taken part in %u ms after it, the restart seen %u ms after it, IDENTITY again "
              "%u ms after it; %u acknowledge(s) since, %u reaching the new boot before the poller saw the restart\n",
              (unsigned) (t_answer - t0), (unsigned) (t_detect - t0), (unsigned) (t_identity - t0), (unsigned) b.acks,
              (unsigned) acks_before_detect);
  CHECK(b.acks_across == 0, "restart: %u acknowledge(s) with the counts read before the restart reached the new boot "
        "(its own short's latch cleared before the hub read it)", (unsigned) b.acks_across);
  CHECK(new_short_shown, "restart: the new boot's short never showed latched (SHORT) in a POWER answer");
  CHECK(b.n_restart_upd == restart_upd0 + 1 && b.p.stats().restarts == 1 && t_identity != 0 &&
            b.reads[TP_IDENTITY] == id_reads0 + 1 && b.d().have_identity,
        "restart: seen once (TU_RESTART %u, restarts %u), IDENTITY read again (%u)",
        (unsigned) (b.n_restart_upd - restart_upd0), (unsigned) b.p.stats().restarts,
        (unsigned) (b.reads[TP_IDENTITY] - id_reads0));
  CHECK(telemetry_reset_text(d.reset_cause) == "watchdog" && d.uptime_s < 20 && d.short_trips == 1 &&
            contains(telemetry_power_events_text(d), "1 short circuit"),
        "restart: the new boot's state: reset '%s', uptime %u (%u before), short trips %u",
        telemetry_reset_text(d.reset_cause).c_str(), (unsigned) d.uptime_s, (unsigned) up_before, d.short_trips);
  CHECK(b.t.latched == 0 && b.t.short_trips == 1,
        "restart: the new boot's short acknowledged in the end (latched %02X)", b.t.latched);
  const float e_after = telemetry_value(d, TV_ENERGY);
  // the board loses what it hadn't stored (its lifetime goes back); the position's Energy (the ledger) doesn't
  CHECK(pos_before > 0 && b.keeper.energy_wh(0) >= pos_before,
        "restart: the position's Energy %.3f Wh after the watchdog restart (%.3f before)", b.keeper.energy_wh(0),
        pos_before);
  std::printf("  lifetime energy across the watchdog restart: the board said %.3f Wh (stored %u), after it %.3f; the "
              "position's Energy (the ledger) %.3f, after it %.3f\n",
              e_hub, (unsigned) e_fw, e_after, pos_before, b.keeper.energy_wh(0));

  // the hub's own power cycle (a wall switch): the supply falls (the dip's lifetime write), the hub stops, powers up
  b.run(65000);
  const float e1 = telemetry_value(b.d(), TV_ENERGY);
  const double pos1 = b.keeper.energy_wh(0);
  const uint32_t restarts0 = b.p.stats().restarts;
  ucrx_supply_mv(3000);
  ucrx_run_ms(30);
  b.p.unpowered();
  CHECK(!b.p.answering(0) && (b.p.take_updates(0) & TU_AVAILABILITY) &&
            receiver_status(b.d(), b.p.answering(0)) == RS_NO_ANSWER,
        "restart: unpowered: unavailable");
  b.power_up(1, RESET_POWER_PIN);
  b.p.locked(ucrx_hub_ms());
  bool early = false;
  for (int i = 0; i < 990; i++) {
    early = early || b.p.due(ucrx_hub_ms());
    ucrx_run_ms(1);
  }
  CHECK(!early, "restart: an RW frame within ULTRA_RW_LOCK_MS of the hub's preamble");
  lights_on(65535);
  b.run(8000);
  CHECK(b.p.answering(0) && b.p.stats().restarts == restarts0 + 1 && telemetry_reset_text(b.d().reset_cause) ==
            "pin, power-on" && b.d().short_trips == 0 && telemetry_power_events_text(b.d()) == "no events",
        "restart: after the power cycle: restarts %u, reset '%s', events '%s'", (unsigned) b.p.stats().restarts,
        telemetry_reset_text(b.d().reset_cause).c_str(), telemetry_power_events_text(b.d()).c_str());
  // the board keeps its whole Wh (the dip's write), not its fraction; the hub's ledger keeps both
  CHECK(std::floor(telemetry_value(b.d(), TV_ENERGY)) >= std::floor(e1) && pos1 > 0 && b.keeper.energy_wh(0) >= pos1,
        "restart: the lifetime energy kept through the power cycle: the board %.3f Wh (%.3f before), the position "
        "%.3f Wh (%.3f before)",
        telemetry_value(b.d(), TV_ENERGY), e1, b.keeper.energy_wh(0), pos1);
  std::printf("  power cycle (the dip's lifetime write first): the board %.3f Wh before, %.3f after; the position's "
              "Energy (the ledger) %.3f, %.3f\n",
              e1, telemetry_value(b.d(), TV_ENERGY), pos1, b.keeper.energy_wh(0));
  CHECK(b.acks_wrong == 0, "restart: %u acknowledges not with the counts read in that boot", (unsigned) b.acks_wrong);
  b.check_retried();
  b.summary();
}

/// (0.2.5) LIFETIME and DIAG against the firmware's record; the retry policy: no retries, "retries used up", a new
/// command switches it on again; the short in LIFETIME's count and the fault memory.
/// (0.2.6) LIFE_SYNC with the receiver's own firmware: lit for 7 min, then a cut without a lifetime write (a watchdog
/// restart: what it hadn't stored is gone): after its power-up the hub's ledger gives the powered / lit time and the
/// whole Wh back in one POLL (applied, merged into its RAM record), never twice; the position's Energy never back.
static void sc_life_sync() {
  std::printf("life sync: the hub's ledger back to the receiver after a cut that lost what it hadn't stored\n");
  Bench b("lifesync");
  b.power_up(0, RESET_POWER_PIN);
  TelemetryIntervals iv = intervals(1000, 5000, 10000);
  iv.lifetime_ms = 10000;
  b.start(1, 0, ULTRA_RW_RATE_1M, iv);
  lights_on(65535);
  b.run(8000);
  if (!(b.d().features & ULTRA_FEAT_LIFE_SYNC)) {
    std::printf("  a firmware without LIFE_SYNC: skipped\n");
    b.summary();
    return;
  }
  b.run(420000);   // (0.1 h and more of powered and lit time, and Wh, not stored yet)
  const LifeCounters led = *b.keeper.counters(0);
  ucrx_truth_t before;
  ucrx_truth(&before);
  b.power_up(1, RESET_WATCHDOG);   // (no write: the record as last stored)
  ucrx_truth_t cut;
  ucrx_truth(&cut);
  CHECK(cut.lt_lit_s / 360u < led.lit && cut.lt_on_s / 360u < led.powered && cut.total_wh < led.mwh / 1000,
        "life sync: the cut lost what it hadn't stored: lit %u (ledger %u), powered %u (%u), %u Wh (%.3f)",
        (unsigned) (cut.lt_lit_s / 360u), (unsigned) led.lit, (unsigned) (cut.lt_on_s / 360u), (unsigned) led.powered,
        (unsigned) cut.total_wh, led.mwh / 1000.0);
  lights_on(65535);
  b.run(20000);
  ucrx_truth_t after;
  ucrx_truth(&after);
  CHECK(b.reads[TP_SYNC] == 1 && b.keeper.last_result(0) == ULTRA_LS_R_APPLIED && !b.keeper.attention(0) &&
            after.lt_lit_s / 360u >= led.lit && after.lt_on_s / 360u >= led.powered &&
            after.total_wh >= led.mwh / 1000 - 1 && after.lt_writes == cut.lt_writes,
        "life sync: one POLL (%u), applied (%u): lit %u, powered %u, %u Wh back; no write for it (records %u -> %u)",
        (unsigned) b.reads[TP_SYNC], b.keeper.last_result(0), (unsigned) (after.lt_lit_s / 360u),
        (unsigned) (after.lt_on_s / 360u), (unsigned) after.total_wh, (unsigned) cut.lt_writes,
        (unsigned) after.lt_writes);
  b.run(30000);
  CHECK(b.reads[TP_SYNC] == 1, "life sync: once per power-up (%u)", (unsigned) b.reads[TP_SYNC]);
  std::printf("  the cut: lit %u -> %u (0.1 h), %u -> %u Wh; synced back: lit %u, %u Wh; position %.3f Wh\n",
              (unsigned) (before.lt_lit_s / 360u), (unsigned) (cut.lt_lit_s / 360u), (unsigned) before.total_wh,
              (unsigned) cut.total_wh, (unsigned) (after.lt_lit_s / 360u), (unsigned) after.total_wh,
              b.keeper.energy_wh(0));
  b.summary();
}

static void sc_retry() {
  std::printf("retry: the retry policy, LIFETIME, DIAG (0.2.5)\n");
  Bench b("retry");
  b.power_up(0, RESET_POWER_PIN);
  TelemetryIntervals iv = intervals(500, 2000, 10000);
  iv.lifetime_ms = iv.diag_ms = 3000;
  b.start(1, 0, ULTRA_RW_RATE_1M, iv, false);
  lights_on(65535);
  b.run(4000);
  if (!b.t.has_025) {
    std::printf("  a 0.2.4 firmware: skipped\n");
    b.summary();
    return;
  }
  CHECK(b.reads[TP_LIFETIME] >= 1 && b.reads[TP_DIAG] >= 1 && b.reads[TP_FAULTS] >= 1,
        "retry: LIFETIME %u, DIAG %u / %u read", (unsigned) b.reads[TP_LIFETIME], (unsigned) b.reads[TP_DIAG],
        (unsigned) b.reads[TP_FAULTS]);
  const float trips0 = telemetry_value(b.d(), TV_TRIPS), ups = telemetry_value(b.d(), TV_POWER_UPS);
  ucrx_cmd_config(ULTRA_CFG_RETRY, 0x0100);   // never retry (1 s)
  b.run(1000);
  ucrx_short();
  bool out = b.run_until(3000, [&] {
    return b.heard && b.tp == TP_STATUS && receiver_why(b.d()) == ULTRA_ST_WHY_RETRIES_OUT;
  });
  b.run(3000);   // (no retry)
  {
    const ReceiverStatus rs = receiver_status(b.d(), true);
    const std::string det = receiver_status_detail(b.d(), rs, 0, -1), st = telemetry_status_text(b.d());
    std::printf("  retries used up: %s '%s'; status text '%s'\n", receiver_status_name(rs), det.c_str(), st.c_str());
    CHECK(out && receiver_why(b.d()) == ULTRA_ST_WHY_RETRIES_OUT && rs == RS_TRIPPED &&
              contains(det, "Tripped by a short circuit at ") && contains(det, "retries are used up") &&
              contains(st, "tripped: short circuit, retries used up"),
          "retry: no retries: %s '%s', '%s'", receiver_status_name(rs), det.c_str(), st.c_str());
  }
  lights_on(30000);   // a new command: on again
  const bool on = b.run_until(3000, [&] {
    return b.heard && b.tp == TP_STATUS && receiver_why(b.d()) != ULTRA_ST_WHY_RETRIES_OUT &&
           !(b.d().power_state & ULTRA_ST_TRIPPED);
  });
  CHECK(on, "retry: a new command switched it on again (why %u, state %02X)", receiver_why(b.d()),
        b.d().power_state);
  b.run(7000);   // LIFETIME and DIAG again
  CHECK(telemetry_value(b.d(), TV_TRIPS) == trips0 + 1 && telemetry_value(b.d(), TV_POWER_UPS) == ups &&
            (b.d().fm_trip & 3) == 1 && contains(telemetry_last_trip(b.d()), "short circuit at "),
        "retry: lifetime trips %.0f (%.0f before), the fault memory %02X, last trip '%s'",
        telemetry_value(b.d(), TV_TRIPS), trips0, b.d().fm_trip, telemetry_last_trip(b.d()).c_str());
  ucrx_cmd_config(ULTRA_CFG_RETRY, 0x010A);   // (the default back)
  b.run(500);
  b.check_retried();
  b.summary();
}

/// (auto_retry) A repeated update session reaches a receiver that already runs the release, and one for other
/// hardware: it doesn't take it, and its light stays as it was (the answer on its status LED only, firmware 0.6.9).
static void sc_update_repeat() {
  std::printf("update repeat: a session for the version it runs / for other hardware, the light held\n");
  static const uint32_t VERSION = 0x000609FF;   // 0.6.9 (as the app's descriptor says: ucrx_update_info)
  for (int feedback = 0; feedback < 2; feedback++) {
    for (int other = 0; other < 2; other++) {
      ucrx_power_up(0, RESET_POWER_PIN);
      ucrx_run_ms(1500);
      ucrx_cmd_config(ULTRA_CFG_FEEDBACK, static_cast<uint16_t>(feedback));
      lights_on(40000);
      ucrx_run_ms(3000);   // (the soft start over: steady)
      ucrx_update_info(2, UPD_HW_ID_DP_ULTRA_RX, 0, VERSION);   // (its own bootloader: a receiver's, revision 0)
      ucrx_truth_t t0, t;
      ucrx_truth(&t0);
      // the image: the descriptor the app checks (the body isn't looked at by a board that doesn't take it)
      static uint8_t img[UPD_DESC_BYTES + 1024];
      std::memset(img, 0, sizeof(img));
      std::memcpy(img, "QDU5", 4);
      img[UPD_D_DESC_VERSION] = UPD_DESC_VERSION;
      img[UPD_D_HW_ID] = other ? 9 : UPD_HW_ID_DP_ULTRA_RX;
      upd_put16(img + UPD_D_HW_REV_MASK, 1u << 0);
      upd_put32(img + UPD_D_FW_VERSION, VERSION);
      upd_put32(img + UPD_D_BODY_LEN, 1024);
      upd_put32(img + UPD_D_BODY_CRC, upd_crc32_words(0xFFFFFFFFu, img + UPD_DESC_BYTES, 256));
      upd_put32(img + UPD_D_LOAD_ADDR, UPD_APP_BODY);
      upd_put32(img + UPD_D_ENTRY, UPD_APP_ENTRY);
      img[UPD_D_MIN_BL] = 2;
      upd_put32(img + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, img, 15u));
      UpdateSession s;
      UpdateSession::Options o;
      CHECK(s.begin(img, sizeof(img), 0x61, o, 384u * 1350u / 1000u + 300u), "update repeat: the session");
      uint8_t f[UPD_FRAME_BYTES];
      uint32_t pause = 0, frames = 0, off = 0, transparent = 0, worst = 0;
      auto held = [&]() {
        ucrx_truth(&t);
        uint32_t d = 0;
        for (int ch = 0; ch < 5; ch++) {
          const uint32_t e = static_cast<uint32_t>(std::abs(static_cast<int>(t.out[ch]) - t0.out[ch]));
          d = e > d ? e : d;
        }
        worst = d > worst ? d : worst;
        off += d > 0;
      };
      while (s.next(f, ucrx_hub_ms() * 1000u, &pause)) {
        ucrx_raw_frame(f, UPD_FRAME_BYTES, pause);
        frames++;
        held();
        transparent += t.upd_transparent;
      }
      // the hub's quiet second, then its normal frames again; the answer shows meanwhile (at END)
      bool shown = false, on_outputs = false;
      for (int ms = 0; ms < 4000; ms += 10) {
        if (ms == 1000)
          ucrx_host(1);
        ucrx_run_ms(10);
        held();
        shown |= t.ack_active != 0;
        on_outputs |= t.ack_active && !t.ack_led_only;
      }
      const char *what = other ? "other hardware" : "the version it runs";
      std::printf("  feedback %s, %s: %u frames (transparent in %u), the answer shown %d (outputs %d), the light off "
                  "its level in %u samples (by up to %u)\n",
                  feedback ? "all" : "red", what, (unsigned) frames, (unsigned) transparent, shown, on_outputs,
                  (unsigned) off, (unsigned) worst);
      CHECK(t.upd_sessions >= 1 && transparent > frames / 2 && shown,
            "update repeat (feedback %d, %s): the receiver decided the session (%u) and answered", feedback, what,
            (unsigned) t.upd_sessions);
      CHECK(off == 0 && !on_outputs,
            "update repeat (feedback %d, %s): the light held through the session and its answer (%u samples off, by "
            "up to %u; the answer on the outputs %d)",
            feedback, what, (unsigned) off, (unsigned) worst, on_outputs);
    }
  }
}

// --------------------------------------------------------------- a month of short wakes (design review, round 9)
/// The poller fed straight from a receiver model's pages (no wire: 3,000 power-ups in seconds): its own page choice
/// (pick_, read_now, the sync's arguments), decoding and restart detection, as an answer on the wire takes them.
struct FeedPoller : TelemetryPoller {
  /// the page receiver 0 is asked for now (TP_COUNT: none due)
  uint8_t due_page(uint32_t now_ms) const {
    int32_t over = 0;
    const uint8_t tp = this->pick_(this->rx_[0], now_ms, &over);
    return tp < TP_COUNT && over >= 0 ? tp : TP_COUNT;
  }
  /// its answer (DATA) to page tp (answer_'s path after the slot's checks)
  void answer(uint8_t tp, const uint8_t *data, uint16_t uid16, uint32_t now_ms) {
    Receiver &r = this->rx_[0];
    if (r.have_uid && uid16 != r.uid16)
      this->restarted_(r, now_ms, true);
    r.have_uid = true;
    r.uid16 = uid16;
    r.misses = 0;
    if (!r.answering) {
      r.answering = true;
      r.reported = false;
      r.updated |= TU_AVAILABILITY;
    }
    r.next[tp] = now_ms + this->interval_(tp);
    this->decode_(r, tp, data, now_ms);
  }
  const uint8_t *sync_args() const { return this->rx_[0].sync_args; }
};

static constexpr uint32_t MONTH_LIT_MW = 30000, MONTH_DARK_MW = 600;   // its input power lit / dark
static constexpr uint64_t UJ_PER_WH = 3600000000ull;

/// A Diff-Power ultra receiver's lifetime record as the firmware keeps it: src/core/energy.c itself (energy_init,
/// energy_second, energy_due / energy_pack / energy_saved: its writes, energy_page, energy_sync: the merge); its
/// supply, lights and power-ups as the scenario says. Every ms of power counted (energy_ms's job, in spans). The
/// truth: what it would count if it lost nothing (powered: the port's on-time, the hub's clock).
struct MonthRx {
  energy_t e{};
  life_rec_t flash{};   // the newest stored record (its RAM starts from it at every power-up)
  uint8_t uid[12]{};
  uint16_t uid16{0};
  uint32_t boot{0}, at{0}, secs{0}, mw{0};   // its power-up, how far it ran (hub ms), its uptime (s), its power
  bool lit{false};
  uint32_t writes{0};
  uint64_t t_on_ms{0}, t_lit_ms{0}, t_uj{0};
  uint32_t t_ups{0};

  void init(const life_rec_t &rec) {
    for (int k = 0; k < 12; k++)
      this->uid[k] = static_cast<uint8_t>(0x51 + 7 * k);
    this->uid16 = ultra_crc16(this->uid, 12);
    this->flash = rec;
    this->t_on_ms = rec.on_s * 1000ull;
    this->t_lit_ms = rec.lit_s * 1000ull;
    this->t_uj = rec.wh * UJ_PER_WH;
    this->t_ups = rec.ups;
  }
  void store() {
    energy_pack(&this->e);
    this->flash = this->e.lt;
    energy_saved(&this->e, this->secs);
    this->writes++;
  }
  void power_up(uint32_t now) {
    this->e.lt = this->flash;
    energy_init(&this->e, 1, 0, 0, 0);
    this->e.ev_save = 1;   // (main.c: the receiver's)
    this->boot = this->at = now;
    this->secs = 0;
    this->lit = false;
    this->mw = MONTH_DARK_MW;
    this->t_ups++;
  }
  /// its milliseconds up to now: energy (whole mWh into boot_mwh), lit, and each second energy_second and a write
  /// if one is due
  void run_to(uint32_t now) {
    while (static_cast<int32_t>(now - this->at) > 0) {
      const uint32_t tick = this->boot + (this->secs + 1) * 1000u;
      const uint32_t to = static_cast<int32_t>(tick - now) <= 0 ? tick : now;
      const uint32_t ms = to - this->at;
      this->e.uj += this->mw * ms;   // (mW x ms = uJ; < 2^32 for a second at 30 W)
      while (this->e.uj >= ENERGY_UJ_PER_MWH) {
        this->e.uj -= ENERGY_UJ_PER_MWH;
        this->e.boot_mwh++;
      }
      this->t_uj += static_cast<uint64_t>(this->mw) * ms;
      if (this->lit) {
        this->e.lit_ms = to - 1 - this->boot;   // (its last lit ms)
        this->e.lit_new = 1;
      }
      this->at = to;
      if (to == tick) {
        this->secs++;
        energy_second(&this->e, to - this->boot, this->secs, 0, 0, 0);
        if (energy_due(&this->e, to - this->boot, 0, ENERGY_DUE_SAVE | ENERGY_DUE_DIP) != ENERGY_DUE_NONE)
          this->store();
      }
    }
  }
  void set_lit(bool on, uint32_t now) {
    this->run_to(now);
    this->lit = on;
    this->mw = on ? MONTH_LIT_MW : MONTH_DARK_MW;
  }
  /// the planned power-off's notice: dark; a write only for something worth storing (ENERGY_PLANNED)
  void notice(uint32_t now) {
    this->set_lit(false, now);
    if (energy_due(&this->e, now - this->boot, 0, ENERGY_DUE_SAVE | ENERGY_DUE_DIP | ENERGY_PLANNED) !=
        ENERGY_DUE_NONE)
      this->store();
  }
  /// the cut: what isn't stored is gone (the next power-up starts from the flash)
  void cut(uint32_t now, uint32_t on_ms) {
    this->run_to(now);
    this->t_on_ms += now - on_ms;
  }
  /// LIFETIME bytes 0..15 (main.c's page)
  void lifetime16(uint8_t *d) const {
    const life_rec_t &r = this->e.lt;
    ultra_put24(d + ULTRA_LT_POWERED, r.on_s / 360u);
    ultra_put24(d + ULTRA_LT_LIT, r.lit_s / 360u);
    ultra_put24(d + ULTRA_LT_POWER_UPS, r.ups);
    ultra_put16(d + ULTRA_LT_OVERLOAD_TRIPS, r.trips[0]);
    ultra_put16(d + ULTRA_LT_SHORT_TRIPS, r.trips[1]);
    ultra_put24(d + ULTRA_LT_DIPS, r.dips);
  }
  /// its answer to page tp now
  void page(uint8_t tp, uint32_t now, uint8_t *d) {
    this->run_to(now);
    std::memset(d, 0, 24);
    switch (tp) {
      case TP_IDENTITY:
        std::memcpy(d, this->uid, 12);
        ultra_put32(d + ULTRA_ID_FEATURES,
                    ULTRA_FEAT_ENERGY | ULTRA_FEAT_LIFETIME | ULTRA_FEAT_LIFE_SYNC | ULTRA_FEAT_STANDBY);
        break;
      case TP_STATUS:
        ultra_put24(d + ULTRA_ST_UPTIME, this->secs);
        ultra_put16(d + ULTRA_ST_VBUS, 0xFFFF);
        break;
      case TP_ENERGY:
        energy_page(&this->e, this->secs, this->mw, d);
        break;
      case TP_LIFETIME:
        this->lifetime16(d);
        ultra_put24(d + ULTRA_LT_RECORDS, this->e.lt.writes);
        break;
      default:
        break;
    }
  }
  /// a LIFE_SYNC POLL: energy_sync, then its answer (main.c's: LIFETIME 0..15, the Wh, result and applied, RECORDS)
  uint8_t sync(const uint8_t *args, uint32_t now, uint8_t *d) {
    this->run_to(now);
    const uint32_t res = energy_sync(&this->e, args, this->uid16);
    std::memset(d, 0, 24);
    this->lifetime16(d);
    ultra_put32(d + ULTRA_LS_A_TOTAL_WH, energy_total_wh(&this->e));
    d[ULTRA_LS_A_RESULT] = static_cast<uint8_t>(res >> 8);
    d[ULTRA_LS_A_APPLIED] = static_cast<uint8_t>(res);
    ultra_put16(d + ULTRA_LS_A_RECORDS, static_cast<uint16_t>(this->e.lt.writes));
    return static_cast<uint8_t>(res >> 8);
  }
};

/// what the board shows after the month (its counters in the wire's units) against the truth
struct MonthEnd {
  int64_t d_pow, d_lit, d_ups, d_wh;      // board - truth
  int64_t h_pow, h_lit, h_ups, h_wh;      // the hub's ledger - truth (the keeper only)
  double truth_h, board_h, truth_lit_h, board_lit_h, truth_wh, board_wh;
  uint32_t truth_ups, board_ups, writes, syncs, applied;
  uint32_t results[8];   // the syncs' results (ULTRA_LS_R_*)
  bool attention;
  double e_board, e_hub;  // the board's / the ledger's energy - the truth, mWh
  uint32_t cuts;
  bool within(int64_t a, int64_t b, int64_t c) const {
    return std::llabs(a) <= 1 && std::llabs(b) <= 1 && std::llabs(c) <= 1;
  }
  /// energy: never above the truth; below it by at most a mWh a cut (the board counts whole mWh: its last reading's
  /// fraction isn't seen) and the Wh fraction a first contact can't see
  bool energy_ok(double e) const { return e <= 0.0 && e >= -(this->cuts + 1000.0); }
  bool board_ok() const {
    return within(d_pow, d_lit, d_ups) && energy_ok(e_board) && e_board >= e_hub - 1000.0;
  }
  bool hub_ok() const { return within(h_pow, h_lit, h_ups) && energy_ok(e_hub); }
};

/// One port, one receiver: `wakes` power-ups (a light 60 s at 30 W, its turn-off period of 170 s dark, the
/// hub's last reads, the notice, the cut, ~10 min off; every 10th wake lit 10 min when `long_tenth`: those store their
/// record). The hub: the component's keeper (power_on / lit_tick / prepare / cut, the poller fed the board's pages,
/// the ESP32 restarted twice in between), or (echo) a naive hub that keeps the highest of its LIFETIME / ENERGY reads
/// and syncs those back.
static MonthEnd month_run(const life_rec_t &rec, int wakes, bool long_tenth, bool echo, uint32_t seed) {
  std::mt19937 rng(seed);
  MonthRx b;
  b.init(rec);
  LifetimeLedger led;
  led.init(8, 4);
  TelemetryIntervals iv;   // (the defaults: STATUS 3 s, ENERGY 60 s, LIFETIME 5 min)
  FeedPoller p;
  LedgerKeeper k;
  uint32_t now = 1000 + rng() % 1000, syncs = 0, applied = 0, results[8]{};
  auto sync = [&](const uint8_t *args, uint8_t *d) {
    const uint8_t res = b.sync(args, now, d);
    syncs++;
    applied += res == ULTRA_LS_R_APPLIED;
    results[res < 7 ? res : 7]++;
  };
  auto start_hub = [&]() {
    p = FeedPoller{};
    p.init(1, ULTRA_RW_RATE_1M, iv);
    k = LedgerKeeper{};
    k.attach(&led, 7u << 16, 1, now);
  };
  start_hub();
  // the echo hub: one entry, the highest of each counter it read
  LedgerEntry ec;
  bool echo_known = false;
  auto echo_read = [&](bool back) {
    uint8_t d[24];
    ReceiverData rd;
    b.page(TP_LIFETIME, now, d);
    decode_receiver_page(rd, ULTRA_PAGE_LIFETIME, d);
    b.page(TP_ENERGY, now, d);
    decode_receiver_page(rd, ULTRA_PAGE_ENERGY, d);
    const LifeCounters rc = receiver_counters(rd);
    if (!echo_known)
      ec.c = rc;
    echo_known = true;
    ec.c.powered = std::max(ec.c.powered, rc.powered);
    ec.c.lit = std::max(ec.c.lit, rc.lit);
    ec.c.ups = std::max(ec.c.ups, rc.ups);
    ec.c.mwh = std::max(ec.c.mwh, rc.mwh);
    ec.records = rd.lt_records;
    if (back && LifetimeLedger::ahead(ec, rc)) {
      uint8_t args[LifetimeLedger::LS_ARGS];
      LifetimeLedger::sync_args(ec, rc, b.uid16, args);
      sync(args, d);
    }
  };
  auto poll = [&]() {
    const uint8_t tp = p.due_page(now);
    if (tp >= TP_COUNT)
      return;
    uint8_t d[24];
    if (tp == TP_SYNC) {
      sync(p.sync_args(), d);
    } else {
      b.page(tp, now, d);
    }
    p.answer(tp, d, b.uid16, now);
    k.on_updates(p, 0, p.take_updates(0), now);
  };
  constexpr uint32_t STEP = 250, BOOT_MS = 40, READY_MS = 1200, HOLD_MS = 170000, NOTICE_MS = 1500;
  // the port on (enable and detect), the board's boot, the port ready (the receivers Waking)
  auto power_on = [&]() {
    const uint32_t on = now;
    if (!echo)
      k.power_on(on);
    b.power_up(on + BOOT_MS);
    now = on + READY_MS;
    if (!echo)
      p.touch_all();
    return on;
  };
  for (int w = 0; w < wakes; w++) {
    if (!echo && (w == wakes / 3 || w == 2 * wakes / 3)) {
      // the ESP32 restarts (the port off meanwhile): its ledger as stored, a new poller and keeper
      std::vector<uint8_t> blob(led.blob_size());
      led.save(blob.data(), static_cast<uint32_t>(w));
      LifetimeLedger back;
      back.init(8, 4);
      uint32_t gen = 0;
      CHECK(back.load(blob.data(), blob.size(), &gen) && gen == static_cast<uint32_t>(w),
            "month: the ledger's stored copy loads (LDG3)");
      led = back;
      start_hub();
    }
    const uint32_t lit_ms = long_tenth && w % 10 == 9 ? 600000 : 60000;
    const uint32_t on = power_on();
    b.set_lit(true, now);
    b.t_lit_ms += lit_ms;
    if (!echo)
      k.lit_tick(0, true, now);
    const uint32_t t_dark = now + lit_ms, t_prep = t_dark + HOLD_MS;
    bool dark = false, prepping = false, echoed = false;
    uint32_t prep_ms = 0;
    for (;; now += STEP) {
      if (!dark && static_cast<int32_t>(now - t_dark) >= 0) {
        b.set_lit(false, t_dark);
        if (!echo)
          k.lit_tick(0, false, t_dark);
        dark = true;
      }
      if (echo) {
        if (!echoed && now - on >= READY_MS + 2000) {
          echo_read(true);   // (after its power-up: read, the highest kept, synced back where ahead)
          echoed = true;
        }
        if (static_cast<int32_t>(now - t_prep) >= 0) {
          echo_read(false);   // (its last reads before the power-off)
          break;
        }
        continue;
      }
      if (!prepping && static_cast<int32_t>(now - t_prep) >= 0) {
        prepping = true;
        prep_ms = now;
        if (k.prepare(p, now, true))
          break;
      } else if (prepping && (k.prepare(p, now, false) || now - prep_ms >= 3000)) {
        break;
      }
      poll();
    }
    // the notice (the port no longer ready), then the cut
    if (!echo)
      p.unpowered();
    b.notice(now);
    const uint32_t cut = now + NOTICE_MS;
    b.cut(cut, on);
    if (!echo)
      k.cut(cut);
    now = cut + 590000 + rng() % 20000;
  }
  // its next power-up: the hub's counters back, then what it shows
  const uint32_t on = power_on();
  if (echo) {
    now += 2000;
    echo_read(true);
  } else {
    for (uint32_t end = now + 30000; static_cast<int32_t>(now - end) < 0; now += STEP)
      poll();
  }
  b.run_to(now);
  const uint64_t t_on = b.t_on_ms + (now - on);
  MonthEnd r{};
  r.truth_h = t_on / 3600000.0;
  r.board_h = b.e.lt.on_s / 3600.0;
  r.truth_lit_h = b.t_lit_ms / 3600000.0;
  r.board_lit_h = b.e.lt.lit_s / 3600.0;
  r.truth_wh = static_cast<double>(b.t_uj) / UJ_PER_WH;
  r.board_wh = energy_total_wh(&b.e) + (b.e.boot_mwh - b.e.mwh_seen) % 1000u / 1000.0;
  r.truth_ups = b.t_ups;
  r.board_ups = b.e.lt.ups;
  r.writes = b.writes;
  r.syncs = syncs;
  r.applied = applied;
  std::memcpy(r.results, results, sizeof(results));
  r.attention = !echo && k.attention(0);
  const int64_t tp = static_cast<int64_t>(t_on / UNIT_MS), tl = static_cast<int64_t>(b.t_lit_ms / UNIT_MS),
                tw = static_cast<int64_t>(b.t_uj / UJ_PER_WH);
  r.d_pow = static_cast<int64_t>(b.e.lt.on_s / 360u) - tp;
  r.d_lit = static_cast<int64_t>(b.e.lt.lit_s / 360u) - tl;
  r.d_ups = static_cast<int64_t>(b.e.lt.ups) - b.t_ups;
  r.d_wh = static_cast<int64_t>(energy_total_wh(&b.e)) - tw;
  const double truth_mwh = static_cast<double>(b.t_uj) / 3600000.0;
  r.e_board = energy_total_wh(&b.e) * 1000.0 + (b.e.boot_mwh - b.e.mwh_seen) % 1000u - truth_mwh;
  r.cuts = static_cast<uint32_t>(wakes);
  const LifeCounters *c = echo ? nullptr : k.counters(0);
  if (c != nullptr) {
    r.h_pow = static_cast<int64_t>(c->powered) - tp;
    r.h_lit = static_cast<int64_t>(c->lit) - tl;
    r.h_ups = static_cast<int64_t>(c->ups) - b.t_ups;
    r.h_wh = static_cast<int64_t>(c->mwh / 1000) - tw;
    r.e_hub = static_cast<double>(c->mwh) - truth_mwh;
  }
  return r;
}

static void month_print(const char *what, const MonthEnd &r) {
  std::printf("  %s: powered %.1f of %.1f h, lit %.1f of %.1f h, power-ups %u of %u, %.1f of %.1f Wh (%u writes, "
              "%u syncs: %u applied, %u once, %u records, %u no-op, %u capped)\n",
              what, r.board_h, r.truth_h, r.board_lit_h, r.truth_lit_h, r.board_ups, r.truth_ups, r.board_wh,
              r.truth_wh, r.writes, r.syncs, r.applied, r.results[ULTRA_LS_R_ONCE], r.results[ULTRA_LS_R_RECORDS],
              r.results[ULTRA_LS_R_NOOP], r.results[ULTRA_LS_R_CAPPED]);
}

/// Design review, round 9: a receiver merges the hub's counters in whole wire units and loses its own time and energy
/// below a unit at every cut, so the ledger must be the hub's own accounting. 30 days x 100 short wakes of a standby
/// port (nothing stored on the board): after its next power-up every counter within one unit of the truth, on the
/// board (energy_sync's merge) and in the hub's ledger; the same with every tenth wake long enough to be stored (the
/// board's own remainders ride on the hub's units: never counted twice). The negative control: a hub that echoes the
/// highest of its reads gives back almost nothing (a record without remainders), or misses the truth by far (one with
/// them: ahead, or behind once the receiver writes the merges it takes).
static void sc_month() {
  std::printf("month: 30 days x 100 short wakes of a standby port, the hub's own ledger against the receiver's merge\n");
  life_rec_t used{};   // a used board: 500 h powered (and 300 s), 200 h lit (and 350 s), 2 kWh
  used.on_s = 360u * 5000u + 300u;
  used.lit_s = 360u * 2000u + 350u;
  used.ups = 900;
  used.wh = 2000;
  used.writes = 40;
  used.magic = LIFE_MAGIC;
  const MonthEnd a = month_run(used, 3000, false, false, 11);
  month_print("hub ledger, nothing stored", a);
  // (no wake stores; the receiver writes only when a merge raises its powered time by a day or more: a few)
  CHECK(a.board_ok() && a.hub_ok() && a.writes <= 15 && !a.attention && a.results[ULTRA_LS_R_UID] == 0,
        "month: every counter back within a unit, energy never above (board - truth: powered %lld, lit %lld, "
        "power-ups %lld, %.0f mWh; ledger: %lld, %lld, %lld, %.0f mWh; %u writes)",
        (long long) a.d_pow, (long long) a.d_lit, (long long) a.d_ups, a.e_board, (long long) a.h_pow,
        (long long) a.h_lit, (long long) a.h_ups, a.e_hub, a.writes);
  const MonthEnd s = month_run(used, 3000, true, false, 12);
  month_print("hub ledger, every 10th stored", s);
  CHECK(s.board_ok() && s.hub_ok() && s.writes >= 250 && !s.attention && s.results[ULTRA_LS_R_UID] == 0,
        "month, every tenth wake stored: within a unit, energy never above (board - truth: %lld, %lld, %lld, %.0f mWh; "
        "ledger: %lld, %lld, %lld, %.0f mWh; %u writes)",
        (long long) s.d_pow, (long long) s.d_lit, (long long) s.d_ups, s.e_board, (long long) s.h_pow,
        (long long) s.h_lit, (long long) s.h_ups, s.e_hub, s.writes);
  // a year of it (design review, round 9: a hub that takes the board's rounded value back during sessions it saw adopts
  // the board's own remainder, ~6 h a year): the truth is summed apart from the ledger (the port's on-times, the lit
  // spans, the integrated power, the boots), so a slow drift shows
  const MonthEnd y = month_run(used, 36500, true, false, 13);
  month_print("hub ledger, a year, every 10th stored", y);
  CHECK(y.board_ok() && y.hub_ok() && !y.attention && y.results[ULTRA_LS_R_UID] == 0,
        "a year, every tenth wake stored: within a unit, no drift, energy never above (board - truth: %lld, %lld, %lld, "
        "%.0f mWh; ledger: %lld, %lld, %lld, %.0f mWh; %u writes)",
        (long long) y.d_pow, (long long) y.d_lit, (long long) y.d_ups, y.e_board, (long long) y.h_pow,
        (long long) y.h_lit, (long long) y.h_ups, y.e_hub, y.writes);
  // the read-to-POLL race against the real merge (design review, round 9): the board's own time passes
  // a 0.1 h between the hub's read and its LIFE_SYNC POLL (its powered step one less, its other caps with it); a ledger
  // ahead in every field, sent at the caps that read allows, still comes back applied
  {
    MonthRx r;
    life_rec_t rec = used;
    rec.on_s = 360u * 5000u + 357u;   // (3 s below a 0.1 h)
    r.init(rec);
    r.power_up(0);
    uint8_t d[24];
    ReceiverData rd;
    r.page(TP_LIFETIME, 500, d);
    decode_receiver_page(rd, ULTRA_PAGE_LIFETIME, d);
    r.page(TP_ENERGY, 500, d);
    decode_receiver_page(rd, ULTRA_PAGE_ENERGY, d);
    const LifeCounters rc = receiver_counters(rd);
    LedgerEntry le;
    le.c = rc;
    le.c.powered += 10;
    le.c.lit += 20;
    le.c.ups += 1000;
    le.c.mwh += 5000000;
    le.records = rd.lt_records;
    uint8_t args[LifetimeLedger::LS_ARGS];
    LifetimeLedger::sync_args(le, rc, r.uid16, args);
    const uint8_t res = r.sync(args, 4500, d);   // (4 s on: 5,001 x 0.1 h now)
    const uint8_t all = ULTRA_LS_F_POWERED | ULTRA_LS_F_LIT | ULTRA_LS_F_UPS | ULTRA_LS_F_WH;
    CHECK(rd.lt_powered == 5000 && r.e.lt.on_s / 360u == 5010 && res == ULTRA_LS_R_APPLIED &&
              d[ULTRA_LS_A_APPLIED] == all,
          "month: the read-to-POLL race: applied (result %u, fields %02X; read at %u, now %u x 0.1 h)", res,
          d[ULTRA_LS_A_APPLIED], rd.lt_powered, r.e.lt.on_s / 360u);
  }
  life_rec_t clean = used;   // (no remainders)
  clean.on_s = 360u * 5000u;
  clean.lit_s = 360u * 2000u;
  const MonthEnd e = month_run(clean, 3000, false, true, 11);
  month_print("echo hub (control)", e);
  const double gained = e.board_h - 500.0, due = e.truth_h - 500.0;
  CHECK(gained < 0.1 * due && e.board_lit_h - 200.0 < 0.1 * (e.truth_lit_h - 200.0) &&
            e.board_ups - 900u < (e.truth_ups - 900u) / 10 && e.board_wh - 2000.0 < 0.1 * (e.truth_wh - 2000.0),
        "month, echo control: almost nothing back (%.1f of %.1f h powered)", gained, due);
  const MonthEnd f = month_run(used, 3000, false, true, 11);
  month_print("echo hub, remainders (control)", f);
  CHECK(std::fabs(f.board_h - f.truth_h) > 0.25 * (f.truth_h - 500.0),
        "month, echo control with remainders: far off the truth (%.1f h for %.1f: ahead with receivers that ratchet "
        "its remainders, behind with ones that write the merges they take)",
        f.board_h, f.truth_h);
}

// ---------------------------------------------- the ledger against an independent truth (design review, round 9)
/// One hub's ledger side, stepped by hand: its ledger, its poller fed the board's pages, its keeper; restart(): the
/// ESP32 restarts (its ledger as last stored, a new poller and keeper).
struct ProbeHub {
  LifetimeLedger l;
  FeedPoller p;
  LedgerKeeper k;
  void init(uint32_t now) {
    this->l.init(8, 4);
    this->restart(now, false);
  }
  void restart(uint32_t now, bool stored = true) {
    if (stored) {
      std::vector<uint8_t> blob(this->l.blob_size());
      this->l.save(blob.data(), 1);
      LifetimeLedger back;
      back.init(8, 4);
      uint32_t gen = 0;
      back.load(blob.data(), blob.size(), &gen);
      this->l = back;
    }
    this->p = FeedPoller{};
    this->p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    this->k = LedgerKeeper{};
    this->k.attach(&this->l, 1u << 16, 1, now);
  }
  void page(MonthRx &b, uint8_t tp, uint32_t now) {
    uint8_t d[24];
    b.page(tp, now, d);
    this->p.answer(tp, d, b.uid16, now);
    this->k.on_updates(this->p, 0, this->p.take_updates(0), now);
  }
  /// its reads (IDENTITY first after a power-up or a restart), and the sync the keeper asks for
  void sample(MonthRx &b, uint32_t now, bool identity = false, bool sync = true) {
    if (identity)
      this->page(b, TP_IDENTITY, now);
    this->page(b, TP_LIFETIME, now);
    this->page(b, TP_ENERGY, now);
    this->page(b, TP_STATUS, now);
    if (sync && this->k.sync_sent(0)) {
      uint8_t d[24];
      b.sync(this->p.sync_args(), now, d);
      this->p.answer(TP_SYNC, d, b.uid16, now);
      this->k.on_updates(this->p, 0, this->p.take_updates(0), now);
    }
  }
};

static life_rec_t used_board() {   // (a used board: 500 h powered and 319 s, 200 h lit and 271 s, 2 kWh, 900 power-ups)
  life_rec_t r{};
  r.magic = LIFE_MAGIC;
  r.on_s = 500 * 3600 + 319;
  r.lit_s = 200 * 3600 + 271;
  r.wh = 2000;
  r.ups = 900;
  return r;
}
static uint64_t ledger_pow_ms(const LedgerEntry &e) { return uint64_t{e.c.powered} * UNIT_MS + e.pow_rem; }
static uint64_t ledger_lit_ms(const LedgerEntry &e) { return uint64_t{e.c.lit} * UNIT_MS + e.lit_rem; }

/// Days of a fixed load (each day 50 wakes lit 60 s at 30 W, 180 s dark at 0.6 W, and one lit 3 h, 810 s dark;
/// the last read at the cut): the same unreported fraction at every cut, which random phases would average away.
/// The truth summed apart from the ledger and the board's counters. restart: the ESP32 restarts 30 s into each long
/// wake, the board powered on.
static void ledger_truth(int days, bool restart) {
  MonthRx b;
  b.init(used_board());
  ProbeHub h;
  uint32_t now = 1000;
  h.init(now);
  uint64_t true_pow = uint64_t{used_board().on_s} * 1000, true_lit = uint64_t{used_board().lit_s} * 1000;
  uint32_t boots = used_board().ups;
  for (int day = 0; day < days; day++) {
    for (int w = 0; w < 51; w++) {
      const uint32_t on = now;
      h.k.power_on(on);
      b.power_up(on);
      boots++;
      now += 1000;
      h.sample(b, now, true);
      const uint32_t light = w == 50 ? 10800000 : 60000;
      b.set_lit(true, now);
      h.k.lit_tick(0, true, now);
      const uint32_t off = now + light;
      while (static_cast<int32_t>(off - now) > 0) {
        now += std::min<uint32_t>(30000, off - now);
        h.sample(b, now);
        h.k.lit_tick(0, true, now);
        if (restart && w == 50 && now == on + 31000) {
          h.restart(now);
          h.k.lit_tick(0, true, now);
          h.sample(b, now, true);
        }
      }
      true_lit += light;
      b.set_lit(false, now);
      h.k.lit_tick(0, false, now);
      now += w == 50 ? 810000 : 180000;
      h.sample(b, now);
      b.notice(now);
      b.cut(now, on);
      h.k.cut(now);
      true_pow += now - on;
      now += 10000;
    }
  }
  const LedgerEntry *e = h.l.find(b.uid);
  const double truth_mwh = static_cast<double>(b.t_uj) / 3600000.0;
  const double e_mwh = e != nullptr ? static_cast<double>(e->c.mwh) - truth_mwh : 0.0;
  std::printf("  fixed load, %d days%s: powered %+lld ms, lit %+lld ms, %+.3f Wh, power-ups %u of %u (%u writes)\n", days,
              restart ? ", ESP32 restarts" : "", e ? (long long) (ledger_pow_ms(*e) - true_pow) : 0LL,
              e ? (long long) (ledger_lit_ms(*e) - true_lit) : 0LL, e_mwh / 1000.0, e ? e->c.ups : 0, boots, b.writes);
  CHECK(e != nullptr && ledger_pow_ms(*e) < true_pow + UNIT_MS && ledger_lit_ms(*e) < true_lit + UNIT_MS &&
            e_mwh <= 0.0 && e->c.ups == boots,
        "fixed load, %d days (restarts %d): never above the truth (powered, lit by a unit at most; energy %+.3f mWh: "
        "never), power-ups exact",
        days, restart, e_mwh);
}

/// A sync's reply lost (the board merged; asked again: ONCE), an hour on, then the ESP32 restarts with the board still
/// powered: that hour and this power-up's energy aren't counted again (a sync moves the power-ups count without a
/// power-up). lose = false: the control, the reply received.
static void ledger_lost_reply(bool lose) {
  MonthRx b;
  life_rec_t rec = used_board();
  rec.ups = 890;
  b.init(rec);
  uint32_t now = 1000;
  b.power_up(now);
  ProbeHub h;
  h.init(now);
  bool made = false;
  LedgerEntry *e = h.l.touch(b.uid, &made);
  e->c.powered = rec.on_s / 360;
  e->c.lit = rec.lit_s / 360;
  e->c.mwh = uint64_t{rec.wh} * 1000;
  e->c.ups = 900;   // (the ledger knows ten power-ups the board lost)
  h.k.power_on(now);
  now += 1000;
  h.sample(b, now, true, false);
  uint8_t answer[24];
  b.sync(h.p.sync_args(), now, answer);   // (merged on the board)
  if (lose)
    b.sync(h.p.sync_args(), now + 20, answer);   // (its reply lost: asked again, ONCE)
  h.p.answer(TP_SYNC, answer, b.uid16, now + 20);
  h.k.on_updates(h.p, 0, h.p.take_updates(0), now + 20);
  b.set_lit(true, now + 20);
  now += 3600020;
  h.sample(b, now);
  const LedgerEntry before = *h.l.find(b.uid);
  h.restart(now);
  h.sample(b, now, true);
  const LedgerEntry *after = h.l.find(b.uid);
  CHECK(after->c.mwh == before.c.mwh && ledger_pow_ms(*after) == ledger_pow_ms(before) && after->c.ups == before.c.ups,
        "a sync's reply %s, an hour on, the ESP32 restarted: nothing counted again (%+lld mWh, %+lld ms, %u -> %u "
        "power-ups)",
        lose ? "lost (ONCE)" : "received", (long long) (after->c.mwh - before.c.mwh),
        (long long) (ledger_pow_ms(*after) - ledger_pow_ms(before)), before.c.ups, after->c.ups);
}

/// Another hub (B, with the fuller history) syncs a board this hub (A) counts, in the same power-up; back to A after
/// its restart, the board still powered: this power-up isn't new again.
static void ledger_two_hubs() {
  MonthRx b;
  life_rec_t rec = used_board();
  rec.ups = 890;
  b.init(rec);
  uint32_t now = 1000;
  b.power_up(now);
  ProbeHub a, other;
  a.init(now);
  other.init(now);
  now += 1000;
  a.sample(b, now, true);   // (A: first contact)
  bool made = false;
  LedgerEntry *oe = other.l.touch(b.uid, &made);
  oe->c = receiver_counters(a.p.data(0));
  oe->c.ups = 900;   // (B knows ten more power-ups)
  b.set_lit(true, now);
  now += 3600000;
  a.sample(b, now);   // (A counts an hour of this power-up)
  const LedgerEntry before = *a.l.find(b.uid);
  other.k.power_on(b.boot);
  other.sample(b, now, true);   // (B saw this power-up: it syncs 901)
  a.restart(now);
  a.sample(b, now, true);
  const LedgerEntry *after = a.l.find(b.uid);
  CHECK(b.e.lt.ups == 901 && after->c.mwh == before.c.mwh && ledger_pow_ms(*after) == ledger_pow_ms(before),
        "another hub synced this power-up (board %u power-ups): back at this hub, nothing new (%+lld mWh, %+lld ms)",
        b.e.lt.ups, (long long) (after->c.mwh - before.c.mwh),
        (long long) (ledger_pow_ms(*after) - ledger_pow_ms(before)));
}

/// A cut after a power read while lit, the lights off since (no read after them): no tail from that power (it
/// would count 30 W for the dark seconds); the energy never above the truth.
static void ledger_lit_tail() {
  MonthRx b;
  b.init(used_board());
  ProbeHub h;
  uint32_t now = 1000;
  h.init(now);
  const uint32_t on = now;
  h.k.power_on(on);
  b.power_up(on);
  now += 1000;
  h.sample(b, now, true);
  b.set_lit(true, now);
  h.k.lit_tick(0, true, now);
  now += 30000;
  h.sample(b, now);   // (its power read while lit)
  b.set_lit(false, now + 1000);
  h.k.lit_tick(0, false, now + 1000);
  now += 40000;   // (dark, no read: then the cut)
  b.notice(now);
  b.cut(now, on);
  h.k.cut(now);
  const LedgerEntry *e = h.l.find(b.uid);
  const double over = e != nullptr ? static_cast<double>(e->c.mwh) - static_cast<double>(b.t_uj) / 3600000.0 : 0.0;
  CHECK(e != nullptr && over <= 0.0, "a cut after a lit power read, dark since: no tail from it (%+.1f mWh)", over);
}

/// An ENERGY reading read while lit, its lights off after it, then counted later: with a STATUS read in the dark
/// (LIT_THEN_STATUS: its power from that reading's cache), or only at the cut (LIT_AT_CUT). Ten wakes, the cut 60 s
/// after the lights went off at 0.6 W: no tail from the lit reading (it would count 30 W for the dark seconds); the
/// energy never above the truth. The control (DARK): a reading read in the dark before the cut still gives its tail
/// (the dark seconds counted: short of the truth by about a mWh a cut, not by 10).
enum LitReading { LIT_THEN_STATUS, LIT_AT_CUT, DARK };
static void ledger_lit_reading(LitReading how) {
  MonthRx b;
  b.init(used_board());
  ProbeHub h;
  uint32_t now = 1000;
  h.init(now);
  for (int w = 0; w < 10; w++) {
    const uint32_t on = now;
    h.k.power_on(on);
    b.power_up(on);
    now += 1000;
    h.sample(b, now, true);
    b.set_lit(true, now);
    h.k.lit_tick(0, true, now);
    now += 30000;
    h.sample(b, now);
    now += 1000;
    h.page(b, TP_ENERGY, now);   // (read while lit: counted at the next STATUS)
    now += 1000;
    b.set_lit(false, now);
    h.k.lit_tick(0, false, now);
    if (how == DARK) {
      now += 1000;
      h.page(b, TP_ENERGY, now);   // (read in the dark)
    }
    if (how != LIT_AT_CUT) {
      now += 1000;
      h.page(b, TP_STATUS, now);   // (it counts the reading now, in the dark)
    }
    now += 60000;
    b.notice(now);
    b.cut(now, on);
    h.k.cut(now);
    now += 10000;
  }
  const LedgerEntry *e = h.l.find(b.uid);
  const double over = e != nullptr ? static_cast<double>(e->c.mwh) - static_cast<double>(b.t_uj) / 3600000.0 : 0.0;
  static const char *const WHAT[] = {"read lit, counted with a STATUS in the dark", "read lit, counted at the cut",
                                      "read in the dark (control)"};
  CHECK(e != nullptr && over <= 0.0 && (how != DARK || over > -20.0),
        "ten cuts after a reading %s: %s (%+.1f mWh against the truth)", WHAT[how],
        how == DARK ? "its tail counted" : "no tail from it", over);
}

static void sc_ledger_truth() {
  std::printf("ledger truth: a fixed load against an independent truth, a lost sync reply, two hubs\n");
  ledger_truth(365, false);
  ledger_truth(30, true);
  ledger_lost_reply(false);
  ledger_lost_reply(true);
  ledger_two_hubs();
  ledger_lit_tail();
  ledger_lit_reading(LIT_THEN_STATUS);
  ledger_lit_reading(LIT_AT_CUT);
  ledger_lit_reading(DARK);
}

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const auto t0 = std::chrono::steady_clock::now();
  if (argc > 1 && std::strcmp(argv[1], "month") == 0) {   // (those alone)
    sc_month();
    sc_ledger_truth();
    std::printf("ultra telemetry contract test (month): %s (%d failures), %.1f s\n", failures ? "FAILED" : "passed",
                failures, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    return failures ? 1 : 0;
  }
  ucrx_init();
  sc_pages();
  sc_soft_start();
  sc_power();
  sc_chain();
  sc_relay();
  sc_lost();
  sc_restart();
  sc_life_sync();
  sc_retry();
  sc_update_repeat();
  sc_month();
  sc_ledger_truth();
  const int soak = ucrx_soak_failures();
  CHECK(soak == 0, "the firmware model's own checks: %d failures", soak);
  const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("ultra telemetry contract test: %s (%d failures), %.0f s simulated hub time, %.1f s\n",
              failures ? "FAILED" : "passed", failures, ucrx_hub_ms() / 1000.0, s);
  return failures ? 1 : 0;
}
