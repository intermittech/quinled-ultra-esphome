#pragma once

// The return channel, host side (protocol 0.2.5, wiki: Return Channel): which receivers answer in the next response
// window and with which page, the RW frame's POLL slots, the answers out of the bytes the hub's UART received, and
// what they say. No ESP-IDF / ESPHome dependencies: tests/ultra_telemetry_test.cpp runs it on a PC against a modelled
// chain, tests/ultra_telemetry_contract_test.cpp against the Diff-Power ultra receiver firmware's own receive / answer
// path. UltraHub sends the frames and reads the UART.
//
// The rules it keeps (wiki: Return Channel, "The hub"):
//  - the receivers are the first boards of the line (a board without a return channel relays nothing, so receivers
//    behind one can't be heard); up to 16 answer in one window (INDEX is a nibble), unique rising indices, relay only
//    (index 0xFF) for receivers in front of an answering one that don't answer this time; one R and one rate;
//  - no RW frame before ULTRA_RW_LOCK_MS of headers after the chain's preamble, at most one per MIN_SPACING_MS;
//  - answers: discard to a SYNC, 32 bytes with a valid CRC; route by INDEX, match SEQ and the page (or BUSY /
//    UNSUPPORTED), UID16 against the board at that position (a new one: its data is dropped, IDENTITY read again);
//  - a missing answer is normal: the page is asked again in a later window (new SEQ, bounded back-off). A receiver
//    that doesn't take part silences every one behind it for that window: only the first missing answer counts. A
//    receiver that keeps missing (CUT_MISSES windows) cuts off the ones behind it: they aren't polled until it answers;
//  - a restart (its uptime fell): nothing is polled past it for 1 s, IDENTITY again, no acknowledge across it;
//  - POWER: read the counts (and SUPPLY's dip count), show them, acknowledge exactly those in the next POWER poll.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "fw_version.h"
#include "ultra_protocol.h"

namespace esphome::dig2analog_ultra {

// LIFE_SYNC (protocol 0.2.6, ULTRA_LS_*): the hub gives a board its ledger back (ultra_ledger.h). The receiver's steps
// per merge (the header's text): a counter ahead by more is ignored, so the hub sends at most this much (powered:
// 0.1 h; lit: what powered got + 1; counts: Δpowered_s / 10 + 64; Wh: 1 kW over Δpowered + 1 h)
static constexpr uint32_t LS_STEP_POWERED = 7200, LS_STEP_EVENTS = 64, LS_STEP_EVENTS_PER_01H = 36,
                          LS_STEP_WH_PER_01H = 100, LS_STEP_WH = 1000;

/// The pages the hub reads; when several are equally overdue, the earlier one goes first.
enum TelemetryPage : uint8_t {
  TP_IDENTITY = 0,  // at the start, after a restart or a new board (UID16)
  TP_POWER,         // board limit, trips and latches (feature POWER_EVENTS)
  TP_STATUS,
  TP_OUTPUTS,       // optional: why the outputs differ from the lights (UNSUPPORTED: not asked again)
  TP_SUPPLY,        // supply dips (feature SUPPLY_DIPS)
  TP_ENERGY,        // feature ENERGY
  TP_HEALTH,        // channel health (feature HEALTH)
  TP_LIFETIME,      // 0.2.5: the odometer (feature LIFETIME)
  TP_DIAG,          // 0.2.5: DIAG part 0, the running board (feature DIAG)
  TP_FAULTS,        // 0.2.5: DIAG part 1, the fault memory and the calibration trend
  TP_SYNC,          // 0.2.6: LIFE_SYNC, only when the ledger asks (TelemetryPoller::request_sync)
  TP_COUNT,
};
static constexpr uint8_t TP_PAGE_ID[TP_COUNT] = {ULTRA_PAGE_IDENTITY, ULTRA_PAGE_POWER,    ULTRA_PAGE_STATUS,
                                                 ULTRA_PAGE_OUTPUTS,  ULTRA_PAGE_SUPPLY,   ULTRA_PAGE_ENERGY,
                                                 ULTRA_PAGE_HEALTH,   ULTRA_PAGE_LIFETIME, ULTRA_PAGE_DIAG,
                                                 ULTRA_PAGE_DIAG,     ULTRA_PAGE_LIFE_SYNC};
/// the IDENTITY feature a page needs (0: every receiver, or tried)
static constexpr uint32_t TP_FEATURE[TP_COUNT] = {0,
                                                  ULTRA_FEAT_POWER_EVENTS,
                                                  0,
                                                  0,
                                                  ULTRA_FEAT_SUPPLY_DIPS,
                                                  ULTRA_FEAT_ENERGY,
                                                  ULTRA_FEAT_HEALTH,
                                                  ULTRA_FEAT_LIFETIME,
                                                  ULTRA_FEAT_DIAG,
                                                  ULTRA_FEAT_DIAG,
                                                  ULTRA_FEAT_LIFE_SYNC};
/// the POLL's argument byte 0 (DIAG: its part)
static constexpr uint8_t TP_PART[TP_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
/// the features of a 0.2.5 receiver (its ENERGY bytes 15-18 and HEALTH's hot flag are set; before: 0)
static constexpr uint32_t FEATURES_025 = ULTRA_FEAT_LIFETIME | ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE;

/// take_updates() bits beyond the pages
static constexpr uint32_t TU_AVAILABILITY = 1u << 30;  // it started or stopped answering
static constexpr uint32_t TU_RESTART = 1u << 29;       // it restarted (or is a different board now)

/// What one receiver said, page by page (big-endian fields decoded; `have_*`: the page was read since the board
/// answers / since it changed).
struct ReceiverData {
  // IDENTITY
  bool have_identity{false};
  uint8_t uid[12]{};
  uint32_t fw_word{0};  // major << 24 | minor << 16 | patch << 8 | pre-release (0 = unknown)
  uint8_t bl_version{0}, hw_id{0};
  uint16_t hw_caps{0};
  uint32_t features{0};
  // STATUS
  bool have_status{false};
  uint32_t fw_version{0};  // major << 16 | minor << 8 | patch (0 = a development build)
  uint8_t hw{0};
  uint32_t uptime_s{0};
  uint8_t reset_cause{0}, power_state{0};
  uint16_t mcu_temp{ULTRA_ST_NA}, board_temp{ULTRA_ST_NA};  // int16 0.1 C as u16; ULTRA_ST_NA = none
  uint16_t vbus_10mv{0xFFFF}, vdda_mv{0}, current_ma{0};
  uint8_t measured{0}, pwm_preset{0};
  uint16_t config_crc{0};
  uint8_t modes{0};
  // OUTPUTS
  bool have_outputs{false};
  uint16_t level[5]{}, ch_ma[5]{};
  uint8_t effects{0}, defined{0}, scene{0}, why{0};
  // POWER
  bool have_power{false};
  uint8_t pw_now{0}, pw_latched{0};
  uint16_t limit_events{0}, overload_trips{0}, short_trips{0};
  uint32_t limit_age_s{0xFFFFFF}, trip_age_s{0xFFFFFF};
  uint16_t limit_ma{0xFFFF};
  uint8_t limit_source{0}, trip_cause{0};
  uint16_t event_ma{0}, trip_ma{0}, factor{65535};
  uint8_t retry_s{0};
  // SUPPLY
  bool have_supply{false};
  uint16_t dips{0}, ref_mv{0}, now_mv{0}, deep_mv{0};
  uint8_t deep_pct{0};
  uint32_t deep_age_s{0xFFFFFF};
  uint8_t logged{0};
  uint32_t dip_age_s{0xFFFFFF};  // the newest logged dip
  uint16_t dip_mv{0};
  uint8_t dip_pct{0};
  uint16_t dip_ms{0}, dip_ma{0};
  // HEALTH
  bool have_health{false};
  uint8_t he_flags{0};
  uint8_t ch_class[5]{};
  int8_t test_dev[5]{ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA};
  int8_t model_dev[5]{ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA};
  int16_t dark_ma{0}, temp_peak{static_cast<int16_t>(ULTRA_ST_NA)};
  uint32_t temp_age_s{0xFFFFFF};
  // ENERGY
  bool have_energy{false};
  uint32_t boot_mwh{0}, total_wh{0}, power_mw{0}, saved_age_s{0xFFFFFF};
  double energy_wh{0};     // the board's lifetime energy, exact (TOTAL_WH + the mWh of this power-up's last Wh); the
                           // Energy sensor shows its position's (ultra_ledger.h: never down)
  uint16_t vb_10mv{0xFFFF};  // (0.2.5) the LED supply after the buck; 0xFFFF = not measured
  uint16_t peak_ma{0};       // (0.2.5) the input current's highest ~100 ms average since the ENERGY answer before
  // LIFETIME (0.2.5): the odometer, kept across power cuts
  bool have_lifetime{false};
  uint32_t lt_powered{0}, lt_lit{0};   // 0.1 h
  uint32_t lt_power_ups{0}, lt_dips{0};
  uint16_t lt_overload_trips{0}, lt_short_trips{0}, lt_updates{0};
  uint8_t lt_temp_peak{0}, lt_current_peak{0}, lt_watchdog{0};   // 0.5 C, 0.1 A
  uint32_t lt_records{0};
  // LIFE_SYNC (0.2.6): the last sync's answer (its LIFETIME bytes and Wh go to the fields above)
  bool have_sync{false};
  uint8_t ls_result{0}, ls_applied{0};
  uint16_t ls_records16{0};
  // DIAG part 0 (0.2.5): the running board
  bool have_diag{false};
  uint8_t dg_reset{0}, bc_cause{0xFF};   // this start's reset cause; the run before's exception (0xFF: none kept)
  uint16_t bc_pc{0};
  uint32_t bc_uptime_s{0};
  uint16_t cpu_pm{0}, cpu_peak_pm{0};    // 0.1 %
  uint16_t ovr_captures{0}, ovr_frame_ends{0}, ovr_late_ms{0};
  uint16_t supply_z_mohm{0};             // 0: not known yet
  uint8_t flash_errs{0}, flash_pending{0}, save_wait{0}, c10_load{0xFF}, c10_peak{0xFF};
  // DIAG part 1 (0.2.5): the fault memory (across power cuts) and the calibration trend
  bool have_faults{false};
  uint8_t fm_trip{0};        // 1 short / 2 overload (bits 0-1) | its current, 0.5 A (bits 2-7); 0 = none
  uint16_t fm_trip_lit_h{0};
  uint8_t fm_dip_pct{0}, fm_dip_10ms{0}, fm_dip_a10{0};
  uint8_t fm_rst{0}, fm_rst_up_h{0};   // 1 watchdog / 2 exception (bits 0-1) | mcause (bits 2-7); 0 = none
  int8_t trend[5]{ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA, ULTRA_HE_NA};
  uint16_t trend_lit_h{0xFFFF};
  uint8_t trend_tests{0};
};

/// A 0.2.5 receiver: its STATUS reason, ENERGY's LED supply and peak, HEALTH's hot flag are set (0 before).
inline bool receiver_025(const ReceiverData &d) { return d.have_identity && (d.features & FEATURES_025) != 0; }
/// STATUS 8 bits 4-7: why the light isn't as the host set it now (ULTRA_ST_WHY_*; 0: as set, or a board before 0.2.5)
inline uint8_t receiver_why(const ReceiverData &d) {
  return d.have_status ? static_cast<uint8_t>((d.power_state >> ULTRA_ST_WHY_SHIFT) & 0x0F) : 0;
}

/// One page's DATA (24 bytes) into d (pure; the page byte as answered; part: the POLL's argument, DIAG's part).
inline void decode_receiver_page(ReceiverData &d, uint8_t page, const uint8_t *p, uint8_t part = 0) {
  switch (page) {
    case ULTRA_PAGE_IDENTITY:
      std::memcpy(d.uid, p, 12);
      d.fw_word = ultra_get32(p + ULTRA_ID_FW_VERSION);
      d.bl_version = p[ULTRA_ID_BL_VERSION];
      d.hw_id = p[ULTRA_ID_HW_ID];
      d.hw_caps = static_cast<uint16_t>(ultra_get16(p + ULTRA_ID_HW_CAPS));
      d.features = ultra_get32(p + ULTRA_ID_FEATURES);
      d.have_identity = true;
      break;
    case ULTRA_PAGE_STATUS:
      d.fw_version = ultra_get24(p + ULTRA_ST_FW_VERSION);
      d.hw = p[ULTRA_ST_HW];
      d.uptime_s = ultra_get24(p + ULTRA_ST_UPTIME);
      d.reset_cause = p[ULTRA_ST_RESET];
      d.power_state = p[ULTRA_ST_STATE];
      d.mcu_temp = static_cast<uint16_t>(ultra_get16(p + ULTRA_ST_MCU_TEMP));
      d.board_temp = static_cast<uint16_t>(ultra_get16(p + ULTRA_ST_TEMP_BOARD));
      d.vbus_10mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_ST_VBUS));
      d.vdda_mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_ST_VDDA));
      d.current_ma = static_cast<uint16_t>(ultra_get16(p + ULTRA_ST_CURRENT));
      d.measured = p[ULTRA_ST_MEASURED];
      d.pwm_preset = p[ULTRA_ST_PWM_PRESET];
      d.config_crc = static_cast<uint16_t>(ultra_get16(p + ULTRA_ST_CONFIG_CRC));
      d.modes = p[ULTRA_ST_MODES];
      d.have_status = true;
      break;
    case ULTRA_PAGE_OUTPUTS:
      for (int i = 0; i < 5; i++) {
        d.level[i] = static_cast<uint16_t>(ultra_get16(p + ULTRA_OU_LEVEL + 2 * i));
        d.ch_ma[i] = static_cast<uint16_t>(ultra_get16(p + ULTRA_OU_CURRENT + 2 * i));
      }
      d.effects = p[ULTRA_OU_EFFECTS];
      d.defined = p[ULTRA_OU_DEFINED];
      d.scene = p[ULTRA_OU_SCENE];
      d.why = p[ULTRA_OU_WHY];
      d.have_outputs = true;
      break;
    case ULTRA_PAGE_POWER:
      d.pw_now = p[ULTRA_PW_NOW];
      d.pw_latched = p[ULTRA_PW_LATCHED];
      d.limit_events = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_LIMIT_EVENTS));
      d.overload_trips = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_OVL_TRIPS));
      d.short_trips = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_SHORT_TRIPS));
      d.limit_age_s = ultra_get24(p + ULTRA_PW_LIMIT_AGE);
      d.trip_age_s = ultra_get24(p + ULTRA_PW_TRIP_AGE);
      d.limit_ma = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_LIMIT_MA));
      d.limit_source = p[ULTRA_PW_SRC_CAUSE] >> 4;
      d.trip_cause = p[ULTRA_PW_SRC_CAUSE] & 0x0F;
      d.event_ma = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_EVENT_MA));
      d.trip_ma = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_TRIP_MA));
      d.factor = static_cast<uint16_t>(ultra_get16(p + ULTRA_PW_FACTOR));
      d.retry_s = p[ULTRA_PW_RETRY_S];
      d.have_power = true;
      break;
    case ULTRA_PAGE_SUPPLY:
      d.dips = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_DIPS));
      d.ref_mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_REF_MV));
      d.now_mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_NOW_MV));
      d.deep_mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_DEEP_MV));
      d.deep_pct = p[ULTRA_SU_DEEP_PCT];
      d.deep_age_s = ultra_get24(p + ULTRA_SU_DEEP_AGE);
      d.logged = p[ULTRA_SU_LOGGED];
      d.dip_age_s = ultra_get24(p + ULTRA_SU_EV_AGE);
      d.dip_mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_EV_MV));
      d.dip_pct = p[ULTRA_SU_EV_PCT];
      d.dip_ms = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_EV_MS));
      d.dip_ma = static_cast<uint16_t>(ultra_get16(p + ULTRA_SU_EV_MA));
      d.have_supply = true;
      break;
    case ULTRA_PAGE_HEALTH:
      d.he_flags = p[ULTRA_HE_FLAGS];
      for (int i = 0; i < 5; i++) {
        d.ch_class[i] = p[ULTRA_HE_CLASS + i];
        d.test_dev[i] = static_cast<int8_t>(p[ULTRA_HE_TEST_DEV + i]);
        d.model_dev[i] = static_cast<int8_t>(p[ULTRA_HE_MODEL_DEV + i]);
      }
      d.dark_ma = static_cast<int16_t>(ultra_get16(p + ULTRA_HE_DARK_MA));
      d.temp_peak = static_cast<int16_t>(ultra_get16(p + ULTRA_HE_TEMP_PEAK));
      d.temp_age_s = ultra_get24(p + ULTRA_HE_TEMP_AGE);
      d.have_health = true;
      break;
    case ULTRA_PAGE_ENERGY:
      d.boot_mwh = ultra_get32(p + ULTRA_EN_BOOT_MWH);
      d.total_wh = ultra_get32(p + ULTRA_EN_TOTAL_WH);
      d.energy_wh = d.total_wh + (ultra_get32(p + ULTRA_EN_BOOT_MWH) % 1000u) / 1000.0;
      d.power_mw = ultra_get32(p + ULTRA_EN_POWER_MW);
      d.saved_age_s = ultra_get24(p + ULTRA_EN_SAVED_AGE);
      d.vb_10mv = static_cast<uint16_t>(ultra_get16(p + ULTRA_EN_VB));
      d.peak_ma = static_cast<uint16_t>(ultra_get16(p + ULTRA_EN_PEAK_MA));
      d.have_energy = true;
      break;
    case ULTRA_PAGE_LIFETIME:
      d.lt_powered = ultra_get24(p + ULTRA_LT_POWERED);
      d.lt_lit = ultra_get24(p + ULTRA_LT_LIT);
      d.lt_power_ups = ultra_get24(p + ULTRA_LT_POWER_UPS);
      d.lt_overload_trips = static_cast<uint16_t>(ultra_get16(p + ULTRA_LT_OVERLOAD_TRIPS));
      d.lt_short_trips = static_cast<uint16_t>(ultra_get16(p + ULTRA_LT_SHORT_TRIPS));
      d.lt_dips = ultra_get24(p + ULTRA_LT_DIPS);
      d.lt_temp_peak = p[ULTRA_LT_TEMP_PEAK];
      d.lt_current_peak = p[ULTRA_LT_CURRENT_PEAK];
      d.lt_watchdog = p[ULTRA_LT_WATCHDOG];
      d.lt_updates = static_cast<uint16_t>(ultra_get16(p + ULTRA_LT_UPDATES));
      d.lt_records = ultra_get24(p + ULTRA_LT_RECORDS);
      d.have_lifetime = true;
      break;
    case ULTRA_PAGE_DIAG:
      if (part == 0) {
        d.dg_reset = p[ULTRA_DG_RESET];
        d.bc_cause = p[ULTRA_DG_BC_CAUSE];
        d.bc_pc = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_BC_PC));
        d.bc_uptime_s = ultra_get24(p + ULTRA_DG_BC_UPTIME);
        d.cpu_pm = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_CPU));
        d.cpu_peak_pm = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_CPU_PEAK));
        d.ovr_captures = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_OVR_CAPTURES));
        d.ovr_frame_ends = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_OVR_FRAME_ENDS));
        d.ovr_late_ms = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_OVR_LATE_MS));
        d.supply_z_mohm = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_SUPPLY_Z));
        d.flash_errs = p[ULTRA_DG_FLASH_ERRS];
        d.flash_pending = p[ULTRA_DG_FLASH_PENDING];
        d.save_wait = p[ULTRA_DG_SAVE_WAIT];
        d.c10_load = p[ULTRA_DG_C10_LOAD];
        d.c10_peak = p[ULTRA_DG_C10_PEAK];
        d.have_diag = true;
      } else {
        d.fm_trip = p[ULTRA_DG_TRIP];
        d.fm_trip_lit_h = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_TRIP_LIT_H));
        d.fm_dip_pct = p[ULTRA_DG_DIP_PCT];
        d.fm_dip_10ms = p[ULTRA_DG_DIP_10MS];
        d.fm_dip_a10 = p[ULTRA_DG_DIP_CURRENT];
        d.fm_rst = p[ULTRA_DG_RST];
        d.fm_rst_up_h = p[ULTRA_DG_RST_UP_H];
        for (int i = 0; i < 5; i++)
          d.trend[i] = static_cast<int8_t>(p[ULTRA_DG_TREND + i]);
        d.trend_lit_h = static_cast<uint16_t>(ultra_get16(p + ULTRA_DG_TREND_LIT_H));
        d.trend_tests = p[ULTRA_DG_TREND_TESTS];
        d.have_faults = true;
      }
      break;
    case ULTRA_PAGE_LIFE_SYNC:   // (0.2.6) LIFETIME 0-15 and the Wh after the merge, its result
      d.lt_powered = ultra_get24(p + ULTRA_LT_POWERED);
      d.lt_lit = ultra_get24(p + ULTRA_LT_LIT);
      d.lt_power_ups = ultra_get24(p + ULTRA_LT_POWER_UPS);
      d.lt_overload_trips = static_cast<uint16_t>(ultra_get16(p + ULTRA_LT_OVERLOAD_TRIPS));
      d.lt_short_trips = static_cast<uint16_t>(ultra_get16(p + ULTRA_LT_SHORT_TRIPS));
      d.lt_dips = ultra_get24(p + ULTRA_LT_DIPS);
      d.total_wh = ultra_get32(p + ULTRA_LS_A_TOTAL_WH);
      d.ls_result = p[ULTRA_LS_A_RESULT];
      d.ls_applied = p[ULTRA_LS_A_APPLIED];
      d.ls_records16 = static_cast<uint16_t>(ultra_get16(p + ULTRA_LS_A_RECORDS));
      d.have_sync = true;
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------------------------------- sensor values / texts
/// The numeric values a receiver's sensors show; NAN = not known (the page not read, or the board doesn't measure it).
enum TelemetryValue : uint8_t {
  TV_SUPPLY_VOLTAGE,  // V
  TV_INPUT_CURRENT,   // A
  TV_INPUT_POWER,     // W
  TV_MCU_TEMPERATURE, // C
  TV_UPTIME,          // s
  TV_ENERGY,          // Wh, the board's lifetime (total_increasing)
  TV_LIMIT_EVENTS,
  TV_OVERLOAD_TRIPS,
  TV_SHORT_TRIPS,
  TV_SUPPLY_DIPS,      // since its power-up (SUPPLY)
  TV_LED_SUPPLY,       // V, after the buck (0.2.5, ENERGY)
  TV_CURRENT_PEAK,     // A, the highest ~100 ms average since the ENERGY read before (0.2.5)
  TV_SUPPLY_IMPEDANCE, // mOhm, the supply / cable (0.2.5, DIAG)
  TV_LIT_HOURS,        // h, lifetime (0.2.5, LIFETIME)
  TV_POWER_UPS,        // lifetime (0.2.5, LIFETIME)
  TV_TRIPS,            // short + overload: lifetime with LIFETIME, else since its power-up (POWER)
  TV_DIPS,             // supply dips: lifetime with LIFETIME, else since its power-up (SUPPLY)
  TV_COUNT,
};

inline float telemetry_value(const ReceiverData &d, TelemetryValue v) {
  const bool volts = d.have_status && (d.measured & ULTRA_ST_M_VBUS) && d.vbus_10mv != 0xFFFF;
  const bool amps = d.have_status && (d.measured & ULTRA_ST_M_CURRENT);
  const bool v025 = receiver_025(d);
  // the lifetime counts once the board has the page (else its since-power-up counts: one source per board)
  const bool life = d.have_identity && (d.features & ULTRA_FEAT_LIFETIME);
  switch (v) {
    case TV_SUPPLY_VOLTAGE:
      return volts ? d.vbus_10mv * 0.01f : NAN;
    case TV_INPUT_CURRENT:
      return amps ? d.current_ma * 0.001f : NAN;
    case TV_INPUT_POWER:   // STATUS's voltage x current (read every few s), else ENERGY's (once a minute)
      if (volts && amps)
        return d.vbus_10mv * 0.01f * d.current_ma * 0.001f;
      return d.have_energy ? d.power_mw * 0.001f : NAN;
    case TV_MCU_TEMPERATURE:
      return d.have_status && d.mcu_temp != ULTRA_ST_NA ? static_cast<int16_t>(d.mcu_temp) * 0.1f : NAN;
    case TV_UPTIME:
      return d.have_status ? static_cast<float>(d.uptime_s) : NAN;
    case TV_ENERGY:
      return d.have_energy ? static_cast<float>(d.energy_wh) : NAN;
    case TV_LIMIT_EVENTS:
      return d.have_power ? d.limit_events : NAN;
    case TV_OVERLOAD_TRIPS:
      return d.have_power ? d.overload_trips : NAN;
    case TV_SHORT_TRIPS:
      return d.have_power ? d.short_trips : NAN;
    case TV_SUPPLY_DIPS:
      return d.have_supply ? d.dips : NAN;
    case TV_LED_SUPPLY:
      return v025 && d.have_energy && d.vb_10mv != 0xFFFF ? d.vb_10mv * 0.01f : NAN;
    case TV_CURRENT_PEAK:
      return v025 && d.have_energy ? d.peak_ma * 0.001f : NAN;
    case TV_SUPPLY_IMPEDANCE:
      return d.have_diag && d.supply_z_mohm != 0 ? static_cast<float>(d.supply_z_mohm) : NAN;
    case TV_LIT_HOURS:
      return d.have_lifetime ? d.lt_lit * 0.1f : NAN;
    case TV_POWER_UPS:
      return d.have_lifetime ? static_cast<float>(d.lt_power_ups) : NAN;
    case TV_TRIPS:
      if (life)
        return d.have_lifetime ? static_cast<float>(d.lt_short_trips + d.lt_overload_trips) : NAN;
      return d.have_power ? static_cast<float>(d.short_trips + d.overload_trips) : NAN;
    case TV_DIPS:
      if (life)
        return d.have_lifetime ? static_cast<float>(d.lt_dips) : NAN;
      return d.have_supply ? d.dips : NAN;
    default:
      return NAN;
  }
}

/// "35 s", "12 min", "3.5 h", "2 d"
inline std::string telemetry_age(uint32_t s) {
  char b[24];
  if (s < 120)
    snprintf(b, sizeof(b), "%u s", static_cast<unsigned>(s));
  else if (s < 7200)
    snprintf(b, sizeof(b), "%u min", static_cast<unsigned>(s / 60));
  else if (s < 172800)
    snprintf(b, sizeof(b), "%.1f h", s / 3600.0);
  else
    snprintf(b, sizeof(b), "%u d", static_cast<unsigned>(s / 86400));
  return b;
}

inline std::string telemetry_amps(uint32_t ma) {
  char b[16];
  snprintf(b, sizeof(b), "%.2f A", ma / 1000.0);
  return b;
}

/// The firmware version: IDENTITY's word ("0.6.8", "0.7.0-beta.2"), else STATUS's ("0.6.8"; 0 = a development build).
inline std::string telemetry_firmware_text(const ReceiverData &d) {
  if (d.have_identity && d.fw_word != 0)
    return format_fw_version(d.fw_word);
  if (!d.have_status)
    return "";
  if (d.fw_version == 0)
    return "development build";
  char b[16];
  snprintf(b, sizeof(b), "%u.%u.%u", static_cast<unsigned>(d.fw_version >> 16),
           static_cast<unsigned>((d.fw_version >> 8) & 0xFF), static_cast<unsigned>(d.fw_version & 0xFF));
  return b;
}

/// The reset cause of this start ("power-on", "watchdog", ...).
inline std::string telemetry_reset_text(uint8_t cause) {
  static const char *const NAMES[6] = {"pin", "power-on", "software", "watchdog", "window watchdog", "low power"};
  std::string s;
  for (int i = 0; i < 6; i++) {
    if (cause & (1u << i)) {
      if (!s.empty())
        s += ", ";
      s += NAMES[i];
    }
  }
  return s.empty() ? "unknown" : s;
}

/// A RISC-V exception (mcause): "exception 2 (illegal instruction)".
inline std::string telemetry_exception(uint8_t mcause) {
  static const char *const NAMES[8] = {"instruction misaligned", "instruction access fault", "illegal instruction",
                                       "breakpoint",             "load misaligned",          "load access fault",
                                       "store misaligned",       "store access fault"};
  std::string s = "exception " + std::to_string(mcause);
  if (mcause < 8)
    s += std::string(" (") + NAMES[mcause] + ")";
  return s;
}

/// Notes after the Status detail that are no problem: (0.2.6) the receiver's lifetime record held (its flash-wear
/// allowance used up: it is written less often).
inline std::string receiver_status_notes(const ReceiverData &d) {
  if (d.have_diag && (d.flash_pending & ULTRA_DG_FP_EXHAUSTED))
    return " Lifetime record held: the receiver saves less often to protect its flash.";
  return "";
}

/// The Last reset text: this start's cause; with DIAG (0.2.5) what the run before left ("software; the run before:
/// up 3.5 h, no exception"), else the last abnormal reset the board keeps across power cuts ("power-on; the last
/// abnormal one: watchdog, 12 h up").
inline std::string telemetry_last_reset(const ReceiverData &d) {
  if (!d.have_status)
    return "";
  std::string s = telemetry_reset_text(d.reset_cause);
  if (d.have_diag && d.bc_cause != 0xFF) {
    s += "; the run before: up " + telemetry_age(d.bc_uptime_s);
    if (d.bc_cause == 0) {
      s += ", no exception";
    } else {
      char b[16];
      snprintf(b, sizeof(b), " at 0x%04X", static_cast<unsigned>(d.bc_pc));
      s += ", " + telemetry_exception(d.bc_cause) + b;
    }
  } else if (d.have_faults && (d.fm_rst & 3) != 0 &&
             !(d.reset_cause & (ULTRA_ST_RST_WATCHDOG | ULTRA_ST_RST_WWDG))) {   // (this start's own: said above)
    s += "; the last abnormal one: ";
    s += (d.fm_rst & 3) == 2 ? telemetry_exception(static_cast<uint8_t>(d.fm_rst >> 2)) : std::string("watchdog");
    s += ", " + std::to_string(d.fm_rst_up_h) + (d.fm_rst_up_h == 255 ? " h or more" : " h") + " up";
  }
  return s;
}

inline const char *telemetry_limit_source(uint8_t src) {
  static const char *const SRC[4] = {"board rating", "user limit", "PWM preset", "fuse"};
  return src < 4 ? SRC[src] : "limit";
}

/// Why the light is dim or off right now: "ok", or the reasons ("tripped: short circuit, retry in 8 s", "limiting to
/// 4.00 A (user limit)", "soft start", "signal loss", "calibration / test jig", ...): OUTPUTS' bits and (0.2.5)
/// STATUS's reason.
inline std::string telemetry_status_text(const ReceiverData &d) {
  if (!d.have_status)
    return "";
  std::vector<std::string> why;
  const uint8_t st = d.power_state, rw = receiver_why(d);
  uint8_t ou = d.have_outputs ? d.why : 0;
  static const uint8_t WHY_OU[12] = {0, ULTRA_OU_W_TRIPPED, ULTRA_OU_W_TRIPPED, ULTRA_OU_W_TRIPPED, ULTRA_OU_W_TEST,
                                     ULTRA_OU_W_TEST, 0, 0, ULTRA_OU_W_SIGNAL_LOSS, ULTRA_OU_W_SOFT_START,
                                     ULTRA_OU_W_DARK_HOLD, ULTRA_OU_W_PATTERN};
  if (rw < 12)
    ou |= WHY_OU[rw];
  if ((st & ULTRA_ST_TRIPPED) || (ou & ULTRA_OU_W_TRIPPED)) {
    std::string t = "tripped";
    if (rw == ULTRA_ST_WHY_SHORT || (rw != ULTRA_ST_WHY_OVERLOAD && d.have_power && d.trip_cause == ULTRA_PW_CAUSE_SHORT))
      t += ": short circuit";
    else if (rw == ULTRA_ST_WHY_OVERLOAD || (d.have_power && d.trip_cause == ULTRA_PW_CAUSE_OVERLOAD))
      t += ": overload";
    if (rw == ULTRA_ST_WHY_RETRIES_OUT)
      t += ", retries used up";
    else if (d.have_power && d.retry_s != 0)
      t += d.retry_s == 255 ? ", retry in 255 s or more" : ", retry in " + std::to_string(d.retry_s) + " s";
    why.push_back(t);
  }
  if ((st & ULTRA_ST_LIMITING) || (ou & ULTRA_OU_W_LIMIT)) {
    std::string t = "limiting";
    if (d.have_power && d.limit_ma != 0xFFFF)
      t += " to " + telemetry_amps(d.limit_ma) + " (" + telemetry_limit_source(d.limit_source) + ")";
    why.push_back(t);
  }
  if ((st & ULTRA_ST_CH_LIMITING) || (ou & ULTRA_OU_W_CH_LIMIT))
    why.push_back("channel limit");
  if (ou & ULTRA_OU_W_SOFT_START)
    why.push_back("soft start");
  if (ou & ULTRA_OU_W_SIGNAL_LOSS)
    why.push_back("signal loss");
  if (ou & ULTRA_OU_W_TEST)
    why.push_back("calibration / test jig");
  if (ou & ULTRA_OU_W_PATTERN)
    why.push_back("identify / acknowledge pattern");
  if (ou & ULTRA_OU_W_DARK_HOLD)
    why.push_back("held dark");
  if (why.empty())
    return "ok";
  std::string s = why[0];
  for (size_t i = 1; i < why.size(); i++)
    s += ", " + why[i];
  return s;
}

/// The power events since the board's power-up, newest details first: "no events", or e.g. "1 short circuit (35 s
/// ago, 12.40 A), 3 limit events (last 2 min ago, 4.10 A), 1 supply dip (to 18.20 V, 76 %, 12 ms, 1.0 h ago)".
inline std::string telemetry_power_events_text(const ReceiverData &d) {
  if (!d.have_power && !d.have_supply)
    return "";
  std::vector<std::string> ev;
  auto count = [](uint32_t n, const char *one, const char *many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
  };
  if (d.have_power) {
    const bool last_short = d.trip_cause == ULTRA_PW_CAUSE_SHORT, last_ovl = d.trip_cause == ULTRA_PW_CAUSE_OVERLOAD;
    if (d.short_trips) {
      std::string t = count(d.short_trips, "short circuit", "short circuits");
      if (last_short && d.trip_age_s != 0xFFFFFF)
        t += " (last " + telemetry_age(d.trip_age_s) + " ago, " + telemetry_amps(d.trip_ma) + ")";
      ev.push_back(t);
    }
    if (d.overload_trips) {
      std::string t = count(d.overload_trips, "overload trip", "overload trips");
      if (last_ovl && d.trip_age_s != 0xFFFFFF)
        t += " (last " + telemetry_age(d.trip_age_s) + " ago, " + telemetry_amps(d.trip_ma) + ")";
      ev.push_back(t);
    }
    if (d.limit_events) {
      std::string t = count(d.limit_events, "limit event", "limit events");
      if (d.limit_age_s != 0xFFFFFF)
        t += " (last " + telemetry_age(d.limit_age_s) + " ago, " + telemetry_amps(d.event_ma) + ")";
      ev.push_back(t);
    }
  }
  if (d.have_supply && d.dips) {
    std::string t = count(d.dips, "supply dip", "supply dips");
    if (d.logged != 0 && d.dip_age_s != 0xFFFFFF) {
      char b[64];
      snprintf(b, sizeof(b), " (last to %.2f V, %u %%, %u ms, ", d.dip_mv / 1000.0, static_cast<unsigned>(d.dip_pct),
               static_cast<unsigned>(d.dip_ms));
      t += b + telemetry_age(d.dip_age_s) + " ago)";
    }
    ev.push_back(t);
  }
  if (ev.empty())
    return "no events";
  std::string s = ev[0];
  for (size_t i = 1; i < ev.size(); i++)
    s += ", " + ev[i];
  return s;
}

/// Channel health from the last calibration test and the running model: "not tested", "ok", or the findings.
inline std::string telemetry_health_text(const ReceiverData &d) {
  if (!d.have_health)
    return "";
  std::vector<std::string> f;
  for (int i = 0; i < 5; i++) {
    const uint8_t c = d.ch_class[i];
    if ((c & 0x0F) != ULTRA_HE_ABNORMAL)
      continue;
    std::string t = "CH" + std::to_string(i + 1) + " abnormal";
    if (c & ULTRA_HE_R_NOT_RISING)
      t += " (current didn't rise with the level)";
    else if (c & ULTRA_HE_R_ABOVE)
      t += " (far above the test before)";
    else if (c & ULTRA_HE_R_BELOW)
      t += " (far below the test before, or no load)";
    f.push_back(t);
  }
  if (d.he_flags & ULTRA_HE_F_DARK)
    f.push_back("current with every output off (a FET stuck on?)");
  if (d.he_flags & ULTRA_HE_F_DRIFT)
    f.push_back("the running model is far off the test on a channel");
  if (receiver_025(d) && (d.he_flags & ULTRA_HE_F_CAL_SUGGESTED))
    f.push_back("a calibration test is suggested");
  std::string s;
  if (f.empty()) {
    s = (d.he_flags & ULTRA_HE_F_TESTED) || d.ch_class[0] != ULTRA_HE_UNTESTED ? "ok" : "not tested";
  } else {
    s = f[0];
    for (size_t i = 1; i < f.size(); i++)
      s += ", " + f[i];
  }
  // (0.2.5) the calibration trend: the newest stored test at 100 % against the first
  if (d.have_faults && d.trend_tests >= 2) {
    std::string t;
    for (int i = 0; i < 5; i++) {
      if (d.trend[i] == ULTRA_HE_NA)
        continue;
      char b[24];
      snprintf(b, sizeof(b), "%sCH%d %+d %%", t.empty() ? "" : ", ", i + 1, static_cast<int>(d.trend[i]));
      t += b;
    }
    if (!t.empty()) {
      s += "; trend over " + std::to_string(d.trend_tests) + " tests";
      if (d.trend_lit_h != 0xFFFF)
        s += " and " + std::to_string(d.trend_lit_h) + " lit h";
      s += ": " + t;
    }
  }
  return s;
}

// ---------------------------------------------------------------------------------------------------------- poller
struct TelemetryIntervals {
  uint32_t status_ms{3000};
  uint32_t power_ms{5000};   // POWER and OUTPUTS
  uint32_t energy_ms{60000};
  uint32_t supply_ms{60000};
  uint32_t health_ms{60000};
  uint32_t lifetime_ms{300000};   // LIFETIME
  uint32_t diag_ms{300000};       // DIAG's two parts (and after every restart: IDENTITY makes every page due)
};

class TelemetryPoller {
 public:
  static constexpr uint32_t MIN_SPACING_MS = 10;   // at most one RW frame per 10 ms (wiki: Return Channel)
  static constexpr uint32_t RETRY_MS = 250;        // a page asked again after a silenced or BUSY answer
  static constexpr uint32_t BACKOFF_MAX_MS = 10000;
  static constexpr uint8_t UNAVAILABLE_MISSES = 3; // windows in a row: its sensors show "unavailable"
  static constexpr uint8_t CUT_MISSES = 3;         // ... and the receivers behind it aren't polled
  static constexpr uint32_t ACK_DELAY_MS = 1000;   // the acknowledge goes with the POWER poll this long after a read
  // ... and within this long of a STATUS that shows no restart: a restarted receiver answers again only after ~0.5 s
  // (its clock rated), so an acknowledge never reaches a new boot whose counts happen to match
  static constexpr uint32_t ACK_FRESH_MS = 100;
  static constexpr uint32_t RESTART_HOLD_MS = ULTRA_RW_LOCK_MS;
  // a page's time further ahead than any interval (24 h at most, and its backoff): from over half a millis() wrap
  // ago (a receiver not heard for that long, a line in standby for weeks): overdue
  static constexpr int32_t STALE_MS = 25 * 3600000;

  /// receivers: the first boards of the line (1..64); rate ULTRA_RW_RATE_*
  void init(uint16_t receivers, uint8_t rate, const TelemetryIntervals &iv, uint8_t seed = 0x5A) {
    this->rx_.assign(receivers, Receiver{});
    this->rate_ = rate < ULTRA_RW_RATES ? rate : ULTRA_RW_RATE_1M;
    this->iv_ = iv;
    for (size_t i = 0; i < this->rx_.size(); i++)
      this->rx_[i].seq = static_cast<uint8_t>(seed + 37 * i);
  }
  uint16_t receivers() const { return static_cast<uint16_t>(this->rx_.size()); }
  uint8_t rate() const { return this->rate_; }
  /// The longest window one RW frame can open (us, W): up to 16 answer.
  uint32_t max_window_us() const {
    const size_t n = this->rx_.size() < ULTRA_RW_MAX_SLOTS ? this->rx_.size() : ULTRA_RW_MAX_SLOTS;
    return ultra_rw_window_us(static_cast<uint8_t>(n), this->rate_);
  }

  /// The chain took a preamble (it locks again): no RW frame before ULTRA_RW_LOCK_MS of headers after it.
  void locked(uint32_t now_ms) {
    this->locked_ = true;
    this->lock_ms_ = now_ms;
    this->lock_wait_ = true;
  }
  /// The boards lost their supply (or the line stopped): every receiver unavailable; they restart.
  /// Every receiver's sensors again (take_updates: TU_AVAILABILITY): their status changed with the line's power
  void touch_all() {
    for (auto &r : this->rx_)
      r.updated |= TU_AVAILABILITY;
  }
  void unpowered() {
    this->locked_ = false;
    for (auto &r : this->rx_) {
      this->unavailable_(r);
      r.misses = 0;
      r.ack_due = false;
    }
  }
  /// The ledger's sync for receiver i (LIFE_SYNC, its 23 argument bytes): asked next (until it answers)
  void request_sync(uint16_t i, const uint8_t *args, uint32_t now_ms) {
    if (i >= this->rx_.size())
      return;
    Receiver &r = this->rx_[i];
    std::memcpy(r.sync_args, args, sizeof(r.sync_args));
    r.sync_due = true;
    r.next[TP_SYNC] = now_ms - 60000;   // (before every other page)
  }
  bool sync_pending(uint16_t i) const { return i < this->rx_.size() && this->rx_[i].sync_due; }
  /// A page of receiver i due now (the ledger's reads)
  void read_now(uint16_t i, uint8_t tp, uint32_t now_ms) {
    if (i < this->rx_.size() && tp < TP_COUNT)
      this->rx_[i].next[tp] = now_ms;
  }

  /// An RW frame should go out now: the chain is locked, the spacing allows one, and a receiver it can hear has a
  /// page due.
  bool due(uint32_t now_ms) const {
    if (!this->locked_ || this->rx_.empty())
      return false;
    // the lock interval after a preamble (> in whole ms: at least ULTRA_RW_LOCK_MS whatever the phase of millis());
    // the first frame after it forgets it (build()), so a lock half a millis() wrap old (24.8 days) can't stop the
    // polls
    if (this->lock_wait_ && static_cast<int32_t>(now_ms - this->lock_ms_) <= (int32_t) ULTRA_RW_LOCK_MS)
      return false;
    if (this->sent_any_ && now_ms - this->last_ms_ < MIN_SPACING_MS)
      return false;
    const size_t reach = this->reach_(now_ms);
    for (size_t i = 0; i < reach; i++) {
      int32_t overdue;
      if (this->pick_(this->rx_[i], now_ms, &overdue) < TP_COUNT && overdue >= 0)
        return true;
    }
    return false;
  }

  /// The RW frame's POLL slots for boards 0..n-1 into slots (n x ULTRA_SLOT_BYTES; boards behind them take normal
  /// slots); returns n (0: nothing to poll). window_us: W, the window after the frame's last bit.
  uint16_t build(uint32_t now_ms, uint8_t *slots, uint32_t *window_us) {
    // what is over is forgotten (compared to an old time half a millis() wrap later, it would look new again)
    if (this->lock_wait_ && static_cast<int32_t>(now_ms - this->lock_ms_) > (int32_t) ULTRA_RW_LOCK_MS)
      this->lock_wait_ = false;
    for (auto &r : this->rx_) {
      if (r.hold && static_cast<int32_t>(now_ms - r.hold_until) >= 0)
        r.hold = false;
    }
    const size_t reach = this->reach_(now_ms);
    this->win_.clear();
    if (reach == 0)
      return 0;
    // who answers: every receiver it can hear (so a missing answer shows which one broke the chain), the 16 most
    // overdue of a longer chain
    std::vector<uint16_t> answer;
    for (size_t i = 0; i < reach; i++)
      answer.push_back(static_cast<uint16_t>(i));
    if (answer.size() > ULTRA_RW_MAX_SLOTS) {
      std::vector<int32_t> over(reach);
      for (size_t i = 0; i < reach; i++)
        this->pick_(this->rx_[i], now_ms, &over[i]);
      std::vector<uint16_t> by = answer;
      for (size_t i = 1; i < by.size(); i++) {   // insertion sort: most overdue first, chain order on ties
        const uint16_t v = by[i];
        size_t j = i;
        for (; j > 0 && over[by[j - 1]] < over[v]; j--)
          by[j] = by[j - 1];
        by[j] = v;
      }
      std::vector<bool> chosen(reach, false);
      for (size_t i = 0; i < ULTRA_RW_MAX_SLOTS; i++)
        chosen[by[i]] = true;
      answer.clear();
      for (size_t i = 0; i < reach; i++) {
        if (chosen[i])
          answer.push_back(static_cast<uint16_t>(i));   // indices rise along the chain
      }
    }
    const uint8_t slots_r = static_cast<uint8_t>(answer.size());
    const uint16_t last = answer.back();
    size_t k = 0;
    for (uint16_t b = 0; b <= last; b++) {
      Receiver &r = this->rx_[b];
      uint8_t *s = slots + ULTRA_SLOT_BYTES * b;
      r.seq++;
      if (k < answer.size() && answer[k] == b) {
        int32_t overdue;
        uint8_t tp = this->pick_(r, now_ms, &overdue);
        if (tp >= TP_COUNT)
          tp = TP_STATUS;
        uint8_t arg[23]{TP_PART[tp]};
        uint32_t nargs = TP_PART[tp] != 0 ? 1 : 0;
        if (tp == TP_SYNC) {   // the ledger's counters (its arguments as asked)
          std::memcpy(arg, r.sync_args, sizeof(r.sync_args));
          nargs = sizeof(r.sync_args);
        }
        if (tp == TP_POWER && r.ack_due && (!r.d.have_status || now_ms - r.status_ms > ACK_FRESH_MS))
          tp = TP_STATUS;   // its STATUS first (a restart drops the acknowledge), the acknowledge in the next window
        if (tp == TP_POWER && r.ack_due) {   // acknowledge exactly the counts as last read (the dips: SUPPLY's)
          arg[0] = ULTRA_PW_ARG_ACK;
          ultra_put16(arg + 1, r.d.limit_events);
          ultra_put16(arg + 3, r.d.overload_trips);
          ultra_put16(arg + 5, r.d.short_trips);
          ultra_put16(arg + 7, r.d.have_supply ? r.d.dips : 0);
          nargs = 9;
          r.ack_sent = true;
        }
        ultra_slot_poll(s, r.seq, static_cast<uint8_t>(k), slots_r, TP_PAGE_ID[tp], this->rate_, arg, nargs);
        this->win_.push_back(Asked{b, tp, static_cast<uint8_t>(k), r.seq, false});
        r.polls++;
        k++;
      } else {
        ultra_slot_poll(s, r.seq, ULTRA_POLL_RELAY_ONLY, slots_r, ULTRA_PAGE_STATUS, this->rate_, nullptr, 0);
      }
    }
    *window_us = ultra_rw_window_us(slots_r, this->rate_);
    this->last_ms_ = now_ms;
    this->sent_any_ = true;
    this->stats_.windows++;
    return static_cast<uint16_t>(last + 1);
  }

  /// The window of the last build() is over: bytes = what the hub's UART received since the frame went out (junk,
  /// breaks, lead bytes and the answers, in any amount). Matches the answers, decodes them, reschedules.
  void finish(uint32_t now_ms, const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i + ULTRA_SLOT_BYTES <= n;) {
      if (bytes[i] == ULTRA_RSP_SYNC && ultra_answer_check(bytes + i) == 0) {
        // a 0xA5 inside an answer's DATA is not a start; junk that passed the CRC by chance (1 in 65,536) but
        // matched no POLL skips nothing
        i += this->answer_(now_ms, bytes + i) ? ULTRA_SLOT_BYTES : 1;
      } else {
        if (bytes[i] == ULTRA_RSP_SYNC)
          this->stats_.bad_crc++;
        i++;
      }
    }
    bool silenced = false;   // a receiver in front of this one didn't answer: this one couldn't be heard
    for (const Asked &a : this->win_) {
      Receiver &r = this->rx_[a.rx];
      if (a.answered)
        continue;
      r.missed++;
      if (silenced) {
        r.next[a.tp] = now_ms + RETRY_MS;
        continue;
      }
      silenced = true;
      this->stats_.missing++;
      if (r.misses < 255)
        r.misses++;
      const uint32_t back = r.misses >= 6 ? BACKOFF_MAX_MS : RETRY_MS << (r.misses - 1);
      r.next[a.tp] = now_ms + (back > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : back);
      if (r.misses >= UNAVAILABLE_MISSES)
        this->unavailable_(r);
    }
    // receivers behind a cut can't be heard: unavailable
    if (this->cut_(now_ms)) {
      for (size_t i = this->reach_(now_ms); i < this->rx_.size(); i++)
        this->unavailable_(this->rx_[i]);
    }
    this->win_.clear();
  }

  const ReceiverData &data(uint16_t i) const { return this->rx_[i].d; }
  bool answering(uint16_t i) const { return this->rx_[i].answering; }
  uint8_t misses(uint16_t i) const { return this->rx_[i].misses; }
  /// POLLs that asked receiver i for an answer, and the ones that got none (missed polls)
  uint32_t polls(uint16_t i) const { return this->rx_[i].polls; }
  uint32_t missed(uint16_t i) const { return this->rx_[i].missed; }
  bool has_uid(uint16_t i) const { return this->rx_[i].have_uid; }
  uint16_t uid16(uint16_t i) const { return this->rx_[i].uid16; }
  /// Read OUTPUTS from receiver i too (several lights on it: their estimated power, why they're dim); off by default
  void set_outputs(uint16_t i, bool on) {
    if (i < this->rx_.size())
      this->rx_[i].outputs = on;
  }
  /// The receiver that keeps missing and cuts off the ones behind it (-1: none).
  int cut_at(uint32_t now_ms) const {
    for (size_t i = 0; i < this->rx_.size(); i++) {
      if (this->rx_[i].misses >= CUT_MISSES)
        return static_cast<int>(i);
    }
    (void) now_ms;
    return -1;
  }
  /// What changed since the last call: bits 1 << TP_* (pages read), TU_AVAILABILITY, TU_RESTART.
  uint32_t take_updates(uint16_t i) {
    const uint32_t u = this->rx_[i].updated;
    this->rx_[i].updated = 0;
    return u;
  }
  struct Stats {
    uint32_t windows{0};     // RW frames
    uint32_t answers{0};     // answers matched
    uint32_t missing{0};     // first missing answers (the ones that count)
    uint32_t bad_crc{0};     // SYNC bytes without a valid answer behind them (junk, broken answers)
    uint32_t stale{0};       // valid answers that matched no POLL of the window (late, a wrong SEQ / page)
    uint32_t contention{0};  // answers with RFLAGS CONTENTION
    uint32_t restarts{0};    // receivers that restarted or were swapped
  };
  const Stats &stats() const { return this->stats_; }
  /// For tests: the SEQ of a receiver's last POLL
  uint8_t seq(uint16_t i) const { return this->rx_[i].seq; }

 protected:
  struct Receiver {
    ReceiverData d;
    bool have_uid{false};
    uint16_t uid16{0};
    bool answering{false};
    bool reported{false};      // its "unavailable" went out (take_updates TU_AVAILABILITY)
    uint8_t misses{0};
    uint8_t seq{0};
    bool need_identity{true};
    uint16_t unsupported{0};   // bit 1 << TP_*: answered UNSUPPORTED (until the board restarts or changes)
    uint32_t next[TP_COUNT]{};
    uint32_t hold_until{0};    // restarted: not polled past before this
    bool hold{false};          // (build() forgets it once over: it can't come back half a millis() wrap later)
    uint32_t status_ms{0};     // when its last STATUS was read (restart detection)
    bool ack_due{false};       // POWER showed a latch: acknowledge with the next POWER poll
    bool ack_sent{false};      // ... it went out with the last one
    bool outputs{false};       // OUTPUTS is read too (a receiver with several lights: their power, why they're dim)
    uint32_t polls{0}, missed{0};   // POLLs that asked it for an answer, and the ones without one
    uint32_t updated{0};
    // LIFE_SYNC: asked by the ledger (its arguments), until it answers
    bool sync_due{false};
    uint8_t sync_args[23]{};
  };
  struct Asked {
    uint16_t rx;
    uint8_t tp, index, seq;
    bool answered;
  };

  uint32_t interval_(uint8_t tp) const {
    switch (tp) {
      case TP_STATUS:
        return this->iv_.status_ms;
      case TP_POWER:
      case TP_OUTPUTS:
        return this->iv_.power_ms;
      case TP_ENERGY:
        return this->iv_.energy_ms;
      case TP_SUPPLY:
        return this->iv_.supply_ms;
      case TP_HEALTH:
        return this->iv_.health_ms;
      case TP_LIFETIME:
        return this->iv_.lifetime_ms;
      case TP_DIAG:
      case TP_FAULTS:
        return this->iv_.diag_ms;
      default:
        return 0;
    }
  }
  bool enabled_(const Receiver &r, uint8_t tp) const {
    if (r.unsupported & (1u << tp))
      return false;
    if (tp == TP_IDENTITY)
      return r.need_identity;
    if (tp == TP_STATUS)
      return true;
    if (!r.d.have_identity)   // the optional pages once IDENTITY says which the board has
      return false;
    if (tp == TP_OUTPUTS && !r.outputs)
      return false;
    if (tp == TP_SYNC && !r.sync_due)
      return false;
    return TP_FEATURE[tp] == 0 || (r.d.features & TP_FEATURE[tp]) != 0;
  }
  /// The page to ask a receiver for: the most overdue one it has (overdue < 0: none due yet, the soonest).
  uint8_t pick_(const Receiver &r, uint32_t now_ms, int32_t *overdue) const {
    uint8_t best = TP_COUNT;
    int32_t most = INT32_MIN;
    for (uint8_t tp = 0; tp < TP_COUNT; tp++) {
      if (!this->enabled_(r, tp))
        continue;
      int32_t o = static_cast<int32_t>(now_ms - r.next[tp]);
      if (o < -STALE_MS)
        o = INT32_MAX / 2;   // (more than any interval ahead: its time from over half a millis() wrap ago: overdue)
      if (o > most) {
        most = o;
        best = tp;
      }
    }
    *overdue = most;
    return best;
  }
  bool cut_(uint32_t now_ms) const { return this->cut_at(now_ms) >= 0; }
  /// it doesn't answer (or can't be heard): its sensors unavailable, once (also one that never answered)
  static void unavailable_(Receiver &r) {
    if (r.answering || !r.reported)
      r.updated |= TU_AVAILABILITY;
    r.answering = false;
    r.reported = true;
  }
  /// The receivers that can be heard now: up to and including one that keeps missing, or one that restarted less
  /// than RESTART_HOLD_MS ago.
  size_t reach_(uint32_t now_ms) const {
    for (size_t i = 0; i < this->rx_.size(); i++) {
      const Receiver &r = this->rx_[i];
      if (r.misses >= CUT_MISSES || (r.hold && static_cast<int32_t>(now_ms - r.hold_until) < 0))
        return i + 1;
    }
    return this->rx_.size();
  }

  /// A new board at a position, or a restart: what it said before is void (IDENTITY again, no acknowledge across it).
  void restarted_(Receiver &r, uint32_t now_ms, bool new_board) {
    if (new_board) {
      r.d = ReceiverData{};
      r.unsupported = 0;
      r.updated |= (1u << TP_COUNT) - 1;   // every value unknown again
      r.sync_due = false;   // (the ledger asks again for the board that's there)
    }
    r.need_identity = true;
    r.next[TP_IDENTITY] = now_ms;
    r.ack_due = r.ack_sent = false;
    r.hold = true;
    r.hold_until = now_ms + RESTART_HOLD_MS;
    r.updated |= TU_RESTART;
    this->stats_.restarts++;
  }

  /// one answer of the window; false: it matched no POLL of it (stale, junk)
  bool answer_(uint32_t now_ms, const uint8_t *a) {
    const uint8_t index = a[2] >> 4, rflags = a[2] & 0x0F, seq = a[3], page = a[1];
    Asked *q = nullptr;
    for (auto &w : this->win_) {
      if (w.index == index && !w.answered)
        q = &w;
    }
    if (q == nullptr || q->seq != seq ||
        (page != TP_PAGE_ID[q->tp] && page != ULTRA_PAGE_BUSY && page != ULTRA_PAGE_UNSUPPORTED)) {
      this->stats_.stale++;
      return false;
    }
    q->answered = true;
    this->stats_.answers++;
    Receiver &r = this->rx_[q->rx];
    const uint16_t uid = static_cast<uint16_t>(ultra_get16(a + 4));
    if (r.have_uid && uid != r.uid16)
      this->restarted_(r, now_ms, true);
    r.have_uid = true;
    r.uid16 = uid;
    r.misses = 0;
    if (!r.answering) {
      r.answering = true;
      r.reported = false;
      r.updated |= TU_AVAILABILITY;
    }
    if (rflags & ULTRA_RSP_CONTENTION)
      this->stats_.contention++;
    if ((rflags & ULTRA_RSP_EVENT) && !r.ack_due && this->enabled_(r, TP_POWER) &&
        static_cast<int32_t>(r.next[TP_POWER] - now_ms) > 0)
      r.next[TP_POWER] = now_ms;   // a power event: read it soon (one being acknowledged: known)
    const uint8_t tp = q->tp;
    if (tp == TP_POWER && r.ack_sent)
      r.ack_due = r.ack_sent = false;   // the answer shows the state after the acknowledge
    if (page == ULTRA_PAGE_BUSY) {
      r.next[tp] = now_ms + RETRY_MS;
      return true;
    }
    if (page == ULTRA_PAGE_UNSUPPORTED) {
      r.unsupported |= static_cast<uint16_t>(1u << tp);
      if (tp == TP_IDENTITY)
        r.need_identity = false;
      return true;
    }
    r.next[tp] = now_ms + this->interval_(tp);   // (decode_ may want it sooner)
    this->decode_(r, tp, a + ULTRA_RSP_DATA, now_ms);
    return true;
  }

  void decode_(Receiver &r, uint8_t tp, const uint8_t *data, uint32_t now_ms) {
    ReceiverData &d = r.d;
    const uint32_t old_up = d.uptime_s, old_ms = r.status_ms;
    const bool had_status = d.have_status;
    const uint8_t old_state = d.power_state;
    decode_receiver_page(d, TP_PAGE_ID[tp], data, TP_PART[tp]);
    r.updated |= 1u << tp;
    switch (tp) {
      case TP_IDENTITY:
        r.need_identity = false;
        for (uint8_t p = TP_POWER; p < TP_COUNT; p++) {   // the pages it has: due now
          if (p != TP_STATUS && this->enabled_(r, p))
            r.next[p] = now_ms;
        }
        break;
      case TP_STATUS: {
        r.status_ms = now_ms;
        if (had_status) {
          // its uptime should have grown by the hub's time since (modulo 2^24): far behind = it restarted
          const uint32_t elapsed = (now_ms - old_ms) / 1000;
          const uint32_t expect = (old_up + elapsed) & 0xFFFFFF;
          const uint32_t behind = (expect - d.uptime_s) & 0xFFFFFF;
          if (behind > 5 + elapsed / 32 && behind < 0x800000)   // (+3 %: the boards' clocks)
            this->restarted_(r, now_ms, false);
        }
        if (d.power_state != old_state) {   // the power state or (0.2.5) its reason changed: the details soon
          if (this->enabled_(r, TP_POWER))
            r.next[TP_POWER] = now_ms;
          if (this->enabled_(r, TP_OUTPUTS))
            r.next[TP_OUTPUTS] = now_ms;   // (why the outputs are dim: else up to power_ms old)
        }
        break;
      }
      case TP_SYNC:
        r.sync_due = false;
        break;
      case TP_POWER:
        if (d.pw_latched != 0) {
          // shown now (the sensors), acknowledged with the next POWER poll: exactly these counts
          r.ack_due = true;
          r.next[TP_POWER] = now_ms + ACK_DELAY_MS;
          if ((d.pw_latched & ULTRA_PW_EV_SUPPLY_DIP) && this->enabled_(r, TP_SUPPLY))
            r.next[TP_SUPPLY] = now_ms;   // a dip: its count first
        } else {
          r.ack_due = false;
        }
        break;
      default:
        break;
    }
  }

  std::vector<Receiver> rx_;
  std::vector<Asked> win_;
  TelemetryIntervals iv_{};
  uint8_t rate_{ULTRA_RW_RATE_1M};
  bool locked_{false};
  bool lock_wait_{false};   // the lock interval not waited out yet (build() forgets it after)
  uint32_t lock_ms_{0};
  bool sent_any_{false};
  uint32_t last_ms_{0};
  Stats stats_{};
};

// --------------------------------------------------------------------------------------- status for Home Assistant
/// A receiver's status, worst first: what its Status sensor shows. The ones up to RS_CAL_SUGGESTED turn its Problem on
/// (the protocol's hub rule, wiki: Return Channel, "Home Assistant": STATUS's reasons short, overload, retries used up,
/// too hot, a supply dip, signal loss; HEALTH's abnormal channel, dark current, calibration suggested, too hot).
enum ReceiverStatus : uint8_t {
  RS_NO_ANSWER = 0,
  RS_TRIPPED,
  RS_CHANNEL_FAULT,
  RS_TOO_HOT,
  RS_SIGNAL_LOSS,
  RS_SUPPLY_DIPS,
  RS_CAL_SUGGESTED,
  RS_LIMITING,
  RS_CALIBRATING,
  RS_SOFT_START,
  RS_HELD_DARK,
  RS_BLINKING,
  RS_OK,
  RS_STANDBY,   // its Diff-Power port isn't on (diffpower): not powered, not polled; never a problem, never the worst
  RS_WAKING,    // its line's power just came back: not heard yet, not missed enough to count as gone (as Standby)
  RS_COUNT,
};
static constexpr int16_t TOO_HOT_C10 = 850;          // a board before 0.2.5 (no hot flag): the MCU at 85 C or more
static constexpr uint32_t TRIP_RECENT_S = 600;       // a trip in the last 10 min still shows (a retrying short)
static constexpr uint32_t DIPS_RECENT_S = 86400;     // supply dips in the last 24 h

inline const char *receiver_status_name(ReceiverStatus s) {
  static const char *const NAMES[RS_COUNT] = {"No answer",   "Tripped",     "Channel fault",
                                              "Too hot",     "Signal loss", "Supply dips",
                                              "Calibration suggested",      "Limiting",
                                              "Calibrating", "Soft start",  "Held dark",
                                              "Blinking",    "OK",          "Standby",
                                              "Waking"};
  return s < RS_COUNT ? NAMES[s] : "OK";
}
inline bool receiver_status_problem(ReceiverStatus s) { return s <= RS_CAL_SUGGESTED; }

/// The channels of `mask` (bit n = CH n+1) that HEALTH classes abnormal.
inline uint8_t abnormal_channels(const ReceiverData &d, uint8_t mask) {
  uint8_t m = 0;
  for (int i = 0; d.have_health && i < 5; i++) {
    if ((mask & (1u << i)) && (d.ch_class[i] & 0x0F) == ULTRA_HE_ABNORMAL)
      m |= static_cast<uint8_t>(1u << i);
  }
  return m;
}

/// "CH2", "CH2 and CH4", "CH1, CH2 and CH4"
inline std::string channel_list(uint8_t mask) {
  std::vector<std::string> ch;
  for (int i = 0; i < 5; i++) {
    if (mask & (1u << i))
      ch.push_back("CH" + std::to_string(i + 1));
  }
  std::string s;
  for (size_t i = 0; i < ch.size(); i++)
    s += (i == 0 ? "" : i + 1 == ch.size() ? " and " : ", ") + ch[i];
  return s;
}

/// light 0: the receiver's status. Else a light's on it (its channels, bit n = CH n+1): only what dims that light (no
/// supply dips, temperature or calibration suggestion; a channel fault on its own channels, or the dark current).
inline ReceiverStatus receiver_status(const ReceiverData &d, bool answering, uint8_t light = 0) {
  if (!answering)
    return RS_NO_ANSWER;
  const uint8_t st = d.have_status ? d.power_state : 0, ou = d.have_outputs ? d.why : 0, why = receiver_why(d);
  const uint8_t hf = d.have_health ? d.he_flags : 0;
  const bool v025 = receiver_025(d);
  if (why == ULTRA_ST_WHY_SHORT || why == ULTRA_ST_WHY_OVERLOAD || why == ULTRA_ST_WHY_RETRIES_OUT ||
      (st & ULTRA_ST_TRIPPED) || (ou & ULTRA_OU_W_TRIPPED) ||
      (d.have_power && d.trip_age_s < TRIP_RECENT_S && (d.short_trips || d.overload_trips)))
    return RS_TRIPPED;
  if ((hf & ULTRA_HE_F_DARK) || abnormal_channels(d, light != 0 ? light : 0x1F) != 0)
    return RS_CHANNEL_FAULT;
  // too hot: the board's own rule since 0.2.5 (HEALTH's flag, with its hysteresis), the hub's before
  const bool hot = why == ULTRA_ST_WHY_HOT ||
                   (v025 ? (hf & ULTRA_HE_F_HOT) != 0
                         : d.have_status && d.mcu_temp != ULTRA_ST_NA && static_cast<int16_t>(d.mcu_temp) >= TOO_HOT_C10);
  if (hot && light == 0)
    return RS_TOO_HOT;
  if (why == ULTRA_ST_WHY_SIGNAL_LOSS || (ou & ULTRA_OU_W_SIGNAL_LOSS))
    return RS_SIGNAL_LOSS;
  if (light == 0) {
    if (why == ULTRA_ST_WHY_SUPPLY || (d.have_supply && d.dips != 0 && d.logged != 0 && d.dip_age_s < DIPS_RECENT_S))
      return RS_SUPPLY_DIPS;
    if (v025 && (hf & ULTRA_HE_F_CAL_SUGGESTED))
      return RS_CAL_SUGGESTED;
  }
  if ((st & (ULTRA_ST_LIMITING | ULTRA_ST_CH_LIMITING)) || (ou & (ULTRA_OU_W_LIMIT | ULTRA_OU_W_CH_LIMIT)))
    return RS_LIMITING;
  if (why == ULTRA_ST_WHY_TEST || why == ULTRA_ST_WHY_CALIBRATING || (ou & ULTRA_OU_W_TEST))
    return RS_CALIBRATING;
  if (why == ULTRA_ST_WHY_SOFT_START || (ou & ULTRA_OU_W_SOFT_START))
    return RS_SOFT_START;
  if (why == ULTRA_ST_WHY_DARK_HOLD || (ou & ULTRA_OU_W_DARK_HOLD))
    return RS_HELD_DARK;
  if (why == ULTRA_ST_WHY_PATTERN || (ou & ULTRA_OU_W_PATTERN))
    return RS_BLINKING;
  return RS_OK;
}

/// A light's status on a receiver with several lights (its channels): the receiver's, as far as it dims the light.
inline ReceiverStatus light_status(const ReceiverData &d, bool answering, uint8_t channels) {
  return receiver_status(d, answering, channels != 0 ? channels : 0x1F);
}

/// "Receivers 1 and 2", "Receiver 1", "Receivers 1-3" (1-based positions before position `pos`, 0-based)
inline std::string receivers_before(uint16_t pos) {
  if (pos == 1)
    return "Receiver 1";
  if (pos == 2)
    return "Receivers 1 and 2";
  return "Receivers 1-" + std::to_string(pos);
}

/// The plain-language line under the Status: short and factual, from the pages (no clock: Home Assistant shows since
/// when). It changes only when its meaning does (Home Assistant records every change): no countdowns or running
/// durations, live numbers rounded. pos: the receiver's position on the port (0-based); cut: the receiver that cuts
/// the port off (-1: none).
inline std::string receiver_status_detail(const ReceiverData &d, ReceiverStatus s, uint16_t pos, int cut) {
  char b[160];
  const uint8_t why = receiver_why(d);
  switch (s) {
    case RS_NO_ANSWER:
      if (cut >= 0 && cut < pos)
        return "No answer: receiver " + std::to_string(cut + 1) +
               " before it doesn't answer, so this one can't be heard. Check that one first.";
      if (pos == 0)
        return "No answer. Check its cable and power, and the hub's RS-485 connection.";
      return "No answer. " + receivers_before(pos) + " before it answer" + (pos == 1 ? "s" : "") +
             ", so check its cable or power.";
    case RS_TRIPPED: {
      // the reason now (0.2.5), else the last trip's cause (POWER)
      const bool is_short = why == ULTRA_ST_WHY_SHORT ||
                            (why != ULTRA_ST_WHY_OVERLOAD && d.have_power && d.trip_cause == ULTRA_PW_CAUSE_SHORT);
      const bool is_overload = !is_short && (why == ULTRA_ST_WHY_OVERLOAD ||
                                             (d.have_power && d.trip_cause == ULTRA_PW_CAUSE_OVERLOAD));
      const char *cause = is_short ? "a short circuit" : is_overload ? "an overload" : "an overcurrent";
      if (d.have_power)
        snprintf(b, sizeof(b), "Tripped by %s at %.2f A", cause, d.trip_ma / 1000.0);
      else
        snprintf(b, sizeof(b), "Tripped by %s", cause);
      std::string t = b;
      if (why == ULTRA_ST_WHY_RETRIES_OUT)
        return t + "; off, its retries are used up. Fix the wiring of its outputs, then switch its lights off and on "
                   "(or its power).";
      if (d.have_power && d.retry_s != 0)
        t += "; it tries again by itself";   // (no countdown: the text changes with its meaning only)
      else if (!(d.power_state & ULTRA_ST_TRIPPED))
        t += "; on again";
      return t + ". Check the wiring of its outputs.";
    }
    case RS_CHANNEL_FAULT: {
      std::string t;
      const uint8_t ab = abnormal_channels(d, 0x1F);
      if (ab != 0) {
        t = channel_list(ab) + ((ab & (ab - 1)) != 0 ? " are" : " is") + " abnormal at the last calibration test";
        for (int i = 0; i < 5; i++) {   // the first one's finding
          if (!(ab & (1u << i)))
            continue;
          const uint8_t c = d.ch_class[i];
          t += c & ULTRA_HE_R_NOT_RISING ? " (the current didn't rise with the level)"
               : c & ULTRA_HE_R_ABOVE    ? " (far above the test before)"
               : c & ULTRA_HE_R_BELOW    ? " (far below the test before, or no load)"
                                         : "";
          break;
        }
        t += ": check its LEDs and wiring.";
      }
      if (d.he_flags & ULTRA_HE_F_DARK)
        t += std::string(t.empty() ? "" : " ") + "Current flows with every output off: a MOSFET may be stuck on.";
      return t;
    }
    case RS_TOO_HOT:   // (its temperature is the MCU temperature entity: not here)
      if (receiver_025(d))
        return "Its MCU is over its warning level. Check the cooling around the receiver.";
      return "Its MCU is at 85 \xC2\xB0" "C or more. Check the cooling around the receiver.";
    case RS_SIGNAL_LOSS:
      return "No frames from the hub: the outputs follow its signal_loss setting.";
    case RS_SUPPLY_DIPS: {
      if (!(d.have_supply && d.dips != 0 && d.logged != 0 && d.dip_age_s < DIPS_RECENT_S))
        return "A supply dip in the last 30 s. The supply or its cable may be too weak.";
      uint32_t recent = 0;   // (the log keeps the last 4)
      if (d.dip_age_s < DIPS_RECENT_S)
        recent = d.logged;
      snprintf(b, sizeof(b), "%u supply dip%s in the last 24 h, the last to %.1f V (%u %%) for %u ms at %.1f A.",
               static_cast<unsigned>(recent), recent == 1 ? "" : "s", d.dip_mv / 1000.0,
               static_cast<unsigned>(d.dip_pct), static_cast<unsigned>(d.dip_ms), d.dip_ma / 1000.0);
      return std::string(b) + " The supply or its cable may be too weak.";
    }
    case RS_CAL_SUGGESTED:
      return "A current calibration test is suggested: its currents are far off the last test, or a new load is "
             "connected.";
    case RS_LIMITING:
      if (d.have_power && d.limit_ma != 0xFFFF && (d.power_state & ULTRA_ST_LIMITING)) {
        // (by about 10 / 20 / ... %: the factor moves with every fade)
        const unsigned pct = static_cast<unsigned>(((65535u - d.factor) * 100u + 32767u) / 65535u);
        if (pct < 5)
          snprintf(b, sizeof(b), "The board limit (%.1f A, %s) is reducing the outputs a little.", d.limit_ma / 1000.0,
                   telemetry_limit_source(d.limit_source));
        else
          snprintf(b, sizeof(b), "The board limit (%.1f A, %s) is reducing the outputs by about %u %%.",
                   d.limit_ma / 1000.0, telemetry_limit_source(d.limit_source), (pct + 5) / 10 * 10);
        return b;
      }
      if (d.power_state & ULTRA_ST_CH_LIMITING)
        return "A channel limit is reducing a channel.";
      return "The board limit is reducing the outputs.";
    case RS_CALIBRATING:
      if (why == ULTRA_ST_WHY_TEST)
        return "The test jig's mode is on.";
      if (why == ULTRA_ST_WHY_CALIBRATING)
        return "A current calibration test is running.";
      return "A current calibration (or the test jig) is running.";
    case RS_SOFT_START:
      return "Soft start: the outputs ramp up after a power-up or a trip.";
    case RS_HELD_DARK:
      return "Held dark by the hub: a current calibration on its line.";
    case RS_BLINKING:
      return "Showing a blink pattern: identify, a firmware update's answer, its version or a fault code.";
    case RS_STANDBY:
      return "The port is in standby (4.5 V); the receiver is off and wakes with its lights.";
    case RS_WAKING:
      return "Its port just woke: waiting for its first answer.";
    default:
      return "OK.";
  }
}

// (The texts below change only with a new event, not with its age: Home Assistant records every change, and shows
// since when a state holds.)

/// The last trip: this power-up's (POWER: "short circuit at 12.40 A"), else the one the board keeps across power cuts
/// (0.2.5, DIAG part 1: "overload at 6.5 A (at 1234 lit hours, before its last power-up)"), or "none".
inline std::string telemetry_last_trip(const ReceiverData &d) {
  char b[96];
  if (d.have_power && (d.short_trips != 0 || d.overload_trips != 0)) {
    snprintf(b, sizeof(b), "%s at %.2f A", d.trip_cause == ULTRA_PW_CAUSE_SHORT ? "short circuit" : "overload",
             d.trip_ma / 1000.0);
    return b;
  }
  if (d.have_faults && (d.fm_trip & 3) != 0) {
    const unsigned half_a = d.fm_trip >> 2;
    snprintf(b, sizeof(b), "%s at %.1f A%s (", (d.fm_trip & 3) == 1 ? "short circuit" : "overload", half_a * 0.5,
             half_a == 63 ? " or more" : "");
    std::string t = b;
    if (d.fm_trip_lit_h != 0xFFFF)
      t += "at " + std::to_string(d.fm_trip_lit_h) + " lit hours, ";
    return t + "before its last power-up)";
  }
  return d.have_power || d.have_faults ? "none" : "";
}

/// The last supply dip: this power-up's (SUPPLY: "to 39.80 V (86 %) for 35 ms at 1.00 A"), else the one the board
/// keeps across power cuts (0.2.5: "to 86 % for 30 ms at 1.0 A (before its last power-up)"), or "none".
inline std::string telemetry_last_dip(const ReceiverData &d) {
  char b[96];
  if (d.have_supply && d.logged != 0) {
    snprintf(b, sizeof(b), "to %.2f V (%u %%) for %u ms at %.2f A", d.dip_mv / 1000.0,
             static_cast<unsigned>(d.dip_pct), static_cast<unsigned>(d.dip_ms), d.dip_ma / 1000.0);
    return b;
  }
  if (d.have_faults && d.fm_dip_pct != 0) {
    if (d.fm_dip_10ms == 255)
      snprintf(b, sizeof(b), "to %u %% for 2.5 s or more at %.1f A (before its last power-up)",
               static_cast<unsigned>(d.fm_dip_pct), d.fm_dip_a10 / 10.0);
    else
      snprintf(b, sizeof(b), "to %u %% for %u ms at %.1f A (before its last power-up)",
               static_cast<unsigned>(d.fm_dip_pct), static_cast<unsigned>(d.fm_dip_10ms) * 10u, d.fm_dip_a10 / 10.0);
    return b;
  }
  return d.have_supply || d.have_faults ? "none" : "";
}

/// The estimated power of the channels in `mask` (bit n = CH n+1): the receiver's measured input power, split by the
/// channel currents of its model (OUTPUTS). NAN while one of them isn't known.
inline float light_power_estimate(const ReceiverData &d, uint8_t mask) {
  const float total = telemetry_value(d, TV_INPUT_POWER);
  if (!d.have_outputs || std::isnan(total))
    return NAN;
  uint32_t all = 0, mine = 0;
  for (int i = 0; i < 5; i++) {
    all += d.ch_ma[i];
    if (mask & (1u << i))
      mine += d.ch_ma[i];
  }
  return all == 0 ? 0.0f : total * static_cast<float>(mine) / static_cast<float>(all);
}

// ------------------------------------------------------------------------------------------------- the port's load
/// A Diff-Power sender port: its receivers' bus-side input currents against the port's limit (3 x 750 mA PTCs = 2.25 A),
/// the power at the sender, and the cable along the chain (from each receiver's bus voltage and the current through
/// each segment: segment k carries the input currents of receivers k..N). Updated after every response window.
/// A receiver that doesn't answer makes the port values incomplete (complete() false): the current and power without
/// its share, no cable loss. The peak (0.2.5): each receiver's highest ~100 ms input current since its ENERGY read
/// before (about a minute), added up: at most what the port carried (the peaks needn't coincide).
class PortMonitor {
 public:
  struct Config {
    float limit_a{2.25f};        // the port's current limit
    float supply_v{NAN};         // the sender's voltage; NAN: learned (receiver 1's highest bus voltage)
    float warning_pct{90.0f};    // a warning (the Status) from this load on
    float problem_pct{95.0f};    // a problem after this load lasted problem_ms (100 %: at once)
    uint32_t problem_ms{30000};
  };
  static constexpr float RESISTANCE_MIN_A = 0.3f;   // a segment's resistance only from this current on (10 mV steps)
  static constexpr float RESISTANCE_ALPHA = 0.2f;   // its smoothing per window

  void configure(const Config &c) { this->cfg_ = c; }
  const Config &config() const { return this->cfg_; }

  void update(uint32_t now_ms, const TelemetryPoller &p) {
    const uint16_t n = p.receivers();
    if (this->drop_.size() != n) {
      this->drop_.assign(n, NAN);
      this->res_.assign(n, NAN);
    }
    std::vector<float> v(n, NAN), i(n, NAN);
    float total = 0.0f, rx_power = 0.0f, peak = 0.0f;
    bool any = false, any_power = false, any_peak = false;
    this->complete_ = true;
    this->missing_ = -1;
    for (uint16_t k = 0; k < n; k++) {
      const bool a = p.answering(k);
      const ReceiverData &d = p.data(k);
      v[k] = a ? telemetry_value(d, TV_SUPPLY_VOLTAGE) : NAN;
      i[k] = a ? telemetry_value(d, TV_INPUT_CURRENT) : NAN;
      if (std::isnan(v[k]) || std::isnan(i[k])) {
        this->complete_ = false;
        if (this->missing_ < 0)
          this->missing_ = k;
      }
      if (!std::isnan(i[k])) {
        total += i[k];
        any = true;
      }
      const float pk = a ? telemetry_value(d, TV_CURRENT_PEAK) : NAN;
      if (!std::isnan(pk))
        any_peak = true;
      if (!std::isnan(pk) || !std::isnan(i[k]))
        peak += std::isnan(pk) ? i[k] : std::isnan(i[k]) || pk > i[k] ? pk : i[k];
      const float w = a ? telemetry_value(d, TV_INPUT_POWER) : NAN;
      if (!std::isnan(w)) {
        rx_power += w;
        any_power = true;
      }
    }
    // the sender's voltage: configured, else learned (receiver 1 at its highest: the least load on the first cable)
    if (std::isnan(this->cfg_.supply_v) && n > 0 && !std::isnan(v[0]))
      this->learned_v_ = std::isnan(this->learned_v_) || v[0] > this->learned_v_ ? v[0] : this->learned_v_;
    const float vs = this->supply_v();
    this->current_ = any ? total : NAN;
    this->load_ = any ? total / this->cfg_.limit_a * 100.0f : NAN;
    this->peak_ = any_peak ? peak : NAN;
    this->power_ = any && !std::isnan(vs) ? vs * total : any_power ? rx_power : NAN;
    // the cable: segment k from receiver k-1 (the sender for k = 0) to receiver k
    this->loss_ = NAN;
    float through = total, loss = 0.0f, prev = vs;
    for (uint16_t k = 0; k < n; k++) {
      this->drop_[k] = !std::isnan(vs) && !std::isnan(v[k]) ? vs - v[k] : NAN;
      if (this->complete_ && !std::isnan(prev)) {
        const float dv = prev - v[k];
        loss += dv * through;
        if (through >= RESISTANCE_MIN_A && dv >= 0.0f) {
          const float r = dv / through;
          this->res_[k] = std::isnan(this->res_[k]) ? r : this->res_[k] + RESISTANCE_ALPHA * (r - this->res_[k]);
        }
      }
      prev = v[k];
      if (this->complete_)
        through -= i[k];
    }
    if (this->complete_ && !std::isnan(vs) && n > 0)
      this->loss_ = loss > 0.0f ? loss : 0.0f;
    // the load: a warning, a problem at 100 % or after problem_ms at problem_pct
    const bool high = !std::isnan(this->load_) && this->load_ >= this->cfg_.problem_pct;
    if (high && !this->high_)
      this->high_since_ = now_ms;
    this->high_ = high;
    this->sustained_ = high && now_ms - this->high_since_ >= this->cfg_.problem_ms;
  }

  float current_a() const { return this->current_; }
  float load_pct() const { return this->load_; }
  /// the receivers' peaks since their ENERGY reads before, added up (A, and against the limit; NAN: none reports one)
  float peak_a() const { return this->peak_; }
  float peak_load_pct() const { return std::isnan(this->peak_) ? NAN : this->peak_ / this->cfg_.limit_a * 100.0f; }
  /// peaks over the limit while the average is below the warning
  bool peaks() const { return !this->warning() && !std::isnan(this->peak_) && this->peak_load_pct() >= 100.0f; }
  /// at the sender: its voltage x the port current; without a sender voltage the receivers' power together
  float power_w() const { return this->power_; }
  float cable_loss_w() const { return this->loss_; }
  float cable_loss_pct() const {
    return std::isnan(this->loss_) || std::isnan(this->power_) || this->power_ <= 0.0f ? NAN
                                                                                       : this->loss_ / this->power_ * 100.0f;
  }
  float supply_v() const { return std::isnan(this->cfg_.supply_v) ? this->learned_v_ : this->cfg_.supply_v; }
  bool complete() const { return this->complete_; }
  /// the first receiver without a current or voltage (-1: none)
  int missing() const { return this->missing_; }
  bool warning() const { return !std::isnan(this->load_) && this->load_ >= this->cfg_.warning_pct; }
  bool over() const { return !std::isnan(this->load_) && this->load_ >= 100.0f; }
  bool problem() const { return this->over() || this->sustained_; }
  /// how long the load has been at problem_pct or more (ms; 0: it isn't)
  uint32_t high_ms(uint32_t now_ms) const { return this->high_ ? now_ms - this->high_since_ : 0; }
  /// the sender's voltage minus receiver k's (V; NAN: not known)
  float drop_v(uint16_t k) const { return k < this->drop_.size() ? this->drop_[k] : NAN; }
  /// the cable segment before receiver k, smoothed (ohm; NAN until it carried RESISTANCE_MIN_A)
  float resistance(uint16_t k) const { return k < this->res_.size() ? this->res_[k] : NAN; }

 protected:
  Config cfg_{};
  float learned_v_{NAN};
  float current_{NAN}, load_{NAN}, power_{NAN}, loss_{NAN}, peak_{NAN};
  bool complete_{false};
  int missing_{-1};
  bool high_{false}, sustained_{false};
  uint32_t high_since_{0};
  std::vector<float> drop_, res_;
};

/// A load in 5 % steps (the texts: they change only with a step, not with every reading)
inline unsigned load_step5(float pct) { return pct <= 0.0f ? 0u : static_cast<unsigned>((pct + 2.5f) / 5.0f) * 5u; }

/// The port's status, for its chip: "95 % load" (from the warning on; in 5 % steps), "No answer", the worst receiver
/// problem, "Peaks 105 %", "2 limiting", or "OK".
inline std::string port_status(const PortMonitor &m, const TelemetryPoller &p) {
  char b[24];
  if (m.warning()) {
    snprintf(b, sizeof(b), "%u %% load", load_step5(m.load_pct()));
    return b;
  }
  ReceiverStatus worst = RS_OK;
  unsigned limiting = 0;
  for (uint16_t k = 0; k < p.receivers(); k++) {
    const ReceiverStatus s = receiver_status(p.data(k), p.answering(k));
    worst = s < worst ? s : worst;
    limiting += s == RS_LIMITING;
  }
  if (receiver_status_problem(worst))
    return receiver_status_name(worst);
  if (m.peaks()) {
    snprintf(b, sizeof(b), "Peaks %u %%", load_step5(m.peak_load_pct()));
    return b;
  }
  if (limiting) {
    snprintf(b, sizeof(b), "%u limiting", limiting);
    return b;
  }
  return "OK";
}

/// The port's plain line: near / over its limit, peaks over it, or a receiver missing from its values, or "OK.". Its
/// numbers rounded (0.1 A, 5 %), no running duration: it changes with its meaning only (Home Assistant records every
/// change, and shows since when).
inline std::string port_status_detail(const PortMonitor &m) {
  char b[200];
  const float limit = m.config().limit_a;
  if (m.warning()) {
    std::string t;
    snprintf(b, sizeof(b), "%.1f A of %.2f A (%u %%)", m.current_a(), limit, load_step5(m.load_pct()));
    t = b;
    if (m.problem())
      t += ". The port's fuses (PTCs) may trip: lower the brightness or move a receiver to a quieter port.";
    else
      t += ": near the port's limit.";
    return t;
  }
  if (m.peaks()) {
    snprintf(b, sizeof(b), "Peaks up to %.1f A (%u %%) in the last minute: short peaks are fine; if they last, the "
             "port's fuses (PTCs) may trip.", m.peak_a(), load_step5(m.peak_load_pct()));
    return b;
  }
  if (!m.complete() && m.missing() >= 0) {
    snprintf(b, sizeof(b), "Receiver %d doesn't report: the port's current and power are without it.",
             m.missing() + 1);
    return b;
  }
  return "OK.";
}

/// A light's estimated energy (Wh): the mean of two power estimates over the time between them; a gap of more than
/// MAX_GAP_MS (no answers, the supply off) isn't counted. Starts at 0 (Home Assistant's total_increasing).
struct EnergyCounter {
  static constexpr uint32_t MAX_GAP_MS = 60000;
  double wh{0.0};
  float last_w{NAN};
  uint32_t last_ms{0};
  void add(float w, uint32_t now_ms) {
    if (!std::isnan(w) && !std::isnan(this->last_w) && now_ms - this->last_ms <= MAX_GAP_MS)
      this->wh += (this->last_w + w) * 0.5 * (now_ms - this->last_ms) / 3600000.0;
    this->last_w = w;
    this->last_ms = now_ms;
  }
};

}  // namespace esphome::dig2analog_ultra
