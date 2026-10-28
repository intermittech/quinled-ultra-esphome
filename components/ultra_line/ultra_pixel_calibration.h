#pragma once

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include "ultra_protocol.h"

namespace esphome::ultra_line {

// Page 0x06 format 2. Coefficients are BUS input mA for the WHOLE used count.
// Keep staging separate: no partial generation may become a dashboard model.
struct PixelCalibration {
  uint8_t meta[24]{}, first[24]{}, coefficients[24]{}, quality[24]{};
  uint8_t active_currents[12]{}, active_quality[8]{}; // retain a matching model while a routine reread is staged
  uint8_t next{1}, stage{0}, hub_error{0};
  bool have{false}, coherent{false};
  uint32_t revision{0};
  uint16_t get(uint8_t offset) const { return static_cast<uint16_t>(ultra_get16(meta + offset)); }
  uint8_t flags() const { return meta[ULTRA_UC_FLAGS]; }
  int32_t idle_ma() const { return static_cast<int32_t>(static_cast<int16_t>(ultra_get16(active_currents))) * 2; }
  uint16_t channel_ma(unsigned i) const { return static_cast<uint16_t>(ultra_get16(active_currents + 2 + 2*i)); }
  bool usable() const {
    return coherent && (flags() & (ULTRA_UCF_VALID | ULTRA_UCF_STALE | ULTRA_UCF_CLEAR_PENDING)) == ULTRA_UCF_VALID &&
           get(ULTRA_UC_USED) != 0;
  }
  static bool same(const uint8_t *a, const uint8_t *b) {
    // Format, flags, count, fingerprint, reference voltages and epoch. Phase/error/elapsed may advance.
    return a[0] == b[0] && a[1] == b[1] && std::memcmp(a + 4, b + 4, 18) == 0;
  }
  void invalidate() { have = coherent = false; stage = 0; next = 1; revision++; }
  void progress_only() { stage = 0; next = 1; }
  /// max_pixels: the board's own limits at 3 / 4 / 5 bytes per pixel (its descriptor's pixel record)
  bool read(uint8_t part, const uint8_t *p, bool running, const uint16_t *max_pixels) {
    if (part != next) { stage = 0; next = 1; return false; }
    if (part == 1) {
      if (p[0] != ULTRA_UC_FORMAT || p[ULTRA_UC_BPP] < 3 || p[ULTRA_UC_BPP] > 5 ||
          p[ULTRA_UC_PHASE] > ULTRA_UCS_FAILED || p[ULTRA_UC_ERROR] > ULTRA_UCE_OVERFLOW) {
        invalidate(); return false;
      }
      const unsigned cap = max_pixels[p[ULTRA_UC_BPP] - 3];
      if (ultra_get16(p+6) > cap || ultra_get16(p+8) > cap || ultra_get16(p+10) > cap) {
        invalidate(); return false;
      }
      if (!same(meta, p)) coherent = false;
      std::memcpy(meta, p, 24); have = true; revision++;
      if (running) { progress_only(); return false; }
      if (stage == 3) {
        coherent = same(first, p) && ultra_get16(coefficients + 2) == get(ULTRA_UC_COUNT) &&
                   quality[0] == ULTRA_UC_FORMAT && quality[1] == 1 &&
                   ultra_get16(quality + 2) == get(ULTRA_UC_COUNT) &&
                   ultra_get16(quality + 4) == get(ULTRA_UC_EPOCH) &&
                   get(ULTRA_UC_USED) == std::min(get(ULTRA_UC_CONFIGURED), get(ULTRA_UC_DETECTED)) &&
                   ((coefficients[1] & 15) == 2) == ((flags() & ULTRA_UCF_VALID) != 0) &&
                   (!(flags() & ULTRA_UCF_VALID) || coefficients[0] == 3);
        if (coherent) {
          std::memcpy(active_currents, coefficients + 7, sizeof(active_currents));
          std::memcpy(active_quality, quality + 6, sizeof(active_quality));
        }
        stage = 0; next = 1;
        return coherent;
      }
      std::memcpy(first, p, 24); stage = 1; next = 0;
    } else if (part == 0 && stage == 1) {
      std::memcpy(coefficients, p, 24); stage = 2; next = 2;
    } else if (part == 2 && stage == 2) {
      std::memcpy(quality, p, 24); stage = 3; next = 1;
    } else { stage = 0; next = 1; }
    return false;
  }
  std::string count_text() const {
    if (!have) return "Pixel count unknown";
    std::string s = "Configured " + std::to_string(get(ULTRA_UC_CONFIGURED)) + ", detected " +
        ((flags() & ULTRA_UCF_AT_LEAST) ? "at least " : "") + std::to_string(get(ULTRA_UC_DETECTED)) +
        ", used " + std::to_string(get(ULTRA_UC_USED));
    if (flags() & ULTRA_UCF_SHORT)
      s += "; fewer pixels found than configured: short strip, wrong LED type or a data break at pixel " +
           std::to_string(get(ULTRA_UC_DETECTED) + 1);
    else if (get(ULTRA_UC_DETECTED) > get(ULTRA_UC_CONFIGURED))
      s += "; more pixels found than configured";
    return s;
  }
  bool count_problem() const { return have && get(ULTRA_UC_DETECTED) && get(ULTRA_UC_DETECTED) != get(ULTRA_UC_CONFIGURED); }
  std::string result_text() const {
    static const char *const phases[] = {"Idle", "Queued", "Acquiring", "Measuring dark", "Counting pixels",
      "Measuring channels", "Checking mixtures", "Validating", "Restoring", "Done", "Failed"};
    static const char *const errors[] = {"", "current limit", "supply changed or missing", "noisy or inconsistent",
      "nonlinear or nonadditive", "configuration changed or invalid", "cancelled", "timed out", "encoder fault",
      "measured above prediction", "missing or delayed ADC", "weak channel", "busy", "invalid arguments",
      "first pixel not detectable", "coefficient overflow"};
    if (hub_error) return hub_error == ULTRA_UCE_TIMEOUT ? "Stopped: timed out or no answer" : "Stopped: cancelled";
    if (!have) return "Digital calibration unavailable";
    std::string s = phases[meta[2]];
    if (meta[3]) s += std::string(": ") + errors[meta[3]];
    if (flags() & ULTRA_UCF_STALE) s += "; stale voltage: recalibrate";
    if (flags() & ULTRA_UCF_PENDING) s += "; pending save (usable in RAM)";
    if (flags() & ULTRA_UCF_CLEAR_PENDING) s += "; clear pending";
    if (coherent && (flags() & ULTRA_UCF_VALID))
      s += "; K " + std::to_string(ultra_get16(active_quality + 0)) + ", noise " + std::to_string(ultra_get16(active_quality + 2)) +
           " mA, residual " + std::to_string(ultra_get16(active_quality + 4)) + " permille, bound " +
           std::to_string(ultra_get16(active_quality + 6)) + " mA/channel/address";
    return s;
  }
  // v2 is deliberately distinct from analog v1. idle_ma is aggregate, counted ONCE.
  // ip_ma is only the apparent idle/used intercept; cp_ma are logical R,G,B,W/WW,CW per address.
  std::string json(bool available) const {
    const bool ok = available && usable();
    const unsigned n = ok ? get(ULTRA_UC_USED) : 0;
    char b[256];
    const int idle = ok ? idle_ma() : 0;
    auto ch = [&](unsigned i) { return ok ? channel_ma(i) / static_cast<double>(n) : 0.0; };
    std::snprintf(b, sizeof(b),
      "{\"v\":2,\"src\":\"%s\",\"n\":%u,\"chip\":%u,\"bpp\":%u,\"order\":%u,\"vb\":%u,\"vl\":%u,\"gen\":%u,\"q\":%u,\"idle_ma\":%d,\"ip_ma\":%.3f,\"cp_ma\":[%.3f,%.3f,%.3f,%.3f,%.3f]}",
      ok ? "calibrated" : "default", n, meta[12], meta[13], get(14), get(16) * 10u, get(18) * 10u, get(20),
      available && have ? flags() : 0, idle, n ? idle / static_cast<double>(n) : 0.0,
      ch(0), ch(1), ch(2), ch(3), ch(4));
    return b;
  }
};

// One manual digital job per chain. Requests are sent once (RMT retry keeps the same SEQ).
// Quiet ownership remains through cancel/restore; only DONE/FAILED or the receiver's own deadline releases it:
// wait_ms = the protocol's host deadline from its descriptor, 50 + ceil(1.5 x the output's cal_ms)
// (ultra_descriptor_calibration_ms: the board's budget plus a 50 % + 50 ms margin).
struct PixelCalibrationJob {
  int board{-1};
  uint8_t action{0}, error{0};
  uint32_t started{0}, observed{0}, wait_ms{0};
  uint16_t prior_count{0}, prior_epoch{0}, prior_elapsed{0};
  uint8_t prior_error{0};
  bool sent{false}, seen_active{false}, waiting{false};
  bool active() const { return board >= 0; }
  bool quiet() const { return active() && !waiting; }
  bool start(uint16_t b, uint32_t now, const PixelCalibration &c, uint32_t deadline_ms, bool wait = false) {
    if (active()) return false;
    board = b; action = ULTRA_IDENT_CAL; started = now; observed = c.revision; wait_ms = deadline_ms;
    prior_count = c.get(4); prior_epoch = c.get(20); prior_elapsed = c.get(22); prior_error = c.meta[3];
    error = 0; sent = seen_active = false; waiting = wait; return true;
  }
  void cancel() { if (active()) { action = ULTRA_IDENT_RELEASE; error = ULTRA_UCE_CANCELLED; } }
  void on_sent(uint32_t now) { if (!sent) started = now; sent = true; }
  bool tick(uint32_t now, const PixelCalibration &c) {
    if (!active()) return false;
    if (now - started >= (sent ? wait_ms : 30000u)) { error = ULTRA_UCE_TIMEOUT; board = -1; return true; }
    if (!sent || c.revision == observed) return false;
    observed = c.revision;
    if (c.have && c.meta[2] >= ULTRA_UCS_QUEUED && c.meta[2] <= ULTRA_UCS_RESTORE) seen_active = true;
    // A terminal response must differ from the result retained before this request.
    if (c.have && c.meta[2] >= ULTRA_UCS_DONE && (seen_active || c.get(4) != prior_count || c.get(20) != prior_epoch || c.get(22) != prior_elapsed || c.meta[3] != prior_error)) {
      error = c.meta[3]; board = -1; return true;
    }
    return false;
  }
};
}  // namespace esphome::ultra_line
