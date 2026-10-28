#pragma once

// Settings from Home Assistant (wiki: Controller-Settings): the rule that keeps them across reboots, and the receivers'
// board settings by name (the action set_receiver_setting, each receiver's Settings text). Pure (host-tested in
// tests/ultra_telemetry_test.cpp); the entities and the hub's side: ultra_receiver.h, ultra_line.cpp.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <strings.h>

#include "ultra_core.h"
#include "ultra_protocol.h"

namespace esphome::ultra_line {

// ----------------------------------------------------------------------------------------------- kept on the ESP32
/// A setting's stored choice: the value set in Home Assistant and the YAML default it was set against
struct SettingRecord {
  static constexpr uint32_t MAGIC = 0x53455431;   // "SET1"
  uint32_t magic{0};
  float yaml{0.0f}, value{0.0f};
};
/// The value at boot: the stored choice while the YAML default is still the one it was set against; a changed YAML
/// default wins (once: the next change in Home Assistant is stored against it)
inline float setting_boot_value(const SettingRecord &r, bool loaded, float yaml) {
  return loaded && r.magic == SettingRecord::MAGIC && r.yaml == yaml ? r.value : yaml;
}

// ---------------------------------------------------------------------------------------- the receivers' settings
/// The keys of set_receiver_setting and of the Settings text (in the text's order)
enum RxKey : uint8_t {
  RK_PWM = 0, RK_DITHER, RK_STAGGER, RK_LOSS, RK_LOSS_TIMEOUT, RK_LIMIT, RK_RETRIES, RK_RETRY_INTERVAL, RK_DTW,
  RK_RESET, RK_COUNT,
};
static constexpr const char *RX_KEYS[RK_COUNT] = {"pwm",   "dither",         "stagger", "loss", "loss_timeout",
                                                  "limit", "retries",        "retry_interval", "dtw",
                                                  "reset"};
/// what each key takes (the log's warning for a bad value)
static constexpr const char *RX_VALUES[RK_COUNT] = {
    "a preset 2.197k-100k (25k, 25 kHz) or 2197-100000 Hz",
    "on, off_at_low_levels or off",
    "off, fixed, fixed_spread, adaptive or adaptive_spread",
    "hold, off, fade_off or scene",
    "0.5s-60s",
    "0-65 A (2.0A) or none",
    "0-254 or forever",
    "1s-255s, whole seconds",
    "on or off",
    "(no value)"};
static constexpr const char *DITHER_NAMES[3] = {"on", "off_at_low_levels", "off"};
static constexpr const char *STAGGER_NAMES[5] = {"off", "fixed", "fixed_spread", "adaptive", "adaptive_spread"};
static constexpr const char *LOSS_NAMES[4] = {"hold", "off", "fade_off", "scene"};
/// the presets' labels (index = CONFIG value): kHz, as the Settings text shows them
static constexpr const char *PWM_LABELS[15] = {"2.197k", "3.107k", "4.395k", "6.215k", "8.789k",
                                               "12.43k", "17.58k", "20k",    "25k",    "30k",
                                               "35.16k", "40k",    "50k",    "70.31k", "100k"};

/// a key by name (case-insensitive); -1: none
inline int rx_key(const std::string &key) {
  for (int k = 0; k < RK_COUNT; k++) {
    if (strcasecmp(key.c_str(), RX_KEYS[k]) == 0)
      return k;
  }
  return -1;
}
/// v without spaces, in lower case
inline std::string setting_word(const std::string &v) {
  std::string s;
  for (char c : v) {
    if (c != ' ')
      s += static_cast<char>(c >= 'A' && c <= 'Z' ? c - 'A' + 'a' : c);
  }
  return s;
}
/// one of names (case-insensitive, spaces ignored): its index; -1: none
inline int setting_choice(const std::string &v, const char *const *names, int n) {
  const std::string w = setting_word(v);
  for (int i = 0; i < n; i++) {
    if (w == names[i])
      return i;
  }
  return -1;
}
/// a number with an optional unit suffix ("3s", "2.0A", "25k", "25 kHz"): its value and the rest; false: none
inline bool setting_number(const std::string &v, float *out, std::string *unit) {
  const std::string w = setting_word(v);
  if (w.empty())
    return false;
  char *end = nullptr;
  const float f = std::strtof(w.c_str(), &end);
  if (end == w.c_str() || !std::isfinite(f))
    return false;
  *out = f;
  *unit = end;
  return true;
}
/// pwm: a preset ("25k", "25 kHz", "12.43k") or a frequency in Hz (2197-100000); the board's (preset, period)
inline bool parse_pwm(const std::string &v, uint16_t *preset, uint16_t *period) {
  float f;
  std::string unit;
  if (!setting_number(v, &f, &unit))
    return false;
  if (unit == "k" || unit == "khz")
    f *= 1000.0f;
  else if (!unit.empty() && unit != "hz")
    return false;
  if (f < 2197.0f * 0.995f || f > 100000.0f * 1.005f)
    return false;
  pwm_for_frequency(f, *preset, *period);
  return true;
}
/// seconds ("3s", "3", "500ms") within [lo, hi]
inline bool parse_seconds(const std::string &v, float lo, float hi, float *s) {
  float f;
  std::string unit;
  if (!setting_number(v, &f, &unit))
    return false;
  if (unit == "ms")
    f /= 1000.0f;
  else if (unit == "min")
    f *= 60.0f;
  else if (!unit.empty() && unit != "s")
    return false;
  if (f < lo || f > hi)
    return false;
  *s = f;
  return true;
}
/// the user current limit: "2.0A", "2", "none" / "0" (no limit); mA
inline bool parse_limit(const std::string &v, uint16_t *ma) {
  if (setting_word(v) == "none") {
    *ma = 0;
    return true;
  }
  float f;
  std::string unit;
  if (!setting_number(v, &f, &unit) || (!unit.empty() && unit != "a") || f < 0.0f || f > 65.0f)
    return false;
  *ma = static_cast<uint16_t>(f * 1000.0f + 0.5f);
  return true;
}
/// trip retries: 0-254, "forever" (255)
inline bool parse_retries(const std::string &v, uint8_t *count) {
  if (setting_word(v) == "forever") {
    *count = 255;
    return true;
  }
  float f;
  std::string unit;
  if (!setting_number(v, &f, &unit) || !unit.empty() || f < 0.0f || f > 254.0f || f != std::floor(f))
    return false;
  *count = static_cast<uint8_t>(f);
  return true;
}
/// on / off (true / false, 1 / 0)
inline bool parse_on_off(const std::string &v, bool *on) {
  const std::string w = setting_word(v);
  if (w == "on" || w == "true" || w == "1") {
    *on = true;
    return true;
  }
  if (w == "off" || w == "false" || w == "0") {
    *on = false;
    return true;
  }
  return false;
}

/// The CONFIG ids a receiver's settings change (RxOverride::cfg's order)
static constexpr uint8_t RX_CFG[8] = {ULTRA_CFG_PWM_PRESET, ULTRA_CFG_PWM_PERIOD, ULTRA_CFG_DITHER,
                                      ULTRA_CFG_STAGGER,    ULTRA_CFG_LOSS,       ULTRA_CFG_LOSS_TIMEOUT,
                                      ULTRA_CFG_BOARD_MAX,  ULTRA_CFG_RETRY};
enum : uint8_t { RX_PWM_PRESET = 0, RX_PWM_PERIOD, RX_DITHER, RX_STAGGER, RX_LOSS, RX_LOSS_TIMEOUT, RX_LIMIT, RX_RETRY };
/// A board's settings from Home Assistant, kept on the ESP32 (a line's blob: one per board, 22 bytes): the CONFIG
/// values set (mask bit k: cfg[k], RX_CFG[k]) and per light index n (bit n) dim to warm set / on.
struct RxOverride {
  uint16_t cfg[8]{};
  uint8_t mask{0}, dtw_set{0}, dtw_on{0};
  uint8_t pad[2]{};
};
/// A board setting (RK_PWM .. RK_RETRY_INTERVAL) into o's CONFIG values; retry: the RETRY value in effect now (count |
/// interval << 8; the other half of it stays). false: a bad value (o unchanged)
inline bool rx_override_cfg(RxOverride &o, int key, const std::string &value, uint16_t retry) {
  switch (key) {
    case RK_PWM: {
      uint16_t preset, period;
      if (!parse_pwm(value, &preset, &period))
        return false;
      o.cfg[RX_PWM_PRESET] = preset;
      o.cfg[RX_PWM_PERIOD] = period;
      o.mask |= 1u << RX_PWM_PRESET | 1u << RX_PWM_PERIOD;
      return true;
    }
    case RK_DITHER:
    case RK_STAGGER:
    case RK_LOSS: {
      int i = key == RK_DITHER    ? setting_choice(value, DITHER_NAMES, 3)
              : key == RK_STAGGER ? setting_choice(value, STAGGER_NAMES, 5)
                                  : setting_choice(value, LOSS_NAMES, 4);
      if (key == RK_STAGGER && i < 0 && setting_word(value) == "spread")
        i = 4;   // (as in the YAML)
      if (i < 0)
        return false;
      const uint8_t k = key == RK_DITHER ? RX_DITHER : key == RK_STAGGER ? RX_STAGGER : RX_LOSS;
      o.cfg[k] = static_cast<uint16_t>(i);
      o.mask |= 1u << k;
      return true;
    }
    case RK_LOSS_TIMEOUT: {
      float s;
      if (!parse_seconds(value, 0.5f, 60.0f, &s))
        return false;
      o.cfg[RX_LOSS_TIMEOUT] = static_cast<uint16_t>(s * 10.0f + 0.5f);
      o.mask |= 1u << RX_LOSS_TIMEOUT;
      return true;
    }
    case RK_LIMIT: {
      uint16_t ma;
      if (!parse_limit(value, &ma))
        return false;
      o.cfg[RX_LIMIT] = ma;
      o.mask |= 1u << RX_LIMIT;
      return true;
    }
    case RK_RETRIES:
    case RK_RETRY_INTERVAL: {
      const uint16_t base = (o.mask & (1u << RX_RETRY)) ? o.cfg[RX_RETRY] : retry;
      uint16_t count = base & 0xFF, iv = base >> 8;
      if (key == RK_RETRIES) {
        uint8_t c;
        if (!parse_retries(value, &c))
          return false;
        count = c;
      } else {
        float s;
        if (!parse_seconds(value, 1.0f, 255.0f, &s) || s != std::floor(s))
          return false;
        iv = static_cast<uint16_t>(s);
      }
      o.cfg[RX_RETRY] = static_cast<uint16_t>(count | (iv == 0 ? 1u : iv) << 8);
      o.mask |= 1u << RX_RETRY;
      return true;
    }
    default:
      return false;
  }
}

/// "25k" (a preset), else the period's frequency in Hz ("31250")
inline std::string pwm_label(uint16_t preset, uint16_t period) {
  if (period == 0)
    return preset < 15 ? PWM_LABELS[preset] : "25k";
  char b[16];
  snprintf(b, sizeof(b), "%u", static_cast<unsigned>((PWM_PERIOD_UNITS_HZ + period / 2) / period));
  return b;
}
/// "3s", "0.5s", "2.5s"
inline std::string seconds_label(float s) {
  char b[16];
  if (std::fabs(s - std::round(s)) < 0.05f)
    snprintf(b, sizeof(b), "%ds", static_cast<int>(std::lround(s)));
  else
    snprintf(b, sizeof(b), "%.1fs", s);
  return b;
}

/// A board's settings as the Settings text shows them: its CONFIG values (have: the hub sends that id), then per light
/// dim to warm ("on", "off", "n/a") for each configured light
struct RxSettingsView {
  uint16_t cfg[HOST_CONFIG_COUNT]{};
  uint32_t have{0};   // bit 1 << id
  std::string dtw;   // per light, joined by '|' ("" without lights)
};
/// "pwm=25k dither=on stagger=adaptive_spread loss=hold loss_timeout=3s limit=2.0A retries=10 retry_interval=1s
/// dtw=on" (within 255 bytes: long text is truncated)
inline std::string rx_settings_text(const RxSettingsView &v) {
  auto has = [&v](uint8_t id) { return (v.have & (1u << id)) != 0; };
  std::string s = "pwm=" + pwm_label(v.cfg[ULTRA_CFG_PWM_PRESET], has(ULTRA_CFG_PWM_PERIOD) ? v.cfg[ULTRA_CFG_PWM_PERIOD] : 0);
  const uint16_t d = v.cfg[ULTRA_CFG_DITHER], st = v.cfg[ULTRA_CFG_STAGGER], l = v.cfg[ULTRA_CFG_LOSS];
  s += std::string(" dither=") + (d < 3 ? DITHER_NAMES[d] : "?");
  s += std::string(" stagger=") + (st < 5 ? STAGGER_NAMES[st] : "?");
  s += std::string(" loss=") + (l < 4 ? LOSS_NAMES[l] : "?");
  s += " loss_timeout=" + seconds_label(v.cfg[ULTRA_CFG_LOSS_TIMEOUT] / 10.0f);
  char b[64];   // (room for any value the compiler can see: no -Wformat-truncation)
  const uint16_t lim = v.cfg[ULTRA_CFG_BOARD_MAX];
  if (!has(ULTRA_CFG_BOARD_MAX) || lim == 0)
    s += " limit=none";
  else {
    snprintf(b, sizeof(b), lim % 100 == 0 ? " limit=%.1fA" : " limit=%.2fA", lim / 1000.0);   // ("2.0A", "2.25A")
    s += b;
  }
  if (has(ULTRA_CFG_RETRY)) {   // (not sent: the board keeps its own)
    const uint16_t r = v.cfg[ULTRA_CFG_RETRY];
    const unsigned count = r & 0xFF, iv = r >> 8;
    if (count == 255)
      snprintf(b, sizeof(b), " retries=forever retry_interval=%us", iv == 0 ? 1u : iv);
    else
      snprintf(b, sizeof(b), " retries=%u retry_interval=%us", count, iv == 0 ? 1u : iv);
    s += b;
  } else {
    s += " retries=board retry_interval=board";
  }
  if (!v.dtw.empty())
    s += " dtw=" + v.dtw;
  if (s.size() > 255) {   // (the effect names last: cut at a character)
    size_t cut = 252;
    while (cut > 0 && (static_cast<uint8_t>(s[cut]) & 0xC0) == 0x80)
      cut--;
    s = s.substr(0, cut) + "...";
  }
  return s;
}

}  // namespace esphome::ultra_line
