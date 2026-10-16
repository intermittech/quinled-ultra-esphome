#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome::dig2analog_ultra {

/// Home Assistant keeps a state of at most 255 characters (a longer one becomes `unknown`): every text sensor's
/// state cut to 255 bytes at a character (UTF-8), "..." where it was cut
static constexpr size_t TEXT_STATE_MAX = 255;
inline std::string text_state(const std::string &s) {
  if (s.size() <= TEXT_STATE_MAX)
    return s;
  size_t cut = TEXT_STATE_MAX - 3;
  while (cut > 0 && (static_cast<uint8_t>(s[cut]) & 0xC0) == 0x80)
    cut--;
  return s.substr(0, cut) + "...";
}

}  // namespace esphome::dig2analog_ultra
