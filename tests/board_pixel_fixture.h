#pragma once
// A Digital-M's pixel record from the firmware's own descriptor (tests/fixtures/board_descriptors.h): its pixel limits
// and its calibration's host deadline (50 + ceil(1.5 x cal_ms))
#include "../components/ultra_line/ultra_descriptor.h"
#include "fixtures/board_descriptors.h"
#include <cstring>
namespace esphome::ultra_line {
inline ultra_desc_pixel_t fixture_pixel(const char *name) {
  ultra_desc_pixel_t px{};
  for (const auto &bd : BOARD_DESCRIPTORS) if (!std::strcmp(bd.name, name)) {
    ultra_descriptor_t d{};
    if (ultra_descriptor_decode(&d, bd.bytes, static_cast<uint16_t>(bd.size)) || ultra_descriptor_pixel(&d, 0, &px)) return {};
  }
  return px;
}
inline uint32_t fixture_cal_wait_ms(const char *name) {
  for (const auto &bd : BOARD_DESCRIPTORS) if (!std::strcmp(bd.name, name)) {
    ultra_descriptor_t d{};
    if (ultra_descriptor_decode(&d, bd.bytes, static_cast<uint16_t>(bd.size))) return 0;
    return ultra_descriptor_calibration_ms(&d, 1, 0);
  }
  return 0;
}
}  // namespace esphome::ultra_line
