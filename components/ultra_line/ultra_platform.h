#pragma once
#include "ultra_protocol.h"
namespace esphome::ultra_line {
static constexpr uint8_t HOST_LIGHTS = 5, HOST_PIXEL_SEGMENTS = 4, HOST_CONFIG_COUNT = 26;
// Cached descriptor facts; these bits have no wire representation.
static constexpr uint32_t RX_CAP_RETURN = 1, RX_CAP_PIXEL = 2, RX_CAP_PIXEL_FX = 4,
                          RX_CAP_CHAIN = 8, RX_CAP_CURRENT = 16, RX_CAP_RELAY = 32, RX_CAP_VBUS = 64;
// Scope membership from the pinned sender library (ultra_sender.c).
inline bool ultra_config_scope_ok(uint8_t target, uint8_t id) {
  if (target == ULTRA_TARGET_BOARD)
    return (id >= 1 && id <= 10) || (id >= 21 && id <= 25) || (id >= 33 && id <= 46);
  if (id == ULTRA_CFG_CH_CURRENT || id == ULTRA_CFG_CH_MAX) return target <= ULTRA_TARGET_MAX;
  return target <= ULTRA_PIXEL_OUTPUT_MAX && id >= ULTRA_CFG_PX_FIRST && id < ULTRA_CFG_PX_FIRST + ULTRA_CFG_PX_COUNT;
}
}
