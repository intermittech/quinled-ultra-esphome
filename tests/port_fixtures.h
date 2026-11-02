#pragma once
// Synthetic release fixtures for the port update tests, never product inventory.
#include <cstring>
#include "../components/ultra_line/fw_manifest.h"
namespace esphome::ultra_line {
/// A manifest entry's signed descriptor, made from the entry itself (version: its version word)
inline void seal_descriptor(FwImageInfo &i, uint32_t version) {
  uint8_t *d = i.descriptor;
  std::memset(d, 0, UPD_DESC_BYTES);
  std::memcpy(d, "QDU6", 4);
  d[UPD_D_VERSION] = UPD_DESC_VERSION;
  d[UPD_D_REV_MIN] = i.rev_min;
  d[UPD_D_REV_MAX] = i.rev_max;
  std::memcpy(d + UPD_D_TARGET, i.target.bytes, 7);
  d[UPD_D_MIN_BL] = i.min_bootloader;
  ultra_put32(d + UPD_D_FW_VERSION, version);
  ultra_put32(d + UPD_D_BODY_LEN, i.size - UPD_DESC_BYTES);
  const uint32_t base = ultra_get32(i.target.bytes + UPD_T_SLOT_BASE);
  ultra_put32(d + UPD_D_LOAD, base);
  ultra_put32(d + UPD_D_ENTRY, base + 2);
  d[UPD_D_RECORD_FORMAT] = 2;
  d[UPD_D_WIRE] = 3;
  d[UPD_D_DESCRIPTOR_FORMAT] = 1;
  ultra_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15));
}
}  // namespace esphome::ultra_line
