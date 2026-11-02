#pragma once
// The channel's signed release manifest (ABI 3): its images by exact immutable target, the trusted layout facts per
// target, and the checks of a downloaded image. Which node gets which image: port_plan.h.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include "fw_signature.h"
#include "fw_version.h"
#include "ultra_update.h"
namespace esphome::ultra_line {
inline const char *fw_channel_name(uint8_t ch) {
  return ch == FW_CH_STABLE ? "stable" : ch == FW_CH_BETA ? "beta" : ch == FW_CH_ALPHA ? "alpha" : nullptr;
}
inline uint8_t fw_channel_from_name(const std::string &s) {
  return s == "stable" ? FW_CH_STABLE : s == "beta" ? FW_CH_BETA : s == "alpha" ? FW_CH_ALPHA : 0;
}
inline std::string fw_manifest_url(const std::string &source, uint8_t channel) {
  const char *name = fw_channel_name(channel); if (!name) return "";
  if (source.find("://") == std::string::npos)
    return "https://raw.githubusercontent.com/" + source + "/channels/" + name + ".json";
  std::string url = source; const size_t at = url.find("{channel}");
  if (at != std::string::npos) url.replace(at, 9, name);
  return url;
}
inline std::string fw_resolve_url(const std::string &manifest, const std::string &url) {
  if (url.find("://") != std::string::npos || manifest.empty()) return url;
  if (!url.empty() && url[0] == '/') {
    const size_t scheme = manifest.find("://");
    return manifest.substr(0, manifest.find('/', scheme == std::string::npos ? 0 : scheme + 3)) + url;
  }
  return manifest.substr(0, manifest.rfind('/') + 1) + url;
}
/// One image of a release: the exact immutable target it is for (board, MCU, package, layout, ABI, required features,
/// variant, slot geometry; byte 7 unused: the revision range), its revision range, minimum bootloader, version and hash.
struct FwImageInfo {
  upd_target_t target{};
  uint8_t rev_min{0}, rev_max{0}, min_bootloader{0};
  uint32_t size{0}; std::string sha256, url;
  std::string version;        // its firmware version ("": the manifest's)
  uint32_t version_word{0};   // set by fw_check_manifest()
  /// the image's 64-byte descriptor, signed with the manifest: the sender job takes every artifact's descriptor
  /// before any body is downloaded
  uint8_t descriptor[UPD_DESC_BYTES]{};
};
/// Trusted firmware-owned facts of one exact immutable target (all 16 bytes): its metadata / bootloader layout and
/// entry / reset forwarding blackout, for nodes the YAML inventory doesn't provision (the signed manifest's layouts).
struct FwLayout {
  upd_target_t target{};
  uint32_t metadata_base{0}, metadata_bytes{0}, bootloader_bytes{0}, blackout_ms{0};
  uint16_t gap_us{0};         // a boot-state processing gap larger than its LINK's (0: none)
};
struct FwManifest {
  std::string name, channel, version, release_url, summary;
  uint32_t version_word{0}; std::vector<FwImageInfo> images;
  std::vector<FwLayout> layouts;
};
inline bool fw_hex_to_bytes(const std::string &hex, uint8_t *out, size_t n) {
  if (!out || hex.size() != 2 * n) return false;
  for (size_t k = 0; k < 2 * n; k++) {
    const char c = hex[k];
    const int v = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 :
                  c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
    if (v < 0) return false;
    out[k / 2] = static_cast<uint8_t>((k & 1) ? (out[k / 2] | v) : (v << 4));
  } return true;
}
/// The signed descriptor is the entry's image: its CRC, target, revision range, minimum bootloader, version, body
/// length and load address
inline bool fw_descriptor_matches(const FwImageInfo &i) {
  const uint8_t *d = i.descriptor;
  return !memcmp(d, "QDU6", 4) && d[UPD_D_VERSION] == UPD_DESC_VERSION &&
         upd_crc32_words(0xFFFFFFFFu, d, 15) == ultra_get32(d + UPD_D_CRC) && !memcmp(d + UPD_D_TARGET, i.target.bytes, 7) &&
         d[UPD_D_REV_MIN] == i.rev_min && d[UPD_D_REV_MAX] == i.rev_max && d[UPD_D_MIN_BL] == i.min_bootloader &&
         ultra_get32(d + UPD_D_FW_VERSION) == i.version_word && ultra_get32(d + UPD_D_BODY_LEN) == i.size - UPD_DESC_BYTES &&
         ultra_get32(d + UPD_D_LOAD) == ultra_get32(i.target.bytes + UPD_T_SLOT_BASE);
}
inline const char *fw_check_manifest(FwManifest &m, uint8_t channel) {
  const char *name = fw_channel_name(channel);
  if (!name || m.channel != name) return "manifest channel mismatch";
  if (!parse_fw_version(m.version, m.version_word)) return "invalid manifest version";
  if (m.images.empty()) return "manifest has no images";
  for (auto &i : m.images) {
    uint8_t sha[32];
    if (!i.target.bytes[0] || i.target.bytes[0] == 255 || i.target.bytes[4] != UPD_BOOT_ABI ||
        (i.target.bytes[5] & ~UPD_FEATURE_MASK) || i.rev_min > i.rev_max || i.size <= 64 ||
        i.size > UPD_INDEX_REACH + 64u || ((i.size - 64u) & 3u) || i.url.empty() ||
        !fw_hex_to_bytes(i.sha256, sha, 32)) return "invalid manifest image";
    if (i.version.empty()) i.version_word = m.version_word;
    else if (!parse_fw_version(i.version, i.version_word)) return "invalid manifest image version";
    if (!fw_descriptor_matches(i)) return "manifest image descriptor differs from its entry";
  }
  // one board target and assembly variant: one image per revision (overlapping ranges would be ambiguous)
  for (size_t k = 0; k < m.images.size(); ++k)
    for (size_t j = 0; j < k; ++j) {
      const FwImageInfo &a = m.images[j], &b = m.images[k];
      if (!memcmp(a.target.bytes, b.target.bytes, 5) && a.target.bytes[6] == b.target.bytes[6] &&
          a.rev_min <= b.rev_max && b.rev_min <= a.rev_max &&
          (a.size != b.size || a.sha256.size() != b.sha256.size() ||
           !std::equal(a.sha256.begin(), a.sha256.end(), b.sha256.begin(),
                       [](char x, char y) { return (x | 0x20) == (y | 0x20); })))
        return "manifest has two images for one board target and revision";
    }
  for (size_t k = 0; k < m.layouts.size(); ++k) {
    const auto &l = m.layouts[k];
    const uint8_t *t = l.target.bytes;
    if (!t[0] || t[0] == 255 || t[4] != UPD_BOOT_ABI || (t[5] & ~UPD_FEATURE_MASK) || l.metadata_bytes < 256 ||
        !l.bootloader_bytes || l.metadata_base < 0x08000000u + l.bootloader_bytes || l.blackout_ms > 60000u ||
        uint64_t(l.metadata_base) + l.metadata_bytes != ultra_get32(t + 8)) return "invalid manifest layout";
    for (size_t j = 0; j < k; ++j)
      if (!memcmp(m.layouts[j].target.bytes, t, 16)) return "manifest has two layouts for one target";
  } return nullptr;
}
/// The signed layout facts of exactly this immutable target (nullptr: none)
inline const FwLayout *fw_layout_for(const FwManifest &m, const upd_target_t &t) {
  for (const auto &l : m.layouts) if (!memcmp(l.target.bytes, t.bytes, 16)) return &l;
  return nullptr;
}
/// Why a node gets no image (fw_resolve_image), the closest image's first failed check
enum FwMiss : uint8_t { FWM_NONE = 0, FWM_MISSING, FWM_REVISION, FWM_FEATURES, FWM_BOOTLOADER, FWM_GEOMETRY,
                        FWM_AMBIGUOUS };
/// The release's image for one node (its trusted tuple and bootloader version): index, or -1 with miss set. Images
/// with the same SHA-256 are one artifact; two different ones that both match are ambiguous.
inline int fw_resolve_image(const FwManifest &m, const upd_target_t &t, uint8_t boot_version, FwMiss *miss,
                            uint8_t *need_boot = nullptr) {
  int found = -1;
  uint8_t closest = FWM_NONE;
  for (size_t k = 0; k < m.images.size(); ++k) {
    const FwImageInfo &i = m.images[k];
    if (memcmp(i.target.bytes, t.bytes, 5) || i.target.bytes[6] != t.bytes[6]) continue;  // another board target
    uint8_t stage = FWM_NONE;
    if (t.bytes[7] < i.rev_min || t.bytes[7] > i.rev_max) stage = FWM_REVISION;
    else if ((i.target.bytes[5] & ~t.bytes[5]) || (i.target.bytes[5] & ~UPD_FEATURE_MASK)) stage = FWM_FEATURES;
    else if (boot_version < i.min_bootloader) stage = FWM_BOOTLOADER;
    else if (memcmp(i.target.bytes + 8, t.bytes + 8, 8)) stage = FWM_GEOMETRY;
    if (stage != FWM_NONE) {
      if (stage >= closest) {
        closest = stage;
        if (stage == FWM_BOOTLOADER && need_boot) *need_boot = i.min_bootloader;
      }
      continue;
    }
    if (found >= 0 && (m.images[found].size != i.size || m.images[found].sha256.size() != i.sha256.size() ||
                       !std::equal(i.sha256.begin(), i.sha256.end(), m.images[found].sha256.begin(),
                                   [](char a, char b) { return (a | 0x20) == (b | 0x20); }))) {
      *miss = FWM_AMBIGUOUS;
      return -1;
    }
    if (found < 0) found = static_cast<int>(k);
  }
  *miss = found >= 0 ? FWM_NONE : closest != FWM_NONE ? static_cast<FwMiss>(closest) : FWM_MISSING;
  return found;
}
/// A downloaded image is the manifest's: its size and SHA-256 (the caller hashes the complete image, after the
/// manifest's signature verified), its descriptor's CRC, target, revision range, minimum bootloader, load address and
/// version, and its body CRC. The boards and the sender job check it against each node's immutable target again.
inline const char *fw_check_image(const uint8_t *img, size_t size, const uint8_t *sha, const FwImageInfo &i) {
  uint8_t want[32];
  if (img == nullptr || size != i.size || size <= UPD_DESC_BYTES) return "image size mismatch";
  if (!sha || !fw_hex_to_bytes(i.sha256, want, 32) || memcmp(want, sha, 32)) return "image SHA-256 mismatch";
  if (memcmp(img, "QDU6", 4) || img[UPD_D_VERSION] != UPD_DESC_VERSION ||
      upd_crc32_words(0xFFFFFFFFu, img, 15) != ultra_get32(img + UPD_D_CRC)) return "invalid ABI-3 descriptor";
  if (memcmp(img, i.descriptor, UPD_DESC_BYTES)) return "descriptor differs from the signed manifest's";
  const uint32_t length = ultra_get32(img + UPD_D_BODY_LEN);
  if (length != size - UPD_DESC_BYTES || (length & 3u) ||
      upd_crc32_words(0xFFFFFFFFu, img + UPD_DESC_BYTES, length / 4u) != ultra_get32(img + UPD_D_BODY_CRC))
    return "image body CRC mismatch";
  if (ultra_get32(img + UPD_D_FW_VERSION) != i.version_word || memcmp(img + UPD_D_TARGET, i.target.bytes, 7) ||
      img[UPD_D_REV_MIN] != i.rev_min || img[UPD_D_REV_MAX] != i.rev_max || img[UPD_D_MIN_BL] != i.min_bootloader ||
      ultra_get32(img + UPD_D_LOAD) != ultra_get32(i.target.bytes + UPD_T_SLOT_BASE))
    return "descriptor differs from manifest";
  return nullptr;
}
inline bool fw_update_available(uint32_t installed, uint32_t latest) { return latest && (!installed || latest > installed); }
}  // namespace esphome::ultra_line
