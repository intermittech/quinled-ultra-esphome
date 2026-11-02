#pragma once

// The port's topology ledger for "update all receivers on this port" (protocol docs/update.md, Port job step 1): one
// record per physical position of the line, merged from the return channel's IDENTITY parts 0/1/2 and board
// descriptor (ultra_telemetry.h, read since the board's current start) and from trusted provisioning (update.py
// inventory: the jig's protected boot-info export for a position), and for a receiver that is silent now from the last
// live identity it reported (PortIdentityStore, kept on the ESP32 over restarts). Unknown, one-way and unreachable
// nodes stay explicit: silence never establishes absence or identity, and neither a hardware id nor a display name
// ever stands in for the immutable target. No ESP-IDF / ESPHome dependencies: tests/port_update_test.cpp runs it on a
// PC.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "ultra_descriptor.h"
#include "ultra_telemetry.h"
#include "ultra_update.h"

namespace esphome::ultra_line {

/// What the ledger knows of a position
enum PortNodeState : uint8_t {
  PNS_REACHABLE = 0,  // answers now; its full identity is from its current start
  PNS_ONE_WAY,        // no return path here: provisioned facts only (an update can end "sent, unverified" at best)
  PNS_UNREACHABLE,    // a receiver position that can't be inventoried now (its provisioned facts, if any, are kept)
  PNS_UNKNOWN,        // no trusted target: a one-way position without provisioning, or an invalid tuple
  PNS_MISMATCH,       // its live identity contradicts the provisioning: another board at this position?
};
/// Why a record is not complete (one per node)
enum PortNodeIssue : uint8_t {
  PNI_NONE = 0,
  PNI_NOT_PROVISIONED,   // no return path and no provisioned boot info
  PNI_SILENT,            // a receiver position that doesn't answer
  PNI_BEHIND_CUT,        // behind a receiver that keeps missing: not polled
  PNI_IDENTITY_PENDING,  // answers, but its IDENTITY parts / descriptor are not complete (yet)
  PNI_INVALID_TARGET,    // its tuple, geometry or entry bound is malformed, or not bootloader ABI 3
  PNI_UID_CHANGED,       // the live UID differs from the provisioned one
  PNI_TARGET_CHANGED,    // the live tuple differs from the provisioned boot info
  PNI_BOOT_CHANGED,      // the live boot version, geometry or entry bound differs from it
  PNI_PROVISION_CONFLICT,  // two provisioned records for the position
};

/// Trusted provisioning of one position (update.py inventory item)
struct PortProvision {
  uint16_t position{0};
  uint8_t boot_info[UPD_BL_INFO_BYTES]{};  // QDB2, boot version, target[16], geometry[3], entry_10ms, build id[7]
  bool has_uid{false};
  uint8_t uid[12]{};
  bool return_channel{false};               // the board answers (the YAML's return_channel; needs the UID)
  bool has_layout{false};
  uint32_t metadata_base{0}, metadata_bytes{0}, bootloader_bytes{0};
  bool has_blackout{false};
  uint32_t blackout_ms{0};                  // its entry / reset forwarding blackout
};

/// What the return channel says of one position now (TelemetryPoller; port_observe())
struct PortObservation {
  const ReceiverData *data{nullptr};  // nullptr: not one of the line's polled receivers
  bool answering{false};
  bool behind_cut{false};
};

/// One position of the port
struct PortNode {
  uint16_t position{0};
  PortNodeState state{PNS_UNKNOWN};
  PortNodeIssue issue{PNI_NOT_PROVISIONED};
  bool live{false};          // the identity below is the return channel's (this start of the board)
  bool provisioned{false};   // a valid provisioned boot-info export covers the position
  bool uid_known{false};
  uint8_t uid[12]{};
  bool target_known{false};  // target, boot version, geometry and entry bound are trusted
  upd_target_t target{};
  uint8_t boot_version{0}, geometry[UPD_GEOMETRY_BYTES]{}, entry_10ms{0};
  bool firmware_known{false};
  uint32_t firmware{0};      // installed version word (IDENTITY part 0); one-way: not known
  bool has_return{false};    // it can answer here (its descriptor's LINK RETURN, or the provisioning)
  bool fw_update_page{false};  // it reports its installed image (feature FW_UPDATE): an update can be verified
  bool link_known{false};    // link_flags / min_gap_us come from its descriptor
  uint8_t link_flags{0};     // ULTRA_LINK_* of its adaptive links together
  uint16_t min_gap_us{0};    // the largest adaptive LINK processing gap
  bool layout_known{false};  // trusted metadata / bootloader layout (provisioning; else the signed manifest's)
  uint32_t metadata_base{0}, metadata_bytes{0}, bootloader_bytes{0};
  bool blackout_known{false};
  uint32_t blackout_ms{0};
  uint32_t evidence_ms{0};   // live: when its identity was complete (millis); provisioned only: 0
  bool remembered{false};    // silent now: its target, UID and version are the last live ones (PortIdentityStore)
  bool uid_seen{false};      // a board answers here with this UID but an identity the ledger can't use (invalid
  uint8_t seen_uid[12]{};    // target): not this position's recorded board if the UIDs differ
};

/// The last live identity of each polled receiver position, kept on the ESP32 over restarts. A receiver revoked by an
/// interrupted update sits dark in its recovery bootloader, which answers no ordinary poll: from this record its
/// Recover firmware button sends it its image blind (the sender's ultra_update_job_recover).
/// Written only when a live identity differs from the record (a new board, its version after an update). Position
/// policy: a record belongs to its position; a board answering there with another UID replaces it (a reachable board
/// with an unusable identity and another UID clears it). Known limit: a board swapped in while dark, never seen
/// answering, can't be told apart from the recorded one (its recovery image is the record's; exact-target, blind,
/// verified only by fresh evidence).
struct PortIdentityStore {
  static constexpr uint32_t MAGIC = 0x30444950;   // "PID0"
  static constexpr uint8_t FORMAT = 1;
  static constexpr size_t NODES = 8;              // the receivers of a Diff-Power port (only receivers answer)
  static constexpr uint8_t F_VALID = 1, F_FW_UPDATE = 2;
  struct Node {
    uint8_t flags, boot_version, entry_10ms, geometry[UPD_GEOMETRY_BYTES];
    uint8_t uid[12], target[UPD_TARGET_BYTES];
    uint32_t firmware;   // its installed version word (IDENTITY part 0)
  };
  uint32_t magic{0};
  uint8_t format{0}, reserved[3]{};
  Node nodes[NODES]{};
};

/// The 16-byte tuple, geometry and entry bound make a well-formed ABI-3 target (docs/update.md). The MCU family is
/// not judged here: an unknown family is inventoried, and then has no image.
inline bool port_target_valid(const uint8_t *t, const uint8_t *geometry, uint8_t entry_10ms) {
  const uint32_t base = ultra_get32(t + UPD_T_SLOT_BASE), size = ultra_get32(t + UPD_T_SLOT_SIZE);
  const uint8_t erase_log2 = geometry[UPD_G_UNITS] & 15u;
  return t[UPD_T_BOARD] != 0 && t[UPD_T_BOARD] != 255 && t[UPD_T_ABI] == UPD_BOOT_ABI &&
         !(t[UPD_T_FEATURES] & ~UPD_FEATURE_MASK) && size != 0 && size <= 0xFFFFFFFFu - base && !(base & 255u) &&
         (geometry[UPD_G_UNITS] >> 4) == 8 && erase_log2 >= 8 && geometry[UPD_G_PROGRAM_MS] &&
         geometry[UPD_G_ERASE_MS] && entry_10ms >= 5;
}

/// A provisioned boot-info export (QDB2, ABI 3, a valid target)
inline bool port_boot_info_valid(const uint8_t *b) {
  return !std::memcmp(b, "QDB2", 4) && b[4] != 0 && port_target_valid(b + 5, b + 21, b[24]);
}

/// The adaptive links of a descriptor: their flags together and the largest gap. false: it doesn't decode.
inline bool port_link_facts(const std::vector<uint8_t> &bytes, uint8_t *flags, uint16_t *gap_us) {
  ultra_descriptor_t desc{};
  if (bytes.empty() || bytes.size() > 0xFFFF ||
      ultra_descriptor_decode(&desc, bytes.data(), static_cast<uint16_t>(bytes.size())))
    return false;
  *flags = 0;
  *gap_us = 0;
  ultra_desc_link_t link{};
  for (uint8_t port = 0; port < 16; port++) {
    if (ultra_descriptor_link(&desc, port, &link) || link.phy != ULTRA_PHY_ADAPTIVE)
      continue;
    *flags |= link.flags;
    if (link.gap_us > *gap_us)
      *gap_us = link.gap_us;
  }
  return true;
}

/// The ledger of one port: every position 0..positions-1, in order
struct PortInventory {
  std::vector<PortNode> nodes;
  uint32_t snapshot_ms{0};

  size_t count(PortNodeState s) const {
    size_t n = 0;
    for (const auto &node : this->nodes)
      n += node.state == s;
    return n;
  }
  /// Every node's immutable target and entry bound are trusted (reachable, one-way, or unreachable but provisioned):
  /// the whole-port waits can be bounded. Layout / blackout facts are the plan's to complete (port_plan.h).
  bool targets_complete() const {
    for (const auto &node : this->nodes)
      if (!node.target_known || node.state == PNS_MISMATCH)
        return false;
    return !this->nodes.empty();
  }
};

/// The line's polled receivers as observations (positions beyond them: none)
inline std::vector<PortObservation> port_observe(const TelemetryPoller &p, uint32_t now_ms) {
  std::vector<PortObservation> out(p.receivers());
  const int cut = p.cut_at(now_ms);
  for (uint16_t i = 0; i < p.receivers(); i++) {
    out[i].data = &p.data(i);
    out[i].answering = p.answering(i);
    out[i].behind_cut = cut >= 0 && i > cut;
  }
  return out;
}

inline void port_take_provision_(PortNode &n, const PortProvision &p) {
  const uint8_t *b = p.boot_info;
  n.provisioned = true;
  n.target_known = true;
  std::memcpy(n.target.bytes, b + 5, UPD_TARGET_BYTES);
  n.boot_version = b[4];
  std::memcpy(n.geometry, b + 21, UPD_GEOMETRY_BYTES);
  n.entry_10ms = b[24];
  if (p.has_uid) {
    n.uid_known = true;
    std::memcpy(n.uid, p.uid, 12);
  }
}

/// The record of a position, if it holds a well-formed target
inline const PortIdentityStore::Node *port_remembered_(const PortIdentityStore *s, uint16_t pos) {
  if (s == nullptr || s->magic != PortIdentityStore::MAGIC || s->format != PortIdentityStore::FORMAT ||
      pos >= PortIdentityStore::NODES)
    return nullptr;
  const PortIdentityStore::Node &r = s->nodes[pos];
  return (r.flags & PortIdentityStore::F_VALID) && port_target_valid(r.target, r.geometry, r.entry_10ms) ? &r : nullptr;
}

/// The ledger: positions 0..positions-1 from the provisioning (any subset of them), the observations (index =
/// position; the line's polled receivers come first) and, for a polled receiver that is silent now, its last live
/// identity (mem; it never overrides a provisioned target it contradicts). Pure: the caller snapshots the poller
/// (port_observe()).
inline PortInventory build_port_inventory(uint16_t positions, const PortProvision *prov, size_t nprov,
                                          const PortObservation *obs, size_t nobs, uint32_t now_ms,
                                          const PortIdentityStore *mem = nullptr) {
  PortInventory inv;
  inv.snapshot_ms = now_ms;
  inv.nodes.resize(positions);
  for (uint16_t pos = 0; pos < positions; pos++) {
    PortNode &n = inv.nodes[pos];
    n.position = pos;
    const PortProvision *p = nullptr;
    bool conflict = false, malformed = false;
    for (size_t k = 0; k < nprov; k++) {
      if (prov[k].position != pos)
        continue;
      if (p != nullptr) {
        conflict = true;
      } else if (port_boot_info_valid(prov[k].boot_info)) {
        p = &prov[k];
      } else {
        malformed = true;
      }
    }
    if (conflict || (malformed && p != nullptr)) {
      n.state = PNS_MISMATCH;
      n.issue = PNI_PROVISION_CONFLICT;
      continue;
    }
    if (p != nullptr) {
      port_take_provision_(n, *p);
      n.has_return = p->return_channel && p->has_uid;
      n.layout_known = p->has_layout;
      n.metadata_base = p->metadata_base;
      n.metadata_bytes = p->metadata_bytes;
      n.bootloader_bytes = p->bootloader_bytes;
      n.blackout_known = p->has_blackout;
      n.blackout_ms = p->blackout_ms;
    }
    const PortObservation *o = pos < nobs && obs[pos].data != nullptr ? &obs[pos] : nullptr;
    if (o == nullptr) {
      // not polled from this hub: one-way here, whatever the board could do elsewhere
      n.has_return = false;
      n.state = p != nullptr ? PNS_ONE_WAY : PNS_UNKNOWN;
      n.issue = p != nullptr ? PNI_NONE : malformed ? PNI_INVALID_TARGET : PNI_NOT_PROVISIONED;
      continue;
    }
    const ReceiverData &d = *o->data;
    if (o->behind_cut || !o->answering || !d.have_identity) {
      // a receiver position that can't be inventoried now: unreachable (its target only if provisioned)
      n.state = PNS_UNREACHABLE;
      n.issue = o->behind_cut ? PNI_BEHIND_CUT : !o->answering ? PNI_SILENT : PNI_IDENTITY_PENDING;
      if (p != nullptr)
        n.has_return = true;   // a polled receiver position
      const PortIdentityStore::Node *r = port_remembered_(mem, pos);
      if (r != nullptr && (p == nullptr || (!std::memcmp(p->boot_info + 5, r->target, UPD_TARGET_BYTES) &&
                                            (!p->has_uid || !std::memcmp(p->uid, r->uid, 12))))) {
        if (p == nullptr) {
          n.target_known = true;
          std::memcpy(n.target.bytes, r->target, UPD_TARGET_BYTES);
          n.boot_version = r->boot_version;
          std::memcpy(n.geometry, r->geometry, UPD_GEOMETRY_BYTES);
          n.entry_10ms = r->entry_10ms;
          n.uid_known = true;
          std::memcpy(n.uid, r->uid, 12);
        }
        n.remembered = true;
        n.has_return = true;
        n.firmware_known = true;
        n.firmware = r->firmware;
        n.fw_update_page = (r->flags & PortIdentityStore::F_FW_UPDATE) != 0;
      }
      continue;
    }
    if (!port_target_valid(d.target, d.geometry, d.entry_10ms)) {
      n = PortNode{};
      n.position = pos;
      n.state = PNS_UNKNOWN;
      n.issue = PNI_INVALID_TARGET;
      n.uid_seen = true;
      std::memcpy(n.seen_uid, d.uid, 12);
      continue;
    }
    if (p != nullptr) {
      const uint8_t *b = p->boot_info;
      PortNodeIssue why = PNI_NONE;
      if (p->has_uid && std::memcmp(p->uid, d.uid, 12))
        why = PNI_UID_CHANGED;
      else if (std::memcmp(b + 5, d.target, UPD_TARGET_BYTES))
        why = PNI_TARGET_CHANGED;
      else if (b[4] != d.bl_version || std::memcmp(b + 21, d.geometry, UPD_GEOMETRY_BYTES) || b[24] != d.entry_10ms)
        why = PNI_BOOT_CHANGED;
      if (why != PNI_NONE) {
        n.state = PNS_MISMATCH;
        n.issue = why;
        continue;
      }
    }
    n.live = true;
    n.target_known = true;
    std::memcpy(n.target.bytes, d.target, UPD_TARGET_BYTES);
    n.boot_version = d.bl_version;
    std::memcpy(n.geometry, d.geometry, UPD_GEOMETRY_BYTES);
    n.entry_10ms = d.entry_10ms;
    n.uid_known = true;
    std::memcpy(n.uid, d.uid, 12);
    n.firmware_known = true;
    n.firmware = d.fw_word;
    n.has_return = (d.hw_caps & RX_CAP_RETURN) != 0;
    n.fw_update_page = (d.features & ULTRA_FEAT_FW_UPDATE) != 0;
    n.link_known = port_link_facts(d.descriptor_bytes, &n.link_flags, &n.min_gap_us);
    n.evidence_ms = d.identity_ms;
    n.state = PNS_REACHABLE;
    n.issue = PNI_NONE;
  }
  return inv;
}

/// The live identities into the store (reachable receivers; the others keep their record). A board answering with
/// another UID than its position's record replaces the record (with an unusable identity: the record is cleared);
/// replaced: a bit per such position (to log). true: it changed (save it).
inline bool port_remember(PortIdentityStore &s, const PortInventory &inv, uint8_t *replaced = nullptr) {
  bool changed = false;
  if (s.magic != PortIdentityStore::MAGIC || s.format != PortIdentityStore::FORMAT) {
    s = PortIdentityStore{};
    s.magic = PortIdentityStore::MAGIC;
    s.format = PortIdentityStore::FORMAT;
    changed = true;
  }
  if (replaced != nullptr)
    *replaced = 0;
  for (const auto &n : inv.nodes) {
    if (n.position >= PortIdentityStore::NODES)
      continue;
    PortIdentityStore::Node &old = s.nodes[n.position];
    const bool recorded = (old.flags & PortIdentityStore::F_VALID) != 0;
    if (n.uid_seen && recorded && std::memcmp(n.seen_uid, old.uid, 12)) {
      old = PortIdentityStore::Node{};   // another board, its identity unusable: the record vouches for nobody
      changed = true;
      if (replaced != nullptr)
        *replaced |= static_cast<uint8_t>(1u << n.position);
      continue;
    }
    if (n.state != PNS_REACHABLE || !n.live)
      continue;
    if (recorded && std::memcmp(n.uid, old.uid, 12) && replaced != nullptr)
      *replaced |= static_cast<uint8_t>(1u << n.position);
    PortIdentityStore::Node r{};
    r.flags = PortIdentityStore::F_VALID | (n.fw_update_page ? PortIdentityStore::F_FW_UPDATE : 0);
    r.boot_version = n.boot_version;
    r.entry_10ms = n.entry_10ms;
    std::memcpy(r.geometry, n.geometry, UPD_GEOMETRY_BYTES);
    std::memcpy(r.uid, n.uid, 12);
    std::memcpy(r.target, n.target.bytes, UPD_TARGET_BYTES);
    r.firmware = n.firmware;
    if (std::memcmp(&r, &s.nodes[n.position], sizeof(r))) {
      s.nodes[n.position] = r;
      changed = true;
    }
  }
  return changed;
}

inline const char *port_node_state_name(PortNodeState s) {
  switch (s) {
    case PNS_REACHABLE: return "reachable";
    case PNS_ONE_WAY: return "one-way";
    case PNS_UNREACHABLE: return "unreachable";
    case PNS_MISMATCH: return "mismatch";
    default: return "unknown";
  }
}

inline const char *port_node_issue_text(PortNodeIssue i) {
  switch (i) {
    case PNI_NONE: return "";
    case PNI_NOT_PROVISIONED: return "no return path and not provisioned";
    case PNI_SILENT: return "does not answer";
    case PNI_BEHIND_CUT: return "behind a receiver that does not answer";
    case PNI_IDENTITY_PENDING: return "identity not read yet";
    case PNI_INVALID_TARGET: return "invalid immutable target";
    case PNI_UID_CHANGED: return "another UID than provisioned";
    case PNI_TARGET_CHANGED: return "another target than provisioned";
    case PNI_BOOT_CHANGED: return "another bootloader than provisioned";
    case PNI_PROVISION_CONFLICT: return "conflicting provisioning";
    default: return "?";
  }
}

}  // namespace esphome::ultra_line
