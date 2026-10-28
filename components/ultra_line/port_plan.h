#pragma once

// The plan of one "update all receivers on this port" job (protocol docs/update.md, Port job steps 2 and 3): from the
// port's ledger (port_inventory.h) and the channel's signed manifest (fw_manifest.h), which image each node gets, with
// identical images deduplicated into one artifact, queued family by family (V407, then V203), and a concrete reason for
// every node that gets none. Automatic delivery needs a complete trusted target and timing inventory: one node without
// them blocks the whole port, with that node's reason. The boards still check every image themselves; this plan never
// authorizes more than their own checks. No ESP-IDF / ESPHome dependencies: tests/port_update_test.cpp.

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "fw_manifest.h"
#include "fw_version.h"
#include "port_inventory.h"
#include "ultra_line_rate.h"

namespace esphome::ultra_line {

/// The MCU families in job order
static constexpr uint8_t PORT_FAMILY_ORDER[2] = {UPD_MCU_V407, UPD_MCU_V203};

/// What the job does for a node
enum PortAction : uint8_t {
  PA_SEND = 0,     // its image is broadcast; the job can verify it, or only send it (expect)
  PA_CURRENT,      // the release's image is what it runs already: none for it
  PA_SKIP,         // no image for it (reason)
  PA_UNREACHABLE,  // it should answer and can't: never counted as delivered (its target may still be trusted)
  PA_NONE,         // no trusted target: nothing can be decided (and the port is blocked)
};
/// Why (one per node)
enum PortReason : uint8_t {
  PRS_NONE = 0,
  PRS_NO_TARGET,         // the ledger has no trusted target (PortNode::issue says why)
  PRS_MISMATCH,          // its live identity contradicts the provisioning
  PRS_FAMILY,            // its MCU has no update family
  PRS_MISSING_ARTIFACT,  // no image for its board / MCU / package / layout / ABI / variant
  PRS_REVISION,          // ... none for its PCB revision
  PRS_FEATURES,          // ... none whose required features it has
  PRS_BOOTLOADER,        // ... none for its bootloader version (need_boot)
  PRS_GEOMETRY,          // ... none for its slot geometry
  PRS_AMBIGUOUS,         // two different images match it
  PRS_NEWER,             // it runs a newer version than the release
  PRS_NO_LAYOUT,         // an image for it, but no trusted metadata / bootloader layout
  PRS_NO_BLACKOUT,       // no trusted entry / reset forwarding blackout bound
  PRS_TOO_MANY,          // more boards than the port type allows (block_position: the first one too many)
  PRS_PORT_TYPE,         // the port's type is not known: no board limit applies
  PRS_SILENT,            // a receiver that should answer doesn't: "update all" refuses (each one named)
  PRS_ANSWERS,           // recovery: the board answers (nothing to recover: Install updates it)
  PRS_NO_RETURN,         // recovery: it has no return path (only a known receiver can be recovered)
  PRS_NOT_OLDER,         // recovery: its known version is the release's or newer
  PRS_OTHER_TARGET,      // recovery: another board type than the one recovered (not part of it)
};
/// The best result the job can establish for a node it sends to (docs/update.md, Job results); PE_RECOVERY: the
/// silent board of a recovery, sent blind from its trusted target (verified if it is found again, else "recovery sent")
enum PortExpect : uint8_t { PE_NONE = 0, PE_VERIFIED, PE_SENT_UNVERIFIED, PE_RECOVERY };
static_assert(PortIdentityStore::NODES == PORT_DIFF_POWER_MAX_BOARDS, "the identity store holds a port's receivers");

struct PortPlanNode {
  uint16_t position{0};
  PortAction action{PA_NONE};
  PortReason reason{PRS_NO_TARGET};
  PortExpect expect{PE_NONE};
  int artifact{-1};          // the release's image for it: index into PortPlan::artifacts (-1: none)
  bool before_known{false};
  uint32_t before{0};        // its version before the job (a one-way node: not known)
  uint32_t expected{0};      // the version of the release's image for it (0: none)
  uint8_t need_boot{0};      // PRS_BOOTLOADER: the bootloader version the closest image needs
  PortReason blocks{PRS_NONE};  // why it blocks the port's automatic delivery (NO_TARGET, MISMATCH, NO_LAYOUT,
                                // NO_BLACKOUT; NONE: it doesn't)
  // the trusted facts the job uses: the provisioning first, else the signed manifest's layout for its target
  bool layout_known{false};
  uint32_t metadata_base{0}, metadata_bytes{0}, bootloader_bytes{0};
  bool blackout_known{false};
  uint32_t blackout_ms{0};
  uint16_t gap_us{0};        // its largest processing gap: LINK, or a larger trusted boot-state one
};

/// One image of the release for this port, deduplicated: the nodes it is sent for, and every node it is the image of
/// (the sender job judges current and newer nodes against it too; only images with nodes to send to are sent)
struct PortArtifact {
  FwImageInfo image;
  uint8_t family{0};
  std::vector<uint16_t> positions;   // PA_SEND
  std::vector<uint16_t> matched;     // every node whose image it is
};

struct PortPlan {
  int recover{-1};                      // a recovery: the silent board's position (-1: "update all")
  std::vector<PortPlanNode> nodes;      // every position of the port, in order
  std::vector<PortArtifact> artifacts;  // job order: the V407 family's images, then the V203 family's
  uint32_t version{0};                  // the release
  bool blocked{true};                   // automatic delivery refused (block_position's reason)
  int block_position{-1};
  PortReason block_reason{PRS_NO_TARGET};
  uint32_t entry_ms{0};                 // whole-port entry wait: the largest entry bound + the largest blackout
  uint16_t gap_us{500};                 // whole-port processing gap (>= 500 us)
  uint8_t port_type{PT_UNKNOWN};
  uint16_t max_boards{0};               // the port type's board limit

  size_t count(PortAction a) const {
    size_t n = 0;
    for (const auto &p : this->nodes)
      n += p.action == a;
    return n;
  }
  bool deliverable() const {
    if (this->blocked)
      return false;
    for (const auto &a : this->artifacts)
      if (!a.positions.empty())
        return true;
    return false;
  }
};

inline PortReason port_reason_from_miss(FwMiss miss) {
  switch (miss) {
    case FWM_REVISION: return PRS_REVISION;
    case FWM_FEATURES: return PRS_FEATURES;
    case FWM_BOOTLOADER: return PRS_BOOTLOADER;
    case FWM_GEOMETRY: return PRS_GEOMETRY;
    case FWM_AMBIGUOUS: return PRS_AMBIGUOUS;
    default: return PRS_MISSING_ARTIFACT;
  }
}

inline bool port_same_image_(const FwImageInfo &a, const FwImageInfo &b) {
  return a.size == b.size && a.sha256.size() == b.sha256.size() &&
         std::equal(a.sha256.begin(), a.sha256.end(), b.sha256.begin(),
                    [](char x, char y) { return (x | 0x20) == (y | 0x20); });
}

/// The plan for a port of this type: "update all receivers on this port" (recover < 0), or the recovery of one silent
/// known board (recover: its position). A node on the release's version is current, one on a newer version skipped:
/// the boards refuse an equal or older version unless the ANNOUNCE carries FORCE, which the sender never sets, and the
/// port update has no force either. "Update all" waits until every receiver that should answer does: a silent one
/// blocks it (something is wrong: power, cable, a board left in its recovery bootloader). A recovery sends only the
/// silent board's image, blind; another silent board doesn't block it (the sender's ultra_update_job_recover).
inline PortPlan port_plan_(const PortInventory &inv, const FwManifest &m, uint8_t port_type, int recover) {
  PortPlan plan;
  plan.recover = recover;
  plan.version = m.version_word;
  plan.port_type = port_type;
  plan.max_boards = port_max_boards(port_type);
  plan.nodes.resize(inv.nodes.size());
  uint32_t entry = 0, blackout = 0;
  uint16_t gap = 500;
  for (size_t k = 0; k < inv.nodes.size(); k++) {
    const PortNode &n = inv.nodes[k];
    PortPlanNode &p = plan.nodes[k];
    p.position = n.position;
    p.before_known = n.firmware_known;
    p.before = n.firmware;
    if (n.state == PNS_MISMATCH || !n.target_known) {
      p.action = PA_NONE;
      p.reason = n.state == PNS_MISMATCH ? PRS_MISMATCH : PRS_NO_TARGET;
      p.blocks = p.reason;
      continue;
    }
    // the trusted facts: provisioning, else the manifest's for exactly this target
    const FwLayout *l = fw_layout_for(m, n.target);
    p.layout_known = n.layout_known || l != nullptr;
    p.metadata_base = n.layout_known ? n.metadata_base : l ? l->metadata_base : 0;
    p.metadata_bytes = n.layout_known ? n.metadata_bytes : l ? l->metadata_bytes : 0;
    p.bootloader_bytes = n.layout_known ? n.bootloader_bytes : l ? l->bootloader_bytes : 0;
    p.blackout_known = n.blackout_known || l != nullptr;
    p.blackout_ms = n.blackout_known ? n.blackout_ms : l ? l->blackout_ms : 0;
    p.gap_us = std::max<uint16_t>(n.min_gap_us, l ? l->gap_us : 0);
    entry = std::max<uint32_t>(entry, uint32_t(n.entry_10ms) * 10u);
    blackout = std::max(blackout, p.blackout_ms);
    gap = std::max(gap, p.gap_us);

    const bool family = std::find(std::begin(PORT_FAMILY_ORDER), std::end(PORT_FAMILY_ORDER),
                                  n.target.bytes[UPD_T_MCU]) != std::end(PORT_FAMILY_ORDER);
    FwMiss miss = FWM_MISSING;
    const int i = family ? fw_resolve_image(m, n.target, n.boot_version, &miss, &p.need_boot) : -1;
    if (i < 0) {
      p.action = PA_SKIP;
      p.reason = family ? port_reason_from_miss(miss) : PRS_FAMILY;
    } else {
      const FwImageInfo &img = m.images[i];
      p.expected = img.version_word;
      size_t a = 0;
      while (a < plan.artifacts.size() && !port_same_image_(plan.artifacts[a].image, img))
        a++;
      if (a == plan.artifacts.size()) {
        plan.artifacts.emplace_back();
        plan.artifacts.back().image = img;
        plan.artifacts.back().family = n.target.bytes[UPD_T_MCU];
      }
      plan.artifacts[a].matched.push_back(n.position);
      p.artifact = static_cast<int>(a);
      if (n.state == PNS_UNREACHABLE && recover == static_cast<int>(n.position) && n.has_return &&
          (!n.firmware_known || n.firmware < img.version_word)) {
        // the recovery's board: it may sit revoked in its recovery bootloader, which answers no ordinary poll; its
        // image goes out blind from its trusted target (an unknown version counts as older)
        p.action = PA_SEND;
        p.reason = PRS_NONE;
        p.expect = PE_RECOVERY;
        plan.artifacts[a].positions.push_back(n.position);
        if (!p.layout_known)
          p.blocks = PRS_NO_LAYOUT;
      } else if (n.state == PNS_UNREACHABLE) {
        p.action = PA_UNREACHABLE;
        p.reason = PRS_NONE;
        if (recover == static_cast<int>(n.position))
          p.blocks = n.has_return ? PRS_NOT_OLDER : PRS_NO_RETURN;
      } else if (recover == static_cast<int>(n.position)) {
        p.action = n.firmware_known && n.firmware == img.version_word ? PA_CURRENT : PA_SKIP;
        p.reason = PRS_NONE;
        p.blocks = n.state == PNS_REACHABLE ? PRS_ANSWERS : PRS_NO_RETURN;
      } else if (n.firmware_known && n.firmware == img.version_word) {
        p.action = PA_CURRENT;
        p.reason = PRS_NONE;
      } else if (n.firmware_known && n.firmware > img.version_word) {
        p.action = PA_SKIP;
        p.reason = PRS_NEWER;
      } else {
        p.action = PA_SEND;
        p.reason = PRS_NONE;
        p.expect = n.state == PNS_REACHABLE && n.has_return && n.fw_update_page ? PE_VERIFIED : PE_SENT_UNVERIFIED;
        plan.artifacts[a].positions.push_back(n.position);
        if (!p.layout_known)
          p.blocks = PRS_NO_LAYOUT;
      }
    }
    // every node's blackout bounds the whole-port entry wait, transit nodes too
    if (!p.blackout_known && p.blocks == PRS_NONE)
      p.blocks = PRS_NO_BLACKOUT;
    // "update all": a receiver that should answer and doesn't (a known one, or one behind it) blocks it; one that just
    // restarted (its identity not read again yet) too, for a moment
    if (recover < 0 && n.state == PNS_UNREACHABLE && n.has_return && p.blocks == PRS_NONE)
      p.blocks = n.issue == PNI_IDENTITY_PENDING ? PRS_NO_TARGET : PRS_SILENT;
  }
  // a recovery: its board's image only; the nodes of other images are not part of it
  if (recover >= 0) {
    const int target = static_cast<size_t>(recover) < plan.nodes.size() ? plan.nodes[recover].artifact : -1;
    for (auto &p : plan.nodes) {
      if (p.action != PA_SEND || p.artifact == target)
        continue;
      p.action = PA_SKIP;
      p.reason = PRS_OTHER_TARGET;
      p.expect = PE_NONE;
      if (p.blocks == PRS_NO_LAYOUT)
        p.blocks = PRS_NONE;   // (nothing is sent to it)
    }
    for (int a = 0; a < static_cast<int>(plan.artifacts.size()); a++)
      if (a != target)
        plan.artifacts[a].positions.clear();
  }
  // job order: family by family (stable: in the order of their first node)
  std::vector<size_t> order(plan.artifacts.size());
  for (size_t a = 0; a < order.size(); a++)
    order[a] = a;
  auto rank = [](uint8_t family) {
    return family == PORT_FAMILY_ORDER[0] ? 0 : family == PORT_FAMILY_ORDER[1] ? 1 : 2;
  };
  std::stable_sort(order.begin(), order.end(), [&](size_t x, size_t y) {
    return rank(plan.artifacts[x].family) < rank(plan.artifacts[y].family);
  });
  std::vector<PortArtifact> sorted;
  std::vector<int> moved(order.size());
  for (size_t a = 0; a < order.size(); a++) {
    moved[order[a]] = static_cast<int>(a);
    sorted.push_back(std::move(plan.artifacts[order[a]]));
  }
  plan.artifacts = std::move(sorted);
  for (auto &p : plan.nodes)
    if (p.artifact >= 0)
      p.artifact = moved[p.artifact];

  plan.entry_ms = entry + blackout;
  plan.gap_us = gap;
  plan.blocked = false;
  for (const auto &p : plan.nodes) {
    if (p.blocks != PRS_NONE) {
      plan.blocked = true;
      plan.block_position = p.position;
      plan.block_reason = p.blocks;
      break;
    }
  }
  if (plan.max_boards == 0) {
    plan.blocked = true;
    plan.block_position = -1;
    plan.block_reason = PRS_PORT_TYPE;
  } else if (plan.nodes.size() > plan.max_boards) {
    plan.blocked = true;
    plan.block_position = plan.max_boards;
    plan.block_reason = PRS_TOO_MANY;
  }
  if (plan.nodes.empty())
    plan.blocked = true;
  if (recover >= 0 && static_cast<size_t>(recover) >= plan.nodes.size()) {
    plan.blocked = true;
    plan.block_position = recover;
    plan.block_reason = PRS_NO_TARGET;
  } else if (recover >= 0 && !plan.blocked && plan.nodes[recover].action != PA_SEND) {
    plan.blocked = true;   // (its image isn't in the release: the node's own reason)
    plan.block_position = recover;
    plan.block_reason = plan.nodes[recover].reason;
  }
  return plan;
}

/// "Update all receivers on this port"
inline PortPlan build_port_plan(const PortInventory &inv, const FwManifest &m, uint8_t port_type) {
  return port_plan_(inv, m, port_type, -1);
}
/// The recovery of one silent known board (its Recover firmware button): its image only, blind
inline PortPlan build_recover_plan(const PortInventory &inv, const FwManifest &m, uint8_t port_type, uint16_t position) {
  return port_plan_(inv, m, port_type, position);
}

/// Why the plan is blocked, naming every node that blocks it (equal reasons together: "B1, B3 not answering: check
/// power and cable"); a port-wide reason alone ("more than 8 boards on the port")
inline std::string port_blocked_text(const PortPlan &plan, const PortInventory *inv = nullptr);

inline const char *port_action_name(PortAction a) {
  switch (a) {
    case PA_SEND: return "send";
    case PA_CURRENT: return "already current";
    case PA_SKIP: return "skip";
    case PA_UNREACHABLE: return "unreachable";
    default: return "unknown";
  }
}

/// A board as the report names it: B1 = the first board after the controller (as in YAML; position: its slot, 0-based)
inline std::string port_board_text(uint16_t position) { return "B" + std::to_string(position + 1); }
/// a range of boards' numbers after the first one's name ("-3" in "B1-3")
inline std::string port_board_number(uint16_t position) { return std::to_string(position + 1); }

/// A reason as text ("" for none); value: the bootloader version needed (PRS_BOOTLOADER) or the port's board limit
/// (PRS_TOO_MANY); n: the node's ledger record for its issue
inline std::string port_reason_text(PortReason r, uint16_t value = 0, const PortNode *n = nullptr) {
  switch (r) {
    case PRS_NONE: return "";
    case PRS_NO_TARGET: return n != nullptr && n->issue != PNI_NONE ? port_node_issue_text(n->issue) : "no trusted target";
    case PRS_MISMATCH: return n != nullptr ? port_node_issue_text(n->issue) : "identity mismatch";
    case PRS_FAMILY: return "no update family for its MCU";
    case PRS_MISSING_ARTIFACT: return "no image for its board in this release";
    case PRS_REVISION: return "no image for its PCB revision";
    case PRS_FEATURES: return "no image for its bootloader features";
    case PRS_BOOTLOADER: return "needs bootloader " + std::to_string(value);
    case PRS_GEOMETRY: return "no image for its flash slot";
    case PRS_AMBIGUOUS: return "two different images match";
    case PRS_NEWER: return "runs a newer version";
    case PRS_NO_LAYOUT: return "no trusted flash layout";
    case PRS_NO_BLACKOUT: return "no trusted entry blackout";
    case PRS_TOO_MANY: return "more than " + std::to_string(value) + " boards on the port";
    case PRS_PORT_TYPE: return "the port's type is not known";
    case PRS_SILENT: return "not answering: check power and cable";
    case PRS_ANSWERS: return "answers: nothing to recover (Install updates it)";
    case PRS_NO_RETURN: return "has no return path: only a known receiver can be recovered";
    case PRS_NOT_OLDER: return "runs this release or a newer one: nothing to recover";
    case PRS_OTHER_TARGET: return "another board type: not part of this recovery";
    default: return "?";
  }
}

inline std::string port_blocked_text(const PortPlan &plan, const PortInventory *inv) {
  if (plan.block_position < 0 || plan.block_reason == PRS_TOO_MANY || plan.block_reason == PRS_PORT_TYPE)
    return port_reason_text(plan.block_reason, plan.max_boards);
  std::vector<std::pair<std::string, std::string>> groups;   // reason text -> "B1, B3"
  for (const auto &p : plan.nodes) {
    const PortReason why = plan.recover >= 0 && static_cast<int>(p.position) == plan.block_position && p.blocks == PRS_NONE
                               ? plan.block_reason
                               : p.blocks;
    if (why == PRS_NONE)
      continue;
    const PortNode *n = inv != nullptr && p.position < inv->nodes.size() ? &inv->nodes[p.position] : nullptr;
    const std::string text = plan.recover == static_cast<int>(p.position) && why == PRS_NO_TARGET
                                 ? "was never seen answering here: nothing to recover from"
                                 : port_reason_text(why, p.need_boot, n);
    auto g = std::find_if(groups.begin(), groups.end(), [&](const auto &x) { return x.first == text; });
    if (g == groups.end())
      groups.emplace_back(text, port_board_text(p.position));
    else
      g->second += ", " + port_board_text(p.position);
  }
  if (groups.empty())   // (a recovery whose board has no image: the node's own reason)
    return port_board_text(static_cast<uint16_t>(plan.block_position)) + " " + port_reason_text(plan.block_reason);
  std::string s;
  for (const auto &g : groups)
    s += (s.empty() ? "" : "; ") + g.second + " " + g.first;
  return s;
}

}  // namespace esphome::ultra_line
