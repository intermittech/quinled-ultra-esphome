// "Update all receivers on this port" on a PC (tests/run_port_update_test.py): the port's topology ledger
// (port_inventory.h) from the poller's real decode path and from provisioning; the tuple-aware manifest and the
// port's plan (fw_manifest.h, port_plan.h).

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../components/ultra_line/port_inventory.h"
#include "../components/ultra_line/port_plan.h"
#include "../components/ultra_line/port_report.h"
#include "../components/ultra_line/port_artifact.h"
#include "port_fixtures.h"
#include "wire3_descriptor_fixture.h"

using namespace esphome::ultra_line;

static int failures = 0, checks = 0;
static void check(bool ok, const char *fmt, ...) {
  checks++;
  if (ok)
    return;
  failures++;
  va_list ap;
  va_start(ap, fmt);
  std::fprintf(stderr, "FAIL: ");
  std::vfprintf(stderr, fmt, ap);
  std::fprintf(stderr, "\n");
  va_end(ap);
}

static constexpr uint32_t CAPS = RX_CAP_RETURN | RX_CAP_RELAY | RX_CAP_VBUS | RX_CAP_CURRENT;

/// The poller with its receivers' state at hand: identity reaches it through its own decode path
struct TestPoller : TelemetryPoller {
  void feed(uint16_t i, uint8_t tp, const uint8_t *data, uint32_t now) { this->decode_(this->rx_[i], tp, data, now); }
  void answering(uint16_t i, bool on) { this->rx_[i].answering = on; }
  void misses(uint16_t i, uint8_t n) { this->rx_[i].misses = n; }
  void restart(uint16_t i, uint32_t now) { this->restarted_(this->rx_[i], now, false); }
  ReceiverData &d(uint16_t i) { return this->rx_[i].d; }
  /// IDENTITY parts 0, 1, 2 and the descriptor of one fixture board (V407 tuple), the last piece at now
  void identify(uint16_t i, uint8_t board, uint8_t revision, const uint8_t *uid, uint32_t fw, uint32_t now) {
    uint8_t data[24]{};
    fixture_identity(data, 0, board, revision, uid, ULTRA_FEAT_FW_UPDATE, CAPS, fw);
    this->feed(i, TP_IDENTITY, data, now - 4);
    std::memset(data, 0, sizeof(data));
    fixture_identity(data, 1, board, revision, uid, ULTRA_FEAT_FW_UPDATE, CAPS, fw);
    this->feed(i, TP_FEATURES, data, now - 3);
    std::memset(data, 0, sizeof(data));
    fixture_identity(data, 2, board, revision, uid, ULTRA_FEAT_FW_UPDATE, CAPS, fw);
    this->feed(i, TP_TARGET, data, now - 2);
    for (uint8_t part = 0;; part++) {
      std::memset(data, 0, sizeof(data));
      if (!fixture_descriptor_part(data, part, board, CAPS))
        break;
      this->feed(i, TP_DESCRIPTOR, data, now);
    }
    this->answering(i, true);
  }
};

static void uid_of(uint8_t *uid, uint8_t seed) {
  for (uint8_t k = 0; k < 12; k++)
    uid[k] = static_cast<uint8_t>(seed * 16 + k);
}

/// A V203 boot-info export (layout 1) for a provisioned position
static PortProvision v203_provision(uint16_t position, uint8_t board = 1) {
  PortProvision p;
  p.position = position;
  const uint8_t t[16] = {board, UPD_MCU_V203, UPD_PACKAGE_F8U6, UPD_LAYOUT_V203, UPD_BOOT_ABI, 0, 0, 0x04,
                         0x08, 0x00, 0x21, 0x00, 0x00, 0x00, 0xDF, 0x00};
  std::memcpy(p.boot_info, "QDB2", 4);
  p.boot_info[4] = 3;
  std::memcpy(p.boot_info + 5, t, 16);
  p.boot_info[21] = UPD_V203_GEOMETRY;
  p.boot_info[22] = UPD_V203_PROGRAM_MS;
  p.boot_info[23] = UPD_V203_ERASE_MS;
  p.boot_info[24] = 5;
  p.has_layout = true;
  p.metadata_base = UPD_V203_METADATA;
  p.metadata_bytes = 256;
  p.bootloader_bytes = UPD_V203_BL_BYTES;
  return p;
}

/// The provisioning that matches a fixture board's live identity
static PortProvision provision_like(const TestPoller &tp, uint16_t i, uint16_t position) {
  TestPoller &p = const_cast<TestPoller &>(tp);
  PortProvision v;
  v.position = position;
  std::memcpy(v.boot_info, "QDB2", 4);
  v.boot_info[4] = p.d(i).bl_version;
  std::memcpy(v.boot_info + 5, p.d(i).target, 16);
  std::memcpy(v.boot_info + 21, p.d(i).geometry, 3);
  v.boot_info[24] = p.d(i).entry_10ms;
  v.has_uid = true;
  std::memcpy(v.uid, p.d(i).uid, 12);
  v.return_channel = true;
  return v;
}

static PortInventory ledger(const TestPoller &p, uint16_t positions, const std::vector<PortProvision> &prov,
                            uint32_t now) {
  const auto obs = port_observe(p, now);
  return build_port_inventory(positions, prov.data(), prov.size(), obs.data(), obs.size(), now);
}

static void test_inventory() {
  uint8_t uid[3][12];
  for (uint8_t k = 0; k < 3; k++)
    uid_of(uid[k], k + 1);

  // three Diff-Power receivers that answer, then two one-way positions behind them
  TestPoller p;
  p.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  for (uint16_t i = 0; i < 3; i++)
    p.identify(i, static_cast<uint8_t>(2 + i), 1, uid[i], 0x000700FF + i, 1000 + 10 * i);
  check(p.d(0).have_identity && p.d(0).identity_ms == 1000, "the poller stamps the identity when it is complete");
  std::vector<PortProvision> prov = {v203_provision(3), v203_provision(4)};
  prov[1].has_blackout = true;
  prov[1].blackout_ms = 30;
  auto inv = ledger(p, 5, prov, 5000);
  check(inv.nodes.size() == 5 && inv.count(PNS_REACHABLE) == 3 && inv.count(PNS_ONE_WAY) == 2,
        "receivers reachable, provisioned positions behind them one-way");
  check(inv.targets_complete(), "every target trusted");
  const PortNode &r1 = inv.nodes[1];
  check(r1.live && r1.uid_known && !std::memcmp(r1.uid, uid[1], 12) && r1.target.bytes[0] == 3 &&
            r1.target.bytes[UPD_T_MCU] == UPD_MCU_V407 && r1.firmware_known && r1.firmware == 0x00070100 &&
            r1.boot_version == 3 && r1.entry_10ms == 10 && r1.geometry[0] == 0x8C && r1.evidence_ms == 1010,
        "a live record carries UID, full tuple, version, boot info and evidence time");
  check(r1.has_return && r1.link_known && (r1.link_flags & ULTRA_LINK_RETURN) && r1.min_gap_us >= 500,
        "its return capability and LINK facts come from its descriptor");
  check(!r1.layout_known && !r1.blackout_known, "no trusted layout from telemetry alone");
  const PortNode &w4 = inv.nodes[4];
  check(!w4.live && w4.provisioned && w4.target_known && !w4.firmware_known && !w4.has_return &&
            w4.target.bytes[UPD_T_MCU] == UPD_MCU_V203 && w4.layout_known && w4.blackout_known && w4.blackout_ms == 30,
        "a one-way record keeps the provisioned tuple, layout and blackout; its version stays unknown");

  // a position nobody can vouch for
  prov.pop_back();
  inv = ledger(p, 5, prov, 5000);
  check(inv.nodes[4].state == PNS_UNKNOWN && inv.nodes[4].issue == PNI_NOT_PROVISIONED && !inv.targets_complete(),
        "an unprovisioned one-way position stays unknown and the targets are incomplete");

  // silence, a cut, a pending identity
  prov = {v203_provision(3), v203_provision(4)};
  p.answering(2, false);
  inv = ledger(p, 5, prov, 5000);
  check(inv.nodes[2].state == PNS_UNREACHABLE && inv.nodes[2].issue == PNI_SILENT && !inv.nodes[2].target_known &&
            !inv.targets_complete(),
        "a silent receiver is unreachable, never absent, and its target is not guessed");
  prov.push_back(provision_like(p, 2, 2));
  inv = ledger(p, 5, prov, 5000);
  check(inv.nodes[2].state == PNS_UNREACHABLE && inv.nodes[2].target_known && inv.nodes[2].has_return &&
            !inv.nodes[2].live && inv.targets_complete(),
        "a silent but provisioned receiver keeps its trusted target, still unreachable");
  prov.pop_back();
  // the ledger kept on the ESP32: the receivers' last live identities, a silent one's trusted target and version
  {
    PortIdentityStore mem;
    p.answering(2, true);
    PortInventory seen = ledger(p, 5, prov, 5000);
    check(port_remember(mem, seen) && !port_remember(mem, seen) && (mem.nodes[2].flags & PortIdentityStore::F_VALID) &&
              !(mem.nodes[3].flags & PortIdentityStore::F_VALID),
          "the store takes the live receivers once (written only when one changes), never the one-way positions");
    const PortIdentityStore saved = mem;   // (the preferences' copy over a restart)
    p.answering(2, false);
    auto obs = port_observe(p, 5000);
    inv = build_port_inventory(5, prov.data(), prov.size(), obs.data(), obs.size(), 5000, &saved);
    check(inv.nodes[2].state == PNS_UNREACHABLE && inv.nodes[2].remembered && inv.nodes[2].target_known &&
              inv.nodes[2].has_return && inv.nodes[2].firmware_known &&
              inv.nodes[2].firmware == seen.nodes[2].firmware && !inv.nodes[2].live &&
              !std::memcmp(inv.nodes[2].uid, seen.nodes[2].uid, 12) && inv.targets_complete(),
          "a silent receiver keeps its last live target, UID and version from the store, still unreachable");
    check(!port_remember(mem, inv), "a silent receiver's record stays as it was");
    // a provisioned target the record contradicts wins (the record is not used)
    std::vector<PortProvision> other = prov;
    other.push_back(v203_provision(2, 7));
    other.back().return_channel = other.back().has_uid = true;
    inv = build_port_inventory(5, other.data(), other.size(), obs.data(), obs.size(), 5000, &saved);
    check(!inv.nodes[2].remembered && inv.nodes[2].target.bytes[UPD_T_BOARD] == 7 && !inv.nodes[2].firmware_known,
          "a provisioned target the record contradicts wins over it");
    PortIdentityStore bad = saved;
    bad.format = 0;
    inv = build_port_inventory(5, prov.data(), prov.size(), obs.data(), obs.size(), 5000, &bad);
    check(!inv.nodes[2].remembered && !inv.nodes[2].target_known, "a store of another format is not used");
    // position policy: another board answering at a recorded position replaces its record, also
    // when its version is older (a stale newer record must not hide it)
    uint8_t other_uid[12];
    uid_of(other_uid, 9);
    p.identify(2, 4, 1, other_uid, 0x000700FF, 5100);
    PortIdentityStore swapped = saved;
    uint8_t replaced = 0;
    inv = ledger(p, 5, prov, 5100);
    check(port_remember(swapped, inv, &replaced) && replaced == (1u << 2) && !std::memcmp(swapped.nodes[2].uid, other_uid, 12) &&
              swapped.nodes[2].firmware == 0x000700FF && swapped.nodes[2].target[UPD_T_BOARD] == 4,
          "a board with another UID at a recorded position replaces the record (and is named: %02x)", replaced);
    check(!port_remember(swapped, inv, &replaced) && replaced == 0, "... once: the same board again changes nothing");
    p.answering(2, false);
    obs = port_observe(p, 5200);
    inv = build_port_inventory(5, prov.data(), prov.size(), obs.data(), obs.size(), 5200, &swapped);
    check(inv.nodes[2].remembered && !std::memcmp(inv.nodes[2].uid, other_uid, 12) && inv.nodes[2].firmware == 0x000700FF,
          "silent afterwards: the new board's record vouches for it (its own version)");
    // ... answering with an identity the ledger can't use (invalid target) and another UID: the record is cleared
    p.answering(2, true);
    obs = port_observe(p, 5300);
    ReceiverData broken = *obs[2].data;
    broken.target[UPD_T_ABI] ^= 0x7F;
    obs[2].data = &broken;
    PortIdentityStore cleared = saved;
    inv = build_port_inventory(5, prov.data(), prov.size(), obs.data(), obs.size(), 5300, &cleared);
    check(inv.nodes[2].issue == PNI_INVALID_TARGET && inv.nodes[2].uid_seen && port_remember(cleared, inv, &replaced) &&
              replaced == (1u << 2) && !(cleared.nodes[2].flags & PortIdentityStore::F_VALID),
          "another board with an unusable identity at a recorded position clears the record");
    p.answering(2, false);
    obs = port_observe(p, 5400);
    inv = build_port_inventory(5, prov.data(), prov.size(), obs.data(), obs.size(), 5400, &cleared);
    check(!inv.nodes[2].remembered && !inv.nodes[2].target_known, "... so nothing vouches for the old board there");
    // the known limit: a board swapped in while dark and never seen answering keeps the old record's identity
    inv = build_port_inventory(5, prov.data(), prov.size(), obs.data(), obs.size(), 5400, &saved);
    check(inv.nodes[2].remembered && !std::memcmp(inv.nodes[2].uid, uid[2], 12),
          "known limit: a never-seen dark replacement can't be told apart from the recorded board");
    p.identify(2, 4, 1, uid[2], 0x000700FF + 2, 5500);
    p.answering(2, true);
  }
  p.answering(2, true);
  p.misses(0, TelemetryPoller::CUT_MISSES);
  inv = ledger(p, 5, prov, 5000);
  check(inv.nodes[0].state == PNS_REACHABLE && inv.nodes[1].issue == PNI_BEHIND_CUT &&
            inv.nodes[2].issue == PNI_BEHIND_CUT && inv.nodes[1].state == PNS_UNREACHABLE,
        "receivers behind one that keeps missing are unreachable (behind the cut), whatever they said before");
  p.misses(0, 0);
  p.restart(1, 6000);
  inv = ledger(p, 5, prov, 6000);
  check(inv.nodes[1].state == PNS_UNREACHABLE && inv.nodes[1].issue == PNI_IDENTITY_PENDING,
        "a restart voids the identity until it is read again");
  p.identify(1, 3, 1, uid[1], 0x000701FF, 7000);
  inv = ledger(p, 5, prov, 7000);
  check(inv.nodes[1].state == PNS_REACHABLE && inv.nodes[1].evidence_ms == 7000 && inv.nodes[1].firmware == 0x000701FF,
        "read again: fresh evidence time and version");

  // provisioning that the live identity contradicts
  for (int fault = 0; fault < 3; fault++) {
    auto v = provision_like(p, 0, 0);
    if (fault == 0) v.uid[3] ^= 1;
    if (fault == 1) v.boot_info[5 + UPD_T_VARIANT] ^= 1;
    if (fault == 2) v.boot_info[23] ^= 1;
    auto pr = prov;
    pr.push_back(v);
    inv = ledger(p, 5, pr, 7000);
    const PortNodeIssue want[3] = {PNI_UID_CHANGED, PNI_TARGET_CHANGED, PNI_BOOT_CHANGED};
    check(inv.nodes[0].state == PNS_MISMATCH && inv.nodes[0].issue == want[fault] && !inv.targets_complete(),
          "a live identity that contradicts the provisioning is a mismatch (fault %d)", fault);
  }
  {
    auto pr = prov;
    pr.push_back(provision_like(p, 0, 0));
    inv = ledger(p, 5, pr, 7000);
    check(inv.nodes[0].state == PNS_REACHABLE && inv.nodes[0].provisioned && inv.nodes[0].live,
          "agreeing provisioning and identity: reachable");
    pr.push_back(v203_provision(3));
    inv = ledger(p, 5, pr, 7000);
    check(inv.nodes[3].state == PNS_MISMATCH && inv.nodes[3].issue == PNI_PROVISION_CONFLICT,
          "two provisioned records for one position conflict");
  }

  // a live tuple that is not a valid ABI-3 target
  p.d(2).target[UPD_T_ABI] = 2;
  inv = ledger(p, 5, prov, 7000);
  check(inv.nodes[2].state == PNS_UNKNOWN && inv.nodes[2].issue == PNI_INVALID_TARGET && !inv.nodes[2].target_known,
        "a live tuple outside ABI 3 is not trusted");
  p.d(2).target[UPD_T_ABI] = UPD_BOOT_ABI;

  // tuple validity, field by field
  const PortProvision good = v203_provision(0);
  check(port_boot_info_valid(good.boot_info), "the V203 export is valid");
  const int fields[] = {5 + UPD_T_BOARD, 5 + UPD_T_ABI, 5 + UPD_T_FEATURES, 5 + UPD_T_SLOT_SIZE + 2, 21, 22, 23, 24};
  const uint8_t bad[] = {0, 2, 0x10, 0, 0x78, 0, 0, 4};
  for (size_t k = 0; k < sizeof(bad); k++) {
    PortProvision v = good;
    v.boot_info[fields[k]] = bad[k];
    if (fields[k] == 5 + UPD_T_SLOT_SIZE + 2) v.boot_info[5 + UPD_T_SLOT_SIZE + 3] = 0;
    check(!port_boot_info_valid(v.boot_info), "boot info field %d = %u is refused", fields[k], bad[k]);
  }
  PortProvision v = good;
  v.boot_info[5 + UPD_T_BOARD] = 255;
  check(!port_boot_info_valid(v.boot_info), "board 255 is refused");
  std::vector<PortProvision> one = {v};
  inv = build_port_inventory(1, one.data(), one.size(), nullptr, 0, 0);
  check(inv.nodes[0].state == PNS_UNKNOWN && inv.nodes[0].issue == PNI_INVALID_TARGET,
        "malformed provisioning is not a trusted target");
}

// ------------------------------------------------------------------------------------------------ manifest and plan
static const uint8_t V203_T[16] = {1, UPD_MCU_V203, UPD_PACKAGE_F8U6, UPD_LAYOUT_V203, UPD_BOOT_ABI, 0, 0, 0x04,
                                   0x08, 0x00, 0x21, 0x00, 0x00, 0x00, 0xDF, 0x00};
static const uint8_t V407_T[16] = {4, UPD_MCU_V407, UPD_PACKAGE_RET6, UPD_LAYOUT_V407, UPD_BOOT_ABI, 3, 0, 0x01,
                                   0x08, 0x00, 0x50, 0x00, 0x00, 0x07, 0xB0, 0x00};

/// A ledger record: live (version known, answers, FW_UPDATE) or one-way (provisioned)
static PortNode node(uint16_t position, const uint8_t *tuple, uint8_t board, bool live, uint32_t fw = 0x000700FF) {
  PortNode n;
  n.position = position;
  n.state = live ? PNS_REACHABLE : PNS_ONE_WAY;
  n.issue = PNI_NONE;
  n.live = live;
  n.provisioned = !live;
  n.target_known = true;
  std::memcpy(n.target.bytes, tuple, 16);
  n.target.bytes[UPD_T_BOARD] = board;
  const bool v407 = tuple[UPD_T_MCU] == UPD_MCU_V407;
  n.boot_version = 3;
  n.geometry[0] = v407 ? UPD_V407_GEOMETRY : UPD_V203_GEOMETRY;
  n.geometry[1] = v407 ? UPD_V407_PROGRAM_MS : UPD_V203_PROGRAM_MS;
  n.geometry[2] = v407 ? UPD_V407_ERASE_MS : UPD_V203_ERASE_MS;
  n.entry_10ms = v407 ? 10 : 5;
  n.firmware_known = n.uid_known = n.has_return = n.fw_update_page = live;
  n.firmware = live ? fw : 0;
  n.uid[0] = static_cast<uint8_t>(position + 1);
  n.min_gap_us = live ? 500 : 0;
  return n;
}
static FwImageInfo image_for(const uint8_t *tuple, uint8_t board, char sha, const char *version = "") {
  FwImageInfo i;
  std::memcpy(i.target.bytes, tuple, 16);
  i.target.bytes[UPD_T_BOARD] = board;
  i.target.bytes[UPD_T_FEATURES] = 0;
  i.target.bytes[UPD_T_REVISION] = 0;
  i.rev_min = 0;
  i.rev_max = 0x0F;
  i.min_bootloader = 3;
  i.size = 64 + 1024;
  i.sha256 = std::string(64, sha);
  i.url = std::string("fw-") + sha + ".bin";
  i.version = version;
  uint32_t word = 0x000701FF;
  if (*version)
    parse_fw_version(version, word);
  seal_descriptor(i, word);
  return i;
}
static FwLayout layout_for(const uint8_t *tuple, uint8_t board, uint32_t blackout) {
  FwLayout l;
  std::memcpy(l.target.bytes, tuple, 16);
  l.target.bytes[UPD_T_BOARD] = board;
  const bool v407 = tuple[UPD_T_MCU] == UPD_MCU_V407;
  l.metadata_base = v407 ? UPD_V407_METADATA : UPD_V203_METADATA;
  l.metadata_bytes = v407 ? 4096 : 256;
  l.bootloader_bytes = v407 ? UPD_V407_BL_BYTES : UPD_V203_BL_BYTES;
  l.blackout_ms = blackout;
  return l;
}
/// Release 0.7.1: V407 board 4, V203 boards 1 and 2, with their signed layouts
static FwManifest release() {
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {image_for(V203_T, 1, 'b'), image_for(V407_T, 4, 'a'), image_for(V203_T, 2, 'c')};
  m.layouts = {layout_for(V203_T, 1, 20), layout_for(V407_T, 4, 40), layout_for(V203_T, 2, 20)};
  return m;
}

static void test_manifest() {
  FwManifest m = release();
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "the release checks");
  check(m.images[0].version_word == 0x000701FF && m.version_word == 0x000701FF, "an image without its own version has the release's");
  m.images[2].version = "0.7.2-beta.1";
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "its signed descriptor must say the same version");
  seal_descriptor(m.images[2], 0x00070241);
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr && m.images[2].version_word == 0x00070241,
        "an image may carry its own version");
  m = release();
  m.images[1].descriptor[UPD_D_MIN_BL] = 2;
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "a descriptor that differs from its entry refuses the release");
  m.images[2].version = "seven";
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "a malformed image version is refused");
  m = release();
  m.layouts[1].metadata_bytes = 256;
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "a layout whose metadata doesn't end at the slot is refused");
  m = release();
  m.layouts.push_back(m.layouts[0]);
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "two layouts for one target are refused");
  m = release();
  m.layouts[0].target.bytes[UPD_T_ABI] = 2;
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "a layout outside ABI 3 is refused");
  m = release();
  fw_check_manifest(m, FW_CH_STABLE);
  upd_target_t t{};
  std::memcpy(t.bytes, V407_T, 16);
  check(fw_layout_for(m, t) == &m.layouts[1], "layouts match the full 16-byte target");
  t.bytes[UPD_T_REVISION] ^= 2;
  check(fw_layout_for(m, t) == nullptr, "... revision byte included");

  // the closest image's first failed check is the reason
  std::memcpy(t.bytes, V203_T, 16);
  FwMiss miss;
  uint8_t need = 0;
  check(fw_resolve_image(m, t, 3, &miss) == 0 && miss == FWM_NONE, "V203 board 1 resolves");
  t.bytes[UPD_T_REVISION] = 0x11;
  check(fw_resolve_image(m, t, 3, &miss) < 0 && miss == FWM_REVISION, "outside the revision range");
  t.bytes[UPD_T_REVISION] = 0x04;
  m.images[0].target.bytes[UPD_T_FEATURES] = UPD_FEATURE_STATUS;
  check(fw_resolve_image(m, t, 3, &miss) < 0 && miss == FWM_FEATURES, "a required feature it lacks");
  m.images[0].target.bytes[UPD_T_FEATURES] = 0;
  m.images[0].min_bootloader = 4;
  check(fw_resolve_image(m, t, 3, &miss, &need) < 0 && miss == FWM_BOOTLOADER && need == 4, "a newer bootloader needed");
  m.images[0].min_bootloader = 3;
  m.images[0].target.bytes[UPD_T_SLOT_SIZE + 3] ^= 4;
  check(fw_resolve_image(m, t, 3, &miss) < 0 && miss == FWM_GEOMETRY, "another slot geometry");
  m.images[0].target.bytes[UPD_T_SLOT_SIZE + 3] ^= 4;
  t.bytes[UPD_T_VARIANT] = 7;
  check(fw_resolve_image(m, t, 3, &miss) < 0 && miss == FWM_MISSING, "another assembly variant has no image");
  t.bytes[UPD_T_VARIANT] = 0;
  m.images.push_back(m.images[0]);
  check(fw_resolve_image(m, t, 3, &miss) == 0, "the same image twice is one artifact");
  m.images.back().sha256 = std::string(64, 'B');
  check(fw_resolve_image(m, t, 3, &miss) == 0, "hex case doesn't make another image");
  m.images.back().sha256 = std::string(64, 'd');
  check(fw_resolve_image(m, t, 3, &miss) < 0 && miss == FWM_AMBIGUOUS, "two different images that match: ambiguous");
  check(fw_check_manifest(m, FW_CH_STABLE) != nullptr, "... and such a release is refused as a whole");
  m.images.back().rev_min = 0x05;
  m.images.back().rev_max = 0x0F;
  m.images[0].rev_max = 0x04;
  seal_descriptor(m.images.back(), 0x000701FF);
  seal_descriptor(m.images[0], 0x000701FF);
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr && fw_resolve_image(m, t, 3, &miss) == 0,
        "disjoint revision ranges: one image per revision");
}

static PortInventory port(std::vector<PortNode> nodes) {
  PortInventory inv;
  inv.nodes = std::move(nodes);
  return inv;
}

static void test_plan() {
  FwManifest m = release();
  fw_check_manifest(m, FW_CH_STABLE);
  // V203 first on the line, a V407 behind, one-way V203s, a current V407
  auto inv = port({node(0, V203_T, 1, true), node(1, V407_T, 4, true), node(2, V203_T, 1, false),
                   node(3, V203_T, 2, false), node(4, V407_T, 4, true, 0x000701FF)});
  PortPlan plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(!plan.blocked && plan.deliverable() && plan.artifacts.size() == 3, "one artifact per identical image");
  check(plan.artifacts[0].family == UPD_MCU_V407 && plan.artifacts[1].family == UPD_MCU_V203 &&
            plan.artifacts[2].family == UPD_MCU_V203,
        "V407 family first, then V203");
  check(plan.artifacts[1].positions == std::vector<uint16_t>({0, 2}) &&
            plan.artifacts[2].positions == std::vector<uint16_t>({3}) &&
            plan.artifacts[0].positions == std::vector<uint16_t>({1}),
        "each artifact lists the nodes it is sent for");
  check(plan.nodes[0].artifact == 1 && plan.nodes[1].artifact == 0 && plan.nodes[3].artifact == 2,
        "the nodes point at their artifact after the family order");
  check(plan.nodes[0].expect == PE_VERIFIED && plan.nodes[2].expect == PE_SENT_UNVERIFIED &&
            plan.nodes[2].action == PA_SEND && !plan.nodes[2].before_known,
        "live nodes can be verified, one-way nodes only sent");
  check(plan.nodes[4].action == PA_CURRENT && plan.nodes[4].expected == 0x000701FF && plan.nodes[4].artifact == 0 &&
            plan.artifacts[0].matched == std::vector<uint16_t>({1, 4}),
        "already current: its image is the job's to judge it by, not sent for it");
  check(plan.nodes[0].before == 0x000700FF && plan.nodes[0].expected == 0x000701FF, "before and expected versions");
  check(plan.entry_ms == 100 + 40 && plan.gap_us == 500, "whole-port entry wait: largest bound + largest blackout");
  check(plan.nodes[1].layout_known && plan.nodes[1].metadata_base == UPD_V407_METADATA &&
            plan.nodes[1].bootloader_bytes == UPD_V407_BL_BYTES,
        "telemetry nodes take the signed layout of their exact target");
  // the provisioning wins over the manifest
  inv.nodes[2].layout_known = inv.nodes[2].blackout_known = true;
  inv.nodes[2].metadata_base = UPD_V203_METADATA;
  inv.nodes[2].metadata_bytes = 256;
  inv.nodes[2].bootloader_bytes = UPD_V203_BL_BYTES;
  inv.nodes[2].blackout_ms = 90;
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.nodes[2].blackout_ms == 90 && plan.entry_ms == 100 + 90, "a provisioned blackout overrides the manifest's");

  // reasons per node
  inv = port({node(0, V203_T, 1, true, 0x000800FF), node(1, V203_T, 9, true), node(2, V407_T, 4, true)});
  inv.nodes[2].target.bytes[UPD_T_REVISION] = 0x11;
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.nodes[0].action == PA_SKIP && plan.nodes[0].reason == PRS_NEWER, "a newer version is skipped");
  check(plan.nodes[1].action == PA_SKIP && plan.nodes[1].reason == PRS_MISSING_ARTIFACT &&
            plan.nodes[1].blocks == PRS_NO_BLACKOUT,
        "a board the release has no image for is skipped; without trusted timing it still blocks the port");
  check(plan.blocked && plan.block_position == 1 && plan.block_reason == PRS_NO_BLACKOUT,
        "the first blocking node and its reason");
  check(plan.nodes[2].reason == PRS_REVISION && port_reason_text(plan.nodes[2].reason) == "no image for its PCB revision",
        "a revision outside every range");
  inv.nodes[1] = node(1, V203_T, 1, true, 0x000701FF);
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.nodes[1].action == PA_CURRENT && plan.nodes[1].expected == 0x000701FF && plan.nodes[1].artifact >= 0 &&
            plan.artifacts[size_t(plan.nodes[1].artifact)].positions.empty(),
        "a board on the release is already current: never sent (no force: the sender never sets FORCE)");
  m.images[0].min_bootloader = 4;
  inv = port({node(0, V203_T, 1, true)});
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.nodes[0].reason == PRS_BOOTLOADER && port_reason_text(PRS_BOOTLOADER, plan.nodes[0].need_boot) ==
                                                      "needs bootloader 4" &&
            !plan.deliverable() && !plan.blocked,
        "needs a newer bootloader: nothing to send, nothing blocks");
  m = release();
  fw_check_manifest(m, FW_CH_STABLE);
  inv = port({node(0, V203_T, 1, true)});
  inv.nodes[0].target.bytes[UPD_T_MCU] = 2;
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.nodes[0].action == PA_SKIP && plan.nodes[0].reason == PRS_FAMILY, "an MCU without an update family");

  // blocking: no trusted target, a mismatch, no layout for a node that gets an image
  inv = port({node(0, V203_T, 1, true), node(1, V203_T, 1, false)});
  inv.nodes[1].target_known = false;
  inv.nodes[1].state = PNS_UNKNOWN;
  inv.nodes[1].issue = PNI_NOT_PROVISIONED;
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.blocked && plan.block_position == 1 && plan.block_reason == PRS_NO_TARGET &&
            plan.nodes[1].action == PA_NONE && plan.nodes[0].action == PA_SEND,
        "a node without a trusted target blocks the port; the others keep their plan for the report");
  check(port_reason_text(plan.nodes[1].reason, 0, &inv.nodes[1]) == "no return path and not provisioned",
        "its reason names the ledger's issue");
  inv.nodes[1] = node(1, V203_T, 1, true);
  inv.nodes[1].state = PNS_MISMATCH;
  inv.nodes[1].issue = PNI_UID_CHANGED;
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.blocked && plan.block_reason == PRS_MISMATCH, "a mismatch blocks the port");
  inv.nodes[1] = node(1, V203_T, 2, true);
  m.layouts.pop_back();
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.blocked && plan.block_position == 1 && plan.block_reason == PRS_NO_LAYOUT && plan.nodes[1].action == PA_SEND,
        "an image without a trusted layout for its node blocks the port");
  m = release();
  fw_check_manifest(m, FW_CH_STABLE);

  // a known receiver that doesn't answer blocks "update all" (something is wrong on the
  // port), every one of them named; each has its own Recover firmware button (a recovery plan: its image only, blind)
  auto quiet = [](PortNode n) {
    n.state = PNS_UNREACHABLE;
    n.issue = PNI_SILENT;
    n.live = false;
    return n;
  };
  inv = port({node(0, V203_T, 1, true), quiet(node(1, V203_T, 1, true, 0x000701FF)),
              quiet(node(2, V203_T, 1, true, 0x000700FF)), node(3, V407_T, 4, true)});
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.blocked && plan.block_reason == PRS_SILENT && plan.block_position == 1 &&
            plan.nodes[1].action == PA_UNREACHABLE && plan.nodes[2].action == PA_UNREACHABLE &&
            plan.nodes[1].expected == 0x000701FF && plan.nodes[0].action == PA_SEND &&
            port_blocked_text(plan) == "B2, B3 not answering: check power and cable",
        "silent known receivers block \"update all\", each named (%s)", port_blocked_text(plan).c_str());
  PortReport silent_rep = port_report_from_plan(plan, &inv);
  port_report_close(silent_rep, PJO_BLOCKED);
  std::string silent_text = port_report_text(silent_rep);
  check(silent_text.find("Not installed 0.7.1: B2, B3 not answering: check power and cable, nothing sent") == 0 &&
            silent_text.find("\nB2-3 not answering: check power and cable") != std::string::npos &&
            silent_rep.nodes[1].reason == PRQ_NOT_ANSWERING,
        "... and in the report (%s)", silent_text.c_str());
  check(port_plan_summary(plan).find("; blocked: B2, B3 not answering: check power and cable") != std::string::npos,
        "... and in the release summary");
  // B2's Recover firmware: its image only (the same image's older live board gets it too: it applies it), blind; the
  // other silent board doesn't block it; another board type is not part of it
  PortPlan rp = build_recover_plan(inv, m, PT_DIFF_POWER, 2);
  check(!rp.blocked && rp.recover == 2 && rp.nodes[2].action == PA_SEND && rp.nodes[2].expect == PE_RECOVERY &&
            rp.nodes[1].action == PA_UNREACHABLE && rp.nodes[0].action == PA_SEND && rp.nodes[3].action == PA_SKIP &&
            rp.nodes[3].reason == PRS_OTHER_TARGET && rp.artifacts[size_t(rp.nodes[2].artifact)].positions ==
            std::vector<uint16_t>({0, 2}) && rp.artifacts[size_t(rp.nodes[3].artifact)].positions.empty(),
        "a recovery: the silent board's image only, blind; another silent board doesn't block it");
  PortReport rrep = port_report_from_plan(rp, &inv);
  check(rrep.recover == 2 && port_report_text(rrep).find("\nB3 0.7.0>0.7.1 recovery") != std::string::npos &&
            port_report_text(rrep).find("\nB4 not part of this recovery") != std::string::npos,
        "... its plan as a report (%s)", port_report_text(rrep).c_str());
  rp = build_recover_plan(inv, m, PT_DIFF_POWER, 1);
  check(rp.blocked && rp.block_position == 1 && rp.block_reason == PRS_NOT_OLDER &&
            port_blocked_text(rp, &inv) == "B2 runs this release or a newer one: nothing to recover",
        "a silent board already on the release: nothing to recover (%s)", port_blocked_text(rp, &inv).c_str());
  rp = build_recover_plan(inv, m, PT_DIFF_POWER, 0);
  check(rp.blocked && rp.block_reason == PRS_ANSWERS &&
            port_blocked_text(rp, &inv) == "B1 answers: nothing to recover (Install updates it)",
        "a board that answers: nothing to recover (%s)", port_blocked_text(rp, &inv).c_str());
  inv.nodes[1].firmware_known = false;
  rp = build_recover_plan(inv, m, PT_DIFF_POWER, 1);
  check(!rp.blocked && rp.nodes[1].expect == PE_RECOVERY, "an unknown version counts as older: recoverable");
  inv.nodes[1].target_known = false;
  inv.nodes[1].has_return = false;
  rp = build_recover_plan(inv, m, PT_DIFF_POWER, 1);
  check(rp.blocked && port_blocked_text(rp, &inv) == "B2 was never seen answering here: nothing to recover from",
        "a board never seen: nothing to recover from (%s)", port_blocked_text(rp, &inv).c_str());
  rp = build_recover_plan(inv, m, PT_DIFF_POWER, 9);
  check(rp.blocked && rp.block_position == 9, "no such board: refused");
  inv = port({node(0, V203_T, 1, false)});
  rp = build_recover_plan(inv, m, PT_D2A_PIN, 0);
  check(rp.blocked && rp.block_reason == PRS_NO_RETURN, "a one-way board can't be recovered (%s)",
        port_blocked_text(rp, &inv).c_str());
  // a restart (its identity not read again yet) blocks "update all" for a moment, named as such
  inv = port({node(0, V203_T, 1, true), quiet(node(1, V203_T, 1, true))});
  inv.nodes[1].issue = PNI_IDENTITY_PENDING;
  plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.blocked && port_blocked_text(plan, &inv) == "B2 identity not read yet",
        "a restarting receiver blocks for a moment (%s)", port_blocked_text(plan, &inv).c_str());

  // the plan from a real ledger (the poller's decode path)
  TestPoller p;
  p.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  uint8_t uid[12];
  uid_of(uid, 9);
  p.identify(0, 4, 1, uid, 0x000700FF, 1000);
  std::vector<PortProvision> prov = {v203_provision(1)};
  prov[0].has_blackout = true;
  prov[0].blackout_ms = 25;
  const auto live = ledger(p, 2, prov, 2000);
  FwManifest r = release();
  r.images[1].target = live.nodes[0].target;   // the fixture's V407 assembly
  r.images[1].target.bytes[UPD_T_FEATURES] = UPD_FEATURE_STATUS;   // the image needs STATUS: the fixture has it
  seal_descriptor(r.images[1], 0x000701FF);
  r.layouts[1].target = live.nodes[0].target;
  fw_check_manifest(r, FW_CH_STABLE);
  plan = build_port_plan(live, r, PT_DIFF_POWER);
  check(!plan.blocked && plan.artifacts.size() == 2 && plan.artifacts[0].positions == std::vector<uint16_t>({0}) &&
            plan.nodes[1].action == PA_SEND && plan.nodes[1].blackout_ms == 25,
        "a live V407 and a provisioned V203: both planned, V407 first");
}

// ------------------------------------------------------------------------------------------------ report
static void test_report() {
  FwManifest m = release();
  fw_check_manifest(m, FW_CH_STABLE);
  // B0 V203 live, B1 V407 live, B2-3 one-way V203 board 1, B4 current, B5 newer, B6 needs bootloader 4, B7 silent
  auto inv = port({node(0, V203_T, 1, true), node(1, V407_T, 4, true), node(2, V203_T, 1, false),
                   node(3, V203_T, 1, false), node(4, V407_T, 4, true, 0x000701FF), node(5, V203_T, 1, true, 0x000800FF),
                   node(6, V203_T, 2, true), node(7, V203_T, 1, true)});
  inv.nodes[6].boot_version = 2;
  m.images[2].min_bootloader = 4;
  inv.nodes[7].state = PNS_UNREACHABLE;
  inv.nodes[7].issue = PNI_SILENT;
  inv.nodes[7].live = false;
  inv.nodes[7].firmware = 0x000701FF;   // (on the release when last seen: not sent to)
  const PortPlan plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(plan.nodes.size() == 8 && plan.blocked && plan.block_reason == PRS_SILENT && plan.block_position == 7,
        "eight boards: the limit, not over it (B8 silent: \"update all\" waits for it)");
  PortReport rep = port_report_from_plan(plan, &inv);
  check(rep.nodes[0].result == PRR_PENDING && rep.nodes[2].expect == PE_SENT_UNVERIFIED &&
            rep.nodes[4].result == PRR_CURRENT && rep.nodes[4].observed == 0x000701FF &&
            rep.nodes[5].result == PRR_SKIPPED && rep.nodes[5].reason == PRQ_NEWER &&
            rep.nodes[6].result == PRR_SKIPPED && rep.nodes[6].reason == PRQ_INCOMPATIBLE &&
            rep.nodes[6].detail == PRS_BOOTLOADER && rep.nodes[6].need_boot == 4 &&
            rep.nodes[7].result == PRR_UNREACHABLE && rep.nodes[7].reason == PRQ_NOT_ANSWERING,
        "the plan's results: pending, current, skipped newer / incompatible, not answering");
  check(rep.count(PRR_PENDING) == 4 && rep.release == 0x000701FF, "four nodes pending");
  std::string text = port_report_text(rep);
  check(text.find("Plan 0.7.1: 4 to update, blocked: B8 not answering: check power and cable") == 0 &&
            text.find("\nB3-4 ?>0.7.1 unverifiable") != std::string::npos &&
            text.find("\nB1-2 0.7.0>0.7.1 to verify") != std::string::npos &&
            text.find("\nB7 skipped: needs bootloader 4") != std::string::npos &&
            text.find("\nB8 not answering: check power and cable") != std::string::npos && text.size() <= PORT_REPORT_MAX,
        "the plan as text: who gets what, equal neighbours as a range (%s)", text.c_str());
  const std::string summary = port_plan_summary(plan);
  check(summary.find("Update all receivers on this port: 4 to update (2 images), 1 current, 2 skipped") == 0,
        "the release summary (%s)", summary.c_str());

  // Install refused (no update transport in this build): nothing was sent, the pending nodes failed unsent
  PortReport refused = rep;
  refused.refusal = PRF_TIMING;
  port_report_close(refused, PJO_REFUSED, 1700000000);
  check(refused.count(PRR_PENDING) == 0 && refused.nodes[0].result == PRR_FAILED &&
            refused.nodes[0].reason == PRQ_TRANSPORT && !refused.nodes[0].sent && refused.nodes[4].result == PRR_CURRENT &&
            refused.finished_s == 1700000000,
        "a refused job: pending nodes failed (transport), unsent; the others keep their result");
  text = port_report_text(refused);
  check(text.find("Not installed 0.7.1: the line's bit timing is not a protocol profile, nothing sent") == 0 &&
            text.find("\nB1-4 not sent 0.7.1") != std::string::npos,
        "the refused job's report (%s)", text.c_str());

  // blocked: the reason in the headline
  auto blocked_inv = inv;
  blocked_inv.nodes[3].target_known = false;
  blocked_inv.nodes[3].state = PNS_UNKNOWN;
  blocked_inv.nodes[3].issue = PNI_NOT_PROVISIONED;
  const PortPlan bplan = build_port_plan(blocked_inv, m, PT_DIFF_POWER);
  PortReport brep = port_report_from_plan(bplan, &blocked_inv);
  check(brep.block_position == 3 && brep.block_reason == PRS_NO_TARGET && brep.nodes[3].result == PRR_UNREACHABLE &&
            brep.nodes[3].reason == PRQ_NO_RETURN,
        "a blocked plan names the node; an unprovisioned one-way node is unknown, never absent");
  port_report_close(brep, PJO_BLOCKED);
  text = port_report_text(brep);
  check(text.find("Not installed 0.7.1: B4 no trusted target, nothing sent") == 0 &&
            text.find("\nB4 unknown: not provisioned") != std::string::npos,
        "the blocked job's report (%s)", text.c_str());

  // stored on the ESP32 and read back
  PortReportStore store;
  port_report_store(refused, store);
  PortReport back;
  check(port_report_load(store, back) && port_report_text(back) == port_report_text(refused) && back.refusal == PRF_TIMING &&
            back.nodes.size() == 8 && back.nodes[6].need_boot == 4 && back.nodes[0].before_known &&
            !back.nodes[2].before_known && back.finished_s == 1700000000,
        "the stored report reads back the same");
  PortReportStore bad = store;
  bad.magic = 0;
  check(!port_report_load(bad, back), "no stored report: none");
  bad = store;
  bad.nodes[1].result = 9;
  check(!port_report_load(bad, back), "a stored result out of range: none");
  bad = store;
  bad.count = 65;
  check(!port_report_load(bad, back), "more nodes than the store holds: none");

  // one-way nodes: their version is never observed; a sent release doesn't stay "available" for them
  PortReport sent = rep;
  for (auto &r : sent.nodes)
    if (r.result == PRR_PENDING) {
      r.result = r.expect == PE_VERIFIED ? PRR_VERIFIED : PRR_SENT_UNVERIFIED;
      r.sent = true;
    }
  sent.outcome = PJO_DONE;
  check(!port_needs_update(rep.nodes[2], &sent) && port_needs_update(rep.nodes[0], &sent) &&
            port_needs_update(rep.nodes[2], nullptr) && !port_needs_update(rep.nodes[4], nullptr),
        "a one-way node the release was sent to doesn't need it again; a live one is judged from its version");
  sent.release = 0x000700FF;
  check(port_needs_update(rep.nodes[2], &sent), "... another release does");
  text = port_report_text(sent);
  check(text.find("Done 0.7.0: 2 verified, 2 unverified, 0 failed") == 0 &&
            text.find("\nB3-4 sent, unverified 0.7.1") != std::string::npos,
        "a finished job's report (%s)", text.c_str());

  // a long report: the full report has every board with its reason (the log writes it, a line
  // each); Home Assistant's state keeps what fits in its 255 characters and names the boards left out and where they
  // are. Eight distinct bootloader refusals on a Diff-Power port; fifty boards on a Dig2Analog-Plus-5ch pin.
  auto long_report = [](size_t n, bool distinct) {
    PortReport r;
    r.outcome = PJO_NONE;
    r.release = 0x000701FF;
    r.nodes.resize(n);
    for (size_t k = 0; k < n; k++) {
      r.nodes[k].position = static_cast<uint16_t>(k);
      r.nodes[k].result = PRR_SKIPPED;
      r.nodes[k].detail = distinct ? PRS_BOOTLOADER : static_cast<uint8_t>(k % 2 ? PRS_GEOMETRY : PRS_FEATURES);
      r.nodes[k].need_boot = static_cast<uint8_t>(10 + k);
    }
    return r;
  };
  auto shown_boards = [](const std::string &s) {   // the boards the state's node lines cover
    size_t n = 0;
    for (size_t at = s.find("\nB"); at != std::string::npos; at = s.find("\nB", at + 1)) {
      const size_t a = std::stoul(s.substr(at + 2));
      const size_t sep = s.find_first_of("- ", at + 2);
      n += s[sep] == '-' ? std::stoul(s.substr(sep + 1)) - a + 1 : 1;
    }
    return n;
  };
  const std::pair<size_t, bool> cases[] = {{8, true}, {50, false}, {50, true}};
  for (const auto &c : cases) {
    const PortReport many = long_report(c.first, c.second);
    const std::string full = port_report_full_text(many);
    const std::vector<PortReportLine> lines = port_report_lines(many);
    size_t covered = 0;
    bool reasons = true;
    for (const auto &l : lines) {
      covered += l.nodes ? l.last - l.first + 1u : 0u;
      if (l.nodes && c.second)
        reasons &= l.first == l.last && l.text == "B" + std::to_string(l.first + 1) + " skipped: needs bootloader " +
                                                      std::to_string(10 + l.first);
    }
    check(reasons && covered == c.first && lines.size() > 1 && full.find(lines.back().text) != std::string::npos,
          "%u boards%s: the full report (the log's lines) has every board with its reason (%u covered)",
          static_cast<unsigned>(c.first), c.second ? ", each refused for its own bootloader" : "",
          static_cast<unsigned>(covered));
    text = port_report_text(many);
    const size_t last = text.rfind('\n');
    const std::string tail = text.substr(last + 1);
    const size_t hidden = std::stoul(tail);
    check(text.size() <= PORT_REPORT_MAX && full.size() > PORT_REPORT_MAX &&
              tail.find(" more boards, B") != std::string::npos &&
              tail.find("-" + std::to_string(c.first) + ": full report in the device's log") != std::string::npos &&
              shown_boards(text.substr(0, last)) + hidden == c.first,
          "%u boards: Home Assistant's state within %u characters (%u), its last line names the %u boards left out and "
          "where they are (%s)", static_cast<unsigned>(c.first), static_cast<unsigned>(PORT_REPORT_MAX),
          static_cast<unsigned>(text.size()), static_cast<unsigned>(hidden), tail.c_str());
  }
  check(port_report_text(rep) == port_report_full_text(rep), "a report that fits is the whole report");

  // the board limit is the port type's: a Diff-Power port eight, a Dig2Analog-Plus-5ch pin fifty
  auto nine = inv;
  nine.nodes.push_back(node(8, V203_T, 1, true));
  PortPlan over = build_port_plan(nine, m, PT_DIFF_POWER);
  check(over.blocked && over.block_position == 8 && over.block_reason == PRS_TOO_MANY && over.max_boards == 8 &&
            port_reason_text(PRS_TOO_MANY, over.max_boards) == "more than 8 boards on the port",
        "a Diff-Power port: nine boards refused");
  PortReport too = port_report_from_plan(over, &nine);
  port_report_close(too, PJO_BLOCKED);
  check(port_report_text(too).find("Not installed 0.7.1: more than 8 boards on the port, nothing sent") == 0 &&
            port_plan_summary(over).find("; blocked: more than 8 boards on the port") != std::string::npos,
        "... said in the report and the summary (%s)", port_report_text(too).c_str());
  auto pin = port({});
  for (uint16_t k = 0; k < 50; k++)
    pin.nodes.push_back(node(k, V203_T, 1, false));
  PortPlan d2a = build_port_plan(pin, m, PT_D2A_PIN);
  check(!d2a.blocked && d2a.max_boards == 50 && d2a.artifacts.size() == 1 && d2a.artifacts[0].positions.size() == 50,
        "a Dig2Analog-Plus-5ch pin: fifty boards planned");
  over = build_port_plan(pin, m, PT_DIFF_POWER);
  check(over.blocked && over.block_reason == PRS_TOO_MANY, "... the same fifty on a Diff-Power port refused");
  pin.nodes.push_back(node(50, V203_T, 1, false));
  over = build_port_plan(pin, m, PT_D2A_PIN);
  check(over.blocked && over.block_position == 50 && over.block_reason == PRS_TOO_MANY,
        "a Dig2Analog-Plus-5ch pin: fifty-one refused");
  PortReport stored = port_report_from_plan(d2a, &pin);
  PortReportStore st;
  port_report_store(stored, st);
  PortReport st_back;
  check(port_report_load(st, st_back) && st_back.nodes.size() == 50, "a fifty-board report is stored whole");
  over = build_port_plan(inv, m, PT_UNKNOWN);
  check(over.blocked && over.block_position < 0 && over.block_reason == PRS_PORT_TYPE,
        "no port type: no limit known, refused");
}

// ------------------------------------------------------------------------------------------------ artifact source
static int starts = 0, frees = 0;
static bool start_ok = true;
static bool fake_start(PortArtifactSource *) {
  starts++;
  return start_ok;
}
static void fake_free(uint8_t *img, size_t) {
  frees++;
  delete[] img;
}
/// The complete image an entry describes (its body CRC sealed into the descriptor), as a new[] buffer
static uint8_t *image_bytes(FwImageInfo &i) {
  auto *img = new uint8_t[i.size];
  const uint32_t length = i.size - UPD_DESC_BYTES;
  for (uint32_t k = 0; k < length; k++)
    img[UPD_DESC_BYTES + k] = uint8_t(k * 13 + 5);
  ultra_put32(i.descriptor + UPD_D_BODY_CRC, upd_crc32_words(0xFFFFFFFFu, img + UPD_DESC_BYTES, length / 4));
  ultra_put32(i.descriptor + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, i.descriptor, 15));
  std::memcpy(img, i.descriptor, UPD_DESC_BYTES);
  return img;
}

static void test_artifact() {
  FwManifest m = release();
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "the release checks (its version words)");
  FwImageInfo info = m.images[0];
  uint8_t *img = image_bytes(info);
  uint8_t sha[32]{};
  info.sha256 = std::string(64, '0');   // (the platform hashes; the fixture's hash is zero)
  PortArtifactSource src;
  src.init(info, "https://h/fw-b.bin", fake_start, fake_free);
  check(PortArtifactSource::prepare(&src) == 1 && starts == 1 && src.state() == PortArtifactSource::LOADING,
        "the first prepare starts the download: pending");
  check(PortArtifactSource::prepare(&src) == 1 && starts == 1, "... pending while it runs, started once");
  uint8_t out[32];
  check(PortArtifactSource::read(&src, 0, out, 32) < 0, "nothing to read before it is loaded");
  src.loaded(img, info.size, sha);
  check(PortArtifactSource::prepare(&src) == 0 && src.state() == PortArtifactSource::READY, "checked: ready");
  check(PortArtifactSource::read(&src, 32, out, 32) == 0 && out[0] == uint8_t(32 * 13 + 5),
        "the body is read from RAM (offset 0: after the descriptor)");
  check(PortArtifactSource::read(&src, info.size - UPD_DESC_BYTES - 16, out, 32) < 0, "never past the body");
  PortArtifactSource::release(&src);
  check(frees == 1 && src.state() == PortArtifactSource::RELEASED && PortArtifactSource::read(&src, 0, out, 4) < 0,
        "released: freed, nothing to read");

  // an image that is not the manifest's: failed, freed
  PortArtifactSource bad;
  bad.init(info, "u", fake_start, fake_free);
  PortArtifactSource::prepare(&bad);
  uint8_t *wrong = image_bytes(info);
  wrong[UPD_DESC_BYTES + 7] ^= 1;   // a body byte: its CRC no longer matches
  bad.loaded(wrong, info.size, sha);
  check(PortArtifactSource::prepare(&bad) < 0 && frees == 2 && bad.error() == "image body CRC mismatch",
        "a corrupt download fails the artifact and is freed (%s)", bad.error().c_str());
  PortArtifactSource::release(&bad);
  uint8_t sha1[32]{};
  sha1[0] = 1;
  PortArtifactSource other;
  other.init(info, "u", fake_start, fake_free);
  PortArtifactSource::prepare(&other);
  other.loaded(image_bytes(info), info.size, sha1);
  check(PortArtifactSource::prepare(&other) < 0 && other.error() == "image SHA-256 mismatch" && frees == 3,
        "another SHA-256: failed (%s)", other.error().c_str());

  // the job lets go while it still downloads: the late image is freed at once
  PortArtifactSource late;
  late.init(info, "u", fake_start, fake_free);
  PortArtifactSource::prepare(&late);
  PortArtifactSource::release(&late);
  late.loaded(image_bytes(info), info.size, sha);
  check(frees == 4 && late.state() == PortArtifactSource::RELEASED, "released while downloading: the image is freed");

  // a download that can't start
  start_ok = false;
  PortArtifactSource none;
  none.init(info, "u", fake_start, fake_free);
  check(PortArtifactSource::prepare(&none) < 0 && none.state() == PortArtifactSource::FAILED, "no download: failed");
  start_ok = true;
}

int main() {
  test_inventory();
  test_manifest();
  test_plan();
  test_report();
  test_artifact();
  std::printf("port update: %d checks, %d failed (inventory, manifest, plan, report)\n", checks, failures);
  return failures ? 1 : 0;
}
