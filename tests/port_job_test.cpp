// The port job's glue (port_job.h) against the sender library's own port job (tests/run_port_job_test.py builds it
// with the pinned sender sources): the plan's nodes and artifacts are what the job takes, its admission and results
// agree with the plan (current, newer, incompatible, pending, one-way), the board limit is the port type's, and the
// constants this component names are the job's.

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../components/ultra_line/port_job.h"
#include "../components/ultra_line/port_job_driver.h"

#include "ultra_sender_job.h"
#include "ultra_sender_update.h"

using namespace esphome::ultra_line;

// the values this component names are the job's
static_assert(PRR_PENDING == ULTRA_JOB_PENDING && PRR_VERIFIED == ULTRA_JOB_VERIFIED && PRR_CURRENT == ULTRA_JOB_CURRENT &&
                  PRR_SKIPPED == ULTRA_JOB_SKIPPED && PRR_FAILED == ULTRA_JOB_FAILED &&
                  PRR_SENT_UNVERIFIED == ULTRA_JOB_SENT_UNVERIFIED && PRR_UNREACHABLE == ULTRA_JOB_UNREACHABLE,
              "results");
static_assert(PRQ_NONE == ULTRA_JOB_REASON_NONE && PRQ_INCOMPATIBLE == ULTRA_JOB_INCOMPATIBLE &&
                  PRQ_NEWER == ULTRA_JOB_NEWER && PRQ_MISSING_ARTIFACT == ULTRA_JOB_MISSING_ARTIFACT &&
                  PRQ_NO_RETURN == ULTRA_JOB_NO_RETURN && PRQ_NO_REPLY == ULTRA_JOB_NO_REPLY &&
                  PRQ_MISMATCH == ULTRA_JOB_MISMATCH && PRQ_ABORT == ULTRA_JOB_ABORT && PRQ_TRANSPORT == ULTRA_JOB_TRANSPORT &&
                  PRQ_REFUSED == ULTRA_JOB_REFUSED && PRQ_RECOVERY_SENT == ULTRA_JOB_RECOVERY_SENT &&
                  PRQ_NOT_ANSWERING == ULTRA_JOB_NOT_ANSWERING && PRQ_PORT_BLOCKED == ULTRA_JOB_PORT_BLOCKED &&
                  PRQ_OTHER_TARGET == ULTRA_JOB_OTHER_TARGET,
              "reasons");
static_assert(PT_DIFF_POWER == ULTRA_PORT_DIFF_POWER && PT_D2A_PIN == ULTRA_PORT_D2A_PIN &&
                  PORT_DIFF_POWER_MAX_BOARDS == ULTRA_PORT_DIFF_POWER_MAX_BOARDS &&
                  PORT_D2A_PIN_MAX_BOARDS == ULTRA_PORT_D2A_PIN_MAX_BOARDS,
              "port types");
static_assert(LCW_DEFAULT == ULTRA_PORT_PROFILE_DEFAULT && LCW_BOARD_COUNT == ULTRA_PORT_PROFILE_BOARD_COUNT &&
                  LCW_TOO_MANY == ULTRA_PORT_PROFILE_TOO_MANY && LCW_NO_BOARDS == ULTRA_PORT_PROFILE_NO_BOARDS &&
                  LCW_UNKNOWN_TYPE == ULTRA_PORT_PROFILE_UNKNOWN_TYPE,
              "the start profile's reasons are the sender helper's");
static_assert(PORT_JOB_TOO_MANY_BOARDS == ULTRA_UPDATE_TOO_MANY_BOARDS &&
                  PORT_JOB_OVERLAPPING_ARTIFACTS == ULTRA_UPDATE_OVERLAPPING_ARTIFACTS &&
                  PORT_JOB_TOO_FEW_PASSES == ULTRA_UPDATE_TOO_FEW_PASSES && PORT_JOB_LINE_TOO_FAST == ULTRA_UPDATE_LINE_TOO_FAST &&
                  PORT_JOB_SILENT_BOARDS == ULTRA_UPDATE_SILENT_BOARDS &&
                  PORT_JOB_NOT_RECOVERABLE == ULTRA_UPDATE_NOT_RECOVERABLE,
              "the job's refusals the report names");
static_assert(PORT_JOB_TARGET == -UPD_ERROR_TARGET && PORT_JOB_BOUNDS == -UPD_ERROR_BOUNDS &&
                  PORT_JOB_CRC == -UPD_ERROR_CRC && PORT_JOB_UNSUPPORTED == -UPD_ERROR_UNSUPPORTED,
              "the job's UPD_ERROR_* refusals the report names");
static_assert(PORT_JOB_BLIND == ULTRA_UPDATE_BLIND, "blind mode");
static_assert(PORT_JOB_PASSES == 3, "three complete DATA passes per blind subtransfer");
static_assert(PORT_FAMILY_ORDER[0] == UPD_MCU_V407 && PORT_FAMILY_ORDER[1] == UPD_MCU_V203, "family order");

static int failures = 0, checks = 0, pending = 0;
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

/// A case a known follow-up makes true: pending (reported, not a failure) until it holds
static void known(bool ok, const char *who, const char *fmt, ...) {
  checks++;
  if (ok)
    return;
  pending++;
  va_list ap;
  va_start(ap, fmt);
  std::printf("PENDING %s: ", who);
  std::vprintf(fmt, ap);
  std::printf("\n");
  va_end(ap);
}

static const uint8_t V203_T[16] = {1, UPD_MCU_V203, UPD_PACKAGE_F8U6, UPD_LAYOUT_V203, UPD_BOOT_ABI, 0, 0, 0x04,
                                   0x08, 0x00, 0x21, 0x00, 0x00, 0x00, 0xDF, 0x00};
static const uint8_t V407_T[16] = {4, UPD_MCU_V407, UPD_PACKAGE_RET6, UPD_LAYOUT_V407, UPD_BOOT_ABI, 3, 0, 0x01,
                                   0x08, 0x00, 0x50, 0x00, 0x00, 0x07, 0xB0, 0x00};

/// A release image for a target: its descriptor (ABI 3) and body
struct Image {
  std::vector<uint8_t> bytes;
};
static Image image_for(const uint8_t *tuple, uint32_t version, uint32_t length = 1024) {
  Image im;
  im.bytes.assign(64 + length, 0);
  uint8_t *d = im.bytes.data();
  std::memcpy(d, "QDU6", 4);
  d[UPD_D_VERSION] = UPD_DESC_VERSION;
  d[UPD_D_REV_MIN] = 0;
  d[UPD_D_REV_MAX] = 0x0F;
  std::memcpy(d + UPD_D_TARGET, tuple, 7);
  d[UPD_D_FEATURES] = 0;
  d[UPD_D_MIN_BL] = 3;
  ultra_put32(d + UPD_D_FW_VERSION, version);
  ultra_put32(d + UPD_D_BODY_LEN, length);
  const uint32_t base = ultra_get32(tuple + UPD_T_SLOT_BASE);
  ultra_put32(d + UPD_D_LOAD, base);
  ultra_put32(d + UPD_D_ENTRY, base + 2);
  d[UPD_D_RECORD_FORMAT] = 2;
  d[UPD_D_WIRE] = 3;
  d[UPD_D_DESCRIPTOR_FORMAT] = 1;
  for (uint32_t k = 0; k < length; k++)
    d[64 + k] = uint8_t(k * 7);
  ultra_put32(d + UPD_D_BODY_CRC, upd_crc32_words(0xFFFFFFFFu, d + 64, length / 4));
  ultra_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15));
  return im;
}
static int read_body(void *context, uint32_t offset, uint8_t *data, uint32_t length) {
  const Image *im = static_cast<const Image *>(context);
  if (uint64_t(offset) + length > im->bytes.size() - 64)
    return -1;
  std::memcpy(data, im->bytes.data() + 64 + offset, length);
  return 0;
}
static int prepared = 0, released = 0;
static int prepare_body(void *) { prepared++; return 0; }
static void release_body(void *) { released++; }

/// A ledger record (live: version known, FW_UPDATE; one-way: provisioned)
static PortNode node(uint16_t position, const uint8_t *tuple, bool live, uint32_t fw = 0x000700FF) {
  PortNode n;
  n.position = position;
  n.state = live ? PNS_REACHABLE : PNS_ONE_WAY;
  n.issue = PNI_NONE;
  n.live = live;
  n.provisioned = !live;
  n.target_known = true;
  std::memcpy(n.target.bytes, tuple, 16);
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
  n.evidence_ms = live ? 900 : 0;
  return n;
}
static FwImageInfo info_for(const Image &im, const uint8_t *tuple, char sha) {
  FwImageInfo i;
  std::memcpy(i.target.bytes, tuple, 16);
  i.target.bytes[UPD_T_FEATURES] = 0;
  i.target.bytes[UPD_T_REVISION] = 0;
  i.rev_max = 0x0F;
  i.min_bootloader = 3;
  i.size = static_cast<uint32_t>(im.bytes.size());
  i.sha256 = std::string(64, sha);
  i.url = "fw.bin";
  std::memcpy(i.descriptor, im.bytes.data(), UPD_DESC_BYTES);   // the image's own, signed with the manifest
  return i;
}
static FwLayout layout_for(const uint8_t *tuple) {
  FwLayout l;
  std::memcpy(l.target.bytes, tuple, 16);
  const bool v407 = tuple[UPD_T_MCU] == UPD_MCU_V407;
  l.metadata_base = v407 ? UPD_V407_METADATA : UPD_V203_METADATA;
  l.metadata_bytes = v407 ? 4096 : 256;
  l.bootloader_bytes = v407 ? UPD_V407_BL_BYTES : UPD_V203_BL_BYTES;
  l.blackout_ms = 30;
  return l;
}

static ultra_sender_board_t boards[ULTRA_UPDATE_MAX_BOARDS];
static ultra_sender_item_t items[512];
static ultra_sender_t line;
/// The job's sender line, at the hub's line profile (fw_update.cpp: the hub's job_timing)
static void line_for(uint16_t count, uint8_t port_type, uint8_t profile = 0) {
  ultra_sender_config_t c;
  ultra_sender_config_init(&c, count, 0);
  c.light_capacity = 0;
  c.clear_unused = 0;
  check(!ultra_timing_profile(&c.timing, profile, 40000000u, 500), "profile %u's timing", profile);
  check(!ultra_sender_init(&line, &c, boards, items, 512), "the sender line for %u boards", count);
  (void) port_type;
}

/// The plan's nodes and artifacts handed to the job; its results back
struct Glue {
  std::vector<ultra_update_node_t> nodes;
  std::vector<ultra_update_artifact_t> artifacts;
  std::vector<ultra_update_result_t> results;
  ultra_update_job_options_t options{};
  ultra_update_job_t job{};
  /// images: by the first character of their SHA-256 in the manifest
  int begin(const PortInventory &inv, const PortPlan &plan, const std::vector<Image> &images, int64_t now_us) {
    nodes.resize(inv.nodes.size());
    for (size_t k = 0; k < inv.nodes.size(); k++)
      port_job_node(nodes[k], inv.nodes[k], plan.nodes[k], now_us, 1000);
    artifacts.resize(plan.artifacts.size());
    for (size_t a = 0; a < plan.artifacts.size(); a++) {
      const Image &im = images[size_t(plan.artifacts[a].image.sha256[0] - 'a')];
      check(!std::memcmp(plan.artifacts[a].image.descriptor, im.bytes.data(), UPD_DESC_BYTES), "artifact %u's descriptor",
            unsigned(a));
      port_job_artifact(artifacts[a], plan.artifacts[a].image.descriptor, const_cast<Image *>(&im), read_body,
                        prepare_body, release_body);
    }
    results.resize(nodes.size());
    port_job_options(options, plan, 7, ULTRA_RW_RATE_1M);
    // "update all", or a recovery (fw_update.cpp start_job_: the same inputs)
    return plan.recover >= 0
               ? ultra_update_job_recover(&job, &line, nodes.data(), static_cast<uint16_t>(nodes.size()),
                                          static_cast<uint16_t>(plan.recover), artifacts.data(),
                                          static_cast<uint16_t>(artifacts.size()), results.data(), &options, now_us)
               : ultra_update_job_begin(&job, &line, nodes.data(), static_cast<uint16_t>(nodes.size()), artifacts.data(),
                                        static_cast<uint16_t>(artifacts.size()), results.data(), &options, now_us);
  }
};

static void test_glue() {
  const uint32_t V070 = 0x000700FF, V071 = 0x000701FF;
  // a V203 receiver to update, a one-way V203 board, a current V407, a newer V203
  PortInventory inv;
  inv.nodes = {node(0, V203_T, true), node(1, V203_T, false), node(2, V407_T, true, V071), node(3, V203_T, true, 0x000800FF)};
  std::vector<Image> images = {image_for(V203_T, V071), image_for(V407_T, V071)};
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {info_for(images[0], V203_T, 'a'), info_for(images[1], V407_T, 'b')};
  m.layouts = {layout_for(V203_T), layout_for(V407_T)};
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "the release checks");
  const PortPlan plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(!plan.blocked && plan.artifacts.size() == 2 && plan.artifacts[1].positions.size() == 2 &&
            plan.nodes[2].action == PA_CURRENT && plan.nodes[3].reason == PRS_NEWER,
        "the plan: one image for two V203 nodes, the V407 current (its image known), one newer");

  line_for(4, PT_DIFF_POWER);
  Glue g;
  const int64_t now = 1000000;
  check(g.begin(inv, plan, images, now) == 0, "the job takes the plan's nodes and artifacts");
  check(g.options.passes == 3 && g.options.mode == ULTRA_UPDATE_BLIND && g.options.port_type == ULTRA_PORT_DIFF_POWER &&
            g.job.options.family_order[0] == UPD_MCU_V407,
        "blind, three passes, the port type, V407 first");
  check(g.nodes[0].trusted && g.nodes[0].reachable && g.nodes[0].participant.has_return &&
            g.nodes[0].participant.layout.metadata_base == UPD_V203_METADATA && g.nodes[0].firmware == V070 &&
            g.nodes[0].blackout_ms == 30 && g.nodes[0].evidence_us == now - 100000,
        "a live node: trusted, reachable, its layout from the manifest, its version, its evidence time");
  check(g.nodes[1].trusted && !g.nodes[1].reachable && !g.nodes[1].participant.has_return && g.nodes[1].firmware == 0,
        "a one-way node: trusted, no return path, its version unknown");
  for (size_t k = 0; k < plan.nodes.size(); k++) {
    const auto want = plan.nodes[k].action == PA_SEND ? PRR_PENDING
                      : plan.nodes[k].action == PA_CURRENT ? PRR_CURRENT
                                                            : PRR_SKIPPED;
    check(g.results[k].result == want, "node %u: the job's result %u is the plan's %u", unsigned(k), g.results[k].result,
          unsigned(want));
  }
  check(g.results[3].reason == ULTRA_JOB_NEWER && g.results[0].expected == V071 && g.results[0].before == V070,
        "newer, expected and before versions agree");

  // its results into the report
  PortReport rep = port_report_from_plan(plan, &inv);
  port_job_take_results(rep, g.results.data(), g.results.size());
  check(rep.nodes[0].result == PRR_PENDING && rep.nodes[2].result == PRR_CURRENT && rep.nodes[3].result == PRR_SKIPPED &&
            rep.nodes[3].detail == PRS_NEWER,
        "the report takes the job's results, keeping the plan's finer reasons");

  // the job runs to its end with no answers: progress on the way, the active image prepared and released once; the
  // live node never confirms (unreachable), the one-way node ends sent, unverified
  uint8_t frame[1024];
  int64_t t = now;
  int frames = 0;
  PortJobProgress mid;
  std::string pass_texts, last_pass;   // each "pass N of M" the text showed on the way, in order
  bool pass_over = false;   // some text counted a pass past the last one
  for (int k = 0; k < 400000 && g.job.state != ULTRA_JOB_DONE; k++) {
    const int n = ultra_update_job_next_frame(&g.job, t, frame, sizeof(frame));
    if (n < 0)
      break;
    if (n > 0 && ++frames == 60) {
      ultra_update_job_progress_t jp{};
      if (ultra_update_job_progress(&g.job, t, &jp) == 0)
        mid = port_job_progress(jp);
    }
    if (n > 0 && frames % 10 == 0) {
      ultra_update_job_progress_t jp{};
      if (ultra_update_job_progress(&g.job, t, &jp) == 0) {
        const std::string text = port_job_progress_text(port_job_progress(jp));
        const size_t at = text.find(": pass ");
        if (at != std::string::npos) {
          const std::string pass = text.substr(at + 2, text.find(',', at) - at - 2);   // "pass 2 of 3"
          pass_over = pass_over || std::atoi(pass.c_str() + 5) > jp.passes;
          if (pass != last_pass)
            pass_texts += (pass_texts.empty() ? "" : " | ") + pass;
          last_pass = pass;
        }
      }
    }
    t = std::max<int64_t>(t + 100, g.job.due_us);
  }
  check(mid.subtransfers == 1 && mid.subtransfer == 1 && mid.passes == 3,
        "progress on the way: one subtransfer, three passes (%u/%u, %u passes)", mid.subtransfer, mid.subtransfers,
        mid.passes);
  check(port_job_progress_text(mid).find("image 1 of 1: pass ") == 0, "progress text (%s)",
        port_job_progress_text(mid).c_str());
  // (bench 2026-10-30: "pass 4 of 3": the job's pass counts from 1 already)
  check(pass_texts == "pass 1 of 3 | pass 2 of 3 | pass 3 of 3" && !pass_over, "the pass text counts 1, 2, 3 of 3 (%s)",
        pass_texts.c_str());
  {
    PortJobProgress q;
    q.subtransfer = 1;
    q.subtransfers = 1;
    q.passes = 3;
    check(port_job_progress_text(q) == "image 1 of 1", "no pass before the transfer starts (%s)",
          port_job_progress_text(q).c_str());
    q.pass = 3;
    check(port_job_progress_text(q) == "image 1 of 1: pass 3 of 3", "the last pass is 3 of 3 (%s)",
          port_job_progress_text(q).c_str());
  }
  check(g.job.state == ULTRA_JOB_DONE && prepared == 1 && released == 1, "done; its image prepared and released once");
  port_job_take_results(rep, g.results.data(), g.results.size());
  check(rep.nodes[0].result == PRR_UNREACHABLE && rep.nodes[0].sent && rep.nodes[1].result == PRR_SENT_UNVERIFIED &&
            rep.nodes[1].sent && rep.nodes[2].result == PRR_CURRENT && rep.nodes[3].result == PRR_SKIPPED,
        "no evidence: the live node unreachable, the one-way node sent, unverified (%u/%u, %u/%u)", rep.nodes[0].result,
        rep.nodes[0].reason, rep.nodes[1].result, rep.nodes[1].reason);
  rep.outcome = PJO_DONE;
  check(port_report_text(rep).find("\nB2 sent, unverified 0.7.1") != std::string::npos, "its report (%s)",
        port_report_text(rep).c_str());

  // aborted early: the pending nodes fail (abort), nothing else changes
  line_for(4, PT_DIFF_POWER);
  Glue ga;
  check(ga.begin(inv, plan, images, now) == 0, "a second job");
  t = now;
  for (int k = 0; k < 20 && ga.job.state != ULTRA_JOB_DONE; k++) {
    if (ultra_update_job_next_frame(&ga.job, t, frame, sizeof(frame)) < 0)
      break;
    t = std::max<int64_t>(t + 100, ga.job.due_us);
  }
  ultra_update_job_abort(&ga.job);
  for (int k = 0; k < 200000 && ga.job.state != ULTRA_JOB_DONE; k++) {
    if (ultra_update_job_next_frame(&ga.job, t, frame, sizeof(frame)) < 0)
      break;
    t = std::max<int64_t>(t + 100, ga.job.due_us);
  }
  PortReport arep = port_report_from_plan(plan, &inv);
  port_job_take_results(arep, ga.results.data(), ga.results.size());
  check(ga.job.state == ULTRA_JOB_DONE && arep.nodes[0].result == PRR_FAILED && arep.nodes[0].reason == PRQ_ABORT &&
            arep.nodes[1].result == PRR_FAILED && arep.nodes[2].result == PRR_CURRENT,
        "an abort fails the pending nodes, the others keep their result (%u/%u)", arep.nodes[0].result,
        arep.nodes[0].reason);

  // the port type's limit: the job refuses what the plan refuses
  PortInventory nine;
  for (uint16_t k = 0; k < 9; k++)
    nine.nodes.push_back(node(k, V203_T, false));
  const PortPlan dp = build_port_plan(nine, m, PT_DIFF_POWER);
  line_for(9, PT_DIFF_POWER);
  Glue g9;
  check(dp.blocked && dp.block_reason == PRS_TOO_MANY, "the plan refuses nine boards on a Diff-Power port");
  PortPlan forced = dp;
  forced.blocked = false;
  check(g9.begin(nine, forced, images, now) == ULTRA_UPDATE_TOO_MANY_BOARDS, "... and so does the job");
  PortInventory fifty;
  for (uint16_t k = 0; k < 50; k++) {
    fifty.nodes.push_back(node(k, V203_T, false));
    fifty.nodes.back().uid_known = true;   // (provisioned with its UID: see the finding below)
  }
  const PortPlan pin = build_port_plan(fifty, m, PT_D2A_PIN);
  // One-way boards without a UID (all zero, no return path) are never selected or confirmed, so the job admits
  // several of them
  {
    PortInventory two;
    two.nodes = {node(0, V203_T, false), node(1, V203_T, false)};
    const PortPlan tp = build_port_plan(two, m, PT_D2A_PIN);
    line_for(2, PT_D2A_PIN);
    Glue gt;
    const int rc = gt.begin(two, tp, images, now);
    known(!tp.blocked && rc == 0, "sender", "two one-way boards without a UID are admitted (job_begin %d)", rc);
    if (rc == 0)
      ultra_update_job_abort(&gt.job);
  }
  line_for(50, PT_D2A_PIN, 3);   // (the pin's speed table: 200 kHz for 35..50 boards)
  Glue g50;
  const int rc50 = g50.begin(fifty, pin, images, now);
  check(!pin.blocked && rc50 == 0 && g50.results[49].result == PRR_PENDING,
        "fifty boards on a Dig2Analog-Plus-5ch pin at 200 kHz: the plan and the job take them (job_begin %d)", rc50);
  ultra_update_job_abort(&g50.job);
  line_for(50, PT_D2A_PIN, 0);
  Glue g50fast;
  const int rc_fast = g50fast.begin(fifty, pin, images, now);
  check(rc_fast == ULTRA_UPDATE_LINE_TOO_FAST &&
            port_refusal_text(PRF_JOB, static_cast<int16_t>(rc_fast)) ==
                "the line runs faster than the pin's speed table allows for its boards",
        "... the same pin still at 800 kHz: the job refuses it, and says why (%d)", rc_fast);
  // admitted at 200 kHz, then the line runs at 800 kHz when the subtransfer begins: the sender refuses it there and
  // keeps the code (begin_code); the report names it, the image's nodes "refused", never "failed: transport"
  {
    line_for(50, PT_D2A_PIN, 3);
    Glue gr;
    const int rc = gr.begin(fifty, pin, images, now);
    line.timing.profile = 0;
    int n = 0;
    for (int k = 0; k < 10 && gr.job.state != ULTRA_JOB_DONE; k++)
      n = ultra_update_job_next_frame(&gr.job, now, frame, sizeof(frame));
    PortReport rr = port_report_from_plan(pin, &fifty);
    port_job_take_results(rr, gr.results.data(), gr.results.size());
    rr.job_rc = static_cast<int16_t>(gr.job.begin_code);
    port_report_close(rr, PJO_DONE);
    const std::string text = port_report_text(rr);
    check(rc == 0 && n == ULTRA_UPDATE_LINE_TOO_FAST && gr.job.begin_code == ULTRA_UPDATE_LINE_TOO_FAST &&
              gr.job.state == ULTRA_JOB_DONE && rr.nodes[0].result == PRR_FAILED && rr.nodes[0].reason == PRQ_REFUSED &&
              rr.nodes[49].reason == PRQ_REFUSED,
          "a subtransfer refused at its begin: its nodes REFUSED, the code kept (%d, %d, %u)", n, gr.job.begin_code,
          rr.nodes[0].reason);
    check(text.find("; image refused: the line runs faster than the pin's speed table allows for its boards") !=
                  std::string::npos &&
              text.find("\nB1-50 refused 0.7.1") != std::string::npos && text.find("transport") == std::string::npos,
          "... and the report names the reason (%s)", text.c_str());
    PortReportStore st;
    port_report_store(rr, st);
    PortReport back;
    check(port_report_load(st, back) && back.job_rc == ULTRA_UPDATE_LINE_TOO_FAST && back.nodes[3].reason == PRQ_REFUSED &&
              port_report_text(back) == text,
          "... also after a reboot (stored)");
  }
  check(port_refusal_text(PRF_JOB, -UPD_ERROR_BOUNDS) ==
            "an image's timing is out of bounds, e.g. its erase lead is longer than the forwarders' transit hold "
            "(UPD_ERROR_BOUNDS)",
        "a plan job_begin refuses for its bounds (a lead longer than the transit hold): named");
  line_for(50, PT_D2A_PIN);
  const PortPlan pin_dp = build_port_plan(fifty, m, PT_DIFF_POWER);
  forced = pin_dp;
  forced.blocked = false;
  Glue g50dp;
  check(pin_dp.blocked && g50dp.begin(fifty, forced, images, now) == ULTRA_UPDATE_TOO_MANY_BOARDS,
        "... the same fifty as a Diff-Power port: both refuse");
}

// ------------------------------------------------------------------------------------------------ driver + verifier
/// The hub as the driver sees it: a clock it advances, a line that is always free, the frames and leases it got;
/// hold_each: every take of the line refused this often first (one of the hub's response windows still open)
struct FakeTransport : PortJobDriver::Transport {
  int64_t clock{0};
  int frames{0}, refuse{0}, leases{0}, releases{0};
  int hold_each{0}, holding{0}, held{0};
  bool leased{false}, sent_unleased{false};
  std::vector<uint8_t> types;   // the first byte of every 48-byte UPD frame
  int64_t now_us() override { return this->clock; }
  bool line_free() override { return true; }
  bool send(const uint8_t *f, size_t n, const ultra_update_transport_t &) override {
    if (this->refuse > 0) {
      this->refuse--;
      return false;
    }
    if (!this->leased)
      this->sent_unleased = true;
    this->frames++;
    if (n == UPD_FRAME_BYTES)
      this->types.push_back(f[0]);
    this->clock += 1000;   // its bits on the wire
    return true;
  }
  bool lease(bool on) override {
    if (on && !this->leased && this->holding < this->hold_each) {
      this->holding++;
      this->held++;
      return false;
    }
    this->holding = 0;
    this->leased = on;
    (on ? this->leases : this->releases)++;
    return true;
  }
};
/// The receivers' answers as the hub's poller reads them
struct VerifyPoller : TelemetryPoller {
  void answer(uint16_t i, const PortNode &n, const uint8_t *descriptor, uint32_t now_ms, uint8_t index = 0) {
    auto &r = this->rx_[i];
    auto &d = r.d;
    r.answering = true;
    d.have_identity = d.have_update = true;
    std::memcpy(d.uid, n.uid, 12);
    std::memcpy(d.target, n.target.bytes, 16);
    std::memcpy(d.geometry, n.geometry, 3);
    d.entry_10ms = n.entry_10ms;
    d.bl_version = n.boot_version;
    d.hw_id = n.target.bytes[UPD_T_BOARD];
    d.hw_rev = n.target.bytes[UPD_T_REVISION];
    d.wire = ULTRA_VERSION;
    d.fw_word = ultra_get32(descriptor + UPD_D_FW_VERSION);
    d.identity_ms = d.update_ms = now_ms;
    uint8_t *a = d.update_answer;
    std::memset(a, 0, 32);
    a[0] = 0xA5;
    a[1] = ULTRA_PAGE_FW_UPDATE;
    a[2] = uint8_t(index << 4);
    d.update_index = index;
    a[3] = 0x42;
    ultra_put16(a + 4, ultra_crc16(n.uid, 12));
    a[6] = 1;
    a[7] = ULTRA_FU_F_VERIFIED | ULTRA_FU_F_CONFIRMED;
    ultra_put32(a + 10, ultra_get32(descriptor + UPD_D_FW_VERSION));
    ultra_put32(a + 14, ultra_get32(descriptor + UPD_D_BODY_LEN));
    ultra_put32(a + 18, ultra_get32(descriptor + UPD_D_BODY_CRC));
    ultra_put32(a + 22, ultra_get32(descriptor + UPD_D_CRC));
    ultra_put16(a + 30, ultra_crc16(a, 30));
  }
};

/// A mixed port end to end through the driver: a V407 receiver, a V203 receiver, a one-way V203 board, a current
/// V407: the V407 image first, then the V203's; the line leased for each subtransfer and handed back for each
/// verification, where the receivers' fresh identity and FW_UPDATE confirm them.
static void test_driver() {
  const uint32_t V070 = 0x000700FF, V071 = 0x000701FF;
  PortInventory inv;
  inv.nodes = {node(0, V407_T, true), node(1, V203_T, true), node(2, V203_T, false), node(3, V407_T, true, V071)};
  inv.nodes[3].uid[1] = 9;   // (another V407 receiver, already on the release)
  std::vector<Image> images = {image_for(V203_T, V071, 512), image_for(V407_T, V071, 768)};
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {info_for(images[0], V203_T, 'a'), info_for(images[1], V407_T, 'b')};
  m.layouts = {layout_for(V203_T), layout_for(V407_T)};
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "driver: the release checks");
  const PortPlan plan = build_port_plan(inv, m, PT_DIFF_POWER);
  check(!plan.blocked && plan.artifacts.size() == 2 && plan.artifacts[0].family == UPD_MCU_V407, "driver: the plan");
  line_for(4, PT_DIFF_POWER);
  Glue g;
  FakeTransport t;
  t.clock = 1000000;
  t.hold_each = 3;   // (each take: the hub reads its open response window first)
  check(g.begin(inv, plan, images, t.clock) == 0, "driver: the job begins");
  g.options.verify_timeout_ms = PORT_JOB_VERIFY_MS;
  g.job.options.verify_timeout_ms = PORT_JOB_VERIFY_MS;
  VerifyPoller p;
  p.init(4, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  PortJobDriver driver;
  static uint8_t buffer[1024];
  driver.begin(&g.job, &t, buffer, sizeof(buffer));
  int verifies = 0, confirmed = 0;
  uint32_t verify_ms = 0;
  bool verifying = false;
  PortJobDriver::Step s = PortJobDriver::WAIT;
  for (int k = 0; k < 3000000 && s != PortJobDriver::DONE; k++) {
    s = driver.step();
    if (s == PortJobDriver::WAIT) {
      t.clock = std::max<int64_t>(t.clock + 100, driver.due_us());
    } else if (s == PortJobDriver::VERIFYING) {
      check(!t.leased, "driver: verifying, the line is the hub's");
      const uint32_t now_ms = uint32_t(t.clock / 1000);
      if (!verifying) {
        verifying = true;
        verifies++;
        verify_ms = now_ms;
        t.clock += 200000;   // (the resync, the lock, the polls)
        continue;
      }
      // the receivers on the active image answer again: identity and FW_UPDATE, read after the job asked
      const uint16_t active = g.job.active_artifact;
      const uint8_t *desc = g.artifacts[active].descriptor.bytes;
      for (uint16_t n = 0; n < 4; n++)
        if (g.results[n].artifact == active && inv.nodes[n].live && g.results[n].transfer_complete)
          p.answer(n, inv.nodes[n], desc, now_ms);
      confirmed += port_job_verify(driver, p, verify_ms, t.clock, now_ms);
      t.clock += 10000;
    } else {
      verifying = false;
    }
  }
  check(s == PortJobDriver::DONE && g.job.state == ULTRA_JOB_DONE && !t.leased, "driver: done, the line handed back");
  check(verifies == 2 && t.leases == 2 && t.releases == 2 && !t.sent_unleased,
        "driver: two subtransfers, each on a leased line, each verified on the hub's (%d verifies, %d leases, %d releases)",
        verifies, t.leases, t.releases);
  check(t.held == 6 && !t.sent_unleased,
        "driver: a take the hub doesn't hand over yet (its response window) waits: no frame before it (%d waits)", t.held);
  check(confirmed == 2 && g.results[0].result == PRR_VERIFIED && g.results[1].result == PRR_VERIFIED &&
            g.results[2].result == PRR_SENT_UNVERIFIED && g.results[3].result == PRR_CURRENT,
        "driver: both receivers verified, the one-way board sent, unverified, the current one untouched (%u %u %u %u)",
        g.results[0].result, g.results[1].result, g.results[2].result, g.results[3].result);
  check(g.results[0].observed == V071 && g.results[1].before == V070, "driver: observed and before versions");
  size_t announces = 0, data = 0, ends = 0;
  for (uint8_t ty : t.types) {
    announces += ty == UPD_ANNOUNCE;
    data += ty == UPD_DATA;
    ends += ty == UPD_END;
  }
  // 768 + 512 bytes: 24 + 16 chunks, three passes each
  check(data == 3 * (24 + 16) && announces >= 8 && ends >= 2, "driver: three complete DATA passes per image (%u DATA)",
        unsigned(data));
  PortReport rep = port_report_from_plan(plan, &inv);
  port_job_take_results(rep, g.results.data(), g.results.size());
  rep.outcome = PJO_DONE;
  check(port_report_text(rep).find("Done 0.7.1: 2 verified, 1 unverified, 0 failed") == 0, "driver: the report (%s)",
        port_report_text(rep).c_str());

  // fresh evidence only: an answer read before the job asked does not confirm
  {
    line_for(4, PT_DIFF_POWER);
    Glue g2;
    FakeTransport t2;
    t2.clock = 1000000;
    check(g2.begin(inv, plan, images, t2.clock) == 0, "driver (stale): begins");
    PortJobDriver d2;
    d2.begin(&g2.job, &t2, buffer, sizeof(buffer));
    VerifyPoller p2;
    p2.init(4, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    PortJobDriver::Step s2 = PortJobDriver::WAIT;
    int stale = 0;
    for (int k = 0; k < 3000000 && s2 != PortJobDriver::DONE; k++) {
      s2 = d2.step();
      if (s2 == PortJobDriver::WAIT) {
        t2.clock = std::max<int64_t>(t2.clock + 100, d2.due_us());
      } else if (s2 == PortJobDriver::VERIFYING) {
        const uint32_t now_ms = uint32_t(t2.clock / 1000);
        const uint8_t *desc = g2.artifacts[g2.job.active_artifact].descriptor.bytes;
        p2.answer(0, inv.nodes[0], desc, now_ms - 50000);   // read before the job asked
        stale += port_job_verify(d2, p2, now_ms, t2.clock, now_ms);
        t2.clock += 100000;
      }
    }
    check(stale == 0 && g2.results[0].result == PRR_UNREACHABLE,
          "driver: a stale answer never confirms; without fresh evidence the receiver is unreachable");
  }

  // a receiver that answers in a window's later slot (INDEX 3)
  {
    line_for(4, PT_DIFF_POWER);
    Glue g3;
    FakeTransport t3;
    t3.clock = 1000000;
    g3.begin(inv, plan, images, t3.clock);
    PortJobDriver d3;
    d3.begin(&g3.job, &t3, buffer, sizeof(buffer));
    VerifyPoller p3;
    p3.init(4, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    PortJobDriver::Step s3 = PortJobDriver::WAIT;
    int given = 0;
    uint32_t asked = 0;
    for (int k = 0; k < 3000000 && s3 != PortJobDriver::DONE; k++) {
      s3 = d3.step();
      if (s3 == PortJobDriver::WAIT) {
        t3.clock = std::max<int64_t>(t3.clock + 100, d3.due_us());
      } else if (s3 == PortJobDriver::VERIFYING) {
        const uint32_t now_ms = uint32_t(t3.clock / 1000);
        if (!asked)
          asked = now_ms;
        const uint16_t active = g3.job.active_artifact;
        for (uint16_t n = 0; n < 4; n++)
          if (g3.results[n].artifact == active && inv.nodes[n].live && g3.results[n].transfer_complete)
            p3.answer(n, inv.nodes[n], g3.artifacts[active].descriptor.bytes, now_ms + 1, 3);
        given += port_job_verify(d3, p3, asked, t3.clock + 1000, now_ms + 1);
        t3.clock += 100000;
      } else {
        asked = 0;
      }
    }
    check(given == 2 && g3.results[0].result == PRR_VERIFIED,
          "an FW_UPDATE answer from a window's later slot confirms (%d confirmed)", given);
  }

  // the transmitter refuses frames: after REFUSALS_MAX in a row the job aborts, and still ends
  {
    line_for(4, PT_DIFF_POWER);
    Glue g4;
    FakeTransport t4;
    t4.clock = 1000000;
    g4.begin(inv, plan, images, t4.clock);
    PortJobDriver d4;
    d4.begin(&g4.job, &t4, buffer, sizeof(buffer));
    t4.refuse = 1000000;
    PortJobDriver::Step s4 = PortJobDriver::WAIT;
    for (int k = 0; k < 3000000 && s4 != PortJobDriver::DONE; k++) {
      s4 = d4.step();
      if (s4 == PortJobDriver::WAIT || s4 == PortJobDriver::VERIFYING)
        t4.clock = std::max<int64_t>(t4.clock + 100, d4.due_us());
    }
    check(s4 == PortJobDriver::DONE && d4.transport_failed() && !t4.leased && g4.results[0].result == PRR_FAILED,
          "driver: a transmitter that keeps refusing aborts the job (%u/%u)", g4.results[0].result, g4.results[0].reason);
  }
}

/// The line's start profile: the sender helper's answer for the port type and configured board count
/// (ultra_port_line_profile), line_rate initial_profile only slower; every table profile inside the pin's link model
static void test_start_profile() {
  struct Case {
    uint8_t port;
    uint16_t boards;
    uint8_t profile, why;
  };
  static const Case CASES[] = {
      {PT_DIFF_POWER, 1, 0, LCW_DEFAULT},    {PT_DIFF_POWER, 8, 0, LCW_DEFAULT},    {PT_DIFF_POWER, 9, 0, LCW_TOO_MANY},
      {PT_D2A_PIN, 1, 0, LCW_DEFAULT},       {PT_D2A_PIN, 20, 0, LCW_DEFAULT},      {PT_D2A_PIN, 21, 1, LCW_BOARD_COUNT},
      {PT_D2A_PIN, 34, 1, LCW_BOARD_COUNT},  {PT_D2A_PIN, 35, 3, LCW_BOARD_COUNT},  {PT_D2A_PIN, 50, 3, LCW_BOARD_COUNT},
      {PT_D2A_PIN, 51, 0, LCW_TOO_MANY},     {PT_D2A_PIN, 0, 0, LCW_NO_BOARDS},     {PT_UNKNOWN, 4, 0, LCW_UNKNOWN_TYPE},
  };
  for (const auto &c : CASES) {
    uint8_t table = 0;
    const uint8_t why = ultra_port_line_profile(c.port, c.boards, &table);
    const LineChoice got = choose_line_profile(why, table, 0);
    check(got.profile == c.profile && got.why == c.why && line_choice_refused(got) == (table > 3),
          "start profile: port type %u, %u boards: profile %u (%u), want %u (%u)", c.port, c.boards, got.profile, got.why,
          c.profile, c.why);
  }
  uint8_t table = 0;
  uint8_t why = ultra_port_line_profile(PT_D2A_PIN, 21, &table);
  const LineChoice slow = choose_line_profile(why, table, 3);
  check(slow.profile == 3 && slow.why == LCW_CONFIGURED, "start profile: initial_profile slows a 21-board pin to 200 kHz");
  why = ultra_port_line_profile(PT_D2A_PIN, 35, &table);
  const LineChoice fast = choose_line_profile(why, table, 1);
  check(fast.profile == 3 && fast.why == LCW_BOARD_COUNT, "start profile: initial_profile never speeds up a 35-board pin");
  check(line_choice_text(PT_D2A_PIN, 35, fast) == "Dig2Analog-Plus-5ch pin, 35 boards: the speed table for its board count",
        "start profile: the table's reason");
  why = ultra_port_line_profile(PT_D2A_PIN, 51, &table);
  check(line_choice_text(PT_D2A_PIN, 51, choose_line_profile(why, table, 0)) ==
            "Dig2Analog-Plus-5ch pin, 51 boards: refused: more boards than the port type chains",
        "start profile: the refusal's reason");
  why = ultra_port_line_profile(PT_DIFF_POWER, 1, &table);
  check(line_choice_text(PT_DIFF_POWER, 1, choose_line_profile(why, table, 0)) == "Diff-Power port, 1 board: the default",
        "start profile: a Diff-Power port's reason");
  int outside = 0;
  for (uint8_t boards = 1; boards <= PORT_D2A_PIN_MAX_BOARDS; boards++) {
    ultra_port_line_profile(PT_D2A_PIN, boards, &table);
    outside += !ultra_d2a_admits(table, boards, 0);
  }
  check(outside == 0, "start profile: every pin table profile inside the link model (%d outside)", outside);
  std::printf("  start profile: Diff-Power 800 kHz, pin 1..20 / 21..34 / 35..50 boards 800 / 600 / 200 kHz, more refused\n");
}

/// A receiver that answers nothing now, with the ledger's last live identity (port_inventory.h, remembered)
static PortNode silent(PortNode n) {
  n.state = PNS_UNREACHABLE;
  n.issue = PNI_SILENT;
  n.live = false;
  n.remembered = true;
  return n;
}

/// One job through the driver: the line's live receivers answer each verification (and the silent one when
/// found_again); returns the report
static PortReport run_recovery(const PortInventory &inv, const PortPlan &plan, const std::vector<Image> &images,
                               bool found_again, Glue &g, int *data_frames) {
  line_for(static_cast<uint16_t>(inv.nodes.size()), PT_DIFF_POWER);
  FakeTransport t;
  t.clock = 1000000;
  check(g.begin(inv, plan, images, t.clock) == 0, "recovery: the job begins");
  g.job.options.verify_timeout_ms = PORT_JOB_VERIFY_MS;
  VerifyPoller p;
  p.init(static_cast<uint16_t>(inv.nodes.size()), ULTRA_RW_RATE_1M, TelemetryIntervals{});
  PortJobDriver driver;
  static uint8_t buffer[1024];
  driver.begin(&g.job, &t, buffer, sizeof(buffer));
  uint32_t verify_ms = 0;
  bool verifying = false;
  PortJobDriver::Step s = PortJobDriver::WAIT;
  for (int k = 0; k < 3000000 && s != PortJobDriver::DONE; k++) {
    s = driver.step();
    if (s == PortJobDriver::WAIT) {
      t.clock = std::max<int64_t>(t.clock + 100, driver.due_us());
    } else if (s == PortJobDriver::VERIFYING) {
      const uint32_t now_ms = uint32_t(t.clock / 1000);
      if (!verifying) {
        verifying = true;
        verify_ms = now_ms;
        t.clock += 200000;
        continue;
      }
      const uint16_t active = g.job.active_artifact;
      const uint8_t *desc = g.artifacts[active].descriptor.bytes;
      for (uint16_t n = 0; n < inv.nodes.size(); n++)
        if (g.results[n].artifact == active && g.results[n].transfer_complete && (inv.nodes[n].live || found_again))
          p.answer(n, inv.nodes[n], desc, now_ms);
      port_job_verify(driver, p, verify_ms, t.clock, now_ms);
      t.clock += 10000;
    } else {
      verifying = false;
    }
  }
  check(s == PortJobDriver::DONE && g.job.state == ULTRA_JOB_DONE && !t.leased, "recovery: done, the line handed back");
  *data_frames = 0;
  for (uint8_t ty : t.types)
    *data_frames += ty == UPD_DATA;
  PortReport rep = port_report_from_plan(plan, &inv);
  port_job_take_results(rep, g.results.data(), g.results.size());
  port_report_close(rep, PJO_DONE);
  return rep;
}

/// A known receiver that doesn't answer means something is wrong on the port, so
/// "update all" refuses (the plan names every silent one; the sender's job_begin agrees, ULTRA_UPDATE_SILENT_BOARDS,
/// every result written). Its Recover firmware button is the explicit action (ultra_update_job_recover): its image
/// only, blind with three passes; verified when it is found again, else "recovery sent"; refused for a board that
/// answers or already runs the release; another board type is not part of it.
static void test_recovery() {
  const uint32_t V070 = 0x000700FF, V071 = 0x000701FF;
  std::vector<Image> images = {image_for(V203_T, V071, 512), image_for(V407_T, V071, 512)};
  FwManifest m;
  m.channel = "stable";
  m.version = "0.7.1";
  m.images = {info_for(images[0], V203_T, 'a'), info_for(images[1], V407_T, 'b')};
  m.layouts = {layout_for(V203_T), layout_for(V407_T)};
  check(fw_check_manifest(m, FW_CH_STABLE) == nullptr, "recovery: the release checks");
  PortInventory inv;
  inv.nodes = {node(0, V203_T, true, V071), silent(node(1, V203_T, true, V070)), node(2, V407_T, true, V070)};
  inv.nodes[1].uid[1] = 7;
  inv.nodes[2].uid[1] = 8;
  // "update all": refused before any frame, the silent board named; the sender agrees
  const PortPlan all = build_port_plan(inv, m, PT_DIFF_POWER);
  check(all.blocked && all.block_reason == PRS_SILENT && port_blocked_text(all) == "B2 not answering: check power and cable",
        "recovery: \"update all\" refuses past the silent board (%s)", port_blocked_text(all).c_str());
  line_for(3, PT_DIFF_POWER);
  Glue g0;
  const int rc0 = g0.begin(inv, all, images, 1000000);
  check(rc0 == ULTRA_UPDATE_SILENT_BOARDS && g0.results[1].result == PRR_UNREACHABLE &&
            g0.results[1].reason == PRQ_NOT_ANSWERING && g0.results[0].result == PRR_SKIPPED &&
            g0.results[0].reason == PRQ_PORT_BLOCKED && g0.results[2].reason == PRQ_PORT_BLOCKED,
        "recovery: ... and so does the sender, every result written (%d: %u/%u, %u/%u)", rc0, g0.results[1].result,
        g0.results[1].reason, g0.results[0].result, g0.results[0].reason);
  PortReport blocked = port_report_from_plan(all, &inv);
  port_job_take_results(blocked, g0.results.data(), g0.results.size());
  port_report_close(blocked, PJO_BLOCKED);
  std::string text = port_report_text(blocked);
  check(text.find("Not installed 0.7.1: B2 not answering: check power and cable, nothing sent") == 0 &&
            text.find("\nB1 not sent: the port is blocked") != std::string::npos,
        "recovery: the refusal's report (%s)", text.c_str());
  // B1's Recover firmware: its V203 image only (B0 runs it already: current); the V407 board is not part of it
  const PortPlan plan = build_recover_plan(inv, m, PT_DIFF_POWER, 1);
  check(!plan.blocked && plan.nodes[1].action == PA_SEND && plan.nodes[1].expect == PE_RECOVERY &&
            plan.nodes[0].action == PA_CURRENT && plan.nodes[2].action == PA_SKIP &&
            plan.nodes[2].reason == PRS_OTHER_TARGET,
        "recovery: the plan: the silent board's image only, the other board type not part of it");
  Glue g;
  int data = 0;
  PortReport rep = run_recovery(inv, plan, images, false, g, &data);
  check(g.results[1].recovery && g.results[1].result == PRR_UNREACHABLE && g.results[1].reason == PRQ_RECOVERY_SENT &&
            rep.nodes[1].sent && rep.nodes[0].result == PRR_CURRENT && g.results[2].result == PRR_SKIPPED &&
            g.results[2].reason == PRQ_OTHER_TARGET && data == 3 * 16,
        "recovery: not found again: unreachable, recovery sent, three blind passes; the V407 not part of it (%u/%u, "
        "%u/%u, %d DATA)", g.results[1].result, g.results[1].reason, g.results[2].result, g.results[2].reason, data);
  text = port_report_text(rep);
  check(text.find("Recovery of B2: recovery sent 0.7.1, not found again") == 0 &&
            text.find("\nB3 not part of this recovery") != std::string::npos,
        "recovery: the report (%s)", text.c_str());
  Glue g2;
  rep = run_recovery(inv, plan, images, true, g2, &data);
  check(g2.results[1].result == PRR_VERIFIED && g2.results[1].observed == V071 &&
            port_report_text(rep).find("Recovery of B2: verified 0.7.1") == 0,
        "recovery: found again with the image: verified (%u/%u)", g2.results[1].result, g2.results[1].reason);
  // refusals: a board that answers; a silent one already on the release (the plan and the sender agree)
  const PortPlan answers = build_recover_plan(inv, m, PT_DIFF_POWER, 0);
  check(answers.blocked && answers.block_reason == PRS_ANSWERS, "recovery: a board that answers: nothing to recover");
  line_for(3, PT_DIFF_POWER);
  Glue g3;
  PortPlan forced = answers;
  forced.blocked = false;
  check(g3.begin(inv, forced, images, 1000000) == ULTRA_UPDATE_NOT_RECOVERABLE, "recovery: ... the sender refuses too");
  inv.nodes[1].firmware = V071;
  const PortPlan same = build_recover_plan(inv, m, PT_DIFF_POWER, 1);
  check(same.blocked && same.block_reason == PRS_NOT_OLDER, "recovery: a silent board on the release: nothing to recover");
  Glue g4;
  forced = same;
  forced.blocked = false;
  check(g4.begin(inv, forced, images, 1000000) == ULTRA_UPDATE_NOT_RECOVERABLE, "recovery: ... the sender refuses too");
}

#include "port_job_threads.inc"

int main() {
  test_glue();
  test_driver();
  test_recovery();
  test_start_profile();
  threaded::test_threaded_handover();
  std::printf("port job glue against the sender job: %d checks, %d failed, %d pending\n", checks, failures, pending);
  return failures ? 1 : 0;
}
