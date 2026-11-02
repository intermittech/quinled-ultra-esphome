#pragma once

// This component's side of the sender library's port job (the protocol's sender/src/ultra_sender_job.h): what it hands
// the job and takes back. The port's ledger and plan (port_inventory.h, port_plan.h) become the job's inventory (one
// node per position with its trusted target, layout and timing facts) and its artifacts (one per deduplicated image,
// queued family by family by the job); the job's per-node results and progress come back into the report
// (port_report.h) and the update entity. The functions are templates over the job's own types, so no copy of them can
// drift: they are instantiated where the sender job is included, and tests/port_job_test.cpp checks them against it.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include "fw_version.h"
#include "port_inventory.h"
#include "port_plan.h"
#include "port_report.h"

namespace esphome::ultra_line {

/// Every blind subtransfer sends its image in this many complete DATA passes, for every MCU family (reliability over
/// speed, as in 0.2.x); up to about a minute per family broadcast is fine. Not a YAML option.
static constexpr uint8_t PORT_JOB_PASSES = 3;
/// The job's blind transfer mode (the sender's ULTRA_UPDATE_BLIND): no SELECT, no status reply needed
static constexpr uint8_t PORT_JOB_BLIND = 0;

/// The job's node for one position: its trusted target and layout, its version before, its timing facts and whether
/// the job can expect fresh evidence from it. now_us / now_ms: one instant on the job's clock and on millis().
template<typename Node> void port_job_node(Node &out, const PortNode &n, const PortPlanNode &p, int64_t now_us,
                                           uint32_t now_ms) {
  std::memset(&out, 0, sizeof(out));
  std::memcpy(out.participant.target.bytes, n.target.bytes, UPD_TARGET_BYTES);
  out.participant.layout.metadata_base = p.metadata_base;
  out.participant.layout.metadata_bytes = p.metadata_bytes;
  out.participant.layout.bootloader_bytes = p.bootloader_bytes;
  out.participant.layout.boot_version = n.boot_version;
  std::memcpy(out.participant.layout.geometry, n.geometry, UPD_GEOMETRY_BYTES);
  out.participant.layout.entry_10ms = n.entry_10ms;
  if (n.uid_known)
    std::memcpy(out.participant.uid, n.uid, 12);
  // only a node whose installed image can be confirmed here waits for evidence (a live node without the FW_UPDATE
  // page ends "sent, unverified", like a one-way one); a silent receiver stays one that should answer
  out.participant.has_return = n.has_return && (!n.live || n.fw_update_page);
  out.firmware = n.firmware_known ? n.firmware : 0;
  out.blackout_ms = p.blackout_ms;
  out.min_gap_us = p.gap_us;
  out.evidence_us = n.live ? now_us - int64_t(uint32_t(now_ms - n.evidence_ms)) * 1000 : 0;
  out.trusted = n.target_known && p.blackout_known && n.state != PNS_MISMATCH;
  out.reachable = n.state == PNS_REACHABLE;
}

/// The job's options for a plan: blind, PORT_JOB_PASSES, the port type's board limit, V407 then V203 (the default
/// order), the default verification time
template<typename Options> void port_job_options(Options &out, const PortPlan &plan, uint8_t first_session,
                                                 uint8_t return_rate, int64_t deadline_us = 0) {
  std::memset(&out, 0, sizeof(out));
  out.mode = PORT_JOB_BLIND;
  out.passes = PORT_JOB_PASSES;
  out.rate = return_rate;
  out.first_session = first_session ? first_session : 1;
  out.port_type = plan.port_type;
  out.family_order[0] = PORT_FAMILY_ORDER[0];
  out.family_order[1] = PORT_FAMILY_ORDER[1];
  out.verify_timeout_ms = 0;
  out.deadline_us = deadline_us;
}

/// One artifact: its signed descriptor (the manifest's, before any body is downloaded) and its source hooks
template<typename Artifact, typename Read, typename Prepare, typename Release>
void port_job_artifact(Artifact &out, const uint8_t *descriptor, void *context, Read read, Prepare prepare,
                       Release release) {
  std::memset(&out, 0, sizeof(out));
  std::memcpy(out.descriptor.bytes, descriptor, UPD_DESC_BYTES);
  out.read = read;
  out.context = context;
  out.authenticated = 1;   // the descriptor came with the manifest whose signature verified
  out.prepare = prepare;
  out.release = release;
}

/// The job's results into the report's records (index = position); the plan's finer reasons stay where the job's
/// result is the plan's (a skipped node keeps "needs bootloader 4")
template<typename Result> void port_job_take_results(PortReport &rep, const Result *results, size_t n) {
  for (size_t k = 0; k < n && k < rep.nodes.size(); k++) {
    const Result &j = results[k];
    PortNodeResult &r = rep.nodes[k];
    const bool same = r.result == j.result && r.reason == j.reason;
    r.result = j.result;
    r.reason = j.reason;
    r.sent = j.transfer_complete != 0;
    r.observed = j.observed;
    r.observed_known = j.result == PRR_VERIFIED || j.result == PRR_CURRENT ||
                       (j.result == PRR_FAILED && j.reason == PRQ_MISMATCH);
    if (j.expected)
      r.expected = j.expected;
    if (!same && r.result != PRR_SKIPPED)
      r.detail = PRS_NONE;
  }
}

/// The job's progress, as the entity shows it. pass: the DATA pass now running, counted from 1 by the job's progress
/// (0 = no transfer yet; never above passes)
struct PortJobProgress {
  uint8_t family{0}, phase{0}, pass{0}, passes{0};
  uint16_t subtransfer{0}, subtransfers{0};
  uint32_t bytes_sent{0}, bytes_total{0};
  uint64_t elapsed_ms{0}, remaining_ms{0};
  /// 0..1 over the whole job: the subtransfers before this one, then this one's bytes
  float fraction() const {
    if (this->subtransfers == 0)
      return 0.0f;
    const float part = this->bytes_total ? float(this->bytes_sent) / float(this->bytes_total) : 0.0f;
    return (float(this->subtransfer > 0 ? this->subtransfer - 1 : 0) + part) / float(this->subtransfers);
  }
};
template<typename Progress> PortJobProgress port_job_progress(const Progress &p) {
  PortJobProgress out;
  out.family = p.family;
  out.phase = p.phase;
  out.pass = p.pass;
  out.passes = p.passes;
  out.subtransfer = p.subtransfer;
  out.subtransfers = p.subtransfers;
  out.bytes_sent = p.bytes_sent;
  out.bytes_total = p.bytes_total;
  out.elapsed_ms = p.elapsed_ms;
  out.remaining_ms = p.estimated_remaining_ms;
  return out;
}
/// "image 1 of 3: pass 2 of 3, 40 %, about 50 s left"
inline std::string port_job_progress_text(const PortJobProgress &p) {
  std::string s = "image " + std::to_string(p.subtransfer) + " of " + std::to_string(p.subtransfers);
  if (p.passes && p.pass)   // (p.pass already counts from 1: adding one here showed "pass 4 of 3" on the bench)
    s += ": pass " + std::to_string(std::min(p.pass, p.passes)) + " of " + std::to_string(p.passes);
  if (p.bytes_total)
    s += ", " + std::to_string(uint32_t(uint64_t(p.bytes_sent) * 100 / p.bytes_total)) + " %";
  if (p.remaining_ms)
    s += ", about " + std::to_string(uint32_t((p.remaining_ms + 999) / 1000)) + " s left";
  return s;
}

/// What the entity hears while a job runs (the transport task calls it on the main loop)
class PortJobListener {
 public:
  virtual ~PortJobListener() = default;
  virtual void job_progress(const PortJobProgress &progress) = 0;
  /// The job ended (done or aborted): every node's result
  virtual void job_finished(const PortReport &report) = 0;
};

}  // namespace esphome::ultra_line
