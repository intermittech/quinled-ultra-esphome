#pragma once

// The report of one "update all receivers on this port" job (protocol docs/update.md, Job results and Port job step
// 8): per node its version before, the version expected and the one observed, its result and a concrete reason. It is
// kept after the job finishes, is aborted or refused, stored on the ESP32 and shown in Home Assistant (fw_update.cpp).
// Transmission is never installation evidence: a node is "verified" only from the sender job's fresh per-node evidence
// (port_job.h); a node without a return path ends "sent, unverified" at best. Before a job, the same records show the
// plan (who gets what). No ESP-IDF / ESPHome dependencies: tests/port_update_test.cpp.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "fw_version.h"
#include "port_inventory.h"
#include "port_plan.h"

namespace esphome::ultra_line {

/// A node's result: the values of the sender port job's ULTRA_JOB_* (port_job.h checks them)
enum PortResult : uint8_t {
  PRR_PENDING = 0,      // planned: the job decides
  PRR_VERIFIED,         // fresh identity and FW_UPDATE confirm the expected image
  PRR_CURRENT,          // it ran the release's image already: none sent for it
  PRR_SKIPPED,          // incompatible, newer, or no image for it (reason)
  PRR_FAILED,           // receiver / transport failure, abort, or fresh telemetry that disagrees
  PRR_SENT_UNVERIFIED,  // sent and waited for, no return path to confirm it
  PRR_UNREACHABLE,      // it can't be inventoried or rediscovered: never success, never proof of absence
};
/// ... and its reason: the values of ULTRA_JOB_REASON_*
enum PortResultReason : uint8_t {
  PRQ_NONE = 0, PRQ_INCOMPATIBLE, PRQ_NEWER, PRQ_MISSING_ARTIFACT, PRQ_NO_RETURN, PRQ_NO_REPLY, PRQ_MISMATCH,
  PRQ_ABORT, PRQ_TRANSPORT,
  PRQ_REFUSED,   // the sender refused its subtransfer at begin (the report's job_rc says why)
  PRQ_RECOVERY_SENT,   // a recovery's board sent blind and not found again
  PRQ_NOT_ANSWERING,   // "update all" refused: this known receiver doesn't answer (power, cable)
  PRQ_PORT_BLOCKED,    // "update all" refused: another board doesn't answer, nothing sent
  PRQ_OTHER_TARGET,    // a recovery of another board type: not part of it
};
/// How the job ended
enum PortJobOutcome : uint8_t {
  PJO_NONE = 0,   // no job yet: a plan
  PJO_REFUSED,    // Install refused before anything was sent (refusal says why)
  PJO_BLOCKED,    // refused: the inventory is incomplete (block_position's reason); nothing sent
  PJO_ABORTED,
  PJO_DONE,
};

/// Why an Install was refused (PJO_REFUSED)
enum PortRefusal : uint8_t {
  PRF_NONE = 0,
  PRF_BUSY,     // another port job runs on this controller
  PRF_TIMING,   // the line's bit timing is not a protocol profile (the job stamps TIME from it)
  PRF_LINE,     // the sender line for the job can't be set up
  PRF_JOB,      // the sender job refused the plan (job_rc)
  PRF_TASK,     // the transport task can't start
  PRF_LEDGER,   // the receivers' identities (their recovery's) could not be written to the flash first
};
/// The sender job's refusals named in the text (its ULTRA_UPDATE_*; port_job_test.cpp checks the values)
static constexpr int16_t PORT_JOB_TOO_MANY_BOARDS = -64, PORT_JOB_OVERLAPPING_ARTIFACTS = -65,
                         PORT_JOB_TOO_FEW_PASSES = -66, PORT_JOB_LINE_TOO_FAST = -67, PORT_JOB_SILENT_BOARDS = -68,
                         PORT_JOB_NOT_RECOVERABLE = -69;
/// ... and its UPD_ERROR_* refusals (-code), at job_begin or at a subtransfer's begin
static constexpr int16_t PORT_JOB_TARGET = -2, PORT_JOB_BOUNDS = -3, PORT_JOB_CRC = -8, PORT_JOB_UNSUPPORTED = -10;
/// What the sender's refusal code means
inline std::string port_job_code_text(int16_t rc) {
  switch (rc) {
    case PORT_JOB_TOO_MANY_BOARDS: return "more boards than the port type chains";
    case PORT_JOB_OVERLAPPING_ARTIFACTS: return "two images of the release accept the same board";
    case PORT_JOB_TOO_FEW_PASSES: return "fewer than three full passes";
    case PORT_JOB_LINE_TOO_FAST: return "the line runs faster than the pin's speed table allows for its boards";
    case PORT_JOB_SILENT_BOARDS: return "a known board on the port doesn't answer";
    case PORT_JOB_NOT_RECOVERABLE: return "the board can't be recovered (it answers, or runs this release already)";
    case PORT_JOB_TARGET: return "two boards with the same identity (UPD_ERROR_TARGET)";
    case PORT_JOB_BOUNDS:
      return "an image's timing is out of bounds, e.g. its erase lead is longer than the forwarders' transit hold "
             "(UPD_ERROR_BOUNDS)";
    case PORT_JOB_CRC: return "an image's body doesn't match its signed CRC (UPD_ERROR_CRC)";
    case PORT_JOB_UNSUPPORTED: return "the line can't carry the update (UPD_ERROR_UNSUPPORTED)";
    default: return "the sender job refused it (" + std::to_string(rc) + ")";
  }
}
inline std::string port_refusal_text(uint8_t r, int16_t rc = 0) {
  switch (r) {
    case PRF_BUSY: return "another port is updating";
    case PRF_TIMING: return "the line's bit timing is not a protocol profile";
    case PRF_LINE: return "the update line can't be set up";
    case PRF_JOB: return port_job_code_text(rc);
    case PRF_TASK: return "the update task can't start";
    case PRF_LEDGER: return "the receivers' identities could not be saved on the controller's flash";
    default: return "refused";
  }
}

struct PortNodeResult {
  uint16_t position{0};
  uint8_t result{PRR_PENDING}, reason{PRQ_NONE};
  uint8_t detail{PRS_NONE};   // the plan's finer reason (PortReason), e.g. needs bootloader need_boot
  uint8_t need_boot{0};
  uint8_t expect{PE_NONE};    // the plan's best result for a node it sends to
  bool sent{false};           // its image was transmitted
  bool before_known{false}, observed_known{false};
  uint32_t before{0}, expected{0}, observed{0};
};

struct PortReport {
  PortJobOutcome outcome{PJO_NONE};
  uint32_t release{0};               // the release version
  uint32_t finished_s{0};            // unix time it ended (0: not known)
  uint16_t block_position{0xFFFF};   // a blocked plan's node (0xFFFF: none, or the whole port)
  uint8_t block_reason{PRS_NONE};
  uint8_t max_boards{0};             // the port type's board limit (PRS_TOO_MANY)
  uint8_t refusal{PRF_NONE};         // PJO_REFUSED: why
  int16_t job_rc{0};                 // the sender job's refusal: of the plan (PRF_JOB), or of a subtransfer at its
                                     // begin (PJO_DONE: its nodes PRQ_REFUSED, the untried ones not sent)
  int16_t recover{-1};               // a recovery's board (-1: "update all")
  std::vector<PortNodeResult> nodes;

  size_t count(uint8_t result) const {
    size_t n = 0;
    for (const auto &r : this->nodes)
      n += r.result == result;
    return n;
  }
};

/// The plan's records: nodes it sends to are pending, the others already have their result
inline PortReport port_report_from_plan(const PortPlan &plan, const PortInventory *inv = nullptr) {
  PortReport rep;
  rep.release = plan.version;
  rep.recover = static_cast<int16_t>(plan.recover);
  rep.max_boards = static_cast<uint8_t>(plan.max_boards);
  if (plan.blocked) {
    rep.block_position = plan.block_position >= 0 ? static_cast<uint16_t>(plan.block_position) : 0xFFFF;
    rep.block_reason = plan.block_reason;
  }
  rep.nodes.resize(plan.nodes.size());
  for (size_t k = 0; k < plan.nodes.size(); k++) {
    const PortPlanNode &p = plan.nodes[k];
    PortNodeResult &r = rep.nodes[k];
    r.position = p.position;
    r.before_known = p.before_known;
    r.before = p.before;
    r.expected = p.expected;
    r.detail = p.reason;
    r.need_boot = p.need_boot;
    r.expect = p.expect;
    switch (p.action) {
      case PA_SEND:
        r.result = PRR_PENDING;
        break;
      case PA_CURRENT:
        r.result = PRR_CURRENT;
        r.observed_known = p.before_known;
        r.observed = p.before;
        break;
      case PA_SKIP:
        r.result = PRR_SKIPPED;
        r.reason = p.reason == PRS_NEWER ? PRQ_NEWER
                   : p.reason == PRS_OTHER_TARGET ? PRQ_OTHER_TARGET
                   : p.reason == PRS_MISSING_ARTIFACT || p.reason == PRS_FAMILY || p.reason == PRS_AMBIGUOUS
                       ? PRQ_MISSING_ARTIFACT
                       : PRQ_INCOMPATIBLE;
        r.observed_known = p.before_known;
        r.observed = p.before;
        break;
      case PA_UNREACHABLE:
        r.result = PRR_UNREACHABLE;
        r.reason = p.blocks == PRS_SILENT ? PRQ_NOT_ANSWERING : PRQ_NO_REPLY;
        break;
      default: {  // no trusted target
        const PortNode *n = inv != nullptr && k < inv->nodes.size() ? &inv->nodes[k] : nullptr;
        if (p.reason == PRS_MISMATCH) {
          r.result = PRR_FAILED;
          r.reason = PRQ_MISMATCH;
        } else {
          r.result = PRR_UNREACHABLE;
          r.reason = n != nullptr && n->state == PNS_UNKNOWN ? PRQ_NO_RETURN : PRQ_NO_REPLY;
        }
      }
    }
  }
  return rep;
}

/// A job that ends before anything was sent (refused, blocked or aborted): every pending node failed, unsent
inline void port_report_close(PortReport &rep, PortJobOutcome outcome, uint32_t now_s = 0) {
  rep.outcome = outcome;
  rep.finished_s = now_s;
  for (auto &r : rep.nodes) {
    if (r.result != PRR_PENDING)
      continue;
    r.result = PRR_FAILED;
    r.reason = outcome == PJO_REFUSED ? PRQ_TRANSPORT : PRQ_ABORT;
    r.sent = false;
  }
}

/// The node still needs the release: pending in the plan, unless the last report (of this release) already shows it
/// sent where nothing can confirm it (a one-way node: its version is never observed)
inline bool port_needs_update(const PortNodeResult &plan, const PortReport *last) {
  if (plan.result != PRR_PENDING)
    return false;
  if (last == nullptr || plan.expect != PE_SENT_UNVERIFIED || last->release != plan.expected)
    return true;
  for (const auto &r : last->nodes)
    if (r.position == plan.position)
      return !(r.sent && r.expected == plan.expected && r.result == PRR_SENT_UNVERIFIED);
  return true;
}

static constexpr size_t PORT_REPORT_MAX = 250;   // (Home Assistant keeps 255 characters of a state)

inline std::string port_result_text_(const PortNodeResult &r, PortJobOutcome outcome, bool refused = false) {
  const std::string to = r.expected ? format_fw_version(r.expected) : std::string("?");
  switch (r.result) {
    case PRR_PENDING:
      return (r.before_known ? format_fw_version(r.before) : std::string("?")) + ">" + to +
             (r.expect == PE_VERIFIED ? " to verify" : r.expect == PE_RECOVERY ? " recovery" : " unverifiable");
    case PRR_VERIFIED: return "verified " + to;
    case PRR_CURRENT: return "current " + to;
    case PRR_SENT_UNVERIFIED: return "sent, unverified " + to;
    case PRR_SKIPPED:
      if (r.reason == PRQ_PORT_BLOCKED)
        return "not sent: the port is blocked";
      if (r.reason == PRQ_OTHER_TARGET)
        return "not part of this recovery";
      return "skipped: " + port_reason_text(static_cast<PortReason>(r.detail), r.need_boot);
    case PRR_UNREACHABLE:
      return r.reason == PRQ_NO_RETURN ? "unknown: not provisioned"
             : r.reason == PRQ_RECOVERY_SENT ? "recovery sent " + to + ", not found again"
             : r.reason == PRQ_NOT_ANSWERING ? "not answering: check power and cable"
                                             : "unreachable";
    default:
      if (r.reason == PRQ_REFUSED)
        return "refused " + to;
      if (!r.sent && (outcome == PJO_REFUSED || outcome == PJO_BLOCKED || outcome == PJO_ABORTED || refused))
        return "not sent " + to;
      return r.reason == PRQ_MISMATCH ? "failed: identity mismatch"
             : r.reason == PRQ_ABORT ? "failed: aborted"
             : r.reason == PRQ_TRANSPORT ? "failed: transport"
             : r.observed_known ? "failed: runs " + format_fw_version(r.observed) : std::string("failed");
  }
}

inline std::string port_block_text_(const PortReport &rep) {
  const std::string why = port_reason_text(static_cast<PortReason>(rep.block_reason), rep.max_boards);
  if (rep.block_reason == PRS_SILENT) {   // every receiver that doesn't answer, named
    std::string who;
    for (const auto &r : rep.nodes)
      if (r.result == PRR_UNREACHABLE && r.reason == PRQ_NOT_ANSWERING)
        who += (who.empty() ? "" : ", ") + port_board_text(r.position);
    if (!who.empty())
      return who + " " + why;
  }
  return rep.block_position != 0xFFFF && rep.block_reason != PRS_TOO_MANY
             ? port_board_text(rep.block_position) + " " + why
             : why;
}

/// One line of the report: its text and the positions it covers (none: the headline)
struct PortReportLine {
  std::string text;
  uint16_t first{0}, last{0};
  bool nodes{false};
};

/// The whole report, never cut: a headline, then every node with its result and reason, equal neighbours as a range
/// ("B1-4 verified 0.7.1"). Every one of a port's boards (8 on a Diff-Power port, 50 on a Dig2Analog-Plus-5ch pin) is in it.
inline std::vector<PortReportLine> port_report_lines(const PortReport &rep) {
  std::string head;
  const std::string release = rep.release ? format_fw_version(rep.release) : std::string("?");
  switch (rep.outcome) {
    case PJO_NONE: {
      size_t send = rep.count(PRR_PENDING);
      head = send ? "Plan " + release + ": " + std::to_string(send) + " to update" : "Plan " + release + ": nothing to send";
      if (rep.block_reason != PRS_NONE)
        head += ", blocked: " + port_block_text_(rep);
      break;
    }
    case PJO_REFUSED:
      head = (rep.recover >= 0 ? "Not recovered " + port_board_text(rep.recover) + " " : "Not installed ") + release +
             ": " + port_refusal_text(rep.refusal, rep.job_rc) + ", nothing sent";
      break;
    case PJO_BLOCKED:
      head = (rep.recover >= 0 ? "Not recovered " : "Not installed ") + release + ": " + port_block_text_(rep) +
             ", nothing sent";
      break;
    case PJO_ABORTED:
      head = rep.recover >= 0 ? "Recovery of " + port_board_text(rep.recover) + " aborted" : "Aborted " + release;
      break;
    default:
      if (rep.recover >= 0) {
        const PortNodeResult *t = nullptr;
        for (const auto &r : rep.nodes)
          if (r.position == rep.recover)
            t = &r;
        head = "Recovery of " + port_board_text(rep.recover) + ": " +
               (t != nullptr ? port_result_text_(*t, rep.outcome, rep.job_rc != 0) : std::string("?"));
      } else {
        head = "Done " + release + ": " + std::to_string(rep.count(PRR_VERIFIED)) + " verified, " +
               std::to_string(rep.count(PRR_SENT_UNVERIFIED)) + " unverified, " +
               std::to_string(rep.count(PRR_FAILED)) + " failed";
      }
      if (rep.job_rc)
        head += "; image refused: " + port_job_code_text(rep.job_rc);
  }
  const bool refused = rep.outcome == PJO_DONE && rep.job_rc != 0;
  std::vector<PortReportLine> lines;
  lines.push_back(PortReportLine{head});
  for (size_t k = 0; k < rep.nodes.size();) {
    const std::string what = port_result_text_(rep.nodes[k], rep.outcome, refused);
    size_t e = k + 1;
    while (e < rep.nodes.size() && rep.nodes[e].position == rep.nodes[e - 1].position + 1 &&
           port_result_text_(rep.nodes[e], rep.outcome, refused) == what)
      e++;
    PortReportLine line{port_board_text(rep.nodes[k].position), rep.nodes[k].position, rep.nodes[e - 1].position, true};
    if (e - k > 1)
      line.text += "-" + port_board_number(rep.nodes[e - 1].position);
    line.text += " " + what;
    lines.push_back(std::move(line));
    k = e;
  }
  return lines;
}

/// The whole report as one text ("\n" between its lines)
inline std::string port_report_full_text(const PortReport &rep) {
  std::string s;
  for (const auto &l : port_report_lines(rep))
    s += (s.empty() ? "" : "\n") + l.text;
  return s;
}

/// The report for Home Assistant's state (PORT_REPORT_MAX): the whole report when it fits; else its first lines and a
/// last one that says how many boards are left out and where they are ("2 more boards, B6-7: full report in the
/// device's log"). The log always has every line (the update entity writes it at a job's end and at each check).
inline std::string port_report_text(const PortReport &rep) {
  const std::vector<PortReportLine> lines = port_report_lines(rep);
  std::string s;
  for (const auto &l : lines)
    s += (s.empty() ? "" : "\n") + l.text;
  if (s.size() <= PORT_REPORT_MAX)
    return s;
  // the most lines that fit with the line naming the rest
  for (size_t keep = lines.size() - 1; keep > 0; keep--) {
    size_t boards = 0;
    for (size_t k = keep; k < lines.size(); k++)
      boards += lines[k].nodes ? lines[k].last - lines[k].first + 1u : 0u;
    const uint16_t from = lines[keep].first, to = lines.back().last;
    const std::string rest = std::to_string(boards) + (boards == 1 ? " more board, " : " more boards, ") +
                             port_board_text(from) + (from != to ? "-" + port_board_number(to) : std::string()) +
                             ": full report in the device's log";
    std::string out;
    for (size_t k = 0; k < keep; k++)
      out += (k ? "\n" : "") + lines[k].text;
    out += "\n" + rest;
    if (out.size() <= PORT_REPORT_MAX)
      return out;
  }
  return lines[0].text.substr(0, PORT_REPORT_MAX);   // (a headline alone longer than the state: never, its parts are short)
}

/// The update entity's release summary: the plan in one line
inline std::string port_plan_summary(const PortPlan &plan) {
  std::string s = "Update all receivers on this port: " + std::to_string(plan.count(PA_SEND)) + " to update";
  size_t images = 0;
  for (const auto &a : plan.artifacts)
    images += !a.positions.empty();
  if (images)
    s += " (" + std::to_string(images) + (images == 1 ? " image)" : " images)");
  s += ", " + std::to_string(plan.count(PA_CURRENT)) + " current, " + std::to_string(plan.count(PA_SKIP)) + " skipped";
  if (plan.count(PA_UNREACHABLE) + plan.count(PA_NONE))
    s += ", " + std::to_string(plan.count(PA_UNREACHABLE) + plan.count(PA_NONE)) + " unreachable or unknown";
  if (plan.blocked)
    s += "; blocked: " + port_blocked_text(plan);
  return s;
}

/// The report on the ESP32's preferences (fixed size: up to the largest port type's boards)
struct PortReportStore {
  static constexpr uint8_t MAGIC = 0x51, FORMAT = 3;
  static constexpr size_t NODES = PORT_D2A_PIN_MAX_BOARDS;
  uint8_t magic{0}, format{0}, outcome{0}, count{0};
  uint32_t release{0}, finished_s{0};
  uint16_t block_position{0xFFFF};
  uint8_t block_reason{0}, max_boards{0};
  uint8_t refusal{0}, reserved{0};
  int16_t job_rc{0}, recover{-1};
  struct Node {
    uint8_t position, result, reason, detail, need_boot, expect, flags, reserved;
    uint32_t before, expected, observed;
  } nodes[NODES];
};

inline void port_report_store(const PortReport &rep, PortReportStore &out) {
  std::memset(&out, 0, sizeof(out));
  out.magic = PortReportStore::MAGIC;
  out.format = PortReportStore::FORMAT;
  out.outcome = rep.outcome;
  out.count = static_cast<uint8_t>(std::min(rep.nodes.size(), PortReportStore::NODES));
  out.release = rep.release;
  out.finished_s = rep.finished_s;
  out.block_position = rep.block_position;
  out.block_reason = rep.block_reason;
  out.max_boards = rep.max_boards;
  out.refusal = rep.refusal;
  out.job_rc = rep.job_rc;
  out.recover = rep.recover;
  for (size_t k = 0; k < out.count; k++) {
    const auto &r = rep.nodes[k];
    auto &n = out.nodes[k];
    n.position = static_cast<uint8_t>(r.position);
    n.result = r.result;
    n.reason = r.reason;
    n.detail = r.detail;
    n.need_boot = r.need_boot;
    n.expect = r.expect;
    n.flags = (r.sent ? 1 : 0) | (r.before_known ? 2 : 0) | (r.observed_known ? 4 : 0);
    n.before = r.before;
    n.expected = r.expected;
    n.observed = r.observed;
  }
}

/// false: no stored report (or another format)
inline bool port_report_load(const PortReportStore &in, PortReport &rep) {
  if (in.magic != PortReportStore::MAGIC || in.format != PortReportStore::FORMAT || in.count > PortReportStore::NODES ||
      in.outcome > PJO_DONE)
    return false;
  rep = PortReport{};
  rep.outcome = static_cast<PortJobOutcome>(in.outcome);
  rep.release = in.release;
  rep.finished_s = in.finished_s;
  rep.block_position = in.block_position;
  rep.block_reason = in.block_reason;
  rep.max_boards = in.max_boards;
  rep.refusal = in.refusal;
  rep.job_rc = in.job_rc;
  rep.recover = in.recover;
  rep.nodes.resize(in.count);
  for (size_t k = 0; k < in.count; k++) {
    const auto &n = in.nodes[k];
    auto &r = rep.nodes[k];
    if (n.result > PRR_UNREACHABLE || n.reason > PRQ_OTHER_TARGET)
      return false;
    r.position = n.position;
    r.result = n.result;
    r.reason = n.reason;
    r.detail = n.detail;
    r.need_boot = n.need_boot;
    r.expect = n.expect;
    r.sent = n.flags & 1;
    r.before_known = n.flags & 2;
    r.observed_known = n.flags & 4;
    r.before = n.before;
    r.expected = n.expected;
    r.observed = n.observed;
  }
  return true;
}

}  // namespace esphome::ultra_line
