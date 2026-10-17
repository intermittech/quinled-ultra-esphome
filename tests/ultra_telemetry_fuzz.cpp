// Fuzz test (design review, round 14): the real return-channel decoder (components/dig2analog_ultra/
// ultra_telemetry.h), the lifetime ledger keeper (ultra_ledger.h), the port monitor and the commissioning, fed random /
// malformed / truncated / boundary byte streams. Built with UBSan (-fsanitize=undefined -fno-sanitize-recover=all): any UB aborts.
//
// Build and run: python tests/run_ultra_telemetry_fuzz.py [seeds] [windows per seed]
//
// Invariants checked after every window:
//  - the poller's window list is empty after finish(); misses <= 255; have_identity == have_board && have_features
//  - every text an entity would show stays <= 255 bytes (Home Assistant drops longer states to "unknown")
//  - a position's Energy (the ledger) never goes down; Last calibrated never in the future
//  - the ledger's stored form round-trips: save -> load -> save gives the same bytes
//  - the commissioning's report stays <= 250 bytes
#include "ultra_ledger.h"
#include "ultra_telemetry.h"
#include "ultra_commission.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace esphome::dig2analog_ultra;

static int failures = 0;
#define CHECK(cond, ...)                                      \
  do {                                                        \
    if (!(cond)) {                                            \
      failures++;                                             \
      if (failures <= 40) {                                   \
        std::printf("  FAIL %s:%d: ", __FILE__, __LINE__);    \
        std::printf(__VA_ARGS__);                             \
        std::printf("\n");                                    \
      }                                                       \
    }                                                         \
  } while (0)

struct FuzzPoller : TelemetryPoller {
  using TelemetryPoller::win_;
  using TelemetryPoller::rx_;
  using TelemetryPoller::Asked;
  size_t reach_now(uint32_t now) const { return this->reach_(now); }
};

static size_t max_text = 0;
static const char *max_text_who = "";
static void note_text(const std::string &s, const char *who) {
  if (s.size() > max_text) {
    max_text = s.size();
    max_text_who = who;
  }
  CHECK(s.size() <= 255, "%s text %zu bytes > 255 (Home Assistant drops it to unknown): %.80s...", who, s.size(),
        s.c_str());
}

int main(int argc, char **argv) {
  const unsigned long seeds = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 200;
  const unsigned long windows_per_seed = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 3000;
  unsigned long frames_fed = 0, windows = 0, answers = 0, stale = 0, bad = 0, restarts = 0;
  for (unsigned long seed = 1; seed <= seeds; seed++) {
    std::mt19937 rng(static_cast<uint32_t>(seed * 7919u));
    auto rnd = [&](uint32_t n) { return static_cast<uint32_t>(rng() % n); };
    const uint16_t n = static_cast<uint16_t>(1 + rnd(64));
    const uint8_t rate = static_cast<uint8_t>(rnd(4));   // (3: invalid: the poller must fall back)
    FuzzPoller p;
    TelemetryIntervals iv;
    if (rnd(2)) {   // short intervals: more pages per window
      iv.status_ms = 30;
      iv.power_ms = 50;
      iv.energy_ms = 200;
      iv.supply_ms = 200;
      iv.health_ms = 300;
      iv.lifetime_ms = 500;
      iv.diag_ms = 500;
    }
    p.init(n, rate, iv, static_cast<uint8_t>(rnd(256)));
    for (uint16_t i = 0; i < n; i++)
      if (rnd(3) == 0)
        p.set_outputs(i, true);
    LifetimeLedger ledger;
    ledger.init(n + 4, n + 8);
    LedgerKeeper keeper;
    keeper.attach(&ledger, 0xABCD0000u, n, 0);
    PortMonitor mon;
    PortMonitor::Config mc;
    if (rnd(2))
      mc.supply_v = 55.0f;
    mon.configure(mc);
    Commissioning com;
    com.configure(iv.status_ms, 0.9f);
    CalibrationLog cal;   // the receivers' calibration records (with a clock in most windows)
    cal.init(n);
    // a stable UID per position, changed now and then (a swapped board)
    std::vector<std::array<uint8_t, 12>> uid(n);
    for (auto &u : uid)
      for (auto &b : u)
        b = static_cast<uint8_t>(rnd(256));
    std::vector<double> last_energy(n, -1.0);
    // millis() starting near a wrap in half the seeds
    uint32_t now = rnd(2) ? 0xFFFFFFFFu - 20000u : 1000u;
    p.locked(now);
    now += 1000 + rnd(200);
    keeper.power_on(now);
    std::vector<uint8_t> bytes;
    uint8_t slots[ULTRA_SLOT_BYTES * TelemetryPoller::MAX_RECEIVERS];
    bool line_on = true;
    for (unsigned long w = 0; w < windows_per_seed; w++) {
      now += 1 + rnd(rnd(50) == 0 ? 400000 : 40);   // (now and then a long gap: stale pages, the ledger's day)
      // the odd state change
      switch (rnd(400)) {
        case 0:
          p.unpowered();
          keeper.cut(now);
          line_on = false;
          break;
        case 1:
          if (!line_on) {
            p.locked(now);
            keeper.power_on(now);
            line_on = true;
          }
          break;
        case 2:
          p.touch_all();
          break;
        case 3:
          p.mark_fresh(now, rnd(2) != 0);
          break;
        case 4: {
          uint8_t args[23];
          for (auto &b : args)
            b = static_cast<uint8_t>(rnd(256));
          p.request_sync(static_cast<uint16_t>(rnd(n + 1)), args, now);   // (n: out of range)
          break;
        }
        case 5:
          p.read_now(static_cast<uint16_t>(rnd(n + 1)), static_cast<uint8_t>(rnd(TP_COUNT + 1)), now);
          break;
        case 6:
          p.set_rate(static_cast<uint8_t>(rnd(5)));
          break;
        case 7:
          for (auto &b : uid[rnd(n)])
            b = static_cast<uint8_t>(rnd(256));   // a swapped board
          break;
        case 8:
          keeper.prepare(p, now, true);
          break;
        case 9:
          com.request(rnd(2) ? Commissioning::FINALIZE : Commissioning::COMMISSION);
          break;
        case 10:
          if (com.queued())
            com.start(now);
          break;
        default:
          break;
      }
      if (!line_on && rnd(10) != 0)
        continue;
      if (!p.due(now)) {
        CHECK(p.due_scan(now) == false, "due() memo says idle but the scan finds a page");
        continue;
      }
      uint32_t win_us = 0;
      const uint16_t sent = p.build(now, slots, &win_us);
      if (sent == 0)
        continue;
      windows++;
      CHECK(sent <= n, "build() polled %u boards of %u", sent, n);
      CHECK(p.win_.size() <= ULTRA_RW_MAX_SLOTS, "%zu answers asked in one window", p.win_.size());
      // the slots are valid POLLs
      for (uint16_t b = 0; b < sent; b++)
        CHECK(ultra_poll_valid(slots + ULTRA_SLOT_BYTES * b) == 0, "slot %u not a valid POLL", b);
      // ---- the byte stream: junk, the answers (mostly right, some wrong / broken / truncated / duplicated), junk
      bytes.clear();
      auto junk = [&](uint32_t k) {
        for (uint32_t i = 0; i < k; i++)
          bytes.push_back(static_cast<uint8_t>(rnd(8) == 0 ? ULTRA_RSP_SYNC : rnd(256)));
      };
      junk(rnd(rnd(20) == 0 ? 600 : 6));
      const std::vector<FuzzPoller::Asked> asked = p.win_;
      for (const auto &a : asked) {
        const uint32_t kind = rnd(100);
        if (kind < 12)
          continue;   // no answer (missing: silences the ones behind)
        uint8_t f[ULTRA_SLOT_BYTES];
        for (auto &b : f)
          b = static_cast<uint8_t>(rnd(256));   // random DATA (every field at every value)
        f[0] = ULTRA_RSP_SYNC;
        uint8_t page = TP_PAGE_ID[a.tp];
        if (kind < 16)
          page = ULTRA_PAGE_BUSY;
        else if (kind < 20)
          page = ULTRA_PAGE_UNSUPPORTED;
        else if (kind < 23)
          page = static_cast<uint8_t>(rnd(256));   // a wrong page
        f[1] = page;
        uint8_t index = a.index;
        if (kind >= 23 && kind < 26)
          index = static_cast<uint8_t>(rnd(16));
        f[2] = static_cast<uint8_t>((index << 4) | (rnd(4) == 0 ? rnd(16) : 0));
        f[3] = kind >= 26 && kind < 29 ? static_cast<uint8_t>(rnd(256)) : a.seq;
        const uint16_t u16 = ultra_crc16(uid[a.rx].data(), 12);
        ultra_put16(f + 4, kind >= 29 && kind < 31 ? static_cast<uint16_t>(rnd(65536)) : u16);
        if (page == ULTRA_PAGE_IDENTITY && rnd(4) != 0 && a.tp == TP_IDENTITY)
          std::memcpy(f + ULTRA_RSP_DATA + ULTRA_ID_UID, uid[a.rx].data(), 12);   // (mostly the position's UID)
        if (page == ULTRA_PAGE_STATUS && rnd(2))
          ultra_put24(f + ULTRA_RSP_DATA + ULTRA_ST_UPTIME, (now / 1000u) & 0xFFFFFF);   // a plausible uptime
        ultra_put16(f + 30, ultra_crc16(f, 30));
        if (kind >= 31 && kind < 34)
          f[1 + rnd(31)] ^= static_cast<uint8_t>(1 << rnd(8));   // broken (CRC fails)
        const size_t cut = kind >= 34 && kind < 38 ? 1 + rnd(31) : ULTRA_SLOT_BYTES;   // truncated
        bytes.insert(bytes.end(), f, f + cut);
        frames_fed++;
        if (kind >= 38 && kind < 40)
          bytes.insert(bytes.end(), f, f + ULTRA_SLOT_BYTES);   // the same answer twice
        if (rnd(3) == 0)
          junk(rnd(4));
      }
      if (rnd(30) == 0)   // a late answer of another window: a valid frame with a random index / seq
        for (int k = 0; k < 3; k++) {
          uint8_t f[ULTRA_SLOT_BYTES];
          for (auto &b : f)
            b = static_cast<uint8_t>(rnd(256));
          f[0] = ULTRA_RSP_SYNC;
          ultra_put16(f + 30, ultra_crc16(f, 30));
          bytes.insert(bytes.end(), f, f + ULTRA_SLOT_BYTES);
        }
      junk(rnd(6));
      if (rnd(40) == 0)
        bytes.resize(rnd(bytes.size() + 1));   // the UART buffer cut anywhere
      const TelemetryPoller::Stats before = p.stats();
      p.finish(now + win_us / 1000, bytes.data(), bytes.size());
      const TelemetryPoller::Stats after = p.stats();
      answers += after.answers - before.answers;
      stale += after.stale - before.stale;
      bad += after.bad_crc - before.bad_crc;
      restarts += after.restarts - before.restarts;
      CHECK(p.win_.empty(), "the window list isn't cleared after finish()");
      CHECK(after.answers - before.answers <= asked.size(), "more answers matched (%u) than asked (%zu)",
            after.answers - before.answers, asked.size());
      CHECK(p.reach_now(now) <= n, "reach beyond the chain");
      // ---- everything the entities would show, on every receiver
      const int cut_at = p.cut_at(now);
      note_text(telemetry_setup_info(p), "telemetry_setup_info");
      for (uint16_t i = 0; i < n; i++) {
        const ReceiverData &d = p.data(i);
        CHECK(d.have_identity == (d.have_board && d.have_features), "rx %u: have_identity inconsistent", i);
        CHECK(p.misses(i) <= 255, "misses overflow");
        const bool ans = p.answering(i);
        const ReceiverStatus s = receiver_status(d, ans);
        note_text(receiver_status_detail(d, s, i, cut_at), "receiver_status_detail");
        note_text(telemetry_status_text(d), "telemetry_status_text");
        note_text(telemetry_power_events_text(d), "telemetry_power_events_text");
        note_text(telemetry_health_text(d), "telemetry_health_text");
        note_text(telemetry_last_trip(d), "telemetry_last_trip");
        note_text(telemetry_last_dip(d), "telemetry_last_dip");
        note_text(telemetry_last_reset(d), "telemetry_last_reset");
        note_text(receiver_status_notes(d), "receiver_status_notes");
        {   // the supply dips against a random dip_warning: the Status, its line, the receiver page's note
          const uint8_t warn = static_cast<uint8_t>(50 + rnd(41));
          const ReceiverStatus sw = receiver_status(d, ans, 0, warn);
          note_text(receiver_status_detail(d, sw, i, cut_at), "receiver_status_detail (dip_warning)");
          note_text(telemetry_supply_note(d, warn), "telemetry_supply_note");
        }
        note_text(telemetry_firmware_text(d), "telemetry_firmware_text");
        for (uint8_t v = 0; v < TV_COUNT; v++)
          (void) telemetry_value(d, static_cast<TelemetryValue>(v));
        for (uint8_t m = 0; m < 32; m++) {
          (void) light_status(d, ans, m);
          (void) light_power_estimate(d, m);
          note_text(receiver_status_detail(d, light_status(d, ans, m), i, cut_at), "light status detail");
        }
        // the ledger
        const uint32_t u = p.take_updates(i);
        keeper.lit_tick(i, rnd(2) != 0, now);
        (void) keeper.lit_due(now);
        (void) keeper.on_updates(p, i, u, now);
        // the calibration record and its texts, the Status in a run of the hub's
        if (u & (1u << TP_CALIBRATION))
          (void) cal.observe(i, d, p.uid16(i), now, rnd(4) != 0 ? 1792000000u + now / 1000u : 0u);
        note_text(cal.result(i, d), "calibration result");
        note_text(calibration_model_text(d.cal_model), "calibration model");
        CHECK(cal.last_unix(i) <= 1792000000u + 0xFFFFFFFFu / 1000u,
              "rx %u: Last calibrated in the future (%u)", i, cal.last_unix(i));
        note_text(receiver_status_detail(d, with_hub_calibration(s, rnd(2) != 0), i, cut_at), "calibrating detail");
        const double e = keeper.energy_wh(i);
        if (e >= 0 && last_energy[i] >= 0)
          CHECK(e + 1e-9 >= last_energy[i], "rx %u: the position's Energy went down %.6f -> %.6f", i,
                last_energy[i], e);
        if (e >= 0)
          last_energy[i] = e;
        (void) keeper.counters(i);
      }
      mon.update(now, p);
      note_text(port_status(mon, p), "port_status");
      note_text(port_status_detail(mon), "port_status_detail");
      (void) mon.cable_loss_pct();
      for (uint16_t k = 0; k < n; k++) {
        (void) mon.drop_v(k);
        (void) mon.resistance(k);
      }
      // the commissioning with a random port state
      CommissionPort cp;
      cp.present = rnd(4) != 0;
      cp.detect = rnd(2) != 0;
      cp.off = rnd(20) == 0;
      cp.board = rnd(4) != 0;
      cp.fault = rnd(20) == 0;
      cp.ready = rnd(3) != 0;
      (void) com.poll(now, rnd(2) ? now / 1000 : 0, cp, p, mon, rnd(3) == 0, rnd(20) == 0);
      note_text(com.status(), "commission status");
      CHECK(com.report().size() <= 250 || com.running(), "commission report %zu bytes", com.report().size());
      // the ledger's stored form round-trips
      if (rnd(200) == 0) {
        std::vector<uint8_t> blob(ledger.blob_size());
        ledger.save(blob.data(), 7);
        LifetimeLedger l2;
        l2.init(n + 4, n + 8);
        uint32_t gen = 0;
        size_t ne = 0, np = 0;
        CHECK(LifetimeLedger::peek(blob.data(), blob.size(), &gen, &ne, &np) && gen == 7, "peek failed");
        CHECK(l2.load(blob.data(), blob.size(), &gen), "load failed");
        std::vector<uint8_t> blob2(l2.blob_size());
        l2.save(blob2.data(), 7);
        CHECK(blob == blob2, "the ledger's stored form doesn't round-trip (%zu vs %zu bytes)", blob.size(),
              blob2.size());
        // a corrupted copy is refused, never loaded
        std::vector<uint8_t> bad_blob = blob;
        bad_blob[rnd(bad_blob.size())] ^= 0x5A;
        CHECK(!l2.load(bad_blob.data(), bad_blob.size(), &gen) || bad_blob == blob, "a corrupted ledger loaded");
        std::vector<uint8_t> shorter(blob.begin(), blob.begin() + rnd(blob.size()));
        CHECK(!l2.load(shorter.data(), shorter.size(), &gen), "a truncated ledger loaded");
      }
    }
    if (seed % 50 == 0)
      std::printf("  seed %lu: %u receivers, rate %u: %lu windows so far, %lu frames fed\n", seed, n, rate, windows,
                  frames_fed);
  }
  std::printf("fuzz: %lu seeds, %lu windows, %lu frames fed (%lu matched, %lu stale, %lu bad SYNC, %lu restarts), "
              "longest text %zu bytes (%s)\n",
              seeds, windows, frames_fed, answers, stale, bad, restarts, max_text, max_text_who);
  std::printf(failures == 0 ? "fuzz: passed (0 failures)\n" : "fuzz: %d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
