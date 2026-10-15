// Exact differential reference: fixtures/ultra_ledger_r10.h, frozen from a73ff9c before round 11.
#include "../components/dig2analog_ultra/ultra_ledger.h"
#include "fixtures/ultra_ledger_r10.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>

using namespace esphome::dig2analog_ultra;
static int failures = 0;
#define CHECK(x) do { if (!(x)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); failures++; } } while (0)

// Track precisely the caller's blob lifetime; the backend fixture counts its pending and read-back buffers.
static void *caller_blob = nullptr;
static size_t caller_size = 0;
void *operator new[](size_t n, const std::nothrow_t &) noexcept {
  caller_size = n;
  caller_blob = std::malloc(n);
  return caller_blob;
}
void operator delete[](void *p) noexcept {
  if (p == caller_blob) { caller_blob = nullptr; caller_size = 0; }
  std::free(p);
}

template<class L> static std::vector<uint8_t> blob(const L &l, uint32_t gen = 17) {
  std::vector<uint8_t> b(l.blob_size());
  l.save(b.data(), gen);
  return b;
}
template<class L> static std::vector<uint8_t> canonical(const L &l) {
  auto b = blob(l);
  LifetimeLedger copy;
  copy.init(256, 256);
  uint32_t gen;
  CHECK(copy.load(b.data(), b.size(), &gen));
  return blob(copy);
}

static void test_time() {
  LifetimeLedger a;
  ledger_r10::LifetimeLedger b;
  LedgerEntry x;
  ledger_r10::LedgerEntry y;
  std::mt19937_64 rng(0x11ed9e);
  const uint64_t edge[] = {0, 1, UNIT_MS - 1, UNIT_MS, UNIT_MS + 1, UINT32_MAX,
                          UINT64_MAX, UINT64_MAX - UNIT_MS};
  for (unsigned i = 0; i < 1000000; i++) {
    x.c.powered = y.c.powered = static_cast<uint32_t>(rng());
    x.c.lit = y.c.lit = static_cast<uint32_t>(rng());
    x.pow_rem = y.pow_rem = static_cast<uint32_t>(rng());
    x.lit_rem = y.lit_rem = static_cast<uint32_t>(rng());
    if (i & 1) { x.pow_rem = y.pow_rem %= UNIT_MS; x.lit_rem = y.lit_rem %= UNIT_MS; }
    const uint64_t ms = i < 64 ? edge[i % 8] : i % 3 == 0 ? rng() : rng() % (2 * UNIT_MS);
    a.clear_dirty(); b.clear_dirty();
    a.add_time(x, i & 2, ms); b.add_time(y, i & 2, ms);
    CHECK(x.c.powered == y.c.powered && x.c.lit == y.c.lit && x.pow_rem == y.pow_rem &&
          x.lit_rem == y.lit_rem && a.dirty() == b.dirty());
  }
  std::printf("add_time: 1,000,000 exact cases, including u64 addition/u32 unit wrap and invalid input remainders\n");
}

template<class L> static void populate(L &l, unsigned rx, unsigned ports, unsigned historical = 0) {
  const size_t n = rx + ports;
  l.init((n + std::max<size_t>(4, n / 2) + 31) / 32 * 32, (n + 31) / 32 * 32);
  for (unsigned i = 0; i < rx + historical; i++) {
    uint8_t uid[12] = {};
    ultra_put32(uid, i + 1);
    bool made, swapped;
    auto &e = *l.touch(uid, &made);
    e.c.powered = 9000 + i; e.c.lit = 800 + i; e.c.ups = 70 + i; e.c.dips = 2 + i;
    e.c.ovl = 3 + i; e.c.shorts = 4 + i; e.c.mwh = 100000 + i;
    e.records = 100 + i; e.sess_boot = 3000 + i; e.sess_up = 2000 + i;
    e.sess_ups = 60 + i; e.sess = true; e.pow_rem = 101 + i; e.lit_rem = 102 + i; e.en_rem = 103 + i;
    if (i < rx) l.position_energy(i + 1, e, &swapped);
  }
  for (unsigned i = 0; i < ports; i++) l.set_counter(i + 1000, 111111 + i);
}

static void test_compact() {
  for (auto counts : {std::pair<unsigned, unsigned>{20, 8}, {50, 16}}) {
    LifetimeLedger a;
    ledger_r10::LifetimeLedger b;
    populate(a, counts.first, counts.second); populate(b, counts.first, counts.second);
    auto compact = blob(a), full = blob(b);
    const size_t expected = 18 + 76 * counts.first + 33 * (counts.first + counts.second);
    CHECK(compact.size() == expected && a.capacity() == b.capacity());
    CHECK(canonical(a) == canonical(b));
    std::printf("%u rx / %u ports: full %zu B -> compact %zu B; capacity %zu boards unchanged\n",
                counts.first, counts.second, full.size(), compact.size(), a.capacity());
    LifetimeLedger new_reader; new_reader.init(128, 96);
    ledger_r10::LifetimeLedger old_reader; old_reader.init(128, 96);
    uint32_t gen = 0;
    CHECK(new_reader.load(full.data(), full.size(), &gen) && gen == 17);
    CHECK(old_reader.load(compact.data(), compact.size(), &gen) && gen == 17);
    CHECK(canonical(new_reader) == canonical(old_reader) && canonical(a) == canonical(new_reader));
    for (size_t n = 0; n < compact.size(); n++) {
      CHECK(!new_reader.load(compact.data(), n, &gen));
    }
    auto before = canonical(new_reader);
    for (size_t i = 0; i < compact.size(); i++) {
      compact[i] ^= 0x40;
      CHECK(!new_reader.load(compact.data(), compact.size(), &gen));
      compact[i] ^= 0x40;
    }
    CHECK(canonical(new_reader) == before);
    auto wrong_count = compact;
    ultra_put16(wrong_count.data() + 12, counts.first + 1);
    ultra_put16(wrong_count.data() + wrong_count.size() - 2,
                ultra_crc16(wrong_count.data(), static_cast<uint32_t>(wrong_count.size() - 2)));
    CHECK(!new_reader.load(wrong_count.data(), wrong_count.size(), &gen));
  }
  // Historical, unheld boards survive; holes compact in the same order as the old reader loaded them.
  LifetimeLedger a; ledger_r10::LifetimeLedger b;
  populate(a, 20, 8, 5); populate(b, 20, 8, 5);
  CHECK(canonical(a) == canonical(b));
  a.set_day(90); b.set_day(90);
  CHECK(canonical(a) == canonical(b));
  bool made;
  uint8_t uid[12] = {0xEE};
  a.touch(uid, &made); b.touch(uid, &made);
  CHECK(canonical(a) == canonical(b));
  a.init(64, 32); b.init(64, 32);
  CHECK(a.blob_size() == 18 && canonical(a) == canonical(b));
  std::printf("LDG3: bidirectional readers, historical rows/holes, empty form, every truncation/byte corruption\n");
}

#define ESP_LOGD(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
namespace current_store {
using Ledger = esphome::dig2analog_ultra::LifetimeLedger;
#include "fixtures/ledger_store_test.inc"
#include "hub_ledger.inc"
#include "fixtures/ledger_store_cases.inc"
}
namespace old_store {
using Ledger = ledger_r10::LifetimeLedger;
#include "fixtures/ledger_store_test.inc"
#include "fixtures/hub_ledger_r10.inc"
#include "fixtures/ledger_store_cases.inc"
}

static void test_store() {
  auto old = old_store::save_cases();
  auto current = current_store::save_cases();
  CHECK(old == current);
  CHECK(old_store::peak_blocks == 3 && current_store::peak_blocks == 2);
  std::printf("save/free differential: identical loaded state/failure/retry outcomes; peak %zu -> %zu blob blocks\n",
              old_store::peak_blocks, current_store::peak_blocks);
  using namespace current_store;
  stored.clear(); reboot(28);
  UltraHub h; h.ledger_service_();
  populate(ledger, 20, 8);
  h.ledger_save_(true); h.ledger_save_(true);
  ledger.clear_dirty();
  bool made;
  uint8_t uid[12] = {99};
  ledger.touch(uid, &made); h.ledger_save_(true);
  const auto good = canonical(ledger);
  CHECK(g_ledger_generation == 2);
  auto stable = stored;
  // The next occupied entry changes the length while runtime allocation stays exactly the same.
  uint8_t uid2[12] = {100};
  for (int point = 0; point <= 2; point++) {
    stored = stable; reboot(28); h.ledger_service_();
    ledger.touch(uid2, &made); auto next = canonical(ledger);
    cut_after = point;
    h.ledger_save_(true);
    CHECK(writes.empty() || writes[0] == fnv1_hash(LEDGER_KEYS[1]));
    cut_after = -1; reboot(28); h.ledger_service_();
    CHECK(canonical(ledger) == (point == 2 ? next : good));
    CHECK(g_ledger_generation == (point == 2 ? 3u : 2u));
  }
  // Inverted ordering is deliberately injected: new size + old blob cannot displace the good copy.
  stored = stable;
  uint32_t wrong_size = static_cast<uint32_t>(ledger.blob_size() + 76);
  stored[fnv1_hash(LEDGER_SIZE_KEYS[1])] = std::vector<uint8_t>(reinterpret_cast<uint8_t *>(&wrong_size),
                                                            reinterpret_cast<uint8_t *>(&wrong_size) + 4);
  reboot(28); h.ledger_service_(); CHECK(g_ledger_generation == 2 && canonical(ledger) == good);
  // Every torn prefix of the newer blob, even with its advertised size, falls back to the other copy.
  stored = stable;
  const auto newest = stable[fnv1_hash(LEDGER_KEYS[0])];
  for (size_t n = 0; n < newest.size(); n++) {
    stored = stable;
    stored[fnv1_hash(LEDGER_KEYS[0])].resize(n);
    reboot(28); h.ledger_service_(); CHECK(g_ledger_generation == 1);
  }
  // Full-size old blob -> compact save, smaller configuration, retained historical rows and generation wrap.
  ledger_r10::LifetimeLedger legacy; populate(legacy, 50, 16, 7);
  stored.clear(); auto full = blob(legacy, UINT32_MAX);
  stored[fnv1_hash(LEDGER_KEYS[1])] = full;
  uint32_t size = static_cast<uint32_t>(full.size());
  stored[fnv1_hash(LEDGER_SIZE_KEYS[1])] = std::vector<uint8_t>(reinterpret_cast<uint8_t *>(&size),
                                                            reinterpret_cast<uint8_t *>(&size) + 4);
  reboot(1); h.ledger_service_();
  CHECK(canonical(ledger) == canonical(legacy) && ledger.capacity() >= 57);
  ledger.touch(uid2, &made); h.ledger_save_(true);
  auto wrapped = canonical(ledger);
  CHECK(g_ledger_generation == 0);
  reboot(1); h.ledger_service_(); CHECK(g_ledger_generation == 0 && canonical(ledger) == wrapped);
  stored[fnv1_hash(LEDGER_KEYS[0])][20] ^= 1;
  reboot(1); h.ledger_service_();
  CHECK(g_ledger_generation == UINT32_MAX && canonical(ledger) == canonical(legacy));
  for (uint32_t invalid_size : {0u, 17u, static_cast<uint32_t>(LEDGER_MAX_BLOB + 1)}) {
    stored[fnv1_hash(LEDGER_SIZE_KEYS[0])] = std::vector<uint8_t>(reinterpret_cast<uint8_t *>(&invalid_size),
                                                              reinterpret_cast<uint8_t *>(&invalid_size) + 4);
    reboot(1); h.ledger_service_(); CHECK(g_ledger_generation == UINT32_MAX);
  }
  // Aged-out rows shrink the blob; the newer empty copy replaces, rather than merges, its predecessor.
  stored.clear(); reboot(1); h.ledger_service_();
  ledger.touch(uid, &made); h.ledger_save_(true);
  ledger.set_day(LEDGER_AGE_DAYS); h.ledger_save_(true);
  CHECK(ledger.blob_size() == 18);
  reboot(1); h.ledger_service_(); CHECK(g_ledger_generation == 2 && ledger.find(uid) == nullptr);
  std::printf("preferences: grow/shrink, blob-before-size cuts, size-before-blob rejection, every torn prefix, "
              "old/new two-copy fallback, resize, aging, generation wrap\n");
}

#define USE_DIFFPOWER
namespace current_line {
using Ledger = esphome::dig2analog_ultra::LifetimeLedger;
using CoreKeeper = esphome::dig2analog_ultra::LedgerKeeper;
#include "fixtures/ledger_line_test.inc"
#include "hub_ledger_line.inc"
#include "fixtures/ledger_line_cases.inc"
}
namespace old_line {
using Ledger = ledger_r10::LifetimeLedger;
using CoreKeeper = ledger_r10::LedgerKeeper;
#include "fixtures/ledger_line_test.inc"
#include "fixtures/hub_ledger_line_r10.inc"
#include "fixtures/ledger_line_cases.inc"
}
#undef USE_DIFFPOWER

namespace current_service_line {
namespace store = current_store;
namespace line = current_line;
#include "fixtures/ledger_service_line_cases.inc"
}
namespace old_service_line {
namespace store = old_store;
namespace line = old_line;
#include "fixtures/ledger_service_line_cases.inc"
}

static void test_line() {
  uint64_t old_ticks, new_ticks, old_span, new_span;
  auto before = old_line::run_line(old_ticks, old_span);
  auto after = current_line::run_line(new_ticks, new_span);
  CHECK(before == after && old_span == new_span && new_ticks < old_ticks);
  CHECK(current_line::multi_line() == old_line::multi_line());
  CHECK(current_service_line::run() == old_service_line::run());
  std::printf("line differential: %.1f simulated days, clock wraps, short pulses/fades, overlapping lights, "
              "power/session changes, late identity, reads/saves/snapshots exact; ticks %llu -> %llu\n",
              new_span / 86400000.0, static_cast<unsigned long long>(old_ticks),
              static_cast<unsigned long long>(new_ticks));
  std::printf("service/line differential: exact 15/30 minute save timestamps and saved remainders, forced off-grid save\n");
  // A blind 50 ms sampler misses a complete 3 ms pulse: a negative control against the tempting whole-body gate.
  LifetimeLedger sampled, exact;
  sampled.init(1, 1); exact.init(1, 1);
  bool made;
  uint8_t uid[12] = {1};
  auto *x = exact.touch(uid, &made), *s = sampled.touch(uid, &made);
  for (unsigned ms = 1; ms <= 1000; ms++) {
    if (ms % 97 >= 1 && ms % 97 < 4) exact.add_time(*x, true, 1);
    if (ms % 50 == 0 && ms % 97 >= 1 && ms % 97 < 4) sampled.add_time(*s, true, 50);
  }
  CHECK(x->lit_rem != s->lit_rem);
  std::printf("blind-gate negative control: 3 ms pulses count %u ms versus %u ms; transition scan is required\n",
              x->lit_rem, s->lit_rem);
}

__attribute__((noinline)) static void tick_old(ledger_r10::LifetimeLedger &l, ledger_r10::LedgerEntry &e, uint64_t ms) {
  l.add_time(e, true, ms);
}
__attribute__((noinline)) static void tick_new(LifetimeLedger &l, LedgerEntry &e, uint64_t ms) {
  l.add_time(e, true, ms);
}
template<class F> static double timed(unsigned n, F f) {
  auto start = std::chrono::steady_clock::now();
  for (unsigned i = 0; i < n; i++) f(i);
  return std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / n;
}
static void measure() {
  ledger_r10::LifetimeLedger old;
  LifetimeLedger current;
  ledger_r10::LedgerEntry oe;
  LedgerEntry ne;
  double ot = timed(5000000, [&](unsigned) { tick_old(old, oe, 1); });
  double nt = timed(5000000, [&](unsigned) { tick_new(current, ne, 1); });
  CHECK(oe.lit_rem == ne.lit_rem && oe.c.lit == ne.c.lit);
  std::printf("host add_time: %.3f -> %.3f ns/call (5,000,000 calls, 1 ms each)\n", ot, nt);
  std::printf("host object bytes: ledger %zu -> %zu; keeper %zu -> %zu; entry %zu -> %zu (no per-receiver growth)\n",
              sizeof(old), sizeof(current), sizeof(ledger_r10::LedgerKeeper), sizeof(LedgerKeeper),
              sizeof(ledger_r10::LedgerEntry), sizeof(LedgerEntry));
  for (auto counts : {std::pair<unsigned, unsigned>{20, 8}, {50, 16}}) {
    populate(old, counts.first, counts.second); populate(current, counts.first, counts.second);
    auto ob = blob(old), nb = blob(current);
    ot = timed(1000, [&](unsigned i) { old.save(ob.data(), i); });
    nt = timed(1000, [&](unsigned i) { current.save(nb.data(), i); });
    std::printf("host serialization %u/%u: %.2f -> %.2f us/save; modeled blob peak %zu -> %zu B "
                "(compaction + early free)\n", counts.first, counts.second, ot / 1000, nt / 1000,
                3 * ob.size(), 2 * nb.size());
    uint64_t old_ticks, new_ticks;
    auto old_state = old_line::bench_line(counts.first, ot, old_ticks);
    auto new_state = current_line::bench_line(counts.first, nt, new_ticks);
    CHECK(old_state == new_state);
    std::printf("host line %u rx, 1 ms loops: %.2f -> %.2f ns/loop; lit_tick %llu -> %llu, exact saved state\n",
                counts.first, ot, nt, static_cast<unsigned long long>(old_ticks),
                static_cast<unsigned long long>(new_ticks));
  }
}
int main() {
  test_time();
  test_compact();
  test_store();
  test_line();
  measure();
  std::printf("ledger differential tests: %s (%d failures)\n", failures ? "FAILED" : "passed", failures);
  return failures != 0;
}
