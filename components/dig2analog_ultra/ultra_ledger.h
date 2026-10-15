#pragma once

// The receivers' lifetime ledger (protocol 0.2.6 LIFE_SYNC; review-r8/fable-sync §4): the hub keeps each board's
// lifetime counters by its full 12-byte UID, as the highest it has seen (the board may lose what it hadn't stored at a
// power cut; the hub doesn't), and gives them back to the board once per power-up when it is ahead. The Home Assistant
// Energy of a position (a line's receiver slot) is the board's ledger energy plus the position's offset: it continues
// over a replaced board, a restart, a wake and a restart of the ESP32 itself, never down. It also keeps the counters
// that belong to no board (a port's energy at the sender, a light's estimated energy): only up. Positions and
// counters are keyed by the line's id (not its pin: a moved line keeps them) or the light's. Pure logic: the host
// tests run it (tests/ultra_telemetry_test.cpp); dig2analog_ultra.cpp keeps it on the ESP32.

#include <cstdint>
#include <cstring>
#include <vector>

#include "ultra_protocol.h"
#include "ultra_telemetry.h"

namespace esphome::dig2analog_ultra {

static constexpr uint32_t LEDGER_MAGIC = 0x4C444732;    // "LDG2" (with each board's session checkpoint)
static constexpr uint32_t SESS_UPS_UNKNOWN = 0xFFFFFFFFu;
static constexpr uint32_t LEDGER_AGE_DAYS = 90;         // a board not seen this many hub days: its entry may go
static constexpr uint32_t LEDGER_SAVE_MS = 15 * 60000;  // saved at most this often while on (and at a planned cut)
static constexpr uint32_t LEDGER_DAY_MS = 86400000;
static constexpr uint32_t SYNC_DAILY_MS = 86400000;     // a long run: compared (and synced if ahead) once a day
static constexpr size_t LEDGER_MAX_BLOB = 1u << 20;       // a stored copy's size at most (its size record checked)
static constexpr uint32_t BOOT_SLACK_MS = 2000;         // a board up this much longer than the hub: on before it

/// A board's lifetime counters in the wire's units (LIFETIME bytes 0-15), its energy in exact mWh.
struct LifeCounters {
  uint32_t powered{0}, lit{0};   // 0.1 h (u24)
  uint32_t ups{0}, dips{0};      // u24
  uint16_t ovl{0}, shorts{0};    // u16 (saturating)
  uint64_t mwh{0};
};

/// The receiver's counters now: LIFETIME's (have_lifetime) and ENERGY's exact mWh (have_energy).
inline LifeCounters receiver_counters(const ReceiverData &d) {
  LifeCounters c;
  c.powered = d.lt_powered;
  c.lit = d.lt_lit;
  c.ups = d.lt_power_ups;
  c.dips = d.lt_dips;
  c.ovl = d.lt_overload_trips;
  c.shorts = d.lt_short_trips;
  c.mwh = static_cast<uint64_t>(d.total_wh) * 1000u + d.boot_mwh % 1000u;
  return c;
}

struct LedgerEntry {
  uint8_t uid[12]{};
  LifeCounters c;
  uint32_t records{0};    // the board's RECORDS (u24) as last read: its low 8 bits are the sync's compare-and-swap
  uint32_t last_day{0};   // the hub day it was last seen
  // its session as last counted (stored with the energy it is part of): its count since power-up then, its uptime
  // and its power-ups count; the same session later (an ESP32 restart under it, the board moved while powered) goes
  // on from that count, so nothing is lost and nothing counted twice
  uint32_t sess_boot{0}, sess_up{0}, sess_ups{SESS_UPS_UNKNOWN};
  bool sess{false};
  bool used{false};
};

/// A line's receiver slot: the board there and its Energy (the board's ledger energy + offset); or (counter) a value of
/// its own that only goes up (a port's energy, a light's).
struct LedgerPosition {
  uint32_t key{0};        // the line (its id's hash) and the receiver index; a counter's own key
  uint8_t uid[12]{};
  bool have_uid{false};
  bool counter{false};
  int64_t offset_mwh{0};
  uint64_t value_mwh{0};  // its Energy as last computed (the base when its board changes); a counter's value
  bool used{false};
};

/// A 32-bit FNV-1a of a string (the keys: a line's id, a light's id; the same as ESPHome's fnv1a_32bit_hash)
inline uint32_t ledger_key(const char *s) {
  uint32_t h = 2166136261u;
  for (; *s != 0; s++) {
    h ^= static_cast<uint8_t>(*s);
    h *= 16777619u;
  }
  return h;
}

class LifetimeLedger;
/// the ESP32's one ledger (every line's receivers) and their count (each line adds its own at its setup: its receivers,
/// its port, its lights' estimates)
inline LifetimeLedger &ultra_ledger();
inline size_t &ledger_receivers() {
  static size_t n = 0;
  return n;
}

class LifetimeLedger {
 public:
  /// room for `entries` boards and `positions` slots (the blob's size follows)
  void init(size_t entries, size_t positions) {
    this->entries_.assign(entries, LedgerEntry{});
    this->positions_.assign(positions, LedgerPosition{});
  }
  size_t capacity() const { return this->entries_.size(); }
  /// loaded (init): before it, nothing is kept (counters read 0)
  bool ready() const { return !this->positions_.empty(); }
  /// A counter's value (mWh; 0: none yet)
  uint64_t counter_mwh(uint32_t key) {
    LedgerPosition *p = this->position_(key, true);
    return p != nullptr ? p->value_mwh : 0;
  }
  /// A counter raised to mwh (never lowered; dirty from a whole Wh)
  void set_counter(uint32_t key, uint64_t mwh) {
    LedgerPosition *p = this->position_(key, true);
    if (p == nullptr || mwh <= p->value_mwh)
      return;
    this->dirty_ |= mwh / 1000 != p->value_mwh / 1000;
    p->value_mwh = mwh;
  }
  uint32_t day() const { return this->day_; }
  /// a hub day passed: boards not seen for LEDGER_AGE_DAYS that no position holds go
  void set_day(uint32_t day) {
    this->day_ = day;
    for (auto &e : this->entries_) {
      if (e.used && day - e.last_day >= LEDGER_AGE_DAYS && !this->held_(e)) {
        e = LedgerEntry{};
        this->dirty_ = true;
      }
    }
  }
  bool dirty() const { return this->dirty_; }
  void clear_dirty() { this->dirty_ = false; }

  LedgerEntry *find(const uint8_t *uid) {
    for (auto &e : this->entries_) {
      if (e.used && std::memcmp(e.uid, uid, 12) == 0)
        return &e;
    }
    return nullptr;
  }
  /// The board's entry; a new one (created: first contact) takes a free slot, else (full) the one not seen longest
  /// that no position holds.
  LedgerEntry *touch(const uint8_t *uid, bool *created) {
    *created = false;
    LedgerEntry *e = this->find(uid);
    if (e == nullptr) {
      e = this->free_slot_();
      if (e == nullptr)
        return nullptr;
      *e = LedgerEntry{};
      std::memcpy(e->uid, uid, 12);
      e->used = true;
      *created = true;
      this->dirty_ = true;
    }
    e->last_day = this->day_;
    return e;
  }
  /// LIFETIME's counters: the highest per field (in the wire's units: a change is one unit or more)
  void merge(LedgerEntry &e, const LifeCounters &rx) {
    int up = raise_(e.c.powered, rx.powered);
    up += raise_(e.c.lit, rx.lit);
    up += raise_(e.c.ups, rx.ups);
    up += raise_(e.c.dips, rx.dips);
    up += raise_(e.c.ovl, rx.ovl);
    up += raise_(e.c.shorts, rx.shorts);
    this->dirty_ |= up != 0;
  }
  /// Energy: the board's use since the last reading (add, from its count since power-up) on top, never below what the
  /// board says it has (exact): the sub-Wh a board drops at each cut stay here.
  void add_energy(LedgerEntry &e, uint64_t add, uint64_t exact) {
    uint64_t v = e.c.mwh + add;
    v = exact > v ? exact : v;
    if (v / 1000 != e.c.mwh / 1000)
      this->dirty_ = true;
    e.c.mwh = v;
  }
  /// What a sync gives the board (rx: its counters now): the ledger where it is ahead, each counter at most the
  /// receiver's cap for one merge (a board far behind, an erased one, comes back over a few power-ups); energy: the
  /// whole Wh it is missing with its own fraction counted (the sub-Wh stay with the hub: never more than the ledger).
  static LifeCounters sync_target(const LedgerEntry &e, const LifeCounters &rx) {
    LifeCounters t = rx;
    t.powered = up_to_(e.c.powered, rx.powered, LS_STEP_POWERED);
    const uint32_t dp = t.powered - rx.powered;
    t.lit = up_to_(e.c.lit, rx.lit, dp + 1);
    const uint32_t ev = dp * LS_STEP_EVENTS_PER_01H + LS_STEP_EVENTS;
    t.ups = up_to_(e.c.ups, rx.ups, ev);
    t.dips = up_to_(e.c.dips, rx.dips, ev);
    t.ovl = static_cast<uint16_t>(up_to_(e.c.ovl, rx.ovl, ev));
    t.shorts = static_cast<uint16_t>(up_to_(e.c.shorts, rx.shorts, ev));
    const uint64_t frac = rx.mwh % 1000, rx_wh = rx.mwh / 1000;
    uint64_t wh = e.c.mwh > frac ? (e.c.mwh - frac) / 1000 : 0;
    const uint64_t cap = rx_wh + static_cast<uint64_t>(dp) * LS_STEP_WH_PER_01H + LS_STEP_WH;
    wh = wh > cap ? cap : wh;
    t.mwh = (wh > rx_wh ? wh : rx_wh) * 1000 + frac;
    return t;
  }
  /// The ledger ahead of the receiver in one wire unit or more (0.1 h, one count, 1 Wh): a sync would raise it.
  static bool ahead(const LedgerEntry &e, const LifeCounters &rx) {
    const LifeCounters t = sync_target(e, rx);
    return t.powered > rx.powered || t.lit > rx.lit || t.ups > rx.ups || t.dips > rx.dips || t.ovl > rx.ovl ||
           t.shorts > rx.shorts || t.mwh / 1000 > rx.mwh / 1000;
  }
  /// The LIFE_SYNC POLL's arguments: UID16, LIFETIME bytes 0-15 and Wh (sync_target), RECORDS low 8 bits.
  static void sync_args(const LedgerEntry &e, const LifeCounters &rx, uint16_t uid16, uint8_t *out) {
    const LifeCounters t = sync_target(e, rx);
    std::memset(out, 0, LS_ARGS);
    ultra_put16(out + ULTRA_LS_UID16, uid16);
    uint8_t *l = out + ULTRA_LS_LIFETIME;
    ultra_put24(l + ULTRA_LT_POWERED, sat24_(t.powered));
    ultra_put24(l + ULTRA_LT_LIT, sat24_(t.lit));
    ultra_put24(l + ULTRA_LT_POWER_UPS, sat24_(t.ups));
    ultra_put16(l + ULTRA_LT_OVERLOAD_TRIPS, t.ovl);
    ultra_put16(l + ULTRA_LT_SHORT_TRIPS, t.shorts);
    ultra_put24(l + ULTRA_LT_DIPS, sat24_(t.dips));
    const uint64_t wh = t.mwh / 1000;
    ultra_put32(out + ULTRA_LS_TOTAL_WH, wh > 0xFFFFFFFFu ? 0xFFFFFFFFu : static_cast<uint32_t>(wh));
    out[ULTRA_LS_RECORDS] = static_cast<uint8_t>(e.records & 0xFF);
  }

  /// The position (key: the line's pin and the index); its board now: e. Returns its Energy (mWh): the board's ledger
  /// energy + its offset. Another board there (a replacement, a board moved in): it goes on from where it was (the
  /// offset re-based), never down; the first board a position ever has: its own energy.
  uint64_t position_energy(uint32_t key, const LedgerEntry &e, bool *swapped) {
    *swapped = false;
    LedgerPosition *p = this->position_(key);
    if (p == nullptr)
      return e.c.mwh;
    if (!p->have_uid || std::memcmp(p->uid, e.uid, 12) != 0) {
      if (p->have_uid) {
        *swapped = true;
        p->offset_mwh = static_cast<int64_t>(p->value_mwh) - static_cast<int64_t>(e.c.mwh);
      } else {
        p->offset_mwh = 0;
      }
      std::memcpy(p->uid, e.uid, 12);
      p->have_uid = true;
      this->dirty_ = true;
    }
    const int64_t v = p->offset_mwh + static_cast<int64_t>(e.c.mwh);
    const uint64_t value = v < 0 ? 0 : static_cast<uint64_t>(v);
    if (value > p->value_mwh) {
      this->dirty_ |= value / 1000 != p->value_mwh / 1000;
      p->value_mwh = value;
    }
    return p->value_mwh;
  }

  // ---- the stored form (two copies alternate: the newer valid one is loaded; a cut during a write keeps the other)
  size_t blob_size() const { return HDR + ENTRY * this->entries_.size() + POS * this->positions_.size() + 2; }
  void save(uint8_t *out, uint32_t generation) const {
    std::memset(out, 0, this->blob_size());
    ultra_put32(out, LEDGER_MAGIC);
    ultra_put32(out + 4, generation);
    ultra_put32(out + 8, this->day_);
    ultra_put16(out + 12, static_cast<uint16_t>(this->entries_.size()));
    ultra_put16(out + 14, static_cast<uint16_t>(this->positions_.size()));
    uint8_t *b = out + HDR;
    for (const auto &e : this->entries_) {
      if (e.used) {
        b[0] = 1;
        std::memcpy(b + 1, e.uid, 12);
        ultra_put32(b + 13, e.c.powered);
        ultra_put32(b + 17, e.c.lit);
        ultra_put32(b + 21, e.c.ups);
        ultra_put32(b + 25, e.c.dips);
        ultra_put16(b + 29, e.c.ovl);
        ultra_put16(b + 31, e.c.shorts);
        put64_(b + 33, e.c.mwh);
        ultra_put24(b + 41, e.records);
        ultra_put32(b + 44, e.last_day);
        ultra_put32(b + 48, e.sess_boot);
        ultra_put32(b + 52, e.sess_up);
        ultra_put32(b + 56, e.sess_ups);
        b[60] = e.sess ? 1 : 0;
      }
      b += ENTRY;
    }
    for (const auto &p : this->positions_) {
      if (p.used) {
        b[0] = static_cast<uint8_t>(1 | (p.have_uid ? 2 : 0) | (p.counter ? 4 : 0));
        ultra_put32(b + 1, p.key);
        std::memcpy(b + 5, p.uid, 12);
        put64_(b + 17, static_cast<uint64_t>(p.offset_mwh));
        put64_(b + 25, p.value_mwh);
      }
      b += POS;
    }
    ultra_put16(out + this->blob_size() - 2, ultra_crc16(out, static_cast<uint32_t>(this->blob_size() - 2)));
  }
  /// A stored copy: valid (magic, size, CRC)? Its generation and how many entries / positions it uses (the
  /// allocation it needs: a copy stored with more or fewer receivers loads too).
  static bool peek(const uint8_t *in, size_t n, uint32_t *generation, size_t *entries, size_t *positions) {
    size_t ne, np;
    if (!valid_(in, n, &ne, &np))
      return false;
    *generation = ultra_get32(in + 4);
    *entries = *positions = 0;
    const uint8_t *b = in + HDR;
    for (size_t i = 0; i < ne; i++, b += ENTRY)
      *entries += b[0] & 1;
    for (size_t i = 0; i < np; i++, b += POS)
      *positions += b[0] & 1;
    return true;
  }
  /// A stored copy (any size: its entries and positions moved into this allocation, as many as fit), replacing
  /// everything (what isn't in it is gone); false: not a valid one (nothing changed).
  bool load(const uint8_t *in, size_t n, uint32_t *generation) {
    size_t ne, np;
    if (!valid_(in, n, &ne, &np))
      return false;
    for (auto &e : this->entries_)
      e = LedgerEntry{};
    for (auto &p : this->positions_)
      p = LedgerPosition{};
    *generation = ultra_get32(in + 4);
    this->day_ = ultra_get32(in + 8);
    const uint8_t *b = in + HDR;
    size_t to = 0;
    for (size_t i = 0; i < ne; i++, b += ENTRY) {
      if (!(b[0] & 1) || to >= this->entries_.size())
        continue;
      LedgerEntry &e = this->entries_[to++];
      e.used = true;
      std::memcpy(e.uid, b + 1, 12);
      e.c.powered = ultra_get32(b + 13);
      e.c.lit = ultra_get32(b + 17);
      e.c.ups = ultra_get32(b + 21);
      e.c.dips = ultra_get32(b + 25);
      e.c.ovl = static_cast<uint16_t>(ultra_get16(b + 29));
      e.c.shorts = static_cast<uint16_t>(ultra_get16(b + 31));
      e.c.mwh = get64_(b + 33);
      e.records = ultra_get24(b + 41);
      e.last_day = ultra_get32(b + 44);
      e.sess_boot = ultra_get32(b + 48);
      e.sess_up = ultra_get32(b + 52);
      e.sess_ups = ultra_get32(b + 56);
      e.sess = (b[60] & 1) != 0;
    }
    to = 0;
    for (size_t i = 0; i < np; i++, b += POS) {
      if (!(b[0] & 1) || to >= this->positions_.size())
        continue;
      LedgerPosition &p = this->positions_[to++];
      p.used = true;
      p.have_uid = (b[0] & 2) != 0;
      p.counter = (b[0] & 4) != 0;
      p.key = ultra_get32(b + 1);
      std::memcpy(p.uid, b + 5, 12);
      p.offset_mwh = static_cast<int64_t>(get64_(b + 17));
      p.value_mwh = get64_(b + 25);
    }
    return true;
  }

  static constexpr uint8_t LS_ARGS = 23;   // (the POLL's arguments: ULTRA_LS_RECORDS + 1)

 protected:
  static constexpr size_t HDR = 16, ENTRY = 64, POS = 33;
  static bool valid_(const uint8_t *in, size_t n, size_t *ne, size_t *np) {
    if (n < HDR + 2 || ultra_get32(in) != LEDGER_MAGIC || ultra_get16(in + n - 2) != ultra_crc16(in, static_cast<uint32_t>(n - 2)))
      return false;
    *ne = ultra_get16(in + 12);
    *np = ultra_get16(in + 14);
    return HDR + ENTRY * *ne + POS * *np + 2 == n;
  }
  static bool raise_(uint32_t &v, uint32_t rx) {
    if (rx <= v)
      return false;
    v = rx;
    return true;
  }
  static bool raise_(uint16_t &v, uint16_t rx) {
    if (rx <= v)
      return false;
    v = rx;
    return true;
  }
  static uint32_t sat24_(uint32_t v) { return v > 0xFFFFFFu ? 0xFFFFFFu : v; }
  static uint32_t up_to_(uint32_t ledger, uint32_t rx, uint32_t step) {
    return ledger <= rx ? rx : ledger - rx > step ? rx + step : ledger;
  }
  static void put64_(uint8_t *p, uint64_t v) {
    ultra_put32(p, static_cast<uint32_t>(v));
    ultra_put32(p + 4, static_cast<uint32_t>(v >> 32));
  }
  static uint64_t get64_(const uint8_t *p) {
    return static_cast<uint64_t>(ultra_get32(p)) | static_cast<uint64_t>(ultra_get32(p + 4)) << 32;
  }
  LedgerEntry *free_slot_() {
    LedgerEntry *oldest = nullptr;
    for (auto &e : this->entries_) {
      if (!e.used)
        return &e;
      if (this->held_(e))
        continue;
      if (oldest == nullptr || static_cast<int32_t>(e.last_day - oldest->last_day) < 0)
        oldest = &e;
    }
    return oldest;
  }
  bool held_(const LedgerEntry &e) const {
    for (const auto &p : this->positions_) {
      if (p.used && p.have_uid && std::memcmp(p.uid, e.uid, 12) == 0)
        return true;
    }
    return false;
  }
  LedgerPosition *position_(uint32_t key, bool counter = false) {
    LedgerPosition *free = nullptr;
    for (auto &p : this->positions_) {
      if (p.used && p.key == key && p.counter == counter)
        return &p;
      if (!p.used && free == nullptr)
        free = &p;
    }
    if (free != nullptr) {
      *free = LedgerPosition{};
      free->used = true;
      free->key = key;
      free->counter = counter;
    }
    return free;
  }

  std::vector<LedgerEntry> entries_;
  std::vector<LedgerPosition> positions_;
  uint32_t day_{0};
  bool dirty_{false};
};

inline LifetimeLedger &ultra_ledger() {
  static LifetimeLedger l;
  return l;
}

/// One line's side of the ledger (its receivers): every LIFETIME answer merged (the highest per counter); first
/// contact adopts; after a board's power-up (or once a day on a long run) the ledger given back when it is ahead (one
/// LIFE_SYNC POLL; the result: applied / no-op / capped: done for this power-up; refused: read again, once more, then
/// "needs attention"); before a planned power-off a final read and that sync (prepare()). The positions' Energy.
///
/// Energy: each ENERGY reading adds the board's use since the one before (its count since power-up, which a sync
/// never moves), never below its own lifetime. The readings wait for the next STATUS: its uptime dates a power-up, so
/// a reading of the new one starts its count and one of the old one goes on (whatever order the pages came in); the
/// 32-bit count may wrap within a power-up. A session's first reading at a position (after a power-up, the ESP32's
/// restart, another board here): the board's session checkpoint in the ledger decides, by its UID wherever it was
/// read last (its power-ups count, else its uptime): the same session goes on from the count it was last counted at
/// (the ESP32 restarted under it, a powered board moved or back), a new one counts all of it, a board whose entry was
/// just made from its lifetime (first contact) only what comes after. A power-off alone never counts as a restart (a
/// supply that stayed on: its use isn't counted twice).
class LedgerKeeper {
 public:
  /// what on_updates() saw (the hub logs it)
  enum Event : uint8_t { EV_NONE = 0, EV_NEW_BOARD = 1, EV_RESTORE = 2, EV_ATTENTION = 4, EV_SYNCED = 8 };

  /// key_base: the line's (its id's hash, low 16 bits clear: the receiver's index goes there; 0xFFFF: its port's
  /// counter); boot_ms: millis() when the ESP32 started (0; the tests: their hub's start)
  void attach(LifetimeLedger *ledger, uint32_t key_base, uint16_t receivers, uint32_t boot_ms = 0) {
    this->ledger_ = ledger;
    this->key_base_ = key_base & 0xFFFF0000u;
    this->rx_.assign(receivers, Rx{});
    this->up_ms_ = 0;
    this->last_ms_ = boot_ms;
  }
  uint32_t key(uint16_t i) const { return this->key_base_ | i; }
  uint32_t port_key() const { return this->key_base_ | 0xFFFFu; }

  /// A receiver's news (TelemetryPoller::take_updates bits): merge, sync, the position. Returns Event bits.
  uint8_t on_updates(TelemetryPoller &p, uint16_t i, uint32_t updates, uint32_t now_ms) {
    if (this->ledger_ == nullptr || i >= this->rx_.size())
      return EV_NONE;
    Rx &x = this->rx_[i];
    const ReceiverData &d = p.data(i);
    uint8_t ev = EV_NONE;
    const uint64_t up = this->up_(now_ms);
    if (updates & TU_RESTART) {   // a power-up (or another board): what it says now, then maybe a sync
      session_reset_(x);
      if (!d.have_identity)
        x.pending = 0;          // another board: the one before's last readings dropped (a hot swap: rare)
    }
    // the board here by its full UID (whatever its UID16 says): another one starts afresh, nothing of the one before
    // (its readings, its count since power-up) carried over
    if (d.have_identity && (!x.have_uid || std::memcmp(x.uid, d.uid, 12) != 0)) {
      if (x.have_uid) {
        session_reset_(x);
        x.pending = 0;
        x.have_boot = false;
      }
      std::memcpy(x.uid, d.uid, 12);
      x.have_uid = true;
    }
    if ((updates & (1u << TP_ENERGY)) && d.have_energy && x.pending < PENDING)
      x.reading[x.pending++] = Reading{d.boot_mwh, receiver_counters(d).mwh, now_ms};   // (dated by the next STATUS)
    if (!d.have_identity)
      return ev;
    const bool life = (updates & (1u << TP_LIFETIME)) && d.have_lifetime;
    const bool status = (updates & (1u << TP_STATUS)) && d.have_status;
    const bool sync = (updates & (1u << TP_SYNC)) && d.have_sync;
    if (!(life || status || sync || (updates & (1u << (TP_IDENTITY))) || x.pending != 0))
      return ev;
    bool created = false;
    LedgerEntry *e = this->ledger_->touch(d.uid, &created);
    if (e == nullptr)
      return ev;
    x.entry = e;
    if (created) {
      x.adopted = true;   // first contact: its own counters, never a sync this power-up
      x.phase = DONE;
      x.last_sync_ms = now_ms == 0 ? 1 : now_ms;
    }
    const LifeCounters rc = receiver_counters(d);
    if (life || sync) {
      if (e->records != 0 && d.lt_records < e->records && !created)
        ev |= EV_RESTORE;   // its record went back (erased): the ledger gives it back over its next power-ups
      e->records = d.lt_records;
    }
    if (life || sync)
      this->ledger_->merge(*e, rc);
    x.life_fresh |= life || sync;
    // the readings in, dated by this STATUS; a new session's first ones once its LIFETIME was read (its power-ups
    // count tells a session the ledger knows from a new one; at most PENDING readings long)
    const bool wait_life = (x.restart_due || !x.have_boot) && (d.features & ULTRA_FEAT_LIFETIME) && !x.life_fresh &&
                           x.pending < PENDING;
    if (status && !wait_life)
      this->energy_(x, *e, d, now_ms, up);
    x.lt_read |= life || !(d.features & ULTRA_FEAT_LIFETIME);
    x.en_read |= !(d.features & ULTRA_FEAT_ENERGY);
    if (sync)
      ev |= this->sync_result_(p, i, d, now_ms);
    // the position's Energy (a board that came: a new one, or one moved here), once this board's energy is known
    if (x.energy_seen) {
      bool swapped = false;
      x.energy_mwh = this->ledger_->position_energy(this->key(i), *e, &swapped);
      x.have_energy = true;
      if (swapped)
        ev |= EV_NEW_BOARD;
    }
    // a sync when the ledger is ahead, once both pages were read since this power-up
    const bool daily = x.phase == DONE && !x.merged && x.last_sync_ms != 0 && now_ms - x.last_sync_ms >= SYNC_DAILY_MS;
    if (daily) {
      x.phase = WAIT;
      x.lt_read = x.en_read = false;
      p.read_now(i, TP_LIFETIME, now_ms);
      p.read_now(i, TP_ENERGY, now_ms);
    }
    if (x.phase == WAIT && x.lt_read && x.en_read) {
      x.last_sync_ms = now_ms == 0 ? 1 : now_ms;
      if ((d.features & ULTRA_FEAT_LIFE_SYNC) && LifetimeLedger::ahead(*e, rc)) {
        uint8_t args[LifetimeLedger::LS_ARGS];
        LifetimeLedger::sync_args(*e, rc, p.uid16(i), args);
        p.request_sync(i, args, now_ms);
        x.phase = SENT;
      } else {
        x.phase = DONE;
      }
    }
    return ev;
  }

  /// Before a planned power-off: a final LIFETIME / ENERGY read of every receiver that answers, then a sync if the
  /// ledger is ahead. first: this power-off's first call. true: ready for the notice (the port gives up after its own
  /// time).
  bool prepare(TelemetryPoller &p, uint32_t now_ms, bool first) {
    if (first) {
      for (uint16_t i = 0; i < this->rx_.size(); i++) {
        Rx &x = this->rx_[i];
        x.final_wait = p.answering(i) && p.data(i).have_identity;
        if (!x.final_wait)
          continue;
        x.lt_read = x.en_read = false;
        x.tries = 0;
        x.phase = WAIT;
        p.read_now(i, TP_LIFETIME, now_ms);
        p.read_now(i, TP_ENERGY, now_ms);
        p.read_now(i, TP_STATUS, now_ms + 50);   // (the reading dated: into the ledger)
      }
    }
    for (uint16_t i = 0; i < this->rx_.size(); i++) {
      const Rx &x = this->rx_[i];
      if (x.final_wait && p.answering(i) && (x.phase != DONE || x.pending != 0))
        return false;
    }
    return true;
  }
  /// The position's Energy (Wh; NAN until its board's ENERGY was read once)
  double energy_wh(uint16_t i) const {
    return i < this->rx_.size() && this->rx_[i].have_energy ? this->rx_[i].energy_mwh / 1000.0 : -1.0;
  }
  /// The ledger's counters of the board at receiver i (nullptr: not known yet)
  const LifeCounters *counters(uint16_t i) const {
    return i < this->rx_.size() && this->rx_[i].entry != nullptr ? &this->rx_[i].entry->c : nullptr;
  }
  bool attention(uint16_t i) const { return i < this->rx_.size() && this->rx_[i].attention; }
  uint8_t last_result(uint16_t i) const { return i < this->rx_.size() ? this->rx_[i].result : 0xFF; }
  bool sync_sent(uint16_t i) const { return i < this->rx_.size() && this->rx_[i].phase == SENT; }

 protected:
  enum Phase : uint8_t { DONE = 0, WAIT, SENT };
  static constexpr uint8_t PENDING = 4;
  struct Reading {
    uint32_t boot;   // its count since power-up (mWh)
    uint64_t exact;  // its lifetime then (mWh)
    uint32_t ms;     // when it was read
  };
  struct Rx {
    Reading reading[PENDING]{};   // ENERGY readings waiting for the next STATUS
    uint8_t pending{0};
    uint32_t boot_last{0};        // the count of the last reading counted (this power-up)
    bool have_boot{false};        // ... known (since the hub started, for this board)
    bool restart_due{false};      // a power-up seen: its first reading not counted yet
    uint8_t uid[12]{};            // the board whose session this is (its full UID)
    bool have_uid{false};
    bool life_fresh{false};       // its LIFETIME read in this session (its power-ups count: which session)
    LedgerEntry *entry{nullptr};
    uint64_t energy_mwh{0};
    bool have_energy{false};
    bool lt_read{false}, en_read{false};   // read since this power-up (or the sync before)
    bool energy_seen{false};               // its ENERGY read since this power-up / since the hub started
    Phase phase{WAIT};
    uint8_t tries{0};
    bool merged{false};      // a sync applied this power-up (the board takes one)
    bool adopted{false};
    bool attention{false};
    bool final_wait{false};
    uint8_t result{0xFF};
    uint32_t last_sync_ms{0};
  };

  /// a power-up or another board: what it says now, then maybe a sync
  static void session_reset_(Rx &x) {
    x.life_fresh = false;
    x.lt_read = x.en_read = false;
    x.phase = WAIT;
    x.tries = 0;
    x.merged = false;
    x.adopted = false;
    x.entry = nullptr;        // (found again by its UID)
    x.energy_seen = false;    // (its position follows it once its ENERGY was read)
    x.restart_due = true;     // (dated by its STATUS)
  }

  /// the hub's time since it started (64-bit: millis() wraps)
  uint64_t up_(uint32_t now_ms) {
    this->up_ms_ += static_cast<uint32_t>(now_ms - this->last_ms_);
    this->last_ms_ = now_ms;
    return this->up_ms_;
  }

  /// The waiting readings into the ledger (a STATUS was just read: a power-up since is known and dated)
  void energy_(Rx &x, LedgerEntry &e, const ReceiverData &d, uint32_t now_ms, uint64_t up) {
    if (!x.have_boot)
      x.restart_due = true;   // the first since the hub started (or since this board came here): its session dated
    uint32_t start_ms = 0, base = 0;
    if (x.restart_due) {
      start_ms = now_ms - d.uptime_s * 1000u;
      if (x.adopted) {
        base = UINT32_MAX;   // (its entry just made from its lifetime: from its first reading on)
      } else if (e.sess) {
        // the session the ledger last counted (its UID's, at any position): still the same one (its power-ups count
        // unchanged; without LIFETIME: its uptime not less) -> on from that count; else a new one: all of it new
        const bool same = (d.features & ULTRA_FEAT_LIFETIME) && d.have_lifetime && e.sess_ups != SESS_UPS_UNKNOWN
                              ? d.lt_power_ups == e.sess_ups
                              : d.uptime_s >= e.sess_up;
        base = same ? e.sess_boot : 0;
      } else {
        // (no session known: all of it new when it came on after the hub, else from its first reading)
        base = static_cast<uint64_t>(d.uptime_s) * 1000u <= up + BOOT_SLACK_MS ? 0 : UINT32_MAX;
      }
    }
    for (uint8_t k = 0; k < x.pending; k++) {
      const Reading &r = x.reading[k];
      // this session's first reading: read after it started (its uptime is whole seconds: or its count went back)
      if (x.restart_due &&
          (static_cast<int32_t>(r.ms - start_ms) >= 0 || (x.have_boot && r.boot < x.boot_last))) {
        x.boot_last = base == UINT32_MAX ? r.boot : base;
        x.have_boot = true;
        x.restart_due = false;
      }
      uint64_t add = 0;
      if (x.have_boot) {
        if (r.boot >= x.boot_last)
          add = r.boot - x.boot_last;
        else if (x.boot_last > 0xF0000000u && r.boot < 0x10000000u)
          add = static_cast<uint32_t>(r.boot - x.boot_last);   // (its 32-bit count wrapped)
        else
          add = r.boot;   // (it went back without a STATUS restart yet: a new power-up from 0)
      }
      this->ledger_->add_energy(e, add, r.exact);
      x.boot_last = r.boot;
      x.have_boot = true;
      x.energy_seen = true;
      if (!x.restart_due) {   // (the session it is counted in now: its checkpoint)
        e.sess_boot = r.boot;
        e.sess_up = d.uptime_s;
        e.sess_ups = (d.features & ULTRA_FEAT_LIFETIME) && d.have_lifetime ? d.lt_power_ups : SESS_UPS_UNKNOWN;
        e.sess = true;
      }
    }
    if (x.pending != 0)
      x.en_read = true;
    x.pending = 0;
  }

  uint8_t sync_result_(TelemetryPoller &p, uint16_t i, const ReceiverData &d, uint32_t now_ms) {
    Rx &x = this->rx_[i];
    x.result = d.ls_result;
    switch (d.ls_result) {
      case ULTRA_LS_R_APPLIED:
        x.merged = true;
        x.phase = DONE;
        x.attention = false;
        if (x.entry != nullptr && x.entry->sess && x.entry->sess_ups != SESS_UPS_UNKNOWN)
          x.entry->sess_ups = d.lt_power_ups;   // (the same session: its power-ups count raised by the sync)
        return EV_SYNCED;
      case ULTRA_LS_R_NOOP:
      case ULTRA_LS_R_CAPPED:   // (capped: the rest at its next power-up)
      case ULTRA_LS_R_ONCE:     // (it took one this power-up already: the hub restarted under it; the rest at its next)
        x.phase = DONE;
        return EV_NONE;
      default:   // refused (another board's UID16, a record written since): read again, once more
        if (x.tries++ == 0) {
          x.phase = WAIT;
          x.lt_read = x.en_read = false;
          p.read_now(i, TP_LIFETIME, now_ms);
          p.read_now(i, TP_ENERGY, now_ms);
          return EV_NONE;
        }
        x.phase = DONE;
        if (!x.attention) {
          x.attention = true;
          return EV_ATTENTION;
        }
        return EV_NONE;
    }
  }

  LifetimeLedger *ledger_{nullptr};
  uint32_t key_base_{0};
  std::vector<Rx> rx_;
  uint64_t up_ms_{0};
  uint32_t last_ms_{0};
};

}  // namespace esphome::dig2analog_ultra
