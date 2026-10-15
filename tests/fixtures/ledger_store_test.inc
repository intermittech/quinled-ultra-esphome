// Included twice: the real round-11 hub methods and their frozen round-10 counterparts, same backend model.
using LifetimeLedger = Ledger;
static Ledger ledger;
static size_t receivers;
static uint32_t now_ms;
static Ledger &ultra_ledger() { return ledger; }
static size_t &ledger_receivers() { return receivers; }
static uint32_t millis() { return now_ms; }
static uint32_t fnv1_hash(const char *s) { return ledger_key(s); }
static std::map<uint32_t, std::vector<uint8_t>> stored, pending;
static std::vector<uint32_t> order, writes;
static int cut_after = -1, fail_sync_at = -1, sync_count = 0;
static bool fail_save = false;
static size_t peak_bytes = 0, peak_blocks = 0;
static void account(size_t readback = 0) {
  size_t bytes = caller_size + readback, blocks = (caller_blob != nullptr) + (readback != 0);
  for (const auto &kv : pending) if (kv.second.size() > 4) { bytes += kv.second.size(); blocks++; }
  peak_bytes = std::max(peak_bytes, bytes); peak_blocks = std::max(peak_blocks, blocks);
}
struct ESPPreferenceObject {
  uint32_t key{};
  bool save(const uint8_t *data, size_t n) {
    if (fail_save) return false;
    if (!pending.count(key)) order.push_back(key);
    pending[key].assign(data, data + n);  // ESP32 backend owns a copy before returning.
    account();
    return true;
  }
  bool load(uint8_t *data, size_t n) {
    auto &src = pending.count(key) ? pending : stored;
    auto it = src.find(key);
    if (it == src.end() || it->second.size() != n) return false;
    std::copy(it->second.begin(), it->second.end(), data);
    return true;
  }
};
struct Preferences {
  ESPPreferenceObject make_preference(size_t, uint32_t key, bool) { return {key}; }
  bool sync() {
    bool ok = sync_count++ != fail_sync_at;
    for (auto key : order) {
      // ESP32 is_changed reads the old NVS value if its length equals the pending value's length.
      const auto it = stored.find(key);
      if (it != stored.end() && it->second.size() == pending[key].size()) account(it->second.size());
      if (cut_after == 0) { ok = false; break; }
      if (cut_after > 0) cut_after--;
      if (ok) { stored[key] = pending[key]; writes.push_back(key); }
    }
    pending.clear(); order.clear();
    return ok;
  }
};
static Preferences prefs;
static Preferences *global_preferences = &prefs;
struct UltraHub { void ledger_service_(); static void ledger_save_(bool); };
