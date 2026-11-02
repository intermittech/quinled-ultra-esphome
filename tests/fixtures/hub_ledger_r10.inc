// Frozen differential reference: a73ff9c59e44cb76f21343e2877133ceb875f9b5 (round 10).
// The ESP32's one lifetime ledger (every line's receivers: a board moved between lines keeps its entry). Two stored
// copies alternate: the newer valid one loads (a cut while one is written keeps the other). Each has its size stored
// beside it (written after it): a copy stored with more or fewer receivers loads too. Stored at most every 15 min
// while it changes by a wire unit, and at a reboot (not at every planned power-off: a port waking 100 times a day
// would write it 100 times; a power loss of the ESP32 loses at most those minutes, counted less, never more); its
// stored form is built only for a save (no copy of it kept in RAM).
namespace {
const char *const LEDGER_KEYS[2] = {"ultra_ledger_a", "ultra_ledger_b"};
const char *const LEDGER_SIZE_KEYS[2] = {"ultra_ledger_a_size", "ultra_ledger_b_size"};
ESPPreferenceObject g_ledger_pref[2], g_ledger_size[2];
uint32_t g_ledger_generation = 0, g_ledger_saved_ms = 0, g_ledger_day_ms = 0;
bool g_ledger_loaded = false;
}

void UltraHub::ledger_service_() {
  LifetimeLedger &l = ultra_ledger();
  const uint32_t now = millis();
  if (!g_ledger_loaded) {   // the first loop: every line has its receivers counted
    g_ledger_loaded = true;
    // the newer valid copy (by its stored size: whatever the receivers were then), loaded alone (what the newer one
    // dropped stays dropped)
    uint32_t newest = 0;
    size_t used_e = 0, used_p = 0;
    bool any = false;
    std::vector<uint8_t> best;
    for (int k = 0; k < 2; k++) {
      g_ledger_size[k] = global_preferences->make_preference(sizeof(uint32_t), fnv1_hash(LEDGER_SIZE_KEYS[k]), true);
      uint32_t size = 0;
      if (!g_ledger_size[k].load(reinterpret_cast<uint8_t *>(&size), sizeof(size)) || size < 18 || size > LEDGER_MAX_BLOB)
        continue;
      ESPPreferenceObject stored = global_preferences->make_preference(size, fnv1_hash(LEDGER_KEYS[k]), true);
      std::vector<uint8_t> b(size);
      uint32_t gen = 0;
      size_t ne = 0, np = 0;
      if (stored.load(b.data(), b.size()) && LifetimeLedger::peek(b.data(), b.size(), &gen, &ne, &np) &&
          (!any || static_cast<int32_t>(gen - newest) > 0)) {
        best.swap(b);
        newest = gen;
        used_e = ne;
        used_p = np;
        any = true;
      }
    }
    // (blocks of 32: a receiver more or less keeps the size; never fewer than the stored copy uses)
    const size_t n = ledger_receivers();
    const size_t pos = std::max((n + 31) / 32 * 32, (used_p + 31) / 32 * 32);
    const size_t room = std::max((n + std::max<size_t>(4, n / 2) + 31) / 32 * 32, (used_e + 31) / 32 * 32);
    l.init(room, pos);
    if (any)
      l.load(best.data(), best.size(), &newest);
    for (int k = 0; k < 2; k++)
      g_ledger_pref[k] = global_preferences->make_preference(l.blob_size(), fnv1_hash(LEDGER_KEYS[k]), true);
    g_ledger_generation = newest;
    g_ledger_saved_ms = g_ledger_day_ms = now;
    ESP_LOGD(TAG, "Lifetime ledger: %u board(s) room, %s", static_cast<unsigned>(l.capacity()),
             any ? "loaded" : "new");
    return;
  }
  if (now - g_ledger_day_ms >= LEDGER_DAY_MS) {   // the ledger's days: the hub's time on
    g_ledger_day_ms += LEDGER_DAY_MS;
    l.set_day(l.day() + 1);
  }
  if (l.dirty() && now - g_ledger_saved_ms >= LEDGER_SAVE_MS)
    ledger_save_(false);
}

void UltraHub::ledger_save_(bool now) {
  LifetimeLedger &l = ultra_ledger();
  if (!g_ledger_loaded || (!l.dirty() && !now))
    return;
  if (l.dirty()) {
    // the next generation into the copy not in use: its blob on the flash first, its size only then (a size never
    // names a blob that isn't there), the copy in use whole throughout. Clean, and that generation the one in use,
    // once both are there; a failure stays dirty and the next save, periodic or forced, writes the same copy again
    // (so does one without the memory for its stored form)
    const size_t n = l.blob_size();
    std::unique_ptr<uint8_t[]> blob(new (std::nothrow) uint8_t[n]);
    if (!blob) {
      ESP_LOGW(TAG, "Lifetime ledger: no memory to store it; again at the next save");
      g_ledger_saved_ms = millis();
      return;
    }
    const uint32_t gen = g_ledger_generation + 1;
    l.save(blob.get(), gen);
    const uint32_t size = static_cast<uint32_t>(n);
    bool ok = g_ledger_pref[gen & 1].save(blob.get(), n) && global_preferences->sync();
    ok = ok && g_ledger_size[gen & 1].save(reinterpret_cast<const uint8_t *>(&size), sizeof(size)) &&
         global_preferences->sync();
    if (ok) {
      g_ledger_generation = gen;
      l.clear_dirty();
    } else {
      ESP_LOGW(TAG, "Lifetime ledger: storing it failed; again at the next save");
    }
  } else {
    global_preferences->sync();   // (a planned power-off, a reboot: the rest on the flash now)
  }
  g_ledger_saved_ms = millis();
}
