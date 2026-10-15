static void reboot(size_t n) {
  ledger = Ledger{}; receivers = n; now_ms = 0;
  g_ledger_loaded = false;
  g_ledger_generation = g_ledger_saved_ms = g_ledger_day_ms = 0;
  pending.clear(); order.clear(); writes.clear(); sync_count = 0;
  fail_save = false; fail_sync_at = -1; cut_after = -1;
}
static std::vector<std::vector<uint8_t>> save_cases() {
  std::vector<std::vector<uint8_t>> states;
  stored.clear(); reboot(28);
  UltraHub h; h.ledger_service_(); populate(ledger, 20, 8);
  uint8_t uid[12] = {};
  ultra_put32(uid, 1);
  for (int i = 0; i < 3; i++) {
    ledger.add_power_up(*ledger.find(uid)); h.ledger_save_(true);
  }
  states.push_back(canonical(ledger));
  auto stable = stored;
  for (int failure = 0; failure < 4; failure++) {
    stored = stable; reboot(28); h.ledger_service_();
    ledger.add_power_up(*ledger.find(uid));
    fail_save = failure == 0;
    fail_sync_at = failure == 1 ? 0 : failure == 2 ? 1 : -1;
    cut_after = failure == 3 ? 0 : -1;
    h.ledger_save_(true);
    CHECK(ledger.dirty() && g_ledger_generation == 3);
    states.push_back(canonical(ledger));
    fail_save = false; fail_sync_at = -1; cut_after = -1;
    h.ledger_save_(true); CHECK(!ledger.dirty() && g_ledger_generation == 4);
    reboot(28); h.ledger_service_(); states.push_back(canonical(ledger));
  }
  return states;
}
