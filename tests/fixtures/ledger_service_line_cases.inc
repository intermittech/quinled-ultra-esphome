static std::vector<std::vector<uint8_t>> run() {
  store::stored.clear(); store::reboot(1);
  store::UltraHub storage;
  storage.ledger_service_();
  line::UltraHub hub;
  int uart;
  line::Light light{0}; light.on = true;
  hub.rw_uart_ = &uart; hub.ready = true; hub.lights_ = {&light};
  hub.poller_.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  hub.keeper_.attach(&store::ledger, 1u << 16, 1);
  auto &d = const_cast<ReceiverData &>(hub.poller_.data(0));
  d.uid[0] = 1; d.have_identity = true;
  hub.keeper_.on_updates(hub.poller_, 0, 1u << TP_IDENTITY, 0);
  std::vector<std::vector<uint8_t>> states;
  uint32_t last_gen = 0;
  auto snapshot = [&]() {
    const auto gen = store::g_ledger_generation;
    const auto &raw = store::stored.at(store::fnv1_hash(store::LEDGER_KEYS[gen & 1]));
    LifetimeLedger decoded;
    decoded.init(32, 32);
    uint32_t read_gen;
    CHECK(decoded.load(raw.data(), raw.size(), &read_gen) && read_gen == gen);
    states.push_back(canonical(decoded));
    states.push_back({static_cast<uint8_t>(gen)});
  };
  for (uint32_t t = 0; t <= 2 * LEDGER_SAVE_MS + 73; t++) {
    store::now_ms = line::now_ms = t;
    storage.ledger_service_();   // production ordering: save before this loop's observation
    if (store::g_ledger_generation != last_gen) {
      CHECK(t == store::g_ledger_generation * LEDGER_SAVE_MS);
      snapshot(); last_gen = store::g_ledger_generation;
    }
    light.on = t % 17137 < 12345;
    hub.ledger_line_();
  }
  CHECK(last_gen == 2);
  storage.ledger_save_(true);   // forced save between cadence ticks
  snapshot();
  return states;
}
