static std::vector<std::vector<uint8_t>> run_line(uint64_t &ticks, uint64_t &span) {
  Ledger ledger;
  ledger.init(8, 8);
  UltraHub h;
  Port port;
  int uart;
  h.rw_uart_ = &uart; h.port_ = &port;
  h.poller_.init(3, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  now_ms = UINT32_MAX - 10000;
  h.keeper_.attach(&ledger, 1u << 16, 3, now_ms);
  for (unsigned i = 0; i < 3; i++) {
    auto &d = const_cast<ReceiverData &>(h.poller_.data(i));
    ultra_put32(d.uid, i + 1); d.have_identity = true;
    d.features = ULTRA_FEAT_LIFETIME | ULTRA_FEAT_ENERGY | ULTRA_FEAT_LIFE_SYNC;
    d.have_status = d.have_lifetime = d.have_energy = true;
    bool made;
    if (i < 2) ledger.touch(d.uid, &made);   // known boards plus one first-contact adoption
  }
  Light a{0}, b{1}, c{2}, shared{1}, invalid{9};
  h.lights_ = {&a, &b, &c, &shared, &invalid};
  port.on = true; port.ms = now_ms; h.ready = true;
  std::vector<std::vector<uint8_t>> snapshots;
  const uint32_t steps[] = {1, 7, 13, 29, 31, 47, 51, 97, 1000, 1000000};
  span = 0;
  for (uint32_t step = 0; step < 400000; step++) {
    const uint32_t dt = step < 10000 ? 1 : steps[step % 10];
    now_ms += dt; span += dt;
    a.on = step % 97 < 3;   // includes pulses entirely inside one 50 ms interval
    b.on = step % 31 < 17;
    c.on = step % 113 < 82;
    shared.on = step % 43 < 22;
    if (step % 79 == 0) { a.fading = true; a.off_ms = now_ms + 23; }
    if (step % 1009 == 0) { port.on = !port.on; port.ms = now_ms; }
    h.ready = port.on && step % 1009 > 2;
    h.ledger_line_();
    if (step % 31 == 0 && h.ready) {   // late discovery, session checkpoints, energy tails and sync decisions
      for (unsigned i = 0; i < 3; i++) {
        auto &d = const_cast<ReceiverData &>(h.poller_.data(i));
        d.uptime_s = (now_ms - port.ms) / 1000;
        d.boot_mwh = (now_ms - port.ms) / (100 + i);
        d.total_wh = 1000 + d.boot_mwh / 1000;
        d.power_mw = 36000;
        d.lt_power_ups = 2; d.lt_records = step / 1009;
        const uint8_t ev = h.keeper_.on_updates(h.poller_, i, (1u << TP_IDENTITY) | (1u << TP_STATUS) |
                                                (1u << TP_ENERGY) | (1u << TP_LIFETIME), now_ms);
        snapshots.push_back({ev, static_cast<uint8_t>(h.keeper_.sync_sent(i))});
      }
    }
    if (step % 103 == 0) {
      for (unsigned i = 0; i < 3; i++) {
        const auto *count = h.keeper_.counters(i);
        snapshots.push_back({static_cast<uint8_t>(count != nullptr),
                             static_cast<uint8_t>(count ? count->lit : 0)});
      }
      snapshots.push_back(canonical(ledger));   // full counters and sub-unit remainders, not just wire units
      snapshots.push_back({static_cast<uint8_t>(ledger.dirty())});
      ledger.clear_dirty();
    }
    if (step % 1069 == 0) {
      Ledger copy = ledger;
      snapshots.push_back(canonical(copy));
    }
  }
  h.keeper_.cut(now_ms + 7);
  snapshots.push_back(canonical(ledger));
  ticks = h.keeper_.ticks;
  return snapshots;
}

static std::vector<uint8_t> bench_line(unsigned n, double &ns, uint64_t &ticks) {
  Ledger ledger;
  ledger.init(128, 96);
  UltraHub h;
  int uart;
  h.rw_uart_ = &uart; h.ready = true;
  h.poller_.init(n, ULTRA_RW_RATE_1M, TelemetryIntervals{});
  h.keeper_.attach(&ledger, 1u << 16, n);
  std::vector<Light> lights(n);
  for (unsigned i = 0; i < n; i++) {
    lights[i].board = i; lights[i].on = true;
    h.lights_.push_back(&lights[i]);
    auto &d = const_cast<ReceiverData &>(h.poller_.data(i));
    ultra_put32(d.uid, i + 1); d.have_identity = true;
    h.keeper_.on_updates(h.poller_, i, 1u << TP_IDENTITY, 0);
  }
  now_ms = 0;
  h.ledger_line_();
  const auto start = std::chrono::steady_clock::now();
  for (unsigned i = 1; i <= 1000000; i++) { now_ms = i; h.ledger_line_(); }
  ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / 1000000;
  ticks = h.keeper_.ticks;
  return canonical(ledger);
}

static std::vector<std::vector<uint8_t>> multi_line() {
  // One ledger, two unsynchronized line loops. A read/save must flush each to its own last observation.
  Ledger ledger;
  ledger.init(8, 8);
  UltraHub a, b;
  int uart;
  Light la{0}, lb{0};
  std::vector<std::vector<uint8_t>> snapshots;
  for (auto *h : {&a, &b}) {
    h->rw_uart_ = &uart; h->ready = true;
    h->poller_.init(1, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    h->keeper_.attach(&ledger, h == &a ? 1u << 16 : 2u << 16, 1);
    auto &d = const_cast<ReceiverData &>(h->poller_.data(0));
    d.uid[0] = h == &a ? 1 : 2; d.have_identity = true;
    h->keeper_.on_updates(h->poller_, 0, 1u << TP_IDENTITY, 0);
  }
  a.lights_ = {&la}; b.lights_ = {&lb};
  for (uint32_t t = 0; t < 100000; t++) {
    now_ms = t;
    if (t % 7 == 0) { la.on = t % 101 < 43; a.ledger_line_(); }
    if (t % 11 == 0) { lb.on = t % 97 < 51; b.ledger_line_(); }
    if (t % 131 == 0) snapshots.push_back(canonical(ledger));
    if (t % 127 == 0) a.keeper_.on_updates(a.poller_, 0, 1u << TP_IDENTITY, t);
  }
  // Keeper copies and re-attachment materialize pending time; copied ledger snapshots own no live hooks.
  Keeper copy = a.keeper_;
  snapshots.push_back(canonical(ledger));
  Ledger snapshot = ledger;
  a.keeper_.attach(&ledger, 1u << 16, 1, now_ms);
  snapshots.push_back(canonical(snapshot));
  snapshots.push_back(canonical(ledger));
  return snapshots;
}
