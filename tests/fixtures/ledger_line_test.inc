static uint32_t now_ms;
static uint32_t millis() { return now_ms; }
struct Light {
  uint16_t board;
  bool on = false, fading = false;
  uint32_t off_ms = 0;
  uint16_t get_board() const { return board; }
  bool is_lit(uint32_t now) {
    if (fading && static_cast<int32_t>(off_ms - now) <= 0) fading = false;
    return on || fading;
  }
};
struct Keeper : CoreKeeper {
  uint64_t ticks = 0;
  void lit_tick(uint16_t i, bool on, uint32_t now) {
    ticks++;
    CoreKeeper::lit_tick(i, on, now);
  }
};
struct Port {
  bool on = false;
  uint32_t ms = 0;
  const Port &core() const { return *this; }
  bool line_on() const { return on; }
  uint32_t line_ms() const { return ms; }
};
struct UltraHub {
  int *rw_uart_ = nullptr;
  Keeper keeper_;
  TelemetryPoller poller_;
  std::vector<Light *> lights_;
  bool ready = false, line_on_ = false;
  Port *port_ = nullptr;
  std::vector<uint8_t> lit_rx_;
  bool boards_powered_() const { return ready; }
  void ledger_line_();
};
