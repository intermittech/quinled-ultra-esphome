// Shared by the telemetry, digital and contract runners; hub-only, no receiver firmware changes.
#include "../components/ultra_line/ultra_commission.h"

class OutputVoltageLowPoller : public TelemetryPoller {
 public:
  void start(uint32_t now = 100) {
    TelemetryIntervals iv;
    iv.energy_ms = 2000;
    this->init(1, ULTRA_RW_RATE_1M, iv);
    auto &r = this->rx_[0];
    r.answering = true;
    r.answering_ms = now;
    r.d.have_identity = r.d.have_status = true;
    r.d.hw_id = 3;
    r.d.hw_caps = RX_CAP_RETURN | RX_CAP_PIXEL_FX | RX_CAP_PIXEL;
    r.d.features = ULTRA_FEAT_ENERGY;
    r.status_ms = now;
  }
  void status(uint32_t now, uint8_t why = 0) {
    uint8_t data[24]{};
    data[ULTRA_ST_STATE] = static_cast<uint8_t>(why << ULTRA_ST_WHY_SHIFT);
    ultra_put24(data + ULTRA_ST_UPTIME, this->rx_[0].d.uptime_s + (now - this->rx_[0].status_ms) / 1000);
    this->decode_(this->rx_[0], TP_STATUS, data, now);
  }
  void energy(uint32_t now, uint16_t vb) {
    this->status(now);
    uint8_t data[24]{};
    ultra_put16(data + ULTRA_EN_VB, vb);
    this->decode_(this->rx_[0], TP_ENERGY, data, now);
  }
  ReceiverData &mutable_data() { return this->rx_[0].d; }
  void restart(uint32_t now) { this->restarted_(this->rx_[0], now, false); }
  void unsupported() { this->rx_[0].unsupported |= 1u << TP_ENERGY; }
  void mark_status_fresh() { this->rx_[0].fresh |= 1u << TP_STATUS; }
  void mark_all_fresh() { this->rx_[0].fresh = (1u << TP_COUNT) - 1; }
};

class OutputVoltageLowCommission : public Commissioning {
 public:
  std::string snapshot(const TelemetryPoller &p) {
    this->answers_(p);
    return this->compose_(true, false, p.receivers());
  }
  void load(OutputVoltageLowPoller &p, uint32_t now) {
    this->t_ = now - SETTLE_MS;
    this->marked_ = true;
    this->last_p_ = &p;
    this->events0_.assign(p.receivers(), 0);
    p.mark_status_fresh();
    this->load_(now, p, PortMonitor{});
  }
  void final_read(OutputVoltageLowPoller &p, uint32_t now) {
    this->phase_ = READ;
    this->kind_ = FINALIZE;
    this->t_ = now;
    this->events0_.assign(p.receivers(), 0);
    p.mark_all_fresh();
    CommissionPort port;
    port.ready = true;
    this->poll(now, 0, port, p, PortMonitor{}, false, false);
  }
};

static void test_output_voltage_low() {
  OutputVoltageLowPoller p;
  p.start();
  auto &f = p.mutable_data().output_voltage_low;
  p.energy(1000, 0); p.energy(2500, 0); p.energy(4000, 0);
  CHECK(!f.known && !f.low, "fuse: first five seconds excluded");
  p.energy(5100, 240); p.energy(6500, 240); p.energy(8099, 240);
  CHECK(!f.problem(), "fuse: three low reports alone do not satisfy three seconds");
  for (uint32_t t = 8100; t < 8200; t++) p.service_output_voltage_low(t, false);
  CHECK(!f.problem(), "fuse: callbacks/timers do not count as fresh ENERGY");
  p.energy(8200, 240);
  CHECK(f.problem(), "fuse: back-fed 2.4 V is a fault after three reports and three seconds");
  CHECK(receiver_status_problem(receiver_status(p.data(0), true)) &&
        receiver_status(p.data(0), true) == RS_OUTPUT_VOLTAGE_LOW, "fuse: shared receiver/port health summary");
  CHECK(receiver_status_detail(p.data(0), RS_OUTPUT_VOLTAGE_LOW, 0, -1) ==
        "Output voltage low: fuse blown, output shorted / overloaded, or the supply module is current-limiting", "fuse: plain status detail");
  OutputVoltageLowCommission cm;
  cm.load(p, 8200);
  CHECK(cm.report().find("Output voltage low: fuse blown, output shorted / overloaded, or the supply module is current-limiting") != std::string::npos,
        "fuse: commissioning load test fails with the plain explanation");
  OutputVoltageLowCommission final;
  final.final_read(p, 8200);
  CHECK(!final.record().passed && final.report().find("Output voltage low: fuse blown, output shorted / overloaded, or the supply module is current-limiting") !=
        std::string::npos, "fuse: commissioning final read fails with the plain explanation");
  p.energy(8300, 400); p.energy(8400, 240); p.energy(8500, 400); p.energy(8600, 500);
  CHECK(f.problem(), "fuse: fewer than three consecutive good reports cannot clear");
  p.energy(8700, 2400);
  CHECK(f.known && !f.fault, "fuse: three good reports clear; exactly 4.0 V is good");
  OutputVoltageLowCommission healthy;
  CHECK(healthy.snapshot(p).find("Output voltage: OK") != std::string::npos, "fuse: commissioning good line");
  p.energy(9000, 0); p.energy(12000, 0);
  CHECK(!f.problem(), "fuse: elapsed time with two reports is insufficient");
  p.energy(12100, 0);
  CHECK(f.problem(), "fuse: zero volts also detects a blown/shorted output");
  p.energy(12200, 0xFFFF);
  CHECK(!f.known && !f.problem() && !f.sampled, "fuse: unknown is unavailable, never ON");
  CHECK(healthy.snapshot(p).find("Output voltage: unavailable") != std::string::npos, "fuse: commissioning unknown line");
  p.energy(13000, 240); p.energy(14500, 240); p.energy(16000, 240);
  CHECK(f.problem(), "fuse: rearmed from fresh samples after unknown");
  p.status(18000); p.status(20000); p.status(21001);
  p.service_output_voltage_low(21001, false);
  CHECK(!f.known && !f.low, "fuse: ENERGY expires while STATUS keeps answering");
  p.energy(22000, 240); p.energy(24000, 240); p.energy(26000, 240);
  CHECK(f.problem(), "fuse: valid fault before intentional hold");
  p.service_output_voltage_low(26001, true);
  CHECK(!f.known && !f.problem(), "fuse: standby/port off/OFF mode/calibration/update hold is unavailable");
  p.energy(28000, 240); p.energy(30000, 240); p.energy(32000, 240);
  CHECK(!f.known && !f.low, "fuse: suppressed reports never build a debounce");
  p.service_output_voltage_low(32001, false);
  p.energy(34000, 240); p.energy(36000, 240);
  CHECK(!f.problem(), "fuse: resume needs a new consecutive sequence");
  p.energy(38000, 240);
  CHECK(f.problem(), "fuse: resumed fresh sequence detects");
  p.status(38001, ULTRA_ST_WHY_CALIBRATING);
  CHECK(!f.known, "fuse: receiver-side calibration also suppresses");
  p.energy(40000, 240); p.energy(42000, 240); p.energy(44000, 240);
  p.unpowered();
  CHECK(!f.known && !f.problem(), "fuse: lost receiver/power invalidates immediately");

  for (int gate = 0; gate < 5; gate++) {
    p.start();
    if (gate == 0) p.mutable_data().hw_caps &= ~RX_CAP_PIXEL;
    if (gate == 1) p.mutable_data().features = 0;
    if (gate == 2) p.mutable_data().hw_caps = 0;
    if (gate == 3) p.mutable_data().have_identity = false;
    if (gate == 4) p.unsupported();
    p.energy(6000, 240); p.energy(8000, 240); p.energy(10000, 240);
    CHECK(!p.data(0).output_voltage_low.known, "fuse: identity/hardware/capability gate %d", gate);
  }
  p.start();
  p.energy(6000, 240); p.energy(8000, 240); p.energy(10000, 240);
  p.restart(10001);
  p.energy(11000, 240); p.energy(13000, 240); p.energy(15000, 240);
  CHECK(!p.data(0).output_voltage_low.known, "fuse: restart starts a new five-second grace");
  p.start(0xFFFFE000u);
  p.energy(0xFFFFF388u, 240); p.energy(0xFFFFFB58u, 240); p.energy(0x00000328u, 240);
  CHECK(p.data(0).output_voltage_low.problem(), "fuse: debounce across millis wrap");
  p.service_output_voltage_low(10000, false);
  CHECK(!p.data(0).output_voltage_low.known, "fuse: stale STATUS/ENERGY across millis wrap");
  std::printf("  output voltage low: threshold, freshness, debounce, holds, capabilities, restart and wrap\n");
}
