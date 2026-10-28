// The cable-drop advisor, the port's 36 V rules and the supply dips of a board without the SUPPLY page:
// hub-only (components/ultra_line/ultra_telemetry.h), the pages decoded from their wire bytes. Included by
// ultra_telemetry_test.cpp.

namespace cable_test {

/// The poller's per-receiver state from a test (its data, its answering), without the wire
class Poller : public TelemetryPoller {
 public:
  void start(uint16_t n) {
    this->init(n, ULTRA_RW_RATE_1M, TelemetryIntervals{});
    for (auto &r : this->rx_) {
      r.answering = true;
      r.answering_ms = 100;
    }
  }
  ReceiverData &of(uint16_t k) { return this->rx_[k].d; }
  void answering_of(uint16_t k, bool on) { this->rx_[k].answering = on; }
  void restart(uint16_t k, uint32_t now_ms) { this->restarted_(this->rx_[k], now_ms, false); }
  void status_page(uint16_t k, uint32_t now_ms, const uint8_t *page) { this->decode_(this->rx_[k], TP_STATUS, page, now_ms); }
};

static void status(ReceiverData &d, uint16_t bus_10mv, uint16_t ma, uint8_t why = 0, bool measured = true,
                   uint32_t uptime_s = 0) {
  uint8_t p[24]{};
  ultra_put24(p + ULTRA_ST_UPTIME, uptime_s);
  ultra_put16(p + ULTRA_ST_VBUS, bus_10mv);
  ultra_put16(p + ULTRA_ST_CURRENT, ma);
  p[ULTRA_ST_MEASURED] = measured ? (ULTRA_ST_M_VBUS | ULTRA_ST_M_CURRENT) : 0;
  p[ULTRA_ST_STATE] = static_cast<uint8_t>(why << ULTRA_ST_WHY_SHIFT);
  decode_receiver_page(d, ULTRA_PAGE_STATUS, p);
}
static void energy(ReceiverData &d, uint16_t vb_10mv, uint16_t z_mohm) {
  uint8_t p[24]{};
  ultra_put16(p + ULTRA_EN_VB, vb_10mv);
  ultra_put16(p + ULTRA_EN_SUPPLY_Z, z_mohm);
  decode_receiver_page(d, ULTRA_PAGE_ENERGY, p);
}
static void power(ReceiverData &d, uint16_t limit_ma) {
  uint8_t p[24]{};
  ultra_put16(p + ULTRA_PW_LIMIT_MA, limit_ma);
  decode_receiver_page(d, ULTRA_PAGE_POWER, p);
}
static void calibration(ReceiverData &d, uint8_t model, uint16_t connected_ma) {
  uint8_t p[24]{};
  p[ULTRA_CA_MODEL] = model;
  ultra_put16(p + CA_CONNECTED, connected_ma);
  decode_receiver_page(d, ULTRA_PAGE_CALIBRATION, p);
}
/// an analog board that has measured (bus, input current, LED supply, Z), with a limit and/or a current model
static ReceiverData analog(uint16_t bus_10mv, uint16_t ma, uint16_t vb_10mv, uint16_t z_mohm, uint16_t limit_ma,
                           uint8_t model = 0, uint16_t connected_ma = 0) {
  ReceiverData d;
  d.have_identity = true;
  d.features = ULTRA_FEAT_ENERGY | ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_CALIBRATION;
  status(d, bus_10mv, ma);
  energy(d, vb_10mv, z_mohm);
  if (limit_ma != 0)
    power(d, limit_ma);
  if (model != 0)
    calibration(d, model, connected_ma);
  return d;
}
/// the STATUS that first flags a dip (why = SUPPLY) at the dip's current: the load step from the reading before it
static void flagged(ReceiverData &d, uint16_t lowest_mv, uint16_t ma, uint32_t uptime_s = 0) {
  status(d, static_cast<uint16_t>(lowest_mv / 10), ma, ULTRA_ST_WHY_SUPPLY, true, uptime_s);
}
/// one logged dip on a board with the SUPPLY page: reference_mv, its lowest reading, its current
static void dip(ReceiverData &d, uint16_t lowest_mv, uint8_t pct, uint16_t ma, uint32_t age_s = 600) {
  uint8_t p[24]{};
  ultra_put16(p + ULTRA_SU_DIPS, 1);
  ultra_put16(p + ULTRA_SU_REF_10MV, 4630);
  ultra_put16(p + ULTRA_SU_DEEP_10MV, lowest_mv / 10);
  p[ULTRA_SU_DEEP_PCT] = pct;
  ultra_put24(p + ULTRA_SU_DEEP_AGE, age_s);
  p[ULTRA_SU_LOGGED] = 1;
  ultra_put24(p + ULTRA_SU_EV_AGE, age_s);
  ultra_put16(p + ULTRA_SU_EV_10MV, lowest_mv / 10);
  p[ULTRA_SU_EV_PCT] = pct;
  ultra_put16(p + ULTRA_SU_EV_MS, 35);
  ultra_put16(p + ULTRA_SU_EV_MA, ma);
  decode_receiver_page(d, ULTRA_PAGE_SUPPLY, p);
}

}  // namespace cable_test

/// A board without the SUPPLY page has only STATUS's reason "a supply dip in the last 30 s". A normal
/// cable drop (up to 20 %) is a note, not a Problem; only a bus reading below dip_warning % of the
/// board's highest bus reading in that dip is.
static void test_lean_supply_dips() {
  using namespace cable_test;
  ReceiverData d;
  d.have_identity = true;
  d.features = ULTRA_FEAT_ENERGY;   // (no ULTRA_FEAT_SUPPLY_DIPS: the lean Digital-M)
  status(d, 5500, 100);               // its highest bus reading: 55.0 V
  CHECK(receiver_status(d, true) == RS_OK && telemetry_supply_note(d, DIP_WARN_PCT) == "No supply dip in the last 30 s.",
        "lean board, no dip: '%s'", receiver_status_name(receiver_status(d, true)));
  status(d, 4800, 2000, ULTRA_ST_WHY_SUPPLY);   // 48.0 V = 87 %: the cable under a strip going full
  CHECK(receiver_status(d, true) == RS_OK && !receiver_status_problem(receiver_status(d, true)) &&
            telemetry_supply_note(d, DIP_WARN_PCT).find("A supply dip in the last 30 s") == 0 &&
            telemetry_supply_note(d, DIP_WARN_PCT).find("80 %") != std::string::npos,
        "a 13 %% cable drop on a lean board is no problem: '%s' / '%s'", receiver_status_name(receiver_status(d, true)),
        telemetry_supply_note(d, DIP_WARN_PCT).c_str());
  CHECK(receiver_status(d, true, 0, 90) == RS_SUPPLY_DIPS, "... but is one against a 90 %% dip_warning");
  status(d, 4300, 2200, ULTRA_ST_WHY_SUPPLY);   // 43.0 V = 78 %: below 80 %
  CHECK(receiver_status(d, true) == RS_SUPPLY_DIPS && receiver_status_problem(RS_SUPPLY_DIPS) &&
            receiver_status_detail(d, RS_SUPPLY_DIPS, 0, -1).find("in the last 30 s") != std::string::npos,
        "a 22 %% bus drop is a problem: '%s'", receiver_status_name(receiver_status(d, true)));
  status(d, 5400, 100, ULTRA_ST_WHY_SUPPLY);   // back up inside the 30 s: the dip is still the same one
  CHECK(receiver_status(d, true) == RS_SUPPLY_DIPS, "the dip's lowest reading holds while STATUS says dip");
  status(d, 5400, 100, 0);   // STATUS's reason is over
  CHECK(receiver_status(d, true) == RS_OK && telemetry_supply_note(d, DIP_WARN_PCT) == "No supply dip in the last 30 s.",
        "the dip is over: '%s'", receiver_status_name(receiver_status(d, true)));
  status(d, 5300, 300, ULTRA_ST_WHY_SUPPLY);   // the next one: its own lowest (96 %), not the one before
  CHECK(receiver_status(d, true) == RS_OK, "the next dip starts again: '%s'", receiver_status_name(receiver_status(d, true)));
  // the bus not measured: no depth at all
  ReceiverData e;
  e.have_identity = true;
  e.features = ULTRA_FEAT_ENERGY;
  status(e, 5500, 100);
  status(e, 0xFFFF, 0, ULTRA_ST_WHY_SUPPLY, false);
  CHECK(receiver_status(e, true) == RS_OK, "no bus reading: only a note");
  // a board with the SUPPLY page: unchanged (its depth is from the page, the reason alone is nothing)
  ReceiverData s = analog(5500, 100, 2400, 0, 0);
  status(s, 4000, 100, ULTRA_ST_WHY_SUPPLY);
  CHECK(receiver_status(s, true) == RS_OK, "a board with the SUPPLY page and no logged dip: STATUS's reason alone is no problem");
  // a restart (a power cycle: perhaps another supply) forgets the highest reading
  Poller p;
  p.start(1);
  p.of(0).have_identity = true;
  status(p.of(0), 5500, 100);
  uint8_t pg[24]{};
  ultra_put24(pg + ULTRA_ST_UPTIME, 10);
  ultra_put16(pg + ULTRA_ST_VBUS, 5500);
  pg[ULTRA_ST_MEASURED] = ULTRA_ST_M_VBUS | ULTRA_ST_M_CURRENT;
  p.status_page(0, 1000, pg);
  CHECK(p.data(0).vbus_hi_10mv == 5500, "the poller keeps the highest bus reading (%u)", p.data(0).vbus_hi_10mv);
  p.restart(0, 2000);
  CHECK(p.data(0).vbus_hi_10mv == 0 && p.data(0).vbus_dip_low_10mv == 0xFFFF, "a restart forgets it");
}

/// The cable-drop advisor: Z x (full-load current - present current), the line it writes, the dips it explains.
static void test_cable_drop() {
  using namespace cable_test;
  // 53.0 V at 1.0 A, supply impedance 300 mOhm, a calibrated model of 2.0 A inside the 2.25 A limit: unloaded 53.3 V, 52.7 V at full load
  ReceiverData a = analog(5300, 1000, 2400, 300, 2250, 2, 2000);
  CableDrop c = cable_drop(a);
  CHECK(c.known && c.basis == 2 && std::fabs(c.full_a - 2.0f) < 1e-4f && std::fabs(c.open_v - 53.3f) < 1e-3f &&
            std::fabs(c.full_v - 52.7f) < 1e-3f && std::fabs(c.drop_pct - 0.6f / 53.3f * 100.0f) < 1e-3f,
        "cable drop: open %.3f V, full %.3f V, %.3f %%", c.open_v, c.full_v, c.drop_pct);
  CHECK(telemetry_cable_drop(a, DIP_WARN_PCT) == "At full load (2.0 A, calibrated) about 52 V at the receiver (1 % drop): fine.",
        "fine: '%s'", telemetry_cable_drop(a, DIP_WARN_PCT).c_str());
  // the limit caps the model; a learned model says so
  ReceiverData lim = analog(5300, 1000, 2400, 300, 1500, 1, 2000);
  CHECK(cable_drop(lim).basis == 1 && std::fabs(cable_drop(lim).full_a - 1.5f) < 1e-4f &&
            telemetry_cable_drop(lim, DIP_WARN_PCT).find("(1.5 A, learned)") != std::string::npos,
        "the board limit caps the connected load: '%s'", telemetry_cable_drop(lim, DIP_WARN_PCT).c_str());
  // no model: the board limit; a long cable beyond the line (24 %): a note, the cable
  ReceiverData far = analog(4600, 1500, 2400, 6000, 2200);
  c = cable_drop(far);
  CHECK(c.known && c.basis == 0 && std::fabs(c.open_v - 55.0f) < 1e-3f && std::fabs(c.full_v - 41.8f) < 1e-3f &&
            telemetry_cable_drop(far, DIP_WARN_PCT) ==
                "At full load (2.2 A, board limit) about 41 V at the receiver (24 % drop): more than the 20 % the line "
                "allows. Shorten the cable or raise the supply voltage.",
        "past the line: '%s'", telemetry_cable_drop(far, DIP_WARN_PCT).c_str());
  CHECK(telemetry_cable_drop(far, 70).find(": fine.") != std::string::npos, "against a 70 %% dip_warning it is fine");
  // a 36 V strip: the receiver needs 40 V at full load (Port v1.1), whatever the line says
  ReceiverData v36 = analog(4600, 1500, 3600, 9100, 2200);
  c = cable_drop(v36);
  CHECK(receiver_is_36v(v36) && !receiver_is_36v(far) && std::fabs(c.full_v - 39.63f) < 1e-3f &&
            telemetry_cable_drop(v36, DIP_WARN_PCT) ==
                "At full load (2.2 A, board limit) about 39 V at the receiver (34 % drop): a 36 V strip needs at least "
                "40 V there. Shorten the cable (up to 50 m on 48 V, 100 m on 55 V) or raise the supply voltage.",
        "36 V under 40 V: '%s'", telemetry_cable_drop(v36, DIP_WARN_PCT).c_str());
  // a present current above the full-load one (a rounded limit): never above the unloaded voltage
  ReceiverData over = analog(5300, 2400, 2400, 300, 2250);
  c = cable_drop(over);
  CHECK(c.known && std::fabs(c.full_v - 53.0f) < 1e-3f && c.full_v < c.open_v, "present > full load: no further drop (%.2f V)", c.full_v);
  // what is missing: Z, a full-load current, the readings
  ReceiverData noz = analog(5300, 1000, 2400, 0, 2250);
  CHECK(!cable_drop(noz).z_known && telemetry_cable_drop(noz, DIP_WARN_PCT) ==
                                       "Not measured yet: the supply impedance needs a change of load.",
        "Z not measured yet: '%s'", telemetry_cable_drop(noz, DIP_WARN_PCT).c_str());
  ReceiverData nolim = analog(5300, 1000, 2400, 300, 0);
  CHECK(!cable_drop(nolim).known && telemetry_cable_drop(nolim, DIP_WARN_PCT) == "Full load not known for this board.",
        "no limit, no model: '%s'", telemetry_cable_drop(nolim, DIP_WARN_PCT).c_str());
  ReceiverData nobus = analog(5300, 1000, 2400, 300, 2250);
  status(nobus, 0xFFFF, 0, 0, false);
  CHECK(!cable_drop(nobus).known && telemetry_cable_drop(nobus, DIP_WARN_PCT).empty(), "no bus reading: nothing to say");
  CHECK(telemetry_cable_drop(ReceiverData{}, DIP_WARN_PCT).empty(), "ENERGY not read: nothing to say");
  // a digital board: its connected load is not the strip's (its calibration is another model): the limit
  ReceiverData dig = analog(5300, 1000, 2400, 300, 2250, 2, 900);
  dig.hw_caps |= RX_CAP_PIXEL;
  CHECK(cable_drop(dig).basis == 0 && std::fabs(cable_drop(dig).full_a - 2.25f) < 1e-4f, "a digital board: the board limit");
}

/// A dip the cable alone explains (a load step, its depth no more than Z x that step) is labelled "cable drop (expected)".
static void test_dip_cable_drop() {
  using namespace cable_test;
  // reference 46.3 V; 0.1 A before the dip, lowest 40.3 V (87 %) at 2.0 A: depth 6.0 V ~ 3.0 ohm x 1.9 A (5.7 V)
  ReceiverData long_cable = analog(4630, 100, 2400, 3000, 2250);
  flagged(long_cable, 40300, 2000);
  dip(long_cable, 40300, 87, 2000);
  CHECK(dip_is_cable_drop(long_cable, 87, 40300, 2000) &&
            telemetry_supply_note(long_cable, DIP_WARN_PCT) ==
                "A supply dip in the last 24 h, to 40.3 V (87 %): cable drop (expected), fine down to 80 %." &&
            telemetry_last_dip(long_cable) == "to 40.30 V (87 %) for 35 ms at 2.00 A, cable drop (expected)",
        "a dip the cable explains: '%s' / '%s'", telemetry_supply_note(long_cable, DIP_WARN_PCT).c_str(),
        telemetry_last_dip(long_cable).c_str());
  // the same depth on a short cable (Z 0.3 ohm: 0.6 V expected) is not
  ReceiverData short_cable = analog(4630, 100, 2400, 300, 2250);
  flagged(short_cable, 40300, 2000);
  dip(short_cable, 40300, 87, 2000);
  CHECK(!dip_is_cable_drop(short_cable, 87, 40300, 2000) &&
            telemetry_supply_note(short_cable, DIP_WARN_PCT) ==
                "A supply dip in the last 24 h, to 40.3 V (87 %): fine down to 80 %." &&
            telemetry_last_dip(short_cable) == "to 40.30 V (87 %) for 35 ms at 2.00 A",
        "a dip the cable does not explain: '%s'", telemetry_supply_note(short_cable, DIP_WARN_PCT).c_str());
  // the tolerance: x 1.25 + 0.3 V of Z x the step (3.0 ohm x 1.9 A = 5.7 V: up to 7.4 V); 38.7 V lowest (84 %) is
  // 7.4 V deep, 37.0 V (82 %) 8.1 V
  CHECK(dip_is_cable_drop(long_cable, 84, 38700, 2000) && !dip_is_cable_drop(long_cable, 82, 37000, 2000),
        "the tolerance edge: 7.4 V deep yes, 8.1 V no");
  // below the line and explained by the cable: the cable is the cause, still a problem
  ReceiverData deep = analog(4630, 100, 2400, 3000, 2250);
  flagged(deep, 36500, 2000);
  dip(deep, 36500, 79, 2000);   // reference 46.2 V, depth 9.7 V: more than Z x I + tolerance: not explained
  CHECK(!dip_is_cable_drop(deep, 79, 36500, 2000) && receiver_status(deep, true) == RS_SUPPLY_DIPS &&
            receiver_status_detail(deep, RS_SUPPLY_DIPS, 1, -1).find("cable drop") == std::string::npos,
        "a deep dip the cable doesn't explain stays the supply's: '%s'",
        receiver_status_detail(deep, RS_SUPPLY_DIPS, 1, -1).c_str());
  ReceiverData deep_cable = analog(4630, 100, 2400, 4000, 2250);   // 4.0 ohm x 1.9 A = 7.6 V (up to 9.8 V)
  flagged(deep_cable, 36500, 2000);
  dip(deep_cable, 36500, 79, 2000);
  CHECK(dip_is_cable_drop(deep_cable, 79, 36500, 2000) && receiver_status(deep_cable, true) == RS_SUPPLY_DIPS &&
            receiver_status_detail(deep_cable, RS_SUPPLY_DIPS, 1, -1) ==
                "1 supply dip in the last 24 h, the last to 36.5 V (79 %) for 35 ms at 2.0 A. Cable drop (expected): "
                "the cable is too long or too thin." &&
            telemetry_supply_note(deep_cable, DIP_WARN_PCT) ==
                "A supply dip in the last 24 h, to 36.5 V (79 %): below 80 %, cable drop (expected): the cable is too "
                "long or too thin.",
        "below the line, explained: '%s'", receiver_status_detail(deep_cable, RS_SUPPLY_DIPS, 1, -1).c_str());
  // Z unknown or the dip's current missing: never labelled
  ReceiverData noz = analog(4630, 100, 2400, 0, 2250);
  flagged(noz, 40300, 2000);
  dip(noz, 40300, 87, 2000);
  CHECK(!dip_is_cable_drop(noz, 87, 40300, 2000) && !dip_is_cable_drop(long_cable, 87, 40300, 0) &&
            !dip_is_cable_drop(long_cable, 0, 40300, 2000) && !dip_is_cable_drop(long_cable, 100, 40300, 2000),
        "no Z / no current / no share: no label");

  // r287: a supply sag at a steady load is no cable drop, whatever Z: 2.0 A before and at the dip, 50 -> 40 V (80 %),
  // 10 V deep; Z 4.0 ohm x 2.0 A (the current at the dip) would have "explained" it
  ReceiverData sag = analog(5000, 2000, 2400, 4000, 2250);
  flagged(sag, 40000, 2000);
  dip(sag, 40000, 80, 2000);
  CHECK(!dip_is_cable_drop(sag, 80, 40000, 2000) && telemetry_last_dip(sag) == "to 40.00 V (80 %) for 35 ms at 2.00 A" &&
            telemetry_supply_note(sag, DIP_WARN_PCT).find("cable drop") == std::string::npos,
        "a supply sag at a steady 2 A: no cable label ('%s')", telemetry_supply_note(sag, DIP_WARN_PCT).c_str());
  // a step too small to matter (2.0 -> 2.05 A) is no step either
  ReceiverData small = analog(4630, 2000, 2400, 3000, 2250);
  flagged(small, 46000, 2050);
  dip(small, 46000, 99, 2050);
  CHECK(!dip_is_cable_drop(small, 99, 46000, 2050), "a 50 mA step: no cable label");
  // no STATUS reading before the dip (the first STATUS already flags it): the load before is not known
  ReceiverData first;
  first.have_identity = true;
  first.features = ULTRA_FEAT_ENERGY | ULTRA_FEAT_SUPPLY_DIPS;
  energy(first, 2400, 3000);
  flagged(first, 40300, 2000);
  dip(first, 40300, 87, 2000);
  CHECK(first.dip_pre_ma == 0xFFFF && !dip_is_cable_drop(first, 87, 40300, 2000),
        "no reading before the dip: no cable label");
  // a later dip of the same flagged episode (the flag at uptime 100 s; SUPPLY read at 130 s: the newest dip began at
  // 125 s, after the flag): its load step is not the one seen before the flag
  ReceiverData later = analog(4630, 100, 2400, 3000, 2250);
  status(later, 4630, 100, 0, true, 95);
  flagged(later, 40300, 2000, 100);
  status(later, 4100, 2000, ULTRA_ST_WHY_SUPPLY, true, 130);
  dip(later, 40300, 87, 2000, 5);
  CHECK(!dip_is_cable_drop(later, 87, 40300, 2000), "a dip after the flagging STATUS: no cable label");
  // ... while the dip that the flag reported (began at 99 s, read at 130 s: 31 s old) is
  dip(later, 40300, 87, 2000, 31);
  CHECK(dip_is_cable_drop(later, 87, 40300, 2000), "the dip the flag reported: the cable label");
}

/// The port's 36 V rules (Port v1.1): at most 4 receivers at 36 V, each keeping 40 V at full load.
static void test_port_load_rules() {
  using namespace cable_test;
  PortMonitor::Config cfg;
  auto run = [&](Poller &p, float supply_v, bool standby, PortLoadRules &r) {
    cfg.supply_v = supply_v;
    PortMonitor m;
    m.configure(cfg);
    r.update(p, m, standby);
  };
  Poller p;
  p.start(5);
  for (uint16_t k = 0; k < 5; k++)   // five 36 V receivers on a short cable (Z 0.3 ohm: ~50 V at full load)
    p.of(k) = analog(5200, 1000, 3600, 300, 2200);
  PortLoadRules rules;
  CHECK(rules.text() == "Not checked yet." && !rules.problem(), "before the first window");
  run(p, NAN, false, rules);
  CHECK(rules.problem() && rules.text() == "36 V: at most 4 receivers per port (5 are at 36 V).",
        "five 36 V receivers: '%s'", rules.text().c_str());
  p.answering_of(4, false);   // (a receiver that doesn't answer isn't counted)
  run(p, NAN, false, rules);
  CHECK(!rules.problem() && rules.text() == "OK.", "four 36 V receivers: '%s'", rules.text().c_str());
  p.answering_of(4, true);
  p.of(3) = analog(5200, 1000, 2400, 300, 2200);   // one of them a 24 V strip: four at 36 V
  run(p, NAN, false, rules);
  CHECK(!rules.problem() && rules.text() == "OK.", "four of five at 36 V: '%s'", rules.text().c_str());
  run(p, NAN, true, rules);
  CHECK(!rules.problem() && rules.text() == "Not checked (port in standby).", "standby: '%s'", rules.text().c_str());
  // rule 2: receiver 2 would get under 40 V at full load (bus 46.0 V at 1.5 A, Z 9 ohm, 2.2 A)
  Poller q;
  q.start(3);
  q.of(0) = analog(5200, 1000, 3600, 300, 2200);
  q.of(1) = analog(4600, 1500, 3600, 9100, 2200);
  q.of(2) = analog(4600, 1500, 2400, 9100, 2200);   // (a 24 V strip: not a 36 V rule)
  PortLoadRules r2;
  run(q, NAN, false, r2);
  CHECK(r2.problem() &&
            r2.text() == "36 V at full load: receiver 2 would get 39.6 V (needs 40 V). Keep the 36 V cable to 50 m on a "
                         "48 V supply, 100 m on 55 V.",
        "rule 2: '%s'", r2.text().c_str());
  run(q, 48.0f, false, r2);
  CHECK(r2.text().find("Keep the 36 V cable to 50 m on this 48 V supply.") != std::string::npos, "48 V: '%s'", r2.text().c_str());
  run(q, 55.0f, false, r2);
  CHECK(r2.text().find("Keep the 36 V cable to 100 m on this 55 V supply.") != std::string::npos, "55 V: '%s'", r2.text().c_str());
  // the release hysteresis: 40.2 V (above the minimum, within 0.5 V of it) keeps the flag, 40.6 V clears it
  q.of(1) = analog(4600, 1500, 3600, 7800, 2200);   // open 46 + 11.7 = 57.7; full = 57.7 - 7.8 x 2.2 = 40.54: clear
  run(q, NAN, false, r2);
  CHECK(!r2.problem() && r2.text() == "OK.", "40.5 V clears it: '%s'", r2.text().c_str());
  q.of(1) = analog(4600, 1500, 3600, 9100, 2200);   // 39.6 V: on
  run(q, NAN, false, r2);
  CHECK(r2.problem(), "39.6 V on again");
  q.of(1) = analog(4600, 1500, 3600, 8300, 2200);   // open 58.45, full = 58.45 - 18.26 = 40.19: held on
  run(q, NAN, false, r2);
  CHECK(r2.problem() && cable_drop(q.data(1)).full_v >= RX36_MIN_BUS_V && cable_drop(q.data(1)).full_v < RX36_MIN_BUS_V + 0.5f,
        "40.2 V is still inside the hysteresis: the flag holds (%.2f V)", cable_drop(q.data(1)).full_v);
  q.of(1) = analog(4600, 1500, 3600, 7800, 2200);
  run(q, NAN, false, r2);
  CHECK(!r2.problem(), "40.5 V clears it");
  // the sender's supply: configured under 40 V no 36 V strip can work
  run(q, 36.0f, false, r2);
  CHECK(r2.problem() &&
            r2.text() == "36 V strips need at least 40 V at the receiver; the sender's supply is 36 V.",
        "supply 36 V: '%s'", r2.text().c_str());
  // both rules at once: within a text sensor's 255 characters
  Poller both;
  both.start(6);
  for (uint16_t k = 0; k < 6; k++)
    both.of(k) = analog(4600, 1500, 3600, 9100, 2200);
  PortLoadRules rb;
  run(both, NAN, false, rb);
  CHECK(rb.problem() && rb.text().size() <= 255 && rb.text().find("36 V: at most 4 receivers per port (6 are at 36 V).") == 0 &&
            rb.text().find("receiver 1 would get 39.6 V") != std::string::npos,
        "both rules (%zu characters): '%s'", rb.text().size(), rb.text().c_str());
  // no LED supply read yet: not checked
  Poller early;
  early.start(1);
  early.of(0).have_status = true;
  PortLoadRules re;
  run(early, NAN, false, re);
  CHECK(!re.problem() && re.text() == "Not checked yet.", "no ENERGY read: '%s'", re.text().c_str());
}

static void test_cable_drop_all() {
  test_lean_supply_dips();
  test_cable_drop();
  test_dip_cable_drop();
  test_port_load_rules();
}
