// Firmware descriptors and generated rational bounds, read from ULTRA_FIRMWARE by the contract runner.
#include "board_bound_descriptors.inc"

static void sc_ledger_bound() {
  struct Board {
    const uint8_t *bytes;
    size_t size;
    uint32_t num, den;
  };
  for (const Board b : {
           Board{bound_analog::board_descriptor, sizeof(bound_analog::board_descriptor),
                 bound_analog::num, bound_analog::den},
           Board{bound_digital203::board_descriptor, sizeof(bound_digital203::board_descriptor),
                 bound_digital203::num, bound_digital203::den},
           Board{bound_digital407::board_descriptor, sizeof(bound_digital407::board_descriptor),
                 bound_digital407::num, bound_digital407::den}}) {
    ReceiverData d;
    d.have_descriptor = true;
    d.descriptor_bytes.assign(b.bytes, b.bytes + b.size);
    ultra_descriptor_t desc{};
    ultra_desc_domain_t input{};
    const bool valid = !ultra_descriptor_decode(&desc, b.bytes, b.size) &&
                       !ultra_descriptor_domain(&desc, 0, &input);
    CHECK(valid && b.num && b.den, "bound: firmware descriptor and input facts decode");
    if (!valid || !b.den) continue;
    for (uint32_t dt = 0; dt <= 7200; dt++) {
      const uint32_t expected = (b.num * (dt + 10) + b.den - 1) / b.den;
      const uint32_t allowance = ultra_lifetime_energy_allowance(input.rating_ma, input.vmax_10mv, dt, UINT32_MAX);
      CHECK(allowance == expected, "bound: exact firmware step at %u", dt);
      // The receiver accepts exactly its allowance; the next Wh is refused, while powered time still merges.
      uint8_t args[LifetimeLedger::LS_ARGS]{};
      ultra_put24(args + ULTRA_LS_LIFETIME + ULTRA_LT_POWERED, dt);
      ultra_put32(args + ULTRA_LS_TOTAL_WH, allowance);
      energy_t rx{};
      uint32_t res = energy_sync(&rx, args, 0, b.num, b.den);
      CHECK((res >> 8) == ULTRA_LS_R_APPLIED && energy_total_wh(&rx) == expected,
            "bound: real merge accepts %u Wh at %u", allowance, dt);
      rx = energy_t{};
      ultra_put32(args + ULTRA_LS_TOTAL_WH, allowance + 1);
      res = energy_sync(&rx, args, 0, b.num, b.den);
      CHECK((res >> 8) == ULTRA_LS_R_CAPPED && energy_total_wh(&rx) == 0 && rx.lt.on_s == dt * 360,
            "bound: real merge refuses the next Wh at %u", dt);
      LifeCounters own;
      own.powered = 100;
      own.mwh = 1000123;
      LedgerEntry e;
      e.c = own;
      e.c.powered += dt;
      e.c.mwh = UINT64_MAX;
      const auto target = LifetimeLedger::sync_target(e, own, d);
      const uint32_t dm = dt ? dt - 1 : 0;
      const uint32_t safe = (b.num * (dm + 10) + b.den - 1) / b.den;
      CHECK(target.mwh == own.mwh + uint64_t(safe) * 1000,
            "bound: hub uses the exact step with read-to-POLL margin at %u", dt);
    }
    // The wire's last Wh is available, and the fractional part never changes or overflows it.
    LifeCounters own;
    own.mwh = uint64_t(UINT32_MAX - 1) * 1000 + 999;
    LedgerEntry e;
    e.c = own;
    e.c.mwh = UINT64_MAX;
    CHECK(LifetimeLedger::sync_target(e, own, d).mwh == uint64_t(UINT32_MAX) * 1000 + 999,
          "bound: wire ceiling leaves exactly one Wh");
    own.mwh += 1000;
    CHECK(!LifetimeLedger::ahead(e, own, d), "bound: full wire counter sends no energy-only merge");

    own = LifeCounters{};
    own.mwh = 1000123;
    e.c = own;
    e.c.mwh += 10000000;
    auto no_energy = [&](const ReceiverData &missing) {
      CHECK(!LifetimeLedger::ahead(e, own, missing), "bound: unknown facts send no energy-only merge");
      e.c.powered = 1;
      const auto target = LifetimeLedger::sync_target(e, own, missing);
      CHECK(target.mwh == own.mwh && target.powered == 1, "bound: unknown facts still merge time");
      e.c.powered = 0;
    };
    auto missing = d;
    missing.have_descriptor = false;
    no_energy(missing);
    missing.have_descriptor = true;
    missing.descriptor_bytes.clear();
    no_energy(missing);
    missing = d;
    missing.descriptor_bytes[4] ^= 1;
    no_energy(missing);
    // An otherwise valid descriptor can omit its power domains entirely.
    missing = d;
    auto &bytes = missing.descriptor_bytes;
    bytes.resize(11);
    bytes[8] = 0;
    for (size_t p = 11; p < b.size; p += 2 + b.bytes[p + 1]) {
      const uint8_t type = b.bytes[p];
      if (type == ULTRA_DESC_DOMAIN || type == ULTRA_DESC_PWM || type == ULTRA_DESC_PIXEL ||
          type == ULTRA_DESC_SENSE || type == ULTRA_DESC_RESOURCES) continue;
      bytes.insert(bytes.end(), b.bytes + p, b.bytes + p + 2 + b.bytes[p + 1]);
      bytes[8]++;
    }
    ultra_put16(bytes.data() + 2, bytes.size());
    ultra_put16(bytes.data() + 4, ultra_crc16(bytes.data() + 6, bytes.size() - 6));
    ultra_descriptor_t no_domains{};
    CHECK(!ultra_descriptor_decode(&no_domains, bytes.data(), bytes.size()), "bound: domain-less fixture is valid");
    no_energy(missing);
    for (unsigned field : {2u, 6u}) {
      missing = d;
      ultra_tlv_t domain{};
      CHECK(!ultra_descriptor_find(&desc, ULTRA_DESC_DOMAIN, 0, &domain), "bound: input TLV found");
      const size_t p = domain.value - desc.bytes;
      ultra_put16(missing.descriptor_bytes.data() + p + field, 0xFFFF);
      if (field == 2) ultra_put16(missing.descriptor_bytes.data() + p + 4, 0xFFFF);
      ultra_put16(missing.descriptor_bytes.data() + 4,
                  ultra_crc16(missing.descriptor_bytes.data() + 6, missing.descriptor_bytes.size() - 6));
      no_energy(missing);
    }
    // Different receiver input facts must change the hub's cap, including fractional-W ratings and large products.
    ultra_tlv_t domain{};
    CHECK(!ultra_descriptor_find(&desc, ULTRA_DESC_DOMAIN, 0, &domain), "bound: input TLV found");
    const size_t p = domain.value - desc.bytes;
    e.c.mwh = UINT64_MAX;
    for (auto facts : {std::pair<uint16_t, uint16_t>{1, 1}, {2251, 5501}, {65534, 65534}, {0, 0}}) {
      auto other = d;
      ultra_put16(other.descriptor_bytes.data() + p + 2, facts.second);
      ultra_put16(other.descriptor_bytes.data() + p + 4, facts.second);
      ultra_put16(other.descriptor_bytes.data() + p + 6, facts.first);
      ultra_put16(other.descriptor_bytes.data() + 4,
                  ultra_crc16(other.descriptor_bytes.data() + 6, other.descriptor_bytes.size() - 6));
      for (uint32_t dt = 0; dt <= 7200; dt++) {
        e.c.powered = dt;
        const uint32_t dm = dt ? dt - 1 : 0;
        const uint64_t expected = (uint64_t(facts.first) * facts.second * (dm + 10) + 999999) / 1000000;
        const uint64_t room = UINT32_MAX - own.mwh / 1000;
        const uint64_t safe = std::min(expected, room);
        CHECK(LifetimeLedger::sync_target(e, own, other).mwh == own.mwh + safe * 1000,
              "bound: each receiver's own facts at %u", dt);
      }
    }
    std::printf("ledger bound: %zu-byte firmware descriptor, all 7,201 steps match real merge; unknown facts and wire ceiling\n",
                b.size);
  }
}
