// Independent wire-to-physical oracle: all 65,536 signed 2 mA encodings.
static void test_signed_current_units() {
  unsigned analog_bad = 0, health_bad = 0, pixel_bad = 0;
  for (uint32_t raw = 0; raw <= 65535; ++raw) {
    const int32_t expected = (raw < 32768 ? static_cast<int32_t>(raw) : static_cast<int32_t>(raw) - 65536) * 2;
    uint8_t payload[24]{};
    ReceiverData receiver;
    ultra_put16(payload + CA_IDLE, raw);
    decode_receiver_page(receiver, ULTRA_PAGE_CALIBRATION, payload);
    analog_bad += receiver.cal_idle_ma != expected;
    ultra_put16(payload + ULTRA_HE_DARK_2MA, raw);
    decode_receiver_page(receiver, ULTRA_PAGE_HEALTH, payload);
    health_bad += receiver.dark_ma != expected;
    PixelCalibration pixel;
    ultra_put16(pixel.active_currents, raw);
    pixel_bad += pixel.idle_ma() != expected;
  }
  CHECK(analog_bad == 0, "analog idle: all signed 2 mA encodings (%u wrong)", analog_bad);
  CHECK(health_bad == 0, "health dark: full signed range without narrowing (%u wrong)", health_bad);
  CHECK(pixel_bad == 0, "pixel aggregate idle: all signed 2 mA encodings (%u wrong)", pixel_bad);
  std::printf("  signed current units: 196608 exact wire/physical comparisons\n");
}
