// Digital calibration wire/reader tests, included by the telemetry runner.
#include "board_pixel_fixture.h"

static void test_pixel_calibration() {
  const ultra_desc_pixel_t v203 = fixture_pixel("ur_digital_m_v203");
  const uint16_t *cap = v203.max_pixels;
  uint8_t m[24]{}, a[24]{}, q[24]{};
  m[0] = 2; m[1] = ULTRA_UCF_VALID | ULTRA_UCF_AGGREGATE | ULTRA_UCF_SHORT | ULTRA_UCF_PENDING;
  m[2] = ULTRA_UCS_DONE; ultra_put16(m+4, 4); ultra_put16(m+6, 120); ultra_put16(m+8, 93);
  ultra_put16(m+10, 93); m[12] = 1; m[13] = 3; ultra_put16(m+14, 136);
  ultra_put16(m+16, 55000); ultra_put16(m+18, 12000); ultra_put16(m+20, 9); ultra_put16(m+22, 3456);
  a[0] = 3; a[1] = 2; ultra_put16(a+2,4); ultra_put16(a+7,static_cast<uint16_t>(-6)); // -12 mA in 2 mA units
  for (int i=0; i<3; i++) ultra_put16(a+9+2*i, 930+i*93);
  q[0]=2; q[1]=1; ultra_put16(q+2,4); ultra_put16(q+4,9); ultra_put16(q+6,10);
  ultra_put16(q+8,50); ultra_put16(q+10,60); ultra_put16(q+12,20);
  auto load = [&](PixelCalibration &c) {
    c.read(1,m,false,cap); c.read(0,a,false,cap); c.read(2,q,false,cap); return c.read(1,m,false,cap);
  };
  PixelCalibration c;
  CHECK(load(c) && c.usable(), "coherent format 2 including pending RAM model");
  CHECK(c.count_text().find("data break at pixel 94") != std::string::npos, "short notice identifies first missing address");
  CHECK(c.result_text().find("pending save") != std::string::npos, "pending is not durable");
  CHECK(c.json(true).find("\"cp_ma\":[10.000,11.000,12.000,0.000,0.000]") != std::string::npos,
        "whole coefficients divided by used count in logical order");
  CHECK(c.json(true).find("\"idle_ma\":-12") != std::string::npos && c.json(true).find("\"ip_ma\":-0.129") != std::string::npos,
        "signed aggregate idle and apparent per-pixel intercept");
  CHECK(c.json(false).find("\"src\":\"default\",\"n\":0") != std::string::npos, "unavailable cannot masquerade as calibrated");
  for (int offset : {1,4,6,8,10,12,13,14,16,18,20}) {
    PixelCalibration bad; bad.read(1,m,false,cap); bad.read(0,a,false,cap); bad.read(2,q,false,cap);
    uint8_t changed[24]; std::memcpy(changed,m,24); changed[offset] ^= 1;
    CHECK(!bad.read(1,changed,false,cap) && !bad.usable(), "torn generation/fingerprint/flags offset %d rejected",offset);
  }
  for (int part : {0,2}) {
    uint8_t *p = part==0 ? a:q;
    p[part==0?3:5] ^= 1;
    CHECK(!load(c) && !c.usable(), "part %d generation must match",part);
    p[part==0?3:5] ^= 1;
  }
  load(c); m[1] |= ULTRA_UCF_STALE; load(c); CHECK(!c.usable(), "valid plus stale is unusable");
  m[1] &= ~ULTRA_UCF_STALE;
  m[1] |= ULTRA_UCF_CLEAR_PENDING; load(c); CHECK(!c.usable(), "clear pending is not a durable reset");
  m[1] = ULTRA_UCF_VALID | ULTRA_UCF_AGGREGATE | ULTRA_UCF_AT_LEAST;
  ultra_put16(m+6,600); ultra_put16(m+8,600); ultra_put16(m+10,600); load(c);
  CHECK(c.count_text().find("at least 600") != std::string::npos, "cap is lower bound");
  // HA text bound at the widest valid wire values, signed idle and per-pixel currents.
  ultra_put16(m+6,1); ultra_put16(m+8,1); ultra_put16(m+10,1); m[12]=255; m[13]=5;
  ultra_put16(m+14,65535); ultra_put16(m+16,65535); ultra_put16(m+18,65535);
  ultra_put16(m+20,65535); ultra_put16(q+4,65535); ultra_put16(a+7,32768);
  for(int i=0;i<5;i++) ultra_put16(a+9+2*i,65535);
  load(c);
  CHECK(c.json(true).size() <= 255 && c.json(true).back()=='}', "maximum width JSON complete: %zu",c.json(true).size());
  c.invalidate(); CHECK(!c.usable() && !c.have, "restart/unavailable invalidates staging and active model");
  // No fresh terminal result means no early release of quiet ownership.
  const uint32_t wait203 = fixture_cal_wait_ms("ur_digital_m_v203");
  load(c); PixelCalibrationJob job; job.start(0,0,c,wait203); job.sent=true;
  c.read(1,m,true,cap);
  CHECK(!job.tick(1000,c), "old DONE is not this run's success");
  m[2]=ULTRA_UCS_CHANNELS; c.read(1,m,true,cap); CHECK(!job.tick(2000,c), "progress retains ownership");
  job.cancel(); CHECK(job.action==ULTRA_IDENT_RELEASE && job.active(), "cancel waits for receiver restoration");
  m[2]=ULTRA_UCS_FAILED; m[3]=ULTRA_UCE_CANCELLED; c.read(1,m,true,cap);
  CHECK(job.tick(3000,c) && !job.active() && job.error==ULTRA_UCE_CANCELLED, "terminal cancel releases");
  // the restore deadline is the receiver's own (its descriptor), per board
  for (const char *name : {"ur_digital_m_v203", "ur_digital_m_v407"}) {
    const ultra_desc_pixel_t px = fixture_pixel(name);
    const uint32_t wait = fixture_cal_wait_ms(name);
    job.start(0,0,c,wait); job.sent=true;
    CHECK(px.cal_ms && wait == 50u + (px.cal_ms*3u+1u)/2u, "%s: deadline %u ms from cal_ms %u", name, (unsigned)wait, (unsigned)px.cal_ms);
    CHECK(!job.tick(wait-1u,c) && job.tick(wait,c) && job.error==ULTRA_UCE_TIMEOUT, "%s: restore deadline %u ms", name, (unsigned)wait);
  }
  // A failed replacement can retain a matching prior model.
  load(c); CHECK(c.usable(), "failed replacement retains valid previous coefficients");
  m[0]=1; load(c); CHECK(!c.usable() && !c.have, "old format rejected");
  m[0]=2;

  // The count cap is the board's own (its descriptor's pixel record), by bytes per pixel: a V407 page above the V203's
  // limits is kept, one above its own is dropped; read as the hub reads it (the descriptor through its decode path).
  struct Board : TelemetryPoller {
    void identify(const BoardDescriptor &bd) {
      init(1,0,TelemetryIntervals{}); locked(0);
      auto &r=rx_[0]; auto &d=r.d;
      d.hw_id=bd.bytes[6]; d.hw_rev=1; d.target[0]=bd.bytes[6]; d.target[6]=bd.bytes[10]; d.target[7]=1;
      d.descriptor_length=static_cast<uint16_t>(bd.size); d.descriptor_crc=static_cast<uint16_t>(ultra_get16(bd.bytes+4));
      d.have_board=d.have_features=d.have_target=true; d.features=ULTRA_FEAT_PIXEL_CALIBRATION;
      r.need_identity=false; r.answering=true;
      for (size_t o=0; o<bd.size; o+=24) {
        uint8_t part[24]{}; std::memcpy(part,bd.bytes+o,std::min<size_t>(24,bd.size-o));
        decode_(r,TP_DESCRIPTOR,part,1000);
      }
    }
    const ReceiverData &d() const { return rx_[0].d; }
    bool load(const uint8_t *m,const uint8_t *a,const uint8_t *q) {
      decode_(rx_[0],TP_UD_META,m,2000); decode_(rx_[0],TP_CALIBRATION,a,2010);
      decode_(rx_[0],TP_UD_QUALITY,q,2020); decode_(rx_[0],TP_UD_META,m,2030);
      return pixel_calibration(0).usable();
    }
  };
  static const struct { const char *name; uint16_t px[3]; } LIMITS[] = {
    {"ur_digital_m_v203", {600, 500, 400}}, {"ur_digital_m_v407", {800, 600, 500}}};
  for (const auto &l : LIMITS) {
    const BoardDescriptor *bd = nullptr;
    for (const auto &x : BOARD_DESCRIPTORS) if (!std::strcmp(x.name, l.name)) bd = &x;
    CHECK(bd != nullptr, "%s in the fixture", l.name);
    if (!bd) continue;
    Board b; b.identify(*bd);
    CHECK(b.d().have_identity && TelemetryPoller::digital_calibration(b.d()), "%s identified from its descriptor", l.name);
    for (int k=0; k<3; k++) {
      const uint16_t lim = l.px[k];
      CHECK(b.d().max_pixels[k]==lim, "%s %d bytes per pixel: %u pixels from the descriptor", l.name, k+3, b.d().max_pixels[k]);
      m[13]=static_cast<uint8_t>(k+3);
      for (uint16_t n : {lim, static_cast<uint16_t>(lim+1)}) {
        ultra_put16(m+6,n); ultra_put16(m+8,n); ultra_put16(m+10,n);
        CHECK(b.load(m,a,q)==(n<=lim), "%s %d bytes per pixel: a calibration of %u pixels %s", l.name, k+3, n,
              n<=lim ? "kept" : "dropped");
      }
      for (int off : {6,8,10}) {   // each count on its own
        ultra_put16(m+6,lim); ultra_put16(m+8,lim); ultra_put16(m+10,lim); ultra_put16(m+off,lim+1);
        CHECK(!b.load(m,a,q), "%s %d bytes per pixel: count at offset %d of %u dropped", l.name, k+3, off, lim+1);
      }
    }
  }
  ultra_put16(m+6,1); ultra_put16(m+8,1); ultra_put16(m+10,1); m[13]=5;

  struct Probe : TelemetryPoller {
    void seed() {
      init(3,0,TelemetryIntervals{}); locked(0);
      for (auto &r:rx_) { r.d.have_identity=true; r.need_identity=false; r.d.hw_id=3;r.d.hw_caps=RX_CAP_PIXEL|RX_CAP_RETURN;
        r.d.features=ULTRA_FEAT_PIXEL_CALIBRATION; r.answering=true;
        std::copy_n(fixture_pixel("ur_digital_m_v203").max_pixels,3,r.d.max_pixels); }
    }
    void decode(uint16_t i,uint8_t tp,const uint8_t *p,uint32_t now) { decode_(rx_[i],tp,p,now); }
    void restart(uint16_t i) { restarted_(rx_[i],9000,false); }
  } poll;
  poll.seed(); uint8_t slots[96]{}; uint32_t win;
  poll.calibration_quiet(2,2000);
  CHECK(!poll.due(2999) && poll.due(3000), "at most one progress window per second");
  CHECK(poll.build(3000,slots,&win)==3, "mixed chain target reached");
  CHECK(slots[ULTRA_POLL_INDEX]==ULTRA_POLL_RELAY_ONLY && slots[32+ULTRA_POLL_INDEX]==ULTRA_POLL_RELAY_ONLY,
        "preceding boards relay only during quiet progress");
  CHECK(slots[64+ULTRA_POLL_PAGE]==ULTRA_PAGE_CALIBRATION && slots[64+ULTRA_POLL_ARG]==1,
        "progress requests part 1");
  CHECK(slots[64+ULTRA_POLL_ARG+1]==ULTRA_CAL_KIND_PIXEL,"pixel calibration kind");
  for(int i=2;i<23;i++) CHECK(slots[64+ULTRA_POLL_ARG+i]==0,"reserved poll arguments zero");
  CHECK(!poll.due(3999) && poll.due(4000), "spacing enforced even with overdue pages");
  poll.calibration_quiet(-1,5000);
  m[0]=2;
  poll.decode(2,TP_UD_META,m,5000); poll.decode(2,TP_CALIBRATION,a,5010);
  poll.decode(2,TP_UD_QUALITY,q,5020); poll.decode(2,TP_UD_META,m,5030);
  CHECK(poll.pixel_calibration(2).usable(), "poller assembles format 2");
  poll.restart(2); CHECK(!poll.pixel_calibration(2).usable(), "same UID restart invalidates digital model");
  CHECK(!TelemetryPoller::digital_calibration(ReceiverData{}), "no discovery never enables calibration");
  std::printf("  digital calibration: coherent parts, counts, JSON, quiet polls, cancellation and timeout\n");
}
