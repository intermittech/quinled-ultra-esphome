#include "../components/ultra_line/ultra_core.h"
#include "../components/ultra_line/ultra_telemetry.h"
#include "../components/ultra_line/ultra_parlio.h"
#include "../components/ultra_line/ultra_host_effects.h"
#include "wire3_descriptor_fixture.h"
#include <cassert>
#include <cstdio>
using namespace esphome::ultra_line;

struct Poller : TelemetryPoller {
  void prepare() { this->rx_.resize(1); this->rx_[0].d.line_ingress=0; }
  void read(uint8_t part, uint8_t *bytes, uint32_t now) {
    this->decode_(this->rx_[0], part ? TP_LINK_CONTEXT : TP_LINK_STATS, bytes, now);
  }
  void restart() { this->restarted_(this->rx_[0], 5000, false); }
};

int main() {
  // answer_link_stats_rate_request from the pinned production protocol vectors.
  static constexpr const char *VEC_ANSWER_LINK_STATS_RATE_REQUEST =
      "a513004276730001131d012c01450139039803b603aa0200020000000007fa58";
  uint8_t answer[32]{};
  for (size_t k=0;k<sizeof answer;k++) {
    unsigned byte; assert(std::sscanf(VEC_ANSWER_LINK_STATS_RATE_REQUEST+2*k,"%2x",&byte)==1);
    answer[k]=byte;
  }
  assert(!ultra_answer_check(answer) && answer[1]==ULTRA_PAGE_LINK_STATS);
  assert(answer[ULTRA_RSP_DATA+ULTRA_LS_FLAGS] & ULTRA_LS_RATE_REQUEST);
  LineRatePolicy a,b;
  a.configure({1000,32,10,3,true}); a.reset(2); b.reset(1);
  LineCounters s{}; s.coherent=true; s.flags=ULTRA_LS_VALID|ULTRA_LS_LOCKED; s.profile=0; s.supported=15;
  s.valid=90;s.bad=10;
  for (uint32_t now=0;now<=3000;now+=1000) {
    s.sampled_ms=now;s.epoch++;
    a.observe(1,s,now);
  }
  assert(a.requested()==1 && a.current()==0 && b.requested()==0);
  a.applied(1); s.profile=1;s.valid=100;s.bad=0;
  for(uint32_t now=4000;now<70000;now+=1000) {s.sampled_ms=now;s.epoch++;a.observe(0,s,now);}
  assert(a.requested()==1); // good current-rate reports never cause an upshift
  assert(!receiver_requests_lower_rate(s));
  assert(a.request(0) && a.requested()==0 && !a.request(4));
  a.applied(0);s.profile=0;s.valid=1;s.bad=1;
  for(uint32_t now=70000;now<80000;now+=1000) {s.sampled_ms=now;s.epoch++;a.observe(0,s,now);}
  assert(a.requested()==0); // insufficient frames cannot request a change
  s.valid=100;s.bad=10;s.flags|=ULTRA_LS_OVERFLOW;
  for(uint32_t now=80000;now<90000;now+=1000) {s.sampled_ms=now;s.epoch++;a.observe(0,s,now);}
  assert(a.requested()==0); // saturated counters are unknown
  s.flags=ULTRA_LS_VALID|ULTRA_LS_LOCKED;s.sampled_ms=0;s.epoch++;
  assert(!a.observe(0,s,90000)); // stale samples never count
  for(uint8_t profile=0;profile<4;profile++) {
    ultra_line_rmt_t r{};assert(!ultra_line_rmt(profile,40000000,&r));
    assert(r.zero_ticks<r.one_ticks && r.one_ticks<r.bit_ticks);
    assert(!ultra_line_allowed(profile,false,false,15,0) || profile==0);
    assert(ultra_line_allowed(profile,true,true,15,100));
    assert(!ultra_line_allowed(profile,true,true,1,100) || profile==0);
  }
  Poller p;p.prepare(); uint8_t widths[24]{},context[24]{};
  widths[ULTRA_LS_FLAGS]=ULTRA_LS_VALID|ULTRA_LS_LOCKED;
  ultra_put16(widths+ULTRA_LS_EPOCH,65535);
  context[ULTRA_LC_FLAGS]=widths[ULTRA_LS_FLAGS];context[ULTRA_LC_PROFILE]=0;context[ULTRA_LC_SUPPORTED]=15;
  ultra_put16(context+ULTRA_LC_EPOCH,65535);ultra_put16(context+ULTRA_LC_FRAMES,123);ultra_put16(context+ULTRA_LC_CRC_ERRORS,4);
  p.read(0,widths,1000);p.read(1,context,1100);
  assert(p.data(0).line_counters.coherent && p.data(0).line_counters.valid==123 && p.data(0).line_counters.bad==4);
  p.read(0,widths,1200);ultra_put16(context+ULTRA_LC_EPOCH,0);p.read(1,context,1300);
  assert(!p.data(0).line_counters.coherent); // an epoch wrap does not authorize mixing parts
  ultra_put16(widths+ULTRA_LS_EPOCH,0);p.read(0,widths,1400);p.read(1,context,3500);
  assert(!p.data(0).line_counters.coherent); // frozen pairs expire after 2 s
  p.read(0,widths,4000);p.restart();p.read(1,context,5100);
  assert(!p.data(0).line_counters.coherent); // never join snapshots across a boot
  // The explicit latch is primary, even with few frames or saturated counters.
  Poller flagged;flagged.prepare();LineRatePolicy rate;rate.reset(1);
  auto *request=answer+ULTRA_RSP_DATA;
  std::memset(context,0,sizeof context);
  context[ULTRA_LC_FLAGS]=request[ULTRA_LS_FLAGS];context[ULTRA_LC_PROFILE]=0;context[ULTRA_LC_SUPPORTED]=15;
  ultra_put16(context+ULTRA_LC_EPOCH,ultra_get16(request+ULTRA_LS_EPOCH)+1);
  flagged.read(0,request,1000);flagged.read(1,context,1100);
  assert(!rate.observe(0,flagged.data(0).line_counters,1100) && rate.requested()==0);
  ultra_put16(context+ULTRA_LC_EPOCH,ultra_get16(request+ULTRA_LS_EPOCH));
  flagged.read(0,request,1200);flagged.read(1,context,1300);
  auto snapshot=flagged.data(0).line_counters;
  assert(snapshot.coherent && receiver_requests_lower_rate(snapshot));
  auto absent=snapshot;absent.flags&=~ULTRA_LS_RATE_REQUEST;
  assert(!rate.observe(0,absent,1300)); // no latch, no counter window
  auto unsupported=snapshot;unsupported.supported=1;
  assert(!rate.observe(0,unsupported,1300) && rate.requested()==0);
  auto stale=snapshot;stale.sampled_ms=0;
  assert(!rate.observe(0,stale,3000));
  auto invalid=snapshot;invalid.flags&=~ULTRA_LS_VALID;
  assert(!rate.observe(0,invalid,1300));
  auto unlocked=snapshot;unlocked.flags&=~ULTRA_LS_LOCKED;
  assert(!rate.observe(0,unlocked,1300));
  snapshot.flags|=ULTRA_LS_OVERFLOW;
  assert(rate.observe(0,snapshot,1300) && rate.requested()==1 && b.requested()==0);
  assert(!rate.observe(0,snapshot,1301)); // one request, even before the port drains
  snapshot.epoch++;
  assert(!rate.observe(0,snapshot,1302)); // sticky latch, new snapshot: still one step
  rate.applied(1);
  assert(!rate.observe(0,snapshot,1303) && rate.requested()==1); // old acquisition/profile
  rate.configure({5000,32,10,3,false});snapshot.profile=1;
  assert(!rate.observe(0,snapshot,1304) && rate.requested()==1);
  rate.configure({5000,32,10,3,true});rate.applied(3);snapshot.profile=3;
  assert(!rate.observe(0,snapshot,1305) && rate.requested()==3);
  // Flags must also match across the pair; an epoch match alone is insufficient.
  context[ULTRA_LC_FLAGS]&=~ULTRA_LS_RATE_REQUEST;
  flagged.read(0,request,1400);flagged.read(1,context,1500);
  assert(!flagged.data(0).line_counters.coherent);
  // Mixed requested rates retain their proportions while sharing one capacity
  // factor; no faster light can consume the slower light's share.
  assert(effect_interval_for_demand(30,20,1,40000)==200);
  assert(effect_interval_for_demand(30,10,1,40000)==400);
  // Profile changes discard old-clock reservations, including a posted future
  // frame after a long quiet interval. Its group number must never carry over.
  for (uint8_t profile=0;profile<4;profile++) {
    ParlioStream stream;ParlioLaneTiming timing[1]{{12,38}};
    assert(stream.init(timing,1,0,25,8,50,16,0));
    auto *lane=stream.lane(0);assert(lane->begin(40,nullptr,nullptr));
    assert(stream.start(0,1000000,1000000,1000));
    std::vector<uint8_t> a(stream.chunk_bytes()),b(stream.chunk_bytes());
    void *buffers[]{a.data(),b.data()};uint8_t frame[40]{};
    lane->reserve_us();assert(lane->post(frame,sizeof frame,2000000,false));
    for(int k=0;k<2;k++) {const auto chunk=stream.render(buffers);assert(stream.handed_over(chunk,1000));}
    assert(!lane->busy());
    lane->reserve_us();assert(lane->post(frame,sizeof frame,500,false));
    const auto old_id=lane->posted_id(),old_epoch=lane->posted_epoch();
    const auto old_end=lane->frame_end_us();
    stream.stop(1001);stream.drop_all();stream.reset_reservations();
    assert(!lane->busy() && lane->take_refused());
    assert(lane->outcome(old_id,old_epoch,old_end,old_end)==ParlioLane::FAILED);
    ultra_line_rmt_t recipe{};assert(!ultra_line_rmt(profile,40000000,&recipe));
    timing[0]={recipe.zero_ticks,recipe.one_ticks};
    assert(stream.init(timing,1,0,25,8,recipe.bit_ticks,16,0));
    assert(stream.start(0,1500000,1500000,1500));
    assert(lane->ready());
    const auto next=lane->reserve_us();
    const auto expected=1500+3*stream.chunk_ns()/1000;
    assert(next>=static_cast<int64_t>(expected) && next<=static_cast<int64_t>(expected+1));
  }
  for (uint8_t profile=0;profile<4;profile++) {
    ultra_line_rmt_t r{};ultra_line_rmt(profile,40000000,&r);
    for(uint16_t boards : {1,3,8}) {
      const uint32_t frame=((boards*32u+8u)*8u*r.bit_ticks+39u)/40u+500u;
      const double budget=boards*300000.0/frame;
      const uint32_t interval=analog_effect_interval(boards*5,boards,frame);
      assert(boards*5*1000.0/interval<=budget+0.001);
      std::printf("  profile %u / %u boards: %.2f SET/s budget, %u ms for %u lights\n",profile,boards,budget,interval,boards*5);
    }
  }
  puts("WIRE-3 RATE CONTRACT PASSED: RATE_REQUEST vector, coherent snapshots, one supported step, counter fallback, no automatic upshift, effect budgets");
}
