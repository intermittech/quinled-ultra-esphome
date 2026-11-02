// Production wire-3 host contract; synthetic inventory, no receiver firmware dependency.
#include "../components/ultra_line/ultra_core.h"
#include "../components/ultra_line/ultra_telemetry.h"
#include "../components/ultra_line/ultra_transport.h"
#include "wire3_descriptor_fixture.h"
#include "fixtures/board_descriptors.h"
#include <cassert>
#include <cstdio>
#include <set>
using namespace esphome::ultra_line;
static_assert(ULTRA_VERSION == 3);

int main() {
  const uint16_t z[5]{}, full[5]{65535,65535,65535,65535,65535};
  const uint8_t roles[5]{ULTRA_ROLE_RED,ULTRA_ROLE_GREEN,ULTRA_ROLE_BLUE,0,0};
  uint8_t slot[32]{};
  ultra_slot_light_def(slot,239,9,roles,2800,ULTRA_DEF_ZERO_MEANS_ZERO,z,full,235);
  assert(slot[1]==239 && slot[29]==235 && ultra_slot_check(slot)==0);
  ultra_slot_set(slot,239,9,ULTRA_SET_FLAGS,123,full,full,1000,-12);
  assert(slot[0]==(ULTRA_SET|ULTRA_SET_FLAGS) && slot[1]==239 && int16_t(ultra_get16(slot+28))==-12);
  for (uint8_t output:{uint8_t(0),uint8_t(14)}) for (uint8_t segment:{uint8_t(0),uint8_t(14),uint8_t(15)}) {
    ultra_pixel_fx_t fx{}; fx.effect=1; fx.flags=255;
    ultra_slot_pixel_fx(slot,output,segment,9,&fx);
    assert(slot[1]==ULTRA_PIXEL_TARGET(output,segment) && slot[27]==127 && ultra_slot_check(slot)==0);
    ultra_pixel_fx_t decoded{}; assert(ultra_pixel_fx_parse(slot,&decoded)==slot[1]);
  }
  for (uint16_t target=0;target<256;target++) for (uint16_t id=0;id<256;id++) {
    const bool expected=target==255?((id>=1&&id<=10)||(id>=21&&id<=25)||(id>=33&&id<=46)):
      ((target<=239&&(id==11||id==12))||(target<=14&&id>=64&&id<=77));
    assert(ultra_config_scope_ok(target,id)==expected);
  }
  for (uint8_t rate=0;rate<3;rate++) {
    assert(ultra_rw_slot_us(rate)==(rate==0?500u:rate==1?835u:1505u));
    assert(ultra_rw_next_us(16,rate,500)==ultra_rw_window_us(16,rate)+580);
    assert(ultra_rw_next_us(16,rate,1200)==ultra_rw_window_us(16,rate)+1280);
  }
  auto bytes=fixture_descriptor(2,RX_CAP_CURRENT|RX_CAP_RETURN);
  ultra_descriptor_t desc{};
  assert(ultra_descriptor_decode(&desc,bytes.data(),bytes.size())==0);
  for(size_t n=0;n<bytes.size();n++) assert(ultra_descriptor_decode(&desc,bytes.data(),n)!=0);
  auto corrupt=bytes;corrupt.back()^=1;
  assert(ultra_descriptor_decode(&desc,corrupt.data(),corrupt.size())!=0);
  // the firmware boards' own descriptors decode; the Digital-M's LINK is RETURN | RELAY | FORWARD | MARKER | UPDATE_FWD (at any
  // position of a CHAIN line, V203 and V407); CHAIN_END excludes FORWARD and RELAY only: with CHAIN_END its link is refused, and
  // CHAIN_END without FORWARD and RELAY (the last board passing update sessions on) decodes
  for(const auto &bd:BOARD_DESCRIPTORS) {
    ultra_descriptor_t d{}; ultra_desc_link_t link{};
    assert(ultra_descriptor_decode(&d,bd.bytes,bd.size)==0 && ultra_descriptor_link(&d,0,&link)==0);
    if(d.board==3) {
      assert(link.flags==(ULTRA_LINK_RETURN|ULTRA_LINK_RELAY|ULTRA_LINK_FORWARD|ULTRA_LINK_MARKER|ULTRA_LINK_UPDATE_FWD));
      std::vector<uint8_t> b(bd.bytes,bd.bytes+bd.size);ultra_tlv_t tlv{};uint16_t pos=11;size_t off=0;
      while(ultra_descriptor_next(&d,&pos,&tlv)>0) if(tlv.type==1){off=size_t(tlv.value-bd.bytes)+1;break;}
      assert(off && b[off]==link.flags);
      const uint8_t last=uint8_t((link.flags&~(ULTRA_LINK_FORWARD|ULTRA_LINK_RELAY))|ULTRA_LINK_CHAIN_END);
      for(uint8_t extra:{uint8_t(0),uint8_t(ULTRA_LINK_FORWARD),uint8_t(ULTRA_LINK_RELAY)}) {
        b[off]=uint8_t(last|extra);ultra_put16(b.data()+4,ultra_crc16(b.data()+6,b.size()-6));
        assert((ultra_descriptor_decode(&d,b.data(),b.size())==0)==(extra==0));
      }
    }
  }
  {
    // CONFIG 25 (the pulse-skip rate): 2000 / 1000 Hz only on a board whose PWM banks say SKIP_RATE (the Analog-M's own
    // descriptor); a board without it (0.7.2 firmware) or without PWM banks drops it, so the CONFIG CRC leaves it out
    for(const auto &bd:BOARD_DESCRIPTORS) {
      ultra_descriptor_t d{}; uint16_t out=0;
      assert(ultra_descriptor_decode(&d,bd.bytes,bd.size)==0);
      const bool pwm=d.channels!=0;
      assert((ultra_descriptor_config(&d,255,ULTRA_CFG_SKIP_RATE,1000,&out)==0)==pwm && (!pwm || out==1000));
      assert((ultra_descriptor_config(&d,255,ULTRA_CFG_SKIP_RATE,2000,&out)==0)==pwm);
      assert(ultra_descriptor_config(&d,255,ULTRA_CFG_SKIP_RATE,500,&out)!=0);
      if(!pwm) continue;
      std::vector<uint8_t> b(bd.bytes,bd.bytes+bd.size);ultra_tlv_t tlv{};uint16_t pos=11;size_t off=0;
      while(ultra_descriptor_next(&d,&pos,&tlv)>0) if(tlv.type==3){off=size_t(tlv.value-bd.bytes)+3;break;}
      assert(off && (b[off]&ULTRA_PWM_SKIP_RATE));
      b[off]&=uint8_t(~ULTRA_PWM_SKIP_RATE);ultra_put16(b.data()+4,ultra_crc16(b.data()+6,b.size()-6));
      ultra_descriptor_t o{}; assert(ultra_descriptor_decode(&o,b.data(),b.size())==0);
      assert(ultra_descriptor_config(&o,255,ULTRA_CFG_SKIP_RATE,2000,&out)!=0);
      Scheduler old; old.init(1,22,1); old.require_descriptors(1);
      assert(old.descriptor(0,b)); old.set_config(0,ULTRA_CFG_SKIP_RATE,2000); assert(!old.has_config(0,ULTRA_CFG_SKIP_RATE));
      Scheduler cur; cur.init(1,22,1); cur.require_descriptors(1);
      std::vector<uint8_t> g(bd.bytes,bd.bytes+bd.size);
      assert(cur.descriptor(0,g)); cur.set_config(0,ULTRA_CFG_SKIP_RATE,1000); assert(cur.has_config(0,ULTRA_CFG_SKIP_RATE));
      const uint8_t canon[]{255,ULTRA_CFG_SKIP_RATE,0x03,0xe8};
      assert(cur.config_crc(0)==ultra_crc16(canon,sizeof canon));
    }
  }
  {
    // a board without a return channel and without a YAML descriptor (a Dig2Analog-Plus-5ch pin up to 20 boards): its
    // LIGHT_DEF and SET go out (waiting would mean NOP slots forever); one that reports its own waits for it (NOP)
    for (uint16_t returns : {uint16_t(0), uint16_t(1)}) {
      Scheduler s; s.init(1,22,1); s.require_descriptors(returns);
      s.light_def(0,0,roles,1000,0,z,full);
      s.light_set(0,0,ULTRA_SET_SMOOTH,1000,full,full,500,1000000);
      s.flush_config();
      uint8_t f[40]{}; std::set<uint8_t> seen;
      for(int k=0;k<40;k++){s.build_frame(k*10000,f);seen.insert(f[0]&ULTRA_TYPE_MASK);}
      assert(returns ? seen.size()==1 && seen.count(ULTRA_NOP) && s.descriptor_missing(0)
                     : seen.count(ULTRA_LIGHT_DEF) && seen.count(ULTRA_SET) && !s.descriptor_missing(0));
    }
  }
  Scheduler scheduler; scheduler.init(1,22,1); scheduler.require_descriptors(1);
  scheduler.light_def(0,0,roles,1000,0,z,full);
  scheduler.light_set(0,0,ULTRA_SET_SMOOTH,1000,full,full,500,1000000);
  scheduler.set_config(0,ULTRA_CFG_PWM_PRESET,8);
  scheduler.set_config(0,13,60000); scheduler.set_config(0,32,60000); scheduler.set_config(0,48,10);
  scheduler.set_channel_config(0,0,ULTRA_CFG_CH_CURRENT,1000);
  scheduler.set_channel_config(0,0,ULTRA_CFG_CH_MAX,1000);
  scheduler.light_scene(0,0,ULTRA_SCENE_POWER_ON,3,1000,full,full);
  scheduler.light_scene(0,0,ULTRA_SCENE_SIGNAL_LOSS,9,2000,full,full);
  scheduler.flush_config();
  uint8_t frame[40]{};
  scheduler.build_frame(0,frame);assert(frame[0]==ULTRA_NOP && !scheduler.pending());
  assert(!scheduler.descriptor(0,corrupt));assert(scheduler.descriptor(0,bytes));
  const uint8_t canonical[]{0,11,3,232,0,12,3,232,255,1,0,8};
  assert(scheduler.config_crc(0)==ultra_crc16(canonical,sizeof canonical));
  bool scenes[2]{};std::set<uint8_t> types;
  for(int k=0;k<300;k++) {
    scheduler.build_frame(k*10000,frame);
    const uint8_t type=frame[0]&ULTRA_TYPE_MASK;types.insert(type);
    assert(ultra_command_known(frame[0]) && ultra_slot_check(frame)==0);
    assert(frame[1]<=239 || frame[1]==255);
    if(type==ULTRA_SCENE){const auto mode=frame[3];assert(mode<2);scenes[mode]=true;
      assert(frame[4]==(mode?9:3));assert(ultra_get16(frame+5)==(mode?2000:1000));}
    if(type==ULTRA_CONFIG)for(int i=0;i<9;i++)assert(frame[3+3*i]==0 || ultra_config_scope_ok(frame[1],frame[3+3*i]));
  }
  assert(scenes[0]&&scenes[1]&&!types.count(3)&&!types.count(4)&&!types.count(10));
  for(unsigned flags=0;flags<256;flags+=32)for(uint8_t type:{uint8_t(3),uint8_t(4),uint8_t(10)}){
    ultra_slot_begin(slot,type|(flags&128),0,flags&96,9);ultra_slot_seal(slot);
    assert(!ultra_command_known(slot[0]) && ultra_slot_check(slot)==0);
  }
  puts("WIRE-3 PRODUCTION CONTRACT PASSED: targets, scopes, CRC, LIGHT_DEF, PIXEL_FX, descriptors (a no-return board's commands go out without one), timing, two SCENE levels, reserved ids");
}
