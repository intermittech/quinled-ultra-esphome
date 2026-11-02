// The analog effect payloads produced by the real output path against the pinned wire-3 byte builders.
// Transport addressing, CONFIG migration and receiver integration are deliberately separate from this contract.
#include "../components/ultra_line/ultra_protocol.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
static_assert(ULTRA_VERSION == 3, "wire-3 contract");
static_assert(ULTRA_SLOT_BYTES == 32, "budgeted SET size");
int main(int argc,char **argv) {
  assert(argc==2);
  FILE *in=std::fopen(argv[1],"rb"); assert(in);
  uint8_t old[ULTRA_SLOT_BYTES], slot[ULTRA_SLOT_BYTES];
  unsigned sets=0, scenes=0;
  while(std::fread(old,sizeof old,1,in)==1) {
    uint16_t a[5],b[5]; const uint8_t *p=old+ULTRA_PAYLOAD;
    if((old[0] & ULTRA_TYPE_MASK)==ULTRA_SET) {
      for(int k=0;k<5;k++) { a[k]=ultra_get16(p+2+2*k); b[k]=ultra_get16(p+12+2*k); }
      const uint8_t flags=old[0]&ULTRA_TYPE_FLAGS;
      ultra_slot_set(slot,old[1],old[2],flags,ultra_get16(p),a,b,ultra_get24(p+22),int16_t(ultra_get16(p+25)));
      assert((slot[0]&ULTRA_TYPE_MASK)==ULTRA_SET); sets++;
    } else {
      assert(old[0]==ULTRA_SCENE && (p[0]==ULTRA_SCENE_POWER_ON || p[0]==ULTRA_SCENE_SIGNAL_LOSS));
      for(int k=0;k<5;k++) { a[k]=ultra_get16(p+4+2*k); b[k]=ultra_get16(p+14+2*k); }
      ultra_slot_scene(slot,old[1],old[2],p[0],p[1],ultra_get16(p+2),a,b); scenes++;
    }
    assert(ultra_slot_check(slot)==0 && ultra_command_known(slot[0]));
    assert(std::memcmp(slot+ULTRA_PAYLOAD,p,ULTRA_PAYLOAD_LEN)==0);
  }
  assert(std::feof(in) && sets==9 && scenes==2); std::fclose(in);
  // Reserved analog opcodes remain unknown even with SET flags or the broadcast bit set and valid CRCs.
  for(unsigned flags=0;flags<256;flags++) {
    for(uint8_t type:{uint8_t(3),uint8_t(4),uint8_t(10)}) {
      if((flags & 0x1F)!=0) continue;
      ultra_slot_begin(slot,type|uint8_t(flags&0x80),0,uint8_t(flags&0x60),1); ultra_slot_seal(slot);
      assert(ultra_slot_check(slot)==0 && !ultra_command_known(slot[0]));
    }
  }
  const uint16_t zero[5]={};
  for(uint8_t scene:{uint8_t(ULTRA_SCENE_POWER_ON),uint8_t(ULTRA_SCENE_SIGNAL_LOSS)}) {
    ultra_slot_scene(slot,239,7,scene,3,0,zero,zero);
    assert(slot[1]==239 && slot[ULTRA_PAYLOAD]==scene && slot[ULTRA_PAYLOAD+1]==3);
    for(unsigned i=ULTRA_PAYLOAD+2;i<30;i++) assert(slot[i]==0);
  }
  ultra_slot_set(slot,239,8,ULTRA_SET_SMOOTH|ULTRA_SET_VIA_OFF,1,zero,zero,250,0);
  assert(slot[1]==239 && (slot[0]&ULTRA_TYPE_FLAGS)==(ULTRA_SET_SMOOTH|ULTRA_SET_VIA_OFF));
  std::puts("WIRE-3 ANALOG CONTRACT PASSED: 9 production effect payloads, 2 levels, reserved-opcode negative controls");
}
