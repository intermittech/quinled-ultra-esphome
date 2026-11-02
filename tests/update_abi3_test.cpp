// ABI-3 update objects on a PC (tests/run_update_abi3_test.py): the frame / descriptor / FW_UPDATE codecs of the
// production ultra_update.h against the protocol's vectors, and the checks of a release's manifest and downloaded
// image (fw_manifest.h). The port job's plan and report: tests/port_update_test.cpp.
#include <cassert>
#include <cstdio>
#include <vector>
#include "../components/ultra_line/fw_manifest.h"
#include "update_abi3_vectors.h"
using namespace esphome::ultra_line;
static void seal(uint8_t *d) { ultra_put32(d+60,upd_crc32_words(0xffffffffu,d,15)); }
static const uint8_t V203[] = {1,1,1,1,3,0,0,0x11,8,0,0x21,0,0,0,0xdf,0};
static std::vector<uint8_t> image(const uint8_t *target,uint32_t length=260) {
  std::vector<uint8_t> v(length+64);
  memcpy(v.data(),abi3_image_v407_range,64);
  memcpy(v.data()+8,target,7);v[6]=0;v[7]=255;v[15]=3;
  ultra_put32(v.data()+20,length);
  const auto base=ultra_get32(target+8);
  ultra_put32(v.data()+28,base);ultra_put32(v.data()+32,base+2);
  for(uint32_t k=0;k<length;++k) v[64+k]=uint8_t(k);
  ultra_put32(v.data()+24,upd_crc32_words(0xffffffffu,v.data()+64,length/4));seal(v.data());return v;
}
static void vectors() {
  const uint8_t *vectors[]={abi3_update_announce0,abi3_update_announce1,abi3_update_data_index65535,
    abi3_update_end,abi3_update_select,abi3_update_status_poll,abi3_update_abort,abi3_update_ping,
    abi3_update_commit_ack,abi3_update_idle_ping};
  for(const auto v:vectors) {
    uint8_t f[48];upd_build_frame(f,v[0],v[1],uint16_t(ultra_get16(v+2)),v[4],v[5],v+6,0xabcdef);
    assert(!memcmp(f,v,48));assert(upd_check_frame(f)==0);
    f[42]^=1;assert(upd_check_frame(f)!=0);
  }
  upd_target_t t;memcpy(t.bytes,abi3_target_single_pixel,16);
  assert(upd_desc_check(abi3_image_v407_range,&t,3)==0);
  assert(upd_desc_check(abi3_image_future_app_formats,&t,3)==0);
  t.bytes[6]^=1;assert(upd_desc_check(abi3_image_v407_range,&t,3)==UPD_ERROR_TARGET);
  for(const auto a:{abi3_answer_fw_update_confirmed,abi3_update_status_busy,abi3_update_status_ready,
                    abi3_update_status_committed}) assert(ultra_answer_check(a)==0);
}
static void manifest() {
  FwManifest m;m.channel="stable";m.version="0.7.0";FwImageInfo i;auto v=image(V203);
  memcpy(i.target.bytes,V203,16);i.rev_min=0;i.rev_max=255;i.min_bootloader=3;i.size=324;i.url="app.bin";
  i.sha256=std::string(64,'0');memcpy(i.descriptor,v.data(),64);m.images.push_back(i);
  assert(!fw_check_manifest(m,FW_CH_STABLE) && m.images[0].version_word==0x000700ff);
  assert(fw_check_manifest(m,FW_CH_BETA));
  m.images[0].size=323;assert(fw_check_manifest(m,FW_CH_STABLE));m.images[0].size=324;
  m.images[0].sha256="xyz";assert(fw_check_manifest(m,FW_CH_STABLE));m.images[0].sha256=std::string(64,'0');
  m.images[0].target.bytes[4]=2;assert(fw_check_manifest(m,FW_CH_STABLE));m.images[0].target.bytes[4]=3;
  assert(!fw_check_manifest(m,FW_CH_STABLE));i=m.images[0];
  m.images[0].descriptor[15]^=1;assert(fw_check_manifest(m,FW_CH_STABLE));m.images[0].descriptor[15]^=1;
  assert(!fw_check_manifest(m,FW_CH_STABLE));
  // a downloaded image is the manifest's: size, SHA-256, descriptor and body CRC, descriptor against the entry
  uint8_t sha[32]{};
  assert(!fw_check_image(v.data(),v.size(),sha,i));
  sha[0]=1;assert(fw_check_image(v.data(),v.size(),sha,i));sha[0]=0;
  assert(fw_check_image(v.data(),v.size()-4,sha,i));
  v[64]^=1;assert(fw_check_image(v.data(),v.size(),sha,i));v[64]^=1;
  v[61]^=1;assert(fw_check_image(v.data(),v.size(),sha,i));v[61]^=1;
  auto resealed=v;resealed[40]^=1;seal(resealed.data());assert(fw_check_image(resealed.data(),resealed.size(),sha,i));
  auto other=i;other.version_word=0x000701ff;assert(fw_check_image(v.data(),v.size(),sha,other));
  other=i;other.rev_max=0x11;assert(fw_check_image(v.data(),v.size(),sha,other));
  other=i;other.min_bootloader=4;assert(fw_check_image(v.data(),v.size(),sha,other));
  other=i;other.target.bytes[0]=2;assert(fw_check_image(v.data(),v.size(),sha,other));
  other=i;other.target.bytes[10]=0x22;assert(fw_check_image(v.data(),v.size(),sha,other));
  assert(fw_manifest_url("x/y",FW_CH_BETA)=="https://raw.githubusercontent.com/x/y/channels/beta.json");
  assert(fw_resolve_url("https://h/c/stable.json","fw.bin")=="https://h/c/fw.bin");
}
int main() { vectors();manifest();puts("ABI-3 update: frame/descriptor vectors, manifest and image checks passed"); }
