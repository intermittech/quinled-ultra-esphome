#pragma once
// Synthetic fixtures from the protocol release, never product inventory.
#include "../components/ultra_line/ultra_descriptor.h"
#include <algorithm>
#include <cstring>
#include <vector>
namespace esphome::ultra_line {
inline constexpr uint8_t FIX_DESCRIPTOR_ANALOG20[] = {1,11,0,123,217,188,254,20,7,1,201,1,17,0,39,4,1,44,24,1,244,80,7,1,1,0,48,1,0,15,2,13,15,1,1,244,18,192,27,88,39,16,8,0,100,3,12,0,5,0,0,7,8,23,112,8,0,250,0,3,12,5,5,0,0,7,8,23,112,8,0,250,0,3,12,10,5,0,0,7,8,23,112,8,0,250,0,3,12,15,5,0,0,7,8,23,112,8,0,250,0,8,20,70,111,114,109,97,116,32,116,101,115,116,32,97,110,97,108,111,103,50,48};
inline constexpr uint8_t FIX_DESCRIPTOR_SINGLE_PIXEL[] = {1,11,0,120,5,240,254,0,5,1,201,1,17,0,39,4,1,44,24,1,244,80,7,1,1,0,48,1,0,15,2,13,15,1,1,244,18,192,27,88,39,16,8,0,100,4,31,0,0,4,35,23,112,7,208,5,220,4,176,4,226,1,144,0,127,7,208,0,60,8,3,0,1,1,244,1,244,0,7,14,46,224,46,224,0,3,13,64,0,0,234,96,1,1,8,24,70,111,114,109,97,116,32,116,101,115,116,32,115,105,110,103,108,101,95,112,105,120,101,108};
inline constexpr uint8_t FIX_IDENTITY_PART2_ANALOG20[] = {165,4,0,66,18,52,254,4,5,4,3,3,201,0,8,0,80,0,0,7,176,0,140,30,100,2,1,10,0,0,6,245};
inline constexpr uint8_t FIX_IDENTITY_PART2_SINGLE_PIXEL[] = {165,4,0,66,18,52,254,4,5,4,3,3,201,0,8,0,80,0,0,7,176,0,140,30,100,2,1,10,0,0,6,245};
inline std::vector<uint8_t> fixture_descriptor(uint8_t board, uint32_t caps) {
  const bool pixel=caps & RX_CAP_PIXEL;
  const auto *begin=pixel?FIX_DESCRIPTOR_SINGLE_PIXEL:FIX_DESCRIPTOR_ANALOG20;
  const size_t size=pixel?sizeof(FIX_DESCRIPTOR_SINGLE_PIXEL):sizeof(FIX_DESCRIPTOR_ANALOG20);
  std::vector<uint8_t> blob(begin,begin+size); blob[6]=board;
  if (pixel) {
    // Negative controls can withdraw local rendering / chain support independently.
    for (size_t p=11;p<blob.size();p+=2+blob[p+1]) if (blob[p]==ULTRA_DESC_PIXEL) {
      if (!(caps & RX_CAP_PIXEL_FX)) blob[p+5] &= ~ULTRA_PIXEL_LOCAL;
      if (!(caps & RX_CAP_CHAIN)) blob[p+5] &= ~ULTRA_PIXEL_CHAIN;
    }
  }
  ultra_put16(blob.data()+4,ultra_crc16(blob.data()+6,blob.size()-6));
  return blob;
}
inline bool fixture_identity(uint8_t *data, uint8_t part, uint8_t board, uint8_t revision,
                             const uint8_t *uid, uint32_t features, uint32_t caps, uint32_t fw=0x000608ff) {
  const auto descriptor=fixture_descriptor(board,caps);
  if (part==0) {
    std::memcpy(data,uid,12);ultra_put32(data+12,fw);data[16]=3;data[17]=board;data[18]=revision;
    data[19]=3;ultra_put16(data+20,descriptor.size());ultra_put16(data+22,ultra_get16(descriptor.data()+4));
  } else if (part==1) ultra_put32(data,features);
  else if (part==2) {
    const auto *answer=(caps&RX_CAP_PIXEL)?FIX_IDENTITY_PART2_SINGLE_PIXEL:FIX_IDENTITY_PART2_ANALOG20;
    std::memcpy(data,answer+6,24);data[0]=board;data[7]=revision;
  } else return false;
  return true;
}
inline bool fixture_descriptor_part(uint8_t *data,uint8_t part,uint8_t board,uint32_t caps) {
  const auto bytes=fixture_descriptor(board,caps);
  const size_t offset=size_t(part)*24;
  if(offset>=bytes.size())return false;
  std::memcpy(data,bytes.data()+offset,std::min<size_t>(24,bytes.size()-offset));return true;
}
}
