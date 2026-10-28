#include "../components/ultra_line/ultra_pixel_calibration.h"
using namespace esphome::ultra_line;
int main() {
  for (unsigned bpp=3;bpp<=5;bpp++) {
    uint8_t m[24]{},a[24]{},q[24]{};
    m[0]=2;m[1]=ULTRA_UCF_VALID|ULTRA_UCF_AGGREGATE;m[2]=ULTRA_UCS_DONE;m[12]=255;m[13]=bpp;
    ultra_put16(m+4,65535);ultra_put16(m+6,1);ultra_put16(m+8,1);ultra_put16(m+10,1);
    for(unsigned offset : {14u,16u,18u,20u})ultra_put16(m+offset,65535);
    a[0]=3;a[1]=2;ultra_put16(a+2,65535);ultra_put16(a+7,32768);
    for(unsigned i=0;i<bpp;i++)ultra_put16(a+9+2*i,65535);
    q[0]=2;q[1]=1;ultra_put16(q+2,65535);ultra_put16(q+4,65535);
    const uint16_t cap[3]{1,1,1};   // (counts of 1: the widest JSON, any board)
    PixelCalibration c;c.read(1,m,false,cap);c.read(0,a,false,cap);c.read(2,q,false,cap);c.read(1,m,false,cap);
    if(!c.usable())return 1;
    std::puts(c.json(true).c_str());std::puts(c.json(false).c_str());
  }
}
