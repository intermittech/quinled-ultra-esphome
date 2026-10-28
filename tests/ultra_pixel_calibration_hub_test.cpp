// Executes the component's actual manual calibration entry points and service method.
#include "../components/ultra_line/ultra_core.h"
#include "../components/ultra_line/ultra_telemetry.h"
#include "board_pixel_fixture.h"
#include <cstdio>
using namespace esphome::ultra_line;
#define USE_ULTRA_LINE_TELEMETRY
#define USE_DIG2ANALOG_UR_DIGITAL_M
#define USE_ULTRA_LINE_UPDATE
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
static uint32_t now_ms=10000;
static uint32_t millis(){return now_ms;}
static constexpr uint32_t CAL_BOOT_DELAY_MS=10000, CAL_POWER_WAIT_MS=30000;
static int failures=0;
#define CHECK(x) do { if(!(x)){std::printf("FAIL %d: %s\n",__LINE__,#x); failures++;} } while(0)
namespace esphome::ultra_line {
struct TestPoller : TelemetryPoller {
 const char *desc="ur_digital_m_v203";
 void seed(uint8_t board=3,uint32_t feature=ULTRA_FEAT_PIXEL_CALIBRATION) {
  init(1,0,TelemetryIntervals{});rx_[0].d.have_identity=true;rx_[0].d.hw_id=board;
  rx_[0].d.pixel_cal_wait_ms=board==3?fixture_cal_wait_ms(desc):0;
  rx_[0].d.hw_caps=board==3?RX_CAP_PIXEL:RX_CAP_CURRENT;rx_[0].d.features=feature;rx_[0].answering=true;
 }
 void progress(uint8_t phase,uint8_t error=0) {
  if(uc_.empty())uc_.resize(1);
  uint8_t m[24]{};m[0]=2;m[2]=phase;m[3]=error;m[13]=3;ultra_put16(m+22,1000);
  decode_(rx_[0],TP_UD_META,m,now_ms);
 }
 void rx0_wait(uint32_t ms){rx_[0].d.pixel_cal_wait_ms=ms;}
};
struct TestLight {bool is_lit(uint32_t){return false;} bool pixel_lit(uint32_t){return false;}};
struct UltraHub {
 TestPoller poller_;Scheduler sched_;PixelCalibrationJob pixel_cal_job_;CalSequencer cal_seq_;
 bool cal_pending_=false,cal_started_=false,cal_power_hold_=false,cal_was_=false;
 bool started_=true,tx_prepared_=false,digital_=true,off=false,standby=false,updating=false;
 bool cal_on_boot_=false,cal_boot_done_=false,cal_only_off_=true;
 uint16_t num_boards_=1,preamble_=0;uint8_t pin_=16,cal_req_duty_=0;
 uint32_t cal_wait_ms_=0,cal_interval_ms_=0,cal_last_ms_=0;
 std::vector<uint16_t> cal_req_boards_,cal_boards_;
 std::vector<TestLight*> lights_,pixel_lights_;
 int publications=0,wakes=0;
 UltraHub(){poller_.seed();sched_.init(1,99,3);}
 bool calibration_running()const{return cal_pending_||cal_seq_.running()||pixel_cal_job_.active();}
 bool port_off()const{return off;} bool port_standby()const{return standby;}
 bool update_running()const{return updating;} bool has_telemetry()const{return true;}
 void rw_publish_(){publications++;} void wake_(){wakes++;}
 void calibrate_boards(const std::vector<uint16_t>&,uint8_t);
 void cancel_calibration();bool any_lit_(uint32_t)const;void service_calibration_();void calibrate_receiver(uint16_t);
};
#include "calibration_methods.inc"
}
int main(){
 UltraHub h;
 h.cal_on_boot_=true;h.service_calibration_();CHECK(!h.pixel_cal_job_.active()&&!h.cal_pending_);
 h.calibrate_receiver(0);CHECK(h.pixel_cal_job_.active()&&h.pixel_cal_job_.action==ULTRA_IDENT_CAL&&h.cal_power_hold_);
 h.calibrate_receiver(0);CHECK(h.pixel_cal_job_.action==ULTRA_IDENT_CAL); // repeated button ignored
 h.poller_.seed();h.service_calibration_();CHECK(h.pixel_cal_job_.quiet());
 h.pixel_cal_job_.on_sent(now_ms);
 h.poller_.progress(ULTRA_UCS_CHANNELS);h.service_calibration_();CHECK(h.pixel_cal_job_.active());
 h.cancel_calibration();CHECK(h.pixel_cal_job_.active()&&h.pixel_cal_job_.action==ULTRA_IDENT_RELEASE);
 h.poller_.progress(ULTRA_UCS_FAILED,ULTRA_UCE_CANCELLED);h.service_calibration_();
 CHECK(!h.pixel_cal_job_.active()&&!h.cal_power_hold_);
 // each board's own deadline (its descriptor): a full-length run that takes its whole budget (cal_ms, the V407's
 // 500 RGBCCT ~6 s) finishes; only the descriptor's host deadline times the job out
 for(const char *name:{"ur_digital_m_v203","ur_digital_m_v407"}){
  const uint32_t wait=fixture_cal_wait_ms(name),cal=fixture_pixel(name).cal_ms;
  CHECK(cal>=6000u&&wait==50u+(cal*3u+1u)/2u);
  UltraHub slow;slow.poller_.desc=name;slow.poller_.seed();slow.calibrate_receiver(0);slow.poller_.seed();slow.service_calibration_();
  slow.pixel_cal_job_.on_sent(now_ms);const uint32_t t0=now_ms;
  slow.poller_.progress(ULTRA_UCS_CHANNELS);now_ms=t0+cal;slow.service_calibration_();CHECK(slow.pixel_cal_job_.active());
  slow.poller_.progress(ULTRA_UCS_DONE);slow.service_calibration_();
  CHECK(!slow.pixel_cal_job_.active()&&slow.poller_.pixel_calibration(0).hub_error==0);
  UltraHub lost;lost.poller_.desc=name;lost.poller_.seed();lost.calibrate_receiver(0);lost.poller_.seed();lost.service_calibration_();
  lost.pixel_cal_job_.on_sent(now_ms);const uint32_t t1=now_ms;lost.poller_.progress(ULTRA_UCS_CHANNELS); // then silence
  now_ms=t1+wait-1u;lost.service_calibration_();CHECK(lost.pixel_cal_job_.active());
  now_ms=t1+wait;lost.service_calibration_();
  CHECK(!lost.pixel_cal_job_.active()&&lost.poller_.pixel_calibration(0).hub_error==ULTRA_UCE_TIMEOUT);
  std::printf("  %s: cal_ms %u, hub deadline %u ms\n",name,(unsigned)cal,(unsigned)wait);
 }
 UltraHub nobudget;nobudget.poller_.seed();nobudget.poller_.rx0_wait(0);nobudget.calibrate_receiver(0);
 CHECK(!nobudget.calibration_running()); // a descriptor without a calibration budget starts no job
 UltraHub old;old.poller_.seed(3,0);old.calibrate_receiver(0);CHECK(!old.calibration_running());
 UltraHub analog;analog.digital_=false;analog.poller_.seed(2,ULTRA_FEAT_CALIBRATION);analog.calibrate_receiver(0);
 CHECK(analog.cal_pending_&&!analog.pixel_cal_job_.active());
 UltraHub unknown;unknown.poller_.seed(3,0);unknown.calibrate_boards({},0);CHECK(!unknown.cal_pending_);
 for(int i=0;i<3;i++){
  UltraHub blocked;
  if(i==0)blocked.off=true;if(i==1)blocked.updating=true;if(i==2)blocked.poller_.seed(3,0);
  blocked.calibrate_receiver(0);CHECK(!blocked.pixel_cal_job_.active());
 }
 UltraHub waking;waking.standby=true;waking.calibrate_receiver(0);
 CHECK(waking.pixel_cal_job_.waiting&&waking.cal_power_hold_);
 waking.service_calibration_();CHECK(!waking.pixel_cal_job_.quiet());
 waking.standby=false;waking.poller_.seed();waking.service_calibration_();CHECK(waking.pixel_cal_job_.quiet());
 UltraHub cancelled;cancelled.calibrate_receiver(0);cancelled.cancel_calibration();CHECK(!cancelled.calibration_running());
 UltraHub fast;fast.calibrate_receiver(0);fast.poller_.seed();fast.service_calibration_();fast.pixel_cal_job_.on_sent(now_ms);
 fast.poller_.progress(ULTRA_UCS_DONE);now_ms+=1000;fast.service_calibration_();CHECK(!fast.pixel_cal_job_.active());
 std::printf("digital hub calibration methods: %s (%d failures)\n",failures?"FAILED":"PASS",failures);
 return failures?1:0;
}
