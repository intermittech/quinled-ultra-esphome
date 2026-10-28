#include "../components/ultra_line/ultra_core.h"
#include "../components/ultra_line/ultra_host_effects.h"
#include "../components/ultra_line/ultra_change_log.h"
#include <cstdio>
#include <string>
#include <initializer_list>
#include <cassert>
#include <array>
static uint32_t now_ms;
static FILE *samples;
static uint32_t millis() { return now_ms; }
static float random_float() { static uint32_t seed=7; seed=seed*1664525+1013904223; return float(seed >> 8)/16777216.0f; }
namespace esphome {
template<class T> using FixedVector = std::vector<T>;
namespace light {
enum class ColorCapability : uint8_t { RGB=32, COLOR_TEMPERATURE=8, COLD_WARM_WHITE=16 };
using ColorMode = uint8_t;
static bool operator&(ColorMode a, ColorCapability b) { return (a & uint8_t(b)) != 0; }
struct LightColorValues {
  float state{1}, brightness{1}, color_brightness{1}, red{1}, green{1}, blue{1}, white{1}, cold_white{1}, warm_white{1}, color_temperature{0};
  ColorMode mode{32};
  bool is_on() const { return state > 0; }
  ColorMode get_color_mode() const { return mode; }
#define FIELD(x) float get_##x() const { return x; } void set_##x(float v) { x = std::clamp(v,0.0f,1.0f); }
  FIELD(state) FIELD(brightness) FIELD(color_brightness) FIELD(red) FIELD(green) FIELD(blue) FIELD(white) FIELD(cold_white) FIELD(warm_white)
#undef FIELD
  float get_color_temperature() const { return color_temperature; }
};
struct LightOutput { virtual ~LightOutput() = default; };
struct LightState;
struct Call {
  LightState *s; LightColorValues v; uint32_t transition{0}; bool finish{false};
  void set_publish(bool) {} void set_save(bool) {}
  void set_effect(uint32_t) { finish=true; }
  void set_transition_length(uint32_t t) { transition=t; }
  void set_transition_length_if_supported(uint32_t t) { transition=t; }
  void from_light_color_values(const LightColorValues &x) { v=x; }
#define CALL(x) void set_##x(float x) { v.set_##x(x); }
  CALL(state) CALL(brightness) CALL(red) CALL(green) CALL(blue) CALL(cold_white) CALL(warm_white)
#undef CALL
  void set_brightness_if_supported(float x) { v.set_brightness(x); }
  void set_color_temperature(float) {}
  void perform();
};
struct Traits { float get_min_mireds() const { return 150; } float get_max_mireds() const { return 400; } };
struct LightState {
  LightColorValues remote_values, current_values; LightOutput *output{nullptr}; uint32_t effect{1};
  LightOutput *get_output() { return output; }
  uint32_t get_current_effect_index() const { return effect; }
  float get_gamma_correct() const { return 1; }
  Traits get_traits() { return {}; }
  Call make_call() { return {this, remote_values}; }
  Call turn_on() { return make_call(); }
  Call turn_off() { auto c=make_call(); c.set_state(false); return c; }
};
struct LightEffect {
  explicit LightEffect(const char *n) : name_(n) {}
  virtual ~LightEffect()=default;
  virtual void start() {} virtual void stop() {} virtual void apply()=0;
  void init(LightState *s) { state_=s; }
 protected: LightState *state_{}; const char *name_;
};
#include "native.inc"
}
namespace ultra_line {
struct InspectScheduler : Scheduler {
  void intended_level(uint8_t selector,uint8_t *slot) const {
    const auto &l=*boards_[0].light[0];
    std::memcpy(slot,selector ? l.loss_scene.slot : l.scene.slot,ULTRA_SLOT_BYTES);
    slot[ULTRA_PAYLOAD]=selector; ultra_slot_seal(slot);
  }
};
struct Hub {
  InspectScheduler sched; Hub() { sched.init(1, 11, 1); }
  size_t lights{0};
  size_t analog_light_count() const { return lights; }
  uint32_t frame_us{830};
  uint32_t effect_interval(uint16_t rate=20) const { return analog_effect_interval(lights,sched.num_boards(),frame_us,rate); }
  Scheduler &scheduler() { return sched; }
  int64_t stream_start_us() { return int64_t(now_ms)*1000+5000; }
  int64_t start_time_us(uint16_t) { return stream_start_us(); }
  void wake_() {}
  // the log of light changes (tests/ultra_change_log_test.cpp): counted only, the frames must not change with it
  size_t changes{0}, streams{0}, kept{0};
  void note_change(uint16_t, const char *, uint8_t, const uint16_t *, uint32_t, int64_t) { changes++; }
  void note_stream(uint16_t, uint8_t) { streams++; }
  void note_kept(uint16_t) { kept++; }
};
enum { GLIDE_OFF, GLIDE_ON, GLIDE_AUTO, EFFECT_HARD=1 };
class UltraLight : public light::LightOutput {
 public:
  Hub *hub_; light::LightState *state_; uint16_t board_{0}; uint8_t index_{0};
  const char *platform_{"dig2analog_plus"}; uint8_t channel_mask_() const { return 0x1F; }
  bool sent_{false}; uint16_t last_m_{0}, last_a_[5]{}, last_b_[5]{};
  uint8_t glide_mode_{GLIDE_AUTO}; GlideTimer glide_; std::vector<int> layers_;
  EffectCadence effect_cadence_; light::LightColorValues effect_pending_values_;
  uint32_t effect_pending_length_{0}; uint8_t effect_pending_flags_{0}; bool effect_pending_{false};
  bool pixel_pending_{false}, pixel_pending_stream_{true}; uint16_t pixel_m_{0},pixel_a_[5]{},pixel_b_[5]{};
  uint32_t pixel_length_{0}; uint8_t pixel_flags_{0};
  light::LightColorValues stored_values_[2]; uint8_t stored_fade_[2]{3,3}; float cold_white_temperature_{150},warm_white_temperature_{400};
  static std::vector<UltraLight *> registry_;
  UltraLight(Hub &h,light::LightState &s):hub_(&h),state_(&s) { s.output=this; registry_.push_back(this); h.lights++; }
  ~UltraLight() { registry_.erase(std::find(registry_.begin(),registry_.end(),this)); hub_->lights--; }
  static UltraLight *find(light::LightOutput *out) { return dynamic_cast<UltraLight *>(out); }
  uint16_t effect_rate() const { return 20; }
  uint32_t effect_interval() const;
  uint8_t effect_kind_(uint32_t) const { return hard ? EFFECT_HARD : 0; }
  bool hard{false};
  void map(const light::LightColorValues &v,uint16_t &m,uint16_t *a,uint16_t *b) const {
    const uint8_t roles[5]={ULTRA_ROLE_RED,ULTRA_ROLE_GREEN,ULTRA_ROLE_BLUE,ULTRA_ROLE_WARM_WHITE,ULTRA_ROLE_COLD_WHITE};
    ColorValues values{v.state,v.brightness,v.color_brightness,v.red,v.green,v.blue,v.white,v.cold_white,v.warm_white,v.mode};
    map_values(roles,values,m,a,b);
  }
  void lit_(uint16_t,uint32_t,int64_t) {}
  void send_(const light::LightColorValues &,uint32_t,uint8_t,bool=true);
  void send_effect_values(const light::LightColorValues &v,uint32_t length) { send_(v,length,0,true); }
  void send_pixel(uint16_t,const uint16_t *,const uint16_t *,uint32_t,uint8_t,bool,int64_t,bool=false);
  void send_stored_level_(uint8_t);
  void loop();
};
std::vector<UltraLight *> UltraLight::registry_;
#include "output.inc"
}
void light::Call::perform() {
  s->current_values=v;
  if (finish) s->effect=0;
  static_cast<ultra_line::UltraLight *>(s->output)->send_(v,transition,0,!finish);
}
}
#include "effects.inc"
using namespace esphome;
using namespace esphome::ultra_line;
static bool removed(uint8_t type) { return type==3 || type==4 || type==10; }
static void exercise(light::LightEffect &fx,const char *name,uint32_t first_length,bool hard=false) {
  now_ms=0; Hub hub; light::LightState state; UltraLight output(hub,state); output.hard=hard;
  fx.init(&state); fx.start();
  uint32_t last=0; unsigned sets=0; std::vector<uint8_t> frame(hub.sched.frame_bytes());
  for(now_ms=0;now_ms<4001;now_ms++) {
    fx.apply(); output.loop();
    if(hub.sched.pending()) {
      hub.sched.build_frame(int64_t(now_ms)*1000,frame.data());
      assert(!removed(frame[0]));
      if(frame[0]==ULTRA_SET) {
        assert(sets==0 || now_ms-last>=50);
        if(sets==0) {
          assert(ultra_get24(frame.data()+ULTRA_PAYLOAD+22)==first_length);
          if(samples) assert(std::fwrite(frame.data(),ULTRA_SLOT_BYTES,1,samples)==1);
        }
        last=now_ms; sets++;
      }
    }
  }
  assert(sets>0 && sets<=81);
  // Smooth component effects always interpolate for the full sample interval.
  if (first_length == 50) assert(ultra_get24(frame.data()+ULTRA_PAYLOAD+22) == 50);
  std::printf("  %s: %u SETs/4s, transition %u ms, cadence >=50 ms\n",name,sets,first_length);
}
int main(int argc,char **argv) {
  if(argc>1) { samples=std::fopen(argv[1],"wb"); assert(samples); }
  UltraPulseEffect pulse("pulse"); pulse.set_update_interval(1000); pulse.set_transition_on_length(500); pulse.set_transition_off_length(700);
  exercise(pulse,"pulse",700);
  UltraRandomEffect random("random"); random.set_update_interval(1000); random.set_transition_length(750); exercise(random,"random",750);
  UltraFlickerEffect flicker("flicker"); flicker.set_alpha(.95); flicker.set_intensity(.2); exercise(flicker,"flicker",0);
  UltraStrobeEffect strobe("strobe"); light::LightColorValues on, off; off.set_state(false);
  strobe.set_colors({{on,100,0},{off,100,0}}); exercise(strobe,"strobe",0,true);
  UltraCandleEffect candle("candle"); exercise(candle,"candle",50);
  UltraBreatheEffect breathe("breathe"); exercise(breathe,"breathe",50);
  UltraColorLoopEffect loop("loop"); exercise(loop,"colour loop",50);
  UltraSunEffect sunrise("sunrise",true),sunset("sunset",false); exercise(sunrise,"sunrise",50); exercise(sunset,"sunset",50);
  assert(std::fabs(breathe_level(2000,4000,0,1)-1)<.00001);
  assert(breathe_level(0,4000,0,1)==0 && sun_progress(1000,1000,true)==1 && sun_progress(1000,1000,false)==0);
  assert(analog_effect_interval(15,3,1540)==50);
  assert(analog_effect_interval(40,8,11060)==185);
  assert(analog_effect_interval(15,3,1540,10)==100);
  EffectCadence cadence; cadence.sent(0xFFFFFFF0); assert(!cadence.ready(4,250) && cadence.ready(250,250));
  for (auto settings : {std::array<unsigned,4>{15,3,1540,50}, {40,8,11060,185}}) {
    now_ms=0; Hub line; const unsigned n=settings[0], boards=settings[1], interval=settings[3];
    line.sched.init(boards,11,1); line.frame_us=settings[2];
    std::vector<light::LightState> states(n);
    std::vector<std::unique_ptr<UltraLight>> outputs;
    for (unsigned k=0;k<n;k++) {
      outputs.emplace_back(new UltraLight(line,states[k]));
      outputs.back()->board_=k/5; outputs.back()->index_=k%5;
    }
    assert(line.effect_interval()==interval);
    std::vector<uint8_t> f(line.sched.frame_bytes());
    std::vector<unsigned> counts(n); std::vector<uint32_t> last(n);
    for (now_ms=0;now_ms<2000;now_ms++) {
      for (auto &l:outputs) { auto v=l->state_->remote_values; v.set_brightness(float(now_ms+1)/2001); l->send_effect_values(v,interval); l->loop(); }
      if (!line.sched.pending()) continue;
      line.sched.build_frame(int64_t(now_ms)*1000,f.data());
      for (unsigned board=0;board<boards;board++) if((f[board*ULTRA_SLOT_BYTES]&ULTRA_TYPE_MASK)==ULTRA_SET) {
        const unsigned k=board*5+f[board*ULTRA_SLOT_BYTES+1];
        assert(!counts[k] || now_ms-last[k]>=interval);
        assert(ultra_get24(f.data()+board*ULTRA_SLOT_BYTES+ULTRA_PAYLOAD+22)==interval);
        counts[k]++; last[k]=now_ms;
      }
    }
    for(auto count:counts) assert(count==counts[0]);
    assert(counts[0]==(2000+interval-1)/interval);
    std::printf("  %u lights, %u boards: %u ms interval, %u SETs/light/2s, equal share\n",n,boards,interval,counts[0]);
  }
  { // Strip/partition samples share the light's limiter and keep the newest pending pixel.
    now_ms=0; Hub line; light::LightState state; UltraLight output(line,state); output.layers_.push_back(1);
    uint16_t one[5]={65535,65535,65535,65535,65535};
    std::vector<uint8_t> f(line.sched.frame_bytes()); unsigned count=0;
    for(now_ms=0;now_ms<=1000;now_ms++) {
      output.send_pixel(uint16_t(now_ms+1),one,one,16,0,true,0,true); output.loop();
      if (line.sched.pending()) { line.sched.build_frame(int64_t(now_ms)*1000,f.data());
        assert(f[0]==ULTRA_SET && ultra_get24(f.data()+ULTRA_PAYLOAD+22)>=50); count++; }
    }
    assert(count==21 && ultra_get16(f.data()+ULTRA_PAYLOAD)==1001);
    auto under=state.remote_values; under.set_brightness(.3f); output.send_effect_values(under,250);
    assert(output.effect_pending_values_.get_brightness()==.3f);
    // Commissioning is an explicit level command even when it is the top layer: retain its redundancy.
    line.sched.init(1,11,3);
    output.send_pixel(2000,one,one,0,0,false,0);
    unsigned copies=0;
    while(line.sched.pending()) { line.sched.build_frame(int64_t(++now_ms)*1000,f.data()); copies++; }
    assert(copies==3);
    std::puts("  strip/partition pixels: same per-light limit, newest pending sample wins");
  }
  { // Completion is once, including across millis wrap, and preserves sunrise's target / sunset off.
    for (bool rise:{true,false}) {
      now_ms=0xFFFFFF00; Hub line; light::LightState state; UltraLight output(line,state);
      UltraSunEffect sun("sun",rise); sun.set_duration(1000); sun.init(&state); sun.start();
      now_ms+=999; sun.apply(); assert(state.effect==1);
      now_ms+=1; sun.apply(); assert(state.effect==0 && state.current_values.is_on()==rise);
      sun.apply(); assert(state.effect==0);
    }
  }
  Hub hub; light::LightState state; UltraLight out(hub,state); std::vector<uint8_t> frame(hub.sched.frame_bytes());
  out.stored_values_[0].set_brightness(.6); out.stored_values_[1].set_brightness(.2); out.stored_fade_[1]=5;
  out.send_stored_level_(0); out.send_stored_level_(1);
  uint8_t power_slot[ULTRA_SLOT_BYTES], loss_slot[ULTRA_SLOT_BYTES];
  hub.sched.intended_level(0,power_slot); hub.sched.intended_level(1,loss_slot);
  assert(power_slot[2]!=loss_slot[2]);
  // Both intended levels are retained on the host and match the wire-3 contract.
  for(uint8_t selector:{uint8_t(0),uint8_t(1)}) {
    uint8_t intended[ULTRA_SLOT_BYTES]; hub.sched.intended_level(selector,intended);
    assert(intended[ULTRA_PAYLOAD]==selector);
    assert(intended[ULTRA_PAYLOAD+1]==(selector?5:3));
    assert(ultra_get16(intended+ULTRA_PAYLOAD+2)==to16(selector?.2f:.6f));
    if(samples) assert(std::fwrite(intended,ULTRA_SLOT_BYTES,1,samples)==1);
  }
  bool seen[2]{};
  for(int k=0;k<100;k++) { hub.sched.build_frame(k*100000,frame.data()); assert(!removed(frame[0] & ULTRA_TYPE_MASK));
    if((frame[0] & ULTRA_TYPE_MASK)==ULTRA_SCENE) {
      const auto selector=frame[ULTRA_PAYLOAD]; assert(selector<2);
      assert(frame[ULTRA_PAYLOAD+1]==(selector?5:3));
      assert(ultra_get16(frame.data()+ULTRA_PAYLOAD+2)==to16(selector?.2f:.6f)); seen[selector]=true;
    }
  }
  assert(seen[0] && seen[1]);
  // Every retired API, replay, and clearing unused lights must stay silent for all reserved analog opcodes.
  hub.sched.clear_unused_lights(0,1); uint8_t params[20]{};
  hub.sched.replay(0);
  for(int k=0;k<300;k++) {
    hub.sched.build_frame(k*100000,frame.data()); assert(!removed(frame[0]));
    if(frame[0]==ULTRA_SCENE && frame[1]==0 && frame[ULTRA_PAYLOAD]==0)
      assert(ultra_get16(frame.data()+ULTRA_PAYLOAD+2)==to16(.6f));
  }
  if(samples) assert(std::fclose(samples)==0);
  std::puts("ALL ANALOG EFFECT TESTS PASSED");
}
