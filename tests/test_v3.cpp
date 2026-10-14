#include <cassert>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstddef>
#include "Arduino.h"
#include "EEPROM.h"
#include "LFR_Config.h"
#include "LFR_Sensor.h"
#include "LFR_Control.h"
#include "LFR_Motor.h"
#include "LFR_PID.h"
#include "LFR_Storage.h"
#include "LFR_OLED.h"
using namespace lfr;
static Calibration calibration() {
  Calibration c={};
  for(int i=0;i<SENSOR_COUNT;++i){c.white[i]=160;c.black[i]=3000;}
  return c;
}
static SensorFrame frame(int pos,bool seen=true) {
  SensorFrame f={};f.pos=pos;f.seen=seen;f.confidence=850;
  f.active=seen?2:0;f.center=seen && pos>-1700 && pos<1700;
  f.leftEdge=seen&&pos<-4450;f.rightEdge=seen&&pos>4450;
  return f;
}
static void testPID() {
  Settings s=defaultSettings();assert(s.kiX100>0);
  PID p;p.reset(1.0f);
  float v0=p.update(1,0.005,s,110,true);
  float v=v0;
  for(int i=0;i<300;++i)v=p.update(1,0.005,s,110,true);
  assert(v>v0+0.25f && p.integral()>0.2f);
  for(int i=0;i<2000;++i)p.update(6.5f,0.005f,s,35.f,false);
  assert(fabsf(p.integral())<=5.01f);
  assert(fabsf(p.update(6,0.005f,s,35,true))<=35.01f);
  printf("PASS real integral, saturation, PID anti-windup\n");
}
static void testDirection() {
  Settings s=defaultSettings();
  Controller c;c.start(10,0);
  Command a=c.tick(frame(-2300),s,15,.005f);
  Command b=c.tick(frame(2300),s,20,.005f);
  assert(a.left<a.right);
  assert(b.left>b.right);
  assert(a.left>=0 && b.right>=0);
  printf("PASS differential steering physical sign\n");
}
static void testCorners() {
  Settings s=defaultSettings();
  Controller c;c.start(0,0);
  SensorFrame left=frame(-5900);
  c.tick(left,s,5,.005f);
  Command turn=c.tick(left,s,10,.005f);
  assert(c.state()==DriveState::Pivot);
  assert(turn.left<0 && turn.right>0);
  // Reacquire centered after minimum period and stable 3 samples.
  SensorFrame center=frame(0);
  for(uint32_t now=15;now<160;now+=5) c.tick(center,s,now,.005f);
  assert(c.state()==DriveState::Follow);
  c.start(0,0);
  SensorFrame right=frame(5700);
  c.tick(right,s,5,.005f);
  turn=c.tick(right,s,10,.005f);
  assert(c.state()==DriveState::Pivot && turn.left>0 && turn.right<0);
  c.tick(frame(0,false),s,930,.005f);
  assert(c.state()==DriveState::Fault);
  printf("PASS left/right corner pivot, reacquisition, timeout\n");
}
static void testGap() {
  Settings s=defaultSettings();Controller c;
  c.start(0,0);
  // Straight gap test: symmetric coast
  c.tick(frame(0),s,2,.005f);
  Command straightGap=c.tick(frame(0,false),s,5,.005f);
  assert(c.state()==DriveState::Gap);
  assert(straightGap.left == straightGap.right);
  for(uint32_t t=10;t<200;t+=5)c.tick(frame(0,false),s,t,.005f);
  assert(c.state()==DriveState::Fault);

  // Curved gap test: arc coasting following previous turn direction
  c.start(0,2500);
  c.tick(frame(2500),s,2,.005f);
  c.tick(frame(0,false),s,5,.005f);
  assert(c.state()==DriveState::Gap);
  Command curveGap=c.tick(frame(0,false),s,10,.005f);
  assert(curveGap.left > curveGap.right);

  s.gapMode=uint8_t(GapMode::Stop);c.start(0,0);
  c.tick(frame(0,false),s,5,.005f);
  c.tick(frame(0,false),s,65,.005f);
  assert(c.state()==DriveState::Fault);
  printf("PASS short gap safety, STOP policy, and predictive curve arc coasting\n");
}
static void testScript() {
  Settings s=defaultSettings();s.navMode=3;
  s.routeCount=1;s.route[0]=uint8_t(RouteStep::Left);
  Controller c;c.start(0,0);
  SensorFrame j=frame(0);j.junction=true;j.active=7;
  c.tick(j,s,5,.005f);c.tick(j,s,10,.005f);
  c.tick(j,s,15,.005f);
  assert(c.state()==DriveState::JunctionAdvance);
  assert(c.junctionCount()==1 && c.routeUsed()==1);
  auto cmd=c.tick(j,s,55,.005f);
  assert(c.state()==DriveState::Pivot && cmd.left<0);
  printf("PASS scripted junction route and delayed pivot\n");
}
static void testSensor() {
  for(int i=0;i<16;++i)mockMuxADC[i]=160;
  Sensor sensor;sensor.begin();
  Settings s=defaultSettings();Calibration cal=calibration();
  SensorFrame f={};
  mockMuxADC[6]=2900;mockMuxADC[7]=3000;
  sensor.read(f,cal,true,s);
  assert(f.seen && f.center && abs(f.pos)<650);
  sensor.resetTracking();
  for(int i=0;i<16;++i)mockMuxADC[i]=160;
  mockMuxADC[0]=2950;mockMuxADC[1]=2950;
  sensor.read(f,cal,true,s);assert(f.seen && f.pos<-5000);
  sensor.resetTracking();
  for(int i=0;i<16;++i)mockMuxADC[i]=160;
  mockMuxADC[12]=2950;mockMuxADC[13]=2950;
  sensor.read(f,cal,true,s);assert(f.seen && f.pos>5000);
  sensor.resetTracking();
  for(int i=0;i<16;++i)mockMuxADC[i]=160;
  s.lineMode=1;
  for(int i=0;i<16;++i)mockMuxADC[i]=3000;
  mockMuxADC[6]=160;mockMuxADC[7]=160;
  sensor.read(f,cal,true,s);
  assert(f.seen && f.center);
  printf("PASS sensor position + black/white line inversion\n");
}
static void testMotor() {
  Settings s=defaultSettings();Motors motor;motor.begin();
  for(int i=0;i<20;++i)motor.drive(Command{100,100},s,false);
  assert(motor.left()>0 && mockPwm[PB8]>0);
  for(int i=0;i<30;++i)motor.drive(Command{-100,0},s,true);
  assert(motor.left()<=0);
  assert(!(mockPwm[PB8]>0 && mockPwm[PB9]>0));
  motor.stop();assert(motor.left()==0&&motor.right()==0);
  printf("PASS motor direction braking and safe reversal\n");
}
static void testStorage() {
  Settings cfg=defaultSettings();Calibration cal=calibration();
  assert(saveSettings(cfg,cal,true));
  Settings out={};Calibration restored={};bool valid=false,migrated=true;
  assert(loadSettings(out,restored,valid,migrated));
  assert(valid && !migrated && out.kiX100==cfg.kiX100);
  cfg.kiX100=255;
  assert(saveSettings(cfg,cal,true));
  assert(loadSettings(out,restored,valid,migrated) && out.kiX100==255);
  // corrupt newest copy B. Backup slot A should still be readable.
  EEPROM.data[768]^=0x55;
  assert(loadSettings(out,restored,valid,migrated));
  assert(out.kiX100==120);
  printf("PASS CRC EEPROM dual-slot save and fallback\n");
}

struct LegacySettingsTest {
  uint8_t lineMode,navMode,gapMode,speedProfile,mirror,finishOn,routeCount;
  uint8_t route[16];
  uint8_t basePWM,maxPWM,curvePWM,turnPWM;
  uint16_t kpX10,kdX100;
};
struct LegacyStoreTest {
  uint32_t magic;
  uint16_t version;
  uint16_t hasCalibration;
  LegacySettingsTest settings;
  Calibration cal;
  uint16_t crc;
};
static uint16_t legacyCRC(const uint8_t* data,size_t length) {
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<length;++i) {
    crc^=uint16_t(data[i]);
    for(int j=0;j<8;++j)crc=crc&1?uint16_t((crc>>1)^0xA001):uint16_t(crc>>1);
  }
  return crc;
}
static void testLegacyMigration() {
  std::memset(EEPROM.data,0,sizeof(EEPROM.data));
  LegacyStoreTest previous={};
  previous.magic=0x4C465232UL;previous.version=1;previous.hasCalibration=1;
  previous.settings.lineMode=0;previous.settings.navMode=3;
  previous.settings.gapMode=0;previous.settings.speedProfile=0;
  previous.settings.mirror=0;previous.settings.finishOn=0;
  previous.settings.routeCount=2;
  previous.settings.route[0]=1;previous.settings.route[1]=2;
  previous.settings.basePWM=65;previous.settings.maxPWM=95;
  previous.settings.curvePWM=45;previous.settings.turnPWM=72;
  previous.settings.kpX10=120;previous.settings.kdX100=14;
  previous.cal=calibration();
  previous.crc=legacyCRC(reinterpret_cast<const uint8_t*>(&previous),
                         offsetof(LegacyStoreTest,crc));
  std::memcpy(&EEPROM.data[256],&previous,sizeof(previous));
  Settings cfg={};Calibration cal={};bool valid=false,migrated=false;
  assert(loadSettings(cfg,cal,valid,migrated));
  assert(migrated && valid && cfg.navMode==3 && cfg.routeCount==2 && cfg.kiX100>0);
  assert(cfg.route[0]==1 && cfg.route[1]==2 && cal.black[7]==3000);
  assert(saveSettings(cfg,cal,valid));
  assert(loadSettings(cfg,cal,valid,migrated) && !migrated && valid);
  printf("PASS V1/V1.1 CRC-verified calibration migration and V3 upgrade\n");
}

static void testOLED() {
  TinyOLED display;assert(display.begin());
  display.text(0,0,"PID V3 READY");display.number(0,16,-6500);
  display.fill(1,32,126,2);display.show();
  printf("PASS tiny SSD1306 init and drawing calls\n");
}

static void testFinishMarker() {
  Settings s=defaultSettings();
  s.finishOn=1;
  Controller c;c.start(0,0);
  SensorFrame normal=frame(0);
  for(uint32_t t=5;t<=2500;t+=5) c.tick(normal,s,t,0.005f);
  assert(c.state()==DriveState::Follow);
  SensorFrame m=frame(0);m.fullMarker=true;m.active=14;
  c.tick(m,s,2505,0.005f);
  assert(c.state()==DriveState::Marker);
  c.tick(m,s,2510,0.005f);
  c.tick(m,s,2515,0.005f);
  Command stop=c.tick(m,s,2520,0.005f);
  assert(c.state()==DriveState::Finished);
  assert(stop.left==0 && stop.right==0);
  printf("PASS high-speed finish marker detection and stop\n");
}

int main() {
  for(int i=0;i<64;++i)mockDigital[i]=HIGH;
  testPID();testDirection();testCorners();testGap();testScript();
  testSensor();testMotor();testStorage();testLegacyMigration();testOLED();
  testFinishMarker();
  puts("ALL V3 HOST TESTS PASSED");
  return 0;
}
