/*
 LFR PRO V3 -- configurable full PID line-following competition firmware.
 TARGET: STM32F103C8T6 Blue Pill + 14ch analog MUX + dual BTS7960 + SSD1306.
 FIRST BOOT: LFR_ALLOW_RACE_START is 0 in LFR_Config.h (bench-safe).
 After calibration, motor polarity, sensor order and BACK stop are verified,
 explicitly set it to 1, Verify, reflash, and first test with raised wheels.
 NOTE: No encoder, IMU or camera; track semantics cannot be guaranteed.
*/
#include <Arduino.h>
#include <Wire.h>
#include "LFR_Config.h"
#include "LFR_OLED.h"
#include "LFR_Sensor.h"
#include "LFR_Motor.h"
#include "LFR_Control.h"
#include "LFR_Storage.h"
using namespace lfr;
 
static TinyOLED oled;
static Sensor sensor;
static Motors motors;
static Controller controller;
static Settings settings=defaultSettings();
static Calibration calibration={};
static Calibration pendingCalibration={};
static bool haveCalibration=false,scannedWhite=false,scannedBlack=false;
static bool changed=false;

static const uint32_t buttons[4]={LFR_PIN_UP,LFR_PIN_DOWN,LFR_PIN_ENTER,LFR_PIN_BACK};
struct ButtonDebounce {bool raw=false,stable=false;uint32_t changedAt=0;};
static ButtonDebounce keys[4];
static uint8_t menuIndex=0,routeChoice=0,tuneIndex=0;
static uint32_t lastDisplayMs=0,lastMotorTickMs=0,motorStartMs=0;
static uint32_t lastControlUs=0;
#if LFR_RACE_TELEMETRY
static uint32_t lastTelemMs=0;
#endif
static uint8_t motorStage=255;
static bool scanBlack=false;
// Mode editing is transactional: UP/DOWN previews, ENTER applies, BACK cancels.
enum class ModeKind : uint8_t {Speed,Line,Navigation,Gap,Finish,Mirror};
static ModeKind modeKind=ModeKind::Speed;
static uint8_t modeDraft=0;


enum class Page : uint8_t {
  Menu,SensorLive,ScanPrompt,Notice,Route,Tune,Arm,Race,Result,MotorPrompt,MotorTest,ModeSelect,ResetConfirm,Countdown
};
static Page page=Page::Menu;

enum class Chime : uint8_t { Beep, Countdown, Go, Success, Error, Finish };
static void soundChime(Chime c) {
  (void)c;
#if !defined(LFR_UI_TEST_ACCESS) && !defined(UNIT_TEST)
#if defined(LFR_PIN_LED) && (LFR_PIN_LED >= 0)
  pinMode(LFR_PIN_LED, OUTPUT);
  digitalWrite(LFR_PIN_LED, LOW);
#endif
  uint16_t freq = 0;
  uint16_t durMs = 0;
  switch(c) {
    case Chime::Beep:      freq = 1800; durMs = 35; break;
    case Chime::Countdown: freq = 1300; durMs = 60; break;
    case Chime::Go:        freq = 2400; durMs = 180; break;
    case Chime::Success:   freq = 2000; durMs = 100; break;
    case Chime::Error:     freq = 650;  durMs = 200; break;
    case Chime::Finish:    freq = 2200; durMs = 250; break;
  }
#if defined(LFR_PIN_BUZZER) && (LFR_PIN_BUZZER >= 0)
  pinMode(LFR_PIN_BUZZER, OUTPUT);
  if(freq > 0) {
    const uint32_t halfPeriodUs = 500000UL / freq;
    const uint32_t endUs = micros() + uint32_t(durMs) * 1000UL;
    while(micros() < endUs) {
      digitalWrite(LFR_PIN_BUZZER, HIGH);
      delayMicroseconds(halfPeriodUs);
      digitalWrite(LFR_PIN_BUZZER, LOW);
      delayMicroseconds(halfPeriodUs);
    }
  }
#else
  (void)freq;
  delay(durMs);
#endif
#if defined(LFR_PIN_LED) && (LFR_PIN_LED >= 0)
  digitalWrite(LFR_PIN_LED, HIGH);
#endif
#endif
}

static uint32_t raceStartMs = 0;
static uint32_t raceElapsedMs = 0;
static int16_t raceTopSpeed = 0;
static uint32_t raceLaunchUntilMs = 0;
static uint32_t countdownStartMs = 0;
static uint8_t countdownStep = 0;
static const char* const names[]={
  "RACE / ARM", "SENSOR LIVE", "WHITE CAL", "BLACK CAL",
  "SPEED", "PID TUNING", "LINE", "NAV",
  "ROUTE EDIT", "GAP", "FINISH", "MIRROR",
  "MOTOR CHECK", "SAVE SETTINGS", "RESET DEFAULTS"
};
static constexpr uint8_t MENU_ITEMS=sizeof(names)/sizeof(names[0]);
static const char *noticeOne="",*noticeTwo="";
static const char* profileName() {
  switch(settings.speedProfile) {
    case 0:return "SAFE";
    case 1:return "BALANCED";
    case 2:return "FAST";
    case 3:return "SPRINT";
    default:return "TECHNICAL";
  }
}
static const char* navName() {
  return settings.navMode==0?"STRAIGHT":settings.navMode==1?"LEFT":
         settings.navMode==2?"RIGHT":"SCRIPT";
}
// Exact supported settings: do not advertise modes the race engine lacks.
static const char* modeTitle(ModeKind k) {
  switch(k) {
    case ModeKind::Speed:return "SPEED PROFILE";
    case ModeKind::Line:return "LINE POLARITY";
    case ModeKind::Navigation:return "NAVIGATION";
    case ModeKind::Gap:return "GAP POLICY";
    case ModeKind::Finish:return "FINISH MARKER";
    default:return "SENSOR MIRROR";
  }
}
static uint8_t modeCount(ModeKind k) {
  if(k==ModeKind::Speed)return 5;
  if(k==ModeKind::Navigation)return 4;
  return 2;
}
static uint8_t modeActual(ModeKind k) {
  switch(k) {
    case ModeKind::Speed:return settings.speedProfile;
    case ModeKind::Line:return settings.lineMode;
    case ModeKind::Navigation:return settings.navMode;
    case ModeKind::Gap:return settings.gapMode;
    case ModeKind::Finish:return settings.finishOn;
    default:return settings.mirror;
  }
}
static const char* modeLabel(ModeKind k,uint8_t v) {
  switch(k) {
    case ModeKind::Speed:return v==0?"SAFE":v==1?"BALANCED":v==2?"FAST":v==3?"SPRINT":"TECHNICAL";
    case ModeKind::Line:return v==0?"BLACK":"WHITE";
    case ModeKind::Navigation:return v==0?"STRAIGHT":v==1?"LEFT":v==2?"RIGHT":"SCRIPT";
    case ModeKind::Gap:return v==0?"RECOVER":"STOP";
    case ModeKind::Finish:return v==0?"OFF":"ON";
    default:return v==0?"NORMAL":"REVERSED";
  }
}
static const char* menuValue(uint8_t item) {
  switch(item) {
    case 4:return modeLabel(ModeKind::Speed,settings.speedProfile);
    case 6:return modeLabel(ModeKind::Line,settings.lineMode);
    case 7:return modeLabel(ModeKind::Navigation,settings.navMode);
    case 9:return modeLabel(ModeKind::Gap,settings.gapMode);
    case 10:return modeLabel(ModeKind::Finish,settings.finishOn);
    case 11:return modeLabel(ModeKind::Mirror,settings.mirror);
    default:return nullptr;
  }
}
static const char* routeName(uint8_t i) {
  const char* a[]={"STRAIGHT","LEFT","RIGHT","U-TURN","DELETE LAST","CLEAR ALL"};
  return a[i<6?i:0];
}
static bool backIsDown() {return digitalRead(LFR_PIN_BACK)==LOW;}
static uint8_t buttonEvent() {
  uint8_t result=0;
  const uint32_t t=millis();
  for(uint8_t i=0;i<4;++i) {
    const bool down=digitalRead(buttons[i])==LOW;
    if(down!=keys[i].raw) {keys[i].raw=down;keys[i].changedAt=t;}
    if((t-keys[i].changedAt)>=24 && keys[i].stable!=down) {
      keys[i].stable=down;
      if(down && !result) result=uint8_t(i+1);
    }
  }
  return result;
}
// -- LFR PRO V3 Professional OLED UI -----------------------------------------
// 128x64 monochrome: header 0..10, body 13..52, footer 54..63.
// All text below is clipped before drawing; there are no full-width overprints.
static uint8_t uiLength(const char *s) {
  uint8_t n=0;
  if(!s)return 0;
  while(*s++ && n<30)++n;
  return n;
}
static void uiText(uint8_t x,uint8_t y,const char *value,uint8_t limit,bool inverted=false) {
  if(!value || x>=128 || y>=64 || !limit)return;
  const uint8_t space=uint8_t((128U-x)/6U);
  if(limit>space)limit=space;
  if(limit>21)limit=21;
  if(!limit)return;
  char shortText[22];uint8_t n=0;
  while(n<limit && value[n]) {shortText[n]=value[n];++n;}
  if(value[n] && n>=3) {shortText[n-2]='.';shortText[n-1]='.';}
  shortText[n]=0;
  oled.text(x,y,shortText,inverted);
}
static void uiCentered(uint8_t y,const char *value,bool inverted=false) {
  if(!value)return;
  const uint8_t len=uiLength(value);
  const uint8_t shown=len>21?21:len;
  uiText(uint8_t((128U-shown*6U)/2U),y,value,shown,inverted);
}
static void topBar(const char *title,const char *tag=nullptr) {
  oled.clear();
  oled.fill(0,0,128,11,true);
  if(tag && *tag) {
    const uint8_t tagLen=uiLength(tag)>7?7:uiLength(tag);
    const uint8_t tagX=uint8_t(126U-tagLen*6U);
    if(tagX>=5) oled.fill(uint8_t(tagX-5),2,1,7,false);
    uiText(3,2,title,tagX>10?uint8_t((tagX-10)/6):0,true);
    uiText(tagX,2,tag,tagLen,true);
  } else {
    uiText(3,2,title,20,true);
  }
}
static void footer(const char *left,const char *right=nullptr) {
  oled.rule(54);
  if(right && *right) {
    const uint8_t rightLen=uiLength(right)>10?10:uiLength(right);
    const uint8_t x=uint8_t(126U-rightLen*6U);
    if(x>4) uiText(2,56,left,uint8_t((x-4U)/6U));
    uiText(x,56,right,rightLen);
  } else uiText(2,56,left,20);
}
static void notice(const char *msg,const char *second) {
  motors.stop();controller.stop();
  noticeOne=msg;noticeTwo=second;page=Page::Notice;
  topBar("SYSTEM MESSAGE");
  uiCentered(18,noticeOne);
  uiCentered(33,noticeTwo);
  footer("BACK","ENT OK");oled.show();
}
static void drawMenu() {
  page=Page::Menu;motors.stop();
  topBar("LFR PRO V3",haveCalibration?(changed?"CAL!":"CAL"):"NO CAL");
  // A fixed two-digit seven-segment number avoids the letter-like 5x7 digits.
  const uint8_t first=menuIndex>=3?uint8_t(menuIndex-3):0;
  for(uint8_t row=0;row<4;++row) {
    const uint8_t item=uint8_t(first+row);
    if(item>=MENU_ITEMS)break;
    const uint8_t y=uint8_t(14+row*10);
    const bool selected=(item==menuIndex);
    if(selected)oled.fill(0,y,122,9,true);
    const uint8_t textY=uint8_t(y+1);
    // Both digits occupy the same 12-pixel region on every row.
    oled.twoDigits(2,textY,uint8_t(item+1),selected);
    const char *value=menuValue(item);
    if(value) {
      const uint8_t valueLen=uiLength(value);
      const uint8_t valueX=uint8_t(120U-valueLen*6U);
      uiText(23,textY,names[item],uint8_t((valueX>27?valueX-27:0)/6U),selected);
      uiText(valueX,textY,value,valueLen,selected);
    } else {
      uiText(23,textY,names[item],16,selected);
    }
  }
  // One clear scroll indicator at the far edge; never overlaps menu text.
  oled.fill(126,14,1,39,true);
  const uint8_t marker=uint8_t(14U+(uint16_t(menuIndex)*32U)/(MENU_ITEMS-1));
  oled.fill(124,marker,4,7,true);
  footer("UP/DN","ENT OPEN");
  oled.show();
}
static void drawSensor(const SensorFrame &f) {
  topBar("SENSOR LIVE",settings.lineMode==0?"BLACK":"WHITE");
  // Calibrated sensor bar-graph: each channel is independently scaled.
  for(uint8_t i=0;i<SENSOR_COUNT;++i) {
    const uint16_t strength=haveCalibration?f.norm[i]:
      uint16_t(uint32_t(f.raw[i])*1000UL/ADC_MAX);
    const uint8_t height=uint8_t(uint32_t(strength)*26UL/1000UL);
    if(height)oled.fill(uint8_t(2+i*9),uint8_t(41-height),6,height,true);
  }
  oled.rule(42);
  oled.text(2,45,"POS");oled.number(25,45,f.seen?f.pos:0);
  oled.text(79,45,"N");oled.number(91,45,f.active);
  footer("BACK EXIT",haveCalibration?"NORM":"RAW");
  oled.show();
}
static void serialSensor(const SensorFrame &f) {
  Serial1.print("RAW:");
  for(uint8_t i=0;i<SENSOR_COUNT;++i) {
    Serial1.print(f.raw[i]);Serial1.print(i==SENSOR_COUNT-1?'\n':',');
  }
  if(haveCalibration) {
    Serial1.print("NORM:");
    for(uint8_t i=0;i<SENSOR_COUNT;++i) {
      Serial1.print(f.norm[i]);Serial1.print(i==SENSOR_COUNT-1?'\n':',');
    }
    Serial1.print("POS:");Serial1.print(f.pos);
    Serial1.print(" ACTIVE:");Serial1.print(f.active);
    Serial1.print(" JUNC:");Serial1.println(f.junction?1:0);
  }
}
static void drawScan() {
  page=Page::ScanPrompt;
  topBar(scanBlack?"BLACK SCAN":"WHITE SCAN",scanBlack?"2/2":"1/2");
  uiCentered(16,scanBlack?"FULL BLACK PAPER":"FULL WHITE PAPER");
  uiCentered(29,"OR PRESS UP KEY");
  uiCentered(41,"FOR AUTO SWEEP");
  footer("UP AUTO","ENT SCAN");oled.show();
}
static void performAutoSweep() {
  motors.stop();
  noticeOne="SWEEPING 14 CH";
  noticeTwo="WIGGLING ON LINE";
  page=Page::Notice;
  topBar("AUTO CAL SWEEP","RUN");
  uiCentered(18,noticeOne);
  uiCentered(33,noticeTwo);
  footer("BACK ABORT","PLEASE WAIT");oled.show();

  uint16_t minVals[SENSOR_COUNT];
  uint16_t maxVals[SENSOR_COUNT];
  for(uint8_t i=0;i<SENSOR_COUNT;++i) {
    minVals[i]=ADC_MAX;
    maxVals[i]=0;
  }
  const int16_t sweepPWM=55;
  Settings sweepSettings=settings;
  sweepSettings.maxPWM=70;
  sweepSettings.accelStep=10;
  sweepSettings.brakeStep=20;

  const uint32_t startMs=millis();
  while(millis()-startMs<900) {
    if(backIsDown()) {motors.stop();notice("AUTO CAL","ABORTED BY USER");return;}
    const uint32_t elapsed=millis()-startMs;
    if(elapsed<225) {
      motors.drive(Command{int16_t(-sweepPWM),sweepPWM},sweepSettings,true);
    } else if(elapsed<675) {
      motors.drive(Command{sweepPWM,int16_t(-sweepPWM)},sweepSettings,true);
    } else {
      motors.drive(Command{int16_t(-sweepPWM),sweepPWM},sweepSettings,true);
    }
    SensorFrame f={};
    sensor.read(f,calibration,false,settings);
    for(uint8_t i=0;i<SENSOR_COUNT;++i) {
      const uint16_t raw=f.raw[i];
      if(raw<minVals[i]) minVals[i]=raw;
      if(raw>maxVals[i]) maxVals[i]=raw;
    }
    delay(2);
  }
  motors.stop();
  for(uint8_t i=0;i<SENSOR_COUNT;++i) {
    pendingCalibration.white[i]=minVals[i];
    pendingCalibration.black[i]=maxVals[i];
  }
  if(validCalibration(pendingCalibration)) {
    calibration=pendingCalibration;
    haveCalibration=true;
    changed=true;
    sensor.resetTracking();
    sensor.buildNormTable(calibration);
    soundChime(Chime::Success);
    notice("AUTO CAL SUCCESS","14/14 SENSORS OK");
  } else {
    soundChime(Chime::Error);
    notice("AUTO CAL FAILED","CHECK CONTRAST");
  }
}
static void performScan() {
  motors.stop();
  uint16_t captured[SENSOR_COUNT];
  sensor.capture(captured);
  if(scanBlack) {
    for(uint8_t i=0;i<SENSOR_COUNT;++i)pendingCalibration.black[i]=captured[i];
    scannedBlack=true;
  } else {
    for(uint8_t i=0;i<SENSOR_COUNT;++i)pendingCalibration.white[i]=captured[i];
    scannedWhite=true;
  }
  Serial1.print(scanBlack?"BLACK:":"WHITE:");
  for(uint8_t i=0;i<SENSOR_COUNT;++i) {
    Serial1.print(captured[i]);Serial1.print(i==SENSOR_COUNT-1?'\n':',');
  }
  if(scannedWhite && scannedBlack) {
    if(!validCalibration(pendingCalibration)) {
      scannedWhite=scannedBlack=false;
      notice("BAD W/B CONTRAST","REPEAT BOTH SCANS");return;
    }
    calibration=pendingCalibration;
    haveCalibration=true;
    scannedWhite=scannedBlack=false;
    sensor.resetTracking();changed=true;
    sensor.buildNormTable(calibration);
    soundChime(Chime::Success);
    notice("CAL VALID 14/14","USE SAVE SETTINGS");return;
  }
  notice("SURFACE CAPTURED","SCAN OTHER COLOR");
}
static void drawRoute() {
  page=Page::Route;
  topBar("ROUTE EDITOR","SCRIPT");
  oled.text(2,16,"ADD");uiText(29,16,routeName(routeChoice),16);
  oled.text(2,29,"STEPS");oled.number(41,29,settings.routeCount);
  oled.text(84,29,"MAX");oled.number(106,29,ROUTE_CAPACITY);
  oled.text(2,42,"SEQ");
  const char* actions="SLRU";
  const uint8_t first=settings.routeCount>10?uint8_t(settings.routeCount-10):0;
  for(uint8_t i=first;i<settings.routeCount;++i) {
    char one[]={actions[settings.route[i]&3],0};
    oled.text(uint8_t(31+(i-first)*9),42,one);
  }
  footer("UP/DN PICK","ENT APPLY");oled.show();
}
static void drawModeSelect() {
  page=Page::ModeSelect;
  topBar(modeTitle(modeKind),"EDIT");
  oled.text(2,16,"ACTIVE");
  uiText(47,16,modeLabel(modeKind,modeActual(modeKind)),13);
  oled.fill(5,27,118,17,true);
  uiCentered(32,modeLabel(modeKind,modeDraft),true);
  oled.text(2,46,"UP/DN  CHANGE");
  footer("BACK CANCEL","ENT APPLY");
  oled.show();
}
static void openModeSelect(ModeKind kind) {
  modeKind=kind;
  modeDraft=modeActual(kind);
  drawModeSelect();
}
static void changeModeDraft(int8_t delta) {
  const uint8_t n=modeCount(modeKind);
  modeDraft=uint8_t((int(modeDraft)+int(delta)+int(n))%int(n));
  drawModeSelect();
}
static void applyModeDraft() {
  if(modeDraft!=modeActual(modeKind)) {
    switch(modeKind) {
      case ModeKind::Speed:
        // Selecting a preset intentionally loads its associated PWM/PID defaults.
        setProfile(settings,modeDraft);break;
      case ModeKind::Line:
        settings.lineMode=modeDraft;sensor.resetTracking();break;
      case ModeKind::Navigation:settings.navMode=modeDraft;break;
      case ModeKind::Gap:settings.gapMode=modeDraft;break;
      case ModeKind::Finish:settings.finishOn=modeDraft;break;
      case ModeKind::Mirror:
        settings.mirror=modeDraft;sensor.resetTracking();break;
    }
    changed=true;
  }
  drawMenu();
}
static void drawResetConfirm() {
  page=Page::ResetConfirm;
  topBar("RESET SETTINGS","CHECK");
  uiCentered(19,"RESTORE DEFAULTS?");
  uiCentered(34,"CAL WILL REMAIN");
  footer("BACK CANCEL","ENT RESET");
  oled.show();
}
static const char* const tuneFields[]={
  "BASE SPEED", "MAX SPEED", "CURVE SPEED", "TURN SPEED", "KP GAIN",
  "KI GAIN", "KD GAIN", "ACCEL RAMP", "BRAKE RAMP", "JUNCTION TIME"
};
static constexpr uint8_t TUNE_FIELDS=sizeof(tuneFields)/sizeof(tuneFields[0]);
static int tuneValue() {
  switch(tuneIndex) {
    case 0:return settings.basePWM;
    case 1:return settings.maxPWM;
    case 2:return settings.curvePWM;
    case 3:return settings.turnPWM;
    case 4:return settings.kpX10;
    case 5:return settings.kiX100;
    case 6:return settings.kdX100;
    case 7:return settings.accelStep;
    case 8:return settings.brakeStep;
    default:return settings.junctionAdvanceMs;
  }
}
// Small number formatter (integer/one/two decimals); avoids printf in flash.
static uint8_t uiUInt(char *out,uint16_t n) {
  char rev[6];uint8_t used=0;
  do{rev[used++]=char('0'+n%10U);n/=10U;}while(n && used<sizeof(rev));
  uint8_t w=0;while(used)out[w++]=rev[--used];
  out[w]=0;return w;
}
static void uiTuneValue(char out[16]) {
  const uint16_t value=uint16_t(tuneValue());
  const uint8_t decimals=tuneIndex==4?1:((tuneIndex==5 || tuneIndex==6)?2:0);
  if(!decimals){uiUInt(out,value);return;}
  const uint16_t factor=decimals==1?10U:100U;
  const uint8_t n=uiUInt(out,uint16_t(value/factor));
  out[n]='.';
  if(decimals==1) {
    out[n+1]=char('0'+value%10U);out[n+2]=0;
  } else {
    out[n+1]=char('0'+(value%100U)/10U);
    out[n+2]=char('0'+value%10U);out[n+3]=0;
  }
}
static void drawTune() {
  page=Page::Tune;
  // Progress belongs in the header; leave the numeric value an uncluttered area.
  char step[8];
  uint8_t n=uiUInt(step,uint16_t(tuneIndex+1));
  step[n++]='/';
  uiUInt(step+n,TUNE_FIELDS);
  topBar("PID / SPEED",step);
  uiCentered(16,tuneFields[tuneIndex]);
  oled.fill(16,28,96,20,true);
  char value[16];uiTuneValue(value);
  const uint8_t chars=uiLength(value)>9?9:uiLength(value);
  const uint8_t x=uint8_t((128U-chars*12U)/2U);
  oled.textBig(x,30,value,true);
  footer("UP/DN + -","ENT NEXT");oled.show();
}
static void modifyTune(int direction) {
  const int step=(tuneIndex>=4 && tuneIndex<=6)?(tuneIndex==4?5:5):
                 (tuneIndex>=7?1:5);
  const int value=tuneValue()+direction*step;
  switch(tuneIndex) {
    case 0:
      settings.basePWM=uint8_t(constrain(value,50,int(settings.maxPWM)));
      if(settings.curvePWM>settings.basePWM)settings.curvePWM=settings.basePWM;
      break;
    case 1:settings.maxPWM=uint8_t(constrain(value,int(settings.basePWM),220));break;
    case 2:settings.curvePWM=uint8_t(constrain(value,35,int(settings.basePWM)));break;
    case 3:settings.turnPWM=uint8_t(constrain(value,45,155));break;
    case 4:settings.kpX10=uint16_t(constrain(value,20,350));break;
    case 5:settings.kiX100=uint16_t(constrain(value,0,600));break;
    case 6:settings.kdX100=uint16_t(constrain(value,0,120));break;
    case 7:settings.accelStep=uint8_t(constrain(value,2,30));break;
    case 8:settings.brakeStep=uint8_t(constrain(value,8,55));break;
    default:settings.junctionAdvanceMs=uint8_t(constrain(value,0,110));break;
  }
  changed=true;drawTune();
}
static void drawArm() {
  page=Page::Arm;
  topBar("RACE ARM",haveCalibration?"READY":"NO CAL");
  oled.text(2,16,"CAL");oled.text(33,16,haveCalibration?"YES":"NO");
  oled.text(72,16,"LINE");oled.text(102,16,settings.lineMode==0?"BLK":"WHT");
  oled.text(2,28,"NAV");uiText(34,28,navName(),15);
  oled.text(2,40,"SPEED");uiText(42,40,profileName(),14);
  footer("BACK","ENT START");oled.show();
}
static void drawMotorPrompt() {
  page=Page::MotorPrompt;
  topBar("MOTOR CHECK","SAFE");
  uiCentered(16,"LIFT BOTH WHEELS");
  uiCentered(29,"L+ R+ L- R-");
  uiCentered(41,"LOW SPEED / 2.4S");
  footer("BACK","ENT TEST");oled.show();
}
static void motorBenchStart() {
  motors.stop();motorStage=255;
  motorStartMs=millis();lastMotorTickMs=0;
  page=Page::MotorTest;
  topBar("MOTOR CHECK","RUN");
  uiCentered(19,"TEST IN PROGRESS");
  uiCentered(36,"BACK = STOP");
  footer("WHEELS OFF","RUNNING");oled.show();
}
static void drawCountdown(uint8_t count) {
  page=Page::Countdown;
  topBar("RACE LAUNCH","READY");
  char num[4]={char('0'+count),0};
  oled.textBig(58,22,num,true);
  uiCentered(46,"STAND CLEAR");
  footer("BACK CANCEL","COUNTDOWN");oled.show();
}
static void finishRace(const char *reason) {
  motors.stop();controller.stop();page=Page::Result;
  raceElapsedMs=millis()-raceStartMs;
  soundChime(Chime::Finish);
  topBar("RACE RESULT",reason);

  const uint16_t sec=uint16_t(raceElapsedMs/1000U);
  const uint16_t ms10=uint16_t((raceElapsedMs%1000U)/10U);
  char timeBuf[12];
  timeBuf[0]=char('0'+(sec/10U)%10U);
  timeBuf[1]=char('0'+sec%10U);
  timeBuf[2]='.';
  timeBuf[3]=char('0'+(ms10/10U)%10U);
  timeBuf[4]=char('0'+ms10%10U);
  timeBuf[5]='s';
  timeBuf[6]=0;

  oled.text(2,16,"TIME");
  oled.textBig(36,15,timeBuf,true);
  oled.text(2,34,"TOP PWM");
  oled.number(55,34,raceTopSpeed);
  oled.text(2,45,"JUNCTIONS");
  oled.number(65,45,controller.junctionCount());
  footer("BACK MENU","ENT OK");oled.show();
  Serial1.print("RACE TIME:");Serial1.print(timeBuf);
  Serial1.print(" TOP_PWM:");Serial1.print(raceTopSpeed);
  Serial1.print(" STOP:");Serial1.println(reason);
}
static void raceStart() {
  motors.stop();
#if LFR_ALLOW_RACE_START == 0
  notice("RACE LOCKED","SET FLAG IN CONFIG");return;
#else
  if(!haveCalibration) {notice("NO CALIBRATION","SCAN WHITE/BLACK");return;}
  if(!saneSettings(settings)) {notice("BAD SETTINGS","REVIEW TUNING");return;}
  if(settings.navMode==uint8_t(NavMode::Script) && !settings.routeCount) {
    notice("EMPTY ROUTE","ADD ROUTE STEPS");return;
  }
  SensorFrame f={};sensor.resetTracking();sensor.read(f,calibration,true,settings);
  if(!f.seen || (!f.fullMarker && (f.pos<-4200 || f.pos>4200))) {
    notice("START LINE INVALID","CENTER ON LINE");return;
  }
  page=Page::Race;
  raceStartMs=millis();
  raceLaunchUntilMs=raceStartMs+140; // 140ms anti-slip launch cap
  raceTopSpeed=0;
  topBar("RACE ACTIVE",profileName());
  uiCentered(17,"FULL PID RUNNING");
  uiCentered(31,navName());
  uiCentered(43,"BACK = E-STOP");
  footer("POWER = STOP","RACING");oled.show();
  controller.start(millis(),f.pos);
  lastControlUs=micros();
#if LFR_RACE_TELEMETRY
  lastTelemMs=millis();
#endif
#endif
}
static void armLaunch() {
  motors.stop();
#if LFR_ALLOW_RACE_START == 0
  notice("RACE LOCKED","SET FLAG IN CONFIG");return;
#else
  if(!haveCalibration) {notice("NO CALIBRATION","SCAN WHITE/BLACK");return;}
  if(!saneSettings(settings)) {notice("BAD SETTINGS","REVIEW TUNING");return;}
  if(settings.navMode==uint8_t(NavMode::Script) && !settings.routeCount) {
    notice("EMPTY ROUTE","ADD ROUTE STEPS");return;
  }
  SensorFrame f={};sensor.resetTracking();sensor.read(f,calibration,true,settings);
  if(!f.seen || (!f.fullMarker && (f.pos<-4200 || f.pos>4200))) {
    notice("START LINE INVALID","CENTER ON LINE");return;
  }
  page=Page::Countdown;
  countdownStartMs=millis();
  countdownStep=3;
  drawCountdown(3);
  soundChime(Chime::Countdown);
#endif
}
static void enterMenu() {
  switch(menuIndex) {
    case 0:drawArm();break;
    case 1:page=Page::SensorLive;lastDisplayMs=0;break;
    case 2:scanBlack=false;drawScan();break;
    case 3:scanBlack=true;drawScan();break;
    case 4:openModeSelect(ModeKind::Speed);break;
    case 5:tuneIndex=0;drawTune();break;
    case 6:openModeSelect(ModeKind::Line);break;
    case 7:openModeSelect(ModeKind::Navigation);break;
    case 8:routeChoice=0;drawRoute();break;
    case 9:openModeSelect(ModeKind::Gap);break;
    case 10:openModeSelect(ModeKind::Finish);break;
    case 11:openModeSelect(ModeKind::Mirror);break;
    case 12:drawMotorPrompt();break;
    case 13:
      motors.stop();
      if(saveSettings(settings,calibration,haveCalibration)) {
        changed=false;notice("SAVED TO FLASH","REBOOT TO VERIFY");
      } else notice("SAVE FAILED","CHECK EEPROM SIZE");
      break;
    case 14:drawResetConfirm();break;
  }
}
static void handleButton(uint8_t event) {
  if(!event)return;
  if(page==Page::Race) {
    if(event==4) finishRace("BACK E-STOP");
    return;
  }
  if(page==Page::MotorTest) {
    if(event==4) {motors.stop();drawMenu();}return;
  }
  if(page==Page::Menu) {
    if(event==1)menuIndex=uint8_t((menuIndex+MENU_ITEMS-1)%MENU_ITEMS);
    if(event==2)menuIndex=uint8_t((menuIndex+1)%MENU_ITEMS);
    if(event==3)enterMenu();
    else if(event==1 || event==2)drawMenu();
    return;
  }
  if(event==4) {motors.stop();drawMenu();return;}
  if(page==Page::Notice || page==Page::Result) {
    if(event==3) drawMenu();
    return;
  }
  if(page==Page::ModeSelect) {
    if(event==1)changeModeDraft(-1);
    if(event==2)changeModeDraft(1);
    if(event==3)applyModeDraft();
    return;
  }
  if(page==Page::ResetConfirm) {
    if(event==3) {
      settings=defaultSettings();changed=true;
      notice("DEFAULTS LOADED","USE SAVE SETTINGS");
    }
    return;
  }
  if(page==Page::Arm) {if(event==3)armLaunch();return;}
  if(page==Page::Countdown) {
    if(event==3) { soundChime(Chime::Go); raceStart(); }
    else if(event==4) { motors.stop(); drawArm(); }
    return;
  }
  if(page==Page::ScanPrompt) {
    if(event==1) performAutoSweep();
    else if(event==3) performScan();
    return;
  }
  if(page==Page::MotorPrompt) {if(event==3)motorBenchStart();return;}
  if(page==Page::Tune) {
    if(event==1)modifyTune(1);
    if(event==2)modifyTune(-1);
    if(event==3) {tuneIndex=uint8_t((tuneIndex+1)%TUNE_FIELDS);drawTune();}
    return;
  }
  if(page==Page::Route) {
    if(event==1)routeChoice=uint8_t((routeChoice+5)%6);
    if(event==2)routeChoice=uint8_t((routeChoice+1)%6);
    if(event==3) {
      if(routeChoice<4 && settings.routeCount<ROUTE_CAPACITY) {
        settings.route[settings.routeCount++]=routeChoice;changed=true;
      } else if(routeChoice==4 && settings.routeCount) {
        --settings.routeCount;changed=true;
      } else if(routeChoice==5) {settings.routeCount=0;changed=true;}
    }
    drawRoute();return;
  }
}
void setup() {
  // MOTOR OFF FIRST: fail-safe on boot, no LED/buzzer drives on shared GPIO.
  motors.begin();
  Serial1.begin(115200);
  delay(30);
  // PB3 BACK pin requires JTAG disabled, retain SWD programming.
#if defined(RCC) && defined(RCC_APB2ENR_AFIOEN)
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
#elif defined(RCC)
  RCC->APB2ENR |= (1U << 0);
#endif
#if defined(AFIO) && defined(AFIO_MAPR_SWJ_CFG_JTAGDISABLE)
  #if defined(AFIO_MAPR_SWJ_CFG)
    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  #else
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  #endif
#endif
  for(uint8_t i=0;i<4;++i)pinMode(buttons[i],INPUT_PULLUP);
  sensor.begin();
  Wire.begin();
  if(!oled.begin()) {
    Serial1.println("FATAL: SSD1306 0x3C NOT FOUND; MOTORS OFF");
    while(true) {motors.stop();delay(60);}
  }
  bool migrated=false;
  if(!loadSettings(settings,calibration,haveCalibration,migrated)) {
    settings=defaultSettings();haveCalibration=false;
    Serial1.println("V3: NEW SETTINGS, CALIBRATION REQUIRED");
  } else if(migrated) {
    Serial1.println("V3: LEGACY V1/V1.1 CAL LOADED - USE SAVE SETTINGS");
  } else {
    Serial1.println("V3: VERIFIED EEPROM DUAL-SLOT SETTINGS");
  }
  pendingCalibration=calibration;
  if(haveCalibration) sensor.buildNormTable(calibration);
  if(!saneSettings(settings)) {settings=defaultSettings();haveCalibration=false;}
  Serial1.println("LFR PRO V3 - FULL PID - SERIAL1 115200");
#if LFR_ALLOW_RACE_START == 0
  Serial1.println("BENCH MODE: RACE LOCKED");
#else
  Serial1.println("WARNING: RACE ENABLED");
#endif
  drawMenu();
}
void loop() {
  // Independent raw BACK polling. This runs before every motor update.
  if((page==Page::Race || page==Page::MotorTest || page==Page::Countdown) && backIsDown()) {
    motors.stop();
    if(page==Page::Race)finishRace("BACK E-STOP");
    else if(page==Page::Countdown)drawArm();
    else drawMenu();
    return;
  }
  const uint8_t event=buttonEvent();
  handleButton(event);
  const uint32_t nowMs=millis();

  if(page==Page::Countdown) {
    const uint32_t elapsed=nowMs-countdownStartMs;
    if(elapsed<500) {
      if(countdownStep!=3) {countdownStep=3;drawCountdown(3);soundChime(Chime::Countdown);}
    } else if(elapsed<1000) {
      if(countdownStep!=2) {countdownStep=2;drawCountdown(2);soundChime(Chime::Countdown);}
    } else if(elapsed<1500) {
      if(countdownStep!=1) {countdownStep=1;drawCountdown(1);soundChime(Chime::Countdown);}
    } else {
      soundChime(Chime::Go);
      raceStart();
    }
    return;
  }

  if(page==Page::MotorTest) {
    if(nowMs-lastMotorTickMs>=5) {
      lastMotorTickMs=nowMs;
      const uint32_t elapsed=nowMs-motorStartMs;
      const uint8_t stage=uint8_t(elapsed/600UL);
      if(stage>=4) {motors.stop();notice("MOTOR TEST DONE","CONFIRM DIRECTIONS");return;}
      if(stage!=motorStage) {motors.stop();motorStage=stage;}
      if(elapsed%600UL>=420) {motors.stop();return;}
      Settings limited=settings;
      limited.maxPWM=90;limited.accelStep=10;
      if(stage==0)motors.drive(Command{65,0},limited,true);
      else if(stage==1)motors.drive(Command{0,65},limited,true);
      else if(stage==2)motors.drive(Command{-65,0},limited,true);
      else motors.drive(Command{0,-65},limited,true);
    }
    return;
  }

  if(page==Page::SensorLive) {
    if(nowMs-lastDisplayMs>=170) {
      lastDisplayMs=nowMs;
      SensorFrame f={};sensor.read(f,calibration,haveCalibration,settings);
      drawSensor(f);serialSensor(f);
    }
    return;
  }

  if(page==Page::Race) {
    const uint32_t tUs=micros();
    const uint32_t interval=tUs-lastControlUs;
    if(interval>=CTRL_PERIOD_US) {
      if(interval>CTRL_WATCHDOG_US) {finishRace("LOOP WATCHDOG");return;}
      lastControlUs=tUs;
      SensorFrame frame={};sensor.read(frame,calibration,true,settings);
      if(backIsDown()) {finishRace("BACK E-STOP");return;}
      Command command=controller.tick(frame,settings,millis(),float(interval)*0.000001f);
      if(!controller.active()) {
        finishRace(controller.state()==DriveState::Finished?"FINISH MARKER":"TRACK / ROUTE FAULT");
        return;
      }
      if(command.left>raceTopSpeed) raceTopSpeed=command.left;
      if(command.right>raceTopSpeed) raceTopSpeed=command.right;
      if(nowMs<raceLaunchUntilMs) {
        // Anti-slip launch cap for first 140ms
        const int16_t launchCap=int16_t((uint16_t(settings.basePWM)*75U)/100U);
        command.left=clamp16(command.left,-launchCap,launchCap);
        command.right=clamp16(command.right,-launchCap,launchCap);
      }
      motors.drive(command,settings,true);
#if LFR_RACE_TELEMETRY
      if(millis()-lastTelemMs>=50) {
        lastTelemMs=millis();
        Serial1.print("P:");Serial1.print(frame.pos);
        Serial1.print(" L:");Serial1.print(command.left);
        Serial1.print(" R:");Serial1.print(command.right);
        Serial1.print(" N:");Serial1.println(frame.active);
      }
#endif
    }
  }
}
