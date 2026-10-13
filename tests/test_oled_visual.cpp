#include <cassert>
#include <cstdio>
#include <cstring>
#include "Arduino.h"
#include "EEPROM.h"
#define LFR_UI_TEST_ACCESS 1
#include "../LFR_PRO_V3/LFR_PRO_V3.ino"
static void saveScreen(const char* name) {
  char path[256];
  std::snprintf(path,sizeof(path),"previews/%s.pgm",name);
  FILE *f=std::fopen(path,"wb");
  assert(f);
  std::fprintf(f,"P5\n128 64\n255\n");
  for(unsigned y=0;y<64;++y)
    for(unsigned x=0;x<128;++x) {
      const unsigned char pix=oled.testPixel(uint8_t(x),uint8_t(y))?255:0;
      std::fwrite(&pix,1,1,f);
    }
  std::fclose(f);
}
int main() {
  for(int i=0;i<64;++i) mockDigital[i]=HIGH;
  for(int i=0;i<16;++i) mockMuxADC[i]=160;
  setup();
  saveScreen("01_menu_home");
  menuIndex=14;drawMenu();saveScreen("02_menu_scroll");
  haveCalibration=true;menuIndex=1;drawMenu();saveScreen("03_menu_cal");
  SensorFrame sf={};sf.seen=true;sf.pos=-320;sf.active=5;
  for(uint8_t i=0;i<SENSOR_COUNT;++i)sf.norm[i]=uint16_t((i*137)%1001);
  drawSensor(sf);saveScreen("04_sensor_live");
  tuneIndex=4;drawTune();saveScreen("05_pid_kp");
  tuneIndex=5;drawTune();saveScreen("06_pid_ki");
  tuneIndex=9;drawTune();saveScreen("07_junction_time");
  scanBlack=true;drawScan();saveScreen("08_scan_black");
  settings.routeCount=4;settings.route[0]=0;settings.route[1]=1;
  settings.route[2]=2;settings.route[3]=3;
  routeChoice=4;drawRoute();saveScreen("09_route_editor");
  drawArm();saveScreen("10_arm_race");
  drawMotorPrompt();saveScreen("11_motor_prompt");
  notice("BAD W/B CONTRAST","REPEAT BOTH SCANS");saveScreen("12_notice");
  finishRace("TRACK / ROUTE FAULT");saveScreen("13_race_result");
  settings.speedProfile=1;
  menuIndex=4;drawMenu();saveScreen("14_menu_speed_balanced");
  enterMenu();saveScreen("15_speed_mode_editor");
  handleButton(2);saveScreen("16_speed_mode_preview");
  handleButton(4);saveScreen("17_speed_mode_cancelled");
  menuIndex=7;drawMenu();enterMenu();saveScreen("18_nav_mode_editor");
  menuIndex=9;drawMenu();enterMenu();saveScreen("19_gap_mode_editor");
  menuIndex=11;drawMenu();enterMenu();saveScreen("20_mirror_mode_editor");
  drawResetConfirm();saveScreen("21_reset_confirmation");
  puts("PASS professional UI snapshot screens (21 views)");
}
