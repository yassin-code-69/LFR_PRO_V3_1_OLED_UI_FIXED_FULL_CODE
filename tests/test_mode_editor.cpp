#include <cassert>
#include <cstdio>
#include <cstring>
#include "Arduino.h"
#include "EEPROM.h"
#define LFR_UI_TEST_ACCESS 1
#include "../LFR_PRO_V3/LFR_PRO_V3.ino"

static void assertOff() {
  assert(mockPwm[PA0]==0 && mockPwm[PA1]==0 && mockPwm[PB8]==0 && mockPwm[PB9]==0);
}
static void testMode(uint8_t index,uint8_t oldVal,uint8_t newVal) {
  menuIndex=index;drawMenu();
  enterMenu();
  assert(page==Page::ModeSelect);
  assert(modeDraft==oldVal);
  // DOWN moves one option; nothing commits until ENTER.
  handleButton(2);
  assert(modeDraft==newVal);
  assert(modeActual(modeKind)==oldVal);
  handleButton(4);
  assert(page==Page::Menu);
  assert(modeActual(modeKind)==oldVal);
  // Open again and commit.
  enterMenu();handleButton(2);handleButton(3);
  assert(page==Page::Menu);
  assert(modeActual(modeKind)==newVal);
  assert(changed);
  assertOff();
}
static unsigned long pixelSignature(TinyOLED &display) {
  unsigned long hash=2166136261UL;
  for(uint8_t y=0;y<8;++y)
    for(uint8_t x=0;x<6;++x) {
      hash ^= display.testPixel(x,y) ? 1U : 0U;
      hash *= 16777619UL;
    }
  return hash;
}
int main() {
  for(int i=0;i<64;++i)mockDigital[i]=HIGH;
  for(int i=0;i<16;++i)mockMuxADC[i]=160;
  setup();
  assertOff();
  // All ten numeric glyphs have unique pixel patterns; notable 1 vs I tested.
  unsigned long sig[10];
  for(uint8_t digit=0;digit<10;++digit) {
    oled.clear();char one[2]={char('0'+digit),0};oled.text(0,0,one);
    sig[digit]=pixelSignature(oled);
    for(uint8_t j=0;j<digit;++j) assert(sig[j]!=sig[digit]);
  }
  oled.clear();oled.text(0,0,"I");
  assert(pixelSignature(oled)!=sig[1]);
  // Mode change is staged and can be cancelled. Full PID is untouched.
  testMode(4,0,1); // speed SAFE -> BALANCED
  assert(settings.kpX10==140 && settings.kiX100==90 && settings.kdX100==23);
  testMode(6,0,1); // line BLACK -> WHITE
  testMode(7,0,1); // junction STRAIGHT -> LEFT
  testMode(9,0,1); // gap RECOVER -> STOP
  testMode(10,0,1); // finish OFF -> ON
  testMode(11,0,1); // mirror NORMAL -> REVERSED
  // Defaults now require explicit second ENTER; BACK cancels.
  menuIndex=14;drawMenu();enterMenu();
  assert(page==Page::ResetConfirm);
  handleButton(4);assert(page==Page::Menu && settings.lineMode==1);
  enterMenu();handleButton(3);
  assert(page==Page::Notice);
  assert(settings.lineMode==0 && settings.navMode==0 && settings.speedProfile==0);
  assertOff();
  puts("PASS readable 0-9, mode CURRENT/PREVIEW/COMMIT/CANCEL, 6 modes, reset confirm");
}
