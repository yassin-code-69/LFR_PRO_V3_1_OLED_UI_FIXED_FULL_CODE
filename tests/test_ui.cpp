#include <cassert>
#include <cstdio>
#include "Arduino.h"
#include "EEPROM.h"
extern void setup();
extern void loop();
static void advance(int ms) {mockMicro+=uint64_t(ms)*1000ULL;loop();}
static void press(int pin) {
  mockDigital[pin]=LOW;loop();advance(27);
  mockDigital[pin]=HIGH;loop();advance(27);
}
static bool allMotorOff() {
  return mockPwm[PA0]==0 && mockPwm[PA1]==0 &&
         mockPwm[PB8]==0 && mockPwm[PB9]==0;
}
int main() {
  for(int i=0;i<64;++i)mockDigital[i]=HIGH;
  for(int i=0;i<16;++i)mockMuxADC[i]=160;
  setup();
  assert(allMotorOff());
  // First firmware is locked. Select Race, confirm Arm, confirm Start.
  press(PB5);press(PB5);
  assert(allMotorOff());
  press(PB3);
  // Move to Sensor Live and back. These should never command a motor.
  press(PB12);press(PB5);advance(200);press(PB3);
  assert(allMotorOff());
  // Menu was on item 1 (Sensor Live), advance 11 to Motor Check item 12.
  for(int i=0;i<11;++i)press(PB12);
  press(PB5); // Motor prompt
  press(PB5); // test starts
  advance(15);
  assert(mockPwm[PB8]>0); // Left motor +
  mockDigital[PB3]=LOW;loop(); // asynchronous raw BACK emergency stop
  assert(allMotorOff());
  mockDigital[PB3]=HIGH;loop();
  puts("PASS full UI smoke: boot / locked race / sensor live / motor BACK stop");
  return 0;
}
