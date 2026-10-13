#pragma once
#include <Arduino.h>
#include "LFR_Config.h"
namespace lfr {
class Motors {
public:
  void begin();
  void stop();
  void drive(Command demand, const Settings &s, bool reverseAllowed);
  int16_t left() const {return left_;}
  int16_t right() const {return right_;}
private:
  int16_t left_=0, right_=0;
  static int16_t slew(int16_t current,int16_t target,const Settings &s);
  static void outputOne(uint32_t a,uint32_t b,int16_t pwm);
};
}
