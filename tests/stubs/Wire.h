#pragma once
#include "Arduino.h"
struct MockWire {
  void begin(){}
  void setClock(uint32_t){}
  void beginTransmission(uint8_t){}
  void write(uint8_t){}
  uint8_t endTransmission(){return 0;}
};
extern MockWire Wire;
