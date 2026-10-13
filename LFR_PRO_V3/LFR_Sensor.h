#pragma once
#include <Arduino.h>
#include "LFR_Config.h"
namespace lfr {
class Sensor {
public:
  void begin();
  void resetTracking();
  void capture(uint16_t destination[SENSOR_COUNT]);
  void read(SensorFrame &f,const Calibration &cal,bool valid,const Settings &s);
private:
  uint16_t filtered_[SENSOR_COUNT]={};
  bool hasFilter_=false;
  int16_t lastPos_=0;
  bool hasPosition_=false;
  void selectChannel(uint8_t channel);
  uint16_t adcChannel(uint8_t channel);
};
}
