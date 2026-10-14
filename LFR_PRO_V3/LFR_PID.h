#pragma once
#include "LFR_Config.h"
namespace lfr {
class PID {
public:
  void reset(float initialMeasurement=0.0f);
  float update(float error, float dt, const Settings &cfg,
               float outputLimit, bool integrate,
               float kpScale = 1.0f, float kdScale = 1.0f);
  float integral() const { return integral_; }
private:
  float lastError_=0.0f;
  float dFiltered_=0.0f;
  float integral_=0.0f;
  bool initialized_=false;
};
}
