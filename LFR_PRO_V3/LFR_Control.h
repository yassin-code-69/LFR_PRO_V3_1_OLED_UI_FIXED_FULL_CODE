#pragma once
#include "LFR_Config.h"
#include "LFR_PID.h"
namespace lfr {
class Controller {
public:
  void start(uint32_t nowMs,int16_t initialPosition);
  void stop();
  Command tick(const SensorFrame &f,const Settings &s,uint32_t nowMs,float dt);
  DriveState state() const {return state_;}
  bool active() const;
  uint8_t junctionCount() const {return junctions_;}
  uint8_t routeUsed() const {return routeUsed_;}
  const char* status() const;
private:
  DriveState state_=DriveState::Idle;
  PID pid_;
  uint32_t started_=0,entered_=0,cooldownUntil_=0,clearSince_=0;
  int16_t lastPos_=0;
  int8_t pivotDirection_=-1;
  uint8_t seenFrames_=0,centerFrames_=0,edgeFrames_=0,junctionFrames_=0,markerFrames_=0,straightFrames_=0;
  int8_t edgeDirection_=0;
  bool oldCenterCleared_=false,junctionLatched_=false;
  uint16_t pivotMinimumMs_=90,pivotMaximumMs_=900;
  RouteStep pending_=RouteStep::Straight;
  uint8_t junctions_=0,routeUsed_=0;
  // Corner Anticipation: counts consecutive frames where outer sensors detect line approaching edges
  uint8_t cornerBrakeFrames_=0;
  // Ghost Recovery Memory: remembers last good line state for intelligent gap recovery
  float lastGoodError_=0.0f;
  float lastGoodRate_=0.0f;
  void enter(DriveState state,uint32_t nowMs);
  void startPivot(int8_t direction,uint32_t nowMs,bool wasCenter,
                  uint16_t minimum=90,uint16_t maximum=900);
  Command pivot(const Settings &s) const;
  bool choose(const Settings &s,const SensorFrame &f,RouteStep &choice);
  Command follow(const SensorFrame &f,const Settings &s,float dt);
};
}
