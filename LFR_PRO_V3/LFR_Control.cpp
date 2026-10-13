#include "LFR_Control.h"
#include <math.h>
namespace lfr {
static float clampF(float v,float a,float b) {
  return v<a?a:(v>b?b:v);
}
void Controller::enter(DriveState state,uint32_t nowMs) {
  state_=state;entered_=nowMs;
  seenFrames_=0;centerFrames_=0;edgeFrames_=0;markerFrames_=0;straightFrames_=0;
}
void Controller::start(uint32_t nowMs,int16_t initialPosition) {
  state_=DriveState::Follow;
  started_=entered_=nowMs;
  cooldownUntil_=0;clearSince_=0;
  lastPos_=initialPosition;
  seenFrames_=centerFrames_=edgeFrames_=junctionFrames_=0;
  edgeDirection_=0;oldCenterCleared_=false;junctionLatched_=false;
  pivotDirection_=-1;pivotMinimumMs_=90;pivotMaximumMs_=900;
  junctions_=routeUsed_=0;
  pending_=RouteStep::Straight;
  pid_.reset(float(initialPosition)*0.001f);
}
void Controller::stop() {state_=DriveState::Idle;pid_.reset();}
bool Controller::active() const {
  return state_==DriveState::Follow || state_==DriveState::Gap ||
         state_==DriveState::Pivot || state_==DriveState::JunctionAdvance ||
         state_==DriveState::Marker;
}
const char* Controller::status() const {
  switch(state_) {
    case DriveState::Idle:return "IDLE";
    case DriveState::Follow:return "FOLLOW";
    case DriveState::Gap:return "GAP";
    case DriveState::Pivot:return "PIVOT";
    case DriveState::JunctionAdvance:return "JUNCTION";
    case DriveState::Marker:return "MARKER";
    case DriveState::Finished:return "FINISHED";
    default:return "FAULT";
  }
}
void Controller::startPivot(int8_t direction,uint32_t nowMs,bool wasCenter,
                            uint16_t minimum,uint16_t maximum) {
  pivotDirection_=direction<0?-1:1;
  pivotMinimumMs_=minimum;pivotMaximumMs_=maximum;
  oldCenterCleared_=!wasCenter;
  pid_.reset(float(lastPos_)*0.001f);
  enter(DriveState::Pivot,nowMs);
}
Command Controller::pivot(const Settings &s) const {
  const int16_t pwm=s.turnPWM;
  return pivotDirection_<0?Command{int16_t(-pwm),pwm}:Command{pwm,int16_t(-pwm)};
}
bool Controller::choose(const Settings &s,const SensorFrame &f,RouteStep &choice) {
  if(s.navMode==uint8_t(NavMode::Script)) {
    if(routeUsed_>=s.routeCount) return false;
    choice=RouteStep(s.route[routeUsed_++]);return true;
  }
  if(s.navMode==uint8_t(NavMode::Left))
    choice=f.leftEdge?RouteStep::Left:(f.center?RouteStep::Straight:RouteStep::Right);
  else if(s.navMode==uint8_t(NavMode::Right))
    choice=f.rightEdge?RouteStep::Right:(f.center?RouteStep::Straight:RouteStep::Left);
  else
    choice=f.center?RouteStep::Straight:(f.leftEdge?RouteStep::Left:RouteStep::Right);
  return true;
}
Command Controller::follow(const SensorFrame &f,const Settings &s,float dt) {
  const float error=float(f.pos)*0.001f;
  const float previous=float(lastPos_)*0.001f;
  const float rate=clampF((error-previous)/dt,-180.f,180.f);
  const float anglePenalty=fabsf(error)/3.8f;
  const float speedPenalty=fabsf(rate)/70.f;
  const float curvature=clampF(fmaxf(anglePenalty,speedPenalty),0.f,1.f);

  // Dynamic Straightaway Turbo Boost:
  // Smoothly ramp up toward maxPWM when dead center (<0.65) and calm rate (<18)
  if(fabsf(error)<0.65f && fabsf(rate)<18.0f && !f.junction && !f.fullMarker) {
    if(straightFrames_<40) ++straightFrames_;
  } else {
    straightFrames_=0;
  }
  float boostSpeed=float(s.basePWM);
  if(straightFrames_>=15) {
    const float boostFraction=float(straightFrames_-15)/25.0f; // gentle 0.0 to 1.0 ramp
    boostSpeed+=float(s.maxPWM-s.basePWM)*(boostFraction>1.0f?1.0f:boostFraction);
  }

  // Smooth continuous deceleration with curvature (no discrete step shocks)
  float speed=boostSpeed-(float(boostSpeed-s.curvePWM)*curvature);
  if(f.confidence<300) speed-=6.f;
  if(speed<float(s.curvePWM)) speed=s.curvePWM;

  const bool stable = f.active<=5 && f.confidence>=250 && !f.junction &&
                      !f.fullMarker && fabsf(error)<2.8f;
  const float steering=pid_.update(error,dt,s,float(s.maxPWM),stable);
  lastPos_=f.pos;

  // Silky-smooth differential steering: inner wheel slows proportionally toward 0 (no reverse jerk)
  const int16_t left=clamp16(int32_t(speed+steering),0,s.maxPWM);
  const int16_t right=clamp16(int32_t(speed-steering),0,s.maxPWM);
  return Command{left,right};
}
Command Controller::tick(const SensorFrame &f,const Settings &s,
                         uint32_t nowMs,float dt) {
  const Command stop={0,0};
  if(!active()) return stop;
  if(!(dt>0.f) || dt>0.030f) {enter(DriveState::Fault,nowMs);return stop;}
  const uint32_t elapsed=nowMs-entered_;
  if(f.seen && !f.fullMarker && !f.junction) {
    if(seenFrames_<8)++seenFrames_;
  } else seenFrames_=0;

  if(state_==DriveState::Pivot) {
    if(elapsed>pivotMaximumMs_) {enter(DriveState::Fault,nowMs);return stop;}
    if(!f.center) oldCenterCleared_=true;
    if(oldCenterCleared_ && f.seen && f.center && f.active<=5 &&
       !f.junction && !f.fullMarker && f.pos>-1900 && f.pos<1900) {
      if(centerFrames_<8)++centerFrames_;
    } else centerFrames_=0;
    if(elapsed>=pivotMinimumMs_ && centerFrames_>=3) {
      enter(DriveState::Follow,nowMs);
      pid_.reset(float(f.pos)*0.001f);lastPos_=f.pos;
      cooldownUntil_=nowMs+200;
      return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
    }
    return pivot(s);
  }

  if(state_==DriveState::Gap) {
    if(seenFrames_>=2) {
      enter(DriveState::Follow,nowMs);
      pid_.reset(float(f.pos)*0.001f);lastPos_=f.pos;
      cooldownUntil_=nowMs+150;
      return follow(f,s,dt);
    }
    // Timeout safeguard: 180ms in Recover (covers gap cleanly), 55ms in Stop
    if(elapsed>(s.gapMode==uint8_t(GapMode::Stop)?55U:180U)) {
      enter(DriveState::Fault,nowMs);return stop;
    }
    // True Gap Recover: hold forward trajectory smoothly across the gap without spinning in circles!
    const int16_t forwardPWM = int16_t((uint16_t(s.basePWM) + uint16_t(s.curvePWM)) / 2U);
    const int16_t bias=clamp16(lastPos_/450,-10,10);
    return Command{clamp16(int32_t(forwardPWM)+bias,0,s.maxPWM),
                   clamp16(int32_t(forwardPWM)-bias,0,s.maxPWM)};
  }

  if(state_==DriveState::Marker) {
    if(f.fullMarker) {
      if(markerFrames_<250) ++markerFrames_;
      if(s.finishOn && (nowMs-started_)>2000 && markerFrames_>=3) {
        enter(DriveState::Finished,nowMs);return stop;
      }
    } else {
      if(s.finishOn && (nowMs-started_)>2000 && markerFrames_>=3) {
        enter(DriveState::Finished,nowMs);return stop;
      }
      enter(DriveState::Follow,nowMs);
      pid_.reset(float(f.pos)*0.001f);
      cooldownUntil_=nowMs+250;junctionLatched_=true;clearSince_=nowMs;
      if(!f.seen) {enter(DriveState::Gap,nowMs);return stop;}
      return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
    }
    if(elapsed>650) {enter(DriveState::Fault,nowMs);return stop;}
    return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
  }

  if(state_==DriveState::JunctionAdvance) {
    if(elapsed>=s.junctionAdvanceMs) {
      if(pending_==RouteStep::Straight) {
        enter(DriveState::Follow,nowMs);
        pid_.reset(float(f.pos)*0.001f);
        return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
      }
      const bool isU=pending_==RouteStep::UTurn;
      const int8_t side=pending_==RouteStep::Right?1:-1;
      startPivot(side,nowMs,f.center,isU?285:90,isU?1450:900);
      return pivot(s);
    }
    return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
  }

  if(state_!=DriveState::Follow) return stop;
  if(f.fullMarker) {
    enter(DriveState::Marker,nowMs);
    return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
  }
  if(!f.seen) {
    // Immediate edge loss: steer toward last observed edge rather than coast ahead.
    if(lastPos_<=-3800 || lastPos_>=3800) {
      startPivot(lastPos_<0?-1:1,nowMs,false,85,800);
      return pivot(s);
    }
    pid_.reset(float(lastPos_)*0.001f);
    enter(DriveState::Gap,nowMs);
    const int16_t forwardPWM = int16_t((uint16_t(s.basePWM) + uint16_t(s.curvePWM)) / 2U);
    return Command{forwardPWM,forwardPWM};
  }

  // For real 90-degree corners, the far edge can be the only detected region.
  int8_t side=0;
  if(!f.junction && !f.center && f.active<=5 &&
     f.leftEdge && !f.rightEdge && f.pos<-4450) side=-1;
  else if(!f.junction && !f.center && f.active<=5 &&
          f.rightEdge && !f.leftEdge && f.pos>4450) side=1;
  if(side==0) {edgeFrames_=0;edgeDirection_=0;}
  else if(side!=edgeDirection_) {edgeDirection_=side;edgeFrames_=1;}
  else if(edgeFrames_<5) ++edgeFrames_;
  if(edgeFrames_>=2) {
    lastPos_=f.pos;
    startPivot(side,nowMs,f.center,90,900);
    return pivot(s);
  }

  // Junction counter debounced and latched until the marker fully clears.
  if(f.junction) {
    clearSince_=0;
    if(junctionFrames_<8) ++junctionFrames_;
  } else {
    junctionFrames_=0;
    if(clearSince_==0) clearSince_=nowMs;
    if(junctionLatched_ && (nowMs-clearSince_)>=175 && nowMs>=cooldownUntil_)
      junctionLatched_=false;
  }
  if(f.junction && junctionFrames_>=3 && !junctionLatched_ &&
     nowMs>=cooldownUntil_) {
    RouteStep chosen;
    if(!choose(s,f,chosen)) {enter(DriveState::Fault,nowMs);return stop;}
    junctionLatched_=true;
    ++junctions_;
    cooldownUntil_=nowMs+360;
    pending_=chosen;
    enter(DriveState::JunctionAdvance,nowMs);
    return Command{int16_t(s.curvePWM),int16_t(s.curvePWM)};
  }
  return follow(f,s,dt);
}
}
