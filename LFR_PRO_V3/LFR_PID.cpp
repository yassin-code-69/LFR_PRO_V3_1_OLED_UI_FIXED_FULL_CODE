#include "LFR_PID.h"
#include <math.h>
namespace lfr {
static float bound(float x,float lower,float upper) {
  return x<lower?lower:(x>upper?upper:x);
}
void PID::reset(float initialError) {
  lastError_=initialError; dFiltered_=0.0f; integral_=0.0f;
  initialized_=true;
}
float PID::update(float error, float dt, const Settings &cfg,
                  float limit, bool integrate) {
  if (!(dt>0.0f) || dt>0.03f) {
    reset(error); return 0.0f;
  }
  if (!initialized_) reset(error);
  const float kp=float(cfg.kpX10)*0.1f;
  const float ki=float(cfg.kiX100)*0.01f;
  const float kd=float(cfg.kdX100)*0.01f;
  // Filtering D: 0.70/0.30 gives crisp corner damping without phase lag or sensor twitch.
  const float rawRate=bound((error-lastError_)/dt,-240.0f,240.0f);
  dFiltered_=0.70f*dFiltered_+0.30f*rawRate;
  lastError_=error;
  // Anti-windup: integrate only near the line and while output is not saturated.
  // Decay stored integral during corners / branch decisions / large errors.
  // Progressive steering: strictly linear near center, progressive bite on sharp corners.
  const float errorCubed = error * error * error;
  const float pTerm = kp * (error + 0.006f * errorCubed);
  const float pd = pTerm + kd * dFiltered_;
  if (integrate && fabsf(error)<2.8f && fabsf(pd+ki*integral_)<limit*0.80f) {
    integral_=bound(integral_+error*dt,-5.0f,5.0f);
  } else {
    integral_*=0.965f;
  }
  return bound(pd+ki*integral_,-limit,limit);
}
}
