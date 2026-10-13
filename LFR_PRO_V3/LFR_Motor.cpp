#include "LFR_Motor.h"
namespace lfr {
void Motors::begin() {
  pinMode(LFR_PIN_LEFT_A,OUTPUT); pinMode(LFR_PIN_LEFT_B,OUTPUT);
  pinMode(LFR_PIN_RIGHT_A,OUTPUT); pinMode(LFR_PIN_RIGHT_B,OUTPUT);
  digitalWrite(LFR_PIN_LEFT_A,LOW);digitalWrite(LFR_PIN_LEFT_B,LOW);
  digitalWrite(LFR_PIN_RIGHT_A,LOW);digitalWrite(LFR_PIN_RIGHT_B,LOW);
  analogWriteResolution(8);
  analogWriteFrequency(10000);
  stop();
}
void Motors::outputOne(uint32_t a,uint32_t b,int16_t pwm) {
  // Never PWM both BTS7960 direction inputs at once.
  if(pwm>0) {
    analogWrite(a,0);
    analogWrite(b,pwm); // positive = forward: V1 direction tested on robot
  } else if(pwm<0) {
    analogWrite(b,0);
    analogWrite(a,-pwm);
  } else {
    analogWrite(a,0);
    analogWrite(b,0);
  }
}
void Motors::stop() {
  left_=right_=0;
  outputOne(LFR_PIN_LEFT_A,LFR_PIN_LEFT_B,0);
  outputOne(LFR_PIN_RIGHT_A,LFR_PIN_RIGHT_B,0);
}
int16_t Motors::slew(int16_t current,int16_t target,const Settings &s) {
  // A reversal ALWAYS passes through 0; deceleration is faster than acceleration.
  if ((current>0 && target<0)||(current<0 && target>0)) target=0;
  const int16_t difference=target-current;
  const int16_t step = (abs(target)<abs(current))?s.brakeStep:s.accelStep;
  if(difference>step) return current+step;
  if(difference<-step) return current-step;
  return target;
}
void Motors::drive(Command demand,const Settings &s,bool reverseAllowed) {
  const int16_t cap=s.maxPWM;
  if (!reverseAllowed) {
    if(demand.left<0)demand.left=0;
    if(demand.right<0)demand.right=0;
  }
  demand.left=clamp16(demand.left,-cap,cap);
  demand.right=clamp16(demand.right,-cap,cap);
  left_=slew(left_,demand.left,s);
  right_=slew(right_,demand.right,s);
  outputOne(LFR_PIN_LEFT_A,LFR_PIN_LEFT_B,left_);
  outputOne(LFR_PIN_RIGHT_A,LFR_PIN_RIGHT_B,right_);
}
}
