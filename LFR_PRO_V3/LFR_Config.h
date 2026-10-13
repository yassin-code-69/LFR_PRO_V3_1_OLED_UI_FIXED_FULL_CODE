#pragma once
#include <stdint.h>
#include <stddef.h>

// LFR PRO V3 -- STM32F103C8T6 Blue Pill / 14-channel analog MUX.
// Mapped to the user's known-working demo and V1 motor bench tests.
// PA4 is MUX S2; PB5 is ENTER. DO NOT also drive LED/buzzer on them.
// PB3 is BACK; firmware disables JTAG but keeps SWD available.
#define LFR_PIN_S0 PA7
#define LFR_PIN_S1 PA6
#define LFR_PIN_S2 PA4
#define LFR_PIN_S3 PA3
#define LFR_PIN_ADC PA2
#define LFR_PIN_LEFT_A PB9
#define LFR_PIN_LEFT_B PB8
#define LFR_PIN_RIGHT_A PA0
#define LFR_PIN_RIGHT_B PA1
#define LFR_PIN_UP PB13
#define LFR_PIN_DOWN PB12
#define LFR_PIN_ENTER PB5
#define LFR_PIN_BACK PB3
#ifndef LFR_PIN_LED
#if defined(PC13)
#define LFR_PIN_LED PC13
#else
#define LFR_PIN_LED -1
#endif
#endif
#ifndef LFR_PIN_BUZZER
#define LFR_PIN_BUZZER -1 // Set to GPIO pin (e.g. PB0, PA8) when physical buzzer is connected
#endif

// FIRST INSTALL: 0 = diagnose only; after checking sensor map, motor direction,
// stop button and calibration, deliberately set to 1 and reflash.
#define LFR_ALLOW_RACE_START 1
// 0 = do not print in the timing-critical race loop; 1 = ~5 Hz UART diagnostic.
#define LFR_RACE_TELEMETRY 0
#define LFR_OLED_ADDRESS 0x3C
// UI orientation: 0 = preserve V3 screen orientation; 1 = rotate display 180 degrees.
// Change this ONLY if the OLED appears physically upside down.
#define LFR_OLED_ROTATE_180 0
// 0 = safe 100kHz I2C; 1 = faster 400kHz if the OLED/wiring support it.
#define LFR_OLED_FAST_I2C 0

namespace lfr {
static constexpr uint8_t SENSOR_COUNT = 14;
static constexpr uint8_t ROUTE_CAPACITY = 20;
static constexpr uint16_t ADC_MAX = 4095;
static constexpr uint16_t MIN_CONTRAST = 250;
static constexpr uint16_t CTRL_PERIOD_US = 3000;  // High-precision 333Hz loop (sub-millimeter line resolution)
static constexpr uint32_t CTRL_WATCHDOG_US = 30000;
static constexpr uint32_t STORE_V3_MAGIC = 0x4C465233UL; // LFR3
static constexpr uint16_t STORE_V3_VERSION = 3;
// Physical LEFT to RIGHT order measured from the front of the LFR.
// This is the original demo assumption; verify by left-center-right sensor sweep.
static constexpr uint8_t MUX_BY_POSITION[SENSOR_COUNT] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
};

enum class LineMode : uint8_t { Black=0, White=1 };
enum class NavMode : uint8_t { Straight=0, Left=1, Right=2, Script=3 };
enum class RouteStep : uint8_t { Straight=0, Left=1, Right=2, UTurn=3 };
enum class GapMode : uint8_t { Recover=0, Stop=1 };
enum class Profile : uint8_t { Safe=0, Balanced=1, Fast=2 };
enum class DriveState : uint8_t {
  Idle, Follow, Gap, Pivot, JunctionAdvance, Marker, Finished, Fault
};

struct Settings {
  uint8_t lineMode;
  uint8_t navMode;
  uint8_t gapMode;
  uint8_t speedProfile;
  uint8_t mirror;
  uint8_t finishOn;
  uint8_t routeCount;
  uint8_t route[ROUTE_CAPACITY];
  uint8_t basePWM;   // nominal straight PWM, not a cap
  uint8_t maxPWM;    // individual wheel output cap
  uint8_t curvePWM;  // speed at a sharp bend
  uint8_t turnPWM;   // pivot wheel PWM
  uint8_t accelStep; // acceleration per 5ms control step
  uint8_t brakeStep; // deceleration per 5ms control step
  uint8_t junctionAdvanceMs; // start pivot after short junction approach
  uint16_t kpX10;    // Kp / 10
  uint16_t kiX100;   // Ki / 100 -- real integral, not zero
  uint16_t kdX100;   // Kd / 100 -- derivative per second
};
struct Calibration {
  uint16_t white[SENSOR_COUNT];
  uint16_t black[SENSOR_COUNT];
};
struct SensorFrame {
  uint16_t raw[SENSOR_COUNT];
  uint16_t norm[SENSOR_COUNT]; // 0=background, 1000=selected line
  int16_t pos;      // -6500 left -> +6500 right
  uint16_t confidence;
  uint8_t active;
  uint8_t groups;
  bool seen;
  bool center;
  bool leftEdge;
  bool rightEdge;
  bool fullMarker;
  bool junction;
};
struct Command { int16_t left; int16_t right; };

inline Settings defaultSettings() {
  Settings s = {};
  s.lineMode=0; s.navMode=0; s.gapMode=0; s.speedProfile=0;
  s.mirror=0; s.finishOn=0; s.routeCount=0;
  s.basePWM=76; s.maxPWM=112; s.curvePWM=52; s.turnPWM=83;
  s.accelStep=10; s.brakeStep=28; s.junctionAdvanceMs=35;
  s.kpX10=120; s.kiX100=120; s.kdX100=18;
  return s;
}
inline void setProfile(Settings &s, uint8_t profile) {
  s.speedProfile=profile%3;
  if (s.speedProfile==0) {
    s.basePWM=76; s.maxPWM=112; s.curvePWM=52; s.turnPWM=83;
    s.accelStep=10; s.brakeStep=28;
    s.kpX10=120; s.kiX100=120; s.kdX100=18;
  } else if (s.speedProfile==1) {
    s.basePWM=96; s.maxPWM=150; s.curvePWM=62; s.turnPWM=100;
    s.accelStep=12; s.brakeStep=32;
    s.kpX10=140; s.kiX100=90; s.kdX100=23;
  } else {
    // Pro Competition Profile (User Track-Verified Winning Setup)
    s.basePWM=165; s.maxPWM=220; s.curvePWM=75; s.turnPWM=125;
    s.accelStep=30; s.brakeStep=45; s.junctionAdvanceMs=25;
    s.kpX10=210; s.kiX100=0; s.kdX100=38;
  }
}
inline bool saneSettings(const Settings &s) {
  if (s.lineMode>1 || s.navMode>3 || s.gapMode>1 || s.speedProfile>2 ||
      s.mirror>1 || s.finishOn>1 || s.routeCount>ROUTE_CAPACITY ||
      s.curvePWM<35 || s.basePWM<50 || s.maxPWM>220 || s.turnPWM<45 ||
      s.turnPWM>155 || s.maxPWM<s.basePWM || s.basePWM<s.curvePWM ||
      s.accelStep<2 || s.accelStep>30 || s.brakeStep<8 || s.brakeStep>55 ||
      s.junctionAdvanceMs>110 ||
      s.kpX10<20 || s.kpX10>350 || s.kiX100>600 || s.kdX100>120) return false;
  for(uint8_t i=0;i<s.routeCount;++i) if(s.route[i]>3) return false;
  return true;
}
inline bool validCalibration(const Calibration &cal) {
  for (uint8_t i=0;i<SENSOR_COUNT;++i) {
    if (cal.white[i]>ADC_MAX || cal.black[i]>ADC_MAX ||
        uint32_t(cal.black[i])<uint32_t(cal.white[i])+MIN_CONTRAST) return false;
  }
  return true;
}
inline int16_t clamp16(int32_t value, int16_t lo, int16_t hi) {
  return value<lo?lo:(value>hi?hi:int16_t(value));
}
} // namespace lfr
