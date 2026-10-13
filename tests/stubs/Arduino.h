#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#define PA0 0U
#define PA1 1U
#define PA2 2U
#define PA3 3U
#define PA4 4U
#define PA6 6U
#define PA7 7U
#define PA9 9U
#define PA10 10U
#define PB3 19U
#define PB5 21U
#define PB8 24U
#define PB9 25U
#define PB12 28U
#define PB13 29U
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_ANALOG 3
#define LOW 0
#define HIGH 1
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE (1U<<24)
struct MockAFIO {uint32_t MAPR=0;};
extern MockAFIO mockAfio;
#define AFIO (&mockAfio)
extern uint64_t mockMicro;
extern int mockDigital[64];
extern int mockPwm[64];
extern int mockMuxADC[16];
void pinMode(uint32_t p,int mode);
void digitalWrite(uint32_t p,int level);
int digitalRead(uint32_t p);
int analogRead(uint32_t p);
void analogWrite(uint32_t p,int duty);
void analogReadResolution(uint8_t bits);
void analogWriteResolution(uint8_t bits);
void analogWriteFrequency(uint32_t freq);
void delayMicroseconds(uint32_t n);
void delay(uint32_t ms);
uint32_t micros();
uint32_t millis();
template<class T> T clampHost(T v,T lo,T hi) {return v<lo?lo:(v>hi?hi:v);}
#define constrain(x,a,b) clampHost<int>(int(x),int(a),int(b))
struct MockSerial {
  void begin(uint32_t) {}
  template<class T>void print(T) {}
  template<class T>void println(T) {}
};
extern MockSerial Serial1;
