#include "Arduino.h"
#include "Wire.h"
#include "EEPROM.h"
MockAFIO mockAfio;
MockSerial Serial1;
MockWire Wire;
MockEEPROM EEPROM;
MockGPIO mockGpioA;
GPIOProxy mockGpioAProxy{&mockGpioA, {&mockGpioA}};
uint64_t mockMicro=0;
int mockDigital[64]={};
int mockPwm[64]={};
int mockMuxADC[16]={};
void pinMode(uint32_t,int){}
void digitalWrite(uint32_t p,int level){mockDigital[p]=level;}
int digitalRead(uint32_t p){return mockDigital[p] ? HIGH : LOW;}
int analogRead(uint32_t) {
  // Decode MUX channel from GPIOA ODR (fast path) or digital pins (fallback)
  // Fast path: read directly from mockGpioA.ODR bit positions
  // S0=PA7(bit7) S1=PA6(bit6) S2=PA4(bit4) S3=PA3(bit3)
  const uint32_t odr=mockGpioA.ODR;
  int mux = ((odr>>7)&1) | (((odr>>6)&1)<<1) |
            (((odr>>4)&1)<<2) | (((odr>>3)&1)<<3);
  return mockMuxADC[mux];
}
void analogWrite(uint32_t p,int duty){mockPwm[p]=duty;}
void analogReadResolution(uint8_t){}
void analogWriteResolution(uint8_t){}
void analogWriteFrequency(uint32_t){}
void delayMicroseconds(uint32_t n){mockMicro+=n;}
void delay(uint32_t ms){mockMicro+=uint64_t(ms)*1000;}
uint32_t micros(){return uint32_t(mockMicro);}
uint32_t millis(){return uint32_t(mockMicro/1000);}
void eeprom_buffer_fill(){}
void eeprom_buffered_write_byte(size_t pos,uint8_t byte){EEPROM.data[pos]=byte;}
void eeprom_buffer_flush(){}
