#include "Arduino.h"
#include "Wire.h"
#include "EEPROM.h"
MockAFIO mockAfio;
MockSerial Serial1;
MockWire Wire;
MockEEPROM EEPROM;
uint64_t mockMicro=0;
int mockDigital[64]={};
int mockPwm[64]={};
int mockMuxADC[16]={};
void pinMode(uint32_t,int){}
void digitalWrite(uint32_t p,int level){mockDigital[p]=level;}
int digitalRead(uint32_t p){return mockDigital[p] ? HIGH : LOW;}
int analogRead(uint32_t) {
  int mux=mockDigital[PA7] | mockDigital[PA6]<<1 |
          mockDigital[PA4]<<2 | mockDigital[PA3]<<3;
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
