#pragma once
#include "Arduino.h"
struct MockEEPROM {
  uint8_t data[1024]={};
  size_t length() const {return sizeof(data);}
  template<typename T> void get(size_t addr,T &dest) {
    std::memcpy(&dest,&data[addr],sizeof(T));
  }
};
extern MockEEPROM EEPROM;
void eeprom_buffer_fill();
void eeprom_buffered_write_byte(size_t address,uint8_t b);
void eeprom_buffer_flush();
