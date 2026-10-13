#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "LFR_Config.h"
namespace lfr {
// Small self-contained SSD1306 driver: avoids a large GFX dependency on 64KB flash.
// Text is uppercase 5x7 ASCII (digits and common punctuation).
class TinyOLED {
public:
  bool begin();
  void clear();
  void show();
  void text(uint8_t x,uint8_t y,const char *string,bool inverse=false);
  void textBig(uint8_t x,uint8_t y,const char *string,bool inverse=false);
  void number(uint8_t x,uint8_t y,int32_t n,bool inverse=false);
  void twoDigits(uint8_t x,uint8_t y,uint8_t n,bool inverse=false);
  void fill(uint8_t x,uint8_t y,uint8_t w,uint8_t h,bool white=true);
  void pixel(uint8_t x,uint8_t y,bool white=true);
  void rule(uint8_t y);
#ifdef LFR_UI_TEST_ACCESS
  bool testPixel(uint8_t x, uint8_t y) const {
    return x<128 && y<64 && ((pixels_[uint16_t(y/8)*128U+x] >> (y&7)) & 1U);
  }
#endif
private:
  uint8_t pixels_[1024]={};
  bool sendCommand(uint8_t cmd);
  void glyph(uint8_t x,uint8_t y,char ch,bool inverse);
};
}
