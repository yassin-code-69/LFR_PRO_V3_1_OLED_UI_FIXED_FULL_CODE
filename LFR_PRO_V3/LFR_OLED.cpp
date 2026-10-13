#include "LFR_OLED.h"
namespace lfr {
static const char glyphSet[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-.:/>=+?%_#!";
static const uint8_t font[][5] = {
  {0,0,0,0,0}, // space
  {0x7E,0x11,0x11,0x11,0x7E}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x09,0x01}, // F
  {0x3E,0x41,0x49,0x49,0x7A}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x0C,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x7F,0x20,0x18,0x20,0x7F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x07,0x08,0x70,0x08,0x07}, // Y
  {0x61,0x51,0x49,0x45,0x43}, // Z
  {0x3E,0x45,0x49,0x51,0x3E}, // 0
  {0x00,0x21,0x7F,0x01,0x00}, // 1
  {0x23,0x45,0x49,0x51,0x21}, // 2
  {0x42,0x41,0x51,0x69,0x46}, // 3
  {0x0C,0x14,0x24,0x7F,0x04}, // 4
  {0x72,0x51,0x51,0x51,0x4E}, // 5
  {0x1E,0x29,0x49,0x49,0x06}, // 6
  {0x40,0x47,0x48,0x50,0x60}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x30,0x49,0x49,0x4A,0x3C}, // 9
  {0x08,0x08,0x08,0x08,0x08}, // -
  {0x00,0x40,0x40,0x00,0x00}, // .
  {0x00,0x36,0x36,0x00,0x00}, // :
  {0x20,0x10,0x08,0x04,0x02}, // /
  {0x08,0x14,0x22,0x41,0x00}, // >
  {0x14,0x14,0x14,0x14,0x14}, // =
  {0x08,0x08,0x3E,0x08,0x08}, // +
  {0x02,0x01,0x51,0x09,0x06}, // ?
  {0x63,0x13,0x08,0x64,0x63}, // %
  {0x40,0x40,0x40,0x40,0x40}, // _
  {0x14,0x7F,0x14,0x7F,0x14}, // #
  {0x00,0x00,0x5F,0x00,0x00}  // !
};
static_assert(sizeof(font)/sizeof(font[0])==sizeof(glyphSet)-1,"glyph table mismatch");
// A 5x7 seven-segment numeric font. Distinguishes 0/O, 1/I and 9/a clearly.
// Segments: top, upper-right, lower-right, bottom, lower-left, upper-left, center.
static const uint8_t digitSegments[10] = {
  0x3F, // 0 A B C D E F
  0x06, // 1 B C
  0x5B, // 2 A B G E D
  0x4F, // 3 A B C D G
  0x66, // 4 B C F G
  0x6D, // 5 A C D F G
  0x7D, // 6 A C D E F G
  0x07, // 7 A B C
  0x7F, // 8 all
  0x6F  // 9 A B C D F G
};
static bool segmentDigitPixel(uint8_t value,uint8_t col,uint8_t row) {
  if(value>9 || col>=5 || row>=7)return false;
  // Digit 1 has a hooked top and a full foot, not a lowercase i or L.
  if(value==1)return (row==0 && col==2) ||
                       (row==1 && col>=1 && col<=2) ||
                       (row>=2 && row<=5 && col==2) ||
                       (row==6 && col<=4);
  const uint8_t mask=digitSegments[value];
  if(row==0 && col>=1 && col<=3)return (mask&0x01)!=0; // A
  if(row>=1 && row<=2 && col==4)return (mask&0x02)!=0; // B
  if(row>=4 && row<=5 && col==4)return (mask&0x04)!=0; // C
  if(row==6 && col>=1 && col<=3)return (mask&0x08)!=0; // D
  if(row>=4 && row<=5 && col==0)return (mask&0x10)!=0; // E
  if(row>=1 && row<=2 && col==0)return (mask&0x20)!=0; // F
  if(row==3 && col>=1 && col<=3)return (mask&0x40)!=0; // G
  return false;
}


bool TinyOLED::sendCommand(uint8_t cmd) {
  Wire.beginTransmission(LFR_OLED_ADDRESS);
  Wire.write(uint8_t(0x00));Wire.write(cmd);
  return Wire.endTransmission()==0;
}
bool TinyOLED::begin() {
  // Match the proven V1 100 kHz I2C behavior first.
#if LFR_OLED_FAST_I2C
  Wire.setClock(400000);
#else
  Wire.setClock(100000);
#endif
  Wire.beginTransmission(LFR_OLED_ADDRESS);
  if(Wire.endTransmission()!=0) return false;
  const uint8_t init[]={
    0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,
    0x20,0x00,
#if LFR_OLED_ROTATE_180
    0xA0,0xC0,
#else
    0xA1,0xC8,
#endif
    0xDA,0x12,0x81,0x8F,0xD9,0xF1,
    0xDB,0x40,0xA4,0xA6,0x2E,0xAF
  };
  for(uint8_t i=0;i<sizeof(init);++i) if(!sendCommand(init[i]))return false;
  clear();show();return true;
}
void TinyOLED::clear() {for(uint16_t i=0;i<1024;++i)pixels_[i]=0;}
void TinyOLED::pixel(uint8_t x,uint8_t y,bool white) {
  if(x>=128 || y>=64)return;
  const uint16_t k=uint16_t(y/8)*128U+x;
  const uint8_t mask=uint8_t(1U<<(y&7));
  if(white)pixels_[k]|=mask;
  else pixels_[k]&=uint8_t(~mask);
}
void TinyOLED::fill(uint8_t x,uint8_t y,uint8_t w,uint8_t h,bool white) {
  const uint16_t right=uint16_t(x)+w;
  const uint16_t bottom=uint16_t(y)+h;
  for(uint16_t yy=y;yy<bottom && yy<64;++yy)
    for(uint16_t xx=x;xx<right && xx<128;++xx)
      pixel(uint8_t(xx),uint8_t(yy),white);
}
void TinyOLED::rule(uint8_t y) {fill(0,y,128,1,true);}
void TinyOLED::glyph(uint8_t x,uint8_t y,char ch,bool inverse) {
  if(ch>='a' && ch<='z')ch=char(ch-'a'+'A');
  uint8_t index=0;
  for(uint8_t i=0;i<sizeof(glyphSet)-1;++i)
    if(glyphSet[i]==ch) {index=i;break;}
  for(uint8_t dx=0;dx<6;++dx) {
    const uint8_t data=(dx<5)?font[index][dx]:0;
    for(uint8_t dy=0;dy<8;++dy) {
      const bool dot=dx<5 && dy<7 && (
        (ch>='0' && ch<='9') ? segmentDigitPixel(uint8_t(ch-'0'),dx,dy) :
        ((data>>dy)&1U)!=0);
      pixel(uint8_t(x+dx),uint8_t(y+dy),inverse?!dot:dot);
    }
  }
}
void TinyOLED::text(uint8_t x,uint8_t y,const char *string,bool inverse) {
  while(*string && x<=122) {glyph(x,y,*string++,inverse);x=uint8_t(x+6);}
}
// Readable double-size numerals for tuning pages, with clipping and no dynamic memory.
void TinyOLED::textBig(uint8_t x,uint8_t y,const char *string,bool inverse) {
  while(*string && x<=116) {
    char ch=*string++;
    if(ch>='a' && ch<='z')ch=char(ch-'a'+'A');
    uint8_t index=0;
    for(uint8_t i=0;i<sizeof(glyphSet)-1;++i) {
      if(glyphSet[i]==ch) {index=i;break;}
    }
    for(uint8_t dx=0;dx<6;++dx) {
      const uint8_t data=(dx<5)?font[index][dx]:0;
      for(uint8_t dy=0;dy<8;++dy) {
        const bool dot=dx<5 && dy<7 && (
          (ch>='0' && ch<='9') ? segmentDigitPixel(uint8_t(ch-'0'),dx,dy) :
          ((data>>dy)&1U)!=0);
        for(uint8_t sx=0;sx<2;++sx)
          for(uint8_t sy=0;sy<2;++sy)
            pixel(uint8_t(x+dx*2+sx),uint8_t(y+dy*2+sy),inverse?!dot:dot);
      }
    }
    x=uint8_t(x+12);
  }
}
void TinyOLED::twoDigits(uint8_t x,uint8_t y,uint8_t value,bool inverse) {
  if(value>99)value=99;
  char text[3];
  text[0]=char('0'+value/10U);
  text[1]=char('0'+value%10U);
  text[2]=0;
  this->text(x,y,text,inverse);
}
void TinyOLED::number(uint8_t x,uint8_t y,int32_t value,bool inverse) {
  char buf[14];uint8_t i=0;
  int64_t n=value;
  if(n<0) {buf[i++]='-';n=-n;}
  char digit[11];uint8_t k=0;
  do{digit[k++]=char('0'+n%10);n/=10;}while(n && k<sizeof(digit));
  while(k) buf[i++]=digit[--k];
  buf[i]=0;
  text(x,y,buf,inverse);
}
void TinyOLED::show() {
  // 128x64 SSD1306, horizontal addressing, 16 data bytes per I2C packet.
  if(!sendCommand(0x21) || !sendCommand(0) || !sendCommand(127) ||
     !sendCommand(0x22) || !sendCommand(0) || !sendCommand(7))return;
  for(uint16_t off=0;off<1024;off+=16) {
    Wire.beginTransmission(LFR_OLED_ADDRESS);
    Wire.write(uint8_t(0x40));
    for(uint8_t i=0;i<16;++i)Wire.write(pixels_[off+i]);
    if(Wire.endTransmission()!=0)break;
  }
}
}
