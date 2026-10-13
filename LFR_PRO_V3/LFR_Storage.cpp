#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>
#include "LFR_Storage.h"
namespace lfr {
struct Store3 {
  uint32_t magic;
  uint16_t version;
  uint16_t hasCalibration;
  uint32_t sequence;
  Settings settings;
  Calibration cal;
  uint16_t crc;
};
struct LegacySettings {
  uint8_t lineMode,navMode,gapMode,speedProfile,mirror,finishOn,routeCount;
  uint8_t route[16];
  uint8_t basePWM,maxPWM,curvePWM,turnPWM;
  uint16_t kpX10,kdX100;
};
struct LegacyStore {
  uint32_t magic;
  uint16_t version;
  uint16_t hasCalibration;
  LegacySettings settings;
  Calibration cal;
  uint16_t crc;
};
static_assert(sizeof(LegacySettings)==32,"Original V1 Settings layout changed");
static_assert(sizeof(Store3)<=220,"Storage exceeds EEPROM slot stride");
static constexpr uint16_t LEGACY_OFFSET=256;
static constexpr uint16_t SLOT_A=512;
static constexpr uint16_t SLOT_B=768;
static constexpr uint32_t LEGACY_MAGIC=0x4C465232UL;

static uint16_t crc16(const uint8_t *data,size_t bytes) {
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<bytes;++i) {
    crc^=uint16_t(data[i]);
    for(uint8_t b=0;b<8;++b)
      crc=(crc&1)?uint16_t((crc>>1)^0xA001):uint16_t(crc>>1);
  }
  return crc;
}
static bool good3(const Store3 &d) {
  return d.magic==STORE_V3_MAGIC && d.version==STORE_V3_VERSION &&
    d.hasCalibration<=1 && saneSettings(d.settings) &&
    (!d.hasCalibration || validCalibration(d.cal)) &&
    d.crc==crc16(reinterpret_cast<const uint8_t*>(&d),offsetof(Store3,crc));
}
static bool goodLegacy(const LegacyStore &d) {
  if(d.magic!=LEGACY_MAGIC || d.version!=1 || d.hasCalibration>1 ||
     d.settings.lineMode>1 || d.settings.navMode>3 ||
     d.settings.gapMode>1 || d.settings.speedProfile>2 ||
     d.settings.mirror>1 || d.settings.finishOn>1 || d.settings.routeCount>16 ||
     (d.hasCalibration && !validCalibration(d.cal)))return false;
  if(d.crc!=crc16(reinterpret_cast<const uint8_t*>(&d),offsetof(LegacyStore,crc)))
    return false;
  for(uint8_t i=0;i<d.settings.routeCount;++i)
    if(d.settings.route[i]>3)return false;
  return true;
}
static bool read3(uint16_t offset,Store3 &d) {
  if(size_t(offset)+sizeof(Store3)>size_t(EEPROM.length()))return false;
  d=Store3{};EEPROM.get(offset,d);return good3(d);
}
static bool readLegacy(LegacyStore &d) {
  if(size_t(LEGACY_OFFSET)+sizeof(LegacyStore)>size_t(EEPROM.length()))return false;
  d=LegacyStore{};EEPROM.get(LEGACY_OFFSET,d);return goodLegacy(d);
}
static uint8_t clamp8(int v,int lo,int hi) {
  return uint8_t(v<lo?lo:(v>hi?hi:v));
}
static void fromLegacy(Settings &cfg,const LegacySettings &old) {
  cfg=defaultSettings();
  cfg.lineMode=old.lineMode;cfg.navMode=old.navMode;
  cfg.gapMode=old.gapMode;cfg.speedProfile=old.speedProfile;
  cfg.mirror=old.mirror;cfg.finishOn=old.finishOn;
  cfg.routeCount=old.routeCount;
  for(uint8_t i=0;i<cfg.routeCount;++i)cfg.route[i]=old.route[i];
  cfg.maxPWM=clamp8(old.maxPWM,60,220);
  cfg.basePWM=clamp8(old.basePWM,50,cfg.maxPWM);
  cfg.curvePWM=clamp8(old.curvePWM,35,cfg.basePWM);
  cfg.turnPWM=clamp8(old.turnPWM,45,155);
  cfg.kpX10=(old.kpX10<20)?20:(old.kpX10>350?350:old.kpX10);
  cfg.kdX100=old.kdX100>120?120:old.kdX100;
  // A deliberately small, nonzero Ki; PID anti-windup limits accumulation.
  cfg.kiX100=120;
}
bool loadSettings(Settings &cfg,Calibration &cal,bool &calValid,bool &migrated) {
  migrated=false;calValid=false;
  Store3 a={},b={};
  const bool va=read3(SLOT_A,a),vb=read3(SLOT_B,b);
  if(va || vb) {
    const Store3 &chosen=(!va)?b:(!vb?a:(int32_t(b.sequence-a.sequence)>0?b:a));
    cfg=chosen.settings;calValid=(chosen.hasCalibration==1);
    if(calValid)cal=chosen.cal;
    return true;
  }
  LegacyStore legacy={};
  if(readLegacy(legacy)) {
    fromLegacy(cfg,legacy.settings);
    calValid=(legacy.hasCalibration==1);
    if(calValid)cal=legacy.cal;
    migrated=true;
    return true;
  }
  cfg=defaultSettings();return false;
}
static bool writeAt(uint16_t offset,const Store3 &record) {
  if(size_t(offset)+sizeof(Store3)>size_t(EEPROM.length()))return false;
#if defined(STM32_CORE_VERSION)
  EEPROM.put(offset, record);
#else
  eeprom_buffer_fill();
  const uint8_t *p=reinterpret_cast<const uint8_t*>(&record);
  for(size_t i=0;i<sizeof(Store3);++i)
    eeprom_buffered_write_byte(offset+i,p[i]);
  eeprom_buffer_flush();
#endif
  Store3 check={};EEPROM.get(offset,check);
  return good3(check) && check.sequence==record.sequence;
}
bool saveSettings(const Settings &cfg,const Calibration &cal,bool calValid) {
  if(!saneSettings(cfg) || (calValid && !validCalibration(cal)))return false;
  if(size_t(SLOT_B)+sizeof(Store3)>size_t(EEPROM.length()))return false;
  Store3 a={},b={};const bool va=read3(SLOT_A,a),vb=read3(SLOT_B,b);
  uint16_t next=SLOT_A;
  uint32_t sequence=1;
  if(va || vb) {
    const bool newerB=vb && (!va || int32_t(b.sequence-a.sequence)>0);
    const Store3 &latest=newerB?b:a;
    sequence=latest.sequence+1;
    next=newerB?SLOT_A:SLOT_B;
  }
  Store3 record={};
  record.magic=STORE_V3_MAGIC;record.version=STORE_V3_VERSION;
  record.hasCalibration=calValid?1:0;record.sequence=sequence;
  record.settings=cfg;
  if(calValid)record.cal=cal;
  record.crc=crc16(reinterpret_cast<const uint8_t*>(&record),offsetof(Store3,crc));
  return writeAt(next,record);
}
}
