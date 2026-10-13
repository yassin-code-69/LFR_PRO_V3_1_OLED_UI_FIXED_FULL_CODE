#pragma once
#include "LFR_Config.h"
namespace lfr {
// Loads current V3 dual-slot store, or one verified legacy V1/V1.1 record.
// 'migrated' means the values came from legacy and need a Save Settings.
bool loadSettings(Settings &cfg,Calibration &cal,bool &calValid,bool &migrated);
bool saveSettings(const Settings &cfg,const Calibration &cal,bool calValid);
}
