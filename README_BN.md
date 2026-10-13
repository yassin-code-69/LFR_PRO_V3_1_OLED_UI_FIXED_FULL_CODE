# LFR PRO V3.1 — Professional OLED UI Fix (বাংলা)

## কোন সমস্যাগুলো ঠিক করা হয়েছে

- OLED-তে `09`, `10`, `11`, `15` সংখ্যাগুলো অক্ষরের মতো লাগত। 0–9 এর জন্য নতুন পৃথক **7-segment-ভিত্তিক 5×7 digit glyph** যোগ করা হয়েছে। Menu index সব সময় দুই অঙ্কে (`01` থেকে `15`) দেখায়।
- আগের firmware-এ `SPEED MODE`, `LINE MODE`, `NAVIGATION`, `GAP POLICY`, `FINISH MARKER`, `SENSOR MIRROR` এ ENTER চাপলেই option সঙ্গে সঙ্গে বদলে যেত। **এখন Main Menu-তেই active value দেখা যায়**। Option খুললে **ACTIVE** value এবং আলাদা highlighted preview দেখা যায়। UP/DOWN preview পরিবর্তন করে, ENTER APPLY করে, BACK CANCEL করে।
- PID-tuning-এর numeric value-এর নিচের crowded line বাদ দিয়ে item number উপরের status bar-এ দেওয়া হয়েছে।
- RESET DEFAULTS এখন confirmation চায়; BACK দিলে আগের settings অক্ষত থাকে।
- Menu scroll, calibration, route editor, sensor bars, race result ও OLED text spacing পরীক্ষা করা হয়েছে।

**গুরুত্বপূর্ণ:** PID / sensor / motor / junction / flash-storage algorithm এবং GPIO mapping আগের V3 version থেকে বদলানো হয়নি। UI ছাড়াও কেবল setting-selection-এর ব্যবহারপদ্ধতি (UI workflow) বদলেছে।

## একদম নতুন OLED Menu কীভাবে ব্যবহার করবে

1. UP / DOWN চাপলে menu পরিবর্তন হবে। প্রতিটি entry-এর সামনে পরিষ্কার `01` ... `15` নম্বর দেখাবে।
2. `05 SPEED`-এর ডান পাশে `SAFE`, `BALANCED`, বা `FAST` দেখা যাবে। `07 NAV`-এ `STRAIGHT/LEFT/RIGHT/SCRIPT`; `10 GAP`-এ `RECOVER/STOP`। `06 LINE`-এ `BLACK/WHITE`; `11 FINISH`-এ `OFF/ON`; `12 MIRROR`-এ `NORMAL/REVERSED`।
3. `SPEED` নির্বাচন করে ENTER চাপলে edit page আসবে। `ACTIVE` লাইনে বর্তমানে প্রয়োগ করা mode, নিচে inverted highlight-এ নতুন নির্বাচন দেখা যাবে।
4. UP/DOWN-এ preview পরিবর্তন করো। **ENTER = APPLY**। **BACK = CANCEL** (কোনো পরিবর্তন নয়)।
5. পরিবর্তন করার পর `14 SAVE SETTINGS` দিয়ে save করো, যেন power off/on-এর পরও থাকে। Menu header-এ `CAL!` থাকলে settings পরিবর্তিত কিন্তু save হয়নি।
6. `15 RESET DEFAULTS` এখন confirmation screen দেখায়। এখানে ENTER দিলে শুধু settings default হবে; valid calibration নিজে থেকে delete হয় না।
7. `SPEED` preset APPLY করলে সংশ্লিষ্ট preset-এর base/max/curve/turn PWM এবং Kp/Ki/Kd **একসঙ্গে preset value-তে** ফেরে। Custom PID tune করলে preset বদলানোর পরে আবার tuning করবে।

## Install ও Verify

- ZIP Extract করো। `LFR_PRO_V3/LFR_PRO_V3.ino` Arduino IDE-তে open করো। একই folder-এ `LFR_OLED.cpp`, `LFR_OLED.h`, `LFR_Config.h`, `LFR_PID.*`, `LFR_Control.*`, `LFR_Sensor.*`, `LFR_Motor.*`, `LFR_Storage.*` রাখতে হবে।
- Board: `Generic STM32F1 series`, part: `BluePill F103C8`, STM32 Core 3.0.0 (আগের working settings)।
- `LFR_Config.h`-এ `#define LFR_ALLOW_RACE_START 0` হলো **safe default**। এর মানে race locked, কিন্তু UI পরীক্ষা করা যাবে। আগের hardware ও BACK E-stop safety test সম্পন্ন করে ইচ্ছাকৃতভাবে Race unlock করতে চাইলে `1` করতে পারবে।
- OLED সোজা থাকলে `LFR_OLED_ROTATE_180 0`। পুরো display 180° উল্টো থাকলে 1; শুধু numeric glyph ঠিক করতে এই setting বদলাবে না।
- প্রথমে Arduino **Verify ✓** করবে। Compile report-এ Flash/RAM limit মানছে কি না দেখো। এর পরে motor connector বিচ্ছিন্ন রেখে UI upload/test করা নিরাপদ।
- Serial upload: Battery OFF → BOOT0=1, BOOT1=0 → FTDI 3.3V-compatible TX→PA10, RX→PA9, GND→GND (FTDI VCC connected নয়) → battery ON → RESET → Upload। Successful হলে power OFF → BOOT0=0 → power ON/RESET।
- Saved calibration ও settings-এর exact status OLED বা Serial Monitor থেকে যাচাই করো। নতুন code flash করার পর V3 settings একই storage layout ব্যবহার করে; তবু reset দিলে calibration active আছে কি না পরীক্ষা করতে হবে।

## Safety

- UI patch পরীক্ষা করার সময় motor connector আলাদা রাখবে এবং motor test / race চালাবে না।
- Code update/flash করার আগে আগের working original firmware source backup রাখবে।
- Actual STM32F103 C8 target compile ও hardware OLED verification **এখনো ব্যবহারকারীর Arduino IDE-তে করতে হবে**। এই প্যাকেজ host-unit-test ও SSD1306 framebuffer snapshot-এ pass করেছে; hardware-এ অপ্রত্যাশিত ত্রুটি অসম্ভব বলা যাচ্ছে না।

## Source structure

```
LFR_PRO_V3/
  LFR_PRO_V3.ino
  LFR_Config.h
  LFR_OLED.h / LFR_OLED.cpp
  LFR_PID.h / LFR_PID.cpp
  LFR_Control.h / LFR_Control.cpp
  LFR_Sensor.h / LFR_Sensor.cpp
  LFR_Motor.h / LFR_Motor.cpp
  LFR_Storage.h / LFR_Storage.cpp
```

`OLED_KEY_SCREENS.png`-এ ছয়টি প্রধান UI preview; `OLED_PREVIEW_UI_FIXED.png`-এ সব ২১টি screen snapshot আছে।
