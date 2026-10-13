#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p /tmp/lfr-v3-tests
CXX=${CXX:-g++}
FLAGS="-std=c++17 -Wall -Wextra -Werror -O2 -Istubs -I../LFR_PRO_V3"
$CXX $FLAGS stubs/mock_arduino.cpp \
  ../LFR_PRO_V3/LFR_PID.cpp ../LFR_PRO_V3/LFR_Control.cpp \
  ../LFR_PRO_V3/LFR_Sensor.cpp ../LFR_PRO_V3/LFR_Motor.cpp \
  ../LFR_PRO_V3/LFR_Storage.cpp ../LFR_PRO_V3/LFR_OLED.cpp \
  test_v3.cpp -o /tmp/lfr-v3-tests/test_v3
/tmp/lfr-v3-tests/test_v3
$CXX $FLAGS -x c++ -c ../LFR_PRO_V3/LFR_PRO_V3.ino -o /tmp/lfr-v3-tests/sketch.o
printf '%s\n' 'PASS full Arduino sketch syntax under mock API'

$CXX $FLAGS stubs/mock_arduino.cpp \
  ../LFR_PRO_V3/LFR_PID.cpp ../LFR_PRO_V3/LFR_Control.cpp \
  ../LFR_PRO_V3/LFR_Sensor.cpp ../LFR_PRO_V3/LFR_Motor.cpp \
  ../LFR_PRO_V3/LFR_Storage.cpp ../LFR_PRO_V3/LFR_OLED.cpp \
  /tmp/lfr-v3-tests/sketch.o test_ui.cpp -o /tmp/lfr-v3-tests/test_ui
/tmp/lfr-v3-tests/test_ui

mkdir -p previews
$CXX $FLAGS -DLFR_UI_TEST_ACCESS stubs/mock_arduino.cpp \
  ../LFR_PRO_V3/LFR_PID.cpp ../LFR_PRO_V3/LFR_Control.cpp \
  ../LFR_PRO_V3/LFR_Sensor.cpp ../LFR_PRO_V3/LFR_Motor.cpp \
  ../LFR_PRO_V3/LFR_Storage.cpp ../LFR_PRO_V3/LFR_OLED.cpp \
  test_oled_visual.cpp -o /tmp/lfr-v3-tests/test_oled_visual
/tmp/lfr-v3-tests/test_oled_visual

$CXX $FLAGS -DLFR_UI_TEST_ACCESS stubs/mock_arduino.cpp \
  ../LFR_PRO_V3/LFR_PID.cpp ../LFR_PRO_V3/LFR_Control.cpp \
  ../LFR_PRO_V3/LFR_Sensor.cpp ../LFR_PRO_V3/LFR_Motor.cpp \
  ../LFR_PRO_V3/LFR_Storage.cpp ../LFR_PRO_V3/LFR_OLED.cpp \
  test_mode_editor.cpp -o /tmp/lfr-v3-tests/test_mode_editor
/tmp/lfr-v3-tests/test_mode_editor
