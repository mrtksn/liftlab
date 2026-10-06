# LiftLab command module for an ESP32: buttons and sticks in, CRSF to an ExpressLRS transmitter module, or ESP-NOW / Wi-Fi.
#   . $IDF_PATH/export.sh && idf.py set-target esp32 && idf.py build
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../.." "${CMAKE_CURRENT_LIST_DIR}/../../fc" "${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_CURRENT_LIST_DIR}/../../esp_radio")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dfb_ground)
