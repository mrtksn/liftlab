# Drone Force Bench flight firmware for the ESP32.
#   . $IDF_PATH/export.sh && idf.py set-target esp32 && idf.py build
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../.." "${CMAKE_CURRENT_LIST_DIR}/..")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dfb_flight)
