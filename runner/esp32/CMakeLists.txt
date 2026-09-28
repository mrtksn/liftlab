# Example ESP-IDF project: the flight controller side of the step runner (idf.py set-target esp32s3 && idf.py build).
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dfb_flight)
