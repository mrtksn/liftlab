# Bench firmware: the step runner on the real chip, with nothing connected to motors or servos.
#   idf.py set-target esp32 && idf.py build          (or esp32s3)
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dfb_bench)
