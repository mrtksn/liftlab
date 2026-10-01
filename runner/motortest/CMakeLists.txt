# Motor test for brushed motors on H-bridge boards (L9110S, L293D), ESP32-S3.
#   idf.py set-target esp32s3 && idf.py build
cmake_minimum_required(VERSION 3.16)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dfb_motortest)
