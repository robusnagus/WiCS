# Wireless Command Station
# Copyright 2020 Nagus
cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME "wics")
set(PROJECT_VER "0.9.1.1")

set(IDF_PATH $(HOME)/Espressif/esp-idf-release-v4.2)
set(SUPPORTED_TARGETS esp32)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(timer_group)
