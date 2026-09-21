cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/drivers")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(safety_monitor)