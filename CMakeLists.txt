cmake_minimum_required(VERSION 3.5)

set (EXTRA_COMPONENT_DIRS "../esp-idf/")

list(APPEND c_compile_options "-Wno-dev")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(servoarm_uros)

