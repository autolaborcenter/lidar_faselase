﻿cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(lidar_faselase LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if (UNIX)

find_package(Threads REQUIRED)

add_library(d10_linux
    src/point_t.hpp

    src/d10_t.hh
    src/d10_t.cc

    app/serial_linux.h
    app/serial_linux.cpp)
target_link_libraries(d10_linux atomic pthread)

add_library(obstacle
    app/robot_outline.hpp

    app/obstacles.h
    app/obstacles.cpp)

EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)
if (architecture STREQUAL "aarch64")

add_executable(d10_aarch64 app/main.cpp)
target_link_libraries(d10_aarch64 d10_linux obstacle)

else ()

add_executable(d10_unix app/main.cpp)
target_link_libraries(d10_unix d10_linux obstacle)

add_executable(test_d10 test/test.cpp)
target_link_libraries(test_d10 d10_linux obstacle)

add_executable(test_geometry test/test_geometry.cpp)

endif ()

endif ()
