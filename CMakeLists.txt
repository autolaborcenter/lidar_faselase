cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(pm1_sdk_2 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(faselase
    src/d10_t.hh
    src/d10_t.cc)

if (WIN32)

elseif (UNIX)

find_package(Threads REQUIRED)

add_executable(lidar_test app/main.cpp
    app/serial_linux.h
    app/serial_linux.cpp

    app/d10_driver_linux.h
    app/d10_driver_linux.cpp)
target_link_libraries(lidar_test faselase ${CMAKE_THREAD_LIBS_INIT})

# ---

EXECUTE_PROCESS(COMMAND uname -m OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE architecture)
if (architecture STREQUAL "aarch64")
else ()
endif ()

endif ()
