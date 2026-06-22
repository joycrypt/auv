cmake_minimum_required(VERSION 3.14)
project(auv_nav2 VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0 -Wall -Wextra -Wpedantic")
endif()

add_executable(auv_nav2
    src/main.cpp
    src/ekf.cpp
    src/csv_parser.cpp
    src/logger.cpp
)

target_include_directories(auv_nav2 PRIVATE include)

# Enable M_PI and other math constants on MSVC/MinGW
target_compile_definitions(auv_nav2 PRIVATE _USE_MATH_DEFINES)

message(STATUS "")
message(STATUS "====================================================")
message(STATUS "  AUV")
message(STATUS "  State  : [px py pz vx vy vz roll pitch yaw]")
message(STATUS "  Sensors: IMU + DVL + Depth + GPS + Magnetometer")
message(STATUS "  Build  : ${CMAKE_BUILD_TYPE}")
message(STATUS "====================================================")
message(STATUS "")
