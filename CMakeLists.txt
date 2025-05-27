cmake_minimum_required(VERSION 3.16)
project(AutoDrive)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/install)

file(GLOB_RECURSE SOURCES
    src/*.cpp
    src/Modules/*.cpp
)

include_directories(
    /usr/include
    /usr/include/opencv4
    /usr/include/libcamera
    include/AutoDrive
    include/AutoDrive/Modules
)

add_executable(AutoDrive ${SOURCES})

target_link_libraries(AutoDrive
    stdc++
    atomic
    pthread
    m
    zmq
    wiringPi
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    opencv_videoio
    camera
    camera-base
)
