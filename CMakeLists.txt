cmake_minimum_required(VERSION 3.16)
project(AutoDrive)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

# 소스 파일 수집
file(GLOB_RECURSE SOURCES
    src/Modules/*.cpp
    src/AutoDrive.cpp
)

# 헤더 파일 포함 디렉토리
include_directories(
    /usr/include
    /usr/include/opencv4
    /usr/include/libcamera
)

# 실행 파일 생성
add_executable(AutoDrive ${SOURCES})

# 라이브러리 링크
target_link_libraries(AutoDrive
    stdc++
    atomic
    pthread
    m
    zmq
    wiringPi
    gpiod
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    opencv_videoio
    camera
    camera-base
)