cmake_minimum_required(VERSION 3.19)

# set the project name
project(mbed_usb_rosserial)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(rosserial_mbed INTERFACE)
target_include_directories(rosserial_mbed INTERFACE include)