cmake_minimum_required(VERSION 3.29)
project(DfuSeFile)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(DfuSeFile
    DfuSeFileTest.cpp)
