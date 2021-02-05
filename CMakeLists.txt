cmake_minimum_required(VERSION 3.11)
project(velodyne_decoder CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O3 -Wall -Wpedantic -Wno-sign-compare)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/calibration.cpp
  src/convert.cpp
  src/rawdata.cpp
  )

#include(FetchContent)
#FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11)
#FetchContent_MakeAvailable(pybind11)
#pybind11_add_module(${PROJECT_NAME} src/python.cpp)
#target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)
