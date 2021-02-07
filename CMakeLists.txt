cmake_minimum_required(VERSION 3.11)
project(velodyne_decoder CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
  "$<$<CONFIG:Debug>:-ggdb3;-Og>"
  "$<$<CONFIG:RelWithDebInfo>:-ggdb3;-O3>"
  "$<$<CONFIG:Release>:-O3>"
  -Wall
  -Wextra
  -Wno-sign-compare
  # disabled since pedantic complains about 'ISO C++ prohibits anonymous structs' for PointXYZIRT
  # -Wpedantic
)

if(DEFINED VERSION_INFO)
  add_compile_definitions(VERSION_INFO=${VERSION_INFO})
endif()

include_directories(include)

find_package(yaml-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11)
FetchContent_MakeAvailable(pybind11)
pybind11_add_module(${PROJECT_NAME} SHARED
  src/python.cpp
  src/calibration.cpp
  src/packet_decoder.cpp
  src/scan_decoder.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${YAML_CPP_LIBRARIES})
