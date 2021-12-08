cmake_minimum_required(VERSION 3.14)
project(velodyne_decoder CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(MSVC)
  add_compile_options(/W4 /O2)
else()
  add_compile_options(
    "$<$<CONFIG:Debug>:-ggdb3;-Og>"
    "$<$<CONFIG:RelWithDebInfo>:-ggdb3;-O3>"
    "$<$<CONFIG:Release>:-O3>"
    -Wall
    -Wno-sign-compare
    -Wextra
    # disabled since pedantic complains about 'ISO C++ prohibits anonymous structs' for PointXYZIRT
    # -Wpedantic
  )
endif()

# version ranges are only available in CMake 3.19+
#find_package(yaml-cpp 0.6...<2 QUIET)
find_package(yaml-cpp 0.6 QUIET)
if(NOT yaml-cpp_FOUND)
  include(FetchContent)
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_SHALLOW ON
    GIT_TAG yaml-cpp-0.7.0
    )
  FetchContent_MakeAvailable(yaml-cpp)
  set(YAML_CPP_LIBRARIES yaml-cpp::yaml-cpp)
endif()

if(DEFINED VERSION_INFO)
  add_compile_definitions(VERSION_INFO=${VERSION_INFO})
endif()

add_library(${PROJECT_NAME} STATIC
  src/calibration.cpp
  src/packet_decoder.cpp
  src/scan_decoder.cpp
  src/stream_decoder.cpp
  )
target_link_libraries(${PROJECT_NAME} PRIVATE ${YAML_CPP_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC include PRIVATE SYSTEM ${YAML_CPP_INCLUDE_DIRS})

if(BUILD_PYTHON)
  #find_package(pybind11 2.6...<3 QUIET)
  find_package(pybind11 2.6 QUIET)
  if(NOT pybind11_FOUND)
    include(FetchContent)
    FetchContent_Declare(pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11.git
      GIT_TAG stable
      GIT_SHALLOW ON
      )
    FetchContent_MakeAvailable(pybind11)
  endif()

  pybind11_add_module(${PROJECT_NAME}_pylib src/python.cpp)
  target_include_directories(${PROJECT_NAME}_pylib PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_pylib PRIVATE ${PROJECT_NAME})
endif()
