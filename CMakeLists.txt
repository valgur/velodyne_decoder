include(cmake/conan.cmake)
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

find_package(yaml-cpp 0.7 REQUIRED)

add_library(${PROJECT_NAME}
  src/calibration.cpp
  src/calib_db.cpp
  src/packet_decoder.cpp
  src/scan_batcher.cpp
  src/scan_decoder.cpp
  src/stream_decoder.cpp
  src/telemetry_packet.cpp
  src/time_conversion.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)

set(BUILD_PYTHON FALSE CACHE BOOL "Whether to build a Python module in addition to the library")
if(BUILD_PYTHON OR SKBUILD)
  find_package(pybind11 2.6 REQUIRED)
  pybind11_add_module(${PROJECT_NAME}_pylib src/python.cpp)
  target_include_directories(${PROJECT_NAME}_pylib PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_pylib PRIVATE ${PROJECT_NAME})

  if (EXISTS VERSION_INFO)
    target_compile_definitions(${PROJECT_NAME}_pylib PRIVATE VERSION_INFO=${VERSION_INFO})
  endif()

  # Install directive for scikit-build
  install(TARGETS ${PROJECT_NAME}_pylib LIBRARY DESTINATION .)
endif()

## Install
if(NOT SKBUILD)
  include(GNUInstallDirs)
  install(TARGETS ${PROJECT_NAME}
          EXPORT ${PROJECT_NAME}_Targets
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)
endif()
