cmake_minimum_required(VERSION 3.15)
include(cmake/third_party.cmake)
project(velodyne_decoder VERSION 3.0.0 LANGUAGES CXX)

## Build flags

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT DEFINED CMAKE_POSITION_INDEPENDENT_CODE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

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
  )
endif()

## Library

find_package(yaml-cpp 0.7 REQUIRED)
find_package(Microsoft.GSL REQUIRED)

add_library(velodyne_decoder
  src/calibration.cpp
  src/calib_db.cpp
  src/packet_decoder.cpp
  src/scan_batcher.cpp
  src/scan_decoder.cpp
  src/stream_decoder.cpp
  src/telemetry_packet.cpp
  src/time_conversion.cpp
  src/types.cpp
)
target_link_libraries(velodyne_decoder
  PRIVATE yaml-cpp::yaml-cpp
  PUBLIC Microsoft.GSL::GSL
)
target_include_directories(velodyne_decoder
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    PUBLIC "$<INSTALL_INTERFACE:include>"
)
target_compile_definitions(velodyne_decoder PUBLIC _USE_MATH_DEFINES)

if(MSVC)
  copy_third_party_dlls()
endif()

## Python bindings
option(BUILD_PYTHON "Build velodyne_decoder_pylib Python module" FALSE)
if(BUILD_PYTHON OR SKBUILD)
  find_package(Python
      REQUIRED COMPONENTS Interpreter Development.Module
      OPTIONAL_COMPONENTS Development.SABIModule)
  find_package(nanobind CONFIG REQUIRED)

  nanobind_add_module(python_bindings STABLE_ABI NB_STATIC src/python.cpp)
  set_target_properties(python_bindings PROPERTIES OUTPUT_NAME velodyne_decoder_pylib)
  target_include_directories(python_bindings PRIVATE include)
  target_link_libraries(python_bindings PRIVATE velodyne_decoder)
  target_compile_definitions(python_bindings PRIVATE VERSION_INFO=${PROJECT_VERSION})

  if(DEFINED SKBUILD_PROJECT_NAME)
    set(PYTHON_INSTALL_DIR ${SKBUILD_PROJECT_NAME})
  elseif(NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/velodyne_decoder)
  endif()

  install(TARGETS python_bindings
      LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
      COMPONENT python)
  install(DIRECTORY src/velodyne_decoder/
      DESTINATION ${PYTHON_INSTALL_DIR}
      COMPONENT python)
endif()

## Install

include(GNUInstallDirs)
install(TARGETS velodyne_decoder
    EXPORT velodyne_decoder-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CMAKE_INSTALL_DIR lib/cmake/velodyne_decoder)
install(EXPORT velodyne_decoder-targets
    NAMESPACE velodyne_decoder::
    DESTINATION "${CMAKE_INSTALL_DIR}"
    FILE velodyne_decoder-targets.cmake
    COMPONENT velodyne_decoder
)

set(THIRD_PARTY_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/velodyne_decoder/third_party")
set(THIRD_PARTY_LIBDIR "${CMAKE_INSTALL_LIBDIR}/velodyne_decoder/third_party")
set(THIRD_PARTY_BINDIR "${CMAKE_INSTALL_BINDIR}/velodyne_decoder/third_party")
if(INSTALL_THIRD_PARTY)
  install_third_party_libs()
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/velodyne_decoder-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/velodyne_decoder-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_DIR}
    PATH_VARS
        CMAKE_INSTALL_INCLUDEDIR
        CMAKE_INSTALL_LIBDIR
        THIRD_PARTY_INCLUDEDIR
        THIRD_PARTY_LIBDIR
        THIRD_PARTY_BINDIR
)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/velodyne_decoder-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/velodyne_decoder-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/velodyne_decoder-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_DIR}"
)
