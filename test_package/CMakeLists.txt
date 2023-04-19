cmake_minimum_required(VERSION 3.16)
project(PackageTest LANGUAGES CXX)

find_package(velodyne_decoder REQUIRED)

add_executable(example example.cpp)
target_link_libraries(example PRIVATE velodyne_decoder::velodyne_decoder)
target_compile_features(example PRIVATE cxx_std_17)
