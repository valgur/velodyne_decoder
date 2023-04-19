# From https://github.com/conan-io/cmake-conan/tree/develop2
#
# The MIT License (MIT)
#
# Copyright (c) 2019 JFrog
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

function(detect_os OS)
    # it could be cross compilation
    message(STATUS "Conan-cmake: cmake_system_name=${CMAKE_SYSTEM_NAME}")
    if(CMAKE_SYSTEM_NAME AND NOT CMAKE_SYSTEM_NAME STREQUAL "Generic")
        if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
            set(${OS} Macos PARENT_SCOPE)
        elseif(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
            set(${OS} Neutrino PARENT_SCOPE)
        else()
            set(${OS} ${CMAKE_SYSTEM_NAME} PARENT_SCOPE)
        endif()
    endif()
endfunction()


function(detect_cxx_standard CXX_STANDARD)
    set(${CXX_STANDARD} ${CMAKE_CXX_STANDARD} PARENT_SCOPE)
    if (CMAKE_CXX_EXTENSIONS)
        set(${CXX_STANDARD} "gnu${CMAKE_CXX_STANDARD}" PARENT_SCOPE)
    endif()
endfunction()


function(detect_compiler COMPILER COMPILER_VERSION)
    if(DEFINED CMAKE_CXX_COMPILER_ID)
        set(_COMPILER ${CMAKE_CXX_COMPILER_ID})
        set(_COMPILER_VERSION ${CMAKE_CXX_COMPILER_VERSION})
    else()
        if(NOT DEFINED CMAKE_C_COMPILER_ID)
            message(FATAL_ERROR "C or C++ compiler not defined")
        endif()
        set(_COMPILER ${CMAKE_C_COMPILER_ID})
        set(_COMPILER_VERSION ${CMAKE_C_COMPILER_VERSION})
    endif()

    message(STATUS "Conan-cmake: CMake compiler=${_COMPILER}") 
    message(STATUS "Conan-cmake: CMake cmpiler version=${_COMPILER_VERSION}")

    if(_COMPILER MATCHES MSVC)
        set(_COMPILER "msvc")
        string(SUBSTRING ${MSVC_VERSION} 0 3 _COMPILER_VERSION)
    elseif(_COMPILER MATCHES AppleClang)
        set(_COMPILER "apple-clang")
        string(REPLACE "." ";" VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
        list(GET VERSION_LIST 0 _COMPILER_VERSION)
    elseif(_COMPILER MATCHES Clang)
        set(_COMPILER "clang")
        string(REPLACE "." ";" VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
        list(GET VERSION_LIST 0 _COMPILER_VERSION)
    elseif(_COMPILER MATCHES GNU)
        set(_COMPILER "gcc")
        string(REPLACE "." ";" VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
        list(GET VERSION_LIST 0 _COMPILER_VERSION)
    endif()

    message(STATUS "Conan-cmake: [settings] compiler=${_COMPILER}") 
    message(STATUS "Conan-cmake: [settings] compiler.version=${_COMPILER_VERSION}")

    set(${COMPILER} ${_COMPILER} PARENT_SCOPE)
    set(${COMPILER_VERSION} ${_COMPILER_VERSION} PARENT_SCOPE)
endfunction()

function(detect_build_type BUILD_TYPE)
    if(NOT CMAKE_CONFIGURATION_TYPES)
        # Only set when we know we are in a single-configuration generator
        # Note: we may want to fail early if `CMAKE_BUILD_TYPE` is not defined
        set(${BUILD_TYPE} ${CMAKE_BUILD_TYPE} PARENT_SCOPE)
    endif()
endfunction()

function(detect_unix_libcxx LIBCXX)
    # Take into account any -stdlib in compile options
    get_directory_property(compile_options DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMPILE_OPTIONS)
    string(GENEX_STRIP "${compile_options}" compile_options)

    # Take into account any _GLIBCXX_USE_CXX11_ABI in compile definitions
    get_directory_property(defines DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMPILE_DEFINITIONS)
    string(GENEX_STRIP "${defines}" defines)

    foreach(define ${defines})
        if(define MATCHES "_GLIBCXX_USE_CXX11_ABI")
            if(define MATCHES "^-D")
                set(compile_options ${compile_options} "${define}")
            else()
                set(compile_options ${compile_options} "-D${define}")
            endif()
        endif()
    endforeach()

    # add additional compiler options ala cmRulePlaceholderExpander::ExpandRuleVariable
    set(EXPAND_CXX_COMPILER ${CMAKE_CXX_COMPILER})
    if(CMAKE_CXX_COMPILER_ARG1)
        # CMake splits CXX="foo bar baz" into CMAKE_CXX_COMPILER="foo", CMAKE_CXX_COMPILER_ARG1="bar baz"
        # without this, ccache, winegcc, or other wrappers might lose all their arguments
        separate_arguments(SPLIT_CXX_COMPILER_ARG1 NATIVE_COMMAND ${CMAKE_CXX_COMPILER_ARG1})
        list(APPEND EXPAND_CXX_COMPILER ${SPLIT_CXX_COMPILER_ARG1})
    endif()

    if(CMAKE_CXX_COMPILE_OPTIONS_TARGET AND CMAKE_CXX_COMPILER_TARGET)
        # without --target= we may be calling the wrong underlying GCC
        list(APPEND EXPAND_CXX_COMPILER "${CMAKE_CXX_COMPILE_OPTIONS_TARGET}${CMAKE_CXX_COMPILER_TARGET}")
    endif()

    if(CMAKE_CXX_COMPILE_OPTIONS_EXTERNAL_TOOLCHAIN AND CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN)
        list(APPEND EXPAND_CXX_COMPILER "${CMAKE_CXX_COMPILE_OPTIONS_EXTERNAL_TOOLCHAIN}${CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN}")
    endif()

    if(CMAKE_CXX_COMPILE_OPTIONS_SYSROOT)
        # without --sysroot= we may find the wrong #include <string>
        if(CMAKE_SYSROOT_COMPILE)
            list(APPEND EXPAND_CXX_COMPILER "${CMAKE_CXX_COMPILE_OPTIONS_SYSROOT}${CMAKE_SYSROOT_COMPILE}")
        elseif(CMAKE_SYSROOT)
            list(APPEND EXPAND_CXX_COMPILER "${CMAKE_CXX_COMPILE_OPTIONS_SYSROOT}${CMAKE_SYSROOT}")
        endif()
    endif()

    separate_arguments(SPLIT_CXX_FLAGS NATIVE_COMMAND ${CMAKE_CXX_FLAGS})

    if(CMAKE_OSX_SYSROOT)
        set(xcode_sysroot_option "--sysroot=${CMAKE_OSX_SYSROOT}")
    endif()

    execute_process(
      COMMAND ${CMAKE_COMMAND} -E echo "#include <string>"
      COMMAND ${EXPAND_CXX_COMPILER} ${SPLIT_CXX_FLAGS} -x c++ ${xcode_sysroot_option} ${compile_options} -E -dM -
      OUTPUT_VARIABLE string_defines
    )

    if(string_defines MATCHES "#define __GLIBCXX__")
        set(${LIBCXX} libstdc++11 PARENT_SCOPE)
    else()
        set(${LIBCXX} libc++ PARENT_SCOPE)
    endif()
endfunction()


function(detect_vs_runtime RUNTIME)
    conan_parse_arguments(${ARGV})
    if(ARGUMENTS_BUILD_TYPE)
        set(build_type "${ARGUMENTS_BUILD_TYPE}")
    elseif(CMAKE_BUILD_TYPE)
        set(build_type "${CMAKE_BUILD_TYPE}")
    else()
        message(FATAL_ERROR "Please specify in command line CMAKE_BUILD_TYPE (-DCMAKE_BUILD_TYPE=Release)")
    endif()

    if(build_type)
        string(TOUPPER "${build_type}" build_type)
    endif()
    set(variables CMAKE_CXX_FLAGS_${build_type} CMAKE_C_FLAGS_${build_type} CMAKE_CXX_FLAGS CMAKE_C_FLAGS)
    foreach(variable ${variables})
        if(NOT "${${variable}}" STREQUAL "")
            string(REPLACE " " ";" flags "${${variable}}")
            foreach (flag ${flags})
                if("${flag}" STREQUAL "/MD" OR "${flag}" STREQUAL "/MDd" OR "${flag}" STREQUAL "/MT" OR "${flag}" STREQUAL "/MTd")
                    string(SUBSTRING "${flag}" 1 -1 runtime)
                    set(${RUNTIME} "${runtime}" PARENT_SCOPE)
                    return()
                endif()
            endforeach()
        endif()
    endforeach()
    if("${build_type}" STREQUAL "DEBUG")
        set(${RUNTIME} "MDd" PARENT_SCOPE)
    else()
        set(${RUNTIME} "MD" PARENT_SCOPE)
    endif()
endfunction()

function(detect_host_profile output_file)
    detect_os(MYOS)
    detect_compiler(MYCOMPILER MYCOMPILER_VERSION)
    detect_cxx_standard(MYCXX_STANDARD)
    detect_build_type(MYBUILD_TYPE)

    if(MYCOMPILER EQUAL "msvc")
        detect_vs_runtime(MYRUNTIME)
    else()
        detect_unix_libcxx(MYLIBCXX)
    endif()

    set(PROFILE "")
    string(APPEND PROFILE "include(default)\n")
    string(APPEND PROFILE "[settings]\n")
    if(MYOS)
        string(APPEND PROFILE os=${MYOS} "\n")
    endif()
    if(MYCOMPILER)
        string(APPEND PROFILE compiler=${MYCOMPILER} "\n")
    endif()
    if(MYCOMPILER_VERSION)
        string(APPEND PROFILE compiler.version=${MYCOMPILER_VERSION} "\n")
    endif()
    if(MYCXX_STANDARD)
        string(APPEND PROFILE compiler.cppstd=${MYCXX_STANDARD} "\n")
    endif()
    if(MYLIBCXX)
        string(APPEND PROFILE compiler.libcxx=${MYLIBCXX} "\n")
    endif()
    if(MYRUNTIME)
        string(APPEND PROFILE compiler.runtime=${MYRUNTIME} "\n")
    endif()
    if(MYBUILD_TYPE)
        string(APPEND PROFILE "build_type=${MYBUILD_TYPE}\n")
    endif()

    if(NOT DEFINED output_file)
        set(_FN "${CMAKE_BINARY_DIR}/profile")
    else()
        set(_FN ${output_file})
    endif()

    string(APPEND PROFILE "[conf]\n")
    string(APPEND PROFILE "tools.build:compiler_executables={'c':'${CMAKE_C_COMPILER}','cpp':'${CMAKE_CXX_COMPILER}'}\n")
    string(APPEND PROFILE "tools.cmake.cmaketoolchain:generator=${CMAKE_GENERATOR}\n")

    message(STATUS "Conan-cmake: Creating profile ${_FN}")
    file(WRITE ${_FN} ${PROFILE})
    message(STATUS "Conan-cmake: Profile: \n${PROFILE}")
endfunction()


function(conan_profile_detect_default)
    message(STATUS "Conan-cmake: Checking if a default profile exists")
    execute_process(COMMAND conan profile path default
                    RESULT_VARIABLE return_code
                    OUTPUT_VARIABLE conan_stdout
                    ERROR_VARIABLE conan_stderr
                    ECHO_ERROR_VARIABLE    # show the text output regardless
                    ECHO_OUTPUT_VARIABLE
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    if(NOT ${return_code} EQUAL "0")
        message(STATUS "Conan-cmake: The default profile doesn't exist, detecting it.")
        execute_process(COMMAND conan profile detect
            RESULT_VARIABLE return_code
            OUTPUT_VARIABLE conan_stdout
            ERROR_VARIABLE conan_stderr
            ECHO_ERROR_VARIABLE    # show the text output regardless
            ECHO_OUTPUT_VARIABLE
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endfunction()


function(conan_install)
    cmake_parse_arguments(ARGS CONAN_ARGS ${ARGN})
    set(CONAN_OUTPUT_FOLDER ${CMAKE_BINARY_DIR}/conan)
    # Invoke "conan install" with the provided arguments
    set(CONAN_ARGS ${CONAN_ARGS} -of=${CONAN_OUTPUT_FOLDER})
    message(STATUS "CMake-conan: conan install ${CMAKE_SOURCE_DIR} ${CONAN_ARGS} ${ARGN}")
    execute_process(COMMAND conan install ${CMAKE_SOURCE_DIR} ${CONAN_ARGS} ${ARGN} --format=json
                    RESULT_VARIABLE return_code
                    OUTPUT_VARIABLE conan_stdout
                    ERROR_VARIABLE conan_stderr
                    ECHO_ERROR_VARIABLE    # show the text output regardless
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    if(NOT "${return_code}" STREQUAL "0")
        message(FATAL_ERROR "Conan install failed='${return_code}'")
    else()
        # the files are generated in a folder that depends on the layout used, if
        # one if specified, but we don't know a priori where this is. 
        # TODO: this can be made more robust if Conan can provide this in the json output
        string(JSON CONAN_GENERATORS_FOLDER GET ${conan_stdout} graph nodes 0 generators_folder)
        # message("conan stdout: ${conan_stdout}")
        message(STATUS "CMake-conan: CONAN_GENERATORS_FOLDER=${CONAN_GENERATORS_FOLDER}")
        set(CONAN_GENERATORS_FOLDER "${CONAN_GENERATORS_FOLDER}" PARENT_SCOPE)
        set(CONAN_INSTALL_SUCCESS TRUE CACHE BOOL "Conan install has been invoked and was successful")
    endif()
endfunction()


macro(conan_provide_dependency package_name)
    if(NOT CONAN_INSTALL_SUCCESS)
        message(STATUS "CMake-conan: first find_package() found. Installing dependencies with Conan")
        conan_profile_detect_default()
        detect_host_profile(${CMAKE_BINARY_DIR}/conan_profile)
        set(common_args -pr:h ${CMAKE_BINARY_DIR}/conan_profile
                               -pr:b ${CMAKE_BINARY_DIR}/conan_profile
                               --build=missing -g CMakeDeps)
        if(NOT CMAKE_CONFIGURATION_TYPES)
            message(STATUS "CMake-conan: Installing single configuration ${CMAKE_BUILD_TYPE}")
            conan_install(${common_args})
        else()
            message(STATUS "CMake-conan: Installing both Debug and Release")
            conan_install(${common_args} -s build_type=Release)
            conan_install(${common_args} -s build_type=Debug)
        endif()
        if (CONAN_INSTALL_SUCCESS)
            set(CONAN_GENERATORS_FOLDER "${CONAN_GENERATORS_FOLDER}" CACHE PATH "Conan generators folder")
        endif()
    else()
        message(STATUS "CMake-conan: find_package(${ARGV1}) found, 'conan install' already ran")
    endif()

    if (CONAN_GENERATORS_FOLDER)
        list(PREPEND CMAKE_PREFIX_PATH "${CONAN_GENERATORS_FOLDER}")
    endif()
    find_package(${ARGN} BYPASS_PROVIDER)
endmacro()
