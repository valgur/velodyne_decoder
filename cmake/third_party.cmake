if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(FIND "${CMAKE_TOOLCHAIN_FILE}" "conan_toolchain.cmake" _result)
if(_result GREATER_EQUAL 0)
    set(FOUND_CONAN_TOOLCHAIN TRUE)
endif()
string(FIND "${CMAKE_TOOLCHAIN_FILE}" "vcpkg.cmake" _result)
if(_result GREATER_EQUAL 0)
    set(FOUND_VCPKG_TOOLCHAIN TRUE)
endif()

if(FOUND_CONAN_TOOLCHAIN)
    message(NOTICE "Conan toolchain already in use, disabling automatic dependency management")
    set(USE_CONAN FALSE)
elseif(FOUND_VCPKG_TOOLCHAIN)
    message(NOTICE "Vcpkg toolchain already in use, disabling automatic dependency management")
    set(USE_CONAN FALSE)
else()
    option(USE_CONAN "Use Conan to automatically manage dependencies" TRUE)
endif()

include(CMakeDependentOption)
cmake_dependent_option(INSTALL_THIRD_PARTY "Install third-party dependencies alongside the project" TRUE "USE_CONAN" FALSE)

if(USE_CONAN)
    if(CMAKE_VERSION GREATER_EQUAL 3.24)
        set(CONAN_INSTALL_ARGS
            --build=missing
            # Deploy the installed dependencies in the build dir for easier installation
            --deployer=full_deploy "--deployer-folder=${CMAKE_BINARY_DIR}"
        )
        list(APPEND CMAKE_PROJECT_TOP_LEVEL_INCLUDES ${CMAKE_CURRENT_LIST_DIR}/conan_provider.cmake)
    else()
        if(NOT EXISTS "${CMAKE_BINARY_DIR}/full_deploy")
            message(FATAL_ERROR
                "CMake 3.24 or greater is required to install Conan dependencies automatically. "
                "You will have to run\n"
                "'conan install . --build=missing --deployer=full_deploy --deployer-folder=build/Release'\n"
                "manually in the repository root instead."
            )
        endif()
        # To use the output from the Conan CMakeDeps generator
        list(PREPEND CMAKE_PREFIX_PATH
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/generators
            ${CMAKE_CURRENT_BINARY_DIR}/build/${CMAKE_BUILD_TYPE}/generators
        )
    endif()
endif()

set(CONAN_DEPLOYER_DIR "${CMAKE_BINARY_DIR}/full_deploy/host")

function(install_third_party_libs)
    # Installs all include directories from Conan packages deployed to the build directory by 'conan install --deploy'
    set(configurations ${CMAKE_CONFIGURATION_TYPES})
    if(NOT configurations)
        set(configurations ${CMAKE_BUILD_TYPE})
    endif()
    file(GLOB_RECURSE conaninfo_files "${CONAN_DEPLOYER_DIR}/*/conaninfo.txt")
    foreach(conaninfo_path ${conaninfo_files})
        # Get relative path
        file(RELATIVE_PATH rel_conaninfo_path "${CONAN_DEPLOYER_DIR}" "${conaninfo_path}")
        # Get package name
        string(REGEX REPLACE "([^/]+)/.*$" "\\1" package_name "${rel_conaninfo_path}")
        # Get package root dir
        get_filename_component(package_root "${conaninfo_path}" DIRECTORY)
        # Copy includes
        if(EXISTS "${package_root}/include")
            install(DIRECTORY "${package_root}/include/"
                    DESTINATION "${THIRD_PARTY_INCLUDEDIR}")
        endif()
        foreach(config ${configurations})
            if(NOT rel_conaninfo_path MATCHES ".+/${config}/.+")
                continue()
            endif()
            # Copy .a, .so, .lib, .dylib
            file(GLOB_RECURSE lib_files "${package_root}/lib/*")
            foreach(lib ${lib_files})
                if(lib MATCHES "\\.(a|so[^/]*]|lib|dylib)$")
                    install(FILES "${lib}"
                            DESTINATION "${THIRD_PARTY_LIBDIR}"
                            CONFIGURATIONS ${config})
                endif()
            endforeach()
            # Copy .dll
            file(GLOB_RECURSE dll_files "${package_root}/bin/*")
            foreach(dll ${dll_files})
                if(dll MATCHES "\\.dll$")
                    install(FILES "${dll}"
                            DESTINATION "${THIRD_PARTY_BINDIR}"
                            CONFIGURATIONS ${config})
                endif()
            endforeach()
        endforeach()
    endforeach()
endfunction()

function(copy_third_party_dlls)
    # Copy all third-party DLLs to build/bin for tests and executables
    file(GLOB_RECURSE dll_files "${CONAN_DEPLOYER_DIR}/*/*.dll")
    if(CMAKE_CONFIGURATION_TYPES)
        foreach(config ${CMAKE_CONFIGURATION_TYPES})
            foreach(dll ${dll_files})
                file(RELATIVE_PATH dll_rel "${CONAN_DEPLOYER_DIR}" "${dll}")
                if(dll_rel MATCHES ".+/${config}/.+")
                    file(COPY ${dll} DESTINATION "${CMAKE_BINARY_DIR}/bin/${config}")
                endif()
            endforeach()
        endforeach()
    else()
        file(COPY ${dll_files} DESTINATION "${CMAKE_BINARY_DIR}/bin")
    endif()
endfunction()
