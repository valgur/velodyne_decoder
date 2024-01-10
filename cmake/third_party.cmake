if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(FIND "${CMAKE_TOOLCHAIN_FILE}" "conan_toolchain.cmake" FOUND_CONAN_TOOLCHAIN)
if(FOUND_CONAN_TOOLCHAIN GREATER_EQUAL 0)
    set(BUILD_TRIGGERED_BY_CONAN TRUE)
endif()

if(DEFINED VCPKG_TOOLCHAIN OR BUILD_TRIGGERED_BY_CONAN)
    set(USE_CONAN FALSE)
    message(NOTICE "Vcpkg or Conan toolchain detected, disabling automatic dependency management")
else()
    option(USE_CONAN "Use Conan to automatically manage dependencies" TRUE)
endif()

include(CMakeDependentOption)
cmake_dependent_option(INSTALL_THIRD_PARTY "Install third-party dependencies alongside the project" TRUE "USE_CONAN" FALSE)

if(USE_CONAN)
    set(CONAN_EXTRA_INSTALL_ARGS
        # Deploy the installed dependencies in the build dir for easier installation
        --deployer=full_deploy --deployer-folder=${CMAKE_BINARY_DIR}
    )
    list(APPEND CMAKE_PROJECT_TOP_LEVEL_INCLUDES ${CMAKE_CURRENT_LIST_DIR}/conan_provider.cmake)
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
