set(USE_CONAN ON CACHE BOOL "Use Conan to automatically manage dependencies")

string(FIND "${CMAKE_TOOLCHAIN_FILE}" "conan_toolchain.cmake" FOUND_CONAN_TOOLCHAIN)
if(FOUND_CONAN_TOOLCHAIN GREATER_EQUAL 0)
  set(BUILD_TRIGGERED_BY_CONAN TRUE)
else()
  set(BUILD_TRIGGERED_BY_CONAN FALSE)
endif()

if(USE_CONAN AND NOT BUILD_TRIGGERED_BY_CONAN)
  if(CMAKE_VERSION GREATER_EQUAL 3.24)
    list(APPEND CMAKE_PROJECT_TOP_LEVEL_INCLUDES ${CMAKE_CURRENT_LIST_DIR}/conan_provider.cmake)
  else()
    message(WARNING
      "CMake 3.24 or greater is required to use Conan dependencies automatically. "
      "You will have to run 'conan install . --build=missing' manually in the source directory instead."
    )
    # To use the output from the Conan CMakeDeps generator
    list(PREPEND CMAKE_PREFIX_PATH
      ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/generators
      ${CMAKE_CURRENT_BINARY_DIR}/build/${CMAKE_BUILD_TYPE}/generators
    )
  endif()
endif()
