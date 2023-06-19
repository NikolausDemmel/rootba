cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

message(STATUS "CMake version: ${CMAKE_VERSION}")

include(cmake/PreventInSourceBuild.cmake)

project(rootba VERSION 0.1.0 LANGUAGES CXX)

# helper
include(cmake/Utils.cmake)

# options
option(BUILD_SHARED_LIBS "Build shared library" ON)

# TODO: eventually, the default should be OFF here (?)
option(ROOTBA_DEVELOPER_MODE "Helpful for developing, but not for building / deploying." ON)

option(ROOTBA_ENABLE_TESTING "Enable tests to be built" ON)

# Make specialization for double / float / static-sizes optional.
# Disabling them can be useful for better compile times during development.
option(ROOTBA_INSTANTIATIONS_DOUBLE "Instatiate templates for Scalar=double." ON)
option(ROOTBA_INSTANTIATIONS_FLOAT "Instatiate templates for Scalar=float." ON)
option(ROOTBA_INSTANTIATIONS_STATIC_LMB "Instatiate templates for statically-size landmark blocks." ON)

if(ROOTBA_INSTANTIATIONS_DOUBLE)
  list(APPEND ROOTBA_COMPILE_DEFINITIONS ROOTBA_INSTANTIATIONS_DOUBLE)
endif()
if(ROOTBA_INSTANTIATIONS_FLOAT)
  list(APPEND ROOTBA_COMPILE_DEFINITIONS ROOTBA_INSTANTIATIONS_FLOAT)
endif()
if(ROOTBA_INSTANTIATIONS_STATIC_LMB)
  list(APPEND ROOTBA_COMPILE_DEFINITIONS ROOTBA_INSTANTIATIONS_STATIC_LMB)
endif()

# set build type if not specified
set_default_build_type(Release)
if(NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

# configure ccache if available and not specified
set_default_compiler_launcher()

# Set C++ standard
if(NOT ROOTBA_CXX_STANDARD)
  set(ROOTBA_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(ROOTBA_DEVELOPER_MODE)
  # For developing make life easier by putting the binaries in the source folder
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
endif()

#sanitizer flags for asan builds
# TODO: ubsan shows some errors with TBB on linux and also some strange errors on macos
#set(ROOTBA_SANITIZER_FLAGS "-fno-omit-frame-pointer -fsanitize=address,undefined")
set(ROOTBA_SANITIZER_FLAGS "-fno-omit-frame-pointer -fsanitize=address")

# debug flags (for Debug and RelWithDebInfo builds)
set(ROOTBA_DEBUG_FLAGS "-DROOTBA_DEBUG -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

# release flags (for Release builds)
set(ROOTBA_RELEASE_FLAGS "-DROOTBA_DISABLE_ASSERTS")

# allow to override from externally passed CMAKE_CXX_FLAGS
# Build types are:
# - Debug: no optimization, debug symbols, assertions
# - RelWithDebInfo: full optimization, debug symbols, assertions
# - Release: full optimization, no debug symbols, no assertions
set(CMAKE_CXX_FLAGS_DEBUG "-g ${ROOTBA_DEBUG_FLAGS} ${CMAKE_CXX_FLAGS}")                # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g ${ROOTBA_DEBUG_FLAGS} ${CMAKE_CXX_FLAGS}")   # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${ROOTBA_RELEASE_FLAGS} ${CMAKE_CXX_FLAGS}")  # cmake default: "-O3 -DNDEBUG"

# CI version with no debug symbols and sanitizer flags
set(CMAKE_CXX_FLAGS_CIDEBUG "-g0 ${ROOTBA_DEBUG_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -g0  ${ROOTBA_DEBUG_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_SANITIZERDEBUG "-g ${ROOTBA_DEBUG_FLAGS} ${ROOTBA_SANITIZER_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_SANITIZERRELWITHDEBINFO "-O3 -g ${ROOTBA_DEBUG_FLAGS} ${ROOTBA_SANITIZER_FLAGS} ${CMAKE_CXX_FLAGS}")

# Save build commands in json to use with clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# TODO: did we need these asan flags at some point?
#set(CMAKE_EXE_LINKER_FLAGS_SANITIZERDEBUG "${ROOTBA_SANITIZER_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS_SANITIZERDEBUG "${ROOTBA_SANITIZER_FLAGS}")
#set(CMAKE_SHARED_LINKER_FLAGS_SANITIZERRELWITHDEBINFO "${ROOTBA_SANITIZER_FLAGS}")
#set(CMAKE_SHARED_LINKER_FLAGS_SANITIZERRELWITHDEBINFO "${ROOTBA_SANITIZER_FLAGS}")

# See comment in ./build_external.sh on the relation of arch=native and Eigen's
# alignment. TLDR: Passing arch=native for all build types is currently the only
# viable option to avoid suble bugs with Eigen.
if(NOT CXX_MARCH)
  if(NOT APPLE OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(CXX_MARCH native)
  else()
    # NOTE: Newer versions of Clang / AppleClang support -march=native on Apple Silicon,
    #       so at some point we can remove this workaround.
    message(STATUS "Running on Apple ${CMAKE_SYSTEM_PROCESSOR}. Not defaulting to -march=native.")
    set(CXX_MARCH "")
  endif()
endif()

if(CXX_MARCH)
  set(ROOTBA_MARCH_FLAG "-march=${CXX_MARCH}")
else()
  set(ROOTBA_MARCH_FLAG "")
endif()

# setup targets for dependencies
include(cmake/SetupDependencies.cmake)

add_subdirectory(external)

if(ROOTBA_ENABLE_TESTING)
  message(STATUS "Testing is enabled")
  # enable before src, which also contains unit tests
  enable_testing()
  # for gtest_discover_test
  include(GoogleTest)
endif()

add_subdirectory(src)

if(ROOTBA_ENABLE_TESTING)
  add_subdirectory(test)
endif()
