macro(set_default_build_type build_type)
  # Set default build type if not specified otherwise.
  # See https://cmake.org/pipermail/cmake/2012-May/050243.html
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE ${build_type} CACHE STRING "Choose the type of build." FORCE)
    message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
      "MinSizeRel" "RelWithDebInfo")
  endif()
endmacro()

macro(set_default_compiler_launcher)
  if(NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
      message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
      set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
      set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    else()
      message(STATUS "Dind't find ccache")
    endif()
  else()
    message(STATUS "Compiler launcher already set. Not configuring ccache.")
    message(STATUS "CMAKE_C_COMPILER_LAUNCHER: ${CMAKE_C_COMPILER_LAUNCHER}")
    message(STATUS "CMAKE_CXX_COMPILER_LAUNCHER: ${CMAKE_CXX_COMPILER_LAUNCHER}")
  endif()
endmacro()

# Custom macro for adding unit tests.
# Inspired by: https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html
macro(rootba_add_test)
  set(options "")
  set(oneValueArgs "")
  set(multiValueArgs "LINK_LIBRARIES;INCLUDE_DIRECTORIES")
  cmake_parse_arguments(ROOTBA_TEST
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  list(POP_FRONT ROOTBA_TEST_UNPARSED_ARGUMENTS ROOTBA_TEST_NAME)
  set(ROOTBA_TEST_SOURCES ${ROOTBA_TEST_UNPARSED_ARGUMENTS})

  add_executable(${ROOTBA_TEST_NAME} ${ROOTBA_TEST_SOURCES})

  # we cannot directly use target_compile_options on ${ROOTBA_TEST_NAME}, since then they would
  # appear before the warning flags inherited from any libraries in ${ROOTBA_TEST_LINK_LIBRARIES}
  # and the -Wno-... arguments would be overwritten again
  add_library(${ROOTBA_TEST_NAME}_compile_options INTERFACE)
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # no-gnu-zero-variadic-macro-arguments: see https://github.com/google/googletest/issues/2271
    target_compile_options(${ROOTBA_TEST_NAME}_compile_options INTERFACE -Wno-gnu-zero-variadic-macro-arguments)
  endif()

  target_link_libraries(${ROOTBA_TEST_NAME} PRIVATE gtest gmock gtest_main ${ROOTBA_TEST_LINK_LIBRARIES} ${ROOTBA_TEST_NAME}_compile_options)

  target_include_directories(${ROOTBA_TEST_NAME} PRIVATE ${ROOTBA_TEST_INCLUDE_DIRECTORIES})

  # Note on test discovery timeout: the default of 5 seconds was sometimes hit on
  # the build server, thus we increase it liberally.
  gtest_discover_tests(${ROOTBA_TEST_NAME}
    WORKING_DIRECTORY
      ${CMAKE_SOURCE_DIR}
    DISCOVERY_TIMEOUT
      60
  )

  set_target_properties(${ROOTBA_TEST_NAME} PROPERTIES FOLDER tests)
endmacro()

function(TARGET_COMPILE_FLAG_OPTION TARGET DEFINE_NAME DOCSTRING DEFAULT_VALUE )
  option(${DEFINE_NAME} "${DOCSTRING}" ${DEFAULT_VALUE})
  message(STATUS "${DEFINE_NAME}: ${${DEFINE_NAME}}")
  if(${DEFINE_NAME})
    target_compile_definitions(${TARGET} PUBLIC ${DEFINE_NAME})
  endif()
endfunction()
