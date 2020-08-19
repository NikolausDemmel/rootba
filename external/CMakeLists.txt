
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# for subprojects
set(CMAKE_CXX_STANDARD ${ROOTBA_CXX_STANDARD})

# Abseil
#add_subdirectory(abseil-cpp EXCLUDE_FROM_ALL)

if(ROOTBA_ENABLE_TESTING)
  set(BUILD_SHARED_LIBS Off)
  # TODO: check CMP0077 error
  #set(INSTALL_GTEST Off)
  # TODO: check CMP0048 and fix upstream

  message(STATUS "Configuring external 'googletest'")
  add_subdirectory(googletest EXCLUDE_FROM_ALL)
  # link to gtest and gtest_main to use gtest


  # ensure gtest include directory is added before any of the
  # system-wide include directories that might contain gtest

  #include_directories(BEFORE ${gtest_SOURCE_DIR}/include)
  #set_property(TARGET gtest PROPERTY BEFORE INTERFACE_INCLUDE_DIRECTORIES  ${gtest_SOURCE_DIR}/include)
  #target_include_directories(gtest BEFORE INTERFACE ${gtest_SOURCE_DIR}/include)

  # TODO: check if this is still the right way...
  #target_include_directories(gtest BEFORE INTERFACE googletest/googletest/include)
endif()

message(STATUS "Done configuring externals")
