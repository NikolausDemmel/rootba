cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
# Note: 3.13 for target_sources with relative paths

add_library(rootba)

target_include_directories(rootba PUBLIC "${CMAKE_SOURCE_DIR}/src")
target_compile_features(rootba PUBLIC cxx_std_${ROOTBA_CXX_STANDARD})
set_target_properties(rootba PROPERTIES CXX_EXTENSIONS OFF)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # don't use pedantic for GCC, since it warns on extra ';' --> fix wise enum
  target_compile_options(rootba PUBLIC -Wpedantic)
endif()
target_compile_options(rootba PUBLIC -ftemplate-backtrace-limit=0 -Wall -Wextra -Wnon-virtual-dtor -Wsign-compare -Wno-unused-parameter ${ROOTBA_MARCH_FLAG})
target_compile_definitions(rootba PUBLIC ${ROOTBA_COMPILE_DEFINITIONS})

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # offsetof is fine even for non POD types...
  target_compile_options(rootba PUBLIC -Wno-invalid-offsetof)
endif()

target_link_libraries(rootba
  PUBLIC
    rootba::Sophus
    rootba::visit_struct
    rootba::wise_enum
    rootba::enum_flags
    rootba::nameof
    rootba::toml11
    rootba::nlohmann_json
    rootba::basalt_headers
    rootba::magic_enum
    rootba::pprint
    rootba::Cereal
    fmt::fmt
    absl::flat_hash_map
    Eigen3::Eigen
    glog::glog
    TBB::tbb
)

add_subdirectory(testing)
add_subdirectory(util)
add_subdirectory(options)
add_subdirectory(bal)
add_subdirectory(qr)
add_subdirectory(sc)
add_subdirectory(solver)
add_subdirectory(cg)
add_subdirectory(ceres)
add_subdirectory(pangolin)
add_subdirectory(cli)
