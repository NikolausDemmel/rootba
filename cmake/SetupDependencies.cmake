# Note: We put this in a separate file to not clutter the main cmake lists.
# Ideally we would have appropriate find_package modules for all dependcies, but
# that is too much effort and we do it just like this for now.

message(STATUS "Setting up dependencies")

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake/modules/")
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/../external/install${ROOTBA_EXTERNAL_BUILD_DIR_SUFFIX}")

# Specify exact version, since we ship it in submodule
find_package(Eigen3 3.4.0 EXACT REQUIRED)
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
# link to target 'Eigen3::Eigen'

add_library(rootba::Sophus INTERFACE IMPORTED)
set_property(TARGET rootba::Sophus PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/Sophus)
add_library(rootba::Cereal INTERFACE IMPORTED)
set_property(TARGET rootba::Cereal PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/cereal/include)
target_compile_definitions(rootba::Cereal INTERFACE CEREAL_THREAD_SAFE=1)

add_library(rootba::visit_struct INTERFACE IMPORTED)
set_property(TARGET rootba::visit_struct PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/visit_struct/include)
add_library(rootba::wise_enum INTERFACE IMPORTED)
set_property(TARGET rootba::wise_enum PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/wise_enum/)
add_library(rootba::enum_flags INTERFACE IMPORTED)
set_property(TARGET rootba::enum_flags PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/enum-flags/include/)
add_library(rootba::nameof INTERFACE IMPORTED)
set_property(TARGET rootba::nameof PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/nameof/include/)
add_library(rootba::toml11 INTERFACE IMPORTED)
set_property(TARGET rootba::toml11 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/toml11/)
add_library(rootba::nlohmann_json INTERFACE IMPORTED)
set_property(TARGET rootba::nlohmann_json PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/json/)
add_library(rootba::basalt_headers INTERFACE IMPORTED)
set_property(TARGET rootba::basalt_headers PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/basalt-headers/include/)
add_library(rootba::clipp INTERFACE IMPORTED)
set_property(TARGET rootba::clipp PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/clipp/include)
add_library(rootba::pprint INTERFACE IMPORTED)
set_property(TARGET rootba::pprint PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/pprint)
add_library(rootba::magic_enum INTERFACE IMPORTED)
set_property(TARGET rootba::magic_enum PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/external/magic_enum)

find_package(Pangolin REQUIRED)
message(STATUS "Found Pangolin headers in: ${Pangolin_INCLUDE_DIR}")
# link to target 'pangolin'


#set(TBB_USE_DEBUG_BUILD FALSE)
find_package(TBB REQUIRED)
message(STATUS "Found TBB ${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR} (interface version ${TBB_INTERFACE_VERSION}) headers in: ${TBB_INCLUDE_DIRS} (TBB_LIBRARIES: ${TBB_LIBRARIES})")
if (TBB_INTERFACE_VERSION LESS 11004)
  # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
  target_compile_definitions(TBB::tbb INTERFACE TBB_PREVIEW_GLOBAL_CONTROL)
endif()
# link to target 'TBB::tbb'

find_package(Glog REQUIRED)
# link to target 'glog::glog'
message(STATUS "Found glog")

find_package(fmt REQUIRED)
# link to target 'fmt::fmt'
message(STATUS "Found {fmt}")

find_package(absl REQUIRED)
# link to targets 'absl::...'
message(STATUS "Found Abseil")

find_package(Ceres REQUIRED)
# link to target 'Ceres::ceres'



message(STATUS "Done setting up dependencies")
