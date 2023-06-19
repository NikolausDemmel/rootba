# IPO seems to result in strange crashes in some configuration; also hard to debug even in Debug build...

#include(CheckIPOSupported)
#check_ipo_supported(LANGUAGES CXX RESULT IPO_SUPPORTED OUTPUT IPO_OUTPUT)
#message(STATUS "Link-time optimization support: ${IPO_SUPPORTED}")
#if(IPO_SUPPORTED)
#  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
#else()
#  message(STATUS "${IPO_OUTPUT}")
#endif()

add_executable(bal bal.cpp)
target_link_libraries(bal rootba_solver rootba_ceres rootba_cli)

add_executable(bal_gui bal_gui.cpp)
target_link_libraries(bal_gui rootba_pangolin rootba_solver rootba_ceres rootba_cli)

add_executable(bal_ceres bal_ceres.cpp)
target_link_libraries(bal_ceres rootba_ceres rootba_cli)

add_executable(bal_qr bal_qr.cpp)
target_link_libraries(bal_qr rootba_solver rootba_cli)

add_executable(bal_sc bal_sc.cpp)
target_link_libraries(bal_sc rootba_solver rootba_cli)
