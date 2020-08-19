cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
# Note: We need 3.10 or later for gtest_discover_tests

# Note: Some unit tests are defined directly in subfolders in the src folder

add_test(NAME smoke_test_bal
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/smoke_tests/test_bal.sh
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_test(NAME smoke_test_bal_qr
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/smoke_tests/test_bal_qr.sh
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_test(NAME smoke_test_bal_sc
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/smoke_tests/test_bal_sc.sh
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_test(NAME smoke_test_bal_ceres
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/smoke_tests/test_bal_ceres.sh
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
