add_library(rootba_sc)

target_sources(rootba_sc
  PUBLIC
    linearization_sc.hpp
    landmark_block.hpp
    linearization_power_sc.hpp
  PRIVATE
)

target_link_libraries(rootba_sc
  PUBLIC
    rootba
)

if(ROOTBA_ENABLE_TESTING)

  rootba_add_test(test_librootba_power_sc
      linearization_power_sc.test.cpp
    LINK_LIBRARIES
      rootba_cg
      rootba_testing
  )

endif()
