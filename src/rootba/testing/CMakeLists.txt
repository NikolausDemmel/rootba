
if(ROOTBA_ENABLE_TESTING)

  add_library(rootba_testing)

  target_link_libraries(rootba_testing
    PUBLIC
      rootba  # make sure we inherit all compile options
      gtest
  )

  target_sources(rootba_testing
     PUBLIC
       eigen_utils.hpp
       float_utils.hpp
       test_jacobian.hpp
       test_types.hpp
  )

endif()
