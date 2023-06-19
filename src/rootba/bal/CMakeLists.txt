
target_sources(rootba
   PUBLIC
     ba_log.hpp
     ba_log_options.hpp
     ba_log_utils.hpp
     bal_app_options.hpp
     bal_bundle_adjustment_helper.hpp
     bal_dataset_options.hpp
     bal_pipeline_summary.hpp
     bal_residual_options.hpp
     common_types.hpp
     bal_problem.hpp
     bal_problem_io.hpp
     residual_info.hpp
     snavely_projection.hpp
     solver_options.hpp
   PRIVATE
     ba_log.cpp
     ba_log_utils.cpp
     bal_bundle_adjustment_helper.cpp
     bal_problem.cpp
     residual_info.cpp
     solver_options.cpp
)

if(ROOTBA_ENABLE_TESTING)

  rootba_add_test(test_librootba_bal
      bal_bundle_adjustment_helper.test.cpp
      bal_problem_io.test.cpp
      snavely_projection.test.cpp
    LINK_LIBRARIES
      rootba
      rootba_testing
  )

endif()
