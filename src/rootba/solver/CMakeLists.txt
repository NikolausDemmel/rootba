add_library(rootba_solver)

target_sources(rootba_solver
  PUBLIC
    bal_bundle_adjustment.hpp
    linearizor.hpp
    linearizor_base.hpp
    linearizor_power_sc.hpp
    linearizor_qr.hpp
    linearizor_sc.hpp
    solver_summary.hpp
  PRIVATE
    bal_bundle_adjustment.cpp
    linearizor.cpp
    linearizor_base.cpp
    linearizor_power_sc.cpp
    linearizor_qr.cpp
    linearizor_sc.cpp
)

target_link_libraries(rootba_solver
  PUBLIC
    rootba
    rootba_cg
    rootba_qr
    rootba_sc
)

if(ROOTBA_ENABLE_TESTING)

  rootba_add_test(test_librootba_solver
      bal_bundle_adjustment.test.cpp
    LINK_LIBRARIES
      rootba_solver
      rootba_testing
  )

endif()
