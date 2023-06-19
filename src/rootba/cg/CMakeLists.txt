add_library(rootba_cg)

target_sources(rootba_cg
  PUBLIC
    block_sparse_matrix.hpp
    conjugate_gradient.hpp
    preconditioner.hpp
    utils.hpp
)

target_link_libraries(rootba_cg
  PUBLIC
    rootba
)

if(ROOTBA_ENABLE_TESTING)

  rootba_add_test(test_librootba_block_sparse_matrix
      block_sparse_matrix.test.cpp
    LINK_LIBRARIES
    rootba_cg
    rootba_testing
  )

  rootba_add_test(test_librootba_preconditioner
      preconditioner.test.cpp
    LINK_LIBRARIES
    rootba_cg
    rootba_testing
  )

endif()
