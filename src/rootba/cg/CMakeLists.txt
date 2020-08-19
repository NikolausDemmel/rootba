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

