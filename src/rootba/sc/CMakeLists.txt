add_library(rootba_sc)

target_sources(rootba_sc
  PUBLIC
    linearization_sc.hpp
    landmark_block.hpp
  PRIVATE
)

target_link_libraries(rootba_sc
  PUBLIC
    rootba
)
