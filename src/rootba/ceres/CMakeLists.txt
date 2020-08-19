add_library(rootba_ceres)

target_sources(rootba_ceres
  PUBLIC
    ba_log_utils.hpp
    loss_function.hpp
    bal_bundle_adjustment.hpp
    bal_iteration_callback.hpp
    bal_residuals.hpp
    option_utils.hpp
    types.hpp
  PRIVATE
    ba_log_utils.cpp
    loss_function.cpp
    bal_bundle_adjustment.cpp
    bal_iteration_callback.cpp
    option_utils.cpp
)

# include order: ceres first; ensure ceres is found first in the specified location, not in /usr/local (on macOS with homebrew)
target_link_libraries(rootba_ceres PUBLIC Ceres::ceres)

target_link_libraries(rootba_ceres
  PUBLIC
    rootba
)
