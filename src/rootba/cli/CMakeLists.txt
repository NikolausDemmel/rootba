add_library(rootba_cli)

target_sources(rootba_cli
  PUBLIC
    bal_cli_utils.hpp
    cli_options.hpp
  PRIVATE
    bal_cli_utils.cpp
    cli_options.cpp
)

target_link_libraries(rootba_cli
  PUBLIC
    rootba
    rootba::clipp
)
