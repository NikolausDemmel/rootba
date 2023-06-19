add_library(rootba_pangolin)

target_sources(rootba_pangolin
  PUBLIC
    bal_image_overlay.hpp
    bal_map_display.hpp
    gui_helpers.hpp
  PRIVATE
    bal_image_overlay.cpp
    bal_map_display.cpp
    gui_helpers.cpp
)

target_link_libraries(rootba_pangolin
  PUBLIC
    rootba
    pango_display
)
