
# disallow in-source builds since it creates a lot of generated
# files in all subdirectories that are a pain to clean
function(prevent_in_source_build)
  get_filename_component(srcdir "${CMAKE_SOURCE_DIR}" REALPATH)
  get_filename_component(bindir "${CMAKE_BINARY_DIR}" REALPATH)

  if("${srcdir}" STREQUAL "${bindir}")
    message(FATAL_ERROR "In source build is disallowed. Please run cmake from 'build' directory. Quitting configuration!")
  endif()
endfunction()

prevent_in_source_build()
