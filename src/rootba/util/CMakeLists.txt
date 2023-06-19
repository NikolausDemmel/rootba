target_sources(rootba
   PUBLIC
     assert.hpp
     cast.hpp
     container_traits.hpp
     eigen_types.hpp
     enum_utils.hpp
     format.hpp
     json_utils.hpp
     own_or_reference.hpp
     pprint_utils.hpp
     serialization.hpp
     stl_utils.hpp
     system_utils.hpp
     tbb_utils.hpp
     template_utils.hpp
     test_io_utils.hpp
     time_utils.hpp
     toml_utils.hpp
     typesafe_utils.hpp
     visit_struct_utils.hpp
   PRIVATE
     system_utils.cpp
     tbb_utils.cpp
)

if(ROOTBA_ENABLE_TESTING)

  rootba_add_test(test_librootba_util
      own_or_reference.test.cpp
      stl_utils.test.cpp
      system_utils.test.cpp
    LINK_LIBRARIES
      rootba
  )

endif()
