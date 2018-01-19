set(libbeam_systemwrappers_sources
      aligned_malloc.h aligned_malloc.cc
)

add_library(libbeam_systemwrappers ${libbeam_systemwrappers_sources})

target_link_libraries(libbeam_systemwrappers)

# Don't duplicate 'lib' prefix
set_target_properties(libbeam_systemwrappers PROPERTIES
  PREFIX ""
)
