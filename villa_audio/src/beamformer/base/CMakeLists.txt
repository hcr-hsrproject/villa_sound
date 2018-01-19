set(libbeam_base_sources
      platform_thread.h platform_thread.cc
      logging.h logging.cc
      checks.h checks.cc
      thread_checker_impl.h thread_checker_impl.cc
      stringencode.h stringencode.cc
      event.h event.cc
      event_tracer.h event_tracer.cc
      timeutils.h timeutils.cc
      criticalsection.h criticalsection.cc
)

add_library(libbeam_base ${libbeam_base_sources})

target_link_libraries(libbeam_base)

# Don't duplicate 'lib' prefix
set_target_properties(libbeam_base PROPERTIES
  PREFIX ""
)

